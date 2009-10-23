/* drivers/usb/function/mtp_tunnel.c
 *
 * Function Device for the Android ADB Protocol
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * base on ADB function, modify to MTP tunnel function
 * by Anthony_Chang <anthony_chang@htc.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <linux/wait.h>
#include <linux/list.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <mach/msm_hsusb.h>

#include "usb_function.h"
#include <linux/usb/cdc.h>

/* please refer: Documentation/ioctl-number.txt and Documentation/ioctl/
 * and choice magic-number */
#define USB_MTP_IOC_MAGIC 0xFF

#define USB_MTP_FUNC_IOC_CANCEL_REQUEST_SET	_IOW(USB_MTP_IOC_MAGIC, 0x20, int)
#define USB_MTP_FUNC_IOC_CANCEL_REQUEST_GET	_IOW(USB_MTP_IOC_MAGIC, 0x21, int)
#define USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_SET	_IOW(USB_MTP_IOC_MAGIC, 0x22, int)
#define USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_GET	_IOW(USB_MTP_IOC_MAGIC, 0x23, int)
#define USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_SET	_IOW(USB_MTP_IOC_MAGIC, 0x24, int)
#define USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_GET	_IOW(USB_MTP_IOC_MAGIC, 0x25, int)
#define USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_SET	_IOW(USB_MTP_IOC_MAGIC, 0x26, int)
#define USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_GET	_IOW(USB_MTP_IOC_MAGIC, 0x27, int)

/* base on Annex. D in PIMA15740-2000 spec */
#define PIMA15740_CANCEL_REQUEST 0x64
#define PIMA15740_GET_EXTENDED_EVENT_DATA 0x65
#define PIMA15740_DEVICE_RESET_REQUEST 0x66
#define PIMA15740_GET_DEVICE_STATUS 0x67

static u16 mtp_tunnel_status = 0xFF;

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(KERN_INFO x)
#endif

/* allocate buffer size to: 16384 byte */
#define ALLOCATE_16K_BUFF

#ifdef ALLOCATE_16K_BUFF
#define TXN_MAX 16384
#else
#define TXN_MAX 4096

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 4
#define TX_REQ_MAX 4
#endif

#define MTP_TUNNEL_FUNCTION_NAME "mtp_tunnel"

struct device mtp_tunnel_dev;

struct mtp_tunnel_context
{
	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
	atomic_t enable_open_excl;
	atomic_t enable_read_excl;
	atomic_t enable_ioctl_excl;
	spinlock_t lock;

	struct usb_endpoint *out;
	struct usb_endpoint *in;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;

	/* the request we're currently reading from */
	struct usb_request *read_req;
	unsigned char *read_buf;
	unsigned read_count;
	int registered;
	struct platform_device *pdev;
	u8 ioctl_tmp[2];
};

static struct mtp_tunnel_context _context;

struct _mtp_specific_request {
	//unsigned char request_specific_sequence;
	unsigned char request_specific_num;
	const char *request_specific_string;
};
/*
static struct _mtp_specific_request mtp_specific_request[] = {
	{ PIMA15740_CANCEL_REQUEST,		"Cancel_Request" },
	{ PIMA15740_GET_EXTENDED_EVENT_DATA,	"Get_Extented_Event_Data" },
	{ PIMA15740_DEVICE_RESET_REQUEST,		"Device_Reset_Request" },
	{ PIMA15740_GET_DEVICE_STATUS,		"Get_Device_Status" },
	{ 0xFF, NULL }
};
*/
static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
void static req_put(struct mtp_tunnel_context *ctxt, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct mtp_tunnel_context *ctxt, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
	return req;
}

static void mtp_tunnel_complete_in(struct usb_endpoint *ept, struct usb_request *req)
{
	struct mtp_tunnel_context *ctxt = req->context;

	if (req->status != 0)
		ctxt->error = 1;

	req_put(ctxt, &ctxt->tx_idle, req);

	wake_up(&ctxt->write_wq);
}

static void mtp_tunnel_complete_out(struct usb_endpoint *ept, struct usb_request *req)
{
	struct mtp_tunnel_context *ctxt = req->context;

	if (req->status != 0) {
		ctxt->error = 1;
		req_put(ctxt, &ctxt->rx_idle, req);
	} else {
		req_put(ctxt, &ctxt->rx_done, req);
	}

	wake_up(&ctxt->read_wq);
}

static ssize_t mtp_tunnel_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct mtp_tunnel_context *ctxt = &_context;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	DBG("mtp_tunnel_read(%d)\n", count);

	if (_lock(&ctxt->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(ctxt->online || ctxt->error)) {
		DBG("mtp_tunnel_read: waiting for online state\n");
		ret = wait_event_interruptible(ctxt->read_wq, (ctxt->online || ctxt->error));
		if (ret < 0) {
			_unlock(&ctxt->read_excl);
			return ret;
		}
	}

	while (count > 0) {
		if (ctxt->error) {
			r = -EIO;
			break;
		}

		/* if we have idle read requests, get them queued */
		while ((req = req_get(ctxt, &ctxt->rx_idle))) {
requeue_req:
			req->length = TXN_MAX;
			ret = usb_ept_queue_xfer(ctxt->out, req);
			if (ret < 0) {
				DBG("mtp_tunnel_read: failed to queue req %p (%d)\n", req, ret);
				r = -EIO;
				ctxt->error = 1;
				req_put(ctxt, &ctxt->rx_idle, req);
				goto fail;
			} else {
				DBG("%s(): rx %p queue\n", __func__, req);
			}
		}

		/* if we have data pending, give it to userspace */
		if (ctxt->read_count > 0) {
			xfer = (ctxt->read_count < count) ? ctxt->read_count : count;

			if (copy_to_user(buf, ctxt->read_buf, xfer)) {
				r = -EFAULT;
				break;
			}
			ctxt->read_buf += xfer;
			ctxt->read_count -= xfer;
			buf += xfer;
			count -= xfer;

			/* if we've emptied the buffer, release the request */
			if (ctxt->read_count == 0) {
				req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
				ctxt->read_req = 0;
			}
			continue;
		}

		/* wait for a request to complete */
		req = 0;
		ret = wait_event_interruptible(ctxt->read_wq,
					       ((req = req_get(ctxt, &ctxt->rx_done)) || ctxt->error));

		if (req != 0) {
			/* if we got a 0-len one we need to put it back into
			** service.  if we made it the current read req we'd
			** be stuck forever
			*/
			if (req->actual == 0)
				goto requeue_req;

			ctxt->read_req = req;
			ctxt->read_count = req->actual;
			ctxt->read_buf = req->buf;
			DBG("%s(): rx %p %d\n", __func__, req, req->actual);
		}

		if (ret < 0) {
			r = ret;
			break;
		}
	}

fail:
	_unlock(&ctxt->read_excl);
	return r;
}

static ssize_t mtp_tunnel_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct mtp_tunnel_context *ctxt = &_context;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG("mtp_tunnel_write(%d)\n", count);

	if (_lock(&ctxt->write_excl))
		return -EBUSY;

	while (count > 0) {
		if (ctxt->error) {
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(ctxt->write_wq,
					       ((req = req_get(ctxt, &ctxt->tx_idle)) || ctxt->error));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			xfer = count > TXN_MAX ? TXN_MAX : count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}

			req->length = xfer;
			ret = usb_ept_queue_xfer(ctxt->in, req);
			if (ret < 0) {
				DBG("mtp_tunnel_write: xfer error %d\n", ret);
				ctxt->error = 1;
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}


	if (req)
		req_put(ctxt, &ctxt->tx_idle, req);

	_unlock(&ctxt->write_excl);
	return r;
}

static int mtp_tunnel_open(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;

	if (_lock(&ctxt->open_excl))
		return -EBUSY;

	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int mtp_tunnel_release(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;

	_unlock(&ctxt->open_excl);
	return 0;
}

static struct file_operations mtp_tunnel_fops = {
	.owner =   THIS_MODULE,
	.read =    mtp_tunnel_read,
	.write =   mtp_tunnel_write,
	.open =    mtp_tunnel_open,
	.release = mtp_tunnel_release,
};

static struct miscdevice mtp_tunnel_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp_tunnel",
	.fops = &mtp_tunnel_fops,
};

static DECLARE_WAIT_QUEUE_HEAD(mtp_tunnel_enable_read_wait);
#if 0
static ssize_t mtp_tunnel_enable_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct mtp_tunnel_context *ctxt = &_context;
	int r = 0;
	int loop_i;

	DBG("mtp_tunnel_read(%d)\n", count);

	if (_lock(&ctxt->enable_read_excl))
		return -EBUSY;
/*
	while(mtp_tunnel_status == 0xFF)	{
		_unlock(&ctxt->enable_read_excl);
		if(fp->f_flags & O_NONBLOCK)	{
			_unlock(&ctxt->enable_read_excl);
			r = -EAGAIN;
			goto fail;
			}
		if(wait_event_interruptible(mtp_tunnel_enable_read_wait,
			mtp_tunnel_status != 0xFF))	{
			r = -ERESTARTSYS;
			goto fail;
			}
		if (_lock(&ctxt->enable_read_excl))
			return -EBUSY;
	}
*/
	for(loop_i = 0; loop_i < ARRAY_SIZE(mtp_specific_request); loop_i++)	{
		if(mtp_tunnel_status == mtp_specific_request[loop_i].request_specific_sequence)	{
			if(mtp_specific_request[loop_i].request_specific_string != NULL)
				r = strlen(mtp_specific_request[loop_i].request_specific_string);
			if(r)
				if(copy_to_user(buf,
					mtp_specific_request[loop_i].request_specific_string, 
					r))
					r = -EFAULT;
			break;
		}
	}
	count -= r;
	*pos += r;
fail:
	_unlock(&ctxt->enable_read_excl);
	return r;
}
#endif
static int
mtp_tunnel_enable_ioctl(struct inode *inode, struct file *file,
	  unsigned int cmd, unsigned long arg)
{
	struct mtp_tunnel_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;
	int tmp_value;

	if (_lock(&ctxt->enable_ioctl_excl))
		return -EBUSY;

	if(_IOC_TYPE(cmd) != USB_MTP_IOC_MAGIC)	{
		printk(KERN_NOTICE "_IOC_TYPE(cmd) != USB_MTP_IOC_MAGIC, return -ENOTTY\n");
		_unlock(&ctxt->enable_ioctl_excl);
		return -ENOTTY;
	}

	switch(cmd) {
		case USB_MTP_FUNC_IOC_CANCEL_REQUEST_SET:
			printk(KERN_NOTICE "%s: USB_MTP_FUNC_IOC_CANCEL_REQUEST_SET\n", __func__);
			mtp_tunnel_status = 0xFF;
		break;
		case USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_SET:
			printk(KERN_NOTICE "%s: USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_SET\n", __func__);
			if (copy_from_user(&tmp_value, argp, sizeof(int)))
				return -EFAULT;
			ctxt->ioctl_tmp[0] = 0xFF & tmp_value;
			ctxt->ioctl_tmp[1] = 0xFF & (tmp_value >> 8);
			printk(KERN_INFO "%s: data_0: 0x%X, data_1: 0x%X #\n\n\n",
				__func__, ctxt->ioctl_tmp[0], ctxt->ioctl_tmp[1]);
			mtp_tunnel_status = 0xFF;
		break;
		case USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_GET:
			printk(KERN_NOTICE "%s: USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_GET\n", __func__);
		break;
#if 0
/* Needless, return -ENOTTY */
		case USB_MTP_FUNC_IOC_CANCEL_REQUEST_GET:
			printk(KERN_NOTICE "%s: USB_MTP_FUNC_IOC_CANCEL_REQUEST_GET\n", __func__);
			return -ENOTTY;
		break;
		case USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_SET:
			printk(KERN_NOTICE "%s: USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_SET\n", __func__);
			return -ENOTTY;
		break;
		case USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_GET:
			printk(KERN_NOTICE "%s: USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_GET\n", __func__);
			return -ENOTTY;
		break;
		case USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_SET:
			printk(KERN_NOTICE "%s: USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_SET\n", __func__);
			return -ENOTTY;
		break;
		case USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_GET:
			printk(KERN_NOTICE "%s: USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_GET\n", __func__);
			return -ENOTTY;
		break;
#endif
		default:
			printk(KERN_NOTICE "%s: default, will return -ENOTTY\n", __func__);
			return -ENOTTY;
		break;
	}
	_unlock(&ctxt->enable_ioctl_excl);
	return 0;
}

static int mtp_tunnel_enable_open(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;

	if (_lock(&ctxt->enable_open_excl))
		return -EBUSY;

	DBG("%s(): Enabling %s function ###\n", __func__, MTP_TUNNEL_FUNCTION_NAME);
	usb_function_enable(MTP_TUNNEL_FUNCTION_NAME, 1);
	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int mtp_tunnel_enable_release(struct inode *ip, struct file *fp)
{
	struct mtp_tunnel_context *ctxt = &_context;

	DBG("%s(): Disabling %s function ###\n", __func__, MTP_TUNNEL_FUNCTION_NAME);
	usb_function_enable(MTP_TUNNEL_FUNCTION_NAME, 0);
	_unlock(&ctxt->enable_open_excl);
	return 0;
}


static struct file_operations mtp_tunnel_fops_enable = {
	.owner =	THIS_MODULE,
	//.read =		mtp_tunnel_enable_read,
	.open =		mtp_tunnel_enable_open,
	.ioctl =	mtp_tunnel_enable_ioctl,
	.release =	mtp_tunnel_enable_release,
};

static struct miscdevice mtp_tunnel_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp_enable",
	.fops = &mtp_tunnel_fops_enable,
};

static ssize_t store_mtp_tunnel_status(struct device *dev, 
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	u32 ul;
	struct mtp_tunnel_context *ctxt = &_context;

	ul = simple_strtoul(buf, NULL, 10);
	if(ul != PIMA15740_CANCEL_REQUEST && ul != PIMA15740_GET_DEVICE_STATUS)	{
		printk(KERN_WARNING "Unknown mtp_tunnel status: 0x%2.2X\n", ul);
		return -EINVAL;
	}
	mtp_tunnel_status = ul;
	kobject_uevent(&ctxt->pdev->dev.kobj, KOBJ_CHANGE);

	return count;
}

static ssize_t show_mtp_tunnel_status(struct device *dev, 
	struct device_attribute *attr,
	char *buf)
{
	int rc = 0;

	switch(mtp_tunnel_status)	{
		case PIMA15740_CANCEL_REQUEST:
			rc += sprintf(buf, "Cancel_Request\n");
		break;
		case PIMA15740_GET_DEVICE_STATUS:
			rc += sprintf(buf, "Get_Device_Status\n");
		break;
	}
	return rc;
}

static DEVICE_ATTR(mtp_tunnel_status, 0644, show_mtp_tunnel_status, store_mtp_tunnel_status);

static void mtp_tunnel_dev_release (struct device *dev) {}

static int mtp_tunnel_probe_register (struct platform_device *pdev)
{
	struct mtp_tunnel_context *ctxt = &_context;
	ctxt->pdev = pdev;
	return 0;
}

static struct platform_driver mtp_tunnel_driver = {
	.probe = mtp_tunnel_probe_register,
	.driver = { .name = MTP_TUNNEL_FUNCTION_NAME, },
};

static void mtp_tunnel_release_register (struct device *dev) {}

static struct platform_device mtp_tunnel_device_register = {
	.name		= MTP_TUNNEL_FUNCTION_NAME,
	.id		= -1,
	.dev		= {
		.release	= mtp_tunnel_release_register,
	},
};

static void mtp_tunnel_unbind(void *_ctxt)
{
	struct mtp_tunnel_context *ctxt = _ctxt;
	struct usb_request *req;

	printk(KERN_DEBUG "mtp_tunnel_unbind()\n");

	while ((req = req_get(ctxt, &ctxt->rx_idle))) {
		usb_ept_free_req(ctxt->out, req);
	}
	while ((req = req_get(ctxt, &ctxt->tx_idle))) {
		usb_ept_free_req(ctxt->in, req);
	}

	ctxt->online = 0;
	ctxt->error = 1;

	if (ctxt->registered)	{
		device_remove_file(&mtp_tunnel_dev, &dev_attr_mtp_tunnel_status);
		device_unregister(&mtp_tunnel_dev);
		ctxt->registered = 0;
		}
	
	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
}

static void mtp_tunnel_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct mtp_tunnel_context *ctxt = _ctxt;
	struct usb_request *req;
	int ret;
#ifndef ALLOCATE_16K_BUFF
	int n;
#endif
	ctxt->registered = 0;
	ctxt->out = ept[0];
	ctxt->in = ept[1];

	printk(KERN_DEBUG "mtp_tunnel_bind() %p, %p\n", ctxt->out, ctxt->in);

#ifndef ALLOCATE_16K_BUFF
	for (n = 0; n < RX_REQ_MAX; n++)
#endif
	{
		req = usb_ept_alloc_req(ctxt->out, TXN_MAX);
		if (req == 0) goto fail;
		req->context = ctxt;
		req->complete = mtp_tunnel_complete_out;
		req_put(ctxt, &ctxt->rx_idle, req);
	}


#ifndef ALLOCATE_16K_BUFF
	for (n = 0; n < TX_REQ_MAX; n++)
#endif
	{
		req = usb_ept_alloc_req(ctxt->in, TXN_MAX);
		if (req == 0) goto fail;
		req->context = ctxt;
		req->complete = mtp_tunnel_complete_in;
		req_put(ctxt, &ctxt->tx_idle, req);
	}

#ifndef ALLOCATE_16K_BUFF
	printk(KERN_DEBUG
	       "mtp_tunnel_bind() allocated %d rx and %d tx requests\n",
	       RX_REQ_MAX, TX_REQ_MAX);
#else
	printk(KERN_DEBUG
		"%s(): allocated buffer: %d\n", __func__, TXN_MAX);
#endif

	misc_register(&mtp_tunnel_device);
	misc_register(&mtp_tunnel_enable_device);

	mtp_tunnel_dev.release = mtp_tunnel_dev_release;
	mtp_tunnel_dev.parent = &ctxt->pdev->dev;
	strcpy(mtp_tunnel_dev.bus_id, "interface");

	ret = device_register(&mtp_tunnel_dev);
	if (ret != 0) {
		printk(KERN_WARNING "mtp_tunnel_dev failed to register device: %d\n", ret);
		goto fail_dev_register_fail;
	}
	ret = device_create_file(&mtp_tunnel_dev, &dev_attr_mtp_tunnel_status);
	if (ret != 0) {
		printk(KERN_WARNING "mtp_tunnel_dev device_create_file failed: %d\n", ret);
		device_unregister(&mtp_tunnel_dev);
		goto fail_dev_register_fail;
	}
	ctxt->registered = 1;
	return;

fail_dev_register_fail:
	printk(KERN_ERR "%s() could not allocate requests\n", __func__);

fail:
	printk(KERN_WARNING "mtp_tunnel_bind() could not allocate requests\n");
	mtp_tunnel_unbind(ctxt);
}

static void mtp_tunnel_configure(int configured, void *_ctxt)
{
	struct mtp_tunnel_context *ctxt = _ctxt;
	struct usb_request *req;

	DBG("mtp_tunnel_configure() %d\n", configured);

	if (configured) {
		ctxt->online = 1;

		/* if we have a stale request being read, recycle it */
		ctxt->read_buf = 0;
		ctxt->read_count = 0;
		if (ctxt->read_req) {
			req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
			ctxt->read_req = 0;
		}

		/* retire any completed rx requests from previous session */
		while ((req = req_get(ctxt, &ctxt->rx_done)))
			req_put(ctxt, &ctxt->rx_idle, req);

	} else {
		ctxt->online = 0;
		ctxt->error = 1;
	}

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
}

static int mtp_tunnel_setup(struct usb_ctrlrequest *ctrl, void *buf,
			int len, void *_ctxt)
{
	struct mtp_tunnel_context *ctxt = _ctxt;
	int value = -EOPNOTSUPP;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	//u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
		case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_CONTROL_LINE_STATE:
			printk(KERN_DEBUG "%s(): USB_CDC_REQ_SET_CONTROL_LINE_STATE\n", __func__);
			value = 1;
			break;
		case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| PIMA15740_CANCEL_REQUEST:
			printk(KERN_DEBUG "%s(): PIMA15740_CANCEL_REQUEST\n", __func__);
			if(w_length != 6)
				break;
			mtp_tunnel_status = PIMA15740_CANCEL_REQUEST;
			kobject_uevent(&ctxt->pdev->dev.kobj, KOBJ_CHANGE);
			// wake_up_interruptible(&mtp_tunnel_enable_read_wait);
			if(wait_event_interruptible_timeout(mtp_tunnel_enable_read_wait,
				mtp_tunnel_status == 0xFF, HZ * 5))
				value = -ERESTARTSYS;
			value = 0;
			break;
		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| PIMA15740_GET_DEVICE_STATUS:
			printk(KERN_DEBUG "%s(): PIMA15740_GET_DEVICE_STATUS(%d)\n", __func__, w_length);
			value = w_length;
			mtp_tunnel_status = PIMA15740_GET_DEVICE_STATUS;
			kobject_uevent(&ctxt->pdev->dev.kobj, KOBJ_CHANGE);
			//wake_up_interruptible(&mtp_tunnel_enable_read_wait);
			if(wait_event_interruptible_timeout(mtp_tunnel_enable_read_wait,
				mtp_tunnel_status == 0xFF, HZ * 5)) {
				value = -ERESTARTSYS;
			} else {
				printk(KERN_INFO "%s: prepare data for send back to PC\n", __func__);
				*(u8 *)buf = ctxt->ioctl_tmp[0];
				*(u8 *)(buf+1) = ctxt->ioctl_tmp[1];
				value = 2;
			}
			break;

	}
	if (value == -EOPNOTSUPP)
		printk(KERN_ERR
			"%s: unknown class-specific control req "
			"%02x.%02x v%04x i%04x l%u\n", __func__,
			ctrl->bRequestType, ctrl->bRequest,
			le16_to_cpu(ctrl->wValue), w_index, w_length);

	if (value == -ERESTARTSYS)
		printk(KERN_ERR
			"%s: wain_event_interruptible fail...(0x%X)\n", __func__, ctrl->bRequest);
	return value;
}

static struct usb_function usb_func_mtp_tunnel = {
	.bind =		mtp_tunnel_bind,
	.unbind =	mtp_tunnel_unbind,
	.configure =	mtp_tunnel_configure,
	.setup =	mtp_tunnel_setup,

	.name = MTP_TUNNEL_FUNCTION_NAME,
	.context = &_context,

	.ifc_class = 0xFF,
	.ifc_subclass = 0xFF,
	.ifc_protocol = 0xFF,

	.ifc_name = MTP_TUNNEL_FUNCTION_NAME,

	.ifc_ept_count = 2,
	.ifc_ept_type = { EPT_BULK_OUT, EPT_BULK_IN },

	/* the mtp_tunnel function is only enabled when its driver file is open */
	.disabled = 1,
	.position_bit = USB_FUNCTION_MTP_TUNNEL_NUM,
	.cdc_desc = NULL,
	.ifc_num = 1,
	.ifc_index = STRING_MTP,
};

static int __init mtp_tunnel_init(void)
{
	struct mtp_tunnel_context *ctxt = &_context;
	int retval;
	DBG("mtp_tunnel_init()\n");

	init_waitqueue_head(&ctxt->read_wq);
	init_waitqueue_head(&ctxt->write_wq);

	atomic_set(&ctxt->open_excl, 0);
	atomic_set(&ctxt->read_excl, 0);
	atomic_set(&ctxt->write_excl, 0);

	atomic_set(&ctxt->enable_open_excl, 0);
	atomic_set(&ctxt->enable_read_excl, 0);
	atomic_set(&ctxt->enable_ioctl_excl, 0);

	spin_lock_init(&ctxt->lock);

	INIT_LIST_HEAD(&ctxt->rx_idle);
	INIT_LIST_HEAD(&ctxt->rx_done);
	INIT_LIST_HEAD(&ctxt->tx_idle);
	retval = platform_driver_register (&mtp_tunnel_driver);
	if (retval < 0)
		return retval;
	retval = platform_device_register (&mtp_tunnel_device_register);
	if (retval < 0)
		goto err_register_device;

	return usb_function_register(&usb_func_mtp_tunnel);

err_register_device:
	platform_driver_unregister(&mtp_tunnel_driver);
	return retval;
}

module_init(mtp_tunnel_init);
