
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>

#include <linux/wait.h>
#include <linux/list.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <linux/platform_device.h>

//#include <asm/arch/msm_smd.h>
#include <mach/msm_smd.h>

#include "usb_function.h"
#include "diag.h"

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(KERN_DEBUG x)
#endif

#if 0
#define TXN_MAX 4096
#define RXN_MAX 4096

#define HDLC_MAX 4096
#define ARM9_MAX 4096
#else
#define TXN_MAX 8192
#define RXN_MAX 8192

#define HDLC_MAX 8192
#define ARM9_MAX 8192
#endif
#define TABLE_SIZE 10

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 4
#define TX_REQ_MAX 4

#define DIAG_FUNCTION_NAME "diag"

int g_bUsbDiagMode = 0;

struct diag_context
{
	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
	atomic_t enable_excl;

	atomic_t open_arm9_excl;
	atomic_t read_arm9_excl;
	atomic_t write_arm9_excl;
		
	spinlock_t lock;
	spinlock_t lock_reg_num;

	struct usb_endpoint *out;
	struct usb_endpoint *in;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;

	struct list_head rx_arm9_idle;
	struct list_head rx_arm9_done;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	wait_queue_head_t read_arm9_wq;


	smd_channel_t *ch;
	int in_busy;
	int isRead;
	int is2ARM11;
	char is7E;
	/* assembly buffer for USB->A9 HDLC frames */
	unsigned char hdlc_buf[HDLC_MAX];
	unsigned hdlc_count;
	unsigned hdlc_escape;
	
	unsigned char id_table[TABLE_SIZE];
	unsigned char toARM9_buf[ARM9_MAX];
	
	/* the request we're currently reading from */
	struct usb_request *read_req;
	unsigned char *read_buf;
	unsigned read_count;

	struct usb_request *read_arm9_req;
	unsigned char *read_arm9_buf;
	unsigned read_arm9_count;
	struct platform_device *pdev;
	u64 tx_count; // To ARM9
	u64 rx_count; // From ARM9
};

static struct diag_context _context;

struct device diag_device;
enum data_access {
	data_set_clear = 0,
	data_set_rx,
	data_get_rx,
	data_set_tx,
	data_get_tx,
};

static void smd_try_to_send(struct diag_context *ctxt);

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

static u64 smd_xfer_count_func(u64 data_length, enum data_access access_behavior)
{
	struct diag_context *ctxt = &_context;
	u64 ret = 0;
	const u64 max_value = 0x7FFFFFFF;
	switch(access_behavior)	{
		case data_set_clear:
			ctxt->tx_count = 0;
			ctxt->rx_count = 0;
		break;
		case data_set_rx:
			if((data_length + ctxt->rx_count) >= max_value)
				ctxt->rx_count = (ctxt->rx_count + data_length) - max_value;
			else
				ctxt->rx_count += data_length;
		break;
		case data_get_rx:
			ret = ctxt->rx_count;
		break;
		case data_set_tx:
			if((data_length + ctxt->tx_count) >= max_value)
				ctxt->tx_count = (ctxt->tx_count + data_length) - max_value;
			else
				ctxt->tx_count += data_length;
		break;
		case data_get_tx:
			ret = ctxt->tx_count;
		break;
	}
	return ret;
}
/* add a request to the tail of a list */
void put_req(struct diag_context *ctxt, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
struct usb_request *get_req(struct diag_context *ctxt, struct list_head *head)
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

static void diag_queue_out(struct diag_context *ctxt)
{
	int ret;
	struct usb_request *req;
	
	/* if we have idle read requests, get them queued */
	while ((req = get_req(ctxt, &ctxt->rx_idle))) {
		req->length = TXN_MAX;
		ret = usb_ept_queue_xfer(ctxt->out, req);
		if (ret < 0) {
			DBG("diag_queue_out: failed to queue req %p (%d)\n", req, ret);
			ctxt->error = 1;
			put_req(ctxt, &ctxt->rx_idle, req);
		} else {
			DBG("rx %p queue\n", req);
		}
	}
}

#if 0
static void diag_process_hdlc(struct diag_context *ctxt, void *_data, unsigned len)
{
	unsigned char *data = _data;
	unsigned count = ctxt->hdlc_count;
	unsigned escape = ctxt->hdlc_escape;
	unsigned char *hdlc = ctxt->hdlc_buf;

	while (len-- > 0) {
		unsigned char x = *data++;
		if (x == 0x7E) { 
			if (count > 2) {
				/* we're just ignoring the crc here */
				//TRACE("PC>", hdlc, count - 2, 0);
				if (ctxt->ch)	{
					smd_write(ctxt->ch, hdlc, count - 2);
					smd_xfer_count_func(count - 2, data_set_tx);
				}
			}
			count = 0;
			escape = 0;
		} else if (x == 0x7D) {
			escape = 1;
		} else {
			if (escape) {
				x = x ^ 0x20;
				escape = 0;
			}
			hdlc[count++] = x;

			/* discard frame if we overflow */
			if (count == HDLC_MAX)
				count = 0;
		}
	}

	ctxt->hdlc_count = count;
	ctxt->hdlc_escape = escape;
}
#endif

static void smd_try_to_send(struct diag_context *ctxt)
{
	int ret;
	if (!ctxt->is2ARM11) {
		//if (ctxt->ch && (!ctxt->in_busy)) {
		if (ctxt->ch) {
			int r = smd_read_avail(ctxt->ch);
			//printk(KERN_ERR "Diag----------->Read data from SMD ch = %d, data Bytes %d\n", ctxt->ch, r);
			/*printk(KERN_DEBUG "smd_read_avail: %d bytes \n", r);*/
			if (r > 8192) {
				printk(KERN_ERR "The SMD data is too large to send!!\n");
				return;
			}
			if (r > 0) {
				struct usb_request *req = get_req(ctxt, &ctxt->tx_idle);
				if (!req) {
					printk(KERN_ERR "There is no enough request to PC!!\n");
					return;
				}
				smd_read(ctxt->ch, req->buf, r);
				smd_xfer_count_func(r, data_set_rx);
				req->length = r;
				//TRACE("A9>", req->buf, r, 1);
				ctxt->in_busy = 1;
				//printk(KERN_ERR "ARM9 data to PC %s\n", (char *)req->buf);
				if ((ret=usb_ept_queue_xfer(ctxt->in, req)) < 0)
					printk(KERN_WARNING "smd_try_to_send: cannot queue bulk in request, ret=%d\n",ret);

			}
		}
	} else {
		if (ctxt->ch) {
			int r = smd_read_avail(ctxt->ch);
			if (r > 8192) {
				printk(KERN_ERR "The SMD data is too large to send!!\n");
				return;
			}
			if (r > 0) {
				struct usb_request *req = get_req(ctxt, &ctxt->rx_arm9_idle);
				if (!req) {
					printk(KERN_ERR "There is no enough request to ARM11!!\n");
					return;
				}
				smd_read(ctxt->ch, req->buf, r);
				smd_xfer_count_func(r, data_set_rx);
				//req->length = r;
				//printk(KERN_ERR "ARM9 data to ARM11 %s\n", (char *)req->buf);
				req->actual = r;
				put_req(ctxt, &ctxt->rx_arm9_done, req);
				wake_up(&ctxt->read_arm9_wq);
			}
		}
	}
}

static void smd_diag_notify(void *priv, unsigned event)
{
	struct diag_context *ctxt = priv;
	smd_try_to_send(ctxt);
}

static void diag_complete_in(struct usb_endpoint *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;

	if (req->status != 0)
		ctxt->error = 1;
	
	ctxt->in_busy = 0;
	put_req(ctxt, &ctxt->tx_idle, req);
	ctxt->is7E = *((char *)req->buf + (req->actual - 1));
	if (ctxt->is7E == 0x7E)
		wake_up(&ctxt->write_wq);
	smd_try_to_send(ctxt);
}

static void diag_complete_out(struct usb_endpoint *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;
	unsigned char *data = req->buf;
	unsigned int cmd_id = *data;
	unsigned char *table = NULL;
	unsigned char tmp;
	unsigned long flags;

	//DBG("Diag_complete_out -> command ID: %02x\n", cmd_id);

	if (req->status != 0) {
		ctxt->error = 1;
		put_req(ctxt, &ctxt->rx_idle, req);
		wake_up(&ctxt->read_wq);
		return ;
	}
	
	if(cmd_id >= 0xfb && cmd_id <= 0xff) //to ARM11
	{
		if (ctxt->isRead) {
			//DBG("Diag_complete_out to ARM11 -> command ID: %02x\n", cmd_id);
			put_req(ctxt, &ctxt->rx_done, req);
			wake_up(&ctxt->read_wq);
			return;
		}
		goto done;
	}
	
	spin_lock_irqsave(&ctxt->lock_reg_num, flags);
	table = ctxt->id_table;
	while((tmp = *table++)) { //to ARM11
		if (tmp == cmd_id) {
			spin_unlock_irqrestore(&ctxt->lock_reg_num, flags);
			if (ctxt->isRead) {
				put_req(ctxt, &ctxt->rx_done, req);
				wake_up(&ctxt->read_wq);
				return;
			}
			goto done;
		}
	}
	spin_unlock_irqrestore(&ctxt->lock_reg_num, flags);
	//to ARM9
	smd_write(ctxt->ch, data, req->actual);
	smd_xfer_count_func(req->actual, data_set_tx);

done:	
	put_req(ctxt, &ctxt->rx_idle, req);
	diag_queue_out(ctxt);

}

static int msm_diag_probe(struct platform_device *pdev)
{
	struct diag_context *ctxt = &_context;
	int r;

	ctxt->pdev = pdev;
	r = smd_open("SMD_DIAG", &ctxt->ch, ctxt, smd_diag_notify);
	return 0;
}

static struct platform_driver msm_smd_ch1_driver = {
	.probe = msm_diag_probe,
	.driver = {
		.name = DIAG_FUNCTION_NAME,
		.owner = THIS_MODULE,
	},
};

static void diag_plat_release (struct device *dev) {}

static struct platform_device diag_plat_device = {
	.name		= DIAG_FUNCTION_NAME,
	.id		= -1,
	.dev		= {
		.release	= diag_plat_release,
	},
};


static ssize_t diag_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;
	int r = 0, xfer;
	int ret;
	ctxt->isRead = 1;

	DBG("diag_read(%d)\n", count);
	
	if (_lock(&ctxt->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(ctxt->online || ctxt->error)) {
		DBG("diag_read: waiting for online state\n");
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

		while ((req = get_req(ctxt, &ctxt->rx_idle))) {
requeue_req:
			req->length = TXN_MAX;
			ret = usb_ept_queue_xfer(ctxt->out, req);
			if (ret < 0) {
				DBG("diag_read: failed to queue req %p (%d)\n", req, ret);
				r = -EIO;
				ctxt->error = 1;
				put_req(ctxt, &ctxt->rx_idle, req);
				goto fail;
			} else {
				DBG("rx %p queue\n", req);
			}
		}

		/* if we have data pending, give it to userspace */
		if (ctxt->read_count > 0) {
			xfer = (ctxt->read_count < count) ? ctxt->read_count : count;
			if (copy_to_user(buf, ctxt->read_buf, xfer)) {
				DBG("diag: copy_to_user fail\n");
				r = -EFAULT;
				break;
			}
			ctxt->read_buf += xfer;
			ctxt->read_count -= xfer;
			buf += xfer;
			count -= xfer;
			r += xfer;

			/* if we've emptied the buffer, release the request */
			if (ctxt->read_count == 0) {
				put_req(ctxt, &ctxt->rx_idle, ctxt->read_req);
				ctxt->read_req = 0;
			}
			continue;
		}

		/* wait for a request to complete */
		req = 0;
		ret = wait_event_interruptible(ctxt->read_wq,
					       ((req = get_req(ctxt, &ctxt->rx_done)) || ctxt->error));

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
			if (ctxt->read_count < count)
				count = ctxt->read_count;
			DBG("rx %p %d\n", req, req->actual);
		}

		if (ret < 0) {
			r = ret;
			break;
		}
	}

fail:
	_unlock(&ctxt->read_excl);
	ctxt->isRead = 0;
	return r;
}

static ssize_t diag_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG("diag_write(%d)\n", count);
	printk(KERN_ERR "diag_write(%d)\n", count);
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
					       (((req = get_req(ctxt, &ctxt->tx_idle)) && (ctxt->is7E == 0x7E)) || ctxt->error));

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
			printk(KERN_ERR "Diag prepare to send PC\n");
			ret = usb_ept_queue_xfer(ctxt->in, req);
			if (ret < 0) {
				printk(KERN_ERR "diag_write: xfer error\n");
				DBG("diag_write: xfer error %d\n", ret);
				ctxt->error = 1;
				r = -EIO;
				break;
			}
			printk(KERN_ERR "diag_write: success\n");
			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}


	if (req)
		put_req(ctxt, &ctxt->tx_idle, req);

	_unlock(&ctxt->write_excl);
	return r;
}

static int
diag_ioctl(struct inode *inode, struct file *file,
	  unsigned int cmd, unsigned long arg)
{
	struct diag_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;
	int tmp_value;
	unsigned long flags;

	if(_IOC_TYPE(cmd) != USB_DIAG_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
		case USB_DIAG_FUNC_IOC_ENABLE_SET:
			if (copy_from_user(&tmp_value, argp, sizeof(int)))
				return -EFAULT;
			if(tmp_value)	{
				if (_lock(&ctxt->enable_excl))
					return -EBUSY;
				usb_function_enable(DIAG_FUNCTION_NAME, 1);
			}	else	{
				usb_function_enable(DIAG_FUNCTION_NAME, 0);
				_unlock(&ctxt->enable_excl);
			}
			break;
		
		case USB_DIAG_FUNC_IOC_ENABLE_GET:
			tmp_value = (int)return_usb_function_enabled(DIAG_FUNCTION_NAME);
			if (copy_to_user(argp, &tmp_value, sizeof(tmp_value)))
				return -EFAULT;
			break;

		case USB_DIAG_FUNC_IOC_REGISTER_SET:
			spin_lock_irqsave(&ctxt->lock_reg_num, flags);
			if ( copy_from_user(ctxt->id_table, (unsigned char *)argp, sizeof(unsigned char)*TABLE_SIZE) ) {
				spin_unlock_irqrestore(&ctxt->lock_reg_num, flags);
				return -EFAULT;
			}
			spin_unlock_irqrestore(&ctxt->lock_reg_num, flags);
			break;

		case USB_DIAG_FUNC_IOC_AMR_SET:
			if (copy_from_user(&ctxt->is2ARM11, argp, sizeof(int)))
				return -EFAULT;
			//printk(KERN_ERR "IOCTL USB_DIAG_FUNC_IOC_AMR_SET %d\n", ctxt->is2ARM11);
			break;
			
		default:
			return -ENOTTY;
	}

	return 0;
}

static int diag_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;
	
	if (_lock(&ctxt->open_excl))
		return -EBUSY;

	/* clear the error latch */
	ctxt->in_busy	= 0;
	ctxt->error = 0;

	req = usb_ept_alloc_req(ctxt->in, 8192);

	req->context = ctxt;
	req->complete = diag_complete_in;
	put_req(ctxt, &ctxt->tx_idle, req);

	g_bUsbDiagMode = 1;
	
	return 0;
}

static int diag_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	
	ctxt->in_busy	= 0;
	_unlock(&ctxt->open_excl);

	g_bUsbDiagMode = 0;

	return 0;
}

static struct file_operations diag_fops = {
	.owner =   THIS_MODULE,
	.read =    diag_read,
	.write =   diag_write,
	.open =    diag_open,
	.ioctl =   diag_ioctl,
	.release = diag_release,
};

static struct miscdevice diag_device_fops = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "diag",
	.fops = &diag_fops,
};

static int diag2arm9_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;

	if (_lock(&ctxt->open_arm9_excl))
		return -EBUSY;

	/* clear the error latch */
	//ctxt->error = 0;
	return 0;
}

static int diag2arm9_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;

	_unlock(&ctxt->open_arm9_excl);
	return 0;
}

static ssize_t diag2arm9_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	int r = count;
	int writed = 0;

	DBG("diag2arm9_write(%d)\n", count);

	if (_lock(&ctxt->write_arm9_excl))
		return -EBUSY;

	while(count > 0) {
		/*
		if (ctxt->error) {
			r = -EIO;
			break;
		}
		*/
		writed = count > ARM9_MAX ? ARM9_MAX : count;
		if (copy_from_user(ctxt->toARM9_buf, buf, writed)) {
				r = -EFAULT;
				break;
		}
		smd_write(ctxt->ch, ctxt->toARM9_buf, writed);
		smd_xfer_count_func(writed, data_set_tx);
		buf += writed;
		count -= writed;
	}

	_unlock(&ctxt->write_arm9_excl);
	return r;
}

static ssize_t diag2arm9_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;
	int r = 0, xfer;
	int ret;
	//ctxt->isRead = 1;

	DBG("diag2arm9_read(%d)\n", count);
	
	if (_lock(&ctxt->read_arm9_excl))
		return -EBUSY;

	/* we will block until we're offline */
	/*
	while (ctxt->online) {
		ret = wait_event_interruptible(ctxt->read_arm9_wq, !(ctxt->online));
		if (ret < 0) {
			_unlock(&ctxt->read_arm9_excl);
			return ret;
		}
	}
	*/
	while (count > 0) {
		/*
		if (ctxt->error) {
			r = -EIO;
			break;
		}
		*/
		/* if we have idle read requests, get them queued */
		/*
		while ((req = get_req(ctxt, &ctxt->rx_idle))) {
requeue_req:
			req->length = TXN_MAX;
			ret = usb_ept_queue_xfer(ctxt->out, req);
			if (ret < 0) {
				DBG("diag_read: failed to queue req %p (%d)\n", req, ret);
				r = -EIO;
				ctxt->error = 1;
				put_req(ctxt, &ctxt->rx_idle, req);
				goto fail;
			} else {
				DBG("rx %p queue\n", req);
			}
		}
		*/
		
		/* if we have data pending, give it to userspace */
		if (ctxt->read_arm9_count > 0) {
			xfer = (ctxt->read_arm9_count < count) ? ctxt->read_arm9_count : count;
			if (copy_to_user(buf, ctxt->read_arm9_buf, xfer)) {
				DBG("diag: copy_to_user fail\n");
				r = -EFAULT;
				break;
			}
			ctxt->read_arm9_buf += xfer;
			ctxt->read_arm9_count -= xfer;
			buf += xfer;
			count -= xfer;
			r += xfer;

			/* if we've emptied the buffer, release the request */
			if (ctxt->read_arm9_count == 0) {
				put_req(ctxt, &ctxt->rx_arm9_idle, ctxt->read_arm9_req);
				ctxt->read_arm9_req = 0;
			}
			continue;
		}

		/* wait for a request to complete */
		req = 0;
		ret = wait_event_interruptible(ctxt->read_arm9_wq,
					       ((req = get_req(ctxt, &ctxt->rx_arm9_done)) || ctxt->error));

		if (req != 0) {
			/* if we got a 0-len one we need to put it back into
			** service.  if we made it the current read req we'd
			** be stuck forever
			*/
			if (req->actual == 0) {
			//	goto requeue_req;
				put_req(ctxt, &ctxt->rx_arm9_idle, req);
				continue;
			}

			ctxt->read_arm9_req = req;
			ctxt->read_arm9_count = req->actual;
			ctxt->read_arm9_buf = req->buf;
			if (ctxt->read_arm9_count < count)
				count = ctxt->read_arm9_count;
			DBG("rx %p %d\n", req, req->actual);
		}

		if (ret < 0) {
			r = ret;
			break;
		}
	}

//fail:
	_unlock(&ctxt->read_arm9_excl);
	//ctxt->isRead = 0;
	return r;
}

static void diag_dev_release (struct device *dev) {}

static ssize_t show_diag_xfer_count(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int length = 0;
	length = sprintf(buf, "tx_count: %llu, rx_count: %llu\n",
		smd_xfer_count_func(0, data_get_tx), smd_xfer_count_func(0, data_get_rx));
	return length;

}

static DEVICE_ATTR(diag_xfer_count, 0444, show_diag_xfer_count, NULL);

static struct file_operations diag2arm9_fops = {
	.owner =   THIS_MODULE,
	.open =    diag2arm9_open,
	.release = diag2arm9_release,
	.write = diag2arm9_write,
	.read = diag2arm9_read,
};

static struct miscdevice diag2arm9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "diag_arm9",
	.fops = &diag2arm9_fops,
};

static void diag_unbind(void *_ctxt)
{
	struct diag_context *ctxt = _ctxt;
	struct usb_request *req;

	printk(KERN_DEBUG "diag_unbind()\n");

	misc_deregister(&diag_device_fops);
	misc_deregister(&diag2arm9_device);

	device_remove_file(&diag_device, &dev_attr_diag_xfer_count);
	device_unregister(&diag_device);
	smd_xfer_count_func(0,data_set_clear);

	while ((req = get_req(ctxt, &ctxt->rx_idle))) {
		usb_ept_free_req(ctxt->out, req);
	}
	while ((req = get_req(ctxt, &ctxt->tx_idle))) {
		usb_ept_free_req(ctxt->in, req);
	}
	while ((req = get_req(ctxt, &ctxt->rx_arm9_idle))) {
		usb_ept_free_req(ctxt->out, req);
	}

	ctxt->online = 0;
	ctxt->error = 1;

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
	wake_up(&ctxt->read_arm9_wq);
}

static void diag_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct diag_context *ctxt = _ctxt;
	struct usb_request *req;
	int n;

	ctxt->out = ept[0];
	ctxt->in = ept[1];


	printk(KERN_DEBUG "diag_bind() %p, %p\n", ctxt->out, ctxt->in);

	for (n = 0; n < RX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->out, 8192);
		if (req == 0) goto fail;
		req->context = ctxt;
		req->complete = diag_complete_out;
		put_req(ctxt, &ctxt->rx_idle, req);
	}

	for (n = 0; n < TX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->in, 8192);
		if (req == 0) goto fail;
		req->context = ctxt;
		req->complete = diag_complete_in;
		put_req(ctxt, &ctxt->tx_idle, req);
	}

	for (n = 0; n < RX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->out, 8192);
		if (req == 0) goto fail;
		req->context = ctxt;
		put_req(ctxt, &ctxt->rx_arm9_idle, req);
	}

	printk(KERN_DEBUG
	       "diag_bind() allocated %d rx and %d tx requests\n",
	       RX_REQ_MAX, TX_REQ_MAX);

	misc_register(&diag_device_fops);
	misc_register(&diag2arm9_device);

	diag_device.release = diag_dev_release;
	diag_device.parent = &ctxt->pdev->dev;
	strcpy(diag_device.bus_id, "interface");
	if (device_register(&diag_device) != 0) {
		DBG("diag failed to register device\n");
		goto fail;
	}
	if (device_create_file(&diag_device, &dev_attr_diag_xfer_count) != 0) {
		DBG("diag device_create_file failed");
		device_unregister(&diag_device);
		goto fail;
	}
	smd_xfer_count_func(0,data_set_clear);
	return;

fail:
	printk(KERN_WARNING "diag_bind() could not allocate requests\n");
	diag_unbind(ctxt);
}

static void diag_configure(int configured, void *_ctxt)
{
	struct diag_context *ctxt = _ctxt;
	struct usb_request *req;

	DBG("diag_configure() %d\n", configured);

	if (configured) {
		ctxt->online = 1;

		/* if we have a stale request being read, recycle it */
		ctxt->read_buf = 0;
		ctxt->read_count = 0;
		ctxt->read_arm9_buf = 0;
		ctxt->read_arm9_count = 0;
		if (ctxt->read_req) {
			put_req(ctxt, &ctxt->rx_idle, ctxt->read_req);
			ctxt->read_req = 0;
		}
		if (ctxt->read_arm9_req) {
			put_req(ctxt, &ctxt->rx_arm9_idle, ctxt->read_arm9_req);
			ctxt->read_arm9_req = 0;
		}

		/* retire any completed rx requests from previous session */
		while ((req = get_req(ctxt, &ctxt->rx_done)))
			put_req(ctxt, &ctxt->rx_idle, req);
		diag_queue_out(ctxt);
	} else {
		ctxt->online = 0;
		ctxt->error = 1;
	}

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
	wake_up(&ctxt->read_arm9_wq);
}

static struct usb_function usb_func_diag = {
	.bind = diag_bind,
	.unbind = diag_unbind,
	.configure = diag_configure,

	.name = DIAG_FUNCTION_NAME,
	.context = &_context,

	.ifc_class = 0xff,
	.ifc_subclass = 0xff,
	.ifc_protocol = 0xff,

	.ifc_name = "diag",

	.ifc_ept_count = 2,
	.ifc_ept_type = { EPT_BULK_OUT, EPT_BULK_IN },

	/* the diag function is only enabled when its driver file is open */
	.disabled = 1,
	.position_bit = USB_FUNCTION_DIAG_NUM,
	.cdc_desc = NULL,
	.ifc_num = 1,
	.ifc_index = STRING_DIAG,

};

static int __init diag_init(void)
{
	int r, i;
	struct diag_context *ctxt = &_context;
	DBG("diag_init()\n");
	
	ctxt->isRead = 0;
	ctxt->is2ARM11 = 0;
	ctxt->is7E = 0x7E;
	
	init_waitqueue_head(&ctxt->read_wq);
	init_waitqueue_head(&ctxt->write_wq);
	init_waitqueue_head(&ctxt->read_arm9_wq);
	
	atomic_set(&ctxt->open_excl, 0);
	atomic_set(&ctxt->read_excl, 0);
	atomic_set(&ctxt->write_excl, 0);
	atomic_set(&ctxt->enable_excl, 0);
	atomic_set(&ctxt->open_arm9_excl, 0);
	atomic_set(&ctxt->read_arm9_excl, 0);
	atomic_set(&ctxt->write_arm9_excl, 0);
	
	spin_lock_init(&ctxt->lock);
	spin_lock_init(&ctxt->lock_reg_num);

	INIT_LIST_HEAD(&ctxt->rx_idle);
	INIT_LIST_HEAD(&ctxt->rx_done);
	INIT_LIST_HEAD(&ctxt->tx_idle);
	INIT_LIST_HEAD(&ctxt->rx_arm9_idle);
	INIT_LIST_HEAD(&ctxt->rx_arm9_done);

	for (i = 0; i < TABLE_SIZE; i++)
		ctxt->id_table[i] = 0;

	r = platform_driver_register(&msm_smd_ch1_driver);
	if(r < 0)	{
		printk(KERN_ERR "%s: Register driver fail\n", __func__);
		return r;
	}
	r = platform_device_register(&diag_plat_device);
	if (r < 0) {
		printk(KERN_ERR "%s: Register device fail\n", __func__);
		goto fail_register_device;
	}
	r = usb_function_register(&usb_func_diag);
	if (r < 0) {
		printk(KERN_ERR "%s: Register function fail\n", __func__);
		goto fail_function_device;
	}
	return r;
fail_function_device:
	platform_device_unregister(&diag_plat_device);
fail_register_device:
	platform_driver_unregister(&msm_smd_ch1_driver);

	return r;
}

module_init(diag_init);
