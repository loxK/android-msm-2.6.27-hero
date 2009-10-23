/* drivers/input/touchscreen/melfas_tsi.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (C) 2009 HTC Inc.
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
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/melfas_tsi.h>
#include <linux/io.h>
#include <linux/earlysuspend.h>

#define MELFAS_I2C_READ_RETRY_TIMES 10
#define swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

static struct workqueue_struct *melfas_wq;

struct melfas_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint32_t flags;
	int (*power)(int on);
	struct early_suspend early_suspend;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

struct melfas_touch_key_data {
	int status;
	int keycode;
	int range_min;
	int range_max;
};

static struct melfas_touch_key_data touch_keys[4] = {
	{
		.status = 0,
		.keycode = KEY_HOME,
		.range_min = 1,
		.range_max = 30,
	},
	{
		.status = 0,
		.keycode = KEY_MENU,
		.range_min = 60,
		.range_max = 120,
	},
	{
		.status = 0,
		.keycode = KEY_BACK,
		.range_min = 200,
		.range_max = 250,
	},
	{
		.status = 0,
		.keycode = KEY_COMPOSE,
		.range_min = 295,
		.range_max = 315,
	},
};

#if 0
static int melfas_init_panel(struct melfas_ts_data *ts)
{
	int ret;
	printk(KERN_INFO "%s\n", __func__);
	return ret;
}
#endif
#if 0
static int melfas_i2c_read_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};
	for (retry = 0; retry <= MELFAS_I2C_READ_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msgs, 1) == 1) {
			if (i2c_transfer(client->adapter, &msgs[1], 1) == 1)
				break;
		}
			mdelay(10);
	}
	dev_dbg(&client->dev, "R [%02X] = %s\n", addr, hex2string(data, length));
	if (retry > MELFAS_I2C_READ_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_read_block retry over %d\n",
			MELFAS_I2C_READ_RETRY_TIMES);
		return -EIO;
	}
	return 0;
}

static int melfas_i2c_write_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[10];
	int i;

	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = 0,
		.len = length + 1,
		.buf = buf,
		}
	};

	//dev_dbg(&client->dev, "W [%02X] = %s\n", addr, hex2string(data, length));
	if (length + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++) {
		buf[i+1] = data[i];
	}

	for (retry = 0; retry <= MELFAS_I2C_READ_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}

	if (retry > MELFAS_I2C_READ_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_write_block retry over %d\n",
			MELFAS_I2C_READ_RETRY_TIMES);
		return -EIO;
	}
	return 0;
}
#endif
static void melfas_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	int bad_data = 0;
	uint8_t buf[10];	
	int k;
	struct melfas_ts_data *ts =
		container_of(work, struct melfas_ts_data, work);

	for (i = 0; i < ((ts->use_irq && !bad_data) ? 1 : 1); i++) {
		ret = i2c_smbus_read_i2c_block_data(ts->client, 0x10, sizeof(buf), buf);
		if (ret < 0) {
			printk(KERN_ERR "%s: i2c_transfer failed\n", __func__);
		} else {
			bad_data = 0;
			if (0)
				break;
			else {
				int pos[2][2];
				int z = buf[5];
				int w = buf[6];
				int finger = buf[0] & 7; /*Read from input information register to get the finger number*/
				int finger2_pressed;
				/* int touch_keydown = buf[9]; */
				uint16_t key_x_axis = (buf[8] & 0xFF) | ((buf[9] & 0x18) << 5);
				//uint16_t key_x_axis = buf[9] & 0x7;
					//printk(KERN_INFO "%s: key_x_axis: %d (0x%2.2X, 0x%2.2X 0x%2.2X) ##\n",
					 //	__func__, key_x_axis, key_x_axis, buf[8], buf[9]);
				/* finger 1 */
				pos[0][0] =  (uint16_t) (buf[2] & 0xFF) | (uint16_t) (buf[1] & 0x3) << 8;
				pos[0][1] =  (uint16_t) (buf[4] & 0xFF) | (uint16_t) (buf[3] & 0x3) << 8;

				if (1) {  /*if (z) {	//skip the unsupported parameter*/
					input_report_abs(ts->input_dev, ABS_X, pos[0][0]);
					input_report_abs(ts->input_dev, ABS_Y, pos[0][1]);
				}

				input_report_abs(ts->input_dev, ABS_PRESSURE, z);
				input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
				input_report_key(ts->input_dev, BTN_TOUCH, finger);
				finger2_pressed = finger > 1 && finger != 7;
				input_report_key(ts->input_dev, BTN_2, finger2_pressed);

				pos[1][0] = pos[1][1] = 0;
				if (finger2_pressed) {
					uint8_t buff[4];
					#if 0
					msg[0].addr = ts->client->addr;
					msg[0].flags = 0;
					msg[0].len = 1;
					msg[0].buf = &start_reg;
					start_reg = 0xD1; /* start from 2nd */
					msg[1].addr = ts->client->addr;
					msg[1].flags = I2C_M_RD;
					msg[1].len = 4;
					msg[1].buf = buff;
					ret = i2c_transfer(ts->client->adapter, msg, 2);
					#endif
					ret = i2c_smbus_read_i2c_block_data(ts->client, 0xD1, sizeof(buff), buff);
					if (ret < 0) {
						printk(KERN_ERR "%s:(1): i2c_transfer failed\n", __func__);
					}
					/* finger 2 */
					pos[1][0] =  (uint16_t) (buff[1] & 0xFF) | (uint16_t) (buff[0] & 0x3) << 8;
					pos[1][1] =  (uint16_t) (buff[3] & 0xFF) | (uint16_t) (buff[2] & 0x3) << 8;
					input_report_abs(ts->input_dev, ABS_HAT0X, pos[1][0]);
					input_report_abs(ts->input_dev, ABS_HAT0Y, pos[1][1]);

				}
				input_sync(ts->input_dev);
				if (key_x_axis) {
					for (k = 0; k < 4; k++) {
						if (key_x_axis >= touch_keys[k].range_min && 
							key_x_axis <= touch_keys[k].range_max && !touch_keys[k].status) {
							//printk(KERN_DEBUG "%s: touch_key DOWN: 0x%X (%d) ++\n",
							//	__func__, touch_keys[k].keycode, key_x_axis);
							touch_keys[k].status = 1;
							input_report_key(ts->input_dev, touch_keys[k].keycode, 1);
							break;
						} else
						if ((key_x_axis < touch_keys[k].range_min || 
							key_x_axis > touch_keys[k].range_max) && touch_keys[k].status) {
							//printk(KERN_DEBUG "%s: touch UP: 0x%X (%d) ++\n",
							//	__func__, touch_keys[k].keycode, key_x_axis);
							touch_keys[k].status = 0;
							input_report_key(ts->input_dev, touch_keys[k].keycode, 0);
							break;
						}
					}
 				} else {
					for (k = 0; k < 4; k++) {
						if(touch_keys[k].status) {
							//printk(KERN_DEBUG "%s: touch_key UP:%x 0x%X (%d) --\n",
							//	__func__, k, touch_keys[k].keycode, key_x_axis);
							touch_keys[k].status = 0;
							input_report_key(ts->input_dev, touch_keys[k].keycode, 0);
						}
					}
				}
				input_sync(ts->input_dev);
			}
		}
	}

	if (ts->use_irq)
		enable_irq(ts->client->irq);

}

static enum hrtimer_restart melfas_ts_timer_func(struct hrtimer *timer)
{
	struct melfas_ts_data *ts = container_of(timer, \
					struct melfas_ts_data, timer);
	queue_work(melfas_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	struct melfas_ts_data *ts = dev_id;

	disable_irq(ts->client->irq);
	queue_work(melfas_wq, &ts->work);
	return IRQ_HANDLED;
}

static int melfas_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts;
	int ret = 0;
	uint16_t max_x, max_y;
	struct melfas_i2c_rmi_platform_data *pdata;
	//uint32_t panel_version;	
	int retry = 10;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, melfas_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}
	msleep(200);

	/* Operation Mode register */
	ret = i2c_smbus_write_byte_data(ts->client, 0x01, 0x82);
	if (ret < 0) {
		printk(KERN_ERR "%s: i2c_smbus_write_byte_data failed (0x01, 0x12)\n", __func__);
		/* fail? */
	}


	while (retry-- > 0) {
		ret = i2c_smbus_read_byte_data(ts->client, 0x20);
		if (ret >= 0)
			break;
		msleep(100);
	}

	if (ret < 0) {
		printk(KERN_ERR "%s: i2c_smbus_read_byte_data failed (0x20) \n", __func__);
		goto err_detect_failed;
	}
	printk(KERN_INFO "%s: Product Major Version %x\n", __func__, ret);

	ret = i2c_smbus_read_byte_data(ts->client, 0x01);
	if (ret < 0) {
		printk(KERN_ERR "%s: i2c_smbus_read_byte_data failed\n", __func__);
		goto err_detect_failed;
	}
	printk(KERN_INFO "%s: Operation Mode %x\n", __func__, ret);

	/* Get the max_x and max_y info */
	ret = i2c_smbus_read_word_data(ts->client, 0x08);
	if (ret < 0) {
		printk(KERN_ERR "%s: i2c_smbus_read_word_data failed (0x08)\n", __func__);
		goto err_detect_failed;
	}
	max_x = (ret & 0xFF) << 8 | (ret >> 8);

	printk(KERN_INFO "%s: i2c_smbus_read_word_data (0x08, %x)\n", __func__, max_x);

	ret = i2c_smbus_read_word_data(ts->client, 0x0A);
	if (ret < 0) {
		printk(KERN_ERR "%s: i2c_smbus_read_word_data failed (0x0A)\n", __func__);
		goto err_detect_failed;
	}
	max_y = (ret & 0xFF) << 8 | (ret >> 8);
	printk(KERN_INFO "%s: i2c_smbus_read_word_data (0x08, %x)\n", __func__, max_y);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "%s: Failed to allocate input device\n", __func__);
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "melfas-tsi";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_COMPOSE, ts->input_dev->keybit);

	printk(KERN_INFO "%s: max_x %d, max_y %d\n", __func__, max_x, max_y);

	input_set_abs_params(ts->input_dev, ABS_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0Y, 0, max_y, 0, 0);
	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	printk(KERN_ERR "melfas_ts_probe: client irq %d", client->irq);

	if (client->irq) {
		ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_LOW, client->name, ts);
		if (ret == 0) {
			/* default value of 0x01 register should be 0x12 => it should have been changed to interrupt mode */
			ret = i2c_smbus_read_byte_data(ts->client, 0x01);
			if (ret & (1 << 6)) {
				free_irq(client->irq, ts);
				printk(KERN_ERR "%s: free client irq %d\n", __func__, client->irq);
			} else
				ret = 0;
		}
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}


	/* poll on status Added by Huimin to check the abnormal polling behavior */
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = melfas_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "%s: Start touchscreen %s in %s mode\n",
		__func__, ts->input_dev->name,
		ts->use_irq ? "interrupt" : "polling");
	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}


static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	/* if work was pending disable-count is now 2 */
	if (ret && ts->use_irq)
		enable_irq(client->irq);

	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0) {
			printk(KERN_ERR "%s: power off failed\n", __func__);
		}
	}
	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	int ret;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "%s: power on failed\n", __func__);
		}
	}
	msleep(200);

	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	else {
		/* enable abs int */
		i2c_smbus_write_byte_data(ts->client, 0x01, 0x02);
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ MELFAS_I2C_RMI_NAME, 0 },
	{ }
};

static struct i2c_driver melfas_ts_driver = {
	.probe		= melfas_ts_probe,
	.remove		= melfas_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
	.id_table	= melfas_ts_id,
	.driver = {
		.name	= MELFAS_I2C_RMI_NAME,
	},
};

static int __devinit melfas_ts_init(void)
{
	melfas_wq = create_singlethread_workqueue("melfas_wq");
	if (!melfas_wq)
		return -ENOMEM;
	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
	if (melfas_wq)
		destroy_workqueue(melfas_wq);
}

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("melfas Touchscreen Driver");
MODULE_LICENSE("GPL");
