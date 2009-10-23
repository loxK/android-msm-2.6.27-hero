/* drivers/input/misc/i2c_matrix_keypad.c
 *
 * Copyright (C) 2007 Google, Inc.
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
 * modify for keypad from i2c via MicroP,
 * Anthony_Chang <anthony_chang@htc.com>
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/i2c_matrix_keypad.h>

static struct workqueue_struct *i2c_keypad_wq;
static const char *version = "0.00.01-beta-2";
#define ENABLE_I2C_KEYPAD_DEBUG

struct i2c_keypad_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	const unsigned short *keymap;
	size_t keymap_size;
	unsigned char break_code_mask;
	uint32_t flags;
	int (*power)(int on);
	u16 microp_sn;	
	unsigned char row_max;
	unsigned char col_max;
	struct mutex i2c_keypad_mutex;
	struct kobject *i2c_keypad_kobj;
	uint8_t keycaps_led;
	uint8_t func_led;
	enum microp_i2c_keypad_serivce service_func;
};
static struct i2c_keypad_data *i2c_key_data_pub;

static int i2c_smbus_read_i2c_block_data_wrapper
	(struct i2c_client *client, u8 command, u8 length, u8 *values)
{
	unsigned short loop_i;
	int ret;
	for(loop_i = 0; loop_i < 100; loop_i++){
		ret = i2c_smbus_read_i2c_block_data(client, command, length, values);
		if(ret > 0)
			break;
		else
			msleep(10);
	}
	return ret;
}

static int i2c_smbus_write_i2c_block_data_wrapper
	(struct i2c_client *client, u8 command, u8 length, u8 *values)
{
	static unsigned short loop_i;
	static int ret;

	for(loop_i = 0; loop_i < 100; loop_i++){
		ret = i2c_smbus_write_i2c_block_data(client, command, length, values);
		if(ret > 0)
			break;
		else
			msleep(10);
	}
	return ret;
}

static ssize_t i2c_keypad_information_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t i = 0;
	
	i += scnprintf(buf + i, PAGE_SIZE - i,
		"keypad driver: %s-%s (0x%4.4X)\n"
		"keypad name: %s\n",
		MICROP_I2C_KEYPAD_DRIVER,
		version,
		i2c_key_data_pub->microp_sn,
		i2c_key_data_pub->input_dev->name);
	return i;
}

static DEVICE_ATTR(i2c_keypad_information, 0444, i2c_keypad_information_show, NULL);

static ssize_t i2c_keycaps_led_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int i = 0;
	uint8_t data[1];
	data[0] = 0x0;

	ret = i2c_smbus_read_i2c_block_data_wrapper(i2c_key_data_pub->client,
		MICROP_I2C_CMD_MISC_REG_WR, sizeof(data), data);
	if(ret <= 0)
		i += scnprintf(buf + i, PAGE_SIZE - i, "-1");
	else
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d", (data[0] & 0x4)?1:0);
	return i;
}

static ssize_t i2c_keycaps_led_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int u;
	uint8_t data[1];

	data[0] = 0x0;
	if(buf[0] == 0x30 || buf[0] == 0x31)
		u = (int)buf[0] - 0x30;
	else
		return 0;
	if(i2c_key_data_pub->func_led != u)	{
		i2c_key_data_pub->func_led = u;
		i2c_key_data_pub->service_func |= 0x1 << keycaps_led_service;
		queue_work(i2c_keypad_wq, &i2c_key_data_pub->work);
	}
	return count;

}

static DEVICE_ATTR(i2c_keycaps_led, 0644,
	i2c_keycaps_led_show, i2c_keycaps_led_store);

static ssize_t i2c_func_led_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int i = 0;
	uint8_t data[1];
	data[0] = 0x0;

	ret = i2c_smbus_read_i2c_block_data_wrapper(i2c_key_data_pub->client,
		MICROP_I2C_CMD_MISC_REG_WR, sizeof(data), data);
	if(ret <= 0)
		i += scnprintf(buf + i, PAGE_SIZE - i, "-1");
	else
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d", (data[0] & 0x2)?1:0);
	return i;
}

static ssize_t i2c_func_led_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int u;
	uint8_t data[1];

	data[0] = 0x0;
	if(buf[0] == 0x30 || buf[0] == 0x31)
		u = (int)buf[0] - 0x30;
	else
		return 0;
	if(i2c_key_data_pub->func_led != u)	{
		i2c_key_data_pub->func_led = u;
		i2c_key_data_pub->service_func |= 0x1 << func_led_service;
		queue_work(i2c_keypad_wq, &i2c_key_data_pub->work);
	}
	return count;
}

static DEVICE_ATTR(i2c_func_led, 0644,
	i2c_func_led_show, i2c_func_led_store);

#ifdef ENABLE_I2C_KEYPAD_DEBUG
static ssize_t i2c_keypad_keymap_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t i = 0;
	unsigned short loop_i, loop_j;
	
	for(loop_i = 0; loop_i < i2c_key_data_pub->row_max; loop_i++)	{
		for(loop_j = 0; loop_j < i2c_key_data_pub->col_max; loop_j++)	{
			i += scnprintf(buf + i, PAGE_SIZE - i,
				"key[%2d, %2d]: KeyCode: 0x%2.2X(%3d)\n",
				loop_i, loop_j,
				i2c_key_data_pub->keymap[(loop_i * i2c_key_data_pub->row_max) + loop_j],
				i2c_key_data_pub->keymap[(loop_i * i2c_key_data_pub->row_max) + loop_j]);
		}
		i += scnprintf(buf + i, PAGE_SIZE - i, "\n");
	}
	return i;
}

static DEVICE_ATTR(i2c_keypad_dump_key_mapping, 0444, i2c_keypad_keymap_show, NULL);
#endif /* ENABLE_I2C_KEYPAD_DEBUG */

static int i2c_keypad_sysfs_init(void)
{
	int ret;

	i2c_key_data_pub->i2c_keypad_kobj = kobject_create_and_add("android_i2c_keypad", NULL);
	if (i2c_key_data_pub->i2c_keypad_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_file(i2c_key_data_pub->i2c_keypad_kobj, &dev_attr_i2c_keypad_information.attr);
	if (ret) {
		printk(KERN_ERR
		       "%s: sysfs_create_group failed\n", __func__);
		goto err_1;
	}
	ret = sysfs_create_file(i2c_key_data_pub->i2c_keypad_kobj, &dev_attr_i2c_keycaps_led.attr);
	if (ret) {
		printk(KERN_ERR
		       "%s: sysfs_create_group failed\n", __func__);
		goto err_2;
	}

	ret = sysfs_create_file(i2c_key_data_pub->i2c_keypad_kobj, &dev_attr_i2c_func_led.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_group failed\n", __func__);
		goto err_3;
	}

#ifdef ENABLE_I2C_KEYPAD_DEBUG
	ret = sysfs_create_file(i2c_key_data_pub->i2c_keypad_kobj, &dev_attr_i2c_keypad_dump_key_mapping.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_group failed\n", __func__);
		goto err_4;
	}
#endif /* ENABLE_I2C_KEYPAD_DEBUG */
	return 0;

#ifdef ENABLE_I2C_KEYPAD_DEBUG
err_4:
	sysfs_remove_file(i2c_key_data_pub->i2c_keypad_kobj, &dev_attr_i2c_func_led.attr);
#endif /* ENABLE_I2C_KEYPAD_DEBUG */
err_3:
	sysfs_remove_file(i2c_key_data_pub->i2c_keypad_kobj, &dev_attr_i2c_keycaps_led.attr);
err_2:
	sysfs_remove_file(i2c_key_data_pub->i2c_keypad_kobj, &dev_attr_i2c_keypad_information.attr);
err_1:
	kobject_del(i2c_key_data_pub->i2c_keypad_kobj);
	return ret;
}

static void i2c_keypad_service_func(void)
{
	static unsigned char data[16];
	static unsigned short keycode;
	static int ret, pressed, loop_i;

	memset(data, 0x0, sizeof(data));

	ret = i2c_smbus_read_i2c_block_data_wrapper(i2c_key_data_pub->client,
		MICROP_I2C_CMD_INPUT_PIN_STATUS_REG_RO, sizeof(data), data);
	if(ret < 0)	{
		printk(KERN_ERR "%s: read sliding pin status fail...\n", __func__);
		return;
	}
	ret = i2c_smbus_read_i2c_block_data_wrapper(i2c_key_data_pub->client,
		MICROP_I2C_CMD_KEYSCAN_RESULT_REG_RO, sizeof(data), data);
	if(ret < 0)	{
		printk(KERN_ERR "%s: read scan keypad status fail...\n", __func__);
		return;
	}
	/*
	if(data[0] == 0x00 || data[0] == 0xFF)	{
		//mdelay(100);
		//schedule_timeout(HZ/100);
		return;
	}
	*/
#ifdef ENABLE_I2C_KEYPAD_DEBUG
	if(ret > 0)
		for(loop_i = 0; loop_i < 2; loop_i++)	{
		printk(KERN_DEBUG "%s: get: data[%2d]: 0x%2.2X ##\n",
			__func__, loop_i, data[loop_i]);
		}
	else
		printk(KERN_DEBUG "%s: get empty data... ##\n", __func__);
#endif
	pressed = (data[0] & i2c_key_data_pub->break_code_mask)?0:1;
	if(!pressed)
		data[0] &= ~i2c_key_data_pub->break_code_mask;
			keycode = i2c_key_data_pub->keymap[(data[0] - 1)];
	if(keycode != 0xFF)	{
		input_report_key(i2c_key_data_pub->input_dev, keycode, pressed);
		printk(KERN_INFO "%s: input_report_key: code: %3d, pressed: %d\n", __func__, keycode, pressed);
	}	else
		printk(KERN_WARNING "%s: get unknown value: 0x%X, 0x%X (%d)\n\n", __func__, data[0], data[1], keycode);
	mdelay(10);
}

static void i2c_keypad_service_led_func(enum microp_i2c_keypad_serivce leds_func)
{
	static unsigned char data[1];
	static int ret;
	ret = i2c_smbus_read_i2c_block_data_wrapper(i2c_key_data_pub->client,
		MICROP_I2C_CMD_MISC_REG_WR, sizeof(data), data);
	if(ret < 0)	{
		printk(KERN_ERR "%s: read sliding pin status fail...\n", __func__);
		return;
	}
	switch(leds_func)	{
		case func_led_service:
			if(i2c_key_data_pub->func_led)	//enable
				data[0] |= 0x2;
			else
				data[0] &= ~0x2;
		break;
		case keycaps_led_service:
			if(i2c_key_data_pub->keycaps_led)	//enable
				data[0] |= 0x4;
			else
				data[0] &= ~0x4;
		break;
		default:
			printk(KERN_ERR "%s: get unknown func_num\n", __func__);
			return;
		break;
	}
	ret = i2c_smbus_write_i2c_block_data_wrapper(i2c_key_data_pub->client,
		MICROP_I2C_CMD_MISC_REG_WR, sizeof(data), data);
	if(ret > 0)	{
	}	else	{
		printk(KERN_ERR "%s: failed on set data\n", __func__);
		return;
	}
}

static void i2c_keypad_work_func(struct work_struct *work)
{
	static unsigned need_enable_irq = 0;
	if(mutex_lock_interruptible(&i2c_key_data_pub->i2c_keypad_mutex) < 0)	{
		printk(KERN_WARNING "%s: mutex_lock_int failure...\n\n", __func__);
		return;
	}
	need_enable_irq = 0;

	if(i2c_key_data_pub->service_func & 0x1 << keypad_service)	{
		need_enable_irq = 1;
		i2c_keypad_service_func();
		i2c_key_data_pub->service_func &= ~(0x1 << keypad_service);
	}	else
	if(i2c_key_data_pub->service_func & 0x1 << func_led_service)	{
		i2c_keypad_service_led_func(func_led_service);
		i2c_key_data_pub->service_func &= ~(0x1 << func_led_service);
	}	else
	if(i2c_key_data_pub->service_func & 0x1 << keycaps_led_service)	{
		i2c_keypad_service_led_func(keycaps_led_service);
		i2c_key_data_pub->service_func &= ~(0x1 << keycaps_led_service);
	}

	mutex_unlock(&i2c_key_data_pub->i2c_keypad_mutex);
	if (need_enable_irq)
		enable_irq(i2c_key_data_pub->client->irq);

}
#if 0
static enum hrtimer_restart i2c_keypad_timer_func(struct hrtimer *timer)
{
	struct i2c_keypad_data *i2c_key_data = container_of(timer, struct i2c_keypad_data, timer);
	/* printk("i2c_keypad_timer_func\n"); */

	queue_work(i2c_keypad_wq, &i2c_key_data->work);

	hrtimer_start(&i2c_key_data->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#endif
static irqreturn_t i2c_keypad_irq_handler(int irq, void *dev_id)
{
	struct i2c_keypad_data *i2c_key_data = dev_id;

	/* printk("i2c_keypad_irq_handler\n"); */
	disable_irq(i2c_key_data->client->irq);
	i2c_key_data_pub->service_func |= 0x1 << keypad_service;
	queue_work(i2c_keypad_wq, &i2c_key_data->work);

	//enable_irq(i2c_key_data->client->irq);
	return IRQ_HANDLED;
}

static int i2c_keypad_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	uint8_t data[2];
	int ret = 0;
	int loop_i;
	struct i2c_matrix_keypad_platform_data *pdata;

	printk(KERN_INFO "%s: run...\n\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	i2c_key_data_pub = kzalloc(sizeof(*i2c_key_data_pub), GFP_KERNEL);
	if (i2c_key_data_pub == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	INIT_WORK(&i2c_key_data_pub->work, i2c_keypad_work_func);

	i2c_key_data_pub->client = client;
	i2c_set_clientdata(client, i2c_key_data_pub);
	pdata = client->dev.platform_data;
	if (pdata)
		i2c_key_data_pub->power = pdata->power;
	if (i2c_key_data_pub->power) {
		ret = i2c_key_data_pub->power(1);
		if (ret < 0) {
			printk(KERN_ERR "%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}

	if(pdata->keymap)	{
		i2c_key_data_pub->keymap = pdata->keymap;
		i2c_key_data_pub->keymap_size = pdata->keymap_size;
		i2c_key_data_pub->break_code_mask = pdata->break_code_mask;
		i2c_key_data_pub->row_max = pdata->row_max;
		i2c_key_data_pub->col_max = pdata->col_max;
	}
	
	if (i2c_smbus_read_i2c_block_data_wrapper(client, MICROP_I2C_CMD_VERSION, sizeof(data), data) < 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed on get microp main version\n");
		goto err_detect_failed;
	}
	i2c_key_data_pub->microp_sn = data[0] | (data[1] << 8);
	if (i2c_smbus_read_i2c_block_data_wrapper(client, MICROP_I2C_CMD_INFORMATION_REG_RO, sizeof(data), data) < 0)	{
		ret = -ENODEV;
		dev_err(&client->dev, "failed on get microp keypad version\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "%s: Product Major Version 0x%x, 0x%x (%d)\n", __func__, data[0], data[1], ret);
/*
	data[0] = 0x0;
	if (i2c_smbus_write_i2c_block_data_wrapper(client, MICROP_I2C_CMD_MISC_REG_WR, 1, data) < 0)	{
		ret = -ENODEV;
		dev_err(&client->dev, "failed on set microp keypad mode\n");
		goto err_detect_failed;
	}
*/
	i2c_key_data_pub->input_dev = input_allocate_device();
	if (i2c_key_data_pub->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "%s: Failed to allocate input device\n", __func__);
		goto err_input_dev_alloc_failed;
	}

	i2c_key_data_pub->input_dev->name = pdata->keypad_name;
	set_bit(EV_KEY, i2c_key_data_pub->input_dev->evbit);
	for(loop_i = 0; loop_i < i2c_key_data_pub->keymap_size; loop_i++)	{
		if (i2c_key_data_pub->keymap[loop_i])
			set_bit(i2c_key_data_pub->keymap[loop_i] & KEY_MAX,
				i2c_key_data_pub->input_dev->keybit);
	}

	if (input_register_device(i2c_key_data_pub->input_dev)) {
		printk(KERN_ERR "%s: Unable to register %s input device\n", __func__,
				i2c_key_data_pub->input_dev->name);
		goto err_input_register_device_failed;
	}

	mutex_init(&i2c_key_data_pub->i2c_keypad_mutex);
	if (client->irq) {
		ret = request_irq(i2c_key_data_pub->client->irq, i2c_keypad_irq_handler, 0,
				i2c_key_data_pub->input_dev->name, i2c_key_data_pub);
		if (ret == 0)
			i2c_key_data_pub->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	/*
	if (!i2c_key_data_pub->use_irq) {
		hrtimer_init(&i2c_key_data_pub->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		i2c_key_data_pub->timer.function = i2c_keypad_timer_func;
		hrtimer_start(&i2c_key_data_pub->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	*/
	printk(KERN_INFO "%s: Start %s in %s mode\n", __func__,
	i2c_key_data_pub->input_dev->name, i2c_key_data_pub->use_irq ? "interrupt" : "polling");

	if(i2c_keypad_sysfs_init() < 0)
		printk(KERN_ERR "%s: initial /sysfs failure...\n\n", __func__);

	return 0;

err_input_register_device_failed:
	input_free_device(i2c_key_data_pub->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
err_power_failed:
	kfree(i2c_key_data_pub);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int i2c_keypad_remove(struct i2c_client *client)
{
	struct i2c_keypad_data *i2c_key_client = i2c_get_clientdata(client);
#ifdef ENABLE_I2C_KEYPAD_DEBUG
	sysfs_remove_file(i2c_key_data_pub->i2c_keypad_kobj, 
		&dev_attr_i2c_keypad_dump_key_mapping.attr);
#endif
	sysfs_remove_file(i2c_key_data_pub->i2c_keypad_kobj, 
		&dev_attr_i2c_keypad_information.attr);

	kobject_del(i2c_key_data_pub->i2c_keypad_kobj);
	if (i2c_key_data_pub->use_irq)
		free_irq(client->irq, i2c_key_client);
	else
		hrtimer_cancel(&i2c_key_data_pub->timer);
	input_unregister_device(i2c_key_data_pub->input_dev);

	kfree(i2c_key_client);
	return 0;
}

static int i2c_keypad_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int i2c_keypad_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id i2c_keypad_id[] = {
	{ MICROP_I2C_KEYPAD_DRIVER, 0 },
	{ }
};

static struct i2c_driver i2c_keypad_driver = {
	.probe		= i2c_keypad_probe,
	.remove		= i2c_keypad_remove,
	.suspend	= i2c_keypad_suspend,
	.resume		= i2c_keypad_resume,
	.id_table	= i2c_keypad_id,
	.driver = {
		.name	= MICROP_I2C_KEYPAD_DRIVER,
	},
};

static int __devinit i2c_keypad_init(void)
{
	printk(KERN_INFO "%s: start initial...\n\n\n", __func__);
	i2c_keypad_wq = create_singlethread_workqueue("i2c_keypad_wq");
	if (!i2c_keypad_wq)
		return -ENOMEM;
	return i2c_add_driver(&i2c_keypad_driver);
}

static void __exit i2c_keypad_exit(void)
{
	i2c_del_driver(&i2c_keypad_driver);
	if (i2c_keypad_wq)
		destroy_workqueue(i2c_keypad_wq);
}

module_init(i2c_keypad_init);
module_exit(i2c_keypad_exit);

MODULE_DESCRIPTION("i2c-matrix-keypad Driver");
MODULE_AUTHOR("Anthony Chang <anthony_chang@htc.com>");
MODULE_LICENSE("GPL");
