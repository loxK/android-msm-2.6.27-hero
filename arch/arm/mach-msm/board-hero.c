/* linux/arch/arm/mach-msm7201a/board-hero.c
 *
 * Copyright (C) 2008 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/keyreset.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <linux/synaptics_i2c_rmi.h>
#include <mach/cy8c_i2c.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/sysdev.h>
#include <linux/delay.h>

#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/setup.h>

#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/sdio_ids.h>

#include <mach/system.h>
#include <mach/vreg.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_fb.h>
#include <mach/proc_comm.h>
#include <mach/devices.h>
#include <mach/h2w.h>
#include <mach/microp_i2c.h>
#include <mach/htc_pwrsink.h>
#include <mach/perflock.h>
#include <mach/drv_callback.h>

#include "board-hero.h"
#include "board-hero-camsensor.h"
static unsigned int hwid = 0;
static unsigned int skuid = 0;
static unsigned int engineerid = 0;

struct hero_axis_info {
	struct gpio_event_axis_info info;
	uint16_t in_state;
	uint16_t out_state;
	uint16_t temp_state;
	uint16_t threshold;
};

static bool nav_just_on;
static int nav_on_jiffies;

unsigned int hero_get_hwid(void)
{
	return hwid;
}

unsigned int hero_get_skuid(void)
{
	return skuid;
}

unsigned hero_get_engineerid(void)
{
	return engineerid;
}

uint16_t hero_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct hero_axis_info *ai = container_of(info, struct hero_axis_info, info);
	uint16_t out = ai->out_state;

	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if((ai->in_state ^ in) & 1)
		out--;
	if((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	if (ai->out_state - ai->temp_state == ai->threshold) {
		ai->temp_state++;
		ai->out_state = ai->temp_state;
	} else if (ai->temp_state - ai->out_state == ai->threshold) {
		ai->temp_state--;
		ai->out_state = ai->temp_state;
	} else if (abs(ai->out_state - ai->temp_state) > ai->threshold)
		ai->temp_state = ai->out_state;

	return ai->temp_state;
}

int hero_nav_power(const struct gpio_event_platform_data *pdata, bool on)
{
	gpio_set_value(HERO_GPIO_JOGBALL_EN, on);
	if (on) {
		nav_just_on = 1;
		nav_on_jiffies = jiffies;
	}
	return 0;
}

static uint32_t hero_x_axis_gpios[] = {
	HERO_GPIO_JOGBALL_LEFT_0, HERO_GPIO_JOGBALL_RIGHT_0
};

static struct hero_axis_info hero_x_axis = {
	.threshold = 1,
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(hero_x_axis_gpios),
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(hero_x_axis_gpios),
		.map = hero_axis_map,
		.gpio = hero_x_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION /*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */,
		.enable_emc_protect_delay = 1 * NSEC_PER_MSEC,
	}
};

static uint32_t hero_y_axis_gpios[] = {
	HERO_GPIO_JOGBALL_UP_0, HERO_GPIO_JOGBALL_DOWN_0
};

static struct hero_axis_info hero_y_axis = {
	.threshold = 1,
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(hero_y_axis_gpios),
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(hero_y_axis_gpios),
		.map = hero_axis_map,
		.gpio = hero_y_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION /*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT  */,
		.enable_emc_protect_delay = 1 * NSEC_PER_MSEC,
	}
};

static struct gpio_event_info *hero_nav_info[] = {
	&hero_x_axis.info.info,
	&hero_y_axis.info.info,
};

static struct gpio_event_platform_data hero_nav_data = {
	.name = "hero-nav",
	.info = hero_nav_info,
	.info_count = ARRAY_SIZE(hero_nav_info),
	.power = hero_nav_power,
};

static struct platform_device hero_nav_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 2,
	.dev		= {
		.platform_data	= &hero_nav_data,
	},
};

static int hero_reset_keys_up[] = {
	BTN_MOUSE,
	0
};

static struct keyreset_platform_data hero_reset_keys_pdata = {
	.keys_up = hero_reset_keys_up,
	.keys_down = {
		KEY_SEND,
		KEY_MENU,
		KEY_END,
		0
	},
};

static struct platform_device hero_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &hero_reset_keys_pdata,
};

static int hero_ts_power(int on)
{
	printk(KERN_INFO "hero_ts_power:%d\n", on);
	if (on) {
		gpio_set_value(HERO_GPIO_TP_EN, 1);
		msleep(250);
		/* enable touch panel level shift */
		gpio_set_value(HERO_TP_LS_EN, 1);
		msleep(2);
	} else {
		gpio_set_value(HERO_TP_LS_EN, 0);
		udelay(50);
		gpio_set_value(HERO_GPIO_TP_EN, 0);
	}
	printk(KERN_INFO "hero_ts_power:%d done\n", on);
	return 0;
}

static struct cy8c_i2c_platform_data hero_cypress_ts_data = {
	.version = 0x0001,
	.abs_x_min = 0,
	.abs_x_max = 319,
	.abs_y_min = 0,
	.abs_y_max = 479,
	.abs_pressure_min = 0,
	.abs_pressure_max = 255,
	.abs_width_min = 0,
	.abs_width_max = 15,
	.power = hero_ts_power,
};

static struct synaptics_i2c_rmi_platform_data hero_ts_data[] = {
	{
		.version = 0x0101,
		.power = hero_ts_power,
		.sensitivity = 7,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -50 * 0x10000 / 4334,
		.inactive_right = -50 * 0x10000 / 4334,
		.inactive_top = -40 * 0x10000 / 6696,
		.inactive_bottom = -40 * 0x10000 / 6696,
		.snap_left_on = 50 * 0x10000 / 4334,
		.snap_left_off = 60 * 0x10000 / 4334,
		.snap_right_on = 50 * 0x10000 / 4334,
		.snap_right_off = 60 * 0x10000 / 4334,
		.snap_top_on = 100 * 0x10000 / 6696,
		.snap_top_off = 110 * 0x10000 / 6696,
		.snap_bottom_on = 100 * 0x10000 / 6696,
		.snap_bottom_off = 110 * 0x10000 / 6696,
		.display_width = 320,
		.display_height = 480,
	},
	{
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334,
		.inactive_right = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334,
		.inactive_top = ((6946 - 6696) / 2) * 0x10000 / 6696,
		.inactive_bottom = ((6946 - 6696) / 2) * 0x10000 / 6696,
		.display_width = 320,
		.display_height = 480,
	}
};

static struct msm_camera_device_platform_data hero_mt9p012_device_data = {
	.sensor_reset	= HERO_GPIO_CAM_RST_N,
	.sensor_pwd	= HERO_CAM_PWDN,
	.vcm_pwd	= HERO_GPIO_VCM_PWDN,
	.config_gpio_on = config_hero_camera_on_gpios,
	.config_gpio_off = config_hero_camera_off_gpios,
};

// for libqcamera sensor interface
static struct msm_camsensor_device_platform_data hero_s5k3e2fx_sensor_info = {
	.sensor_i2c_read = hero_s5k3e2fx_i2c_read,
	.sensor_i2c_write = hero_s5k3e2fx_i2c_write,
	.sensor_probe_initial = hero_s5k3e2fx_probe_init,
	.sensor_deinit = hero_s5k3e2fx_sensor_deinit,
	.sensor_write_exposuregain =  hero_s5k3e2fx_write_exposuregain,
	.sensor_set_pclk = hero_s5k3e2fx_set_pclk,
	.sensor_setting = hero_s5k3e2fx_sensor_setting,
	.sensor_resume = hero_s5k3e2fx_resume,
	.sensor_suspend = hero_s5k3e2fx_suspend,
	.sensor_power_down = hero_s5k3e2fx_power_down,
	.sensor_power_up = hero_s5k3e2fx_power_up,
	.msmclk_rate_set = hero_msm_camio_clk_rate_set,
	.msmclk_disable = hero_msm_camio_clk_disable,
	.msmclk_enable = hero_msm_camio_clk_enable,
	.msmclk_camif_clk_select = hero_camif_clk_select,
	.msmio_camif_pad_reg_reset = hero_s5k3e2fx_camif_pad_reg_reset,
	.msmio_camif_app_reset = hero_s5k3e2fx_camif_app_reset,
	.msmio_camif_Reset2 = hero_s5k3e2fx_camif_reset2,
};

static struct msm_camera_device_platform_data hero_s5k3e2fx_device_data = {
	.sensor_reset	= HERO_GPIO_CAM_RST_N,
	.sensor_pwd	= HERO_CAM_PWDN,
	.vcm_pwd	= HERO_GPIO_VCM_PWDN,
	.config_gpio_on = config_hero_camera_on_gpios,
	.config_gpio_off = config_hero_camera_off_gpios,
	.sensor_info = &hero_s5k3e2fx_sensor_info,
};

static int hero_microp_intr_debounce(uint8_t *pin_status);
static void hero_microp_intr_function(uint8_t *pin_status);

static struct microp_pin_config microp_pins_skuid_0[] = {
	MICROP_PIN(23, MICROP_PIN_CONFIG_PULL_UP),
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(10, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(16, MICROP_PIN_CONFIG_GPO),
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x01 },
	},
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "lcd-backlight",
		.pin    = 6,
		.config = MICROP_PIN_CONFIG_PWM,
		.freq   = MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels = { 30, 48, 66, 84, 102, 133, 163, 194, 224, 255 },
		.dutys	= {  8, 16, 34, 61,  96, 138, 167, 195, 227, 255 },
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.levels = { 25 },
		.config = MICROP_PIN_CONFIG_GPO_PWM,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 24, 60, 425, 497, 569, 638 },
	},
	{
		.pin	 = 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask 	 = { 0x00, 0x01, 0x00 },
		.intr_debounce = hero_microp_intr_debounce,
		.intr_function = hero_microp_intr_function,
		.init_intr_function = 1,
	}
};

/* XC and enable LABC */
static struct microp_pin_config microp_pins_skuid_1[] = {
	MICROP_PIN(23, MICROP_PIN_CONFIG_PULL_UP),
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(10, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(16, MICROP_PIN_CONFIG_GPO_INV),
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x01 },
	},
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.levels = { 25 },
		.config = MICROP_PIN_CONFIG_GPO_PWM,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 24, 60, 425, 497, 569, 638 },
	},
	{
		.name	= "35mm_adc",
		.pin	= 25,
		.adc_pin = 7,
		.config = MICROP_PIN_CONFIG_ADC_READ,
	},
	{
		.name   = "microp_intrrupt",
		.pin	 = 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask 	 = { 0x00, 0x01, 0x00 },
		.intr_debounce = hero_microp_intr_debounce,
		.intr_function = hero_microp_intr_function,
		.init_intr_function = 1,
	}
};

// XD, add jogball backlight function
static struct microp_pin_config microp_pins_skuid_2[] = {
	MICROP_PIN(23, MICROP_PIN_CONFIG_PULL_UP),
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x03 },
	},
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.levels = { 25 },
		.config = MICROP_PIN_CONFIG_GPO_PWM,
	},
	{
		.name	= "jogball-backlight",
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_PWM,
		.auto_if_on = 1,
		.i_am_jogball_function = 1,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 24, 60, 425, 497, 569, 638 },
	},
	{
		.name	= "35mm_adc",
		.pin	= 25,
		.adc_pin = 7,
		.config = MICROP_PIN_CONFIG_ADC_READ,
	},
	{
		.name   = "microp_intrrupt",
		.pin	 = 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask 	 = { 0x00, 0x01, 0x00 },
		.intr_debounce = hero_microp_intr_debounce,
		.intr_function = hero_microp_intr_function,
		.init_intr_function = 1,
	}
};

// XE, 11pin mic function
static struct microp_pin_config microp_pins_skuid_3[] = {
	MICROP_PIN(23, MICROP_PIN_CONFIG_PULL_UP),
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x03 },
	},
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.levels = { 25 },
		.config = MICROP_PIN_CONFIG_GPO_PWM,
	},
	{
		.name	= "jogball-backlight",
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_PWM,
		.auto_if_on = 1,
		.i_am_jogball_function = 1,
	},
	{
		.name = "microp_11pin_mic",
		.pin = 16,
		.config = MICROP_PIN_CONFIG_MIC,
		.init_value = 1,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 24, 60, 425, 497, 569, 638 },
	},
	{
		.name	= "35mm_adc",
		.pin	= 25,
		.adc_pin = 7,
		.config = MICROP_PIN_CONFIG_ADC_READ,
	},
	{
		.name   = "microp_intrrupt",
		.pin	 = 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask 	 = { 0x00, 0x01, 0x00 },
		.intr_debounce = hero_microp_intr_debounce,
		.intr_function = hero_microp_intr_function,
		.init_intr_function = 1,
	}
};

static struct microp_i2c_platform_data microp_data = {
	.num_pins   = ARRAY_SIZE(microp_pins_skuid_0),
	.pin_config = microp_pins_skuid_0,
	.gpio_reset = HERO_GPIO_UP_RESET_N,
	.cabc_backlight_enable = 0,
	.microp_enable_early_suspend = 1,
	.microp_enable_reset_button = 1,
};

#define DEBOUNCE_LENGTH 4
static int hero_microp_intr_debounce(uint8_t *pin_status)
{
//Per HW RD's request, wait 300 mill-seconds.
#if 1
	mdelay(300);
	return 0;
#else
	static int count;
	static uint8_t data[DEBOUNCE_LENGTH];

	if (pin_status[0] == 0 && pin_status[1] == 0 && pin_status[2] == 0) {
		mdelay(5);
		return 1;
	}
	/*
	printk(KERN_INFO "hero_microp_intr_debounce : %02X %02X %02X\n",
		pin_status[0], pin_status[1], pin_status[2]);
	*/
	if (count < DEBOUNCE_LENGTH - 1) {
		data[count] = pin_status[1] & 0x01;
		count++;
	} else {
		data[DEBOUNCE_LENGTH - 1] = pin_status[1] & 0x01;
		for (count = 0; count < DEBOUNCE_LENGTH - 1; count++)
			if (data[count] != data[count + 1])
				break;
		if (count == DEBOUNCE_LENGTH - 1) {
			count = 0;
			return 0;
		}
		for (count = 0; count < DEBOUNCE_LENGTH - 1; count++)
			data[count] = data[count + 1];
	}

	mdelay(20);

	return 1;
#endif
}

void hero_headset_mic_select(uint8_t select)
{
	microp_i2c_set_pin_mode(4, select, microp_data.dev_id);
}

static void hero_microp_intr_function(uint8_t *pin_status)
{
	static int last_insert = 0;
	int insert;
	/*
	printk(KERN_INFO "hero_microp_intr_function : %02X %02X %02X\n",
		pin_status[0], pin_status[1], pin_status[2]);
	*/
	if (pin_status[1] & 0x01) {
		insert = 0;
	} else {
		insert = 1;
	}

	if (last_insert != insert) {
		printk(KERN_INFO "hero_microp_intr_function : %s\n", insert ? "inserted" : "not inserted");
		microp_i2c_set_pin_mode(4, insert, microp_data.dev_id);
#ifdef CONFIG_HTC_HEADSET
		cnf_driver_event("H2W_extend_headset", &insert);
#endif
		last_insert = insert;
	}
}

static struct akm8973_platform_data compass_platform_data = {
	.reset = HERO_GPIO_COMPASS_RST_N,
	.intr = HERO_GPIO_COMPASS_INT_N,
};

static struct bma150_platform_data gsensor_platform_data = {
	.intr = HERO_GPIO_GSENSOR_INT_N,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),
		.platform_data = &hero_ts_data,
		.irq = HERO_GPIO_TO_INT(HERO_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(CY8C_I2C_NAME, 0x13),
		.platform_data = &hero_cypress_ts_data,
		.irq = HERO_GPIO_TO_INT(HERO_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = HERO_GPIO_TO_INT(HERO_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = HERO_GPIO_TO_INT(HERO_GPIO_COMPASS_INT_N),
	},
};

static struct i2c_board_info i2c_bma150 = {
	I2C_BOARD_INFO(BMA150_I2C_NAME, 0x38),
	.platform_data = &gsensor_platform_data,
	.irq = HERO_GPIO_TO_INT(HERO_GPIO_GSENSOR_INT_N),
};

static struct i2c_board_info i2c_mt9p012 = {
	I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	.platform_data = &hero_mt9p012_device_data,
};
static struct i2c_board_info i2c_s5k3e2fx = {
	I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	.platform_data = &hero_s5k3e2fx_device_data,
};

static void hero_phy_reset(void)
{
	printk(KERN_INFO "%s\n", __func__);
	gpio_set_value(HERO_GPIO_USB_PHY_RST_N, 0);
	mdelay(10);
	gpio_set_value(HERO_GPIO_USB_PHY_RST_N, 1);
	mdelay(10);
}

static void hero_phy_shutdown(void)
{
	printk(KERN_INFO "%s\n", __func__);
}

static struct pwr_sink hero_pwrsink_table[] = {
	{
		.id	= PWRSINK_AUDIO,
		.ua_max	= 100000,
	},
	{
		.id	= PWRSINK_BACKLIGHT,
		.ua_max	= 125000,
	},
	{
		.id	= PWRSINK_LED_BUTTON,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_LED_KEYBOARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_GP_CLK,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_BLUETOOTH,
		.ua_max	= 15000,
	},
	{
		.id	= PWRSINK_CAMERA,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_SDCARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_VIDEO,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id	= PWRSINK_SYSTEM_LOAD,
		.ua_max	= 100000,
		.percent_util = 38,
	},
};

static int hero_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void hero_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void hero_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int hero_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data hero_pwrsink_data = {
	.num_sinks	= ARRAY_SIZE(hero_pwrsink_table),
	.sinks		= hero_pwrsink_table,
	.suspend_late	= hero_pwrsink_suspend_late,
	.resume_early	= hero_pwrsink_resume_early,
	.suspend_early	= hero_pwrsink_suspend_early,
	.resume_late	= hero_pwrsink_resume_late,
};

static struct platform_device hero_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev	= {
		.platform_data = &hero_pwrsink_data,
	},
};
/* Switch between UART3 and GPIO */
static uint32_t uart3_on_gpio_table[] = {
	/* RX */
	PCOM_GPIO_CFG(HERO_GPIO_UART3_RX, 1, GPIO_INPUT, GPIO_NO_PULL, 0),
	/* TX */
	PCOM_GPIO_CFG(HERO_GPIO_UART3_TX, 1, GPIO_OUTPUT, GPIO_NO_PULL, 0),
};

/* default TX,RX to GPI */
static uint32_t uart3_off_gpio_table[] = {
	/* RX, H2W DATA */
	PCOM_GPIO_CFG(HERO_GPIO_H2W_DATA, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	/* TX, H2W CLK */
	PCOM_GPIO_CFG(HERO_GPIO_H2W_CLK, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
};

static int hero_h2w_path = H2W_GPIO;

static void h2w_configure(int route)
{
	printk(KERN_INFO "H2W route = %d \n", route);
	switch (route) {
	case H2W_UART3:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+1, 0);
		hero_h2w_path = H2W_UART3;
		printk(KERN_INFO "H2W -> UART3\n");
		break;
	case H2W_GPIO:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpio_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpio_table+1, 0);
		hero_h2w_path = H2W_GPIO;
		printk(KERN_INFO "H2W -> GPIO\n");
		break;
	}
}

static void h2w_defconfig(void)
{
	h2w_configure(H2W_GPIO);
}

static void set_h2w_dat(int n)
{
	gpio_set_value(HERO_GPIO_H2W_DATA, n);
}

static void set_h2w_clk(int n)
{
	gpio_set_value(HERO_GPIO_H2W_CLK, n);
}

static void set_h2w_dat_dir(int n)
{
	if (n == 0) /* input */
		gpio_direction_input(HERO_GPIO_H2W_DATA);
	else
		gpio_configure(HERO_GPIO_H2W_DATA, GPIOF_DRIVE_OUTPUT);
}

static void set_h2w_clk_dir(int n)
{
	if (n == 0) /* input */
		gpio_direction_input(HERO_GPIO_H2W_CLK);
	else
		gpio_configure(HERO_GPIO_H2W_CLK, GPIOF_DRIVE_OUTPUT);
}

static int get_h2w_dat(void)
{
	return gpio_get_value(HERO_GPIO_H2W_DATA);
}

static int get_h2w_clk(void)
{
	return gpio_get_value(HERO_GPIO_H2W_CLK);
}

#ifdef CONFIG_HTC_HEADSET
static int set_h2w_path(const char *val, struct kernel_param *kp)
{
	int ret = -EINVAL;
	int enable;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	switch (hero_h2w_path) {
	case H2W_GPIO:
		enable = 1;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	case H2W_UART3:
		enable = 0;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	default:
		hero_h2w_path = -1;
		return -EINVAL;
	}

	h2w_configure(hero_h2w_path);
	return ret;
}

module_param_call(h2w_path, set_h2w_path, param_get_int,
		&hero_h2w_path, S_IWUSR | S_IRUGO);

#endif
static struct h2w_platform_data hero_h2w_data = {
	.h2w_power		= HERO_GPIO_EXT_3V_EN,
	.cable_in1		= HERO_GPIO_CABLE_IN1_XAXB,
	.cable_in2		= HERO_GPIO_CABLE_IN2,
	.h2w_clk		= HERO_GPIO_H2W_CLK,
	.h2w_data		= HERO_GPIO_H2W_DATA,
	.headset_mic_35mm	= HERO_GPIO_HEADSET_MIC,
/*	.ext_mic_sel		= HERO_GPIO_AUD_EXTMIC_SEL, */
	.debug_uart 		= H2W_UART3,
	.config 		= h2w_configure,
	.defconfig 		= h2w_defconfig,
	.set_dat		= set_h2w_dat,
	.set_clk		= set_h2w_clk,
	.set_dat_dir		= set_h2w_dat_dir,
	.set_clk_dir		= set_h2w_clk_dir,
	.get_dat		= get_h2w_dat,
	.get_clk		= get_h2w_clk,
	.headset_mic_sel	= hero_headset_mic_select,
};

static struct h2w_platform_data hero_h2w_data_xc = {
	.h2w_power		= HERO_GPIO_EXT_3V_EN,
	.cable_in1		= HERO_GPIO_CABLE_IN1,
	.cable_in2		= HERO_GPIO_CABLE_IN2,
	.h2w_clk		= HERO_GPIO_H2W_CLK,
	.h2w_data		= HERO_GPIO_H2W_DATA,
	.headset_mic_35mm	= HERO_GPIO_HEADSET_MIC,
	.ext_mic_sel		= HERO_GPIO_AUD_EXTMIC_SEL,
	.debug_uart 		= H2W_UART3,
	.config 		= h2w_configure,
	.defconfig 		= h2w_defconfig,
	.set_dat		= set_h2w_dat,
	.set_clk		= set_h2w_clk,
	.set_dat_dir		= set_h2w_dat_dir,
	.set_clk_dir		= set_h2w_clk_dir,
	.get_dat		= get_h2w_dat,
	.get_clk		= get_h2w_clk,
/*	.headset_mic_sel	= hero_headset_mic_select, */
};

static struct h2w_platform_data hero_h2w_data_xe = {
	.h2w_power		= HERO_GPIO_EXT_3V_EN,
	.cable_in1		= HERO_GPIO_CABLE_IN1,
	.cable_in2		= HERO_GPIO_CABLE_IN2,
	.h2w_clk		= HERO_GPIO_H2W_CLK,
	.h2w_data		= HERO_GPIO_H2W_DATA,
	.headset_mic_35mm	= HERO_GPIO_HEADSET_MIC,
	.ext_mic_sel		= HERO_GPIO_AUD_EXTMIC_SEL,
	.debug_uart 		= H2W_UART3,
	.config 		= h2w_configure,
	.defconfig 		= h2w_defconfig,
	.set_dat		= set_h2w_dat,
	.set_clk		= set_h2w_clk,
	.set_dat_dir		= set_h2w_dat_dir,
	.set_clk_dir		= set_h2w_clk_dir,
	.get_dat		= get_h2w_dat,
	.get_clk		= get_h2w_clk,
/*	.headset_mic_sel	= hero_headset_mic_select, */
	.flags			= _35MM_MIC_DET_L2H,
};

static struct platform_device hero_h2w = {
	.name		= "h2w",
	.id			= -1,
	.dev		= {
		.platform_data	= &hero_h2w_data,
	},
};

static struct platform_device hero_h2w_xc = {
	.name		= "h2w",
	.id			= -1,
	.dev		= {
		.platform_data	= &hero_h2w_data_xc,
	},
};

static struct platform_device hero_h2w_xe = {
	.name		= "h2w",
	.id			= -1,
	.dev		= {
		.platform_data	= &hero_h2w_data_xe,
	},
};

static struct platform_device hero_rfkill = {
	.name = "hero_rfkill",
	.id = -1,
};

static struct msm_pmem_setting pmem_setting = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = MSM_PMEM_GPU1_BASE,
	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};

static struct platform_device *devices2[] __initdata = {
	&hero_h2w_xe,
	&hero_rfkill,
	&hero_reset_keys_device,
#ifdef CONFIG_HTC_PWRSINK
	&hero_pwr_sink,
#endif
	&hero_nav_device,
};

static struct platform_device *devices1[] __initdata = {
	&hero_h2w_xc,
	&hero_rfkill,
	&hero_reset_keys_device,
#ifdef CONFIG_HTC_PWRSINK
	&hero_pwr_sink,
#endif
	&hero_nav_device,
};

static struct platform_device *devices0[] __initdata = {
	&hero_h2w,
	&hero_rfkill,
	&hero_reset_keys_device,
#ifdef CONFIG_HTC_PWRSINK
	&hero_pwr_sink,
#endif
};


extern struct sys_timer msm_timer;

static void __init hero_init_irq(void)
{
	printk("hero_init_irq()\n");
	msm_init_irq();
}

static uint opt_disable_uart3;

module_param_named(disable_uart3, opt_disable_uart3, uint, 0);

#if 0	//allenou, bt test, no need this one, 12/15
static int hero_bluetooth_power_on;
extern int hero_bt_fastclock_power(int on);

static void bluetooth_set_power(int on)
{
	if (on) {
		hero_bt_fastclock_power(1);
		udelay(10);
		gpio_configure(HERO_GPIO_WB_SHUT_DOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	} else {
		gpio_configure(HERO_GPIO_WB_SHUT_DOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
		hero_bt_fastclock_power(0);
	}
}

static int bluetooth_set_power_on(const char *val, struct kernel_param *kp)
{
	int ret;
	ret = param_set_bool(val, kp);
	if (!ret)
		bluetooth_set_power(hero_bluetooth_power_on);
	return ret;
}

module_param_call(bluetooth_power_on, bluetooth_set_power_on, param_get_bool,
		  &hero_bluetooth_power_on, S_IWUSR | S_IRUGO);
#endif

static char bt_chip_id[10] = "brfxxxx";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

static void hero_reset(void)
{
	gpio_set_value(HERO_GPIO_PS_HOLD, 0);
}

static uint32_t gpio_table[] = {
	PCOM_GPIO_CFG(HERO_GPIO_I2C_CLK, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_I2C_DAT , 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
	/* BLUETOOTH */
	#ifdef CONFIG_SERIAL_MSM_HS
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RTS, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* RTS */
	PCOM_GPIO_CFG(HERO_GPIO_UART1_CTS, 2, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA), /* CTS */
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RX, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* RX */
	PCOM_GPIO_CFG(HERO_GPIO_UART1_TX, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* TX */
	#else
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RTS, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* RTS */
	PCOM_GPIO_CFG(HERO_GPIO_UART1_CTS, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* CTS */
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RX, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* RX */
	PCOM_GPIO_CFG(HERO_GPIO_UART1_TX, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* TX */
	#endif
};


static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for (n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

void config_hero_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_hero_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static void __init config_gpios(void)
{
	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));
	config_hero_camera_off_gpios();
}

void msm_serial_debug_init(unsigned int base, int irq,
			   const char *clkname, int signal_irq);

static struct msm_acpu_clock_platform_data hero_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
#if defined(CONFIG_TURBO_MODE)
	.wait_for_irq_khz = 176000000,
#else
	.wait_for_irq_khz = 128000000,
#endif
};

static unsigned hero_perf_acpu_table[] = {
	245760000,
	480000000,
	528000000,
};

static struct perflock_platform_data hero_perflock_data = {
	.perf_acpu_table = hero_perf_acpu_table,
	.table_size = ARRAY_SIZE(hero_perf_acpu_table),
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(45),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
	.cpu_lock_supported = 1,
};
#endif

static void __init hero_init(void)
{
	int rc;
	printk(KERN_INFO "hero_init() revision: 0x%X\n", system_rev);
	config_gpios();
	printk(KERN_INFO "%s: skuid: 0x%X, hwid: 0x%X, enginner_id: 0x%X\n",
		__func__, hero_get_skuid(), hero_get_hwid(), hero_get_engineerid());

	msm_hw_reset_hook = hero_reset;

	gpio_direction_output(HERO_TP_LS_EN, 0);

	msm_acpu_clock_init(&hero_clock_data);
	perflock_init(&hero_perflock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
				      &msm_device_uart3.dev, 1);
#endif

	/* Init bluetooth clock and shutdown pin */
//	bluetooth_set_power(hero_bluetooth_power_on);	//allenou, bt test, no need this one, 12/15

	msm_add_devices();

	#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_add_serial_devices(MSM_SERIAL_UART1DM);
	#else
	msm_add_serial_devices(MSM_SERIAL_UART1);
	#endif

	msm_add_serial_devices(MSM_SERIAL_UART3);

	//msm_change_usb_id(0x0bb4, 0x0c04);
	msm_add_usb_devices(hero_phy_reset, hero_phy_shutdown);

	msm_add_mem_devices(&pmem_setting);

	msm_init_pmic_vibrator();

	rc = hero_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	if (system_rev == 0) {
		hero_reset_keys_pdata.keys_down[0] = KEY_HOME;
		hero_reset_keys_pdata.keys_down[2] = KEY_POWER;
	}

	if (system_rev == 0 || system_rev == 1) {
		platform_add_devices(devices0, ARRAY_SIZE(devices0));
		for (rc = 0; rc < ARRAY_SIZE(i2c_devices); rc++) {
			if (!strcmp(i2c_devices[rc].type, MICROP_I2C_NAME))
				i2c_devices[rc].irq = HERO_GPIO_TO_INT(HERO_GPIO_UP_INT_N_XAXB);
			if (!strcmp(i2c_devices[rc].type, AKM8973_I2C_NAME))
				i2c_devices[rc].irq = HERO_GPIO_TO_INT(HERO_GPIO_COMPASS_INT_N_XAXB);
		}
	} else if (system_rev == 2 || system_rev == 3) /*XC and XD*/
		platform_add_devices(devices1, ARRAY_SIZE(devices1));
	else /*above XE*/
		platform_add_devices(devices2, ARRAY_SIZE(devices2));

	i2c_register_board_info(0, &i2c_bma150, 1);

	if (hero_get_engineerid() || system_rev > 2) {
		if (system_rev >= 4) {
			microp_data.num_pins = ARRAY_SIZE(microp_pins_skuid_3);
			microp_data.pin_config = microp_pins_skuid_3;
		} else if(system_rev >= 3) {
			microp_data.num_pins = ARRAY_SIZE(microp_pins_skuid_2);
			microp_data.pin_config = microp_pins_skuid_2;
		} else {
			microp_data.num_pins = ARRAY_SIZE(microp_pins_skuid_1);
			microp_data.pin_config = microp_pins_skuid_1;
		}
		microp_data.cabc_backlight_enable = 1;
	}
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	i2c_register_board_info(0, &i2c_mt9p012, 1);

	/* Remove camera samsung sensor driver for MFG device reset issue */
	/*
	i2c_register_board_info(0, &i2c_s5k3e2fx, 1);
	*/
}

static void __init hero_fixup(struct machine_desc *desc, struct tag *tags,
			      char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = MSM_LINUX_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_SIZE);
	mi->bank[0].size = MSM_LINUX_SIZE;

	hwid = parse_tag_hwid((const struct tag *)tags);
	skuid = parse_tag_skuid((const struct tag *)tags);
	engineerid = parse_tag_engineerid((const struct tag *)tags);

}

static void __init hero_map_io(void)
{
	msm_map_common_io();
	msm_clock_init();
}

MACHINE_START(HERO, "hero")
/* Maintainer: Kant Kang <kant_kang@htc.com> */
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x19200100,
	.fixup          = hero_fixup,
	.map_io         = hero_map_io,
	.init_irq       = hero_init_irq,
	.init_machine   = hero_init,
	.timer          = &msm_timer,
MACHINE_END
