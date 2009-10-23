/* arch/arm/mach-msm/board-trout.c
 *
 * Copyright (C) 2008 Google, Inc.
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
#include <linux/timed_gpio.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/elan_i2c.h>
#include <linux/akm8976.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>
#include <linux/delay.h>

#include <asm/gpio.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <mach/system.h>
#include <mach/vreg.h>
#include <mach/h2w.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>

#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/mmc.h>
#include <linux/mmc/sdio_ids.h>

#include "board-trout.h"

#include <mach/gpio_chip.h>

#include <mach/board.h>
#include <mach/htc_pwrsink.h>
#include <mach/msm_serial_hs.h>

#include <mach/proc_comm.h>
#include <mach/devices.h>
//#define ENABLE_SD_DOOR_FUNCTION

void msm_init_irq(void);
void msm_init_gpio(void);
static void config_camera_on_gpios(void);
static void config_camera_off_gpios(void);

extern int trout_init_mmc(unsigned int);

struct trout_axis_info {
	struct gpio_event_axis_info info;
	uint16_t in_state;
	uint16_t out_state;
};
static bool nav_just_on;
static int nav_on_jiffies;
static int smi_sz = 64;

uint16_t trout_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct trout_axis_info *ai = container_of(info, struct trout_axis_info, info);
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
	return out;
}

int trout_nav_power(const struct gpio_event_platform_data *pdata, bool on)
{
	gpio_set_value(TROUT_GPIO_JOG_EN, on);
	if (on) {
		nav_just_on = 1;
		nav_on_jiffies = jiffies;
	}
	return 0;
}

static uint32_t trout_4_x_axis_gpios[] = {
	TROUT_4_BALL_LEFT_0, TROUT_4_BALL_RIGHT_0
};
static uint32_t trout_5_x_axis_gpios[] = {
	TROUT_5_BALL_LEFT_0, TROUT_5_BALL_RIGHT_0
};

static struct trout_axis_info trout_x_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(trout_5_x_axis_gpios),
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(trout_5_x_axis_gpios),
		.map = trout_axis_map,
		.gpio = trout_5_x_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION /*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */
	}
};

static uint32_t trout_4_y_axis_gpios[] = {
	TROUT_4_BALL_UP_0, TROUT_4_BALL_DOWN_0
};
static uint32_t trout_5_y_axis_gpios[] = {
	TROUT_5_BALL_UP_0, TROUT_5_BALL_DOWN_0
};

static struct trout_axis_info trout_y_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(trout_5_y_axis_gpios),
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(trout_5_y_axis_gpios),
		.map = trout_axis_map,
		.gpio = trout_5_y_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION /*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */
	}
};

static struct gpio_event_direct_entry trout_nav_buttons[] = {
	{ TROUT_GPIO_NAVI_ACT_N, BTN_MOUSE }
};

static struct gpio_event_input_info trout_nav_button_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = trout_nav_buttons,
	.keymap_size = ARRAY_SIZE(trout_nav_buttons)
};

static struct gpio_event_info *trout_nav_info[] = {
	&trout_x_axis.info.info,
	&trout_y_axis.info.info,
	&trout_nav_button_info.info
};

static struct gpio_event_platform_data trout_nav_data = {
	.name = "trout-nav",
	.info = trout_nav_info,
	.info_count = ARRAY_SIZE(trout_nav_info),
	.power = trout_nav_power,
};

static struct platform_device trout_nav_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 2,
	.dev		= {
		.platform_data	= &trout_nav_data,
	},
};

static int trout_reset_keys_up[] = {
	BTN_MOUSE,
	0
};

static struct keyreset_platform_data trout_reset_keys_pdata = {
	.keys_up = trout_reset_keys_up,
	.keys_down = {
		KEY_SEND,
		KEY_MENU,
		KEY_END,
		0
	},
};

struct platform_device trout_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &trout_reset_keys_pdata,
};

static int trout_ts_power(int on)
{
	int tp_ls_gpio = system_rev < 5 ? TROUT_4_TP_LS_EN : TROUT_5_TP_LS_EN;
	if (on) {
		gpio_set_value(TROUT_GPIO_TP_I2C_PULL, 1);
		gpio_set_value(TROUT_GPIO_TP_EN, 1);
		/* touchscreen must be powered before we enable i2c pullup */
		msleep(2);
		/* enable touch panel level shift */
		gpio_set_value(tp_ls_gpio, 1);
		msleep(2);
	}
	else {
		gpio_set_value(tp_ls_gpio, 0);
		gpio_set_value(TROUT_GPIO_TP_EN, 0);
		gpio_set_value(TROUT_GPIO_TP_I2C_PULL, 0);
	}
	return 0;
}

static struct synaptics_i2c_rmi_platform_data trout_ts_data[] = {
	{
		.version = 0x010c,
		.power = trout_ts_power,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -100 * 0x10000 / 4334,
		.inactive_right = -100 * 0x10000 / 4334,
		.inactive_top = -40 * 0x10000 / 6696,
		.inactive_bottom = -40 * 0x10000 / 6696,
		.snap_left_on = 300 * 0x10000 / 4334,
		.snap_left_off = 310 * 0x10000 / 4334,
		.snap_right_on = 300 * 0x10000 / 4334,
		.snap_right_off = 310 * 0x10000 / 4334,
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

static struct akm8976_platform_data compass_platform_data = {
	.reset = TROUT_GPIO_COMPASS_RST_N,
	.clk_on = TROUT_GPIO_COMPASS_32K_EN,
	.intr = TROUT_GPIO_COMPASS_IRQ,
};

static struct msm_camera_device_platform_data trout_camera_data = {
	.sensor_reset	= 108,
	.sensor_pwd	= 85,
	.vcm_pwd	= TROUT_GPIO_VCM_PWDN,
	.config_gpio_on = config_camera_on_gpios,
	.config_gpio_off = config_camera_off_gpios,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),
		.platform_data = &trout_ts_data,
		.irq = TROUT_GPIO_TO_INT(TROUT_GPIO_TP_ATT_N)
	},
/*	didn't support elan in Trout
	{
		I2C_BOARD_INFO("elan-touch", 0x10),
		.irq = TROUT_GPIO_TO_INT(TROUT_GPIO_TP_ATT_N),
	},
*/
	{
		I2C_BOARD_INFO("akm8976", 0x1C),
		.platform_data = &compass_platform_data,
		.irq = TROUT_GPIO_TO_INT(TROUT_GPIO_COMPASS_IRQ),
	},
	{
		I2C_BOARD_INFO("pca963x", 0x62),
	},
	{
		I2C_BOARD_INFO("mt9t013", 0x6C >> 1),
		.platform_data = &trout_camera_data,
	},
};

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = TROUT_GPIO_HAPTIC_PWM,
		.max_timeout = 15000,
	},
	{
		.name = "flash",
		.gpio = TROUT_GPIO_FLASH_EN,
		.max_timeout = 400,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device android_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &timed_gpio_data,
	},
};

static struct gpio_led android_led_list[] = {
	{
		.name = "spotlight",
		.gpio = TROUT_GPIO_SPOTLIGHT_EN,
	},
	{
		.name = "keyboard-backlight",
		.gpio = TROUT_GPIO_QTKEY_LED_EN,
	},
	{
		.name = "button-backlight",
		.gpio = TROUT_GPIO_UI_LED_EN,
	},
};

static struct gpio_led_platform_data android_leds_data = {
	.num_leds	= ARRAY_SIZE(android_led_list),
	.leds		= android_led_list,
};

static struct platform_device android_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &android_leds_data,
	},
};

#ifdef ENABLE_SD_DOOR_FUNCTION
static struct gpio_switch_platform_data sd_door_switch_data = {
	.name		= "sd-door",
	.gpio		= TROUT_GPIO_SD_DOOR_N,
	.state_on	= "open",
	.state_off	= "closed",
};

static struct platform_device sd_door_switch = {
	.name		= "switch-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &sd_door_switch_data,
	},
};
#endif


static void trout_phy_reset(void)
{
	gpio_set_value(TROUT_GPIO_USB_PHY_RST_N, 0);
	mdelay(10);
	gpio_set_value(TROUT_GPIO_USB_PHY_RST_N, 1);
	mdelay(10);
}

static struct pwr_sink trout_pwrsink_table[] = {
	{
		.id	= PWRSINK_AUDIO,
		.ua_max	= 90000,
	},
	{
		.id	= PWRSINK_BACKLIGHT,
		.ua_max	= 128000,
	},
	{
		.id	= PWRSINK_LED_BUTTON,
		.ua_max	= 17000,
	},
	{
		.id	= PWRSINK_LED_KEYBOARD,
		.ua_max	= 22000,
	},
	{
		.id	= PWRSINK_GP_CLK,
		.ua_max	= 30000,
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

static struct pwr_sink_platform_data trout_pwrsink_data = {
	.num_sinks	= ARRAY_SIZE(trout_pwrsink_table),
	.sinks		= trout_pwrsink_table,
	.suspend_late	= NULL,
	.resume_early	= NULL,
	.suspend_early	= NULL,
	.resume_late	= NULL,
};

static struct platform_device trout_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev	= {
		.platform_data = &trout_pwrsink_data,
	},
};

static struct platform_device trout_rfkill = {
	.name = "trout_rfkill",
	.id = -1,
};

/* RTS/CTS to GPO/GPI. */
static uint32_t uart1_on_gpio_table[] = {
	#ifdef CONFIG_SERIAL_MSM_HS	/*allenou, uart hs test, 2008/11/18*/
	PCOM_GPIO_CFG(TROUT_GPIO_UART1_RTS, 2, GPIO_OUTPUT,
			GPIO_PULL_UP, GPIO_8MA), /* RTS */
	PCOM_GPIO_CFG(TROUT_GPIO_UART1_CTS, 2, GPIO_INPUT,
			GPIO_PULL_UP, GPIO_8MA), /* CTS */
	#else
	PCOM_GPIO_CFG(TROUT_GPIO_UART1_RTS, 1, GPIO_OUTPUT,
			GPIO_PULL_UP, GPIO_4MA), /* RTS */
	PCOM_GPIO_CFG(TROUT_GPIO_UART1_CTS, 1, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_4MA), /* CTS */
	#endif
};

/* RTS,CTS to BT. */
static uint32_t uart1_off_gpio_table[] = {
	PCOM_GPIO_CFG(TROUT_GPIO_UART1_RTS, 0, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA), /* RTS */
	PCOM_GPIO_CFG(TROUT_GPIO_UART1_CTS, 0, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA), /* CTS */
};
static int trout_h2w_path = H2W_UART3;
static void configure_cpld(int route)
{
	switch (route) {
	case H2W_UART1:
		/* Make sure uart1 funtion pin opened. */
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
				uart1_on_gpio_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
				uart1_on_gpio_table+1, 0);
		gpio_set_value(TROUT_GPIO_H2W_SEL0, 1);
		gpio_set_value(TROUT_GPIO_H2W_SEL1, 0);
		trout_h2w_path = H2W_UART1;
		printk(KERN_INFO " route = H2W-UART1, BT-X, UART3-X\n");
		break;
	case H2W_BT:
		gpio_set_value(TROUT_GPIO_H2W_SEL0, 1);
		gpio_set_value(TROUT_GPIO_H2W_SEL1, 1);
		/* UART1 RTS/CTS to GPO/GPI. */
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
				uart1_off_gpio_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
				uart1_off_gpio_table+1, 0);
		trout_h2w_path = H2W_BT;
		printk(KERN_INFO " route = H2W-BT, UART1-X, UART3-X\n");
		break;
	case H2W_UART3:
		gpio_set_value(TROUT_GPIO_H2W_SEL0, 0);
		gpio_set_value(TROUT_GPIO_H2W_SEL1, 1);
		/* Make sure uart1 funtion pin opened. */
		trout_h2w_path = H2W_UART3;
		printk(KERN_INFO " route = H2W-UART3, BT-UART1\n");
		break;
	case H2W_GPIO:
		gpio_set_value(TROUT_GPIO_H2W_SEL0, 0);
		gpio_set_value(TROUT_GPIO_H2W_SEL1, 0);
		/* Make sure uart1 funtion pin opened. */
		trout_h2w_path = H2W_GPIO;
		printk(KERN_INFO " route = H2W-GPIO, BT-UART1\n");
		break;
	}
}

#ifdef CONFIG_HTC_HEADSET
static int set_h2w_path(const char *val, struct kernel_param *kp)
{
	int ret = -EINVAL;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	switch (trout_h2w_path) {
	case H2W_GPIO:
	case H2W_UART1:
	case H2W_UART3:
	case H2W_BT:
		break;
	default:
		trout_h2w_path = -1;
		return -EINVAL;
	}

	configure_cpld(trout_h2w_path);
	return ret;
}

module_param_call(h2w_path, set_h2w_path, param_get_int,
		&trout_h2w_path, S_IWUSR | S_IRUGO);
#endif

static void h2w_defconfig(void)
{
	configure_cpld(H2W_GPIO);
}

static void set_h2w_dat(int n)
{
	gpio_set_value(TROUT_GPIO_H2W_DAT_GPO, n);
}

static void set_h2w_clk(int n)
{
	gpio_set_value(TROUT_GPIO_H2W_CLK_GPO, n);
}

static void set_h2w_dat_dir(int n)
{
	gpio_set_value(TROUT_GPIO_H2W_DAT_DIR, n);
}

static void set_h2w_clk_dir(int n)
{
	gpio_set_value(TROUT_GPIO_H2W_CLK_DIR, n);
}

static int get_h2w_dat(void)
{
	return gpio_get_value(TROUT_GPIO_H2W_DAT_GPI);
}

static int get_h2w_clk(void)
{
	return gpio_get_value(TROUT_GPIO_H2W_CLK_GPI);
}


static struct h2w_platform_data trout_h2w_data = {
	.h2w_power		= TROUT_GPIO_H2W_POWER,
	.cable_in1		= TROUT_GPIO_CABLE_IN1,
	.cable_in2		= TROUT_GPIO_CABLE_IN2,
	.h2w_clk		= TROUT_GPIO_H2W_CLK_GPI,
	.h2w_data		= TROUT_GPIO_H2W_DAT_GPI,
	.headset_mic_35mm	= TROUT_GPIO_AUD_HSMIC_DET_N,
	.debug_uart 		= H2W_UART3,
	.config 		= configure_cpld,
	.defconfig 		= h2w_defconfig,
	.set_dat		= set_h2w_dat,
	.set_clk		= set_h2w_clk,
	.set_dat_dir		= set_h2w_dat_dir,
	.set_clk_dir		= set_h2w_clk_dir,
	.get_dat		= get_h2w_dat,
	.get_clk		= get_h2w_clk,
};

static struct platform_device trout_h2w = {
	.name		= "h2w",
	.id			= -1,
	.dev		= {
		.platform_data	= &trout_h2w_data,
	},
};

static struct msm_pmem_setting pmem_setting_32 = {
	.pmem_start = SMI32_MSM_PMEM_MDP_BASE,
	.pmem_size = SMI32_MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = SMI32_MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = SMI32_MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = MSM_PMEM_GPU1_BASE,
	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = 0,
	.pmem_camera_size = 0,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};

static struct msm_pmem_setting pmem_setting_64 = {
	.pmem_start = SMI64_MSM_PMEM_MDP_BASE,
	.pmem_size = SMI64_MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = SMI64_MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = SMI64_MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = MSM_PMEM_GPU1_BASE,
	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = SMI64_MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = SMI64_MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};

static struct platform_device *devices[] __initdata = {
	&trout_nav_device,
	&trout_reset_keys_device,
	&android_leds,
#ifdef ENABLE_SD_DOOR_FUNCTION
	&sd_door_switch,
#endif
	&android_timed_gpios,
	&trout_rfkill,
	&trout_h2w,
#ifdef CONFIG_HTC_PWRSINK
	&trout_pwr_sink,
#endif
};

extern struct sys_timer msm_timer;

static void __init trout_init_irq(void)
{
	printk("trout_init_irq()\n");
	msm_init_irq();
}

static uint opt_disable_uart3;

module_param_named(disable_uart3, opt_disable_uart3, uint, 0);

static void trout_reset(void)
{
	gpio_set_value(TROUT_GPIO_PS_HOLD, 0);
}

static uint32_t gpio_table[] = {
	/* BLUETOOTH */
#ifdef CONFIG_SERIAL_MSM_HS
	PCOM_GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* RTS */
	PCOM_GPIO_CFG(44, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* CTS */
	PCOM_GPIO_CFG(45, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* RX */
	PCOM_GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* TX */
#else
	PCOM_GPIO_CFG(43, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* RTS */
	PCOM_GPIO_CFG(44, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* CTS */
	PCOM_GPIO_CFG(45, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* RX */
	PCOM_GPIO_CFG(46, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* TX */
#endif
};


static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
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
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static void __init config_gpios(void)
{
	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));
	config_camera_off_gpios();
}

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

static struct msm_acpu_clock_platform_data trout_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
	.wait_for_irq_khz = 128000000,
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(45),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};
#endif

static void __init trout_init(void)
{
	int rc;

	printk("trout_init() revision=%d\n", system_rev);

	/*
	 * Setup common MSM GPIOS
	 */
	config_gpios();

	msm_hw_reset_hook = trout_reset;

	gpio_direction_output(system_rev < 5 ?
			      TROUT_4_TP_LS_EN : TROUT_5_TP_LS_EN, 0);

	msm_acpu_clock_init(&trout_clock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
				      &msm_device_uart3.dev, 1);
#endif

	/* gpio_configure(108, IRQF_TRIGGER_LOW); */

	/* put the AF VCM in powerdown mode to avoid noise */
	gpio_set_value(TROUT_GPIO_VCM_PWDN, 1);
	mdelay(100);

	if (system_rev < 5) {
		trout_x_axis.info.gpio = trout_4_x_axis_gpios;
		trout_y_axis.info.gpio = trout_4_y_axis_gpios;
	}

	msm_add_devices();

	msm_add_serial_devices(MSM_SERIAL_UART1);

	msm_add_serial_devices(MSM_SERIAL_UART3);

	msm_add_usb_devices(trout_phy_reset, NULL);

	if (32 == smi_sz)
		msm_add_mem_devices(&pmem_setting_32);
	else
		msm_add_mem_devices(&pmem_setting_64);

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_add_serial_devices(MSM_SERIAL_UART1DM);
#endif

	rc = trout_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	#ifdef ENABLE_SD_DOOR_FUNCTION
	/* SD card door should wake the device */
	set_irq_wake(TROUT_GPIO_TO_INT(TROUT_GPIO_SD_DOOR_N), 1);
	#endif
}

static struct map_desc trout_io_desc[] __initdata = {
	{
		.virtual = TROUT_CPLD_BASE,
		.pfn     = __phys_to_pfn(TROUT_CPLD_START),
		.length  = TROUT_CPLD_SIZE,
		.type    = MT_DEVICE_NONSHARED
	}
};

int trout_get_smi_size(void)
{
	printk("get_smi_size=%d\r\n", smi_sz);

	return smi_sz;
}

static void __init trout_fixup(struct machine_desc *desc, struct tag *tags,
                               char **cmdline, struct meminfo *mi)
{
	smi_sz = parse_tag_smi((const struct tag*)tags);
	printk("trout_fixup:smisize=%d\n", smi_sz);

	mi->nr_banks=1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	if (32 == smi_sz) {
		mi->bank[0].size = (84*1024*1024);
	} else if (64 == smi_sz){
		mi->bank[0].size = (101*1024*1024);
	} else {
		printk(KERN_ERR "can not get smi size\n");
		//BUG();

		//Give a default value when not get smi size
		smi_sz = 64;
		mi->bank[0].size = (101*1024*1024);
		printk(KERN_ERR "use default  :  smisize=%d\n", smi_sz);
	}
}

static void __init trout_map_io(void)
{
	msm_map_common_io();
	iotable_init(trout_io_desc, ARRAY_SIZE(trout_io_desc));
	msm_clock_init();
}

MACHINE_START(TROUT, "trout")
/* Maintainer: Brian Swetland <swetland@google.com> */
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x10000100,
	.fixup          = trout_fixup,
	.map_io         = trout_map_io,
	.init_irq       = trout_init_irq,
	.init_machine   = trout_init,
	.timer          = &msm_timer,
MACHINE_END
