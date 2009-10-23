/* arch/arm/mach-msm/include/mach/board.h
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
 */

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/input.h>
#include <asm/setup.h>

/* platform device data structures */
struct msm_acpu_clock_platform_data {
	uint32_t acpu_switch_time_us;
	uint32_t max_speed_delta_khz;
	uint32_t vdd_switch_time_us;
	unsigned long power_collapse_khz;
	unsigned long wait_for_irq_khz;
};
enum msm_camera_flash_t {
  MSM_CAMERA_FLASH_NONE,
  MSM_CAMERA_FLASH_LED
};
struct msm_camera_sensor_info {
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	int mclk;
	const char *sensor_name;
	enum msm_camera_flash_t flash_type;
	int (*sensor_probe)(void *, void *);
};

struct msm_camera_io_ext {
	uint32_t mdcphy;
	uint32_t mdcsz;
	uint32_t appphy;
	uint32_t appsz;
};
//for mm-camera
struct msm_camera_platform_data{
	void (*camera_gpio_on) (void);
	void (*camera_gpio_off)(void);
	uint8_t snum;
	struct msm_camera_sensor_info *sinfo;
	struct msm_camera_io_ext ioext;
};
//for libqcamera
struct msm_camsensor_device_platform_data{
	int	(*sensor_i2c_read)(unsigned short u_addr, unsigned short *pu_data);
	int	(*sensor_i2c_write)(unsigned short waddr, unsigned short wdata);
	int 	(*sensor_probe_initial)(void *clinet);
	void (*sensor_deinit)(void);
	int 	(*sensor_write_exposuregain)(
		uint32_t mode, uint16_t line, uint16_t gain, 
		uint16_t linelengthpck, uint16_t framelengthlines);
	int 	(*sensor_set_pclk)(int rt, int div_adj);
	int	(*sensor_setting)(unsigned long arg);
	int    (*sensor_resume)(void *client);
	int    (*sensor_suspend)(void *client,pm_message_t mesg);
	int 	(*sensor_power_down)(void);
	int 	(*sensor_power_up)(void);
	int    (*msmclk_rate_set)(int rate);
	int    (*msmclk_disable)(int clk_type);
	int    (*msmclk_enable)(int clk_type);
	int 	(*msmclk_camif_clk_select)(int internal);
	int 	(*msmio_camif_pad_reg_reset)(void);
	int	(*msmio_camif_app_reset)(void);
	void	(*msmio_camif_Reset2)(void);
};

struct msm_camera_device_platform_data{
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	void (*config_gpio_on) (void);
	void (*config_gpio_off)(void);
	struct msm_camsensor_device_platform_data *sensor_info;
};

struct msm_pmem_setting{
	resource_size_t pmem_start;
	resource_size_t pmem_size;
	resource_size_t pmem_adsp_start;
	resource_size_t pmem_adsp_size;
	resource_size_t pmem_gpu0_start;
	resource_size_t pmem_gpu0_size;
	resource_size_t pmem_gpu1_start;
	resource_size_t pmem_gpu1_size;
	resource_size_t pmem_camera_start;
	resource_size_t pmem_camera_size;
	resource_size_t ram_console_start;
	resource_size_t ram_console_size;
#ifdef CONFIG_BUILD_CIQ
	resource_size_t pmem_ciq_start;
	resource_size_t pmem_ciq_size;
#endif
};

#define MSM_UART1       (0x1)
#define MSM_UART2       (0x2)
#define MSM_UART3       (0x4)
#define MSM_UART1_DM    (0x10)
#define MSM_UART2_DM    (0x20)
#define MSM_UART3_DM    (0x40)

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_add_devices(void);
void __init msm_add_usb_devices(void (*phy_reset) (void), void (*phy_shutdown) (void));
void __init msm_change_usb_id(__u16 vendor_id, __u16 product_id);
void __init msm_add_mem_devices(struct msm_pmem_setting *setting);
void __init msm_map_common_io(void);
void __init msm_init_irq(void);
void __init msm_clock_init(void);
void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *);
void __init msm_init_pmic_vibrator(void);

struct mmc_platform_data;
int __init msm_add_sdcc_devices(unsigned int controller, struct mmc_platform_data *plat);
int __init msm_add_serial_devices(unsigned uart);

#if defined(CONFIG_USB_FUNCTION_MSM_HSUSB) || defined(CONFIG_USB_MSM_72K)
void msm_hsusb_set_vbus_state(int online);
/* START: add USB connected notify function */
struct t_usb_status_notifier{
	struct list_head notifier_link;
	const char *name;
	void (*func)(int online);
};
	int usb_register_notifier(struct t_usb_status_notifier *);
	static LIST_HEAD(g_lh_usb_notifier_list);
/* END: add USB connected notify function */
#else
static inline void msm_hsusb_set_vbus_state(int online) {}
#endif

int board_mfg_mode(void);
int parse_tag_smi(const struct tag *tags);
int parse_tag_hwid(const struct tag * tags);
int parse_tag_skuid(const struct tag * tags);
int parse_tag_engineerid(const struct tag * tags);
void board_get_keycaps_tag(char **);
void board_get_cid_tag(char **);
void board_get_carrier_tag(char **);
void board_get_mid_tag(char **);
char *board_serialno(void);

/* 
 * Obviously, we need these in all project.
 * To export a function to get these is too lousy.
 * Each BSP can include board.h to get these.
 *
 * Jay, 15/May/09'
 * */
extern int panel_type;
extern unsigned engineer_id;

#endif
