/* linux/arch/arm/mach-msm/board-heroc-camsensor.h
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
 
#ifndef _HERO_CAMSENSOR_
#define _HERO_CAMSENSOR_

int hero_s5k3e2fx_i2c_write(unsigned short waddr, unsigned short wdata);
int hero_s5k3e2fx_i2c_read(unsigned short u_addr, unsigned short *pu_data);
int hero_s5k3e2fx_probe_init(void *client);
void hero_s5k3e2fx_sensor_deinit(void);
int hero_s5k3e2fx_write_exposuregain(
	uint32_t mode, uint16_t line, uint16_t gain, 
	uint16_t linelengthpck, uint16_t framelengthlines);
int hero_s5k3e2fx_set_pclk(int rt, int div_adj);
int hero_s5k3e2fx_sensor_setting(unsigned long arg);
int hero_s5k3e2fx_resume(void *client);
int hero_s5k3e2fx_suspend(void *client,pm_message_t mesg);
int hero_s5k3e2fx_power_down(void);
int hero_s5k3e2fx_power_up(void);
int hero_msm_camio_clk_rate_set(int rate);
int hero_msm_camio_clk_disable(int clk_type);
int hero_msm_camio_clk_enable (int clk_type);
int hero_s5k3e2fx_camif_pad_reg_reset(void);
int hero_s5k3e2fx_camif_app_reset(void);
void hero_s5k3e2fx_camif_reset2(void);
int hero_camif_clk_select(int internal);

#endif
