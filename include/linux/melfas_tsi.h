/*
 * include/linux/melfas_tsi.h - platform data structure for f75375s sensor
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

#ifndef _LINUX_melfas_TSI_H
#define _LINUX_melfas_TSI_H

#define MELFAS_I2C_RMI_NAME "melfas-tsi-ts"
#define melfas_I2C_ADDR	0x22
#define MELFAS_I2C_CMD_STATUS 0X00
#define MELFAS_I2C_CMD_OP_MODE 0x01
#define MELFAS_I2C_CMD_SENS_CONTROL 0x02
#define MELFAS_I2C_CMD_PARTIAL_SCAN_MODE 0x03
#define MELFAS_I2C_CMD_X_SIZE_UPPER 0x08
#define MELFAS_I2C_CMD_X_SIZE_LOWER 0x09
#define MELFAS_I2C_CMD_Y_SIZE_UPPER 0x0A
#define MELFAS_I2C_CMD_Y_SIZE_LOWER 0x0B
#define MELFAS_I2C_CMD_INPUT_INFORMATION 0x10
#define MELFAS_I2C_CMD_X_POSITION_UPPER_1 0x11
#define MELFAS_I2C_CMD_X_POSITION_LOWER_1 0x12
#define MELFAS_I2C_CMD_Y_POSITION_UPPER_1 0x13
#define MELFAS_I2C_CMD_Y_POSITION_LOWER_1 0x14
#define MELFAS_I2C_CMD_Z_POSITION 0x15
#define MELFAS_I2C_CMD_Z_WIDTH 0x16
#define MELFAS_I2C_CMD_GSERTURE 0x17
#define MELFAS_I2C_CMD_PIVOT_DETECTION 0x18
#define MELFAS_I2C_CMD_FIRMWARE_VER 0x20
#define MELFAS_I2C_CMD_X_POSITION_UPPER_2 0xD1
#define MELFAS_I2C_CMD_X_POSITION_LOWER_2 0xD2
#define MELFAS_I2C_CMD_Y_POSITION_UPPER_2 0xD3
#define MELFAS_I2C_CMD_Y_POSITION_LOWER_2 0xD4

enum {
	melfas_FLIP_X = 1UL << 0,
	melfas_FLIP_Y = 1UL << 1,
	melfas_SWAP_XY = 1UL << 2,
	melfas_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
};

struct melfas_i2c_rmi_platform_data {
	uint32_t version;	/* Use this entry for panels with */
				/* (major << 8 | minor) version or above. */
				/* If non-zero another array entry follows */
	int (*power)(int on);	/* Only valid in first array entry */
	uint32_t flags;
};

#endif /* _LINUX_melfas_I2C_RMI_H */
