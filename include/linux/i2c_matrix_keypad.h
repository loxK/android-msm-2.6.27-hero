/*
 * include/linux/i2c_matrix_keypad.h
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
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

#ifndef _LINUX_HTC_I2C_KEYPAD_H
#define _LINUX_HTC_I2C_KEYPAD_H

#define MICROP_I2C_KEYPAD_DRIVER "microp-i2c-keypad"

#define MICROP_I2C_CMD_KEYSCAN_RESULT_REG_RO 0x10
#define MICROP_I2C_CMD_INPUT_PIN_STATUS_REG_RO 0x11
#define MICROP_I2C_CMD_INFORMATION_REG_RO 0x12
#define MICROP_I2C_CMD_MISC_REG_WR 0x13

#define MICROP_I2C_CMD_VERSION	0x29

enum microp_i2c_keypad_serivce {
	keypad_service = 0x0,
	keycaps_led_service,
	func_led_service
};

struct i2c_matrix_keypad_mapping	{
	const unsigned char i2c_value;
	const unsigned short key_code;
};

struct i2c_matrix_keypad_platform_data {
	uint32_t version;
	const char *keypad_name;
	int (*power)(int on);	/* Only valid in first array entry */
	const unsigned short *keymap;
	size_t keymap_size;
	unsigned char row_max;
	unsigned char col_max;
	unsigned char break_code_mask;
	uint32_t flags;
};

#endif /* _LINUX_HTC_I2C_KEYPAD_H */
