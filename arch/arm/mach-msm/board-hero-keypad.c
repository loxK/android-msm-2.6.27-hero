/* linux/arch/arm/mach-msm7201a/board-hero-keypad.c
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


#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <asm/mach-types.h>

#include "board-hero.h"


static unsigned int hero_col_gpios[] = { 35, 34, 33 };
static unsigned int hero_row_gpios[] = { 42, 41, 40 };

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(hero_row_gpios) + (row))

static const unsigned short hero_keymap0[ARRAY_SIZE(hero_col_gpios) * ARRAY_SIZE(hero_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_HOME,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(0, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(1, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

	[KEYMAP_INDEX(2, 0)] = KEY_BACK,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(2, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

};

static const unsigned short hero_keymap1[ARRAY_SIZE(hero_col_gpios) * ARRAY_SIZE(hero_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_BACK,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND,
	//[KEYMAP_INDEX(0, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(1, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

	[KEYMAP_INDEX(2, 0)] = KEY_HOME,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(2, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove
};

static const unsigned short hero_keymap2[ARRAY_SIZE(hero_col_gpios) * ARRAY_SIZE(hero_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_SEARCH,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND,

	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_BACK,

	[KEYMAP_INDEX(2, 0)] = KEY_HOME,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = KEY_F21,
};

static const unsigned short hero_keymap2_engin3[ARRAY_SIZE(hero_col_gpios) * ARRAY_SIZE(hero_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_BACK,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND,

	[KEYMAP_INDEX(1, 0)] = KEY_HOME,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_MENU,

	[KEYMAP_INDEX(2, 0)] = KEY_SEARCH,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = KEY_F21,
};

static struct gpio_event_matrix_info hero_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = hero_keymap2,
	.output_gpios = hero_col_gpios,
	.input_gpios = hero_row_gpios,
	.noutputs = ARRAY_SIZE(hero_col_gpios),
	.ninputs = ARRAY_SIZE(hero_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,	
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS |GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};

static struct gpio_event_direct_entry hero_keypad_nav_map[] = {
	{ HERO_POWER_KEY,              KEY_END      },
};

static struct gpio_event_input_info hero_keypad_nav_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = hero_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(hero_keypad_nav_map)
};

static struct gpio_event_info *hero_keypad_info[] = {
	&hero_keypad_matrix_info.info,
	&hero_keypad_nav_info.info,
};

static struct gpio_event_platform_data hero_keypad_data = {
	.name = "hero-keypad",
	.info = hero_keypad_info,
	.info_count = ARRAY_SIZE(hero_keypad_info)
};

static struct platform_device hero_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &hero_keypad_data,
	},
};

static int __init hero_init_keypad(void)
{
	if (!machine_is_hero())
		return 0;

	if(system_rev == 0)	{ /* XA */
		hero_keypad_matrix_info.keymap = hero_keymap0;
	}	else
	if(system_rev == 1)	{ /* XB */
		hero_keypad_matrix_info.keymap = hero_keymap1;
	}

	if(hero_get_engineerid() == 3)
		hero_keypad_matrix_info.keymap = hero_keymap2_engin3;

	return platform_device_register(&hero_keypad_device);
}

device_initcall(hero_init_keypad);
