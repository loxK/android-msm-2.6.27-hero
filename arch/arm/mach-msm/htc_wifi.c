/* arch/arm/plat-msm7k/htc_wifi.c
 *
 * Copyright (C) 2008 HTC, Inc.
 * Author: Max Tsai <max_tsai@htc.com>
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Dmitry Shmidt <dimitrysh@google.com>
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

#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/err.h>
#endif

#include <linux/module.h>
#include <asm/mach-types.h>
#include <linux/wifi_tiwlan.h>

#define BOARD_WIFI_FUN(b) \
	extern int b##_wifi_set_carddetect(int);\
	extern int b##_wifi_power(int);\
	extern int b##_wifi_reset(int);

#define BOARD_WIFI(b, a) \
	{\
		.name =			a,\
		.set_carddetect =	b##_wifi_set_carddetect,\
		.set_power =		b##_wifi_power,\
		.set_reset =		b##_wifi_reset,\
		.mem_prealloc           = NULL,\
	},

#ifdef CONFIG_WIFI_MEM_PREALLOC
typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[WMPA_NUMBER_OF_SECTIONS] = {
	{ NULL, (WMPA_SECTION_SIZE_0 + WMPA_SECTION_HEADER) },
	{ NULL, (WMPA_SECTION_SIZE_1 + WMPA_SECTION_HEADER) },
	{ NULL, (WMPA_SECTION_SIZE_2 + WMPA_SECTION_HEADER) }
};

static void *wifi_mem_prealloc(int section, unsigned long size)
{
	if( (section < 0) || (section >= WMPA_NUMBER_OF_SECTIONS) )
		return NULL;
	if( wifi_mem_array[section].size < size )
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init init_wifi_mem( void )
{
	int i, rc = 0;

	for(i=0;( i < WMPA_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = vmalloc(wifi_mem_array[i].size);
		if( wifi_mem_array[i].mem_ptr == NULL )
			rc = -ENOMEM;
	}

	if (rc)
		printk(KERN_CRIT "WiFi Memory init failure(%d)\n", rc);
	return rc;
}
device_initcall(init_wifi_mem);
#endif

static struct wifi_platform_data *board_wifi;

#ifndef CONFIG_WIFI_CONTROL_FUNC
int set_set_carddetect(int val)
{
	if (board_wifi)
		board_wifi->set_carddetect(val);
	else
		printk(KERN_ERR "WiFi: unable to detect card\n");
	return 0;
}
EXPORT_SYMBOL(set_set_carddetect);

int set_power(int on)
{
	if (board_wifi)
		return board_wifi->set_power(on);
	else {
		printk(KERN_ERR "WiFi: power %s fails\n", on? "on": "off");
		return -ENODEV;
	}
}
EXPORT_SYMBOL(set_power);

int set_reset(int on)
{
	if (board_wifi)
		board_wifi->set_reset(on);
	else
		printk(KERN_ERR "WiFi: reset fails\n");
	return 0;
}
EXPORT_SYMBOL(set_reset);
#endif


/*
 * add board name here
 */
#ifdef CONFIG_ARCH_MSM7200A
#ifdef CONFIG_MACH_HERO
BOARD_WIFI_FUN(hero)
#endif
#ifdef CONFIG_MACH_SAPPHIRE
BOARD_WIFI_FUN(sapphire)
#endif
#ifdef CONFIG_MACH_HERO_ESPRESSO
BOARD_WIFI_FUN(espresso)
#endif
#endif

#ifdef CONFIG_ARCH_MSM7201A
#ifdef CONFIG_MACH_SAPPHIRE
BOARD_WIFI_FUN(sapphire)
#endif
#ifdef CONFIG_MACH_TROUT
BOARD_WIFI_FUN(trout)
#endif
#endif

#ifdef CONFIG_ARCH_MSM7225
#ifdef CONFIG_MACH_MEMPHIS
BOARD_WIFI_FUN(memphis)
#endif
#ifdef CONFIG_MACH_JADE
BOARD_WIFI_FUN(jade)
#endif
#ifdef CONFIG_MACH_BAHAMAS
BOARD_WIFI_FUN(bahamas)
#endif
#endif
#ifdef CONFIG_ARCH_MSM7501A
#ifdef CONFIG_MACH_HEROC
BOARD_WIFI_FUN(heroc)
#endif
#ifdef CONFIG_MACH_DESIREC
BOARD_WIFI_FUN(desirec)
#endif
#endif

static struct wifi_platform_data board_wifi_tbl[] = {
#ifdef CONFIG_ARCH_MSM7200A
#ifdef CONFIG_MACH_HERO
	BOARD_WIFI(hero, "hero")
#endif
#ifdef CONFIG_MACH_HERO_ESPRESSO
	BOARD_WIFI(espresso, "espresso")
#endif
#ifdef CONFIG_MACH_SAPPHIRE
	BOARD_WIFI(sapphire, "sapphire")
#endif

#endif

#ifdef CONFIG_ARCH_MSM7201A
#ifdef CONFIG_MACH_SAPPHIRE
	BOARD_WIFI(sapphire, "sapphire")
#endif
#ifdef CONFIG_MACH_TROUT
	BOARD_WIFI(trout, "trout")
#endif
#endif

#ifdef CONFIG_ARCH_MSM7225
#ifdef CONFIG_MACH_MEMPHIS
	BOARD_WIFI(memphis, "memphis")
#endif
#ifdef CONFIG_MACH_JADE
	BOARD_WIFI(jade, "jade")
#endif
#ifdef CONFIG_MACH_BAHAMAS
	BOARD_WIFI(bahamas, "bahamas")
#endif
#endif

#ifdef CONFIG_ARCH_MSM7501A
#ifdef CONFIG_MACH_HEROC
	BOARD_WIFI(heroc, "heroc")
#endif
#ifdef CONFIG_MACH_DESIREC
	BOARD_WIFI(desirec, "desirec")
#endif
#endif
};


static struct wifi_platform_data *board_get(const char *id)
{
	int n;
	if (!id)
		return 0;
	for (n = 0; n < ARRAY_SIZE(board_wifi_tbl); n++) {
		if (!strcmp(board_wifi_tbl[n].name, id))
				return board_wifi_tbl + n;
	}
	return 0;
}

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct platform_device wifi_ctrl_dev = { 
	.name           = "msm_wifi",
	.id             = 1,
	.num_resources  = 0,
	.resource       = NULL,
};
#endif


int __init board_wifi_init(void)
{
#ifdef CONFIG_WIFI_CONTROL_FUNC
	int rc = 0;
#endif
	board_wifi = 0;

	/*
	 * add the case for the platform not here
	 */
	switch (machine_arch_type) {
#ifdef CONFIG_ARCH_MSM7200A
#ifdef CONFIG_MACH_HERO
	case MACH_TYPE_HERO:
		board_wifi = board_get("hero");
		break;
#endif
#ifdef CONFIG_MACH_SAPPHIRE
	case MACH_TYPE_SAPPHIRE:
		board_wifi = board_get("sapphire");
		break;
#endif
#ifdef CONFIG_MACH_HERO_ESPRESSO
	case MACH_TYPE_HERO_ESPRESSO:
		board_wifi = board_get("espresso");
		break;
#endif
#endif

#ifdef CONFIG_ARCH_MSM7201A
#ifdef CONFIG_MACH_SAPPHIRE
	case MACH_TYPE_SAPPHIRE:
		board_wifi = board_get("sapphire");
		break;
#endif
#ifdef CONFIG_MACH_TROUT
	case MACH_TYPE_TROUT:
		board_wifi = board_get("trout");
		break;
#endif
#endif

#ifdef CONFIG_ARCH_MSM7225
#ifdef CONFIG_MACH_MEMPHIS
	case MACH_TYPE_MEMPHIS:
		board_wifi = board_get("memphis");
		break;
#endif
#ifdef CONFIG_MACH_JADE
	case MACH_TYPE_JADE:
		board_wifi = board_get("jade");
		break;
	case MACH_TYPE_HTCJADE:
		board_wifi = board_get("jade");
		break;
#endif
#ifdef CONFIG_MACH_BAHAMAS
	case MACH_TYPE_BAHAMAS:
		board_wifi = board_get("bahamas");
		break;
#endif
#endif
#ifdef CONFIG_ARCH_MSM7501A
#ifdef CONFIG_MACH_HEROC
	case MACH_TYPE_HEROC:
		board_wifi = board_get("heroc");
		break;
#endif
#ifdef CONFIG_MACH_DESIREC
	case MACH_TYPE_DESIREC:
		board_wifi = board_get("desirec");
		break;
#endif
#endif
	default:
		printk(KERN_ERR "WiFi: unknown machine type %d\n", machine_arch_type);
		return -1;
	};

	if (!board_wifi)
		printk(KERN_ERR "WiFi: seek vainly for machine type\n");

#ifdef CONFIG_WIFI_CONTROL_FUNC
#ifdef CONFIG_WIFI_MEM_PREALLOC
	board_wifi->mem_prealloc = wifi_mem_prealloc;
#endif
	wifi_ctrl_dev.dev.platform_data = board_wifi;
	rc = platform_device_register(&wifi_ctrl_dev);
	if (rc)
		printk(KERN_CRIT "WiFi: control device init failure(%d)\n", rc);
	printk(KERN_INFO "WiFi: control device registered.\n");
	return rc;
#endif
	return 0;
}

#ifdef CONFIG_WIFI_CONTROL_FUNC
device_initcall(board_wifi_init);
#else
late_initcall(board_wifi_init);
#endif
