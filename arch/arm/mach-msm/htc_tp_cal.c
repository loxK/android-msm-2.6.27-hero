/* arch/arm/mach-msm/htc_tp_cal.c
 *
 * Code to extract TP calibration information from ATAG set up 
 * by the bootloader.
 *
 * Copyright (C) 2009 HTC Corporation
 * Author: Zion Huang <Zion_Huang@htc.com>
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/setup.h>

/* configuration tags specific to TP */
#define ATAG_TP	0x41387898 /* TP */ //0x89768976//akm //0x57494649//Wifi
//#define ATAG_TP	0x89768976//akm

//#define MAX_CALI_SIZE	0x18U
//#define MAX_CALI_SIZE	0x40U//0x1000U//12
#define MAX_CALI_SIZE	0x20U//0x1000U//12
#define ATAG_TP_DEBUG

static unsigned char tp_cal_ram[MAX_CALI_SIZE];
//static short tp_cal_ram[MAX_CALI_SIZE];

unsigned char *get_tp_cal_ram(void)
//short *get_tp_cal_ram(void)
{
	return(tp_cal_ram);
}
EXPORT_SYMBOL(get_tp_cal_ram);

static int __init parse_tag_tp(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	unsigned size;

	size = min((tag->hdr.size - 2) * sizeof(__u32), MAX_CALI_SIZE);
	//size = min((tag->hdr.size - 2) * sizeof(__u16), MAX_CALI_SIZE);

	printk(KERN_INFO "TP Data size = %d , 0x%x, size = %d\n",
			tag->hdr.size, tag->hdr.tag, size);

#ifdef ATAG_TP_DEBUG
	unsigned i;
	unsigned char *ptr;

	ptr = dptr;
	printk(KERN_INFO
	       "TP Data size = %d , 0x%x\n",
	       tag->hdr.size, tag->hdr.tag);
	for (i = 0; i < size; i++)
		printk(KERN_INFO "%02x ", *ptr++);
#endif
	memcpy((void *)tp_cal_ram, (void *)dptr, size);
	return 0;
}

__tagtable(ATAG_TP, parse_tag_tp);
