/*
 * Copyright (C) 2007 HTC Incorporated
 * Author: Jay Tu (jay_tu@htc.com)
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
#ifndef _HTC_BATTERY_H_
#define _HTC_BATTERY_H_
#include <linux/notifier.h>

#define BATT_EVENT_SUSPEND	0x01

#ifdef CONFIG_HTC_BATTCHG
extern int batt_register_client(struct notifier_block *nb);
extern int batt_unregister_client(struct notifier_block *nb);
extern int batt_notifier_call_chain(unsigned long val, void *v);
#else
static int batt_register_client(struct notifier_block *nb)
{
	return 0;
}

static int batt_unregister_client(struct notifier_block *nb)
{
	return 0;
}

static int batt_notifier_call_chain(unsigned long val, void *v)
{
	return 0;
}
#endif
#endif
