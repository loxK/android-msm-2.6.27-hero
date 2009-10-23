/* arch/arm/mach-msm/htc_battery.c
 *
 * Copyright (C) 2008 HTC Corporation.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <mach/htc_battery.h>
#include <linux/rtc.h>
#ifdef CONFIG_HTC_BATTCHG_SMEM
#include "smd_private.h"
#endif

static struct wake_lock vbus_wake_lock;

enum {
	HTC_BATT_DEBUG_M2A_RPC = 1U << 0,
	HTC_BATT_DEBUG_A2M_RPC = 1U << 1,
	HTC_BATT_DEBUG_UEVT = 1U << 2,
	HTC_BATT_DEBUG_USER_QUERY = 1U << 3,
	HTC_BATT_DEBUG_USB_NOTIFY = 1U << 4,
	HTC_BATT_DEBUG_SMEM = 1U << 5,
};
static int htc_batt_debug_mask = HTC_BATT_DEBUG_M2A_RPC | HTC_BATT_DEBUG_A2M_RPC
	| HTC_BATT_DEBUG_UEVT | HTC_BATT_DEBUG_USB_NOTIFY | HTC_BATT_DEBUG_SMEM;
module_param_named(debug_mask, htc_batt_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

//#define BATT_LOG(x...)	printk(KERN_INFO "batt: " x)
#define BATT_LOG(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_INFO "batt: " x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

//#define BATT_ERR(x...)	printk(KERN_ERR "batt: err: " x)
#define BATT_ERR(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_ERR "batt: err:" x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

/* rpc related */
#define APP_BATT_PDEV_NAME		"rs30100001:00000000"
#define APP_BATT_PROG			0x30100001
#define APP_BATT_VER			0
#define HTC_PROCEDURE_BATTERY_NULL	0
#define HTC_PROCEDURE_GET_BATT_LEVEL	1
#define HTC_PROCEDURE_GET_BATT_INFO	2
#define HTC_PROCEDURE_GET_CABLE_STATUS	3
#define HTC_PROCEDURE_SET_BATT_DELTA	4
#define HTC_PROCEDURE_SET_FULL_LEVEL	7

#define GPIO_TROUT_MBAT_IN		21
#define GPIO_TROUT_USB_ID		90
#define GPIO_TROUT_CHARGER_EN	128
#define GPIO_TROUT_ISET			129
/* Sapphire pin changes:
 *  USB_ID (GPIO 90) is renamed to AC_IN (GPIO 30)
 *  CHARGER_EN (CPLD MISC2 bit[0]) is move to PMIC (MPP_14).
 *  ISET (CPLD MISC2 bit[1]) is move to PMIC (MPP_13). */
#define GPIO_SAPPHIRE_USB_ID	30

#define GPIO_BATTERY_DETECTION		GPIO_TROUT_MBAT_IN
#define GPIO_BATTERY_CHARGER_EN		GPIO_TROUT_CHARGER_EN

/* Charge current selection */
#define GPIO_BATTERY_CHARGER_CURRENT	GPIO_TROUT_ISET


typedef enum {
	DISABLE = 0,
	ENABLE_SLOW_CHG,
	ENABLE_FAST_CHG
} batt_ctl_t;

/* This order is the same as htc_power_supplies[]
 * And it's also the same as htc_cable_status_update()
 */
typedef enum {
	CHARGER_UNKNOWN = -1,
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

const char *charger_tags[] = {"none", "USB", "AC"};

struct battery_info_reply {
	u32 batt_id;		/* Battery ID from ADC */
	u32 batt_vol;		/* Battery voltage from ADC */
	u32 batt_temp;		/* Battery Temperature (C) from formula and ADC */
	u32 batt_current;	/* Battery current from ADC */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 full_bat;		/* Full capacity of battery (mAh) */
};

struct htc_battery_info {
	int present;
	unsigned long update_time;

	/* lock to protect the battery info */
	struct mutex lock;

	/* lock held while calling the arm9 to query the battery info */
	struct mutex rpc_lock;
	struct battery_info_reply rep;
};

static struct msm_rpc_endpoint *endpoint;

static struct htc_battery_info htc_batt_info;

/* Remove cache mechanism to prevent cable status not sync. */
static unsigned int cache_time;

static int htc_battery_initial = 0;

static enum power_supply_property htc_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property htc_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

/* HTC dedicated attributes */
static ssize_t htc_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);

static int htc_power_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static int htc_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static struct power_supply htc_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = htc_battery_properties,
		.num_properties = ARRAY_SIZE(htc_battery_properties),
		.get_property = htc_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
};

static void usb_status_notifier_func(int online);
static int g_usb_online;
static struct t_usb_status_notifier usb_status_notifier = {
	.name = "htc_battery",
	.func = usb_status_notifier_func,
};

/* -------------------------------------------------------------------------- */
/* For sleep charging screen. */
static int zcharge_enabled;
static int htc_is_zcharge_enabled(void)
{
	return zcharge_enabled;
}
static int __init enable_zcharge_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	zcharge_enabled = cal;
	return 1;
}
__setup("enable_zcharge=", enable_zcharge_setup);

static int htc_is_cable_in(void)
{
	if (!htc_batt_info.update_time) {
		BATT_ERR("%s: battery driver hasn't been initialized yet.", __func__);
		return -EINVAL;
	}
	return (htc_batt_info.rep.charging_source != CHARGER_BATTERY) ? 1 : 0;
}

/**
 * htc_power_policy - check if it obeys our policy
 * return 0 for no errors, to indicate it follows policy.
 * non zero otherwise.
 **/
static int __htc_power_policy(void)
{
	if (!zcharge_enabled)
		return 0;

	if (htc_is_cable_in())
		return 1;

	return 0;
}

/*
 * Jay, 7/1/09'
 */
static int htc_power_policy(struct notifier_block *nfb,                 
		unsigned long action, void *ignored)                              
{       
	int rc;                                                                         
	switch (action) {
	case BATT_EVENT_SUSPEND:
		rc = __htc_power_policy();
		if (rc)
			return NOTIFY_STOP;
		else
			return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}                                                                                
#if defined(CONFIG_DEBUG_FS)
int htc_battery_set_charging(batt_ctl_t ctl);
static int batt_debug_set(void *data, u64 val)
{
	return htc_battery_set_charging((batt_ctl_t) val);
}

static int batt_debug_get(void *data, u64 *val)
{
	return -ENOSYS;
}

DEFINE_SIMPLE_ATTRIBUTE(batt_debug_fops, batt_debug_get, batt_debug_set, "%llu\n");
static int __init batt_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("htc_battery", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("charger_state", 0644, dent, NULL, &batt_debug_fops);

	return 0;
}

device_initcall(batt_debug_init);
#endif

static int init_batt_gpio(void)
{
	if (!machine_is_trout())
		return 0;

	if (gpio_request(GPIO_BATTERY_DETECTION, "batt_detect") < 0)
		goto gpio_failed;
	if (gpio_request(GPIO_BATTERY_CHARGER_EN, "charger_en") < 0)
		goto gpio_failed;
	if (gpio_request(GPIO_BATTERY_CHARGER_CURRENT, "charge_current") < 0)
		goto gpio_failed;

	return 0;

gpio_failed:	
	return -EINVAL;
	
}

/* 
 *	battery_charging_ctrl - battery charing control.
 * 	@ctl:			battery control command
 *
 */
static int battery_charging_ctrl(batt_ctl_t ctl)
{
	int result = 0;

	/* The charing operations are move to A9 in Sapphire. */
	if (!machine_is_trout())
		return result;

	switch (ctl) {
	case DISABLE:
		BATT_LOG("charger OFF");
		/* 0 for enable; 1 disable */
		result = gpio_direction_output(GPIO_TROUT_CHARGER_EN, 1);
		break;
	case ENABLE_SLOW_CHG:
		BATT_LOG("charger ON (SLOW)");
		result = gpio_direction_output(GPIO_TROUT_ISET, 0);
		result = gpio_direction_output(GPIO_TROUT_CHARGER_EN, 0);
		break;
	case ENABLE_FAST_CHG:
		BATT_LOG("charger ON (FAST)");
		result = gpio_direction_output(GPIO_TROUT_ISET, 1);
		result = gpio_direction_output(GPIO_TROUT_CHARGER_EN, 0);
		break;
	default:
		BATT_ERR("%s: Not supported battery ctr called.!", __func__);
		result = -EINVAL;
		break;
	}
	
	return result;
}

int htc_battery_set_charging(batt_ctl_t ctl)
{
	int rc;
	
	if ((rc = battery_charging_ctrl(ctl)) < 0)
		goto result;
	
	if (!htc_battery_initial) {
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
	} else {
		mutex_lock(&htc_batt_info.lock);
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
		mutex_unlock(&htc_batt_info.lock);
	}
result:	
	return rc;
}

int htc_battery_status_update(u32 curr_level)
{
	int notify;
	if (!htc_battery_initial)
		return 0;

	mutex_lock(&htc_batt_info.lock);
	notify = (htc_batt_info.rep.level != curr_level);
	htc_batt_info.rep.level = curr_level;
	mutex_unlock(&htc_batt_info.lock);
#if 0
	if (notify) {
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_UEVT)
			BATT_LOG("power_supply_changed: battery");
	}
#else
	/* we don't check level here for charging over temp RPC call */
	power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
	if (htc_batt_debug_mask & HTC_BATT_DEBUG_UEVT)
		BATT_LOG("power_supply_changed: battery");
#endif
	return 0;
}

#ifdef CONFIG_HTC_BATTCHG_SMEM
static int htc_set_smem_cable_type(u32 cable_type);
#else
static int htc_set_smem_cable_type(u32 cable_type) { return -ENOTSUP; }
#endif
int update_port_list_charging_state(int enable);
int htc_cable_status_update(int status)
{
	int rc = 0;
	unsigned last_source;

	/*
	BATT_LOG("%s: status=%d, htc_battery_initial=%d, g_usb_online=%d, last_source=%d",
		__func__, status, htc_battery_initial, g_usb_online, htc_batt_info.rep.charging_source);
	*/
	if (!htc_battery_initial)
		return 0;
	
	if (status < CHARGER_BATTERY || status > CHARGER_AC) {
		BATT_ERR("%s: Not supported cable status received!", __func__);
		return -EINVAL;
	}

	mutex_lock(&htc_batt_info.lock);
	/* A9 reports USB charging when helf AC cable in and China AC charger. */
	/* Work arround: notify userspace AC charging first,
	and notify USB charging again when receiving usb connected notificaiton from usb driver. */
	/* China AC detection:
	 * Write SMEM as AC first, and update SMEM to USB
	 * if receives USB notification */
	last_source = htc_batt_info.rep.charging_source;
	if (status == CHARGER_USB && g_usb_online == 0) {
		htc_set_smem_cable_type(CHARGER_AC);
		htc_batt_info.rep.charging_source = CHARGER_AC;
	} else {
		htc_set_smem_cable_type(status);
		htc_batt_info.rep.charging_source = status;
		/* usb driver will not notify usb offline. */
		if (status == CHARGER_BATTERY && g_usb_online == 1)
			g_usb_online = 0;
	}

	/* TODO: Don't call usb driver again with the same cable status. */
	msm_hsusb_set_vbus_state(status == CHARGER_USB);

	if (htc_batt_info.rep.charging_source != last_source) {
		update_port_list_charging_state(!(htc_batt_info.rep.charging_source == CHARGER_BATTERY));
		/* Lock suspend only when USB in for ADB or other USB functions. */
		if (htc_batt_info.rep.charging_source == CHARGER_USB) {
			wake_lock(&vbus_wake_lock);
		} else if (__htc_power_policy()) {
			/* Lock suspend for DOPOD charging animation */
			wake_lock(&vbus_wake_lock);
		} else {
			if (htc_batt_info.rep.charging_source == CHARGER_AC
					&& last_source == CHARGER_USB)
				BATT_ERR("%s: USB->AC\n", __func__);
			/* give userspace some time to see the uevent and update
			 * LED state or whatnot...
			 */
			wake_lock_timeout(&vbus_wake_lock, HZ*6);
		}
		if (htc_batt_info.rep.charging_source == CHARGER_BATTERY || last_source == CHARGER_BATTERY)
			power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
		if (htc_batt_info.rep.charging_source == CHARGER_USB || last_source == CHARGER_USB)
			power_supply_changed(&htc_power_supplies[CHARGER_USB]);
		if (htc_batt_info.rep.charging_source == CHARGER_AC || last_source == CHARGER_AC)
			power_supply_changed(&htc_power_supplies[CHARGER_AC]);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_UEVT)
			BATT_LOG("power_supply_changed: %s -> %s",
			charger_tags[last_source], charger_tags[htc_batt_info.rep.charging_source]);
	}
	mutex_unlock(&htc_batt_info.lock);

	return rc;
}

/* A9 reports USB charging when helf AC cable in and China AC charger. */
/* Work arround: notify userspace AC charging first,
and notify USB charging again when receiving usb connected notificaiton from usb driver. */
static void usb_status_notifier_func(int online)
{
	mutex_lock(&htc_batt_info.lock);
	if (htc_batt_debug_mask & HTC_BATT_DEBUG_USB_NOTIFY)
		BATT_LOG("%s: online=%d, g_usb_online=%d", __func__, online, g_usb_online);
	if (g_usb_online != online) {
		g_usb_online = online;
		if (online && htc_batt_info.rep.charging_source == CHARGER_AC) {
			mutex_unlock(&htc_batt_info.lock);
			htc_cable_status_update(CHARGER_USB);
			mutex_lock(&htc_batt_info.lock);
		} else if (online) {
			BATT_LOG("warning: usb connected but charging source=%d", htc_batt_info.rep.charging_source);
		}
	}
	mutex_unlock(&htc_batt_info.lock);
}
static int htc_get_batt_info(struct battery_info_reply *buffer)
{
	struct rpc_request_hdr req;
	
	struct htc_get_batt_info_rep {
		struct rpc_reply_hdr hdr;
		struct battery_info_reply info;
	} rep;
	
	int rc;

	if (buffer == NULL) 
		return -EINVAL;

	rc = msm_rpc_call_reply(endpoint, HTC_PROCEDURE_GET_BATT_INFO,
				&req, sizeof(req),
				&rep, sizeof(rep),
				5 * HZ);
	if (rc < 0)
		return rc;
	
	mutex_lock(&htc_batt_info.lock);
	buffer->batt_id 		= be32_to_cpu(rep.info.batt_id);
	buffer->batt_vol 		= be32_to_cpu(rep.info.batt_vol);
	buffer->batt_temp 		= be32_to_cpu(rep.info.batt_temp);
	buffer->batt_current 		= be32_to_cpu(rep.info.batt_current);
	buffer->level 			= be32_to_cpu(rep.info.level);
	/* Move the rules of charging_source to cable_status_update. */
	/* buffer->charging_source 	= be32_to_cpu(rep.info.charging_source); */
	buffer->charging_enabled 	= be32_to_cpu(rep.info.charging_enabled);
	buffer->full_bat 		= be32_to_cpu(rep.info.full_bat);
	mutex_unlock(&htc_batt_info.lock);

	if (htc_batt_debug_mask & HTC_BATT_DEBUG_A2M_RPC)
		BATT_LOG("A2M_RPC: get_batt_info: batt_id=%d, batt_vol=%d, batt_temp=%d, "
			"batt_current=%d, level=%d, charging_source=%d, "
			"charging_enabled=%d, full_bat=%d",
			buffer->batt_id, buffer->batt_vol, buffer->batt_temp,
			buffer->batt_current, buffer->level, buffer->charging_source,
			buffer->charging_enabled, buffer->full_bat);

	return 0;
}

#ifdef CONFIG_HTC_BATTCHG_SMEM
struct htc_batt_info_full {
	u32 batt_id;
	u32 batt_vol;
	u32 batt_vol_last;
	u32 batt_temp;
	u32 batt_current;
	u32 batt_current_last;
	u32 batt_discharge_current;

	u32 VREF_2;
	u32 VREF;
	u32 ADC4096_VREF;

	u32 Rtemp;
	s32  Temp;
	s32  Temp_last;

	u32 pd_M;
	u32 MBAT_pd;
	s32 I_MBAT;

	u32 pd_temp;
	u32 percent_last;
	u32 percent_update;
	u32 dis_percent;

	u32 vbus;
	u32 usbid;
	u32 charging_source;

	u32 MBAT_IN;
	u32 full_bat;

	u32 eval_current;
	u32 eval_current_last;
	u32 charging_enabled;

	u32 timeout;
	u32 fullcharge;
	u32 level;
	u32 delta;

	u32 chg_time;
	s32 level_change;
	u32 sleep_timer_count;
	u32 OT_led_on;
	u32 overloading_charge;

	u32 a2m_cable_type;
	u32 reserve2;
	u32 reserve3;
	u32 reserve4;
	u32 reserve5;
};

/* SMEM_BATT_INFO is allocated by A9 after first A2M RPC is sent. */
static struct htc_batt_info_full *smem_batt_info;

static int htc_get_batt_info_smem(struct battery_info_reply *buffer)
{
	if (!smem_batt_info) {
		smem_batt_info = smem_alloc(SMEM_BATT_INFO,
			sizeof(struct htc_batt_info_full));
		if (!smem_batt_info) {
			BATT_ERR("battery SMEM allocate fail, "
				"use RPC instead of\n");
			return htc_get_batt_info(buffer);
		}
	}

	if (!buffer)
		return -EINVAL;

	mutex_lock(&htc_batt_info.lock);
	buffer->batt_id = smem_batt_info->batt_id;
	buffer->batt_vol = smem_batt_info->batt_vol;
	buffer->batt_temp = smem_batt_info->Temp;
	buffer->batt_current = smem_batt_info->batt_current;
	/* Fix issue that recharging percent drop to 99%. */
	/* The level in SMEM is for A9 internal use,
	 * always use value reported by M2A level update RPC. */
#if 0
	buffer->level 	= smem_batt_info->percent_update;
#endif
	/* Move the rules of charging_source to cable_status_update. */
	/* buffer->charging_source 	= be32_to_cpu(smem_batt_info->charging_source); */
	buffer->charging_enabled = smem_batt_info->charging_enabled;
	buffer->full_bat = smem_batt_info->full_bat;
	mutex_unlock(&htc_batt_info.lock);

	if (htc_batt_debug_mask & HTC_BATT_DEBUG_SMEM)
		BATT_LOG("SMEM_BATT: get_batt_info: batt_id=%d, batt_vol=%d, batt_temp=%d, "
			"batt_current=%d, level=%d, charging_source=%d, "
			"charging_enabled=%d, full_bat=%d",
			buffer->batt_id, buffer->batt_vol, buffer->batt_temp,
			buffer->batt_current, buffer->level, buffer->charging_source,
			buffer->charging_enabled, buffer->full_bat);

	return 0;
}

static ssize_t htc_battery_show_smem(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int len = 0;

	if (!smem_batt_info) {
		smem_batt_info = smem_alloc(SMEM_BATT_INFO,
			sizeof(struct htc_batt_info_full));
		if (!smem_batt_info) {
			BATT_ERR("Show SMEM: allocate fail\n");
			return 0;
		}
	}

	if (!strcmp(attr->attr.name, "smem_raw")) {
		len = sizeof(struct htc_batt_info_full);
		memcpy(buf, smem_batt_info, len);
	} else if (!strcmp(attr->attr.name, "smem_text")) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"batt_id: %d\n"
			"batt_vol: %d\n"
			"batt_vol_last: %d\n"
			"batt_temp: %d\n"
			"batt_current: %d\n"
			"batt_current_last: %d\n"
			"batt_discharge_current: %d\n"
			"VREF_2: %d\n"
			"VREF: %d\n"
			"ADC4096_VREF: %d\n"
			"Rtemp: %d\n"
			"Temp: %d\n"
			"Temp_last: %d\n"
			"pd_M: %d\n"
			"MBAT_pd: %d\n"
			"I_MBAT: %d\n"
			"pd_temp: %d\n"
			"percent_last: %d\n"
			"percent_update: %d\n"
			"dis_percent: %d\n"
			"vbus: %d\n"
			"usbid: %d\n"
			"charging_source: %d\n"
			"MBAT_IN: %d\n"
			"full_bat: %d\n"
			"eval_current: %d\n"
			"eval_current_last: %d\n"
			"charging_enabled: %d\n"
			"timeout: %d\n"
			"fullcharge: %d\n"
			"level: %d\n"
			"delta: %d\n"
			"chg_time: %d\n"
			"level_change: %d\n"
			"sleep_timer_count: %d\n"
			"OT_led_on: %d\n"
			"overloading_charge: %d\n"
			"a2m_cable_type: %d\n",
			smem_batt_info->batt_id,
			smem_batt_info->batt_vol,
			smem_batt_info->batt_vol_last,
			smem_batt_info->batt_temp,
			smem_batt_info->batt_current,
			smem_batt_info->batt_current_last,
			smem_batt_info->batt_discharge_current,
			smem_batt_info->VREF_2,
			smem_batt_info->VREF,
			smem_batt_info->ADC4096_VREF,
			smem_batt_info->Rtemp,
			smem_batt_info->Temp,
			smem_batt_info->Temp_last,
			smem_batt_info->pd_M,
			smem_batt_info->MBAT_pd,
			smem_batt_info->I_MBAT,
			smem_batt_info->pd_temp,
			smem_batt_info->percent_last,
			smem_batt_info->percent_update,
			smem_batt_info->dis_percent,
			smem_batt_info->vbus,
			smem_batt_info->usbid,
			smem_batt_info->charging_source,
			smem_batt_info->MBAT_IN,
			smem_batt_info->full_bat,
			smem_batt_info->eval_current,
			smem_batt_info->eval_current_last,
			smem_batt_info->charging_enabled,
			smem_batt_info->timeout,
			smem_batt_info->fullcharge,
			smem_batt_info->level,
			smem_batt_info->delta,
			smem_batt_info->chg_time,
			smem_batt_info->level_change,
			smem_batt_info->sleep_timer_count,
			smem_batt_info->OT_led_on,
			smem_batt_info->overloading_charge,
			smem_batt_info->a2m_cable_type);
	}

	return len;
}

static int htc_set_smem_cable_type(u32 cable_type)
{
	if (!smem_batt_info) {
		smem_batt_info = smem_alloc(SMEM_BATT_INFO,
				sizeof(struct htc_batt_info_full));
		if (!smem_batt_info) {
			BATT_ERR("Update SMEM: allocate fail\n");
			return -EINVAL;
		}
	}

	smem_batt_info->a2m_cable_type = cable_type;
	BATT_LOG("Update SMEM: cable type %d\n", cable_type);

	return 0;
}
#endif

/* -------------------------------------------------------------------------- */
static int htc_power_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	
	mutex_lock(&htc_batt_info.lock);
	
	charger = htc_batt_info.rep.charging_source;
	/* ARM9 decides charging_enabled value by battery id */
	if (htc_batt_info.rep.batt_id == 255)
		charger = CHARGER_BATTERY;

	mutex_unlock(&htc_batt_info.lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
			if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
				BATT_LOG("%s: %s: online=%d", __func__, psy->name, val->intval);
		} else if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = (charger ==  CHARGER_USB ? 1 : 0);
			if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
				BATT_LOG("%s: %s: online=%d", __func__, psy->name, val->intval);
		} else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int htc_battery_get_charging_status(void)
{
	u32 level;
	charger_type_t charger;	
	int ret;
	
	mutex_lock(&htc_batt_info.lock);

	charger = htc_batt_info.rep.charging_source;
	/* ARM9 decides charging_enabled value by battery id */
	if (htc_batt_info.rep.batt_id == 255)
		charger = CHARGER_UNKNOWN;
	
	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
		level = htc_batt_info.rep.level;
		if (level == 100)
			ret = POWER_SUPPLY_STATUS_FULL;
		else
			ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	mutex_unlock(&htc_batt_info.lock);
	return ret;
}

static int htc_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = htc_battery_get_charging_status();
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: status=%d", __func__, psy->name, val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (htc_batt_info.rep.batt_temp >= 500 || htc_batt_info.rep.batt_temp <= 0)
			val->intval =  POWER_SUPPLY_HEALTH_OVERHEAT;
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: health=%d", __func__, psy->name, val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = htc_batt_info.present;
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: present=%d", __func__, psy->name, val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: technology=%d", __func__, psy->name, val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&htc_batt_info.lock);
		val->intval = htc_batt_info.rep.level;
		mutex_unlock(&htc_batt_info.lock);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY)
			BATT_LOG("%s: %s: capacity=%d", __func__, psy->name, val->intval);
		break;
	default:		
		return -EINVAL;
	}
	
	return 0;
}

#define HTC_BATTERY_ATTR(_name)							\
{										\
	.attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE },	\
	.show = htc_battery_show_property,					\
	.store = NULL,								\
}

static struct device_attribute htc_battery_attrs[] = {
	HTC_BATTERY_ATTR(batt_id),
	HTC_BATTERY_ATTR(batt_vol),
	HTC_BATTERY_ATTR(batt_temp),
	HTC_BATTERY_ATTR(batt_current),
	HTC_BATTERY_ATTR(charging_source),
	HTC_BATTERY_ATTR(charging_enabled),
	HTC_BATTERY_ATTR(full_bat),
#ifdef CONFIG_HTC_BATTCHG_SMEM
	__ATTR(smem_raw, S_IRUGO, htc_battery_show_smem, NULL),
	__ATTR(smem_text, S_IRUGO, htc_battery_show_smem, NULL),
#endif
};

enum {
	BATT_ID = 0,
	BATT_VOL,
	BATT_TEMP,
	BATT_CURRENT,
	CHARGING_SOURCE,
	CHARGING_ENABLED,
	FULL_BAT,
};

static int htc_rpc_set_delta(unsigned delta)
{
	struct set_batt_delta_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	req.data = cpu_to_be32(delta);
	return msm_rpc_call(endpoint, HTC_PROCEDURE_SET_BATT_DELTA,
			    &req, sizeof(req), 5 * HZ);
}

static int htc_rpc_set_full_level(unsigned level)
{
	struct set_batt_full_level_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	req.data = cpu_to_be32(level);
	return msm_rpc_call(endpoint, HTC_PROCEDURE_SET_FULL_LEVEL,
			    &req, sizeof(req), 5 * HZ);
}

static ssize_t htc_battery_set_delta(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int rc;
	unsigned long delta = 0;
	
	delta = simple_strtoul(buf, NULL, 10);

	if (delta > 100)
		return -EINVAL;

	mutex_lock(&htc_batt_info.rpc_lock);
	rc = htc_rpc_set_delta(delta);
	mutex_unlock(&htc_batt_info.rpc_lock);
	if (rc < 0)
		return rc;
	return count;
}

static ssize_t htc_battery_set_full_level(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int rc;
	unsigned long percent = 100;

	percent = simple_strtoul(buf, NULL, 10);

	if (percent > 100 || percent == 0)
		return -EINVAL;

	mutex_lock(&htc_batt_info.rpc_lock);
	rc = htc_rpc_set_full_level(percent);
	mutex_unlock(&htc_batt_info.rpc_lock);
	if (rc < 0)
		return rc;
	return count;
}

static struct device_attribute htc_set_delta_attrs[] = {
	__ATTR(delta, S_IWUSR | S_IWGRP, NULL, htc_battery_set_delta),
	__ATTR(full_level, S_IWUSR | S_IWGRP, NULL, htc_battery_set_full_level),
};

static int htc_battery_create_attrs(struct device * dev)
{
	int i = 0, j = 0, rc = 0;
	
	for (i = 0; i < ARRAY_SIZE(htc_battery_attrs); i++) {
		rc = device_create_file(dev, &htc_battery_attrs[i]);
		if (rc)
			goto htc_attrs_failed;
	}

	for (j = 0; j < ARRAY_SIZE(htc_set_delta_attrs); j++) {
		rc = device_create_file(dev, &htc_set_delta_attrs[j]);
		if (rc)
			goto htc_delta_attrs_failed;
	}
	
	goto succeed;
	
htc_attrs_failed:
	while (i--)
		device_remove_file(dev, &htc_battery_attrs[i]);
htc_delta_attrs_failed:
	while (j--)
		device_remove_file(dev, &htc_set_delta_attrs[i]);
succeed:	
	return rc;
}

static ssize_t htc_battery_show_property(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - htc_battery_attrs;
	
	/* rpc lock is used to prevent two threads from calling
	 * into the get info rpc at the same time
	 */

	mutex_lock(&htc_batt_info.rpc_lock);
	/* check cache time to decide if we need to update */
	if (htc_batt_info.update_time &&
			time_before(jiffies, htc_batt_info.update_time +
			msecs_to_jiffies(cache_time))) {
		BATT_LOG("%s: use cached values", __func__);
		goto dont_need_update;
	}

#ifdef CONFIG_HTC_BATTCHG_SMEM
	if (htc_get_batt_info_smem(&htc_batt_info.rep) < 0) {
		BATT_ERR("%s: smem read failed!!!", __func__);
	} else {
		htc_batt_info.update_time = jiffies;
	}
#else
	if (htc_get_batt_info(&htc_batt_info.rep) < 0) {
		BATT_ERR("%s: rpc failed!!!", __func__);
	} else {
		htc_batt_info.update_time = jiffies;
	}
#endif

dont_need_update:
	mutex_unlock(&htc_batt_info.rpc_lock);

	mutex_lock(&htc_batt_info.lock);
	switch (off) {
	case BATT_ID:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_id);
		break;
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_vol);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_temp);
		break;
	case BATT_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_current);
		break;
	case CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_source);
		break;
	case CHARGING_ENABLED:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_enabled);
		break;		
	case FULL_BAT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.full_bat);
		break;
	default:
		i = -EINVAL;
	}	
	mutex_unlock(&htc_batt_info.lock);
	
	if (htc_batt_debug_mask & HTC_BATT_DEBUG_USER_QUERY) {
		if (i < 0)
			BATT_LOG("%s: battery: attribute is not supported: %d", __func__, off);
		else
			BATT_LOG("%s: battery: %s=%s", __func__, attr->attr.name, buf);
	}
	return i;
}

static int htc_battery_probe(struct platform_device *pdev)
{
	int i, rc;

	/* init battery gpio */
	if ((rc = init_batt_gpio()) < 0) {
		BATT_ERR("%s: init battery gpio failed!", __func__);
		return rc;
	}

	/* init structure data member */
	htc_batt_info.update_time 	= jiffies;
	/* A9 will shutdown the phone if battery is pluged out, so this value is always 1.
	htc_batt_info.present 		= gpio_get_value(GPIO_TROUT_MBAT_IN);
	*/
	htc_batt_info.present 		= 1;
	
	/* init rpc */
	endpoint = msm_rpc_connect(APP_BATT_PROG, APP_BATT_VER, 0);
	if (IS_ERR(endpoint)) {
		BATT_ERR("%s: init rpc failed! rc = %ld",
		       __func__, PTR_ERR(endpoint));
		return rc;
	}

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(htc_power_supplies); i++) {
		rc = power_supply_register(&pdev->dev, &htc_power_supplies[i]);
		if (rc)
			BATT_ERR("%s: Failed to register power supply (%d)", __func__, rc);
	}

	/* create htc detail attributes */
	htc_battery_create_attrs(htc_power_supplies[CHARGER_BATTERY].dev);

	/* After battery driver gets initialized, send rpc request to inquiry
	 * the battery status in case of we lost some info
	 */
	htc_battery_initial = 1;

	mutex_lock(&htc_batt_info.rpc_lock);
	htc_batt_info.rep.charging_source = CHARGER_BATTERY;
	if (htc_get_batt_info(&htc_batt_info.rep) < 0)
		BATT_ERR("%s: get info failed", __func__);

	if (htc_rpc_set_delta(1) < 0)
		BATT_ERR("%s: set delta failed", __func__);
	htc_batt_info.update_time = jiffies;
	mutex_unlock(&htc_batt_info.rpc_lock);
	return 0;
}

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= APP_BATT_PDEV_NAME,
		.owner	= THIS_MODULE,
	},
};

/* batt_mtoa server definitions */
#define BATT_MTOA_PROG				0x30100000
#define BATT_MTOA_VERS				0
#define RPC_BATT_MTOA_NULL			0
#define RPC_BATT_MTOA_SET_CHARGING_PROC		1
#define RPC_BATT_MTOA_CABLE_STATUS_UPDATE_PROC	2
#define RPC_BATT_MTOA_LEVEL_UPDATE_PROC		3

struct rpc_batt_mtoa_set_charging_args {
	int enable;
};

struct rpc_batt_mtoa_cable_status_update_args {
	int status;
};

struct rpc_dem_battery_update_args {
	uint32_t level;
};

static int handle_battery_call(struct msm_rpc_server *server,
			       struct rpc_request_hdr *req, unsigned len)
{	
	switch (req->procedure) {
	case RPC_BATT_MTOA_NULL:
		return 0;

	case RPC_BATT_MTOA_SET_CHARGING_PROC: {
		struct rpc_batt_mtoa_set_charging_args *args;
		args = (struct rpc_batt_mtoa_set_charging_args *)(req + 1);
		args->enable = be32_to_cpu(args->enable);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_M2A_RPC)
			BATT_LOG("M2A_RPC: set_charging: %d", args->enable);
		htc_battery_set_charging(args->enable);
		return 0;
	}
	case RPC_BATT_MTOA_CABLE_STATUS_UPDATE_PROC: {
		struct rpc_batt_mtoa_cable_status_update_args *args;
		args = (struct rpc_batt_mtoa_cable_status_update_args *)(req + 1);
		args->status = be32_to_cpu(args->status);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_M2A_RPC)
			BATT_LOG("M2A_RPC: cable_update: %s", charger_tags[args->status]);
		htc_cable_status_update(args->status);
		return 0;
	}
	case RPC_BATT_MTOA_LEVEL_UPDATE_PROC: {
		struct rpc_dem_battery_update_args *args;
		args = (struct rpc_dem_battery_update_args *)(req + 1);
		args->level = be32_to_cpu(args->level);
		if (htc_batt_debug_mask & HTC_BATT_DEBUG_M2A_RPC)
			BATT_LOG("M2A_RPC: level_update: %d", args->level);
		htc_battery_status_update(args->level);
		return 0;
	}
	default:
		BATT_ERR("%s: program 0x%08x:%d: unknown procedure %d",
		       __func__, req->prog, req->vers, req->procedure);
		return -ENODEV;
	}
}

static struct msm_rpc_server battery_server = {
	.prog = BATT_MTOA_PROG,
	.vers = BATT_MTOA_VERS,
	.rpc_call = handle_battery_call,
};

static struct notifier_block batt_notify = {
	.notifier_call = htc_power_policy,
};

static BLOCKING_NOTIFIER_HEAD(battery_notifier_list);
int batt_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&battery_notifier_list, nb);
}

int batt_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&battery_notifier_list, nb);
}

int batt_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&battery_notifier_list, val, v);
}

static int __init htc_battery_init(void)
{
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	mutex_init(&htc_batt_info.lock);
	mutex_init(&htc_batt_info.rpc_lock);
	usb_register_notifier(&usb_status_notifier);
	msm_rpc_create_server(&battery_server);
	platform_driver_register(&htc_battery_driver);
	batt_register_client(&batt_notify);
	return 0;
}

module_init(htc_battery_init);
MODULE_DESCRIPTION("HTC Battery Driver");
MODULE_LICENSE("GPL");
EXPORT_SYMBOL(htc_is_cable_in);
EXPORT_SYMBOL(htc_is_zcharge_enabled);
