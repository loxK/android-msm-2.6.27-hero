/* arch/arm/mach-msm/perflock.c
 *
 * Copyright (C) 2008 HTC Corporation
 * Author: Eiven Peng <eiven_peng@htc.com>
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/earlysuspend.h>
#include <linux/cpufreq.h>
#include <linux/timer.h>
#include <mach/perflock.h>
#include <mach/proc_comm.h>
#include "acpuclock.h"

#define PERF_LOCK_INITIALIZED	(1U << 0)
#define PERF_LOCK_ACTIVE		(1U << 1)

enum {
	PERF_LOCK_DEBUG = 1U << 0,
	PERF_EXPIRE_DEBUG = 1U << 1,
	PERF_CPUFREQ_NOTIFY_DEBUG = 1U << 2,
	PERF_CPUFREQ_LOCK_DEBUG = 1U << 3,
	PERF_SCREEN_ON_POLICY_DEBUG = 1U << 4,
};

static LIST_HEAD(active_perf_locks);
static LIST_HEAD(inactive_perf_locks);
static DEFINE_SPINLOCK(list_lock);
static DEFINE_SPINLOCK(policy_update_lock);
static int initialized;
static unsigned int *perf_acpu_table;
static unsigned int table_size;
static unsigned int curr_lock_speed;
static struct cpufreq_policy *cpufreq_policy;

#ifdef CONFIG_PERF_LOCK_DEBUG
static int debug_mask = PERF_LOCK_DEBUG | PERF_EXPIRE_DEBUG |
	PERF_CPUFREQ_NOTIFY_DEBUG | PERF_CPUFREQ_LOCK_DEBUG;
#else
static int debug_mask = PERF_CPUFREQ_LOCK_DEBUG | PERF_SCREEN_ON_POLICY_DEBUG;
#endif
module_param_call(debug_mask, param_set_int, param_get_int,
		&debug_mask, S_IWUSR | S_IRUGO);

static unsigned int get_perflock_speed(void);
static void print_active_locks(void);

#ifdef CONFIG_PERFLOCK_SCREEN_POLICY
/* Increase cpufreq minumum frequency when screen on.
    Pull down to lowest speed when screen off. */
static unsigned int screen_off_policy_req;
static unsigned int screen_on_policy_req;
static void perflock_early_suspend(struct early_suspend *handler)
{
	unsigned long irqflags;

	spin_lock_irqsave(&policy_update_lock, irqflags);
	if (screen_on_policy_req) {
		screen_on_policy_req--;
		spin_unlock_irqrestore(&policy_update_lock, irqflags);
		return;
	}
	screen_off_policy_req++;
	spin_unlock_irqrestore(&policy_update_lock, irqflags);

	if (cpufreq_policy)
		cpufreq_update_policy(cpufreq_policy->cpu);
}

static void perflock_late_resume(struct early_suspend *handler)
{
	unsigned long irqflags;

	/* Work around for display driver, need to increase cpu speed immediately. */
	unsigned int lock_speed = get_perflock_speed() / 1000;
	if (lock_speed > CONFIG_PERFLOCK_SCREEN_ON_MIN) {
		acpuclk_set_rate(lock_speed * 1000, 0);
	}
	else {
		acpuclk_set_rate(CONFIG_PERFLOCK_SCREEN_ON_MIN * 1000, 0);
	}

	spin_lock_irqsave(&policy_update_lock, irqflags);
	if (screen_off_policy_req) {
		screen_off_policy_req--;
		spin_unlock_irqrestore(&policy_update_lock, irqflags);
		return;
	}
	screen_on_policy_req++;
	spin_unlock_irqrestore(&policy_update_lock, irqflags);

	if (cpufreq_policy)
		cpufreq_update_policy(cpufreq_policy->cpu);
}

static struct early_suspend perflock_power_suspend = {
	.suspend = perflock_early_suspend,
	.resume = perflock_late_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};

static int __init perflock_screen_policy_init(void)
{
	register_early_suspend(&perflock_power_suspend);

	screen_on_policy_req++;
	if (cpufreq_policy)
		cpufreq_update_policy(cpufreq_policy->cpu);

	return 0;
}

late_initcall(perflock_screen_policy_init);
#endif

static unsigned int policy_min = CONFIG_MSM_CPU_FREQ_SCALING_MIN;
static unsigned int policy_max = CONFIG_MSM_CPU_FREQ_SCALING_MAX;
static int perflock_notifier_call(struct notifier_block *self,
			       unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int lock_speed;
	unsigned long irqflags;

	spin_lock_irqsave(&policy_update_lock, irqflags);
	if (debug_mask & PERF_CPUFREQ_NOTIFY_DEBUG)
		pr_info("%s: event=%ld, policy->min=%d, policy->max=%d\n",
			__func__, event, policy->min, policy->max);

	if (event == CPUFREQ_START)
		cpufreq_policy = policy;
	else if (event == CPUFREQ_NOTIFY) {
		/* Each time cpufreq_update_policy, min/max will reset, need to set it again. */
#ifdef CONFIG_PERFLOCK_SCREEN_POLICY
		if (screen_on_policy_req) {
			if (debug_mask & PERF_SCREEN_ON_POLICY_DEBUG)
				pr_info("%s: screen_on_policy_req %d, policy_min %d\n",
					__func__, screen_on_policy_req, CONFIG_PERFLOCK_SCREEN_ON_MIN);
			policy_min = CONFIG_PERFLOCK_SCREEN_ON_MIN;
			policy_max = CONFIG_PERFLOCK_SCREEN_ON_MAX;
			screen_on_policy_req--;
		} else if (screen_off_policy_req) {
			if (debug_mask & PERF_SCREEN_ON_POLICY_DEBUG)
				pr_info("%s: screen_off_policy_req %d, policy_min %d\n",
					__func__, screen_off_policy_req, CONFIG_MSM_CPU_FREQ_SCALING_MIN);
			policy_min = CONFIG_PERFLOCK_SCREEN_OFF_MIN;
			policy_max = CONFIG_PERFLOCK_SCREEN_OFF_MAX;
			screen_off_policy_req--;
		}
#endif
		lock_speed = get_perflock_speed() / 1000;
		if (lock_speed) {
			policy->min = lock_speed;
			policy->max = lock_speed;
			if (debug_mask & PERF_CPUFREQ_LOCK_DEBUG) {
				pr_info("%s: cpufreq lock speed %d\n",
					__func__, lock_speed);
				print_active_locks();
			}
		} else {
			policy->min = policy_min;
			policy->max = policy_max;
			if (debug_mask & PERF_CPUFREQ_LOCK_DEBUG)
				pr_info("%s: cpufreq recover policy %d %d\n",
					__func__, policy->min, policy->max);
		}
		curr_lock_speed = lock_speed;
	}
	spin_unlock_irqrestore(&policy_update_lock, irqflags);

	return 0;
}

static struct notifier_block perflock_notifier = {
	.notifier_call = perflock_notifier_call,
};

static unsigned int get_perflock_speed(void)
{
	unsigned long irqflags;
	struct perf_lock *lock;
	unsigned int perf_level = 0;

	/* Get the maxmimum perf level. */
	if (list_empty(&active_perf_locks))
		return 0;

	spin_lock_irqsave(&list_lock, irqflags);
	list_for_each_entry(lock, &active_perf_locks, link) {
		if (lock->level > perf_level)
			perf_level = lock->level;
	}
	spin_unlock_irqrestore(&list_lock, irqflags);

	return perf_acpu_table[perf_level];
}

static void print_active_locks(void)
{
	unsigned long irqflags;
	struct perf_lock *lock;

	spin_lock_irqsave(&list_lock, irqflags);
	list_for_each_entry(lock, &active_perf_locks, link) {
		pr_info("active perf lock '%s'\n", lock->name);
	}
	spin_unlock_irqrestore(&list_lock, irqflags);
}

void perf_lock_init(struct perf_lock *lock, unsigned int level, const char *name)
{
	unsigned long irqflags = 0;

	WARN_ON(!name);
	WARN_ON(level >= PERF_LOCK_INVALID);
	WARN_ON(lock->flags & PERF_LOCK_INITIALIZED);

	if ((!name) || (level >= PERF_LOCK_INVALID) ||
			(lock->flags & PERF_LOCK_INITIALIZED)) {
		pr_err("%s: ERROR \"%s\" flags %x level %d\n",
			__func__, name, lock->flags, level);
		return;
	}
	lock->name = name;
	lock->flags = PERF_LOCK_INITIALIZED;
	lock->level = level;

	INIT_LIST_HEAD(&lock->link);
	spin_lock_irqsave(&list_lock, irqflags);
	list_add(&lock->link, &inactive_perf_locks);
	spin_unlock_irqrestore(&list_lock, irqflags);
}
EXPORT_SYMBOL(perf_lock_init);

void perf_lock(struct perf_lock *lock)
{
	unsigned long irqflags;

	WARN_ON(!initialized);
	WARN_ON((lock->flags & PERF_LOCK_INITIALIZED) == 0);
	WARN_ON(lock->flags & PERF_LOCK_ACTIVE);

	spin_lock_irqsave(&list_lock, irqflags);
	if (debug_mask & PERF_LOCK_DEBUG)
		pr_info("%s: '%s', flags %d level %d\n",
			__func__, lock->name, lock->flags, lock->level);
	if (lock->flags & PERF_LOCK_ACTIVE) {
		pr_err("%s: over-locked\n", __func__);
		return;
	}
	lock->flags |= PERF_LOCK_ACTIVE;
	list_del(&lock->link);
	list_add(&lock->link, &active_perf_locks);
	spin_unlock_irqrestore(&list_lock, irqflags);

	/* Update cpufreq policy - scaling_min/scaling_max */
	if (cpufreq_policy &&
			(curr_lock_speed != (get_perflock_speed() / 1000)))
		cpufreq_update_policy(cpufreq_policy->cpu);
}
EXPORT_SYMBOL(perf_lock);

#define PERF_UNLOCK_DELAY		(HZ)
static void do_expire_perf_locks(struct work_struct *work)
{
	if (debug_mask & PERF_EXPIRE_DEBUG)
		pr_info("%s: timed out to unlock\n", __func__);

	if (list_empty(&active_perf_locks) && cpufreq_policy) {
		if (debug_mask & PERF_EXPIRE_DEBUG)
			pr_info("%s: update cpufreq policy\n", __func__);
		cpufreq_update_policy(cpufreq_policy->cpu);
	}
}
static DECLARE_DELAYED_WORK(work_expire_perf_locks, do_expire_perf_locks);

void perf_unlock(struct perf_lock *lock)
{
	unsigned long irqflags;

	WARN_ON(!initialized);
	WARN_ON((lock->flags & PERF_LOCK_ACTIVE) == 0);

	spin_lock_irqsave(&list_lock, irqflags);
	if (debug_mask & PERF_LOCK_DEBUG)
		pr_info("%s: '%s', flags %d level %d\n",
			__func__, lock->name, lock->flags, lock->level);
	if (!(lock->flags & PERF_LOCK_ACTIVE)) {
		pr_err("%s: under-locked\n", __func__);
		return;
	}
	lock->flags &= ~PERF_LOCK_ACTIVE;
	list_del(&lock->link);
	list_add(&lock->link, &inactive_perf_locks);
	spin_unlock_irqrestore(&list_lock, irqflags);

	/* Prevent lock/unlock quickly, add a timeout to release perf_lock */
	if (list_empty(&active_perf_locks))
		schedule_delayed_work(&work_expire_perf_locks,
			PERF_UNLOCK_DELAY);
	else
		cancel_delayed_work_sync(&work_expire_perf_locks);
}
EXPORT_SYMBOL(perf_unlock);

inline int is_perf_lock_active(struct perf_lock *lock)
{
	return (lock->flags & PERF_LOCK_ACTIVE);
}
EXPORT_SYMBOL(is_perf_lock_active);

int is_perf_locked(void)
{
	return (!list_empty(&active_perf_locks));
}
EXPORT_SYMBOL(is_perf_locked);


#ifdef CONFIG_PERFLOCK_BOOT_LOCK
/* Stop cpufreq and lock cpu, shorten boot time. */
#define BOOT_LOCK_TIMEOUT	(60 * HZ)
static struct perf_lock boot_perf_lock;

static void do_expire_boot_lock(struct work_struct *work)
{
	perf_unlock(&boot_perf_lock);
	pr_info("Release 'boot-time' perf_lock\n");
}
static DECLARE_DELAYED_WORK(work_expire_boot_lock, do_expire_boot_lock);
#endif

void __init perflock_init(struct perflock_platform_data *pdata)
{
	if (!pdata)
		goto invalid_config;

	perf_acpu_table = pdata->perf_acpu_table;
	table_size = pdata->table_size;
	if (!perf_acpu_table || !table_size)
		goto invalid_config;
	if (table_size < PERF_LOCK_INVALID)
		goto invalid_config;
	cpufreq_register_notifier(&perflock_notifier, CPUFREQ_POLICY_NOTIFIER);

	initialized = 1;

#ifdef CONFIG_PERFLOCK_BOOT_LOCK
	/* Stop cpufreq and lock cpu, shorten boot time. */
	perf_lock_init(&boot_perf_lock, PERF_LOCK_HIGHEST, "boot-time");
	perf_lock(&boot_perf_lock);
	schedule_delayed_work(&work_expire_boot_lock, BOOT_LOCK_TIMEOUT);
	pr_info("Acquire 'boot-time' perf_lock\n");
#endif

	return;

invalid_config:
	pr_err("%s: invalid configuration data, %p %d %d\n", __func__,
		perf_acpu_table, table_size, PERF_LOCK_INVALID);
}
