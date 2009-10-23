/* arch/arm/mach-msm/clock.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spinlock.h>

#include "clock.h"
#include <mach/proc_comm.h>
#include <mach/msm_rpcrouter.h>
#include <linux/debugfs.h>

static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clocks_lock);
static LIST_HEAD(clocks);

/*
 * glue for the proc_comm interface
 */
static inline int pc_clk_enable(unsigned id)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_ENABLE, &id, NULL);
}

static inline void pc_clk_disable(unsigned id)
{
	msm_proc_comm(PCOM_CLKCTL_RPC_DISABLE, &id, NULL);
}

static inline int pc_clk_set_rate(unsigned id, unsigned rate)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_SET_RATE, &id, &rate);
}

static int pc_clk_set_min_rate(unsigned id, unsigned rate)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_MIN_RATE, &id, &rate);
}

static inline int pc_clk_set_max_rate(unsigned id, unsigned rate)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_MAX_RATE, &id, &rate);
}

static inline int pc_clk_set_flags(unsigned id, unsigned flags)
{
	return msm_proc_comm(PCOM_CLKCTL_RPC_SET_FLAGS, &id, &flags);
}

static inline unsigned pc_clk_get_rate(unsigned id)
{
	if (msm_proc_comm(PCOM_CLKCTL_RPC_RATE, &id, NULL))
		return 0;
	else
		return id;
}

static inline unsigned pc_clk_is_enabled(unsigned id)
{
	if (msm_proc_comm(PCOM_CLKCTL_RPC_ENABLED, &id, NULL))
		return 0;
	else
		return id;
}

static inline int pc_pll_request(unsigned id, unsigned on)
{
	on = !!on;
	return msm_proc_comm(PCOM_CLKCTL_RPC_PLL_REQUEST, &id, &on);
}

/*
 * Standard clock functions defined in include/linux/clk.h
 */
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *clk;

	mutex_lock(&clocks_mutex);

	list_for_each_entry(clk, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == dev)
			goto found_it;

	list_for_each_entry(clk, &clocks, list)
		if (!strcmp(id, clk->name) && clk->dev == NULL)
			goto found_it;

	clk = ERR_PTR(-ENOENT);
found_it:
	mutex_unlock(&clocks_mutex);
	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&clocks_lock, flags);
	clk->count++;
	if (clk->count == 1)
		pc_clk_enable(clk->id);
	spin_unlock_irqrestore(&clocks_lock, flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->count > 0)
		clk->count--;
	if (clk->count == 0)
		pc_clk_disable(clk->id);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	return pc_clk_get_rate(clk->id);
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EPERM;
	if (clk->flags & CLKFLAG_USE_MAX_TO_SET) {
		ret = pc_clk_set_max_rate(clk->id, rate);
		if (ret) {
			printk(KERN_ERR "pc_clk_set_max_rate failed (%s)\n",
				clk->name);
			return ret;
		}
	}
	if (clk->flags & CLKFLAG_USE_MIN_TO_SET) {
		ret = pc_clk_set_min_rate(clk->id, rate);
		if (ret) {
			printk(KERN_ERR "pc_clk_set_min_rate failed (%s)\n",
							clk->name);
			return ret;
		}
	}
	if (clk->flags & CLKFLAG_USE_MAX_TO_SET ||
		clk->flags & CLKFLAG_USE_MIN_TO_SET)
		return ret;

	return pc_clk_set_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	return -ENOSYS;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_flags(struct clk *clk, unsigned long flags)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;
	return pc_clk_set_flags(clk->id, flags);
}
EXPORT_SYMBOL(clk_set_flags);


void __init msm_clock_init(void)
{
	struct clk *clk;

	spin_lock_init(&clocks_lock);
	mutex_lock(&clocks_mutex);
	for (clk = msm_clocks; clk && clk->name; clk++) {
		list_add_tail(&clk->list, &clocks);
	}
	mutex_unlock(&clocks_mutex);
}

/* The bootloader and/or AMSS may have left various clocks enabled.
 * Disable any clocks that belong to us (CLKFLAG_AUTO_OFF) but have
 * not been explicitly enabled by a clk_enable() call.
 */
static int __init clock_late_init(void)
{
	unsigned long flags;
	struct clk *clk;
	unsigned count = 0;

	mutex_lock(&clocks_mutex);
	list_for_each_entry(clk, &clocks, list) {
		if (clk->flags & CLKFLAG_AUTO_OFF) {
			spin_lock_irqsave(&clocks_lock, flags);
			if (!clk->count) {
				count++;
				pc_clk_disable(clk->id);
			}
			spin_unlock_irqrestore(&clocks_lock, flags);
		}
	}
	mutex_unlock(&clocks_mutex);
	pr_info("clock_late_init() disabled %d unused clocks\n", count);
	return 0;
}

late_initcall(clock_late_init);

#ifdef CONFIG_DEBUG_FS
#define A9_MPLL_ID	1
#define A9_GPLL_ID	2
#define A9_BPLL_0_ID	3
#define A9_BPLL_1_ID	4
#define HTC_A2M_PLL_RPOG	0x30100002
#define HTC_A2M_PLL_VERS 	0
#define HTC_ONCRPC_GET_PLL_STATE	7

struct clk_pll {
	const char	*name;
	uint32_t		id;
};

static struct clk_pll msm_plls[] = {
	{"mpll", A9_MPLL_ID},
	{"gpll", A9_GPLL_ID},
	{"bpll0", A9_BPLL_0_ID},
};

static int clock_debug_set(void *data, u64 val)
{
	struct clk *clock = data;
	int ret;

	ret = clk_set_rate(clock, val);
	if (ret != 0)
		printk(KERN_ERR "clk_set_rate failed (%d)\n", ret);
	return ret;
}

static int clock_debug_get(void *data, u64 *val)
{
	struct clk *clock = data;
	*val = clk_get_rate(clock);
	return 0;
}

static int clock_enable_set(void *data, u64 val)
{
	struct clk *clock = data;
	int rc = 0;

	if (val) {
		rc = pc_clk_enable(clock->id);
	} else {
		pc_clk_disable(clock->id);
	}
	return rc;
}

static int clock_enable_get(void *data, u64 *val)
{
	struct clk *clock = data;
	*val = pc_clk_is_enabled(clock->id);
	return 0;
}

static int pll_debug_get(void *data, u64 *val)
{
	int rc;
	struct clk_pll *pll = data;
	struct msm_rpc_endpoint *endpoint;
	struct pll_set_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} clk_req;
	struct pll_set_rep {
		struct rpc_reply_hdr hdr;
		uint32_t value;
	} clk_rep;

	endpoint = msm_rpc_connect(HTC_A2M_PLL_RPOG, HTC_A2M_PLL_VERS, 0);
	if (IS_ERR(endpoint)) {
		printk(KERN_ERR "%s: init rpc failed! rc = %ld\n",
				__func__, PTR_ERR(endpoint));
		return -1;
	}
	clk_req.data = cpu_to_be32(pll->id);
	rc = msm_rpc_call_reply(endpoint, HTC_ONCRPC_GET_PLL_STATE, &clk_req,
					sizeof(clk_req), &clk_rep,
					sizeof(clk_rep), 5 * HZ);
	if (rc < 0) {
		printk(KERN_ERR "GET_PLL_STATE failed %d.\n", rc);
		return rc;
	}
	/* pll status: off, reset, warmup, bypass, on */
	*val = be32_to_cpu(clk_rep.value);
	return 0;
}

static int pll_debug_set(void *data, u64 val)
{
	return -ENOSYS;
}

DEFINE_SIMPLE_ATTRIBUTE(clock_rate_fops, clock_debug_get, clock_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(clock_enable_fops, clock_enable_get, clock_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(pll_debug_fops, pll_debug_get, pll_debug_set, "%llu\n");

static int __init clock_debug_init(void)
{
	struct dentry *dent_rate;
	struct dentry *dent_enable;
	struct clk *clock;
	struct clk_pll *pll;
	unsigned n = 0;

	dent_rate = debugfs_create_dir("clk_rate", 0);
	if (IS_ERR(dent_rate))
		return PTR_ERR(dent_rate);

	dent_enable = debugfs_create_dir("clk_enable", 0);
	if (IS_ERR(dent_enable))
		return PTR_ERR(dent_enable);

	for (clock = msm_clocks; clock && clock->name; clock++) {
		debugfs_create_file(clock->name, 0644, dent_rate,
				    clock, &clock_rate_fops);
		debugfs_create_file(clock->name, 0644, dent_enable,
				    clock, &clock_enable_fops);
	}

	for (n = 0; n < ARRAY_SIZE(msm_plls); ++n) {
		pll = msm_plls + n;
		debugfs_create_file(pll->name, S_IFREG | S_IRUGO,
						dent_enable, (void *)pll, &pll_debug_fops);
	}


	return 0;
}

device_initcall(clock_debug_init);
#endif /* CONFIG_DEBUG_FS */

