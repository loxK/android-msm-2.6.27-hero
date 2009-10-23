/* drivers/misc/ls_alg.c
 *
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/hrtimer.h>
#include <linux/ls_alg.h>

#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif

static struct class *ls_alg_class;

#define to_ls_alg(w, m) container_of(w, struct ls_alg_classdev, m);

enum {
	LS_CONTINUE = (1U << 0),
	LS_SET_DIMMING = (1U << 1),
};

static ssize_t
alg_cdev_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	return i;
}

static ssize_t
alg_cdev_store(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	return count;
}

#define LS_ATTR(name) __ATTR(name, 0644, alg_cdev_show, alg_cdev_store)

/* todo: pass from cabc driver */
#define LEVEL_3 34

static struct device_attribute alg_attrs[] = {
	LS_ATTR(level_up),
	LS_ATTR(level_down),
};

static void alg_set_dimming(struct ls_alg_classdev *alg_cdev)
{
	unsigned long flags;

	spin_lock_irqsave(&alg_cdev->lock, flags);
	if (alg_cdev->masks & LS_SET_DIMMING) {
		spin_unlock_irqrestore(&alg_cdev->lock, flags);
		return;
	}
	alg_cdev->masks |= LS_SET_DIMMING;
	spin_unlock_irqrestore(&alg_cdev->lock, flags);

	alg_cdev->dimming_set(alg_cdev);
}

static void alg_update_work(struct work_struct *work)
{
	unsigned long flags, restart_time = 750000000;
	int tmp;
	struct ls_alg_classdev *alg_cdev = to_ls_alg(work, update_work);

	printk(KERN_INFO "%s: Desire(%d %d), Current(%d %d)\n", __func__,
			alg_cdev->desire_level + 1, alg_cdev->desire_brightness,
			alg_cdev->level + 1, alg_cdev->brightness);

	spin_lock_irqsave(&alg_cdev->lock, flags);
	tmp = alg_cdev->brightness;
	if (tmp > 85) {
		tmp -= 25;
	} else {
		if ((alg_cdev->dimming_set) &&
		    !(alg_cdev->masks & LS_SET_DIMMING)) {
			spin_unlock_irqrestore(&alg_cdev->lock, flags);

			alg_set_dimming(alg_cdev);
			spin_lock_irqsave(&alg_cdev->lock, flags);
		}
		tmp -= 16;
		restart_time = 250000000; /* 0.25s */
	}

	if (tmp < 0 || tmp <= alg_cdev->desire_brightness) {
		alg_cdev->brightness = alg_cdev->desire_brightness;
		alg_cdev->level = alg_cdev->desire_level;
		goto end;
	}

	if (tmp <= LEVEL_3) {
		alg_cdev->brightness = LEVEL_3;
		alg_cdev->level = 2;
		goto end;
	}
	alg_cdev->brightness = tmp;
	spin_unlock_irqrestore(&alg_cdev->lock, flags);

	alg_cdev->update(alg_cdev, alg_cdev->brightness, 0);
	hrtimer_start(&alg_cdev->update_timer,
			ktime_set(0, restart_time),
			HRTIMER_MODE_REL);
	return;
end:
	alg_cdev->masks &= ~LS_SET_DIMMING;
	spin_unlock_irqrestore(&alg_cdev->lock, flags);
	alg_cdev->update(alg_cdev, alg_cdev->brightness, 0);
}

static enum hrtimer_restart
alg_timer_func(struct hrtimer *timer)
{
	unsigned long flags;
	struct ls_alg_classdev *alg_cdev = to_ls_alg(timer, update_timer);

	spin_lock_irqsave(&alg_cdev->lock, flags);
	if (alg_cdev->masks & LS_CONTINUE) {
		spin_unlock_irqrestore(&alg_cdev->lock, flags);
		schedule_work(&alg_cdev->update_work);
	} else {
		alg_cdev->masks &= ~LS_SET_DIMMING;
		spin_unlock_irqrestore(&alg_cdev->lock, flags);
	}
	return HRTIMER_NORESTART;
}

static void
ls_alg_update(struct ls_alg_classdev *alg_cdev, int level, int br)
{
	unsigned long flags;

	printk(KERN_INFO "%s: Desire(%d), Current(%d)\n",
			__func__, br, alg_cdev->brightness);

	spin_lock_irqsave(&alg_cdev->lock, flags);
	if (alg_cdev->brightness < br) {
		alg_cdev->brightness = br;
		alg_cdev->level = level;
		spin_unlock_irqrestore(&alg_cdev->lock, flags);

		alg_cdev->update(alg_cdev, br, 1);
	} else {
		alg_cdev->masks |= LS_CONTINUE;
		spin_unlock_irqrestore(&alg_cdev->lock, flags);

		if (!hrtimer_active(&alg_cdev->update_timer)) {
			hrtimer_start(&alg_cdev->update_timer,
				      ktime_set(0, NSEC_PER_SEC/60),
				      HRTIMER_MODE_REL);
		}
	}
}

static inline int alg_should_update(struct ls_alg_classdev *alg_cdev)
{
	int update = 0;
	unsigned long flags;
	
	spin_lock_irqsave(&alg_cdev->lock, flags);
	if (alg_cdev->desire_brightness > alg_cdev->brightness) {
		update = 1;
		goto end;
	}

	if (alg_cdev->desire_level <= (alg_cdev->level -
				alg_cdev->level_down)) {
		if (alg_cdev->level == 2)
			goto end;	
		update = 1;
	}
end:	
	spin_unlock_irqrestore(&alg_cdev->lock, flags);
	return update;
}

static void alg_process_work(struct work_struct *work)
{
	int curr_br, curr_lvl;
	int desire_br, desire_lvl;
	unsigned long flags;
	struct ls_alg_classdev *alg_cdev = to_ls_alg(work, process_work);

	B(KERN_DEBUG "%s: enter.\n", __func__);
	spin_lock_irqsave(&alg_cdev->lock, flags);
	curr_lvl = alg_cdev->level;
	curr_br = alg_cdev->brightness;
	desire_br = alg_cdev->desire_brightness;
	desire_lvl = alg_cdev->desire_level;
	spin_unlock_irqrestore(&alg_cdev->lock, flags);

	if (curr_br == 0)
		ls_alg_update(alg_cdev, desire_lvl, desire_br);
	else if (alg_should_update(alg_cdev))
		ls_alg_update(alg_cdev, desire_lvl, desire_br);
	else {
		printk(KERN_DEBUG "%s: Do not proceed, curr (%d - %d), "
				"desire (%d - %d)\n", __func__, curr_br,
				curr_lvl, desire_br, desire_lvl);
	}
}

int ls_alg_process(struct ls_alg_classdev *alg_cdev, int level, int br)
{
	int curr_br;
	unsigned long flags;

	B(KERN_DEBUG "%s: enter.\n", __func__);

	spin_lock_irqsave(&alg_cdev->lock, flags);
	curr_br = alg_cdev->brightness;
	alg_cdev->masks &= ~LS_CONTINUE;
	spin_unlock_irqrestore(&alg_cdev->lock, flags);

	if (curr_br != br) {
		alg_cdev->desire_brightness = br;
		alg_cdev->desire_level = level;
		schedule_work(&alg_cdev->process_work);
	}
	return 0;
}

int ls_alg_create(struct device *parent, struct ls_alg_classdev *alg_cdev)
{
	int i, rc = 0;

	B(KERN_DEBUG "%s: enter\n", __func__);

	alg_cdev->dev = device_create_drvdata(ls_alg_class, parent, 0, alg_cdev,
					      "%s", alg_cdev->name);
	if (IS_ERR(alg_cdev->dev)) {
		rc = PTR_ERR(alg_cdev->dev);
		goto dev_create_failed;
	}

	for (i = 0; i < ARRAY_SIZE(alg_attrs); i++) {
		rc = device_create_file(alg_cdev->dev,
					&alg_attrs[i]);
		if (rc)
			goto err_out;
	}

	alg_cdev->level_up = 1;
	alg_cdev->level_down = 2;
	hrtimer_init(&alg_cdev->update_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	alg_cdev->update_timer.function = alg_timer_func;
	INIT_WORK(&alg_cdev->update_work, alg_update_work);
	INIT_WORK(&alg_cdev->process_work, alg_process_work);
	spin_lock_init(&alg_cdev->lock);
	return 0;

err_out:
	while (i--)
		device_remove_file(alg_cdev->dev, &alg_attrs[i]);
dev_create_failed:
	return rc;
}

static int __init ls_alg_init(void)
{
	ls_alg_class = class_create(THIS_MODULE, "ls_alg");
	if (IS_ERR(ls_alg_class))
		return PTR_ERR(ls_alg_class);
	return 0;
}

static void __exit ls_alg_exit(void)
{
	class_destroy(ls_alg_class);
}

subsys_initcall(ls_alg_init);
module_exit(ls_alg_exit);

MODULE_AUTHOR("Jay Tu");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Light sensor Algorithm driver");
