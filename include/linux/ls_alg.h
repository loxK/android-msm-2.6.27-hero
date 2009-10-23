/* include/linux/ls_alg.h
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
#ifndef __LS_ALGORITHM_H_
#define __LS_ALGORITHM_H_

struct ls_alg_classdev {
	const char *name;
	int brightness;
	int level;
	int desire_brightness;
	int desire_level;
	struct device *dev;
	int (*update)(struct ls_alg_classdev *, int brightness, int increase);
	int (*dimming_set)(struct ls_alg_classdev *);
	/* private */
	int level_up;
	int level_down;
	uint32_t masks;
	spinlock_t lock;
	struct hrtimer update_timer;
	struct work_struct update_work;
	struct work_struct process_work;
};

#ifdef CONFIG_LIGHTSENSOR_ALG
extern int ls_alg_create(struct device *, struct ls_alg_classdev *);
extern int ls_alg_process(struct ls_alg_classdev *, int level, int brightness);
#else
static int
ls_alg_create(struct device *dev, struct ls_alg_classdev *ls_cdev)
{
	return 0;
}

static int
ls_alg_process(struct ls_alg_classdev *ls_cdev, int level, int brightness)
{
	ls_cdev->brightness = brightness;
	ls_cdev->update(ls_cdev, brightness);
	return 0;
}
#endif
#endif
