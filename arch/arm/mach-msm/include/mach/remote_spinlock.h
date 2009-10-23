/** include/asm-arm/arch-msm/remote_spinlock.h
 *
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * Author: Saravana Kannan <skannan@qualcomm.com>
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 *
 */

#ifndef __ASM__ARCH_QC_REMOTE_SPINLOCK_H
#define __ASM__ARCH_QC_REMOTE_SPINLOCK_H

#include <linux/types.h>

typedef struct {
	volatile uint32_t lock;
} raw_remote_spinlock_t;

typedef raw_remote_spinlock_t *_remote_spinlock_t;

#define remote_spin_lock_id_t uint32_t

int _remote_spin_lock_init(remote_spin_lock_id_t id, _remote_spinlock_t *lock);
void _remote_spin_lock(_remote_spinlock_t *lock);
void _remote_spin_unlock(_remote_spinlock_t *lock);

#endif
