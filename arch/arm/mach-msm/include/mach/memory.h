/* arch/arm/mach-msm/include/mach/memory.h
 *
 * Copyright (C) 2007 Google, Inc.
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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* physical offset of RAM */
#ifdef CONFIG_ARCH_MSM7225
#define PHYS_OFFSET		UL(0x1B400000)
#elif defined(CONFIG_ARCH_MSM7200A)
#define PHYS_OFFSET		UL(0x19200000)
#elif defined(CONFIG_ARCH_MSM7501A)
#define PHYS_OFFSET		UL(0x19200000)
#elif defined(CONFIG_ARCH_MSM7201A)
#define PHYS_OFFSET		UL(0x19200000)
#else
#define PHYS_OFFSET		UL(0x10000000)
#endif

/* bus address and physical addresses are identical */
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)

#define HAS_ARCH_IO_REMAP_PFN_RANGE

#endif

