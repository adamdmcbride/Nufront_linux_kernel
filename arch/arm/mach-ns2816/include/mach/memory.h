/*
 *  arch/arm/mach-ns2816/include/mach/memory.h
 *
 *  Copyright (C) 2010 NUFRONT Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
#ifdef CONFIG_NS2816_HIGH_PHYS_OFFSET
#define PHYS_OFFSET		UL(0x80000000)
#else
#define PHYS_OFFSET		UL(0x00000000)
#endif

/*
#ifdef CONFIG_MACH_NS2816TB
#include <mach/ns2816_tb_memory.h>
#endif

#ifdef CONFIG_MACH_NS2816_NTNB
#include <mach/ns2816_ntnb_memory.h>
#endif

#ifdef CONFIG_MACH_NS2816_NTPAD
#include <mach/ns2816_ntpad_memory.h>
#endif
*/

#if !defined(__ASSEMBLY__) && defined(CONFIG_ZONE_DMA)
extern void ns2816_adjust_zones(int node, unsigned long *size,
				  unsigned long *hole);
#define arch_adjust_zones(node, size, hole) \
	ns2816_adjust_zones(node, size, hole)

#define MAX_DMA_ADDRESS		(PAGE_OFFSET + SZ_256M)
#endif

#define CONSISTENT_DMA_SIZE SZ_4M

#endif
