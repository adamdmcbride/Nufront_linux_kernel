/*
 * arch/arm/mach-ns2416/power.h
 *
 * Declarations for power state transition code
 *
 * Copyright (c) 2010, Nufront Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __MACH_NS2416_POWER_H__
#define __MACH_NS2416_POWER_H__

struct ns2416_suspend_platform_data {
	unsigned long cpu_timer;   		/* CPU power good time in us,  S2-STR/S3-STR/S4-STD */
	unsigned long cpu_off_timer;    /* CPU power off time us, S2-STR/S3-STR/S4-STD */
//	unsigned long core_timer;  		/* core power good time in ticks,  S5-PWDN */
//	unsigned long core_off_timer;   /* core power off time ticks, S5-PWDN */
	unsigned long wake_enb;    		/* mask of enabled wake pads */
	unsigned long wake_high;   		/* high-level-triggered wake pads */
	unsigned long wake_low;    		/* low-level-triggered wake pads */
	unsigned long wake_any;    		/* any-edge-triggered wake pads */
//	bool dram_suspend;         		/* platform supports DRAM self-refresh */
//	bool core_off;             		/* platform supports core voltage off */
//	bool corereq_high;         		/* Core power request active-high */
//	bool sysclkreq_high;       		/* System clock request is active-high */
//	bool separate_req;         		/* Core & CPU power request are separate */
};

extern void ns2416_system_power_off(void);
extern void ns2416_system_restart(void);
extern void ns2416_system_idle(void);
extern void ns2416_system_working(void);

#endif //__MACH_NS2416_POWER_H__
