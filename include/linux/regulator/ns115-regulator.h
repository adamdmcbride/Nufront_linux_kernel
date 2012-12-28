/*
 * linux/regulator/ns115-regulator.h
 *
 * Interface for regulator driver for internal power switch of ns115 SOC.
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
 *
 */

#ifndef __LINUX_REGULATOR_NS115_H
#define __LINUX_REGULATOR_NS115_H

#include <linux/regulator/machine.h>

enum ns115_regulator_id {
	NS115_PWR_MALI_GP,
	NS115_PWR_MALI_L2C,
	NS115_PWR_MALI_PP0,
	NS115_PWR_GC300,
	NS115_PWR_VPU_G1,
	NS115_PWR_VPU_H1,
	NS115_PWR_ISP,
	NS115_PWR_ZSP,
	NS115_PWR_PLL0,
	NS115_PWR_PLL1,
	NS115_PWR_PLL2,
	NS115_PWR_PLL3,
	NS115_PWR_PLL4,
	NS115_PWR_PLL5,
	NS115_PWR_PLL6,
};

struct ns115_regulator_platform_data {
	int id;
	struct regulator_init_data regulator;
	unsigned init_enable:1;
	unsigned init_apply:1;
};

#endif
