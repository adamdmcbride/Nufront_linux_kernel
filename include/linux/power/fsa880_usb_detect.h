/*
 * include/linux/power/fsa880_usb_detect.h
 *
 * Core driver implementation to access RICOH583 power management chip.
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

#ifndef FSA880_USB_DETECT_H_
#define FSA880_USB_DETECT_H_

#include <linux/power/ns115-battery.h>

void fsa880_int_func(void);
int fsa880_get_chg_type(enum ns115_charger_type * type);

#endif
