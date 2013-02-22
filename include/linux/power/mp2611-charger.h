/*
 * the header file of charger driver for mp2611
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef MP2611_CHARGER_H_
#define MP2611_CHARGER_H_

struct mp2611_charger_platform_data {
	int stat1_gpio;
	int stat2_gpio;
	int ac_chg_current;
	int usb_chg_current;
};

#endif
