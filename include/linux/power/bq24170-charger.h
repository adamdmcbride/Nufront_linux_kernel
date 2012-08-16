/*
 * the header file of charger driver for bq24170 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef BQ24170_CHARGER_H_
#define BQ24170_CHARGER_H_

struct bq24170_charger_platform_data {
	int stat_gpio;
	int ac_chg_current;
};

#endif
