/*
 * I2C definitions for NUSMART
 *
 * Copyright (C) 2012 Nufront Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __NUSMART_I2C_H
#define __NUSMART_I2C_H

#include <linux/types.h>

/* 
	This macro is used to set touch screen transit mode. If this macro is opend, 
	touch panel works in fast mode with clock 400KHZ, otherwise touch panel 
	works in standard mode with clock 100KHZ.
*/
#define TOUCHSCREEN_I2C_BUS_FAST_MODE

struct nusmart_i2c_platform_data
{
	enum {I2C_SPEED_STD = 1, I2C_SPEED_FAST}speed;
	char* clk_id;
};

#endif
