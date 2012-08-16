/*
 *  linux/arch/arm/mach-ns115/clock.h
 *
 *  Copyright (C) 2004 ARM Limited.
 *  Written by Deep Blue Solutions Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _MACH_CLOCK_H_
#define _MACH_CLOCK_H_

struct clk {
        int                  enabled;
        unsigned long        rate;
	unsigned long	     delay;

        int                 (*enable)(struct clk * c);
        int                 (*disable)(struct clk * c);
        int                 (*set_rate)(struct clk *c, unsigned long rate, unsigned int port);
        unsigned long       (*get_rate)(struct clk *c);
};

#endif

