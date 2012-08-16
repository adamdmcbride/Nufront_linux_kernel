/*
 *  linux/arch/arm/mach-ns2816/clock.c
 *
 *  Copyright (C) 2010 NUFRONT Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/cnt32_to_63.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/clkdev.h>
#include <asm/delay.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/board-ns2816.h>

#include "clock.h"
#include "prcm.h"

static DEFINE_SPINLOCK(clocks_lock);

int clk_enable(struct clk *clk)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->enabled++ == 0)
		ret = clk->enable(clk);
	spin_unlock_irqrestore(&clocks_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;
	WARN_ON(clk->enabled == 0);

	spin_lock_irqsave(&clocks_lock, flags);
	if (--clk->enabled == 0)
		clk->disable(clk);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	if (IS_ERR(clk))
		return 0;

	if (clk->rate != 0)
		return clk->rate;

	if (clk->get_rate != NULL)
		return (clk->get_rate)(clk);

	return 0;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	return rate;
}

EXPORT_SYMBOL(clk_round_rate);


int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = 0;
	if (IS_ERR(clk))
		return -EINVAL;

	WARN_ON(clk->set_rate == NULL);

	if (clk->set_rate == NULL)
        	return -EINVAL;

	spin_lock(&clocks_lock);
       	ret = (clk->set_rate)(clk, rate, 0);
	if(ret >= 0)
		clk->rate = rate;
       	spin_unlock(&clocks_lock);

	if (clk->delay)
		udelay(clk->delay);

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

struct clk *clk_get_parent(struct clk *clk)
{
        return NULL;
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
         return -EINVAL;
}
EXPORT_SYMBOL(clk_set_parent);


void clks_register(struct clk_lookup *clks, size_t num)
{
	int i;

	for (i = 0; i < num; i++)
		clkdev_add(&clks[i]);
}

