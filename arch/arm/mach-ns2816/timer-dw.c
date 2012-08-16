/*
 *  linux/arch/arm/mach-ns2816/timer-dw.c
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/amba/bus.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <asm/clkdev.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/hardware/arm_timer.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>

#include <asm/hardware/gic.h>

#include <mach/platform.h>
#include <mach/irqs.h>
#include <mach/board-ns2816.h>

#include <asm/sched_clock.h>
#include "core.h"

/*
 * How long is the timer interval?
 */
#define TIMER_INTERVAL	(TICKS_PER_uSEC * mSEC_10)
#define TIMER_RELOAD	(TIMER_INTERVAL)
#define TIMER_DIVISOR	1	// (TIMER_CTRL_DIV1)
#define TICKS2USECS(x)	((x) / TICKS_PER_uSEC)


static DEFINE_CLOCK_DATA(cd);
void __iomem *_timer0_va_base;
void __iomem *_timer3_va_base;

struct _timer_context_
{
	unsigned int timer_x_ctl;
	unsigned int timer_x_load;
};

struct _timer_context_ timer_context[2];
static int _oneshot_gate = 0;
#define	TIMERX_LOAD		(0x00)  //
#define	TIMERX_VALUE		(0x04)  //
#define	TIMERX_CTRL		(0x08)  //(0: enable/diable 1: mode 2: interupt mask
					//mode: 0 free-runing, 1: user-defined use load register
#define TIMERX_EOI		(0x0c)  //clr TIMERX active interrupt
#define	TIMERX_INT_STAT		(0x10)  //TIMERX interrupt status
#define	TIMERS_INT_STAT		(0xa0)  //all interrupt status ?
#define	TIMERS_EOI		(0xa4)	//clr all active interrupts
#define	TIMER_RAWINT_STAT	(0xa8)  //all interrupt status (contain the unmasked interrupt)

#define	TIMERX_ENABLE		(1 << 0)
#define	TIMERX_MODE_PERIODIC	(1 << 1)
#define	TIMERX_MASK_INT		(1 << 2)

static void timerx_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *clk)
{
	unsigned long ctrl;

	switch(mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel(TIMER_RELOAD, _timer0_va_base + TIMERX_LOAD);
		ctrl = TIMERX_ENABLE | TIMERX_MODE_PERIODIC;
		_oneshot_gate = 0;
		printk("timer mode periodic!!!!!!!!!!!!!!!!!!!!!!\n");
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/*snopsys timer not support mode*/
		ctrl = TIMERX_ENABLE | TIMERX_MODE_PERIODIC;
		_oneshot_gate = 1;
		printk("timer mode oneshot!!!!!!!!!!!!!!!!!!!!!!\n");
		break;
	case CLOCK_EVT_MODE_RESUME:
		ctrl = readl(_timer0_va_base + TIMERX_CTRL);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		_oneshot_gate = 0;
		ctrl = 0;
	}

	writel(ctrl, _timer0_va_base + TIMERX_CTRL);
	
}

static int timerx_set_next_event(unsigned long evt,
				struct clock_event_device *unused)
{
	unsigned long ctrl = readl(_timer0_va_base + TIMERX_CTRL);
	ctrl &= ~TIMERX_ENABLE;
	writel(ctrl, _timer0_va_base + TIMERX_CTRL);
	writel(evt, _timer0_va_base + TIMERX_LOAD);
	writel(ctrl | TIMERX_ENABLE, _timer0_va_base + TIMERX_CTRL);

	return 0;
}

static struct clock_event_device _timer0_clockevent =	 {
	.name		= "timer0",
	.shift		= 20,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= timerx_set_mode,
	.set_next_event	= timerx_set_next_event,
	.rating		= 300,
	.cpumask	= cpu_all_mask,
};

static void __init ns2816_clockevents_init(unsigned int timer_irq)
{
	_timer0_clockevent.irq = timer_irq;
	_timer0_clockevent.mult =
		//div_sc(66000000, NSEC_PER_SEC, _timer0_clockevent.shift);
		div_sc(66666666, NSEC_PER_SEC, _timer0_clockevent.shift);
	_timer0_clockevent.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &_timer0_clockevent);
	_timer0_clockevent.min_delta_ns =
		clockevent_delta2ns(0xf, &_timer0_clockevent);

	printk(KERN_INFO "ns2816_clockevents_init, timer0, mult = %x\n",
		_timer0_clockevent.mult);
	clockevents_register_device(&_timer0_clockevent);
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t ns2816_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &_timer0_clockevent;
	unsigned int ctrl;

	/* clear the interrupt */
	readl(_timer0_va_base + TIMERX_EOI);
	readl(_timer0_va_base + TIMERS_EOI);
#if 1
	if(_oneshot_gate) {
		ctrl = readl(_timer0_va_base + TIMERX_CTRL);
		ctrl &= ~TIMERX_ENABLE;
//		printk("oneshot clear ctrl %d\n",ctrl);
		writel(ctrl, _timer0_va_base + TIMERX_CTRL);
//		_oneshot_gate = 0; should not clear,clear on mode chage.
	}
	
#endif
	evt->event_handler(evt);

	return IRQ_HANDLED;
}


static struct irqaction ns2816_timer_irq = {
	.name		= "NS2816 Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= ns2816_timer_interrupt,
	.dev_id		= &_timer0_clockevent,
};

static cycle_t ns2816_get_cycles(struct clocksource *cs)
{
	return ~readl(_timer3_va_base + TIMERX_VALUE);
}

void ns2816_timer_resume(struct clocksource *cs)
{
	printk("timer %s resume\n", cs->name);
	writel(timer_context[0].timer_x_load, TIMERX_LOAD + _timer0_va_base);
	writel(timer_context[1].timer_x_load, TIMERX_LOAD + _timer3_va_base);
	writel(timer_context[0].timer_x_ctl, TIMERX_CTRL + _timer0_va_base);
	writel(timer_context[1].timer_x_ctl, TIMERX_CTRL + _timer3_va_base);
}

void ns2816_timer_suspend(struct clocksource *cs)
{
	printk("timer %s suspend\n", cs->name);
	timer_context[0].timer_x_load = readl(TIMERX_LOAD + _timer0_va_base);
	timer_context[0].timer_x_ctl = readl(TIMERX_CTRL + _timer0_va_base);
	timer_context[1].timer_x_load = readl(TIMERX_LOAD + _timer3_va_base);
	timer_context[1].timer_x_ctl = readl(TIMERX_CTRL + _timer3_va_base);
}

static struct clocksource clocksource_ns2816 = {
	.name	= "NS2816 timer3",
	.rating	= 200,
	.read	= ns2816_get_cycles,
	.mask	= CLOCKSOURCE_MASK(32),
	.shift	= 20,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend = ns2816_timer_suspend,
	.resume	 = ns2816_timer_resume,
};

static void notrace ns2816_update_sched_clock(void)
{
	u32 cyc;

	cyc = ns2816_get_cycles(NULL);

	update_sched_clock(&cd, cyc, (u32)~0);
}

static void __init ns2816_clocksource_init(void)
{
	/* setup timer 0 as free-running clocksource */
	writel(0, _timer3_va_base + TIMERX_CTRL);
	writel(0xffffffff, _timer3_va_base + TIMERX_LOAD);
	writel(3, _timer3_va_base + TIMERX_CTRL);

	init_sched_clock(&cd, ns2816_update_sched_clock, 32, 66666666);
	clocksource_register_hz(&clocksource_ns2816, 66666666);

}

void __init ns2816_timer_init(void)
{
	//select timer's clk

	_timer0_va_base = __io_address(NS2816_TIMER_BASE);
	_timer3_va_base = _timer0_va_base + 0x14 * 3;

	writel(0, TIMERX_CTRL + _timer0_va_base);
	writel(0, TIMERX_CTRL + _timer3_va_base);

	setup_irq(IRQ_NS2816_TIMERS_INTR0, &ns2816_timer_irq);
	ns2816_clocksource_init();
	ns2816_clockevents_init(IRQ_NS2816_TIMERS_INTR0);
}


