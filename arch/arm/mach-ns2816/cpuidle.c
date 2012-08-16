/*
 * arch/arm/mach-ns2816/cpuidle.c
 *
 * CPU idle driver for NS2816 CPUs
 *
 * Copyright (c) 2010, NUFRONT Corporation.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/hrtimer.h>
#include <linux/cpu.h>
#include <linux/io.h>
#include <linux/tick.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>

struct cpuidle_driver ns2816_idle = {
	.name = "ns2816_idle",
	.owner = THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device *, idle_devices);

#ifdef CONFIG_SUSPEND
//#define CPU_POWER_GATE
#endif

#ifdef CPU_POWER_GATE

#define LATENCY_FACTOR_SHIFT 8

static unsigned int latency_factor __read_mostly = 80;	// factor ~ 0.3
static unsigned int pwrgood_latency = 2000;
static unsigned int system_is_suspending = 0;
module_param(latency_factor, uint, 0644);


static int lp2_supported = 0;

struct ns2816_suspend_platform_data
{
	unsigned int cpu_timer;
};

extern unsigned int ns2816_lp2_allowed(void);
extern unsigned int ns2816_suspend_lp2(unsigned int);
static int ns2816_idle_enter_wfi(struct cpuidle_device *dev,
	struct cpuidle_state *state);

static int ns2816_idle_enter_lp2(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t enter;
	s64 request, us, latency, idle_us;
	struct tick_sched *ts = tick_get_tick_sched(dev->cpu);
	unsigned int last_sample = (unsigned int)cpuidle_get_statedata(state);

	/* LP2 not possible when running in SMP mode */
	smp_rmb();
	idle_us = state->exit_latency + state->target_residency;
//	printk("idle_us %lld, state->exit_latency %d, state->target_residency %d\n", idle_us, state->exit_latency, state->target_residency);
	request = ktime_to_us(tick_nohz_get_sleep_length());
	printk("before idle_us %lld,request %lld\n", idle_us, request);

	if (!lp2_supported || request <= idle_us || (!ts->tick_stopped) ||
		system_is_suspending || (!ns2816_lp2_allowed())) {
		dev->last_state = &dev->states[0];
		return ns2816_idle_enter_wfi(dev, &dev->states[0]);
	}

	local_irq_disable();
	enter = ktime_get();
	request -= state->exit_latency;
	us = ns2816_suspend_lp2((unsigned int)max_t(s64, 200, request));
	idle_us = ktime_to_us(ktime_sub(ktime_get(), enter));

	latency = pwrgood_latency + idle_us - us;
	printk("after idle_us %lld,request %lld, us %lld ,latency %lld\n", idle_us, request, us, latency);
	cpuidle_set_statedata(state, (void*)(unsigned int)(latency));
	state->exit_latency = (12*latency + 4*last_sample) >> 4;
	state->target_residency = (latency_factor*state->exit_latency) >>
		LATENCY_FACTOR_SHIFT;

	/* adjust kernel timers */
	hrtimer_peek_ahead_timers();

	local_irq_enable();
	return (int)idle_us;
}

static int ns2816_idle_lp2_allowed(struct notifier_block *nfb,
	unsigned long action, void *hcpu)
{
	switch (action) {
	case CPU_UP_PREPARE:
		lp2_supported = 0;
		smp_wmb();
		break;
	case CPU_UP_CANCELED:
	case CPU_POST_DEAD:
		lp2_supported = (num_online_cpus()==1);
		smp_wmb();
		break;
	}

	return NOTIFY_OK;
}

static int ns2816_cpuidle_notifier(struct notifier_block *nfb,
	unsigned long event, void *data)
{
	int notification = NOTIFY_OK;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		system_is_suspending = 1;
		smp_wmb();
		break;
	case PM_POST_SUSPEND:
		system_is_suspending = 0;
		smp_wmb();
		break;
	default:
		printk(KERN_ERR "%s: unknown event %ld\n", __func__, event);
		notification = NOTIFY_DONE;
	}

	return notification;
}

#endif

static int ns2816_idle_enter_wfi(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t enter, exit;
	s64 us;

	local_irq_disable();

	smp_rmb();

	enter = ktime_get();
	if (!need_resched()) {
		stop_critical_timings();
		dsb();
		__asm__ volatile ("wfi");
		start_critical_timings();
	}
	exit = ktime_get();
	enter = ktime_sub(exit, enter);
	us = ktime_to_us(enter);
	local_irq_enable();
	//printk("%d idle time = %lld us\n", dev->cpu, us);
	return (int)us;
}


static int ns2816_idle_enter(unsigned int cpu)
{
	struct cpuidle_device *dev;
	struct cpuidle_state *state;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->state_count = 0;
	dev->cpu = cpu;

	state = &dev->states[0];
	snprintf(state->name, CPUIDLE_NAME_LEN, "LP_WFI");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU wfi");
	state->exit_latency = 10;
	state->target_residency = 0;
	state->power_usage = 600;
	state->flags = CPUIDLE_FLAG_SHALLOW | CPUIDLE_FLAG_TIME_VALID;
	state->enter = ns2816_idle_enter_wfi;
	dev->safe_state = state;
	dev->state_count++;
#ifdef CPU_POWER_GATE
	if (cpu == 0) {
		state = &dev->states[1];
		snprintf(state->name, CPUIDLE_NAME_LEN, "LP2");
		snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU power-gate");
		state->exit_latency = 2500;

		state->target_residency = (state->exit_latency *
			latency_factor) >> LATENCY_FACTOR_SHIFT;
		state->power_usage = 10;
		state->flags = CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_TIME_VALID;
		state->enter = ns2816_idle_enter_lp2;
		cpuidle_set_statedata(state, (void*)state->exit_latency);

		dev->safe_state = state;
		dev->state_count++;
	}
#endif
	if (cpuidle_register_device(dev)) {
		pr_err("CPU%u: failed to register idle device\n", cpu);
		kfree(dev);
		return -EIO;
	}
	per_cpu(idle_devices, cpu) = dev;
	return 0;
}

static int __init ns2816_cpuidle_init(void)
{
	unsigned int cpu = smp_processor_id();
	int ret;


#ifdef CPU_POWER_GATE
	lp2_supported = (num_online_cpus()==1);
	hotcpu_notifier(ns2816_idle_lp2_allowed, 0);
	pm_notifier(ns2816_cpuidle_notifier, 0);
#endif

	ret = cpuidle_register_driver(&ns2816_idle);

	printk("ns2816_cpuidle_init ret = %08X\n", ret);

	if (ret)
		return ret;

	for_each_possible_cpu(cpu) {
		if (ns2816_idle_enter(cpu))
			pr_err("CPU%u: error initializing idle loop\n", cpu);
	}


	return 0;
}

static void __exit ns2816_cpuidle_exit(void)
{
	cpuidle_unregister_driver(&ns2816_idle);
}


module_init(ns2816_cpuidle_init);
module_exit(ns2816_cpuidle_exit);
