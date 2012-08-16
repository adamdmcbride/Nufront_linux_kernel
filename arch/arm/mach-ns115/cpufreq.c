/*
 * arch/arm/mach-ns115/cpufreq.c
 *
 * Copyright (C) 2011 NUFRONT, Inc.
 *
 * Author:
 *	zeyuan <zeyuan.xu@nufront.com>
 *	Based on Colin Cross <ccross@google.com>
 *	Based on arch/arm/plat-omap/cpu-omap.c, (C) 2005 Nokia Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*#define DEBUG*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/suspend.h>

#include <asm/system.h>

#include <mach/hardware.h>

#include <linux/regulator/consumer.h>

#include "cpu-ns115.h"

#define CLK_NAME		"ns115_cpu"
#define REGU_NAME		"vdd_cpu"
#define DEFAULT_VOL_FREQ	800000
/* Frequency table index must be sequential starting at 0 */
static struct cpufreq_frequency_table freq_table[] = {
	{ 0, 400000 },
	{ 1, 800000 },
#ifdef CONFIG_CHIP_NORMAL
	{ 2, 1000000 },
	{ 3, CPUFREQ_TABLE_END },
#elif defined CONFIG_CHIP_HIGH
	{ 2, 1000000 },
	{ 3, 1200000 },
	{ 4, CPUFREQ_TABLE_END },
#else /*default set to max to 800MHz*/
	{ 2, CPUFREQ_TABLE_END },
#endif
/*
	{ 0, 200000 },
	{ 1, 300000 },
	{ 2, 400000 },
	{ 3, 500000 },
	{ 4, 600000 },
	{ 5, 700000 },
	{ 6, 800000 },
	{ 7, 900000 },
	{ 8, 1000000 },
	{ 9, CPUFREQ_TABLE_END },
*/
};

#define NUM_CPUS	2

static struct clk *cpu_clk;
static struct regulator	*cpu_regu;

static unsigned long target_cpu_speed[NUM_CPUS];
static DEFINE_MUTEX(ns115_cpu_lock);
static bool is_suspended;
static int cpu_vol;

struct fvs
{
	unsigned long 	rate;	/*KHz*/
	int		vol;	/*mV*/
};

const static struct fvs fvs_table[] =
{
/*
	{200000, 1040},
	{300000, 1040},
	{400000, 1040},
	{500000, 1070},
	{600000, 1070},
*/
	{200000, 1100},
	{300000, 1100},
	{400000, 1100},
	{500000, 1100},
	{600000, 1100},
	{700000, 1100},
	{800000, 1100},
	{900000, 1220},
	{1000000, 1220},
	{1200000, 1220},
	{1500000, 1220},
};

int ns115_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

unsigned int ns115_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= NUM_CPUS)
		return 0;

	rate = clk_get_rate(cpu_clk) / 1000;
	return rate;
}

inline int get_target_voltage(unsigned long rate)
{
	int idx = 0;
	for(idx = 0; idx < ARRAY_SIZE(fvs_table); idx++) {
		if(fvs_table[idx].rate == rate)
			return fvs_table[idx].vol;
	}
	WARN_ON(1);
	return fvs_table[ARRAY_SIZE(fvs_table)-1].vol;
}

static int ns115_update_cpu_speed(unsigned long rate)
{
	int ret = 0;
	int cpu_vol_new = get_target_voltage(rate);
	struct cpufreq_freqs freqs;

	freqs.old = ns115_getspeed(0);
	freqs.new = rate;

	if (freqs.old == freqs.new)
		return ret;

	/*
	 * Vote on memory bus frequency based on cpu frequency
	 * This sets the minimum frequency, display or avp may request higher
	 */

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

#ifdef DEBUG
	printk(KERN_DEBUG "cpufreq-ns115: transition: %u --> %u\n",
			freqs.old, freqs.new);
#endif
	if((cpu_vol_new == cpu_vol) || (NULL == cpu_regu)) {
		/*
		 * only update clock when there is no regulator
		 * or there is no need to update cpu voltage.
		 */
		ret = clk_set_rate(cpu_clk, freqs.new * 1000);
		if (ret) {
			pr_err("cpu-ns115: Failed to set cpu frequency to %d kHz\n",
					freqs.new);
			return ret;
		}
	} else if(cpu_vol_new > cpu_vol) {
		/*
		 * rise up voltage before scale up cpu frequency.
		 */
		ret = regulator_set_voltage(cpu_regu, cpu_vol_new * 1000, cpu_vol_new * 1000);
		if(ret < 0) {
			pr_err("cpu-ns115: Failed to set cpu voltage to %d mV\n",cpu_vol_new);
			return ret;
		}

		ret = clk_set_rate(cpu_clk, freqs.new * 1000);
		if (ret) {
			pr_err("cpu-ns115: Failed to set cpu frequency to %d kHz\n",
					freqs.new);
			return ret;
		}

	} else {
		/*
		 * reduce voltage after scale down cpu frequency.
		 */
		ret = clk_set_rate(cpu_clk, freqs.new * 1000);
		if (ret) {
			pr_err("cpu-ns115: Failed to set cpu frequency to %d kHz\n",
					freqs.new);
			return ret;
		}

		ret = regulator_set_voltage(cpu_regu, cpu_vol_new * 1000, cpu_vol_new * 1000);
		if(ret < 0)
			pr_err("cpu-ns115: Failed to set cpu voltage to %d mV\n",cpu_vol_new);
	}

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static unsigned long ns115_cpu_highest_speed(void)
{
	unsigned long rate = 0;
	int i;

	for_each_online_cpu(i)
		rate = max(rate, target_cpu_speed[i]);
	/*avold target rate exceed default max rate with default voltage*/
	if(cpu_regu == NULL)
		rate = min(rate, (unsigned long)DEFAULT_VOL_FREQ);
	return rate;
}

static int ns115_target(struct cpufreq_policy *policy,
		unsigned int target_freq,
		unsigned int relation)
{
	int idx;
	unsigned int freq;
	int ret = 0;
	int rate = 0;

	mutex_lock(&ns115_cpu_lock);

	if (is_suspended) {
		ret = -EBUSY;
		goto out;
	}

	cpufreq_frequency_table_target(policy, freq_table, target_freq,
			relation, &idx);

	freq = freq_table[idx].frequency;
	target_cpu_speed[policy->cpu] = freq;

	rate = ns115_cpu_highest_speed();

	ret = ns115_update_cpu_speed(rate);

#ifdef CONFIG_AUTO_HOTPLUG
	ns115_auto_hotplug_governor(rate, false);
#endif
out:
	mutex_unlock(&ns115_cpu_lock);
	return ret;
}

static int ns115_pm_notify(struct notifier_block *nb, unsigned long event,
		void *dummy)
{
	mutex_lock(&ns115_cpu_lock);
	if (event == PM_SUSPEND_PREPARE) {
		is_suspended = true;
		pr_info("ns115 cpufreq suspend: setting frequency to %d kHz\n",
				DEFAULT_VOL_FREQ);
		ns115_update_cpu_speed(DEFAULT_VOL_FREQ);

#ifdef CONFIG_AUTO_HOTPLUG
		ns115_auto_hotplug_suspend();
#endif
	} else if (event == PM_POST_SUSPEND) {
		is_suspended = false;

#ifdef CONFIG_AUTO_HOTPLUG
		ns115_auto_hotplug_resume();
#endif
	}
	mutex_unlock(&ns115_cpu_lock);

	return NOTIFY_OK;
}

static struct notifier_block ns115_cpu_pm_notifier = {
	.notifier_call = ns115_pm_notify,
};

static int ns115_cpu_init(struct cpufreq_policy *policy)
{
	if (policy->cpu >= NUM_CPUS)
		return -EINVAL;

	cpu_clk = clk_get_sys(NULL, CLK_NAME);
	if (IS_ERR(cpu_clk)) {
		pr_err("cpu freq:get clock failed.\n");
		return PTR_ERR(cpu_clk);
	}

	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);
	policy->cur = ns115_getspeed(policy->cpu);
	target_cpu_speed[policy->cpu] = policy->cur;

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 300 * 1000;

	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	if (policy->cpu == 0)
		register_pm_notifier(&ns115_cpu_pm_notifier);

	return 0;
}

static int ns115_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	clk_put(cpu_clk);
	return 0;
}

static struct freq_attr *ns115_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver ns115_cpufreq_driver = {
	.verify		= ns115_verify_speed,
	.target		= ns115_target,
	.get		= ns115_getspeed,
	.init		= ns115_cpu_init,
	.exit		= ns115_cpu_exit,
	.name		= "ns115",
	.attr		= ns115_cpufreq_attr,
};

static int __init ns115_cpufreq_init(void)
{
#ifdef CONFIG_AUTO_HOTPLUG
	ns115_auto_hotplug_init(&ns115_cpu_lock, freq_table[1].frequency, freq_table[0].frequency);
#endif
	return cpufreq_register_driver(&ns115_cpufreq_driver);
}

static void __exit ns115_cpufreq_exit(void)
{
#ifdef CONFIG_AUTO_HOTPLUG
	ns115_auto_hotplug_exit();
#endif
	cpufreq_unregister_driver(&ns115_cpufreq_driver);
}

static int __init ns115_cpu_regu_init(void)
{
	/*initialize cpu regulator after regulator ready*/
	cpu_regu = regulator_get(NULL, REGU_NAME);
	if(!IS_ERR(cpu_regu)) {
		cpu_vol = regulator_get_voltage(cpu_regu);
		pr_info("cpu freq:cpu voltage:%dmV\n", cpu_vol);
	} else {
		pr_err("cpu freq:get regulator failed.\n");
	}	
	return 0;
}
late_initcall(ns115_cpu_regu_init);

MODULE_AUTHOR("zeyuan <zeyuan.xu@nufront.com>");
MODULE_DESCRIPTION("cpufreq driver for NS115");
MODULE_LICENSE("GPL");
module_init(ns115_cpufreq_init);
module_exit(ns115_cpufreq_exit);
