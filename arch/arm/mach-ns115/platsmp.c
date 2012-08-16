/*
 *  linux/arch/arm/mach-ns115/platsmp.c
 *
 *  Copyright (C) 2010 NUFRONT Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/localtimer.h>
#include <asm/unified.h>

#include <mach/smp.h>
#include <mach/board-ns115.h>
#include <asm/smp_scu.h>
#include <asm/hardware/gic.h>

//#include "core.h"
#include "prcm.h"
#include "scm.h"
#include "common.h"

extern void ns115_secondary_startup(void);
extern int ns115_iram_init(void);
/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
volatile int pen_release = -1;

static void __iomem *scu_base_addr(void)
{
	return __io_address(NS115_SCU_BASE);
}

static inline unsigned int get_core_count(void)
{
	void __iomem *scu_base = scu_base_addr();
	if (scu_base)
		return scu_get_core_count(scu_base);
	return 1;
}

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	void __iomem *scu_base = scu_base_addr();
	unsigned int  scu_cfg = 0;
	printk("platform_secondary_init(corb init): %d\n", cpu);

	trace_hardirqs_off();
	common_init();

	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	//gic_cpu_init(0, gic_cpu_base_addr);
	//gic_cpu_init(0, __io_address(NS115_GIC_CPU_BASE));

	gic_secondary_init(0);

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	pen_release = -1;
	smp_wmb();

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);

	if (scu_base) {
		scu_cfg = __raw_readl(scu_base + 0x04);
		printk("scm configruation = %x\n", scu_cfg);
	}
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	
	printk("--------------%s\n", __func__);
	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 * Note that "pen_release" is the hardware CPU ID, whereas
	 * "cpu" is Linux's internal ID.
	 */
	flush_cache_all();
	outer_clean_range(__pa(&secondary_data), __pa(&secondary_data + 1));
	pen_release = cpu;
	flush_cache_all();
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));

	/*
	 * XXX
	 *
	 * This is a later addition to the booting protocol: the
	 * bootMonitor now puts secondary cores into WFI, so
	 * poke_milo() no longer gets the cores moving; we need
	 * to send a soft interrupt to wake the secondary core.
	 * Use smp_cross_call() for this, since there's little
	 * point duplicating the code here
	 */
	dsb();
	smp_cross_call(cpumask_of(cpu));

//	gic_raise_softirq(cpumask_of(cpu), 1);
	timeout = jiffies + (5 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

static void __init poke_milo(void)
{
	/* nobody is to be released from the pen yet */
	pen_release = -1;

	printk("--------------%s\n", __func__);
	/*
	 * Write the address of secondary startup into the system-wide flags
	 * register. The BootMonitor waits for this register to become
	 * non-zero.
	 */
	__raw_writel(BSYM(virt_to_phys(ns115_secondary_startup)),
		     __io_address(NS115_PRCM_BASE+0x28));

	mb();
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	printk("--------------%s\n", __func__);

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = get_core_count();
	int i;
	
	printk(KERN_EMERG "ns115 %s\n", __func__);

	/* sanity check */
	if (ncores == 0) {
		printk(KERN_ERR
		       "Realview: strange CM count of 0? Default to 1\n");

		ncores = 1;
	}

	if (ncores > NR_CPUS) {
		printk(KERN_WARNING
		       "Realview: no. of cores (%d) greater than configured "
		       "maximum of %d - clipping\n",
		       ncores, NR_CPUS);
		ncores = NR_CPUS;
	}

//	smp_store_cpu_info(cpu);

	/*
	 * are we trying to boot more cores than exist?
	 */
	if (max_cpus > ncores)
		max_cpus = ncores;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	/*
	 * Initialise the SCU if there are more than one CPU and let
	 * them know where to start. Note that, on modern versions of
	 * MILO, the "poke" doesn't actually do anything until each
	 * individual core is sent a soft interrupt to get it out of
	 * WFI
	 */
	if (max_cpus > 1) {
		/*
		 * Enable the local timer or broadcast device for the
		 * boot CPU, but only if we have more than one CPU.
		 */

		printk(KERN_EMERG "ns115 2 %s\n", __func__);
		set_smp_cross_call(gic_raise_softirq);
		scu_enable(scu_base_addr());
		poke_milo();
	}
}
