/*
 * NS115 Power Management Routines
 *
 * Copyright (C) 2010 Nufront Corporation
 *
 * Based on pm.c for ns115
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/irq.h>

#include <asm/hardware/gic.h>
#include <asm/localtimer.h>
#include <asm/tlbflush.h>
#include <mach/board-ns115.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/soc_power_ctrl.h>
#include <asm/hardware/gic.h>
#include <asm/pgalloc.h>
#include <asm/mmu_context.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include "power.h"
#include "prcm.h"
#include "scm.h"
#include "pm_firmware.h"
#include "cpu-ns115.h"

void *ns115_context_area = NULL;

extern void __cortex_a9_save(int mode);
extern unsigned int __cortex_a9_save_size;
extern unsigned int __tear_down_master_size;
extern void __ns115_lp_reset(void);
extern void __ns115_resume(void);
extern void __tear_down_master(void);
extern void __shut_off_mmu(void);
extern void __ns115_iram_entry(void);
extern unsigned int __ns115_iram_entry_size;

extern void ns115_secondary_startup(void);

extern void io373x_suspend_system(void);

extern void ns115_timer_resume(void);
extern void ns115_timer_suspend(void);
unsigned int ns115_pgd_phys;
static pgd_t *ns115_pgd;

#ifdef CONFIG_PM
static suspend_state_t suspend_state;
 
#define MAX_IRQ_LINE    (NR_IRQS_NS115>>5)
#define DDR_REG_RESTORE_NUM	11
#define DDR_REG_RESTORE_DEBUG	0

struct ns115_intc_ctx {
	u32 reg_dist_config[MAX_IRQ_LINE * 2];
	u32 reg_dist_target[MAX_IRQ_LINE * 8];
	u32 reg_dist_priority[MAX_IRQ_LINE * 8];
	u32 reg_dist_enable_set[MAX_IRQ_LINE];
	u32 reg_dist_ctrl;
	u32 reg_interf_mask;
	u32 reg_interf_ctrl;
};
static struct ns115_intc_ctx intc_ctx;


static int create_suspend_pgtable(void)
{
        int i;
        pmd_t *pmd;
        /* arrays of virtual-to-physical mappings which must be
         * present to safely boot hotplugged / LP2-idled CPUs.
         * ns115_hotplug_startup (hotplug reset vector) is mapped
         * VA=PA so that the translation post-MMU is the same as
         * pre-MMU, IRAM is mapped VA=PA so that SDRAM self-refresh
         * can safely disable the MMU */
        unsigned long addr_v[] = {
		NS115_SARRAM_BASE,
                PHYS_OFFSET,
                (unsigned long)ns115_context_area,
              	/*(unsigned long)virt_to_phys(ns115_hotplug_startup),*/
                (unsigned long)__cortex_a9_save,
                (unsigned long)virt_to_phys(__shut_off_mmu),
		0xc0100000,
		0xc0200000,
		0xc0300000,
        };
        unsigned long addr_p[] = {
		NS115_SARRAM_BASE,
                PHYS_OFFSET,
                (unsigned long)virt_to_phys(ns115_context_area),
               /*(unsigned long)virt_to_phys(ns115_hotplug_startup),*/
                (unsigned long)virt_to_phys(__cortex_a9_save),
                (unsigned long)virt_to_phys(__shut_off_mmu),
		0x80100000,
		0x80200000,
		0x80300000,
        };
        unsigned int flags = PMD_TYPE_SECT | PMD_SECT_AP_WRITE |
                PMD_SECT_WBWA | PMD_SECT_S;
	ns115_pgd = pgd_alloc(&init_mm);
        if (!ns115_pgd)
                return -ENOMEM;

        for (i=0; i<ARRAY_SIZE(addr_p); i++) {
                unsigned long v = addr_v[i];
#if 0
                pmd = pmd_offset(ns2816_pgd + pgd_index(v), v);
                *pmd = __pmd((addr_p[i] & PGDIR_MASK) | flags);
#endif
                pmd = pmd_offset((unsigned int *)(ns115_pgd) + (v>>20), v);
                *pmd = __pmd((addr_p[i] & (~((1<<20)-1))/*PGDIR_MASK*/) | flags);

                printk(KERN_EMERG"%d vaddr:0x%08lx,paddr:0x%08lx (0x%p, 0x%08x)\n",i,\
                                addr_v[i],addr_p[i], (unsigned int *)pmd, *((unsigned int *)pmd));
                flush_pmd_entry(pmd);
                outer_clean_range(__pa(pmd), __pa(pmd + 1));
        }

        ns115_pgd_phys = virt_to_phys(ns115_pgd);
        __cpuc_flush_dcache_area(&ns115_pgd_phys,
                sizeof(ns115_pgd_phys));
        outer_clean_range(__pa(&ns115_pgd_phys),
                __pa(&ns115_pgd_phys+1));

        __cpuc_flush_dcache_area(&ns115_context_area,
                sizeof(ns115_context_area));
        outer_clean_range(__pa(&ns115_context_area),
                __pa(&ns115_context_area+1));

	printk(KERN_EMERG "create suspend pgd: ns115_pgd_phys = %x\n", ns115_pgd_phys);

        return 0;
}

static int ns115_pm_prepare(void)
{
	disable_hlt();
	printk(KERN_EMERG "%s.\n", __func__);
	return 0;
}

static void ns115_irq_suspend(void)
{
	unsigned long max_irq;
	int i;
	void __iomem *dist_base = __io_address(NS115_GIC_DIST_BASE);
	void __iomem *cpu_base = __io_address(NS115_GIC_CPU_BASE);

	i = (readl(dist_base + GIC_DIST_CTR) & 0x1f) + 1;
	if (i > MAX_IRQ_LINE)
		i = MAX_IRQ_LINE;
	max_irq = i * 32; 

	for (i = 0; i < max_irq; i += 16)
		intc_ctx.reg_dist_config[i/16] = readl(dist_base + GIC_DIST_CONFIG + i * 4 / 16);

	for (i = 0; i < max_irq; i += 4) {
		intc_ctx.reg_dist_target[i/4] = readl(dist_base + GIC_DIST_TARGET + i * 4 / 4);
		intc_ctx.reg_dist_priority[i/4] = readl(dist_base + GIC_DIST_PRI + i * 4 / 4);
	}
	for (i = 0; i < max_irq; i += 32) {
		intc_ctx.reg_dist_enable_set[i/32] = readl(dist_base + GIC_DIST_ENABLE_SET + i * 4 / 32);
	}
	intc_ctx.reg_dist_ctrl = readl(dist_base + GIC_DIST_CTRL);
	intc_ctx.reg_interf_mask = readl(cpu_base + GIC_CPU_PRIMASK);
	intc_ctx.reg_interf_ctrl = readl(cpu_base + GIC_CPU_CTRL);

	writel(0, dist_base + GIC_DIST_CTRL);
	writel(0, cpu_base + GIC_CPU_CTRL);

}

static void ns115_irq_resume(void)
{
	unsigned long max_irq;
	int i;
	void __iomem *dist_base = __io_address(NS115_GIC_DIST_BASE);
	void __iomem *cpu_base = __io_address(NS115_GIC_CPU_BASE);

	i = (readl(dist_base + GIC_DIST_CTR) & 0x1f) + 1;
	if (i > MAX_IRQ_LINE)
		i = MAX_IRQ_LINE;
	max_irq = i * 32; 

	for (i = 0; i < max_irq; i += 16)
		writel(intc_ctx.reg_dist_config[i/16], dist_base + GIC_DIST_CONFIG + i * 4 / 16);

	for (i = 0; i < max_irq; i += 4) {
		writel(intc_ctx.reg_dist_target[i/4], dist_base + GIC_DIST_TARGET + i * 4 / 4); 
		writel(intc_ctx.reg_dist_priority[i/4], dist_base + GIC_DIST_PRI + i * 4 / 4); 
	}
	for (i = 0; i < max_irq; i += 32) {
		writel(0xffffffff, dist_base + GIC_DIST_ENABLE_CLEAR + i * 4 / 32);
		writel(intc_ctx.reg_dist_enable_set[i/32], dist_base + GIC_DIST_ENABLE_SET + i * 4 / 32);
	}
	writel(intc_ctx.reg_interf_mask, cpu_base + GIC_CPU_PRIMASK);

	writel(intc_ctx.reg_dist_ctrl, dist_base + GIC_DIST_CTRL);
	writel(intc_ctx.reg_interf_ctrl, cpu_base + GIC_CPU_CTRL);

}

void mem_max_save(unsigned int *addr, unsigned int *ddr_reg){
	unsigned int i = 0;
	for (i = 0; i < DDR_REG_RESTORE_NUM; i++) {
		ddr_reg[i] = readl(__io_address(addr[i]));
#if DDR_REG_RESTORE_DEBUG
		printk(KERN_DEBUG "save addr value %d\n", *(volatile unsigned *)(__io_address(addr[i])));
#endif

	}
}

void mem_max_restore(unsigned int *addr, unsigned int *ddr_reg){
	unsigned int i = 0;
	for (i = 0; i < DDR_REG_RESTORE_NUM; i++) {
		writel(ddr_reg[i], __io_address(addr[i]));
#if DDR_REG_RESTORE_DEBUG
		printk(KERN_DEBUG "restore addr value %d\n", *(volatile unsigned *)(__io_address(addr[i])));
#endif
	}
}

#define MISC0_OFFSET 0x20
#define MISC1_OFFSET 0x24
#define MISC2_OFFSET 0x28
#define MISC3_OFFSET 0x2c

static int ns115_pm_suspend(void)
{
	int ret = 0;
	unsigned long flags;
	void __iomem *entry_base = __io_address(RESUME_ENTRY);
	void __iomem *stat = __io_address(SYS_STAT);
	unsigned int ddr_reg[DDR_REG_RESTORE_NUM] = {0,0,0,0,0,0,0,0,0,0,0};
	unsigned int ddr_reg_list[DDR_REG_RESTORE_NUM] = {
		0x08008000,
		0x08008020,
		0x08008024,
		0x08008030,
		0x08008034,
		0x08008040,
		0x08008044,
		0x08008050,
		0x08008054,
		0x08008060,
		0x08008064,
	};
	//void __iomem *prcm_base = __io_address(NS115_PRCM_BASE);

	printk(KERN_EMERG "ns115_pm_suspend start ...\n");
	printk(KERN_EMERG "ns115_resume = %x, ns115_pgd_phys = %x\n", virt_to_phys(__ns115_resume), ns115_pgd_phys);
	/*this code will not overwrite code for lp2 restore.*/

	local_irq_save(flags);
	
	__raw_writel(suspend_state, stat);
	__raw_writel(virt_to_phys(__ns115_lp_reset), entry_base);
	
	if (suspend_state == PM_SUSPEND_MEM) {
		mem_max_save(ddr_reg_list, ddr_reg);
		ns115_irq_suspend();
	}

	wmb();

	flush_cache_all();
#ifdef CONFIG_OUTER_CACHE
	outer_shutdown();
#endif

	__cortex_a9_save(suspend_state);

#ifdef CONFIG_OUTER_CACHE
	outer_restart();
#endif
	printk(KERN_EMERG "back from __cortex_a9_save().\n");

	ret = 0;

	if (suspend_state == PM_SUSPEND_MEM) {
		udelay(100); /*maybe wait cpu1 to shutdown status*/
		ns115_irq_resume();
		mem_max_restore(ddr_reg_list, ddr_reg);
		printk(KERN_EMERG "ns115_irq_resume() finished.\n");
	}
	local_irq_restore(flags);

	printk(KERN_EMERG "exit ns115_pm_suspend.\n");

	return ret;
}

unsigned int ns115_suspend_lp2(unsigned int us);

static int ns115_pm_standby(void)
{
	local_irq_disable();
//	ns115_suspend_lp2(0);
	__asm("wfi");
	printk(KERN_EMERG "standby wakeup.\n");

	local_irq_enable();
	printk(KERN_EMERG "%s.\n", __func__);
	return 0;
}

static int ns115_pm_enter(suspend_state_t unused)
{
	int ret = 0;

	switch (suspend_state) {
		case PM_SUSPEND_STANDBY:
			ret = ns115_pm_standby();
			break;
		case PM_SUSPEND_MEM:
			ret = ns115_pm_suspend();
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}

static void ns115_pm_finish(void)
{
	enable_hlt();
	printk(KERN_EMERG "ns115_pm_finish.\n");
}

/* Hooks to enable / disable UART interrupts during suspend */
static int ns115_pm_begin(suspend_state_t state)
{
	suspend_state = state;
	printk(KERN_EMERG "%s.\n", __func__);
	return 0;
}

static void ns115_pm_end(void)
{
	suspend_state = PM_SUSPEND_ON;
	printk(KERN_EMERG "%s.\n", __func__);
	return;
}

static int ns115_pm_valid_state(suspend_state_t state)
{
	printk(KERN_EMERG "%s.%d\n", __func__, state);
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;

		default:
			return 0;
	}
}

static struct platform_suspend_ops ns115_pm_ops = {
	.begin      = ns115_pm_begin,
	.end        = ns115_pm_end,
	.prepare    = ns115_pm_prepare,
	.enter      = ns115_pm_enter,
	.finish     = ns115_pm_finish,
	.valid      = ns115_pm_valid_state,
};
#endif // CONFIG_SUSPEND

#define TIMER_CLK	66
#define US_DIV 		1
#define LP_TIMER	NS115_TIMER_BASE+0x14
#define WK_TIMER	NS115_TIMER_BASE+0x28
#define WK_IRQ		IRQ_NS115_TIMERS_INTR2
#if 0
void set_lp2_timer(unsigned int us)
{
	unsigned int load = us*TIMER_CLK/US_DIV;
	/*set wakeup timer*/	
	__raw_writel(load, IO_ADDRESS(WK_TIMER) + 0x0);
	__raw_writel(0x3, IO_ADDRESS(WK_TIMER) + 0x8);
	__raw_writel(0xffffffff, IO_ADDRESS(LP_TIMER) + 0x0);
	__raw_writel(0x5, IO_ADDRESS(LP_TIMER) + 0x8);/*mask;free run;enable*/
}
unsigned int restore_lp2_timer(void)
{
	unsigned int cur = __raw_readl(IO_ADDRESS(LP_TIMER) + 0x4);
	__raw_writel(0x0, IO_ADDRESS(LP_TIMER) + 0x8);/*disable*/
	cur = 0xffffffff - cur;
	cur = cur*US_DIV/TIMER_CLK;
	__raw_writel(0, IO_ADDRESS(WK_TIMER) + 0x8);
	__raw_readl(IO_ADDRESS(WK_TIMER) + 0xc);

	return cur;
}


#endif
#if 0
/*us ==0 for standby*/
unsigned int  ns115_suspend_lp2(unsigned int us)
{
	//unsigned int mode, entry, exit;
	unsigned int reg, lp2timelast;
	int ret = 0;
	unsigned long flags;
	void __iomem *scm_base = __io_address(NS115_SCM_BASE);
	//void __iomem *gic_cpu_base = __io_address(NS115_GIC_CPU_BASE);
	int ltimer_ie = 0;
	
//	printk("suspend lp2 %u\n",us);
	local_irq_save(flags);

	__raw_writel(virt_to_phys(__ns115_lp_reset), (scm_base + SCM_WFI_ENTRY_REG_OFFSET));
#ifdef CONFIG_SMP
	//__raw_writel(virt_to_phys(ns115_secondary_startup), (scm_base + SCM_WFI_ENTRY_REG_OFFSET));
#endif
	//ltimer_ie = local_timer_save();
	printk("suspend lp2 %u\n",us);

	flush_cache_all();

	reg = __raw_readl((IO_ADDRESS(NS115_PRCM_BASE)+0x14));
	//reg |= (3<<12);
	//reg |= (2<<12);
	__raw_writel(reg, (IO_ADDRESS(NS115_PRCM_BASE)+0x14));
	reg = __raw_readl((IO_ADDRESS(NS115_SCU_BASE)+0x8));
	//reg |= 0x3;
	//reg |= (0x3<<8);
	__raw_writel(reg, (IO_ADDRESS(NS115_SCU_BASE)+0x8));

	
	set_lp2_timer(us);
	barrier();
	__cortex_a9_save(0x2);
	barrier();

	lp2timelast = restore_lp2_timer();
	reg = __raw_readl((IO_ADDRESS(NS115_SCU_BASE)+0x8));
	//reg &= ~(0x3<<8);
	__raw_writel(reg, (IO_ADDRESS(NS115_SCU_BASE)+0x8));
//	printk("x lp2last %d\n",lp2timelast);
//	__raw_writel(PM_SUSPEND_ON, (scm_base + SCM_NS2416_SYS_STATE_OFFSET));
//	__asm("b .");

	ret = lp2timelast;
	if(ltimer_ie != 0)
	;//	local_timer_restore();
	local_irq_restore(flags);

//	printk("ret %d\n",ret);
	return ret;
}
#endif
/*
static irqreturn_t l2_timer_irq(int irq, void *dev_id)
{
	printk("timer_irq\n");
	return IRQ_HANDLED;
}
*/

static unsigned int lp2_allowed = 0;
bool ns115_lp2_allowed(void)
{
	return lp2_allowed&&(system_state == SYSTEM_RUNNING);
}

static int __init ns115_pm_init(void)
{
	printk("Power Management for NS115.\n");
	ns115_context_area = kzalloc(CONTEXT_SIZE_WORDS*sizeof(u32), GFP_KERNEL);

	if (!ns115_context_area)
		return 0;

#ifdef CONFIG_PM
	if(create_suspend_pgtable())
		return 0;
	
	suspend_set_ops(&ns115_pm_ops);
#endif /* CONFIG_SUSPEND */
	printk("Power Management for NS115.finished...............\n");
	lp2_allowed = 1;
#ifdef CONFIG_AUTO_HOTPLUG
	ns115_auto_hotplug_enable(true);
#endif
	return 0;
}

late_initcall(ns115_pm_init);
