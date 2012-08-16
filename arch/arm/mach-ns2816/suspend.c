/*
 * NS2816 Power Management Routines
 *
 * Copyright (C) 2010 Nufront Corporation
 *
 * Based on pm.c for ns2816
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
#include <mach/board-ns2816.h>
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
#include "pm.h"
#include "scm.h"

#define IO_IRAM_PHYS 	(0x00000000)

void *ns2816_context_area = NULL;

static void __iomem *iram_code = IO_ADDRESS(NS2816_IRAM_CODE_AREA);
extern void __cortex_a9_save(int mode);
extern unsigned int __cortex_a9_save_size;
extern unsigned int __tear_down_master_size;
extern void __ns2816_lp_reset(void);
extern void __ns2816_resume(void);
extern void __tear_down_master(void);
extern void __shut_off_mmu(void);
extern void __ns2816_iram_entry(void);
extern unsigned int __ns2816_iram_entry_size;

extern void ns2816_secondary_startup(void);

extern void io373x_suspend_system(void);

extern void ns2816_timer_resume(void);
extern void ns2816_timer_suspend(void);
unsigned int ns2816_pgd_phys;
static pgd_t *ns2816_pgd;

extern int ns2816_iram_init(void);

#define DEBUG
#ifdef DEBUG
#define dbg(args...)  
#else
#define dbg(args...) printk(args)
#endif

#ifdef CONFIG_SUSPEND
static suspend_state_t suspend_state;
 
#define MAX_IRQ_LINE    (NR_IRQS_NS2816>>5)

struct ns2816_intc_ctx {
	u32 reg_dist_config[MAX_IRQ_LINE * 2];
	u32 reg_dist_target[MAX_IRQ_LINE * 8];
	u32 reg_dist_priority[MAX_IRQ_LINE * 8];
	u32 reg_dist_enable_set[MAX_IRQ_LINE];
	u32 reg_dist_ctrl;
	u32 reg_interf_mask;
	u32 reg_interf_ctrl;
};
static struct ns2816_intc_ctx intc_ctx;


static int create_suspend_pgtable(void)
{
        int i;
        pmd_t *pmd;
        /* arrays of virtual-to-physical mappings which must be
         * present to safely boot hotplugged / LP2-idled CPUs.
         * ns2816_hotplug_startup (hotplug reset vector) is mapped             
         * VA=PA so that the translation post-MMU is the same as              
         * pre-MMU, IRAM is mapped VA=PA so that SDRAM self-refresh           
         * can safely disable the MMU */                                      
        unsigned long addr_v[] = {                                            
                PHYS_OFFSET,                                                  
                IO_IRAM_PHYS,                                                 
                (unsigned long)ns2816_context_area,                            
              // (unsigned long)virt_to_phys(ns2816_hotplug_startup),           
                (unsigned long)__cortex_a9_save,                           
                (unsigned long)virt_to_phys(__shut_off_mmu),
		0xc0100000,
		0xc0200000,
		0xc0300000,                  
        };                                                                    
        unsigned long addr_p[] = {                                            
                PHYS_OFFSET,                                                  
                IO_IRAM_PHYS,                                                 
                (unsigned long)virt_to_phys(ns2816_context_area),              
               // (unsigned long)virt_to_phys(ns2816_hotplug_startup),           
                (unsigned long)virt_to_phys(__cortex_a9_save),             
                (unsigned long)virt_to_phys(__shut_off_mmu),                  
		0x80100000,
		0x80200000,
		0x80300000,
        };                                                                    
        unsigned int flags = PMD_TYPE_SECT | PMD_SECT_AP_WRITE |              
                PMD_SECT_WBWA | PMD_SECT_S;   
  	ns2816_pgd = pgd_alloc(&init_mm);
        if (!ns2816_pgd)
                return -ENOMEM;

        for (i=0; i<ARRAY_SIZE(addr_p); i++) {
                unsigned long v = addr_v[i];
                pmd = pmd_offset(ns2816_pgd + pgd_index(v), v);
                *pmd = __pmd((addr_p[i] & PGDIR_MASK) | flags);
                flush_pmd_entry(pmd);
                outer_clean_range(__pa(pmd), __pa(pmd + 1));
        }

        ns2816_pgd_phys = virt_to_phys(ns2816_pgd);
        __cpuc_flush_dcache_area(&ns2816_pgd_phys,
                sizeof(ns2816_pgd_phys));
        outer_clean_range(__pa(&ns2816_pgd_phys),
                __pa(&ns2816_pgd_phys+1));

        __cpuc_flush_dcache_area(&ns2816_context_area,
                sizeof(ns2816_context_area));
        outer_clean_range(__pa(&ns2816_context_area),
                __pa(&ns2816_context_area+1));

	printk(KERN_EMERG "create suspend pgd: ns2816_pgd_phys = %x\n", ns2816_pgd_phys);

        return 0;
}

static int ns2816_pm_prepare(void)
{
	disable_hlt();
	dbg(KERN_EMERG "%s.\n", __func__);
	return 0;
}

static void ns2816_irq_suspend(void)
{
	unsigned long max_irq;
	int i;
	void __iomem *dist_base = __io_address(NS2816_GIC_DIST_BASE);
	void __iomem *cpu_base = __io_address(NS2816_GIC_CPU_BASE);

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

static void ns2816_irq_resume(void)
{
	unsigned long max_irq;
	int i;
	void __iomem *dist_base = __io_address(NS2816_GIC_DIST_BASE);
	void __iomem *cpu_base = __io_address(NS2816_GIC_CPU_BASE);

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

static int ns2816_pm_suspend(void)
{
	int ret = 0;
	unsigned long flags;
	void __iomem *scm_base = __io_address(NS2816_SCM_BASE);
	unsigned int scm_pinmux_config0 = 1;
	unsigned int scm_pinmux_config1 = 0;
	unsigned int ddr_config = 0x200;

	io373x_suspend_system();

	printk(KERN_EMERG "ns2816_pm_suspend start ...\n");
	printk(KERN_EMERG "ns2816_resume = %x, ns2816_pgd_phys = %x\n", virt_to_phys(__ns2816_resume), ns2816_pgd_phys);
	/*this code will not overwrite code for lp2 restore.*/
	memcpy(iram_code, __tear_down_master, __tear_down_master_size);

	local_irq_save(flags);
	__raw_writel(virt_to_phys(__ns2816_lp_reset), (scm_base + SCM_WFIENT_RSENT_OFFSET));
#ifdef CONFIG_SMP
	__raw_writel(virt_to_phys(__ns2816_lp_reset), (scm_base + SCM_WFI_ENTRY_REG_OFFSET));
#endif
	
	if (suspend_state == PM_SUSPEND_MEM) {
		ns2816_irq_suspend();
	}

	wmb();

	flush_cache_all();
#ifdef CONFIG_OUTER_CACHE
	outer_shutdown();
#endif
	scm_pinmux_config0 = readl(scm_base + 0x4c);
	scm_pinmux_config1 = readl(scm_base + 0x50);
	ddr_config = readl(__io_address(NS2816_DDR_CFG_BASE) + 0x200);

	__cortex_a9_save(suspend_state);

	printk(KERN_EMERG "1. pinmax 0x051c004c: 0x%x\n", readl(__io_address(0x051c004c)));

	writel(ddr_config, __io_address(NS2816_DDR_CFG_BASE) + 0x200);

	writel(scm_pinmux_config0, scm_base + 0x4c);
	writel(scm_pinmux_config1, scm_base + 0x50);

	printk(KERN_EMERG "2. pinmax 0x051c004c: 0x%x\n", readl(__io_address(0x051c004c)));
#ifdef CONFIG_OUTER_CACHE
	outer_restart();
#endif
	printk(KERN_EMERG "back from __cortex_a9_save().\n");

	ret = 0;

	if (suspend_state == PM_SUSPEND_MEM) {
		ns2816_iram_init();
		udelay(100); //maybe wait cpu1 to shutdown status
		ns2816_irq_resume();
		printk(KERN_EMERG "ns2816_irq_resume() finished.\n");
		__raw_writel(PM_SUSPEND_ON, (scm_base + SCM_NS2416_SYS_STATE_OFFSET));
#ifdef CONFIG_MACH_HUANGSHANS
		wifi_power(ON);
#endif
	}
	local_irq_restore(flags);

	printk(KERN_EMERG "exit ns2816_pm_suspend.\n");

	return ret;
}

unsigned int ns2816_suspend_lp2(unsigned int us);

static int ns2816_pm_standby(void)
{
	unsigned long flags;
	local_irq_disable();
//	ns2816_suspend_lp2(0);
	__asm("wfi");
	printk(KERN_EMERG "standby wakeup.\n");

	local_irq_enable();
	printk(KERN_EMERG "%s.\n", __func__);
	return 0;
}

static int ns2816_pm_enter(suspend_state_t unused)
{
	int ret = 0;

	switch (suspend_state) {
		case PM_SUSPEND_STANDBY:
			ret = ns2816_pm_standby();
			break;
		case PM_SUSPEND_MEM:
			ret = ns2816_pm_suspend();
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}

static void ns2816_pm_finish(void)
{
	enable_hlt();
	dbg(KERN_EMERG "ns2816_pm_finish.\n");
}

/* Hooks to enable / disable UART interrupts during suspend */
static int ns2816_pm_begin(suspend_state_t state)
{
	suspend_state = state;
	dbg(KERN_EMERG "%s.\n", __func__);
	return 0;
}

static void ns2816_pm_end(void)
{
	suspend_state = PM_SUSPEND_ON;
	dbg(KERN_EMERG "%s.\n", __func__);
	return;
}

static int ns2816_pm_valid_state(suspend_state_t state)
{
	dbg(KERN_EMERG "%s.%d\n", __func__, state);
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;

		default:
			return 0;
	}
}

static struct platform_suspend_ops ns2816_pm_ops = {
	.begin      = ns2816_pm_begin,
	.end        = ns2816_pm_end,
	.prepare    = ns2816_pm_prepare,
	.enter      = ns2816_pm_enter,
	.finish     = ns2816_pm_finish,
	.valid      = ns2816_pm_valid_state,
};
#endif // CONFIG_SUSPEND

#define TIMER_CLK	66
#define US_DIV 		1
#define LP_TIMER	NS2816_TIMER_BASE+0x14
#define WK_TIMER	NS2816_TIMER_BASE+0x28
#define WK_IRQ		IRQ_NS2816_TIMERS_INTR2
#if 1
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
/*us ==0 for standby*/
unsigned int  ns2816_suspend_lp2(unsigned int us)
{
	//unsigned int mode, entry, exit;
	unsigned int reg, lp2timelast;
	int ret = 0;
	unsigned long flags;
	void __iomem *scm_base = __io_address(NS2816_SCM_BASE);
	//void __iomem *gic_cpu_base = __io_address(NS2816_GIC_CPU_BASE);
	int ltimer_ie = 0;
	
//	printk("suspend lp2 %u\n",us);
	local_irq_save(flags);

	__raw_writel(virt_to_phys(__ns2816_lp_reset), (scm_base + SCM_WFI_ENTRY_REG_OFFSET));
#ifdef CONFIG_SMP
	//__raw_writel(virt_to_phys(ns2816_secondary_startup), (scm_base + SCM_WFI_ENTRY_REG_OFFSET));
#endif
	//ltimer_ie = local_timer_save();
	printk("suspend lp2 %u\n",us);

	flush_cache_all();

	reg = __raw_readl((IO_ADDRESS(NS2816_PRCM_BASE)+0x14));	
	//reg |= (3<<12);
	//reg |= (2<<12);
	__raw_writel(reg, (IO_ADDRESS(NS2816_PRCM_BASE)+0x14));	
	reg = __raw_readl((IO_ADDRESS(NS2816_SCU_BASE)+0x8));
	//reg |= 0x3;
	//reg |= (0x3<<8);
	__raw_writel(reg, (IO_ADDRESS(NS2816_SCU_BASE)+0x8));

	
	set_lp2_timer(us);
	barrier();
	__cortex_a9_save(0x2);
	barrier();

	lp2timelast = restore_lp2_timer();
	reg = __raw_readl((IO_ADDRESS(NS2816_SCU_BASE)+0x8));
	//reg &= ~(0x3<<8);
	__raw_writel(reg, (IO_ADDRESS(NS2816_SCU_BASE)+0x8));
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
extern int __init ns2816_idle_init(void);
int ns2816_iram_init(void)
{
	/*called by pm_init and resume from suspend to ram*/
	void  __iomem * addr = __io_address(NS2816_IRAM_BASE);
	void  __iomem * rap = __io_address(NS2816_RAP_BASE);
	writel(1, rap+8);
	dsb();
	memcpy(addr, __ns2816_iram_entry, __ns2816_iram_entry_size);
	mb();
	
	return 0;
};
/*
static irqreturn_t l2_timer_irq(int irq, void *dev_id)
{
	printk("timer_irq\n");
	return IRQ_HANDLED;
}
*/

static unsigned int lp2_allowed = 0;
bool ns2816_lp2_allowed(void)
{
	return lp2_allowed&&(system_state == SYSTEM_RUNNING);
}

static int __init ns2816_pm_init(void)
{
	//int irq = 32;
	ns2816_iram_init();
	printk("Power Management for NS2816.\n");
	ns2816_context_area = kzalloc(CONTEXT_SIZE_WORDS*sizeof(u32), GFP_KERNEL);

	//if(request_irq(IRQ_NS2816_TIMERS_INTR2, l2_timer_irq, 0/*IRQF_DISABLED*/,"timer2",l2_timer_irq)<0)
	//	printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!timer irq failed.\n");

	//for(irq; irq < 155; irq++) {
	//	irq_set_affinity(irq,cpumask_of(0));
	//}
	
	//irq_set_affinity(IRQ_NS2816_TIMERS_INTR2,cpumask_of(1));
	

	if (!ns2816_context_area)
		return 0;

#ifdef CONFIG_SUSPEND
	if(create_suspend_pgtable())
		return 0;
	
	suspend_set_ops(&ns2816_pm_ops);
#endif /* CONFIG_SUSPEND */
	printk("Power Management for NS2816.finished...............\n");
	lp2_allowed = 1;
	return 0;
}

late_initcall(ns2816_pm_init);
