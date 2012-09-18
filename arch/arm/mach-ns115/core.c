/*
 *  linux/arch/arm/mach-ns115/core.c
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
#include <linux/cnt32_to_63.h>
#include <linux/io.h>
#include <linux/smsc911x.h>
#include <linux/ata_platform.h>
#include <linux/dw_dmac.h>
#include <linux/amba/mmci.h>
#include <linux/amba/pl330.h>
#include <linux/rfkill.h>
#include <linux/rfkill-gpio.h>
#ifdef CONFIG_MFD_RICOH583
#include <linux/mfd/ricoh583.h>
#endif

#include <linux/power/power_switch.h>

#include <asm/clkdev.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/smp_twd.h>
#include <asm/mach-types.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/time.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>

#include <asm/hardware/gic.h>

#include <mach/platform.h>
#include <mach/irqs.h>
#include <mach/mmc.h>
#include <mach/i2c.h>
#include <mach/board-ns115.h>
#include <mach/camera.h>
#include <mach/system.h>

#ifdef CONFIG_GENERIC_GPIO
#include <mach/gpio.h>
#endif

#include <asm/pmu.h>

#include <linux/serial_8250.h>
#include <linux/serial_reg.h>

#include <linux/mmc/dw_mmc.h>

#include <linux/android_pmem.h>
#include <mach/memory.h>
#include "core.h"
#include "prcm.h"
#include <mach/get_bootargs.h>
#include <linux/input.h>
#include <linux/nusmart_gpio_keys.h>
#include <asm/cacheflush.h>

/* used by entry-macro.S and platsmp.c */
//void __iomem *gic_cpu_base_addr;
void (*__ns115_reset)(char mode);

struct ns115_backlight_config {
	int default_intensity;
	int (*set_power)(struct device *dev, int state);
};

struct timed_gpio {
	const char *name;
	unsigned   gpio;
	int        max_timeout;
	u8         active_low;
};      

struct timed_gpio_platform_data {
	int num_gpios;
	struct timed_gpio *gpios;
};      

/*
 * Adjust the zones if there are restrictions for DMA access.
 */
#ifdef CONFIG_ZONE_DMA
void __init ns115_adjust_zones(int node, unsigned long *size,
		unsigned long *hole)
{
	unsigned long dma_size = SZ_256M >> PAGE_SHIFT;
#ifdef CONFIG_MACH_NS115_PAD_TEST
	if (!machine_is_ns115_pad_test() || node || (size[0] <= dma_size))
#endif
		return;

	size[ZONE_NORMAL] = size[0] - dma_size;
	size[ZONE_DMA] = dma_size;
	hole[ZONE_NORMAL] = hole[0];
	hole[ZONE_DMA] = 0;
}
#endif
/*
 *ns115 io table
 */
struct map_desc ns115_io_desc[] __initdata = {
	{
		.virtual = IO_ADDRESS(NS115_GIC_CPU_BASE),
		.pfn     = __phys_to_pfn(NS115_GIC_CPU_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_GIC_DIST_BASE),
		.pfn     = __phys_to_pfn(NS115_GIC_DIST_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_L220_BASE),
		.pfn     = __phys_to_pfn(NS115_L220_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_PRCM_BASE),
		.pfn     = __phys_to_pfn(NS115_PRCM_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_SCM_BASE),
		.pfn     = __phys_to_pfn(NS115_SCM_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_SARRAM_BASE),
		.pfn     = __phys_to_pfn(NS115_SARRAM_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_CAMERIC_BASE),
		.pfn     = __phys_to_pfn(NS115_CAMERIC_BASE),
		.length  = SZ_128K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_DISPCTR0_BASE),
		.pfn     = __phys_to_pfn(NS115_DISPCTR0_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_DISPCTR1_BASE),
		.pfn     = __phys_to_pfn(NS115_DISPCTR1_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_GPIO0_BASE),
		.pfn     = __phys_to_pfn(NS115_GPIO0_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_TIMER0_BASE),
		.pfn     = __phys_to_pfn(NS115_TIMER0_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_TIMER1_BASE),
		.pfn     = __phys_to_pfn(NS115_TIMER1_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_GPIO1_BASE),
		.pfn     = __phys_to_pfn(NS115_GPIO1_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_AUX_BASE),
		.pfn     = __phys_to_pfn(NS115_AUX_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},

	{
		.virtual = IO_ADDRESS(NS115_I2S0_BASE),
		.pfn     = __phys_to_pfn(NS115_I2S0_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_I2S1_BASE),
		.pfn     = __phys_to_pfn(NS115_I2S1_BASE),
		.length  = SZ_4K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_UART0_BASE),
		.pfn     = __phys_to_pfn(NS115_UART0_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_UART1_BASE),
		.pfn     = __phys_to_pfn(NS115_UART1_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_UART2_BASE),
		.pfn     = __phys_to_pfn(NS115_UART2_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_UART3_BASE),
		.pfn     = __phys_to_pfn(NS115_UART3_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_DMA_330S_BASE),
		.pfn     = __phys_to_pfn(NS115_DMA_330S_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_I2C0_BASE),
		.pfn     = __phys_to_pfn(NS115_I2C0_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_I2C1_BASE),
		.pfn     = __phys_to_pfn(NS115_I2C1_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
        {
                .virtual = IO_ADDRESS(NS115_HDMI_BASE),
                .pfn     = __phys_to_pfn(NS115_HDMI_BASE),
                .length  = SZ_64K,
                .type    = MT_DEVICE,
        },
/*
	{
		.virtual = IO_ADDRESS(NS115_I2C2_BASE),
		.pfn     = __phys_to_pfn(NS115_I2C2_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_I2C3_BASE),
		.pfn     = __phys_to_pfn(NS115_I2C3_BASE),
		.length  = SZ_64K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_DRAMCTRL_BASE),
		.pfn     = __phys_to_pfn(NS115_DRAMCTRL_BASE),
		.length  = SZ_32K,
		.type    = MT_DEVICE,
	},
*/
	{
		.virtual = IO_ADDRESS(NS115_MEM_MAX0_BASE),
		.pfn     = __phys_to_pfn(NS115_MEM_MAX0_BASE),
		.length  = SZ_1K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_L3_ISS_TA_BASE),
		.pfn     = __phys_to_pfn(NS115_L3_ISS_TA_BASE),
		.length  = SZ_1K,
		.type    = MT_DEVICE,
	},
	{
		.virtual = IO_ADDRESS(NS115_CAMERIC_TA_BASE),
		.pfn     = __phys_to_pfn(NS115_CAMERIC_TA_BASE),
		.length  = SZ_1K,
		.type    = MT_DEVICE,
	},
};

void __init ns115_map_io(void)
{
	iotable_init(ns115_io_desc, ARRAY_SIZE(ns115_io_desc));
}

static int ns115_set_wake(struct irq_data *d, unsigned int on)
{
	void __iomem * wake_addr = __io_address(NS115_PRCM_BASE + 0x14);
	unsigned int val = 0;

	if(d->irq < GPIO_IRQ_BASE1 && d->irq >= (GPIO_IRQ_BASE0 + INT_NUM0)) {
		val = readl(wake_addr);	
		if(on) {
			val |= (1 << (d->irq - GPIO_IRQ_BASE0));
			val |= (1 << 16);
		} else {
			val &= ~(1 << (d->irq - GPIO_IRQ_BASE0));
		}
		writel(val, wake_addr);
	} else {
		return -ENXIO;
	}
	return 0;
}

void __init gic_init_irq(void)
{
	gic_arch_extn.irq_set_wake = ns115_set_wake;
	gic_init(0, 29, __io_address(NS115_GIC_DIST_BASE),
			__io_address(NS115_GIC_CPU_BASE));
}

static void __init ns115_all_timer_init(void)
{

#ifdef CONFIG_LOCAL_TIMERS
	twd_base = __io_address(NS115_TWD_BASE);
#endif

	ns115_timer_init();
}

struct sys_timer ns115_timer = {
	.init = ns115_all_timer_init,
};

static void ns115_reset(char mode)
{
	prcm_glb_soft_reset();
}

static void ns115_power_off(void)
{
#ifdef CONFIG_MFD_RICOH583
	ricoh583_power_off();
#endif
}

/*
 *   When call cpu_v7_proc_fin, cpu can hung-up and system restart fail!!!
 *   Root cause is unknown!!! <2.6.33 not exist this bug> 
 */
static void ns115_restart(char mode, const char *cmd)
{
  	unsigned long flags;
        extern void setup_mm_for_reboot(char mode);

        local_irq_save(flags);
        __cpuc_flush_kern_all();        //== flush_cache_all();
        __cpuc_flush_user_all();

        //cpu_v7_proc_fin();

        setup_mm_for_reboot(mode);
	arch_reset(mode, cmd);

}

void arch_reset(char mode, const char *cmd)
{
#ifdef CONFIG_MFD_RICOH583
	ricoh583_restart(mode, cmd);
#else
	if (__ns115_reset)
		__ns115_reset(mode);
#endif
}

#ifdef CONFIG_CACHE_L2X0
int __init ns115_l2x0_init(void)
{
	void __iomem *l2x0_base = __io_address(NS115_L220_BASE);
	unsigned int val;

	if (!(readl(l2x0_base + L2X0_CTRL) & 1)) {
		/* set RAM latencies to 3 cycle for eASIC */
		//writel(0x777, l2x0_base + L2X0_TAG_LATENCY_CTRL);
		//writel(0x777, l2x0_base + L2X0_DATA_LATENCY_CTRL);
		writel(0x444, l2x0_base + L2X0_TAG_LATENCY_CTRL);
		writel(0x444, l2x0_base + L2X0_DATA_LATENCY_CTRL);
		//writel(0x00000003, l2x0_base + L2X0_PREFETCH_CTRL);
		writel(0x70000007, l2x0_base + L2X0_PREFETCH_CTRL);
		writel(0x00000001, l2x0_base + L2X0_ADDR_FILTER_START);
		writel(0x10000000, l2x0_base + L2X0_ADDR_FILTER_END);
	}

	/* 64KB way size, 16-way associativity, parity enable, event monitor bus enable,
	   full line of zero enable, shared attribute internally ignored
	   early BRESP enable, data prefetch & instruction prefetch enable,
	 */

	/* Those features are not clear in linux: value = 0x0c771000
	   non-secure interrupt access control, non-secure lockdown enable
	 */

	val = 0x7C460001;
	l2x0_init(l2x0_base, val, 0xc200ffff);

	return 0;
}

early_initcall(ns115_l2x0_init);
#endif


/************************************************************
 *	Bus priority-level set
 ************************************************************/
static DEFINE_SPINLOCK(prio_lock);

static inline void bus_prio_set(unsigned int offs, unsigned int val_l, unsigned int val_h)
{
	unsigned long flag;
	void * __iomem addr = __io_address(NS115_MEM_MAX0_BASE);

	spin_lock_irqsave(&prio_lock, flag);
	writel(val_l, addr + offs);
	writel(val_h, addr + offs + 4);
	spin_unlock_irqrestore(&prio_lock, flag);
}

static inline void bus_prio_clear(unsigned int offs)
{
	unsigned long flag;
	void * __iomem addr = __io_address(NS115_MEM_MAX0_BASE);

	spin_lock_irqsave(&prio_lock, flag);
	writel(0, addr + offs);
	spin_unlock_irqrestore(&prio_lock, flag);
}

#define THREAD_4_OFFSET		0x60
#define WORDS_PER_PERIOD_BIT	16

static void cam_priority_set(int flag)
{
	unsigned int val_l, val_h;
	val_l = 0x2;
	val_h = (0x100 << WORDS_PER_PERIOD_BIT) | 0x4;

	if (flag)
		bus_prio_set(THREAD_4_OFFSET, val_l, val_h);
	else
		bus_prio_clear(THREAD_4_OFFSET);
}

/*
   static struct resource pmu_resources[] = {
   [0] = {
   .start = IRQ_NS115_OSPREY_PMUIRQ,
   .end   = IRQ_NS115_OSPREY_PMUIRQ,
   .flags = IORESOURCE_IRQ,
   },
   [1] = {
   .start = IRQ_NS115_OSPREY_PMUIRQ1,
   .end   = IRQ_NS115_OSPREY_PMUIRQ1,
   .flags = IORESOURCE_IRQ,
   },
   };

   struct platform_device ns2816_pmu_device = {
   .name          = "arm-pmu",
   .id            = ARM_PMU_DEVICE_CPU,
   .num_resources = ARRAY_SIZE(pmu_resources),
   .resource      = pmu_resources,
   };


   static struct resource pmu_resources[] = {
   [0] = {
   .start = IRQ_NS115_OSPREY_PMUIRQ,
   .end   = IRQ_NS115_OSPREY_PMUIRQ,
   .flags = IORESOURCE_IRQ,
   },
   [1] = {
   .start = IRQ_NS115_OSPREY_PMUIRQ1,
   .end   = IRQ_NS115_OSPREY_PMUIRQ1,
   .flags = IORESOURCE_IRQ,
   },
   };

   struct platform_device pmu_device = {
   .name          = "arm-pmu",
   .id            = ARM_PMU_DEVICE_CPU,
   .num_resources = ARRAY_SIZE(pmu_resources),
   .resource      = pmu_resources,
   };
 */

/*##############################################################*/
/*		NS115 SD/MMC controller device			*/
/*##############################################################*/

static struct resource ns115_sdmmc_resource[] = {
	[0] = {
		.start = NS115_SDMMC_BASE,
		.end   = NS115_SDMMC_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_SDMMC_INTR,
		.end   = IRQ_NS115_SDMMC_INTR,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_sdmmc_device = {
	.name = "ns115-sdmmc",
	.id   = 0,
	.dev  = {
		.platform_data =  NULL,//&nusmart_sdmmc_data,
	},
	.num_resources = 2,
	.resource      = ns115_sdmmc_resource,
};

//by wangzhi modify 2012.3.31
struct resource ns115_hdmi_resources[] = {
	[0] = {
		.name	= "HDMI",
		.start	= NS115_HDMI_BASE,
		.end	= NS115_HDMI_BASE + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
        [1] = {
		.start = IRQ_NS115_HDMI_PCL,
		.end   = IRQ_NS115_HDMI_PCL,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_hdmi_device = {
	.name		= "ns115-soc-hdmi",
	.num_resources	= ARRAY_SIZE(ns115_hdmi_resources),
	.resource	= ns115_hdmi_resources,
	.id             = -1,
};
/*##############################################################*/
/*		NS115 I2C controller device			*/
/*##############################################################*/
static struct resource ns115_i2c0_resource[] = {
	[0] = {
		.start = NS115_I2C0_BASE,
		.end   = NS115_I2C0_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_I2C0_INTR,
		.end   = IRQ_NS115_I2C0_INTR,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource ns115_i2c1_resource[] = {
	[0] = {
		.start = NS115_I2C1_BASE,
		.end   = NS115_I2C1_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_I2C1_INTR,
		.end   = IRQ_NS115_I2C1_INTR,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource ns115_i2c2_resource[] = {
	[0] = {
		.start = NS115_I2C2_BASE,
		.end   = NS115_I2C2_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_I2C2_INTR,
		.end   = IRQ_NS115_I2C2_INTR,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource ns115_i2c3_resource[] = {
	[0] = {
		.start = NS115_I2C3_BASE,
		.end   = NS115_I2C3_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_I2C3_INTR,
		.end   = IRQ_NS115_I2C3_INTR,
		.flags = IORESOURCE_IRQ,
	},
};

static struct nusmart_i2c_platform_data nusmart_i2c_data[] = {
	{
		.speed = I2C_SPEED_STD,
		.clk_id = "ns115_i2c0", 
	},
	{
		.speed = I2C_SPEED_STD,
		.clk_id = "ns115_i2c1", 
	},
	{
		.speed = I2C_SPEED_STD,
		.clk_id = "ns115_i2c2", 
	},
#ifdef TOUCHSCREEN_I2C_BUS_FAST_MODE
	{
		.speed = I2C_SPEED_FAST,
		.clk_id = "ns115_i2c3", 
	},
#else
	{
		.speed = I2C_SPEED_STD,
		.clk_id = "ns115_i2c3", 
	},
#endif
}; 

struct platform_device ns115_i2c_device[] = {
	{
		.name          = "i2c_designware",
		.id            = I2C_BUS_0,
		.num_resources = 2,
		.resource      = ns115_i2c0_resource,
		.dev = {
			.platform_data  = &nusmart_i2c_data[0],
		},
	},
	{
		.name          = "i2c_designware",
		.id            = I2C_BUS_1,
		.num_resources = 2,
		.resource      = ns115_i2c1_resource,
		.dev = {
			.platform_data  = &nusmart_i2c_data[1],
		},
	},
	{
		.name          = "i2c_designware",
		.id            = I2C_BUS_2,
		.num_resources = 2,
		.resource      = ns115_i2c2_resource,
		.dev = {
			.platform_data  = &nusmart_i2c_data[2],
		},
	},
	{
		.name          = "i2c_designware",
		.id            = I2C_BUS_3,
		.num_resources = 2,
		.resource      = ns115_i2c3_resource,
		.dev = {
			.platform_data  = &nusmart_i2c_data[3],
		},
	},
};


/*##############################################################*/
/*		NS115 usb device controller device		*/
/*##############################################################*/
static struct resource ns115_udc_resource[] = {
	[0] = {                                                   
		.start          = NS115_USB_OTG_BASE,
		.end            = NS115_USB_OTG_BASE + SZ_64K - 1,              
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_NS115_USB_OTG,                             
		.end            = IRQ_NS115_USB_OTG,
		.flags          = IORESOURCE_IRQ,                 
	},
};                                                                

struct platform_device ns115_udc_device = {
	.name           = "design_ware_udc",
	.id             = 0,                                      
	.num_resources  = ARRAY_SIZE(ns115_udc_resource),    
	.resource       = ns115_udc_resource,                
};   

/*##############################################################*/
/*		NS115 uart controller device			*/
/*##############################################################*/
static struct plat_serial8250_port serial_platform_data[] = {
	{
		.mapbase  = NS115_UART0_BASE,
		.irq      = IRQ_NS115_UART0_INTR,
		.flags    = UPF_FIXED_TYPE | UPF_IOREMAP | UPF_SKIP_TEST,
		.type	  = PORT_16550A,
		.iotype   = UPIO_MEM32,
		.regshift = 2,
		.uartclk  = 66000000,
	},{
		.mapbase  = NS115_UART1_BASE,
		.irq      = IRQ_NS115_UART1_INTR,
		.flags    = UPF_FIXED_TYPE | UPF_IOREMAP | UPF_SKIP_TEST,
		.type     = PORT_16550A,
		.iotype   = UPIO_MEM32,
		.regshift = 2,
		.uartclk  = 66000000,
	},{
		.mapbase  = NS115_UART2_BASE,
		.irq      = IRQ_NS115_UART2_INTR,
		.flags    = UPF_FIXED_TYPE | UPF_IOREMAP | UPF_SKIP_TEST,
		.type	  = PORT_NF_16550A,
		.iotype   = UPIO_MEM32,
		.regshift = 2,
		.uartclk  = 32000000,
	},{
		.mapbase  = NS115_UART3_BASE,
		.irq      = IRQ_NS115_UART3_INTR,
		.flags    = UPF_FIXED_TYPE | UPF_IOREMAP | UPF_SKIP_TEST,
		.type	  = PORT_NF_16550A,
		.iotype   = UPIO_MEM32,
		.regshift = 2,
		.uartclk  = 66000000,
        },{
		.flags    = 0,
	},
};

struct platform_device ns115_serial_device = {
	.name = "serial8250",
	.id   = PLAT8250_DEV_PLATFORM,
	.dev  = {
		.platform_data  = serial_platform_data,
	},
};


/*##############################################################*/
/*		NS115 i2s controller device			*/
/*##############################################################*/

static struct resource ns115_i2s0_resource[] = {
	[0] = {
		.start = NS115_I2S0_BASE,
		.end   = NS115_I2S0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_I2S0_INTR,
		.end   = IRQ_NS115_I2S0_INTR,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource ns115_i2s1_resource[] = {
	[0] = {
		.start = NS115_I2S1_BASE,
		.end   = NS115_I2S1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_I2S1_INTR,
		.end   = IRQ_NS115_I2S1_INTR,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_i2s_device[] = {
	{
		.name          = "ns115-i2s",
		.id            = -1,
		.num_resources = 2,
		.resource      = ns115_i2s0_resource,
	},
	{
		.name          = "ns115-i2s",
		.id            = -1,
		.num_resources = 2,
		.resource      = ns115_i2s1_resource,
	},
};

/*
   struct platform_device ns115_i2s_plat_device = {
   .name          = "ns115-i2s-plat",
   .id            = -1,
   .num_resources = 0,
   .resource      = NULL,
   };
 */
struct platform_device ns115_i2s_plat_device = {
	.name          = "ns115-i2s-plat",
	.id            = -1,
	.num_resources = 2,
	.resource      = ns115_i2s0_resource,
};

struct platform_device ns115_pcm_device = {
	.name          = "nusmart-pcm-audio",
	.id            = -1,
	.num_resources = 0,
	.resource      = NULL,
};

/*##############################################################*/
/*		NS2816 usb controller device			*/
/*##############################################################*/

/*
 * USB ehci register define
 */

static struct resource ns115_ehci_resources[] = {
	[0] = {
		.start = NS115_USB_EHCI_BASE,
		.end   = NS115_USB_EHCI_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_USB_EHCI,
		.end   = IRQ_NS115_USB_EHCI,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_usb_ehci_device = {
	.name          = "ns2816-ehci",
	.num_resources = 2,
	.resource      = ns115_ehci_resources,
	.dev = {
		.coherent_dma_mask = ~0,
	},                      
};


/*
 * USB ohci register define
 */

static struct resource ns115_ohci_resources[] = {
	[0] = {
		.start = NS115_USB_OHCI_BASE,
		.end   = NS115_USB_OHCI_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_USB_OHCI,
		.end   = IRQ_NS115_USB_OHCI,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_usb_ohci_device = {
	.name          = "ns2816-ohci",
	.num_resources = 2,
	.resource      = ns115_ohci_resources,
	.dev = {
		.coherent_dma_mask = ~0,
	},                      
};

/*##############################################################*/
/*		NS115 lcd controller device			*/
/*##############################################################*/
#define LCDC_NAME_NULCD  "nusmartfb"
#define LCDC_NAME_NULCD1 "nusmartfb1"
static struct resource ns115fb0_resources[] = {
	[0] = {
		.start = NS115_DISPCTR0_BASE,
		.end   = NS115_DISPCTR0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = { 
		.start = IRQ_NS115_DISPCTR0_ERR, 
		.end   = IRQ_NS115_DISPCTR0_HINT,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource ns115fb1_resources[] = {
	[0] = {
		.start = NS115_DISPCTR1_BASE,
		.end   = NS115_DISPCTR1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = { 
		.start = IRQ_NS115_DISPCTR1_ERR, 
		.end   = IRQ_NS115_DISPCTR1_HINT,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_clcd_device[] = {
	{
		.name = LCDC_NAME_NULCD,
		.id = 0,
		.dev = {
			.coherent_dma_mask = DMA_BIT_MASK(32),
		},
		.resource = ns115fb0_resources,
		.num_resources =2,
	},
	{
		.name = LCDC_NAME_NULCD1,
		.id = 1,
		.dev = {
			.coherent_dma_mask = DMA_BIT_MASK(32),
		},
		.resource = ns115fb1_resources,
		.num_resources =2,
	},
};

/*##############################################################*/
/*		NS115 wakeup timer device			*/
/*##############################################################*/
#define WAKEUP_TIMER_NAME "ns115_wk_timer"
static struct resource ns115_wk_timer_resources[] = {
	[0] = {
		.start = NS115_TIMER0_BASE,
		.end   = NS115_TIMER0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = { 
		.start = IRQ_NS115_TIMER0_WAKEUP_0, 
		.end   = IRQ_NS115_TIMER0_WAKEUP_1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_wk_timer_device = {
	.name = WAKEUP_TIMER_NAME,
	.id = -1,
	.resource = ns115_wk_timer_resources,
	.num_resources = ARRAY_SIZE(ns115_wk_timer_resources),
};

/*##############################################################*/
/*             NS115 CamerIC controller device                    */
/*##############################################################*/
static struct mrv_camera_pdata camera_host_data = {
	.flags = MRV_CAM_PLATFORM_DEFAULT,
	.mclk_10khz = 2400,
	.prio_set = &cam_priority_set,
};

static struct resource ns115_camera_resources[] = {
	[0] = {
		.start = NS115_CAMERIC_BASE,
		.end   = NS115_CAMERIC_BASE + SZ_128K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_NS115_CAMERIC_ISP,
		.end   = IRQ_NS115_CAMERIC_MI,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_camera_device = {
	.name = "mrv_camera",
	.id   = 0,
	.dev  = {
		.platform_data = &camera_host_data,
	},
	.resource = ns115_camera_resources,
	.num_resources = 2,
};

/*##############################################################*/
/*             NS115 Camera sensor device                    */
/*##############################################################*/
struct platform_device gc0329_camera_device = {
	.name = "soc-camera-pdrv",
	.id   = 0,
};

struct platform_device ov5640_camera_device = {
	.name = "soc-camera-pdrv",
	.id   = 1,
};

/*m@nufront: start TODO*/
/*##############################################################*/
/*             NS115 ON2 G1v2 controller device                    */
/*##############################################################*/
static struct resource ns115_vpu_dec_resource[] = {
	[0] = {
		.start          = NS115_ON2_DEC_BASE,
		.end            = NS115_ON2_DEC_BASE + SZ_64K - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_NS115_XINTDEC,
		.end            = IRQ_NS115_XINTDEC,
		.flags          = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_vpu_dec_device = {
	.name           = "ns115-hx170",
	.id             = -1,
	.num_resources  = 2,
	.resource       = ns115_vpu_dec_resource,
};

/*##############################################################*/
/*             NS115 ON2 H1enc controller device                    */
/*##############################################################*/

static struct resource ns115_vpu_enc_resource[] = {
	[0] = {
		.start          = NS115_ON2_ENC_BASE,
		.end            = NS115_ON2_ENC_BASE + SZ_64K - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_NS115_HINTENC,
		.end            = IRQ_NS115_HINTENC,
		.flags          = IORESOURCE_IRQ,
	},
};

struct platform_device ns115_vpu_enc_device = {
	.name           = "ns115-hx280",
	.id             = -1,
	.num_resources  = 2,
	.resource       = ns115_vpu_enc_resource,
};

/*m@nufront: end TODO*/

#ifdef CONFIG_MACH_NS115_PBX_BOARD
static struct smsc911x_platform_config smsc911x_config = {
	.flags         = SMSC911X_USE_32BIT,
	.irq_polarity  = SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type      = SMSC911X_IRQ_TYPE_PUSH_PULL,
	.phy_interface = PHY_INTERFACE_MODE_MII,
};

static struct resource realview_pbx_smsc911x_resources[] = {
	[0] = {
		.start = REALVIEW_PBX_ETH_BASE,
		.end   = REALVIEW_PBX_ETH_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_PBX_ETH,
		.end   = IRQ_PBX_ETH,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device realview_eth_device = {
	.name = "smsc911x",
	.id   = 0,
	.dev  = {
		.platform_data = &smsc911x_config,
	},
	.resource = realview_pbx_smsc911x_resources,
	.num_resources = 2,
};
#endif

int board_device_register(struct platform_device *pdev, void * data)
{
	pdev->dev.platform_data = data;
	return platform_device_register(pdev);
}

int soc_plat_register_devices(struct soc_plat_dev *sdev, int size)
{
	int idx = 0, ret = 0;
	for(idx = 0; idx < size; idx++) {
		if(sdev[idx].data)
			sdev[idx].pdev->dev.platform_data = sdev[idx].data;
		ret = platform_device_register(sdev[idx].pdev);
		if(ret  < 0)
			return ret;
	}
	return 0;
}

int ns115_system_pm_init(void)
{
	__ns115_reset = ns115_reset;
	arm_pm_restart = ns115_restart;
	pm_power_off = ns115_power_off;
	return 0;
}

void soc_amba_register_devices(struct amba_device **adev, int size)
{
	int idx = 0;
	for(idx = 0; idx < size; idx++) {
		amba_device_register(adev[idx], &iomem_resource);
	}
}

#ifdef CONFIG_DRM_MALI
struct platform_device ns115_mali_drm_device  = {
	.name = "mali_drm",
	.id   = -1,
};
#endif

/*##############################################################*/
/*              NS2816 PWM  controller device                   */
/*##############################################################*/

struct ns115_backlight_config mistral_bl_data = {
	.default_intensity      = 6,
};

struct platform_device ns115_backlight_device = {
	.name           = "ns115-bl",
	.id             = -1,
	.dev            = {
		.platform_data = &mistral_bl_data,
	},
};

/*##############################################################*/
/*             NS115 PL330 DMA device                          */
/*##############################################################*/
#ifdef CONFIG_PL330_DMA

#define NS115_DMA_330S_IRQ IRQ_NS115_DMA330_INTR0 


static struct dma_pl330_peri pl330_plat_data_peri[]={
	[0] ={  .peri_id = 0,
		.rqtype = MEMTOMEM,
	},
};
static struct dma_pl330_platdata pl330_plat_data ={
	.nr_valid_peri = 1,
	.mcbuf_sz = SZ_64K,             
	.peri = &pl330_plat_data_peri,
};
AMBA_DEVICE(pl330_dma,  "dma-pl330",    NS115_DMA_330S, SZ_4K, &pl330_plat_data);
#endif

/*************************************************************
 *            NUSMART  GPIO KEYS Devices 
 *            Author   xujing jing.xu@nufront.com
 *            Date     2012-04-05
 **************************************************************/
static struct gpio_keys_button nusmart_buttons_data[KEY_NUM] = {
	{
		.code = KEY_VOLUMEDOWN,
		.irq = IRQ_NS115_GPIO1_INTR18,
		.key_pressed = PRESSED_IS_LOWLEVEL,
		.desc = "ns115_keys",
	},
	{
		.code = KEY_VOLUMEUP,
		.irq = IRQ_NS115_GPIO1_INTR20,
		.key_pressed = PRESSED_IS_LOWLEVEL,
		.desc = "ns115_keys",
	}
};
static struct gpio_keys_platform_data nusmart_gpio_keys_data = {
	.buttons = nusmart_buttons_data,
	.nbuttons = KEY_NUM,
	.name = "ns115-keys",
};


struct platform_device nusmart_gpio_keys_device = {
	.name           = "nusmart-gpio-keys",
	.id             = 0,
	.dev            = {
		.platform_data =  &nusmart_gpio_keys_data,
	},
};

struct rfkill_gpio_platform_data ns115ref_bt_rfkill_data = {
	.name = "bcm_rfkill",
	.reset_gpio = 8 + 32 + 13,
	.shutdown_gpio = 8 + 32 + 25,
	.power_clk_name = NULL,
	.type = RFKILL_TYPE_BLUETOOTH, 
};

struct platform_device ns115ref_bt_rfkill_device = {
	.name = "rfkill_gpio",
	.id             = -1,
	.dev            = {
		.platform_data =  &ns115ref_bt_rfkill_data,
	},
}; 

#ifdef CONFIG_SND_SOC_ALC5631
struct  ns115ref_rt5631_jd_platform_data {
    unsigned int irq;
};

struct ns115ref_rt5631_jd_platform_data ns115ref_rt5631_jd_data = {
    .irq = IRQ_NS115_GPIO0_WAKEUP_2,
};

struct platform_device ns115ref_rt5631_jd_device = {
    .name = "rt5631-jd",
    .dev            = {
        .platform_data =  &ns115ref_rt5631_jd_data,
    },
};
#endif

/************************************************************
 *	Power switch of soc
 ************************************************************/
#define ZSP_OFFSET	7
#define ISP_OFFSET	6
#define VPU_H1_OFFSET	5
#define VPU_G1_OFFSET	4
#define GC300_OFFSET	3
#define MALI_PP_OFFSET	2
#define MALI_L2_OFFSET	1
#define MALI_GP_OFFSET	0

#define MALI_MASK	BIT(MALI_GP_OFFSET) | BIT(MALI_L2_OFFSET) |\
			BIT(MALI_PP_OFFSET)

static DEFINE_SPINLOCK(switch_lock);

static inline void sync_set(unsigned int mask)
{
	unsigned long flag;
	void * __iomem addr = __io_address(PRCM_SYS_PWR_CTRL);
	unsigned int regval = 0;

	spin_lock_irqsave(&switch_lock, flag);
	regval = readl(addr);
	regval |= mask;
	writel(regval, addr);	
	spin_unlock_irqrestore(&switch_lock, flag);
}

static inline void sync_clear(unsigned int mask)
{
	unsigned long flag;
	void * __iomem addr = __io_address(PRCM_SYS_PWR_CTRL);
	unsigned int regval = 0;

	spin_lock_irqsave(&switch_lock, flag);
	regval = readl(addr);
	regval &= ~mask;
	writel(regval, addr);	
	spin_unlock_irqrestore(&switch_lock, flag);
}

static inline int get_state(unsigned int mask)
{
	void * __iomem addr = __io_address(PRCM_SYS_PWR_CTRL);
	unsigned int regval = 0;

	regval = readl(addr);
	regval &= mask;
	return (regval == 0 ? 0 : 1);
}

int zsp_power_on(void * dev_id)
{
	sync_set(BIT(ZSP_OFFSET));
	return 0;
}
int zsp_power_off(void * dev_id)
{
	sync_clear(BIT(ZSP_OFFSET));
	return 0;
}
int zsp_get_state(void * dev_id)
{
	return get_state(BIT(ZSP_OFFSET));
}
struct power_switch_ops zsp_switch = {
	.power_on	= zsp_power_on,
	.power_off	= zsp_power_off,
	.get_state	= zsp_get_state,
};

int isp_power_on(void * dev_id)
{
	sync_set(BIT(ISP_OFFSET));
	return 0;
}
int isp_power_off(void * dev_id)
{
	sync_clear(BIT(ISP_OFFSET));
	return 0;
}
int isp_get_state(void * dev_id)
{
	return get_state(BIT(ISP_OFFSET));
}
struct power_switch_ops isp_switch = {
	.power_on	= isp_power_on,
	.power_off	= isp_power_off,
	.get_state	= isp_get_state,
};

int vpu_h1_power_on(void * dev_id)
{
	sync_set(BIT(VPU_H1_OFFSET));
	return 0;
}
int vpu_h1_power_off(void * dev_id)
{
	sync_clear(BIT(VPU_H1_OFFSET));
	return 0;
}
int vpu_h1_get_state(void * dev_id)
{
	return get_state(BIT(VPU_H1_OFFSET));
}
struct power_switch_ops vpu_h1_switch = {
	.power_on	= vpu_h1_power_on,
	.power_off	= vpu_h1_power_off,
	.get_state	= vpu_h1_get_state,
};

int vpu_g1_power_on(void * dev_id)
{
	sync_set(BIT(VPU_G1_OFFSET));
	return 0;
}
int vpu_g1_power_off(void * dev_id)
{
	sync_clear(BIT(VPU_G1_OFFSET));
	return 0;
}
int vpu_g1_get_state(void * dev_id)
{
	return get_state(BIT(VPU_G1_OFFSET));
}
struct power_switch_ops vpu_g1_switch = {
	.power_on	= vpu_g1_power_on,
	.power_off	= vpu_g1_power_off,
	.get_state	= vpu_g1_get_state,
};

int gc300_power_on(void * dev_id)
{
	sync_set(BIT(GC300_OFFSET));
	return 0;
}
int gc300_power_off(void * dev_id)
{
	sync_clear(BIT(GC300_OFFSET));
	return 0;
}
int gc300_get_state(void * dev_id)
{
	return get_state(BIT(GC300_OFFSET));
}
struct power_switch_ops gc300_switch = {
	.power_on	= gc300_power_on,
	.power_off	= gc300_power_off,
	.get_state	= gc300_get_state,
};

int mali_power_on(void * dev_id)
{
	sync_set(MALI_MASK);
	return 0;
}
int mali_power_off(void * dev_id)
{
	sync_clear(MALI_MASK);
	return 0;
}
int mali_get_state(void * dev_id)
{
	return get_state(MALI_MASK);
}
struct power_switch_ops mali_switch = {
	.power_on	= mali_power_on,
	.power_off	= mali_power_off,
	.get_state	= mali_get_state,
};

struct timed_gpio gpios_config[] = {
       [0] = {
       		.name = "vibrator",
       		.gpio = 35,
        	.max_timeout = 5000000,
        	.active_low  = 0,
	}
};

struct timed_gpio_platform_data  ns115_vibrator_pdata = {
	.num_gpios 	= 1,
	.gpios	= gpios_config,
};

struct platform_device ns115_vibrator_device = {
	.name           = "timed-gpio",
	.id             = -1,
	.dev            = {
		.platform_data = &ns115_vibrator_pdata,
	},
};
