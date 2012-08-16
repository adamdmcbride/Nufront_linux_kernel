/*
 *  linux/arch/arm/mach-ns2816/core.c
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
#include <mach/board-ns2816.h>

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
#include <asm/cacheflush.h>


struct ns2816_backlight_config {
         int default_intensity;
         int (*set_power)(struct device *dev, int state);
 };

struct timed_gpio {
        const char *name;
        unsigned        gpio;
        int             max_timeout;
        u8              active_low;
};      

struct timed_gpio_platform_data {
        int             num_gpios;
        struct timed_gpio *gpios;
};      

#ifdef CONFIG_ZONE_DMA
/*
 * Adjust the zones if there are restrictions for DMA access.
 */
void __init ns2816_adjust_zones(int node, unsigned long *size,
				  unsigned long *hole)
{
	unsigned long dma_size = SZ_256M >> PAGE_SHIFT;

	if (!machine_is_ns2816tb() || node || (size[0] <= dma_size))
		return;

	size[ZONE_NORMAL] = size[0] - dma_size;
	size[ZONE_DMA] = dma_size;
	hole[ZONE_NORMAL] = hole[0];
	hole[ZONE_DMA] = 0;
}
#endif
/*
 *ns2816 io table
 */
struct map_desc ns2816_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(NS2816_PRCM_BASE),
		.pfn		= __phys_to_pfn(NS2816_PRCM_BASE),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(NS2816_SCM_BASE),
		.pfn		= __phys_to_pfn(NS2816_SCM_BASE),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	},{
		.virtual	= IO_ADDRESS(NS2816_GIC_CPU_BASE),
		.pfn		= __phys_to_pfn(NS2816_GIC_CPU_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(NS2816_GIC_DIST_BASE),
		.pfn		= __phys_to_pfn(NS2816_GIC_DIST_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual        = IO_ADDRESS(NS2816_L220_BASE),
		.pfn            = __phys_to_pfn(NS2816_L220_BASE),
		.length         = SZ_4K,
		.type           = MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(NS2816_TIMER_BASE),
		.pfn		= __phys_to_pfn(NS2816_TIMER_BASE),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(NS2816_GPIO_BASE),
		.pfn		= __phys_to_pfn(NS2816_GPIO_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},{
                .virtual        = IO_ADDRESS(NS2816_I2S_BASE),
                .pfn            = __phys_to_pfn(NS2816_I2S_BASE),
                .length         = SZ_4K,
                .type           = MT_DEVICE,
        },{
                .virtual        = IO_ADDRESS(NS2816_DMAC_BASE),
                .pfn            = __phys_to_pfn(NS2816_DMAC_BASE),
                .length         = SZ_4K,
                .type           = MT_DEVICE,
        },
	{
		.virtual	= IO_ADDRESS(NS2816_I2C0_BASE),
		.pfn		= __phys_to_pfn(NS2816_I2C0_BASE),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	},


	{
		.virtual        = IO_ADDRESS(NS2816_LCDC_BASE),
		.pfn            = __phys_to_pfn(NS2816_LCDC_BASE),
		.length         = SZ_4K,
		.type           = MT_DEVICE,
	}, 
	{
		.virtual        = IO_ADDRESS(NS2816_IRAM_BASE),
		.pfn            = __phys_to_pfn(NS2816_IRAM_BASE),
		.length         = SZ_32K,
		.type           = MT_DEVICE,
	}, 
	{
    		.virtual        = IO_ADDRESS(NS2816_RAP_BASE),
                .pfn            = __phys_to_pfn(NS2816_RAP_BASE),
                .length         = SZ_32K,
                .type           = MT_DEVICE,
        }, 
	{
    		.virtual        = IO_ADDRESS(NS2816_DDR_CFG_BASE),
                .pfn            = __phys_to_pfn(NS2816_DDR_CFG_BASE),
                .length         = SZ_32K,
                .type           = MT_DEVICE,
        }, 
#ifdef CONFIG_DEBUG_LL
	{
		.virtual	= IO_ADDRESS(NS2816_UART0_BASE),
		.pfn		= __phys_to_pfn(NS2816_UART0_BASE),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	},
#endif
};
void __init ns2816_map_io(void)
{
	iotable_init(ns2816_io_desc, ARRAY_SIZE(ns2816_io_desc));
}
void __init gic_init_irq(void)
{
	gic_init(0, 29, __io_address(NS2816_GIC_DIST_BASE),
			 __io_address(NS2816_GIC_CPU_BASE));

#ifdef CONFIG_GENERIC_GPIO
	ns2816_init_gpio();
#endif
}
static void __init ns2816_all_timer_init(void)
{

#ifdef CONFIG_LOCAL_TIMERS
	twd_base = __io_address(NS2816_TWD_BASE);
#endif

	ns2816_timer_init();
}

struct sys_timer ns2816_timer = {
	.init		= ns2816_all_timer_init,
};

extern  void io373x_restart_system(void);
extern  void io373x_poweroff(void);

static void ns2816_reset(char mode)
{
	io373x_restart_system();	
}

static void ns2816_power_off(void)
{

}

/*
 *   When call cpu_v7_proc_fin, cpu can hung-up and system restart fail!!!
 *   Root cause is unknown!!! <2.6.33 not exist this bug> 
 */
static void ns2816_restart(char mode, const char *cmd)
{
  	unsigned long flags;
        extern void setup_mm_for_reboot(char mode);

        local_irq_save(flags);
        __cpuc_flush_kern_all();        //== flush_cache_all();
        __cpuc_flush_user_all();

        //cpu_v7_proc_fin();

        setup_mm_for_reboot(mode);
        ns2816_reset('h');

        //arm_machine_restart(mode, cmd)
}

#ifdef CONFIG_CACHE_L2X0
int __init ns2816_l2x0_init(void)
{
	void __iomem *l2x0_base = __io_address(NS2816_L220_BASE);
	unsigned int val;

	if (!(readl(l2x0_base + L2X0_CTRL) & 1)) {
		/* set RAM latencies to 8 cycle for eASIC */
		writel(0x111, l2x0_base + L2X0_TAG_LATENCY_CTRL);
		writel(0x111, l2x0_base + L2X0_DATA_LATENCY_CTRL);
		//writel(0x00000003, l2x0_base + L2X0_PREFETCH_CTRL);//0x70200007
	}

	/* 64KB way size, 16-way associativity, parity enable, event monitor bus enable,
	   full line of zero enable, shared attribute internally ignored
	   early BRESP enable, data prefetch & instruction prefetch enable,
	*/

	/* Those features are not clear in linux: value = 0x0c771000
	   non-secure interrupt access control, non-secure lockdown enable
	*/

	val = 0x7C470001;
	l2x0_init(l2x0_base, val, 0xc200ffff);//0x8200c3fe);	

	return 0;
}

early_initcall(ns2816_l2x0_init);
#endif



/*
static struct resource pmu_resources[] = {
	[0] = {
		.start		= IRQ_NS2816_OSPREY_PMUIRQ,
		.end		= IRQ_NS2816_OSPREY_PMUIRQ,
		.flags		= IORESOURCE_IRQ,
	},
	[1] = {
		.start		= IRQ_NS2816_OSPREY_PMUIRQ1,
		.end		= IRQ_NS2816_OSPREY_PMUIRQ1,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device ns2816_pmu_device = {
	.name			= "arm-pmu",
	.id			= ARM_PMU_DEVICE_CPU,
	.num_resources		= ARRAY_SIZE(pmu_resources),
	.resource		= pmu_resources,
};
*/

static struct resource pmu_resources[] = {
	[0] = {
		.start		= IRQ_NS2816_OSPREY_PMUIRQ,
		.end		= IRQ_NS2816_OSPREY_PMUIRQ,
		.flags		= IORESOURCE_IRQ,
	},
	[1] = {
		.start		= IRQ_NS2816_OSPREY_PMUIRQ1,
		.end		= IRQ_NS2816_OSPREY_PMUIRQ1,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device pmu_device = {
	.name			= "arm-pmu",
	.id			= ARM_PMU_DEVICE_CPU,
	.num_resources		= ARRAY_SIZE(pmu_resources),
	.resource		= pmu_resources,
};

/*##############################################################*/
/*		NS2816 gmac controller device		 	*/
/*##############################################################*/

static struct resource ns2816_emac_resources[]={
	[0] = {
		.start	=	NS2816_ETH_BASE,
		.end	=	NS2816_ETH_BASE + SZ_64K - 1,
		.flags	=	IORESOURCE_MEM,
	},
	[1] = {
		.start 	=	IRQ_NS2816_GMAC_TOP_SBD,
		.end	=	IRQ_NS2816_GMAC_TOP_SBD,	
		.flags	=	IORESOURCE_IRQ,
	},

};

struct platform_device ns2816_gmac_device = {
        .name                   = "ns2816-emac",
        .id                     = -1,
	.num_resources	=	ARRAY_SIZE(ns2816_emac_resources),
	.resource	=	ns2816_emac_resources,
};

/*##############################################################*/
/*		NS2816 SD/MMC controller device			*/
/*##############################################################*/

static struct	ns2816_mmc_platform_data nusmart_sdmmc_data = {
	.max_freq	= 50000000,
	.nr_slots	= 3,
	.slots[0]	= {					//emmc
		.wires		=	4,
		.card_type	=	0,
		//.card_type	=	SDIO_CARD,
		.ocr_mask	=	MMC_VDD_32_33|MMC_VDD_33_34,
		//.device_max_freq	=	25000000,	/* 8.3Mhz */
#ifdef CONFIG_MACH_NS2816_YANGCHENG
		.device_max_freq	=	25000000/2,	/* 8.3Mhz */
#else
		.device_max_freq	=	25000000,	/* 8.3Mhz */
#endif
		//.device_max_freq	=	8300000,	/* 25Mhz */
		.name		=	"nusmart-sdmmc.0",
	},
	.slots[1]	= {
		.wires		=	4,
		.card_type	=	SDIO_CARD,
		//.card_type	=	0,
		.ocr_mask	=	MMC_VDD_32_33|MMC_VDD_33_34,
		.device_max_freq	=	25000000/2,	/* 8.3Mhz */
		.name		=	"nusmart-sdmmc.1",
	},
	.slots[2]	= {					//tf card
		.wires		=	4,
		//.wires		=	4,
		//.card_type	=	SDIO_CARD,
		.card_type	=	0,
		.ocr_mask	=	MMC_VDD_32_33|MMC_VDD_33_34,
		//.device_max_freq	=	8300000,	/* 25Mhz */

#ifdef CONFIG_MACH_NS2816_YANGCHENG
		.device_max_freq	=	25000000,	/* 25Mhz */	
#else
		.device_max_freq	=	25000000/2,	/* 25Mhz */	
#endif
		.name		=	"nusmart-sdio.0",
	},
};

static struct resource ns2816_sdmmc_resource[] = {
	[0] = {
		.start		= NS2816_SDMMC_BASE,
		.end		= NS2816_SDMMC_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_NS2816_SDMMC,
		.end		= IRQ_NS2816_SDMMC,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device ns2816_sdmmc_device = {
	.name		= "ns2816-sdmmc",
	.id		= 0,
	.dev	={
			.platform_data =  &nusmart_sdmmc_data,
		},
				
	.num_resources	= 2,
	.resource	= ns2816_sdmmc_resource,
};

/***************************************************************
 *	 	DW MMC data
 **************************************************************/
static int dwmmc_init(u32 slot_id, irq_handler_t irq_handle, void * p)
{
	return 0;
};
static void dwmmc_select_slot(u32 slot_id)
{
};
static int dwmmc_get_ocr(u32 slot_id)
{
	return MMC_VDD_32_33|MMC_VDD_33_34;
};

static int dwmmc_get_ro(u32 slot_id)
{
	return 0;
};
static int dwmmc_get_bus_wd(u32 slot_id)
{
	if(slot_id == 2)
		return 8;
	else
		return 4;
};
static int dwmmc_get_max_hz(u32 slot_id)
{
	/* all slot for 25000000 */
	return 25000000;
};

static int dwmmc_get_slot_id(u32 mmc_id)
{
	return mmc_id;
};

static struct dw_mci_board nusmart_dwmmc_data = {
	.num_slots 	= 3,
	.bus_hz		= 50000000,
	.detect_delay_ms= 800,
	.quirks		= DW_MCI_QUIRK_RETRY_DELAY\
				/* | DW_MCI_QUIRK_IDMAC_DTO*/,
	.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED |  MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SDIO_IRQ,
	.init		= dwmmc_init,
	.select_slot	= dwmmc_select_slot,
	.get_ocr	= dwmmc_get_ocr,
	.get_cd		= NULL,//,
	.get_ro		= dwmmc_get_ro,
	.get_bus_wd	= dwmmc_get_bus_wd,
	.get_max_hz	= dwmmc_get_max_hz,
	.get_slot_id	= dwmmc_get_slot_id,
};
/***************************************************************
 *	 	DW MMC Devices
 **************************************************************/
struct platform_device ns2816_dwmmc_device = {
	.name		= "dw_mmc",
	.id		= 0,
	.dev		=
	{
		.platform_data =  &nusmart_dwmmc_data,
                .coherent_dma_mask = DMA_BIT_MASK(32),
        },
				
	.num_resources	= 2,
	.resource	= ns2816_sdmmc_resource,
};


/*##############################################################*/
/*	NS2816 SATA AHCI controller device			*/
/*##############################################################*/

static struct resource ns2816_ahci_resource[] = {
	[0] = {
		.start		= NS2816_SATA_BASE,
		.end		= NS2816_SATA_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_NS2816_SATA_INTRQ,
		.end		= IRQ_NS2816_SATA_INTRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device ns2816_ahci_device = {
	.name		= "ns2816-ahci",
	.id		= 0,
	.num_resources	= 2,
	.resource	= ns2816_ahci_resource,
};

/*##############################################################*/
/*		NS2816 RTC controller device			*/
/*##############################################################*/

static struct resource ns2816_rtc_resource[] = {
	[0] = {
		.start		= NS2816_RTC_BASE,
		.end		= NS2816_RTC_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_NS2816_RTC_INTR,
		.end		= IRQ_NS2816_RTC_INTR,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device ns2816_rtc_device = {
	.name		= "ns2816-rtc",
	.id		= 0,
	.num_resources	= 2,
	.resource	= ns2816_rtc_resource,
};


/*##############################################################*/
/*		NS2816 I2C controller device			*/
/*##############################################################*/
static struct resource ns2816_i2c0_resource[] = {
	[0] = {
		.start		= NS2816_I2C0_BASE,
		.end		= NS2816_I2C0_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_NS2816_I2C_0_INTR,
		.end		= IRQ_NS2816_I2C_0_INTR,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct resource ns2816_i2c1_resource[] = {
	[0] = {
		.start		= NS2816_I2C1_BASE,
		.end		= NS2816_I2C1_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_NS2816_I2C_1_INTR,
		.end		= IRQ_NS2816_I2C_1_INTR,
		.flags		= IORESOURCE_IRQ,
	},
};
static struct resource ns2816_i2c2_resource[] = {
	[0] = {
		.start		= NS2816_I2C2_BASE,
		.end		= NS2816_I2C2_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_NS2816_I2C_2_INTR,
		.end		= IRQ_NS2816_I2C_2_INTR,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct nusmart_i2c_platform_data nusmart_i2c_data[] = {
#ifdef TOUCHSCREEN_I2C_BUS_FAST_MODE
	{
		.speed = I2C_SPEED_FAST,
	},
#else
	{
		.speed = I2C_SPEED_STD,
	},
#endif
	{
		.speed = I2C_SPEED_STD,
	},
	{
		.speed = I2C_SPEED_STD,
	},
}; 

struct platform_device ns2816_i2c_device[] = {
	{
	.name		= "i2c_designware",
	.id		= I2C_BUS_1,
	.num_resources	= 2,
	.resource	= ns2816_i2c1_resource,
	.dev = {
            .platform_data  = &nusmart_i2c_data[0],
        },
	},
	{
	.name		= "i2c_designware",
	.id		= I2C_BUS_2,
	.num_resources	= 2,
	.resource	= ns2816_i2c2_resource,
	.dev = {
            .platform_data  = &nusmart_i2c_data[1],
        },
	},
	{
	.name		= "i2c_designware",
	.id		= I2C_BUS_0,
	.num_resources	= 2,
	.resource	= ns2816_i2c0_resource,
	.dev = {
            .platform_data  = &nusmart_i2c_data[2],
        },
	},

	{
	.name		= "",
	.id		= 0,
	.num_resources	= 0,
	.resource	=NULL
	},
};


/*##############################################################*/
/*		NS2816 uart controller device			*/
/*##############################################################*/
static struct plat_serial8250_port serial_platform_data[] = {
        {
                .mapbase        = NS2816_UART0_BASE,
                .irq            = IRQ_NS2816_UART_0_INTR,
                .flags          = UPF_BOOT_AUTOCONF | UPF_IOREMAP | UPF_SKIP_TEST | UPF_SHARE_IRQ,
                .iotype         = UPIO_MEM32,
                .regshift       = 2,
                .uartclk        = 66666666,
        },
	{
                .mapbase        = NS2816_UART1_BASE,
                .irq            = IRQ_NS2816_UART_1_INTR,
                .flags          = UPF_BOOT_AUTOCONF | UPF_IOREMAP | UPF_SKIP_TEST | UPF_SHARE_IRQ,
                .iotype         = UPIO_MEM32,
                .regshift       = 2,
                .uartclk        = 66666666,
        },
	{
		.flags		= 0,
	},
};

struct platform_device ns2816_serial_device = {
        .name                   = "serial8250",
        .id                     = PLAT8250_DEV_PLATFORM,
        .dev                    = {
                .platform_data  = serial_platform_data,
        },
};


/*##############################################################*/
/*		NS2816 i2s controller device			*/
/*##############################################################*/

static struct resource ns2816_i2s_resource[] = {
	[0] = {
		.start		= NS2816_I2S_BASE,
		.end		= NS2816_I2S_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_NS2816_I2S_INTR,
		.end		= IRQ_NS2816_I2S_INTR,
		.flags		= IORESOURCE_IRQ,
	},
};


struct platform_device ns2816_i2s_device = {
	.name		= "ns2816-i2s",
	.id		= -1,
	.num_resources	= 2,
	.resource	= ns2816_i2s_resource,
};

struct platform_device ns2816_i2s_plat_device = {
	.name		= "ns2816-i2s-plat",
	.id		= -1,
	.num_resources	= 0,
	.resource	= NULL,
};

struct platform_device ns2816_pcm_device = {
	.name		= "nusmart-pcm-audio",
	.id		= -1,
	.num_resources	= 0,
	.resource	= NULL,
};

/*##############################################################*/
/*		NS2816 usb controller device			*/
/*##############################################################*/

/*
 * USB ehci register define
 */
static struct resource ns2816_ehci_resources[] = {
	[0] = {
		.start		= NS2816_USB_EHCI_BASE,
		.end		= NS2816_USB_EHCI_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_NS2816_USB_INTR_IRQ,
		.end		= IRQ_NS2816_USB_INTR_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device ns2816_usb_ehci_device = {
	.name			= "ns2816-ehci",
	.num_resources		= 2,
	.resource		= ns2816_ehci_resources,
        .dev = {
                .coherent_dma_mask = ~0,
        },                      
};

int ns2816_usb_ehci_register(struct resource *res)
{
	ns2816_usb_ehci_device.resource = res;
	return platform_device_register(&ns2816_usb_ehci_device);
}

/*
 * USB ohci register define
 */
static struct resource ns2816_ohci_resources[] = {
	[0] = {
		.start		= NS2816_USB_OHCI_BASE,
		.end		= NS2816_USB_OHCI_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_NS2816_USB_OHCI_0_INTR,
		.end		= IRQ_NS2816_USB_OHCI_0_INTR,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device ns2816_usb_ohci_device = {
	.name			= "ns2816-ohci",
	.num_resources		= 2,
	.resource		= ns2816_ohci_resources,
        .dev = {
                .coherent_dma_mask = ~0,
        },                      
};

int ns2816_usb_ohci_register(struct resource *res)
{
	ns2816_usb_ohci_device.resource = res;
	return platform_device_register(&ns2816_usb_ohci_device);
}

void ns2816_usb_register(void)
{
	ns2816_usb_ehci_register(ns2816_ehci_resources);
	ns2816_usb_ohci_register(ns2816_ohci_resources);
}

/*##############################################################*/
/*		NS2816 lcd controller device			*/
/*##############################################################*/
#define LCDC_NAME_NULCD "ns2816fb"
static struct resource ns2816fb_resources[] = {
        [0] = {
                .start          = NS2816_LCDC_BASE,
                .end            = NS2816_LCDC_BASE + SZ_4K - 1,
                .flags          = IORESOURCE_MEM,
        },
	[1] = { 
		.start          = IRQ_NS2816_DISPLAY_DERR, 
		.end            = IRQ_NS2816_DISPLAY_VERT,
		.flags          = IORESOURCE_IRQ,
	},
};

struct platform_device ns2816_clcd = {
	.name = LCDC_NAME_NULCD,
	.id = 0,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource = ns2816fb_resources,
	.num_resources =2,
};
/*##############################################################*/
/*		NS2816 lcd pl111 controller device		*/
/*##############################################################*/
#define LCDC_NAME_PL "ns2816plfb"
static struct resource ns2816plfb_resources[] = {
        [0] = {
                .start          = NS2816_LCDC_BASE,
                .end            = NS2816_LCDC_BASE + SZ_4K - 1,
                .flags          = IORESOURCE_MEM,
        },
	[1] = { 
		.start          = IRQ_NS2816_PL111_VCOMP_INTR, 
		.end            = IRQ_NS2816_PL111_MBE_INTR,
		.flags          = IORESOURCE_IRQ,
	},
};

struct platform_device ns2816pl_clcd = {
	.name = LCDC_NAME_PL,
	.id = 0,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource = ns2816plfb_resources,
	.num_resources =2,
};



static struct android_pmem_platform_data android_pmem_pdata = {
/*       .name = "pmem",*/
        .name = "pmem_adsp",
//	.start = nusmart_pmem_base(),
//       .size  = nusmart_pmem_len(),
	.start = 0,	//this value will be set at the proper driver
	.size = 0,	//this value will be set at the proper driver
       .no_allocator = 0,
       .cached = 0,
};

struct platform_device android_pmem_device = {
       .name = "android_pmem",
       .id = 0,
       .dev = { .platform_data = &android_pmem_pdata }, 
};

/*##############################################################*/
/*              NS2816 dma controller device                    */
/*##############################################################*/

#ifdef CONFIG_PL330_DMA
/*##################### PL330 DMA ##############################*/
#define NS2816_DMA_330_IRQ IRQ_NS2816_DMA_IRQ 


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
AMBA_DEVICE(pl330_dma,  "dma-pl330",    NS2816_DMA_330, SZ_4K, &pl330_plat_data);


#endif

/*##################### dW_DMA ##############################*/
static struct dw_dma_platform_data dw_dmac_platform_data = {
        .nr_channels            = 8,
};


static struct resource dw_dma_resources[] = {
        [0] = {
                .start          = NS2816_DMAC_BASE,
                .end            = NS2816_DMAC_BASE + SZ_4K - 1,
                .flags          = IORESOURCE_MEM,
        },
        [1] = {
                .start          = IRQ_NS2816_DMAC_COMBINED,
                .end            = IRQ_NS2816_DMAC_COMBINED,
                .flags          = IORESOURCE_IRQ,
        },
};

struct platform_device ns2816_dw_dma_device = {
        .name                   = "ns2816_dw_dma",
        .id                     = -1,
        .num_resources          = ARRAY_SIZE(dw_dma_resources),
        .resource               = dw_dma_resources,
        .dev                    = {
                .platform_data  = &dw_dmac_platform_data,
        },
};


/*##############################################################*/
/*             NS2816 VPU controller device                    */
/*##############################################################*/

static struct resource ns2816_vpu_resource[] = {
       [0] = {
               .start          = NS2816_VPU_BASE,
               .end            = NS2816_VPU_BASE + SZ_64K - 1,
               .flags          = IORESOURCE_MEM,
       },
       [1] = {
               .start          = IRQ_NS2816_VPU_XINTDEC,
               .end            = IRQ_NS2816_VPU_XINTDEC,
               .flags          = IORESOURCE_IRQ,
       },
};

struct platform_device ns2816_vpu_device = {
       .name           = "ns2816-hx170",
       .id             = -1,
       .num_resources  = 2,
       .resource       = ns2816_vpu_resource,
};

/*##############################################################*/
/*		NS2816 temperture  sensor  controller device		 	*/
/*##############################################################*/

struct platform_device ns2816_temp_device = {
        .name                   = "ns2816-temp",
        .id                     = -1,
	.num_resources	=	0,
	.resource	=	NULL,
};

/*##############################################################*/
/*              NS2816 PWM  controller device                   */
/*##############################################################*/
   
struct ns2816_backlight_config mistral_bl_data = {
	  .default_intensity      = 0xa0,
};
	 
struct platform_device ns2816_backlight_device = {
	.name           = "ns2816-bl",
	.id             = -1,
	.dev            = {
		.platform_data = &mistral_bl_data,
	},
};

struct timed_gpio gpios_config[] = {
       [0] = {
       		.name = "vibrator",
       		.gpio = 45,
        	.max_timeout = 5000000,
        	.active_low  = 0,
	}
};

struct timed_gpio_platform_data  ns2816_vibrator_pdata = {
	.num_gpios 	= 1,
	.gpios	= gpios_config,
};

struct platform_device ns2816_vibrator_device = {
	.name           = "timed-gpio",
	.id             = -1,
	.dev            = {
		.platform_data = &ns2816_vibrator_pdata,
	},
};

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

int ns2816_system_pm_init(void)
{
	__ns2816_reset = ns2816_reset;
	pm_power_off = ns2816_power_off;
	arm_pm_restart = ns2816_restart;
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
struct platform_device ns2816_mali_drm_device  = {
	.name = "mali_drm",
	.id   = -1,
};
#endif

