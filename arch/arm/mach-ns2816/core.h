/*
 *  linux/arch/arm/mach-ns2816/core.h
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

#ifndef __ASM_ARCH_NS2816_H
#define __ASM_ARCH_NS2816_H

#include <linux/amba/bus.h>
#include <linux/io.h>

#include <asm/setup.h>
#include <asm/leds.h>

#define SOC_PLAT_DEV(dev, pdata) \
		{.pdev = dev, .data = pdata}

struct soc_plat_dev
{
	struct platform_device * pdev;
	void * data;
};

#define AMBA_DEVICE(name,busid,base,size,plat)			\
struct amba_device name##_device = {			\
	.dev		= {					\
		.coherent_dma_mask = ~0,			\
		.init_name = busid,				\
		.platform_data = plat,				\
	},							\
	.res		= {					\
		.start	= base##_BASE,		\
		.end	= (base##_BASE) + size - 1,	\
		.flags	= IORESOURCE_MEM,			\
	},							\
	.dma_mask	= ~0,					\
	.irq		= base##_IRQ,				\
	/* .dma		= base##_DMA,*/				\
}


struct machine_desc;
extern struct platform_device ns2816_gmac_device;
extern struct platform_device ns2816_sdmmc_device;
extern struct platform_device ns2816_dwmmc_device;
extern struct platform_device ns2816_ahci_device;  
extern struct platform_device ns2816_serial_device;
extern struct platform_device ns2816_i2s_plat_device;
extern struct platform_device ns2816_pcm_device;
extern struct platform_device android_pmem_device; 
extern struct platform_device ns2816_i2c_device[];
extern struct platform_device ns2816_rtc_device;
extern struct platform_device ns2816_i2s_device;
extern struct platform_device ns2816_clcd;
extern struct platform_device ns2816pl_clcd;
extern struct platform_device ns2816_vpu_device;
extern struct platform_device ns2816_temp_device;
extern struct platform_device pmu_device;
extern struct platform_device ns2816_dw_dma_device;
extern struct platform_device ns2816_usb_ehci_device;
extern struct platform_device ns2816_usb_ohci_device;
extern struct platform_device ns2816_backlight_device; 
extern struct platform_device ns2816_vibrator_device; 
#ifdef CONFIG_DRM_MALI
extern struct platform_device ns2816_mali_drm_device;
#endif
extern struct platform_device ns2816_vibrator_device;
extern void ns2816_timer_init(void);
extern void ns2816_lcd_register(void);

extern void __iomem *_timer0_va_base;
extern void __iomem *_timer3_va_base;


extern struct clcd_board clcd_plat_data;
extern void __iomem *gic_cpu_base_addr;
extern void __iomem *timer0_va_base;
extern void __iomem *timer1_va_base;
extern void __iomem *timer2_va_base;
extern void __iomem *timer3_va_base;

extern struct sys_timer ns2816_timer;
extern struct amba_device pl330_dma_device;

//extern void realview_leds_event(led_event_t ledevt);
extern void (*__ns2816_reset)(char);
extern int board_device_register(struct platform_device *pdev, void * data);
extern int __init soc_plat_register_devices(struct soc_plat_dev *sdev, int size);
extern int __init ns2816_system_pm_init(void);
extern void __init soc_amba_register_devices(struct amba_device **adev, int size);
extern void __init ns2816_map_io(void);
extern void __init gic_init_irq(void);

#define I2C_BUS_0	2
#define I2C_BUS_1	0
#define I2C_BUS_2	1

#endif
