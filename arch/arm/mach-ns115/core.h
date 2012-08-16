/*
 *  linux/arch/arm/mach-ns115/core.h
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

#ifndef __ASM_ARCH_NS115_H
#define __ASM_ARCH_NS115_H

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

#define AMBA_DEVICE(name,busid,base,size,plat)\
struct amba_device name##_device = {\
	.dev = {\
		.coherent_dma_mask = ~0,\
		.init_name = busid,\
		.platform_data = plat,\
	},\
	.res = {\
		.start = base##_BASE,\
		.end   = (base##_BASE) + size - 1,\
		.flags = IORESOURCE_MEM,\
	},\
	.dma_mask = ~0,\
	.irq      = base##_IRQ,\
/*	.dma      = base##_DMA,*/\
}


struct machine_desc;
extern struct platform_device ns115_sdmmc_device;
extern struct platform_device ns115_serial_device;
extern struct platform_device ns115_i2c_device[];
extern struct platform_device ns115_clcd_device[];
extern struct platform_device ns115_wk_timer_device;
extern struct platform_device ns115_i2s_plat_device;
extern struct platform_device ns115_pcm_device;
extern struct platform_device ns115_udc_device;
extern struct platform_device ns115_usb_ehci_device;
extern struct platform_device ns115_usb_ohci_device;
extern struct platform_device ns115_camera_device;
extern struct platform_device ns115_vibrator_device;
#ifdef CONFIG_SOC_CAMERA_GC0329
extern struct platform_device gc0329_camera_device;
#endif
#ifdef CONFIG_SOC_CAMERA_OV5640
extern struct platform_device ov5640_camera_device;
#endif

#ifdef CONFIG_DRM_MALI
extern struct platform_device ns115_mali_drm_device;
#endif
#ifdef CONFIG_PL330_DMA
extern struct amba_device pl330_dma_device;
#endif

extern struct platform_device ns115_backlight_device;
extern void ns115_timer_init(void);
/*m@nufront*/
extern struct platform_device ns115_vpu_dec_device;
extern struct platform_device ns115_vpu_enc_device;

extern struct platform_device nusmart_gpio_keys_device;

extern struct platform_device ns115_hdmi_device;//by wangzhi add

extern struct platform_device ns115ref_bt_rfkill_device;

extern struct platform_device ns115ref_rt5631_jd_device;

extern void __iomem *_timer0_va_base;
extern void __iomem *_timer3_va_base;


extern void __iomem *gic_cpu_base_addr;
extern void __iomem *timer0_va_base;
extern void __iomem *timer1_va_base;
extern void __iomem *timer2_va_base;
extern void __iomem *timer3_va_base;

extern struct sys_timer ns115_timer;

extern void (*__ns115_reset)(char);
extern int board_device_register(struct platform_device *pdev, void * data);
extern int __init soc_plat_register_devices(struct soc_plat_dev *sdev, int size);
extern void __init soc_amba_register_devices(struct amba_device **adev, int size);
extern int __init ns115_system_pm_init(void);
extern void __init ns115_map_io(void);
extern void __init gic_init_irq(void);

/*power switch*/
extern struct power_switch_ops mali_switch;
extern struct power_switch_ops gc300_switch;
extern struct power_switch_ops vpu_g1_switch;
extern struct power_switch_ops vpu_h1_switch;
extern struct power_switch_ops isp_switch;
extern struct power_switch_ops zsp_switch;

#define I2C_BUS_0	0
#define I2C_BUS_1	1
#define I2C_BUS_2	2
#define I2C_BUS_3	3

#endif
