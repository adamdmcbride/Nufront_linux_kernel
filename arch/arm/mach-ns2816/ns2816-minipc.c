/*
 *  arch/arm/mach-ns2816/ns2816-huashan.c
 *
 *  Copyright (C) 2010 NUFRONT Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl061.h>
#include <linux/amba/mmci.h>
#include <linux/io.h>

#include <linux/i2c.h>

#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/pmu.h>
#include <asm/smp_twd.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/board-ns2816.h>
#include <mach/irqs.h>


#include <mach/lcdc.h>
#include <mach/extend.h>

#include "core.h"
#include "prcm.h"
#include "scm.h"
#include "common.h"

#include <mach/get_bootargs.h>

static struct lcdc_platform_data lcdc_data = 
{
	.ddc_adapter	= I2C_BUS_2,
};

static struct soc_plat_dev plat_devs[] = 
{
	SOC_PLAT_DEV(&pmu_device, 		NULL),
	SOC_PLAT_DEV(&ns2816_rtc_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_i2c_device[0], 	NULL),
	SOC_PLAT_DEV(&ns2816_i2c_device[1], 	NULL),
	SOC_PLAT_DEV(&ns2816_i2c_device[2], 	NULL),
	SOC_PLAT_DEV(&ns2816_usb_ehci_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_usb_ohci_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_gmac_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_serial_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_dwmmc_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_i2s_plat_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_pcm_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_ahci_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_clcd, 		&lcdc_data),
	SOC_PLAT_DEV(&ns2816pl_clcd, 		&lcdc_data),
	SOC_PLAT_DEV(&android_pmem_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_dw_dma_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_on2_device, 	NULL),

#ifdef CONFIG_DRM_MALI
	SOC_PLAT_DEV(&ns2816_mali_drm_device, 	NULL),
#endif	
};

#ifdef CONFIG_PL330_DMAC
static struct amba_device * amba_devs[] = 
{
	&pl330_dma_device,
};
#endif

static struct extend_i2c_device __initdata extend_i2c_devs[] = 
{
#ifdef CONFIG_MFD_IO373X_I2C
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_ec_io373x, NULL, IRQ_NS2816_GPIO_INTR1),
#endif
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_hdmi_7033, NULL, EXT_IRQ_NOTSPEC),
	EXT_I2C_DEV(I2C_BUS_1, &ns2816_snd_wm8960, NULL, EXT_IRQ_NOTSPEC),
};

static void __init ns2816_huashan_init(void)
{
	common_init();
	ddr_pm_init();
	scm_init();

	soc_plat_register_devices(plat_devs, ARRAY_SIZE(plat_devs));

#ifdef CONFIG_PL330_DMAC
	soc_amba_register_devices(amba_devs, ARRAY_SIZE(amba_devs));
#endif
	ext_i2c_register_devices(extend_i2c_devs,ARRAY_SIZE(extend_i2c_devs));
	ns2816_system_pm_init();

	printk("on2_base =0x%x, on2_size = 0x%x \nlcdc base = 0x%x,lcd_size = 0x%x \npmem_base = 0x%x, pmem_size = 0x%x \ngpu_size =0x%x, ump_size = 0x%x\n",
		nusmart_on2_base(), 
		nusmart_on2_len(),
		nusmart_lcd_base(),
		nusmart_lcd_len(),
		nusmart_pmem_base(),
		nusmart_pmem_len(),
		nusmart_mali_len(),
		nusmart_mali_ump_len());
}

MACHINE_START(NS2816_HUASHAN, "NUFRONT-NS2816")
	.boot_params	= PHYS_OFFSET + 0x00000100,
	.map_io		= ns2816_map_io,
	.init_irq	= gic_init_irq,
	.timer		= &ns2816_timer,
	.init_machine	= ns2816_huashan_init,
MACHINE_END
