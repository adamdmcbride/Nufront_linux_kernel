/*
 *  arch/arm/mach-ns2816/ns2816-tb.c
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

#include <mach/gpio.h>
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
#ifndef CONFIG_MMC_NS2816
	SOC_PLAT_DEV(&ns2816_dwmmc_device, 	NULL),
#else
	SOC_PLAT_DEV(&ns2816_sdmmc_device, 	NULL),
#endif
	SOC_PLAT_DEV(&ns2816_i2s_plat_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_ahci_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_clcd, 		&lcdc_data),
	SOC_PLAT_DEV(&ns2816pl_clcd, 		&lcdc_data),
//	SOC_PLAT_DEV(&android_pmem_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_dw_dma_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_vpu_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_temp_device, 	NULL),

};

static struct amba_device * amba_devs[] = 
{
	&pl330_dma_device,
};

static struct extend_i2c_device __initdata extend_i2c_devs[] = 
{
#ifdef CONFIG_MFD_IO373X_I2C
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_ec_io373x, NULL, IRQ_NS2816_GPIO_INTR1, USE_DEFAULT),
#endif
};

static void __init ns2816_tb_init(void)
{
	common_init();
	ddr_pm_init();
	scm_init();

	soc_plat_register_devices(plat_devs, ARRAY_SIZE(plat_devs));
	soc_amba_register_devices(amba_devs, ARRAY_SIZE(amba_devs));
	ext_i2c_register_devices(extend_i2c_devs,ARRAY_SIZE(extend_i2c_devs));
	ns2816_system_pm_init();
}

MACHINE_START(NS2816TB, "NUFRONT-NS2816")
	.boot_params	= PHYS_OFFSET + 0x00000100,
	.map_io		= ns2816_map_io,
	.init_irq	= gic_init_irq,
	.timer		= &ns2816_timer,
	.init_machine	= ns2816_tb_init,
MACHINE_END
