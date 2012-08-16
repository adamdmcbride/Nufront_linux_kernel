/*
 *  arch/arm/mach-ns2816/ns2816-huangshan.c
 *
 *  Copyright (C) 2011 NUFRONT Limited
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
#include <linux/delay.h>
#include <linux/mmc/dw_mmc.h>

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

//#include <mach/gpio.h>
#include <mach/lcdc.h>
#include <mach/extend.h>
#include <mach/soc_power_ctrl.h>

#include "core.h"
#include "prcm.h"
#include "scm.h"
#include "common.h"
#include "clock.h"

#include <mach/get_bootargs.h>
/*****************************************************
 *		board specific data
 ****************************************************/

#ifdef CONFIG_SENSORS_KXUD9
/*
 *kxud9
 */
#include <linux/kxud9.h>	//gsensor kxud9
#define KXUD9_DEVICE_MAP 1
#define KXUD9_MAP_X (KXUD9_DEVICE_MAP-1)%2
#define KXUD9_MAP_Y KXUD9_DEVICE_MAP%2
#define KXUD9_NEG_X (KXUD9_DEVICE_MAP/2)%2
#define KXUD9_NEG_Y (KXUD9_DEVICE_MAP+1)/4
#define KXUD9_NEG_Z (KXUD9_DEVICE_MAP-1)/4

static struct kxud9_platform_data kxud9_pdata_huangshan = {
	.min_interval = 1,
	.poll_interval = 20,
	.g_range = KXUD9_G_2G,
	.axis_map_x = KXUD9_MAP_X,
	.axis_map_y = KXUD9_MAP_Y,
	.axis_map_z = 2,
	.negate_x = KXUD9_NEG_X,
	.negate_y = KXUD9_NEG_Y,
	.negate_z = 1,
	.ctrl_regc_init = KXUD9_G_2G | ODR50D,
	.ctrl_regb_init = ENABLE,
};
#endif

/*
 * kxtj9
 */
#ifdef CONFIG_INPUT_KXTJ9
#include <linux/input/kxtj9.h>
#define KXTJ9_DEVICE_MAP 1
#define KXTJ9_MAP_X ((KXTJ9_DEVICE_MAP-1)%2)
#define KXTJ9_MAP_Y (KXTJ9_DEVICE_MAP%2)
#define KXTJ9_NEG_X ((KXTJ9_DEVICE_MAP/2)%2)
#define KXTJ9_NEG_Y (((KXTJ9_DEVICE_MAP+1)/4)%2)
#define KXTJ9_NEG_Z ((KXTJ9_DEVICE_MAP-1)/4)

struct kxtj9_platform_data kxtj9_pdata_huangshan = {
	.min_interval = 5,
	.poll_interval = 20,
	.device_map = KXTJ9_DEVICE_MAP,
	.axis_map_x = KXTJ9_MAP_X,
	.axis_map_y = KXTJ9_MAP_Y,
	.axis_map_z = 2,
	.negate_x = KXTJ9_NEG_X,
	.negate_y = KXTJ9_NEG_Y,
	//.negate_z = KXTJ9_NEG_Z,
	.negate_z = 1,
	.res_12bit = RES_12BIT,
	.g_range = KXTJ9_G_2G,
};
#endif /* CONFIG_INPUT_KXTJ9 */


/* lcdc data */
static struct lcdc_platform_data lcdc_data = 
{
	.ddc_adapter	= I2C_BUS_0,
};
/* sd/mmc data */
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
//	if(slot_id == 2)
//		return 8;
//	else
		return 4;
};
static int dwmmc_get_max_hz(u32 slot_id)
{
	if(slot_id == 0)
		return 25000000/2;

	if(slot_id == 1)
		return 25000000/2;

	return 25000000;
};
static int dwmmc_get_slot_id(u32 mmc_id)
{
	return 2-mmc_id;
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

/*****************************************************
 *		end of data
 ****************************************************/

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
	SOC_PLAT_DEV(&ns2816_dwmmc_device, 	&nusmart_dwmmc_data),
#else
	SOC_PLAT_DEV(&ns2816_sdmmc_device, 	NULL),
#endif
	SOC_PLAT_DEV(&ns2816_pcm_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_i2s_plat_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_ahci_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_clcd, 		&lcdc_data),
	SOC_PLAT_DEV(&ns2816pl_clcd, 		&lcdc_data),
//	SOC_PLAT_DEV(&android_pmem_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_dw_dma_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_vpu_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_temp_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_backlight_device, 	NULL),
	SOC_PLAT_DEV(&ns2816_vibrator_device, 	NULL),
};

#ifdef CONFIG_PL330_DMA
static int pl330_clk_enable(struct clk * c)
{
	return 0;
}
static int pl330_clk_disable(struct clk * c)
{
	return 0;
}
static struct clk pl330_clk={
	.enable=pl330_clk_enable,
	.disable=pl330_clk_disable,
};

static struct amba_device * amba_devs[] = 
{
	&pl330_dma_device,
};
#endif

static struct extend_i2c_device __initdata extend_i2c_devs[] = 
{
#ifdef CONFIG_MFD_IO373X_I2C
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_ec_io373x, NULL, \
			IRQ_NS2816_GPIO_INTR1, USE_DEFAULT),
#endif
#ifdef CONFIG_SENSORS_KXUD9
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_gs_kxud9, &kxud9_pdata_huangshan, \
			EXT_IRQ_NOTSPEC, 0x19),
#endif
#ifdef CONFIG_INPUT_KXTJ9
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_gs_kxtj9, &kxtj9_pdata_huangshan, \
			EXT_IRQ_NOTSPEC, 0x0f),
#endif

#ifdef CONFIG_SENSORS_CM3212
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_ls_cm3212, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_hdmi_sil902x, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_snd_wm8960, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BIG
	EXT_I2C_DEV(I2C_BUS_1, &ns2816_tp_goodix, NULL, \
			IRQ_NS2816_GPIO_INTR2, USE_DEFAULT),
#endif
};

static struct pinmux_setting __initdata pinmux[] = 
{
	GPIO_A_15,
	GPIO_A_21,
	GPIO_B_14_15,
	GPIO_B_18_19,
	MMC_SLOT_2,
	UART_1,
};
static void __init ns2816_huangshan_init(void)
{
	common_init();
	ddr_pm_init();
	scm_init();
	pinmux_init(pinmux, ARRAY_SIZE(pinmux));

	wifi_power(OFF);
	mdelay(100);
	wifi_power(ON);
	soc_plat_register_devices(plat_devs, ARRAY_SIZE(plat_devs));

#ifdef CONFIG_PL330_DMA
		
	(&pl330_dma_device)->pclk = &pl330_clk;

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

MACHINE_START(HUANGSHANS, "NUFRONT-NS2816")
	//.phys_io	= NS2816_AXI_MAIN_BASE,
	//.io_pg_offst	= (IO_ADDRESS(NS2816_AXI_MAIN_BASE) >> 18) & 0xfffc,
	.boot_params	= PHYS_OFFSET + 0x00000100,
	.map_io		= ns2816_map_io,
	.init_irq	= gic_init_irq,
	.timer		= &ns2816_timer,
	.init_machine	= ns2816_huangshan_init,
MACHINE_END

