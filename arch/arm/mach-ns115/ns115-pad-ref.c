/*
 *  arch/arm/mach-ns115/ns115.c
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
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>


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
#include <mach/board-ns115.h>
#include <mach/irqs.h>
#include <mach/lcdc.h>
#include <mach/extend.h>
#include <mach/get_bootargs.h>
#include <mach/gpio.h>

#include <media/soc_camera.h>
#include <mach/soc_power_ctrl.h>
#include <mach/mmc.h>
#include <mach/wake_timer.h>
#ifdef CONFIG_NS115_BATTERY
#include <linux/power/ns115-battery.h>
#endif
#ifdef CONFIG_BQ24170_CHARGER
#include <linux/power/bq24170-charger.h>
#endif

#ifdef CONFIG_NS115_EFUSE_SUPPORT
#include <mach/efuse.h>
#endif

#include "core.h"
#include "prcm.h"
#include "scm.h"
#include "common.h"
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

static struct kxtj9_platform_data kxtj9_ns115_pdata = {
	.min_interval = 5,
	.poll_interval = 20,
	.device_map = KXTJ9_DEVICE_MAP,
	.axis_map_x = KXTJ9_MAP_X,
	.axis_map_y = KXTJ9_MAP_Y,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = KXTJ9_NEG_Y,
	.negate_z = 1,
	/*.negate_z = 1,*/
	.res_12bit = RES_12BIT,
	.g_range = KXTJ9_G_2G,
};
#endif /* CONFIG_INPUT_KXTJ9 */

NS115_PINMUX_DECLARE(pad_refboard);

#if 1
static struct lcdc_platform_data lcdc_data =
{
	.ddc_adapter = I2C_BUS_2,
};
#endif

#ifdef CONFIG_BATTERY_BQ27410_GASGAUGE

static struct i2c_board_info bq27410_gasgauge = {
	I2C_BOARD_INFO("bq27410-gasgauge", 0xAA >>1 ),
};
#endif

/* NS115 soc camera gc0329 device */
static struct i2c_board_info gc0329_camera_i2c = {
	I2C_BOARD_INFO("gc0329", 0x62 >> 1),
};

static int gc0329_power(struct device * dev, int on)
{
#define GPIO_PWDN	(8+('d'-'a')*32+3)	// gpio_pd3
#define GPIO_RST	(8+('d'-'a')*32+2)	// gpio_pd2
#define GPIO_PWEN	(8+('c'-'a')*32+22)	// gpio_pc22
#define CLK_ID		"ns115_alt0"
#define CLK_RATE	24000000

	int ret = 0;
	struct clk *clk;

	ret = gpio_request(GPIO_PWDN, "gc0329");
	if (ret) {
		return ret;
	}

	ret = gpio_request(GPIO_RST, "gc0329");
	if (ret) {
		goto err_rst;
	}

	ret = gpio_request(GPIO_PWEN, "gc0329");
	if (ret) {
		goto err_pwen;
	}

	clk = clk_get(NULL, CLK_ID);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
	goto err_clk;
	}

	if (on) {
		gpio_direction_output(GPIO_PWEN, 1);
		clk_enable(clk);
		clk_set_rate(clk, CLK_RATE);
		mdelay(2);
		gpio_direction_output(GPIO_PWDN, 0);
		mdelay(2);
		gpio_direction_output(GPIO_RST, 1);
		mdelay(2);
	}
	else {
		mdelay(2);
		gpio_direction_output(GPIO_PWDN, 1);
		mdelay(2);
		gpio_direction_output(GPIO_RST, 0);
		mdelay(2);
		clk_disable(clk);
		gpio_direction_output(GPIO_PWEN, 0);
	}

	clk_put(clk);
err_clk:
	gpio_free(GPIO_PWEN);
err_pwen:
	gpio_free(GPIO_RST);
err_rst:
	gpio_free(GPIO_PWDN);

	return ret;

#undef GPIO_PWDN
#undef GPIO_RST
#undef GPIO_PWEN
#undef CLK_ID
#undef CLK_RATE
}

static struct i2c_board_info ov5640_camera_i2c = {
	I2C_BOARD_INFO("ov5640", 0x78 >> 1),
};

static int ov5640_power(struct device * dev, int on)
{
#define GPIO_PWDN	(8+('d'-'a')*32+5)	// gpio_pd5
#define GPIO_RST	(8+('d'-'a')*32+4)	// gpio_pd4
#define GPIO_PWEN	(8+('c'-'a')*32+22)	// gpio_pc22
#define CLK_ID		"ns115_alt0"
#define CLK_RATE	24000000

	int ret = 0;
	struct clk *clk;

	ret = gpio_request(GPIO_PWDN, "ov5640");
	if (ret) {
		return ret;
	}

	ret = gpio_request(GPIO_RST, "ov5640");
	if (ret) {
		goto err_rst;
	}

	ret = gpio_request(GPIO_PWEN, "ov5640");
	if (ret) {
		goto err_pwen;
	}

	clk = clk_get(NULL, CLK_ID);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto err_clk;
	}

	if (on) {
		gpio_direction_output(GPIO_PWEN, 1);
		clk_enable(clk);
		clk_set_rate(clk, CLK_RATE);
		mdelay(5);
		gpio_direction_output(GPIO_PWDN, 0);
		mdelay(1);
		gpio_direction_output(GPIO_RST, 1);
		mdelay(20);
	}
	else {
		gpio_direction_output(GPIO_RST, 0);
		clk_disable(clk);
		gpio_direction_output(GPIO_PWEN, 0);
		gpio_direction_output(GPIO_PWDN, 1);
	}

	clk_put(clk);
err_clk:
	gpio_free(GPIO_PWEN);
err_pwen:
	gpio_free(GPIO_RST);
err_rst:
	gpio_free(GPIO_PWDN);

	return ret;

#undef GPIO_PWDN
#undef GPIO_RST
#undef GPIO_PWEN
#undef CLK_ID
#undef CLK_RATE
}

static struct regulator_bulk_data camera_regulators[] = {
	{ .supply = "vdd_cam0_io_1v8" },
	{ .supply = "vdd_cam1_io_1v8" },
};

static struct soc_camera_link iclink[] = {
	{
		.bus_id		= 0, /* Must match the camera ID */
		.board_info	= &gc0329_camera_i2c,
		.i2c_adapter_id	= 1,
		.regulators	= &camera_regulators[0],
		.num_regulators	= 1,
		.power		= gc0329_power,
	},
	{
		.bus_id		= 0, /* Must match the camera ID */
		.board_info	= &ov5640_camera_i2c,
		.i2c_adapter_id	= 1,
		.regulators	= &camera_regulators[1],
		.num_regulators	= 1,
		.power		= ov5640_power,
	},
};

static int slot0_voltage_switch(void)
{
	printk(KERN_ERR "Enter %s\n", __func__);
	struct regulator *regu = regulator_get(NULL, "vddio_gpio2");

	if(IS_ERR(regu)) {
		printk(KERN_ERR "%s: regulator_get failed\n", __func__);
		return PTR_ERR(regu);
	}
	regulator_enable(regu);
	regulator_set_voltage(regu, 1800000, 1800000);
	mdelay(2);
	printk(KERN_ERR "Leave %s\n", __func__);
	return 0;
}

static int slot_attr_init(struct evatronix_sdio_slot_data *sd,
					struct mmc_host *mmc, int id)
{
	if(!sd || id > 3)
		return -EINVAL;

	mmc->caps 	= 0;
	mmc->f_min	= 400000;

	mmc->f_max 	= sd->freq;
	mmc->ocr_avail 	= sd->ocr_avail;
	mmc->caps 	= sd->caps;
	mmc->pm_caps 	= sd->pm_caps;

	printk(KERN_INFO "slot%u: f_min %d, f_max %d, ocr_avial 0x%x, caps 0x%x, pm_caps 0x%x",
				id, mmc->f_min, mmc->f_max,
				mmc->ocr_avail, mmc->caps, mmc->pm_caps);
	return 0;
}

//#define WIFI_REG_ON_GPIO	(1)
#define WIFI_REG_ON_GPIO	(8+32+17)
static struct ns115_mmc_platform_data nusmart_sdmmc_data = {
	.ref_clk		= 100000000,
	.nr_slots 		= 3,
	.gpio			= WIFI_REG_ON_GPIO,
	.detect_delay_ms	= 2*1000,
	.slot_attr_init		= slot_attr_init,

	.slots[0] = {
		.ctype       	= SD_CARD,
		.force_rescan	= false,
		.caps		= (MMC_CAP_4_BIT_DATA|
					MMC_CAP_SD_HIGHSPEED|MMC_CAP_MMC_HIGHSPEED/*|
					MMC_CAP_UHS_SDR12|MMC_CAP_UHS_SDR50*/),
		.freq 		= 25000000,
		.ocr_avail	= 0xff8000,	//2.6V-3.7V

		.voltage_switch = slot0_voltage_switch,
	},

	.slots[1] = {
		.ctype       	= EMMC_CARD,
		.force_rescan	= true,
		.caps		= (MMC_CAP_NONREMOVABLE|
					MMC_CAP_8_BIT_DATA|MMC_CAP_MMC_HIGHSPEED),
		.freq 		= 50000000,
		.ocr_avail	= 0xff8000,
	},

	.slots[2] = {
		.ctype       	= SDIO_CARD,
		.force_rescan	= true,
		.caps		= (MMC_CAP_4_BIT_DATA|MMC_CAP_SD_HIGHSPEED|
					MMC_CAP_NONREMOVABLE/*|MMC_CAP_SDIO_IRQ*/),
		.pm_caps	= (MMC_PM_KEEP_POWER|MMC_PM_IGNORE_PM_NOTIFY),
		.freq 		= 25000000,
		.ocr_avail	= 0xff8000,
	},

	.slots[3] = {
		.ctype       	= SD_CARD,
		.freq 		= 25000000/2,
	},
};

#ifdef CONFIG_NS115_BATTERY
static struct ns115_battery_platform_data ns115_batt_pdata = {
	.update_time = 5,//seconds
	.safety_time = 60 * 10,//minute
	.consumption = 4000,//mW, average value
	.pre_chg_mvolts = 3100,
	.full_mvolts = 4150,
};

static struct platform_device ns115_batt_device = {
	.id = -1,
	.name = "ns115_battery",
};
#endif

#ifdef CONFIG_BQ24170_CHARGER
static struct bq24170_charger_platform_data bq24170_charger_pdata = {
	.stat_gpio = 17 + 8,
	.ac_chg_current = 2000,
};

static struct platform_device bq24170_charger_device = {
	.id = -1,
	.name = "bq24170_charger",
};
#endif
#ifdef CONFIG_FSA880_USB_DETECT
#define FSA880_GPIO_IRQ		0
struct i2c_board_info __initdata fsa880_i2c_dev = {
	I2C_BOARD_INFO("fsa880_usb_detect", 0x25),
	.irq		= FSA880_GPIO_IRQ,
};
#endif
/*for suspend test*/
#define N3S3_FW_NAME	"n3s3_fw_ref.bin"
static struct wake_timer_data wake_data = {
	.fw_name = N3S3_FW_NAME,
	.suspend_ms = 2000,
	.wake_ms = 2000,
};

static struct soc_plat_dev plat_devs[] =
{
	SOC_PLAT_DEV(&ns115_serial_device, 	NULL),
	SOC_PLAT_DEV(&ns115_sdmmc_device,  	&nusmart_sdmmc_data),
	SOC_PLAT_DEV(&ns115_clcd_device[0],   	&lcdc_data),
	SOC_PLAT_DEV(&ns115_clcd_device[1],   	NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[0], 	NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[1], 	NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[2], 	NULL),
	SOC_PLAT_DEV(&ns115_wk_timer_device, 	&wake_data),
	SOC_PLAT_DEV(&ns115_udc_device,         NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[3], 	NULL),
	SOC_PLAT_DEV(&ns115_pcm_device,		NULL),
	SOC_PLAT_DEV(&ns115_i2s_plat_device,	NULL),
	SOC_PLAT_DEV(&ns115_vpu_dec_device, 	NULL),  //a@nufront
	SOC_PLAT_DEV(&ns115_vpu_enc_device, 	NULL),  //a@nufront
	SOC_PLAT_DEV(&ns115_usb_ehci_device, 	NULL),
	SOC_PLAT_DEV(&ns115_usb_ohci_device, 	NULL),
	SOC_PLAT_DEV(&ns115_backlight_device, 	NULL),
	SOC_PLAT_DEV(&ns115_camera_device,	NULL),
	SOC_PLAT_DEV(&ns115_hdmi_device,        NULL),  //wangzhi
	SOC_PLAT_DEV(&nusmart_gpio_keys_device, NULL),
    SOC_PLAT_DEV(&ns115_vibrator_device,   NULL),
#ifdef CONFIG_SND_SOC_ALC5631
    SOC_PLAT_DEV(&ns115ref_rt5631_jd_device, NULL),
#endif
	/*check following setting before you enable it*/
#ifdef CONFIG_SOC_CAMERA_OV5640
	SOC_PLAT_DEV(&ov5640_camera_device,	&iclink[1]),
#endif
#ifdef CONFIG_SOC_CAMERA_GC0329
	SOC_PLAT_DEV(&gc0329_camera_device,	&iclink[0]),
#endif

	SOC_PLAT_DEV(&ns115ref_bt_rfkill_device, NULL),
#ifdef CONFIG_NS115_BATTERY
	SOC_PLAT_DEV(&ns115_batt_device, &ns115_batt_pdata),
#endif
#ifdef CONFIG_BQ24170_CHARGER
	SOC_PLAT_DEV(&bq24170_charger_device, &bq24170_charger_pdata),
#endif
};

static struct extend_i2c_device __initdata extend_i2c_devs[] =
{
#ifdef CONFIG_MFD_RICOH583
	EXT_I2C_DEV(I2C_BUS_1, &ricoh583_i2c_dev, NULL, \
			IRQ_NS115_GPIO0_WAKEUP_5, USE_DEFAULT),
#endif
#ifdef CONFIG_FSA880_USB_DETECT
	EXT_I2C_DEV(I2C_BUS_1, &fsa880_i2c_dev, NULL, \
			FSA880_GPIO_IRQ, USE_DEFAULT),
#endif
	/*check following setting before you enable it*/
#ifdef CONFIG_MFD_IO373X_I2C
	EXT_I2C_DEV(I2C_BUS_0, &ns115_ec_io373x, NULL, \
			IRQ_NS115_GPIO0_WAKEUP_5, USE_DEFAULT),
#endif

#ifdef CONFIG_INPUT_KXTJ9
	EXT_I2C_DEV(I2C_BUS_0, &ns115_gs_kxtj9, &kxtj9_ns115_pdata, \
			EXT_IRQ_NOTSPEC, 0x0f),
#endif

#ifdef CONFIG_SENSORS_CM3212
	EXT_I2C_DEV(I2C_BUS_2, &ns115_ls_cm3212, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif

#ifdef CONFIG_SENSORS_AMI30X
	EXT_I2C_DEV(I2C_BUS_0, &ns115_cs_ami30x, NULL, \
			IRQ_NS115_GPIO1_INTR24, 0x0e),
#endif
#ifdef CONFIG_SND_SOC_ALC5631
	EXT_I2C_DEV(I2C_BUS_0, &ns115_snd_alc5631, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif

#if 0
#if 0
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_hdmi_sil902x, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_snd_wm8960, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BIG
	EXT_I2C_DEV(I2C_BUS_3, &ns115_tp_goodix, NULL, \
			IRQ_NS115_GPIO1_INTR6, USE_DEFAULT),
#endif

#ifdef CONFIG_BATTERY_BQ27410_GASGAUGE
	EXT_I2C_DEV(I2C_BUS_1, &bq27410_gasgauge, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif
};

#ifdef CONFIG_PL330_DMA
static struct amba_device * amba_devs[] =
{
	&pl330_dma_device,
};

static int __init dma_pl330_init(void)
{
	soc_amba_register_devices(amba_devs, ARRAY_SIZE(amba_devs));
	return 0;
}

device_initcall(dma_pl330_init);

#endif

static void __init ns115_pad_ref_init(void)
{
	common_init();
#if 0
	ddr_pm_init();
	scm_init();
#endif
	NS115_PINMUX_INIT(pad_refboard);

	bt_init();
	/*set bt_fm_switch to high and get fm in from BT module*/
	bt_fm_switch(1);
#ifdef CONFIG_BCMDHD
	extern void wifi_power_init(int);
	wifi_power_init(WIFI_REG_ON_GPIO);
#endif

#ifdef CONFIG_SND_SOC_ALC5631
    rt5631_gpio_ref_init();
#endif
	soc_plat_register_devices(plat_devs, ARRAY_SIZE(plat_devs));

	ext_i2c_register_devices(extend_i2c_devs,ARRAY_SIZE(extend_i2c_devs));
	ns115_system_pm_init();

	printk("on2_base = 0x%x, on2_size = 0x%x\n lcd_base = 0x%x, \
			lcd_size = 0x%x\n gpu_size = 0x%x, ump_size = 0x%x\n",\
			nusmart_on2_base(),
			nusmart_on2_len(),
			nusmart_lcd_base(),
			nusmart_lcd_len(),
			nusmart_mali_len(),
			nusmart_mali_ump_len());

}

struct gpio_data __initdata ref_data = {
	.gpio_ddr = {BIT(0) | BIT(4) | BIT(5),0x0,BIT(21) | BIT(30),0x0,0x0},
};

static void __init pad_ref_gic_init(void)
{
	gic_init_irq();
#ifdef CONFIG_GENERIC_GPIO
	ns115_init_gpio(&ref_data);
#endif
}

MACHINE_START(NS115_PAD_REF, "NUFRONT-NS115-PAD-REF")
.boot_params  = PHYS_OFFSET + 0x00000100,
	.map_io       = ns115_map_io,
	.init_irq     = pad_ref_gic_init,
	.timer        = &ns115_timer,
	.init_machine = ns115_pad_ref_init,
	MACHINE_END
