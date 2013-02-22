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
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/irq.h>

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
#include <mach/ns115-cpufreq.h>
#include <mach/i2c.h>
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
#ifdef CONFIG_REGULATOR_GPIO
#include <linux/regulator/gpio-regulator.h>
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

NS115_PINMUX_DECLARE(phone_testboard);

#if 1
static struct lcdc_platform_data lcdc_data =
{
	.ddc_adapter = I2C_BUS_2,
};
#endif

static struct lcdc_platform_data dsi_data =
{
};

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

	clk = clk_get(NULL, CLK_ID);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
	goto err_clk;
	}

	if (on) {
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
	}

	clk_put(clk);
err_clk:
	gpio_free(GPIO_RST);
err_rst:
	gpio_free(GPIO_PWDN);

	return ret;

#undef GPIO_PWDN
#undef GPIO_RST
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

	clk = clk_get(NULL, CLK_ID);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto err_clk;
	}

	if (on) {
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
		gpio_direction_output(GPIO_PWDN, 1);
	}

	clk_put(clk);
err_clk:
	gpio_free(GPIO_RST);
err_rst:
	gpio_free(GPIO_PWDN);

	return ret;

#undef GPIO_PWDN
#undef GPIO_RST
#undef CLK_ID
#undef CLK_RATE
}

static struct regulator_bulk_data camera_regulators[] = {
	{ .supply = "avdd_cam_2v8" },
	{ .supply = "vddio_cam_1v8" },
	{ .supply = "vdd_focus_2v8" },
};

static struct soc_camera_link iclink[] = {
	{
		.bus_id		= 0, /* Must match the camera ID */
		.board_info	= &gc0329_camera_i2c,
		.i2c_adapter_id	= 2,
		.regulators	= camera_regulators,
		.num_regulators	= 2,
		.power		= gc0329_power,
	},
	{
		.bus_id		= 0, /* Must match the camera ID */
		.board_info	= &ov5640_camera_i2c,
		.i2c_adapter_id	= 2,
		.regulators	= camera_regulators,
		.num_regulators	= 3,
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

	printk(KERN_WARNING "%s slot%u: f_min %d, f_max %d, ocr_avial 0x%x, caps 0x%x, pm_caps 0x%x",__func__,
				id, mmc->f_min, mmc->f_max,
				mmc->ocr_avail, mmc->caps, mmc->pm_caps);
	return 0;
}

//#define WIFI_REG_ON_GPIO	(1)
#define WIFI_REG_ON_GPIO	(8+17)
static struct ns115_mmc_platform_data nusmart_sdmmc_data = {
	.ref_clk		= 100000000,
	.nr_slots 		= 3,
	.gpio			= WIFI_REG_ON_GPIO,
	.detect_delay_ms	= 2*1000,
	.slot_attr_init		= slot_attr_init,

	.slots[0] = {
		.ctype       	= SD_CARD,
		.force_rescan	= false,
		.caps		= (MMC_CAP_4_BIT_DATA/*|
					MMC_CAP_SD_HIGHSPEED|MMC_CAP_MMC_HIGHSPEED|
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
		.freq 		= 25000000,
		.ocr_avail	= 0xff8000,
	},

	.slots[2] = {
		.ctype       	= SDIO_CARD,
		.force_rescan	= true,
		.caps		= (MMC_CAP_4_BIT_DATA|/*MMC_CAP_SD_HIGHSPEED|*/
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
	.update_time = 10,//seconds
	.safety_time = 60 * 10,//minute
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
	.usb_chg_current = 400,
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
#define N3S3_FW_NAME	"n3s3_fw_ns115m.bin"
static struct wake_timer_data wake_data = {
	.fw_name = N3S3_FW_NAME,
	.suspend_ms = 2000,
	.wake_ms = 2000,
};

#ifdef CONFIG_REGULATOR_GPIO
static struct regulator_consumer_supply gpio_19_supply[] = {
	REGULATOR_SUPPLY("vddio_lcd_1v8", NULL),
	REGULATOR_SUPPLY("vdd_lcd_2v8", NULL),
	REGULATOR_SUPPLY("vddio_tp_1v8", NULL),
	REGULATOR_SUPPLY("vdd_tp_2v8", NULL),
	REGULATOR_SUPPLY("vddio_gpio3_1v8", NULL),
};

static struct regulator_consumer_supply gpio_23_supply[] = {
	REGULATOR_SUPPLY("vdd_focus_2v8", NULL),
};

static struct regulator_consumer_supply gpio_93_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_5v", NULL),
	REGULATOR_SUPPLY("vdd_vbus_5v", NULL),
};

static struct regulator_consumer_supply gpio_94_supply[] = {
	REGULATOR_SUPPLY("avdd_cam_2v8", NULL),
	REGULATOR_SUPPLY("vddio_cam_1v8", NULL),
};

#define GPIO_REG_PDATA_INIT(_gpio,  _voltage, _supply_reg, _always_on, \
		_boot_on, _apply_uv, _init_enable, _init_apply)      \
static struct gpio_regulator_platform_data gpio_reg_pdata_##_gpio = \
{								\
	.regulator = {						\
		.constraints = {				\
			.min_uV = (_voltage)*1000,		\
			.max_uV = (_voltage)*1000,		\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
					REGULATOR_MODE_STANDBY), \
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
					REGULATOR_CHANGE_STATUS |  \
					REGULATOR_CHANGE_VOLTAGE), \
			.always_on = _always_on,		\
			.boot_on = _boot_on,			\
			.apply_uV = _apply_uv,			\
		},						\
		.num_consumer_supplies =			\
		ARRAY_SIZE(gpio_##_gpio##_supply),	\
		.consumer_supplies = gpio_##_gpio##_supply, \
		.supply_regulator = _supply_reg,		\
	},							\
	.gpio = _gpio,				\
	.init_enable = _init_enable,				\
	.init_apply = _init_apply,				\
};	\
static struct platform_device gpio_reg_pdev_##_gpio = {	\
	.id = _gpio,	\
	.name = "gpio-regulator",	\
}

GPIO_REG_PDATA_INIT(19, 1800, NULL, 0, 0, 0, 1, 1);
GPIO_REG_PDATA_INIT(23, 2800, NULL, 0, 0, 0, 0, 0);
GPIO_REG_PDATA_INIT(93, 5000, NULL, 0, 0, 0, 1, 1);
GPIO_REG_PDATA_INIT(94, 2800, NULL, 0, 0, 0, 0, 0);

#define GPIO_REG_PLAT_DEV(_gpio)	\
	SOC_PLAT_DEV(&gpio_reg_pdev_##_gpio, &gpio_reg_pdata_##_gpio)

#define GPIO_REG_PLAT_DEV_REG	\
	GPIO_REG_PLAT_DEV(19),	\
	GPIO_REG_PLAT_DEV(23),	\
	GPIO_REG_PLAT_DEV(93),	\
	GPIO_REG_PLAT_DEV(94)

#endif //CONFIG_REGULATOR_GPIO

static struct soc_plat_dev plat_devs[] =
{
	SOC_PLAT_DEV(&ns115_serial_device, 	NULL),
#if !defined CONFIG_USE_OF
	SOC_PLAT_DEV(&ns115_sdmmc_device,  	&nusmart_sdmmc_data),
#endif
	SOC_PLAT_DEV(&ns115_clcd_device[0],   	&lcdc_data),
	SOC_PLAT_DEV(&ns115_clcd_device[1],   	NULL),
	SOC_PLAT_DEV(&ns115_dsi_device,   	&dsi_data),
#ifndef CONFIG_USE_OF
	SOC_PLAT_DEV(&ns115_i2c_device[0], 	NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[1], 	NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[2], 	NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[3], 	NULL),
#endif
	SOC_PLAT_DEV(&ns115_wk_timer_device, 	&wake_data),
	SOC_PLAT_DEV(&ns115_udc_device,         NULL),
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

#if (defined(CONFIG_SND_SOC_ALC5631) || defined(CONFIG_SND_SOC_ALC3261))
	SOC_PLAT_DEV(&ns115_jd_device, NULL),
#endif

	/*check following setting before you enable it*/
#ifdef CONFIG_SOC_CAMERA_OV5640
	SOC_PLAT_DEV(&ov5640_camera_device,	&iclink[1]),
#endif
#ifdef CONFIG_SOC_CAMERA_GC0329
	SOC_PLAT_DEV(&gc0329_camera_device,	&iclink[0]),
#endif

#if !defined(CONFIG_USE_OF) || !defined(CONFIG_BT_NW53)
	SOC_PLAT_DEV(&ns115ref_bt_rfkill_device, NULL),
#endif
#ifdef CONFIG_NS115_BATTERY
	SOC_PLAT_DEV(&ns115_batt_device, &ns115_batt_pdata),
#endif
#ifdef CONFIG_BQ24170_CHARGER
	SOC_PLAT_DEV(&bq24170_charger_device, &bq24170_charger_pdata),
#endif
#ifdef CONFIG_REGULATOR_GPIO
	GPIO_REG_PLAT_DEV_REG,
#endif
#ifdef CONFIG_TELINK_MU600
	SOC_PLAT_DEV(&mu600_gpio_device, NULL),
#endif
#if defined(CONFIG_BT_NW53) && !defined(CONFIG_USE_OF)
	SOC_PLAT_DEV(&nw53_bluesleep_device,NULL),
#endif
};

static struct extend_i2c_device __initdata extend_i2c_devs[] =
{
#if 0
#ifdef CONFIG_MFD_RICOH583
	EXT_I2C_DEV(I2C_BUS_1, &ricoh583_i2c_dev, NULL, \
			IRQ_NS115_GPIO0_WAKEUP_5, USE_DEFAULT),
#endif
#endif
#ifdef CONFIG_FSA880_USB_DETECT
	EXT_I2C_DEV(I2C_BUS_1, &fsa880_i2c_dev, NULL, \
			FSA880_GPIO_IRQ, USE_DEFAULT),
#endif
#ifdef CONFIG_MFD_TPS80032
	EXT_I2C_DEV(I2C_BUS_1, &tps80032_i2c_dev, NULL, \
			IRQ_NS115_GPIO0_WAKEUP_5, USE_DEFAULT),
#endif
	/*check following setting before you enable it*/
#ifdef CONFIG_MFD_IO373X_I2C
	EXT_I2C_DEV(I2C_BUS_0, &ns115_ec_io373x, NULL, \
			IRQ_NS115_GPIO0_WAKEUP_5, USE_DEFAULT),
#endif
#if 0
#ifdef CONFIG_INPUT_KXTJ9
	EXT_I2C_DEV(I2C_BUS_0, &ns115_gs_kxtj9, &kxtj9_ns115_pdata, \
			EXT_IRQ_NOTSPEC, 0x0f),
#endif
#endif

#ifdef CONFIG_SENSORS_CM3212
	EXT_I2C_DEV(I2C_BUS_2, &ns115_ls_cm3212, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif

#ifdef CONFIG_MPU_SENSORS_AMI306
	EXT_I2C_DEV(I2C_BUS_0, &ns115_cs_ami30x, NULL, \
			IRQ_NS115_GPIO1_INTR16, 0x0e),
#endif

#ifdef CONFIG_SENSORS_AMI30X
	EXT_I2C_DEV(I2C_BUS_0, &ns115_cs_ami30x, NULL, \
			IRQ_NS115_GPIO1_INTR24, 0x0e),
#endif

#ifdef CONFIG_SND_SOC_ALC5631
	EXT_I2C_DEV(I2C_BUS_0, &ns115_snd_alc5631, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif

#ifdef CONFIG_SND_SOC_ALC3261
	EXT_I2C_DEV(I2C_BUS_0, &ns115_snd_alc3261, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif

#ifndef CONFIG_OF
#ifdef CONFIG_TOUCHSCREEN_FT5X06
	EXT_I2C_DEV(I2C_BUS_3, &ns115_tp_ft5x06, NULL, \
			IRQ_NS115_GPIO1_INTR6, USE_DEFAULT),
#endif

#ifdef CONFIG_SENSORS_TMD27713
	EXT_I2C_DEV(I2C_BUS_0, &ns115_snesor_tmd, NULL, \
			IRQ_NS115_GPIO0_WAKEUP_4, USE_DEFAULT),
#endif


#ifdef CONFIG_SENSORS_MPU3050
	EXT_I2C_DEV(I2C_BUS_0, &ns115_sensor_mpu3050, NULL, \
			IRQ_NS115_GPIO1_INTR25, USE_DEFAULT),
#endif

#ifdef CONFIG_MPU_SENSORS_BMA250
	EXT_I2C_DEV(I2C_BUS_0, &ns115_sensor_bma250, NULL, \
			IRQ_NS115_GPIO1_INTR26, USE_DEFAULT),
#endif
#endif
#if 0
#if 0
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_hdmi_sil902x, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_snd_wm8960, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif
#endif
#ifndef CONFIG_OF
#ifdef CONFIG_TOUCHSCREEN_GT9XX
	EXT_I2C_DEV(I2C_BUS_3, &ns115_tp_goodix, NULL, \
			IRQ_NS115_GPIO1_INTR6, USE_DEFAULT),
#endif
#endif
#if 0
#ifdef CONFIG_BATTERY_BQ27410_GASGAUGE
	EXT_I2C_DEV(I2C_BUS_1, &bq27410_gasgauge, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif
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

static void __init ns115_phone_test_init(void)
{
	common_init();
#if 0
	ddr_pm_init();
	scm_init();
#endif
	NS115_PINMUX_INIT(phone_testboard);

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

#ifdef CONFIG_SND_SOC_ALC3261
	rt3261_gpio_test_init();
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

#ifdef CONFIG_USE_OF
extern struct nusmart_i2c_platform_data nusmart_i2c_data[];
static struct of_dev_auxdata ns115_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("nufront,ns115-sdmmc", NS115_SDMMC_BASE, "ns115-sdmmc.0", &nusmart_sdmmc_data),
	OF_DEV_AUXDATA("nufront,designware-i2c", NS115_I2C0_BASE, "ns115-i2c.0", &nusmart_i2c_data[0]),
	OF_DEV_AUXDATA("nufront,designware-i2c", NS115_I2C1_BASE, "ns115-i2c.1", &nusmart_i2c_data[1]),
	OF_DEV_AUXDATA("nufront,designware-i2c", NS115_I2C2_BASE, "ns115-i2c.2", &nusmart_i2c_data[2]),
	OF_DEV_AUXDATA("nufront,designware-i2c", NS115_I2C3_BASE, "ns115-i2c.3", &nusmart_i2c_data[3]),
	OF_DEV_AUXDATA("nufront,ns115-i2s", NS115_I2S0_BASE, "ns115-i2s.0", NULL),
	{}
};
static struct of_device_id ns115_dt_match_table[] __initdata = {
	{ .compatible = "nufront,designware-i2c", },
	{ .compatible = "nufront,ns115-sdmmc", },
	{}
};
static struct of_device_id ns115_dt_gic_match[] __initdata = {
	{ .compatible = "nufront,ns115-gic", },
	{}
};
static const char * ns115_dt_board_compat[] = {
	"nufront,ns115",
	"nufront,ns115m",
	NULL
};

static char *pinmux_compat = "nufront,ns115-pinmux";
static int of_ns115_int_pinmux()
{
	struct device_node *of_pinmux = NULL;
	const __be32 *list;
	u32 size,pin_setting[NS115_PINMUX_SIZE] = {0};
	u32 i=0;

	of_pinmux = of_find_node_by_path("/pinmux");
	if(!of_pinmux)
		return -1;
	if(of_device_is_compatible(of_pinmux, pinmux_compat))
	{
		list = of_get_property(of_pinmux,"setting_table",&size);
		size /= sizeof(*list);
		early_print("find pinmux of node size is %d\n",size);
		if(size != NS115_PINMUX_SIZE)
			size = NS115_PINMUX_SIZE;
		while(i < size)
		{
			*(pin_setting+i) = be32_to_cpup(list++);
			early_print("pin(%d~%d) setting 0x%x\n",i*4,i*4+3,*(pin_setting+i));
			i ++;
		}
		pinmux_init(pin_setting,size);
		return 0;
	}
	else
		return -1;
}
#endif
static void __init ns115_dt_init(void)
{

#ifdef CONFIG_USE_OF
	struct device_node *node;
	/*
	 * Finished with the static registrations now; fill in the missing
	 * devices
	 */

	node = of_find_matching_node_by_address(NULL, ns115_dt_gic_match,NS115_GIC_DIST_BASE);
	if (node)
		irq_domain_add_simple(node, IRQ_NS115_GIC_START);
	else
		early_print("%s can not find gic node\n",__func__);

	common_init();
	if(of_ns115_int_pinmux())
		NS115_PINMUX_INIT(phone_testboard);
	of_platform_populate(NULL, ns115_dt_match_table, ns115_auxdata_lookup, NULL);
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
#endif
}

struct gpio_data __initdata ref_data = {
	.gpio_ddr = {BIT(0) | BIT(4) | BIT(5),0x0,BIT(21) | BIT(30),0x0,0x0},
};

static void __init phone_test_gic_init(void)
{
	gic_init_irq();
#ifdef CONFIG_GENERIC_GPIO
	ns115_init_gpio(&ref_data);
#endif
}

#ifndef CONFIG_USE_OF
MACHINE_START(NS115_PHONE_TEST, "NUFRONT-NS115-PHONE-TEST")
.boot_params  = PHYS_OFFSET + 0x00000100,
	.map_io       = ns115_map_io,
	.init_irq     = phone_test_gic_init,
	.timer        = &ns115_timer,
	.init_machine = ns115_phone_test_init,
	MACHINE_END
#else
DT_MACHINE_START(NS115_PHONE_TEST, "NUFRONT-NS115-PHONE-TEST")
	.map_io		= ns115_map_io,
	.init_irq	= phone_test_gic_init,
	.timer		= &ns115_timer,
	.init_machine	= ns115_dt_init,
	.dt_compat	= ns115_dt_board_compat,
	MACHINE_END
#endif
