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
#include <mach/ns115-cpufreq.h>

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
#ifdef CONFIG_REGULATOR_NS115
#include <linux/regulator/ns115-regulator.h>
#endif
#ifdef CONFIG_MFD_RICOH583
#include <linux/mfd/ricoh583.h>
#include <linux/regulator/ricoh583-regulator.h>
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
		.caps		= (MMC_CAP_4_BIT_DATA/*|
					MMC_CAP_SD_HIGHSPEED|MMC_CAP_MMC_HIGHSPEED|
					MMC_CAP_UHS_SDR12|MMC_CAP_UHS_SDR50*/),
		.freq 		= 25000000,
		.ocr_avail	= 0xff8000,	//2.6V-3.7V

		.voltage_switch = NULL, //slot0_voltage_switch,
	},

	.slots[1] = {
		.ctype       	= EMMC_CARD,
		.force_rescan	= true,
		.caps		= (MMC_CAP_NONREMOVABLE|
					MMC_CAP_8_BIT_DATA/*|MMC_CAP_MMC_HIGHSPEED*/),
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
	.update_time = 5,//seconds
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
#define N3S3_FW_NAME	"n3s3_fw_ref.bin"
static struct wake_timer_data wake_data = {
	.fw_name = N3S3_FW_NAME,
	.suspend_ms = 2000,
	.wake_ms = 2000,
};

#ifdef CONFIG_REGULATOR_GPIO
static struct regulator_consumer_supply gpio_142_supply[] = {
	REGULATOR_SUPPLY("vdd_mali", NULL),
	REGULATOR_SUPPLY("vdd_2d", NULL),
	REGULATOR_SUPPLY("vdd_core", NULL),
	REGULATOR_SUPPLY("vdd_cpu_ram", NULL),
	REGULATOR_SUPPLY("vddl_usb", NULL),
	REGULATOR_SUPPLY("vdd_isp", NULL),
	REGULATOR_SUPPLY("vdd_enc", NULL),
	REGULATOR_SUPPLY("vdd_dec", NULL),
	REGULATOR_SUPPLY("vdd_zsp", NULL),
};

static struct regulator_consumer_supply gpio_8_supply[] = {
	REGULATOR_SUPPLY("vdd_sd_2v8", NULL),
	REGULATOR_SUPPLY("dvdd_lcd", NULL),
	REGULATOR_SUPPLY("avdd_lcd", NULL),
	REGULATOR_SUPPLY("vdd_tp", NULL),
	REGULATOR_SUPPLY("avdd_cps_2v8", NULL),
	REGULATOR_SUPPLY("avdd_sensor", NULL),
	REGULATOR_SUPPLY("avdd_lsen_2v8", NULL),
	REGULATOR_SUPPLY("avdd_aud_2v8", NULL),
};

static struct regulator_consumer_supply gpio_58_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_flash", NULL),
};

static struct regulator_consumer_supply gpio_94_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_vcm_2v8", NULL),
	REGULATOR_SUPPLY("vdd_fcam_2v8", NULL),
};

static struct regulator_consumer_supply gpio_140_supply[] = {
	REGULATOR_SUPPLY("3g_vdd", NULL),
};

static struct regulator_consumer_supply gpio_136_supply[] = {
	REGULATOR_SUPPLY("vdd_gps_1v8", NULL),
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

GPIO_REG_PDATA_INIT(142, 1100, NULL, 0, 0, 0, 0, 0);
GPIO_REG_PDATA_INIT(8,   3300, NULL, 0, 0, 0, 0, 0);
GPIO_REG_PDATA_INIT(58,  3000, NULL, 0, 0, 0, 0, 0);
GPIO_REG_PDATA_INIT(94,  2800, NULL, 0, 0, 0, 0, 0);
GPIO_REG_PDATA_INIT(140, 3300, NULL, 0, 0, 0, 0, 0);
GPIO_REG_PDATA_INIT(136, 1800, NULL, 0, 0, 0, 0, 0);

#define GPIO_REG_PLAT_DEV(_gpio)	\
	SOC_PLAT_DEV(&gpio_reg_pdev_##_gpio, &gpio_reg_pdata_##_gpio)

#define GPIO_REG_PLAT_DEV_REG	\
	GPIO_REG_PLAT_DEV(142),	\
	GPIO_REG_PLAT_DEV(8),	\
	GPIO_REG_PLAT_DEV(58),	\
	GPIO_REG_PLAT_DEV(94),	\
	GPIO_REG_PLAT_DEV(140),	\
	GPIO_REG_PLAT_DEV(136)	\

#endif //CONFIG_REGULATOR_GPIO

#ifdef CONFIG_REGULATOR_NS115
static struct regulator_consumer_supply ns115_mali_gp_supply[] = {
	REGULATOR_SUPPLY("soc_mali_gp", NULL),
};

static struct regulator_consumer_supply ns115_mali_l2c_supply[] = {
	REGULATOR_SUPPLY("soc_mali_l2c", NULL),
};

static struct regulator_consumer_supply ns115_mali_pp0_supply[] = {
	REGULATOR_SUPPLY("soc_mali_pp0", NULL),
};

static struct regulator_consumer_supply ns115_gc300_supply[] = {
	REGULATOR_SUPPLY("soc_gc300", NULL),
};

static struct regulator_consumer_supply ns115_vpu_g1_supply[] = {
	REGULATOR_SUPPLY("soc_vpu_g1", NULL),
};

static struct regulator_consumer_supply ns115_vpu_h1_supply[] = {
	REGULATOR_SUPPLY("soc_vpu_h1", NULL),
};

static struct regulator_consumer_supply ns115_isp_supply[] = {
	REGULATOR_SUPPLY("soc_isp", NULL),
};

static struct regulator_consumer_supply ns115_zsp_supply[] = {
	REGULATOR_SUPPLY("soc_zsp", NULL),
};

static struct regulator_consumer_supply ns115_pll0_supply[] = {
	REGULATOR_SUPPLY("soc_pll0", NULL),
};

static struct regulator_consumer_supply ns115_pll1_supply[] = {
	REGULATOR_SUPPLY("soc_pll1", NULL),
};

static struct regulator_consumer_supply ns115_pll2_supply[] = {
	REGULATOR_SUPPLY("soc_pll2", NULL),
};

static struct regulator_consumer_supply ns115_pll3_supply[] = {
	REGULATOR_SUPPLY("soc_pll3", NULL),
};

static struct regulator_consumer_supply ns115_pll4_supply[] = {
	REGULATOR_SUPPLY("soc_pll4", NULL),
};

static struct regulator_consumer_supply ns115_pll5_supply[] = {
	REGULATOR_SUPPLY("soc_pll5", NULL),
};

static struct regulator_consumer_supply ns115_pll6_supply[] = {
	REGULATOR_SUPPLY("soc_pll6", NULL),
};

#define NS115_REG_PDATA_INIT(_id, _name, _voltage, _supply_reg, _always_on, \
		_boot_on, _apply_uv, _init_enable, _init_apply)      \
static struct ns115_regulator_platform_data ns115_reg_pdata_##_name= \
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
		ARRAY_SIZE(ns115_##_name##_supply),	\
		.consumer_supplies = ns115_##_name##_supply, \
		.supply_regulator = _supply_reg,		\
	},							\
	.id = NS115_PWR_##_id,				\
	.init_enable = _init_enable,				\
	.init_apply = _init_apply,				\
};	\
static struct platform_device ns115_reg_pdev_##_name = {	\
	.id = NS115_PWR_##_id,	\
	.name = "ns115-regulator",	\
}

NS115_REG_PDATA_INIT(MALI_GP, mali_gp, 1100, "GPIO_142", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(MALI_L2C, mali_l2c, 1100, "GPIO_142", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(MALI_PP0, mali_pp0, 1100, "GPIO_142", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(GC300, gc300, 1100, "GPIO_142", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(VPU_G1, vpu_g1, 1100, "GPIO_142", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(VPU_H1, vpu_h1, 1100, "GPIO_142", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(ISP, isp, 1100, "GPIO_142", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(ZSP, zsp, 1100, "GPIO_142", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(PLL0, pll0, 1100, "RICOH583_LDO0", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(PLL1, pll1, 1100, "RICOH583_LDO0", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(PLL2, pll2, 1100, "RICOH583_LDO0", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(PLL3, pll3, 1100, "RICOH583_LDO0", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(PLL4, pll4, 1100, "RICOH583_LDO0", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(PLL5, pll5, 1100, "RICOH583_LDO0", 0, 0, 0, 0, 0);
NS115_REG_PDATA_INIT(PLL6, pll6, 1100, "RICOH583_LDO0", 0, 0, 0, 0, 0);

#define NS115_REG_PLAT_DEV(_name)	\
	SOC_PLAT_DEV(&ns115_reg_pdev_##_name, &ns115_reg_pdata_##_name)

#define NS115_REG_PLAT_DEV_REG	\
	NS115_REG_PLAT_DEV(mali_gp),	\
	NS115_REG_PLAT_DEV(mali_l2c),	\
	NS115_REG_PLAT_DEV(mali_pp0),	\
	NS115_REG_PLAT_DEV(gc300),	\
	NS115_REG_PLAT_DEV(vpu_g1),	\
	NS115_REG_PLAT_DEV(vpu_h1),	\
	NS115_REG_PLAT_DEV(isp),	\
	NS115_REG_PLAT_DEV(zsp),	\
	NS115_REG_PLAT_DEV(pll0),	\
	NS115_REG_PLAT_DEV(pll1),	\
	NS115_REG_PLAT_DEV(pll2),	\
	NS115_REG_PLAT_DEV(pll3),	\
	NS115_REG_PLAT_DEV(pll4),	\
	NS115_REG_PLAT_DEV(pll5),	\
	NS115_REG_PLAT_DEV(pll6)

#endif //CONFIG_REGULATOR_NS115

#ifdef CONFIG_MFD_RICOH583
#define RICOH583_IRQ_BASE   (IRQ_NS115_ZSP2CPU + 1)
#define RICOH583_GPIO_BASE   136
#define RICOH583_GPIO_IRQ   IRQ_NS115_GPIO0_WAKEUP_5

#ifdef CONFIG_REGULATOR_RICOH583
static struct regulator_consumer_supply ricoh583_dc1_supply_0[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply ricoh583_dc0_supply_0[] = {
#if 0
	REGULATOR_SUPPLY("vdd_main", NULL),
	REGULATOR_SUPPLY("vdd_2d", NULL),
	REGULATOR_SUPPLY("vdd_core", NULL),
	REGULATOR_SUPPLY("vdd_cpu_ram", NULL),
	REGULATOR_SUPPLY("vddl_usb", NULL),
	REGULATOR_SUPPLY("vdd_isp", NULL),
	REGULATOR_SUPPLY("vdd_enc", NULL),
	REGULATOR_SUPPLY("vdd_dec", NULL),
	REGULATOR_SUPPLY("vdd_zsp", NULL),
#endif
};

static struct regulator_consumer_supply ricoh583_dc2_supply_0[] = {
	REGULATOR_SUPPLY("vddio_gpio", NULL),
	REGULATOR_SUPPLY("vdd_tp_io", NULL),
	REGULATOR_SUPPLY("vdd_aud_io_1v8", NULL),
	REGULATOR_SUPPLY("vdd_emmc_1v8", NULL),
	REGULATOR_SUPPLY("vdd_cps_1v8", NULL),
	REGULATOR_SUPPLY("vdd_sensor", NULL),
	REGULATOR_SUPPLY("vdd_lsen_1v8", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
};

static struct regulator_consumer_supply ricoh583_dc3_supply_0[] = {
	REGULATOR_SUPPLY("vdd_ddr", NULL),
};
static struct regulator_consumer_supply ricoh583_ldo0_supply_0[] = {
	REGULATOR_SUPPLY("avdd_pll0", NULL),
	REGULATOR_SUPPLY("avdd_pll1", NULL),
	REGULATOR_SUPPLY("avdd_pll2", NULL),
	REGULATOR_SUPPLY("avdd_pll3", NULL),
	REGULATOR_SUPPLY("avdd_pll4", NULL),
	REGULATOR_SUPPLY("avdd_pll5", NULL),
	REGULATOR_SUPPLY("avdd_pll6", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo1_supply_0[] = {
	REGULATOR_SUPPLY("vddio_gpio2", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo2_supply_0[] = {
	REGULATOR_SUPPLY("vddio_gpio3", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo3_supply_0[] = {
	REGULATOR_SUPPLY("avdd_emmc_2v8", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo4_supply_0[] = {
	REGULATOR_SUPPLY("vdd_wakeup", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo5_supply_0[] = {
	REGULATOR_SUPPLY("vddio_wakeup", NULL),
	REGULATOR_SUPPLY("vdd_ddr1", NULL),
	REGULATOR_SUPPLY("vddio_wakeup33", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo6_supply_0[] = {
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
};
static struct regulator_consumer_supply ricoh583_ldo7_supply_0[] = {
	REGULATOR_SUPPLY("vddio_isp", NULL),
	REGULATOR_SUPPLY("vdd_cam0_io_1v8", "soc-camera-pdrv.0"),
	REGULATOR_SUPPLY("vdd_cam1_io_1v8", "soc-camera-pdrv.1"),
};

static struct regulator_consumer_supply ricoh583_ldo8_supply_0[] = {
	REGULATOR_SUPPLY("vdd_wifi_1v8", NULL),
};

static struct regulator_consumer_supply ricoh583_ldo9_supply_0[] = {
	REGULATOR_SUPPLY("avdd_usb", NULL),
};

#define RICOH_PDATA_INIT(_name, _sname, _minmv, _maxmv, _supply_reg, _always_on, \
		_boot_on, _apply_uv, _init_mV, _init_enable, _init_apply, _flags,      \
		_ext_contol, _ds_slots) \
static struct ricoh583_regulator_platform_data pdata_##_name##_##_sname = \
{								\
	.regulator = {						\
		.constraints = {				\
			.min_uV = (_minmv)*1000,		\
			.max_uV = (_maxmv)*1000,		\
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
		ARRAY_SIZE(ricoh583_##_name##_supply_##_sname),	\
		.consumer_supplies = ricoh583_##_name##_supply_##_sname, \
		.supply_regulator = _supply_reg,		\
	},							\
	.init_uV =  _init_mV * 1000,				\
	.init_enable = _init_enable,				\
	.init_apply = _init_apply,				\
	.deepsleep_slots = _ds_slots,				\
	.flags = _flags,					\
	.ext_pwr_req = _ext_contol,				\
}

RICOH_PDATA_INIT(dc0, 0,         700,  1500, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0);
RICOH_PDATA_INIT(dc1, 0,         750,  1500, 0, 1, 1, 0, -1, 0, 0, 0, 1, 7);
RICOH_PDATA_INIT(dc2, 0,         900,  2400, 0, 1, 1, 0, -1, 0, 0, 0, 1, 5);
RICOH_PDATA_INIT(dc3, 0,         900,  2400, 0, 1, 1, 0, -1, 0, 0, 0, 0, 0);

RICOH_PDATA_INIT(ldo0, 0,        1000, 3300, 0, 1, 1, 0, -1, 0, 0, 0, 1, 4);
RICOH_PDATA_INIT(ldo1, 0,        1000, 3300, 0, 1, 1, 0, -1, 0, 0, 0, 1, 5);
RICOH_PDATA_INIT(ldo2, 0,        1050, 1050, 0, 1, 1, 0, -1, 0, 0, 0, 1, 5);

RICOH_PDATA_INIT(ldo3, 0,        1000, 3300, 0, 1, 1, 0, -1, 0, 0, 0, 1, 5);
RICOH_PDATA_INIT(ldo4, 0,        750,  1500, 0, 1, 1, 0, -1, 0, 0, 0, 0, 0);
RICOH_PDATA_INIT(ldo5, 0,        1000, 3300, 0, 1, 1, 0, -1, 0, 0, 0, 0, 0);

RICOH_PDATA_INIT(ldo6, 0,        1200, 1200, 0, 1, 1, 0, -1, 0, 0, 0, 1, 4);
RICOH_PDATA_INIT(ldo7, 0,        1200, 1200, 0, 0, 0, 0, -1, 0, 0, 0, 1, 5);
RICOH_PDATA_INIT(ldo8, 0,        900,  3400, 0, 0, 0, 0, 1800, 1, 1, 0, 1, 5);
RICOH_PDATA_INIT(ldo9, 0,        900,  3400, 0, 1, 1, 0, -1, 0, 0, 0, 1, 4);

#define RICOH_REG(_id, _name, _sname)			\
{							\
	.id	= RICOH583_ID_##_id,			\
	.name	= "ricoh583-regulator",			\
	.platform_data	= &pdata_##_name##_##_sname,	\
}

#define RICOH583_DEV_REG    \
	RICOH_REG(DC0, dc0, 0),			\
RICOH_REG(DC1, dc1, 0),		\
RICOH_REG(DC2, dc2, 0),		\
RICOH_REG(DC3, dc3, 0),		\
RICOH_REG(LDO0, ldo0, 0),		\
RICOH_REG(LDO1, ldo1, 0),		\
RICOH_REG(LDO2, ldo2, 0),		\
RICOH_REG(LDO3, ldo3, 0),		\
RICOH_REG(LDO4, ldo4, 0),		\
RICOH_REG(LDO5, ldo5, 0),		\
RICOH_REG(LDO6, ldo6, 0),		\
RICOH_REG(LDO7, ldo7, 0),		\
RICOH_REG(LDO8, ldo8, 0),		\
RICOH_REG(LDO9, ldo9, 0)
#endif

#ifdef CONFIG_RTC_DRV_RC5T583
static struct ricoh583_rtc_platform_data ricoh583_rtc_data = {
	.irq = RICOH583_IRQ_BASE + RICOH583_IRQ_YALE,
	.time = {
		.tm_year = 2012,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 0,
		.tm_min = 0,
		.tm_sec = 0,
	},
};

#define RICOH583_RTC_REG				\
{						\
	.id	= -1,				\
	.name	= "rtc_ricoh583",		\
	.platform_data = &ricoh583_rtc_data,		\
}
#endif

#ifdef CONFIG_INPUT_RICOH583_PWRKEY
static struct ricoh583_pwrkey_platform_data ricoh583_pwrkey_data = {
	.irq = RICOH583_IRQ_BASE + RICOH583_IRQ_ONKEY,
	.delay_ms = 20,
};

#define RICOH583_PWRKEY_REG     \
{       \
	.id = -1,    \
	.name = "ricoh583-pwrkey",    \
	.platform_data = &ricoh583_pwrkey_data,     \
}
#endif

#ifdef CONFIG_BATTERY_RICOH583
static int ricoh583_capacity[][2] = {
	{100, 4160},
	{99, 4144},
	{98, 4128},
	{97, 4112},
	{96, 4096},
	{95, 4080},
	{94, 4064},
	{93, 4048},
	{92, 4032},
	{91, 4016},
	{90, 4000},
	{89, 3996},
	{88, 3992},
	{87, 3988},
	{86, 3984},
	{85, 3980},
	{84, 3976},
	{83, 3972},
	{82, 3968},
	{81, 3964},
	{80, 3962},
	{79, 3959},
	{78, 3956},
	{77, 3954},
	{76, 3952},
	{75, 3950},
	{74, 3948},
	{73, 3946},
	{72, 3944},
	{71, 3942},
	{70, 3940},
	{79, 3936},
	{68, 3932},
	{67, 3928},
	{66, 3924},
	{65, 3920},
	{64, 3916},
	{63, 3912},
	{62, 3908},
	{61, 3904},
	{60, 3900},
	{59, 3894},
	{58, 3888},
	{57, 3882},
	{56, 3876},
	{55, 3870},
	{54, 3864},
	{53, 3858},
	{52, 3852},
	{51, 3846},
	{50, 3840},
	{49, 3835},
	{48, 3830},
	{47, 3825},
	{46, 3820},
	{45, 3815},
	{44, 3810},
	{43, 3805},
	{42, 3800},
	{41, 3795},
	{40, 3790},
	{39, 3783},
	{38, 3776},
	{37, 3769},
	{36, 3762},
	{35, 3755},
	{34, 3748},
	{33, 3741},
	{32, 3734},
	{31, 3727},
	{30, 3720},
	{29, 3716},
	{28, 3712},
	{27, 3708},
	{26, 3704},
	{25, 3700},
	{24, 3696},
	{23, 3692},
	{22, 3688},
	{21, 3684},
	{20, 3680},
	{19, 3676},
	{18, 3672},
	{17, 3668},
	{16, 3664},
	{15, 3660},
	{14, 3656},
	{13, 3652},
	{12, 3648},
	{11, 3644},
	{10, 3640},
	{9, 3636},
	{8, 3632},
	{7, 3628},
	{6, 3624},
	{5, 3620},
	{4, 3616},
	{3, 3612},
	{2, 3608},
	{1, 3604},
	{0, 3600},
};

static struct ricoh583_battery_platform_data ricoh583_battery_data = {
	.irq_base = RICOH583_IRQ_BASE,
	.adc_channel = RICOH583_ADC_CHANNEL_AIN1,
	.multiple = 300,    //300%
	.alarm_mvolts = 3660,  //15%
	.power_off_mvolts = 3400,
	.adc_vdd_mvolts = 2800,
	.pre_chg_mvolts = 3100,
	.normal_pwr = 4000,//mW, average value
	.early_pwr = 895,//mW
	.suspend_pwr = 25,//mW
	.resistor_mohm = 95,
	.max_mAh = 4000,
	.capacity_table = &ricoh583_capacity,
	.table_size = sizeof(ricoh583_capacity) / (sizeof(int) * 2),
};
#define RICOH583_BATTERY_REG    \
{       \
	.id = -1,    \
	.name = "ricoh583-battery",     \
	.platform_data = &ricoh583_battery_data,    \
}
#endif

#ifdef CONFIG_RICOH583_AC_DETECT
static struct ricoh583_ac_detect_platform_data ricoh583_ac_detect_data = {
	.irq = RICOH583_IRQ_BASE + RICOH583_IRQ_ACOK,
	.usb_gpio = 6,
	.usb_effect = 0, /*low effective when usb plug*/
};
#define RICOH583_AC_DETECT_REG	\
{	\
	.id = -1,	\
	.name = "ricoh583_ac_detect",	\
	.platform_data = &ricoh583_ac_detect_data,	\
}
#endif

static struct ricoh583_subdev_info ricoh_devs_dcdc[] = {
	RICOH583_DEV_REG,
#ifdef CONFIG_RTC_DRV_RC5T583
	RICOH583_RTC_REG,
#endif
#ifdef CONFIG_INPUT_RICOH583_PWRKEY
	RICOH583_PWRKEY_REG,
#endif
#ifdef CONFIG_BATTERY_RICOH583
	RICOH583_BATTERY_REG,
#endif
#ifdef CONFIG_RICOH583_AC_DETECT
	RICOH583_AC_DETECT_REG,
#endif
};

#define RICOH_GPIO_INIT(_init_apply, _pulldn, _output_mode, _output_val, _ext_contol, _ds_slots) \
{					\
	.pulldn_en = _pulldn,		\
	.output_mode_en = _output_mode,	\
	.output_val = _output_val,	\
	.init_apply = _init_apply,	\
	.ext_pwr_req = _ext_contol,				\
	.deepsleep_slots = _ds_slots,				\
}
struct ricoh583_gpio_init_data ricoh_gpio_data[] = {
	RICOH_GPIO_INIT(false, false, false, 0, 0, 0),  //GPIO0
	RICOH_GPIO_INIT(false, false, false, 0, 1, 0),  //GPIO1
	RICOH_GPIO_INIT(false, false, false, 0, 1, 6),  //GPIO2
	RICOH_GPIO_INIT(false, false, false, 0, 1, 5),  //GPIO3
	RICOH_GPIO_INIT(false, true,  false, 1, 0, 0),  //GPIO4
	RICOH_GPIO_INIT(true, true, true, 1, 0, 0),  //GPIO5
	RICOH_GPIO_INIT(false, false, false, 0, 1, 6),  //GPIO6
	RICOH_GPIO_INIT(false, false, false, 0, 0, 0),  //GPIO7
};


static struct ricoh583_platform_data ricoh_platform = {
	.num_subdevs = ARRAY_SIZE(ricoh_devs_dcdc),
	.subdevs = ricoh_devs_dcdc,
	.irq_base	= RICOH583_IRQ_BASE,
	.gpio_base	= RICOH583_GPIO_BASE,
	.gpio_init_data = ricoh_gpio_data,
	.num_gpioinit_data = ARRAY_SIZE(ricoh_gpio_data),
	.enable_shutdown_pin = true,
};

struct i2c_board_info __initdata ricoh583_i2c_dev = {
	I2C_BOARD_INFO("ricoh583", 0x34),
	.irq		= RICOH583_GPIO_IRQ,
	.platform_data	= &ricoh_platform,
};
#endif //CONFIG_MFD_RICOH583

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
	SOC_PLAT_DEV(&ns115_jd_device, NULL),
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
#ifdef CONFIG_REGULATOR_GPIO
	GPIO_REG_PLAT_DEV_REG,
#endif
#ifdef CONFIG_REGULATOR_NS115
	NS115_REG_PLAT_DEV_REG,
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
