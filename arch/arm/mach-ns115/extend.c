/*
 *  arch/arm/mach-ns115/extend.c
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
#ifdef CONFIG_MFD_RICOH583
#include <linux/mfd/ricoh583.h>
#include <linux/regulator/ricoh583-regulator.h>
#endif

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
#include <mach/board-ns115.h>
#include <mach/irqs.h>
#include <mach/extend.h>

#include "core.h"
#include "prcm.h"
#include "scm.h"
#include "common.h"

#include <mach/get_bootargs.h>

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

struct kxud9_platform_data kxud9_pdata = {
	.min_interval = 1,
	.poll_interval = 20,
	.g_range = KXUD9_G_2G,
	.axis_map_x = KXUD9_MAP_X,
	.axis_map_y = KXUD9_MAP_Y,
	.axis_map_z = 2,
	.negate_x = KXUD9_NEG_X,
	.negate_y = KXUD9_NEG_Y,
	.negate_z = KXUD9_NEG_Z,
	.ctrl_regc_init = KXUD9_G_2G | ODR50D,
	.ctrl_regb_init = ENABLE,
};
/*
 *gsensor kxud9
 */
struct i2c_board_info __initdata ns115_gs_kxud9 =
{
	I2C_BOARD_INFO("kxud9",0x18), 
	.platform_data = &kxud9_pdata,
};
#endif

/*
 * gsensor kxtj9-1005
 */
#ifdef CONFIG_INPUT_KXTJ9
#include <linux/input/kxtj9.h>
#define KXTJ9_DEVICE_MAP 1
#define KXTJ9_MAP_X ((KXTJ9_DEVICE_MAP-1)%2)
#define KXTJ9_MAP_Y (KXTJ9_DEVICE_MAP%2)
#define KXTJ9_NEG_X ((KXTJ9_DEVICE_MAP/2)%2)
#define KXTJ9_NEG_Y (((KXTJ9_DEVICE_MAP+1)/4)%2)
#define KXTJ9_NEG_Z ((KXTJ9_DEVICE_MAP-1)/4)
struct kxtj9_platform_data kxtj9_pdata = {
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

/*
 * gsensor kxtj9-1005
 */
struct i2c_board_info __initdata ns115_gs_kxtj9 =
{
	I2C_BOARD_INFO("kxtj9", KXTJ9_I2C_ADDR),
	.platform_data = &kxtj9_pdata,
	.irq = IRQ_NS115_GPIO1_INTR25,
};
#endif

/*
 *lightsensor cm3212
 */
struct i2c_board_info __initdata ns115_ls_cm3212 =
{
	I2C_BOARD_INFO("cm3212",0x90/2), 
};


struct touch_panel_platform_data ns115_tp_platform_data =
{
	.irq_reset = IRQ_NS115_GPIO1_INTR7, 
};

/*
 *touch screen
 */
struct i2c_board_info __initdata ns115_tp_goodix = 
{      
	I2C_BOARD_INFO("Goodix-TS", 0x55),
	.irq = IRQ_NS115_GPIO1_INTR6,
	.platform_data = &ns115_tp_platform_data, 
};     

struct i2c_board_info __initdata ns115_tp_ft5x06 =
{
	I2C_BOARD_INFO("ft5x0x_ts", 0x38),
	.irq = IRQ_NS115_GPIO1_INTR6,
	.platform_data = &ns115_tp_platform_data,
};

struct i2c_board_info __initdata ns115_tp_sis = 
{
	I2C_BOARD_INFO("sis_i2c_ts", 0x05),
	.irq = IRQ_NS115_GPIO1_INTR2,
};

struct i2c_board_info __initdata ns115_tp_zt2083 = 
{

	I2C_BOARD_INFO("zt2083_ts", 0x48),
	.irq = IRQ_NS115_GPIO1_INTR2,
};   

/*
 *Touchscreen ILITEK 10.1''
 */
struct i2c_board_info __initdata ns115_tp_ilitek = 
{
	I2C_BOARD_INFO("ilitek-tp", 0x41),
	.irq = IRQ_NS115_GPIO1_INTR2, 
};


/*
 *io373x
 */
struct i2c_board_info __initdata ns115_ec_io373x = 
{
	I2C_BOARD_INFO("io373x-i2c", 0xc4/2),
	.irq = IRQ_NS115_GPIO1_INTR1, 
};

/*
 *sound alc5631
 */
struct i2c_board_info __initdata ns115_snd_alc5631 = 
{
	I2C_BOARD_INFO("rt5631", 0x34/2),
};
/*
 *sound wm8960
 */
struct i2c_board_info __initdata ns115_snd_wm8960 = 
{
	I2C_BOARD_INFO("wm8960", 0x34/2),
};

/*
 *hdmi 7033
 */
struct i2c_board_info __initdata ns115_hdmi_7033 = 
{
	I2C_BOARD_INFO("nusmart-hdmi", 0x76),
};


/*
 *hdmi 7033 audio
 */
struct i2c_board_info __initdata ns115_hdmi_7033_audio = 
{
	I2C_BOARD_INFO("ch7033-audio", 0x19),
};


#ifdef CONFIG_SENSORS_AMI30X
/*
 *compass sensor ami306
 */
struct i2c_board_info __initdata ns115_cs_ami30x =
{
	I2C_BOARD_INFO("ami30x",0x0e),
	.irq = IRQ_NS115_GPIO1_INTR24,
};
#endif

int __init ext_i2c_register_devices(struct extend_i2c_device * devs, int size)
{
	int idx = 0, ret = 0;
	for(idx = 0; idx < size; idx++) {
		if(devs[idx].irq != EXT_IRQ_NOTSPEC)
			devs[idx].bd->irq = devs[idx].irq;
		if(devs[idx].new_addr != USE_DEFAULT)
			devs[idx].bd->addr = devs[idx].new_addr;
		if(devs[idx].data != NULL)
			devs[idx].bd->platform_data = devs[idx].data;
		ret = i2c_register_board_info(devs[idx].bus_id, \
				devs[idx].bd,1);
		if(ret < 0)
			return ret;
	}
	return 0;
}

#ifdef CONFIG_MFD_RICOH583
static struct regulator_consumer_supply ricoh583_dc1_supply_0[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply ricoh583_dc0_supply_0[] = {
	REGULATOR_SUPPLY("vdd_main", NULL),
	REGULATOR_SUPPLY("vdd_2d", NULL),
	REGULATOR_SUPPLY("vdd_core", NULL),
	REGULATOR_SUPPLY("vdd_cpu_ram", NULL),
	REGULATOR_SUPPLY("vddl_usb", NULL),
	REGULATOR_SUPPLY("vdd_isp", NULL),
	REGULATOR_SUPPLY("vdd_enc", NULL),
	REGULATOR_SUPPLY("vdd_dec", NULL),
	REGULATOR_SUPPLY("vdd_zsp", NULL),
};

static struct regulator_consumer_supply ricoh583_dc2_supply_0[] = {
	REGULATOR_SUPPLY("vddio_gpio", NULL),
	REGULATOR_SUPPLY("vdd_tp_io", NULL),
	REGULATOR_SUPPLY("vdd_aud_io_1v8", NULL),
	REGULATOR_SUPPLY("vdd_emmc_1v8", NULL),
	REGULATOR_SUPPLY("vdd_gps_1v8", NULL),
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

RICOH_PDATA_INIT(dc0, 0,         700,  1500, 0, 1, 1, 0, -1, 0, 0, 0, 0, 0);
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
RICOH_PDATA_INIT(ldo8, 0,        900,  3400, 0, 1, 1, 0, 1800, 1, 1, 0, 0, 0);
RICOH_PDATA_INIT(ldo9, 0,        900,  3400, 0, 1, 1, 0, -1, 0, 0, 0, 1, 4);

#define RICOH583_IRQ_BASE   (IRQ_NS115_ZSP2CPU + 1)
#define RICOH583_GPIO_BASE   136
#define RICOH583_GPIO_IRQ   IRQ_NS115_GPIO0_WAKEUP_5

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
	{95, 4100},
	{90, 4000},
	{85, 3981},
	{80, 3960},
	{75, 3953},
	{70, 3940},
	{65, 3920},
	{60, 3900},
	{55, 3876},
	{50, 3840},
	{45, 3816},
	{40, 3790},
	{35, 3756},
	{30, 3720},
	{25, 3700},
	{20, 3680},
	{15, 3660},
	{10, 3640},
	{5, 3620},
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
	.full_mvolts = 4150,
	.normal_pwr = 4000,//mW, average value
	.early_pwr = 895,//mW
	.suspend_pwr = 73,//mW
	.resistor_mohm = 75,
	.max_mAh = 4000,
	.capacity_table = &ricoh583_capacity,
	.table_size = sizeof(ricoh583_capacity) / (sizeof(int) * 2),
	.table_step = 5,
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
	RICOH_GPIO_INIT(false, false, false, 0, 0, 0),  //GPIO4
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
