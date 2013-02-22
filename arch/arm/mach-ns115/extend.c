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
#ifdef CONFIG_MFD_TPS80032
#include <linux/mfd/tps80032.h>
#endif

#include <linux/i2c.h>
#include <linux/mpu.h>
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
struct i2c_board_info __initdata ns115_gs_kxud9 = {
	I2C_BOARD_INFO("kxud9", 0x18),
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
struct i2c_board_info __initdata ns115_gs_kxtj9 = {
	I2C_BOARD_INFO("kxtj9", KXTJ9_I2C_ADDR),
	.platform_data = &kxtj9_pdata,
	.irq = IRQ_NS115_GPIO1_INTR25,
};
#endif

/*
 *lightsensor cm3212
 */
struct i2c_board_info __initdata ns115_ls_cm3212 = {
	I2C_BOARD_INFO("cm3212", 0x90 / 2),
};

struct touch_panel_platform_data ns115_tp_platform_data = {
	.irq_reset = IRQ_NS115_GPIO1_INTR7,
};

/*G-roy*/
static struct mpu_platform_data mpu3050_data = {
	.int_config = 0x10,
	.orientation = {0, -1, 0,
			1, 0, 0,
			0, 0, 1},
};

/* accel */
static struct ext_slave_platform_data inv_mpu_bma250_data = {
	.bus = EXT_SLAVE_BUS_SECONDARY,
	.orientation = {1, 0, 0,
			0, 1, 0,
			0, 0, 1},
};

/*
 *touch screen
 */
struct i2c_board_info __initdata ns115_tp_goodix = {
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BIG
	I2C_BOARD_INFO("Goodix-TS", 0x55),
#endif
#ifdef CONFIG_TOUCHSCREEN_GT9XX
	I2C_BOARD_INFO("Good9XX-TS", 0x5d),
#endif
	.irq = IRQ_NS115_GPIO1_INTR6,
	.platform_data = &ns115_tp_platform_data,
};

struct i2c_board_info __initdata ns115_tp_gt811 = {
	I2C_BOARD_INFO("GT811-TS", 0x5d),
	.irq = IRQ_NS115_GPIO1_INTR6,
	.platform_data = &ns115_tp_platform_data,
};

struct i2c_board_info __initdata ns115_tp_ft5x06 = {
	I2C_BOARD_INFO("ft5x0x_ts", 0x38),
	.irq = IRQ_NS115_GPIO1_INTR6,
	.platform_data = &ns115_tp_platform_data,
};

/*
 light/prox sensor
 */
struct i2c_board_info __initdata ns115_snesor_tmd = {
	I2C_BOARD_INFO("tritonFN", 0x39),
	.irq = IRQ_NS115_GPIO0_WAKEUP_4,
};

/*
 Groy
 */
struct i2c_board_info __initdata ns115_sensor_mpu3050 = {
	I2C_BOARD_INFO("mpu3050", 0x68),
	.irq = IRQ_NS115_GPIO1_INTR25,
	.platform_data = &mpu3050_data,
};
/*****************************************************************
 * »        g-sensor support  MPU_BMA250/BMA250/DMARD06
 * »       MPU_BMA250: connect the bma250 to MPU3050  support ns115_phone
 * »       BMA250:connect the bma250 to ns115 support ns115_pad
 * »       DMARD06:connect the bma250 to ns115 support ns115_pad
 * *******************************************************************/
#ifdef CONFIG_MPU_SENSORS_BMA250
/*
 g-sensor
 */
struct i2c_board_info __initdata ns115_sensor_bma250 = {
	I2C_BOARD_INFO("bma250", 0x18),
	.irq = IRQ_NS115_GPIO1_INTR26,
	.platform_data = &inv_mpu_bma250_data,
};
#endif

#ifdef CONFIG_SENSORS_BMA250
struct i2c_board_info __initdata ns115_gs_bma250 =
{
	I2C_BOARD_INFO("bma250",0x18),
	.irq = IRQ_NS115_GPIO1_INTR25,
};
#endif

#ifdef CONFIG_SENSORS_DMARD06
struct i2c_board_info __initdata ns115_gs_dmard06 =
{
	I2C_BOARD_INFO("dmard06",0x1c),
	.irq = IRQ_NS115_GPIO1_INTR25,
};
#endif

struct i2c_board_info __initdata ns115_tp_sis = {
	I2C_BOARD_INFO("sis_i2c_ts", 0x05),
	.irq = IRQ_NS115_GPIO1_INTR2,
};

struct i2c_board_info __initdata ns115_tp_zt2083 = {

	I2C_BOARD_INFO("zt2083_ts", 0x48),
	.irq = IRQ_NS115_GPIO1_INTR2,
};

/*
 *Touchscreen ILITEK 10.1''
 */
struct i2c_board_info __initdata ns115_tp_ilitek = {
	I2C_BOARD_INFO("ilitek-tp", 0x41),
	.irq = IRQ_NS115_GPIO1_INTR2,
};

/*
 *io373x
 */
struct i2c_board_info __initdata ns115_ec_io373x = {
	I2C_BOARD_INFO("io373x-i2c", 0xc4 / 2),
	.irq = IRQ_NS115_GPIO1_INTR1,
};

/*
 *sound alc5631
 */
struct i2c_board_info __initdata ns115_snd_alc5631 = {
	I2C_BOARD_INFO("rt5631", 0x34 / 2),
};

/*
 *sound alc3261
 */
struct i2c_board_info __initdata ns115_snd_alc3261 = {
	I2C_BOARD_INFO("rt3261", 0x38 / 2),
};

/*
 *sound wm8960
 */
struct i2c_board_info __initdata ns115_snd_wm8960 = {
	I2C_BOARD_INFO("wm8960", 0x34 / 2),
};

/*
 *hdmi 7033
 */
struct i2c_board_info __initdata ns115_hdmi_7033 = {
	I2C_BOARD_INFO("nusmart-hdmi", 0x76),
};

/*
 *hdmi 7033 audio
 */
struct i2c_board_info __initdata ns115_hdmi_7033_audio = {
	I2C_BOARD_INFO("ch7033-audio", 0x19),
};

/*
 *compass sensor ami306
 */
#ifdef  CONFIG_MPU_SENSORS_AMI306
static struct ext_slave_platform_data inv_mpu_ami306_data = {
	.bus = EXT_SLAVE_BUS_PRIMARY,
	.orientation = {-1, 0, 0,
			0, 1, 0,
			0, 0, -1},
};
#endif

struct i2c_board_info __initdata ns115_cs_ami30x = {
#ifdef CONFIG_MPU_SENSORS_AMI306
	I2C_BOARD_INFO("ami306", 0x0e),
	.irq = IRQ_NS115_GPIO1_INTR16,
	.platform_data = &inv_mpu_ami306_data,
#else
	I2C_BOARD_INFO("ami30x", 0x0e),
	.irq = IRQ_NS115_GPIO1_INTR24,
#endif
};

int __init ext_i2c_register_devices(struct extend_i2c_device *devs, int size)
{
	int idx = 0, ret = 0;
	for (idx = 0; idx < size; idx++) {
		if (devs[idx].irq != EXT_IRQ_NOTSPEC)
			devs[idx].bd->irq = devs[idx].irq;
		if (devs[idx].new_addr != USE_DEFAULT)
			devs[idx].bd->addr = devs[idx].new_addr;
		if (devs[idx].data != NULL)
			devs[idx].bd->platform_data = devs[idx].data;
		ret = i2c_register_board_info(devs[idx].bus_id,
					      devs[idx].bd, 1);
		if (ret < 0)
			return ret;
	}
	return 0;
}

#ifdef CONFIG_MFD_TPS80032

#ifdef CONFIG_REGULATOR_TPS80032
static struct regulator_consumer_supply tps80032_smps1_supply[] = {
	REGULATOR_SUPPLY("lvdd_core", NULL),
};

static struct regulator_consumer_supply tps80032_smps2_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply tps80032_smps3_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr_1v2", NULL),
};

static struct regulator_consumer_supply tps80032_smps4_supply[] = {
	REGULATOR_SUPPLY("hvdd_core", NULL),
};

static struct regulator_consumer_supply tps80032_smps5_supply[] = {
	REGULATOR_SUPPLY("vddio_emcc_1v8", NULL),
	REGULATOR_SUPPLY("vdd_gps_1v8", NULL),
	REGULATOR_SUPPLY("vddio_sensor_1v8", NULL),
	REGULATOR_SUPPLY("vddio_1v8", NULL),
};

static struct regulator_consumer_supply tps80032_ldoln_supply[] = {
	REGULATOR_SUPPLY("avdd_pll", NULL),
};

static struct regulator_consumer_supply tps80032_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_wakeup_1v1", NULL),
};

static struct regulator_consumer_supply tps80032_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_1v8", NULL),
	REGULATOR_SUPPLY("vddio_wakeup_1v8", NULL),
};

static struct regulator_consumer_supply tps80032_ldo3_supply[] = {
	REGULATOR_SUPPLY("avdd_codec_3v3", NULL),
	REGULATOR_SUPPLY("vdd_sensor_3v3", NULL),
};

static struct regulator_consumer_supply tps80032_ldo4_supply[] = {
	REGULATOR_SUPPLY("vddio_codec_1v8", NULL),
};

static struct regulator_consumer_supply tps80032_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_tf_2v8", NULL),
};

static struct regulator_consumer_supply tps80032_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd_emmc_2v8", NULL),
};

static struct regulator_consumer_supply tps80032_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_wifi_1v8", NULL),
};

static struct regulator_consumer_supply tps80032_ldousb_supply[] = {
	REGULATOR_SUPPLY("avdd_usb_3v3", NULL),
};

#define TPS80032_PDATA_INIT(_name, _minmv, _maxmv, _always_on, \
		_boot_on, _apply_uv, _init_mV, _init_enable, _init_apply,      \
		_sleep_apply, _sleep_en) \
static struct tps80032_regulator_platform_data pdata_##_name = 	\
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
		ARRAY_SIZE(tps80032_##_name##_supply),	\
		.consumer_supplies = tps80032_##_name##_supply, \
	},							\
	.init_uV =  _init_mV * 1000,				\
	.init_enable = _init_enable,				\
	.init_apply = _init_apply,				\
	.sleep_apply= _sleep_apply,				\
	.sleep_enable = _sleep_en,				\
}

//_name, _minmv, _maxmv, _always_on, _boot_on, _apply_uv, 
//_init_mV, _init_enable, _init_apply,_sleep_apply, _sleep_en

TPS80032_PDATA_INIT(smps1, 600, 2100, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(smps2, 600, 2100, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(smps3, 600, 2100, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(smps4, 600, 2100, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(smps5, 600, 2100, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(ldoln, 1000, 3300, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(ldo1, 1000, 3300, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(ldo2, 1000, 3300, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(ldo3, 1000, 3300, 0, 1, 0, 3300, 1, 1, 0, 0);
TPS80032_PDATA_INIT(ldo4, 1000, 3300, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(ldo5, 1000, 3300, 0, 1, 0, 3300, 1, 1, 0, 0);
TPS80032_PDATA_INIT(ldo6, 1000, 3300, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(ldo7, 1000, 3300, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(ldousb, 1000, 3300, 0, 1, 0, 0, 0, 0, 1, 0);

#define TPS80032_REG(_id, _name)			\
{							\
	.id	= TPS80032_ID_##_id,			\
	.name	= "tps80032-regulator",			\
	.platform_data	= &pdata_##_name,	\
}

#define TPS80032_DEV_REG    \
TPS80032_REG(SMPS1, smps1),			\
TPS80032_REG(SMPS2, smps2),			\
TPS80032_REG(SMPS3, smps3),			\
TPS80032_REG(SMPS4, smps4),			\
TPS80032_REG(SMPS5, smps5),			\
TPS80032_REG(LDOLN, ldoln),			\
TPS80032_REG(LDO1, ldo1),			\
TPS80032_REG(LDO2, ldo2),			\
TPS80032_REG(LDO3, ldo3),			\
TPS80032_REG(LDO4, ldo4),			\
TPS80032_REG(LDO5, ldo5),			\
TPS80032_REG(LDO6, ldo6),			\
TPS80032_REG(LDO7, ldo7),			\
TPS80032_REG(LDOUSB, ldousb)

#endif //CONFIG_REGULATOR_TPS80032

#define TPS80032_IRQ_BASE   (IRQ_NS115_ZSP2CPU + 1)
#define TPS80032_GPIO_IRQ   IRQ_NS115_GPIO0_WAKEUP_5

#ifdef CONFIG_INPUT_TPS80032_PWRKEY
static struct tps80032_pwrkey_platform_data tps80032_pwrkey_data = {
	.irq = TPS80032_IRQ_BASE + TPS80032_IRQ_PWRON,
};

#define TPS80032_PWRKEY_REG     \
{       \
	.id = -1,    \
	.name = "tps80032-pwrkey",    \
	.platform_data = &tps80032_pwrkey_data,     \
}
#endif

#ifdef CONFIG_RTC_DRV_TPS80032
static struct tps80032_rtc_platform_data tps80032_rtc_data = {
	.irq = TPS80032_IRQ_BASE + TPS80032_IRQ_RTC_ALARM,
};

#define TPS80032_RTC_REG				\
{						\
	.id	= -1,				\
	.name	= "rtc-tps80032",		\
	.platform_data = &tps80032_rtc_data,		\
}
#endif

static struct tps80032_adc_platform_data tps80032_adc_data = {
	.sw2_irq = TPS80032_IRQ_BASE + TPS80032_IRQ_GPADC_SW2_EOC,
};

#define TPS80032_ADC_REG	\
{	\
	.id = -1,	\
	.name = "tps80032-adc",	\
	.platform_data = &tps80032_adc_data,	\
}

#ifdef CONFIG_BATTERY_TPS80032
static int tps80032_capacity_table[][2] = {
	{100, 4160},
	{99, 4142},
	{98, 4125},
	{97, 4110},
	{96, 4102},
	{95, 4095},
	{94, 4088},
	{93, 4081},
	{92, 4070},
	{91, 4060},
	{90, 4053},
	{89, 4044},
	{88, 4038},
	{87, 4033},
	{86, 4028},
	{85, 4023},
	{84, 4017},
	{83, 4012},
	{82, 4003},
	{81, 3996},
	{80, 3990},
	{79, 3983},
	{78, 3978},
	{77, 3971},
	{76, 3965},
	{75, 3959},
	{74, 3952},
	{73, 3945},
	{72, 3940},
	{71, 3934},
	{70, 3928},
	{69, 3922},
	{68, 3915},
	{67, 3910},
	{66, 3904},
	{65, 3898},
	{64, 3893},
	{63, 3886},
	{62, 3880},
	{61, 3874},
	{60, 3866},
	{59, 3858},
	{58, 3849},
	{57, 3841},
	{56, 3835},
	{55, 3828},
	{54, 3823},
	{53, 3819},
	{52, 3815},
	{51, 3812},
	{50, 3807},
	{49, 3805},
	{48, 3802},
	{47, 3799},
	{46, 3796},
	{45, 3793},
	{44, 3791},
	{43, 3788},
	{42, 3786},
	{41, 3784},
	{40, 3781},
	{39, 3779},
	{38, 3778},
	{37, 3776},
	{36, 3775},
	{35, 3773},
	{34, 3772},
	{33, 3771},
	{32, 3770},
	{31, 3768},
	{30, 3767},
	{29, 3764},
	{28, 3761},
	{27, 3758},
	{26, 3755},
	{25, 3752},
	{24, 3748},
	{23, 3745},
	{22, 3740},
	{21, 3737},
	{20, 3735},
	{19, 3730},
	{18, 3726},
	{17, 3721},
	{16, 3715},
	{15, 3710},
	{14, 3704},
	{13, 3696},
	{12, 3688},
	{11, 3683},
	{10, 3681},
	{9, 3679},
	{8, 3678},
	{7, 3674},
	{6, 3670},
	{5, 3656},
	{4, 3623},
	{3, 3577},
	{2, 3517},
	{1, 3432},
	{0, 3350}
};

static struct tps80032_battery_platform_data tps80032_batt_data = {
	.autocal_irq = TPS80032_IRQ_BASE + TPS80032_IRQ_CC_AUTOCAL,
	.resistor = 75,		//mohm
	.alarm_mvolts = 3600,
	.power_off_mvolts = 3400,
	.max_mAh = 1700,
	.capacity_table = &tps80032_capacity_table,
	.table_size = sizeof(tps80032_capacity_table) / (sizeof(int) * 2),
};

#define TPS80032_BATTERY_REG	\
{	\
	.id = -1,	\
	.name = "tps80032-battery",	\
	.platform_data = &tps80032_batt_data,	\
}
#endif

#ifdef CONFIG_TPS80032_CHG_CTRL
static struct tps80032_chg_ctrl_platform_data tps80032_chg_ctrl_data = {
	.irq = TPS80032_IRQ_BASE + TPS80032_IRQ_CHRG_CTRL,
};

#define TPS80032_CHG_CTRL_REG	\
{	\
	.id = -1,	\
	.name = "tps80032-chg-ctrl",	\
	.platform_data = &tps80032_chg_ctrl_data,	\
}
#endif

#ifdef CONFIG_TPS80032_CHARGER
static struct tps80032_charger_platform_data tps80032_charger_data = {
	.irq = TPS80032_IRQ_BASE + TPS80032_IRQ_INT_CHRG,
};

#define TPS80032_CHARGER_REG	\
{	\
	.id = -1,	\
	.name = "tps80032-charger",	\
	.platform_data = &tps80032_charger_data,	\
}
#endif

#ifdef CONFIG_LEDS_TPS80032
static struct tps80032_leds_platform_data tps80032_leds_data = {
	.name = "tps80032-chg-led",
	.cur = TPS80032_LEDS_2_5MA,
	.source = TPS80032_LEDS_EXTPIN,
	.mode = TPS80032_LEDS_HW,
};

#define TPS80032_LEDS_REG	\
{	\
	.id = -1,	\
	.name = "leds-tps80032",	\
	.platform_data = &tps80032_leds_data,	\
}
#endif

static struct tps80032_subdev_info tps80032_subdevs[] = {
	TPS80032_ADC_REG,
#ifdef CONFIG_REGULATOR_TPS80032
	TPS80032_DEV_REG,
#endif
#ifdef CONFIG_INPUT_TPS80032_PWRKEY
	TPS80032_PWRKEY_REG,
#endif
#ifdef CONFIG_RTC_DRV_TPS80032
	TPS80032_RTC_REG,
#endif
#ifdef CONFIG_BATTERY_TPS80032
	TPS80032_BATTERY_REG,
#endif
#ifdef CONFIG_TPS80032_CHG_CTRL
	TPS80032_CHG_CTRL_REG,
#endif
#ifdef CONFIG_TPS80032_CHARGER
	TPS80032_CHARGER_REG,
#endif
#ifdef CONFIG_LEDS_TPS80032
	TPS80032_LEDS_REG,
#endif
};

static struct tps80032_platform_data tps80032_platform = {
	.num_subdevs = ARRAY_SIZE(tps80032_subdevs),
	.subdevs = tps80032_subdevs,
	.irq_base = TPS80032_IRQ_BASE,
};

struct i2c_board_info __initdata tps80032_i2c_dev = {
	I2C_BOARD_INFO("tps80032", 0x48),
	.irq = TPS80032_GPIO_IRQ,
	.platform_data = &tps80032_platform,
};
#endif //CONFIG_MFD_TPS80032
