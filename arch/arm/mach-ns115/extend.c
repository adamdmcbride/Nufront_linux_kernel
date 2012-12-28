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
#ifdef CONFIG_MFD_TPS80032
#include <linux/mfd/tps80032.h>
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
RICOH_PDATA_INIT(ldo8, 0,        900,  3400, 0, 0, 0, 0, 1800, 1, 1, 0, 0, 0);
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
	.full_mvolts = 4150,
	.normal_pwr = 4000,//mW, average value
	.early_pwr = 895,//mW
	.suspend_pwr = 73,//mW
	.resistor_mohm = 75,
	.max_mAh = 4000,
	.capacity_table = &ricoh583_capacity,
	.table_size = sizeof(ricoh583_capacity) / (sizeof(int) * 2),
	.table_step = 1,
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

#ifdef CONFIG_MFD_TPS80032

#ifdef CONFIG_REGULATOR_TPS80032
static struct regulator_consumer_supply tps80032_smps1_supply[] = {
	REGULATOR_SUPPLY("smps1", NULL),
};
static struct regulator_consumer_supply tps80032_smps2_supply[] = {
	REGULATOR_SUPPLY("smps2", NULL),
};
static struct regulator_consumer_supply tps80032_smps3_supply[] = {
	REGULATOR_SUPPLY("smps3", NULL),
};
static struct regulator_consumer_supply tps80032_smps4_supply[] = {
	REGULATOR_SUPPLY("smps4", NULL),
};
static struct regulator_consumer_supply tps80032_smps5_supply[] = {
	REGULATOR_SUPPLY("smps5", NULL),
};
static struct regulator_consumer_supply tps80032_ldoln_supply[] = {
	REGULATOR_SUPPLY("ldoln", NULL),
};
static struct regulator_consumer_supply tps80032_ldo1_supply[] = {
	REGULATOR_SUPPLY("ldo1", NULL),
};
static struct regulator_consumer_supply tps80032_ldo2_supply[] = {
	REGULATOR_SUPPLY("ldo2", NULL),
};
static struct regulator_consumer_supply tps80032_ldo3_supply[] = {
	REGULATOR_SUPPLY("ldo3", NULL),
};
static struct regulator_consumer_supply tps80032_ldo4_supply[] = {
	REGULATOR_SUPPLY("ldo4", NULL),
};
static struct regulator_consumer_supply tps80032_ldo5_supply[] = {
	REGULATOR_SUPPLY("ldo5", NULL),
};
static struct regulator_consumer_supply tps80032_ldo6_supply[] = {
	REGULATOR_SUPPLY("ldo6", NULL),
};
static struct regulator_consumer_supply tps80032_ldo7_supply[] = {
	REGULATOR_SUPPLY("ldo7", NULL),
};
static struct regulator_consumer_supply tps80032_ldousb_supply[] = {
	REGULATOR_SUPPLY("ldousb", NULL),
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

TPS80032_PDATA_INIT(smps1, 600,  2100, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(smps2, 600,  2100, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(smps3, 600,  2100, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(smps4, 600,  2100, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(smps5, 600,  2100, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(ldoln, 1000,  3300, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(ldo1,  1000,  3300, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(ldo2,  1000,  3300, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(ldo3,  1000,  3300, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(ldo4,  1000,  3300, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(ldo5,  1000,  3300, 0, 1, 0, 3300, 1, 1, 0, 0);
TPS80032_PDATA_INIT(ldo6,  1000,  3300, 0, 1, 0, 0, 0, 0, 1, 0);
TPS80032_PDATA_INIT(ldo7,  1000,  3300, 0, 1, 0, 0, 0, 0, 0, 0);
TPS80032_PDATA_INIT(ldousb,1000,  3300, 0, 1, 0, 0, 0, 0, 1, 0);

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
	.max_capacity = 1200,//mAh
	.resistor = 75,//mohm
	.alarm_mvolts = 3600,  
	.power_off_mvolts = 3400,  
	.max_mAh = 1200,
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
	.irq_base	= TPS80032_IRQ_BASE,
};

struct i2c_board_info __initdata tps80032_i2c_dev = {
	I2C_BOARD_INFO("tps80032", 0x48),
	.irq		= TPS80032_GPIO_IRQ,
	.platform_data	= &tps80032_platform,
};
#endif //CONFIG_MFD_TPS80032
