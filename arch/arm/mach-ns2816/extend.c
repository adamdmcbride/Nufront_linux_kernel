/*
 *  arch/arm/mach-ns2816/extend.c
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
struct i2c_board_info __initdata ns2816_gs_kxud9 =
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
struct i2c_board_info __initdata ns2816_gs_kxtj9 =
{
	I2C_BOARD_INFO("kxtj9", KXTJ9_I2C_ADDR),
	.platform_data = &kxtj9_pdata,
	.irq = IRQ_NS2816_GPIO_INTR3,
};
#endif
 
/*
 *lightsensor cm3212
 */
struct i2c_board_info __initdata ns2816_ls_cm3212 =
{
	I2C_BOARD_INFO("cm3212",0x90/2), 
};

/*
 *touch screen
 */
struct i2c_board_info __initdata ns2816_tp_goodix = 
{      
	I2C_BOARD_INFO("Goodix-TS", 0x55),
	.irq = IRQ_NS2816_GPIO_INTR2,
};     

struct i2c_board_info __initdata ns2816_tp_sis = 
{
        I2C_BOARD_INFO("sis_i2c_ts", 0x05),
       .irq = IRQ_NS2816_GPIO_INTR2,
};

struct i2c_board_info __initdata ns2816_tp_zt2083 = 
{

        I2C_BOARD_INFO("zt2083_ts", 0x48),
       .irq = IRQ_NS2816_GPIO_INTR2,
};   
 
/*
 *Touchscreen ILITEK 10.1''
 */
struct i2c_board_info __initdata ns2816_tp_ilitek = 
{
	I2C_BOARD_INFO("ilitek-tp", 0x41),
	.irq = IRQ_NS2816_GPIO_INTR2, 
};

 
/*
 *io373x
 */
struct i2c_board_info __initdata ns2816_ec_io373x = 
{
	I2C_BOARD_INFO("io373x-i2c", 0xc4/2),
	.irq = IRQ_NS2816_GPIO_INTR1, 
};

/*
 *sound wm8960
 */
struct i2c_board_info __initdata ns2816_snd_wm8960 = 
{
	I2C_BOARD_INFO("wm8960", 0x34/2),
};

/*
 *hdmi 7033
 */
struct i2c_board_info __initdata ns2816_hdmi_7033 = 
{
	I2C_BOARD_INFO("nusmart-hdmi", 0x76),
};


/*
 *hdmi 7033 audio
 */
struct i2c_board_info __initdata ns2816_hdmi_7033_audio = 
{
	I2C_BOARD_INFO("ch7033-audio", 0x19),
};

/*
 *hdmi Sil902x
 */
struct sil902x_platformdata {
	unsigned char uc_gpio;
};
struct sil902x_platformdata sil902x_data = {
	.uc_gpio = 33,		// GPIOB[1] for sil902x reset pin
};
struct i2c_board_info __initdata ns2816_hdmi_sil902x = 
{
	I2C_BOARD_INFO("sil902x-hdmi", 0x39),
	.irq = IRQ_NS2816_GPIO_INTR0, 
	.platform_data = &sil902x_data,
};


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

