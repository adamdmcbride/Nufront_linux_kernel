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
#include <mach/wake_timer.h>

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
	.negate_x = KXTJ9_NEG_X,
	.negate_y = KXTJ9_NEG_Y,
	.negate_z = KXTJ9_NEG_Z,
	/*.negate_z = 1,*/
	.res_12bit = RES_12BIT,
	.g_range = KXTJ9_G_2G,
};
#endif /* CONFIG_INPUT_KXTJ9 */

NS115_PINMUX_DECLARE(pad_testboard);

#if 1
static struct lcdc_platform_data lcdc_data = 
{
	.ddc_adapter = I2C_BUS_2,
};
#endif

/* NS115 soc camera gc0329 device */
static struct i2c_board_info gc0329_camera_i2c = {
	I2C_BOARD_INFO("gc0329", 0x62 >> 1),
};

static int gc0329_power(struct device * dev, int on)
{
#define GPIO_PWDN       (8+('d'-'a')*32+3)      // gpio_pd3
#define GPIO_RST        (8+('d'-'a')*32+2)      // gpio_pd2
#define GPIO_PWEN   	(8+('b'-'a')*32+14)     // gpio_pb14
#define CLK_ID          "ns115_alt0"
#define CLK_RATE        24000000

	int ret = 0;
	struct clk * clk;

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

	if(on) {
		gpio_direction_output(GPIO_PWEN, 1);
		clk_enable(clk);
	        clk_set_rate(clk, CLK_RATE);
	        mdelay(2);
	        gpio_direction_output(GPIO_PWDN, 0);
	        mdelay(2);
	        gpio_direction_output(GPIO_RST, 0);
	        mdelay(2);
	        gpio_direction_output(GPIO_RST, 1);
	        mdelay(2);
	} else {
	        mdelay(2);
		gpio_direction_output(GPIO_PWDN, 1);
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
#define GPIO_PWDN       (8+('d'-'a')*32+5)      // gpio_pd5
#define GPIO_RST        (8+('d'-'a')*32+4)      // gpio_pd4
#define GPIO_PWEN	(8+('b'-'a')*32+14)     // gpio_pb14
#define CLK_ID          "ns115_alt0"
#define CLK_RATE        24000000

	int ret = 0;
	struct clk * clk;

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

	if(on) {
		gpio_direction_output(GPIO_PWEN, 1);
		clk_enable(clk);
	        clk_set_rate(clk, CLK_RATE);
	        mdelay(5);
	        gpio_direction_output(GPIO_PWDN, 0);
	        mdelay(1);
	        gpio_direction_output(GPIO_RST, 1);
	        mdelay(20);
	} else {
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

static struct soc_camera_link iclink[] = {
	{
		.bus_id		= 0, /* Must match the camera ID */
		.board_info	= &gc0329_camera_i2c,
		.i2c_adapter_id	= 1,
		.power = gc0329_power,
	},
	{
		.bus_id		= 0, /* Must match the camera ID */
		.board_info	= &ov5640_camera_i2c,
		.i2c_adapter_id	= 1,
		.power = ov5640_power,
	},
};

/*for suspend test*/
#define N3S3_FW_NAME	"n3s3_fw.bin"
static struct wake_timer_data wake_data = {
	.fw_name = N3S3_FW_NAME,
	.suspend_ms = 5000,
	.wake_ms = 10000,
};

static struct soc_plat_dev plat_devs[] = 
{
	SOC_PLAT_DEV(&ns115_serial_device, 	NULL),
	SOC_PLAT_DEV(&ns115_sdmmc_device,  	NULL),
	SOC_PLAT_DEV(&ns115_clcd_device[0],   	&lcdc_data),
	SOC_PLAT_DEV(&ns115_clcd_device[1],   	NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[0], 	NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[1], 	NULL),
	SOC_PLAT_DEV(&ns115_i2c_device[2], 	NULL),
	SOC_PLAT_DEV(&ns115_wk_timer_device, 	NULL),	/*set wake_data in need*/
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
#ifdef CONFIG_SOC_CAMERA_OV5640
	SOC_PLAT_DEV(&ov5640_camera_device, &iclink[1]),
#endif
#ifdef CONFIG_SOC_CAMERA_GC0329
	SOC_PLAT_DEV(&gc0329_camera_device, &iclink[0]),
#endif
	SOC_PLAT_DEV(&nusmart_gpio_keys_device, NULL),
	SOC_PLAT_DEV(&ns115_hdmi_device,        NULL),  //wangzhi
};

static struct extend_i2c_device __initdata extend_i2c_devs[] = 
{
#ifdef CONFIG_MFD_RICOH583
	EXT_I2C_DEV(I2C_BUS_1, &ricoh583_i2c_dev, NULL, \
			IRQ_NS115_GPIO0_WAKEUP_5, USE_DEFAULT),
#endif
#ifdef CONFIG_MFD_IO373X_I2C
	EXT_I2C_DEV(I2C_BUS_0, &ns115_ec_io373x, NULL, \
			IRQ_NS115_GPIO0_WAKEUP_5, USE_DEFAULT),
#endif

#ifdef CONFIG_SND_SOC_ALC5631
	EXT_I2C_DEV(I2C_BUS_0, &ns115_snd_alc5631, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif

#ifdef CONFIG_INPUT_KXTJ9
	EXT_I2C_DEV(I2C_BUS_0, &ns115_gs_kxtj9, &kxtj9_ns115_pdata, \
			EXT_IRQ_NOTSPEC, 0x0f),
#endif

#ifdef CONFIG_SENSORS_CM3212
	EXT_I2C_DEV(I2C_BUS_2, &ns115_ls_cm3212, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif
#if 0
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_hdmi_sil902x, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
	EXT_I2C_DEV(I2C_BUS_0, &ns2816_snd_wm8960, NULL, \
			EXT_IRQ_NOTSPEC, USE_DEFAULT),
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BIG
	EXT_I2C_DEV(I2C_BUS_3, &ns115_tp_goodix, NULL, \
			IRQ_NS115_GPIO1_INTR6, USE_DEFAULT),
#endif
#ifdef CONFIG_SENSORS_AMI30X
	EXT_I2C_DEV(I2C_BUS_0, &ns115_cs_ami30x, NULL, \
			IRQ_NS115_GPIO1_INTR24, 0x0e),
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

static void __init ns115_pad_test_init(void)
{
	common_init();
#if 0
	ddr_pm_init();
	scm_init();
#endif
	NS115_PINMUX_INIT(pad_testboard);

	bt_init();
#ifdef CONFIG_SND_SOC_ALC5631
    rt5631_gpio_test_init();
#endif
	soc_plat_register_devices(plat_devs, ARRAY_SIZE(plat_devs));

	ext_i2c_register_devices(extend_i2c_devs,ARRAY_SIZE(extend_i2c_devs));
	/*	ns115_system_pm_init();*/
	ns115_system_pm_init();

	pr_info("on2_base = 0x%x, on2_size = 0x%x\n lcd_base = 0x%x, \
			lcd_size = 0x%x\n gpu_size = 0x%x, ump_size = 0x%x\n",\
			nusmart_on2_base(), 
			nusmart_on2_len(),
			nusmart_lcd_base(),
			nusmart_lcd_len(),
			nusmart_mali_len(),
			nusmart_mali_ump_len());
}

static struct gpio_data __initdata test_data = {
	.gpio_ddr = {0x0,0x0,0x0,0x0,0x0},
};

static void __init pad_test_gic_init(void)
{
	gic_init_irq();
#ifdef CONFIG_GENERIC_GPIO
	ns115_init_gpio(&test_data);
#endif
}


MACHINE_START(NS115_PAD_TEST, "NUFRONT-NS115-PAD-TEST")
	.boot_params  = PHYS_OFFSET + 0x00000100,
	.map_io       = ns115_map_io,
	.init_irq     = pad_test_gic_init,
	.timer        = &ns115_timer,
	.init_machine = ns115_pad_test_init,
MACHINE_END
