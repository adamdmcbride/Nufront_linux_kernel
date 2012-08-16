/*
 * Backlight driver for ns2816 based boards.
 *
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include <mach/gpio.h>
#include <mach/hardware.h>

#define SENSOR_CTRL   0x64
#define FAN_CTRL 			0x68
#define NS2816BL_MAX_INTENSITY		0xff
#define NS2816_SCM_BASE_PWM  0x051c0000

struct ns2816_backlight {
	int powermode;
	int current_intensity;

	struct device *dev;
	struct ns2816_backlight_config *pdata;
};
struct ns2816_backlight_config {
         int default_intensity;
         int (*set_power)(struct device *dev, int state);
};

static void inline ns2816bl_send_intensity(int intensity)
{
	int temp;
	unsigned int scm_base_addr_pwm = (unsigned int) __io_address(NS2816_SCM_BASE_PWM);
	printk(KERN_EMERG"%s %d\n", __func__, intensity);

	temp = readl(scm_base_addr_pwm + FAN_CTRL);
	temp &= (~0xff00);
	intensity = temp | (((intensity * 25) & 0xff) << 8);
	writel(intensity, scm_base_addr_pwm + FAN_CTRL);
}

static void inline ns2816bl_send_enable(int enable)
{
	if(enable != 0)
	{
		gpio_request(21, "LCD-ON");
		gpio_request(15, "BL-ON");
		gpio_direction_output(15,1);
		gpio_direction_output(21,1);
		gpio_free(15);
		gpio_free(21);
	} else {
		gpio_request(21, "LCD-ON");
		gpio_request(15, "BL-ON");
		gpio_direction_output(15,0);
		gpio_direction_output(21,0);
		gpio_free(15);
		gpio_free(21);
	}
//	ns2816_writeb(enable, NS2816_PWL_CLK_ENABLE);
}

static void ns2816bl_blank(struct ns2816_backlight *bl, int mode)
{
	if (bl->pdata->set_power)
		bl->pdata->set_power(bl->dev, mode);

	switch (mode) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		ns2816bl_send_intensity(0);
		ns2816bl_send_enable(0);
		break;

	case FB_BLANK_UNBLANK:
		ns2816bl_send_intensity(bl->current_intensity);
		ns2816bl_send_enable(1);
		break;
	}
}

#ifdef CONFIG_PM
static int ns2816bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct ns2816_backlight *bl = dev_get_drvdata(&dev->dev);

	ns2816bl_blank(bl, FB_BLANK_POWERDOWN);
	return 0;
}

static int ns2816bl_resume(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct ns2816_backlight *bl = dev_get_drvdata(&dev->dev);

	ns2816bl_blank(bl, bl->powermode);
	return 0;
}
#else
#define ns2816bl_suspend	NULL
#define ns2816bl_resume	NULL
#endif

static int ns2816bl_set_power(struct backlight_device *dev, int state)
{
	struct ns2816_backlight *bl = dev_get_drvdata(&dev->dev);

	ns2816bl_blank(bl, state);
	bl->powermode = state;

	return 0;
}

static int ns2816bl_update_status(struct backlight_device *dev)
{
	struct ns2816_backlight *bl = dev_get_drvdata(&dev->dev);

	if (bl->current_intensity != dev->props.brightness) {
		if (bl->powermode == FB_BLANK_UNBLANK){
			ns2816bl_send_intensity(dev->props.brightness);
		}
		bl->current_intensity = dev->props.brightness;
	}

	if (dev->props.fb_blank != bl->powermode)
		ns2816bl_set_power(dev, dev->props.fb_blank);

	return 0;
}

static int ns2816bl_get_intensity(struct backlight_device *dev)
{
	struct ns2816_backlight *bl = dev_get_drvdata(&dev->dev);
	return bl->current_intensity;
}

static const struct backlight_ops ns2816bl_ops = {
	.get_brightness = ns2816bl_get_intensity,
	.update_status  = ns2816bl_update_status,
};

static int ns2816bl_probe(struct platform_device *pdev)
{
	int temp;
	unsigned int scm_base_addr_pwm = (unsigned int) __io_address(NS2816_SCM_BASE_PWM);
	struct backlight_device *dev;
	struct ns2816_backlight *bl;
	struct ns2816_backlight_config *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -ENXIO;

	bl = kzalloc(sizeof(struct ns2816_backlight), GFP_KERNEL);
	if (unlikely(!bl))
		return -ENOMEM;

	dev = backlight_device_register("ns-backlight", &pdev->dev, bl, &ns2816bl_ops, NULL);
	if (IS_ERR(dev)) {
		kfree(bl);
		return PTR_ERR(dev);
	}
    
    	temp = readl(scm_base_addr_pwm + SENSOR_CTRL);
       	temp |= 0xe;
        writel(temp, scm_base_addr_pwm + SENSOR_CTRL);

	bl->powermode = FB_BLANK_POWERDOWN;
	bl->current_intensity = 0x0;

	bl->pdata = pdata;
	bl->dev = &pdev->dev;

	platform_set_drvdata(pdev, dev);


	dev->props.fb_blank = FB_BLANK_UNBLANK;
	dev->props.max_brightness = NS2816BL_MAX_INTENSITY;
	dev->props.brightness = pdata->default_intensity;
	backlight_update_status(dev);

	printk(KERN_INFO "NS2816 LCD backlight initialised\n");

	return 0;
}

static int ns2816bl_remove(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct ns2816_backlight *bl = dev_get_drvdata(&dev->dev);

	backlight_device_unregister(dev);
	kfree(bl);

	return 0;
}

static struct platform_driver ns2816bl_driver = {
	.probe		= ns2816bl_probe,
	.remove		= ns2816bl_remove,
	.suspend	= ns2816bl_suspend,
	.resume		= ns2816bl_resume,
	.driver		= {
		.name	= "ns2816-bl",
	},
};

static int __init ns2816bl_init(void)
{
	return platform_driver_register(&ns2816bl_driver);
}

static void __exit ns2816bl_exit(void)
{
	platform_driver_unregister(&ns2816bl_driver);
}

module_init(ns2816bl_init);
module_exit(ns2816bl_exit);

MODULE_AUTHOR("hunianhang <nianhang.hu@nufront.com>");
MODULE_DESCRIPTION("ns2816 LCD Backlight driver");
MODULE_LICENSE("GPL");


