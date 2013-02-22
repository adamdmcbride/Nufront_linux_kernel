/*
 * Backlight driver for NS115 based boards.
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
#include <linux/delay.h>
#include <linux/earlysuspend.h>

#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/board-ns115.h>

#define NS115BL_MAX_INTENSITY	10
static unsigned int ns115_base_addr_pwm;

struct ns115_backlight {
	int powermode;
	int current_intensity;

	struct device *dev;
	struct early_suspend early_suspend;
	struct ns115_backlight_config *pdata;
};

struct ns115_backlight_config {
	int default_intensity;
	int (*set_power)(struct device *dev, int state);
};

static int brightness[] = {
			0, //0
			13,//1
			14,//2
			15,//3
			16,//4
			17,//5
			18,//6
			19,//7
			20,//8
			21,//9
			30 //10
};

static void inline ns115bl_send_intensity(int intensity)
{
	int temp;
	printk(KERN_INFO"%s %d\n", __func__, intensity);

	if (intensity>10 || intensity<0) {
		return;
	}

	temp = readl(ns115_base_addr_pwm);
	temp &= (~0x1f);
	temp = temp | (((brightness[intensity]) & 0x1f));
	writel(temp, ns115_base_addr_pwm);
}

#if (defined CONFIG_PM) && (!defined CONFIG_HAS_EARLYSUSPEND)
static int ns115bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct ns115_backlight *bl = dev_get_drvdata(&dev->dev);

	ns115bl_send_intensity(0);
	writel(0x0, ns115_base_addr_pwm);
	return 0;
}

static int ns115bl_resume(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct ns115_backlight *bl = dev_get_drvdata(&dev->dev);

	writel(0x28820, ns115_base_addr_pwm);
	ns115bl_send_intensity(bl->current_intensity);
	return 0;
}
#else
#define ns115bl_suspend	NULL
#define ns115bl_resume	NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ns115bl_early_suspend(struct early_suspend *h)
{

	struct ns115_backlight *bl;
	bl = container_of(h, struct ns115_backlight, early_suspend);

	ns115bl_send_intensity(0);
	writel(0x0, ns115_base_addr_pwm);
}

static void ns115bl_late_resume(struct early_suspend *h)
{
	struct ns115_backlight *bl;
	bl = container_of(h, struct ns115_backlight, early_suspend);

	writel(0x28820, ns115_base_addr_pwm);
	ns115bl_send_intensity(bl->current_intensity);
}
#endif

static int ns115bl_update_status(struct backlight_device *dev)
{
	struct ns115_backlight *bl = dev_get_drvdata(&dev->dev);

	if (bl->current_intensity != dev->props.brightness) {
		ns115bl_send_intensity(dev->props.brightness);
		bl->current_intensity = dev->props.brightness;
	}

	return 0;
}

static int ns115bl_get_intensity(struct backlight_device *dev)
{
	struct ns115_backlight *bl = dev_get_drvdata(&dev->dev);
	return bl->current_intensity;
}

static const struct backlight_ops ns115bl_ops = {
	.get_brightness = ns115bl_get_intensity,
	.update_status  = ns115bl_update_status,
};

static int ns115bl_probe(struct platform_device *pdev)
{
	int temp;
	struct backlight_device *dev;
	struct ns115_backlight *bl;
	struct ns115_backlight_config *pdata = pdev->dev.platform_data;
	ns115_base_addr_pwm = (unsigned int) __io_address(NS115_AUX_BASE);

	if (!pdata)
		return -ENXIO;

	bl = kzalloc(sizeof(struct ns115_backlight), GFP_KERNEL);
	if (unlikely(!bl))
		return -ENOMEM;

	dev = backlight_device_register("ns-backlight", &pdev->dev, bl, &ns115bl_ops, NULL);
	if (IS_ERR(dev)) {
		kfree(bl);
		return PTR_ERR(dev);
	}

	temp = 0x28820; // 500Hz Frequency
	writel(temp, ns115_base_addr_pwm);

	bl->powermode = FB_BLANK_POWERDOWN;
	bl->current_intensity = 0x0;

	bl->pdata = pdata;
	bl->dev = &pdev->dev;

	platform_set_drvdata(pdev, dev);

	dev->props.fb_blank = FB_BLANK_UNBLANK;
	dev->props.max_brightness = NS115BL_MAX_INTENSITY;
	dev->props.brightness = pdata->default_intensity;
	bl->current_intensity = dev->props.brightness;
	ns115bl_send_intensity(bl->current_intensity);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	bl->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	bl->early_suspend.suspend = ns115bl_early_suspend;
	bl->early_suspend.resume = ns115bl_late_resume;
	register_early_suspend(&bl->early_suspend);
	#endif

	printk(KERN_INFO "NS115M LCD backlight initialised\n");
	return 0;
}

static int ns115bl_remove(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct ns115_backlight *bl = dev_get_drvdata(&dev->dev);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&bl->early_suspend);
	#endif

	backlight_device_unregister(dev);
	kfree(bl);

	return 0;
}

static struct platform_driver ns115bl_driver = {
	.probe		= ns115bl_probe,
	.remove		= ns115bl_remove,
	.suspend	= ns115bl_suspend,
	.resume		= ns115bl_resume,
	.driver		= {
		.name	= "ns115-bl",
	},
};

static int __init ns115bl_init(void)
{
	return platform_driver_register(&ns115bl_driver);
}

static void __exit ns115bl_exit(void)
{
	platform_driver_unregister(&ns115bl_driver);
}

module_init(ns115bl_init);
module_exit(ns115bl_exit);

MODULE_AUTHOR("Nufront LTD");
MODULE_DESCRIPTION("ns115m LCD Backlight driver");
MODULE_LICENSE("GPL");

