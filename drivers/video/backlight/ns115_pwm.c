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


struct ns115_backlight {
	int powermode;
	int current_intensity;

	struct device *dev;
	struct mutex enable_lock;
	struct early_suspend early_suspend;
	struct ns115_backlight_config *pdata;
};
struct ns115_backlight_config {
         int default_intensity;
         int (*set_power)(struct device *dev, int state);
};

static void inline ns115bl_send_intensity(int intensity)
{
	int temp;
	unsigned int ns115_base_addr_pwm = (unsigned int) __io_address(NS115_AUX_BASE);
	printk(KERN_INFO"%s %d\n", __func__, intensity);

    //temp = readl(ns115_base_addr_pwm);
    temp = 0x28420; // 1K Frequency
	temp &= (~0x1f);
	temp = temp | (((intensity*3) & 0x1f));
	//printk("=======value :%x\n", temp);
	//printk("=======intensity value:%d\n", (intensity*25/30) & 0x1f);
	writel(temp, ns115_base_addr_pwm);
}

static void inline bl_delay(int ms)
{
	if(in_atomic())
		mdelay(ms);
	else
		msleep(ms);
}
static void inline ns115bl_send_enable(int enable)
{
	if(enable != 0)
	{
		gpio_request(12, "LCD-ON");
		gpio_request(13, "BL-ON");
#ifdef CONFIG_MACH_NS115_PAD_REF
		gpio_request(102, "LVDS-SHUTDOWN"); //gpio C[30]
#endif 
		gpio_direction_output(12,1);//LCD_ON
		bl_delay(25);
#ifdef CONFIG_MACH_NS115_PAD_REF
		gpio_direction_output(102,1);
		bl_delay(250);
#endif 
		gpio_direction_output(13,1);
#ifdef CONFIG_MACH_NS115_PAD_REF
		gpio_free(102); 
#endif
		gpio_free(13);
		gpio_free(12);
	} else {
		gpio_request(12, "LCD-ON");
		gpio_request(13, "BL-ON");
#ifdef CONFIG_MACH_NS115_PAD_REF
		gpio_request(102, "LVDS-SHUTDOWN"); //gpio C[30]
#endif 
		gpio_direction_output(13,0);
		bl_delay(250);
#ifdef CONFIG_MACH_NS115_PAD_REF
		gpio_direction_output(102,0);
		bl_delay(25);
#endif 
		gpio_direction_output(12,0);//LCD_ON
#ifdef CONFIG_MACH_NS115_PAD_REF
		gpio_free(102); 
#endif
		gpio_free(13);
		gpio_free(12);

	}
//	ns115_writeb(enable, ns115_PWL_CLK_ENABLE);
}

static void ns115bl_blank(struct ns115_backlight *bl, int mode)
{
	if (bl->pdata->set_power)
		bl->pdata->set_power(bl->dev, mode);

	switch (mode) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		mutex_lock(&bl->enable_lock);
		ns115bl_send_intensity(0);
		ns115bl_send_enable(0);
		mutex_unlock(&bl->enable_lock);
		break;

	case FB_BLANK_UNBLANK:
		mutex_lock(&bl->enable_lock);
		ns115bl_send_intensity(bl->current_intensity);
		ns115bl_send_enable(1);
		mutex_unlock(&bl->enable_lock);
		break;
	}
}

#if (defined CONFIG_PM) && (!defined CONFIG_HAS_EARLYSUSPEND)
static int ns115bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct ns115_backlight *bl = dev_get_drvdata(&dev->dev);

	ns115bl_blank(bl, FB_BLANK_POWERDOWN);
	return 0;
}

static int ns115bl_resume(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct ns115_backlight *bl = dev_get_drvdata(&dev->dev);

	ns115bl_blank(bl, FB_BLANK_UNBLANK);
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

	ns115bl_blank(bl, FB_BLANK_NORMAL);
}

static void ns115bl_late_resume(struct early_suspend *h)
{
	struct ns115_backlight *bl;
	bl = container_of(h, struct ns115_backlight, early_suspend);

	ns115bl_blank(bl, FB_BLANK_UNBLANK);
}
#endif

static int ns115bl_set_power(struct backlight_device *dev, int state)
{
	struct ns115_backlight *bl = dev_get_drvdata(&dev->dev);

	ns115bl_blank(bl, state);
	bl->powermode = state;

	return 0;
}

static int ns115bl_update_status(struct backlight_device *dev)
{
	struct ns115_backlight *bl = dev_get_drvdata(&dev->dev);

	if (bl->current_intensity != dev->props.brightness) {
		if (bl->powermode == FB_BLANK_UNBLANK){
			ns115bl_send_intensity(dev->props.brightness);
		}
		bl->current_intensity = dev->props.brightness;
	}

	if (dev->props.fb_blank != bl->powermode)
		ns115bl_set_power(dev, dev->props.fb_blank);

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
	unsigned int ns115_base_addr_pwm = (unsigned int) __io_address(NS115_AUX_BASE);
	struct backlight_device *dev;
	struct ns115_backlight *bl;
	struct ns115_backlight_config *pdata = pdev->dev.platform_data;

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

	mutex_init(&bl->enable_lock);
    //temp = readl(ns115_base_addr_pwm);
    temp = 0x28420; // 1K Frequency
    writel(temp, ns115_base_addr_pwm);

	bl->powermode = FB_BLANK_POWERDOWN;
	bl->current_intensity = 0x0;

	bl->pdata = pdata;
	bl->dev = &pdev->dev;

	platform_set_drvdata(pdev, dev);

	dev->props.fb_blank = FB_BLANK_UNBLANK;
	dev->props.max_brightness = NS115BL_MAX_INTENSITY;
	dev->props.brightness = pdata->default_intensity;
	backlight_update_status(dev);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	bl->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	bl->early_suspend.suspend = ns115bl_early_suspend;
	bl->early_suspend.resume = ns115bl_late_resume;
	register_early_suspend(&bl->early_suspend);
	#endif

	//printk(KERN_INFO "NS115 LCD backlight initialised\n");
	printk("NS115 LCD backlight initialised\n");

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
MODULE_DESCRIPTION("ns115 LCD Backlight driver");
MODULE_LICENSE("GPL");


