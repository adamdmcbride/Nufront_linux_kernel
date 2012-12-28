/*
 * LED driver for TPS80032 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/mfd/tps80032.h>

#define TPS80032_LED_PWM_CTRL1	2, 0xF4
#define TPS80032_LED_PWM_CTRL2	2, 0xF5

#define CURR_LED_MASK	0x30
#define CURR_LED_POS	4
#define LED_IN_MASK		0x0c
#define LED_IN_POS		2
#define LED_MODE_MASK	0x03
#define LED_MODE_POS	0

#define TPS80032_LED_FULL	0xff

struct tps80032_leds_info {
	struct device *dev;
	struct led_classdev cdev;
	const char *name;
	int brightness;
	enum tps80032_leds_current cur;
	enum tps80032_leds_source source;
	enum tps80032_leds_mode mode;
};

static int tps80032_leds_on(struct led_classdev *cdev)
{
	int ret;
	struct tps80032_leds_info *info = 
		container_of(cdev, struct tps80032_leds_info, cdev);

	ret = tps80032_reg_update(TPS80032_LED_PWM_CTRL2,
			info->mode << LED_MODE_POS, LED_MODE_MASK);

	return ret;
}

static int tps80032_leds_off(struct led_classdev *cdev)
{
	int ret;

	ret = tps80032_reg_update(TPS80032_LED_PWM_CTRL2,
			TPS80032_LEDS_OFF << LED_MODE_POS, LED_MODE_MASK);

	return ret;
}

static int tps80032_leds_brightness(struct led_classdev *cdev, int value)
{
	int ret;

	ret = tps80032_write(TPS80032_LED_PWM_CTRL1, value);

	return ret;
}

static void tps80032_leds_set(struct led_classdev *cdev,
			   enum led_brightness value)
{
	struct tps80032_leds_info *info = 
		container_of(cdev, struct tps80032_leds_info, cdev);

	if (info->brightness == value){
		return;
	}
	switch (value){
		case LED_OFF:
			tps80032_leds_off(cdev);
			break;
		case LED_HALF:
			tps80032_leds_brightness(cdev, value);
			if (info->brightness <= 0){
				tps80032_leds_on(cdev);
			}
			break;
		case LED_FULL:
			tps80032_leds_brightness(cdev, TPS80032_LED_FULL);
			if (info->brightness <= 0){
				tps80032_leds_on(cdev);
			}
			break;
	}

	info->brightness = value;
	return;
}

static int tps80032_leds_init(struct tps80032_leds_info *info)
{
	int ret;

	ret = tps80032_reg_update(TPS80032_LED_PWM_CTRL2,
			info->cur << CURR_LED_POS, CURR_LED_MASK);
	ret |= tps80032_reg_update(TPS80032_LED_PWM_CTRL2,
			info->source << LED_IN_POS, LED_IN_MASK);

	return ret;
}

static int tps80032_leds_probe(struct platform_device *pdev)
{
	struct tps80032_leds_platform_data *pdata;
	struct tps80032_leds_info *info;
	int ret;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "No platform data!\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct tps80032_leds_info), GFP_KERNEL);
	if (info == NULL){
		return -ENOMEM;
	}
	info->dev = &pdev->dev;
	info->cur = pdata->cur;
	info->source = pdata->source;
	info->mode = pdata->mode;
	info->name = pdata->name;
	dev_set_drvdata(&pdev->dev, info);

	info->brightness = -1;
	info->cdev.name = info->name;
	info->cdev.brightness_set = tps80032_leds_set;

	ret = led_classdev_register(info->dev, &info->cdev);
	if (ret < 0) {
		dev_err(info->dev, "Failed to register LED: %d\n", ret);
		goto out;
	}
	ret = tps80032_leds_init(info);
	if (ret < 0){
		dev_err(info->dev, "tps80032 leds init failed!\n");
		goto out;
	}
	return 0;
out:
	kfree(info);
	return ret;
}

static int tps80032_leds_remove(struct platform_device *pdev)
{
	struct tps80032_leds_info *info = platform_get_drvdata(pdev);

	led_classdev_unregister(&info->cdev);
	kfree(info);

	return 0;
}

static struct platform_driver tps80032_leds_driver = {
	.driver	= {
		.name	= "leds-tps80032",
		.owner	= THIS_MODULE,
	},
	.probe	= tps80032_leds_probe,
	.remove	= tps80032_leds_remove,
};

static int __devinit tps80032_leds_dev_init(void)
{
	return platform_driver_register(&tps80032_leds_driver);
}
module_init(tps80032_leds_dev_init);

static void __devexit tps80032_leds_exit(void)
{
	platform_driver_unregister(&tps80032_leds_driver);
}
module_exit(tps80032_leds_exit);

MODULE_DESCRIPTION("LED driver for TPS80032 PMIC");
MODULE_AUTHOR("jianguo Wu<jianguo.wu@nufront.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-tps80032");
