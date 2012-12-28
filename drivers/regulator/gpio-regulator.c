/*
 * drivers/regulator/gpio-regulator.c
 *
 * Regulator driver for external LDO and DCDC control by GPIO.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/regulator/gpio-regulator.h>

#define ENABLE_DELAY	500//us

struct gpio_regulator {
	int		gpio;
	int		voltage;
	int		delay;
	char	name[10];
	struct regulator_desc	desc;
	struct device		*dev;
};

static int gpio_regulator_enable_time(struct regulator_dev *rdev)
{
	struct gpio_regulator *info = rdev_get_drvdata(rdev);

	return info->delay;
}

static int gpio_reg_is_enabled(struct regulator_dev *rdev)
{
	struct gpio_regulator *info = rdev_get_drvdata(rdev);
	int val;
	int ret;

	ret = gpio_request(info->gpio, rdev->desc->name);
	if (ret < 0) {
		dev_err(&rdev->dev, "gpio: %d request error!\n", info->gpio);
		return ret;
	}
	val = gpio_get_value_cansleep(info->gpio);
	gpio_free(info->gpio);
	dev_dbg(info->dev, "%s val: 0x%x\n", __func__, val);

	return val;
}

static int gpio_reg_set_value(struct gpio_regulator *info, int value)
{
	int ret;

	ret = gpio_request(info->gpio, info->name);
	if (ret < 0) {
		dev_err(info->dev, "gpio: %d request error!\n", info->gpio);
		return ret;
	}
	ret = gpio_direction_output(info->gpio, !!value);
	gpio_free(info->gpio);

	return ret;
}

static int gpio_reg_enable(struct regulator_dev *rdev)
{
	struct gpio_regulator *info = rdev_get_drvdata(rdev);
	int ret;

	dev_dbg(info->dev, "%s\n", __func__);
	ret = gpio_reg_set_value(info, 1);

	return ret;
}

static int gpio_reg_disable(struct regulator_dev *rdev)
{
	struct gpio_regulator *info = rdev_get_drvdata(rdev);

	return gpio_reg_set_value(info, 0);
}

static int gpio_list_voltage(struct regulator_dev *rdev, unsigned index)
{
	struct gpio_regulator *info = rdev_get_drvdata(rdev);

	return info->voltage;
}

static int gpio_get_voltage(struct regulator_dev *rdev)
{
	struct gpio_regulator *info = rdev_get_drvdata(rdev);

	return info->voltage;
}

static struct regulator_ops gpio_ops = {
	.list_voltage	= gpio_list_voltage,
	.get_voltage	= gpio_get_voltage,
	.enable		= gpio_reg_enable,
	.disable	= gpio_reg_disable,
	.is_enabled	= gpio_reg_is_enabled,
	.enable_time	= gpio_regulator_enable_time,
};

static int gpio_regulator_preinit(struct gpio_regulator *info,
		struct gpio_regulator_platform_data *gpio_pdata)
{
	int ret = 0;

	info->gpio = gpio_pdata->gpio;
	info->voltage = gpio_pdata->regulator.constraints.min_uV;
	info->delay = ENABLE_DELAY;
	sprintf(info->name, "GPIO_%d", gpio_pdata->gpio);
	info->desc.name = info->name;
	info->desc.id = gpio_pdata->gpio;
	info->desc.n_voltages = 0;
	info->desc.ops = &gpio_ops;
	info->desc.type = REGULATOR_VOLTAGE;
	info->desc.owner = THIS_MODULE;

	if (!gpio_pdata->init_apply){
		return 0;
	}

	if (gpio_pdata->init_enable){
		ret = gpio_reg_set_value(info, 1);
	}else{
		ret = gpio_reg_set_value(info, 0);
	}
	if (ret < 0){
		dev_err(info->dev, "Not able to %s rail %d err %d\n",
				(gpio_pdata->init_enable) ? "enable" : "disable",
				info->desc.id, ret);
	}

	return ret;
}

static int __devinit gpio_regulator_probe(struct platform_device *pdev)
{
	struct gpio_regulator *info = NULL;
	struct regulator_dev *rdev;
	struct gpio_regulator_platform_data *tps_pdata;
	int err;

	tps_pdata = pdev->dev.platform_data;
	dev_dbg(&pdev->dev, "Probing regulator %d\n", tps_pdata->gpio);

	info = kzalloc(sizeof(struct gpio_regulator), GFP_KERNEL);
	if (!info){
		return -ENOMEM;
	}
	info->dev = &pdev->dev;

	err = gpio_regulator_preinit(info, tps_pdata);
	if (err) {
		dev_err(&pdev->dev, "Fail in pre-initialisation\n");
		 goto out;
	}
	rdev = regulator_register(&info->desc, &pdev->dev,
			&tps_pdata->regulator, info);
	if (IS_ERR_OR_NULL(rdev)) {
		dev_err(&pdev->dev, "failed to register regulator %s\n",
				info->desc.name);
		err = PTR_ERR(rdev);
		goto out;
	}
	platform_set_drvdata(pdev, rdev);

	return 0;
out:
	kfree(info);
	return err;
}

static int __devexit gpio_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	struct gpio_regulator *info = rdev_get_drvdata(rdev);

	regulator_unregister(rdev);
	kfree(info);

	return 0;
}

static struct platform_driver gpio_regulator_driver = {
	.driver	= {
		.name	= "gpio-regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= gpio_regulator_probe,
	.remove		= __devexit_p(gpio_regulator_remove),
};

static int __init gpio_regulator_init(void)
{
	return platform_driver_register(&gpio_regulator_driver);
}
device_initcall(gpio_regulator_init);

static void __exit gpio_regulator_exit(void)
{
	platform_driver_unregister(&gpio_regulator_driver);
}
module_exit(gpio_regulator_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("GPIO regulator driver");
MODULE_ALIAS("platform:gpio-regulator");
MODULE_LICENSE("GPL");
