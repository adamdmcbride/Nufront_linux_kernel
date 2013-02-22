/*
 * charger driver for mp2611
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/power/ns115-battery.h>
#include <linux/power/mp2611-charger.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define STAT2_DELAY_TIME	msecs_to_jiffies(500)

struct mp2611_charger_info {
	int	stat1_gpio;
	int	stat2_gpio;
	int	stat1_irq;
	struct device	*dev;
	struct delayed_work	work;
};

static struct mp2611_charger_info * g_info;

static int mp2611_start_chg(struct ns115_charger * hw_chg, enum ns115_charging_type type)
{
	dev_info(g_info->dev, "start charging!\n");

	return 0;
}

static int mp2611_stop_chg(struct ns115_charger * hw_chg)
{
	dev_info(g_info->dev, "stop charging!\n");

	return 0;
}

static irqreturn_t mp2611_irq(int irq, void *_info)
{
	struct mp2611_charger_info *info = _info;

	disable_irq_nosync(irq);
	schedule_delayed_work(&info->work, STAT2_DELAY_TIME);

	return IRQ_HANDLED;
}

static void mp2611_work(struct work_struct * work)
{
	struct mp2611_charger_info *info = g_info;
	int ret;
	int stat1, stat2;

	ret = gpio_request(info->stat1_gpio, "mp2611_charger");
	if (ret < 0){
		dev_err(info->dev, "request gpio: %d error: %d!", info->stat1_gpio, ret);
		goto err;
	}
	gpio_direction_input(info->stat1_gpio);
	stat1 = gpio_get_value(info->stat1_gpio);
	gpio_free(info->stat1_gpio);

	ret = gpio_request(info->stat2_gpio, "mp2611_charger");
	if (ret < 0){
		dev_err(info->dev, "request gpio: %d error: %d!", info->stat2_gpio, ret);
		goto err;
	}
	gpio_direction_input(info->stat2_gpio);
	stat2 = gpio_get_value(info->stat2_gpio);
	gpio_free(info->stat2_gpio);

	if (stat1 && !stat2){
		ns115_battery_notify_event(CHG_ERROR_EVENT);
	}

	enable_irq(info->stat1_irq);
err:
	return;

}

static struct ns115_charger mp2611_ac_chg = {
	.type = CHG_TYPE_AC,
	.name = "charger_ac",
	.start_charging = mp2611_start_chg,
	.stop_charging = mp2611_stop_chg,
};

static struct ns115_charger mp2611_usb_chg = {
	.type = CHG_TYPE_USB,
	.name = "charger_usb",
	.start_charging = mp2611_start_chg,
	.stop_charging = mp2611_stop_chg,
};

static __devinit int mp2611_charger_probe(struct platform_device *pdev)
{
	struct mp2611_charger_info *info;
	struct mp2611_charger_platform_data *pdata;
	int ret;

	info = kzalloc(sizeof(struct mp2611_charger_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	pdata = pdev->dev.platform_data;
	info->stat1_gpio = pdata->stat1_gpio;
	info->stat2_gpio = pdata->stat2_gpio;
	g_info = info;

	mp2611_ac_chg.chg_current = pdata->ac_chg_current;
	mp2611_usb_chg.chg_current = pdata->usb_chg_current;
	ret = ns115_charger_register(&mp2611_ac_chg);
	if (ret){
		goto out;
	}
	ret = ns115_charger_register(&mp2611_usb_chg);
	if (ret){
		goto err_ac;
	}
	if (info->stat1_gpio){
		info->stat1_irq = gpio_to_irq(info->stat1_gpio);
		ret = request_threaded_irq(info->stat1_irq, NULL, mp2611_irq,
			IRQF_TRIGGER_RISING, "mp2611_stat1_irq", info);
		if (ret){
			dev_err(info->dev, "%s: request irq: %d failed: %d\n",
					__func__, info->stat1_irq, ret);
			goto err_usb;
		}
	}else{
		info->stat1_irq = 0;
	}

	INIT_DELAYED_WORK(&info->work, mp2611_work);

	dev_info(info->dev, "%s is OK!\n", __func__);

	return 0;

err_usb:
	ns115_charger_unregister(&mp2611_usb_chg);
err_ac:
	ns115_charger_unregister(&mp2611_ac_chg);
out:
	kfree(info);
	return ret;
}

static int __devexit mp2611_charger_remove(struct platform_device *pdev)
{
	struct mp2611_charger_info *info = platform_get_drvdata(pdev);

	if (info->stat1_irq){
		free_irq(info->stat1_irq, info);
	}
	kfree(info);
	ns115_charger_unregister(&mp2611_ac_chg);
	ns115_charger_unregister(&mp2611_usb_chg);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver mp2611_charger_driver = {
	.driver		= {
		.name	= "mp2611_charger",
		.owner	= THIS_MODULE,
	},
	.probe		= mp2611_charger_probe,
	.remove		= __devexit_p(mp2611_charger_remove),
};

static int __init mp2611_charger_init(void)
{
	return platform_driver_register(&mp2611_charger_driver);
}
device_initcall(mp2611_charger_init);

static void __exit mp2611_charger_exit(void)
{
	platform_driver_unregister(&mp2611_charger_driver);
}
module_exit(mp2611_charger_exit);


MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("mp2611 charger driver");
MODULE_LICENSE("GPL");
