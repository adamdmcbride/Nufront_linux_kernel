/*
 *the usb divice and host switch driver of SGM7227
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/usb/sgm7227_switch.h>

#define ID_DELAY_TIME	msecs_to_jiffies(500)

//#define SGM7227_DEBUG
#ifdef SGM7227_DEBUG
#define  PDBG(dev, format,...)	\
	dev_err(dev, format, ##__VA_ARGS__)
#define  PINFO(dev, format,...)	\
	dev_err(dev, format, ##__VA_ARGS__)
#else
#define  PDBG(dev, format,...)	do{}while(0)
#define  PINFO(dev, format,...)	\
	dev_info(dev, format, ##__VA_ARGS__)
#endif

struct sgm7227_info {
	struct device *dev;
	int id_gpio;
	int id_irq;
	int ctl_gpio;
	int usb_5v_gpio;
	int host_channel;/*host channel, 1=HSD1, 2=HSD2*/
	int host_online;
	struct delayed_work work;
};

enum sgm7227_mode {
	SGM7227_HOST = 1,
	SGM7227_DEVICE
};

static int sgm7227_switch_ctl(struct sgm7227_info *info, enum sgm7227_mode mode)
{
	gpio_direction_output(info->ctl_gpio, mode != info->host_channel);
	gpio_direction_output(info->usb_5v_gpio, mode == SGM7227_HOST);

	return 0;
}

static irqreturn_t sgm7227_irq(int irq, void *_info)
{
	struct sgm7227_info * info = _info;

	disable_irq_nosync(info->id_irq);
	schedule_delayed_work(&info->work, ID_DELAY_TIME);

	return IRQ_HANDLED;
}

static void sgm7227_work(struct work_struct * work)
{
	struct sgm7227_info *info = container_of(work, struct sgm7227_info, work.work);
	int ret, val, online;

	val = gpio_get_value(info->id_gpio);
	online = !val;
	if (info->host_online == online){
		PINFO(info->dev, "%s the state isn't changed!\n", __func__);
		enable_irq(info->id_irq);
		return;
	}

	info->host_online = online;
	free_irq(info->id_irq, info);
	if (info->host_online){
		sgm7227_switch_ctl(info, SGM7227_HOST);
		ret = request_threaded_irq(info->id_irq, NULL, sgm7227_irq,
			IRQF_TRIGGER_RISING, "sgm7227_switch", info);
		PINFO(info->dev, "%s: host mode!\n", __func__);
	}else{
		sgm7227_switch_ctl(info, SGM7227_DEVICE);
		ret = request_threaded_irq(info->id_irq, NULL, sgm7227_irq,
			IRQF_TRIGGER_FALLING, "sgm7227_switch", info);
		PINFO(info->dev, "%s: device mode!\n", __func__);
	}
	if (ret < 0) {
		dev_err(info->dev, "%s: request irq: %d failed: %d\n",
				__func__, info->id_irq, ret);
	}

	return;
}

static __devinit int sgm7227_probe(struct platform_device *pdev)
{
	struct sgm7227_info *info;
	struct sgm7227_platform_data *pdata;
	int ret, val;

	info = kzalloc(sizeof(struct sgm7227_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	pdata = pdev->dev.platform_data;
	info->ctl_gpio = pdata->ctl_gpio;
	info->usb_5v_gpio = pdata->usb_5v_gpio;
	info->host_channel = pdata->host_channel;
	info->id_gpio = pdata->usb_id_gpio;
	info->id_irq = gpio_to_irq(info->id_gpio);

	ret = gpio_request(info->ctl_gpio, "sgm7227_ctl");
	if (ret){
		dev_err(info->dev, "%s: gpip: %d request failed: %d\n",
				__func__, info->ctl_gpio, ret);
		goto out;
	}

	ret = gpio_request(info->usb_5v_gpio, "sgm7227_usb_5v");
	if (ret){
		dev_err(info->dev, "%s: gpip: %d request failed: %d\n",
				__func__, info->usb_5v_gpio, ret);
		goto ctl_gpio;
	}

	ret = gpio_request(info->id_gpio, "sgm7227_id");
	if (ret){
		dev_err(info->dev, "%s: gpip: %d request failed: %d\n",
				__func__, info->id_gpio, ret);
		goto usb_5v_gpio;
	}
	gpio_direction_input(info->id_gpio);
	val = gpio_get_value(info->id_gpio);
	if (val){
		info->host_online = 0;
		dev_info(info->dev, "%s: device mode!\n", __func__);
		sgm7227_switch_ctl(info, SGM7227_DEVICE);
		ret = request_threaded_irq(info->id_irq, NULL, sgm7227_irq,
			IRQF_TRIGGER_FALLING, "sgm7227_switch", info);
	}else{
		info->host_online = 1;
		dev_info(info->dev, "%s: host mode!\n", __func__);
		sgm7227_switch_ctl(info, SGM7227_HOST);
		ret = request_threaded_irq(info->id_irq, NULL, sgm7227_irq,
			IRQF_TRIGGER_RISING, "sgm7227_switch", info);
	}
	if (ret){
		dev_err(info->dev, "%s: request irq: %d failed: %d\n",
				__func__, info->id_irq, ret);
		goto id_gpio;
	}

	INIT_DELAYED_WORK(&info->work, sgm7227_work);

	dev_info(info->dev, "%s is OK!\n", __func__);

	return 0;


id_gpio:
	gpio_free(info->id_gpio);
usb_5v_gpio:
	gpio_free(info->usb_5v_gpio);
ctl_gpio:
	gpio_free(info->ctl_gpio);
out:
	kfree(info);
	return ret;
}

static int __devexit sgm7227_remove(struct platform_device *pdev)
{
	struct sgm7227_info *info = platform_get_drvdata(pdev);

	if (info->id_irq){
		free_irq(info->id_irq, info);
	}
	gpio_free(info->id_gpio);
	gpio_free(info->usb_5v_gpio);
	gpio_free(info->ctl_gpio);
	kfree(info);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver sgm7227_driver = {
	.driver		= {
		.name	= "sgm7227_switch",
		.owner	= THIS_MODULE,
	},
	.probe		= sgm7227_probe,
	.remove		= __devexit_p(sgm7227_remove),
};

static int __init sgm7227_init(void)
{
	return platform_driver_register(&sgm7227_driver);
}
device_initcall(sgm7227_init);

static void __exit sgm7227_exit(void)
{
	platform_driver_unregister(&sgm7227_driver);
}
module_exit(sgm7227_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("SGM7227 USB switch driver");
MODULE_LICENSE("GPL");
