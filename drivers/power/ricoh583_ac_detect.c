/*
 * AC charger detect driver for ricoh583 PMIC 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mfd/ricoh583.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/power/ns115-battery.h>

#define RICOH583_USB_CHARGER_DETECT		1

#if RICOH583_USB_CHARGER_DETECT
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/delay.h>
#endif

#define RICOH583_PWRMON1_REG 0x1B
#define RICOH583_PWRIRSL1_REG 0x1C

struct ricoh583_ac_detect_info {
	struct device      *dev;
	struct work_struct  work;
#if RICOH583_USB_CHARGER_DETECT
	struct delayed_work usb_work;
	int			usb_gpio;
	int			usb_irq;
	int			usb_online;
#endif
	int			irq;
	int			online;
};

static struct ricoh583_ac_detect_info * g_info;

int n3_otg_plug()
{
#ifdef RICOH583_USB_CHARGER_DETECT
	if(g_info != NULL) {
		return g_info->usb_online;
	}
#endif
	return 0;		
}
EXPORT_SYMBOL(n3_otg_plug);

static void __ricoh583_charger_plug(enum ns115_charger_type type, int ext)
{
	if (!g_info->online){
		g_info->online = 1;
		ns115_charger_plug(type);
	}else if(ext){
		ns115_charger_plug(type);
	}
}

void ricoh583_charger_plug(enum ns115_charger_type type)
{
	__ricoh583_charger_plug(type, 1);
}

void ricoh583_charger_unplug(void)
{
	if (g_info->online){
		g_info->online = 0;
		ns115_charger_unplug();
	}
}

static void ricoh583_ac_detect_irq_work(struct work_struct * work)
{
	uint8_t val;
	int ret;

	ret = ricoh583_read(g_info->dev->parent, RICOH583_PWRMON1_REG, &val);
	if (ret){
		dev_err(g_info->dev, "%s i2c read error: %d\n", __func__, ret);
		return ;
	}
	if (val & (1 << 1)){
		dev_info(g_info->dev, "AC charger is plugged!\n");
		__ricoh583_charger_plug(CHG_TYPE_AC, 0);
		return ;
	}else{
		dev_info(g_info->dev, "AC charger is unplugged!\n");
		ricoh583_charger_unplug();
		return ;
	}
}

static irqreturn_t ricoh583_ac_detect_irq(int irq, void *_info)
{
	struct ricoh583_ac_detect_info * info = _info;

	schedule_work(&info->work);

	return IRQ_HANDLED;
}

#if RICOH583_USB_CHARGER_DETECT
#define VBUS_DELAY_TIME		msecs_to_jiffies(500)

static irqreturn_t ricoh583_usb_irq(int irq, void *_info)
{
	struct ricoh583_ac_detect_info * info = _info;

	disable_irq_nosync(info->usb_irq);
	schedule_delayed_work(&info->usb_work, VBUS_DELAY_TIME);

	return IRQ_HANDLED;
}

#ifdef CONFIG_USB_DESIGN_WARE
extern void dwc_otg_usb_det(void);
#endif

#ifdef CONFIG_FSA880_USB_DETECT
extern void fsa880_int_func(void);
#endif

static void ricoh583_usb_work(struct work_struct * work)
{
	int ret;
	int val;

	ret = gpio_request(g_info->usb_gpio, "usb_detect");
	if (ret < 0){
		enable_irq(g_info->usb_irq);
		dev_err(g_info->dev, "%s: gpio %d request failed!\n", __func__, g_info->usb_gpio);
		return;
	}
	gpio_direction_input(g_info->usb_gpio);

	val = gpio_get_value(g_info->usb_gpio);
	gpio_free(g_info->usb_gpio);
	val = !val;
	if (g_info->usb_online == val){
		dev_info(g_info->dev, "%s the state isn't changed!\n", __func__);
		enable_irq(g_info->usb_irq);
		return;
	}
	
	g_info->usb_online = val;
	free_irq(g_info->usb_irq, g_info);
	if (g_info->usb_online){
		ret = request_threaded_irq(g_info->usb_irq, NULL, ricoh583_usb_irq,
			IRQF_TRIGGER_RISING, "usb_charger_detect", g_info);
		dev_info(g_info->dev, "USB charger is plugged!\n");
		ricoh583_charger_plug(CHG_TYPE_USB);
	}else{
		ret = request_threaded_irq(g_info->usb_irq, NULL, ricoh583_usb_irq,
			IRQF_TRIGGER_FALLING, "usb_charger_detect", g_info);
		dev_info(g_info->dev, "USB charger is unplugged!\n");
		ricoh583_charger_unplug();
		msleep(500);
		schedule_work(&g_info->work);
#ifdef CONFIG_USB_DESIGN_WARE
		dwc_otg_usb_det();
#endif
	}

#ifdef CONFIG_FSA880_USB_DETECT
	fsa880_int_func();
#endif

	if (ret < 0) {
		dev_err(g_info->dev, "Can't get %d IRQ for ricoh583 usb charger detect: %d\n",
				g_info->usb_irq, ret);
	}
}
#endif

static __devinit int ricoh583_ac_detect_probe(struct platform_device *pdev)
{
	struct ricoh583_ac_detect_info *info;
	struct ricoh583_ac_detect_platform_data *pdata;
	int ret;

	info = kzalloc(sizeof(struct ricoh583_ac_detect_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	pdata = pdev->dev.platform_data;
	info->irq = pdata->irq;
#if RICOH583_USB_CHARGER_DETECT
	info->usb_gpio = pdata->usb_gpio;
	info->usb_irq = gpio_to_irq(info->usb_gpio);
#endif
	platform_set_drvdata(pdev, info);
	info->online = 0;
	g_info = info;

	ret = ricoh583_set_bits(info->dev->parent, RICOH583_PWRIRSL1_REG, 1 << 1);//both edge trigger
	if (ret){
		goto out;
	}
	INIT_WORK(&info->work, ricoh583_ac_detect_irq_work);
	ret = request_threaded_irq(info->irq, NULL, ricoh583_ac_detect_irq,
			IRQF_ONESHOT, "ricoh583_ac_detect", info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for ricoh583 charger detect: %d\n",
				info->irq, ret);
		goto out;
	}
#if RICOH583_USB_CHARGER_DETECT
	INIT_DELAYED_WORK(&info->usb_work, ricoh583_usb_work);
	ret = gpio_request(info->usb_gpio, "usb_detect");
	if (ret < 0){
		dev_err(&pdev->dev, "%s: gpio %d request failed!\n", __func__, info->usb_gpio);
		goto irq;
	}
	gpio_direction_input(info->usb_gpio);
	int val = gpio_get_value(info->usb_gpio);
	gpio_free(info->usb_gpio);

	if (val){
		info->usb_online = 0;
		ret = request_threaded_irq(info->usb_irq, NULL, ricoh583_usb_irq,
			IRQF_TRIGGER_FALLING, "usb_charger_detect", info);
	}else{
		info->usb_online = 1;
		dev_info(g_info->dev, "USB charger is plugged!\n");
		ricoh583_charger_plug(CHG_TYPE_USB);
		ret = request_threaded_irq(info->usb_irq, NULL, ricoh583_usb_irq,
			IRQF_TRIGGER_RISING, "usb_charger_detect", info);
	}
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for ricoh583 usb charger detect: %d\n",
				info->usb_irq, ret);
		goto irq;
	}
	device_init_wakeup(info->dev, 1);
	enable_irq_wake(info->usb_irq);
#endif
	
	dev_info(&pdev->dev, "%s is OK!\n", __func__);

	return 0;
irq:
	free_irq(info->irq, info);
out:
	kfree(info);
	return -1;
}
static int __devexit ricoh583_ac_detect_remove(struct platform_device *pdev)
{
	struct ricoh583_ac_detect_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
#if RICOH583_USB_CHARGER_DETECT
	free_irq(info->usb_irq, info);
#endif
	kfree(info);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver ricoh583_ac_detect_driver = {
	.driver		= {
		.name	= "ricoh583_ac_detect",
		.owner	= THIS_MODULE,
	},
	.probe		= ricoh583_ac_detect_probe,
	.remove		= __devexit_p(ricoh583_ac_detect_remove),
};

static int __init ricoh583_ac_detect_init(void)
{
	return platform_driver_register(&ricoh583_ac_detect_driver);
}
late_initcall(ricoh583_ac_detect_init);

static void __exit ricoh583_ac_detect_exit(void)
{
	platform_driver_unregister(&ricoh583_ac_detect_driver);
}
module_exit(ricoh583_ac_detect_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("ricoh583 charger detect driver");
MODULE_LICENSE("GPL");
