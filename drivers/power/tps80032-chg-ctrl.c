/*
 * Charger controller driver for tps80032 PMIC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/mfd/tps80032.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/power/ns115-battery.h>

#define TPS80032_CONTROLLER_CTRL2	2, 0xDA
#define TPS80032_CONTROLLER_VSEL_COMP	2, 0xDB
#define TPS80032_CONTROLLER_INT_MASK	2, 0xE0
#define TPS80032_CONTROLLER_CTRL1	2, 0xE1
#define TPS80032_CONTROLLER_WDG		2, 0xE2
#define TPS80032_CONTROLLER_STAT1	2, 0xE3

#define STAT_BAT_TEMP_OVRANGE	1
#define STAT_BAT_REMOVED	(1 << 1)
#define STAT_VBUS_DET		(1 << 2)
#define STAT_FAULT_WDG		(1 << 4)

#define MVAC_FAULT	(1 << 7)
#define MVAC_EOC	(1 << 6)
#define MLINCH_GATED	(1 << 5)
#define MBAT_REMOVED	(1 << 4)
#define MFAULT_WDG	(1 << 3)
#define MBAT_TEMP	(1 << 2)
#define MVBUS_DET	(1 << 1)
#define MVAC_DET	1
#define EN_CHARGER	(1 << 4)
#define WDG_RST		(1 << 7)
#define WDG_MASK	0x7F

#define VBUS_DELAY_TIME		msecs_to_jiffies(500)

struct tps80032_chg_ctrl_info {
	struct device *dev;
	struct delayed_work work;
	int irq;
	int vbus_online;
};

static void tps80032_chg_ctrl_work(struct work_struct * work)
{
	struct tps80032_chg_ctrl_info * info = 
		container_of(work, struct tps80032_chg_ctrl_info, work.work);
	int ret;
	u8 val;

	ret = tps80032_read(TPS80032_CONTROLLER_STAT1, &val);
	if (ret < 0){
		dev_err(info->dev, "read interrupt status failed: %d", ret);
		goto out;
	}
	dev_dbg(info->dev, "charger controller interrupt status: 0x%x\n", val);
	if (val & STAT_BAT_REMOVED){
		dev_err(info->dev, "battery is removed!\n");
		ns115_battery_notify_event(CHG_BATT_REMOVED);
	}
	if (val & STAT_FAULT_WDG){
		dev_err(info->dev, "charger watchdog timeout!\n");
		ns115_battery_notify_event(CHG_ERROR_EVENT);
	}
	if (val & STAT_BAT_TEMP_OVRANGE){
		dev_err(info->dev, "battery temperature out of range!\n");
		ns115_battery_notify_event(CHG_BATT_TEMP_OUTOFRANGE);
	}
	if (val & STAT_VBUS_DET){
		if (info->vbus_online == 0){
			info->vbus_online = 1;
			dev_info(info->dev, "USB charger is plugged!\n");
			ns115_charger_plug(CHG_TYPE_USB);
		}
	}else if (info->vbus_online){
		info->vbus_online = 0;
		dev_info(info->dev, "charger is unplugged!\n");
		ns115_charger_unplug();
	}

out:
	enable_irq(info->irq);
	return;
}

static irqreturn_t tps80032_chg_ctrl_irq(int irq, void * _info)
{
	struct tps80032_chg_ctrl_info * info = _info;

	disable_irq_nosync(info->irq);
	schedule_delayed_work(&info->work, VBUS_DELAY_TIME);

	return IRQ_HANDLED;
}

int tps80032_chg_watchdog_init(int seconds)
{
	int ret;
	u8 val;

	if (seconds < 0 || seconds > 127){
		return -1;
	}
	val = seconds & WDG_MASK;
	ret = tps80032_write(TPS80032_CONTROLLER_WDG, val);

	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_chg_watchdog_init);

int tps80032_chg_watchdog_reset(void)
{
	int ret;

	ret = tps80032_set_bits(TPS80032_CONTROLLER_WDG, WDG_RST);

	return ret;
}
EXPORT_SYMBOL_GPL(tps80032_chg_watchdog_reset);

static int tps80032_chg_ctrl_init(struct tps80032_chg_ctrl_info * info)
{
	int ret;
	u8 val;

	//mask interrupt
	ret = tps80032_write(TPS80032_CONTROLLER_INT_MASK, 
			MVAC_FAULT | MVAC_EOC | MLINCH_GATED | MVAC_DET);
	if (ret < 0){
		dev_err(info->dev, "set interrupt mask register failed: %d", ret);
		return -1;
	}
	//enable charger
	ret = tps80032_set_bits(TPS80032_CONTROLLER_CTRL1, EN_CHARGER);
	if (ret < 0){
		dev_err(info->dev, "enable USB charging failed:%d\n", ret);
		return -1;
	}
	ret = tps80032_read(TPS80032_CONTROLLER_STAT1, &val);
	if (ret < 0){
		dev_err(info->dev, "read interrupt status failed: %d", ret);
		return -1;
	}
	if (val & STAT_VBUS_DET){
		info->vbus_online = 1;
		ns115_charger_plug(CHG_TYPE_USB);
	}else{
		info->vbus_online = 0;
	}

	return 0;
}

static __devinit int tps80032_chg_ctrl_probe(struct platform_device *pdev)
{
	struct tps80032_chg_ctrl_info *info;
	struct tps80032_chg_ctrl_platform_data *pdata;
	int ret;

	info = kzalloc(sizeof(struct tps80032_chg_ctrl_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	pdata = pdev->dev.platform_data;
	info->irq = pdata->irq;

	platform_set_drvdata(pdev, info);

	INIT_DELAYED_WORK(&info->work, tps80032_chg_ctrl_work);

	ret = tps80032_chg_ctrl_init(info);
	if (ret){
		goto out;
	}
	ret = request_threaded_irq(info->irq, NULL, tps80032_chg_ctrl_irq,
			IRQF_ONESHOT, "tps80032_chg_ctrl", info);
	if (ret < 0){
		dev_err(info->dev, "request charger controller irq:%d failed: %d!\n",
				info->irq, ret);
		goto out;
	}
	dev_info(info->dev, "%s is OK!\n", __func__);

	return 0;

out:
	kfree(info);
	return ret;
}

static int __devexit tps80032_chg_ctrl_remove(struct platform_device *pdev)
{
	struct tps80032_chg_ctrl_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	kfree(info);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver tps80032_chg_ctrl_driver = {
	.driver		= {
		.name	= "tps80032-chg-ctrl",
		.owner	= THIS_MODULE,
	},
	.probe		= tps80032_chg_ctrl_probe,
	.remove		= __devexit_p(tps80032_chg_ctrl_remove),
};

static int __init tps80032_chg_ctrl_dev_init(void)
{
	return platform_driver_register(&tps80032_chg_ctrl_driver);
}
late_initcall(tps80032_chg_ctrl_dev_init);

static void __exit tps80032_chg_ctrl_exit(void)
{
	platform_driver_unregister(&tps80032_chg_ctrl_driver);
}
module_exit(tps80032_chg_ctrl_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("tps80032 charger controller driver");
MODULE_LICENSE("GPL");
