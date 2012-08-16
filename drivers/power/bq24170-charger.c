/*
 * charger driver for bq24170 
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
#include <linux/power/bq24170-charger.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#define BQ24170_STAT_FAULT_CYCLE	1000

struct bq24170_charger_info {
	struct device      *dev;
	struct work_struct  work;
	struct mutex		irq_lock;
	struct mutex		work_lock;
	struct mutex		stat_lock;
	struct workqueue_struct *queue;
	enum ns115_charger_state chg_stat;
	int		stat_gpio;
	int		irq;
	int		irq_requested;
	int		stop;
};

static struct bq24170_charger_info * g_info;

static int bq24170_start_chg(struct ns115_charger * hw_chg, enum ns115_charging_type type)
{
	mutex_lock(&g_info->stat_lock);
	g_info->stop = 0;
	mutex_unlock(&g_info->stat_lock);
	queue_work(g_info->queue, &g_info->work);
	dev_info(g_info->dev, "start charging!\n");

	return 0;
}

static int bq24170_stop_chg(struct ns115_charger * hw_chg)
{
	mutex_lock(&g_info->stat_lock);
	g_info->stop = 1;
	mutex_unlock(&g_info->stat_lock);

	mutex_lock(&g_info->irq_lock);
	if (g_info->irq_requested){
		free_irq(g_info->irq, g_info);
		g_info->irq_requested = 0;
	}
	mutex_unlock(&g_info->irq_lock);
	dev_info(g_info->dev, "stop charging!\n");

	return 0;
}

static irqreturn_t bq24170_irq(int irq, void *_info)
{
	struct bq24170_charger_info *info = _info;

	disable_irq_nosync(info->irq);
	queue_work(info->queue, &info->work);

	return IRQ_HANDLED;
}

static void bq24170_work(struct work_struct * work)
{
	int ret, val;
	int i, high_count;
	struct bq24170_charger_info *info = container_of(work, struct bq24170_charger_info, work);

	mutex_lock(&info->stat_lock);
	if (info->stop){
		mutex_unlock(&info->stat_lock);
		dev_info(info->dev, "the charger is stopped! stop checking the status!\n");
		return;
	}
	mutex_unlock(&info->stat_lock);

	mutex_lock(&info->work_lock);
	ret = gpio_request(info->stat_gpio, "bq24170_charger");
	if (ret < 0){
		dev_err(info->dev, "request gpio: %d error: %d!", info->stat_gpio, ret);
		goto work_lock;
	}
	gpio_direction_input(info->stat_gpio);

	dev_info(info->dev, "start checking the charging status!\n");
	msleep(BQ24170_STAT_FAULT_CYCLE / 2);
	for (i = 0, high_count = 0; i < 2; ++i){
		mutex_lock(&info->stat_lock);
		if (info->stop){
			dev_info(info->dev, "the charger is stopped! stop checking the status!\n");
			gpio_free(info->stat_gpio);
			goto stat_lock;
		}
		mutex_unlock(&info->stat_lock);
		msleep(BQ24170_STAT_FAULT_CYCLE);
		val = gpio_get_value(info->stat_gpio);
		high_count += val;
	}
	gpio_free(info->stat_gpio);

	mutex_lock(&info->stat_lock);
	if (info->stop){
		dev_info(info->dev, "the charger is stopped! stop checking the status!\n");
		goto stat_lock;
	}
	switch (high_count){
		case 0:
			info->chg_stat = CHG_DONE_STATE;
			dev_info(info->dev, "the status is charging done!\n");
			ns115_battery_notify_event(CHG_DONE_EVENT);
			goto stat_lock;
		case 1:
			info->chg_stat = CHG_ABSENT_STATE;
			dev_err(info->dev, "the status is error!\n");
			ns115_battery_notify_event(CHG_UNPLUG_EVENT);
			goto stat_lock;
		case 2:
			info->chg_stat = CHG_CHARGING_STATE;
			dev_info(info->dev, "the status is charging!\n");
			goto done;
	}
stat_lock:
	mutex_unlock(&info->stat_lock);
	goto work_lock;

done:
	mutex_unlock(&info->stat_lock);

	mutex_lock(&info->irq_lock);
	if (info->irq_requested){
		free_irq(info->irq, info);
		info->irq_requested = 0;
	}
	if (info->stop){
		dev_info(info->dev, "charging is stoped, don't request status irq!\n");
		goto irq_lock;
	}
	if (val){
		ret = request_threaded_irq(info->irq, NULL, bq24170_irq,
			IRQF_TRIGGER_FALLING, "bq24170-charger", info);
	}else{
		ret = request_threaded_irq(info->irq, NULL, bq24170_irq,
			IRQF_TRIGGER_RISING, "bq24170-charger", info);
	}
	if (ret < 0) {
		dev_err(info->dev, "Can't get %d IRQ: %d\n", info->irq, ret);
		goto irq_lock;
	}
	info->irq_requested = 1;

irq_lock:
	mutex_unlock(&info->irq_lock);
work_lock:
	mutex_unlock(&info->work_lock);
	return;

}

static struct ns115_charger bq24170_ac_chg = {
	.type = CHG_TYPE_AC,
	.name = "charger_ac",
	.start_charging = bq24170_start_chg,
	.stop_charging = bq24170_stop_chg,
};

static struct ns115_charger bq24170_usb_chg = {
	.type = CHG_TYPE_USB,
	.name = "charger_usb",
	.chg_current = 400,
	.start_charging = bq24170_start_chg,
	.stop_charging = bq24170_stop_chg,
};

static __devinit int bq24170_charger_probe(struct platform_device *pdev)
{
	struct bq24170_charger_info *info;
	struct bq24170_charger_platform_data *pdata;
	int ret;

	info = kzalloc(sizeof(struct bq24170_charger_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	pdata = pdev->dev.platform_data;
	info->stat_gpio = pdata->stat_gpio;
	info->irq = gpio_to_irq(info->stat_gpio);
	info->irq_requested = 0;
	info->stop = 1;
	g_info = info;

	bq24170_ac_chg.chg_current = pdata->ac_chg_current;
	ret = ns115_charger_register(&bq24170_ac_chg);
	if (ret){
		goto out;
	}
	ret = ns115_charger_register(&bq24170_usb_chg);
	if (ret){
		goto err_ac;
	}

	mutex_init(&info->irq_lock);
	mutex_init(&info->work_lock);
	mutex_init(&info->stat_lock);
	INIT_WORK(&info->work, bq24170_work);
	info->queue = create_workqueue("bq24170-charger");
	if (!info->queue){
		dev_err(info->dev, "create workqueue error!\n");
		ret = -ENOMEM;
		goto err_usb;
	}

	dev_info(info->dev, "%s is OK!\n", __func__);

	return 0;

err_usb:
	ns115_charger_unregister(&bq24170_usb_chg);
err_ac:
	ns115_charger_unregister(&bq24170_ac_chg);
out:
	kfree(info);
	return ret;
}

static int __devexit bq24170_charger_remove(struct platform_device *pdev)
{
	struct bq24170_charger_info *info = platform_get_drvdata(pdev);

	flush_workqueue(info->queue);
	destroy_workqueue(info->queue);
	if (info->irq_requested){
		free_irq(info->irq, info);
	}
	mutex_destroy(&info->irq_lock);
	mutex_destroy(&info->work_lock);
	mutex_destroy(&info->stat_lock);
	kfree(info);
	ns115_charger_unregister(&bq24170_ac_chg);
	ns115_charger_unregister(&bq24170_usb_chg);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver bq24170_charger_driver = {
	.driver		= {
		.name	= "bq24170_charger",
		.owner	= THIS_MODULE,
	},
	.probe		= bq24170_charger_probe,
	.remove		= __devexit_p(bq24170_charger_remove),
};

static int __init bq24170_charger_init(void)
{
	return platform_driver_register(&bq24170_charger_driver);
}
device_initcall(bq24170_charger_init);

static void __exit bq24170_charger_exit(void)
{
	platform_driver_unregister(&bq24170_charger_driver);
}
module_exit(bq24170_charger_exit);


MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("bq24170 charger driver");
MODULE_LICENSE("GPL");
