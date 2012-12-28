/*
 * Charger driver for tps80032 PMIC
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mfd/tps80032.h>
#include <linux/power/ns115-battery.h>

#define TPS80032_CHARGERUSB_VSYSREG	2, 0xDC
#define TPS80032_CHARGERUSB_VICHRG_PC	2, 0xDD
#define TPS80032_LINEAR_CHRG_STS	2, 0xDE
#define TPS80032_CONTROLLER_CTRL1	2, 0xE1
#define TPS80032_CHARGERUSB_INT_STS	2, 0xE4
#define TPS80032_CHARGERUSB_INT_MASK	2, 0xE5
#define TPS80032_CHARGERUSB_STS_INT1	2, 0xE6
#define TPS80032_CHARGERUSB_STS_INT2	2, 0xE7
#define TPS80032_CHARGERUSB_CTRL1	2, 0xE8
#define TPS80032_CHARGERUSB_CTRL2	2, 0xE9
#define TPS80032_CHARGERUSB_CTRL3	2, 0xEA
#define TPS80032_CHARGERUSB_VOREG	2, 0xEC
#define TPS80032_CHARGERUSB_VICHRG	2, 0xED
#define TPS80032_CHARGERUSB_CINLIMIT	2, 0xEE
#define TPS80032_CHARGERUSB_CTRLLIMIT1	2, 0xEF
#define TPS80032_CHARGERUSB_CTRLLIMIT2	2, 0xF0
#define TPS80032_ANTICOLLAPSE_CTRL1		2, 0xF1

#define LOCK_LIMIT	(1 << 4)
#define TERM		(1 << 4)
#define CHARGE_ONCE		(1 << 6)
#define ANTICOLL_ANA	(1 << 2)
#define BUCK_VTH_4440V	(3 << 5)
#define STAT_EN_LINCH		(1 << 4)
#define STAT_CURRENT_TERM	(1 << 3)
#define STAT_CHARGERUSB_STAT	(1 << 2)
#define STAT_CHARGERUSB_THMERG	(1 << 1)
#define STAT_CHARGERUSB_FAULT	1
#define INT1_TMERG		(1 << 7)
#define INT1_SLP_MODE	(1 << 1)
#define INT2_CHARGER_DONE	(1 << 1)
#define CTRL_EN_LINCH		(1 << 5)
#define MCHARGERUSB_FAULT	1
#define MCHARGERUSB_THMERG	(1 << 1)
#define MCHARGERUSB_STAT	(1 << 2)
#define MCURRENT_TERM	(1 << 3)
#define MEN_LINCH		(1<< 4)

#define LIMIT_VOL_MASK	0x3F
#define LIMIT_VOL_MIN	3500	
#define LIMIT_VOL_STEP	20	
#define LIMIT_CUR_MASK	0x0F
#define LIMIT_CUR_MIN	100	
#define LIMIT_CUR_STEP	100	

#define POP	1//sense resitor=20mohm,POP=1;68mohm,POP=0
#define CHG_CUR_MASK 0x0F
#if POP
#define CHG_CUR_MIN_S	100
#define CHG_CUR_STEP_S	100
#else
#define CHG_CUR_MIN_S	300
#define CHG_CUR_STEP_S	50
#endif
#define CHG_CUR_MIN_L	500
#define CHG_CUR_STEP_L	100
#define CHG_VOL_MASK	0x3F
#define CHG_VOL_MIN		3500	
#define CHG_VOL_STEP	20	
#define TERM_CUR_MASK	0xE0
#define TERM_CUR_POS	5
#define TERM_CUR_MIN	50
#define TERM_CUR_STEP	50

#define CIN_LIMIT_500MA		0x09
#define CIN_LIMIT_1800MA	0x20
#define CIN_LIMIT_2100MA	0x21
#define CIN_LIMIT_2250MA	0x22	

#define CHG_TERM_CUR	100
#define FULL_VOLTAGE	4160
#define MAX_CHG_CURRENT	1500
#define MAX_CHG_VOLTAGE	4210
#define WATCHDOG_TIME	60

struct tps80032_charger_info {
	struct device *dev;
	struct work_struct irq_work;
	struct delayed_work wdg_work;
	struct workqueue_struct * queue;
	int irq;
};

static struct tps80032_charger_info *g_info = NULL;

static int tps80032_start_precharge(struct tps80032_charger_info *info)
{
	return 0;
}

static int tps80032_start_fastcharge(struct tps80032_charger_info *info, int chg_current)
{
	int ret;
	u8 cur_val, vol_val, term_cur;

	if (chg_current > MAX_CHG_CURRENT){
		dev_err(info->dev, "the charge current %dmA is too large!\n", chg_current);
		goto err;
	}
	if (chg_current < CHG_CUR_MIN_L){
		cur_val = ((chg_current - CHG_CUR_MIN_S) / CHG_CUR_STEP_S) & CHG_CUR_MASK;
		chg_current = cur_val * CHG_CUR_STEP_S + CHG_CUR_MIN_S;
	}else{
		cur_val = ((chg_current - CHG_CUR_MIN_L) / CHG_CUR_STEP_L + 0x04) & CHG_CUR_MASK;
		chg_current = (cur_val - 0x04) * CHG_CUR_STEP_L + CHG_CUR_MIN_L;
	}
	ret = tps80032_write(TPS80032_CHARGERUSB_VICHRG, cur_val);
	if (ret < 0){
		dev_err(info->dev, "set charge current failed: %d!\n", ret);
		goto err;
	}
	vol_val = ((FULL_VOLTAGE - CHG_VOL_MIN) / CHG_VOL_STEP) & CHG_VOL_MASK;
	ret = tps80032_write(TPS80032_CHARGERUSB_VOREG, vol_val);
	if (ret < 0){
		dev_err(info->dev, "set charge voltage failed: %d!\n", ret);
		goto err;
	}
	dev_dbg(info->dev, "charge voltage: %dmV\n",
			vol_val * CHG_VOL_STEP + CHG_VOL_MIN);
	term_cur = (((CHG_TERM_CUR - TERM_CUR_MIN) / TERM_CUR_STEP) 
			<< TERM_CUR_POS) & TERM_CUR_MASK;
	ret = tps80032_write(TPS80032_CHARGERUSB_CTRL2, term_cur);
	if (ret < 0){
		dev_err(info->dev, "set charge termination current failed: %d\n", ret);
		goto err;
	}
	dev_dbg(info->dev, "charge termination current: %dmA\n",
			(term_cur >> TERM_CUR_POS) * TERM_CUR_STEP + TERM_CUR_MIN);

	ret = tps80032_set_bits(TPS80032_CHARGERUSB_CTRL1, TERM);
	ret |= tps80032_set_bits(TPS80032_CHARGERUSB_CTRL3, CHARGE_ONCE);
	if (chg_current <= 500){
		ret |= tps80032_write(TPS80032_CHARGERUSB_CINLIMIT, CIN_LIMIT_500MA);
	}else{
		ret |= tps80032_write(TPS80032_CHARGERUSB_CINLIMIT, CIN_LIMIT_1800MA);
	}
	ret |= tps80032_set_bits(TPS80032_ANTICOLLAPSE_CTRL1, BUCK_VTH_4440V | ANTICOLL_ANA);
	ret |= tps80032_set_bits(TPS80032_CONTROLLER_CTRL1, CTRL_EN_LINCH);
	if (ret < 0){
		dev_err(info->dev, "set charger failed!\n");
		goto err;
	}
	ret = tps80032_chg_watchdog_init(WATCHDOG_TIME);
	if (ret < 0){
		dev_err(info->dev, "watchdog init failed!\n");
		goto err;
	}
	queue_delayed_work(info->queue, &info->wdg_work, WATCHDOG_TIME - 2);
	dev_info(info->dev, "start charging! charge current: %dmA\n", chg_current);

	return ret;
err:
	return -1;
}

static int tps80032_start_usb_chg(struct ns115_charger * usb_chg, enum ns115_charging_type type)
{
	struct tps80032_charger_info *info = g_info;

	if (info == NULL){
		dev_err(info->dev, "g_info is NULL. init isn't complete!\n");
		return -1;
	}
	if (type == CHGING_TYPE_FAST){
		return tps80032_start_fastcharge(info, usb_chg->chg_current);
	}else{
		return tps80032_start_precharge(info);
	}
}

static int tps80032_start_ac_chg(struct ns115_charger * ac_chg, enum ns115_charging_type type)
{
	struct tps80032_charger_info *info = g_info;

	if (info == NULL){
		dev_err(info->dev, "g_info is NULL. init isn't complete!\n");
		return -1;
	}
	if (type == CHGING_TYPE_FAST){
		return tps80032_start_fastcharge(info, ac_chg->chg_current);
	}else{
		return tps80032_start_precharge(info);
	}
}

static int tps80032_stop_chg(struct ns115_charger * hw_chg)
{
	struct tps80032_charger_info *info = g_info;
	int ret;

	if (info == NULL){
		dev_err(info->dev, "g_info is NULL. init isn't complete!\n");
		return -1;
	}
	cancel_delayed_work(&g_info->wdg_work);
	ret = tps80032_set_bits(TPS80032_CONTROLLER_CTRL1, CTRL_EN_LINCH);
	dev_info(info->dev, "stop charging!\n");

	return ret;
}

static void tps80032_charger_wdg(struct work_struct * work)
{
	struct tps80032_charger_info *info = 
		container_of(work, struct tps80032_charger_info, wdg_work.work);
	int ret;

	ret = tps80032_chg_watchdog_reset();
	if (ret < 0){
		dev_err(info->dev, "reset watchdog failed!\n");
		return;
	}
	queue_delayed_work(info->queue, &info->wdg_work, WATCHDOG_TIME - 2);
}

static void tps80032_charger_work(struct work_struct * work)
{
	struct tps80032_charger_info *info = 
		container_of(work, struct tps80032_charger_info, irq_work);
	int ret;
	u8 val;

	ret = tps80032_read(TPS80032_CHARGERUSB_INT_STS, &val);
	dev_info(info->dev, "int status(0xE4): 0x%x\n", val);
	if (ret < 0){
		dev_err(info->dev, "read int status failed: %d!", ret);
		return;
	}
	if (val & STAT_CHARGERUSB_FAULT){
		tps80032_read(TPS80032_CHARGERUSB_STS_INT1, &val);
		if (val & ~(INT1_TMERG | INT1_SLP_MODE)){
			dev_err(info->dev, "charger error: int1(0xE6): 0x%x\n", val);
			ns115_battery_notify_event(CHG_ERROR_EVENT);
			return;
		}
	}
	if (val & STAT_CHARGERUSB_STAT){
		tps80032_read(TPS80032_CHARGERUSB_STS_INT2, &val);
		if (val & INT2_CHARGER_DONE){
			ns115_battery_notify_event(CHG_DONE_EVENT);
		}
	}

	return;
}

static irqreturn_t tps80032_charger_irq(int irq, void *_info)
{
	struct tps80032_charger_info *info = _info;

	queue_work(info->queue, &info->irq_work);

	return IRQ_HANDLED;
}

static int tps80032_charger_init(struct tps80032_charger_info * info)
{
	int ret;
	u8 cur_val, vol_val, val;

	cur_val = ((MAX_CHG_CURRENT - LIMIT_CUR_MIN) / LIMIT_CUR_STEP) & LIMIT_CUR_MASK;
	ret = tps80032_write(TPS80032_CHARGERUSB_CTRLLIMIT2, cur_val);
	vol_val = ((MAX_CHG_VOLTAGE - LIMIT_VOL_MIN) / LIMIT_VOL_STEP) & LIMIT_VOL_MASK;
	ret |= tps80032_write(TPS80032_CHARGERUSB_CTRLLIMIT1, vol_val);
	if (ret < 0){
		dev_err(info->dev, "set limit current and voltage failed: %d\n", ret);
		goto err;
	}
	ret = tps80032_read(TPS80032_CHARGERUSB_CTRLLIMIT2, &val);
	if ((ret < 0) || ((val & LIMIT_CUR_MASK) != cur_val)){
		dev_err(info->dev, "the limit current is wrong!\n");
		goto err;
	}
	ret = tps80032_read(TPS80032_CHARGERUSB_CTRLLIMIT1, &val);
	if ((ret < 0) || ((val & LIMIT_VOL_MASK) != vol_val)){
		dev_err(info->dev, "the limit voltage is wrong!\n");
		goto err;
	}
	ret = tps80032_set_bits(TPS80032_CHARGERUSB_CTRLLIMIT2, LOCK_LIMIT);
	ret |= tps80032_write(TPS80032_CHARGERUSB_INT_MASK,
			(MEN_LINCH | MCURRENT_TERM | MCHARGERUSB_THMERG));

	return ret;
err:
	return -1;
}

static struct ns115_charger tps80032_usb_chg = {
	.type = CHG_TYPE_USB,
	.name = "charger_usb",
	.chg_current = 500,
	.start_charging = tps80032_start_usb_chg,
	.stop_charging = tps80032_stop_chg,
};

static struct ns115_charger tps80032_ac_chg = {
	.type = CHG_TYPE_AC,
	.name = "charger_ac",
	.chg_current = 1000,
	.start_charging = tps80032_start_ac_chg,
	.stop_charging = tps80032_stop_chg,
};

static __devinit int tps80032_charger_probe(struct platform_device *pdev)
{
	struct tps80032_charger_info *info;
	struct tps80032_charger_platform_data *pdata;
	int ret;

	info = kzalloc(sizeof(struct tps80032_charger_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	pdata = pdev->dev.platform_data;
	info->irq = pdata->irq;

	platform_set_drvdata(pdev, info);
	g_info = info;

	INIT_WORK(&info->irq_work, tps80032_charger_work);
	INIT_DELAYED_WORK(&info->wdg_work, tps80032_charger_wdg);
	ret = request_threaded_irq(info->irq, NULL, tps80032_charger_irq,
			IRQF_ONESHOT, "tps80032-charger", info);
	if (ret < 0){
		dev_err(info->dev, "request tps80032 charger irq:%d failed: %d!\n",
				info->irq, ret);
		goto out;
	}

	ret = tps80032_charger_init(info);
	if (ret){
		goto irq;
	}
	ret = ns115_charger_register(&tps80032_usb_chg);
	if (ret){
		goto irq;
	}
	ret = ns115_charger_register(&tps80032_ac_chg);
	if (ret){
		goto irq;
	}
	info->queue = create_workqueue("tps80032 charger");
	if (!info->queue) {
		goto irq;
	}

	dev_info(info->dev, "%s is OK!\n", __func__);

	return 0;

irq:
	free_irq(info->irq, info);
out:
	kfree(info);
	g_info = NULL;
	return ret;
}

static int __devexit tps80032_charger_remove(struct platform_device *pdev)
{
	struct tps80032_charger_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	destroy_workqueue(info->queue);
	kfree(info);
	g_info = NULL;
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int tps80032_charger_suspend(struct device *dev)
{
	return 0;
}

static int tps80032_charger_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops tps80032_charger_pm_ops = {
	.suspend	= tps80032_charger_suspend,
	.resume		= tps80032_charger_resume,
};
#endif

static struct platform_driver tps80032_charger_driver = {
	.driver		= {
		.name	= "tps80032-charger",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &tps80032_charger_pm_ops,
#endif
	},
	.probe		= tps80032_charger_probe,
	.remove		= __devexit_p(tps80032_charger_remove),
};

static int __init tps80032_charger_dev_init(void)
{
	return platform_driver_register(&tps80032_charger_driver);
}
device_initcall(tps80032_charger_dev_init);

static void __exit tps80032_charger_exit(void)
{
	platform_driver_unregister(&tps80032_charger_driver);
}
module_exit(tps80032_charger_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("tps80032 Charger driver");
MODULE_LICENSE("GPL");
