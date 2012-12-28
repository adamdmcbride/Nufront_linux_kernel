/*
 * Battery driver for tps80032 PMIC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/mfd/tps80032.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/power/ns115-battery.h>
#include <linux/completion.h>

//#define DEBUG_EN
#ifdef DEBUG_EN
#define  PDBG(dev, format,...)	\
	dev_err(dev, format, ##__VA_ARGS__)
#define  PINFO(format,...)	\
	dev_err(dev, format, ##__VA_ARGS__)
#else
#define  PDBG(dev, format,...)	do{}while(0)
#define  PINFO(dev, format,...)	\
	dev_info(dev, format, ##__VA_ARGS__)
#endif

//if platform data is NULL, use the defualt value
#define TPS80032_ALARM_MVOLTS	3600
#define TPS80032_POWER_OFF_MVOLTS	3400
#define TPS80032_ADC_VDD_MVOLTS		2800

#define TPS80032_MIN_MVOLTS			3350
#define TPS80032_MAX_MVOLTS			4160

#define TPS80032_AUTOCAL_TIMEOUT	(3 * HZ)
#define VFS		62	//mV
#define SENSE_RESISTOR	20 //mohm
#define CLOCK_SOURCE	32768

//reg_id, reg_addr
#define TPS80032_FG_REG_00	2, 0xC0
#define TPS80032_FG_REG_01	2, 0xC1
#define TPS80032_FG_REG_02	2, 0xC2
#define TPS80032_FG_REG_03	2, 0xC3
#define TPS80032_FG_REG_04	2, 0xC4
#define TPS80032_FG_REG_05	2, 0xC5
#define TPS80032_FG_REG_06	2, 0xC6
#define TPS80032_FG_REG_07	2, 0xC7
#define TPS80032_FG_REG_08	2, 0xC8
#define TPS80032_FG_REG_09	2, 0xC9
#define TPS80032_FG_REG_10	2, 0xCA
#define TPS80032_FG_REG_11	2, 0xCB
#define TPS80032_REG_TOGGLE1	2, 0x90

#define FGDITHS	(1 << 7)
#define FGDITHR	(1 << 7)
#define FGS		(1 << 5)
#define FGR		(1 << 4)
#define CC_AUTOCLEAR	(1 << 2)
#define CC_CAL_EN		(1 << 1)
#define CC_PAUSE		1

enum {
	UPDATE_RATE_250MS,
	UPDATE_RATE_62_5MS,
	UPDATE_RATE_15_6MS,
	UPDATE_RATE_3_9MS,
};

struct tps80032_battery_info {
	struct device      *dev;
	struct mutex		lock;
	struct completion	autocal_comp;
	int		autocal_irq;
	int		cal_offset;
	int		init_cc;
	int		max_cc;
	int		resistor;
	int		tmp_channel;
	int		power_off_mvolts;
	int     alarm_mvolts;
	int		alarm_irq;
	int     min_mvolts;
	int     max_mvolts;
	int     cur_mvolts;
	int		(*capacity_table)[][2];
	int		table_size;
};

static struct tps80032_battery_info * g_info;

static int tps80032_gauge_calibration(struct tps80032_battery_info * info)
{
	int ret;
	int data;
	u8 reg_data[2];

	mutex_lock(&info->lock);
	info->cal_offset = 0;
	ret = tps80032_set_bits(TPS80032_FG_REG_00, CC_CAL_EN);
	if (ret < 0){
		dev_err(info->dev, "Enable calibration failed: %d!\n", ret);
		goto err;
	}

	init_completion(&info->autocal_comp);
	ret = wait_for_completion_interruptible_timeout(&info->autocal_comp, TPS80032_AUTOCAL_TIMEOUT);
	if (!ret){
		dev_err(info->dev, "wait auto calibration complete timeout!\n");
		goto err;
	}
	ret = tps80032_bulk_reads(TPS80032_FG_REG_08, 2, reg_data);
	if (ret < 0){
		dev_err(info->dev, "reg calibration offset data failed: %d!\n", ret);
		goto err;
	}
	ret = tps80032_set_bits(TPS80032_FG_REG_00, CC_AUTOCLEAR);
	if (ret < 0){
		dev_err(info->dev, "auto clear failed!\n");
		goto err;
	}

	data = reg_data[1] & 0x01;
	data = (data << 8) | reg_data[0];
	if (reg_data[1] & 0x02){
		data = -data;
	}
	info->cal_offset = data;
	PDBG(info->dev, "FG_REG_08(0xc8): 0x%x\n", reg_data[0]);
	PDBG(info->dev, "FG_REG_09(0xc9): 0x%x\n", reg_data[1]);
	PDBG(info->dev, "the auto calibration offset value: %d\n", data);

	mutex_unlock(&info->lock);
	return 0;
err:
	mutex_unlock(&info->lock);
	return -1;

}

static irqreturn_t tps80032_autocal_irq(int irq, void *_info)
{
	struct tps80032_battery_info * info = _info;

	complete(&info->autocal_comp);
	return IRQ_HANDLED;
}

static int tps80032_get_capacity(int mvolts)
{
	struct tps80032_battery_info *info = g_info;
	int ret;
	int accum, counter, iacc, cur_cc, capacity;
	u8 reg_data[7];

	mutex_lock(&info->lock);
	ret = tps80032_set_bits(TPS80032_FG_REG_00, CC_PAUSE);
	ret |= tps80032_bulk_reads(TPS80032_FG_REG_01, 7, reg_data);
	ret |= tps80032_clr_bits(TPS80032_FG_REG_00, CC_PAUSE);

	if (ret < 0){
		dev_err(info->dev, "%s: read and write the register failed!\n", __func__);
		goto err;
	}
	counter = reg_data[2];
	counter = (counter << 8) | reg_data[1];
	counter = (counter << 8) | reg_data[0];
	PDBG(info->dev, "Sample counter value: %d\n", counter);

	accum = reg_data[6];
	accum = (accum << 8) | reg_data[5];
	accum = (accum << 8) | reg_data[4];
	accum = (accum << 8) | reg_data[3];
	PDBG(info->dev, "FG_REG_04(0xc4): 0x%x\n", reg_data[3]);
	PDBG(info->dev, "FG_REG_05(0xc5): 0x%x\n", reg_data[4]);
	PDBG(info->dev, "FG_REG_06(0xc6): 0x%x\n", reg_data[5]);
	PDBG(info->dev, "FG_REG_07(0xc7): 0x%x\n", reg_data[6]);
	PDBG(info->dev, "Accumulated value: %d\n", accum);

	iacc = (accum - (info->cal_offset * counter)) * VFS 
		/ (SENSE_RESISTOR * CLOCK_SOURCE * 36 / 10);
	PDBG(info->dev, "accumulated current(VFS=62mV): %dmAh\n", iacc);
	iacc = (accum - (info->cal_offset * counter)) * 1250 
		/ (SENSE_RESISTOR * CLOCK_SOURCE * 36 / 10);
	PDBG(info->dev, "Accumulated current(VFS=1250mV): %dmAh\n", iacc);
	cur_cc = info->init_cc + iacc;
	capacity = (cur_cc * 100) / info->max_cc;

	mutex_unlock(&info->lock);
	return capacity;
err:
	mutex_unlock(&info->lock);
	return -1;
}

static int tps80032_get_mvolts(void)
{
	struct tps80032_battery_info *info = g_info;
	int volt;

	volt = tps80032_get_adc_value(TPS80032_ADC18_BAT_VOLT, 5000, 0);
	if (!volt){
		dev_err(info->dev, "get battery voltage failed!\n");
		return 0;
	}

	return volt;
}

static int tps80032_get_current(void)
{
	struct tps80032_battery_info *info = g_info;
	int cur;
	//int ret;
	//u8 reg_data[2];

#if 1
	cur = tps80032_get_adc_value(TPS80032_ADC17_BAT_CUR, 0, 0);
	if (!cur){
		dev_err(info->dev, "get battery current failed!\n");
		return 0;
	}
	dev_dbg(info->dev, "GPADC current: %d\n", cur);
#else
	ret = tps80032_bulk_reads(TPS80032_FG_REG_10, 2, reg_data);
	if (ret < 0){
		dev_err(info->dev, "read CC_INTEG failed: %d\n", ret);
		return 0;
	}
	cur = reg_data[1] & 0x1f;
	cur = (cur << 8) | reg_data[0];
	if (reg_data[1] & 0x20){
		cur = -cur;
	}

	PDBG(info->dev, "reg_data0: 0x%x\n", reg_data[0]);
	PDBG(info->dev, "reg_data1: 0x%x\n", reg_data[1]);
	PDBG(info->dev, "CC_INTEG: %d, offset: %d\n", cur, info->cal_offset);
	cur -= info->cal_offset;

	cur = cur * VFS * 1000 / (SENSE_RESISTOR * 8191);
	PDBG(info->dev, "current: %d\n", cur);
#endif

	return cur;
}

static int tps80032_calib_start_chg(void)
{
	struct tps80032_battery_info *info = g_info;
	int ret, capacity;

	capacity = tps80032_get_capacity(0);
	info->init_cc = info->max_cc * capacity / 100;
	PINFO(info->dev, "capacity is %d%%. reset the init_cc is %dmAh\n",
			capacity, info->init_cc);
	ret = tps80032_gauge_calibration(info);

	return ret;
}

static int tps80032_calib_stop_chg(void)
{
	struct tps80032_battery_info *info = g_info;
	int ret, capacity;

	capacity = tps80032_get_capacity(0);
	info->init_cc = info->max_cc * capacity / 100;
	PINFO(info->dev, "capacity is %d%%. reset the init_cc is %dmAh\n",
			capacity, info->init_cc);
	ret = tps80032_gauge_calibration(info);

	return ret;
}

static int tps80032_calib_chg_done(void)
{
	struct tps80032_battery_info *info = g_info;
	int ret;

	info->init_cc = info->max_cc;
	PINFO(info->dev, "charging is done. reset the capacity 100%%\n");
	ret = tps80032_gauge_calibration(info);

	return ret;
}

static int tps80032_capacity_init(struct tps80032_battery_info *info)
{
	int mvolts;
	int cur;
	int i;
	int capacity;

	mvolts = tps80032_get_mvolts();
	if (mvolts < 0){
		dev_err(info->dev, "%s: get voltage failed!\n", __func__);
		return -1;
	}
	dev_info(info->dev, "init battery voltage: %dmV\n", mvolts);

	cur = tps80032_get_current();
	if (!cur){
		dev_err(info->dev, "%s: get current failed!\n", __func__);
		return -1;
	}
	dev_info(info->dev, "init battery current: %dmA\n", cur);

	mvolts -= info->resistor * cur / 1000;
	dev_info(info->dev, "calibration battery voltage: %dmV\n", mvolts);

	for (capacity = 0,i = info->table_size - 1; i >= 0; --i){
		if (mvolts < (*(info->capacity_table))[i][1]){
			capacity = (*(info->capacity_table))[i][0];
			break;
		}
	}
	info->init_cc = info->max_cc * capacity / 100;
	dev_info(info->dev, "init capacity: %d%%. init cc: %dmAh. Max cc: %dmAh\n",
			capacity, info->init_cc, info->max_cc);

	return 0;
}

static int tps80032_gauge_init(struct tps80032_battery_info *info)
{
	int ret;

	ret = tps80032_capacity_init(info);
	if (ret < 0){
		dev_err(info->dev, "calculate init capacity failed!\n");
		return -1;
	}
	ret = tps80032_reg_update(TPS80032_FG_REG_00, UPDATE_RATE_250MS, 0xC0);
	if (ret < 0){
		dev_err(info->dev, "Set update rate failed!\n");
		return -1;
	}
	ret = tps80032_set_bits(TPS80032_REG_TOGGLE1, FGS | FGDITHS);
	if (ret < 0){
		dev_err(info->dev, "Enable gas gauge failed!\n");
		return -1;
	}
	ret = tps80032_gauge_calibration(info);
	if (ret < 0){
		dev_err(info->dev, "Gas Gauge calibration failed!\n");
		return -1;
	}

	return 0;
}

static struct ns115_battery_gauge tps80032_battery_gauge = {
	.get_battery_mvolts = tps80032_get_mvolts,
	.get_battery_current = tps80032_get_current,
	.get_battery_capacity = tps80032_get_capacity,
	.calib_gauge_start_chg = tps80032_calib_start_chg,
	.calib_gauge_stop_chg = tps80032_calib_stop_chg,
	.calib_gauge_chg_done = tps80032_calib_chg_done,
};

static __devinit int tps80032_battery_probe(struct platform_device *pdev)
{
	struct tps80032_battery_info *info;
	struct tps80032_battery_platform_data *pdata;
	int ret;

	info = kzalloc(sizeof(struct tps80032_battery_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	pdata = pdev->dev.platform_data;
	info->autocal_irq = pdata->autocal_irq;
	info->max_cc = pdata->max_capacity;
	info->resistor = pdata->resistor;
	info->alarm_mvolts = pdata->alarm_mvolts;
	info->power_off_mvolts = pdata->power_off_mvolts;
	info->min_mvolts = TPS80032_MIN_MVOLTS;
	info->max_mvolts = TPS80032_MAX_MVOLTS;
	info->capacity_table = pdata->capacity_table;
	info->table_size = pdata->table_size;

	if (info->alarm_mvolts == 0){
		info->alarm_mvolts = TPS80032_ALARM_MVOLTS;
	}
	if (info->power_off_mvolts == 0){
		info->power_off_mvolts = TPS80032_POWER_OFF_MVOLTS;
	}

	platform_set_drvdata(pdev, info);
	g_info = info;
	mutex_init(&info->lock);

	ret = request_threaded_irq(info->autocal_irq, NULL, tps80032_autocal_irq,
			IRQF_ONESHOT, "tps80032_autocal", info);
	if (ret < 0){
		dev_err(info->dev, "request auto calibration irq:%d failed: %d!\n",
				info->autocal_irq, ret);
		goto out;
	}

	ret = tps80032_gauge_init(info);
	if (ret){
		goto irq;
	}
	tps80032_battery_gauge.resistor_mohm = info->resistor;
	ret = ns115_battery_gauge_register(&tps80032_battery_gauge);
	if (ret){
		goto irq;
	}
	dev_info(info->dev, "%s is OK!\n", __func__);

	return 0;

irq:
	free_irq(info->autocal_irq, info);
out:
	kfree(info);
	return ret;
}

static int __devexit tps80032_battery_remove(struct platform_device *pdev)
{
	struct tps80032_battery_info *info = platform_get_drvdata(pdev);

	free_irq(info->autocal_irq, info);
	kfree(info);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int tps80032_battery_suspend(struct device *dev)
{
	return 0;
}

static int tps80032_battery_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops tps80032_battery_pm_ops = {
	.suspend	= tps80032_battery_suspend,
	.resume		= tps80032_battery_resume,
};
#endif

static struct platform_driver tps80032_battery_driver = {
	.driver		= {
		.name	= "tps80032-battery",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &tps80032_battery_pm_ops,
#endif
	},
	.probe		= tps80032_battery_probe,
	.remove		= __devexit_p(tps80032_battery_remove),
};

static int __init tps80032_battery_init(void)
{
	return platform_driver_register(&tps80032_battery_driver);
}
device_initcall(tps80032_battery_init);

static void __exit tps80032_battery_exit(void)
{
	platform_driver_unregister(&tps80032_battery_driver);
}
module_exit(tps80032_battery_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("tps80032 Battery driver");
MODULE_LICENSE("GPL");
