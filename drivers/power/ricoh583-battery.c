/*
 * Battery driver for ricoh583 PMIC
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
#include <linux/power_supply.h>
#include <linux/mfd/ricoh583.h>
#include <linux/interrupt.h>
#include <linux/power/ns115-battery.h>
#include <linux/completion.h>

#define RICOH583_REG_ADCCNT1	0x70
#define RICOH583_REG_ADCCNT2	0x71
#define RICOH583_REG_ADCCNT3	0x72
#define RICOH583_REG_VBATDATAH	0x74
#define RICOH583_REG_VBATTHL	0x80

#define RICOH583_AUTOTIM_250MS	0x0
#define RICOH583_AUTOTIM_500MS	0x1
#define RICOH583_AUTOTIM_1S		0x2
#define RICOH583_AUTOTIM_2S		0x3
#define RICOH583_AUTOTIM_4S		0x4
#define RICOH583_AUTOTIM_8S		0x5
#define RICOH583_AUTOTIM_16S	0x6
//if platform data is NULL, use the defualt value
#define RICOH583_ALARM_MVOLTS	3660 //15%
#define RICOH583_POWER_OFF_MVOLTS	3600
#define RICOH583_ADC_VDD_MVOLTS		2800

#define RICOH583_MIN_MVOLTS 3600
#define RICOH583_MAX_MVOLTS 4160
#define RICOH583_ADC_TIMEOUT (1 * HZ)

#define ADRQ_MASK	0x30
#define ADRQ_POS	4

enum ricoh583_alarm_mode {
	ALARM_LOW,
	ALARM_OFF,
};

struct ricoh583_battery_info {
	struct device      *dev;
	enum ricoh583_adc_channel adc_channel;
	enum ricoh583_alarm_mode  alarm_mode;
	struct mutex		lock;
	struct completion	adc_comp;
	int         multiple;
	int         adc_vdd_mvolts;
	int			power_off_mvolts;
	int         alarm_mvolts;
	int			alarm_irq;
	int			adc_irq;
	int         min_mvolts;
	int         max_mvolts;
	int         cur_mvolts;
	int			resistor_mohm;
};

static struct ricoh583_battery_info * g_info;

enum ricoh583_adc_mode{
	ADC_MODE_OFF,
	ADC_MODE_SINGLE,
	ADC_MODE_AUTO
};

static int capacity_map[][2] = {
	{100, 4160},
	{90, 4000},
	{80, 3960},
	{70, 3940},
	{60, 3900},
	{50, 3840},
	{40, 3790},
	{30, 3720},
	{20, 3680},
	{10, 3640},
	{0, 3600},
};

static int ricoh583_adc_mode_set(struct ricoh583_battery_info *info, enum ricoh583_adc_mode mode)
{
	int ret;

	mutex_lock(&info->lock);
	ret =ricoh583_update(info->dev->parent, RICOH583_REG_ADCCNT3, mode << ADRQ_POS, ADRQ_MASK);
	mutex_unlock(&info->lock);

	return ret;
}

static int ricoh583_alarm_mvolts_set(struct ricoh583_battery_info *info, int volts)
{
	int ret = 0;
	uint8_t reg;

	if (info->adc_channel <= RICOH583_ADC_CHANNEL_VIN8){
		volts /= 3;
	}
	volts = volts * 100 / info->multiple;
	volts = volts * 4095 / info->adc_vdd_mvolts;
	volts = volts >> 4;
	reg = RICOH583_REG_VBATTHL + info->adc_channel * 2;
	ret = ricoh583_write(info->dev->parent, reg, volts);//set alarm voltage
	if (ret < 0){
		dev_err(info->dev, "%s: ricoh583 write error:%d\n", __func__, ret);
		return -1;
	}

	return 0;
}

static int ricoh583_init_battery(struct ricoh583_battery_info *info)
{
	int ret = 0;
	uint8_t val = 0;

	ricoh583_alarm_mvolts_set(info, info->alarm_mvolts);
	info->alarm_mode = ALARM_LOW;
	ret = ricoh583_write(info->dev->parent, RICOH583_REG_ADCCNT2, 
			RICOH583_AUTOTIM_16S); //conversion time of ADC auto-mode
	if (info->adc_channel <= RICOH583_ADC_CHANNEL_VIN8){
		val = 1 << (info->adc_channel + 3);
	}else{
		val = 1 << (info->adc_channel - 3);
	}
	ret |= ricoh583_write(info->dev->parent, RICOH583_REG_ADCCNT1, val); //disable auto-mode for channel
	val = info->adc_channel ;
	ret |= ricoh583_write(info->dev->parent, RICOH583_REG_ADCCNT3, val); //one time, channel
	if (ret < 0){
		dev_err(info->dev, "%s: ricoh583 write error:%d\n", __func__, ret);
		return -1;
	}

	return ret;
}

static int ricoh583_get_mvolts(void)
{
	struct ricoh583_battery_info *info = g_info;
	uint8_t data_l = 0, data_h = 0;
	uint8_t adc_reg = RICOH583_REG_VBATDATAH + info->adc_channel * 2;
	int ret, data;

	ricoh583_adc_mode_set(info, ADC_MODE_SINGLE);
	mutex_lock(&info->lock);

	init_completion(&info->adc_comp);
	ret = wait_for_completion_interruptible_timeout(&info->adc_comp, RICOH583_ADC_TIMEOUT);
	if (!ret){
		dev_err(info->dev, "ADC conversion time out!\n");
		goto err;
	}

	ricoh583_read(info->dev->parent, adc_reg, &data_h);
	ricoh583_read(info->dev->parent, adc_reg + 1, &data_l);
	data = ((data_h & 0xff) << 4) | (data_l & 0x0f);
	data = data * info->adc_vdd_mvolts / 4095;
	data = data * info->multiple / 100;
	if (info->adc_channel <= RICOH583_ADC_CHANNEL_VIN8){
		data = data * 3;
	}
	info->cur_mvolts = data;
	mutex_unlock(&info->lock);

	return data;
err:
	mutex_unlock(&info->lock);
	dev_err(info->dev, "%s error!\n", __func__);
	return 0;
}

static int ricoh583_get_capacity(int mvolts)
{
	int i;
	struct ricoh583_battery_info *info = g_info;

	info->cur_mvolts = mvolts;
	for (i = 0; i < sizeof(capacity_map) / (sizeof(int) * 2); ++i){
		if (mvolts >= capacity_map[i][1]){
			return capacity_map[i][0];
		}
	}
	return 0;
}

static struct ns115_battery_gauge ricoh583_battery_gauge = {
	.get_battery_mvolts = ricoh583_get_mvolts,
	.get_battery_capacity = ricoh583_get_capacity,
};

static irqreturn_t ricoh583_battery_adc(int irq, void *_info)
{
	struct ricoh583_battery_info *info = _info;

	complete(&info->adc_comp);

	return IRQ_HANDLED;
}

static irqreturn_t ricoh583_battery_alarm(int irq, void *_info)
{
	struct ricoh583_battery_info *info = _info;

	dev_info(info->dev, "%s\n", __func__);
	ns115_battery_notify_event(CHG_BATT_STATUS_CHANGED);

	return IRQ_HANDLED;
}

static __devinit int ricoh583_battery_probe(struct platform_device *pdev)
{
	struct ricoh583_battery_info *info;
	struct ricoh583_battery_platform_data *pdata;
	int ret;

	info = kzalloc(sizeof(struct ricoh583_battery_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	pdata = pdev->dev.platform_data;
	info->alarm_irq = pdata->irq_base + RICOH583_IRQ_VBATL + pdata->adc_channel;
	info->adc_irq = pdata->irq_base + RICOH583_IRQ_ADCEND;
	info->adc_channel = pdata->adc_channel;
	info->multiple = pdata->multiple;
	info->alarm_mvolts = pdata->alarm_mvolts;
	info->power_off_mvolts = pdata->power_off_mvolts;
	info->adc_vdd_mvolts = pdata->adc_vdd_mvolts; 
	info->min_mvolts = RICOH583_MIN_MVOLTS;
	info->max_mvolts = RICOH583_MAX_MVOLTS;
	
	if (info->alarm_mvolts == 0){
		info->alarm_mvolts = RICOH583_ALARM_MVOLTS;
	}
	if (info->power_off_mvolts == 0){
		info->power_off_mvolts = RICOH583_POWER_OFF_MVOLTS;
	}
	if (info->adc_vdd_mvolts == 0){
		info->adc_vdd_mvolts = RICOH583_ADC_VDD_MVOLTS;
	}

	platform_set_drvdata(pdev, info);
	g_info = info;

	ret = request_threaded_irq(info->alarm_irq, NULL, ricoh583_battery_alarm,
			IRQF_ONESHOT, "ricoh583_battery_alarm", info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for ricoh583 battery: %d\n",
				info->alarm_irq, ret);
		goto out;
	}
	ret = request_threaded_irq(info->adc_irq, NULL, ricoh583_battery_adc,
			IRQF_ONESHOT, "ricoh583_battery_adc", info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for ricoh583 battery: %d\n",
				info->adc_irq, ret);
		goto alarm_irq;
	}

	mutex_init(&info->lock);

	ret = ricoh583_init_battery(info);
	if (ret){
		goto irq;
	}

	ricoh583_battery_gauge.resistor_mohm = pdata->resistor_mohm;
	ret = ns115_battery_gauge_register(&ricoh583_battery_gauge);
	if (ret){
		goto irq;
	}

	dev_info(info->dev, "%s is OK!\n", __func__);

	return 0;

irq:
	free_irq(info->adc_irq, info);
alarm_irq:
	free_irq(info->alarm_irq, info);
out:
	kfree(info);
	return ret;
}

static int __devexit ricoh583_battery_remove(struct platform_device *pdev)
{
	struct ricoh583_battery_info *info = platform_get_drvdata(pdev);

	free_irq(info->alarm_irq, info);
	free_irq(info->adc_irq, info);
	kfree(info);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int ricoh583_battery_suspend(struct device *dev)
{
	struct ricoh583_battery_info *info = dev_get_drvdata(dev);

	if (info->alarm_mode == ALARM_LOW && info->cur_mvolts < info->alarm_mvolts){
		ricoh583_alarm_mvolts_set(info, info->power_off_mvolts);
		info->alarm_mode = ALARM_OFF;
	}else if (info->alarm_mode == ALARM_OFF && info->cur_mvolts > info->alarm_mvolts){
		ricoh583_alarm_mvolts_set(info, info->alarm_mvolts);
		info->alarm_mode = ALARM_LOW;
	}
	ricoh583_adc_mode_set(info, ADC_MODE_AUTO);

	return 0;
}

static int ricoh583_battery_resume(struct device *dev)
{
	struct ricoh583_battery_info *info = dev_get_drvdata(dev);
	ricoh583_adc_mode_set(info, ADC_MODE_OFF);

	return 0;
}

static struct dev_pm_ops ricoh583_battery_pm_ops = {
	.suspend	= ricoh583_battery_suspend,
	.resume		= ricoh583_battery_resume,
};
#endif

static struct platform_driver ricoh583_battery_driver = {
	.driver		= {
		.name	= "ricoh583-battery",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &ricoh583_battery_pm_ops,
#endif
	},
	.probe		= ricoh583_battery_probe,
	.remove		= __devexit_p(ricoh583_battery_remove),
};

static int __init ricoh583_battery_init(void)
{
	return platform_driver_register(&ricoh583_battery_driver);
}
device_initcall(ricoh583_battery_init);

static void __exit ricoh583_battery_exit(void)
{
	platform_driver_unregister(&ricoh583_battery_driver);
}
module_exit(ricoh583_battery_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("ricoh583 Battery driver");
MODULE_LICENSE("GPL");
