/*
 * rtc-tps80032.c -- tps8032 Real Time Clock interface
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <linux/mfd/tps80032.h>

/*
 * RTC block register offsets 
 */
enum {
	REG_SECONDS_REG = 0,
	REG_MINUTES_REG,
	REG_HOURS_REG,
	REG_DAYS_REG,
	REG_MONTHS_REG,
	REG_YEARS_REG,
	REG_WEEKS_REG,

	REG_ALARM_SECONDS_REG,
	REG_ALARM_MINUTES_REG,
	REG_ALARM_HOURS_REG,
	REG_ALARM_DAYS_REG,
	REG_ALARM_MONTHS_REG,
	REG_ALARM_YEARS_REG,

	REG_RTC_CTRL_REG,
	REG_RTC_STATUS_REG,
	REG_RTC_INTERRUPTS_REG,

	REG_RTC_COMP_LSB_REG,
	REG_RTC_COMP_MSB_REG,
};

static const u8 tps80032_rtc_reg_map[] = {
	[REG_SECONDS_REG] = 0x00,
	[REG_MINUTES_REG] = 0x01,
	[REG_HOURS_REG] = 0x02,
	[REG_DAYS_REG] = 0x03,
	[REG_MONTHS_REG] = 0x04,
	[REG_YEARS_REG] = 0x05,
	[REG_WEEKS_REG] = 0x06,

	[REG_ALARM_SECONDS_REG] = 0x08,
	[REG_ALARM_MINUTES_REG] = 0x09,
	[REG_ALARM_HOURS_REG] = 0x0A,
	[REG_ALARM_DAYS_REG] = 0x0B,
	[REG_ALARM_MONTHS_REG] = 0x0C,
	[REG_ALARM_YEARS_REG] = 0x0D,

	[REG_RTC_CTRL_REG] = 0x10,
	[REG_RTC_STATUS_REG] = 0x11,
	[REG_RTC_INTERRUPTS_REG] = 0x12,

	[REG_RTC_COMP_LSB_REG] = 0x13,
	[REG_RTC_COMP_MSB_REG] = 0x14,
};

/* RTC_CTRL_REG bitfields */
#define BIT_RTC_CTRL_REG_STOP_RTC_M              0x01
#define BIT_RTC_CTRL_REG_ROUND_30S_M             0x02
#define BIT_RTC_CTRL_REG_AUTO_COMP_M             0x04
#define BIT_RTC_CTRL_REG_MODE_12_24_M            0x08
#define BIT_RTC_CTRL_REG_TEST_MODE_M             0x10
#define BIT_RTC_CTRL_REG_SET_32_COUNTER_M        0x20
#define BIT_RTC_CTRL_REG_GET_TIME_M              0x40

/* RTC_STATUS_REG bitfields */
#define BIT_RTC_STATUS_REG_RUN_M                 0x02
#define BIT_RTC_STATUS_REG_1S_EVENT_M            0x04
#define BIT_RTC_STATUS_REG_1M_EVENT_M            0x08
#define BIT_RTC_STATUS_REG_1H_EVENT_M            0x10
#define BIT_RTC_STATUS_REG_1D_EVENT_M            0x20
#define BIT_RTC_STATUS_REG_ALARM_M               0x40
#define BIT_RTC_STATUS_REG_POWER_UP_M            0x80

/* RTC_INTERRUPTS_REG bitfields */
#define BIT_RTC_INTERRUPTS_REG_EVERY_M           0x03
#define BIT_RTC_INTERRUPTS_REG_IT_TIMER_M        0x04
#define BIT_RTC_INTERRUPTS_REG_IT_ALARM_M        0x08


/* REG_SECONDS_REG through REG_YEARS_REG is how many registers? */
#define ALL_TIME_REGS		6
#define RTC_REG_ID		1

/*----------------------------------------------------------------------*/
static u8  *rtc_reg_map;

/*
 * Supports 1 byte read from TPS80032 RTC register.
 */
static int tps80032_rtc_read(u8 *data, u8 reg)
{
	int ret;

	ret = tps80032_read(RTC_REG_ID, rtc_reg_map[reg], data);
	if (ret < 0)
		pr_err("tps80032_rtc: Could not read"
		       "register %X - error %d\n", reg, ret);
	return ret;
}

/*
 * Supports 1 byte write to TPS80032 RTC registers.
 */
static int tps80032_rtc_write(u8 data, u8 reg)
{
	int ret;

	ret = tps80032_write(RTC_REG_ID, rtc_reg_map[reg], data);
	if (ret < 0)
		pr_err("tps80032_rtc: Could not write TPS80032"
		       "register %X - error %d\n", reg, ret);
	return ret;
}

/*
 * Cache the value for timer/alarm interrupts register; this is
 * only changed by callers holding rtc ops lock (or resume).
 */
static unsigned char rtc_irq_bits;

/*
 * Enable 1/second update and/or alarm interrupts.
 */
static int set_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;
	int ret;

	val = rtc_irq_bits | bit;
	val &= ~BIT_RTC_INTERRUPTS_REG_EVERY_M;
	ret = tps80032_rtc_write(val, REG_RTC_INTERRUPTS_REG);
	if (ret == 0)
		rtc_irq_bits = val;

	return ret;
}

/*
 * Disable update and/or alarm interrupts.
 */
static int mask_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;
	int ret;

	val = rtc_irq_bits & ~bit;
	ret = tps80032_rtc_write(val, REG_RTC_INTERRUPTS_REG);
	if (ret == 0)
		rtc_irq_bits = val;

	return ret;
}

static int tps80032_rtc_alarm_irq_enable(struct device *dev, unsigned enabled)
{
	int ret;

	if (enabled)
		ret = set_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
	else
		ret = mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);

	return ret;
}

/*
 * Gets current TPS80032 RTC time and date parameters.
 *
 * The RTC's time/alarm representation is not what gmtime(3) requires
 * Linux to use:
 *
 *  - Months are 1..12 vs Linux 0-11
 *  - Years are 0..99 vs Linux 1900..N (we assume 21st century)
 */
static int tps80032_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;
	u8 save_control;

	ret = tps80032_rtc_read(&save_control, REG_RTC_CTRL_REG);
	if (ret < 0)
		return ret;

	save_control |= BIT_RTC_CTRL_REG_GET_TIME_M;

	ret = tps80032_rtc_write(save_control, REG_RTC_CTRL_REG);
	if (ret < 0)
		return ret;

	ret = tps80032_bulk_reads(RTC_REG_ID, rtc_reg_map[REG_SECONDS_REG], 
			ALL_TIME_REGS, rtc_data);

	if (ret < 0) {
		dev_err(dev, "rtc_read_time error %d\n", ret);
		return ret;
	}

	tm->tm_sec = bcd2bin(rtc_data[0]);
	tm->tm_min = bcd2bin(rtc_data[1]);
	tm->tm_hour = bcd2bin(rtc_data[2]);
	tm->tm_mday = bcd2bin(rtc_data[3]);
	tm->tm_mon = bcd2bin(rtc_data[4]) - 1;
	tm->tm_year = bcd2bin(rtc_data[5]) + 100;

	return ret;
}

static int tps80032_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char save_control;
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;

	rtc_data[1] = bin2bcd(tm->tm_sec);
	rtc_data[2] = bin2bcd(tm->tm_min);
	rtc_data[3] = bin2bcd(tm->tm_hour);
	rtc_data[4] = bin2bcd(tm->tm_mday);
	rtc_data[5] = bin2bcd(tm->tm_mon + 1);
	rtc_data[6] = bin2bcd(tm->tm_year - 100);

	/* Stop RTC while updating the TC registers */
	ret = tps80032_rtc_read(&save_control, REG_RTC_CTRL_REG);
	if (ret < 0)
		goto out;

	save_control &= ~BIT_RTC_CTRL_REG_STOP_RTC_M;
	tps80032_rtc_write(save_control, REG_RTC_CTRL_REG);
	if (ret < 0)
		goto out;

	/* update all the time registers in one shot */
	ret = tps80032_bulk_writes(RTC_REG_ID, (rtc_reg_map[REG_SECONDS_REG]),
			ALL_TIME_REGS, rtc_data);
	if (ret < 0) {
		dev_err(dev, "rtc_set_time error %d\n", ret);
		goto out;
	}

	/* Start back RTC */
	save_control |= BIT_RTC_CTRL_REG_STOP_RTC_M;
	ret = tps80032_rtc_write(save_control, REG_RTC_CTRL_REG);

out:
	return ret;
}

/*
 * Gets current TPS80032 RTC alarm time.
 */
static int tps80032_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;

	ret = tps80032_bulk_reads(RTC_REG_ID, (rtc_reg_map[REG_ALARM_SECONDS_REG]),
			ALL_TIME_REGS, rtc_data);
	if (ret < 0) {
		dev_err(dev, "rtc_read_alarm error %d\n", ret);
		return ret;
	}

	/* some of these fields may be wildcard/"match all" */
	alm->time.tm_sec = bcd2bin(rtc_data[0]);
	alm->time.tm_min = bcd2bin(rtc_data[1]);
	alm->time.tm_hour = bcd2bin(rtc_data[2]);
	alm->time.tm_mday = bcd2bin(rtc_data[3]);
	alm->time.tm_mon = bcd2bin(rtc_data[4]) - 1;
	alm->time.tm_year = bcd2bin(rtc_data[5]) + 100;

	/* report cached alarm enable state */
	if (rtc_irq_bits & BIT_RTC_INTERRUPTS_REG_IT_ALARM_M)
		alm->enabled = 1;

	return ret;
}

static int tps80032_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned char alarm_data[ALL_TIME_REGS + 1];
	int ret;

	ret = tps80032_rtc_alarm_irq_enable(dev, 0);
	if (ret)
		goto out;

	alarm_data[1] = bin2bcd(alm->time.tm_sec);
	alarm_data[2] = bin2bcd(alm->time.tm_min);
	alarm_data[3] = bin2bcd(alm->time.tm_hour);
	alarm_data[4] = bin2bcd(alm->time.tm_mday);
	alarm_data[5] = bin2bcd(alm->time.tm_mon + 1);
	alarm_data[6] = bin2bcd(alm->time.tm_year - 100);

	/* update all the alarm registers in one shot */
	ret = tps80032_bulk_writes(RTC_REG_ID, (rtc_reg_map[REG_ALARM_SECONDS_REG]), 
			ALL_TIME_REGS, alarm_data);
	if (ret) {
		dev_err(dev, "rtc_set_alarm error %d\n", ret);
		goto out;
	}

	if (alm->enabled)
		ret = tps80032_rtc_alarm_irq_enable(dev, 1);
out:
	return ret;
}

static irqreturn_t tps80032_rtc_interrupt(int irq, void *rtc)
{
	unsigned long events = 0;
	int ret = IRQ_NONE;
	int res;
	u8 rd_reg;

#ifdef CONFIG_LOCKDEP
	/* WORKAROUND for lockdep forcing IRQF_DISABLED on us, which
	 * we don't want and can't tolerate.  Although it might be
	 * friendlier not to borrow this thread context...
	 */
	local_irq_enable();
#endif

	res = tps80032_rtc_read(&rd_reg, REG_RTC_STATUS_REG);
	if (res)
		goto out;
	/*
	 * Figure out source of interrupt: ALARM or TIMER in RTC_STATUS_REG.
	 * only one (ALARM or RTC) interrupt source may be enabled
	 * at time, we also could check our results
	 * by reading RTS_INTERRUPTS_REGISTER[IT_TIMER,IT_ALARM]
	 */
	if (rd_reg & BIT_RTC_STATUS_REG_ALARM_M)
		events |= RTC_IRQF | RTC_AF;
	else
		events |= RTC_IRQF | RTC_UF;

	res = tps80032_rtc_write(rd_reg | BIT_RTC_STATUS_REG_ALARM_M,
				   REG_RTC_STATUS_REG);
	if (res)
		goto out;

	/* Notify RTC core on event */
	rtc_update_irq(rtc, 1, events);

	ret = IRQ_HANDLED;
out:
	return ret;
}

static struct rtc_class_ops tps80032_rtc_ops = {
	.read_time	= tps80032_rtc_read_time,
	.set_time	= tps80032_rtc_set_time,
	.read_alarm	= tps80032_rtc_read_alarm,
	.set_alarm	= tps80032_rtc_set_alarm,
	.alarm_irq_enable = tps80032_rtc_alarm_irq_enable,
};

/*----------------------------------------------------------------------*/

static int __devinit tps80032_rtc_probe(struct platform_device *pdev)
{
	struct tps80032_rtc_platform_data *pdata = pdev->dev.platform_data;
	struct rtc_device *rtc;
	int ret = 0;
	int irq;
	u8 rd_reg;


	irq = pdata->irq; 
	rtc = rtc_device_register(pdev->name,
				  &pdev->dev, &tps80032_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = PTR_ERR(rtc);
		dev_err(&pdev->dev, "can't register RTC device, err %ld\n",
			PTR_ERR(rtc));
		goto out0;

	}

	platform_set_drvdata(pdev, rtc);

	ret = tps80032_rtc_read(&rd_reg, REG_RTC_STATUS_REG);
	if (ret < 0)
		goto out1;

	if (rd_reg & BIT_RTC_STATUS_REG_POWER_UP_M)
		dev_warn(&pdev->dev, "Power up reset detected.\n");

	if (rd_reg & BIT_RTC_STATUS_REG_ALARM_M)
		dev_warn(&pdev->dev, "Pending Alarm interrupt detected.\n");

	/* Clear RTC Power up reset and pending alarm interrupts */
	ret = tps80032_rtc_write(rd_reg, REG_RTC_STATUS_REG);
	if (ret < 0)
		goto out1;

	ret = request_threaded_irq(irq, NULL, tps80032_rtc_interrupt,
				IRQF_ONESHOT, dev_name(&rtc->dev), rtc);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ is not free.\n");
		goto out1;
	}

	/* Check RTC module status, Enable if it is off */
	ret = tps80032_rtc_read(&rd_reg, REG_RTC_CTRL_REG);
	if (ret < 0)
		goto out2;

	if (!(rd_reg & BIT_RTC_CTRL_REG_STOP_RTC_M)) {
		dev_info(&pdev->dev, "Enabling TPS80032-RTC.\n");
		rd_reg = BIT_RTC_CTRL_REG_STOP_RTC_M;
		ret = tps80032_rtc_write(rd_reg, REG_RTC_CTRL_REG);
		if (ret < 0)
			goto out2;
	}

	/* init cached IRQ enable bits */
	ret = tps80032_rtc_read(&rtc_irq_bits, REG_RTC_INTERRUPTS_REG);
	if (ret < 0)
		goto out2;
	dev_info(&pdev->dev, "%s is OK!\n", __func__);

	return 0;

out2:
	free_irq(irq, rtc);
out1:
	rtc_device_unregister(rtc);
out0:
	return ret;
}

/*
 * Disable all TPS80032 RTC module interrupts.
 * Sets status flag to free.
 */
static int __devexit tps80032_rtc_remove(struct platform_device *pdev)
{
	/* leave rtc running, but disable irqs */
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);

	free_irq(irq, rtc);

	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void tps80032_rtc_shutdown(struct platform_device *pdev)
{
	/* mask timer interrupts, but leave alarm interrupts on to enable
	   power-on when alarm is triggered */
	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);
}

#ifdef CONFIG_PM

static unsigned char irqstat;

static int tps80032_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	irqstat = rtc_irq_bits;

	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);
	return 0;
}

static int tps80032_rtc_resume(struct platform_device *pdev)
{
	set_rtc_irq_bit(irqstat);
	return 0;
}

#else
#define tps80032_rtc_suspend NULL
#define tps80032_rtc_resume  NULL
#endif

MODULE_ALIAS("platform:tps80032-rtc");

static struct platform_driver tps80032rtc_driver = {
	.probe		= tps80032_rtc_probe,
	.remove		= __devexit_p(tps80032_rtc_remove),
	.shutdown	= tps80032_rtc_shutdown,
	.suspend	= tps80032_rtc_suspend,
	.resume		= tps80032_rtc_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "rtc-tps80032",
	},
};

static int __init tps80032_rtc_init(void)
{
	rtc_reg_map = (u8 *) tps80032_rtc_reg_map;

	return platform_driver_register(&tps80032rtc_driver);
}
module_init(tps80032_rtc_init);

static void __exit tps80032_rtc_exit(void)
{
	platform_driver_unregister(&tps80032rtc_driver);
}
module_exit(tps80032_rtc_exit);

MODULE_AUTHOR("Wu Jianguo<jianguo.wu@nufront.com>");
MODULE_DESCRIPTION("TPS80032 RTC driver");
MODULE_LICENSE("GPL");
