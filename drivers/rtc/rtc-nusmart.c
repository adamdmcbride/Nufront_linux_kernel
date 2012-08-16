/*
 * rtc-nusmart.c -- nusmart Real Time Clock interface
 *
 * Copyright (C) 2010 Nufront Software, Inc
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <asm/delay.h>
#define	 PRINT_DBG_INFO

#ifdef	PRINT_DBG_INFO 
	#define DBG_PRINT(fmt, args...) printk( KERN_EMERG "nusmart rtc: " fmt, ## args) 
#else 
	#define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif 
/*
 * RTC block register offsets 
 */
#define	RTC_CCVR_REG				 0x00
#define	RTC_CMR_REG				 0x04
#define	RTC_CLR_REG				 0x08
#define	RTC_CCR_REG				 0x0c
#define	RTC_STAT_REG				 0x10
#define	RTC_RSTAT_REG				 0x14
#define	RTC_EOI_REG				 0x18

/* RTC_CTRL_REG bitfields */
#define BIT_RTC_CTRL_REG_INT_EN			 0x01
#define BIT_RTC_CTRL_REG_INT_MASK		 0x02
#define BIT_RTC_CTRL_REG_RTC_EN			 0x04
#define BIT_RTC_CTRL_REG_WRAP_EN		 0x08

/* RTC_STATUS_REG bitfields */
#define BIT_RTC_STATUS_REG_INT_ACTIVE            0x01

/*----------------------------------------------------------------------*/

struct nusmart_rtc {
	struct rtc_device	*rtc;
	int 			irq;  	
	struct resource 	*mem_res;
	void __iomem		*base;
};


void nusmart_rtc_alarmirq_disable(struct nusmart_rtc *rtc)
{
	unsigned int ccr = readl(rtc->base + RTC_CCR_REG);

	ccr &= ~BIT_RTC_CTRL_REG_INT_EN;

	writel(ccr, rtc->base + RTC_CCR_REG);
	DBG_PRINT("alarm irq diable, RTC_CCR_REG:  0x%x\n", 
			readl(rtc->base + RTC_CCR_REG));
}

void nusmart_rtc_alarmirq_enable(struct nusmart_rtc *rtc)
{
	unsigned int ccr = readl(rtc->base + RTC_CCR_REG);

	ccr |= BIT_RTC_CTRL_REG_INT_EN;

	writel(ccr, rtc->base + RTC_CCR_REG);
	DBG_PRINT("alarm irq eable, RTC_CCR_REG:  0x%x\n", 
			readl(rtc->base + RTC_CCR_REG));
}

static irqreturn_t nusmart_rtc_interrupt(int irq, void * context)
{
	struct nusmart_rtc *rtc = context;
	readl(rtc->base + RTC_EOI_REG);

	DBG_PRINT("nusmart rtc interrupt ...\n");
	rtc_update_irq(rtc->rtc, 1, RTC_AF | RTC_IRQF);
	
	return IRQ_HANDLED;
}

static int nusmart_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct nusmart_rtc *rtc = dev_get_drvdata(dev);

	switch(cmd) {
	case RTC_AIE_OFF:
		nusmart_rtc_alarmirq_disable(rtc);
		break;
	case RTC_AIE_ON:
		nusmart_rtc_alarmirq_enable(rtc);
		break;
	default:
		DBG_PRINT("cmd = %d\n", cmd);
		return -ENOIOCTLCMD;
	}
	return 0;
}

static int nusmart_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct nusmart_rtc *rtc = dev_get_drvdata(dev);

	rtc_time_to_tm(readl(rtc->base + RTC_CMR_REG), &alrm->time);

	DBG_PRINT("read alarm\n");
	return 0;
}

static int nusmart_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct nusmart_rtc *rtc = dev_get_drvdata(dev);
	unsigned long time;
	int ret;

	DBG_PRINT("set alarm, raw_stat =%x , stat = %x\n", 
			readl(rtc->base + RTC_RSTAT_REG), readl(rtc->base + RTC_STAT_REG));
	/*
	 * At the moment, we can only deal with non-wildcarded alarm times.
	 */
	ret = rtc_valid_tm(&alrm->time);
	if (ret == 0)
		ret = rtc_tm_to_time(&alrm->time, &time);
	if (ret == 0)
		writel(time, rtc->base + RTC_CMR_REG);
	return ret;
}

static int nusmart_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct nusmart_rtc *rtc = dev_get_drvdata(dev);

	rtc_time_to_tm(readl(rtc->base + RTC_CCVR_REG), tm);

	DBG_PRINT("read time:  %d-%02d-%02d %02d:%02d:%02d\n",
				tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
				tm->tm_hour, tm->tm_min, tm->tm_sec);
	return 0;
}

/*
 * Set the RTC time.  Unfortunately, we can't accurately set
 * the point at which the counter updates.
 *
 * Also, since RTC_CLR is transferred to RTC_CR on next rising
 * edge of the 1Hz clock, we must write the time one second
 * in advance.
 */
static int nusmart_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct nusmart_rtc *rtc = dev_get_drvdata(dev);
	unsigned long time;
	int ret;

	DBG_PRINT("write time:  %d-%02d-%02d %02d:%02d:%02d\n",
				tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
				tm->tm_hour, tm->tm_min, tm->tm_sec);
	ret = rtc_tm_to_time(tm, &time);
	if (ret == 0)
		writel(time, rtc->base + RTC_CLR_REG);

	return ret;
}

static struct rtc_class_ops nusmart_rtc_ops = {
	.ioctl		= nusmart_rtc_ioctl,
	.read_time	= nusmart_rtc_read_time,
	.set_time	= nusmart_rtc_set_time,
	.read_alarm	= nusmart_rtc_read_alarm,
	.set_alarm	= nusmart_rtc_set_alarm,
};

/*----------------------------------------------------------------------*/

static int __devinit nusmart_rtc_probe(struct platform_device *pdev)
{
	struct nusmart_rtc  *rtc;
	int irq = 0, loop_count = 0; 	
	struct resource * mem_res;
	int ret = 0;
	struct rtc_time tm;
	unsigned long time;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0){
		ret = -ENXIO;
		DBG_PRINT("platform_get_irq fail!\n");
		goto err_out;
	}

	DBG_PRINT("platform_get_irq success, irq = %d\n", irq);

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		ret = -EINVAL;
		DBG_PRINT("platform_get_resource fail!\n");
		goto err_out;
	}

	DBG_PRINT("platform_get_resource success, raw mem_base = %x\n", 
				mem_res->start);

	if (!request_mem_region(mem_res->start, SZ_4K, pdev->name)){
		ret = -EBUSY;
		goto err_out;
	}

	DBG_PRINT("platform device name = %s\n", pdev->name);

	rtc = kmalloc(sizeof(*rtc), GFP_KERNEL);
	if (!rtc) {
		ret = -ENOMEM;
		goto err_rtc;
	}

	DBG_PRINT("kmalloc success rtc = %x\n", (unsigned int)rtc);

	rtc->irq = irq;
	rtc->mem_res = mem_res;

	rtc->base = ioremap(mem_res->start, SZ_4K);
	if (!rtc->base) {
		ret = -ENOMEM;
		goto err_map;
	}

	DBG_PRINT("ioremap success, rtc->base = %x\n", (unsigned int)rtc->base);
	readl(rtc->base + RTC_EOI_REG);

#if 0
	rtc_time_to_tm(readl(rtc->base + RTC_CCVR_REG), &tm);

	if((tm.tm_year == 70) && (tm.tm_mon == 0) && (tm.tm_mday == 1)) {
		tm.tm_mday = 8;
		rtc_tm_to_time(&tm, &time);
		writel(time, rtc->base + RTC_CLR_REG);
		udelay(200);
       		 while(readl(rtc->base + RTC_CCVR_REG) < time) {
			udelay(10);
			loop_count ++;
			if(loop_count >= 140000)
				break;
		}
		rtc_time_to_tm(readl(rtc->base + RTC_CCVR_REG), &tm);
		DBG_PRINT("read current day = %d, loop = %d \n", tm.tm_mday, loop_count);
	}
#endif

#ifdef	PRINT_DBG_INFO
//	 writel(0x4d114000, rtc->base + RTC_CLR_REG);
 //       while(readl(rtc->base + RTC_CCVR_REG) < 0x1000);
 
	DBG_PRINT("RTC_CCVR_REG: 0x%x\n", readl(rtc->base + RTC_CCVR_REG));
	DBG_PRINT("RTC_CMP_REG:  0x%x\n", readl(rtc->base + RTC_CMR_REG));
	DBG_PRINT("RTC_CLR_REG:  0x%x\n", readl(rtc->base + RTC_CLR_REG));
	DBG_PRINT("RTC_CCR_REG:  0x%x\n", readl(rtc->base + RTC_CCR_REG));
	DBG_PRINT("RTC_STAT_REG: 0x%x\n", readl(rtc->base + RTC_STAT_REG));
	DBG_PRINT("RTC_STAT_REG: 0x%x\n", readl(rtc->base + RTC_RSTAT_REG));
#endif

	platform_set_drvdata(pdev, rtc);

	rtc->rtc = rtc_device_register(pdev->name,
				  &pdev->dev, &nusmart_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "can't register RTC device, err %ld\n",
			PTR_ERR(rtc));
		goto err_reg;

	}

	DBG_PRINT("rtc_device_register  success\n");

	ret = request_irq(irq, nusmart_rtc_interrupt, IRQF_SHARED, 
				pdev->name, rtc);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ is not free.\n");
		goto err_irq;
	}

	DBG_PRINT("request_irq success\n");

	nusmart_rtc_alarmirq_enable(rtc);

	return ret;

 err_irq:
	rtc_device_unregister(rtc->rtc);
 err_reg:
	iounmap(rtc->base);
 err_map:
	kfree(rtc);
 err_rtc:
	release_mem_region(mem_res->start, SZ_4K);
 err_out:
	return ret;
}

/*
 * Disable all Nusmart RTC module interrupts.
 * Sets status flag to free.
 */
static int __devexit nusmart_rtc_remove(struct platform_device *pdev)
{
	struct nusmart_rtc *rtc = platform_get_drvdata(pdev);

	DBG_PRINT("numart_rtc_remove\n");

	platform_set_drvdata(pdev, NULL);
	free_irq(rtc->irq, rtc);

	rtc_device_unregister(rtc->rtc);

	iounmap(rtc->base);
	release_mem_region(rtc->mem_res->start, SZ_4K);
	kfree(rtc);
	return 0;
}

#ifdef CONFIG_PM

static int nusmart_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nusmart_rtc_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define nusmart_rtc_suspend NULL
#define nusmart_rtc_resume  NULL
#endif

MODULE_ALIAS("platform:nusmart_rtc");

static struct platform_driver nusmart_driver = {
	.probe		= nusmart_rtc_probe,
	.remove		= __devexit_p(nusmart_rtc_remove),
	.suspend	= nusmart_rtc_suspend,
	.resume		= nusmart_rtc_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ns2816-rtc",
	},
};

static int __init nusmart_rtc_init(void)
{
	DBG_PRINT("nusmart_rtc_init enter... \n");
	return platform_driver_register(&nusmart_driver);
}

static void __exit nusmart_rtc_exit(void)
{
	platform_driver_unregister(&nusmart_driver);
	DBG_PRINT("nusmart_rtc_exit exit... \n");
}

module_init(nusmart_rtc_init);
module_exit(nusmart_rtc_exit);

MODULE_AUTHOR("Nufront Software");
MODULE_LICENSE("GPL");

