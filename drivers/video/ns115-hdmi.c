/*
 * NuSmart High-Definition Multimedia Interface (HDMI) driver
 * for SiliconImage HDMI 1.4a Tx Advanced IP cores
 *
 * Copyright (C) 2012, wangzhi <zhi.wang@nufront.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/gpio.h>
#include "ns115-hdmi.h"
#include <mach/irqs-ns115.h>
#include <mach/board-ns115.h>
#include <mach/hardware.h>
#include <linux/ioport.h>
#include <linux/i2c.h>
#include <linux/earlysuspend.h>
#include "edid.h"

///////////#define NS115_HDMI_INTERRUPT_GPIO_EDGE
//#define NS115_HDMI_INTERRUPT_GPIO_LEVEL
#define GPIO_NS115_HDMI_INTERRUPT 4

//#define NS115_I2C_EDID

#if defined(NS115_I2C_EDID)
#define I2C_BUS_0 0
#define I2C_BUS_DDC I2C_BUS_0
#endif

#ifdef CONFIG_SWITCH
#define HDMI_SWITCH_CLASS
#endif

#define LCDC1_NOTIFIER_HDMI_SUPPORT

#ifdef CONFIG_SOUND
#define SND_NOTIFIER_HDMI_SUPPORT
#endif

const static u8 str_hdmi_plug[] = "1";
const static u8 str_hdmi_unplug[] = "0";
static struct ns115_hdmi_data *hdmi_pt = NULL;
static int hdmi_irq = 0;

static u8 hdmi_edid_buff[HDMI_EDID_MAX_LENGTH+1] = {0};
static struct ns115_hdmi_lcdc_data ns115_hdmi_lcdc;
//static u8 g_audio_Checksum;// Audio checksum

#if defined(LCDC1_NOTIFIER_HDMI_SUPPORT)
extern int send_hdmi_hotplug_event(unsigned long val, void *v);
#endif
static irqreturn_t ns115_hdmi_hotplug(int irq, void *dev_id);
void hdmi_notifier(int flag);
/*
DByte1:
F7 Y1 Y0 A0 B1 B0 S1 S0
0   0   0   0   0  0   0   0  
DByte2:
C1 C0 M1 M0 R3 R2 R1 R0
0    0   0   0    0   0  0  0

*/
static unsigned char avi_info_frame_data[] = {
	0x00,///1////////////DByte1
	0x00,///2
	0xA8,///3
	0x00,///4
	0x04,///5
	0x00,///6////////////
	0x00,///7
	0x00,///8
	0x00,///9
	0x00,///10
	0x00,///11
	0x00,///12
	0x00,///13
	0x00///14
};

static void ns115_hdmi_set_irq(int irq)
{
	hdmi_irq = irq;
}
static int ns115_hdmi_get_irq(void)
{
	return hdmi_irq;
}

#if defined(HDMI_SWITCH_CLASS)
static ssize_t switch_hdmi_print_state(struct switch_dev *sdev, char *buf)
{
	struct ns115_hdmi_data	*hdmi =
		container_of(sdev, struct ns115_hdmi_data, sdev);

	if (hdmi->hp_state == HDMI_HOTPLUG_CONNECTED)
	{
		return sprintf(buf, "%s\n", str_hdmi_plug);
	}
	else if (hdmi->hp_state == HDMI_HOTPLUG_DISCONNECTED)
	{
		return sprintf(buf, "%s\n", str_hdmi_unplug);
	}
	return -1;
}
#endif

/*
   1. Clear the service event.Write TPI 0x3D[1:0] = 11 to clear the event.
   2. Read and record the active pin states.TPI 0x3D[3:2] provide active state information for the HPD and RSEN signal levels.
   3. Determine the event that occurred.
   If the prior state was ¡°plugged-in¡±, an Unplug event has occurred. An interrupt is registered
   immediately in TPI 0x3D[1:0] for an Unplug event. TPI 0x3D[3:2] = 00 confirms an Unplug
   event; any other state indicates instability in the connection (which should be monitored
   until it becomes stable before proceeding).
   If TPI 0x3D[2] = 1, a Plug event has occurred and has likely stabilized. Interrupts due to
   Plug events are delayed until after the HPD signal has stabilized at a High level for
   ~500ms. Host software can choose to provide additional de-bouncing if desired (See Hot
   Plug Delay De-bounce).
   4. Take action as needed.
   For a confirmed Plug event, the host should use the Request / Grant protocol to take
   control of the DDC bus and read EDID
   - For an Unplug event, the host should stop sending all video. This step also ensures that no
   secure video content is being sent.

   Hotplug interrupt occurred, read EDID
   */
static void ns115_hdmi_hpd_work_fn(struct work_struct *work)
{
	struct ns115_hdmi_data *hdmi = container_of(work, struct ns115_hdmi_data, edid_work.work);

	unsigned char  InterruptStatusImage = 0;
	int err;
#if !defined(NS115_I2C_EDID)
	u8 SysCtrlReg;
	u8 edid_block_count = 1;
#endif
#if defined(LCDC1_NOTIFIER_HDMI_SUPPORT)
	int ret;
#endif
	bool b_edid = false;
#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
	unsigned long flags_irq = 0;
	int level_gpio = 0;
#endif

	mutex_lock(&hdmi->mutex);

#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
	free_irq(hdmi->irq, hdmi);
	printk(KERN_WARNING "[HDMI:wangzhi]%d:hr_msleep(100);.\r\n",__LINE__);
	hr_msleep(100);
	level_gpio = gpio_get_value(GPIO_NS115_HDMI_INTERRUPT);
	printk(KERN_WARNING "[HDMI:wangzhi]%d:level_gpio: 0x%X.\r\n",__LINE__, level_gpio);
	if (level_gpio == 1)
	{
		if (hdmi->hp_state == HDMI_HOTPLUG_DISCONNECTED)
		{
#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE)
			flags_irq = IRQF_TRIGGER_FALLING;
#elif defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
			flags_irq = IRQF_TRIGGER_LOW;
#endif
			printk(KERN_WARNING "[HDMI:wangzhi]%d:hdmi plug bounce!\r\n",__LINE__);
			enable_irq(hdmi->irq);
			goto err_hdmi_work;
		}
#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE)
		flags_irq = IRQF_TRIGGER_FALLING;
#elif defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
		flags_irq = IRQF_TRIGGER_LOW;
#endif
		ns115_hdmi_tmds_clock_disable(hdmi);
		ns115_hdmi_DisableTMDS(hdmi);

		hdmi->hp_state = HDMI_HOTPLUG_DISCONNECTED;
		printk(KERN_WARNING "[HDMI:wangzhi]%d:HDMI_HOTPLUG_DISCONNECTED!\r\n",__LINE__);
	}
	else
	{
		if (hdmi->hp_state == HDMI_HOTPLUG_CONNECTED)
		{
#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE)
			flags_irq = IRQF_TRIGGER_RISING;
#elif defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
			flags_irq = IRQF_TRIGGER_HIGH;
#endif
			printk(KERN_WARNING "[HDMI:wangzhi]%d:hdmi plug bounce!\r\n",__LINE__);
			enable_irq(hdmi->irq);
			goto err_hdmi_work;
		}
#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE)
		flags_irq = IRQF_TRIGGER_RISING;
#elif defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
		flags_irq = IRQF_TRIGGER_HIGH;
#endif
		ns115_hdmi_configure(hdmi);
		hdmi->hp_state = HDMI_HOTPLUG_CONNECTED;
		printk(KERN_WARNING "[HDMI:wangzhi]%d:HDMI_HOTPLUG_CONNECTED!\r\n",__LINE__);
	}
#else
	hr_msleep(500);
	InterruptStatusImage = ReadByteHDMI(hdmi,TPI_INTERRUPT_STATUS_REG);
	ReadSetWriteTPI(hdmi,TPI_INTERRUPT_STATUS_REG, (RECEIVER_SENSE_EVENT_ENDING |
				CONNECTION_EVENT_PENDING));// Clear this event
	printk(KERN_WARNING "[HDMI:wangzhi]%d:TPI_INTERRUPT_STATUS_REG: 0x%x.\r\n",__LINE__, InterruptStatusImage);

	if ((hdmi->hp_state == HDMI_HOTPLUG_CONNECTED) &&
			(((InterruptStatusImage >> 2) & 0x01) == 0x00))
	{
		ns115_hdmi_tmds_clock_disable(hdmi);
		ns115_hdmi_DisableTMDS(hdmi);
		hdmi->hp_state = HDMI_HOTPLUG_DISCONNECTED;
		printk(KERN_WARNING "[HDMI:wangzhi]%d:HDMI_HOTPLUG_DISCONNECTED!\r\n",__LINE__);
	}
	else if ((hdmi->hp_state == HDMI_HOTPLUG_DISCONNECTED) &&
			(((InterruptStatusImage >> 2) & 0x01) == 0x01))
	{
		printk(KERN_WARNING "[HDMI:wangzhi]%d:hr_msleep(500);.\r\n",__LINE__);
		hr_msleep(500);
		ns115_hdmi_configure(hdmi);
		//ns115_hdmi_tmds_clock_enable(hdmi);
		hdmi->hp_state = HDMI_HOTPLUG_CONNECTED;        
		printk(KERN_WARNING "[HDMI:wangzhi]%d:HDMI_HOTPLUG_CONNECTED!\r\n",__LINE__);
	}
	else
	{
		printk(KERN_WARNING "[HDMI:wangzhi]%d:hdmi plug bounce!\r\n",__LINE__);
		goto err_hdmi_work;
	}
#endif
#if defined(HDMI_SWITCH_CLASS)
	switch_set_state(&hdmi->sdev, hdmi->hp_state);
#endif

	/* HDMI plug in */
	if (hdmi->hp_state == HDMI_HOTPLUG_CONNECTED) 
	{
#if !defined(NS115_I2C_EDID)
		if (GetDDC_Access(hdmi,&SysCtrlReg))
		{
#endif
			err = ns115_hdmi_EDIDRead(hdmi,EDID_BLOCK_SIZE,hdmi_edid_buff,EDID_BLOCK_0_OFFSET);

			if (err < 0 ){
				printk(KERN_WARNING "[HDMI:wangzhi]Read EDID Error.\r\n");
			}
			else
			{
				b_edid = true;
#if !defined(NS115_I2C_EDID)
				if (hdmi_edid_buff[OFFSET_BYTE_END_OF_PADING] == 0x01)///End of Padding 
				{
					edid_block_count = 2;
					err = ns115_hdmi_EDIDRead(hdmi,EDID_BLOCK_SIZE,hdmi_edid_buff+EDID_BLOCK_SIZE,EDID_BLOCK_1_OFFSET);
				}
#endif
			}
#if !defined(NS115_I2C_EDID)
			if (!ReleaseDDC(hdmi,SysCtrlReg))// Host must release DDC bus once it is done reading EDID
			{
				printk(KERN_WARNING "[HDMI:wangzhi]EDID -> DDC bus release failed\n");
				//EDID_DDC_BUS_RELEASE_FAILURE;
				goto err_edid;
			}
#endif
#if !defined(NS115_I2C_EDID)
		}
		else
		{
			printk(KERN_WARNING "[HDMI:wangzhi]EDID -> DDC bus access failed.\r\n");
			goto err_edid;
		}
#endif
err_edid:
#if defined(SND_NOTIFIER_HDMI_SUPPORT)
		hdmi_notifier(1);
#endif
#if defined(LCDC1_NOTIFIER_HDMI_SUPPORT)
		printk(KERN_WARNING "==============================\n");
		if (b_edid == true)
		{
			ns115_hdmi_lcdc.pedid = hdmi_edid_buff;
		}
		else
		{
			ns115_hdmi_lcdc.pedid = NULL;
		}
		ret = send_hdmi_hotplug_event(1, &ns115_hdmi_lcdc);
		printk(KERN_WARNING "==============================\n");
		if (ret)
		{
			printk(KERN_WARNING "notifier_call_chain error\n");
		}
		else
		{
			ns115_hdmi_lcdc_video(ns115_hdmi_lcdc.horizontal_res,
					ns115_hdmi_lcdc.vertical_res,ns115_hdmi_lcdc.pixel_clk);

		}
#endif
	}
	/* HDMI disconnect */
	else if (hdmi->hp_state == HDMI_HOTPLUG_DISCONNECTED)
	{
#if defined(SND_NOTIFIER_HDMI_SUPPORT)
		hdmi_notifier(0);
#endif

#if defined(LCDC1_NOTIFIER_HDMI_SUPPORT)
		printk(KERN_WARNING "==============================\n");
		ret = send_hdmi_hotplug_event(2, NULL);
		printk(KERN_WARNING "==============================\n");
		if (ret)
			printk(KERN_WARNING "notifier_call_chain error\n");
#endif

	}
err_hdmi_work:
	mutex_unlock(&hdmi->mutex);
#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
	printk(KERN_WARNING "[HDMI:wangzhi]%d:flags_irq: 0x%X.\r\n",__LINE__, flags_irq);
	ret = request_irq(hdmi->irq, ns115_hdmi_hotplug, flags_irq,
			"ns115hdmi", hdmi);
#else
	enable_irq(hdmi->irq);
#endif
}

static irqreturn_t ns115_hdmi_hotplug(int irq, void *dev_id)
{
	int temp = 0;
	struct ns115_hdmi_data *hdmi = (struct ns115_hdmi_data*)dev_id;

	disable_irq_nosync(irq);

	temp = schedule_delayed_work(&hdmi->edid_work, 0);//msecs_to_jiffies(10)
	printk(KERN_WARNING "[HDMI:wangzhi]schedule_delayed_work:%d\r\n",temp);
	return IRQ_HANDLED;  
}

static void do_nsleep(unsigned int msecs, struct hrtimer_sleeper *sleeper,
		int sigs)
{
	enum hrtimer_mode mode = HRTIMER_MODE_REL;
	int state = sigs ? TASK_INTERRUPTIBLE : TASK_UNINTERRUPTIBLE;

	/*
	 * This is really just a reworked and simplified version
	 * of do_nanosleep().
	 */
	hrtimer_init(&sleeper->timer, CLOCK_MONOTONIC, mode);
	sleeper->timer.node.expires = ktime_set(msecs / 1000,
			(msecs % 1000) * NSEC_PER_MSEC);
	hrtimer_init_sleeper(sleeper, current);

	do {
		set_current_state(state);
		hrtimer_start(&sleeper->timer, sleeper->timer.node.expires, mode);
		if (sleeper->task)
			schedule();
		hrtimer_cancel(&sleeper->timer);
		mode = HRTIMER_MODE_ABS;
	} while (sleeper->task && !(sigs && signal_pending(current)));
}

/**
 * msleep - sleep safely even with waitqueue interruptions
 * @msecs: Time in milliseconds to sleep for
 */
void hr_msleep(unsigned int msecs)
{
	struct hrtimer_sleeper sleeper;

	do_nsleep(msecs, &sleeper, 0);
}

static u8 ReadByteHDMI(struct ns115_hdmi_data *hdmi,u8 RegOffset)
{
	return readb(hdmi->base + (RegOffset << 2));
}

static void WriteByteHDMI(struct ns115_hdmi_data *hdmi,u8 RegOffset, u8 Data)
{
	writeb(Data, hdmi->base + (RegOffset << 2));
	hr_msleep(1);
}

static u8 Readu16HDMI(struct ns115_hdmi_data *hdmi,u16 uc_regoffset)
{
	return readb(hdmi->base + (uc_regoffset << 2));
}

static void  Writeu16HDMI(struct ns115_hdmi_data *hdmi,u16 uc_regoffset, u8 uc_bufdata)
{
	writeb(uc_bufdata, hdmi->base + (uc_regoffset << 2));
}

static void ns115_hdmi_en(struct ns115_hdmi_data *hdmi)
{
	StartTPI(hdmi);

#if 1
	//WriteByteHDMI(hdmi,TPI_PIX_CLK_LSB, 0x00);// Set pixel rate for 74.24MHz (7424 = 0x1D00)  
	//WriteByteHDMI(hdmi,TPI_PIX_CLK_MSB, 0x1D);//
#else
	WriteByteHDMI(hdmi,TPI_PIX_CLK_LSB, 0xE4);// Set pixel rate for 225MHz (22500 = 0x57E4)  
	WriteByteHDMI(hdmi,TPI_PIX_CLK_MSB, 0x57);//
#endif

	ns115_hdmi_set_pt(hdmi);

	/*default setting:*/
	hdmi->sample_rate = 48000;
	hdmi->channels = 2;
	hdmi->resolution = 16;

	hdmi->pixel_clk = 74250000;
	hdmi->horizontal_res = 1280;//HV_SIZE///1280*720
	hdmi->vertical_res = 720;

	ns115_hdmi_configure(hdmi);

	//Hot Plug Delay De-bounce settting
	WriteIndexedRegister(hdmi,INDEXED_PAGE_1,REG_HOT_PLUG_DELAY_DEBOUNCE,
			SETTING_HOT_PLUG_DELAY_DEBOUNCE);
	//hr_msleep(5);
	ns115_hdmi_tmds_clock_enable(hdmi);
}

static bool StartTPI(struct ns115_hdmi_data *hdmi)
{
	u8 devID = 0x00;
	u16 wID = 0x0000;

	printk(KERN_WARNING "[HDMI:wangzhi]>>StartTPI()\n");

	WriteByteHDMI(hdmi,TPI_ENABLE, 0x00);// Write "0" to 72:C7 to start HW TPI mode

	//0x93
	devID = ReadIndexedRegister(hdmi,INDEXED_PAGE_0, TXL_PAGE_0_DEV_IDH_ADDR);

	wID = devID;
	wID <<= 8;
	//0x34
	devID = ReadIndexedRegister(hdmi,INDEXED_PAGE_0, TXL_PAGE_0_DEV_IDL_ADDR);
	wID |= devID;

	devID = ReadByteHDMI(hdmi,TPI_DEVICE_ID);//0xB4

	printk(KERN_WARNING "[HDMI:wangzhi]TPI_DEVICE_ID:0x%04X\n", (u16) wID);

	if (devID == SiI_DEVICE_ID)
	{
		printk (SiI_DEVICE_STRING);
		return true;
	}

	printk(KERN_WARNING "[HDMI:wangzhi]Unsupported TX\n");
	return false;
}

#ifdef CONFIG_EARLYSUSPEND
static void ns115_hdmi_early_suspend(struct early_suspend * h);
static void ns115_hdmi_early_resume(struct early_suspend * h);

static struct early_suspend es = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = ns115_hdmi_early_suspend,
	.resume = ns115_hdmi_early_resume,
};

static void ns115_hdmi_early_suspend(struct early_suspend * h)
{
	struct ns115_hdmi_data *hdmi = ns115_hdmi_get_pt();

	printk(KERN_WARNING "[HDMI:wangzhi]ns115_hdmi_early_suspend()\n");
#if !(defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL))
	DisableInterrupts(hdmi,HOT_PLUG_EVENT);
#endif
}

static void ns115_hdmi_early_resume(struct early_suspend * h)
{
	struct ns115_hdmi_data *hdmi = ns115_hdmi_get_pt();
	int irq = ns115_hdmi_get_irq();

	printk(KERN_WARNING "[HDMI:wangzhi]ns115_hdmi_early_resume()\n");

#if !(defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL))
	//	hdmi->hp_state = HDMI_HOTPLUG_DISCONNECTED;
	EnableInterrupts(hdmi,HOT_PLUG_EVENT);
#endif
	ns115_hdmi_hotplug(irq,hdmi);
}
#endif

static int __init ns115_hdmi_probe(struct platform_device *pdev)
{
#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
	int irq = gpio_to_irq(GPIO_NS115_HDMI_INTERRUPT);
#else
	int irq = platform_get_irq(pdev, 0);
#endif
	struct resource * res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int ret;
	struct ns115_hdmi_data *hdmi;

	if (irq < 0)
		return -ENODEV;

	hdmi = kzalloc(sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi) {
		dev_err(&pdev->dev, "Cannot allocate device data\n");
		return -ENOMEM;
	}

#if defined(HDMI_SWITCH_CLASS)
	hdmi->name_on = NULL;
	hdmi->name_off = NULL;
	hdmi->state_on = NULL;
	hdmi->state_off = NULL;
	hdmi->sdev.name = "hdmi";
	hdmi->sdev.print_state = switch_hdmi_print_state;
	hdmi->sdev.print_name = NULL;
#endif

#if defined(HDMI_SWITCH_CLASS)
	ret = switch_dev_register(&hdmi->sdev);
	if (ret < 0)
		goto err_switch_dev_register;
#endif

	mutex_init(&hdmi->mutex);
	hdmi->dev = &pdev->dev;

	hdmi->page_reg = PAGE_0_HDMI;

	hdmi->base = __io_address(res->start);

	hdmi->irq = irq;

	platform_set_drvdata(pdev, hdmi);

	INIT_DELAYED_WORK(&hdmi->edid_work,ns115_hdmi_hpd_work_fn);

	hdmi->hp_state = HDMI_HOTPLUG_DISCONNECTED;

	ns115_hdmi_en(hdmi);
	ns115_hdmi_set_irq(irq);

#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE)
	ret = gpio_request(GPIO_NS115_HDMI_INTERRUPT, "hpd_hdmi");
	gpio_direction_input(GPIO_NS115_HDMI_INTERRUPT);

	ret = request_irq(irq, ns115_hdmi_hotplug, IRQF_TRIGGER_FALLING,
			"ns115hdmi", hdmi);
#elif defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
	ret = gpio_request(GPIO_NS115_HDMI_INTERRUPT, "hpd_hdmi");
	gpio_direction_input(GPIO_NS115_HDMI_INTERRUPT);
	ret = request_irq(irq, ns115_hdmi_hotplug, IRQF_TRIGGER_LOW,
			"ns115hdmi", hdmi);
#else
	ret = request_irq(irq, ns115_hdmi_hotplug, 0,
			"ns115hdmi", hdmi);
#endif
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to request irq: %d\n", ret);
		goto err_release;
	}

#if !(defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL))
	EnableInterrupts(hdmi,HOT_PLUG_EVENT);
#endif
	ns115_hdmi_hotplug(irq,hdmi);

#ifdef CONFIG_EARLYSUSPEND
	register_early_suspend(&es);
#endif

	return 0;
err_release:
	mutex_destroy(&hdmi->mutex);
err_switch_dev_register:
	kfree(hdmi);
	return ret;
}

#if defined(CONFIG_PM)
int ns115_hdmi_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int ns115_hdmi_resume(struct platform_device *pdev)
{
#if defined(NS115_HDMI_INTERRUPT_GPIO)
	int irq = gpio_to_irq(4);
#else
	struct ns115_hdmi_data *hdmi = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);
	ns115_hdmi_en(hdmi);
	hr_msleep(500);
	hdmi->hp_state = HDMI_HOTPLUG_DISCONNECTED;
	EnableInterrupts(hdmi,HOT_PLUG_EVENT);
#endif
	ns115_hdmi_hotplug(irq,hdmi);
	return 0;
}
#endif

static struct platform_driver ns115_hdmi_driver = {
	.remove		= __exit_p(ns115_hdmi_remove),
	.driver = {
		.name	= "ns115-soc-hdmi",
	},
#if defined(CONFIG_PM)
	.suspend = ns115_hdmi_suspend,
	.resume = ns115_hdmi_resume,
#endif
};

static int __exit ns115_hdmi_remove(struct platform_device *pdev)
{
	struct ns115_hdmi_data *hdmi = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
	int hpd_gpio = irq_to_gpio(irq);
#endif

#if !(defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL))
	DisableInterrupts(hdmi,0xFF);
#endif

	printk(KERN_WARNING "[HDMI:wangzhi]ns115_hdmi_remove\r\n");

	/* No new work will be scheduled, wait for running ISR */
	free_irq(irq, hdmi);
#if defined(NS115_HDMI_INTERRUPT_GPIO_EDGE) || defined(NS115_HDMI_INTERRUPT_GPIO_LEVEL)
	gpio_free(hpd_gpio);
#endif
	cancel_delayed_work_sync(&hdmi->edid_work);

#if defined(HDMI_SWITCH_CLASS)
	switch_dev_unregister(&hdmi->sdev);
#endif
	mutex_destroy(&hdmi->mutex);
	kfree(hdmi);
	ns115_hdmi_set_pt(NULL);
	return 0;
}

static u8 ReadIndexedRegister(struct ns115_hdmi_data *hdmi,u8 PageNum, u8 RegOffset)
{
	WriteByteHDMI(hdmi,TPI_INTERNAL_PAGE_REG, PageNum);		// Internal page
	WriteByteHDMI(hdmi,TPI_INDEXED_OFFSET_REG, RegOffset);	// Indexed register
	return ReadByteHDMI(hdmi,TPI_INDEXED_VALUE_REG); 		// Return read value
}

static void WriteIndexedRegister(struct ns115_hdmi_data *hdmi,u8 PageNum, u8 RegOffset, u8 RegValue)
{
	WriteByteHDMI(hdmi,TPI_INTERNAL_PAGE_REG, PageNum);  // Internal page
	WriteByteHDMI(hdmi,TPI_INDEXED_OFFSET_REG, RegOffset);  // Indexed register
	WriteByteHDMI(hdmi,TPI_INDEXED_VALUE_REG, RegValue);    // Read value into buffer
}

static void ns115_hdmi_lcdc_video(u32 horizontal_res,
		u32 vertical_res,
		u32 pixel_clk)
{
	struct ns115_hdmi_data *hdmi = ns115_hdmi_get_pt();
	if ((hdmi != NULL) && (hdmi->hp_state == HDMI_HOTPLUG_CONNECTED))
	{
		printk(KERN_WARNING "[HDMI:wangzhi]horizontal_res:%d.\r\n",horizontal_res);
		printk(KERN_WARNING "[HDMI:wangzhi]vertical_res:%d.\r\n",vertical_res);    
		printk(KERN_WARNING "[HDMI:wangzhi]pixel_clk:%d.\r\n",pixel_clk);

		if ((hdmi->horizontal_res != horizontal_res) ||
				(hdmi->vertical_res != vertical_res) ||
				(hdmi->pixel_clk != pixel_clk))
		{
			hdmi->horizontal_res = horizontal_res;
			hdmi->vertical_res = vertical_res;
			hdmi->pixel_clk = pixel_clk;
			ns115_hdmi_configure(hdmi);
		}
	}

}

static void ns115_hdmi_video_config(struct ns115_hdmi_data *hdmi)
{
#if 0
	u8 B_Data[SIZE_AVI_INFOFRAME];
	u8 i;

	printk(KERN_WARNING "[HDMI:wangzhi]>>SetAVI_InfoFrames()\n");
	//
	// Disable transmission of AVI InfoFrames during re-configuration
	WriteByteHDMI(hdmi,MISC_INFO_FRAMES_CTRL, DISABLE_AVI);         //  Disbale MPEG/Vendor Specific InfoFrames

	for (i = 0; i < SIZE_AVI_INFOFRAME; i++)
		B_Data[i] = 0;

	//////////////////////////////////////////////////////////////////////////////////////////
	//  AVI InfoFrame is set by the FW, so we have some work to do to put it all in the proper
	//  format for the TPI 0x0C through 0x19 registers.:sed
	//////////////////////////////////////////////////////////////////////////////////////////

	//DB1
	// AVI InfoFrame DByte1 contains Output ColorSpace
	B_Data[1] = 0;

	//DB2
	B_Data[2] = 0;

	//DB3
	B_Data[3] = 0xA8;

	//DB4
	B_Data[4] = 0;

	//DB5
	B_Data[5] = 0x04;

	//////////////////////////////////////////////////////////////////////////////////////////
	// Calculate AVI InfoFrame ChecKsum
	//////////////////////////////////////////////////////////////////////////////////////////
	B_Data[0] = 0x82 + 0x02 + 0x0D;
	for (i = 1; i < SIZE_AVI_INFOFRAME; i++)
	{
		B_Data[0] += B_Data[i];
	}
	B_Data[0] = 0x100 - B_Data[0];

	//////////////////////////////////////////////////////////////////////////////////////////
	// Write the Inforframe data to the TPI Infoframe registers.  This automatically
	// Enables InfoFrame transmission and repeat by over-writing Register 0x19.
	//////////////////////////////////////////////////////////////////////////////////////////
	WriteBlockHDMI(hdmi,TPI_AVI_BYTE_0, SIZE_AVI_INFOFRAME, B_Data);
#else
	int i;
	unsigned char val, vals[14];
	unsigned short horizontal_res;
	unsigned short vertical_res;
	unsigned short pixel_clk;

	memset(vals, 0, sizeof(vals));

	// Fill the TPI Video Mode Data structure
	vals[0] = ((hdmi->pixel_clk>>16) & 0xFF);                   /* Pixel clock */
	vals[1] = (((hdmi->pixel_clk>>16) & 0xFF00) >> 8);
	vals[2] = VERTICAL_FREQ;                        /* Vertical freq */
	vals[3] = 0x00;
	vals[4] = (hdmi->horizontal_res & 0xFF);              /* Horizontal pixels*/
	vals[5] = ((hdmi->horizontal_res & 0xFF00) >> 8);
	vals[6] = (hdmi->vertical_res & 0xFF);                /* Vertical pixels */
	vals[7] = ((hdmi->vertical_res & 0xFF00) >> 8);

	/* Write out the TPI Video Mode Data */
	for (i = 0; i < 8; i ++) {
		WriteByteHDMI(hdmi,HDMI_TPI_VIDEO_DATA_BASE_REG + i, vals[i]);
	}
#if 0
	// Write out the TPI Pixel Repetition Data (24 bit wide bus, falling edge, no pixel replication)
	val = TPI_AVI_PIXEL_REP_BUS_24BIT | TPI_AVI_PIXEL_REP_FALLING_EDGE | TPI_AVI_PIXEL_REP_NONE;
	WriteByteHDMI(hdmi,HDMI_TPI_PIXEL_REPETITION_REG, val);
#endif
	// Write out the TPI Pixel Repetition Data (24 bit wide bus, falling edge, no pixel replication)
	val = TPI_AVI_PIXEL_REP_TCLLKSEL_x1 | TPI_AVI_PIXEL_REP_BUS_24BIT | TPI_AVI_PIXEL_REP_FALLING_EDGE | TPI_AVI_PIXEL_REP_NONE;
	WriteByteHDMI(hdmi,HDMI_TPI_PIXEL_REPETITION_REG, val);
#if 0
	val = ReadByteHDMI(hdmi,HDMI_TPI_PIXEL_REPETITION_REG);
	printk(KERN_ALERT"[HDMI:wangzhi]HDMI_TPI_PIXEL_REPETITION_REG:0x%x\n",val);
#endif        
	// Write out the TPI AVI Input Format
	val = TPI_AVI_INPUT_BITMODE_8BIT | TPI_AVI_INPUT_RANGE_AUTO | TPI_AVI_INPUT_COLORSPACE_RGB;
	WriteByteHDMI(hdmi,HDMI_TPI_AVI_IN_FORMAT_REG, val);

	// Write out the TPI AVI Output Format
	val = TPI_AVI_OUTPUT_CONV_BT709 | TPI_AVI_OUTPUT_RANGE_AUTO | TPI_AVI_OUTPUT_COLORSPACE_RGBHDMI;
	WriteByteHDMI(hdmi,HDMI_TPI_AVI_OUT_FORMAT_REG, val);
#if 1
	// Write out the TPI System Control Data to power down
	val = TPI_SYS_CTRL_POWER_DOWN;
	WriteByteHDMI(hdmi,HDMI_SYS_CTRL_DATA_REG, val);
#endif
	// Write out the TPI AVI InfoFrame Data (all defaults)
	// Compute CRC
	val = 0x82 + 0x02 + 13;
	for (i = 1; i < sizeof(avi_info_frame_data); i++) {
		val += avi_info_frame_data[i];
	}
	avi_info_frame_data[0] = 0x100 - val;

	for (i = 0; i < sizeof(avi_info_frame_data); i ++) {
		WriteByteHDMI(hdmi,HDMI_TPI_AVI_DBYTE_BASE_REG + i, avi_info_frame_data[i]);
	}
#endif
}

static void ns115_hdmi_avi_infoframe_setup(struct ns115_hdmi_data *hdmi)
{

}

static struct ns115_hdmi_data* ns115_hdmi_get_pt(void)
{
	return hdmi_pt;
}

static void ns115_hdmi_set_pt(struct ns115_hdmi_data* hdmi)
{
	hdmi_pt = hdmi;
}

void ns115_hdmi_i2s_audio(u32 sample_rate, u32 channels, u32 resolution)
{
	struct ns115_hdmi_data *hdmi = ns115_hdmi_get_pt();

	if ((hdmi != NULL) && (hdmi->hp_state == HDMI_HOTPLUG_CONNECTED))
	{
		printk(KERN_WARNING "[HDMI:wangzhi]sample_rate:%d.\r\n",sample_rate);
		printk(KERN_WARNING "[HDMI:wangzhi]channels:%d.\r\n",channels);    
		printk(KERN_WARNING "[HDMI:wangzhi]resolution:%d.\r\n",resolution);

		if ((hdmi->sample_rate != sample_rate) ||
				(hdmi->channels != channels) ||
				(hdmi->resolution != resolution))
		{
			hdmi->sample_rate = sample_rate;
			if ((channels >=2) &&
					(channels <=8))
			{
				hdmi->channels = channels;
			}
			hdmi->resolution = resolution;
			ns115_hdmi_configure(hdmi);
		}
	}
}

EXPORT_SYMBOL(ns115_hdmi_i2s_audio);

static void ns115_hdmi_audio_config(struct ns115_hdmi_data *hdmi)
{
	u8 val;
	u8 SS,SS_streamhead;
	u8 SF,SF_streamhead;

	switch (hdmi->resolution) {
		case 16:
			SS = BIT_AUDIO_16BIT;
			SS_streamhead = 0x02;
			break;
		case 20:
			SS = BIT_AUDIO_20BIT;
			SS_streamhead = 0x0A;
			break;
		case 24:
			SS = BIT_AUDIO_24BIT;
			SS_streamhead = 0x0B;
		default:
			SS = BIT_AUDIO_16BIT;
			SS_streamhead = 0x02;
			break;
	}

	switch (hdmi->sample_rate) {
		case 32000:
			SF = BIT_AUDIO_32KHZ;
			SF_streamhead = 0x03;
			break;
		case 44100:
			SF = BIT_AUDIO_44_1KHZ;
			SF_streamhead = 0x00;
			break;
		case 48000:
			SF = BIT_AUDIO_48KHZ;
			SF_streamhead = 0x02;
		case 88200:
			SF = BIT_AUDIO_88_2KHZ;
			SF_streamhead = 0x08;
			break;
		case 96000:
			SF = BIT_AUDIO_96KHZ;
			SF_streamhead = 0x0A;
			break;
		case 176400:
			SF = BIT_AUDIO_176_4KHZ;
			SF_streamhead = 0x0C;
		case 192000:
			SF = BIT_AUDIO_192KHZ;
			SF_streamhead = 0x0E;
		default:
			SF = BIT_AUDIO_48KHZ;
			SF_streamhead = 0x02;
			break;
	}

	/*
	// 1.
	Audio In Mode Register===I2S input channel #0: 
	SD0_EN
	AUD_EN
	*/
	WriteIndexedRegister(hdmi,INDEXED_PAGE_1,0x14,0x11);

	// 2.Select I2S interface and Mute Audio// Set to I2S, Mute
	val = AUD_IF_I2S | BIT_AUDIO_MUTE;
	WriteByteHDMI(hdmi,TPI_AUDIO_INTERFACE_REG, val);

	// 3.Select the general incoming SD format.
	//I2S - Input Configuration - replace with call to API ConfigI2SInput
	WriteByteHDMI(hdmi,TPI_I2S_IN_CFG, MCLK_MULTIPLIER | SCK_SAMPLE_RISING_EDGE);

	// 4.program each of the SD inputs.
	//I2S_AUDIO
	// I2S - Map channels - replace with call to API MAPI2S
	WriteByteHDMI(hdmi,TPI_I2S_EN, 0x80);//sd0

	// 5.Configure sample rate and sample size
	//TPI Audio Configuration Write Data
	val = (SS | SF);
	WriteByteHDMI(hdmi,TPI_AUDIO_SAMPLE_CTRL, val);

	// 6.Configure I2S Channel status
	// I2S - Stream Header Settings - replace with call to API SetI2S_StreamHeader
	//Stream Header Settings for I2S
	WriteByteHDMI(hdmi,TPI_I2S_CHST_0, 0x00); 
	WriteByteHDMI(hdmi,TPI_I2S_CHST_1, 0x00);
	WriteByteHDMI(hdmi,TPI_I2S_CHST_2, 0x00);

	WriteByteHDMI(hdmi,TPI_I2S_CHST_3, SF_streamhead);//0010 ¨C 48kHz
	WriteByteHDMI(hdmi,TPI_I2S_CHST_4, SS_streamhead);///0010 ¨C 16 bits

	val = ReadByteHDMI(hdmi,TPI_AUDIO_INTERFACE_REG);
	val &= ~BIT_4;//I2S, Layout 0, Unmute// Unmute I2S
	WriteByteHDMI(hdmi,TPI_AUDIO_INTERFACE_REG, val);

	val = ReadByteHDMI(hdmi,TPI_AUDIO_INTERFACE_REG);
}

bool ns115_hdmi_audio_infoframe_setup(struct ns115_hdmi_data *hdmi,u8 ChannelCount, u8 CodingType, 
		u8 SS, u8 Fs, u8 SpeakerConfig)
{
	u8 B_Data[SIZE_AUDIO_INFOFRAME] = {0};  // 15
	u8 i;

	for (i = 0; i < SIZE_AUDIO_INFOFRAME; i++)
		B_Data[i] = 0;

	WriteByteHDMI(hdmi,MISC_INFO_FRAMES_CTRL, DISABLE_AUDIO);         //  Disbale MPEG/Vendor Specific InfoFrames

	B_Data[0] = TYPE_AUDIO_INFOFRAMES;      // 0x84
	B_Data[1] = MISC_INFOFRAMES_VERSION;    // 0x01
	B_Data[2] = AUDIO_INFOFRAMES_LENGTH;    // 0x0A
	B_Data[3] = TYPE_AUDIO_INFOFRAMES+      // Calculate checksum - 0x84 + 0x01 + 0x0A
		MISC_INFOFRAMES_VERSION+ 
		AUDIO_INFOFRAMES_LENGTH;
	/*****************PB1*****************/
	//2_CHANNELS
	B_Data[4] = ChannelCount;               // 0 for "Refer to Stream Header" or for 2 Channels. 0x07 for 8 Channels
	//0x00
	B_Data[4] |= (CodingType << 4);         // 0xC7[7:4] == 0b1001 for DSD Audio
	/*****************PB2*****************/
	//0x00 0x00
	B_Data[5] = (Fs >> 1) | (SS >> 6);
	/*****************PB3*****************/
	//B_Data[6] = 0x00;
	/*****************PB4*****************/
	//0x00
	B_Data[7] = SpeakerConfig;
	/*****************PB5*****************/
	//B_Data[8] = 0x00;

	for (i = 4; i < SIZE_AUDIO_INFOFRAME; i++)
		B_Data[3] += B_Data[i];

	B_Data[3] = 0x100 - B_Data[3];
	//g_audio_Checksum = B_Data[3];   // Audio checksum for global use

	//Configure Audio Infoframe Header
	///0xBF
	WriteByteHDMI(hdmi,MISC_INFO_FRAMES_CTRL, EN_AND_RPT_AUDIO);//11000 010  // Re-enable Audio InfoFrame transmission and repeat
	///0xC0==0xBF + 1
	WriteBlockHDMI(hdmi,MISC_INFO_FRAMES_TYPE, SIZE_AUDIO_INFOFRAME, B_Data);
	//WriteByteHDMI(hdmi,TPI_AUDIO_INTERFACE_REG, 0x80);//I2S, Layout 0, Unmute
	return true;  
}

void WriteBlockHDMI(struct ns115_hdmi_data *hdmi,u8 TPI_Offset, unsigned short NBytes, u8 * pData)//WriteBlockTPI ()
{
	u8 i = 0;
	for (i=0;i<NBytes;i++)
	{
		WriteByteHDMI(hdmi,TPI_Offset+i, pData[i]);
	}
}

static bool ns115_hdmi_must_reconfigure(struct ns115_hdmi_data *hdmi)
{

	return true;
}

static void ns115_hdmi_TxPowerStateD0 (struct ns115_hdmi_data *hdmi)
{
	ReadModifyWriteTPI(hdmi,TPI_DEVICE_POWER_STATE_CTRL_REG, TX_POWER_STATE_MASK, TX_POWER_STATE_D0);
	printk(KERN_WARNING "[HDMI:wangzhi]TX Power State D0\n");
	hdmi->txPowerState = TX_POWER_STATE_D0;
}

static void ns115_hdmi_TxPowerStateD3 (struct ns115_hdmi_data *hdmi)
{
	ReadModifyWriteTPI(hdmi,TPI_DEVICE_POWER_STATE_CTRL_REG, TX_POWER_STATE_MASK, TX_POWER_STATE_D3);

	printk(KERN_WARNING "[HDMI:wangzhi]TX Power State D3\n");
	hdmi->txPowerState = TX_POWER_STATE_D3;
}

static void ns115_hdmi_EnableTMDS (struct ns115_hdmi_data *hdmi)
{
	printk(KERN_WARNING "[HDMI:wangzhi]TMDS -> Enabled\n");

	ReadModifyWriteTPI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG, 
			TMDS_OUTPUT_CONTROL_MASK, TMDS_OUTPUT_CONTROL_ACTIVE);
	hdmi->tmdsPoweredUp = true;
}

static void ns115_hdmi_DisableTMDS (struct ns115_hdmi_data *hdmi)
{
	printk(KERN_WARNING "[HDMI:wangzhi]TMDS -> Disabled\n");

	ReadModifyWriteTPI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG, TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK,
			TMDS_OUTPUT_CONTROL_POWER_DOWN | AV_MUTE_MUTED);
	hdmi->tmdsPoweredUp = false;
}

static void ns115_hdmi_configure(struct ns115_hdmi_data *hdmi)
{
	///start
	WriteByteHDMI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG, BIT_4 | BIT_0);// Disable TMDS output

	WriteByteHDMI(hdmi,TPI_DEVICE_POWER_STATE_CTRL_REG, TX_POWER_STATE_D0);// Enter full-operation D0 state

	/* Configure video format */
	ns115_hdmi_video_config(hdmi);
	/* Auxiliary Video Information (AVI) InfoFrame */
	//ns115_hdmi_avi_infoframe_setup(hdmi);

	/* Configure audio format */
	ns115_hdmi_audio_config(hdmi);

	/* Audio InfoFrame */
	ns115_hdmi_audio_infoframe_setup(hdmi,hdmi->channels - 1, 0x00, 0x00, 0x00, 0x00);

	///end
	WriteByteHDMI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG, BIT_0);// Enable TMDS output
}

static void ns115_hdmi_tmds_clock_disable(struct ns115_hdmi_data *hdmi)
{
	unsigned int temp = 0;

	//PHY disable clock
	temp = Readu16HDMI(hdmi,REG_TMDS_CONTROL_7);
	if (temp != 0)
	{
		temp = 0x00;
		Writeu16HDMI(hdmi,REG_TMDS_CONTROL_7, temp);
	}
	temp = Readu16HDMI(hdmi,REG_TMDS_CONTROL_9);
	if (temp != 0)
	{
		temp = 0x00;
		Writeu16HDMI(hdmi,REG_TMDS_CONTROL_9, temp);	
	}
}

static void ns115_hdmi_tmds_clock_enable(struct ns115_hdmi_data *hdmi)
{
	unsigned int temp = 0;

	//PHY enable clock
	temp = Readu16HDMI(hdmi,REG_TMDS_CONTROL_7);
	printk(KERN_WARNING "[HDMI:wangzhi]REG_TMDS_CONTROL_7:0x%x\n",temp);
	if (temp == 0)
	{
		temp  |= CEC_OSC_EN_BIT;
		Writeu16HDMI(hdmi,REG_TMDS_CONTROL_7, temp);
		hr_msleep(1);
	}
	temp = Readu16HDMI(hdmi,REG_TMDS_CONTROL_9);
	printk(KERN_WARNING "[HDMI:wangzhi]REG_TMDS_CONTROL_9:0x%x\n",temp);
	if (temp == 0)
	{
		temp  |=CEC_OSC_CTRL_BIT1_TO_BIT4;
		Writeu16HDMI(hdmi,REG_TMDS_CONTROL_9, temp);
		hr_msleep(1);
	}
}
#if defined(NS115_I2C_EDID)
static int ns115_hdmi_EDIDRead(struct ns115_hdmi_data *hdmi,
		unsigned int ui_num, u8* uc_RegBuf, u8 block_edid)
{
	int ddc_adapter=I2C_BUS_DDC;
	struct i2c_adapter* adapter;
	unsigned char * edid = NULL;
	int i = 0;
	int ret = 0;

	adapter = i2c_get_adapter(ddc_adapter);
	if(adapter) {
		edid=fb_ddc_read_i2c(adapter);
		if(edid){
			memcpy(uc_RegBuf,edid,2*EDID_LENGTH);
			kfree(edid);
			printk("EDID for lcd 1:\n");
			for(i = 0; i < 2*EDID_LENGTH; i++)
			{
				if ((i % 16) == 0) {
					printk("\n");
					printk("%02X | %02X", i, uc_RegBuf[i]);
				} else {
					printk(" %02X", uc_RegBuf[i]);
				}
			}
			ret = 1;
		}
		i2c_put_adapter(adapter);

	}else{
		ret = -1;
		printk(KERN_EMERG "begin get lcd1 adapter failed.\n");
	}
	return ret;
}
#else
static int ns115_hdmi_EDIDRead(struct ns115_hdmi_data *hdmi,
		unsigned int ui_num, u8* uc_RegBuf, u8 block_edid)
{
	unsigned char val;
	unsigned int i = 0;
	unsigned int j = 0; 
	unsigned int k = 0; 	
	unsigned int retries = 0;
	int ret = 0;
	u8 count_ddc_read;
	u8 temp = 0;

	count_ddc_read = ui_num / MAX_SIZE_DDC_FIFOCNT;

	//phy enable clock
	ns115_hdmi_tmds_clock_enable(hdmi);

	//into Page 0
#if 1
	///ref hdmi rtl code
	WriteByteHDMI(hdmi,TPI_INTERNAL_PAGE_REG, 0x80); 
#else
	///ref hdmi datasheet
	WriteIndexedRegister(hdmi,INDEXED_PAGE_0,0xC7,0x80);
#endif

	//mapping the DDC register to APB directly
	//1.load slave addr
	ReadSetWriteTPI(hdmi,DDC_ADDR, 0xA0);

	//2.load offset addr
	ReadSetWriteTPI(hdmi,DDC_OFFSET, block_edid);

	//3.load Byte count
	ReadSetWriteTPI(hdmi,DDC_CNT1, ui_num);///EDID_BLOCK_SIZE
	ReadSetWriteTPI(hdmi,DDC_CNT2, 0);

	//4.clear FIFO
	WriteByteHDMI(hdmi,DDC_CMD, CLEAR_FIFO_DDC_CMD);

	//config DDC clock
	///Clock SCL
	WriteByteHDMI(hdmi,DDC_CMD, CLOCK_SCL_DDC);

	//5.set DDC_CMD (Current Address Read with no ACK on last byte-0000 or 
	//Sequential Read with no ACK on last byte -0010)
	///Enhanced DDC Read with no ACK on last byte 
	WriteByteHDMI(hdmi,DDC_CMD, 0x04);///0b0100
	hr_msleep(1);

	for(i = 0; i < count_ddc_read;)//8
	{
		//6.
		val = ReadByteHDMI(hdmi,DDC_STATUS);

		if (val&FIFO_FULL)
		{
			i++;
			for(j = 0; j < MAX_SIZE_DDC_FIFOCNT; j++)
			{
				*uc_RegBuf++ = ReadByteHDMI(hdmi,DDC_DATA);
			}
		}
		else
		{
			//temp = ReadByteHDMI(hdmi,DDC_FIFO_CNT);
			//printk(KERN_WARNING "[HDMI:wangzhi]i:%d:DDC_FIFO_CNT:0x%x.\n",i,temp);
		}
		hr_msleep(5);
		k++;
		if (k > T_DDC_STATUS)
		{
			ret = -1;
			printk(KERN_WARNING "[HDMI:wangzhi]i:%d:DDC_STATUS:0x%x.\n",i,val);
			printk(KERN_WARNING "[HDMI:wangzhi]Read EDID timeout!\n");
			goto err_edid;
		}
	}

err_edid:

	val = 0;
	do {
		val = ReadByteHDMI(hdmi,DDC_STATUS);
		if (retries++ > T_DDC_STATUS)
		{
			ret = 1;
		}
		else
		{
			hr_msleep(1);
		}
	} while ((val & IN_PROG) == 1);	

	WriteByteHDMI(hdmi,TPI_ENABLE, 0x00);// Write "0" to 72:C7 to start HW TPI mode
	return ret;  
}

static bool GetDDC_Access (struct ns115_hdmi_data *hdmi,u8* SysCtrlRegVal)
{
	u8 sysCtrl;
	u8 DDCReqTimeout = T_DDC_ACCESS;
	u8 TPI_ControlImage;

	printk(KERN_WARNING "[HDMI:wangzhi]>>GetDDC_Access()\n");

	sysCtrl = ReadByteHDMI (hdmi,TPI_SYSTEM_CONTROL_DATA_REG);			// Read and store original value. Will be passed into ReleaseDDC()
	*SysCtrlRegVal = sysCtrl;

	sysCtrl |= DDC_BUS_REQUEST_REQUESTED;
	WriteByteHDMI (hdmi,TPI_SYSTEM_CONTROL_DATA_REG, sysCtrl);

	while (DDCReqTimeout--)											// Loop till 0x1A[1] reads "1"
	{
		TPI_ControlImage = ReadByteHDMI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG);

		if (TPI_ControlImage & DDC_BUS_GRANT_MASK)					// When 0x1A[1] reads "1"
		{
			sysCtrl |= DDC_BUS_GRANT_GRANTED;
			WriteByteHDMI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG, sysCtrl);		// lock host DDC bus access (0x1A[2:1] = 11)
			return true;
		}
		WriteByteHDMI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG, sysCtrl);			// 0x1A[2] = "1" - Requst the DDC bus
		hr_msleep(200);
	}

	sysCtrl &= ~DDC_BUS_REQUEST_REQUESTED;
	WriteByteHDMI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG, sysCtrl);				// Failure... restore original value.
	return false;
}

static bool ReleaseDDC(struct ns115_hdmi_data *hdmi,u8 SysCtrlRegVal)
{
	u8 DDCReqTimeout = T_DDC_ACCESS;
	u8 TPI_ControlImage;

	printk(KERN_WARNING "[HDMI:wangzhi]>>ReleaseDDC()\n");

	SysCtrlRegVal &= ~BITS_2_1;					// Just to be sure bits [2:1] are 0 before it is written

	while (DDCReqTimeout--)						// Loop till 0x1A[1] reads "0"
	{
		// Cannot use ReadClearWriteTPI() here. A read of TPI_SYSTEM_CONTROL is invalid while DDC is granted.
		// Doing so will return 0xFF, and cause an invalid value to be written back.
		WriteByteHDMI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG, SysCtrlRegVal);
		TPI_ControlImage = ReadByteHDMI(hdmi,TPI_SYSTEM_CONTROL_DATA_REG);
		printk(KERN_WARNING "[HDMI:wangzhi]ReleaseDDC:TPI_SYSTEM_CONTROL_DATA_REG:0x%x\r\n",TPI_ControlImage);
		if (!(TPI_ControlImage & BITS_2_1))		// When 0x1A[2:1] read "0"
			return true;
		//hr_msleep(5);
	}

	return false;								// Failed to release DDC bus control
}
#endif
static void ReadModifyWriteTPI(struct ns115_hdmi_data *hdmi,u8 Offset, u8 Mask, u8 Value)
{
	u8 Tmp;

	Tmp = ReadByteHDMI(hdmi,Offset);
	Tmp &= ~Mask;
	Tmp |= (Value & Mask);
	WriteByteHDMI(hdmi,Offset, Tmp);
}

static void ReadSetWriteTPI(struct ns115_hdmi_data *hdmi,u8 Offset, u8 Pattern)
{
	u8 Tmp = 0;

	Tmp = ReadByteHDMI(hdmi,Offset);
	Tmp |= Pattern;
	WriteByteHDMI(hdmi,Offset, Tmp);
}

static void ReadClearWriteTPI(struct ns115_hdmi_data *hdmi,u8 Offset, u8 Pattern)
{
	u8 Tmp;

	Tmp = ReadByteHDMI(hdmi,Offset);

	Tmp &= ~Pattern;
	WriteByteHDMI(hdmi,Offset, Tmp);
}

static void EnableInterrupts(struct ns115_hdmi_data *hdmi,u8 Interrupt_Pattern)
{
	WriteByteHDMI(hdmi,TPI_INTERRUPT_ENABLE_REG, Interrupt_Pattern);
	//ReadSetWriteTPI(hdmi,TPI_INTERRUPT_ENABLE_REG, Interrupt_Pattern);
}

static void  DisableInterrupts(struct ns115_hdmi_data *hdmi,unsigned char Interrupt_Pattern)
{
	ReadClearWriteTPI(hdmi,TPI_INTERRUPT_ENABLE_REG, Interrupt_Pattern);
}

static int __init ns115_hdmi_init(void)
{
	return platform_driver_probe(&ns115_hdmi_driver, ns115_hdmi_probe);
}

static void __exit ns115_hdmi_exit(void)
{
	platform_driver_unregister(&ns115_hdmi_driver);
}

module_init(ns115_hdmi_init);
module_exit(ns115_hdmi_exit);

MODULE_AUTHOR("wangzhi <zhi.wang@nufront.com>");
MODULE_DESCRIPTION("ns115 / soc HDMI driver");
MODULE_LICENSE("GPL v2");
