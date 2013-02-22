/*
 * nusmart_machine.c  --  SoC audio for nusmart-chip
 *
 *
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>

#include <mach/i2s.h>
#include "../codecs/rt5631.h"
#include "nusmart-plat.h"
#include "nusmart-eva-i2s-dai.h"

#ifdef	PRINT_DBG_INFO
#define DBG_PRINT(fmt, args...) printk( KERN_INFO "nusmart-mach: " fmt, ## args)
#else
#define DBG_PRINT(fmt, args...)	/* not debugging: nothing */
#endif

struct ns115_jd_pridata {
	int irq;
	int gpio;
	int report;
	struct switch_dev sdev;
	struct work_struct work;
	struct snd_soc_codec *codec;
};
struct ns115_jd_platform_data {
	unsigned int irq;
};
static struct ns115_jd_pridata ns115_jd_data;
struct ns115_jd_platform_data *ns115_jd_dev_data;
static struct workqueue_struct *ns115_jd_wq;

static int nusmart_machine_startup(struct snd_pcm_substream *substream)
{
	//struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_codec *codec = rtd->socdev->card->codec;
	//DBG_PRINT("nusmart_machine_startup ..\n");
	/* check the jack status at stream startup */
	return 0;
}

static int nusmart_machine_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	//struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	//struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	//unsigned int clk = 0;
	//int ret = 0;
	//DBG_PRINT("nusmart_machine_hw_params ..\n");
	return 0;
}

static const struct snd_soc_dapm_widget rt5631_dapm_widgets[] = {

	SND_SOC_DAPM_MIC("Mic Jack1", NULL),
	SND_SOC_DAPM_MIC("Mic Jack2", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", nusmart_rt5631_event_spk),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("AXI IN", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {

	/* Mic Jack --> MIC_IN */
	{"MIC Bias1", NULL, "Mic Jack1"},
	{"MIC1", NULL, "MIC Bias1"},

	{"MIC Bias2", NULL, "Mic Jack2"},
	{"MIC2", NULL, "MIC Bias2"},
	/* HP_OUT --> Headphone Jack */
	{"Headphone Jack", NULL, "HPOL"},
	{"Headphone Jack", NULL, "HPOR"},
	/* LINE_OUT --> Ext Speaker */
	{"Ext Spk", NULL, "SPOL"},
	{"Ext Spk", NULL, "SPOR"},

	{"AXIL", NULL, "AXI IN"},
	{"AXIR", NULL, "AXI IN"},
};

static const struct snd_kcontrol_new nusmart_controls[] = {
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack1"),
	SOC_DAPM_PIN_SWITCH("Mic Jack2"),
	SOC_DAPM_PIN_SWITCH("AXI IN"),
};

static int nusmart_rt5631_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_add_controls(codec, nusmart_controls,
			     ARRAY_SIZE(nusmart_controls));
	snd_soc_dapm_new_controls(dapm, rt5631_dapm_widgets,
				  ARRAY_SIZE(rt5631_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_enable_pin(dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	//snd_soc_dapm_enable_pin(dapm, "AXI IN");
	snd_soc_dapm_sync(dapm);

	ns115_jd_data.codec = codec;

	return 0;
}

static struct snd_soc_ops nusmart_machine_ops = {
	.startup = nusmart_machine_startup,
	.hw_params = nusmart_machine_hw_params,
};

/*
 * Logic for a alc5631 as connected on a Sharp SL-Cxx00 Device
 */
static int nusmart_machine_alc5631_init(struct snd_soc_codec *codec)
{
	DBG_PRINT("nusmart_machine_alc5631_init ..\n");
	return 0;
}

/* nusmart_machine digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ns115_dai = {
	.name = "rt5631",
	.stream_name = "RT5631",
#ifdef CONFIG_SND_NUSMART_SOC_I2S
	.cpu_dai_name = "ns115-i2s",
#endif
#ifdef CONFIG_SND_NUSMART_SOC_EVA_I2S
	.cpu_dai_name = "eva-i2s-dai",
#endif
	.codec_dai_name = "rt5631-hifi",
	.platform_name = "nusmart-pcm-audio",
	.codec_name = "rt5631.0-001a",
	.ops = &nusmart_machine_ops,
	.init = nusmart_rt5631_init,
};

static struct snd_soc_card snd_soc_ns115 = {
	.name = "ns115-audio",
	//.platform = &nusmart_soc_platform,
	.dai_link = &ns115_dai,
	.num_links = 1,
};

int ns115_jd_get_state(void)
{
	struct ns115_jd_pridata *jack = &ns115_jd_data;
	int state;

	state = gpio_get_value(jack->gpio);
	return (!!state);
}

EXPORT_SYMBOL_GPL(ns115_jd_get_state);

static irqreturn_t ns115_jd_irq_handler(int irq, void *data)
{
	struct ns115_jd_pridata *jack = &ns115_jd_data;

	disable_irq_nosync(irq);
	queue_work(ns115_jd_wq, &jack->work);

	return IRQ_HANDLED;
}

static void ns115_jd_work_func(struct work_struct *work)
{
	struct ns115_jd_pridata *jack =
	    container_of(work, struct ns115_jd_pridata, work);
	int irqret, state, state_tmp, deb_cnt = 0;
	unsigned long irqflags;

	//delay 100 ms and check the gpio 5 times to avoid wrong state
	msleep(100);
	state = gpio_get_value(jack->gpio);
	for (deb_cnt = 0; deb_cnt < 5;) {
		msleep(60);
		state_tmp = gpio_get_value(jack->gpio);
		if (state == state_tmp) {
			deb_cnt++;
		} else {
			state = state_tmp;
			deb_cnt = 0;
		}
	}
	irqflags = state ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	if (state != jack->report) {
		jack->report = state;
#ifdef CONFIG_MACH_NS115_PAD_PROTOTYPE
		switch_set_state(&jack->sdev, !state);
#else
		switch_set_state(&jack->sdev, !!state);
		if (ns115_jd_data.codec != NULL)
			nusmart_rt5631_jack_event(ns115_jd_data.codec, !!state);
#endif
	}
	free_irq(jack->irq, NULL);
	irqret = request_irq(jack->irq, ns115_jd_irq_handler, irqflags,
			     "ns115-jd", NULL);
}

#ifdef CONFIG_PM
static int ns115_jd_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int ns115_jd_resume(struct platform_device *pdev)
{
	int irqret, state;
	unsigned long irqflags;
	struct ns115_jd_pridata *jack = &ns115_jd_data;

	state = gpio_get_value(jack->gpio);
	irqflags = state ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	if (state != jack->report) {
		jack->report = state;
#ifdef CONFIG_MACH_NS115_PAD_PROTOTYPE
		switch_set_state(&jack->sdev, !state);
#else
		switch_set_state(&jack->sdev, !!state);
#endif
	}
	free_irq(jack->irq, NULL);
	irqret = request_irq(jack->irq, ns115_jd_irq_handler, irqflags,
			     "ns115-jd", NULL);
	return 0;
}

#else
#define ns115_jd_suspend	NULL
#define ns115_jd_resume		NULL
#endif

static int ns115_jd_probe(struct platform_device *pdev)
{
	int ret, irqret;
	unsigned long irqflags;
	struct ns115_jd_pridata *jack = &ns115_jd_data;

	DBG_PRINT("%s ....\n", __func__);

	ns115_jd_dev_data =
	    (struct ns115_jd_platform_data *)pdev->dev.platform_data;
	jack->irq = ns115_jd_dev_data->irq;
	jack->gpio = irq_to_gpio(ns115_jd_dev_data->irq);

	INIT_WORK(&jack->work, ns115_jd_work_func);

	ret = gpio_request(jack->gpio, "RT5631_JD");	//d15 - CODEC_PWR_EN
	if (ret < 0) {
		DBG_PRINT("RT5631_JD request failed \n");
		return ret;
	}
	gpio_direction_input(jack->gpio);
	jack->report = gpio_get_value(jack->gpio);

	irqflags = jack->report ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	DBG_PRINT("gpio = %d, irqflags=0x%x", jack->gpio, irqflags);
	/* switch-class based headset detection */
	jack->sdev.name = "h2w";
	ret = switch_dev_register(&jack->sdev);
	if (ret) {
		DBG_PRINT("error registering switch device %d\n", ret);
		return ret;
	}

#ifdef CONFIG_MACH_NS115_PAD_PROTOTYPE
	switch_set_state(&jack->sdev, !jack->report);
#else
	switch_set_state(&jack->sdev, !!jack->report);
	if (ns115_jd_data.codec != NULL)
		nusmart_rt5631_jack_event(ns115_jd_data.codec, !!jack->report);
#endif

	irqret = request_irq(jack->irq, ns115_jd_irq_handler, irqflags,
			     "ns115-jd", NULL);

	if (irqret) {
		DBG_PRINT("JD request irq failed\n");
	}

	return ret;
}

static int __devexit ns115_jd_remove(struct platform_device *pdev)
{
	struct ns115_jd_pridata *jack = &ns115_jd_data;

	DBG_PRINT("%s ....\n", __func__);

	free_irq(jack->irq, NULL);
	cancel_work_sync(&jack->work);
	return 0;
}

static struct platform_driver ns115_jd_driver = {
	.probe = ns115_jd_probe,
	.remove = ns115_jd_remove,
	.suspend = ns115_jd_suspend,
	.resume = ns115_jd_resume,
	.driver = {
		   .name = "ns115-jd",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *nusmart_snd_device;

static int __init nusmart_machine_init(void)
{
	int ret;

	nusmart_snd_device = platform_device_alloc("soc-audio", -1);
	if (!nusmart_snd_device) {
		return -ENOMEM;
	}
	platform_set_drvdata(nusmart_snd_device, &snd_soc_ns115);

	ret = platform_device_add(nusmart_snd_device);

	if (ret) {
		platform_device_put(nusmart_snd_device);
	}

	/*create a work queue for jd */
	ns115_jd_wq = create_workqueue("ns115_jd_wq");
	if (!ns115_jd_wq) {
		DBG_PRINT("creat ns115_jd_wq faiked\n");
	}

	ret = platform_driver_register(&ns115_jd_driver);
	if (ret) {
		return -ENOMEM;
	}

	return ret;
}

static void __exit nusmart_machine_exit(void)
{
	platform_device_unregister(nusmart_snd_device);
	if (ns115_jd_wq) {
		destroy_workqueue(ns115_jd_wq);
	}
	platform_driver_unregister(&ns115_jd_driver);
}

module_init(nusmart_machine_init);
module_exit(nusmart_machine_exit);

MODULE_AUTHOR("Nufront Software");
MODULE_DESCRIPTION("Nufront Software");
MODULE_LICENSE("GPL");
