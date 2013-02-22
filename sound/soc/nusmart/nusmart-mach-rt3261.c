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
#include "../codecs/rt3261.h"
#include "nusmart-plat.h"
#include "nusmart-eva-i2s-dai.h"

#define	 PRINT_DBG_INFO

#ifdef	PRINT_DBG_INFO
#define DBG_PRINT(fmt, args...) printk( KERN_INFO "nusmart-mach-rt3261: " fmt, ## args)
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
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	return 0;
}

static const struct snd_soc_dapm_widget rt3261_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("HP Mic Jack", NULL),
	SND_SOC_DAPM_MIC("FM", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SPK("Ext Earpiece", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {

	/* Mic Jack --> MIC_IN */
	{"IN1P", NULL, "Mic Jack"},
	{"IN1N", NULL, "Mic Jack"},
	{"IN2P", NULL, "HP Mic Jack"},
	{"IN2N", NULL, "FM"},
	/* HP_OUT --> Headphone Jack */
	{"Headphone Jack", NULL, "HPOL"},
	{"Headphone Jack", NULL, "HPOR"},
	/* LINE_OUT --> Ext Speaker */
	{"Ext Spk", NULL, "SPOLP"},
	{"Ext Spk", NULL, "SPOLN"},
	{"Ext Earpiece", NULL, "SPORP"},
	{"Ext Earpiece", NULL, "SPORN"},

};

static const struct snd_kcontrol_new nusmart_controls[] = {
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
	SOC_DAPM_PIN_SWITCH("Ext Earpiece"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("HP Mic Jack"),
	SOC_DAPM_PIN_SWITCH("FM"),
};

static int nusmart_rt3261_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	snd_soc_add_controls(codec, nusmart_controls,
			     ARRAY_SIZE(nusmart_controls));
	snd_soc_dapm_new_controls(dapm, rt3261_dapm_widgets,
				  ARRAY_SIZE(rt3261_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_disable_pin(dapm, "Ext Spk");
	snd_soc_dapm_disable_pin(dapm, "Ext Earpiece");
	snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_disable_pin(dapm, "Mic Jack");
	snd_soc_dapm_disable_pin(dapm, "HP Mic Jack");
	snd_soc_dapm_disable_pin(dapm, "FM");
	snd_soc_dapm_sync(dapm);

	codec_dai->driver->ops->set_fmt(codec_dai,
					SND_SOC_DAIFMT_CBS_CFS |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_I2S);
	cpu_dai->driver->ops->set_fmt(cpu_dai,
				      SND_SOC_DAIFMT_CBM_CFM |
				      SND_SOC_DAIFMT_NB_NF |
				      SND_SOC_DAIFMT_I2S);

	/*if (!strcmp(codec_dai->name, "rt3261-aif1")) {
	   codec_dai->driver->ops->set_pll(codec_dai, 1, RT3261_PLL1_S_BCLK1, 48000*32*2, 12288000);
	   codec_dai->driver->ops->set_sysclk(codec_dai, RT3261_SCLK_S_PLL1, 12288000, 0);
	   } */
	codec_dai->driver->ops->set_pll(codec_dai, 1, RT3261_PLL1_S_MCLK,
					12000000, 24576000);
	codec_dai->driver->ops->set_sysclk(codec_dai, RT3261_SCLK_S_PLL1,
					   24576000, 0);

	ns115_jd_data.codec = codec;
	return 0;
}

static int nusmart_bp_stream_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	codec_dai->driver->ops->set_fmt(codec_dai,
					SND_SOC_DAIFMT_CBM_CFM |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_DSP_A |
					SND_SOC_DAIFMT_IB_NF);

	return 0;
}

static struct snd_soc_ops nusmart_machine_ops = {
	.startup = nusmart_machine_startup,
	.hw_params = nusmart_machine_hw_params,
};

/* nusmart_machine digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ns115_dai_links[] = {
	{
	 .name = "ns115-ap-dai-link",
	 .stream_name = "ns115-ap-stream",
#ifdef CONFIG_SND_NUSMART_SOC_I2S
	 .cpu_dai_name = "ns115-i2s",
#endif
#ifdef CONFIG_SND_NUSMART_SOC_EVA_I2S
	 .cpu_dai_name = "eva-i2s-dai",
#endif
	 .codec_dai_name = "rt3261-aif1",
	 .platform_name = "nusmart-pcm-audio",
	 .codec_name = "rt3261.0-001c",
	 .ops = &nusmart_machine_ops,
	 .init = nusmart_rt3261_init,
	 },
	{
	 .name = "ns115-bp-dai-link",
	 .stream_name = "ns115-bp-stream",
	 .platform_name = "fake-platform",
	 .cpu_dai_name = "MODEM",
	 //.no_pcm = 1,
	 //  .no_codec = 1,
	 .ignore_suspend = 1,
	 .no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
	 .codec_dai_name = "rt3261-aif2",
	 .codec_name = "rt3261.0-001c",
	 .ops = &nusmart_machine_ops,
	 .init = nusmart_bp_stream_init,
	 },
};

/* nusmart_machine audio private data */
/*
   static struct rt3261_setup_data nusmart_alc3261_setup = {
#ifdef CONFIG_MACH_NS2816_DEV_BOARD
.i2c_bus = 2,
#else
.i2c_bus = 0,
#endif
.i2c_address = 0x1a,
};
*/

static struct snd_soc_card snd_soc_ns115 = {
	.name = "ns115-audio",
	//.platform = &nusmart_soc_platform,
	.dai_link = ns115_dai_links,
	.num_links = ARRAY_SIZE(ns115_dai_links),
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
		switch_set_state(&jack->sdev, !!state);
		if (ns115_jd_data.codec != NULL)
			nusmart_rt3261_jack_event(ns115_jd_data.codec, !!state);
	}
	free_irq(jack->irq, NULL);
	irqret =
	    request_irq(jack->irq, ns115_jd_irq_handler, irqflags, "ns115-jd",
			NULL);
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
		switch_set_state(&jack->sdev, !!state);
	}
	free_irq(jack->irq, NULL);
	irqret =
	    request_irq(jack->irq, ns115_jd_irq_handler, irqflags, "ns115-jd",
			NULL);
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

	ret = gpio_request(jack->gpio, "RT3261_JD");	//d15 - CODEC_PWR_EN
	if (ret < 0) {
		DBG_PRINT("RT3261_JD request failed \n");
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

	switch_set_state(&jack->sdev, !!jack->report);
	if (ns115_jd_data.codec != NULL)
		nusmart_rt3261_jack_event(ns115_jd_data.codec, !!jack->report);

	irqret =
	    request_irq(jack->irq, ns115_jd_irq_handler, irqflags, "ns115-jd",
			NULL);

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

	ret = gpio_request(63, "rt3261_ldo");
	gpio_direction_output(63, 1);
	gpio_free(63);

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
