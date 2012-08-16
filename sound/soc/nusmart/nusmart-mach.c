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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
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

#define	 PRINT_DBG_INFO

#ifdef	PRINT_DBG_INFO 
#define DBG_PRINT(fmt, args...) printk( KERN_INFO "nusmart-mach: " fmt, ## args)
#else
#define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif 
extern int nusmart_rt5631_event_spk(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event);

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

	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", nusmart_rt5631_event_spk),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("AXI IN", NULL),
};

static const struct snd_soc_dapm_route audio_map[]={

	/* Mic Jack --> MIC_IN*/
	{"MIC Bias1", NULL, "Mic Jack"},
	{"MIC1", NULL, "MIC Bias1"},
	/* HP_OUT --> Headphone Jack */
	{"Headphone Jack", NULL, "HPOL"},
	{"Headphone Jack", NULL, "HPOR"},
	/* LINE_OUT --> Ext Speaker */
	{"Ext Spk", NULL, "SPOL"},
	{"Ext Spk", NULL, "SPOR"},

	{"AXIL", NULL, "AXI IN"},
	{"AXIR", NULL, "AXI IN"},
} ;

static const struct snd_kcontrol_new nusmart_controls[] = {
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
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
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	//snd_soc_dapm_enable_pin(dapm, "AXI IN");
	snd_soc_dapm_sync(dapm);

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
	.cpu_dai_name = "ns115-i2s-plat",
#endif
	.codec_dai_name = "rt5631-hifi",
	.platform_name = "nusmart-pcm-audio",
	.codec_name = "rt5631.0-001a",
	.ops = &nusmart_machine_ops,
	.init = nusmart_rt5631_init,
};


/* nusmart_machine audio private data */
/*
   static struct rt5631_setup_data nusmart_alc5631_setup = {
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
	.dai_link = &ns115_dai,
	.num_links = 1,
};
/* nusmart_machine audio subsystem */
/*
   static struct snd_soc_device nusmart_snd_devdata = {
   .card = &snd_soc_ns115,
   .codec_dev = &soc_codec_dev_rt5631,
   .codec_data = &nusmart_alc5631_setup,
   };
   */
static struct platform_device *nusmart_snd_device;

static int __init nusmart_machine_init(void)
{
	int ret, err = 0;

	nusmart_snd_device = platform_device_alloc("soc-audio", -1);
	if (!nusmart_snd_device)
	{
		return -ENOMEM;
	}
	platform_set_drvdata(nusmart_snd_device, &snd_soc_ns115);

	ret = platform_device_add(nusmart_snd_device);

	if (ret)
	{
		platform_device_put(nusmart_snd_device);
	}
	return ret;
}

static void __exit nusmart_machine_exit(void)
{
	platform_device_unregister(nusmart_snd_device);
}

module_init(nusmart_machine_init);
module_exit(nusmart_machine_exit);

MODULE_AUTHOR("Nufront Software");
MODULE_DESCRIPTION("Nufront Software");
MODULE_LICENSE("GPL");
