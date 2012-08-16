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
#include "../codecs/wm8960.h"
#include "nusmart-plat.h"
#include "nusmart-eva-i2s-dai.h"

//#define	 PRINT_DBG_INFO

#ifdef	PRINT_DBG_INFO 
	#define DBG_PRINT(fmt, args...) printk( KERN_INFO "nusmart-mach: " fmt, ## args) 
#else 
	#define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif 

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

static struct snd_soc_ops nusmart_machine_ops = {
	.startup = nusmart_machine_startup,
	.hw_params = nusmart_machine_hw_params,
};
/*
static const struct snd_soc_dapm_widget e800_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic (Internal1)", NULL),
	SND_SOC_DAPM_MIC("Mic (Internal2)", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_PGA_E("Headphone Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			e800_hp_amp_event, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("Speaker Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			e800_spk_amp_event, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "Headphone Amp"},

	{"Speaker Amp", NULL, "MONOOUT"},
	{"Speaker", NULL, "Speaker Amp"},

	{"MIC1", NULL, "Mic (Internal1)"},
	{"MIC2", NULL, "Mic (Internal2)"},
};

static int wm8960_mach_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_new_controls(dapm, e800_dapm_widgets,
					ARRAY_SIZE(e800_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_sync(dapm);

	return 0;

}
*/
/* nusmart_machine digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ns2816_dai = {
	.name = "wm8960",
	.stream_name = "WM8960",
#ifdef CONFIG_SND_NUSMART_SOC_I2S
	//.cpu_dai = &nusmart_i2s_dai,
	.cpu_dai_name = "ns2816-i2s",
#endif
#ifdef CONFIG_SND_NUSMART_SOC_EVA_I2S
        //.cpu_dai = &eva_i2s_dai,
        //.cpu_dai_name = "eva-i2s-dai",
	.cpu_dai_name = "ns2816-i2s-plat",
#endif
	//.codec_dai = &wm8960_dai,
	.codec_dai_name = "wm8960-hifi",
	.platform_name = "nusmart-pcm-audio",
	//.codec_name = "wm8960-codec",
	.codec_name = "wm8960-codec.2-001a",
	.ops = &nusmart_machine_ops,
//	.init	= wm8960_mach_init,
};


static struct snd_soc_card snd_soc_ns2816 = {
	.name = "ns2816-audio",
	//.platform = &nusmart_soc_platform,
	.dai_link = &ns2816_dai,
	.num_links = 1,
};
/* nusmart_machine audio subsystem */
/*
static struct snd_soc_device nusmart_snd_devdata = {
	.card = &snd_soc_ns2816,
	.codec_dev = &soc_codec_dev_wm8960,
};
*/

static struct platform_device *nusmart_snd_device;

static int __init nusmart_machine_init(void)
{
	int ret;

	DBG_PRINT("nusmart_machine_init \n");

	nusmart_snd_device = platform_device_alloc("soc-audio", -1);
	if (!nusmart_snd_device)
		return -ENOMEM;

	platform_set_drvdata(nusmart_snd_device, &snd_soc_ns2816);

	ret = platform_device_add(nusmart_snd_device);

	if (ret)
		platform_device_put(nusmart_snd_device);
	
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
