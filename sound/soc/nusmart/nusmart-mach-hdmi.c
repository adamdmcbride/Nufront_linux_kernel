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
#include "../codecs/hdmicodec.h"
#include "../codecs/wm8960.h"
#include "nusmart-plat.h"
#include "nusmart-eva-i2s-dai.h"





static int nusmart_machine_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static int nusmart_machine_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;
}

static struct snd_soc_ops nusmart_machine_ops = {
	.startup = nusmart_machine_startup,
	.hw_params = nusmart_machine_hw_params,
};

/* nusmart_machine digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ns2816_dai = {
	.name = "hdmi_ch7033",
	.stream_name = "hdmi_ch7033",
#ifdef CONFIG_SND_NUSMART_SOC_I2S
	//.cpu_dai = &nusmart_i2s_dai,
	.cpu_dai_name = "ns2816-i2s",
#endif
#ifdef CONFIG_SND_NUSMART_SOC_EVA_I2S
        //.cpu_dai = &eva_i2s_dai,
        .cpu_dai = "ns2816-i2s-plat",
#endif
	.codec_dai = &hdmicodec_dai,
	.ops = &nusmart_machine_ops,
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
	.codec_dev = &soc_codec_dev_ch7033,
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

	platform_set_drvdata(nusmart_snd_device, &nusmart_snd_devdata);
	nusmart_snd_devdata.dev = &nusmart_snd_device->dev;

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
