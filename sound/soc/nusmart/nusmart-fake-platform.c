/*
 * soc-util.c  --  ALSA SoC Audio Layer utility functions
 *
 * Copyright 2009 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *         Liam Girdwood <lrg@slimlogic.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

static int fake_pcm_open(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "---------------> fake_pcm_open\n");
	return 0;
}

static int fake_pcm_close(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "---------------> fake_pcm_close\n");
	return 0;
}

static struct snd_pcm_ops fake_pcm_ops = {
	.open           = fake_pcm_open,
	//	.hw_params	= aess_hw_params,
	//	.prepare	= aess_prepare,
	.close	        = fake_pcm_close,
	//	.pointer	= aess_pointer,
	//	.mmap		= aess_mmap,
};

static unsigned int fake_pcm_read(struct snd_soc_platform *platform,
		unsigned int reg)
{
	return 0;
}

static int fake_pcm_write(struct snd_soc_platform *platform, unsigned int reg,
		unsigned int val)
{
	return 0;
}

static int fake_pcm_probe(struct snd_soc_platform *platform)
{
	return 0;
}

static int fake_pcm_remove(struct snd_soc_platform *platform)
{
	return 0;
}


static struct snd_soc_platform_driver fake_platform = {
	.ops		= &fake_pcm_ops,
	.probe		= fake_pcm_probe,
	.remove		= fake_pcm_remove,
	.read		= fake_pcm_read,
	.write		= fake_pcm_write,
	//	.stream_event = aess_stream_event,
};

static __devinit int fake_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &fake_platform);
}

static __devexit int fake_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);

	return 0;
}

static struct platform_driver fake_platform_driver = {
	.driver = {
		.name = "fake-platform",
		.owner = THIS_MODULE,
	},
	.probe = fake_platform_probe,
	.remove = __devexit_p(fake_platform_remove),
};

static struct platform_device *fake_platform_dev;

int __init fake_platform_init(void)
{
	int ret;

	fake_platform_dev = platform_device_alloc("fake-platform", -1);
	if (!fake_platform_dev)
		return -ENOMEM;

	ret = platform_device_add(fake_platform_dev);
	if (ret != 0) {
		platform_device_put(fake_platform_dev);
		return ret;
	}

	ret = platform_driver_register(&fake_platform_driver);
	if (ret != 0)
		platform_device_unregister(fake_platform_dev);

	printk(KERN_INFO "--------------> fake platform init\n");

	return ret;
}

void __exit fake_platform_exit(void)
{
	platform_device_unregister(fake_platform_dev);
	platform_driver_unregister(&fake_platform_driver);
}

module_init(fake_platform_init);
module_exit(fake_platform_exit);

MODULE_DESCRIPTION("a fake platform");
MODULE_AUTHOR("rthuang <rentong.huang@nufront.com>");
MODULE_LICENSE("GPL");
