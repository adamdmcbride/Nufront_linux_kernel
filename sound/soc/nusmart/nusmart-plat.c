/*
 * nusmart-i2s.c -- alsa soc audio layer
 *
 * Copyright (C) 2010 Nufront Software, Inc
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <linux/interrupt.h>
#include <mach/hardware.h>

#include <linux/dma-mapping.h>
#include <sound/pcm_params.h>
#include <asm/dma.h>

#define	 PRINT_DBG_INFO

#ifdef	PRINT_DBG_INFO 
	#define DBG_PRINT(fmt, args...) printk( KERN_INFO "nusmart i2s: " fmt, ## args) 
#else 
	#define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif 

/*
 * I2S Controller Register and Bit Definitions
 */
#define	IER	(0x00)  //Enable dw_apb_i2s	
#define	IRER	(0x04)  //Enable i2s receiver	
#define	ITER	(0x08)  //Enable i2s transmitter 	
#define	CER	(0x0c)  //Clock enbale register
#define	CCR	(0x10)  //Clock configruaion register
#define	RXFFR	(0x14)  //Receive block all fifo reset
#define	TXFFR	(0x18)  //Transmitter block all fifo reset

#define	LRBR0	(0x20) //Left recevier buffer register 0
#define	LTHR0	(0x20) //Left transmit holding register 0
#define	RRBR0	(0x24) //Right receiver buffer register 0
#define	RTHR0	(0x24) //Right transmit holding register 0
#define	RER0	(0x28)	//Receive channel enbale register 0
#define	TER0	(0x2c)  //Trasmitter channel enable register 0
#define	RCR0	(0x30)  //Receiver configuraion registe 0
#define	TCR0	(0x34)  //Transmitter configuraion registe 0
#define	ISR0	(0x38) //Interupt status register 0
#define	ISR1	(0x78) //Interupt status register 1
#define	ISR2	(0xb8) //Interupt status register 2
#define	ISR3	(0xf8) //Interupt status register 3
#define	IMR0	(0x3c) //Interupt mask register 0
#define	ROR0	(0x40) //Clear receive overrun interrupt 0
#define	TOR0	(0x44) //Clear transmit overrun interrupt 0
#define	RFCR0	(0x48) //Receive fifo trigger confige 0	
#define	TFCR0	(0x4c) //Transmit fifo trigger confige 0	
#define	RFF0	(0x50) //Receive channel fifo flush 0
#define	TFF0	(0x54) //Transmit channel fifo flush 0

#define	TXFOM	(1 << 5) //Mask tx overrun interrupt
#define	TXFEM	(1 << 4) //Mask tx empty interrupt
#define	RXFOM	(1 << 1) //Mask rx overrun interrupt
#define	RXFFM	(1 << 0) //Mask rx triggler level interrupt
#define	TXFO 	(1 << 5) //Tx overrun interrupt
#define	TXFE 	(1 << 4) //Tx empty interrupt
#define	RXFO 	(1 << 1) //Rx overrun interrupt
#define	RXFF 	(1 << 0) //Rx triggler level interrupt
#define	WLEN_12	 (1)
#define	WLEN_16	 (2)
#define	WLEN_20	 (3)
#define	WLEN_24	 (4)
#define	WLEN_32  (5)
#define WSS_16	 (0 << 3)
#define WSS_24	 (1 << 3)
#define WSS_32	 (2 << 3)
#define	SCLK_G_12 (1 << 0)
#define	SCLK_G_16 (2 << 0)
#define	SCLK_G_24 (3 << 0)
#define	SCLK_G_32 (4 << 0)

#define	I2S_TXCHET	2
#define	I2S_RXCHFT	6

#define	NS2816_DEMO

#ifdef	NS2816_DEMO
#include <mach/irqs.h>
	
#define	SCM_DEMO_FLAG		(0x48)
#define	SCM_DEMO_DMA_ADDR	(0x4c)
#define	SCM_DEMO_DMA_SIZE	(0x50)
#define	SCM_DEMO_TRANS_SIZE	(0x54)
#define TIMERX_EOI		(0x0c)  //clr TIMERX active interrupt
#endif

struct nusmart_i2s_data {
	int 			irq;  	
	struct resource 	*mem_res;
	void __iomem		*base;
	struct snd_soc_device   *socdev;
	struct clk		*clk_i2s;
};

struct nusmart_i2s_data * nusmart_i2s = {0, NULL, NULL,NULL,NULL};	

struct nusmart_runtime_data {
	unsigned int		rcv_bytes_count;
	unsigned int		tran_bytes_count;
	unsigned int		fmt;
	unsigned int		rate;
	unsigned int		interrupt_count;
};
	
#ifdef	PRINT_DBG_INFO

void print_i2s_regs(void * context)
{
	//struct nusmart_i2s_data * i2s = (struct nusmart_i2s_data *)context;
	struct nusmart_i2s_data * i2s = nusmart_i2s;
	int i = 0;

	printk("IER (%02X) = %x\n", IER, (unsigned int)readl(i2s->base + IER));
	printk("ITER(%02X) = %x\n", ITER, (unsigned int)readl(i2s->base + ITER));
	printk("CER (%02X) = %x\n", CER, (unsigned int)readl(i2s->base + CER));
	printk("CCR (%02X) = %x\n", CCR, (unsigned int)readl(i2s->base + CCR));
	printk("TER0(%02X) = %x\n", TER0, (unsigned int)readl(i2s->base + TER0));
	printk("TCR0(%02X) = %x\n", TCR0, (unsigned int)readl(i2s->base + TCR0));
	printk("IMR0(%02X) = %x\n", IMR0, (unsigned int)readl(i2s->base + IMR0));
	printk("TFCR0(%02X) = %x\n",TFCR0, (unsigned int)readl(i2s->base + TFCR0));

	/*	
	printk("IRER = %x\n", (unsigned int)readl(i2s->base + IRER));
	printk("RER0 = %x\n", (unsigned int)readl(i2s->base + RER0));
	printk("RCR0 = %x\n", (unsigned int)readl(i2s->base + RCR0));
	printk("RFCR0 = %x\n", (unsigned int)readl(i2s->base + RFCR0));
	*/
	for(i = 0; i < 0x68; i+=4)
		printk("reg%02x: %08X\n", i, (unsigned int)readl(i2s->base + i));
}
#else
void	print_i2s_regs(void * context) {}
#endif //PRINT_DBG_INFO

static irqreturn_t nusmart_i2s_interrupt(int irq, void * context);
static const struct snd_pcm_hardware nusmart_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 24, //16,
	.period_bytes_max	= 200,//5*1024, //4096,
	.periods_min		= 24, //16,
	.periods_max		= 199,//5119,//255,
	.buffer_bytes_max	= 4800,//120 * 1024, //64 * 1024,
	.fifo_size		= 24, //16,
};

int nusmart_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data * rtd;

	struct timespec		tp;
	rtd = kzalloc(sizeof(*rtd), GFP_KERNEL);
	if (!rtd)
		return -ENOMEM;

	runtime->private_data = rtd;

	runtime->hw = nusmart_pcm_hardware;

	tp = current_kernel_time();	
	
	DBG_PRINT("nusmart_pcm_open, sec = 0x%x nsec = 0x%x\n",
		 (unsigned int)tp.tv_sec, 
		 (unsigned int)tp.tv_nsec);
	return 0;
}

int nusmart_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *rtd = runtime->private_data;

	struct timespec		tp;

	tp = current_kernel_time();	
	
	DBG_PRINT("nusmart_pcm_close, sec = 0x%x nsec = 0x%x, interrupt_count = 0x%x\n", 
			(unsigned int)tp.tv_sec, 
			(unsigned int)tp.tv_nsec, 
			rtd->interrupt_count);
	DBG_PRINT("nusmart_pcm_close, stream = %d\n", substream->stream);
	kfree(rtd);
	return 0;
}

static int nusmart_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *rtd_prv = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);
	
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	rtd_prv->rcv_bytes_count = 0;
	rtd_prv->tran_bytes_count = 0;
	rtd_prv->rate = params_rate(params);
	rtd_prv->fmt = params_format(params);

	DBG_PRINT("buffer_bytes  = 0x%x, perd_bytes = 0x%x, perd_size = %d, ch = %d, fmt = %d, rate = %d \n", 
		params_buffer_bytes(params),
		params_period_bytes(params),
		params_period_size(params),
		params_channels(params),
		params_format(params),
		params_rate(params) 
		);
	return 0;
}

static int nusmart_pcm_hw_free(struct snd_pcm_substream *substream)
{
	DBG_PRINT("nusmart_pcm_hw_free, stream = %d\n", substream->stream);
	return 0;
}

int nusmart_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	DBG_PRINT("nusmart_pcm_prepare, stream = %d,runtime->dma_area[0]: %02X\n", 
		substream->stream, (unsigned char)runtime->dma_area[0]);
	DBG_PRINT("nusmart_pcm_prepare, rate = %d, fmt = %d, ch = %d, bufsize = 0x%x, period_size = 0x%x\n", 
		runtime->rate,
		runtime->format,
		runtime->channels,
		(unsigned int)runtime->buffer_size,
		(unsigned int)runtime->period_size
		);
	return 0;
}

int nusmart_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

	DBG_PRINT("nusmart_pcm_trigger, cmd = %d, stream = %d\n",
		cmd, substream->stream );

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

snd_pcm_uframes_t nusmart_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *rtd_prv = runtime->private_data;

	snd_pcm_uframes_t x = 0;

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		x= bytes_to_frames(runtime, rtd_prv->tran_bytes_count);
	else
		x= bytes_to_frames(runtime, rtd_prv->rcv_bytes_count);
	
	if(x > runtime->buffer_size)
		DBG_PRINT("pcm_pointer, x = 0x%lx\n", x);
	

	if (x == runtime->buffer_size) {   //buffersize = 0x2dd5
		x = 0;
	}
	return x;
}

int nusmart_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	
	DBG_PRINT("nusmart_pcm_mmap, area = %x, phy_addr = %x, bytes = %d\n",
		(unsigned int)runtime->dma_area,
		(unsigned int)runtime->dma_addr,
		runtime->dma_bytes
		);
	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

struct snd_pcm_ops nusmart_pcm_ops = {
	.open		= nusmart_pcm_open,
	.close		= nusmart_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= nusmart_pcm_hw_params,
	.hw_free	= nusmart_pcm_hw_free,
	.prepare	= nusmart_pcm_prepare,
	.trigger	= nusmart_pcm_trigger,
	.pointer	= nusmart_pcm_pointer,
	.mmap		= nusmart_pcm_mmap,
};

static u64 nusmart_pcm_dmamask = DMA_BIT_MASK(32); //DMA_32BIT_MASK;

int nusmart_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = nusmart_pcm_hardware.buffer_bytes_max;
	

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;

	DBG_PRINT("pcm_allocate_dma_buffer, dma_phys = 0x%x, size = 0x%x, stream = %x\n",buf->addr,	size, stream);

	return 0;
}

void nusmart_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		DBG_PRINT("pcm_free_dma_buffer, size = %x, stream = %x\n",
			buf->bytes, stream);

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

//static int nusmart_soc_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
//	struct snd_pcm *pcm)
static int nusmart_soc_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	
	struct snd_card *card = rtd->card->snd_card;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	DBG_PRINT("nusmart_soc_pcm_new\n");

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &nusmart_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32); //DMA_32BIT_MASK;

	if (dai->driver->playback.channels_min) {
		ret = nusmart_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->driver->capture.channels_min) {
		ret = nusmart_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_platform_driver nusmart_soc_platform = {
//	.name		= "nusmart-audio",
	.ops 		= &nusmart_pcm_ops,
	.pcm_new	= nusmart_soc_pcm_new,
	.pcm_free	= nusmart_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(nusmart_soc_platform);

static int __init nusmart_soc_platform_init(struct platform_device *pdev)
{
	
	DBG_PRINT("nusmart_soc_platform_init\n");
	return snd_soc_register_platform(&pdev->dev, &nusmart_soc_platform);
}
//module_init(nusmart_soc_platform_init);

static void __exit nusmart_soc_platform_exit(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
}
//module_exit(nusmart_soc_platform_exit);

static struct platform_driver nusmart_pcm_driver = {
	.driver = {
			.name = "nusmart-audio",
			.owner = THIS_MODULE,
	},

	.probe = nusmart_soc_platform_init,
	.remove = __devexit_p(nusmart_soc_platform_exit),
};

static int __init snd_nusmart_pcm_init(void)
{
	return platform_driver_register(&nusmart_pcm_driver);
}
module_init(snd_nusmart_pcm_init);

static void __exit snd_nusmart_pcm_exit(void)
{
	platform_driver_unregister(&nusmart_pcm_driver);
}
module_exit(snd_nusmart_pcm_exit);

static int nusmart_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	DBG_PRINT("nusmart_i2s_set_dai_fmt, fmt = %d\n", fmt);
	return (fmt == SND_SOC_DAIFMT_I2S) ? 0 : -EINVAL;
}

static int nusmart_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{
	// dir == SND_SOC_CLOCK_OUT
	// dir == SND_SOC_CLOCK_IN

	DBG_PRINT("set_dai_sysclk, cpu_dai->active = %d\n", cpu_dai->active);

	DBG_PRINT("set_dai_sysclk, freq = %d, dir = %d\n", freq, dir);
	return 0;
}

static int nusmart_i2s_startup(struct snd_pcm_substream *substream, 
				struct snd_soc_dai * cpu_dai )
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct nusmart_i2s_data * i2s = rtd->dai->cpu_dai->private_data;
	struct nusmart_i2s_data * i2s = nusmart_i2s;
	unsigned int reg_offset = substream->number * 0x40;
	int ret = -ENOMEM;

	DBG_PRINT("nusmart_i2s_startup, i2s_ch = %d, stream = %d, active = %d\n",
		substream->number, substream->stream, cpu_dai->active);

	clk_enable(i2s->clk_i2s);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (substream->number) {	
		case 0:
			ret = request_irq(i2s->irq, nusmart_i2s_interrupt, 
				IRQF_SHARED, "nusmart_i2s_ch0_tx", substream);
			break;
		case 1:
			ret = request_irq(i2s->irq, nusmart_i2s_interrupt, 
				IRQF_SHARED, "nusmart_i2s_ch1_tx", substream);
			break;
		case 2:
			ret = request_irq(i2s->irq, nusmart_i2s_interrupt, 
				IRQF_SHARED, "nusmart_i2s_ch2_tx", substream);
			break;
		case 3:
			ret = request_irq(i2s->irq, nusmart_i2s_interrupt, 
				IRQF_SHARED,"nusmart_i2s_ch3_tx", substream);
			break;
		default:
			ret = -EINVAL;
		break;
		}
	}
	else {
		if(substream->number == 0)
			ret = request_irq(i2s->irq, nusmart_i2s_interrupt, 
				IRQF_SHARED, "nusmart_i2s_ch0_rx", substream);
		else
			ret = -EINVAL;
	}
	if (ret < 0) {
		goto out_irq;
	}

	DBG_PRINT("request_irq success: irq = %d, i2s_ch = %d, stream = %d\n", 
				i2s->irq, substream->number, substream->stream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		writel(1, i2s->base + TFF0 + reg_offset);
		writel(I2S_TXCHET, i2s->base + TFCR0 + reg_offset);
		writel(1, i2s->base + TER0 + reg_offset);
		if(readl(i2s->base + ITER) == 0) {
			writel(1, i2s->base + ITER);
			DBG_PRINT("enable i2s and clock\n");
		}
	} else {
		writel(1, i2s->base + RFF0 + reg_offset);
		writel(I2S_RXCHFT, i2s->base + RFCR0 + reg_offset);
		writel(1, i2s->base + RER0 + reg_offset);
		if(readl(i2s->base + IRER) == 0) {
			writel(1, i2s->base + IRER);
			DBG_PRINT("enable i2s and clock\n");
		}		
	}

out_irq:
	return ret;
}

static int set_i2s_sysclock(struct nusmart_i2s_data * i2s, int rate, int fmt, int ch)
{
	unsigned int sample_bits = 16, clk_hz;
	int ret = 0;

	switch(fmt) {
	case SNDRV_PCM_FORMAT_S8:
		sample_bits = 8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_bits = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sample_bits = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sample_bits = 32;
		break;
	}

	clk_hz = rate * ch * sample_bits;

#ifdef NS2816_DEMO
	if(sample_bits == 16)
		clk_hz *= 2;
#endif
	clk_set_rate(i2s->clk_i2s, clk_hz);

	DBG_PRINT("set_i2s_sysclock = %d\n", clk_hz);
	return ret;
}

static int nusmart_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai * cpu_dai )
{
	//struct nusmart_i2s_data * i2s = cpu_dai->private_data;
	struct nusmart_i2s_data * i2s = nusmart_i2s;
	unsigned int reg_offset = substream->number * 0x40;
	unsigned int wlen, wss;

	DBG_PRINT("nusmart_i2s_hw_params, i2s_ch = %d\n", substream->number);
	DBG_PRINT("nusmart_i2s_hw_params, rate = %d\n", params_rate(params));
	DBG_PRINT("nusmart_i2s_hw_params, fmt = %d\n",params_format(params));

	set_i2s_sysclock(i2s, params_rate(params), params_format(params), 
			params_channels(params));
	
	wss = WSS_16;
	wlen = WLEN_16;
	
	switch(params_format(params)) {

	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_S16_LE:
		wss = WSS_16;
		wlen = WLEN_16;
		break;	
	case SNDRV_PCM_FORMAT_S24_LE:
		wss = WSS_24;
		wlen = WLEN_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		wss = WSS_32;
		wlen = WLEN_32;
		break;
	}
#ifdef NS2816_DEMO
	wss = WSS_32;
	wlen = WLEN_32;
#endif
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		writel(wlen, i2s->base + TCR0 + reg_offset);
	} else {
		writel(wlen, i2s->base + RCR0 + reg_offset);
	}
	
	writel(wss, i2s->base + CCR + reg_offset);

	return 0;
}

static int nusmart_i2s_trigger(struct snd_pcm_substream *substream, int cmd, 
				struct snd_soc_dai * cpu_dai) 
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct nusmart_i2s_data * i2s = rtd->dai->cpu_dai->private_data;
	struct nusmart_i2s_data * i2s = nusmart_i2s;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *rtd_prv = runtime->private_data;
	unsigned int reg_value, reg_offset = substream->number * 0x40;
#ifdef	NS2816_DEMO
	void __iomem *scm_base = __io_address(0x051c0000);
#endif

	DBG_PRINT("nusmart_i2s_trigger, i2s_ch = %d, cmd = %d, stream = %x\n", 
			substream->number,
			cmd, 
			(unsigned int)substream);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		rtd_prv->rcv_bytes_count = 0;
		rtd_prv->tran_bytes_count = 0;
		rtd_prv->interrupt_count = 0;
#ifdef	NS2816_DEMO
		writel(runtime->dma_addr, scm_base + SCM_DEMO_DMA_ADDR);
		writel(runtime->dma_bytes, scm_base + SCM_DEMO_DMA_SIZE);
		writel(0, scm_base + SCM_DEMO_TRANS_SIZE);
		wmb();
		rmb();
		DBG_PRINT("nusmart_i2s_trigger, dma_phys = 0x%x, dma_bytes = 0x%x\n", 
			readl(scm_base + SCM_DEMO_DMA_ADDR), 
			readl(scm_base + SCM_DEMO_DMA_SIZE));
		writel(1, scm_base + SCM_DEMO_FLAG);	//trigger flag		
#endif
		DBG_PRINT("PCM_TRIGGER_START, active = %d\n", cpu_dai->active);
		writel(1, i2s->base + IER);
		writel(1, i2s->base + CER);
		disable_irq(i2s->irq);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			reg_value = readl(i2s->base + IMR0 + reg_offset);
			reg_value &= ~(TXFOM | TXFEM);
			writel(reg_value, i2s->base + IMR0 + reg_offset);
		}
		else {
			reg_value = readl(i2s->base + IMR0 + reg_offset);
			reg_value &= ~(RXFOM | RXFFM);
			writel(reg_value, i2s->base + IMR0 + reg_offset);
		}

		print_i2s_regs(i2s);
		DBG_PRINT("%02x %02x %02x %02x %02x %02x %02x %02x\n",
			runtime->dma_area[0],
			runtime->dma_area[1],
			runtime->dma_area[2],
			runtime->dma_area[3],
			runtime->dma_area[4],
			runtime->dma_area[5],
			runtime->dma_area[6],
			runtime->dma_area[7]
			  );
		/*	for(count = 0; count < 8; count ++) {
				writel(0, i2s->base + LTHR0);
				writel(0, i2s->base + RTHR0);
			}
		*/

		enable_irq(i2s->irq);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
#ifdef	NS2816_DEMO
	writel(0, scm_base + SCM_DEMO_FLAG);	//trigger stop flag		
#endif
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			reg_value = readl(i2s->base + IMR0 + reg_offset);
			reg_value |= (TXFOM | TXFEM);
			writel(reg_value, i2s->base + IMR0 + reg_offset);
		}
		else {
			reg_value = readl(i2s->base + IMR0 + reg_offset);
			reg_value |= (RXFOM | RXFFM);
			writel(reg_value, i2s->base + IMR0 + reg_offset);
		}	
		writel(0, i2s->base + IER);
		writel(1, i2s->base + CER);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void nusmart_i2s_shutdown(struct snd_pcm_substream *substream, 
			struct snd_soc_dai * cpu_dai) 
{
	//struct nusmart_i2s_data * i2s = cpu_dai->private_data;
	struct nusmart_i2s_data * i2s = nusmart_i2s;
	unsigned int reg_offset = substream->number * 0x40, reg_value;
	
	DBG_PRINT("nusmart_i2s_shutdown, i2s_ch = %d, stream = %d\n", 
		substream->number, substream->stream);
	
	disable_irq(i2s->irq);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		reg_value = readl(i2s->base + IMR0 + reg_offset);
		reg_value |= TXFOM | TXFEM;
		writel(reg_value, i2s->base + IMR0 + reg_offset);
		writel(1, i2s->base + TFF0 + reg_offset);
		readl(i2s->base + TOR0 + reg_offset);
		writel(0, i2s->base + TER0 + reg_offset);

		if((readl(i2s->base + TER0) == 0) && 
			(readl(i2s->base + TER0 + 0x40) == 0) &&
			(readl(i2s->base + TER0 + 0x80) == 0) &&
			(readl(i2s->base + TER0 + 0xc0) == 0)) {
			writel(0, i2s->base + ITER);
			DBG_PRINT("disable i2s transmitter block\n");
		}
	} else {
		reg_value = readl(i2s->base + IMR0 + reg_offset);
		reg_value |= RXFOM | RXFFM;
		writel(reg_value, i2s->base + IMR0 + reg_offset);
		writel(1, i2s->base + RFF0 + reg_offset);
		readl(i2s->base + ROR0 + reg_offset);
		writel(0, i2s->base + RER0 + reg_offset);
		writel(0, i2s->base + IRER);
		DBG_PRINT("disable i2s receiver block\n");
	}
	
	if(readl(i2s->base + ITER) == 0 && readl(i2s->base + IRER) == 0) {
		DBG_PRINT("disable i2s block\n");
		writel(0, i2s->base + CER); 
		writel(0, i2s->base + IER); 
		clk_disable(i2s->clk_i2s);
	}

	switch (substream->number) {	
	case 0:
	case 1:
	case 2:
	case 3:
		free_irq(i2s->irq, substream);
		break;
	default:
		break;
	}
	DBG_PRINT("free_irq success: irq = %d, i2s_ch = %d, stream = %d\n", 
			i2s->irq, substream->number, substream->stream);
}

#ifdef CONFIG_PM
static int nusmart_i2s_suspend(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;
	return 0;
}

static int nusmart_i2s_resume(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	return 0;
}

#else
#define nusmart_i2s_suspend	NULL
#define nusmart_i2s_resume	NULL
#endif

 
#ifdef	NS2816_DEMO
static inline void nusmart_i2s_playback(struct snd_pcm_substream *substream )
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *rtd_prv = runtime->private_data;
	void __iomem *scm_base = __io_address(0x051c0000);
	unsigned int trans_size = readl(scm_base + SCM_DEMO_TRANS_SIZE);

//	if(rtd_prv->interrupt_count % 16 == 0)
//		DBG_PRINT("time8 interrupts, trans_size = %x\n", trans_size);
	switch(rtd_prv->fmt) {
	case SNDRV_PCM_FORMAT_S16_LE:
		rtd_prv->tran_bytes_count = trans_size;
		snd_pcm_period_elapsed(substream);
		if(rtd_prv->tran_bytes_count == runtime->dma_bytes)
			rtd_prv->tran_bytes_count = 0;
			
		rtd_prv->interrupt_count++;
		break;
	}	
}
static irqreturn_t nusmart_i2s_interrupt(int irq, void * context)
{
	struct snd_pcm_substream *substream = context;	      

	void __iomem *timer8_va_base = __io_address(0x05100000) + 0x14 * 7; //timer8
	writel(0, timer8_va_base + 0x08);
	readl(timer8_va_base + TIMERX_EOI);
	readl(__io_address(0x05100000) + 0xa8);

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		nusmart_i2s_playback(substream);
	}
		
	return IRQ_HANDLED;
}
#else

static inline void nusmart_i2s_capture(struct snd_pcm_substream *substream )
{

}

static inline void nusmart_i2s_playback(struct snd_pcm_substream *substream )
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	//struct nusmart_i2s_data * i2s = rtd->dai->cpu_dai->private_data;
	struct nusmart_i2s_data * i2s = nusmart_i2s;
	struct nusmart_runtime_data *rtd_prv = runtime->private_data;
	unsigned int offset = 0, count = 0;
	unsigned int *u32d;
	unsigned short *u16d;

	offset = rtd_prv->tran_bytes_count;
	switch(rtd_prv->fmt) {
	case SNDRV_PCM_FORMAT_S16_LE:
		u16d = (unsigned short *)(runtime->dma_area + offset);
		if(runtime->channels == 2) {
			for(count = 0; count < 6; count ++) {
				writel((*u16d), i2s->base + LTHR0);
				u16d ++;	
				writel((*u16d), i2s->base + RTHR0);
				u16d ++;
			}
			rtd_prv->tran_bytes_count += 24;
		}
		else if(runtime->channels == 1) {
			for(count = 0; count < 6; count ++) {
				writel((*u16d), i2s->base + LTHR0);
				writel(0, i2s->base + RTHR0);
				u16d ++;
			}
			rtd_prv->tran_bytes_count += 12;

		}
		snd_pcm_period_elapsed(substream);
		if(rtd_prv->tran_bytes_count == runtime->dma_bytes)
			rtd_prv->tran_bytes_count = 0;
			
		rtd_prv->interrupt_count++;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		DBG_PRINT("error ...\n");
		u32d = (unsigned int *)(runtime->dma_area + offset);
		for(count = 0; count < 6; count ++) {
			writel(*u32d ++, i2s->base + LTHR0);	
			writel(*u32d ++, i2s->base + RTHR0);
		}	
		rtd_prv->tran_bytes_count += 48;
		snd_pcm_period_elapsed(substream);
		break;
	}	
}
 
static irqreturn_t nusmart_i2s_interrupt(int irq, void * context)
{
	struct snd_pcm_substream *substream = context;	      
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct nusmart_i2s_data * i2s = rtd->dai->cpu_dai->private_data;
	struct nusmart_i2s_data * i2s = nusmart_i2s;
	unsigned int isr0;

	isr0 = readl(i2s->base + ISR0);
	if(isr0 & TXFO) {
		readl(i2s->base + TOR0);
	}
	//DBG_PRINT("isr0 = %x, substream = %x\n", isr0, (unsigned int)substream);

	if((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) && (isr0 && TXFE)) {
		nusmart_i2s_playback(substream);
	}
	else if((substream->stream == SNDRV_PCM_STREAM_CAPTURE) && (isr0 && RXFF)) {
		nusmart_i2s_capture(substream);
	}

	isr0 = readl(i2s->base + ISR0);
	if(isr0 & TXFO) {
		readl(i2s->base + TOR0);
		DBG_PRINT("txfo, isr0 = %x\n", isr0);
	}
		
	return IRQ_HANDLED;
}
#endif

static void nusmart_i2s_init(void * context)
{
	//struct nusmart_i2s_data * i2s = (struct nusmart_i2s_data *)context;
	struct nusmart_i2s_data * i2s = nusmart_i2s;
	writel(0, i2s->base + TER0);
	writel(0, i2s->base + TER0 + 0x40);
	writel(0, i2s->base + TER0 + 0x80);
	writel(0, i2s->base + TER0 + 0xc0);
	
	writel(0, i2s->base + RER0);
	writel(0, i2s->base + RER0 + 0x40);
	writel(0, i2s->base + RER0 + 0x80);
	writel(0, i2s->base + RER0 + 0xc0);
	writel(0, i2s->base + IER);
}

static int nusmart_i2s_probe(struct platform_device *pdev, 
		struct snd_soc_dai * cpu_dai)
{
	int irq, ret = 0;
	struct resource * mem_res = NULL;
	struct nusmart_i2s_data * i2s;	
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct clk * clk_i2s = NULL;

	clk_i2s = clk_get(&pdev->dev, "soc-audio");
	if (IS_ERR(clk_i2s)) {
		printk("ns2816_i2s clk_get error\n");
		return PTR_ERR(clk_i2s);
	}

	irq = platform_get_irq(pdev, 0);
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (irq < 0 || !mem_res){
		ret = -ENXIO;
		DBG_PRINT("platform_get_irq fail or get_resource fail!\n");
		goto out_getres;
	}

	//i2s = kmalloc(sizeof(*i2s), GFP_KERNEL);
	i2s = nusmart_i2s;

	if(i2s == NULL) {
		ret = -ENOMEM;
		goto out_kmalloc;
	}

	i2s->clk_i2s = clk_i2s;

	DBG_PRINT("ns2816_i2s_probe :pdev->name = %s\n" ,pdev->name);
	
	if (!request_mem_region(mem_res->start, SZ_4K, pdev->name)){
		ret = -EBUSY;
		goto out_req_memreg;
	}

	i2s->base = ioremap(mem_res->start, SZ_4K);
	if (!i2s->base) {
		ret = -ENOMEM;
		goto out_ioremap;
	}

	i2s->socdev = socdev;
	i2s->irq = irq;
	i2s->mem_res = mem_res;

	nusmart_i2s_init(i2s);
	
//	cpu_dai->private_data = i2s;
#ifdef NS2816_DEMO
	i2s->irq = IRQ_NS2816_TIMERS_INTR7;
	irq = i2s->irq;	
#endif
	DBG_PRINT("nusmart_i2s_probe, irq = %d\n", irq);
	return  0;

out_ioremap:
	release_mem_region(mem_res->start, SZ_4K);
out_req_memreg:
	kfree(i2s);
out_kmalloc:
out_getres:
	clk_put(clk_i2s);
	return ret;	
}

static void nusmart_i2s_remove(struct platform_device *dev,
		struct snd_soc_dai * cpu_dai)
{	
	//struct nusmart_i2s_data * i2s 
	//		= (struct nusmart_i2s_data *)cpu_dai->private_data;
	struct nusmart_i2s_data * i2s = nusmart_i2s;
	
	DBG_PRINT("nusmart_i2s_remove\n");
	iounmap(i2s->base);
	release_mem_region(i2s->mem_res->start, SZ_4K);
	kfree(i2s);
	//cpu_dai->private_data = NULL;
	clk_put(i2s->clk_i2s);
	i2s->clk_i2s = ERR_PTR(-ENOENT);
}

#define NUSMART_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

#define NUSMART_I2S_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
                         SNDRV_PCM_FMTBIT_S24_LE | \
                         SNDRV_PCM_FMTBIT_S32_LE)
static struct snd_soc_dai_ops nusmart_i2s_dai_ops = {
	.startup	= nusmart_i2s_startup,
	.shutdown	= nusmart_i2s_shutdown,
	.trigger	= nusmart_i2s_trigger,
	.hw_params	= nusmart_i2s_hw_params,
	.set_fmt	= nusmart_i2s_set_dai_fmt,
	.set_sysclk	= nusmart_i2s_set_dai_sysclk,
};

struct snd_soc_dai_driver nusmart_i2s_dai = {
	.name = "ns2816-i2s",
	.id = 0,
	.probe = nusmart_i2s_probe,
	.remove = nusmart_i2s_remove,
	.suspend = nusmart_i2s_suspend,
	.resume = nusmart_i2s_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = NUSMART_I2S_RATES,
		.formats = NUSMART_I2S_FORMATS,},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = NUSMART_I2S_RATES,
		.formats = NUSMART_I2S_FORMATS,},
	.ops =  &nusmart_i2s_dai_ops,
	.symmetric_rates = 1,
};

EXPORT_SYMBOL_GPL(nusmart_i2s_dai);

static int ns2816_i2s_probe(struct platform_device *dev)
{
	int ret;

	printk("snd_soc_register_dai: nusmart_i2s_dai\n");
	//nusmart_i2s_dai.dev = &dev->dev;
	//nusmart_i2s_dai.private_data = NULL;
	ret = snd_soc_register_dai(&dev->dev, &nusmart_i2s_dai);

	return ret;
}

static int __devexit ns2816_i2s_remove(struct platform_device *dev)
{
	snd_soc_unregister_dai(&dev->dev);
	return 0;
}

static struct platform_driver ns2816_i2s_driver = {
	.probe = ns2816_i2s_probe,
	.remove = __devexit_p(ns2816_i2s_remove),

	.driver = {
		.name = "ns2816-i2s-plat",
		.owner = THIS_MODULE,
	},
};

static int __init ns2816_i2s_init(void)
{
	printk("ns2816_i2s_driver_init\n");
	return platform_driver_register(&ns2816_i2s_driver);
}

static void __exit ns2816_i2s_exit(void)
{
	platform_driver_unregister(&ns2816_i2s_driver);
}

module_init(ns2816_i2s_init);
module_exit(ns2816_i2s_exit);


/* Module information */
MODULE_AUTHOR("Nufront Software");
MODULE_LICENSE("GPL");

