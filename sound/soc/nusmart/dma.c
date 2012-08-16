/*
 * dma.c  --  ALSA Soc Audio Layer
 *
 * (c) 2006 Wolfson Microelectronics PLC.
 * Graeme Gregory graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Copyright 2004-2005 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/dma.h>
#include <mach/nusmart-dma-pl330.h>
#include <mach/board-ns115.h>
#include <mach/irqs-ns115.h>
#include "dma.h"

#include "pl330_dmac.h"

#ifdef  PRINT_DBG_INFO 
#define DBG_PRINT(fmt, args...) printk(KERN_INFO "nusmart-mach: " fmt, ## args)
#else
#define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif


#define NS115_DMA4I2S_CHANNELS 2
#define PL330_I2S_OFFSET 6
#define PL330_MAX_CHANS 8

#define _SA     0x400
#define SA(n)       (_SA + (n)*0x20)

#define _DA     0x404
#define DA(n)       (_DA + (n)*0x20)

struct pl330_pridata {
	void __iomem * base;
};

static struct pl330_pridata pl330_data;
static void* dma_chan_param[NS115_DMA4I2S_CHANNELS]; 
struct nusmart_runtime_data *prtd_playback;
struct nusmart_runtime_data *prtd_record;

/* instruction set functions */
static inline int pl330_dmaaddh(u8 *desc_pool_virt, u16 imm, bool ra)
{
	u8 opcode = DMAADDH | (ra << 1);

	writeb(opcode, desc_pool_virt++);
	writew(imm, desc_pool_virt);
	return 3;
}

static inline int pl330_dmaend(u8 *desc_pool_virt)
{
	u8 opcode = DMAEND;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmaflushp(u8 *desc_pool_virt, u8 periph)
{
	u8 opcode = DMAFLUSHHP;

	writeb(opcode, desc_pool_virt++);
	writeb(periph << 3, desc_pool_virt);
	return 2;
}

static inline int pl330_dmald(u8 *desc_pool_virt)
{
	u8 opcode = DMALD;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmalds(u8 *desc_pool_virt)
{
	u8 opcode = DMALDS;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmaldb(u8 *desc_pool_virt)
{
	u8 opcode = DMALDB;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmaldps(u8 *desc_pool_virt, u8 periph)
{
	u8 opcode = DMALDPS;

	writeb(opcode, desc_pool_virt++);
	writeb(periph << 3, desc_pool_virt);
	return 2;
}

static inline int pl330_dmaldpb(u8 *desc_pool_virt, u8 periph)
{
	u8 opcode = DMALDPB;

	writeb(opcode, desc_pool_virt++);
	writeb(periph << 3, desc_pool_virt);
	return 2;
}

static inline int pl330_dmalp(u8 *desc_pool_virt, u8 iter, bool lc)
{
	u8 opcode = DMALP | (lc << 1);

	writeb(opcode, desc_pool_virt++);
	writeb(iter, desc_pool_virt);
	return 2;
}

static inline int pl330_dmalpend(u8 *desc_pool_virt, u8 backwards_jump, bool lc)
{
	u8 opcode = DMALPEND | (lc << 2);

	writeb(opcode, desc_pool_virt++);
	writeb(backwards_jump, desc_pool_virt);
	return 2;
}

static inline int pl330_dmalpends(u8 *desc_pool_virt, u8 backwards_jump,
		bool lc)
{
	u8 opcode = DMALPENDS | (lc << 2);

	writeb(opcode, desc_pool_virt++);
	writeb(backwards_jump, desc_pool_virt);
	return 2;
}

static inline int pl330_dmalpendb(u8 *desc_pool_virt, u8 backwards_jump,
		bool lc)
{
	u8 opcode = DMALPENDB | (lc << 2);

	writeb(opcode, desc_pool_virt++);
	writeb(backwards_jump, desc_pool_virt);
	return 2;
}

static inline int pl330_dmalpfe(u8 *desc_pool_virt, u8 backwards_jump, bool lc)
{
	u8 opcode = DMALPFE | (lc << 2);

	writeb(opcode, desc_pool_virt++);
	writeb(backwards_jump, desc_pool_virt);
	return 2;
}

static inline int pl330_dmakill(u8 *desc_pool_virt)
{
	u8 opcode = DMAKILL;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmamov(u8 *desc_pool_virt, u8 rd, u32 imm)
{
	u8 opcode = DMAMOV;

	writeb(opcode, desc_pool_virt++);
	writeb(rd, desc_pool_virt++);
	writel(imm, desc_pool_virt);
	return 6;
}

static inline int pl330_dmanop(u8 *desc_pool_virt)
{
	u8 opcode = DMANOP;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmarmb(u8 *desc_pool_virt)
{
	u8 opcode = DMARMB;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmasev(u8 *desc_pool_virt, u8 event_num)
{
	u8 opcode = DMASEV;

	writeb(opcode, desc_pool_virt++);
	writeb(event_num << 3, desc_pool_virt);
	return 2;
}

static inline int pl330_dmast(u8 *desc_pool_virt)
{
	u8 opcode = DMAST;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmasts(u8 *desc_pool_virt)
{
	u8 opcode = DMASTS;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmastb(u8 *desc_pool_virt)
{
	u8 opcode = DMASTB;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmastps(u8 *desc_pool_virt, u8 periph)
{
	u8 opcode = DMASTPS;

	writeb(opcode, desc_pool_virt++);
	writeb(periph << 3, desc_pool_virt);
	return 2;
}

static inline int pl330_dmastpb(u8 *desc_pool_virt, u8 periph)
{
	u8 opcode = DMASTPB;

	writeb(opcode, desc_pool_virt++);
	writeb(periph << 3, desc_pool_virt);
	return 2;
}

static inline int pl330_dmastz(u8 *desc_pool_virt)
{
	u8 opcode = DMASTZ;

	writeb(opcode, desc_pool_virt);
	return 1;
}

static inline int pl330_dmawfe(u8 *desc_pool_virt, u8 event_num, bool invalid)
{
	u8 opcode = DMAWFE;

	writeb(opcode, desc_pool_virt++);
	writeb((event_num << 3) | (invalid << 1), desc_pool_virt);
	return 2;
}

static inline int pl330_dmawfps(u8 *desc_pool_virt, u8 periph)
{
	u8 opcode = DMAWFPS;

	writeb(opcode, desc_pool_virt++);
	writeb(periph << 3, desc_pool_virt);
	return 2;
}

static inline int pl330_dmawfpb(u8 *desc_pool_virt, u8 periph)
{
	u8 opcode = DMAWFPB;

	writeb(opcode, desc_pool_virt++);
	writeb(periph << 3, desc_pool_virt);
	return 2;
}

static inline int pl330_dmawfpp(u8 *desc_pool_virt, u8 periph)
{
	u8 opcode = DMAWFPP;

	writeb(opcode, desc_pool_virt++);
	writeb(periph << 3, desc_pool_virt);
	return 2;
}

static inline int pl330_dmawmb(u8 *desc_pool_virt)
{
	u8 opcode = DMAWMB;

	writeb(opcode, desc_pool_virt);
	return 1;
}
/*/let one chan execute a desc*/
static void pl330_dmago(int pl330_ch_id, struct nusmart_runtime_data *runtime,
		bool ns)
{
	unsigned int val;
	DBG_PRINT("Entered %s\n", __func__);

	u8 opcode = DMAGO | (ns << 1);
	val = (pl330_ch_id << 24) | (opcode << 16) | (pl330_ch_id << 8);
	writel(val, pl330_data.base + PL330_DBGINST0);

	val = runtime->lli_start;
	writel(val, pl330_data.base + PL330_DBGINST1);

	writel(0, pl330_data.base + PL330_DBGCMD);
}

/*/let one chan execute a desc*/
static void pl330_dmakill_id(int pl330_ch_id)
{
	unsigned int val;
	val = readl(pl330_data.base+ PL330_DBGSTATUS);
	while(val == 1)
	{
		val = readl(pl330_data.base+ PL330_DBGSTATUS);
	}
	val = DMAKILL<<16 | pl330_ch_id << 8 | 0x01;
	writel(val, pl330_data.base + PL330_DBGINST0);

	writel(0, pl330_data.base + PL330_DBGCMD);
}


static void pl330_getposition(u32 dma_chan, dma_addr_t *src, dma_addr_t *dst)
{
	*src = readl(pl330_data.base + SA(dma_chan));
	*dst = readl(pl330_data.base + DA(dma_chan));
}

static void pl330_inten(u32 dma_chan)
{
	unsigned int inten;

	DBG_PRINT("entered %s\n", __func__);

	inten = readl(pl330_data.base + PL330_INTEN);
	inten |= 1 << dma_chan;
	writel(inten, pl330_data.base + PL330_INTEN);
}

static void pl330_intdisable(u32 dma_chan)
{
	unsigned int inten;

	DBG_PRINT("entered %s\n", __func__);

	inten = readl(pl330_data.base + PL330_INTEN);
	inten &= ~(1 << dma_chan);
	writel(inten, pl330_data.base + PL330_INTEN);
}

static unsigned int pl330_make_instructions(struct snd_pcm_substream *substream, dma_addr_t dest, dma_addr_t src,
		size_t len, enum dma_data_direction direction, bool end_flag)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *prtd = runtime->private_data;
	void *buf = prtd->desc_pool_virt;
	u32 control, control_rest;
	unsigned int inst_size = 0;
	unsigned int loop_count0, loop_count_rest = 0;
	unsigned int loop_count1 = 0;
	unsigned int loop_start0 = 0;
	unsigned int loop_start1 = 0;
	unsigned int loop_startfe = 0;
	unsigned int brst_size = 0;
	unsigned int brst_size_val = 0;


	DBG_PRINT("Entered %s\n", __func__);

	brst_size = prtd->dma_data->i2s_width;
	brst_size_val = 1<<prtd->dma_data->i2s_width;
	/* calculate CCR*/
	if (direction == DMA_TO_DEVICE) {
		control = SRC_INCR | SRC_BURST_LEN_4 | SRC_PROT_CTRL_2 | (brst_size<<1) |
			DST_FIXED | DST_BURST_LEN_4 | DST_PROT_CTRL_2 | (brst_size<<15);
	} else if (direction == DMA_FROM_DEVICE) {
		control = SRC_FIXED | SRC_BURST_LEN_4 | SRC_PROT_CTRL_2 | (brst_size<<1) |
			DST_INCR | DST_BURST_LEN_4 | DST_PROT_CTRL_2 | (brst_size<<15);
	} else {
		control = SRC_INCR | SRC_BURST_LEN_4 | SRC_PROT_CTRL_2 | (brst_size<<1) |
			DST_INCR | DST_BURST_LEN_4 | DST_PROT_CTRL_2 | (brst_size<<15);
	}
	loop_count1 = runtime->periods - 1;
	loop_count0 = len/(4*brst_size_val) - 1;//(burst_len*burst_size)
	loop_count_rest = len % (4*brst_size_val);
	//should calc if the lc overflow, but is not needed here due to dma limit settings

	DBG_PRINT("\ndesc_pool_phys: 0x%x ", prtd->desc_pool_virt);
	DBG_PRINT("\nlli_start: 0x%x ", prtd->lli_start);
	DBG_PRINT("control: 0x%x ", control);
	DBG_PRINT("dest: 0x%x ", dest);
	DBG_PRINT("src: 0x%x ", src);
	DBG_PRINT("len: 0x%x\n", len);
	DBG_PRINT("brst_size_val: 0x%x\n", brst_size_val);
	DBG_PRINT("loop_count1: 0x%x\n", loop_count1);
	DBG_PRINT("loop_count0: 0x%x\n", loop_count0);
	DBG_PRINT("loop_count_rest: 0x%x\n", loop_count_rest);

	u8 periph = prtd->dma_data->dma_interface_id;
	/* write instruction sets on buffer */
	loop_startfe = 0;
	inst_size += pl330_dmamov(buf + inst_size, RD_DAR, dest);
	inst_size += pl330_dmamov(buf + inst_size, RD_SAR, src);
	inst_size += pl330_dmamov(buf + inst_size, RD_CCR, control);

	inst_size += pl330_dmaflushp(buf + inst_size, periph);
	if (loop_count1) {
		inst_size += pl330_dmalp(buf + inst_size, loop_count1, LC_1);
		loop_start1 = inst_size;
	}
	if (loop_count0) {
		inst_size += pl330_dmalp(buf + inst_size, loop_count0, LC_0);
		loop_start0 = inst_size;
	}
	if (direction == DMA_TO_DEVICE) {
		inst_size += pl330_dmawfpb(buf + inst_size, periph);
		inst_size += pl330_dmald(buf + inst_size);
		inst_size += pl330_dmastpb(buf + inst_size, periph);
	} else if (direction == DMA_FROM_DEVICE) {
		inst_size += pl330_dmawfpb(buf + inst_size, periph);
		inst_size += pl330_dmaldpb(buf + inst_size, periph);
		inst_size += pl330_dmast(buf + inst_size);
	} else {
		inst_size += pl330_dmald(buf + inst_size);
		inst_size += pl330_dmarmb(buf + inst_size);
		inst_size += pl330_dmast(buf + inst_size);
		inst_size += pl330_dmawmb(buf + inst_size);
	}


	if (loop_count0)
		inst_size += pl330_dmalpend(buf + inst_size,
				inst_size - loop_start0, LC_0);

	if(loop_count_rest)
	{
		if (direction == DMA_TO_DEVICE) {
			control_rest = SRC_INCR | SRC_BURST_LEN_1 | SRC_PROT_CTRL_2 | (brst_size<<1) |
				DST_FIXED | DST_BURST_LEN_1 | DST_PROT_CTRL_2 |  (brst_size<<15);
		} else if (direction == DMA_FROM_DEVICE) {
			control_rest = SRC_FIXED | SRC_BURST_LEN_1 | SRC_PROT_CTRL_2 | (brst_size<<1) |
				DST_INCR | DST_BURST_LEN_1 | DST_PROT_CTRL_2 |  (brst_size<<15);
		} else {
			control_rest = SRC_INCR | SRC_BURST_LEN_1 | SRC_PROT_CTRL_2 | (brst_size<<1) |
				DST_INCR | DST_BURST_LEN_1 | DST_PROT_CTRL_2 | (brst_size<<15);
		}
		inst_size += pl330_dmamov(buf + inst_size, RD_CCR, control_rest);

		inst_size += pl330_dmalp(buf + inst_size, (loop_count_rest/brst_size_val)-1, LC_0);
		loop_start0 = inst_size;

		if (direction == DMA_TO_DEVICE) {
			inst_size += pl330_dmawfpb(buf + inst_size, periph);
			inst_size += pl330_dmald(buf + inst_size);
			inst_size += pl330_dmastpb(buf + inst_size, periph);
		} else if (direction == DMA_FROM_DEVICE) {
			inst_size += pl330_dmawfpb(buf + inst_size, periph);
			inst_size += pl330_dmaldpb(buf + inst_size, periph);
			inst_size += pl330_dmast(buf + inst_size);
		} else {
			inst_size += pl330_dmald(buf + inst_size);
			inst_size += pl330_dmarmb(buf + inst_size);
			inst_size += pl330_dmast(buf + inst_size);
			inst_size += pl330_dmawmb(buf + inst_size);
		}

		inst_size += pl330_dmalpend(buf + inst_size,
				inst_size - loop_start0, LC_0);
	}

	inst_size += pl330_dmasev(buf + inst_size, prtd->dma_data->dma_chan);
	if (loop_count1){
		inst_size += pl330_dmalpend(buf + inst_size,
				inst_size - loop_start1, LC_1);
	}

	inst_size += pl330_dmalpfe(buf + inst_size,
			inst_size - loop_startfe, LC_1);
	if(end_flag)
	{
		inst_size += pl330_dmasev(buf + inst_size, prtd->dma_data->dma_chan);
		inst_size += pl330_dmaend(buf + inst_size);
	}

	return inst_size;
}

static const struct snd_pcm_hardware dma_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S24_LE |
		SNDRV_PCM_FMTBIT_S8,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 128*1024,
	.period_bytes_min	= 16,
	.period_bytes_max	= 1920,
	.periods_min		= 2,
	.periods_max		= 64,
	.fifo_size		= 32,

};

static int dma_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct nusmart_runtime_data *prtd = runtime->private_data;
	//struct nusmart_pcm_dma_data *dma_data = rtd->cpu_dai->dma_data;
	struct nusmart_pcm_dma_data *dma_data;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);


	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	if (!dma_data)
		return 0;


	switch(params_format(params)) {

		case SNDRV_PCM_FORMAT_S8:
			dma_data->i2s_width = 0;
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			dma_data->i2s_width = 1;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			dma_data->i2s_width = 2;
			break;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	if (prtd->dma_data)
		return 0;
	prtd->dma_data = dma_data;

	return 0;

}

static int dma_hw_free(struct snd_pcm_substream *substream)
{
	//struct runtime_data *prtd = substream->runtime->private_data;

	DBG_PRINT("Entered %s\n", __func__);

	/* TODO - do we need to ensure DMA flushed */
	snd_pcm_set_runtime_buffer(substream, NULL);


	return 0;
}

static irqreturn_t nusmart_pl330_interrupt(int irq, void *params)
{
	unsigned int intstatus;
	unsigned int inten;
	int i;
	struct snd_pcm_substream *substream;

	//DBG_PRINT("Entered %s\n", __func__);
	intstatus = readl(pl330_data.base + PL330_INTSTATUS);
	if (intstatus == 0)
		return IRQ_HANDLED;
	inten = readl(pl330_data.base + PL330_INTEN);
	for (i = PL330_I2S_OFFSET; i < PL330_MAX_CHANS; i++) {
		if (intstatus & (1 << i)) {
			/* Just clear the irq and update related stream, do not to disable the irq*/
			writel(1 << i, pl330_data.base+ PL330_INTCLR);
			/* disable channel interrupt */
			if(dma_chan_param[i - PL330_I2S_OFFSET])
			{
				substream = dma_chan_param[i - PL330_I2S_OFFSET];
				snd_pcm_period_elapsed(substream);
			}
		}
	}
	return IRQ_HANDLED;
}
static int dma_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *prtd = runtime->private_data;
	struct nusmart_pcm_dma_data *dma_data = prtd->dma_data;
	size_t                  len;
	dma_addr_t              dest,src;
	unsigned int            src_width;
	unsigned int            dst_width;
	u32        inst_size;

	len = runtime->dma_bytes;

	DBG_PRINT("Entered %s\n", __func__);
	//dma burst width select, and actually width should equal to the available bit of PCM
	src_width = dst_width = dma_data->i2s_width;
	len = (runtime->period_size << src_width) * runtime->channels;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		dest = dma_data->dev_addr;
		src  = runtime->dma_addr;
		inst_size = pl330_make_instructions(substream, dest, src,
				len, DMA_TO_DEVICE, 0);
	}
	else
	{
		src   = dma_data->dev_addr;
		dest  = runtime->dma_addr;
		inst_size = pl330_make_instructions(substream, dest, src,
				len, DMA_FROM_DEVICE, 0);
	}

	DBG_PRINT("nusmart_pcm_prepare, period = %d, period_size=%d, buffer=%d, inst_size = %d\n",
			runtime->periods, runtime->period_size, runtime->buffer_size, inst_size);

	return 0;
}

static int dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *prtd = runtime->private_data;
	struct nusmart_pcm_dma_data *dma_data = prtd->dma_data;
	int ret = 0;

	DBG_PRINT("Entered %s with cmd of %d\n", __func__, cmd);

	spin_lock(&prtd->lock);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			//prtd->state |= ST_RUNNING;
			dma_chan_param[dma_data->dma_chan - PL330_I2S_OFFSET] = substream;
			pl330_inten(dma_data->dma_chan);
			pl330_dmago(dma_data->dma_chan, prtd, 0);
			break;

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			//prtd->state &= ~ST_RUNNING;
			dma_chan_param[dma_data->dma_chan - PL330_I2S_OFFSET] = NULL;
			pl330_intdisable(dma_data->dma_chan);
			pl330_dmakill_id(dma_data->dma_chan);
			break;

		default:
			ret = -EINVAL;
			break;
	}

	spin_unlock(&prtd->lock);

	return ret;
}

static snd_pcm_uframes_t dma_pointer(struct snd_pcm_substream *substream)
{

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *prtd = runtime->private_data;
	dma_addr_t src, dst;
	dma_addr_t ptr;
	snd_pcm_uframes_t offset;

	pl330_getposition(prtd->dma_data->dma_chan, &src, &dst);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ptr = dst;
		offset = bytes_to_frames(runtime, ptr - runtime->dma_addr);
	} else {
		ptr = src;
		offset = bytes_to_frames(runtime, ptr - runtime->dma_addr);
	}

	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

static int dma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	DBG_PRINT("nusmart_pcm_open\n");
	snd_soc_set_runtime_hwparams(substream, &dma_hardware);
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		runtime->private_data = prtd_playback;
	}
	else
	{
		runtime->private_data = prtd_record;
	}
	return ret;
}

static int dma_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;


	struct nusmart_pcm_dma_data *dma_data;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	DBG_PRINT("dma_close, chan = %d\n", dma_data->dma_chan);

	return 0;
}

static int dma_mmap(struct snd_pcm_substream *substream,
		struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	DBG_PRINT("Entered %s\n", __func__);

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
			runtime->dma_area,
			runtime->dma_addr,
			runtime->dma_bytes);
}

static struct snd_pcm_ops dma_ops = {
	.open		= dma_open,
	.close		= dma_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= dma_hw_params,
	.hw_free	= dma_hw_free,
	.prepare	= dma_prepare,
	.trigger	= dma_trigger,
	.pointer	= dma_pointer,
	.mmap		= dma_mmap,
};

static int preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = dma_hardware.buffer_bytes_max;

	DBG_PRINT("Entered %s\n", __func__);

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
			&buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void dma_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	DBG_PRINT("Entered %s\n", __func__);

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				buf->area, buf->addr);
		buf->area = NULL;
	}
}

static u64 dma_mask = DMA_BIT_MASK(32);

static int dma_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	DBG_PRINT("Entered %s\n", __func__);

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &dma_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (dai->driver->playback.channels_min) {
		ret = preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->driver->capture.channels_min) {
		ret = preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
out:
	return ret;
}


static int nusmart_pcm_probe(struct snd_soc_platform *platform)
{
	int ret = 0;

	DBG_PRINT("nusmart_pcm_probe\n");

	pl330_data.base = __io_address(NS115_DMA_330S_BASE);
	ret = request_irq(IRQ_NS115_DMA330_INTR6, nusmart_pl330_interrupt, IRQF_SHARED,
			"alsa_pl330_dmac",  &dma_chan_param);
	if (ret)
	{
		DBG_PRINT("request irq failed\n");
		goto out;
	}
	ret = request_irq(IRQ_NS115_DMA330_INTR7, nusmart_pl330_interrupt, IRQF_SHARED,
			"alsa_pl330_dmac",  &dma_chan_param);
	if (ret)
	{
		DBG_PRINT("request irq failed\n");
		goto err_free_irq1;
	}

	prtd_record = kzalloc(sizeof(*prtd_record), GFP_KERNEL);
	if (prtd_record == NULL) {
		DBG_PRINT("nusmart_pcm_probe can not alloc nusmart_runtime_data for record\n");
		ret = -ENOMEM;
		goto err_free_irq0;
	}

	prtd_record->desc_pool_virt = dma_alloc_coherent(NULL, PL330_POOL_SIZE, &(prtd_record->lli_start), GFP_KERNEL);

	if(prtd_record->desc_pool_virt == NULL)
	{
		DBG_PRINT("nusmart_pcm_probe can not alloc dma descriptor for record\n");
		ret = -ENOMEM;
		goto err_free_prtd_record;
	}

	spin_lock_init(&prtd_record->lock);

	prtd_playback = kzalloc(sizeof(*prtd_playback), GFP_KERNEL);
	if (prtd_playback == NULL) {
		DBG_PRINT("nusmart_pcm_probe can not alloc nusmart_runtime_data for playback\n");
		ret = -ENOMEM;
		goto err_free_prtd_record_pool;
	}

	prtd_playback->desc_pool_virt = dma_alloc_coherent(NULL, PL330_POOL_SIZE, &(prtd_playback->lli_start), GFP_KERNEL);

	if(prtd_playback->desc_pool_virt == NULL)
	{
		DBG_PRINT("nusmart_pcm_probe can not alloc dma descriptor for record\n");
		ret = -ENOMEM;
		goto err_free_prtd_playback;
	}

	spin_lock_init(&prtd_playback->lock);

	goto out;

err_free_prtd_playback:
	kfree(prtd_playback);

err_free_prtd_record_pool:
	dma_free_coherent(NULL, PL330_POOL_SIZE, prtd_record->desc_pool_virt, prtd_record->lli_start);

err_free_prtd_record:
	kfree(prtd_record);

err_free_irq0:
	free_irq(IRQ_NS115_DMA330_INTR7, &dma_chan_param);

err_free_irq1:
	free_irq(IRQ_NS115_DMA330_INTR6, &dma_chan_param);

out:
	return ret;
}

static int nusmart_pcm_remove(struct snd_soc_platform *platform)
{

	free_irq(IRQ_NS115_DMA330_INTR7, &dma_chan_param);
	free_irq(IRQ_NS115_DMA330_INTR6, &dma_chan_param);
	dma_free_coherent(NULL, PL330_POOL_SIZE, prtd_record->desc_pool_virt, prtd_record->lli_start);
	kfree(prtd_record);
	dma_free_coherent(NULL, PL330_POOL_SIZE, prtd_playback->desc_pool_virt, prtd_playback->lli_start);
	kfree(prtd_playback);

	return 0;
}

static struct snd_soc_platform_driver nusmart_asoc_platform = {
	.probe		= nusmart_pcm_probe,
	.remove		= nusmart_pcm_remove,
	.ops		= &dma_ops,
	.pcm_new	= dma_new,
	.pcm_free	= dma_free_dma_buffers,
};

static int __devinit nusmart_asoc_platform_probe(struct platform_device *pdev)
{
	DBG_PRINT("******************nusmart_asoc_platform_probe*************************\n");
	return snd_soc_register_platform(&pdev->dev, &nusmart_asoc_platform);
}

static int __devexit nusmart_asoc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver asoc_dma_driver = {
	.driver = {
		.name = "nusmart-pcm-audio",
		.owner = THIS_MODULE,
	},

	.probe = nusmart_asoc_platform_probe,
	.remove = __devexit_p(nusmart_asoc_platform_remove),
};

static int __init nusmart_asoc_init(void)
{
	return platform_driver_register(&asoc_dma_driver);
}
module_init(nusmart_asoc_init);

static void __exit nusmart_asoc_exit(void)
{
	platform_driver_unregister(&asoc_dma_driver);
}
module_exit(nusmart_asoc_exit);

MODULE_AUTHOR("Nufront");
MODULE_DESCRIPTION("Nufront ASoC DMA Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nufront-audio");
