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
#include <linux/slab.h>

//#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <sound/pcm_params.h>
//#include <asm/dma.h>

#include "nusmart-pcm.h"

//#include "../../../drivers/dma/dw_dmac_regs.h"
#include <mach/nusmart_dw_dma.h>
#include <mach/board-ns2816.h>

//#define PRINT_DBG_INFO

#ifdef  PRINT_DBG_INFO 
        //#define DBG_PRINT(fmt, args...) printk( KERN_INFO "nusmart pcm: " fmt, ## args) 
        #define DBG_PRINT(fmt, args...) printk( KERN_EMERG "nusmart pcm: " fmt, ## args) 
#else
        #define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif

static void* dma_chan_param[NS2816_DWC_MAX_CHANNELS]; 
spinlock_t                      dma_chan_lock;
static void __iomem*       g_dw_regs;
#define		PCM_BUFFER_SIZE		(3840 *4)//	(122880)

static u64 nusmart_pcm_dmamask = DMA_BIT_MASK(32);
static const struct snd_pcm_hardware nusmart_pcm_hardware = {
        .info                   = SNDRV_PCM_INFO_MMAP |
                                  SNDRV_PCM_INFO_MMAP_VALID |
                                  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER |
				  SNDRV_PCM_INFO_BATCH,
        .formats                = SNDRV_PCM_FMTBIT_S16_LE |
                                        SNDRV_PCM_FMTBIT_S24_LE |
                                        SNDRV_PCM_FMTBIT_S8,
	.rate_min		= 8000,
	.rate_max		= 192000,
    	.period_bytes_min       = (PCM_BUFFER_SIZE/NS2816_DWC_MAX_CH_DESC), //16,
	.period_bytes_max       = NS2816_DWC_MAX_COUNT,	//one chan max count
        .periods_min            = PCM_BUFFER_SIZE/NS2816_DWC_MAX_COUNT,//32,  //2 at least to get a cyclic buffer
        .periods_max            = NS2816_DWC_MAX_CH_DESC,//max desc - 1
        .buffer_bytes_max       = PCM_BUFFER_SIZE,//128*1024,// all dma desc run with each at max count
//	.channels_min		= 2,
//	.channels_max		= 5,
};

/*
struct nusmart_runtime_data {
		spinlock_t                      lock;
		void __iomem            	*ch_regs;  //reg for this channel
		void __iomem            	*dw_regs;  //reg for dw 
		struct dw_lli			*lli;  //64 lli max
		dma_addr_t			lli_start;  
		struct nusmart_pcm_dma_data     *dma_data;
		u8				mask;
};
*/

static snd_pcm_uframes_t nusmart_pcm_pointer(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct nusmart_runtime_data *prtd = runtime->private_data;
        unsigned long flags;
        
	dma_addr_t ptr;
        snd_pcm_uframes_t offset;

        if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
                ptr = channel_readl(prtd->ch_regs, DAR);
                offset = bytes_to_frames(runtime, ptr - runtime->dma_addr);
        } else {
		ptr = channel_readl(prtd->ch_regs, SAR);
                offset = bytes_to_frames(runtime, ptr - runtime->dma_addr);
	}

        spin_lock_irqsave(&prtd->lock, flags);
	offset = prtd->samples_count;
        spin_unlock_irqrestore(&prtd->lock, flags);
	
        if (offset >= runtime->buffer_size)
                offset = 0;

//	DBG_PRINT("pcm-pointer : offset=0x%x\n", offset);
        return offset;
}

static int nusmart_pcm_mmap(struct snd_pcm_substream *substream,
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

static int nusmart_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct nusmart_runtime_data *prtd = runtime->private_data;
        struct nusmart_pcm_dma_data *dma_data = prtd->dma_data;
        unsigned long flags;
        int ret = 0;

	DBG_PRINT("nusmart_pcm_trigger, cmd=%d\n", cmd);
	



        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        	spin_lock_irqsave(&dma_chan_lock, flags);
		dma_chan_param[dma_data->dma_chan] = substream;
        	spin_unlock_irqrestore(&dma_chan_lock, flags);
	
        	spin_lock_irqsave(&prtd->lock, flags);
		prtd->interrupt_count = 0;
		prtd->samples_count = 0;
		channel_set_bit(prtd->dw_regs, CH_EN, prtd->mask);
        	spin_unlock_irqrestore(&prtd->lock, flags);
                break;
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        	spin_lock_irqsave(&dma_chan_lock, flags);
		dma_chan_param[dma_data->dma_chan] = NULL;
        	spin_unlock_irqrestore(&dma_chan_lock, flags);
        
		spin_lock_irqsave(&prtd->lock, flags);
        	channel_clear_bit(prtd->dw_regs, CH_EN, prtd->mask);
        	spin_unlock_irqrestore(&prtd->lock, flags);
	        break;
        default:
                ret = -EINVAL;
        }

        return ret;
}

void show_dw_regs(struct nusmart_runtime_data *prtd)
{
                DBG_PRINT("DW_CHAN:sar = 0x%x\n", channel_readl(prtd->ch_regs, SAR));
                DBG_PRINT("DW_CHAN:ctlhi = 0x%x\n", channel_readl(prtd->ch_regs, CTL_HI));
                DBG_PRINT("DW_CHAN:ctllo = 0x%x\n", channel_readl(prtd->ch_regs, CTL_LO));
                DBG_PRINT("DW_CHAN:llp = 0x%x\n", channel_readl(prtd->ch_regs, LLP));
                DBG_PRINT("DW_CHAN:cfghi = 0x%x\n", channel_readl(prtd->ch_regs, CFG_HI));
                DBG_PRINT("DW_CHAN:cfglo = 0x%x\n", channel_readl(prtd->ch_regs, CFG_LO));

}


static int nusmart_pcm_prepare(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct nusmart_runtime_data *prtd = runtime->private_data;
        struct nusmart_pcm_dma_data *dma_data = prtd->dma_data;


        size_t                  xfer_count, len;
        size_t                  offset;
        dma_addr_t              dest,src;
        unsigned int            src_width;
        unsigned int            dst_width;
        u32                     ctllo;
        u32                     cfglo = 0;
        u32                     cfghi;
        u32                     rw;
        u32                     lli_cnt = 0;
	//unsigned long 		flags = 0;

	len = runtime->dma_bytes;

	//dma burst width select, and actually width should equal to the available bit of PCM
        src_width = dst_width = dma_data->i2s_width;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		rw = 1;
		dest = dma_data->dev_addr;
		src  = runtime->dma_addr;
		cfghi = (dma_data->dma_interface_id) << DST_PER | 1 << FIFO_MODE;		
                ctllo = DWC_DEFAULT_CTLLO
                        | DWC_CTLL_DST_WIDTH(dst_width)
                        | DWC_CTLL_SRC_WIDTH(src_width)
                        | DWC_CTLL_DST_FIX
                        | DWC_CTLL_SRC_INC
                        | DWC_CTLL_FC_M2P;
		//	| DWC_CTLL_INT_EN;
	}
	else
	{
		rw = 0;
                src   = dma_data->dev_addr;
                dest  = runtime->dma_addr;
                cfghi = (dma_data->dma_interface_id) << SRC_PER | 1 << FIFO_MODE;
                ctllo = DWC_DEFAULT_CTLLO
                        | DWC_CTLL_DST_WIDTH(dst_width)
                        | DWC_CTLL_SRC_WIDTH(src_width)
                        | DWC_CTLL_DST_INC
                        | DWC_CTLL_SRC_FIX
                        | DWC_CTLL_FC_P2M;
                        //| DWC_CTLL_INT_EN;
	}
/*
        for (offset = 0; offset < len; offset += xfer_count << src_width) {
                xfer_count = min_t(size_t, (len - offset) >> src_width,NS2816_DWC_MAX_COUNT);

                prtd->lli[lli_cnt].sar = src + offset*rw;
                prtd->lli[lli_cnt].dar = dest + offset*(1-rw);
                prtd->lli[lli_cnt].ctllo = ctllo;
                prtd->lli[lli_cnt].ctlhi = xfer_count;
		prtd->lli[lli_cnt].llp = prtd->lli_start + sizeof(struct dw_lli) * ((lli_cnt+1)%(runtime->periods));        
		
	DBG_PRINT("lli[%d]:sar=0x%x, dar=0x%x, ctlhi=0x%x, ctllo=0x%x, llp=0x%x\n", lli_cnt, prtd->lli[lli_cnt].sar, prtd->lli[lli_cnt].dar, prtd->lli[lli_cnt].ctlhi, prtd->lli[lli_cnt].ctllo, prtd->lli[lli_cnt].llp);
		lli_cnt++;
	
	}
//	prtd->lli[lli_cnt-1].llp = prtd->lli_start;	
*/ 

	len = (runtime->period_size << src_width) * runtime->channels;
	offset = 0;
	for(lli_cnt=0; lli_cnt<runtime->periods; lli_cnt++)
	{
		prtd->lli[lli_cnt].sar = src + offset*rw;
		prtd->lli[lli_cnt].dar = dest + offset*(1-rw);
		prtd->lli[lli_cnt].ctllo = ctllo;

		prtd->lli[lli_cnt].ctllo = prtd->lli[lli_cnt].ctllo | DWC_CTLL_INT_EN;
		prtd->lli[lli_cnt].ctlhi = runtime->channels*runtime->period_size; 
		prtd->lli[lli_cnt].llp = prtd->lli_start + sizeof(struct dw_lli)*((lli_cnt+1)%(runtime->periods));        	  
		offset += len;
//		DBG_PRINT("lli[%d]:sar=0x%x, dar=0x%x, ctlhi=0x%x, ctllo=0x%x, llp=0x%x\n", lli_cnt, prtd->lli[lli_cnt].sar, prtd->lli[lli_cnt].dar, prtd->lli[lli_cnt].ctlhi, prtd->lli[lli_cnt].ctllo, prtd->lli[lli_cnt].llp);
	}

        dma_writel(prtd->dw_regs, CFG, DW_CFG_DMA_EN);  //enable the whole DW_DMA


        //clear irq
        channel_set_bit(prtd->dw_regs, CLEAR.XFER, prtd->mask);
        channel_set_bit(prtd->dw_regs, CLEAR.BLOCK, prtd->mask);
        channel_set_bit(prtd->dw_regs, CLEAR.ERROR, prtd->mask);

        channel_set_bit(prtd->dw_regs, MASK.XFER, prtd->mask);
        channel_set_bit(prtd->dw_regs, MASK.BLOCK, prtd->mask);
        channel_set_bit(prtd->dw_regs, MASK.ERROR, prtd->mask);
 
	channel_writel(prtd->ch_regs, SAR, prtd->lli[0].sar);
	channel_writel(prtd->ch_regs, DAR, prtd->lli[0].dar);
	channel_writel(prtd->ch_regs, CTL_HI, prtd->lli[0].ctlhi);
	channel_writel(prtd->ch_regs, CTL_LO, prtd->lli[0].ctllo);
	channel_writel(prtd->ch_regs, LLP, prtd->lli[0].llp);
	channel_writel(prtd->ch_regs, CFG_HI, cfghi);
	channel_writel(prtd->ch_regs, CFG_LO, cfglo);

//	lli_cnt = 0;
//	printk("nusmart_pcm_prepare, period = %d, period_size=%d, buffer=%d\n", runtime->periods, runtime->period_size, runtime->buffer_size);


	return 0; 
		
}


//get chan and dw regs address
static int nusmart_pcm_hw_params(struct snd_pcm_substream *substream,
                              struct snd_pcm_hw_params *params)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct nusmart_runtime_data *prtd = runtime->private_data;
        //struct nusmart_pcm_dma_data *dma_data = rtd->cpu_dai->dma_data;
        struct nusmart_pcm_dma_data *dma_data;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	
	DBG_PRINT("nusmart_pcm_hw_params for dw chan %d\n", dma_data->dma_chan);

        /* return if this is a bufferless transfer e.g.
         * codec <--> BT codec or GSM modem -- lg FIXME */
        if (!dma_data)
                return 0;


        snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
        runtime->dma_bytes = params_buffer_bytes(params);

        if (prtd->dma_data)
                return 0;
        prtd->dma_data = dma_data;

	prtd->dw_regs = __io_address(NS2816_DMAC_BASE);
	g_dw_regs = prtd->dw_regs;
	prtd->ch_regs = __io_address(NS2816_DMAC_BASE) + sizeof(struct dw_dma_chan_regs)*dma_data->dma_chan;

	prtd->mask = 1 << dma_data->dma_chan;

	return 0;
}

static int nusmart_pcm_hw_free(struct snd_pcm_substream *substream)
{

 	snd_pcm_set_runtime_buffer(substream, NULL);

        return 0;
}


static irqreturn_t nusmart_dw_dma_interrupt(int irq, void *params)
{
	int i, status, snd_pcm_call;
	struct snd_pcm_substream *substream;
	struct nusmart_runtime_data *prtd;// = substream->runtime->private_data;
        unsigned long flags;

	status = dma_readl(g_dw_regs, STATUS.BLOCK); 

	for(i=0; i<NS2816_DWC_MAX_CHANNELS; i++)
	{

		if(status & (1<<i))
		{
			snd_pcm_call = 0;
			substream = NULL;

       			spin_lock_irqsave(&dma_chan_lock, flags); /*maybe delete dma_chan_lock to perfomance */
			if(dma_chan_param[i]) {
				substream = dma_chan_param[i];
			}
        		spin_unlock_irqrestore(&dma_chan_lock, flags);
        	
			if(substream != NULL) {
				prtd = substream->runtime->private_data;
				spin_lock_irqsave(&prtd->lock, flags);
				prtd->interrupt_count ++;
				prtd->samples_count = prtd->interrupt_count * substream->runtime->period_size;
				if(substream->runtime->periods > 0) 
					prtd->interrupt_count %= substream->runtime->periods;
				snd_pcm_call = 1;	
        			spin_unlock_irqrestore(&prtd->lock, flags);
			}

			if(snd_pcm_call == 1)
				snd_pcm_period_elapsed(substream);

		        channel_set_bit(g_dw_regs, CLEAR.XFER, 1<<i);
		        channel_set_bit(g_dw_regs, CLEAR.BLOCK, 1<<i);
		        channel_set_bit(g_dw_regs, CLEAR.ERROR, 1<<i);
		}
		
	}

	return IRQ_HANDLED;
}


//free nusmart_runtime_data for this substream
//free irq
int nusmart_pcm_close(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *prtd = runtime->private_data;
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct nusmart_pcm_dma_data *dma_data = rtd->dai->cpu_dai->dma_data;

        struct nusmart_pcm_dma_data *dma_data;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
        DBG_PRINT("nusmart_pcm_close, chan = %d\n", dma_data->dma_chan);
	
	dma_free_coherent(NULL, sizeof(struct dw_lli)*NS2816_DWC_MAX_CH_DESC, prtd->lli, prtd->lli_start);

        kfree(prtd);

        return 0;
}

//alloc nusmart_runtime_data for this substream
//register irq
static int nusmart_pcm_open(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct nusmart_runtime_data *prtd;
        int ret;

	DBG_PRINT("nusmart_pcm_open\n");
        snd_soc_set_runtime_hwparams(substream, &nusmart_pcm_hardware);

        /* Ensure that buffer size is a multiple of period size */
        ret = snd_pcm_hw_constraint_integer(runtime,
                                            SNDRV_PCM_HW_PARAM_PERIODS);
        if (ret < 0)
                goto out;

        prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
        if (prtd == NULL) {
                ret = -ENOMEM;
                goto out;
        }

	prtd->lli = dma_alloc_coherent(NULL, sizeof(struct dw_lli)*NS2816_DWC_MAX_CH_DESC, &(prtd->lli_start), GFP_KERNEL);

	if(prtd->lli == NULL)
	{
		ret = -ENOMEM;
                goto out;
	}


        spin_lock_init(&prtd->lock);
        runtime->private_data = prtd;


out:
        return ret;
}

struct snd_pcm_ops nusmart_pcm_ops = {
        .open           = nusmart_pcm_open,
        .close          = nusmart_pcm_close,
        .ioctl          = snd_pcm_lib_ioctl,
        .hw_params      = nusmart_pcm_hw_params,
        .hw_free        = nusmart_pcm_hw_free,
        .prepare        = nusmart_pcm_prepare,
        .trigger        = nusmart_pcm_trigger,
        .pointer        = nusmart_pcm_pointer,
        .mmap           = nusmart_pcm_mmap,
};


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

        DBG_PRINT("pcm_allocate_dma_buffer, dma_phys = 0x%x, size = 0x%x, stream = %x\n",buf->addr,size, stream);

        return 0;
}

static void nusmart_pcm_free_dma_buffers(struct snd_pcm *pcm)
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

                dma_free_writecombine(pcm->card->dev, buf->bytes,
                                      buf->area, buf->addr);
                buf->area = NULL;
        }
}

//static int nusmart_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
//                 struct snd_pcm *pcm)
static int nusmart_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;
        int ret = 0;

        if (!card->dev->dma_mask)
                card->dev->dma_mask = &nusmart_pcm_dmamask;
        if (!card->dev->coherent_dma_mask)
                card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

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

static int nusmart_pcm_probe(struct snd_soc_platform *platform)
{
	int ret= 0;

	printk(KERN_EMERG "%s ...\n", __func__);

        spin_lock_init(&dma_chan_lock);
        ret = request_irq(IRQ_NS2816_DMAC_COMBINED, nusmart_dw_dma_interrupt, IRQF_SHARED, "alsa_dw_dmac", &dma_chan_param);
        if (ret)
        {
                DBG_PRINT("request irq failed\n");
        }
	
	return ret;
}

static int nusmart_pcm_remove(struct snd_soc_platform *platform)
{

	free_irq(IRQ_NS2816_DMAC_COMBINED, &dma_chan_param);
	return 0;
}

struct snd_soc_platform_driver nusmart_soc_platform = {
	.probe		= nusmart_pcm_probe,
	.remove		= nusmart_pcm_remove,
//        .name           = "nusmart-pcm-audio",
        .ops        = &nusmart_pcm_ops,
        .pcm_new        = nusmart_pcm_new,
        .pcm_free       = nusmart_pcm_free_dma_buffers,
};
//EXPORT_SYMBOL_GPL(nusmart_soc_platform);

static int __devinit nusmart_soc_platform_init(struct platform_device *pdev)
{
	printk(KERN_EMERG "%s ...\n", __func__);
	DBG_PRINT("nusmart_soc_platform_init\n");

        return snd_soc_register_platform(&pdev->dev, &nusmart_soc_platform);
}
//module_init(nusmart_soc_platform_init);

static int __devexit nusmart_soc_platform_exit(struct platform_device *pdev)
{
        snd_soc_unregister_platform(&pdev->dev);
	return 0;
}
//module_exit(nusmart_soc_platform_exit);

static struct platform_driver nusmart_pcm_driver = {
	.driver = {
			.name = "nusmart-pcm-audio",
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

MODULE_AUTHOR("Nufront");
MODULE_DESCRIPTION("NS2816 PCM DMA module");
MODULE_LICENSE("GPL");
