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

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <sound/pcm_params.h>
#include <asm/dma.h>

#include <mach/nusmart-dma-pl330.h>
#include <mach/board-ns115.h>
#include "nusmart-eva-i2s-dai.h"
#include "dma.h"
#include "pl330_dmac.h"

#ifdef  PRINT_DBG_INFO 
#define DBG_PRINT(fmt, args...) printk( KERN_INFO "nusmart-mach: " fmt, ## args) 
#else
#define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif

//used to save reg value when suspend
struct eva_i2s_pridata {
	u32 ctrl;
	u32 stat;
	u32 srr;
	u32 cid_ctrl;
	u32 tfifo_ctrl;
	u32 rfifo_ctrl;
	u32 dev_conf;
	u32 fmt;
	void __iomem * base;
};
struct hdmi_params {
	u32 sample_rate;
	u32 channels;
	u32 resolution;
};
enum ns115_hdmi_status {
	NS115_HDMI_ON,
	NS115_HDMI_OFF,
	NS115_HDMI_NONE,
};

#ifdef CONFIG_FB_HDMI_NS115
extern void ns115_hdmi_i2s_audio(u32 sample_rate, u32 channels, u32 resolution);
#endif

static int state = NS115_DMAOP_NONE;
static int hdmi_state = NS115_HDMI_NONE;
static struct eva_i2s_pridata eva_i2s_data;
static struct clk *eva_i2s_clk;
static struct clk *ns115_m_clk;
static struct hdmi_params hdmi_hw_params;

static struct nusmart_pcm_dma_data eva_i2s_pcm_stereo_out = {
	.name			= "I2S PCM Stereo out",
	.dev_addr		= (NS115_I2S0_BASE + NS115_IISFIFO),   //only for in PBX
	.dma_interface_id	= 23,
	.i2s_chan		= 1,
	.dma_chan		= 6,
};

static struct nusmart_pcm_dma_data eva_i2s_pcm_stereo_in = {
	.name			= "I2S PCM Stereo in",
	.dev_addr               = (NS115_I2S0_BASE + NS115_IISFIFO),
	.dma_interface_id	= 22,
	.i2s_chan		= 0,
	.dma_chan		= 7,
};

void hdmi_notifier(int flag)
{
	if(flag)
	{
		/*hdmi hotplug in*/
		hdmi_state = NS115_HDMI_ON;
		if(state == NS115_DMAOP_STARTED)
		{
			/*config hdmi audio params*/
#ifdef CONFIG_FB_HDMI_NS115
			ns115_hdmi_i2s_audio(hdmi_hw_params.sample_rate, hdmi_hw_params.channels, hdmi_hw_params.resolution);
#endif
		}
	}
	else
	{
		/*hdmi hotplug out*/
		hdmi_state = NS115_HDMI_OFF;
	}
}
EXPORT_SYMBOL_GPL(hdmi_notifier);

void eva_i2s_print_reg(void)
{

	DBG_PRINT("eva_i2s:ctrl = 0x%x\n", __raw_readl(eva_i2s_data.base+CTRL));
	DBG_PRINT("eva_i2s:stat = 0x%x\n", __raw_readl(eva_i2s_data.base+STAT));
	DBG_PRINT("eva_i2s:srr = 0x%x\n", __raw_readl(eva_i2s_data.base+SRR));
	DBG_PRINT("eva_i2s:cid = 0x%x\n", __raw_readl(eva_i2s_data.base+CID_CTRL));
	DBG_PRINT("eva_i2s:tfifo_ctrl = 0x%x\n", __raw_readl(eva_i2s_data.base+TFIFO_CTRL));
	DBG_PRINT("eva_i2s:rfifo_ctrl = 0x%x\n", __raw_readl(eva_i2s_data.base+RFIFO_CTRL));
	DBG_PRINT("eva_i2s:dev_conf = 0x%x\n", __raw_readl(eva_i2s_data.base+DEV_CONF));
	DBG_PRINT("eva_i2s:tfifo_stat = 0x%x\n", __raw_readl(eva_i2s_data.base+TFIFO_STAT));
	DBG_PRINT("eva_i2s:rfifo_stat = 0x%x\n", __raw_readl(eva_i2s_data.base+RFIFO_STAT));
}


static int eva_i2s_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	DBG_PRINT("%s ....\n", __func__);

	BUG_ON(IS_ERR(eva_i2s_clk));
	clk_enable(ns115_m_clk);
	clk_enable(eva_i2s_clk);
	return 0;
}

/* wait for I2S controller to be ready */
static int eva_i2s_wait(void)
{
	unsigned int i2s_reg;

	/* flush the FIFO */
	i2s_reg = __raw_readl(eva_i2s_data.base + CTRL);
	i2s_reg &= ~(TFIFO_RST | RFIFO_RST);
	__raw_writel(i2s_reg, eva_i2s_data.base + CTRL);

	return 0;
}

static int eva_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	/* interface format */
	if(fmt == SND_SOC_DAIFMT_I2S)
		return 0;
	else
	{
		DBG_PRINT("eva_i2s_set_dai_fmt, fmt = %d\n", fmt);
		return -EINVAL;
	}
}

static int eva_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{

	DBG_PRINT("set_dai_sysclk, cpu_dai->active = %d\n", cpu_dai->active);

	DBG_PRINT("set_dai_sysclk, freq = %d, dir = %d\n", freq, dir);
	return 0;
}


static int eva_i2s_cal_sample_rate(int fclk, int asf)
{
	int width = 32;
	int sample_rate = 0;
	int aux1,aux2,aux3,aux4,aux5;
	int error;
	int i;

	aux1 = (int)(fclk/asf);
	sample_rate = 0;
	error = 1000;
	for(i=0; i<2; i++) {
		aux2 = (int)(aux1/(2*width))+i;
		aux3 = aux2*2*width;
		aux4 = fclk/aux3;
		aux5 = 1000*(asf-aux4)/asf;
		aux5 = aux5 <0?(0-aux5):aux5;
		if (aux5 < error) {
			sample_rate=aux2;
			error=aux5;
		}
	}

	return sample_rate;
}

static int eva_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	unsigned int resolution, sample_rate, reg;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct nusmart_pcm_dma_data * pda;
	unsigned long clk_rate = 0; 

	eva_i2s_wait();

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pda = &eva_i2s_pcm_stereo_out;
	else
		pda = &eva_i2s_pcm_stereo_in;

	snd_soc_dai_set_dma_data(dai, substream, pda);

	switch(params_format(params)) {

		case SNDRV_PCM_FORMAT_S8:
			resolution = 8;
			//pda->i2s_width = 0;
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			resolution = 16;
			//pda->i2s_width = 1;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			resolution = 24;
			//pda->i2s_width = 2;
			break;
	}

	clk_rate = clk_get_rate(eva_i2s_clk);
	sample_rate = eva_i2s_cal_sample_rate(clk_rate, params_rate(params));

	DBG_PRINT("eva-i2s-hw-params: resolution=%d, i2s_clk=%d,rate=%d, sample_rate=%d\n",resolution, clk_rate,params_rate(params),sample_rate);

	//SRR reg
	__raw_writel(SSR(sample_rate,resolution), eva_i2s_data.base+SRR);

	//dev_conf
	__raw_writel(0x0209, eva_i2s_data.base+DEV_CONF);


	//CTRL and fifo reg
	reg = __raw_readl(eva_i2s_data.base + CTRL);
	reg |= TSYNC_LOOP_BACK | T_MS  | TSYNC_RST | 0x1E << 8;  //chan 0 = ADC, chan 1-4 = DAC
	__raw_writel(reg, eva_i2s_data.base + CTRL);

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		__raw_writel(FIFOC(8,24), eva_i2s_data.base + TFIFO_CTRL);

		/*config for HDMI audio*/
		hdmi_hw_params.sample_rate = params_rate(params);
		hdmi_hw_params.resolution = resolution;

	}
	else
	{
		__raw_writel(FIFOC(8,24), eva_i2s_data.base + RFIFO_CTRL);
		//enable chan 1 as receiver
		reg = __raw_readl(eva_i2s_data.base + CTRL);
		reg |= RSYNC_LOOP_BACK | RSYNC_RST;
		//reg |= RSYNC_RST|LOOPBACK_0_1;
		__raw_writel(reg, eva_i2s_data.base + CTRL);
	}

	return 0;
}
static int eva_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	int reg;

	int ret = 0;

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct nusmart_runtime_data *prtd = runtime->private_data;
	struct nusmart_pcm_dma_data *pda = prtd->dma_data;

	reg = __raw_readl(eva_i2s_data.base + STAT);


	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		reg &= ~0x1;
		__raw_writel(reg, eva_i2s_data.base + STAT);
		if(runtime->channels == 2)
			pda->i2s_chan = 0x2;
		if(runtime->channels == 4)
			pda->i2s_chan = 0x6;
		if(runtime->channels == 6)
			pda->i2s_chan = 0xE;
		if(runtime->channels == 8)
			pda->i2s_chan = 0x1E;

		hdmi_hw_params.channels = runtime->channels;

		if(hdmi_state == NS115_HDMI_ON)
		{
#ifdef CONFIG_FB_HDMI_NS115
			ns115_hdmi_i2s_audio(hdmi_hw_params.sample_rate, hdmi_hw_params.channels, hdmi_hw_params.resolution);
#endif
		}
	}
	else
	{
		reg &= ~0x10;
		__raw_writel(reg, eva_i2s_data.base + STAT);
		pda->i2s_chan = 0x1;
	}

	DBG_PRINT("eva_i2s_trigger, cmd=%d, chans = 0x%d\n", cmd, runtime->channels);
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:

			reg = __raw_readl(eva_i2s_data.base + CTRL);
			if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			{
				reg &= ~TSYNC_RST;   //set chan as transmit and enable
				__raw_writel(reg, eva_i2s_data.base + CTRL);

			}
			reg |= pda->i2s_chan | TSYNC_RST;   //set chan as transmit and enable
			__raw_writel(reg, eva_i2s_data.base + CTRL);

			reg = __raw_readl(eva_i2s_data.base + CID_CTRL);
			reg |= 0x28000;   //enable int masks
			__raw_writel(reg, eva_i2s_data.base + CID_CTRL);

			state = NS115_DMAOP_STARTED;
			break;

		case SNDRV_PCM_TRIGGER_STOP:
			reg = __raw_readl(eva_i2s_data.base + CTRL);
			reg &= ~(pda->i2s_chan);
			__raw_writel(reg, eva_i2s_data.base + CTRL);

			reg = __raw_readl(eva_i2s_data.base + CID_CTRL);
			reg &= ~0x28000;   //disable int masks
			__raw_writel(reg, eva_i2s_data.base + CID_CTRL);

			break;
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			break;

		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			state = NS115_DMAOP_STARTED;
			break;

		default:
			ret = -EINVAL;
	}

	eva_i2s_print_reg();
	return ret;


}

static void eva_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	DBG_PRINT("%s ....\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	} else {
	}

	//reset
	//__raw_writel(0, eva_i2s_data.base + CTRL);

	clk_disable(eva_i2s_clk);
}

#ifdef CONFIG_PM
static int eva_i2s_suspend(struct snd_soc_dai *dai)
{
	//struct eva_i2s_pridata *pdata = (struct eva_i2s_pridata *)dai->private_data;
	struct eva_i2s_pridata *pdata = &eva_i2s_data;
	void __iomem * base = pdata->base;

	DBG_PRINT("eva_i2s_suspend\n");
	/* store registers */
	pdata->ctrl = __raw_readl(base+CTRL);
	pdata->stat = __raw_readl(base+STAT);
	pdata->srr  = __raw_readl(base+SRR);
	pdata->cid_ctrl = __raw_readl(base+CID_CTRL);
	pdata->tfifo_ctrl = __raw_readl(base+TFIFO_CTRL);
	pdata->rfifo_ctrl = __raw_readl(base+RFIFO_CTRL);
	pdata->dev_conf = __raw_readl(base+DEV_CONF);

	/* deactivate link */
	eva_i2s_wait();
	return 0;
}

static int eva_i2s_resume(struct snd_soc_dai *dai)
{
	//struct eva_i2s_pridata *pdata = (struct eva_i2s_pridata *)dai->private_data;
	struct eva_i2s_pridata *pdata = &eva_i2s_data;
	void __iomem * base = pdata->base;

	eva_i2s_wait();

	__raw_writel(pdata->ctrl, base+CTRL);
	//__raw_writel(pdata->stat, base+STAT);
	__raw_writel(pdata->srr,base+SRR);
	__raw_writel(pdata->cid_ctrl, base+CID_CTRL);
	__raw_writel(pdata->tfifo_ctrl, base+TFIFO_CTRL);
	__raw_writel(pdata->rfifo_ctrl, base+RFIFO_CTRL);
	__raw_writel(pdata->dev_conf, base+DEV_CONF);

	return 0;
}

#else
#define eva_i2s_suspend	NULL
#define eva_i2s_resume	NULL
#endif

static struct snd_soc_dai_ops eva_i2s_dai_ops = {
	.startup	= eva_i2s_startup,
	.shutdown	= eva_i2s_shutdown,
	.trigger	= eva_i2s_trigger,
	.hw_params	= eva_i2s_hw_params,
	.set_fmt	= eva_i2s_set_dai_fmt,
	.set_sysclk	= eva_i2s_set_dai_sysclk,
};

static void fake_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
}

static int fake_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return 0;
}

static int fake_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	return 0;
}

static int fake_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	return 0;
}

static int fake_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	return 0;
}

static int fake_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static struct snd_soc_dai_ops fake_dai_ops = {
	.startup	= fake_startup,
	.shutdown	= fake_shutdown,
	.trigger	= fake_trigger,
	.hw_params	= fake_hw_params,
	.set_fmt	= fake_set_dai_fmt,
	.set_sysclk	= fake_set_dai_sysclk,
};

struct snd_soc_dai_driver eva_i2s_dai[] = {
	{
		.name = "eva-i2s-dai",
		.id = 0,
		.suspend = eva_i2s_suspend,
		.resume = eva_i2s_resume,
		.playback = {
			.channels_min = 2,
			.channels_max = 8,
			.rates = NUSMART_I2S_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,},
		.capture = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = NUSMART_I2S_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,},
		.ops = &eva_i2s_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "MODEM",
		.playback = {
			.stream_name = "MODEM Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "MODEM Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &fake_dai_ops,
	},
};

/*Used to avoid underr when playback*/
static irqreturn_t ns115_i2s_irq_handler(int irq, void *data)
{
	int reg;
	reg = __raw_readl(eva_i2s_data.base+STAT);
	if(reg & 0x03)
	{
		reg &=0xfffffffe;
		__raw_writel(reg, eva_i2s_data.base+STAT);
		return IRQ_HANDLED;
	}
	else
		return IRQ_NONE;
}

//probe the i2s device as dev of dai and register as the cpu dai
//dev = nusmart_snd_device defined in machine part.
#define PLL5_4ALSA_CLK 983040000
static int eva_i2s_probe(struct platform_device *dev)
{
	int ret, irq, irqret, retries = 0;
	unsigned int i2s_reg;
	static struct clk *ns115_pll5_clk;
	unsigned long pll5_clk_rate;

	DBG_PRINT("%s ....\n", __func__);
	ns115_pll5_clk = clk_get(&dev->dev, "ns115_pll5");
	if (IS_ERR(ns115_pll5_clk))
	{
		DBG_PRINT("eva_i2s_driver_probe: ns115_pll5_clk clk_get error\n");
		return PTR_ERR(ns115_pll5_clk);
	}
	pll5_clk_rate = clk_get_rate(ns115_pll5_clk);
	if(pll5_clk_rate != PLL5_4ALSA_CLK)
	{
		do {
			if(retries++ > 4)
				return PTR_ERR(ns115_pll5_clk);
			ret = clk_set_rate(ns115_pll5_clk, PLL5_4ALSA_CLK);

		} while(ret);
	}

	eva_i2s_clk = clk_get(&dev->dev, "ns115_i2s0");
	if (IS_ERR(eva_i2s_clk))
	{
		DBG_PRINT("eva_i2s_driver_probe: clk_get error\n");
		return PTR_ERR(eva_i2s_clk);
	}
	//ret = snd_soc_register_dai(&dev->dev, &eva_i2s_dai);
	ret = snd_soc_register_dais(&dev->dev, eva_i2s_dai, ARRAY_SIZE(eva_i2s_dai));
	if (ret != 0)
		clk_put(eva_i2s_clk);

	ns115_m_clk = clk_get(&dev->dev, "ns115_mclk");
	if (IS_ERR(ns115_m_clk))
	{
		DBG_PRINT("eva_i2s_driver_probe: ns115_m_clk get error\n");
	}
	eva_i2s_data.base = __io_address(NS115_I2S0_BASE);

	//reset sfr
	i2s_reg = 0;
	__raw_writel(i2s_reg, eva_i2s_data.base+CTRL);

	/*use I2S in IRQ mode*/
	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		DBG_PRINT("Request irq for I2S failed\n");
		return ret;
	}
	else
	{
		irqret = request_irq(irq, ns115_i2s_irq_handler, 0,
				dev_name(&dev->dev), NULL);
	}

	DBG_PRINT("eva_i2s_probe success\n");
	return ret;
}
static int __devexit eva_i2s_remove(struct platform_device *dev)
{
	DBG_PRINT("%s ....\n", __func__);
	snd_soc_unregister_dai(&dev->dev);
	clk_put(eva_i2s_clk);
	eva_i2s_clk = ERR_PTR(-ENOENT);
	return 0;
}

static struct platform_driver eva_i2s_driver = {
	.probe = eva_i2s_probe,
	.remove = __devexit_p(eva_i2s_remove),

	.driver = {
		.name = "ns115-i2s-plat",
		.owner = THIS_MODULE,
	},
};

static int __init eva_i2s_init(void)
{
	eva_i2s_clk = ERR_PTR(-ENOENT);
	return platform_driver_register(&eva_i2s_driver);
}

static void __exit eva_i2s_exit(void)
{
	platform_driver_unregister(&eva_i2s_driver);
}

module_init(eva_i2s_init);
module_exit(eva_i2s_exit);

/* Module information */
MODULE_AUTHOR("Nufront");
MODULE_DESCRIPTION("eva I2S SoC Interface");
MODULE_LICENSE("GPL");
