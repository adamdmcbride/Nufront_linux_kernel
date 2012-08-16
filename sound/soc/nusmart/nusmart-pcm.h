#ifndef __NUSMART_PCM_H__
#define __NUSMART_PCM_H__

struct nusmart_pcm_dma_data {
        char *name;                     /* stream identifier */
//	struct dma_chan *chan;
        volatile u32 dma_chan;            /* the DMA request channel to use */
        u32 dev_addr;                   /* device physical address for DMA */
	u32 dma_interface_id;		
	u32 i2s_chan;		
	u32 i2s_width;		
};

struct nusmart_runtime_data {
                spinlock_t                      lock;
                void __iomem                    *ch_regs;  //reg for this channel
                void __iomem                    *dw_regs;  //reg for dw 
                struct dw_lli                   *lli;  //64 lli max
                dma_addr_t                      lli_start;
                struct nusmart_pcm_dma_data     *dma_data;
                u8                              mask;
		u32				interrupt_count;
		u32				samples_count;
};

extern struct snd_soc_platform_driver nusmart_soc_platform;

#endif
