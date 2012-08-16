/*
 *  dma.h --
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  ALSA PCM interface for the NS115&NS2816 CPU
 */

#ifndef _NUSMART_AUDIO_H
#define _NUSMART_AUDIO_H

#if 0
struct nusmart_dma_params {
    struct nusmart115_dma_client *client;	/* stream identifier */
    int channel;				/* Channel ID */
    dma_addr_t dma_addr;
    int dma_size;			/* Size of the DMA transfer */
};
#endif

struct nusmart_pcm_dma_data {
    char *name;                     /* stream identifier */
    //  struct dma_chan *chan;
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
    void                            *desc_pool_virt;
    //struct dw_lli                   *lli;  //64 lli max
    dma_addr_t                      lli_start;
    struct nusmart_pcm_dma_data     *dma_data;
    u8                              mask;
};

extern struct snd_soc_platform nusmart_soc_platform;

#endif
