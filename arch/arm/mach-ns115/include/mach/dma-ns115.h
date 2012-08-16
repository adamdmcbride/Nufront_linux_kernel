/* linux/arch/arm/mach-ns115/include/mach/dma-ns115.h
 *
 * Copyright (C) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Nufront DMA support - per SoC functions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/dma-core.h>

extern struct sysdev_class dma_sysclass;
extern struct ns115_dma_chan ns115_chans[S3C_DMA_CHANNELS];

#define DMA_CH_VALID		(1<<31)
#define DMA_CH_NEVER		(1<<30)

struct ns115_dma_addr {
    unsigned long		from;
    unsigned long		to;
};

/* struct ns115_dma_map
 *
 * this holds the mapping information for the channel selected
 * to be connected to the specified device
 */

struct ns115_dma_map {
    const char		*name;
    struct ns115_dma_addr  hw_addr;

    unsigned long		 channels[S3C_DMA_CHANNELS];
    unsigned long		 channels_rx[S3C_DMA_CHANNELS];
};

struct ns115_dma_selection {
    struct ns115_dma_map	*map;
    unsigned long		 map_size;
    unsigned long		 dcon_mask;

    void	(*select)(struct ns115_dma_chan *chan,
            struct ns115_dma_map *map);

    void	(*direction)(struct ns115_dma_chan *chan,
            struct ns115_dma_map *map,
            enum ns115_dmasrc dir);
};

extern int ns115_dma_init_map(struct ns115_dma_selection *sel);

/* struct ns115_dma_order_ch
 *
 * channel map for one of the `enum dma_ch` dma channels. the list
 * entry contains a set of low-level channel numbers, orred with
 * DMA_CH_VALID, which are checked in the order in the array.
 */

struct ns115_dma_order_ch {
    unsigned int	list[S3C_DMA_CHANNELS];	/* list of channels */
    unsigned int	flags;				/* flags */
};

/* struct ns115_dma_order
 *
 * information provided by either the core or the board to give the
 * dma system a hint on how to allocate channels
 */

struct ns115_dma_order {
    struct ns115_dma_order_ch	channels[DMACH_MAX];
};

extern int ns115_dma_order_set(struct ns115_dma_order *map);

/* DMA init code, called from the cpu support code */

extern int ns115_dma_init(void);

extern int ns115_dma_init(unsigned int channels, unsigned int irq,
        unsigned int stride);
