/* arch/arm/mach-ns115/include/mach/dma.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * NUFRONT DMA core support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

extern struct ns115_dma_chan *nusmart_dma_lookup_channel(unsigned int channel);

extern struct ns115_dma_chan *nusmart_dma_chan_map[];

/* the currently allocated channel information */
extern struct ns115_dma_chan ns115_chans[];


