 /*
 * Copyright (C) 2012 Nufront Co. Ltd.
 *	Liu Zhi Gang <zhigang.liu@nufront.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __NUSMART_PL330_PDATA_H
#define __NUSMART_PL330_PDATA_H

#include <mach/nusmart-dma-pl330.h>

/*
 * Every PL330 DMAC has max 32 peripheral interfaces,
 * of which some may be not be really used in your
 * DMAC's configuration.
 * Populate this array of 32 peri i/fs with relevant
 * channel IDs for used peri i/f and DMACH_MAX for
 * those unused.
 *
 * The platforms just need to provide this info
 * to the NUSMART DMA API driver for PL330.
 */
struct nusmart_pl330_platdata {
	enum dma_ch peri[32];
};

#endif /* __NUSMART_PL330_PDATA_H */
