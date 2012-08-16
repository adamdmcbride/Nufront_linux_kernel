/*
 * Copyright (C) 2010 Nufront Co. Ltd.
 *	Liu Zhi Gang <zhigang.liu@nufront.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef	__NUSMART_DMA_PL330_H_
#define	__NUSMART_DMA_PL330_H_

#define NS115_DMAF_AUTOSTART		(1 << 0)
#define NS115_DMAF_CIRCULAR		(1 << 1)

/*
 * PL330 can assign any channel to communicate with
 * any of the peripherals attched to the DMAC.
 * For the sake of consistency across client drivers,
 * We keep the channel names unchanged and only add
 * missing peripherals are added.
 * Order is not important since NUSMART PL330 API driver
 * use these just as IDs.
 */
enum dma_ch {
    DMACH_I2C0_RX,
    DMACH_I2C0_TX,
    DMACH_I2C1_RX,
    DMACH_I2C1_TX,
    DMACH_I2C2_RX,
    DMACH_I2C2_TX,
    DMACH_I2C3_RX,
    DMACH_I2C3_TX,
    DMACH_SSI0_RX,
    DMACH_SSI0_TX,
    DMACH_SSI1_RX,
    DMACH_SSI1_TX,
    DMACH_SSI2_RX,
    DMACH_SSI2_TX,
    DMACH_UART0_RX,
    DMACH_UART0_TX,
    DMACH_UART1_RX,
    DMACH_UART1_TX,
    DMACH_UART2_RX,
    DMACH_UART2_TX,
    DMACH_UART3_RX,
    DMACH_UART3_TX,
    DMACH_I2S0_RX,
    DMACH_I2S0_TX,
    DMACH_I2S1_RX,
    DMACH_I2S1_TX,
    DMACH_SSI3_RX,
    DMACH_SSI3_TX,
    /* END Marker, also used to denote a reserved channel */
    DMACH_MAX,
};

static inline bool nusmart_dma_has_circular(void)
{
    return true;
}

#include <mach/dma.h>

#endif	/* __NUSMART_DMA_PL330_H_ */
