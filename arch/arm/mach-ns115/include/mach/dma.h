/* arch/arm/mach-ns115/include/mach/dma.h
 *
 * Copyright (C) 2003-2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Nusmart DMA support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef  _DMA_H_
#define _DMA_H_

enum ns115_dma_buffresult {
    NS115_RES_OK,
    NS115_RES_ERR,
    NS115_RES_ABORT
};

enum ns115_dmasrc {
    NS115_DMASRC_HW,		/* source is memory */
    NS115_DMASRC_MEM		/* source is hardware */
};

/* enum ns115_chan_op
 *
 * operation codes passed to the DMA code by the user, and also used
 * to inform the current channel owner of any changes to the system state
 */

enum ns115_chan_op {
    NS115_DMAOP_START,
    NS115_DMAOP_STOP,
    NS115_DMAOP_PAUSE,
    NS115_DMAOP_RESUME,
    NS115_DMAOP_FLUSH,
    NS115_DMAOP_TIMEOUT,		/* internal signal to handler */
    NS115_DMAOP_STARTED,		/* indicate channel started */
    NS115_DMAOP_NONE,
};

struct ns115_dma_client {
    char                *name;
};

struct ns115_dma_chan;
enum dma_ch;

/* ns115_dma_cbfn_t
 *
 * buffer callback routine type
 */

typedef void (*ns115_dma_cbfn_t)(struct ns115_dma_chan *,
        void *buf, int size,
        enum ns115_dma_buffresult result);

typedef int  (*ns115_dma_opfn_t)(struct ns115_dma_chan *,
        enum ns115_chan_op );



/* ns115_dma_request
 *
 * request a dma channel exclusivley
 */

extern int ns115_dma_request(enum dma_ch channel,
        struct ns115_dma_client *, void *dev);


/* ns115_dma_ctrl
 *
 * change the state of the dma channel
 */

extern int ns115_dma_ctrl(enum dma_ch channel, enum ns115_chan_op op);

/* ns115_dma_setflags
 *
 * set the channel's flags to a given state
 */

extern int ns115_dma_setflags(enum dma_ch channel,
        unsigned int flags);

/* ns115_dma_free
 *
 * free the dma channel (will also abort any outstanding operations)
 */

extern int ns115_dma_free(enum dma_ch channel, struct ns115_dma_client *);

/* ns115_dma_enqueue
 *
 * place the given buffer onto the queue of operations for the channel.
 * The buffer must be allocated from dma coherent memory, or the Dcache/WB
 * drained before the buffer is given to the DMA system.
 */

extern int ns115_dma_enqueue(enum dma_ch channel, void *id,
        dma_addr_t data, int size);

/* ns115_dma_config
 *
 * configure the dma channel
 */

extern int ns115_dma_config(enum dma_ch channel, int xferunit);

/* ns115_dma_devconfig
 *
 * configure the device we're talking to
 */

extern int ns115_dma_devconfig(enum dma_ch channel,
        enum ns115_dmasrc source, unsigned long devaddr);

/* ns115_dma_getposition
 *
 * get the position that the dma transfer is currently at
 */

extern int ns115_dma_getposition(enum dma_ch channel,
        dma_addr_t *src, dma_addr_t *dest);

extern int ns115_dma_set_opfn(enum dma_ch, ns115_dma_opfn_t rtn);
extern int ns115_dma_set_buffdone_fn(enum dma_ch, ns115_dma_cbfn_t rtn);

#endif
