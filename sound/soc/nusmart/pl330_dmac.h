/*
 * Copyright (C) 2009 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __PL330_DMAC_H
#define __PL330_DMAC_H

#define PL330_MAX_CHANS		8
#define PL330_MAX_LOOPS		256
#define PL330_POOL_SIZE		SZ_16K
#define PL330_DESC_SIZE		2048
#define PL330_DESC_NUM		2

/* registers */
#define PL330_DS		0x00
#define PL330_DPC		0x04
#define PL330_INTEN		0x20			/* R/W */
#define PL330_ES		0x24
#define PL330_INTSTATUS		0x28
#define PL330_INTCLR		0x2c			/* W/O */
#define PL330_FSM		0x30
#define PL330_FSC		0x34
#define PL330_FTM		0x38
#define PL330_FTC(ch)		(0x40 + (ch << 2))
#define PL330_CS(ch)		(0x100 + (ch << 3))
#define PL330_CPC(ch)		(0x104 + (ch << 3))
#define PL330_SA(ch)		(0x400 + (ch << 5))
#define PL330_DA(ch)		(0x404 + (ch << 5))
#define PL330_CC(ch)		(0x408 + (ch << 5))
#define PL330_LC0(ch)		(0x40c + (ch << 5))
#define PL330_LC1(ch)		(0x410 + (ch << 5))
#define PL330_DBGSTATUS		0xd00
#define PL330_DBGCMD		0xd04			/* W/O */
#define PL330_DBGINST0		0xd08			/* W/O */
#define PL330_DBGINST1		0xd0c			/* W/O */
#define PL330_CR0		0xe00
#define PL330_CR1		0xe04
#define PL330_CR2		0xe08
#define PL330_CR3		0xe0c
#define PL330_CR4		0xe10
#define PL330_CRDN		0xe14
#define PL330_PERIPH_ID0	0xfe0
#define PL330_PERIPH_ID1	0xfe4
#define PL330_PERIPH_ID2	0xfe8
#define PL330_PERIPH_ID3	0xfec
#define PL330_PCELL_ID0		0xff0
#define PL330_PCELL_ID1		0xff4
#define PL330_PCELL_ID2		0xff8
#define PL330_PCELL_ID3		0xffc

/* PL330_CC */
#define PL330_SRC_INC			(1)
#define PL330_SRC_BSIZE_1BYTE		(0)
#define PL330_SRC_BSIZE_2BYTE		(1)
#define PL330_SRC_BSIZE_4BYTE		(2)
#define PL330_SRC_BSIZE_8BYTE		(3)
#define PL330_SRC_BSIZE_16BYTE		(4)
#define PL330_SRC_BSIZE(n)		(n)
#define PL330_SRC_BLEN(n)		((n - 1))
#define PL330_DEST_INC			(1)
#define PL330_DEST_BSIZE_1BYTE		(0)
#define PL330_DEST_BSIZE_2BYTE		(1)
#define PL330_DEST_BSIZE_4BYTE		(2)
#define PL330_DEST_BSIZE_8BYTE		(3)
#define PL330_DEST_BSIZE_16BYTE		(4)
#define PL330_DEST_BSIZE(n)		(n)
#define PL330_DEST_BLEN(n)		((n - 1))

/* PL330_DBGSTATUS */
#define PL330_DBG_IDLE		0
#define PL330_DBG_BUSY		1

/* instruction set opcode */
#define DMAADDH			(0x54)
#define DMAEND			(0x00)
#define DMAFLUSHHP		(0x35)
#define DMAGO			(0xa0)
#define DMALD			(0x04)
#define DMALDS			(0x05)
#define DMALDB			(0x07)
#define DMALDPS			(0x25)
#define DMALDPB			(0x27)
#define DMALP			(0x20)
#define DMALPEND		(0x38)
#define DMALPENDS		(0x39)
#define DMALPENDB		(0x3b)
#define DMALPFE			(0x28)
#define DMAKILL			(0x01)
#define DMAMOV			(0xbc)
#define DMANOP			(0xbc)
#define DMARMB			(0x12)
#define DMASEV			(0x34)
#define DMAST			(0x08)
#define DMASTS			(0x09)
#define DMASTB			(0x0b)
#define DMASTPS			(0x29)
#define DMASTPB			(0x2b)
#define DMASTZ			(0x0c)
#define DMAWFE			(0x36)
#define DMAWFPS			(0x30)
#define DMAWFPB			(0x32)
#define DMAWFPP			(0x31)
#define DMAWMB			(0x13)

/* ra DMAADDH */
#define RA_SA			0
#define RA_DA			1

/* ns DMAGO */
#define NS_SECURE		0
#define NS_NONSECURE		1

/* lc DMALP* */
#define LC_0			0
#define LC_1			1

/* rd DMAMOV */
#define RD_SAR			0
#define RD_CCR			1
#define RD_DAR			2

/* invalid DMAWFE */
#define INVALID_OFF		0
#define INVALID_ON		1

#define SRC_FIXED (0x0)
#define SRC_INCR  (0x1)
#define DST_FIXED (0x0 << 14)
#define DST_INCR   (0x1 << 14)

#define SRC_BURST_SIZE_1 (0x0 << 1)
#define SRC_BURST_SIZE_2 (0x1 << 1)
#define SRC_BURST_SIZE_4 (0x2 << 1)
#define SRC_BURST_SIZE_8 (0x3 << 1)
#define SRC_BURST_SIZE_16 (0x4 << 1)

#define DST_BURST_SIZE_1 (0x0 << 15)
#define DST_BURST_SIZE_2 (0x1 << 15)
#define DST_BURST_SIZE_4 (0x2 << 15)
#define DST_BURST_SIZE_8 (0x3 << 15)
#define DST_BURST_SIZE_16 (0x4 << 15)

#define SRC_BURST_LEN_1 (0x0 << 4)
#define SRC_BURST_LEN_2 (0x1 << 4)
#define SRC_BURST_LEN_3 (0x2 << 4)
#define SRC_BURST_LEN_4 (0x3 << 4)
#define SRC_BURST_LEN_5 (0x4 << 4)
#define SRC_BURST_LEN_6 (0x5 << 4)
#define SRC_BURST_LEN_7 (0x6 << 4)
#define SRC_BURST_LEN_8 (0x7 << 4)
#define SRC_BURST_LEN_9 (0x8 << 4)
#define SRC_BURST_LEN_10 (0x9 << 4)
#define SRC_BURST_LEN_11 (0xA << 4)
#define SRC_BURST_LEN_12 (0xB << 4)
#define SRC_BURST_LEN_13 (0xC << 4)
#define SRC_BURST_LEN_14 (0xD << 4)
#define SRC_BURST_LEN_15 (0xE << 4)
#define SRC_BURST_LEN_16 (0xF << 4)

#define DST_BURST_LEN_1 (0x0 << 18)
#define DST_BURST_LEN_2 (0x1 << 18)
#define DST_BURST_LEN_3 (0x2 << 18)
#define DST_BURST_LEN_4 (0x3 << 18)
#define DST_BURST_LEN_5 (0x4 << 18)
#define DST_BURST_LEN_6 (0x5 << 18)
#define DST_BURST_LEN_7 (0x6 << 18)
#define DST_BURST_LEN_8 (0x7 << 18)
#define DST_BURST_LEN_9 (0x8 << 18)
#define DST_BURST_LEN_10 (0x9 << 18)
#define DST_BURST_LEN_11 (0xA << 18)
#define DST_BURST_LEN_12 (0xB << 18)
#define DST_BURST_LEN_13 (0xC << 18)
#define DST_BURST_LEN_14 (0xD << 18)
#define DST_BURST_LEN_15 (0xE << 18)
#define DST_BURST_LEN_16 (0xF << 18)

#define SRC_PROT_CTRL_0 (0x0 << 8)
#define SRC_PROT_CTRL_1 (0x1 << 8)
#define SRC_PROT_CTRL_2 (0x2 << 8)
#define SRC_PROT_CTRL_3 (0x3 << 8)
#define SRC_PROT_CTRL_4 (0x4 << 8)
#define SRC_PROT_CTRL_5 (0x5 << 8)
#define SRC_PROT_CTRL_6 (0x6 << 8)
#define SRC_PROT_CTRL_7 (0x7 << 8)

#define DST_PROT_CTRL_0 (0x0 << 22)
#define DST_PROT_CTRL_1 (0x1 << 22)
#define DST_PROT_CTRL_2 (0x2 << 22)
#define DST_PROT_CTRL_3 (0x3 << 22)
#define DST_PROT_CTRL_4 (0x4 << 22)
#define DST_PROT_CTRL_5 (0x5 << 22)
#define DST_PROT_CTRL_6 (0x6 << 22)
#define DST_PROT_CTRL_7 (0x7 << 22)

#define SRC_CACHE_CTRL_0 (0x0 << 11)
#define SRC_CACHE_CTRL_1 (0x1 << 11)
#define SRC_CACHE_CTRL_2 (0x2 << 11)
#define SRC_CACHE_CTRL_3 (0x3 << 11)
#define SRC_CACHE_CTRL_4 (0x4 << 11)
#define SRC_CACHE_CTRL_5 (0x5 << 11)
#define SRC_CACHE_CTRL_6 (0x6 << 11)
#define SRC_CACHE_CTRL_7 (0x7 << 11)

#define DST_CACHE_CTRL_0 (0x0 << 25)
#define DST_CACHE_CTRL_1 (0x1 << 25)
#define DST_CACHE_CTRL_2 (0x2 << 25)
#define DST_CACHE_CTRL_3 (0x3 << 25)
#define DST_CACHE_CTRL_4 (0x4 << 25)
#define DST_CACHE_CTRL_5 (0x5 << 25)
#define DST_CACHE_CTRL_6 (0x6 << 25)
#define DST_CACHE_CTRL_7 (0x7 << 25)

#endif
