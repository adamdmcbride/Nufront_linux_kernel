/*
 * Synopsys DesignWare Multimedia Card Interface driver
 *  (Based on NXP driver for lpc 31xx)
 *
 * Copyright (C) 2009 NXP Semiconductors
 * Copyright (C) 2009, 2010 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _DW_MMC_H_
#define _DW_MMC_H_

#define SDMMC_CTRL		0x000
#define SDMMC_PWREN		0x004
#define SDMMC_CLKDIV		0x008
#define SDMMC_CLKSRC		0x00c
#define SDMMC_CLKENA		0x010
#define SDMMC_TMOUT		0x014
#define SDMMC_CTYPE		0x018
#define SDMMC_BLKSIZ		0x01c
#define SDMMC_BYTCNT		0x020
#define SDMMC_INTMASK		0x024
#define SDMMC_CMDARG		0x028
#define SDMMC_CMD		0x02c
#define SDMMC_RESP0		0x030
#define SDMMC_RESP1		0x034
#define SDMMC_RESP2		0x038
#define SDMMC_RESP3		0x03c
#define SDMMC_MINTSTS		0x040
#define SDMMC_RINTSTS		0x044
#define SDMMC_STATUS		0x048
#define SDMMC_FIFOTH		0x04c
#define SDMMC_CDETECT		0x050
#define SDMMC_WRTPRT		0x054
#define SDMMC_GPIO		0x058
#define SDMMC_TCBCNT		0x05c
#define SDMMC_TBBCNT		0x060
#define SDMMC_DEBNCE		0x064
#define SDMMC_USRID		0x068
#define SDMMC_VERID		0x06c
#define SDMMC_HCON		0x070
#define SDMMC_UHS_REG		0x074
#define SDMMC_BMOD		0x080
#define SDMMC_PLDMND		0x084
#define SDMMC_DBADDR		0x088
#define SDMMC_IDSTS		0x08c
#define SDMMC_IDINTEN		0x090
#define SDMMC_DSCADDR		0x094
#define SDMMC_BUFADDR		0x098
#define SDMMC_DATA		0x100

/* shift bit field */
#define _SBF(f, v)		((v) << (f))

/* Control register defines */
#define SDMMC_CTRL_USE_IDMAC		BIT(25)
#define SDMMC_CTRL_CEATA_INT_EN		BIT(11)
#define SDMMC_CTRL_SEND_AS_CCSD		BIT(10)
#define SDMMC_CTRL_SEND_CCSD		BIT(9)
#define SDMMC_CTRL_ABRT_READ_DATA	BIT(8)
#define SDMMC_CTRL_SEND_IRQ_RESP	BIT(7)
#define SDMMC_CTRL_READ_WAIT		BIT(6)
#define SDMMC_CTRL_DMA_ENABLE		BIT(5)
#define SDMMC_CTRL_INT_ENABLE		BIT(4)
#define SDMMC_CTRL_DMA_RESET		BIT(2)
#define SDMMC_CTRL_FIFO_RESET		BIT(1)
#define SDMMC_CTRL_RESET		BIT(0)
/* Clock Enable register defines */
#define SDMMC_CLKEN_LOW_PWR		BIT(16)
//#define SDMMC_CLKEN_ENABLE		BIT(0)
#define SDMMC_CLKEN_ENABLE(x)		BIT(x)
/* time-out register defines */
#define SDMMC_TMOUT_DATA(n)		_SBF(8, (n))
#define SDMMC_TMOUT_DATA_MSK		0xFFFFFF00
#define SDMMC_TMOUT_RESP(n)		((n) & 0xFF)
#define SDMMC_TMOUT_RESP_MSK		0xFF
/* card-type register defines */
#define SDMMC_CTYPE_8BIT		BIT(16)
#define SDMMC_CTYPE_4BIT		BIT(0)
#define SDMMC_CTYPE_1BIT		0
/* Interrupt status & mask register defines */
#define SDMMC_INT_SDIO			BIT(16)
#define SDMMC_INT_EBE			BIT(15)
#define SDMMC_INT_ACD			BIT(14)
#define SDMMC_INT_SBE			BIT(13)
#define SDMMC_INT_HLE			BIT(12)
#define SDMMC_INT_FRUN			BIT(11)
#define SDMMC_INT_HTO			BIT(10)
#define SDMMC_INT_DTO			BIT(9)
#define SDMMC_INT_RTO			BIT(8)
#define SDMMC_INT_DCRC			BIT(7)
#define SDMMC_INT_RCRC			BIT(6)
#define SDMMC_INT_RXDR			BIT(5)
#define SDMMC_INT_TXDR			BIT(4)
#define SDMMC_INT_DATA_OVER		BIT(3)
#define SDMMC_INT_CMD_DONE		BIT(2)
#define SDMMC_INT_RESP_ERR		BIT(1)
#define SDMMC_INT_CD			BIT(0)
#define SDMMC_INT_ERROR			0xbfc2
/* Command register defines */
#define SDMMC_CMD_START			BIT(31)
#define SDMMC_CMD_CCS_EXP		BIT(23)
#define SDMMC_CMD_CEATA_RD		BIT(22)
#define SDMMC_CMD_UPD_CLK		BIT(21)
#define SDMMC_CMD_INIT			BIT(15)
#define SDMMC_CMD_STOP			BIT(14)
#define SDMMC_CMD_PRV_DAT_WAIT		BIT(13)
#define SDMMC_CMD_SEND_STOP		BIT(12)
#define SDMMC_CMD_STRM_MODE		BIT(11)
#define SDMMC_CMD_DAT_WR		BIT(10)
#define SDMMC_CMD_DAT_EXP		BIT(9)
#define SDMMC_CMD_RESP_CRC		BIT(8)
#define SDMMC_CMD_RESP_LONG		BIT(7)
#define SDMMC_CMD_RESP_EXP		BIT(6)
#define SDMMC_CMD_INDX(n)		((n) & 0x1F)
/* Status register defines */
#define SDMMC_GET_FCNT(x)		(((x)>>17) & 0x1FF)
#define SDMMC_FIFO_SZ			32
/* Internal DMAC interrupt defines */
#define SDMMC_IDMAC_INT_AI		BIT(9)
#define SDMMC_IDMAC_INT_NI		BIT(8)
#define SDMMC_IDMAC_INT_CES		BIT(5)
#define SDMMC_IDMAC_INT_DU		BIT(4)
#define SDMMC_IDMAC_INT_FBE		BIT(2)
#define SDMMC_IDMAC_INT_RI		BIT(1)
#define SDMMC_IDMAC_INT_TI		BIT(0)
/* Internal DMAC bus mode bits */
#define SDMMC_IDMAC_ENABLE		BIT(7)
#define SDMMC_IDMAC_FB			BIT(1)
#define SDMMC_IDMAC_SWRESET		BIT(0)

/* Register access macros */
#define mci_readl(dev, reg)			\
	__raw_readl(dev->regs + SDMMC_##reg)
#define mci_writel(dev, reg, value)			\
	__raw_writel((value), dev->regs + SDMMC_##reg)

/* 16-bit FIFO access macros */
#define mci_readw(dev, reg)			\
	__raw_readw(dev->regs + SDMMC_##reg)
#define mci_writew(dev, reg, value)			\
	__raw_writew((value), dev->regs + SDMMC_##reg)

/* 64-bit FIFO access macros */
#ifdef readq
#define mci_readq(dev, reg)			\
	__raw_readq(dev->regs + SDMMC_##reg)
#define mci_writeq(dev, reg, value)			\
	__raw_writeq((value), dev->regs + SDMMC_##reg)
#else
/*
 * Dummy readq implementation for architectures that don't define it.
 *
 * We would assume that none of these architectures would configure
 * the IP block with a 64bit FIFO width, so this code will never be
 * executed on those machines. Defining these macros here keeps the
 * rest of the code free from ifdefs.
 */
#define mci_readq(dev, reg)			\
	(*(volatile u64 __force *)(dev->regs + SDMMC_##reg))
#define mci_writeq(dev, reg, value)			\
	(*(volatile u64 __force *)(dev->regs + SDMMC_##reg) = value)
#endif

#define DWC_CTRL		 0x00
#define DWC_PWREN		 0x04
#define DWC_CLKDIV		 0x08
#define DWC_CLKSRC		 0x0c
#define DWC_CLKENA		 0x10
#define DWC_TMOUT		 0x14
#define DWC_CTYPE		 0x18
#define DWC_BLKSIZ		 0x1c
#define DWC_BYTCNT		 0x20
#define DWC_INTMASK		 0x24
#define DWC_CMDARG		 0x28
#define DWC_CMD			 0x2c
#define DWC_RESP0		 0x30
#define DWC_RESP1		 0x34
#define DWC_RESP2		 0x38
#define DWC_RESP3		 0x3c
#define DWC_MINTSTS		 0x40
#define DWC_RINTSTS		 0x44
#define DWC_STATUS		 0x48
#define DWC_FIFOTH		 0x4c
#define DWC_CDETECT		 0x50
#define DWC_WRTPRT		 0x54
#define DWC_GPIO		 0x58
#define DWC_TCBCNT		 0x5c
#define DWC_TBBCNT		 0x60
#define DWC_DEBNCE		 0x64
#define DWC_USRID		 0x68
#define DWC_VERID		 0x6c
#define DWC_HCON		 0x70
#define DWC_UHS_REG		 0x74
#define DWC_BMOD		 0x80
#define DWC_PLDMND		 0x84
#define DWC_DBADDR		 0x88
#define DWC_IDSTS		 0x8c
#define DWC_IDINTEN		 0x90
#define DWC_DSCADDR		 0x94
#define DWC_BUFADDR		 0x98

#define NUSMART_MMC_READ(host,reg) mci_readl(host, reg)
#define print_reg(host)	\
{							\
	printk(KERN_EMERG "%s: offset %0x DWC_CTRL %x\n",	__func__, DWC_CTRL, NUSMART_MMC_READ(host, CTRL)); \
	printk(KERN_EMERG "%s: offset %0x DWC_PWREN %x\n",	__func__, DWC_PWREN, NUSMART_MMC_READ(host, PWREN)); \
	printk(KERN_EMERG "%s: offset %0x DWC_CLKDIV %x\n",	__func__, DWC_CLKDIV, NUSMART_MMC_READ(host, CLKDIV)); \
	printk(KERN_EMERG "%s: offset %0x DWC_CLKSRC %x\n",	__func__, DWC_CLKSRC, NUSMART_MMC_READ(host, CLKSRC)); \
	printk(KERN_EMERG "%s: offset %0x DWC_CLKENA %x\n",	__func__, DWC_CLKENA, NUSMART_MMC_READ(host, CLKENA)); \
	printk(KERN_EMERG "%s: offset %0x DWC_TMOUT %x\n",	__func__, DWC_TMOUT, NUSMART_MMC_READ(host, TMOUT));  \
	printk(KERN_EMERG "%s: offset %0x DWC_CTYPE %x\n",	__func__, DWC_CTYPE, NUSMART_MMC_READ(host, CTYPE)); \
	printk(KERN_EMERG "%s: offset %0x DWC_BLKSIZ %x\n",	__func__, DWC_BLKSIZ, NUSMART_MMC_READ(host, BLKSIZ)); \
	printk(KERN_EMERG "%s: offset %0x DWC_BYTCNT %x\n",	__func__, DWC_BYTCNT, NUSMART_MMC_READ(host, BYTCNT)); \
	printk(KERN_EMERG "%s: offset %0x DWC_INTMASK %x\n",	__func__, DWC_INTMASK, NUSMART_MMC_READ(host, INTMASK)); \
	printk(KERN_EMERG "%s: offset %0x DWC_CMDARG %x\n",	__func__, DWC_CMDARG, NUSMART_MMC_READ(host, CMDARG));  \
	printk(KERN_EMERG "%s: offset %0x DWC_CMD %x\n",	__func__, DWC_CMD, NUSMART_MMC_READ(host, CMD)); \
	printk(KERN_EMERG "%s: offset %0x DWC_MINTSTS %x\n",	__func__, DWC_MINTSTS, NUSMART_MMC_READ(host, MINTSTS));  \
	printk(KERN_EMERG "%s: offset %0x DWC_RINTSTS %x\n",	__func__, DWC_RINTSTS, NUSMART_MMC_READ(host, RINTSTS));  \
	printk(KERN_EMERG "%s: offset %0x DWC_STATUS %x\n",	__func__, DWC_STATUS, NUSMART_MMC_READ(host, STATUS));  \
	printk(KERN_EMERG "%s: offset %0x DWC_FIFOTH %x\n",	__func__, DWC_FIFOTH, NUSMART_MMC_READ(host, FIFOTH));  \
	printk(KERN_EMERG "%s: offset %0x DWC_CDETECT %x\n",	__func__, DWC_CDETECT, NUSMART_MMC_READ(host, CDETECT)); \
/*	printk("%s: offset %0x DWC_WRTPRT %x\n",	__func__, DWC_WRTPRT, NUSMART_MMC_READ(host, WRTPRT)); */\
	printk(KERN_EMERG "%s: offset %0x DWC_GPIO %x\n",	__func__, DWC_GPIO, NUSMART_MMC_READ(host, GPIO)); \
	printk(KERN_EMERG "%s: offset %0x DWC_TCBCNT %x\n",	__func__, DWC_TCBCNT, NUSMART_MMC_READ(host, TCBCNT));  \
	printk(KERN_EMERG "%s: offset %0x DWC_TBBCNT %x\n",	__func__, DWC_TBBCNT, NUSMART_MMC_READ(host, TBBCNT));  \
	printk(KERN_EMERG "%s: offset %0x DWC_DEBNCE %x\n",	__func__, DWC_DEBNCE, NUSMART_MMC_READ(host, DEBNCE));  \
	printk(KERN_EMERG "%s: offset %0x DWC_USRID %x\n",	__func__, DWC_USRID, NUSMART_MMC_READ(host, USRID));  \
	printk(KERN_EMERG "%s: offset %0x DWC_VERID %x\n",	__func__, DWC_VERID, NUSMART_MMC_READ(host, VERID));  \
	printk(KERN_EMERG "%s: offset %0x DWC_HCON %x\n",		__func__, DWC_HCON, NUSMART_MMC_READ(host, HCON)); \
	printk(KERN_EMERG "%s: offset %0x DWC_UHS_REG %x\n",	__func__, DWC_UHS_REG, NUSMART_MMC_READ(host, UHS_REG));   \
	printk(KERN_EMERG "%s: offset %0x DWC_BMOD %x\n",	__func__, DWC_BMOD, NUSMART_MMC_READ(host, BMOD)); \
	printk(KERN_EMERG "%s: offset %0x DWC_PLDMND %x\n",	__func__, DWC_PLDMND, NUSMART_MMC_READ(host, PLDMND));  \
	printk(KERN_EMERG "%s: offset %0x DWC_DBADDR %x\n",	__func__, DWC_DBADDR, NUSMART_MMC_READ(host, DBADDR)); \
	printk(KERN_EMERG "%s: offset %0x DWC_IDSTS %x\n",	__func__, DWC_IDSTS, NUSMART_MMC_READ(host, IDSTS)); \
	printk(KERN_EMERG "%s: offset %0x  DWC_IDINTEN %x\n",	__func__, DWC_IDINTEN, NUSMART_MMC_READ(host, IDINTEN)); \
	printk(KERN_EMERG "%s: offset %0x DWC_DSCADDR %x\n",	__func__, DWC_DSCADDR,NUSMART_MMC_READ(host, DSCADDR)); \
	printk(KERN_EMERG "%s: offset %0x DWC_BUFADDR %x\n",	__func__, DWC_BUFADDR, NUSMART_MMC_READ(host, BUFADDR)); \
}


#endif /* _DW_MMC_H_ */
