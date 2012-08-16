/*
 * MMC definitions for NUSMART
 *
 * Copyright (C) 2010 Nufront Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __NUSMART_MMC_H
#define __NUSMART_MMC_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mmc/host.h>

//#include <mach/board.h>

#define NUSMART_MMC_MAX_SLOTS	3

//sdio card type value is 1 , otherwise is sd/mmc 
#define	SDIO_CARD	1

struct device;
struct mmc_host;

struct ns2816_mmc_platform_data {
	/* back-link to device */
	struct device *dev;
	/* set if your board has components or wiring that limits the
	 * maximum frequency on the MMC bus */
	unsigned int max_freq;
	unsigned int nr_slots;

	struct	nusmart_mmc_slot_data {
		/* 4 wire signaling is optional, and is used for SD/SDIO/HSMMC;
		 * 8 wire signaling is also optional, and is used with HSMMC
		 */
		u8 wires;
		u8 card_type;

		unsigned int ocr_mask;			/* available voltages */
		unsigned long detect_delay;		/* delay in jiffies before detecting cards after interrupt */
		unsigned long device_max_freq;
		const char *name;
	}slots[NUSMART_MMC_MAX_SLOTS];
};

#endif
