#ifndef	__NS115_SDIO_H
#define	__NS115_SDIO_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mmc/host.h>

#define	MAX_SLOTS	4

#define	SD_CARD		1
#define	MMC_CARD	2
#define	EMMC_CARD	4
#define	SDIO_CARD	8

struct device;
struct mmc_host;

struct ns115_mmc_platform_data {
	/* link to device */
	struct device *dev;
	u8		nr_slots;
	unsigned int 	ref_clk;
	unsigned int 	detect_delay_ms;
	unsigned int 	gpio;
	int (*slot_attr_init) (struct evatronix_sdio_slot_data *sd,
					struct mmc_host *mmc, unsigned int id);

	struct evatronix_sdio_slot_data {
		u8	ctype;
		bool	force_rescan;
		u32	caps;
		u32 	pm_caps;
		u32	freq;
		u32	ocr_avail;

		int (*voltage_switch)(void);
	} slots[MAX_SLOTS];
};

#endif
