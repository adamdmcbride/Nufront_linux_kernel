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
#include <linux/blkdev.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/dw_mmc.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>

#include "dw_mmc.h"

/*support IDMAC Only*/
//#define DEBUG

#ifdef DEBUG
int trace_on = 0;
#define CTRACE_ON()	trace_on = 1
#define CTRACE(fmt,args...) if (trace_on) printk(KERN_EMERG"dw_mmc_ct %s "#fmt"\n",__func__,##args)




#define HIS_SIZE 500
u32 cmds[HIS_SIZE+1];
void save_sts(u32 c)
{
	int idx = 0;
	for(idx = HIS_SIZE; idx > 0; idx--) {
		cmds[idx] = cmds[idx-1];
	}
	cmds[0] = c;
}
void dump_sts(void)
{
	int idx = 0;
	for(idx = 0; idx < HIS_SIZE; idx++)
		printk(KERN_EMERG"%d:STS%d",idx,cmds[idx]);
}
#else
int trace_on = 0;
//#define CTRACE(fmt,args...) if (trace_on) printk(KERN_EMERG"dw_mmc_ct %s "#fmt"\n",__func__,##args)
#define CTRACE(fmt,args...)  
#define CTRACE_ON()
#define save_sts(x) 
#define dump_sts() 
#endif

#define DBG_PRINT_(fmt, args...) //printk( KERN_EMERG "" fmt, ## args) 

#define ERR(fmt,args...) printk(KERN_ERR"dw_mmc_ct %s "#fmt"\n",__func__,##args)

/* Common flag combinations */
#define DW_MCI_DATA_ERROR_FLAGS	(SDMMC_INT_DTO | SDMMC_INT_DCRC | \
				 SDMMC_INT_HTO | SDMMC_INT_SBE  | \
				 SDMMC_INT_EBE )
#define DW_MCI_CMD_ERROR_FLAGS	(SDMMC_INT_RTO | SDMMC_INT_RCRC | \
				 SDMMC_INT_RESP_ERR)
#define DW_MCI_ERROR_FLAGS	(DW_MCI_DATA_ERROR_FLAGS | \
				 DW_MCI_CMD_ERROR_FLAGS  | SDMMC_INT_HLE)
#define DW_MCI_SEND_STATUS	1
#define DW_MCI_RECV_STATUS	2
#define DW_MCI_DMA_THRESHOLD	4
#define DW_MCI_SDIO_MASK	0xFF0000
#define IS_SDIO_DIRECT_READ(cmd) ((cmd->opcode == SD_IO_RW_DIRECT)&&\
					(!(cmd->arg & 0x80000000)))
#define IS_SDIO_ABORT(cmd) ((cmd->opcode == SD_IO_RW_DIRECT)&&\
				(((cmd->arg&(0x1ffff<<9))>>9) == SDIO_CCCR_ABORT)&&\
				(cmd->arg & 0x80000000))

struct idmac_desc {
	u32		des0;	/* Control Descriptor */
#define IDMAC_DES0_DIC	BIT(1)
#define IDMAC_DES0_LD	BIT(2)
#define IDMAC_DES0_FD	BIT(3)
#define IDMAC_DES0_CH	BIT(4)
#define IDMAC_DES0_ER	BIT(5)
#define IDMAC_DES0_CES	BIT(30)
#define IDMAC_DES0_OWN	BIT(31)

	u32		des1;	/* Buffer sizes */
#define IDMAC_SET_BUFFER1_SIZE(d, s) \
	((d)->des1 = ((d)->des1 & 0x03ffc000) | ((s) & 0x3fff))

	u32		des2;	/* buffer 1 physical address */

	u32		des3;	/* buffer 2 physical address */
};

/**
 * struct dw_mci_slot - MMC slot state
 * @mmc: The mmc_host representing this slot.
 * @host: The MMC controller this slot is using.
 * @ctype: Card type for this slot.
 * @mrq: mmc_request currently being processed or waiting to be
 *	processed, or NULL when the slot is idle.
 * @queue_node: List node for placing this node in the @queue list of
 *	&struct dw_mci.
 * @clock: Clock rate configured by set_ios(). Protected by host->lock.
 * @flags: Random state bits associated with the slot.
 * @id: Number of this slot.
 * @last_detect_state: Most recently observed card detect state.
 */
struct dw_mci_slot {
	struct mmc_host		*mmc;
	struct dw_mci		*host;

	u32			ctype;

	struct mmc_request	*mrq;
	struct list_head	queue_node;

	unsigned int		clock;
	unsigned int		clock_req;
	unsigned long		flags;
#define DW_MMC_CARD_PRESENT	0
#define DW_MMC_CARD_NEED_INIT	1
	int			id;
	int			last_detect_state;

#ifdef	REQUEST_STAT
	u32			tm_start;
	u32			tm_end;
#endif
};

void dw_mci_logs(void);

#if defined(CONFIG_DEBUG_FS)
static int dw_mci_req_show(struct seq_file *s, void *v)
{
	struct dw_mci_slot *slot = s->private;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_command *stop;
	struct mmc_data	*data;

	/* Make sure we get a consistent snapshot */
	spin_lock_bh(&slot->host->lock);
	mrq = slot->mrq;

	if (mrq) {
		cmd = mrq->cmd;
		data = mrq->data;
		stop = mrq->stop;

		if (cmd)
			seq_printf(s,
				   "CMD%u(0x%x) flg %x rsp %x %x %x %x err %d\n",
				   cmd->opcode, cmd->arg, cmd->flags,
				   cmd->resp[0], cmd->resp[1], cmd->resp[2],
				   cmd->resp[2], cmd->error);
		if (data)
			seq_printf(s, "DATA %u / %u * %u flg %x err %d\n",
				   data->bytes_xfered, data->blocks,
				   data->blksz, data->flags, data->error);
		if (stop)
			seq_printf(s,
				   "CMD%u(0x%x) flg %x rsp %x %x %x %x err %d\n",
				   stop->opcode, stop->arg, stop->flags,
				   stop->resp[0], stop->resp[1], stop->resp[2],
				   stop->resp[2], stop->error);
	}

	spin_unlock_bh(&slot->host->lock);

	return 0;
}

static int dw_mci_req_open(struct inode *inode, struct file *file)
{
	return single_open(file, dw_mci_req_show, inode->i_private);
}

static const struct file_operations dw_mci_req_fops = {
	.owner		= THIS_MODULE,
	.open		= dw_mci_req_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dw_mci_regs_show(struct seq_file *s, void *v)
{
	seq_printf(s, "STATUS:\t0x%08x\n", SDMMC_STATUS);
	seq_printf(s, "RINTSTS:\t0x%08x\n", SDMMC_RINTSTS);
	seq_printf(s, "CMD:\t0x%08x\n", SDMMC_CMD);
	seq_printf(s, "CTRL:\t0x%08x\n", SDMMC_CTRL);
	seq_printf(s, "INTMASK:\t0x%08x\n", SDMMC_INTMASK);
	seq_printf(s, "CLKENA:\t0x%08x\n", SDMMC_CLKENA);

	return 0;
}

static int dw_mci_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dw_mci_regs_show, inode->i_private);
}

static const struct file_operations dw_mci_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= dw_mci_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void dw_mci_init_debugfs(struct dw_mci_slot *slot)
{
	struct mmc_host	*mmc = slot->mmc;
	struct dw_mci *host = slot->host;
	struct dentry *root;
	struct dentry *node;

	root = mmc->debugfs_root;
	if (!root)
		return;

	node = debugfs_create_file("regs", S_IRUSR, root, host,
				   &dw_mci_regs_fops);
	if (!node)
		goto err;

	node = debugfs_create_file("req", S_IRUSR, root, slot,
				   &dw_mci_req_fops);
	if (!node)
		goto err;

	node = debugfs_create_u32("state", S_IRUSR, root, (u32 *)&host->state);
	if (!node)
		goto err;

	node = debugfs_create_x32("pending_events", S_IRUSR, root,
				  (u32 *)&host->pending_events);
	if (!node)
		goto err;

	node = debugfs_create_x32("completed_events", S_IRUSR, root,
				  (u32 *)&host->completed_events);
	if (!node)
		goto err;

	return;

err:
	dev_err(&mmc->class_dev, "failed to initialize debugfs for slot\n");
}
#endif /* defined(CONFIG_DEBUG_FS) */

static void dw_mci_set_timeout(struct dw_mci *host)
{
	CTRACE(); 
	/* timeout (maximum) */
	mci_writel(host, TMOUT, 0xffffff40);
}

static u32 dw_mci_prepare_command(struct mmc_host *mmc, struct mmc_command *cmd)
{
	struct mmc_data	*data;
	struct dw_mci_slot *slot = mmc_priv(mmc);
	u32 cmdr;
	CTRACE("slot:%d, cmd->opcode %d,cmd->retries %d",slot->id ,cmd->opcode, cmd->retries); 
	cmd->error = -EINPROGRESS;

	cmdr = cmd->opcode;

	if ((cmdr == MMC_STOP_TRANSMISSION)||IS_SDIO_ABORT(cmd))
		cmdr |= SDMMC_CMD_STOP;
	else
		cmdr |= SDMMC_CMD_PRV_DAT_WAIT;

	cmdr |= ((slot->id & 0xf) <<16);

	if(IS_SDIO_ABORT(cmd))
		printk(KERN_EMERG"abort cmdarg:%#x\n",cmd->arg);

	if (cmd->flags & MMC_RSP_PRESENT) {
		/* We expect a response, so set this bit */
		cmdr |= SDMMC_CMD_RESP_EXP;
		if (cmd->flags & MMC_RSP_136)
			cmdr |= SDMMC_CMD_RESP_LONG;
	}

	if (cmd->flags & MMC_RSP_CRC)
		cmdr |= SDMMC_CMD_RESP_CRC;

	data = cmd->data;
	if (data) {
		cmdr |= SDMMC_CMD_DAT_EXP;
		if (data->flags & MMC_DATA_STREAM)
			cmdr |= SDMMC_CMD_STRM_MODE;
		if (data->flags & MMC_DATA_WRITE)
			cmdr |= SDMMC_CMD_DAT_WR;
	}

	return cmdr;
}

static bool mci_wait_reset(struct device *dev, struct dw_mci *host);
static void dw_mci_save(struct dw_mci *host);
static void dw_mci_restore(struct dw_mci *host);

static void dw_mci_start_command(struct dw_mci *host,
				 struct mmc_command *cmd, u32 cmd_flags)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(5000);
	unsigned int cmd_status = 0;

	CTRACE(); 
	host->cmd = cmd;

	mci_writel(host, CMDARG, cmd->arg);
	smp_rmb();
	wmb();
	smp_wmb();	
	mci_writel(host, CMD, cmd_flags | SDMMC_CMD_START);
	wmb();
	smp_wmb();
	smp_rmb();

	while (time_before(jiffies, timeout)) {
		smp_rmb();
		cmd_status = mci_readl(host, CMD);
		smp_rmb();
		if (!(cmd_status & SDMMC_CMD_START))
			return;
	}
	print_reg(host);
#ifdef DEBUG
	print_reg(host);
#endif	
//	BUG();
	dw_mci_save(host);
	mci_wait_reset(NULL, host);

	/* Restore the old value at FIFOTH register */
	mci_writel(host, FIFOTH, host->fifoth_val);

	ERR("RINTSTS 0x%x\n", mci_readl(host, RINTSTS));

	mci_writel(host, RINTSTS, 0xFFFFFFFE);
	mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE | SDMMC_INT_DATA_OVER |
		   SDMMC_INT_TXDR | SDMMC_INT_RXDR |
		   DW_MCI_ERROR_FLAGS | SDMMC_INT_CD);

	dw_mci_restore(host);

	mci_writel(host, CTRL, SDMMC_CTRL_INT_ENABLE);
	mci_writel(host, RINTSTS, 0xFFFFFFFE);
	/* Restore over */

	/* reload the cmd */
	mci_writel(host, CMDARG, cmd->arg);
	smp_rmb();
	wmb();
	smp_wmb();	
	mci_writel(host, CMD, cmd_flags | SDMMC_CMD_START);
	wmb();
	smp_wmb();
	smp_rmb();

	cmd_status = mci_readl(host, CMD);
	smp_rmb();
	smp_wmb();
	cmd_status = mci_readl(host, CMD);
	smp_rmb();
	smp_wmb();
	if (!(cmd_status & SDMMC_CMD_START))
		return;
#ifdef DEBUG
	print_reg(host);
#endif	

	BUG();
}

static void send_stop_cmd(struct dw_mci *host, struct mmc_data *data)
{
	CTRACE(); 
	if((host->stop_cmdr&0x1f) != 12)
		printk(KERN_EMERG"dw_mmc stop cmd %x\n",host->stop_cmdr);
	dw_mci_start_command(host, data->stop, host->stop_cmdr);
}

/* DMA interface functions */
static void dw_mci_stop_dma(struct dw_mci *host)
{
	CTRACE(); 
	if (host->use_dma) {
		host->dma_ops->stop(host);
		host->dma_ops->cleanup(host);
	} else {
		BUG();
		/* Data transfer was stopped by the interrupt handler */
		set_bit(EVENT_XFER_COMPLETE, &host->pending_events);
	}
}

static void dw_mci_dma_cleanup(struct dw_mci *host)
{
	struct mmc_data *data = host->data;
	CTRACE(); 

	if (data)
		dma_unmap_sg(&host->pdev->dev, data->sg, data->sg_len,
			     ((data->flags & MMC_DATA_WRITE)
			      ? DMA_TO_DEVICE : DMA_FROM_DEVICE));
}

static void dw_mci_idmac_stop_dma(struct dw_mci *host)
{
	u32 temp;
	CTRACE(); 

	/* Disable and reset the IDMAC interface */
	temp = mci_readl(host, CTRL);
	temp &= ~SDMMC_CTRL_USE_IDMAC;
	temp |= SDMMC_CTRL_DMA_RESET;
	//temp |= SDMMC_CTRL_FIFO_RESET;	/*reset fifo*/
	mci_writel(host, CTRL, temp);

	/* Stop the IDMAC running */
	temp = mci_readl(host, BMOD);
	temp &= ~(SDMMC_IDMAC_ENABLE | SDMMC_IDMAC_FB);
	mci_writel(host, BMOD, temp);
}

static void dw_mci_idmac_complete_dma(struct dw_mci *host)
{
	struct mmc_data *data = host->data;

	CTRACE(); 
	dev_vdbg(&host->pdev->dev, "DMA complete\n");

	host->dma_ops->cleanup(host);

	/*
	 * If the card was removed, data will be NULL. No point in trying to
	 * send the stop command or waiting for NBUSY in this case.
	 */
	if (data) {
		set_bit(EVENT_XFER_COMPLETE, &host->pending_events);
		tasklet_schedule(&host->tasklet);
	}
}

static void dw_mci_translate_sglist(struct dw_mci *host, struct mmc_data *data,
				    unsigned int sg_len)
{
	int i;
	struct idmac_desc *desc = host->sg_cpu;
	CTRACE(); 

	for (i = 0; i < sg_len; i++, desc++) {
		unsigned int length = sg_dma_len(&data->sg[i]);
		u32 mem_addr = sg_dma_address(&data->sg[i]);

		/* Set the OWN bit and disable interrupts for this descriptor */
		desc->des0 = IDMAC_DES0_OWN | IDMAC_DES0_DIC | IDMAC_DES0_CH;

		/* Buffer length */
		IDMAC_SET_BUFFER1_SIZE(desc, length);

		/* Physical address to DMA to/from */
		desc->des2 = mem_addr;
	}

	/* Set first descriptor */
	desc = host->sg_cpu;
	desc->des0 |= IDMAC_DES0_FD;

	/* Set last descriptor */
	desc = host->sg_cpu + (i - 1) * sizeof(struct idmac_desc);
	desc->des0 &= ~(IDMAC_DES0_CH | IDMAC_DES0_DIC);
	desc->des0 |= IDMAC_DES0_LD;

	wmb();
}

static void dw_mci_idmac_start_dma(struct dw_mci *host, unsigned int sg_len)
{
	u32 temp;
	CTRACE(); 

	dw_mci_translate_sglist(host, host->data, sg_len);

	/* Select IDMAC interface */
	temp = mci_readl(host, CTRL);
	temp |= SDMMC_CTRL_USE_IDMAC;
	mci_writel(host, CTRL, temp);

	wmb();

	/* Enable the IDMAC */
	temp = mci_readl(host, BMOD);
	temp |= SDMMC_IDMAC_ENABLE | SDMMC_IDMAC_FB;
	mci_writel(host, BMOD, temp);

	/* Start it running */
	mci_writel(host, PLDMND, 1);
}

static int dw_mci_idmac_init(struct dw_mci *host)
{
	struct idmac_desc *p;
	int i;

	CTRACE(); 
	/* Number of descriptors in the ring buffer */
	host->ring_size = PAGE_SIZE / sizeof(struct idmac_desc);

	/* Forward link the descriptor list */
	for (i = 0, p = host->sg_cpu; i < host->ring_size - 1; i++, p++)
		p->des3 = host->sg_dma + (sizeof(struct idmac_desc) * (i + 1));

	/* Set the last descriptor as the end-of-ring descriptor */
	p->des3 = host->sg_dma;
	p->des0 = IDMAC_DES0_ER;

	/* Mask out interrupts - get Tx & Rx complete only */
	mci_writel(host, IDINTEN, SDMMC_IDMAC_INT_NI | SDMMC_IDMAC_INT_RI |
		   SDMMC_IDMAC_INT_TI);

	/* Set the descriptor base address */
	mci_writel(host, DBADDR, host->sg_dma);
	return 0;
}

static struct dw_mci_dma_ops dw_mci_idmac_ops = {
	.init = dw_mci_idmac_init,
	.start = dw_mci_idmac_start_dma,
	.stop = dw_mci_idmac_stop_dma,
	.complete = dw_mci_idmac_complete_dma,
	.cleanup = dw_mci_dma_cleanup,
};

static int dw_mci_submit_data_dma(struct dw_mci *host, struct mmc_data *data)
{
	struct scatterlist *sg;
	unsigned int i, direction, sg_len;
	unsigned long flags;
	u32 temp;
	CTRACE(); 

	/* If we don't have a channel, we can't do DMA */
	if (!host->use_dma)
		return -ENODEV;

	/*
	 * We don't do DMA on "complex" transfers, i.e. with
	 * non-word-aligned buffers or lengths. Also, we don't bother
	 * with all the DMA setup overhead for short transfers.
	 */
	if (data->blocks * data->blksz < DW_MCI_DMA_THRESHOLD)
		return -EINVAL;
	if (data->blksz & 3)
		return -EINVAL;

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3 || sg->length & 3)
			return -EINVAL;
	}

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	sg_len = dma_map_sg(&host->pdev->dev, data->sg, data->sg_len,
			    direction);

	dev_vdbg(&host->pdev->dev,
		 "sd sg_cpu: %#lx sg_dma: %#lx sg_len: %d\n",
		 (unsigned long)host->sg_cpu, (unsigned long)host->sg_dma,
		 sg_len);

	host->dma_ops->start(host, sg_len);

	return 0;
}

static void dw_mci_submit_data(struct dw_mci *host, struct mmc_data *data)
{
	u32 temp;

	CTRACE(); 
	data->error = -EINPROGRESS;

	WARN_ON(host->data);
	host->sg = NULL;
	host->data = data;

	if (dw_mci_submit_data_dma(host, data)) {
		BUG();
		host->sg = data->sg;
		host->pio_offset = 0;
		if (data->flags & MMC_DATA_READ)
			host->dir_status = DW_MCI_RECV_STATUS;
		else
			host->dir_status = DW_MCI_SEND_STATUS;

	}
}

static void mci_send_cmd(struct dw_mci_slot *slot, u32 cmd, u32 arg)
{
	struct dw_mci *host = slot->host;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned int cmd_status = 0;

	CTRACE("slot:%d",slot->id); 
	mci_writel(host, CMDARG, arg);
	wmb();
	mci_writel(host, CMD, SDMMC_CMD_START | cmd);

	while (time_before(jiffies, timeout)) {
		cmd_status = mci_readl(host, CMD);
		if (!(cmd_status & SDMMC_CMD_START))
			return;
	}
	dev_err(&slot->mmc->class_dev,
		"Timeout sending command (cmd %#x arg %#x status %#x)\n",
		cmd, arg, cmd_status);
}

static void dw_mci_setup_bus(struct dw_mci_slot *slot)
{
	struct dw_mci *host = slot->host;
	u32 div;
	u32 divreg;
	u32 ctype;
	u32 clkena;
	u32 clksrc;

	CTRACE("slot:%d,slot clk:%d,host speed %d",slot->id, slot->clock_req, slot->clock); 
	if (slot->clock != slot->clock_req) {
		if (host->bus_hz % slot->clock_req)
			/*
			 * move the + 1 after the divide to prevent
			 * over-clocking the card.
			 */
			div = ((host->bus_hz / slot->clock_req) >> 1) + 1;
		else
			div = (host->bus_hz  / slot->clock_req) >> 1;

		dev_info(&slot->mmc->class_dev,
			 "Bus speed (slot %d) = %dHz (slot req %dHz, actual %dHZ"
			 " div = %d)\n", slot->id, host->bus_hz, slot->clock_req,
			 div ? ((host->bus_hz / div) >> 1) : host->bus_hz, div);

		/* save old clock setting*/
		clkena = mci_readl(host, CLKENA);
		clksrc = mci_readl(host, CLKSRC);
		divreg = mci_readl(host, CLKDIV);
		/* disable clock */
		mci_writel(host, CLKENA, 0);
		mci_writel(host, CLKSRC, 0);

		/* inform CIU */
		mci_send_cmd(slot,
			     SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

		mci_writel(host, CLKSRC, (1<<2)|(2<<4));

		/* set clock to desired speed */
		
		mci_writel(host, CLKDIV, (divreg & (~(0xff<<(slot->id<<3))))|(div<<(slot->id<<3)));
		//mci_writel(host, CLKDIV, (div<<(slot->id<<3)));
		printk(KERN_EMERG"CLKDIV %#x--------------------\n",mci_readl(host, CLKDIV));
		/* inform CIU */
		mci_send_cmd(slot,
			     SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

		/* enable clock */
		mci_writel(host, CLKENA, clkena | SDMMC_CLKEN_ENABLE(slot->id));

		/* inform CIU */
		mci_send_cmd(slot,
			     SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

		slot->clock = slot->clock_req;
	}

	/* Set the current slot bus width */
	ctype = mci_readl(host, CTYPE);
	ctype &= ~( 0x10001 << slot->id);
	mci_writel(host, CTYPE, ctype | ( slot->ctype << slot->id));
}

static void dw_mci_start_request(struct dw_mci *host,
				 struct dw_mci_slot *slot)
{
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data	*data;
	u32 cmdflags;

	CTRACE("slot:%d",slot->id); 
	mrq = slot->mrq;
	if (host->pdata->select_slot)
		host->pdata->select_slot(slot->id);

	/* Slot specific timing and width adjustment */
	dw_mci_setup_bus(slot);

	host->cur_slot = slot;
	host->mrq = mrq;

	host->pending_events = 0;
	host->completed_events = 0;
	host->data_status = 0;

	if(host->cur_slot->id == 1) {
		DBG_PRINT_("CMD%d, write = %d\n", mrq->cmd->opcode, (mrq->cmd->arg & 0x80000000) ? 1:0);
	}

	data = mrq->data;
	if (data) {
		dw_mci_set_timeout(host);
		mci_writel(host, BYTCNT, data->blksz*data->blocks);
		mci_writel(host, BLKSIZ, data->blksz);
	} else {
		mci_writel(host, BYTCNT, 0);
		mci_writel(host, BLKSIZ, 0);
	}

	cmd = mrq->cmd;
	cmdflags = dw_mci_prepare_command(slot->mmc, cmd);

	/* this is the first command, send the initialization clock */
	if (test_and_clear_bit(DW_MMC_CARD_NEED_INIT, &slot->flags))
		cmdflags |= SDMMC_CMD_INIT;

	if (data) {
		dw_mci_submit_data(host, data);
		wmb();
	}

	dw_mci_start_command(host, cmd, cmdflags);

	if (mrq->stop)
		host->stop_cmdr = dw_mci_prepare_command(slot->mmc, mrq->stop);
}

static void dw_mci_queue_request(struct dw_mci *host, struct dw_mci_slot *slot,
				 struct mmc_request *mrq)
{
	unsigned long flags;

	dev_vdbg(&slot->mmc->class_dev, "queue request: state=%d\n",
		 host->state);

	CTRACE("slot:%d,mrq retries %d",slot->id,mrq->cmd->retries); 
	host->tasklet_process = 5; 
	spin_lock_bh(&host->lock);
	host->tasklet_process = 6; 
	CTRACE("slot:%d lock",slot->id); 
	slot->mrq = mrq;
#ifdef	REQUEST_STAT
	slot->tm_start = jiffies; 
#endif
	
	if (host->state == STATE_IDLE) {
		host->tasklet_process = 8; 
		host->state = STATE_SENDING_CMD;
		dw_mci_start_request(host, slot);
	} else {
		list_add_tail(&slot->queue_node, &host->queue);
	}

	CTRACE("slot:%d unlock",slot->id); 
	spin_unlock_bh(&host->lock);
}

static void dw_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;

	CTRACE("slot:%d",slot->id); 
	WARN_ON(slot->mrq);

	if (!test_bit(DW_MMC_CARD_PRESENT, &slot->flags)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	/* We don't support multiple blocks of weird lengths. */
	dw_mci_queue_request(host, slot, mrq);
}

static void dw_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	CTRACE("slot:%d",slot->id); 

	/* set default 1 bit mode */
	slot->ctype = SDMMC_CTYPE_1BIT;

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		slot->ctype = SDMMC_CTYPE_1BIT;
		break;
	case MMC_BUS_WIDTH_4:
		slot->ctype = SDMMC_CTYPE_4BIT;
		break;
	case MMC_BUS_WIDTH_8:
		slot->ctype = SDMMC_CTYPE_8BIT;
		break;
	}

	if (ios->clock) {
		/*
		 * Use mirror of ios->clock to prevent race with mmc
		 * core ios update when finding the minimum.
		 */
		slot->clock_req = ios->clock;
	}

	switch (ios->power_mode) {
	case MMC_POWER_UP:
		set_bit(DW_MMC_CARD_NEED_INIT, &slot->flags);
		break;
	default:
		break;
	}
}

static int dw_mci_get_ro(struct mmc_host *mmc)
{
	int read_only;
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci_board *brd = slot->host->pdata;

	CTRACE("slot:%d",slot->id); 
	/* Use platform get_ro function, else try on board write protect */
	if (brd->get_ro)
		read_only = brd->get_ro(slot->id);
	else
		read_only =
			mci_readl(slot->host, WRTPRT) & (1 << slot->id) ? 1 : 0;

	dev_dbg(&mmc->class_dev, "card is %s\n",
		read_only ? "read-only" : "read-write");

	return read_only;
}

static int dw_mci_get_cd(struct mmc_host *mmc)
{
	int present;
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci_board *brd = slot->host->pdata;

	CTRACE("slot:%d",slot->id); 
	ERR("slot:%d, CDETECT 0x%x",slot->id, mci_readl(slot->host, CDETECT)); 
	/* Use platform get_cd function, else try onboard card detect */
	if (brd->quirks & DW_MCI_QUIRK_BROKEN_CARD_DETECTION)
		present = 1;
	else if (brd->get_cd)
		present = !brd->get_cd(slot->id);
	else
		present = (mci_readl(slot->host, CDETECT) & (1 << slot->id))
			== 0 ? 1 : 0;

	if (present) {
		ERR("card is present\n");
		dev_dbg(&mmc->class_dev, "card is present\n");
	}
	else {
		ERR( "card is not present\n");
		dev_dbg(&mmc->class_dev, "card is not present\n");
	}
	return present;
}

static void dw_mci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct dw_mci_slot *slot = mmc_priv(mmc);
	struct dw_mci *host = slot->host;
	u32 intmask, tmp,flags;
	u32 sid = slot->id;
	CTRACE("slot:%d",slot->id); 

	spin_lock_irqsave(&host->irq_lock, flags);
	intmask = mci_readl(host, INTMASK);
	
	tmp = intmask;

	if(enable)
	{
		intmask |= (1 << (sid + 16));
		mci_writel(host, INTMASK, intmask);
	}else{
		intmask &= ~(1 << (sid + 16));
		mci_writel(host, INTMASK, intmask);
	}

	spin_unlock_irqrestore(&host->irq_lock, flags);

	DBG_PRINT_("%s :%d, intmask = 0x%x, set_clr intmask = 0x%x\n", __func__, enable, tmp, mci_readl(host, INTMASK));
	smp_wmb();
}


static const struct mmc_host_ops dw_mci_ops = {
	.request	= dw_mci_request,
	.set_ios	= dw_mci_set_ios,
	.get_ro		= dw_mci_get_ro,
	.get_cd		= dw_mci_get_cd,
	.enable_sdio_irq = dw_mci_enable_sdio_irq,
};

static void dw_mci_request_end(struct dw_mci *host, struct mmc_request *mrq)
//	__releases(&host->lock)
//	__acquires(&host->lock)
{
	struct dw_mci_slot *slot;
	struct mmc_host	*prev_mmc = host->cur_slot->mmc;
	if(mrq->cmd->error || (mrq->data && mrq->data->error)) {
		printk(KERN_EMERG "%s : call dw_mci_logs error %d\n", __func__, mrq->cmd->error);
		dw_mci_logs();
		if(mrq->data && (mrq->cmd->opcode & (MMC_READ_SINGLE_BLOCK | MMC_READ_MULTIPLE_BLOCK)))  
		{
			mrq->data->bytes_xfered = mci_readl(host, TCBCNT);
			printk(KERN_EMERG "%s : date byte read %d\n", __func__, mrq->data->bytes_xfered);
		}
	}

#ifdef	REQUEST_STAT
	host->cur_slot->tm_end = (jiffies - host->cur_slot->tm_start) * 1000 / HZ;

	if(host->cur_slot->tm_end > 20)
		printk("req process time %d ms\n", host->cur_slot->tm_end);	
#endif				
	if(host->cur_slot->id == 1) {
		DBG_PRINT_("End CMD%d, INTMASK 0x%x\n", mrq->cmd->opcode, mci_readl(host, INTMASK));
	}

	CTRACE("cur slot:%d",host->cur_slot->id); 
	WARN_ON(host->cmd || host->data);
	host->cur_slot->mrq = NULL;
#ifdef	REQUEST_STAT
	host->cur_slot->tm_start = 0;
	host->cur_slot->tm_end = 0;
#endif
	host->mrq = NULL;
	host->pending_events = 0;
	host->rawstat = 0;
	host->irqstat = 0;
	host->idsts = 0;
	host->cmd_status = 0;

	CTRACE("cur slot:%d unlock",host->cur_slot->id); 
	spin_unlock(&host->lock);
	mmc_request_done(prev_mmc, mrq);
	spin_lock(&host->lock);
	CTRACE("cur slot:%d lock",host->cur_slot->id); 
	host->cur_slot = NULL;

	if (!list_empty(&host->queue)) {
		slot = list_entry(host->queue.next,
				struct dw_mci_slot, queue_node);
		list_del(&slot->queue_node);
		dev_vdbg(&host->pdev->dev, "list not empty: %s is next\n",
				mmc_hostname(slot->mmc));
		host->state = STATE_SENDING_CMD;
		dw_mci_start_request(host, slot);
	} else {
		dev_vdbg(&host->pdev->dev, "list empty\n");
		host->state = STATE_IDLE;
	}
}

static void dw_mci_command_complete(struct dw_mci *host, struct mmc_command *cmd)
{
	u32 status = host->cmd_status;

	CTRACE(); 
	host->cmd_status = 0;

	/* Read the response from the card (up to 16 bytes) */
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[3] = mci_readl(host, RESP0);
			cmd->resp[2] = mci_readl(host, RESP1);
			cmd->resp[1] = mci_readl(host, RESP2);
			cmd->resp[0] = mci_readl(host, RESP3);
		} else {
			cmd->resp[0] = mci_readl(host, RESP0);
			cmd->resp[1] = 0;
			cmd->resp[2] = 0;
			cmd->resp[3] = 0;
		}
	}

	if (status & SDMMC_INT_RTO)
		cmd->error = -ETIMEDOUT;
	else if ((cmd->flags & MMC_RSP_CRC) && (status & SDMMC_INT_RCRC))
		cmd->error = -EILSEQ;
	else if (status & SDMMC_INT_RESP_ERR)
		cmd->error = -EIO;
	else
		cmd->error = 0;

	if (cmd->error) {
		/* newer ip versions need a delay between retries */
		if (host->quirks & DW_MCI_QUIRK_RETRY_DELAY)
			mdelay(20);

		if (cmd->data) {
			host->data = NULL;
			dw_mci_stop_dma(host);
		}
	}
}

static bool mci_wait_reset_fifo(struct dw_mci *host)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned int ctrl;

	CTRACE(); 
	ctrl = mci_readl(host, CTRL);
	mci_writel(host, CTRL, (ctrl | SDMMC_CTRL_FIFO_RESET));

	/* wait till resets clear */
	do {
		ctrl = mci_readl(host, CTRL);
		if (!(ctrl & SDMMC_CTRL_FIFO_RESET))
			return true;
	} while (time_before(jiffies, timeout));

	return false;
}

static void dw_mci_tasklet_func(unsigned long priv)
{
	struct dw_mci *host = (struct dw_mci *)priv;
	struct mmc_data	*data;
	struct mmc_command *cmd;
	enum dw_mci_state state;
	enum dw_mci_state prev_state;
	u32 status, ctrl;

	CTRACE("state:%d",host->state);
	host->tasklet_process = 1; 
	spin_lock(&host->lock);
	host->tasklet_process = 2; 
	CTRACE("state:%d lock",host->state); 

	state = host->state;
	data = host->data;

	do {
		prev_state = state;

		switch (state) {
		case STATE_IDLE:
			CTRACE("state IDLE"); 
			break;

		case STATE_SENDING_CMD:
			CTRACE("state SENDING CMD"); 
			if (!test_and_clear_bit(EVENT_CMD_COMPLETE,
						&host->pending_events))
				break;
		
			cmd = host->cmd;
			host->cmd = NULL;
			set_bit(EVENT_CMD_COMPLETE, &host->completed_events);
			dw_mci_command_complete(host, host->mrq->cmd);
			if (!host->mrq->data || cmd->error) {
				dw_mci_request_end(host, host->mrq);
				goto unlock;
			}

			prev_state = state = STATE_SENDING_DATA;
			/* fall through */

		case STATE_SENDING_DATA:
			CTRACE("state SENDING DATA"); 
			if (test_and_clear_bit(EVENT_DATA_ERROR,
					       &host->pending_events)) {
				dw_mci_stop_dma(host);
				if (data->stop)
				{
					send_stop_cmd(host, data);
				}
				state = STATE_DATA_ERROR;
				break;
			}

			if (!test_and_clear_bit(EVENT_XFER_COMPLETE,
						&host->pending_events))
				break;

			set_bit(EVENT_XFER_COMPLETE, &host->completed_events);
			prev_state = state = STATE_DATA_BUSY;
			/* fall through */

		case STATE_DATA_BUSY:
			CTRACE("state DATA BUSY"); 
			if (!test_and_clear_bit(EVENT_DATA_COMPLETE,
						&host->pending_events))
				break;
			//clear_bit(EVENT_DATA_ERROR, &host->pending_events);
			host->data = NULL;
			set_bit(EVENT_DATA_COMPLETE, &host->completed_events);
			status = host->data_status;

			if (status & DW_MCI_DATA_ERROR_FLAGS) {
				if (status & SDMMC_INT_DTO) {
					dev_err(&host->pdev->dev,
						"data timeout error\n");
					data->error = -ETIMEDOUT;
				} else if (status & SDMMC_INT_DCRC) {
					dev_err(&host->pdev->dev,
						"data CRC error\n");
					data->error = -EILSEQ;
				} else {
					dev_err(&host->pdev->dev,
						"data FIFO error "
						"(status=%08x), blocks = %d, blksz = %d, data->stop = 0x%x\n",
						status, data->blocks, data->blksz, data->stop);
					data->error = -EIO;
				}
				/*
				 * After an error, there may be data lingering
				 * in the FIFO, so reset it - doing so
				 * generates a block interrupt, hence setting
				 * the scatter-gather pointer to NULL.
				 */
				dw_mci_logs();
				host->sg = NULL;
				mci_wait_reset_fifo(host);
			} else {
				data->bytes_xfered = data->blocks * data->blksz;
				data->error = 0;
			}

			if (!data->stop) {
				dw_mci_request_end(host, host->mrq);
				goto unlock;
			}

			prev_state = state = STATE_SENDING_STOP;
			if (!data->error)
				send_stop_cmd(host, data);
			/* fall through */

		case STATE_SENDING_STOP:
			CTRACE("state SENDING STOP"); 
			if (!test_and_clear_bit(EVENT_CMD_COMPLETE,
						&host->pending_events))
				break;

			host->cmd = NULL;
			dw_mci_command_complete(host, host->mrq->stop);
			dw_mci_request_end(host, host->mrq);
			goto unlock;

		case STATE_DATA_ERROR:

			state = STATE_DATA_BUSY;
			break;
		}
	} while (state != prev_state);

	host->state = state;
unlock:
	CTRACE("state:%d unlock",host->state); 
	spin_unlock(&host->lock);

}

static void dw_mci_cmd_interrupt(struct dw_mci *host, u32 status)
{
	CTRACE("status:0x%x",status); 
	if (!host->cmd_status)
		host->cmd_status = status;

	smp_wmb();

	set_bit(EVENT_CMD_COMPLETE, &host->pending_events);
	tasklet_schedule(&host->tasklet);
}

static irqreturn_t dw_mci_interrupt(int irq, void *dev_id)
{
	struct dw_mci *host = dev_id;
	u32 status, pending;
	unsigned int pass_count = 0;
	int idx = 0;	
#ifdef DEBUG	
	u32 dsts = 0, dpend = 0, ddsts = 0;	
#endif
	unsigned int mci_status;
	do {
		status = mci_readl(host, RINTSTS);
		pending = mci_readl(host, MINTSTS); /* read-only mask reg */
		mci_writel(host, RINTSTS, status);
		CTRACE("irq status %x, pending %x",status,pending); 
#ifdef DEBUG 
		dsts |= status;
		dpend |= pending;
#endif
		host->rawstat |= status;
		host->irqstat |= pending;

		/*
		 * DTO fix - version 2.10a and below, and only if internal DMA
		 * is configured.
		 */
		if (host->quirks & DW_MCI_QUIRK_IDMAC_DTO) {
			if (!pending &&
			    ((mci_readl(host, STATUS) >> 17) & 0x1fff))
				pending |= SDMMC_INT_DATA_OVER;
		}

		if (!pending)
			break;

		if (pending & DW_MCI_SDIO_MASK) {
			for(idx = 0; (idx < host->num_slots)&&(pending & DW_MCI_SDIO_MASK); idx++) {
				if(pending & (1 << (idx+16))) {
					BUG_ON(host->slot[idx]->mmc == NULL);
										
					DBG_PRINT_("sdio interrupt ..., and disable mmc sdio interrrupt\n");
					mmc_signal_sdio_irq(host->slot[idx]->mmc);
					mci_writel(host, RINTSTS, pending & (1<<(idx+16)));
					pending &= ~(1<<(idx+16));
				}
			}
		}
		if (pending & DW_MCI_CMD_ERROR_FLAGS) {
		//	printk(KERN_EMERG "%s ....cmd_status 0x%x cmd error pending 0x%x\n", __func__, status, pending);
		
		//	mci_writel(host, RINTSTS, DW_MCI_CMD_ERROR_FLAGS);
			host->cmd_status = status;
			smp_wmb();
			//set_bit(EVENT_CMD_COMPLETE, &host->pending_events);
			//tasklet_schedule(&host->tasklet);
		}

		if (pending & DW_MCI_DATA_ERROR_FLAGS) {
		//	printk(KERN_EMERG "%s ....data_status 0x%x data error pending 0x%x\n", __func__, status, pending);
		//	mci_writel(host, RINTSTS, DW_MCI_DATA_ERROR_FLAGS);

			host->data_status = status;
			smp_wmb();
			set_bit(EVENT_DATA_ERROR, &host->pending_events);
			if(pending & (SDMMC_INT_SBE | SDMMC_INT_HTO))
				set_bit(EVENT_DATA_COMPLETE, &host->pending_events);
			tasklet_schedule(&host->tasklet);
		}

		if (pending & SDMMC_INT_DATA_OVER) {
		//	mci_writel(host, RINTSTS, SDMMC_INT_DATA_OVER);
			if (!host->data_status)
				host->data_status = status;
			mci_status = mci_readl(host, STATUS);
			if(mci_status & 0x3ffe0000) {
				ERR("irq check status mci_status %x, host state %x", mci_status, host->state);
			}
			smp_wmb();
			set_bit(EVENT_DATA_COMPLETE, &host->pending_events);
			tasklet_schedule(&host->tasklet);
		}
		/*clean fifo interrupt status*/

		if (pending & SDMMC_INT_CMD_DONE) {
	//		mci_writel(host, RINTSTS, SDMMC_INT_CMD_DONE);
			dw_mci_cmd_interrupt(host, status);
		}

		if (pending & SDMMC_INT_CD) {
	//		mci_writel(host, RINTSTS, SDMMC_INT_CD);
			tasklet_schedule(&host->card_tasklet);
		}




	} while (pass_count++ < 5);

	/* Handle DMA interrupts */
	pending = mci_readl(host, IDSTS);
	host->idsts |= pending;

#ifdef DEBUG
	ddsts = pending;
#endif
	if (pending & (SDMMC_IDMAC_INT_TI | SDMMC_IDMAC_INT_RI)) {
		mci_writel(host, IDSTS, SDMMC_IDMAC_INT_TI | SDMMC_IDMAC_INT_RI);
		mci_writel(host, IDSTS, SDMMC_IDMAC_INT_NI);
		/*should not set for multi slot existed.*/
		//set_bit(EVENT_DATA_COMPLETE, &host->pending_events);
		host->dma_ops->complete(host);
	}
	CTRACE("irq dma pending %x",pending);
#ifdef DEBUG
	pending = mci_readl(host, MINTSTS); /* read-only mask reg */
	if(pending)
		printk(KERN_EMERG"irq pending ... mintsts %x, handled raw %x, pending %x\n",pending, dsts, dpend);
	pending = mci_readl(host, IDSTS);
	if(pending&0x103)
		printk(KERN_EMERG"irq pending ... idmasts %x, handled %x\n",pending, ddsts);
#endif
	return IRQ_HANDLED;
}

static void dw_mci_tasklet_card(unsigned long data)
{
	struct dw_mci *host = (struct dw_mci *)data;
	int i;

	CTRACE(); 
	for (i = 0; i < host->num_slots; i++) {
		struct dw_mci_slot *slot = host->slot[i];
		struct mmc_host *mmc = slot->mmc;
		struct mmc_request *mrq;
		int present;
		u32 ctrl, pwr;

		present = dw_mci_get_cd(mmc);
		while (present != slot->last_detect_state) {
			host->tasklet_process = 3; 
			spin_lock(&host->lock);
			host->tasklet_process = 4; 
			CTRACE("lock"); 

			dev_dbg(&slot->mmc->class_dev, "card %s\n",
				present ? "inserted" : "removed");

			/*Update PWREN*/
			pwr = mci_readl(host, PWREN);
			if(present) {
				pwr |= (1<<slot->id);
			}
			else {
				pwr &= ~(1<<slot->id);
			}
			mci_writel(host, PWREN, pwr);

			/* Card change detected */
			slot->last_detect_state = present;

			/* Power up slot */
			if (present != 0) {
				if (host->pdata->setpower)
					host->pdata->setpower(slot->id,
							      mmc->ocr_avail);

				set_bit(DW_MMC_CARD_PRESENT, &slot->flags);
			} else {
				if (host->pdata->setpower)
					host->pdata->setpower(slot->id, 0);
				clear_bit(DW_MMC_CARD_PRESENT, &slot->flags);
			}

			/* Clean up queue if present */
			mrq = slot->mrq;
			if (mrq) {
				if (mrq == host->mrq) {
					host->data = NULL;
					host->cmd = NULL;

					switch (host->state) {
					case STATE_IDLE:
						break;
					case STATE_SENDING_CMD:
						mrq->cmd->error = -ENOMEDIUM;
						if (!mrq->data)
							break;
						/* fall through */
					case STATE_SENDING_DATA:
						mrq->data->error = -ENOMEDIUM;
						dw_mci_stop_dma(host);
						break;
					case STATE_DATA_BUSY:
					case STATE_DATA_ERROR:
						if (mrq->data->error == -EINPROGRESS)
						{
							mrq->data->error = -ENOMEDIUM;
						}
						if (!mrq->stop)
						{
							break;
						}
						/* fall through */
					case STATE_SENDING_STOP:
						mrq->stop->error = -ENOMEDIUM;
						break;
					}

					/* Power down slot */
					if (present == 0) {

						/*
						 * Clear down the FIFO - doing so generates a
						 * block interrupt, hence setting the
						 * scatter-gather pointer to NULL.
						 */
						host->sg = NULL;

						ctrl = mci_readl(host, CTRL);
						ctrl |= SDMMC_CTRL_FIFO_RESET;
						mci_writel(host, CTRL, ctrl);

						ctrl = mci_readl(host, BMOD);
						ctrl |= 0x01; /* Software reset of DMA */
						mci_writel(host, BMOD, ctrl);

					}

					dw_mci_request_end(host, mrq);
				} else {
					if(!list_empty(&slot->queue_node)){
						list_del(&slot->queue_node);
						mrq->cmd->error = -ENOMEDIUM;
						if (mrq->data)
							mrq->data->error = -ENOMEDIUM;
						if (mrq->stop)
							mrq->stop->error = -ENOMEDIUM;

						slot->mrq = NULL;
						CTRACE("unlock"); 
						spin_unlock(&host->lock);
						mmc_request_done(slot->mmc, mrq);
						spin_lock(&host->lock);
						CTRACE("lock"); 
					}
				}
			}
			CTRACE("unlock"); 
			spin_unlock(&host->lock);
			present = dw_mci_get_cd(mmc);
		}

		mmc_detect_change(slot->mmc,
			msecs_to_jiffies(host->pdata->detect_delay_ms));
	}
}

struct dw_mci * g_host;

void dw_mci_logs()
{
	struct mmc_data	*data = g_host->mrq->data;
	struct mmc_command *cmd = g_host->mrq->cmd;
	u8 * data_buf = NULL;
	u32 i;

	printk(KERN_EMERG "CMD %d\n", cmd->opcode);		
	printk(KERN_EMERG "rawstats = 0x%x\n", g_host->rawstat);	
	printk(KERN_EMERG "irqstats = 0x%x\n", g_host->irqstat);	
	printk(KERN_EMERG "idsts = 0x%x\n", g_host->idsts);
	printk(KERN_EMERG "pending_events = 0x%x\n", g_host->pending_events);
	printk(KERN_EMERG "completed_events = 0x%x\n", g_host->completed_events);
	printk(KERN_EMERG "state = 0x%x\n", g_host->state);
	printk(KERN_EMERG "tasklet_process = 0x%x\n", g_host->tasklet_process);
	printk(KERN_EMERG "in_interrupt = 0x%x\n", 	in_interrupt());
	printk(KERN_EMERG "softirq_pending = 0x%x\n", local_softirq_pending() );
	printk(KERN_EMERG "tasklet->state = 0x%x\n", g_host->tasklet.state );
	printk(KERN_EMERG "tasklet->count = 0x%x\n", g_host->tasklet.count );
	printk(KERN_EMERG "slot id %d\n", (mci_readl(g_host, CMD) & 0x1f0000)>>16);

	if(data) {
		if(data->flags & MMC_DATA_WRITE)
			printk(KERN_EMERG "data to device\n");
		else
			printk(KERN_EMERG "data from device\n");
			
		printk(KERN_EMERG "blksz = %d, blocks = %d\n", data->blksz, data->blocks);

	}
}

EXPORT_SYMBOL(dw_mci_logs);

static int __init dw_mci_init_slot(struct dw_mci *host, unsigned int id)
{
	struct mmc_host *mmc;
	struct dw_mci_slot *slot;
	CTRACE("id:%d",id); 

	mmc = mmc_alloc_host(sizeof(struct dw_mci_slot), &host->pdev->dev);
	if (!mmc)
		return -ENOMEM;

	slot = mmc_priv(mmc);
	slot->id = id;
	slot->mmc = mmc;
	slot->host = host;
	INIT_LIST_HEAD(&slot->queue_node);

#ifdef	REQUEST_STAT
	slot->tm_start = 0;
	slot->tm_end = 0;
#endif
	mmc->ops = &dw_mci_ops;
	mmc->f_min = 400000;
	
	if(host->pdata->get_max_hz)
		mmc->f_max = host->pdata->get_max_hz(id);
	else
		mmc->f_max = 25000000;

	if (host->pdata->get_ocr)
		mmc->ocr_avail = host->pdata->get_ocr(id);
	else
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	/*
	 * Start with slot power disabled, it will be enabled when a card
	 * is detected.
	 */
	if (host->pdata->setpower)
		host->pdata->setpower(id, 0);

	if (host->pdata->caps)
		mmc->caps = 0;//host->pdata->caps;
	else
		mmc->caps = 0;

	if (host->pdata->get_bus_wd) {
		if (host->pdata->get_bus_wd(slot->id) == 4)
			mmc->caps |= MMC_CAP_4_BIT_DATA;
		else if (host->pdata->get_bus_wd(slot->id) == 8)
			mmc->caps |= MMC_CAP_8_BIT_DATA;
	}
	
	printk("slot id = %d ****************************mmc->caps = 0x%x \n", id, mmc->caps);
	
	if (host->pdata->quirks & DW_MCI_QUIRK_HIGHSPEED)
		mmc->caps |= MMC_CAP_SD_HIGHSPEED;

	mmc->max_segs = host->ring_size;
//	mmc->max_phys_segs = host->ring_size;
	mmc->max_blk_size = 65536;
	mmc->max_blk_count = host->ring_size;
	mmc->max_seg_size = PAGE_SIZE;
	mmc->max_req_size = mmc->max_seg_size * mmc->max_blk_count;

	host->vmmc = regulator_get(mmc_dev(mmc), "vmmc");
	if (IS_ERR(host->vmmc)) {
		printk(KERN_INFO "%s: no vmmc regulator found\n", mmc_hostname(mmc));
		host->vmmc = NULL;
	} else
		regulator_enable(host->vmmc);

	if (dw_mci_get_cd(mmc))
		set_bit(DW_MMC_CARD_PRESENT, &slot->flags);
	else
		clear_bit(DW_MMC_CARD_PRESENT, &slot->flags);

	host->slot[id] = slot;
	mmc_add_host(mmc);

#if defined(CONFIG_DEBUG_FS)
	dw_mci_init_debugfs(slot);
#endif

	/* Card initially undetected */
	slot->last_detect_state = 0;

	return 0;
}

static void dw_mci_cleanup_slot(struct dw_mci_slot *slot, unsigned int id)
{
	CTRACE("id:%d",id); 
	/* Shutdown detect IRQ */
	if (slot->host->pdata->exit)
		slot->host->pdata->exit(id);

	/* Debugfs stuff is cleaned up by mmc core */
	mmc_remove_host(slot->mmc);
	slot->host->slot[id] = NULL;
	mmc_free_host(slot->mmc);
}

static void dw_mci_init_dma(struct dw_mci *host)
{
	CTRACE(); 
	/* Alloc memory for sg translation */
	host->sg_cpu = dma_alloc_coherent(&host->pdev->dev, PAGE_SIZE,
					  &host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		dev_err(&host->pdev->dev, "%s: could not alloc DMA memory\n",
			__func__);
		goto no_dma;
	}

	/* Determine which DMA interface to use */
	host->dma_ops = &dw_mci_idmac_ops;
	dev_info(&host->pdev->dev, "Using internal DMA controller.\n");

	if (!host->dma_ops)
		goto no_dma;

	if (host->dma_ops->init) {
		if (host->dma_ops->init(host)) {
			dev_err(&host->pdev->dev, "%s: Unable to initialize "
				"DMA Controller.\n", __func__);
			goto no_dma;
		}
	} else {
		dev_err(&host->pdev->dev, "DMA initialization not found.\n");
		goto no_dma;
	}

	host->use_dma = 1;
	return;

no_dma:
	BUG();
	host->use_dma = 0;
	return;
}

static bool mci_wait_reset(struct device *dev, struct dw_mci *host)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned int ctrl;

	CTRACE(); 
	mci_writel(host, CTRL, (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET |
				SDMMC_CTRL_DMA_RESET));

	/* wait till resets clear */
	do {
		ctrl = mci_readl(host, CTRL);
		if (!(ctrl & (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET |
			      SDMMC_CTRL_DMA_RESET)))
			return true;
	} while (time_before(jiffies, timeout));

	if(dev)
		dev_err(dev, "Timeout resetting block (ctrl %#x)\n", ctrl);

	return false;
}

static int dw_mci_probe(struct platform_device *pdev)
{
	struct dw_mci *host;
	struct resource	*regs;
	struct dw_mci_board *pdata;
	int irq, ret, i, width;
	int id;
	u32 fifo_size;

	CTRACE(); 
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	host = kzalloc(sizeof(struct dw_mci), GFP_KERNEL);
	if (!host)
		return -ENOMEM;
	g_host = host;

	host->pdev = pdev;
	host->pdata = pdata = pdev->dev.platform_data;
	if (!pdata || !pdata->init) {
		dev_err(&pdev->dev,
			"Platform data must supply init function\n");
		ret = -ENODEV;
		goto err_freehost;
	}

	if (!pdata->select_slot && pdata->num_slots > 1) {
		dev_err(&pdev->dev,
			"Platform data must supply select_slot function\n");
		ret = -ENODEV;
		goto err_freehost;
	}

	if (!pdata->bus_hz) {
		dev_err(&pdev->dev,
			"Platform data must supply bus speed\n");
		ret = -ENODEV;
		goto err_freehost;
	}

	host->bus_hz = pdata->bus_hz;
	host->quirks = pdata->quirks;

	host->rawstat = 0;
	host->idsts = 0;
	host->irqstat = 0;

	spin_lock_init(&host->lock);
	spin_lock_init(&host->irq_lock);
	INIT_LIST_HEAD(&host->queue);

	ret = -ENOMEM;
	host->regs = ioremap(regs->start, regs->end - regs->start + 1);
	if (!host->regs)
		goto err_freehost;
#ifdef DEBUG
	i = mci_readl(host, HCON);
	printk(KERN_EMERG"+++++dw mmc feature++++\n");
	printk(KERN_EMERG"support card:%s\n",(i&0x1)==0?"mmc only":"sd mmc");
	printk(KERN_EMERG"max card num:%d\n",((i&(0x1f<<1))>>1)+1);
	printk(KERN_EMERG"bus type:%s\n",(i&(1<<6))==0?"APB":"AHB");
	printk(KERN_EMERG"data width:%d\n",16<<((i&(7<<7))>>7));
	printk(KERN_EMERG"addr width:%d\n",((i&(0x3f<<10))>>10)+1);
	printk(KERN_EMERG"dma:%d\n",((i&(3<<16))>>16));
	printk(KERN_EMERG"ge dma width:%d\n",16<<((i&(7<<18))>>18));
	printk(KERN_EMERG"fifo:%s\n",((i&(1<<21))==0?"out side":"inside"));
	printk(KERN_EMERG"hold:%s\n",((i&(1<<22))==0?"no":"yes"));
	printk(KERN_EMERG"clk false:%s\n",((i&(1<<23))==0?"no":"yes"));
	printk(KERN_EMERG"divider num:%d\n",((i&(3<<24))>>24)+1);
	printk(KERN_EMERG"+++dw mmc feature end+++\n");
#endif	
	/* disable clock to CIU */
	/* in case sd/mmc used in bootloader*/
	mci_writel(host, CTRL, 0);
	mci_writel(host, BMOD, 0);
	mci_writel(host, IDSTS, SDMMC_IDMAC_INT_TI | SDMMC_IDMAC_INT_RI | SDMMC_IDMAC_INT_NI);
	mci_writel(host, CLKENA, 0);
	mci_writel(host, CLKSRC, 0);
	mci_writel(host, CTYPE, 0);
	mci_writel(host, FIFOTH, 0x1f0000);
	mci_writel(host, PWREN, 0);

	host->dma_ops = pdata->dma_ops;
	dw_mci_init_dma(host);

	/*
	 * Get the host data width - this assumes that HCON has been set with
	 * the correct values.
	 */
	i = (mci_readl(host, HCON) >> 7) & 0x7;
	if (!i) {
		width = 16;
		host->data_shift = 1;
	} else if (i == 2) {
		width = 64;
		host->data_shift = 3;
	} else {
		/* Check for a reserved value, and warn if it is */
		WARN((i != 1),
		     "HCON reports a reserved host data width!\n"
		     "Defaulting to 32-bit access.\n");
		width = 32;
		host->data_shift = 2;
	}

	/* Reset all blocks */
	if (!mci_wait_reset(&pdev->dev, host)) {
		ret = -ENODEV;
		goto err_dmaunmap;
	}

	/* Clear the interrupts for the host controller */
	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	mci_writel(host, INTMASK, 0); /* disable all mmc interrupt first */

	/* Put in max timeout */
	mci_writel(host, TMOUT, 0xFFFFFFFF);

	/*
	 * FIFO threshold settings  RxMark  = fifo_size / 2 - 1,
	 *                          Tx Mark = fifo_size / 2 DMA Size = 8
	 */
	fifo_size = mci_readl(host, FIFOTH);
	fifo_size = ((fifo_size >> 16) & 0x7ff)+1;
	host->fifoth_val = ((0x2 << 28) | ((fifo_size/2 - 1) << 16) |
			((fifo_size/2) << 0));
	mci_writel(host, FIFOTH, host->fifoth_val);

	/* disable clock to CIU */
	mci_writel(host, CLKENA, 0);
	mci_writel(host, CLKSRC, 0);
	mci_writel(host, PWREN, 0);

	tasklet_init(&host->tasklet, dw_mci_tasklet_func, (unsigned long)host);
	tasklet_init(&host->card_tasklet,
		     dw_mci_tasklet_card, (unsigned long)host);


	ret = request_irq(irq, dw_mci_interrupt, 0, "dw-mci", host);
	if (ret)
		goto err_dmaunmap;

	platform_set_drvdata(pdev, host);

	if (host->pdata->num_slots)
		host->num_slots = host->pdata->num_slots;
	else
		host->num_slots = ((mci_readl(host, HCON) >> 1) & 0x1F) + 1;

	BUG_ON(host->num_slots > MAX_MCI_SLOTS);
	/* We need at least one slot to succeed */
	for (i = 0; i < host->num_slots; i++) {
		if(host->pdata->get_slot_id)
			id = host->pdata->get_slot_id(i);
		else
			id = i;
		ret = dw_mci_init_slot(host, id);
		if (ret) {
			ret = -ENODEV;
			goto err_init_slot;
		}
	}

	tasklet_schedule(&host->card_tasklet);

	/*
	 * Enable interrupts for command done, data over, data empty, card det,
	 * receive ready and error such as transmit, receive timeout, crc error
	 */
	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE | SDMMC_INT_DATA_OVER |
		   DW_MCI_ERROR_FLAGS | SDMMC_INT_CD);
	mci_writel(host, CTRL, SDMMC_CTRL_INT_ENABLE); /* Enable mci interrupt */

	dev_info(&pdev->dev, "DW MMC controller at irq %d, "
		 "%d bit host data width\n", irq, width);
	if (host->quirks & DW_MCI_QUIRK_IDMAC_DTO)
		dev_info(&pdev->dev, "Internal DMAC interrupt fix enabled.\n");

	return 0;

err_init_slot:
	/* De-init any initialized slots */
	while (i > 0) {
		if (host->slot[i])
			dw_mci_cleanup_slot(host->slot[i], i);
		i--;
	}
	free_irq(irq, host);

err_dmaunmap:
	if (host->use_dma && host->dma_ops->exit)
		host->dma_ops->exit(host);
	dma_free_coherent(&host->pdev->dev, PAGE_SIZE,
			  host->sg_cpu, host->sg_dma);
	iounmap(host->regs);

	if (host->vmmc) {
		regulator_disable(host->vmmc);
		regulator_put(host->vmmc);
	}


err_freehost:
	kfree(host);
	return ret;
}

static int __exit dw_mci_remove(struct platform_device *pdev)
{
	struct dw_mci *host = platform_get_drvdata(pdev);
	int i;
	CTRACE(); 

	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	mci_writel(host, INTMASK, 0); /* disable all mmc interrupt first */

	platform_set_drvdata(pdev, NULL);

	for (i = 0; i < host->num_slots; i++) {
		dev_dbg(&pdev->dev, "remove slot %d\n", i);
		if (host->slot[i])
			dw_mci_cleanup_slot(host->slot[i], i);
	}

	/* disable clock to CIU */
	mci_writel(host, CLKENA, 0);
	mci_writel(host, CLKSRC, 0);
	mci_writel(host, PWREN, 0);

	free_irq(platform_get_irq(pdev, 0), host);
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);

	if (host->use_dma && host->dma_ops->exit)
		host->dma_ops->exit(host);

	if (host->vmmc) {
		regulator_disable(host->vmmc);
		regulator_put(host->vmmc);
	}

	iounmap(host->regs);

	kfree(host);
	return 0;
}

#ifdef CONFIG_PM
/*
 * TODO: we should probably disable the clock to the card in the suspend path.
 */
static void mci_cmd(struct dw_mci *host, u32 cmd, u32 arg)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned int cmd_status = 0;

	mci_writel(host, CMDARG, arg);
	wmb();
	mci_writel(host, CMD, SDMMC_CMD_START | cmd);

	while (time_before(jiffies, timeout)) {
		cmd_status = mci_readl(host, CMD);
		if (!(cmd_status & SDMMC_CMD_START))
			return;
	}
}
static void dw_mci_save(struct dw_mci *host)
{
	host->div = mci_readl(host, CLKDIV);
	host->clksrc = mci_readl(host, CLKSRC);
	host->clken = mci_readl(host, CLKENA);
	host->ctype = mci_readl(host, CTYPE);
	host->pwren = mci_readl(host, PWREN);
}
static void dw_mci_restore(struct dw_mci *host)
{
	mci_writel(host, PWREN, host->pwren);
	mci_writel(host, CLKENA, 0);
	mci_writel(host, CLKSRC, 0);

	/* inform CIU */
	mci_cmd(host, SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

	mci_writel(host, CLKSRC, host->clksrc);

	/* set clock to desired speed */

	mci_writel(host, CLKDIV, host->div);
	/* inform CIU */
	mci_cmd(host, SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

	/* enable clock */
	mci_writel(host, CLKENA, host->clken);

	/* inform CIU */
	mci_cmd(host, SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

	mci_writel(host, CTYPE, host->ctype);

}
static int dw_mci_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	int i, ret;
	struct dw_mci *host = platform_get_drvdata(pdev);

	CTRACE(); 
	dw_mci_save(host);

	if (host->vmmc)
		regulator_enable(host->vmmc);

	for (i = 0; i < host->num_slots; i++) {
		struct dw_mci_slot *slot = host->slot[i];
		if (!slot)
			continue;
		ret = mmc_suspend_host(slot->mmc);
		if (ret < 0) {
			while (--i >= 0) {
				slot = host->slot[i];
				if (slot)
					mmc_resume_host(host->slot[i]->mmc);
			}
			return ret;
		}
	}

	if (host->vmmc)
		regulator_disable(host->vmmc);

	return 0;
}

static int dw_mci_resume(struct platform_device *pdev)
{
	int i, ret;
	struct dw_mci *host = platform_get_drvdata(pdev);

	CTRACE(); 
	if (host->dma_ops->init)
		host->dma_ops->init(host);

	if (!mci_wait_reset(&pdev->dev, host)) {
		ret = -ENODEV;
		return ret;
	}

	/* Restore the old value at FIFOTH register */
	mci_writel(host, FIFOTH, host->fifoth_val);

	ERR("RINTSTS 0x%x\n", mci_readl(host, RINTSTS));

	mci_writel(host, RINTSTS, 0xFFFFFFFE);
	mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE | SDMMC_INT_DATA_OVER |
		   DW_MCI_ERROR_FLAGS | SDMMC_INT_CD);

	dw_mci_restore(host);

	mci_writel(host, CTRL, SDMMC_CTRL_INT_ENABLE);

	for (i = 0; i < host->num_slots; i++) {
		struct dw_mci_slot *slot = host->slot[i];
		struct mmc_card * card = NULL;
		if (!slot)
			continue;
		card = host->slot[i]->mmc->card;
		if(card && card->type == MMC_TYPE_SDIO) {
			mmc_detect_change(host->slot[i]->mmc, msecs_to_jiffies(200));
		} 
		else {
			ret = mmc_resume_host(host->slot[i]->mmc);
			if (ret < 0)
				return ret;
		}
	}

	ERR("INTMASK 0x%x, CTRL 0x%x\n", mci_readl(host, INTMASK), mci_readl(host, CTRL));
	return 0;
}
#else
#define dw_mci_suspend	NULL
#define dw_mci_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver dw_mci_driver = {
	.remove		= __exit_p(dw_mci_remove),
	.suspend	= dw_mci_suspend,
	.resume		= dw_mci_resume,
	.driver		= {
		.name		= "dw_mmc",
	},
};

static int __init dw_mci_init(void)
{
	CTRACE(); 
	return platform_driver_probe(&dw_mci_driver, dw_mci_probe);
}

static void __exit dw_mci_exit(void)
{
	CTRACE(); 
	platform_driver_unregister(&dw_mci_driver);
}

module_init(dw_mci_init);
module_exit(dw_mci_exit);

MODULE_DESCRIPTION("DW Multimedia Card Interface driver");
MODULE_AUTHOR("NXP Semiconductor VietNam");
MODULE_AUTHOR("Imagination Technologies Ltd");
MODULE_LICENSE("GPL v2");
