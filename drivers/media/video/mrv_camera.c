/*
 * V4L2 Driver for MRV camera host
 *
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/videobuf2-dma-contig.h>

#include <mach/camera.h>
#include "mrv_camera.h"

#define CAM_DRV_NAME "mrv_camera"
#define ISP_CLK_NAME "ns115_isp_clk"


#define BUS_FLAGS	(SOCAM_MASTER |			\
		SOCAM_HSYNC_ACTIVE_HIGH |	\
		SOCAM_HSYNC_ACTIVE_LOW |	\
		SOCAM_VSYNC_ACTIVE_HIGH |	\
		SOCAM_VSYNC_ACTIVE_LOW |	\
		SOCAM_PCLK_SAMPLE_RISING |	\
		SOCAM_PCLK_SAMPLE_FALLING |	\
		SOCAM_DATA_ACTIVE_HIGH |	\
		SOCAM_DATAWIDTH_8)

#define MAX_SUPPORT_WIDTH		2592
#define MAX_SUPPORT_HEIGHT		1944
#define MAX_VIDEO_MEM_SIZE		(64 * 1024 * 1024)
#define MIN_FRAME_RATE			2

#define V4L2_CID_NUSMART_CAMERA_DEV_INPUT	(V4L2_CID_PRIVATE_BASE + 120)
#define V4L2_CID_NUSMART_STREAM_OUTPUT_PATH	(V4L2_CID_PRIVATE_BASE + 121)
#define V4L2_CID_NUSMART_STREAM_OUTPUT_SIZE	(V4L2_CID_PRIVATE_BASE + 123)

#define TEST_RGB_MODE

enum v4l2_camera_dev_input {
	CAMERA_INPUT_FRONT,
	CAMERA_INPUT_BACK,
};

enum v4l2_stream_output_path {
	STREAM_NONE,
	STREAM_CAPTURE,
	STREAM_PREVIEW,
	STREAM_BOTH,
};

enum mrv_isp_state {
	MRV_ISP_IDLE,
	MRV_ISP_BUSY,
	MRV_ISP_WAIT,
};

struct frame_size {
	u32	w;
	u32	h;
};

struct mrv_spec {
	u32			input_mux; //0: port1, 1: port2
	u32			path_mux;  //0: dis, 1: mp, 2: sp
	struct frame_size	out;
};

struct mrv_buffer {
	struct vb2_buffer		vb;
	struct list_head		queue;
	u32				paddr;
};

struct mrv_cam_dev {
	struct soc_camera_host		ici;
	struct soc_camera_device	*icd;
	struct mrv_camera_pdata		*pdata;
	struct mrv_buffer		*active;
	struct clk			*clk;
	struct list_head		capture;
	struct vb2_alloc_ctx		*alloc_ctx;
	struct mrv_spec			spec;
	enum mrv_isp_state		state;
	wait_queue_head_t		stop_wq;
	void __iomem			*reg_base;
	unsigned int			irq_isp;
	unsigned int			irq_mi;
	unsigned long			mclk;
	spinlock_t			lock;
};

static inline void mrv_writel(struct mrv_cam_dev *pcdev, u32 reg, u32 val)
{
	writel(val, pcdev->reg_base + reg);
}
static inline u32 mrv_readl(struct mrv_cam_dev *pcdev, u32 reg)
{
	return readl(pcdev->reg_base + reg);
}

static void mrv_config_scaling(struct mrv_cam_dev *pcdev, u32 in_w, u32 in_h)
{
	u32 out_w, out_h, factor, flag = 0;
	out_w = pcdev->spec.out.w;
	out_h = pcdev->spec.out.h;

	if (pcdev->spec.path_mux & 0x1) {
		if (out_w > in_w) {
			factor = MRV_SCALING_UP(in_w, out_w);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_HY, factor);
			factor = MRV_SCALING_UP(in_w/2, out_w/2);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_HCB, factor);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_HCR, factor);
			flag |= (MRV_SCALE_UP_HY | MRV_SCALE_UP_HC);
			flag |= (MRV_SCALE_EN_HY | MRV_SCALE_EN_HC);
		}
		else if (out_w < in_w) {
			factor = MRV_SCALING_DW(in_w, out_w);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_HY, factor);
			factor = MRV_SCALING_DW(in_w/2, out_w/2);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_HCB, factor);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_HCR, factor);
			flag &= ~(MRV_SCALE_UP_HY | MRV_SCALE_UP_HC);
			flag |= (MRV_SCALE_EN_HY | MRV_SCALE_EN_HC);
		}
		else
			flag &= ~(MRV_SCALE_EN_HY | MRV_SCALE_EN_HC);

		if (out_h > in_h) {
			factor = MRV_SCALING_UP(in_h, out_h);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_VY, factor);
			flag |= MRV_SCALE_UP_VY;
			flag |= MRV_SCALE_EN_VY;
			if (out_h/2 > in_h) {
				factor = MRV_SCALING_UP(in_h, out_h/2);
				mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_VC, factor);
				flag |= MRV_SCALE_UP_VC;
				flag |= MRV_SCALE_EN_VC;
			}
			else if (out_h/2 < in_h) {
				factor = MRV_SCALING_DW(in_h, out_h/2);
				mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_VC, factor);
				flag &= ~MRV_SCALE_UP_VC;
				flag |= MRV_SCALE_EN_VC;
			}
			else
				flag &= ~MRV_SCALE_EN_VC;
		}
		else if (out_h < in_h) {
			factor = MRV_SCALING_DW(in_h, out_h);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_VY, factor);
			factor = MRV_SCALING_DW(in_h, out_h/2);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_VC, factor);
			flag &= ~(MRV_SCALE_UP_VY | MRV_SCALE_UP_VC);
			flag |= (MRV_SCALE_EN_VY | MRV_SCALE_EN_VC);
		}
		else {
			flag &= ~MRV_SCALE_EN_VY;
			factor = MRV_SCALING_DW(in_h, out_h/2);
			mrv_writel(pcdev, MRV_MRSZ + MRSZ_SCALE_VC, factor);
			flag &= ~MRV_SCALE_UP_VC;
			flag |= MRV_SCALE_EN_VC;
		}

		flag |= MRV_SCALE_UPD;
		mrv_writel(pcdev, MRV_MRSZ + MRSZ_CTRL, flag);
	}
	else if (pcdev->spec.path_mux & 0x2) {
		if (out_w > in_w) {
			factor = MRV_SCALING_UP(in_w, out_w);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_HY, factor);
			factor = MRV_SCALING_UP(in_w/2, out_w/2);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_HCB, factor);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_HCR, factor);
			flag |= (MRV_SCALE_UP_HY | MRV_SCALE_UP_HC);
			flag |= (MRV_SCALE_EN_HY | MRV_SCALE_EN_HC);
		}
		else if (out_w < in_w) {
			factor = MRV_SCALING_DW(in_w, out_w);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_HY, factor);
			factor = MRV_SCALING_DW(in_w/2, out_w/2);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_HCB, factor);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_HCR, factor);
			flag &= ~(MRV_SCALE_UP_HY | MRV_SCALE_UP_HC);
			flag |= (MRV_SCALE_EN_HY | MRV_SCALE_EN_HC);
		}
		else
			flag &= ~(MRV_SCALE_EN_HY | MRV_SCALE_EN_HC);

		if (out_h > in_h) {
			factor = MRV_SCALING_UP(in_h, out_h);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_VY, factor);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_VC, factor);
			flag |= (MRV_SCALE_UP_VY | MRV_SCALE_UP_VC);
			flag |= (MRV_SCALE_EN_VY | MRV_SCALE_EN_VC);
		}
		else if (out_h < in_h) {
			factor = MRV_SCALING_DW(in_h, out_h);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_VY, factor);
			mrv_writel(pcdev, MRV_SRSZ + SRSZ_SCALE_VC, factor);
			flag &= ~(MRV_SCALE_UP_VY | MRV_SCALE_UP_VC);
			flag |= (MRV_SCALE_EN_VY | MRV_SCALE_EN_VC);
		}
		else
			flag &= ~(MRV_SCALE_EN_VY | MRV_SCALE_EN_VC);

		flag |= MRV_SCALE_UPD;
		mrv_writel(pcdev, MRV_SRSZ + SRSZ_CTRL, flag);
	}
	return;
}

static irqreturn_t mrv_isp_irq(int irq, void *dev_id)
{
	struct mrv_cam_dev *pcdev = dev_id;
	struct mrv_buffer *buf;
	u32 pending, val;
	int state;
	irqreturn_t ret = IRQ_NONE;
	spin_lock(&pcdev->lock);
	buf = pcdev->active;
	state = pcdev->state;
	spin_unlock(&pcdev->lock);

	/* get interrupt status */
	pending = mrv_readl(pcdev, MRV_ISP + ISP_MIS);
	mrv_writel(pcdev, MRV_ISP + ISP_ICR, pending);

	if (pending & MRV_INT_VSYNC_MASK) {
		if (MRV_ISP_IDLE == state) {
			spin_lock(&pcdev->lock);
			pcdev->state = MRV_ISP_BUSY;
			spin_unlock(&pcdev->lock);
		}
		/* Disable ISP */
		val = mrv_readl(pcdev, MRV_ISP + ISP_FLAGS_SHD);
		if (val & MRV_ISP_ENABLE) {
			val = mrv_readl(pcdev, MRV_ISP + ISP_CTRL);
			val &= ~(MRV_ISP_INF_ENA | MRV_ISP_ENABLE);
			val |= MRV_ISP_GEN_UPD;
			mrv_writel(pcdev, MRV_ISP + ISP_CTRL, val);
		}
		else if (buf) {
			val = mrv_readl(pcdev, MRV_ISP + ISP_CTRL);
			val |= MRV_ISP_ENABLE | MRV_ISP_GEN_UPD;
			mrv_writel(pcdev, MRV_ISP + ISP_CTRL, val);
		}
		ret = IRQ_HANDLED;
	}
	else if ((pending & MRV_INT_FROUT_MASK) &&
			(MRV_ISP_WAIT == state)) {
		spin_lock(&pcdev->lock);
		pcdev->state = MRV_ISP_IDLE;
		mrv_writel(pcdev, MRV_ISP + ISP_IMSC, 0);
		spin_unlock(&pcdev->lock);
		ret = IRQ_HANDLED;
	}
	else {
		if (pending & MRV_INT_DATA_LOSS) {
			val = mrv_readl(pcdev, MRV_MI + MI_STATUS);
			mrv_writel(pcdev, MRV_MI + MI_STATUS_CLR, val);
			printk(KERN_ERR "MI_STATUS: 0x%x\n", val);
		}
		if (pending & MRV_INT_PIC_SIZE_ERR) {
			val = mrv_readl(pcdev, MRV_ISP + ISP_ERR);
			mrv_writel(pcdev, MRV_ISP + ISP_ERR_CLR, val);
			printk(KERN_ERR "ISP_ERR: 0x%x\n", val);

			val = mrv_readl(pcdev, MRV_ISP + ISP_CTRL);
			val &= ~MRV_ISP_ENABLE;
			val |= MRV_ISP_CFG_UPD;
			mrv_writel(pcdev, MRV_ISP + ISP_CTRL, val);

			mrv_writel(pcdev, VI_IRCL, MRV_VI_IRCL_ISP | MRV_VI_IRCL_MI);
			udelay(1);
			mrv_writel(pcdev, VI_IRCL, 0);

			mrv_writel(pcdev, MRV_MI + MI_INIT, MRV_MI_SKIP_ENA);
			val = mrv_readl(pcdev, MRV_ISP + ISP_CTRL);
			val |= MRV_ISP_CFG_UPD;
			mrv_writel(pcdev, MRV_ISP + ISP_CTRL, val);
			val |= MRV_ISP_GEN_UPD | MRV_ISP_ENABLE;
			mrv_writel(pcdev, MRV_ISP + ISP_CTRL, val);
		}
		ret = IRQ_HANDLED;
	}

	return ret;
}

static irqreturn_t mrv_mi_irq(int irq, void *dev_id)
{
	struct mrv_cam_dev *pcdev = dev_id;
	struct mrv_buffer *buf;
	struct vb2_buffer *vb;
	struct frame_size crop;
	u32 base, size, pending, val;
	static u32 last_w = 0;
	static u32 last_h = 0;
	static int flag = 1;
	int state;
	spin_lock(&pcdev->lock);
	buf = pcdev->active;
	vb = &buf->vb;
	state = pcdev->state;
	spin_unlock(&pcdev->lock);

	/* Get interrupt status & clear interrupt */
	pending = mrv_readl(pcdev, MRV_MI + MI_MIS);
	mrv_writel(pcdev, MRV_MI + MI_ICR, pending);

	/* Mask MI interrupt & wakeup stop_wq */
	if (MRV_ISP_IDLE == state) {
		wake_up_interruptible(&pcdev->stop_wq);
		return IRQ_HANDLED;
	}
	/* Deal with buffers */
	if (buf && flag) {
		list_del_init(&buf->queue);
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	}
	spin_lock(&pcdev->lock);
	if (list_empty(&pcdev->capture)) {
		printk(KERN_INFO "Warning: No buffer for Camera\n");
		flag = 0;
		pcdev->active = NULL;
		spin_unlock(&pcdev->lock);
		goto enable_isp;
	}
	if (0 == flag) {
		flag = 1;
	}
	else {
		pcdev->active = list_entry(pcdev->capture.next,
				struct mrv_buffer, queue);
	}
	base = pcdev->active->paddr;
	size = pcdev->spec.out.w * pcdev->spec.out.h;
	spin_unlock(&pcdev->lock);

	/* Update MI output address & size */
	val = mrv_readl(pcdev, MRV_MI + MI_CTRL);
	if (pending & MRV_INT_MP_MASK) {
		mrv_writel(pcdev, MRV_MI + MI_MP_Y_BASE_AD_INIT, base);
		mrv_writel(pcdev, MRV_MI + MI_MP_Y_SIZE_INIT, size);
		mrv_writel(pcdev, MRV_MI + MI_MP_CB_BASE_AD_INIT, base + size);
		mrv_writel(pcdev, MRV_MI + MI_MP_CB_SIZE_INIT, (size >> 1));
	}
	else if (pending & MRV_INT_SP_MASK) {
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_BASE_AD_INIT, base);
#ifdef TEST_RGB_MODE
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_SIZE_INIT, (size << 1));
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_LLENGTH, pcdev->spec.out.w);
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_PIC_WIDTH, pcdev->spec.out.w);
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_PIC_HEIGHT, pcdev->spec.out.h);
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_PIC_SIZE, (size << 1));
#else
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_SIZE_INIT, size);
		mrv_writel(pcdev, MRV_MI + MI_SP_CB_BASE_AD_INIT, base + size);
		mrv_writel(pcdev, MRV_MI + MI_SP_CB_SIZE_INIT, (size >> 1));
#endif
	}
	else
		return IRQ_NONE;
	/* Update DMA address & MI config */
	val |= MRV_MI_INIT_OFFS_ENA | MRV_MI_INIT_BASE_ENA;
	mrv_writel(pcdev, MRV_MI + MI_CTRL, val);
	mrv_writel(pcdev, MRV_MI + MI_INIT, MRV_MI_INIT_ENA);

enable_isp:
	/* Enable ISP */
	val = mrv_readl(pcdev, MRV_ISP + ISP_CTRL);
	val |= MRV_ISP_INF_ENA | MRV_ISP_ENABLE;
	pending = mrv_readl(pcdev, MRV_ISP + ISP_MIS);
	mrv_writel(pcdev, MRV_ISP + ISP_ICR, pending);
	if (pending & MRV_INT_VSYNC_MASK)
		val |= MRV_ISP_GEN_UPD;
	else
		val |= MRV_ISP_CFG_UPD;
	mrv_writel(pcdev, MRV_ISP + ISP_CTRL, val);
	/* Update scaling config */
	crop.w = mrv_readl(pcdev, MRV_ISP + ISP_OUT_H_SIZE_SHD);
	crop.h = mrv_readl(pcdev, MRV_ISP + ISP_OUT_V_SIZE_SHD);
	if ((crop.w != last_w) || (crop.h != last_h))
		mrv_config_scaling(pcdev, crop.w, crop.h);
	last_w = crop.w;
	last_h = crop.h;

	return IRQ_HANDLED;
}

static void start_dma(struct mrv_cam_dev *pcdev, struct mrv_buffer *buf)
{
	u32 base, size, val;
	struct frame_size crop;

	/* Init Scaling config */
	crop.w = mrv_readl(pcdev, MRV_ISP + ISP_OUT_H_SIZE_SHD);
	crop.h = mrv_readl(pcdev, MRV_ISP + ISP_OUT_V_SIZE_SHD);
	mrv_config_scaling(pcdev, crop.w, crop.h);

	/* Init MI output address & size */
	base = buf->paddr;
	size = pcdev->spec.out.w * pcdev->spec.out.h;
	val = mrv_readl(pcdev, MRV_MI + MI_CTRL);
	if (pcdev->spec.path_mux & 0x1) {
		val |= 0x1;
		mrv_writel(pcdev, MRV_MI + MI_MP_Y_BASE_AD_INIT, base);
		mrv_writel(pcdev, MRV_MI + MI_MP_Y_SIZE_INIT, size);
		mrv_writel(pcdev, MRV_MI + MI_MP_CB_BASE_AD_INIT, base + size);
		mrv_writel(pcdev, MRV_MI + MI_MP_CB_SIZE_INIT, (size >> 1));
		val |= MRV_MI_MP_OUT_SEMI;
	}
	else if (pcdev->spec.path_mux & 0x2) {
		val |= 0x2;
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_BASE_AD_INIT, base);
#ifdef TEST_RGB_MODE
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_SIZE_INIT, (size << 1));
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_LLENGTH, pcdev->spec.out.w);
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_PIC_WIDTH, pcdev->spec.out.w);
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_PIC_HEIGHT, pcdev->spec.out.h);
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_PIC_SIZE, (size << 1));
		val |= MRV_MI_SP_OUT_565_FMT | MRV_MI_SP_IN_422_FMT;
#else
		mrv_writel(pcdev, MRV_MI + MI_SP_Y_SIZE_INIT, size);
		mrv_writel(pcdev, MRV_MI + MI_SP_CB_BASE_AD_INIT, base + size);
		mrv_writel(pcdev, MRV_MI + MI_SP_CB_SIZE_INIT, (size >> 1));
		val |= MRV_MI_SP_OUT_420_FMT | MRV_MI_SP_OUT_SEMI;
		val |= MRV_MI_SP_IN_422_FMT;
#endif
	}

	/* Enable base & offset update */
	val |= MRV_MI_INIT_OFFS_ENA | MRV_MI_INIT_BASE_ENA;
	mrv_writel(pcdev, MRV_MI + MI_CTRL, val);
	mrv_writel(pcdev, MRV_MI + MI_INIT, MRV_MI_INIT_ENA);
	/* Enable ISP */
	val = mrv_readl(pcdev, MRV_ISP + ISP_CTRL);
	val |= MRV_ISP_INF_ENA;
	mrv_writel(pcdev, MRV_ISP + ISP_CTRL, val);
}

/*-----------------------------------------------------------------------------
  Videobuf operations
  -----------------------------------------------------------------------------*/
static int mrv_queue_setup(struct vb2_queue *vq,
		unsigned int *count, unsigned int *num_planes,
		unsigned long sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;

	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);


	if (bytes_per_line < 0)
		return bytes_per_line;

	if (!*count)
		*count = 2;

	if (sizes[0] * *count > MAX_VIDEO_MEM_SIZE)
		*count = MAX_VIDEO_MEM_SIZE / sizes[0];

	*num_planes = 1;
	sizes[0] = bytes_per_line * icd->user_height;
	alloc_ctxs[0] = pcdev->alloc_ctx;

	pcdev->active = NULL;

	printk(KERN_INFO "count=%d, size=%ld\n", *count, sizes[0]);

	return 0;
}

static int mrv_buf_init(struct vb2_buffer *vb)
{
	struct mrv_buffer *buf = container_of(vb, struct mrv_buffer, vb);

	INIT_LIST_HEAD(&buf->queue);

	return 0;
}

static int mrv_buf_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct mrv_buffer *buf = container_of(vb, struct mrv_buffer, vb);
	unsigned long size;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	size = bytes_per_line * icd->user_height;

	if (vb2_plane_size(vb, 0) < size ||
			vb2_plane_size(vb, 0) < size) {
		printk(KERN_ERR "Buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -ENOBUFS;
	}

	vb2_set_plane_payload(&buf->vb, 0, size);

	buf->paddr = vb2_dma_contig_plane_paddr(vb, 0);

	return 0;
}


static void mrv_buf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	struct mrv_buffer *buf = container_of(vb, struct mrv_buffer, vb);
	unsigned long flags;

	spin_lock_irqsave(&pcdev->lock, flags);

	list_add_tail(&buf->queue, &pcdev->capture);

	if (!pcdev->active) {
		printk(KERN_INFO "Queue buffer for Camera\n");
		pcdev->active = buf;
		if (MRV_ISP_IDLE == pcdev->state)
			start_dma(pcdev, buf);
	}

	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static void mrv_buf_cleanup(struct vb2_buffer *vb)
{
	struct mrv_buffer *buf = container_of(vb, struct mrv_buffer, vb);

	buf->paddr = 0;
}

static int mrv_start_streaming(struct vb2_queue *vq)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	unsigned long flags;
	u32 val;

	if (pcdev->pdata && pcdev->pdata->prio_set)
		pcdev->pdata->prio_set(1);

	spin_lock_irqsave(&pcdev->lock, flags);

	/* Set data path */
	val = mrv_readl(pcdev, VI_DPCL);
	val |= pcdev->spec.input_mux << MRV_DPCL_INPUT_BIT;
	val |= 1 << MRV_DPCL_MPMUX_BIT;
	val |= pcdev->spec.path_mux << MRV_DPCL_CHMOD_BIT;
	mrv_writel(pcdev, VI_DPCL, val);

	pcdev->state = MRV_ISP_IDLE;
	/* Clear all interrupt */
	mrv_writel(pcdev, MRV_ISP + ISP_ICR, 0xffff);
	mrv_writel(pcdev, MRV_MI + MI_ICR, 0xffff);
	/* Enable ISP & MI interrupt */
	val = MRV_INT_VSYNC_MASK | MRV_INT_PIC_SIZE_ERR | MRV_INT_DATA_LOSS;
	mrv_writel(pcdev, MRV_ISP + ISP_IMSC, val);
	mrv_writel(pcdev, MRV_MI + MI_IMSC, pcdev->spec.path_mux);
	/* Enable ISP config */
	val = MRV_ISP_CFG_UPD | MRV_ISP_INF_ENA | MRV_ISP_MODE;
	mrv_writel(pcdev, MRV_ISP + ISP_CTRL, val);
	val |= MRV_ISP_GEN_UPD | MRV_ISP_ENABLE;
	mrv_writel(pcdev, MRV_ISP + ISP_CTRL, val);

	spin_unlock_irqrestore(&pcdev->lock, flags);

	return 0;
}

/* abort streaming and wait for last buffer */
static int mrv_stop_streaming(struct vb2_queue *vq)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	struct mrv_buffer *buf, *node;
	unsigned long flags;
	long timeout;
	u32 val;
	int ret;

	spin_lock_irqsave(&pcdev->lock, flags);

	pcdev->state = MRV_ISP_WAIT;
	/* enable ISP frame out interrupt */
	mrv_writel(pcdev, MRV_ISP + ISP_ICR, MRV_INT_FROUT_MASK);
	val = mrv_readl(pcdev, MRV_ISP + ISP_IMSC);
	val |= MRV_INT_FROUT_MASK;
	mrv_writel(pcdev, MRV_ISP + ISP_IMSC, val);

	spin_unlock_irqrestore(&pcdev->lock, flags);

	/* Wait until the end of the current frame. */
	timeout = HZ / MIN_FRAME_RATE;
	ret = wait_event_interruptible_timeout(pcdev->stop_wq,
			pcdev->state == MRV_ISP_IDLE, timeout);
	if (ret > 0)
		ret = 0;
	else if (0 == ret)
	{
		printk(KERN_INFO "Warning %d: Waiting for frame end timeout\n",
				pcdev->state);
		ret = -ETIMEDOUT;
	}

	spin_lock_irqsave(&pcdev->lock, flags);

	pcdev->active = NULL;
	/* Release all active buffers */
	list_for_each_entry_safe(buf, node, &pcdev->capture, queue) {
		list_del_init(&buf->queue);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	/* Disable interrupts */
	mrv_writel(pcdev, MRV_MI + MI_IMSC, 0);
	mrv_writel(pcdev, MRV_ISP + ISP_IMSC, 0);
	/* Disable all modules */
	mrv_writel(pcdev, MRV_ISP + ISP_CTRL, MRV_ISP_CFG_UPD);
	mrv_writel(pcdev, MRV_MRSZ + MRSZ_CTRL, MRV_SCALE_DIS);
	mrv_writel(pcdev, MRV_SRSZ + SRSZ_CTRL, MRV_SCALE_DIS);
	mrv_writel(pcdev, MRV_MI + MI_CTRL, 0);
	mrv_writel(pcdev, MRV_MI + MI_INIT, MRV_MI_INIT_ENA);
	pcdev->state = MRV_ISP_IDLE;

	spin_unlock_irqrestore(&pcdev->lock, flags);

	if (pcdev->pdata && pcdev->pdata->prio_set)
		pcdev->pdata->prio_set(0);

	return ret;
}

static struct vb2_ops mrv_videobuf_ops = {
	.queue_setup		= mrv_queue_setup,
	.buf_init		= mrv_buf_init,
	.buf_prepare		= mrv_buf_prepare,
	.buf_cleanup		= mrv_buf_cleanup,
	.buf_queue		= mrv_buf_queue,
	.start_streaming	= mrv_start_streaming,
	.stop_streaming		= mrv_stop_streaming,
	.wait_prepare		= soc_camera_unlock,
	.wait_finish		= soc_camera_lock,
};

/*-----------------------------------------------------------------------------
  SOC camera operations for the device
  -----------------------------------------------------------------------------*/
static int mrv_camera_init_videobuf(struct vb2_queue *q,
		struct soc_camera_device *icd)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_USERPTR;
	q->drv_priv = icd;
	q->buf_struct_size = sizeof(struct mrv_buffer);
	q->ops = &mrv_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;

	return vb2_queue_init(q);
}

static int mrv_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;

	if (pcdev->icd)
		return -EBUSY;

	/* enable isp clk & power on */
	clk_enable(pcdev->clk);

	if (icd->ops && icd->ops->resume)
		icd->ops->resume(icd);

	pcdev->icd = icd;

	printk(KERN_INFO "Nusmart Camera driver attached to camera %d\n",
			icd->devnum);

	return 0;
}

static void mrv_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	struct pm_message state;

	if (icd != pcdev->icd) {
		printk(KERN_INFO "%s icd: %p, pcdev: %p, pcdev->icd: %p\n",
				__func__, icd, pcdev, pcdev->icd);
		return;
	}

	state.event = 0;

	if (icd->ops && icd->ops->suspend)
		icd->ops->suspend(icd, state);

	pcdev->icd = NULL;

	/* disable isp clk & power off */
	clk_disable(pcdev->clk);

	printk(KERN_INFO "Nusmart Camera driver detached from camera %d\n",
			icd->devnum);
}

static void mrv_reset(struct mrv_cam_dev *pcdev)
{
	u32 val;

	/* Reset CamerIC */
	mrv_writel(pcdev, VI_IRCL, MRV_VI_IRCL_HW);
	do {
		udelay(1);
		val = mrv_readl(pcdev, VI_IRCL);
	} while (val & MRV_VI_IRCL_HW);
	/* Set clk gating */
	mrv_writel(pcdev, VI_ICCL, MRV_VI_ICCL_DF);
	udelay(1);
	/* Disable All Interrupt */
	mrv_writel(pcdev, MRV_MI + MI_IMSC, 0);
	mrv_writel(pcdev, MRV_ISP + ISP_IMSC, 0);
}

static int configure_geometry(struct mrv_cam_dev *pcdev, u32 width,
		u32 height, enum v4l2_mbus_pixelcode code)
{
	u32 val;
	unsigned long flags;

	mrv_reset(pcdev);

	spin_lock_irqsave(&pcdev->lock, flags);

	/* Set ISP_ACQ_PROP: CCIR_SEQ */
	val = mrv_readl(pcdev, MRV_ISP + ISP_ACQ_PROP);

	switch (code) {
		case V4L2_MBUS_FMT_VYUY8_2X8:
			val |= MRV_ACQ_CCIR_VYUY;
			break;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			val |= MRV_ACQ_CCIR_UYVY;
			break;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			val |= MRV_ACQ_CCIR_YVYU;
			break;
		case V4L2_MBUS_FMT_YUYV8_2X8:
			val &= ~MRV_ACQ_CCIR_VYUY;
			break;
		default:
			printk(KERN_ERR "pixelcode %d unsupported\n", code);
			spin_unlock_irqrestore(&pcdev->lock, flags);
			return -EINVAL;
	}

	val |= MRV_ACQ_CONV_422;
	mrv_writel(pcdev, MRV_ISP + ISP_ACQ_PROP, val);

	/* Set width */
	pcdev->spec.out.w = width;
	mrv_writel(pcdev, MRV_ISP + ISP_ACQ_H_SIZE, (width << 1));
	mrv_writel(pcdev, MRV_ISP + ISP_OUT_H_SIZE, width);
	/* Set height */
	pcdev->spec.out.h = height;
	mrv_writel(pcdev, MRV_ISP + ISP_ACQ_V_SIZE, height);
	mrv_writel(pcdev, MRV_ISP + ISP_OUT_V_SIZE, height);

	spin_unlock_irqrestore(&pcdev->lock, flags);

	return 0;
}

static int mrv_camera_try_bus_param(struct soc_camera_device *icd,
		unsigned char buswidth)
{
	unsigned long camera_flags;
	int ret;

	camera_flags = icd->ops->query_bus_param(icd);
	ret = soc_camera_bus_param_compatible(camera_flags, BUS_FLAGS);
	if (!ret)
		return -EINVAL;
	return 0;
}

static const struct soc_mbus_pixelfmt mrv_camera_formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_YUYV,
		.name			= "Packed YUV422 16 bit",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_2X8_PADHI,
		.order			= SOC_MBUS_ORDER_LE,
	},
};

/* This will be corrected as we get more formats */
static bool mrv_camera_packing_supported(const struct soc_mbus_pixelfmt *fmt)
{
	return (fmt->bits_per_sample == 8 &&
			fmt->packing == SOC_MBUS_PACKING_2X8_PADHI) ||
		(fmt->bits_per_sample > 8 &&
		 fmt->packing == SOC_MBUS_PACKING_EXTEND16);
}

static int mrv_camera_get_formats(struct soc_camera_device *icd,
		unsigned int idx,
		struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int formats = 0, ret;
	/* sensor format */
	enum v4l2_mbus_pixelcode code;
	/* soc camera host format */
	const struct soc_mbus_pixelfmt *fmt;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		/* No more formats */
		return ret;

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		printk(KERN_ERR "Invalid format code #%u: %x\n", idx, code);
		return -EINVAL;
	}

	/* This also checks support for the requested bits-per-sample */
	ret = mrv_camera_try_bus_param(icd, fmt->bits_per_sample);
	if (ret < 0) {
		printk(KERN_ERR "Fail to try the bus parameters.\n");
		return ret;
	}

	switch (code) {
		case V4L2_MBUS_FMT_UYVY8_2X8:
		case V4L2_MBUS_FMT_VYUY8_2X8:
		case V4L2_MBUS_FMT_YUYV8_2X8:
		case V4L2_MBUS_FMT_YVYU8_2X8:
			formats++;
			if (xlate) {
				xlate->host_fmt	= &mrv_camera_formats[0];
				xlate->code = code;
				xlate++;
				printk(KERN_INFO "Providing format %s using code %x\n",
						mrv_camera_formats[0].name, code);
			}
			break;
		default:
			if (!mrv_camera_packing_supported(fmt))
				return -EINVAL;
			if (xlate)
				printk(KERN_INFO "Providing format %s in pass-through mode\n",
						fmt->name);
	}

	/* Generic pass-through */
	formats++;
	if (xlate) {
		xlate->host_fmt	= fmt;
		xlate->code = code;
		xlate++;
	}

	return formats;
}

static int mrv_camera_set_fmt(struct soc_camera_device *icd,
		struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		printk(KERN_ALERT "Format %x not found\n",
				pix->pixelformat);
		return -EINVAL;
	}

	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	if (mf.code != xlate->code)
		return -EINVAL;

	pix->width		= mf.width;
	pix->height		= mf.height;
	pix->field		= mf.field;
	pix->colorspace		= mf.colorspace;
	icd->current_fmt	= xlate;

	ret = configure_geometry(pcdev, pix->width, pix->height, xlate->code);
	if (ret < 0)
		return ret;

	printk(KERN_INFO "Finally set format %dx%d\n", pix->width, pix->height);
	return ret;
}

static int mrv_camera_try_fmt(struct soc_camera_device *icd,
		struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;
	/* TODO: limit to mrv hardware capabilities */

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		printk(KERN_ERR "Format %x not found\n",
				pix->pixelformat);
		return -EINVAL;
	}

	/* limit to Nusmart MRV hardware capabilities */
	if (pix->height > MAX_SUPPORT_HEIGHT)
		pix->height = MAX_SUPPORT_HEIGHT;
	if (pix->width > MAX_SUPPORT_WIDTH)
		pix->width = MAX_SUPPORT_WIDTH;

	/* limit to sensor capabilities */
	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width	= mf.width;
	pix->height	= mf.height;
	pix->colorspace	= mf.colorspace;

	switch (mf.field) {
		case V4L2_FIELD_ANY:
			pix->field = V4L2_FIELD_NONE;
			break;
		case V4L2_FIELD_NONE:
			break;
		default:
			printk(KERN_ERR "Field type %d unsupported.\n", mf.field);
			ret = -EINVAL;
	}

	return ret;
}

static int mrv_camera_set_bus_param(struct soc_camera_device *icd, u32 pixfmt)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	unsigned long camera_flags, common_flags;
	unsigned long flags;
	int ret;
	u32 val = 0;

	camera_flags = icd->ops->query_bus_param(icd);
	common_flags = soc_camera_bus_param_compatible(camera_flags, BUS_FLAGS);
	printk(KERN_INFO "Flags cam: 0x%lx host: 0x%x common: 0x%lx\n",
			camera_flags, BUS_FLAGS, common_flags);

	if (!common_flags)
		return -EINVAL;

	/* Make choises, based on platform preferences */
	if ((common_flags & SOCAM_HSYNC_ACTIVE_HIGH) &&
			(common_flags & SOCAM_HSYNC_ACTIVE_LOW)) {
		if (!pcdev->pdata || pcdev->pdata->flags & MRV_CAM_HSYNC_H)
			common_flags &= ~SOCAM_HSYNC_ACTIVE_LOW;
		else
			common_flags &= ~SOCAM_HSYNC_ACTIVE_HIGH;
	}

	if ((common_flags & SOCAM_VSYNC_ACTIVE_HIGH) &&
			(common_flags & SOCAM_VSYNC_ACTIVE_LOW)) {
		if (!pcdev->pdata || pcdev->pdata->flags & MRV_CAM_VSYNC_L)
			common_flags &= ~SOCAM_VSYNC_ACTIVE_HIGH;
		else
			common_flags &= ~SOCAM_VSYNC_ACTIVE_LOW;
	}

	if ((common_flags & SOCAM_PCLK_SAMPLE_RISING) &&
			(common_flags & SOCAM_PCLK_SAMPLE_FALLING)) {
		if (!pcdev->pdata || pcdev->pdata->flags & MRV_CAM_PCLK_SAMP_R)
			common_flags &= ~SOCAM_PCLK_SAMPLE_FALLING;
		else
			common_flags &= ~SOCAM_PCLK_SAMPLE_RISING;
	}

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0) {
		printk(KERN_ERR "Camera set_bus_param(%lx) returned %d\n",
				common_flags, ret);
		return ret;
	}

	/* set ISP_ACQ_PROP */
	spin_lock_irqsave(&pcdev->lock, flags);

	val = mrv_readl(pcdev, MRV_ISP + ISP_ACQ_PROP);

	if (common_flags & SOCAM_HSYNC_ACTIVE_LOW)
		val |= MRV_ACQ_HSYNC_ACTIVE_LOW;
	if (common_flags & SOCAM_VSYNC_ACTIVE_LOW)
		val |= MRV_ACQ_VSYNC_ACTIVE_LOW;
	if (common_flags & SOCAM_PCLK_SAMPLE_RISING)
		val |= MRV_ACQ_PIXCLK_ACTIVE_RISING;

	mrv_writel(pcdev, MRV_ISP + ISP_ACQ_PROP, val);

	spin_unlock_irqrestore(&pcdev->lock, flags);

	return 0;

}

static unsigned int mrv_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int mrv_camera_querycap(struct soc_camera_host *ici,
		struct v4l2_capability *cap)
{
	/* cap->name is set by the friendly caller:-> */
	strcpy(cap->driver, "mrv-camera");
	strlcpy(cap->card, "Nusmart Camera Interface", sizeof(cap->card));
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int mrv_camera_get_crop(struct soc_camera_device *icd,
		struct v4l2_crop *crop)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	unsigned long flags;

	if (!crop)
		return -EINVAL;

	spin_lock_irqsave(&pcdev->lock, flags);

	crop->c.left = mrv_readl(pcdev, MRV_ISP + ISP_OUT_H_OFFS_SHD);
	crop->c.top = mrv_readl(pcdev, MRV_ISP + ISP_OUT_V_OFFS_SHD);
	crop->c.width = mrv_readl(pcdev, MRV_ISP + ISP_OUT_H_SIZE_SHD);
	crop->c.height = mrv_readl(pcdev, MRV_ISP + ISP_OUT_V_SIZE_SHD);

	spin_unlock_irqrestore(&pcdev->lock, flags);

	return 0;
}

static int mrv_camera_set_crop(struct soc_camera_device *icd,
		struct v4l2_crop *crop)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	unsigned long flags;

	if (!crop)
		return -EINVAL;
	spin_lock_irqsave(&pcdev->lock, flags);

	if (MRV_ISP_BUSY == pcdev->state) {

		mrv_writel(pcdev, MRV_ISP + ISP_OUT_H_OFFS, crop->c.left);
		mrv_writel(pcdev, MRV_ISP + ISP_OUT_V_OFFS, crop->c.top);

	}

	mrv_writel(pcdev, MRV_ISP + ISP_OUT_H_OFFS, crop->c.left);
	mrv_writel(pcdev, MRV_ISP + ISP_OUT_V_OFFS, crop->c.top);
	mrv_writel(pcdev, MRV_ISP + ISP_OUT_H_SIZE, crop->c.width);
	mrv_writel(pcdev, MRV_ISP + ISP_OUT_V_SIZE, crop->c.height);

	spin_unlock_irqrestore(&pcdev->lock, flags);

	return 0;
}

static int mrv_camera_set_livecrop(struct soc_camera_device *icd,
		struct v4l2_crop *crop)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	unsigned long flags;

	if (!crop)
		return -EINVAL;

	spin_lock_irqsave(&pcdev->lock, flags);

	mrv_writel(pcdev, MRV_ISP + ISP_OUT_H_OFFS, crop->c.left);
	mrv_writel(pcdev, MRV_ISP + ISP_OUT_V_OFFS, crop->c.top);
	mrv_writel(pcdev, MRV_ISP + ISP_OUT_H_SIZE, crop->c.width);
	mrv_writel(pcdev, MRV_ISP + ISP_OUT_V_SIZE, crop->c.height);

	spin_unlock_irqrestore(&pcdev->lock, flags);

	return 0;
}

static const struct v4l2_queryctrl mrv_camera_controls[] = {
	{
		.id		= V4L2_CID_NUSMART_CAMERA_DEV_INPUT,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Input path select",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_NUSMART_STREAM_OUTPUT_PATH,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Output path select",
		.minimum	= 1,
		.maximum	= 2,
		.step		= 1,
		.default_value	= 2,
	},
	{
		.id		= V4L2_CID_NUSMART_STREAM_OUTPUT_SIZE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Set output size",
	},
};

static int mrv_camera_set_ctrl(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;

	if (!ctrl) {
		printk(KERN_ERR "v4l2_control is NULL\n");
		return -EINVAL;
	}

	switch (ctrl->id) {
		case V4L2_CID_NUSMART_CAMERA_DEV_INPUT:
			pcdev->spec.input_mux = ctrl->value;
			break;
		case V4L2_CID_NUSMART_STREAM_OUTPUT_PATH:
			pcdev->spec.path_mux = ctrl->value;
			break;
		case V4L2_CID_NUSMART_STREAM_OUTPUT_SIZE:
			pcdev->spec.out.w = (ctrl->value >> 16);
			pcdev->spec.out.h = (ctrl->value & 0xffff);
			break;
		default:
			return -ENOIOCTLCMD;
	}

	return 0;
}

#if 0
static int mrv_camera_suspend(struct soc_camera_device *icd, pm_message_t state)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	int ret = 0;

	/* disable isp */
	/*need to be add*/

	if ((pcdev->icd) && (pcdev->icd->ops->suspend))
		ret = pcdev->icd->ops->suspend(pcdev->icd, state);

	return ret;
}

static int mrv_camera_resume(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mrv_cam_dev *pcdev = ici->priv;
	int ret = 0;

	/* enable isp */
	/*need to be add*/

	if ((pcdev->icd) && (pcdev->icd->ops->resume))
		ret = pcdev->icd->ops->resume(pcdev->icd);

	/* Restart frame capture if active buffer exists */
	if (!ret && pcdev->active)

		return ret;
}
#endif

static struct soc_camera_host_ops mrv_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add		= mrv_camera_add_device,
	.remove		= mrv_camera_remove_device,
	.set_fmt	= mrv_camera_set_fmt,
	.try_fmt	= mrv_camera_try_fmt,
	.get_formats	= mrv_camera_get_formats,
	.set_bus_param	= mrv_camera_set_bus_param,
	.init_videobuf2	= mrv_camera_init_videobuf,
	.poll		= mrv_camera_poll,
	.querycap	= mrv_camera_querycap,
	.get_crop	= mrv_camera_get_crop,
	.set_crop	= mrv_camera_set_crop,
	.set_livecrop	= mrv_camera_set_livecrop,
	.controls	= mrv_camera_controls,
	.num_controls	= ARRAY_SIZE(mrv_camera_controls),
	.set_ctrl	= mrv_camera_set_ctrl,
	.suspend	= NULL,
	.resume		= NULL,
};

static int __devinit mrv_camera_probe(struct platform_device *pdev)
{
	struct mrv_cam_dev *pcdev;
	struct resource *res;
	unsigned int irq;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || irq < 0) {
		printk(KERN_ERR "Not enough MRV platform resources\n");
		err = -ENODEV;
		goto exit;
	}

	pcdev = kzalloc(sizeof(struct mrv_cam_dev), GFP_KERNEL);
	if (!pcdev) {
		printk(KERN_ERR "Could not allocate mrv_cam_dev\n");
		err = -ENOMEM;
		goto exit;
	}

	pcdev->clk = clk_get(NULL, ISP_CLK_NAME);
	if (IS_ERR(pcdev->clk)) {
		err = PTR_ERR(pcdev->clk);
		printk(KERN_ERR "Could not get clk\n");
		goto exit_kfree;
	}
	err = clk_enable(pcdev->clk);
	if (err)
		goto exit_clk;
	clk_disable(pcdev->clk);

	pcdev->pdata = pdev->dev.platform_data;
	if (pcdev->pdata)
		pcdev->mclk = pcdev->pdata->mclk_10khz * 10000;
	if (!pcdev->mclk) {
		printk(KERN_INFO "mclk == 0! Please, fix your platform data. "
				"Using default 200MHz\n");
		pcdev->mclk = 200000000;
	}

	pcdev->active = NULL;
	spin_lock_init(&pcdev->lock);
	init_waitqueue_head(&pcdev->stop_wq);
	INIT_LIST_HEAD(&pcdev->capture);

	pcdev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(pcdev->alloc_ctx)) {
		PTR_ERR(pcdev->alloc_ctx);
		printk(KERN_ERR "alloc_ctx failed\n");
		goto exit_clk;
	}

	pcdev->reg_base = ioremap(res->start, resource_size(res));
	if (!pcdev->reg_base) {
		err = -ENOMEM;
		printk(KERN_ERR "ioremap failed\n");
		goto exit_alloc_ctx;
	}
	printk(KERN_INFO "ISP base: 0x%x\n", pcdev->reg_base);
	pcdev->irq_isp = irq;
	pcdev->irq_mi = irq + 1;


	/* request irq */
	err = request_irq(pcdev->irq_isp, mrv_isp_irq,
			IRQF_DISABLED, "mrv-isp", pcdev);
	if (err) {
		printk(KERN_ERR "MRV ISP interrupt register failed\n");
		goto exit_iounmap;
	}

	err = request_irq(pcdev->irq_mi, mrv_mi_irq,
			IRQF_DISABLED, "mrv-mi", pcdev);
	if (err) {
		printk(KERN_ERR "MRV MI interrupt register failed\n");
		goto exit_free_isp_irq;
	}

	pcdev->ici.drv_name = CAM_DRV_NAME;
	pcdev->ici.ops = &mrv_soc_camera_host_ops;
	pcdev->ici.priv = pcdev;
	pcdev->ici.v4l2_dev.dev = &pdev->dev;
	pcdev->ici.nr = pdev->id;

	err = soc_camera_host_register(&pcdev->ici);
	if (err)
		goto exit_free_mi_irq;

	printk(KERN_INFO "MRV Camera driver loaded\n");
	return 0;

exit_free_mi_irq:
	free_irq(pcdev->irq_mi, pcdev);
exit_free_isp_irq:
	free_irq(pcdev->irq_isp, pcdev);
exit_iounmap:
	iounmap(pcdev->reg_base);
exit_alloc_ctx:
	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);
exit_clk:
	clk_put(pcdev->clk);
exit_kfree:
	kfree(pcdev);
exit:
	return err;
}

static int __devexit mrv_camera_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct mrv_cam_dev *pcdev = container_of(soc_host,
			struct mrv_cam_dev, ici);

	soc_camera_host_unregister(soc_host);
	free_irq(pcdev->irq_mi, pcdev);
	free_irq(pcdev->irq_isp, pcdev);
	iounmap(pcdev->reg_base);
	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);
	clk_put(pcdev->clk);
	kfree(pcdev);

	printk(KERN_INFO "MRV Camera driver unloaded\n");

	return 0;
}

static struct platform_driver mrv_camera_driver = {
	.probe = mrv_camera_probe,
	.remove = __devexit_p(mrv_camera_remove),
	.driver = {
		.name = CAM_DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init mrv_camera_init(void)
{
	return platform_driver_register(&mrv_camera_driver);
}

static void __exit mrv_camera_exit(void)
{
	platform_driver_unregister(&mrv_camera_driver);
}

module_init(mrv_camera_init);
module_exit(mrv_camera_exit);

MODULE_DESCRIPTION("MRV SoC Camera Host driver");
MODULE_AUTHOR("Wei Xing <weixing@nufront.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" CAM_DRV_NAME);
