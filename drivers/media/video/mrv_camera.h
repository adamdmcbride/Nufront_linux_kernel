/**
 * Define the registers of CamerIC Lite(Silicon Image)
 * Create by weixing@nufront.com
 */

#ifndef __MRV_CAMERA_H__
#define __MRV_CAMERA_H__

#define MRV_IE                          0x0200
#define MRV_SI                          0x0300
#define MRV_ISP                         0x0400
#define MRV_CPROC                       0x0800
#define MRV_MRSZ                        0x0C00
#define MRV_SRSZ                        0x1000
#define MRV_MI                          0x1400

#define VI_CCL                          0x0000
#define VI_ID                           0x0008
#define VI_ICCL                         0x0010
#define VI_IRCL                         0x0014
#define VI_DPCL                         0x0018

#define IE_CTRL                         0x0000
#define IE_COLOR_SEL                    0x0004
#define IE_MAT_1                        0x0008
#define IE_MAT_2                        0x000C
#define IE_MAT_3                        0x0010
#define IE_MAT_4                        0x0014
#define IE_MAT_5                        0x0018
#define IE_TINT                         0x001C
#define IE_CTRL_SHD                     0x0020
#define IE_SHARPEN                      0x0024

#define SI_CTRL                         0x0000
#define SI_OFFSET_X                     0x0004
#define SI_OFFSET_Y                     0x0008
#define SI_COLOR_Y                      0x000C
#define SI_COLOR_CB                     0x0010
#define SI_COLOR_CR                     0x0014

#define ISP_CTRL                        0x0000
#define ISP_ACQ_PROP                    0x0004
#define ISP_ACQ_H_OFFS                  0x0008
#define ISP_ACQ_V_OFFS                  0x000C
#define ISP_ACQ_H_SIZE                  0x0010
#define ISP_ACQ_V_SIZE                  0x0014
#define ISP_ACQ_NR_FRAMES               0x0018
#define ISP_OUT_H_OFFS                  0x0194
#define ISP_OUT_V_OFFS                  0x0198
#define ISP_OUT_H_SIZE                  0x019C
#define ISP_OUT_V_SIZE                  0x01A0
#define ISP_FLAGS_SHD                   0x01A8
#define ISP_OUT_H_OFFS_SHD              0x01AC
#define ISP_OUT_V_OFFS_SHD              0x01B0
#define ISP_OUT_H_SIZE_SHD              0x01B4
#define ISP_OUT_V_SIZE_SHD              0x01B8
#define ISP_IMSC                        0x01BC
#define ISP_RIS                         0x01C0
#define ISP_MIS                         0x01C4
#define ISP_ICR                         0x01C8
#define ISP_ISR                         0x01CC
#define ISP_ERR                         0x023C
#define ISP_ERR_CLR                     0x0240
#define ISP_FRAME_COUNT                 0x0244

#define CPROC_CTRL                      0x0000
#define CPROC_CONTRAST                  0x0004
#define CPROC_BRIGHTNESS                0x0008
#define CPROC_SATURATION                0x000C
#define CPROC_HUE                       0x0010

#define MRSZ_CTRL                       0x0000
#define MRSZ_SCALE_HY                   0x0004
#define MRSZ_SCALE_HCB                  0x0008
#define MRSZ_SCALE_HCR                  0x000C
#define MRSZ_SCALE_VY                   0x0010
#define MRSZ_SCALE_VC                   0x0014
#define MRSZ_PHASE_HY                   0x0018
#define MRSZ_PHASE_HC                   0x001C
#define MRSZ_PHASE_VY                   0x0020
#define MRSZ_PHASE_VC                   0x0024
#define MRSZ_SCALE_LUT_ADDR             0x0028
#define MRSZ_SCALE_LUT                  0x002C
#define MRSZ_CTRL_SHD                   0x0030
#define MRSZ_SCALE_HY_SHD               0x0034
#define MRSZ_SCALE_HCB_SHD              0x0038
#define MRSZ_SCALE_HCR_SHD              0x003C
#define MRSZ_SCALE_VY_SHD               0x0040
#define MRSZ_SCALE_VC_SHD               0x0044
#define MRSZ_PHASE_HY_SHD               0x0048
#define MRSZ_PHASE_HC_SHD               0x004C
#define MRSZ_PHASE_VY_SHD               0x0050
#define MRSZ_PHASE_VC_SHD               0x0054

#define SRSZ_CTRL                       0x0000
#define SRSZ_SCALE_HY                   0x0004
#define SRSZ_SCALE_HCB                  0x0008
#define SRSZ_SCALE_HCR                  0x000C
#define SRSZ_SCALE_VY                   0x0010
#define SRSZ_SCALE_VC                   0x0014
#define SRSZ_PHASE_HY                   0x0018
#define SRSZ_PHASE_HC                   0x001C
#define SRSZ_PHASE_VY                   0x0020
#define SRSZ_PHASE_VC                   0x0024
#define SRSZ_SCALE_LUT_ADDR             0x0028
#define SRSZ_SCALE_LUT                  0x002C
#define SRSZ_CTRL_SHD                   0x0030
#define SRSZ_SCALE_HY_SHD               0x0034
#define SRSZ_SCALE_HCB_SHD              0x0038
#define SRSZ_SCALE_HCR_SHD              0x003C
#define SRSZ_SCALE_VY_SHD               0x0040
#define SRSZ_SCALE_VC_SHD               0x0044
#define SRSZ_PHASE_HY_SHD               0x0048
#define SRSZ_PHASE_HC_SHD               0x004C
#define SRSZ_PHASE_VY_SHD               0x0050
#define SRSZ_PHASE_VC_SHD               0x0054

#define MI_CTRL                         0x0000
#define MI_INIT                         0x0004
#define MI_MP_Y_BASE_AD_INIT            0x0008
#define MI_MP_Y_SIZE_INIT               0x000C
#define MI_MP_Y_OFFS_CNT_INIT           0x0010
#define MI_MP_Y_OFFS_CNT_START          0x0014
#define MI_MP_Y_IRQ_OFFS_INIT           0x0018
#define MI_MP_CB_BASE_AD_INIT           0x001C
#define MI_MP_CB_SIZE_INIT              0x0020
#define MI_MP_CB_OFFS_CNT_INIT          0x0024
#define MI_MP_CB_OFFS_CNT_START         0x0028
#define MI_MP_CR_BASE_AD_INIT           0x002C
#define MI_MP_CR_SIZE_INIT              0x0030
#define MI_MP_CR_OFFS_CNT_INIT          0x0034
#define MI_MP_CR_OFFS_CNT_START         0x0038
#define MI_SP_Y_BASE_AD_INIT            0x003C
#define MI_SP_Y_SIZE_INIT               0x0040
#define MI_SP_Y_OFFS_CNT_INIT           0x0044
#define MI_SP_Y_OFFS_CNT_START          0x0048
#define MI_SP_Y_LLENGTH                 0x004C
#define MI_SP_CB_BASE_AD_INIT           0x0050
#define MI_SP_CB_SIZE_INIT              0x0054
#define MI_SP_CB_OFFS_CNT_INIT          0x0058
#define MI_SP_CB_OFFS_CNT_START         0x005C
#define MI_SP_CR_BASE_AD_INIT           0x0060
#define MI_SP_CR_SIZE_INIT              0x0064
#define MI_SP_CR_OFFS_CNT_INIT          0x0068
#define MI_SP_CR_OFFS_CNT_START         0x006C
#define MI_BYTE_CNT                     0x0070
#define MI_CTRL_SHD                     0x0074
#define MI_MP_Y_BASE_AD_SHD             0x0078
#define MI_MP_Y_SIZE_SHD                0x007C
#define MI_MP_Y_OFFS_CNT_SHD            0x0080
#define MI_MP_Y_IRQ_OFFS_SHD            0x0084
#define MI_MP_CB_BASE_AD_SHD            0x0088
#define MI_MP_CB_SIZE_SHD               0x008C
#define MI_MP_CB_OFFS_CNT_SHD           0x0090
#define MI_MP_CR_BASE_AD_SHD            0x0094
#define MI_MP_CR_SIZE_SHD               0x0098
#define MI_MP_CR_OFFS_CNT_SHD           0x009C
#define MI_SP_Y_BASE_AD_SHD             0x00A0
#define MI_SP_Y_SIZE_SHD                0x00A4
#define MI_SP_Y_OFFS_CNT_SHD            0x00A8
#define MI_SP_Y_IRQ_OFFS_SHD            0x00AC
#define MI_SP_CB_BASE_AD_SHD            0x00B0
#define MI_SP_CB_SIZE_SHD               0x00B4
#define MI_SP_CB_OFFS_CNT_SHD           0x00B8
#define MI_SP_CR_BASE_AD_SHD            0x00BC
#define MI_SP_CR_SIZE_SHD               0x00C0
#define MI_SP_CR_OFFS_CNT_SHD           0x00C4

#define MI_DMA_Y_PIC_START_AD           0x00C8
#define MI_DMA_Y_PIC_WIDTH              0x00CC
#define MI_DMA_Y_LLENGTH                0x00D0
#define MI_DMA_Y_PIC_SIZE               0x00D4
#define MI_DMA_CB_PIC_START_AD          0x00D8
#define MI_DMA_CR_PIC_START_AD          0x00E8
#define MI_IMSC                         0x00F8
#define MI_RIS                          0x00FC
#define MI_MIS                          0x0100
#define MI_ICR                          0x0104
#define MI_ISR                          0x0108
#define MI_STATUS                       0x010C
#define MI_STATUS_CLR                   0x0110
#define MI_SP_Y_PIC_WIDTH               0x0114
#define MI_SP_Y_PIC_HEIGHT              0x0118
#define MI_SP_Y_PIC_SIZE                0x011C
#define MI_DMA_CTRL                     0x0120
#define MI_DMA_START                    0x0124
#define MI_DMA_STATUS                   0x0128
#define MI_PIXEL_CNT                    0x012C


#define MRV_VI_CCL_DIS			(1 << 2)
#define MRV_VI_CCL_ENA			(0 << 2)

#define MRV_VI_IRCL_HW			(1 << 7)
#define MRV_VI_IRCL_MI			(1 << 6)
#define MRV_VI_IRCL_SR			(1 << 4)
#define MRV_VI_IRCL_MR			(1 << 3)
#define MRV_VI_IRCL_ISP			(1 << 0)

#define MRV_VI_ICCL_SI			(1 << 9)
#define MRV_VI_ICCL_IE			(1 << 8)
#define MRV_VI_ICCL_MI			(1 << 6)
#define MRV_VI_ICCL_SR			(1 << 4)
#define MRV_VI_ICCL_MR			(1 << 3)
#define MRV_VI_ICCL_CP			(1 << 1)
#define MRV_VI_ICCL_ISP			(1 << 0)
#define MRV_VI_ICCL_DF			MRV_VI_ICCL_MI |\
					MRV_VI_ICCL_SR |\
					MRV_VI_ICCL_MR |\
					MRV_VI_ICCL_ISP

#define MRV_DPCL_DEFAULT		0x10d
#define MRV_DPCL_INPUT_BIT		8
#define MRV_DPCL_CHMOD_BIT		2
#define MRV_DPCL_MPMUX_BIT		0

#define MRV_ISP_GEN_UPD			(1 << 10)
#define MRV_ISP_CFG_UPD			(1 << 9)
#define MRV_ISP_INF_ENA			(1 << 4)
#define MRV_ISP_MODE			(1 << 2)
#define MRV_ISP_ENABLE			(1 << 0)

#define MRV_CFG_ISP_DIS			0x414
#define MRV_CFG_ISP_UPD			0x415
#define MRV_CFG_ISP_ENA			0x615

#define MRV_ACQ_CCIR_VYUY		(3 << 7)
#define MRV_ACQ_CCIR_UYVY		(2 << 7)
#define MRV_ACQ_CCIR_YVYU		(1 << 7)
#define MRV_ACQ_CONV_422		(1 << 5)

#define MRV_ACQ_VSYNC_ACTIVE_LOW	(1 << 2)
#define MRV_ACQ_HSYNC_ACTIVE_LOW	(1 << 1)
#define MRV_ACQ_PIXCLK_ACTIVE_RISING	(1 << 0)

#define MRV_SCALING_UP(i,o)		((((i)-1)<<16)/((o)-1))
#define MRV_SCALING_DW(i,o)		(((((o)-1)<<16)/((i)-1))+1)
#define MRV_SCALE_DIS			0x100
#define MRV_SCALE_UPD			(1 << 8)
#define MRV_SCALE_UP_VC			(1 << 7)
#define MRV_SCALE_UP_VY			(1 << 6)
#define MRV_SCALE_UP_HC			(1 << 5)
#define MRV_SCALE_UP_HY			(1 << 4)
#define MRV_SCALE_EN_VC			(1 << 3)
#define MRV_SCALE_EN_VY			(1 << 2)
#define MRV_SCALE_EN_HC			(1 << 1)
#define MRV_SCALE_EN_HY			(1 << 0)

#define MRV_MI_SP_OUT_888_FMT		(6 << 28) //RGB 565
#define MRV_MI_SP_OUT_565_FMT		(4 << 28) //RGB 565
#define MRV_MI_SP_OUT_422_FMT		(2 << 28) //YUV 422
#define MRV_MI_SP_OUT_420_FMT		(1 << 28) //YUV 420
#define MRV_MI_SP_IN_422_FMT		(2 << 26) //YUV 422
#define MRV_MI_SP_IN_420_FMT		(1 << 26) //YUV 420
#define MRV_MI_SP_OUT_SEMI		(1 << 24) //semi planar
#define MRV_MI_MP_OUT_SEMI		(1 << 22) //semi planar
#define MRV_MI_INIT_OFFS_ENA		(1 << 21)
#define MRV_MI_INIT_BASE_ENA		(1 << 20)

#define MRV_MI_INIT_ENA			(1 << 4)
#define MRV_MI_SKIP_ENA			(1 << 2)

#define MRV_INT_VSYNC_MASK		(1 << 6)
#define MRV_INT_FRIN_MASK		(1 << 5)
#define MRV_INT_PIC_SIZE_ERR		(1 << 3)
#define MRV_INT_DATA_LOSS		(1 << 2)
#define MRV_INT_FROUT_MASK		(1 << 1)

#define MRV_INT_MP_MASK			(1 << 0)
#define MRV_INT_SP_MASK			(1 << 1)

#endif
