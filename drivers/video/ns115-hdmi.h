/*
 * NuSmart High-Definition Multimedia Interface (HDMI)
 *
 * Copyright (C) 2012, wangzhi <zhi.wang@nufront.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef NS115_HDMI_H
#define NS115_HDMI_H

#include <linux/fb.h>
#include <linux/switch.h>

/*
 * flags format
 *
 * 0x0000000A
 *
 * A: Audio source select
 */

/* Audio source select */
#define HDMI_SND_SRC_MASK	(0xF << 0)
#define HDMI_SND_SRC_I2S	(0 << 0) /* default */
#define HDMI_SND_SRC_SPDIF	(1 << 0)
#define HDMI_SND_SRC_DSD	(2 << 0)
#define HDMI_SND_SRC_HBR	(3 << 0)

#define TPI_PIX_CLK_LSB						(0x00)
#define TPI_PIX_CLK_MSB						(0x01)

#define TPI_SYSTEM_CONTROL_DATA_REG			(0x1A)
#define TPI_DEVICE_POWER_STATE_CTRL_REG		(0x1E)

#define TX_POWER_STATE_MASK					(BIT_1 | BIT_0)
#define TX_POWER_STATE_D0					(0x00)
#define TX_POWER_STATE_D1					(0x01)
#define TX_POWER_STATE_D2					(0x02)
#define TX_POWER_STATE_D3					(0x03)

// Audio Configuration Regiaters
//==============================
#define TPI_AUDIO_INTERFACE_REG				(0x26)
#define TPI_AUDIO_HANDLING					(0x25)

#define TPI_AUDIO_SAMPLE_CTRL				(0x27)

// TPI Enable Register
//====================
#define TPI_ENABLE							(0xC7)

// TPI Identification Registers
//=============================
#define TPI_DEVICE_ID						(0x1B)
#define TPI_DEVICE_REV_ID					(0x1C)
#define TPI_RESERVED2						(0x1D)

#define TPI_AVI_POWER_STATE_D2 (2 << 0)

#define TPI_SYS_CTRL_DDC_BUS_REQUEST (1 << 2)
#define TPI_SYS_CTRL_DDC_BUS_GRANTED (1 << 1)

#define DDC_BUS_GRANT_GRANTED				(0x02)
#define DDC_BUS_REQUEST_REQUESTED			(BIT_2)

// HDMI EDID Length
#define HDMI_EDID_MAX_LENGTH    256//128//256

#define IN_PROG                 (1 << 4)
#define DDC_CNT1                0xF0
#define DDC_CNT2                0xF1
#define DDC_STATUS	            0xF2
#define DDC_CMD	                0xF3
#define DDC_DATA                0xF4
#define DDC_FIFO_CNT            0xF5

#define CLOCK_SCL_DDC           0x0A//0b1010

#define DDC_SEGM                0xEE
#define DDC_OFFSET	            0xEF
#define DDC_DELAY_CNT           0xF6

#define MAX_SIZE_DDC_FIFOCNT    0x10//16

#define CLEAR_FIFO_DDC_CMD      0x09//0b1001

#define TPI_INTERNAL_PAGE_REG	0xBC
#define TPI_INDEXED_OFFSET_REG	0xBD
#define TPI_INDEXED_VALUE_REG	0xBE
#define DDC_ADDR	        0xED
#define FIFO_FULL               BIT_3//1<<3
//#define MAX_FIFO_SIZE_DDC       16

#define TPI_INTERRUPT_STATUS_REG			(0x3D)
#define HOT_PLUG_EVENT_MASK					(BIT_0)
#define BIT_0                   0x01
#define BIT_1                   0x02
#define BIT_2                   0x04
#define BIT_3                   0x08
#define BIT_4                   0x10
#define BIT_5                   0x20
#define BIT_6                   0x40
#define BIT_7                   0x80

#define RX_SENSE_CURRENT_STATE_BIT      BIT_3
#define HOT_PLUG_PIN_CURRENT_STATE      BIT_2
#define RECEIVER_SENSE_EVENT_ENDING     BIT_1
#define CONNECTION_EVENT_PENDING        BIT_0

#define TPI_INTERRUPT_ENABLE_REG			(0x3C)

// Interrupt Masks
//================
#define HOT_PLUG_EVENT          0x01
#define RX_SENSE_EVENT          0x02
#define HOT_PLUG_STATE          0x04
#define RX_SENSE_STATE          0x08

#define AUDIO_ERROR_EVENT       0x10
#define SECURITY_CHANGE_EVENT   0x20
#define V_READY_EVENT           0x40
#define HDCP_CHANGE_EVENT       0x80

#define NON_MASKABLE_INT		0xFF

#define HOT_PLUG_PIN_STATE_MASK				(BIT_2)
#define HOT_PLUG_PIN_STATE_LOW				(0x00)
#define HOT_PLUG_PIN_STATE_HIGH				(BIT_2)//(0x04)

#define DDC_BUS_GRANT_MASK					(BIT_1)

#define T_DDC_ACCESS                        5
#define T_DDC_STATUS                        5

#define BITS_1_0                0x03
#define BITS_2_1                0x06
#define BITS_2_1_0              0x07
#define BITS_3_2                0x0C
#define BITS_4_3_2              0x1C  
#define BITS_5_4                0x30
#define BITS_5_4_3				0x38
#define BITS_6_5                0x60
#define BITS_6_5_4              0x70
#define BITS_7_6                0xC0

#define EDID_BLOCK_0_OFFSET 0x00
#define EDID_BLOCK_1_OFFSET 0x80

#define EDID_BLOCK_SIZE                 128
#define EDID_HDR_NO_OF_FF               0x06
#define NUM_OF_EXTEN_ADDR               0x7E
#define OFFSET_BYTE_END_OF_PADING       (EDID_BLOCK_SIZE - 2)

///TMDS Control 7 Register
#define REG_TMDS_CONTROL_7          0x606
#define CEC_OSC_EN_BIT              BIT_4

///TMDS Control 9 Register
#define REG_TMDS_CONTROL_9          0x608
#define CEC_OSC_CTRL_BIT1_TO_BIT4   (0x09 << 1)//b1001

#define SCK_SAMPLE_FALLING_EDGE			0x00
#define SCK_SAMPLE_RISING_EDGE			0x80

#define MCLK_MULTIPLIER                (1 << 4)///001 ¨C 256

/*CEA-861-E page 76*/
#define AUDIO_CHANNELS_OF_2            0x01
#define AUDIO_CHANNELS_OF_3            0x02
#define AUDIO_CHANNELS_OF_4            0x03
#define AUDIO_CHANNELS_OF_5            0x04
#define AUDIO_CHANNELS_OF_6            0x05
#define AUDIO_CHANNELS_OF_7            0x06
#define AUDIO_CHANNELS_OF_8            0x07

// Audio sample rate bit definition
#define BIT_AUDIO_16BIT						0x40///0b0010  0x02
#define BIT_AUDIO_20BIT						0x80///0b1010  0x0A
#define BIT_AUDIO_24BIT						0xC0///0b1011  0x0B

#define BIT_AUDIO_32KHZ						0x08///0b0011  0x03
#define BIT_AUDIO_44_1KHZ				    0x10///0b0000  0x00
#define BIT_AUDIO_48KHZ						0x18///0b0010  0x02
#define BIT_AUDIO_88_2KHZ					0x20///0b1000  0x08
#define BIT_AUDIO_96KHZ					    0x28///0b1010  0x0A
#define BIT_AUDIO_176_4KHZ					0x30///0b1100  0x0C
#define BIT_AUDIO_192KHZ					0x38///0b1110  0x0E

// Configuration of I2S Interface
//===============================

#define TPI_I2S_EN							(0x1F)
#define TPI_I2S_IN_CFG						(0x20)

//Stream Header Settings for I2S
// Available only when TPI 0x26[7:6]=10 to select I2S input
//=========================================================
#define TPI_I2S_CHST_0						(0x21)
#define TPI_I2S_CHST_1						(0x22)
#define TPI_I2S_CHST_2						(0x23)
#define TPI_I2S_CHST_3						(0x24)
#define TPI_I2S_CHST_4						(0x25)

#define AUD_IF_I2S              0x80///bit:7_6->10

// Audio Modes
//============
#define AUD_PASS_BASIC      0x00
#define AUD_PASS_ALL        0x01
#define AUD_DOWN_SAMPLE     0x02
#define AUD_DO_NOT_CHECK    0x03

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// Indexed Register Offsets, Constants
//====================================
#define INDEXED_PAGE_0_HUH		0x00
#define INDEXED_PAGE_0			0x01
#define INDEXED_PAGE_1			0x02
#define INDEXED_PAGE_2			0x03

#define REG_HOT_PLUG_DELAY_DEBOUNCE         0x7C
#define STEPS_HOT_PLUG_DELAY_DEBOUNCE       (3.4)//ms
#define SETTING_HOT_PLUG_DELAY_DEBOUNCE     0x93//~(147*3.4)~500ms

#define TXL_PAGE_0_DEV_IDL_ADDR 0x02
#define TXL_PAGE_0_DEV_IDH_ADDR 0x03
#define SiI_DEVICE_ID           0xB4
#define SiI_DEVICE_STRING       KERN_EMERG "[HDMI:wangzhi]SiI hdmi ip core in ns115!\n"


#define DEVICE_ID_LOW_BYTE      0x02
#define DEVICE_ID_HI_BYTE       0x03
#define AUDIO_INPUT_LENGTH		0x24

#define TMDS_CONT_REG           0x82

#define SW_RESET                0x05
#define POWER_DOWN              0x6F


#define DIV_BY_2                0x00
#define MULT_BY_1               0x01
#define MULT_BY_2               0x02
#define MULT_BY_4               0x03

//Audio Maps
//==========
#define REG_I2S_CHSTATUS_0				0x21
#define REG_I2S_CHSTATUS_1				0x22
#define REG_I2S_CHSTATUS_2				0x23
#define REG_I2S_CHSTATUS_3				0x24
#define REG_I2S_CHSTATUS_4				(0x25)//TPI_AUDIO_HANDLING

#define SIZE_AUDIO_INFOFRAME		0x0E//0x0F // Audio (n = 14 bytes); according to PR.

// Audio Configure bit definition
#define BIT_AUDIO_PCM						0x01
#define BIT_AUDIO_MUTE						BIT_4//0x10
#define BIT_AUDIO_I2S						0x80

// Misc InfoFrames
//================
#define MISC_INFO_FRAMES_CTRL				(0xBF)
#define MISC_INFO_FRAMES_TYPE				(0xC0)
#define MISC_INFO_FRAMES_VER				(0xC1)
#define MISC_INFO_FRAMES_LEN				(0xC2)
#define MISC_INFO_FRAMES_CHKSUM				(0xC3)

/* HDMI_TPI_AUDIO_CONFIG_BYTE2_REG */
#define TPI_AUDIO_CODING_STREAM_HEADER (0 << 0)
#define TPI_AUDIO_CODING_PCM (1 << 0)
#define TPI_AUDIO_CODING_AC3 (2 << 0)
#define TPI_AUDIO_CODING_MPEG1 (3 << 0)
#define TPI_AUDIO_CODING_MP3 (4 << 0)
#define TPI_AUDIO_CODING_MPEG2 (5 << 0)
#define TPI_AUDIO_CODING_AAC (6 << 0)
#define TPI_AUDIO_CODING_DTS (7 << 0)
#define TPI_AUDIO_CODING_ATRAC (8 << 0)
#define TPI_AUDIO_MUTE_DISABLE (0 << 4)
#define TPI_AUDIO_MUTE_ENABLE (1 << 4)
#define TPI_AUDIO_INTERFACE_DISABLE (0 << 6)
#define TPI_AUDIO_INTERFACE_SPDIF (1 << 6)
#define TPI_AUDIO_INTERFACE_I2S (2 << 6)

/* HDMI_TPI_AUDIO_CONFIG_BYTE2_REG */
#define TPI_AUDIO_CODING_STREAM_HEADER (0 << 0)
#define TPI_AUDIO_CODING_PCM (1 << 0)
#define TPI_AUDIO_CODING_AC3 (2 << 0)
#define TPI_AUDIO_CODING_MPEG1 (3 << 0)
#define TPI_AUDIO_CODING_MP3 (4 << 0)
#define TPI_AUDIO_CODING_MPEG2 (5 << 0)
#define TPI_AUDIO_CODING_AAC (6 << 0)
#define TPI_AUDIO_CODING_DTS (7 << 0)
#define TPI_AUDIO_CODING_ATRAC (8 << 0)
#define TPI_AUDIO_MUTE_DISABLE (0 << 4)
#define TPI_AUDIO_MUTE_ENABLE (1 << 4)
#define TPI_AUDIO_INTERFACE_DISABLE (0 << 6)
#define TPI_AUDIO_INTERFACE_SPDIF (1 << 6)
#define TPI_AUDIO_INTERFACE_I2S (2 << 6)

#define EN_AND_RPT_AUDIO			0xC2
#define DISABLE_AUDIO				0x02

// InfoFrames types according to CEA-861-E spec
#define TYPE_HDMI_VSIF_INFOFRAMES   0x81
#define TYPE_AVI_INFOFRAMES			0x82
#define TYPE_SPD_INFOFRAMES       	0x83
#define TYPE_AUDIO_INFOFRAMES       0x84
#define TYPE_MPEG_INFOFRAMES		0x85
#define TYPE_GBD_INFOFRAMES       	0x0A
#define TYPE_ISRC1_INFOFRAMES		0x05
#define TYPE_ISRC2_INFOFRAMES		0x06

#define AVI_INFOFRAMES_VERSION      0x02
#define MISC_INFOFRAMES_VERSION     0x01

// Audio Infoframe header definition, refer to HDM spec 1.3, IEC60958, CEA-861-D
#define EN_AUDIO_INFOFRAMES					0xC2
#define TYPE_AUDIO_INFOFRAMES				0x84
#define AUDIO_INFOFRAMES_VERSION			0x01
#define AUDIO_INFOFRAMES_LENGTH				0x0A
#define AUDIO_INFOFRAME_HEADER_LEN			14

#define REG_INTERRUPT_STATUS			0x3D
#define REG_INTERRUPT_CTRL				0x3C

// Interrupt Control bit definition
#define BIT_HOTPLUG_ENABLE					0x01

// Register of Sil9022/24 definition
#define REG_SYSTEM_CTRL					0x1A

#define DDC_FIFOCNT      0xF5

#define ESTABLISHED_TIMING_INDEX        35      // Offset of Established Timing in EDID block

#define VERTICAL_FREQ 0x3C
#define HDMI_TPI_VIDEO_DATA_BASE_REG 0x00
#define TPI_AVI_PIXEL_REP_TCLLKSEL_x1 (01 << 6)
#define TPI_AVI_PIXEL_REP_BUS_24BIT (1 << 5)
#define TPI_AVI_PIXEL_REP_FALLING_EDGE (0 << 4)
#define TPI_AVI_PIXEL_REP_NONE (0 << 0)
#define TPI_AVI_PIXEL_REP_NONE (0 << 0)
#define HDMI_TPI_PIXEL_REPETITION_REG 0x08
#define TPI_AVI_INPUT_BITMODE_8BIT (0 << 7)
#define TPI_AVI_INPUT_RANGE_AUTO (0 << 2)
#define TPI_AVI_INPUT_COLORSPACE_RGB (0 << 0)

#define HDMI_TPI_AVI_INOUT_BASE_REG 0x09

#define HDMI_TPI_AVI_IN_FORMAT_REG (HDMI_TPI_AVI_INOUT_BASE_REG + 0x00)
//#define TPI_AVI_INPUT_COLORSPACE_RGB (0 << 0)
#define HDMI_TPI_AVI_DBYTE_BASE_REG 0x0C
#define HDMI_SYS_CTRL_DATA_REG 0x1A
#define TPI_AVI_OUTPUT_CONV_BT709 (1 << 4)
#define HDMI_TPI_AVI_OUT_FORMAT_REG (HDMI_TPI_AVI_INOUT_BASE_REG + 0x01)
#define TPI_AVI_OUTPUT_RANGE_AUTO (0 << 2)
#define TPI_AVI_OUTPUT_COLORSPACE_RGBHDMI (0 << 0)
#define TPI_SYS_CTRL_POWER_DOWN (1 << 4)

// InfoFrames types according to CEA-861-E spec
#define TYPE_HDMI_VSIF_INFOFRAMES   0x81
#define TYPE_AVI_INFOFRAMES			0x82
#define TYPE_SPD_INFOFRAMES       	0x83
#define TYPE_AUDIO_INFOFRAMES       0x84
#define TYPE_MPEG_INFOFRAMES		0x85
#define TYPE_GBD_INFOFRAMES       	0x0A
#define TYPE_ISRC1_INFOFRAMES		0x05
#define TYPE_ISRC2_INFOFRAMES		0x06

#define INFOFRAMES_BUFF_LENGTH      0x1F
#define SIZE_AVI_INFOFRAME          0x0E    // including checksum byte
#define AUDIO_INFOFRAMES_LENGTH     0x0A
#define SPD_INFOFRAMES_LENGTH     	0x19	
#define MPEG_INFOFRAME_LENGTH 		0x0A	// 10 bytes
#define MISC_INFOFRAMES_LENGTH      0x1B
#define PACKET_INFOFRAMES_LENGTH    0x1C

#define AVI_INFOFRAMES_VERSION      0x02
#define MISC_INFOFRAMES_VERSION     0x01

#define TPI_AVI_BYTE_0						(0x0C)
#define TPI_AVI_BYTE_1						(0x0D)
#define TPI_AVI_BYTE_2						(0x0E)
#define TPI_AVI_BYTE_3						(0x0F)
#define TPI_AVI_BYTE_4						(0x10)
#define TPI_AVI_BYTE_5						(0x11)
#define EN_AND_RPT_AVI				0xC0	// Not normally used.  Write to TPI 0x19 instead
#define DISABLE_AVI					0x00	// But this is used to Disable

#define TMDS_OUTPUT_CONTROL_MASK			(BIT_4)
#define TMDS_OUTPUT_CONTROL_ACTIVE			(0x00)
#define TMDS_OUTPUT_CONTROL_POWER_DOWN		(0x10)

#define AV_MUTE_MASK						(BIT_3)
#define AV_MUTE_NORMAL						(0x00)
#define AV_MUTE_MUTED						(0x08)

enum EDID_ErrorCodes
{
	EDID_OK,
	EDID_INCORRECT_HEADER,
	EDID_CHECKSUM_ERROR,
	EDID_NO_861_EXTENSIONS,
	EDID_SHORT_DESCRIPTORS_OK,
	EDID_LONG_DESCRIPTORS_OK,
	EDID_EXT_TAG_ERROR,
	EDID_REV_ADDR_ERROR,
	EDID_V_DESCR_OVERFLOW,
	EDID_UNKNOWN_TAG_CODE,
	EDID_NO_DETAILED_DESCRIPTORS,
	EDID_DDC_BUS_REQ_FAILURE,
	EDID_DDC_BUS_RELEASE_FAILURE
};

enum hotplug_state {
	HDMI_HOTPLUG_DISCONNECTED = 0,
	HDMI_HOTPLUG_CONNECTED,
	HDMI_HOTPLUG_EDID_DONE,
};

//Register Address Groups
typedef enum __hdmi_reg_addr_page__ {
	PAGE_0_HDMI = 0,
	PAGE_1_HDMI,
	PAGE_2_HDMI,
	PAGE_TPI_HDMI,
	PAGE_CEC_HDMI,
	PAGE_EHDMI_HDMI,
}hdmi_reg_addr_page;

struct ns115_hdmi_data {
	void __iomem *base;
	enum hotplug_state hp_state;    /* hot-plug status */
	hdmi_reg_addr_page page_reg;
	u16 dev_id;
	u8 preprogrammed_vic;           /* use a pre-programmed VIC or
									   the external mode */
	u8 edid_block_addr;
	u8 edid_segment_nr;
	u8 edid_blocks;

	struct clk *hdmi_clk;
	struct device *dev;
	struct fb_info *info;
	struct mutex mutex;             /* Protect the info pointer */
	u32 gpio;
	struct tasklet_struct tasklet;
	struct delayed_work     edid_work;
	struct fb_monspecs monspec;
	struct fb_var_screeninfo var;
	struct notifier_block notifier;
	struct switch_dev sdev;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	bool tmdsPoweredUp;
	u8 txPowerState;
	u32 sample_rate;        ///Audio Sample Size
	u32 channels;           ///Audio Sample Frequency
	u32 resolution;         ///Audio channel
	unsigned int irq;
	bool hdmi_sink;
	u32 horizontal_res;
	u32 vertical_res;
	u32 pixel_clk;
};

struct ns115_hdmi_lcdc_data {
	u32 horizontal_res;
	u32 vertical_res;
	u32 pixel_clk;
	u8 *pedid;
};

static bool StartTPI(struct ns115_hdmi_data *hdmi);

static void  DisableInterrupts(struct ns115_hdmi_data *hdmi,unsigned char Interrupt_Pattern);
static bool GetDDC_Access (struct ns115_hdmi_data *hdmi,u8* SysCtrlRegVal);
static bool ReleaseDDC(struct ns115_hdmi_data *hdmi,u8 SysCtrlRegVal);

static void ReadModifyWriteTPI(struct ns115_hdmi_data *hdmi,u8 Offset, u8 Mask, u8 Value);

static int ns115_hdmi_EDIDRead(struct ns115_hdmi_data *hdmi,unsigned int ui_num, 
		u8* uc_RegBuf, 
		u8 block_edid);

static void WriteByteHDMI(struct ns115_hdmi_data *hdmi,u8 RegOffset, u8 Data);
static u8 ReadByteHDMI(struct ns115_hdmi_data *hdmi,u8 RegOffset);
static void EnableInterrupts(struct ns115_hdmi_data *hdmi,u8 Interrupt_Pattern);
static void WriteIndexedRegister(struct ns115_hdmi_data *hdmi,u8 PageNum, u8 RegOffset, u8 RegValue);
static u8 ReadIndexedRegister(struct ns115_hdmi_data *hdmi,u8 PageNum, u8 RegOffset);

static void ns115_hdmi_en(struct ns115_hdmi_data *hdmi);
static void ns115_hdmi_configure(struct ns115_hdmi_data *hdmi);
static bool ns115_hdmi_must_reconfigure(struct ns115_hdmi_data *hdmi);

bool ns115_hdmi_audio_infoframe_setup(struct ns115_hdmi_data *hdmi,u8 ChannelCount, u8 CodingType, u8 SS, u8 Fs, u8 SpeakerConfig);

static void ns115_hdmi_audio_config(struct ns115_hdmi_data *hdmi);
static void ns115_hdmi_avi_infoframe_setup(struct ns115_hdmi_data *hdmi);
static void ns115_hdmi_video_config(struct ns115_hdmi_data *hdmi);
void WriteBlockHDMI(struct ns115_hdmi_data *hdmi,u8 TPI_Offset, unsigned short NBytes, u8 * pData);

static void  Writeu16HDMI(struct ns115_hdmi_data *hdmi,u16 uc_regoffset, u8 uc_bufdata);
static u8 Readu16HDMI(struct ns115_hdmi_data *hdmi,u16 uc_regoffset);
static void ns115_hdmi_tmds_clock_enable(struct ns115_hdmi_data *hdmi);
static void ns115_hdmi_tmds_clock_disable(struct ns115_hdmi_data *hdmi);
static void ns115_hdmi_set_pt(struct ns115_hdmi_data* hdmi);
static struct ns115_hdmi_data* ns115_hdmi_get_pt(void);
static void ns115_hdmi_DisableTMDS (struct ns115_hdmi_data *hdmi);
static void ns115_hdmi_EnableTMDS (struct ns115_hdmi_data *hdmi);
static void ReadSetWriteTPI(struct ns115_hdmi_data *hdmi,u8 Offset, u8 Pattern);
void hr_msleep(unsigned int msecs);
static void ns115_hdmi_lcdc_video(u32 horizontal_res,
		u32 vertical_res,
		u32 pixel_clk);
#endif
