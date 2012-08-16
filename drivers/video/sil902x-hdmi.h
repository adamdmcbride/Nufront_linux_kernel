/*
 * Base driver for SIL902X HDMI chip (I2C)
 *
 * Copyright (C) 2011 NUFRONT INC.
 *
 */

#ifndef _SIL902X_HDMI_H
#define _SIL902X_HDMI_H

// Silicon Image SIL9022/24/A Device ID definition
#define SIL9022_DEVICEID					0xB00000
#define SIL9022A_DEVICEID					0xb00203

// Silicon Image SIL9022/24/A DDC I2C address (7-bit mode)
#define DDC_I2CADDRESS						0x50

// HDMI EDID related information definition
#define HDMI_EDID_LENGTH					128
#define ROM_EDID_ADDR						0x00
#define NUM_OF_EXTEN_ADDR					0x7E

// HDMI related operation error definition
#define HDMI_RETURN_OK						0x00
#define HDMI_RETURN_ERROR					0x01
#define HDMI_RETURN_NO861_EXT				0x02

// Hot plug connection status definition
#define HOTPLUG_DISCONNECTED					0
#define HOTPLUG_CONNECTED					1
#define UNPLUG_CONFIRMED					0x03

// Earlysuspend status
#define EARLYSUSPEND_NO						0
#define EARLYSUSPEND_YES					1

// Audio Infoframe header definition, refer to HDM spec 1.3, IEC60958, CEA-861-D
#define EN_AUDIO_INFOFRAMES					0xC2
#define TYPE_AUDIO_INFOFRAMES				0x84
#define AUDIO_INFOFRAMES_VERSION			0x01
#define AUDIO_INFOFRAMES_LENGTH				0x0A
#define AUDIO_INFOFRAME_HEADER_LEN			14

// Misc InfoFrame Type
#define MISC_INFO_TYPE						0x04
#define MISC_INFO_ALWAYSSET					0x80
#define MISC_INFO_VERSION					0x01
#define MISC_INFO_LENGTH					0x0A

// Channel Count for Audio Infoframe header definition
#define AUDIO_CHANNEL_2						1
#define AUDIO_CHANNEL_3						2
#define AUDIO_CHANNEL_4						3
#define AUDIO_CHANNEL_5						4
#define AUDIO_CHANNEL_6						5
#define AUDIO_CHANNEL_7						6
#define AUDIO_CHANNEL_8						7

// Audio type for Audio Infoframe header definition
#define AUDIO_PCM							1
#define AUDIO_AC3							2
#define AUDIO_MPEG1							3
#define AUIDO_MP3							4
#define AUDIO_MPEG2							5
#define AUDIO_AAC							6
#define AUDIO_DTS							7
#define AUDIO_ATRAC							8

// Sample size for Audio Infoframe header definition
#define SAMPLESIZE_16BIT					1
#define SAMPLESIZE_20BIT					2
#define SAMPLESIZE_24BIT					3

// Sample rate for Audio Infoframe header definition
#define SAMPLERATE_32KHZ					1
#define SAMPLERATE_441KHZ					2
#define SAMPLERATE_48KHZ					3
#define SAMPLERATE_882KHZ					4
#define SAMPLERATE_96KHZ					5
#define SAMPLERATE_1764KHZ					6
#define SAMPLERATE_192KHZ					7

// Definition of HDMI device status
#define POWER_D0						0x01
#define POWER_D3						0x02

// Power bit definition
#define BIT_DRIVEN_TX_BRIDGE					0x10
#define BIT_POWER_D0						0x00
#define BIT_POWER_D2						0x02
#define BIT_POWER_D3						0x03

// Interrupt Control bit definition
#define BIT_HOTPLUG_ENABLE					0x01
#define BIT_CPIEVENT_ENABLE					0x08

// Interrupt status bit definition
#define BIT_HOTPLUG_CONNECTION					0x01
#define BIT_SENSE_EVENT						0x02
#define BIT_HOTPLUG_CTRLBUS					0x04

// System Control bit definition
#define BIT_DDCBUS_REQUEST					0x04
#define BIT_DDCBUS_GRANT					0x02

// Audio Configure bit definition
#define BIT_AUDIO_PCM						0x01
#define BIT_AUDIO_MUTE						0x10
#define BIT_AUDIO_I2S						0x80

// Audio sample rate bit definition
#define BIT_AUDIO_16BIT						0x40
#define BIT_AUDIO_48KHZ						0x18

// Register of Sil9022/24 definition
#define REG_SYSTEM_CTRL					0x1A
#define REG_DEVID					0x1B
#define REG_POWER_CTRL					0x1E
#define REG_I2S_MAPPING					0x1F
#define REG_I2S_CONFIGURE				0x20
#define REG_I2S_CHSTATUS_0				0x21
#define REG_I2S_CHSTATUS_1				0x22
#define REG_I2S_CHSTATUS_2				0x23
#define REG_I2S_CHSTATUS_3				0x24
#define REG_I2S_CHSTATUS_4				0x25
#define REG_AUDIO_CONFIGURE				0x26
#define REG_AUDIO_SAMPLERATE				0x27
#define REG_SECURITY_CTRL				0x2A
#define REG_INTERRUPT_CTRL				0x3C
#define REG_INTERRUPT_STATUS				0x3D
#define REG_AUDIO_INFOFRAME				0xBF
#define REG_MISC_INFOFRAME				0xC0
#define REG_SOFT_RESET					0x40

struct Sil9022_RegCfg{
	unsigned char uc_RegIndex;
	unsigned char uc_RegData;
};


struct sil9022_setup_data {
	unsigned short i2c_address;
	unsigned short i2c_bus;
};

// Audio Infoframe Header Struct
struct Audio_Infoframe_Header {
	unsigned char uc_ChCount;
	unsigned char uc_AudioType;
	unsigned char uc_SampleSize;
	unsigned char uc_SampleFreq;
	unsigned char uc_SpeakerCfg;
};

#endif
