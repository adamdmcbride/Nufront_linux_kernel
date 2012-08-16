/*
 * Base driver for CH7033 HDMI chip (I2C)
 *
 * Copyright (C) 2011 NUFRONT INC.
 *
 */

#ifndef _NUSMART_HDMI_H
#define _NUSMART_HDMI_H

#define REG_PAGE			0x03
#define REG_EDID			0x18


#define PAGE_1				0x0
#define PAGE_2				0x1
#define PAGE_3				0x3
#define PAGE_4				0x4



//Chrontel type define: 
typedef unsigned long long	ch_uint64;
typedef unsigned int		ch_uint32;
typedef unsigned short		ch_uint16;
typedef unsigned char		ch_uint8;
typedef ch_uint32		ch_bool;
#define ch_true			1
#define ch_false		0



#ifdef CONFIG_FB_HDMI_TERMINAL_TV

static ch_uint32 CH7033_Nulcdc_1280x720_720p[][2] = {
// 1280x720-60Hz, 16/32-bit, NuLCDC
	{ 0x03, 0x04 },	{ 0x52, 0x01 },	{ 0x52, 0x03 },	{ 0x03, 0x00 },	{ 0x07, 0x91 },	{ 0x08, 0x0F },	{ 0x09, 0x02 },	{ 0x0B, 0x35 },
	{ 0x0C, 0x32 },	{ 0x0D, 0x90 },	{ 0x0F, 0x9B },	{ 0x10, 0x28 }, { 0x11, 0x1A },	{ 0x12, 0xE4 },	{ 0x13, 0x0C },	{ 0x15, 0x19 },
	{ 0x16, 0x05 },	{ 0x19, 0xF9 },	{ 0x1A, 0x22 },	{ 0x1B, 0x0A },	{ 0x1C, 0x69 },	{ 0x1D, 0x78 },	{ 0x1E, 0xC8 },	{ 0x1F, 0x35 },
	{ 0x20, 0x00 },	{ 0x21, 0x72 },	{ 0x25, 0x52 },	{ 0x26, 0xD0 },	{ 0x27, 0xEE },	{ 0x2E, 0x3D },	{ 0x3D, 0x8A },	{ 0x40, 0x04 },
	{ 0x55, 0x6E },	{ 0x56, 0x28 },	{ 0x58, 0x05 },	{ 0x59, 0x05 },	{ 0x5E, 0x54 },	{ 0x60, 0x9B },	{ 0x61, 0x02 },	{ 0x64, 0x27 },
	{ 0x6B, 0x72 },	{ 0x6C, 0x19 },	{ 0x7E, 0x8F },	{ 0x03, 0x01 }, { 0x08, 0x05 },	{ 0x0B, 0x75 },	{ 0x0C, 0x6A },	{ 0x0D, 0x21 },
	{ 0x0F, 0x9D },	{ 0x12, 0xD4 },	{ 0x13, 0x28 },	{ 0x14, 0x80 },	{ 0x15, 0xE1 },	{ 0x1B, 0x9B },	{ 0x1C, 0x8A },	{ 0x1D, 0x55 },
	{ 0x23, 0xE3 },	{ 0x28, 0x1D },	{ 0x29, 0x01 },	{ 0x6B, 0x10 },	{ 0x6C, 0x04 },	{ 0x03, 0x03 },	{ 0x28, 0x04 },	{ 0x03, 0x04 },
	{ 0x10, 0x01 },	{ 0x11, 0x22 },	{ 0x12, 0x0A },	{ 0x13, 0x02 },	{ 0x14, 0x88 },	{ 0x15, 0x70 },	{ 0x54, 0xC4 },	{ 0x55, 0x5B },
	{ 0x56, 0x4D },	{ 0x61, 0xC4 },	{ 0x7F, 0xFE },
};



static ch_uint32 CH7033_Nulcdc_1920x1080_1080p[][2] = {
// 1920x1080-60Hz, 16/32-bit, NuLCDC
	{ 0x03, 0x04 },	{ 0x52, 0x01 },	{ 0x52, 0x03 },	{ 0x03, 0x00 },	{ 0x07, 0x91 },	{ 0x08, 0x0F },	{ 0x09, 0x02 },	{ 0x0B, 0x35 },
	{ 0x0C, 0x32 },	{ 0x0D, 0x90 },	{ 0x0F, 0x9B },	{ 0x10, 0x28 },	{ 0x11, 0x1A },	{ 0x12, 0xE4 },	{ 0x13, 0x0C },	{ 0x15, 0x19 },
	{ 0x16, 0x05 },	{ 0x19, 0xF9 },	{ 0x1A, 0x22 },	{ 0x1B, 0x0A },	{ 0x1C, 0x69 },	{ 0x1D, 0x78 },	{ 0x1E, 0xC8 },	{ 0x1F, 0x47 },
	{ 0x21, 0x98 },	{ 0x25, 0x24 },	{ 0x26, 0x38 },	{ 0x27, 0x65 },	{ 0x2E, 0x3D },	{ 0x3D, 0x8A },	{ 0x40, 0x10 },	{ 0x55, 0x58 },
	{ 0x56, 0x2C },	{ 0x58, 0x04 },	{ 0x59, 0x05 },	{ 0x5E, 0x54 },	{ 0x64, 0x2A },	{ 0x68, 0x46 },	{ 0x6B, 0x7B },	{ 0x6C, 0x16 },
	{ 0x7E, 0x8F },	{ 0x03, 0x01 },	{ 0x08, 0x05 },	{ 0x0B, 0x75 },	{ 0x0C, 0x74 },	{ 0x0D, 0x21 },	{ 0x0F, 0x9D },	{ 0x12, 0xD6 },
	{ 0x13, 0x28 },	{ 0x14, 0x80 },	{ 0x15, 0xE1 },	{ 0x1B, 0xE8 },	{ 0x1C, 0x4E },	{ 0x1D, 0x80 },	{ 0x23, 0xE3 },	{ 0x28, 0x3A },
	{ 0x29, 0x02 },	{ 0x6B, 0x10 },	{ 0x6C, 0x00 },	{ 0x03, 0x03 },	{ 0x28, 0x04 },	{ 0x03, 0x04 },	{ 0x10, 0x02 },	{ 0x11, 0x44 },
	{ 0x12, 0x14 },	{ 0x13, 0x02 },	{ 0x14, 0x88 },	{ 0x15, 0x70 },	{ 0x54, 0xC4 },	{ 0x55, 0x5B },	{ 0x56, 0x4D },	{ 0x61, 0xC4 },
	{ 0x7F, 0xFE },
};


static ch_uint32 CH7033_Nulcdc_1024x768_720p[][2] = {
// 1024x768-60Hz, 16/32-bit, NuLCDC
	{ 0x03, 0x04 },	{ 0x52, 0x01 },	{ 0x52, 0x03 },	{ 0x03, 0x00 },	{ 0x07, 0x91 },	{ 0x08, 0x0F },	{ 0x09, 0x02 },	{ 0x0B, 0x2C },
	{ 0x0C, 0x24 },	{ 0x0D, 0x5E },	{ 0x0F, 0x14 },	{ 0x10, 0x90 },	{ 0x11, 0x1B },	{ 0x12, 0x0A },	{ 0x13, 0x30 },	{ 0x15, 0x08 },
	{ 0x16, 0x06 },	{ 0x1A, 0xFD },	{ 0x1B, 0xE8 },	{ 0x1C, 0x69 },	{ 0x1D, 0x78 },	{ 0x1E, 0xC8 },	{ 0x1F, 0x35 },	{ 0x20, 0x00 },
	{ 0x21, 0x72 },	{ 0x25, 0x12 },	{ 0x26, 0xD0 },	{ 0x27, 0xEE },	{ 0x2E, 0x3D },	{ 0x3D, 0x8A },	{ 0x40, 0x04 },	{ 0x55, 0x6E },
	{ 0x56, 0x28 },	{ 0x58, 0x05 },	{ 0x59, 0x05 },	{ 0x5E, 0x54 },	{ 0x64, 0x22 },	{ 0x68, 0x42 },	{ 0x6B, 0x68 },	{ 0x6C, 0x12 },
	{ 0x7E, 0x8F },	{ 0x03, 0x01 },	{ 0x08, 0x05 },	{ 0x0B, 0x75 },	{ 0x0C, 0x6A },	{ 0x0D, 0x21 },	{ 0x0F, 0x9D },	{ 0x12, 0xD4 },
	{ 0x13, 0x28 },	{ 0x14, 0x80 },	{ 0x15, 0xE1 },	{ 0x1B, 0x99 },	{ 0x1C, 0x8C },	{ 0x1D, 0x54 },	{ 0x23, 0xE3 },	{ 0x28, 0x1D },
	{ 0x29, 0x01 },	{ 0x6B, 0x10 },	{ 0x6C, 0x04 },	{ 0x03, 0x03 },	{ 0x28, 0x04 },	{ 0x03, 0x04 },	{ 0x10, 0x01 },	{ 0x11, 0x22 },
	{ 0x12, 0x0A },	{ 0x13, 0x02 },	{ 0x14, 0x88 },	{ 0x15, 0x70 },	{ 0x54, 0xC4 },	{ 0x55, 0x5B },	{ 0x56, 0x4D },	{ 0x61, 0xC4 },
	{ 0x7F, 0xFE },
};


static ch_uint32 CH7033_Nulcdc_1024x768_1080p[][2] = {
// 1024x768-60Hz, 16/32-bit, NuLCDC
	{ 0x03, 0x04 },	{ 0x52, 0x01 },	{ 0x52, 0x03 },	{ 0x03, 0x00 },	{ 0x07, 0x91 },	{ 0x08, 0x0F },	{ 0x09, 0x02 },	{ 0x0B, 0x2C },
	{ 0x0C, 0x24 },	{ 0x0D, 0x5E },	{ 0x0F, 0x14 },	{ 0x10, 0x90 },	{ 0x11, 0x1B },	{ 0x12, 0x0A },	{ 0x13, 0x30 },	{ 0x15, 0x08 },
	{ 0x16, 0x06 },	{ 0x1A, 0xFD },	{ 0x1B, 0xE8 },	{ 0x1C, 0x69 },	{ 0x1D, 0x78 },	{ 0x1E, 0xC8 },	{ 0x1F, 0x47 },	{ 0x21, 0x98 },
	{ 0x25, 0x24 },	{ 0x26, 0x38 },	{ 0x27, 0x65 },	{ 0x2E, 0x3D },	{ 0x3D, 0x8A },	{ 0x40, 0x10 },	{ 0x55, 0x58 },	{ 0x56, 0x2C },
	{ 0x58, 0x04 },	{ 0x59, 0x05 },	{ 0x5E, 0x54 },	{ 0x64, 0x22 },	{ 0x68, 0x46 },	{ 0x6B, 0x68 },	{ 0x6C, 0x12 },	{ 0x7E, 0x8F },
	{ 0x03, 0x01 },	{ 0x08, 0x05 },	{ 0x0B, 0x75 },	{ 0x0C, 0x74 },	{ 0x0D, 0x21 },	{ 0x0F, 0x9D },	{ 0x12, 0xD6 },	{ 0x13, 0x28 },
	{ 0x14, 0x80 },	{ 0x15, 0xE1 },	{ 0x1B, 0xE6 },	{ 0x1C, 0x0C },	{ 0x1D, 0x7F },	{ 0x23, 0xE3 },	{ 0x28, 0x3A },	{ 0x29, 0x02 },
	{ 0x6B, 0x10 },	{ 0x6C, 0x00 },	{ 0x03, 0x03 },	{ 0x28, 0x04 },	{ 0x03, 0x04 },	{ 0x10, 0x02 },	{ 0x11, 0x44 },	{ 0x12, 0x14 },
	{ 0x13, 0x02 },	{ 0x14, 0x88 },	{ 0x15, 0x70 },	{ 0x54, 0xC4 },	{ 0x55, 0x5B },	{ 0x56, 0x4D },	{ 0x61, 0xC4 },	{ 0x7F, 0xFE },

};

static ch_uint32 CH7033_Nulcdc_1024x600_720p[][2] = {
// 1024x600-60Hz, 16/32-bit, NuLCDC
	{ 0x03, 0x04 },	{ 0x52, 0x01 },	{ 0x52, 0x03 },	{ 0x03, 0x00 },	{ 0x07, 0x91 },	{ 0x08, 0x0F },	{ 0x09, 0x02 },	{ 0x0B, 0x2C },
	{ 0x0C, 0x24 },	{ 0x0D, 0x5E },	{ 0x0F, 0x3E },	{ 0x10, 0x20 },	{ 0x11, 0x12 },	{ 0x12, 0x62 },	{ 0x13, 0x7B },	{ 0x15, 0x08 },
	{ 0x16, 0x0A },	{ 0x1A, 0xC4 },	{ 0x1B, 0xE0 },	{ 0x1C, 0x69 },	{ 0x1D, 0x78 },	{ 0x1E, 0xC8 },	{ 0x1F, 0x35 },	{ 0x20, 0x00 },
	{ 0x21, 0x72 },	{ 0x25, 0x12 }, { 0x26, 0xD0 },	{ 0x27, 0xEE },	{ 0x2E, 0x3D },	{ 0x3D, 0x8A },	{ 0x40, 0x04 },	{ 0x55, 0x6E },
	{ 0x56, 0x28 },	{ 0x58, 0x05 },	{ 0x59, 0x05 },	{ 0x5E, 0x54 },	{ 0x68, 0x46 },	{ 0x6B, 0x6D },	{ 0x6C, 0x1B },	{ 0x7E, 0x8F },
	{ 0x03, 0x01 },	{ 0x08, 0x05 }, { 0x0B, 0x75 },	{ 0x0C, 0x6A },	{ 0x0D, 0x21 },	{ 0x0F, 0x9D },	{ 0x12, 0xD4 },	{ 0x13, 0x28 },
	{ 0x14, 0x80 },	{ 0x15, 0xE1 },	{ 0x1B, 0x99 },	{ 0x1C, 0x8C },	{ 0x1D, 0x54 },	{ 0x23, 0xE3 },	{ 0x28, 0x1D },	{ 0x29, 0x01 },
	{ 0x6B, 0x10 },	{ 0x6C, 0x04 },	{ 0x03, 0x03 },	{ 0x28, 0x04 },	{ 0x03, 0x04 },	{ 0x10, 0x01 },	{ 0x11, 0x22 },	{ 0x12, 0x0A },
	{ 0x13, 0x02 },	{ 0x14, 0x88 },	{ 0x15, 0x70 },	{ 0x54, 0xC4 },	{ 0x55, 0x5B },	{ 0x56, 0x4D },	{ 0x61, 0xC4 },	{ 0x7F, 0xFE },
};



#else


static ch_uint32 CH7033_Nulcdc_1280x720_720p[][2] = {
// 1280x720-60Hz, 16/32-bit, NuLCDC
	{ 0x03, 0x04 },	{ 0x52, 0x01 },	{ 0x52, 0x03 },	{ 0x03, 0x00 },	{ 0x07, 0x91 },	{ 0x08, 0x0F },	{ 0x09, 0x02 },	{ 0x0B, 0x35 },
	{ 0x0C, 0x14 },	{ 0x0D, 0x90 },	{ 0x0F, 0xAA },	{ 0x10, 0x28 },	{ 0x11, 0x12 },	{ 0x12, 0xDA },	{ 0x13, 0xF8 },	{ 0x15, 0x0F },
	{ 0x16, 0x05 },	{ 0x19, 0xF9 },	{ 0x1A, 0x22 },	{ 0x1B, 0x0A },	{ 0x1C, 0x69 },	{ 0x1D, 0x78 },	{ 0x1E, 0xC8 },	{ 0x1F, 0x35 },
	{ 0x20, 0x00 },	{ 0x21, 0x72 },	{ 0x25, 0x52 },	{ 0x26, 0xD0 },	{ 0x27, 0xEE },	{ 0x2E, 0x3D },	{ 0x3D, 0x8A },	{ 0x40, 0x04 },
	{ 0x55, 0x6E },	{ 0x56, 0x28 },	{ 0x58, 0x05 },	{ 0x59, 0x05 },	{ 0x5E, 0x54 },	{ 0x60, 0xA0 },	{ 0x64, 0x28 },	{ 0x6B, 0x73 },
	{ 0x7E, 0x8F },	{ 0x03, 0x01 },	{ 0x08, 0x05 },	{ 0x0B, 0x75 },	{ 0x0C, 0x6A },	{ 0x0D, 0x21 },	{ 0x0F, 0x9D },	{ 0x12, 0xD4 },
	{ 0x13, 0x28 },	{ 0x14, 0x80 },	{ 0x15, 0xE1 },	{ 0x23, 0xE3 },	{ 0x28, 0x1D },	{ 0x29, 0x01 },	{ 0x6B, 0x10 },	{ 0x6C, 0x04 },
	{ 0x03, 0x03 },	{ 0x28, 0x04 },	{ 0x03, 0x04 },	{ 0x10, 0x01 },	{ 0x11, 0x22 },	{ 0x12, 0x0A },	{ 0x13, 0x02 },	{ 0x14, 0x88 },
	{ 0x15, 0x70 },	{ 0x54, 0xC4 },	{ 0x55, 0x5B },	{ 0x56, 0x4D },	{ 0x61, 0xC4 },	{ 0x7F, 0xFE },
};


static ch_uint32 CH7033_Nulcdc_1024x768_720p[][2] = {
// 1024x768-60Hz, 16/32-bit, NuLCDC
	{ 0x03, 0x04 },	{ 0x52, 0x01 },	{ 0x52, 0x03 },	{ 0x03, 0x00 },	{ 0x07, 0x91 },	{ 0x08, 0x0F },	{ 0x09, 0x02 },	{ 0x0B, 0x2C },
	{ 0x0C, 0x1E },	{ 0x0D, 0x4A },	{ 0x0F, 0x04 },	{ 0x10, 0x90 },	{ 0x11, 0x1B },	{ 0x12, 0x04 },	{ 0x13, 0x2A },	{ 0x15, 0x08 },
	{ 0x16, 0x06 },	{ 0x1A, 0xFD },	{ 0x1B, 0xE8 },	{ 0x1C, 0x69 },	{ 0x1D, 0x78 },	{ 0x1E, 0xC8 },	{ 0x1F, 0x35 },	{ 0x20, 0x00 },
	{ 0x21, 0x72 },	{ 0x25, 0x12 },	{ 0x26, 0xD0 },	{ 0x27, 0xEE },	{ 0x2E, 0x3D },	{ 0x3D, 0x8A },	{ 0x40, 0x04 },	{ 0x55, 0x6E },
	{ 0x56, 0x28 },	{ 0x58, 0x05 },	{ 0x59, 0x05 },	{ 0x5E, 0x54 },	{ 0x64, 0x21 },	{ 0x68, 0x42 },	{ 0x6B, 0x65 },	{ 0x6C, 0x11 },
	{ 0x7E, 0x8F },	{ 0x03, 0x01 },	{ 0x08, 0x05 },	{ 0x0B, 0x75 },	{ 0x0C, 0x6A },	{ 0x0D, 0x21 },	{ 0x0F, 0x9D },	{ 0x12, 0xD4 },
	{ 0x13, 0x28 },	{ 0x14, 0x80 },	{ 0x15, 0xE1 },	{ 0x23, 0xE3 },	{ 0x28, 0x1D },	{ 0x29, 0x01 },	{ 0x6B, 0x10 },	{ 0x6C, 0x04 },
	{ 0x03, 0x03 },	{ 0x28, 0x04 },	{ 0x03, 0x04 },	{ 0x10, 0x01 },	{ 0x11, 0x22 },	{ 0x12, 0x0A },	{ 0x13, 0x02 },	{ 0x14, 0x88 },
	{ 0x15, 0x70 },	{ 0x54, 0xC4 },	{ 0x55, 0x5B },	{ 0x56, 0x4D },	{ 0x61, 0xC4 },	{ 0x7F, 0xFE },
};


static ch_uint32 CH7033_Nulcdc_1024x600_720p[][2] = {
// 1024x600-60Hz, 16/32-bit, NuLCDC
#if 0
	{ 0x03, 0x04 },	{ 0x52, 0x01 },	{ 0x52, 0x03 },	{ 0x03, 0x00 },	{ 0x07, 0x91 }, { 0x08, 0x0F },	{ 0x09, 0x02 },	{ 0x0B, 0x2C },
	{ 0x0C, 0x14 },	{ 0x0D, 0x40 },	{ 0x0F, 0x28 },	{ 0x10, 0x20 },	{ 0x11, 0x12 },	{ 0x12, 0x5E },	{ 0x13, 0x76 },	{ 0x15, 0x08 },
	{ 0x16, 0x0A },	{ 0x1A, 0xC4 },	{ 0x1B, 0xE0 },	{ 0x1C, 0x69 },	{ 0x1D, 0x78 },	{ 0x1E, 0xC8 },	{ 0x1F, 0x35 },	{ 0x20, 0x00 },
	{ 0x21, 0x72 },	{ 0x25, 0x12 },	{ 0x26, 0xD0 },	{ 0x27, 0xEE },	{ 0x2E, 0x3D },	{ 0x3D, 0x8A },	{ 0x40, 0x04 },	{ 0x55, 0x6E },
	{ 0x56, 0x28 },	{ 0x58, 0x05 },	{ 0x59, 0x05 },	{ 0x5E, 0x54 },	{ 0x68, 0x46 },	{ 0x6B, 0x6A },	{ 0x6C, 0x1E },	{ 0x7E, 0x8F },
	{ 0x03, 0x01 },	{ 0x08, 0x05 },	{ 0x0B, 0x75 },	{ 0x0C, 0x6A },	{ 0x0D, 0x21 },	{ 0x0F, 0x9D }, { 0x12, 0xD4 },	{ 0x13, 0x28 },
	{ 0x14, 0x80 },	{ 0x15, 0xE1 },	{ 0x23, 0xE3 },	{ 0x28, 0x1D },	{ 0x29, 0x01 },	{ 0x6B, 0x10 },	{ 0x6C, 0x04 },	{ 0x03, 0x03 },
	{ 0x28, 0x04 },	{ 0x03, 0x04 },	{ 0x10, 0x01 },	{ 0x11, 0x22 },	{ 0x12, 0x0A },	{ 0x13, 0x02 },	{ 0x14, 0x88 },	{ 0x15, 0x70 },
	{ 0x54, 0xC4 },	{ 0x55, 0x5B },	{ 0x56, 0x4D },	{ 0x61, 0xC4 },	{ 0x7F, 0xFE },
#endif	
#if 1
// 1024x600-60Hz  720p
	{ 0x03, 0x04 }, { 0x52, 0x01 }, { 0x52, 0x03 }, { 0x03, 0x00 }, { 0x07, 0x91 }, { 0x08, 0x0F }, { 0x09, 0x02 }, { 0x0B, 0x2C },
	{ 0x0C, 0x00 }, { 0x0D, 0x40 }, { 0x0F, 0x30 }, { 0x10, 0x20 }, { 0x11, 0x12 }, { 0x12, 0x58 }, { 0x13, 0x76 }, { 0x15, 0x08 },
	{ 0x16, 0x0A }, { 0x19, 0xF9 }, { 0x1A, 0x86 }, { 0x1B, 0xA0 }, { 0x1C, 0x69 }, { 0x1D, 0x78 }, { 0x1E, 0xC8 }, { 0x1F, 0x35 },
	{ 0x20, 0x00 }, { 0x21, 0x72 }, { 0x25, 0x12 }, { 0x26, 0xD0 }, { 0x27, 0xEE }, { 0x2E, 0x3D }, { 0x3D, 0x8A }, { 0x40, 0x04 },
 	{ 0x55, 0x6E }, { 0x56, 0x28 }, { 0x58, 0x05 }, { 0x59, 0x05 }, { 0x5E, 0x54 }, { 0x64, 0x30 }, { 0x68, 0x46 }, { 0x6B, 0x72 },
 	{ 0x6E, 0xA8 }, { 0x74, 0x30 }, { 0x7E, 0x8F }, { 0x03, 0x01 }, { 0x08, 0x05 }, { 0x0B, 0x75 }, { 0x0C, 0x6A }, { 0x0D, 0x21 },
	{ 0x0F, 0x9D }, { 0x12, 0xD4 }, { 0x13, 0x28 }, { 0x14, 0x80 }, { 0x15, 0xE1 }, { 0x23, 0xE3 }, { 0x28, 0x1D }, { 0x29, 0x01 },
	{ 0x6B, 0x10 }, { 0x6C, 0x04 }, { 0x03, 0x03 }, { 0x28, 0x04 }, { 0x03, 0x04 }, { 0x10, 0x01 }, { 0x11, 0x22 }, { 0x12, 0x0A },
	{ 0x13, 0x02 }, { 0x14, 0x88 }, { 0x15, 0x70 }, { 0x54, 0xC4 }, { 0x55, 0x5B }, { 0x56, 0x4D }, { 0x61, 0xE4 }, { 0x7F, 0xFE },
#else
// 1024x600-60Hz 1080p
        { 0x03, 0x04 },{ 0x52, 0x01 },{ 0x52, 0x03 },{ 0x03, 0x00 },{ 0x07, 0x91 },{ 0x08, 0x0F },{ 0x09, 0x02 },{ 0x0B, 0x2C },
	{ 0x0C, 0x00 },{ 0x0D, 0x40 },{ 0x0F, 0x30 },{ 0x10, 0x20 },{ 0x11, 0x12 },{ 0x12, 0x58 },{ 0x13, 0x76 },{ 0x15, 0x08 },
	{ 0x16, 0x0A },{ 0x19, 0xF9 },{ 0x1A, 0x86 },{ 0x1B, 0xA0 },{ 0x1C, 0x69 },{ 0x1D, 0x78 },{ 0x1E, 0xC8 },{ 0x1F, 0x47 },
	{ 0x21, 0x98 },{ 0x25, 0x24 },{ 0x26, 0x38 },{ 0x27, 0x65 },{ 0x2E, 0x3D },{ 0x3D, 0x8A },{ 0x40, 0x10 },{ 0x55, 0x58 },
	{ 0x56, 0x2C },{ 0x58, 0x04 },{ 0x59, 0x05 },{ 0x5E, 0x54 },{ 0x64, 0x30 },{ 0x68, 0x46 },{ 0x6B, 0x72 },{ 0x6E, 0xA8 },
	{ 0x74, 0x30 },{ 0x7E, 0x8F },{ 0x03, 0x01 },{ 0x08, 0x05 },{ 0x0B, 0x75 },{ 0x0C, 0x74 },{ 0x0D, 0x21 },{ 0x0F, 0x9D },
	{ 0x12, 0xD6 },{ 0x13, 0x28 },{ 0x14, 0x80 },{ 0x15, 0xE1 },{ 0x23, 0xE3 },{ 0x28, 0x3A },{ 0x29, 0x02 },{ 0x6B, 0x10 },
	{ 0x6C, 0x00 },{ 0x03, 0x03 },{ 0x28, 0x04 },{ 0x03, 0x04 },{ 0x10, 0x02 },{ 0x11, 0x44 },{ 0x12, 0x14 },{ 0x13, 0x02 },
	{ 0x14, 0x88 },{ 0x15, 0x70 },{ 0x54, 0xC4 },{ 0x55, 0x5B },{ 0x56, 0x4D },{ 0x61, 0xE4 },{ 0x7F, 0xFE },

#endif
};



static ch_uint32 CH7033_Nulcdc_1920x1080_1080p[][2] = {
// 1920x1080-60Hz, 16/32-bit, NuLCDC
	{ 0x03, 0x04 },	{ 0x52, 0x01 },	{ 0x52, 0x03 },	{ 0x03, 0x00 },	{ 0x07, 0x91 },	{ 0x08, 0x0F },	{ 0x09, 0x02 },	{ 0x0B, 0x35 },
	{ 0x0C, 0x32 },	{ 0x0D, 0x90 },	{ 0x0F, 0x9B },	{ 0x10, 0x28 },	{ 0x11, 0x1A },	{ 0x12, 0xE4 },	{ 0x13, 0x0C },	{ 0x15, 0x19 },
	{ 0x16, 0x05 },	{ 0x19, 0xF9 },	{ 0x1A, 0x22 },	{ 0x1B, 0x0A },	{ 0x1C, 0x69 },	{ 0x1D, 0x78 },	{ 0x1E, 0xC8 },	{ 0x1F, 0x47 },
	{ 0x21, 0x98 },	{ 0x25, 0x24 },	{ 0x26, 0x38 },	{ 0x27, 0x65 },	{ 0x2E, 0x3D },	{ 0x3D, 0x8A },	{ 0x40, 0x10 },	{ 0x55, 0x58 },
	{ 0x56, 0x2C },	{ 0x58, 0x04 },	{ 0x59, 0x05 },	{ 0x5E, 0x54 },	{ 0x64, 0x2A },	{ 0x68, 0x46 },	{ 0x6B, 0x7B },	{ 0x6C, 0x16 },
	{ 0x7E, 0x8F },	{ 0x03, 0x01 },	{ 0x08, 0x05 },	{ 0x0B, 0x75 },	{ 0x0C, 0x74 },	{ 0x0D, 0x21 },	{ 0x0F, 0x9D },	{ 0x12, 0xD6 },
	{ 0x13, 0x28 },	{ 0x14, 0x80 },	{ 0x15, 0xE1 },	{ 0x1B, 0xE8 },	{ 0x1C, 0x4E },	{ 0x1D, 0x80 },	{ 0x23, 0xE3 },	{ 0x28, 0x3A },
	{ 0x29, 0x02 },	{ 0x6B, 0x10 },	{ 0x6C, 0x00 },	{ 0x03, 0x03 },	{ 0x28, 0x04 },	{ 0x03, 0x04 },	{ 0x10, 0x02 },	{ 0x11, 0x44 },
	{ 0x12, 0x14 },	{ 0x13, 0x02 },	{ 0x14, 0x88 },	{ 0x15, 0x70 },	{ 0x54, 0xC4 },	{ 0x55, 0x5B },	{ 0x56, 0x4D },	{ 0x61, 0xC4 },
	{ 0x7F, 0xFE },
};


#endif





#endif
