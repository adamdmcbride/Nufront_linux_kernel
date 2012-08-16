/*
    camera.h - MRV camera driver header file

    Copyright (C) 2003, Intel Corporation
    Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __ASM_ARCH_CAMERA_H_
#define __ASM_ARCH_CAMERA_H_

#define MRV_CAM_MASTER        (1<<0)
#define MRV_CAM_SLAVE         (1<<1)
#define MRV_CAM_HSYNC_H       (1<<2)
#define MRV_CAM_HSYNC_L       (1<<3)
#define MRV_CAM_VSYNC_H       (1<<4)
#define MRV_CAM_VSYNC_L       (1<<5)
#define MRV_CAM_DATAWIDTH_8   (1<<7)
#define MRV_CAM_DATAWIDTH_10  (1<<9)
#define MRV_CAM_PCLK_SAMP_R   (1<<12)
#define MRV_CAM_PCLK_SAMP_F   (1<<13)

#define MRV_CAM_PLATFORM_DEFAULT MRV_CAM_MASTER |\
				 MRV_CAM_HSYNC_H | MRV_CAM_VSYNC_L |\
				 MRV_CAM_DATAWIDTH_8 | MRV_CAM_PCLK_SAMP_R

struct mrv_camera_pdata {
	unsigned long flags;
	unsigned long mclk_10khz;
	void (*prio_set)(int);
};

#endif /* __ASM_ARCH_CAMERA_H_ */
