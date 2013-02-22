/*
 *the usb divice and host switch driver of SGM7227
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SGM7227_SWITCH_H_
#define SGM7227_SWITCH_H_

struct sgm7227_platform_data {
	int usb_id_gpio;
	int ctl_gpio;
	int usb_5v_gpio;
	int host_channel;/*host channel, 1=HSD1, 2=HSD2*/
};

#endif
