/*
 * Base driver for ENE IO373X chip (bus interface)
 *
 * Copyright (C) 2010 ENE TECHNOLOGY INC.
 *
 */

#ifndef _IO373X_H_
#define _IO373X_H_

struct device;
struct io373x;

enum { IO373X_BUS_I2C, IO373X_BUS_SPI };

struct io373x_bus_ops {
    int bustype;
    int (*read_regs) (struct device *dev, unsigned short start_reg, unsigned char *buf, int byte_cnt);
    int (*write_regs) (struct device *dev, unsigned short start_reg, unsigned char *buf, int byte_cnt);
};

// io373x sub device IDs assigned by f/w.
#define SUBDEV_ID_KBD       0x01
#define SUBDEV_ID_MOU       0x02
#define SUBDEV_ID_TS        0x03
#define SUBDEV_ID_CIR       0x04

// platform device IDs assigned by io373x base driver.
#define PDEV_ID_PS2_KBD     0
#define PDEV_ID_PS2_MOU     1
#define PDEV_ID_ONLY_ONE   -1

//#define IO373X_KEY_FN_ESC	(0x66)
#define IO373X_KEY_BTN_MOUSE	(0x66)
//#define IO373X_KEY_FN_F1	(0x5f)
#define IO373X_KEY_SLEEP_S3	(0x5f)
//#define IO373X_KEY_FN_F2	(0x65)
#define IO373X_KEY_WIFI		(0x65)
//#define IO373X_KEY_FN_F3	(0x70)
#define IO373X_KEY_CRTLCD_SWITCH	(0x70)
//#define IO373X_KEY_FN_F4	(0x5c)		//maybe need change
#define IO373X_KEY_BRIGHTNESSDOWN	(0x5c)
//#define IO373X_KEY_FN_F5	(0x67)
#define IO373X_KEY_BRIGHTNESSUP		(0x67)
//#define IO373X_KEY_FN_F9	(0x60)
#define IO373X_KEY_LID_SWITCH	(0x60)	
#define IO373X_KEY_FN_F10	(0x5e)
//#define IO373X_KEY_FN_F11	(0x69)
#define IO373X_KEY_BLUETOOTH	(0x69)
#define IO373X_KEY_FN_F12	(0x71)

/*
#define IO373X_KEY_SOUND_OFF	(0xA6)
#define IO373X_KEY_SOUND_UP	(0xA0)
#define IO373X_KEY_SOUND_DOWN	(0xA1)

//touch pad enable or disable
#define IO373X_KEY_TOUCHPAD	(0xB4)
#define IO373X_KEY_CRTLCD_SWITCH	(0xB5)
#define IO373X_KEY_BRIGHTNESSDOWN	(0xB6)
#define IO373X_KEY_BRIGHTNESSUP		(0xB7)
//back light on/off
#define	IO373X_KEY_BACKLIGHT	(0xB8)
//wifi on/off
#define	IO373X_KEY_WIFI		(0xBA)
#define IO373X_KEY_BLUETOOTH	(0xBB)
//gsensor on/off
#define IO373X_KEY_GSENSOR	(0xBC)
//3G on/off
#define IO373X_KEY_3G		(0xBF)
#define IO373X_KEY_CAMERA	(0xC0)
//internet
#define IO373X_KEY_WWW		(0xC1)
#define IO373X_KEY_EMAIL	(0xC2)
*/

#define IO373X_KEY_POWER_DOWN	(0x77)
#define IO373X_KEY_POWER_UP	(0xF7)

struct io373x_subdev_data
{
    unsigned char dev_id;
    unsigned char count;
    unsigned char buf[16];
};

extern struct workqueue_struct *io373x_wq;

extern int io373x_register_notifier(struct io373x *io373x, struct notifier_block *nb);
extern int io373x_unregister_notifier(struct io373x *io373x, struct notifier_block *nb);

extern int io373x_read_regs(struct io373x *io373x, unsigned short start_reg, unsigned char *buf, int byte_cnt);
extern int io373x_write_regs(struct io373x *io373x, unsigned short start_reg, unsigned char *buf, int byte_cnt);

struct io373x *io373x_probe(struct device *dev, int irq, struct io373x_bus_ops *bus_ops);
int io373x_remove(struct io373x *io373x);

#endif // _IO373X_H_

