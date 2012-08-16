#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

#define KEY_NUM	2

#define PRESSED_IS_LOWLEVEL	0
#define PRESSED_IS_HIGHLEVEL	1

struct gpio_keys_button {
	/* Configuration parameters */
	unsigned int code;	/* input event code (KEY_*, SW_*) */
	int irq;
	int key_pressed;
	const char *desc;
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
	const char *name;		/* input device name */
};

#endif

