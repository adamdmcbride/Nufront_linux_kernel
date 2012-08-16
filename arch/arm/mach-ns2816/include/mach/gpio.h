#ifndef NS2816_GPIO_H_
#define NS2816_GPIO_H_

#define ARCH_NR_GPIOS		64
#include <asm-generic/gpio.h>
#define NS2816_GPIO_BASE	0x05130000
#define INT_NUM			8
#define GPIO_BASE		0
#define GPIO_IRQ_BASE		(53+32)
#define GPIO_LINE_MAX		64
/*data out put*/
#define GPIO_PORTA_DR		0x0
/*0 in;1 out*/
#define GPIO_PORTA_DDR		0x4
/*0 soft;1 hard*/
#define GPIO_PORTA_CTRL		0x8
/*data out put*/
#define GPIO_PORTB_DR		0xc
/*0 in;1 out*/
#define GPIO_PORTB_DDR		0x10
/*0 soft;1 hard*/
#define GPIO_PORTB_CTRL		0x14
/*0 gpio;1 interrupt*/
#define GPIO_INTEN		0x30
/*0 pass;1 mask*/
#define GPIO_INTMASK		0x34
/*0 level;1 edge*/
#define GPIO_INTTYPE		0x38
/*0 active low or failing;1 active high or rising*/
#define GPIO_PORTOFFSET(x)	((x>>5)*0xc)
#define GPIO_EXTOFFSET(x)	((x>>5)*4)
#define GPIO_MASK(x)		(1<<(x%32))

#define GPIO_INTPOL		0x3c
#define GPIO_INTSTAT		0x40
#define GPIO_INTRAWSTAT		0x44
/*0 leave;1 clear interrupt*/
#define GPIO_EOI		0x4c
#define GPIO_PORTA_EXT		0x50
/*config for hardware control*/
#define BOARD_GPIO_CFG		0x0000
/*config for interrupt line*/
#define BOARD_GPIO_INTEN	0x00ff
/*enable all interrupt in gpio,masked in gic dist*/
#define BOARD_GPIO_INTMASK	0x0000

#if defined(CONFIG_TOUCHSCREEN_ILITEK)
#define BOARD_GPIO_INTTYPECFG	0x0000
#else
/*config for interrupt type 0 level;1 edge*/
#define BOARD_GPIO_INTTYPECFG	0x0000
#endif

#if defined(CONFIG_TOUCHSCREEN_SIS_I2C) || defined(CONFIG_TOUCHSCREEN_ZT2083_TS) || defined(CONFIG_TOUCHSCREEN_ILITEK)
/*config for interrupt type 0 low;1 high*/
#define BOARD_GPIO_INTPOLCFG	0x00ff
#else
#define BOARD_GPIO_INTPOLCFG	0x00ff
#endif
#if 1
static inline int is_valid_gpio(unsigned gpio)
{
	return gpio<(GPIO_BASE+GPIO_LINE_MAX);
}
static inline int gpio_to_irq(int gpio)
{
	return GPIO_IRQ_BASE-gpio+GPIO_BASE;
}
static inline int irq_to_gpio(int irq)
{
	return GPIO_IRQ_BASE-irq+GPIO_BASE;
}

extern void __init ns2816_init_gpio(void);
static inline int gpio_get_value(unsigned gpio)
{
	return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	__gpio_set_value(gpio, value);
}
#endif
#endif
