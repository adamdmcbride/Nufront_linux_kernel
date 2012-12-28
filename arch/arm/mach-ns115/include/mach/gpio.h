#ifndef NS115_GPIO_H_
#define NS115_GPIO_H_

#ifdef CONFIG_MFD_RICOH583
#define ARCH_NR_GPIOS		(32*4+8+8)
#else
#define ARCH_NR_GPIOS		(32*4+8)
#endif
#include <asm-generic/gpio.h>
#include <mach/irqs-ns115.h>

#define INT_NUM0		8
#define INT_NUM1		32
#define INT_GPIO		(INT_NUM0 + INT_NUM1)
#define GPIO_BASE0		0
#define GPIO_BASE1		8
#define GPIO_IRQ_BASE1		IRQ_NS115_GPIO1_INTR0                 
#define GPIO_IRQ_BASE0		IRQ_NS115_GPIO0_WAKEUP_0                                
#define GPIO0_LINE_MAX		(8)
#define GPIO1_LINE_MAX		(32*4)
#define GPIO_LINE_MAX		(GPIO0_LINE_MAX	+ GPIO1_LINE_MAX)
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
/*data out put*/
#define GPIO_PORTC_DR		0x18
/*0 in;1 out*/
#define GPIO_PORTC_DDR		0x1c
/*0 soft;1 hard*/
#define GPIO_PORTC_CTRL		0x20
/*data out put*/
#define GPIO_PORTD_DR		0x24
/*0 in;1 out*/
#define GPIO_PORTD_DDR		0x28
/*0 soft;1 hard*/
#define GPIO_PORTD_CTRL		0x2c
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
#define BOARD_GPIO_INTEN	0xffffffff
/*enable all interrupt in gpio,masked in gic dist*/
#define BOARD_GPIO_INTMASK	0x0000

#if defined(CONFIG_TOUCHSCREEN_ILITEK)
#define BOARD_GPIO_INTTYPECFG	0x0000
#else
/*config for interrupt type 0 level;1 edge*/
#define BOARD_GPIO_INTTYPECFG	0xffffffff
#endif

#if defined(CONFIG_TOUCHSCREEN_SIS_I2C) || defined(CONFIG_TOUCHSCREEN_ZT2083_TS) || defined(CONFIG_TOUCHSCREEN_ILITEK) || defined(CONFIG_TOUCHSCREEN_GOODIX_BIG)
/*config for interrupt type 0 low;1 high*/
#define BOARD_GPIO_INTPOLCFG	(0xffffffff&0xffffffbf)
#else
#define BOARD_GPIO_INTPOLCFG	0xffffffff
#endif

#if 0
static inline int is_valid_gpio(unsigned gpio)
{
	return gpio<(GPIO_BASE0+GPIO_LINE_MAX);
}
#endif

static inline int gpio_to_irq(int gpio)
{
	return __gpio_to_irq(gpio);
}

static inline int irq_to_gpio(int irq)
{
	if(irq < GPIO_IRQ_BASE1)
		return GPIO_BASE0+irq-GPIO_IRQ_BASE0;
	else
		return GPIO_BASE1+irq-GPIO_IRQ_BASE1;
}

static inline int gpio_get_value(unsigned gpio)
{
	return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	__gpio_set_value(gpio, value);
}

struct gpio_data
{
	/*temporary solution*/
	/*[0] GPIO A*/
	/*[1] GPIO B*/
	/*[2] GPIO C*/
	/*[3] GPIO D*/
	/*[4] GPIO wk*/
	unsigned int gpio_ddr[5];
};
extern void __init ns115_init_gpio(struct gpio_data * data);
#endif
