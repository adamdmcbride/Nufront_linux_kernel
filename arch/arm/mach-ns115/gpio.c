/*
 *  linux/arch/arm/mach-ns115/gpio.c
 *
 *  ns115 GPIO handling
 *
 *  Author:	zeyuan xu
 *  Copyright:	Nusmart
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/syscore_ops.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/board-ns115.h>

struct ns115_gpio_chip {
	struct gpio_chip chip;
	struct irq_chip *gic_chip;
	void __iomem	*regbase;
	char label[10];
	unsigned int 	irq_mask;
	spinlock_t	*lock;
#ifdef CONFIG_PM
	unsigned long	saved_gpactrl;
	unsigned long	saved_gpaddr;
	unsigned long	saved_gpadr;
	unsigned long	saved_gpbctrl;
	unsigned long	saved_gpbddr;
	unsigned long	saved_gpbdr;
	unsigned long	saved_gpcctrl;
	unsigned long	saved_gpcddr;
	unsigned long	saved_gpcdr;
	unsigned long	saved_gpdctrl;
	unsigned long	saved_gpdddr;
	unsigned long	saved_gpddr;
	unsigned long	saved_ginttype;
	unsigned long	saved_gintpol;
	unsigned long	saved_ginten;
#endif
};

#define DBG_PRINT(fmt, args...) printk( KERN_EMERG "nusmart gpio: " fmt, ## args) 

static struct ns115_gpio_chip * g_chip;
static struct ns115_gpio_chip * g_wchip;

static DEFINE_SPINLOCK(gpio_lock);
static DEFINE_SPINLOCK(gpio_wlock);

static inline void __iomem *gpio_chip_base(struct gpio_chip *c)
{
	return container_of(c, struct ns115_gpio_chip, chip)->regbase;
}

static inline struct ns115_gpio_chip *gpio_chip_to_ns115_chip(struct gpio_chip *c)
{
	return container_of(c, struct ns115_gpio_chip, chip);
}

static inline struct ns115_gpio_chip *irq_to_ns115_chip(int irq)
{
	return irq < GPIO_IRQ_BASE1 ? g_wchip:g_chip;
}

static inline int gpio_to_offset(int gpio)
{
	return gpio < GPIO_BASE1 ? gpio:gpio-GPIO_BASE1;
}

static int ns115_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *base = gpio_chip_base(chip);
	spinlock_t *lock = gpio_chip_to_ns115_chip(chip)->lock;
	uint32_t value, mask = GPIO_MASK(offset);
	unsigned long flags;

	spin_lock_irqsave(lock, flags);

	value = __raw_readl(base + GPIO_PORTA_DDR + GPIO_PORTOFFSET(offset));
	value &= ~mask;
	__raw_writel(value, base + GPIO_PORTA_DDR + GPIO_PORTOFFSET(offset));

	spin_unlock_irqrestore(lock, flags);
	return 0;
}

static int ns115_gpio_direction_output(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	void __iomem *base = gpio_chip_base(chip);
	spinlock_t *lock = gpio_chip_to_ns115_chip(chip)->lock;
	uint32_t tmp, mask = GPIO_MASK(offset);
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	tmp = __raw_readl(base + GPIO_PORTA_DR + GPIO_PORTOFFSET(offset));
	if(value == 0)
		tmp &= ~mask;
	else
		tmp |= mask;
	__raw_writel(tmp, base + GPIO_PORTA_DR + GPIO_PORTOFFSET(offset));


	tmp = __raw_readl(base + GPIO_PORTA_DDR + GPIO_PORTOFFSET(offset));
	tmp |= mask;
	__raw_writel(tmp, base + GPIO_PORTA_DDR + GPIO_PORTOFFSET(offset));

	spin_unlock_irqrestore(lock, flags);
	return 0;
}

static int ns115_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	void * ptr = gpio_chip_base(chip) + GPIO_PORTA_EXT + GPIO_EXTOFFSET(offset);
	//printk(KERN_EMERG"%p value 0x%08x\n", ptr, __raw_readl(ptr));
	return (__raw_readl(ptr) & GPIO_MASK(offset))!=0;
}

static void ns115_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	int tmp;
	unsigned long flags;
	spinlock_t *lock = gpio_chip_to_ns115_chip(chip)->lock;
	spin_lock_irqsave(lock, flags);
	tmp = __raw_readl(gpio_chip_base(chip) + GPIO_PORTA_DR + GPIO_PORTOFFSET(offset));
	if(!value)
		tmp &=~GPIO_MASK(offset);
	else
		tmp |=GPIO_MASK(offset);

	__raw_writel(tmp, gpio_chip_base(chip) + GPIO_PORTA_DR + GPIO_PORTOFFSET(offset));
	spin_unlock_irqrestore(lock, flags);
}

static void __init ns115_gpio0(int enable)
{
	void __iomem * regbase = __io_address(NS115_PRCM_BASE);
	unsigned int regval = readl(regbase);
	if(enable)
		regval |= (1<<1);
	else
		regval ^= (1<<1);
	writel(regval, regbase);
}

/* TODO: gpio init need always be success
 * the error handler just for complaint
 * this driver need be rewritten for NS115 GPIO system
 */
static int __init ns115_init_gpio_chip(struct gpio_data * data)
{
	struct ns115_gpio_chip *chip;
	struct gpio_chip * c;

	ns115_gpio0(1);

	chip = kzalloc(sizeof(struct ns115_gpio_chip), GFP_KERNEL);
	if (chip == NULL) {
		pr_err("%s: failed to allocate GPIO chip\n", __func__);
		return -ENOMEM;
	}

	sprintf(chip->label, "gpio-1");
	chip->regbase = __io_address(NS115_GPIO1_BASE);
	chip->lock = &gpio_lock;
	if(!chip->regbase) {
		pr_err("%s: failed to remap GPIO mem\n", __func__);
		return -EBUSY;
	}

	if(data) {
		pr_info("gpio_ddr 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x",\
			data->gpio_ddr[0], data->gpio_ddr[1], data->gpio_ddr[2],\
			data->gpio_ddr[3], data->gpio_ddr[4]);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTA_CTRL);
		__raw_writel(data->gpio_ddr[0], chip->regbase + GPIO_PORTA_DDR);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTB_CTRL);
		__raw_writel(data->gpio_ddr[1], chip->regbase + GPIO_PORTB_DDR);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTC_CTRL);
		__raw_writel(data->gpio_ddr[2], chip->regbase + GPIO_PORTC_DDR);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTD_CTRL);
		__raw_writel(data->gpio_ddr[3], chip->regbase + GPIO_PORTD_DDR);

	} else {
		pr_warn("gpio_ddr is NULL");
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTA_CTRL);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTA_DDR);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTB_CTRL);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTB_DDR);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTC_CTRL);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTC_DDR);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTD_CTRL);
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTD_DDR);
	}

	c = &chip->chip;
	c->label = chip->label;
	c->base  = GPIO_BASE1;
	c->direction_input  = ns115_gpio_direction_input;
	c->direction_output = ns115_gpio_direction_output;
	c->get = ns115_gpio_get;
	c->set = ns115_gpio_set;
	/* number of GPIOs on last bank may be less than 32 */
	c->ngpio = GPIO1_LINE_MAX;
	if(gpiochip_add(c)<0)
		 pr_err("%s: failed to add GPIO chip\n", __func__);
	g_chip = chip;

	chip = kzalloc(sizeof(struct ns115_gpio_chip), GFP_KERNEL);
	if (chip == NULL) {
		pr_err("%s: failed to allocate GPIO chip\n", __func__);
		return -ENOMEM;
	}

	sprintf(chip->label, "gpio-0");
	chip->regbase = __io_address(NS115_GPIO0_BASE);
	chip->lock = &gpio_wlock;

	if(!chip->regbase) {
		pr_err("%s: failed to remap GPIO mem\n", __func__);
		return -EBUSY;
	}
	__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTA_CTRL);
	if(data) {
		__raw_writel(data->gpio_ddr[4], chip->regbase + GPIO_PORTA_DDR);
	} else {
		__raw_writel(BOARD_GPIO_CFG, chip->regbase + GPIO_PORTA_DDR);
	}

	c = &chip->chip;
	c->label = chip->label;
	c->base  = GPIO_BASE0;
	c->direction_input  = ns115_gpio_direction_input;
	c->direction_output = ns115_gpio_direction_output;
	c->get = ns115_gpio_get;
	c->set = ns115_gpio_set;
	/* number of GPIOs on last bank may be less than 32 */
	c->ngpio = GPIO0_LINE_MAX;
	if(gpiochip_add(c)<0)
		 pr_err("%s: failed to add GPIO chip\n", __func__);
	g_wchip = chip;
	return 0;
}

static int ns115_gpio_irq_type(struct irq_data *d, unsigned int type)
{
	struct ns115_gpio_chip *c;
	int gpio = gpio_to_offset(irq_to_gpio(d->irq));
	unsigned long mask = (1 << gpio);
	unsigned int inten,tmp,pol;
	
	c = irq_to_ns115_chip(d->irq);

	tmp = __raw_readl(c->regbase + GPIO_INTTYPE);
	pol = __raw_readl(c->regbase + GPIO_INTPOL);
	inten = __raw_readl(c->regbase + GPIO_INTEN);

	if (type == IRQ_TYPE_PROBE) {
		/* Don't mess with enabled GPIOs using preconfigured edges or
		 * GPIOs set to alternate function or to output during probe
		 */
		if (inten & mask)
			return 0;
		if(tmp & mask) {
			if(pol & mask)
				type = IRQ_TYPE_EDGE_RISING;
			else
				type = IRQ_TYPE_EDGE_FALLING;
		} else {
			if(pol & mask)
				type = IRQ_TYPE_LEVEL_HIGH;
			else
				type = IRQ_TYPE_LEVEL_LOW;
		}
	}
	if(type & IRQ_TYPE_EDGE_RISING)
		tmp |= mask,pol |=mask;
	if(type & IRQ_TYPE_EDGE_FALLING)
		tmp |= mask,pol &=~mask;
	if(type & IRQ_TYPE_LEVEL_HIGH)
		tmp &= ~mask,pol |=mask;
	if(type & IRQ_TYPE_LEVEL_LOW)
		tmp &= ~mask,pol &=~mask;

	__raw_writel(tmp, c->regbase + GPIO_INTTYPE);
	__raw_writel(pol, c->regbase + GPIO_INTPOL);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__irq_set_handler_locked(d->irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__irq_set_handler_locked(d->irq, handle_edge_irq);


	pr_debug("%s: IRQ%d (GPIO%d) - edge%s%s\n", __func__, d->irq, gpio,
		((type & IRQ_TYPE_EDGE_RISING)  ? " rising"  : ""),
		((type & IRQ_TYPE_EDGE_FALLING) ? " falling" : ""));
	pr_debug("%s: IRQ%d (GPIO%d) - level%s%s\n", __func__, d->irq, gpio,
		((type & IRQ_TYPE_EDGE_RISING)  ? " high"  : ""),
		((type & IRQ_TYPE_EDGE_FALLING) ? " low" : ""));
	return IRQ_SET_MASK_OK;
}

static void ns115_ack_gpio(struct irq_data *d)
{
	int gpio = gpio_to_offset(irq_to_gpio(d->irq));
	struct ns115_gpio_chip *c = irq_to_ns115_chip(d->irq);
	struct irq_chip * gic_chip = c->gic_chip;
	/* clear interrupt for edge trigger of gpio */
	__raw_writel((1<<gpio), c->regbase + GPIO_EOI);
	gic_chip->irq_eoi(d);
}

static int ns115_set_wake(struct irq_data *d, unsigned int on)
{
	void __iomem * wake_addr = __io_address(NS115_PRCM_BASE + 0x14);
	unsigned int val = 0;

	if(d->irq < GPIO_IRQ_BASE0 || d->irq >= (GPIO_IRQ_BASE0 + INT_NUM0)) {
		return -ENXIO;
	}

	val = readl(wake_addr);	
	if(on) {
		val |= (1 << (d->irq - GPIO_IRQ_BASE0));
		val |= (1 << 16);
	} else {
		val &= ~(1 << (d->irq - GPIO_IRQ_BASE0));
	}
	writel(val, wake_addr);
	return 0;
}

#ifdef CONFIG_PM
static int ns115_gpio_suspend(void)
{
	struct ns115_gpio_chip *c;

	c = g_chip;

	c->saved_gpactrl	=	__raw_readl(c->regbase + GPIO_PORTA_CTRL);
	c->saved_gpaddr		=	__raw_readl(c->regbase + GPIO_PORTA_DDR);
	c->saved_gpbctrl	=	__raw_readl(c->regbase + GPIO_PORTB_CTRL);
	c->saved_gpbddr		=	__raw_readl(c->regbase + GPIO_PORTB_DDR);
	c->saved_gpcctrl	=	__raw_readl(c->regbase + GPIO_PORTC_CTRL);
	c->saved_gpcddr		=	__raw_readl(c->regbase + GPIO_PORTC_DDR);
	c->saved_gpdctrl	=	__raw_readl(c->regbase + GPIO_PORTD_CTRL);
	c->saved_gpdddr		=	__raw_readl(c->regbase + GPIO_PORTD_DDR);
	c->saved_ginttype	=	__raw_readl(c->regbase + GPIO_INTTYPE);
	c->saved_gintpol	=	__raw_readl(c->regbase + GPIO_INTPOL);
	c->saved_ginten		=	__raw_readl(c->regbase + GPIO_INTEN);
	c->saved_gpadr		=	__raw_readl(c->regbase + GPIO_PORTA_DR);
	c->saved_gpbdr		=	__raw_readl(c->regbase + GPIO_PORTB_DR);
	c->saved_gpcdr		=	__raw_readl(c->regbase + GPIO_PORTC_DR);
	c->saved_gpddr		=	__raw_readl(c->regbase + GPIO_PORTD_DR);
	/* Clear GPIO transition detect bits */
	/*__raw_writel(0x0, c->regbase + GPIO_INTEN);*/
	printk(KERN_EMERG "gpio suspend!\n");
	return 0;
}

static void ns115_gpio_resume(void)
{
	struct ns115_gpio_chip *c;

	c = g_chip;

	__raw_writel(c->saved_gpactrl,	c->regbase + GPIO_PORTA_CTRL);
	__raw_writel(c->saved_gpadr,	c->regbase + GPIO_PORTA_DR);
	__raw_writel(c->saved_gpaddr,	c->regbase + GPIO_PORTA_DDR);
	__raw_writel(c->saved_gpbctrl,	c->regbase + GPIO_PORTB_CTRL);
	__raw_writel(c->saved_gpbdr,	c->regbase + GPIO_PORTB_DR);
	__raw_writel(c->saved_gpbddr,	c->regbase + GPIO_PORTB_DDR);
	__raw_writel(c->saved_gpcctrl,	c->regbase + GPIO_PORTC_CTRL);
	__raw_writel(c->saved_gpcdr,	c->regbase + GPIO_PORTC_DR);
	__raw_writel(c->saved_gpcddr,	c->regbase + GPIO_PORTC_DDR);
	__raw_writel(c->saved_gpdctrl,	c->regbase + GPIO_PORTD_CTRL);
	__raw_writel(c->saved_gpddr,	c->regbase + GPIO_PORTD_DR);
	__raw_writel(c->saved_gpdddr,	c->regbase + GPIO_PORTD_DDR);
	__raw_writel(c->saved_ginttype,	c->regbase + GPIO_INTTYPE);
	__raw_writel(c->saved_gintpol, 	c->regbase + GPIO_INTPOL);
	__raw_writel(c->saved_ginten,	c->regbase + GPIO_INTEN);
	printk(KERN_EMERG "gpio resume!\n");
}
#else
#define ns115_gpio_suspend	NULL
#define ns115_gpio_resume	NULL
#endif

static struct irq_chip ns115_gpio_irq_chip = {
	.name		= "GPIO",
	.irq_ack	= ns115_ack_gpio,
	.irq_set_type	= ns115_gpio_irq_type,
	.irq_set_wake	= ns115_set_wake,
};

struct syscore_ops ns115_gpio_ops = {
	.suspend	= ns115_gpio_suspend,
	.resume		= ns115_gpio_resume,
};

void __init ns115_init_gpio(struct gpio_data * data)
{
	struct ns115_gpio_chip *c;
	int gpio, irq;
	struct irq_chip *gic_chip = irq_get_chip(gpio_to_irq(0));

	/* Initialize GPIO chip */
	ns115_init_gpio_chip(data);

	c = g_chip;
	c->gic_chip = gic_chip;

	/* GPIO IRQ Configure */
	/*level or edge*/
	__raw_writel(BOARD_GPIO_INTTYPECFG,c->regbase + GPIO_INTTYPE);
	/*positive or negative*/
	__raw_writel(BOARD_GPIO_INTPOLCFG,c->regbase + GPIO_INTPOL);
	/*enable*/
	__raw_writel(BOARD_GPIO_INTEN, c->regbase + GPIO_INTEN);
	__raw_writel(BOARD_GPIO_INTMASK, c->regbase + GPIO_INTMASK);

	c = g_wchip;
	c->gic_chip = gic_chip;

	/* GPIO IRQ Configure */
	/*level or edge*/
	__raw_writel(BOARD_GPIO_INTTYPECFG,c->regbase + GPIO_INTTYPE);
	/*positive or negative*/
	__raw_writel(BOARD_GPIO_INTPOLCFG,c->regbase + GPIO_INTPOL);
	/*enable*/
	__raw_writel(BOARD_GPIO_INTEN, c->regbase + GPIO_INTEN);
	__raw_writel(BOARD_GPIO_INTMASK, c->regbase + GPIO_INTMASK);

	ns115_gpio_irq_chip.irq_mask = gic_chip->irq_mask;
	ns115_gpio_irq_chip.irq_unmask = gic_chip->irq_unmask;
#ifdef CONFIG_SMP
	ns115_gpio_irq_chip.irq_set_affinity = gic_chip->irq_set_affinity;
#endif

	for (gpio = GPIO_BASE0; gpio < GPIO_BASE0 + INT_GPIO; gpio++) {
		irq  = gpio_to_irq(gpio);
		irq_set_chip_and_handler(irq, &ns115_gpio_irq_chip,
					 handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	register_syscore_ops(&ns115_gpio_ops);
}


