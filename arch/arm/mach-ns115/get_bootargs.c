/*
 *  linux/arch/arm/mach-ns115/get_bootargs.c
 *
 *  ns115 get bootargs
 *
 *  Author:	jinying wu
 *  Copyright:	Nusmart
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <asm/system.h>
#include <linux/kernel.h>

#include <mach/get_bootargs.h>

static unsigned long pmem_base   = 0xB3800000;
static unsigned long pmem_len    = 0x00000000;	//No pmem
static unsigned long fb_mem_len  = 0x02000000;	//32M
static unsigned long vpu_mem_len = 0x0a000000;	//160M
static unsigned long zsp_mem_len = 0x00800000;	//8M
static unsigned long gpu_mem_len = 0x10000000;	//256M
static unsigned long ump_mem_len = 0x09000000;	//144M

unsigned long nusmart_parsemem(char *str)
{

	char *endptr;   /* local pointer to end of parsed string */
	unsigned long ret = simple_strtoul(str, &endptr, 10);

	printk(KERN_EMERG "nusmart_parsemem ret = %ld endptr=%c\n", ret, *endptr);

	switch (*endptr) {
	case 'U':
		ret <<= 40;
	case 'u':
		ret <<= 40;
	case 'G':
		ret <<= 30;
	case 'g':
		ret <<= 30;
	case 'M':
		ret <<= 20;
	case 'm':
		ret <<= 20;
	case 'K':
		ret <<= 10;
	case 'k':
		ret <<= 10;
	default:
		break;
	}

	wmb();
	rmb();	
	printk(KERN_EMERG "nusmart_parsemem is 0x%0x\n", ret);
	return ret;
}

static int __init pmem_mem_setup(char *str)
{
	char *s;

	for (s = str; *s; s++)
		if (*s == ',')
			break;

	pmem_base = simple_strtoul(str, NULL, 16);
	s++;

	//pmem_len = simple_strtoul(s, NULL, 10);
	pmem_len = memparse(s, &s);

	printk(KERN_EMERG "pmem_base 0x%0x pmem_len 0x%0x\n",
		pmem_base, pmem_len);

	return 1;
}

static int __init gpu_mem_setup(char *str)
{

	gpu_mem_len = memparse(str, &str);

	printk(KERN_EMERG "gpu_mem_len is 0x%0x\n", gpu_mem_len);
	return 1;
}

static int __init ump_mem_setup(char *str)
{

	ump_mem_len = memparse(str, &str);

	printk(KERN_EMERG "ump_mem_len is 0x%0x\n", ump_mem_len);
	return 1;
}

static int __init vpu_mem_setup(char *str)
{
	vpu_mem_len = memparse(str, &str);

	printk(KERN_EMERG "vpu_mem_len is 0x%0x\n", vpu_mem_len);
	return 1;
}

static int __init fb_mem_setup(char *str)
{

	fb_mem_len = memparse(str, &str);

//	get_option(&str, &fb_mem_len);

	printk(KERN_EMERG "fb_mem_len is 0x%0x\n", fb_mem_len);
	return 1;
}

__setup("pmem=", pmem_mem_setup);
__setup("gpumem=", gpu_mem_setup);
__setup("umpmem=", ump_mem_setup);
__setup("vpumem=", vpu_mem_setup);
__setup("fbmem=", fb_mem_setup);

unsigned int nusmart_pmem_base(void)
{
	return pmem_base; 
}

EXPORT_SYMBOL(nusmart_pmem_base);

unsigned int nusmart_pmem_len(void)
{
	return pmem_len; 
}
EXPORT_SYMBOL(nusmart_pmem_len);

unsigned int nusmart_lcd_base(void)
{
	return (pmem_base + pmem_len);
}
EXPORT_SYMBOL(nusmart_lcd_base);

unsigned int nusmart_lcd_len(void)
{
	return (fb_mem_len);
}
EXPORT_SYMBOL(nusmart_lcd_len);

unsigned int nusmart_on2_base(void)
{
	return (pmem_base + pmem_len + fb_mem_len);
}
EXPORT_SYMBOL(nusmart_on2_base);

unsigned int nusmart_on2_len(void)
{
	return (vpu_mem_len);
}
EXPORT_SYMBOL(nusmart_on2_len);

unsigned int nusmart_zsp_base(void)
{
	return (pmem_base + pmem_len + fb_mem_len + vpu_mem_len);
}
EXPORT_SYMBOL(nusmart_zsp_base);

unsigned int nusmart_zsp_len(void)
{
	return (zsp_mem_len);
}
EXPORT_SYMBOL(nusmart_zsp_len);

unsigned int nusmart_mali_len(void)
{
	return (gpu_mem_len);
}
EXPORT_SYMBOL(nusmart_mali_len);

unsigned int nusmart_mali_ump_len(void)
{
	return (ump_mem_len);
}
EXPORT_SYMBOL(nusmart_mali_ump_len);

unsigned int nusmart_mali_memvalid_size(void)
{
	return (pmem_len + fb_mem_len);
}
EXPORT_SYMBOL(nusmart_mali_memvalid_size);
