/* linux/arch/arm/mach-ns115/efuse.c
 *
 * Copyright (c) 2011 Nufront Co., Ltd.
 *		http://www.nufront.com
 *
 *   Author: zhengxing
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <mach/board-ns115.h>
#include <mach/efuse.h>
#include <mach/hardware.h>
#include <linux/delay.h>



#define READ_EFUSE_DATA(nByte, bitOffset) \
	((nByte - 1) << 9 | (bitOffset / 8))



//#define EFUSE_DEBUG
#if EFUSE_DEBUG
	#define EFuse_debug(fmt, args...)  printk(KERN_INFO "efuse_debug, " fmt "\n" , ##args)
#else
	#define EFuse_debug(fmt, args...)
#endif


static struct EFuse_data   ns115_efuse_data = {0};
static int    bInit = 0;

void dump_efuse_data(struct EFuse_data* pdata) {
	EFuse_debug("#################################");
	EFuse_debug("compile: " __DATE__ " " __TIME__);
	EFuse_debug("pdata.high: %x, pdata.low: %x", pdata->a.high, pdata->a.low);
	EFuse_debug("cpu_max_speed: %d", pdata->b.cpu_max_speed);
	EFuse_debug("leakage_limit: %d", pdata->b.leakage_limit);
	EFuse_debug("site_no: %d",       pdata->b.site_no);
	EFuse_debug("make_second: %d",   pdata->b.make_second);
	EFuse_debug("make_minute: %d",   pdata->b.make_minute);
	EFuse_debug("make_hour: %d",     pdata->b.make_hour);
	EFuse_debug("make_day: %d",      pdata->b.make_day);

	EFuse_debug("make_month: %d",    pdata->b.make_month);
	EFuse_debug("make_year: %d",     pdata->b.make_year);
	EFuse_debug("ATE_no: %d",        pdata->b.ATE_no);
	EFuse_debug("factory_no: %d",    pdata->b.factory_no);
	EFuse_debug("#################################");
}

struct EFuse_data* get_ns115_efuse_data() {
	if (!bInit || (ns115_efuse_data.a.low == 0 && ns115_efuse_data.a.high == 0))
		return NULL;
	return &ns115_efuse_data;
}

#if 0
// this code worked well, so do not remove.
static void read_ns115_efuse_data(struct EFuse_data* pdata) {
	volatile void __iomem* pbase  = ioremap_nocache(NS115_EFUSE_BASE, 4096);
	volatile void __iomem* pcen   = pbase + NS115_EFUSE_CEN;   	// equal to use IO_ADDRESS(..)
	volatile void __iomem* paddr  = pbase + NS115_EFUSE_ADDR;
	volatile void __iomem* preq   = pbase + NS115_EFUSE_REQ;
	volatile void __iomem* pint   = ioremap_nocache(0x05041000, 4096); //pbase + 0x124;
	volatile void __iomem* pintmask= pbase + 0x120;
	volatile void __iomem* pintunmask = pbase + 0x12c;
	volatile void __iomem* pstatus = pbase + 0x100;

	//writel (1, pcen);

	while ((readl(pstatus) & 07) != 3) {
		writel(0x1, pcen);
		udelay(10);
	}

	if ((readl(pstatus) & 07) != 3) {    // check status is standby.
		EFuse_debug("ERROR: efuse status isnot standby!!  init fail. %x", *(int*)pstatus);
	}

	writel(0x1, pintunmask);
	writel(READ_EFUSE_DATA(8, 1024), paddr);    		// offset is 1024, efuse address is 0xe80
	writel(0x1, preq);

	volatile unsigned int *pintdata = pint + 0xd0c;
	//int intdata = *pintdata;
	while(! (readl(pint + 0xd0c) & 1 << 18));

	volatile unsigned int __iomem* pbuff0 = pbase + NS115_EFUSE_DATA_0;
	volatile unsigned int __iomem* pbuff1 = pbase + NS115_EFUSE_DATA_1;

	pdata->a.low  = readl(pbuff0);
	pdata->a.high = readl(pbuff1);

	//int intdata2 = *pintdata;

	writel(0x1, pintmask);
	//intdata = *pintdata & 1 << 18;
	writel(0, pcen);                    // close device;
	iounmap(pint);
	iounmap(pbase);

	EFuse_debug(" dump: %x, %x", pdata->a.low, pdata->a.high);
	//printk("read efuse data. %x, %x, %x\n", intdata, intdata2, *pintdata);
}
#endif

static void read_ns115_efuse_data(struct EFuse_data* pdata) {
	extern unsigned int system_serial_low;
	extern unsigned int system_serial_high;

	if (pdata) {
		pdata->a.low  = system_serial_low;
		pdata->a.high = system_serial_high;
	}
}

void init_ns115_efuse_data() {
	if (bInit)
		return;

	memset(&ns115_efuse_data, 0, sizeof(ns115_efuse_data));

	read_ns115_efuse_data(&ns115_efuse_data);
		bInit = 1;
	dump_efuse_data(&ns115_efuse_data);
}

EXPORT_SYMBOL_GPL(get_ns115_efuse_data);
EXPORT_SYMBOL_GPL(init_ns115_efuse_data);
EXPORT_SYMBOL_GPL(dump_efuse_data);

