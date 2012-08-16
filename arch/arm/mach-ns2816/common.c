
#include <asm/io.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <mach/board-ns2816.h>
#include <mach/hardware.h>

#include "common.h"

void common_init(void)
{
        unsigned int reg;
        asm volatile ("mrc p15, 0, %0, c15, c0, 0" : "=r" (reg) : : "cc");
        reg |= 1; 			/*enable dynamic clock gating*/
        asm volatile ("mcr p15, 0, %0, c15, c0, 0" : : "r" (reg) : "cc");

}

void ddr_pm_init(void)
{
	void __iomem * alpmr = __io_address(NS2816_DDR_CFG_BASE + 0xbc);	
	void __iomem * iocr = __io_address(NS2816_DDR_CFG_BASE + 0x8);	
	void __iomem * hpcr0 = __io_address(NS2816_DDR_CFG_BASE + 0x200);	
	void __iomem * lcdc = __io_address(NS2816_LCDC_BASE);
	unsigned int reg = readl(alpmr);
	reg |= (0x3<<25) | (0xa0);	/*timing test needed.auto pm for ddr*/
	writel(reg, alpmr);
	reg = readl(iocr);
	reg |= (0x3<<30) | (0x3<<18) | (0x3 << 22);	/*ODT,PAD...*/
	writel(reg, iocr);
	
	writel(0, lcdc);
	writel(0, lcdc+0x18);

	writel(0x200, hpcr0);
}

void pinmux_set(int group, int value)
{
       void __iomem * addr;
       unsigned int mask = 0;
       unsigned int regval = 0;
       if(group <= 9)
               addr = __io_address(NS2816_SCM_BASE+0x4c);
       else
               addr = __io_address(NS2816_SCM_BASE+0x50);

       regval = __raw_readl(addr);
       group %= 10;
       mask = (7 << (group * 3));
       value = ((value & 0x7)<<(group * 3));
       regval &= ~mask;
       regval |= value;
       __raw_writel(regval, addr);
       printk(KERN_EMERG"%p:%08x.\n", addr, regval);
}

int pinmux_init(struct pinmux_setting * setting, int len)
{
	unsigned int flag = 0;
	int idx = 0;
	for(idx = 0; idx < len; idx++) {
		if(setting[idx].group < PINMUX_GROUP_MAX && \
			!(flag & BIT(setting[idx].group))) {
			pinmux_set(setting[idx].group, setting[idx].value);
			flag |= BIT(setting[idx].group);
		} else {
			printk(KERN_ERR"suspicious setting of \
				pinmux group %d\n", setting[idx].group);
			return -EINVAL;
		};
	}	
	printk(KERN_EMERG"pinmux init ok.\n");
	return 0;
}

