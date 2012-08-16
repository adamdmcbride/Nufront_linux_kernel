
#include <asm/io.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <mach/board-ns115.h>
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
	void __iomem * lcdc0 = __io_address(NS115_DISPCTR0_BASE);
	void __iomem * lcdc1 = __io_address(NS115_DISPCTR1_BASE);

	/* close lcd0 & lcd1 */
	writel(0, lcdc0);
	writel(0, lcdc1);

}

void print_pinmux(void) {
	int idx = 0, pin = 0, pidx = 0;
	unsigned int val = 0;
	void __iomem * addr = __io_address(NS115_SCM_BASE);

	for(idx = 0; idx < NS115_PINMUX_SIZE; idx++) {
		val = readl(addr + (idx << 2));
		for(pidx = 0; pidx < 4; pidx ++) {
			printk(KERN_DEBUG"Pin(%03d-%03d) %d %d %d %d\n",\
				pin,pin+3,(val & 0x3),((val >> 2) & 0x3),\
				((val >> 4) & 0x3),((val >> 6) & 0x3));
			val >>= 8;
			pin += 4;
		}
	}	
}

int pinmux_init(unsigned int * setting, int len)
{
	int idx = 0, pflag = 1;
	void __iomem * addr = __io_address(NS115_SCM_BASE);
	if(len != NS115_PINMUX_SIZE) {
		printk(KERN_ERR"PINMUX init data do not match NS115.\n");
		len = len > NS115_PINMUX_SIZE?NS115_PINMUX_SIZE:len;
		pflag = 1;
	}
	for(idx = 0; idx < len; idx++) {
		writel(*(setting+idx),(addr + (idx << 2)));
	}	
	if(pflag)
		print_pinmux();	
	return 0;
}

