#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/cnt32_to_63.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <asm/clkdev.h>
#include <asm/delay.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/board-ns115.h>
#include <mach/platform.h>

#include "prcm.h"
#include "scm.h"

#if 0
/* software should first write 0b00011011, 
 * and then write 0b11100100 to efuse_read_cfg, 
 * efuse data will be valid on efuse_data_out(offset 0x80~0xfc) after 35us
 */
void efuse_read_config(void)
{
	void __iomem *addr = __io_address(SCM_EFUSE_CONFIG);
	u32 tmp;

	__raw_writel(0x1b, addr);
	//flush
	tmp = __raw_readl(addr);

	__raw_writel(0xe4, addr);
	//flush
	tmp = __raw_readl(addr);
}

void debug_signale_group_select(unsigned int sel)
{
	void __iomem *addr = __io_address(SCM_DEBUG_SIG_SEL);
	
	__raw_writel(sel, addr);
}

void sub_system_arbitration_control(void)
{

	return;
/*
	unsigned int value;
	unsigned int default_value = 0x00003210;
	void __iomem *addr = __io_address(SCM_SYSTEM_ARB_SET);

	value = __raw_readl(addr);


	if(default_value != value)
	{
		__raw_writel(default_value, addr);
		printk("set sub system arbitration control register value to defalue value.\n");
	}
*/
}

/* change usb phy voltage level
 * args: flags!=0, means add Valid threshold
 *	 flags==0, means sub Valid threshold
 *	 level, add or sub voltage level, example value 9 means 9%
 */
void usb_phy_voltage_change(int flags, unsigned int level)
{
	unsigned int value;
	void __iomem *addr = __io_address(SCM_USB_PHY_ANALOG_TUNE_SHARP);

	value = __raw_readl(addr);
	value &= (~0x7);

	if(flags)
	{
		switch(level)
		{
			case 9:
				value |= 0x7;
				break;
			case 6:
				value |= 0x6;
				break;
			case 3:
				value |= 0x5;
				break;
			default:
				value |= 0x4;
				break;
		}
	}else{
		switch(level)
		{
			case 3:
				value |= 0x3;
				break;
			case 6:
				value |= 0x2;
				break;
			case 9:
				value |= 0x1;
				break;
			case 12:
				value |= 0x0;
				break;
			default:
				value |= 0x4;
				break;
		}
	}

	__raw_writel(value, addr);	

	printk("change usb phy voltage level to 0x%x [2:0] bit\n", value);
}

//program sata control 0 register bit[5:3], let it's value equal to 011b
void sata_control(void)
{
	unsigned int value;
	void __iomem *addr = __io_address(SCM_SATA_CONTROL_0);

	value = __raw_readl(addr);
	if(0x18 == (value & 0x38))
	{
		printk("SATA Control 0 value is alread equal to 0x%08x\n", value);
		return;
	}

	value &= (~0x38); //[5:3] clear
	value |= 0x18; //[5:3] set 011
	__raw_writel(value, addr);
	
	value = __raw_readl(addr);
	printk("Change SATA Control 0's value to 0x%08x\n", value);
}

unsigned int cpu_status(void)
{
	unsigned int value;
	void __iomem *addr = __io_address(SCM_CPU_STATUS);

	value = __raw_readl(addr);
	return value;
}

void scm_init(void)
{
	sata_control();
	sub_system_arbitration_control();
}
#endif
