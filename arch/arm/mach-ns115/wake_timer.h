
#ifndef _WAKE_TIMER_H_
#define _WAKE_TIMER_H_

#define OFFSET	0x14
#define LOAD	0x0
#define CUR	0x4
#define CTRL	0x8
#define EOI	0xc
#define ISTS	0x10

#define CTRL_EN_BIT  	0x1
#define CTRL_MODE_BIT	(0x1<<1)
#define CTRL_MASK_BIT	(0x1<<2)

#define FREQ_REF_KHZ	12000

struct wake_timer
{
	void __iomem * mmio;	
	int irq[2];
	struct platform_device * pdev;
	struct wake_lock wake_lock;
	int wake_ms;
	int suspend_ms;
	int can_suspend;
	int stat;
	int count;
#ifdef CONFIG_PM_WAKEUP_DEVICE_AUTO_TEST_SUSPEND
	struct input_dev *input_dev;
#endif
};

#endif

