/*
 *  ENE IO373X PS/2 kbd, mouse driver
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/mfd/io373x.h>

#define IO373X_REG_KBD_OUT      0x81DC
#define IO373X_REG_AUX_OUT      0x81DE

//#define	PS2_DATA_DEBUG

static char *subdev_names[] = { "kbd", "mou" };

struct io373x_ps2_out
{
    unsigned char data;
    unsigned char flag;
};

struct io373x_ps2 {
    struct platform_device *pdev;
    struct serio *serio;
    struct io373x *io373x;
    unsigned long subdev_id;
    unsigned short out_reg;
    struct notifier_block nb;
    struct mutex lock;
};

static int io373x_ps2_notify(struct notifier_block *nb, unsigned long val, void *data)
{
    struct io373x_ps2 *ps2 = container_of(nb, struct io373x_ps2, nb);   
   
    if (val == ps2->subdev_id) {
        int i;
        struct io373x_subdev_data *subdev_data = data;

        for (i = 0; i < subdev_data->count; i++) {
#ifdef PS2_DATA_DEBUG
	    printk("subdev_id:%ld, %02x \n", val, subdev_data->buf[i]);
#endif

	    if( (val == SUBDEV_ID_KBD)  && ((subdev_data->buf[i] == IO373X_KEY_POWER_DOWN) || 
			(IO373X_KEY_POWER_UP == subdev_data->buf[i])) )
			continue;

            serio_interrupt(ps2->serio, subdev_data->buf[i], 0);
	}
    }
    return 0;
}

static int io373x_ps2_write(struct serio *serio, unsigned char byte)
{
    struct io373x_ps2 *ps2 = 0;
    struct io373x_ps2_out out;
    int ret = 0;
    unsigned int buffer = 0, buffer_1 = 0, count = 10000;
 
    ps2 = serio->port_data;   

    out.flag = 1;
    out.data = byte;
#if 1
    while(count --) {
	io373x_read_regs(ps2->io373x, ps2->out_reg + 1, (unsigned char *) &buffer, 1);
	if((buffer & 1) == 0)
		break;
	udelay(10);
    }	
    io373x_read_regs(ps2->io373x, ps2->out_reg , (unsigned char *) &buffer_1, 1);
#endif
    ret =  io373x_write_regs(ps2->io373x, ps2->out_reg, (unsigned char *) &out, sizeof(out));
 #ifdef PS2_DATA_DEBUG
    printk("subdev_id:%d, write byte : %02x, ret = %d, count = %d, rflag = %x,rdata = %x\n", 
			ps2->subdev_id, byte, ret, count, buffer, buffer_1);
#endif
   return ret;
}

static int io373x_ps2_register_port(struct io373x_ps2 *ps2)
{
    struct serio *serio;
    
    // NOTE: no need to free serio when unregister.
    serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
    if (!serio)
        return -ENOMEM;

    serio->id.type      = (ps2->pdev->id == PDEV_ID_PS2_KBD) ? SERIO_8042_XL : SERIO_PS_PSTHRU;
    serio->write        = io373x_ps2_write;
    serio->dev.parent   = &ps2->pdev->dev;
    snprintf(serio->name, sizeof(serio->name), "io373x %s port", subdev_names[ps2->pdev->id]);

    // We are "io373x-ps2.N/serio", not "io373x-ps2/serioN", where N=0 or 1. That is,
    //   we create 2 io373x-ps2 platform devices, each platform device creates 1 serio.
    snprintf(serio->phys, sizeof(serio->phys), "%s/serio", dev_name(&ps2->pdev->dev));

    ps2->serio = serio;
    serio->port_data = ps2;
    serio_register_port(serio);
    return 0;
}

static int io373x_ps2_probe(struct platform_device *pdev)
{
    struct io373x *io373x = dev_get_drvdata(pdev->dev.parent);
    struct io373x_ps2 *ps2 = 0;
    bool notifier_registerd = false;
    int err = 0;

    printk("io373x_ps2_probe %s, io373x = %x\n", subdev_names[pdev->id], (unsigned int)io373x);

    if (!(ps2 = kzalloc(sizeof(struct io373x_ps2), GFP_KERNEL)))
        return -ENOMEM;

    ps2->io373x = io373x;
    ps2->pdev = pdev;
    ps2->subdev_id = (pdev->id == PDEV_ID_PS2_KBD) ? SUBDEV_ID_KBD : SUBDEV_ID_MOU;
    ps2->out_reg = (pdev->id == PDEV_ID_PS2_KBD) ? IO373X_REG_KBD_OUT : IO373X_REG_AUX_OUT;
    ps2->nb.notifier_call = io373x_ps2_notify;
    platform_set_drvdata(pdev, ps2);

    if ((err = io373x_register_notifier(ps2->io373x, &ps2->nb)))
        goto error_exit;
    else
        notifier_registerd = true;

    if ((err = io373x_ps2_register_port(ps2)))
        goto error_exit;

     printk("io373x_ps2_probe, ps2 = %x\n", (unsigned int)ps2);
    return 0;

error_exit:

    if (ps2->serio)
        serio_unregister_port(ps2->serio);

    if (notifier_registerd)
        io373x_unregister_notifier(ps2->io373x, &ps2->nb);

    kfree(ps2);

    return err;
}

static int io373x_ps2_remove(struct platform_device *pdev)
{
    struct io373x_ps2 *ps2 = platform_get_drvdata(pdev);

    printk("io373x_ps2_remove %s\n", subdev_names[pdev->id]);

    serio_unregister_port(ps2->serio);

    io373x_unregister_notifier(ps2->io373x, &ps2->nb);

    kfree(ps2);

    return 0;
}

static struct platform_driver io373x_ps2_driver = {
    .driver = {
        .name   = "io373x-ps2",
        .owner  = THIS_MODULE,
    },
    .probe      = io373x_ps2_probe,
    .remove     = __devexit_p(io373x_ps2_remove),
};

static int __init io373x_ps2_init(void)
{
    printk("io373x_ps2_init\n");

    return platform_driver_register(&io373x_ps2_driver);
}

static void __exit io373x_ps2_exit(void)
{
    printk("io373x_ps2_exit\n");
    
    platform_driver_unregister(&io373x_ps2_driver);
}

module_init(io373x_ps2_init);
module_exit(io373x_ps2_exit);
MODULE_AUTHOR("flychen");
MODULE_DESCRIPTION("IO373X PS/2 Keyboard, Mouse driver");
MODULE_LICENSE("GPL");
