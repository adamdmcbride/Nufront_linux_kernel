/*
 * Base driver for ENE IO373X chip
 *
 * Copyright (C) 2010 ENE TECHNOLOGY INC.
 *
 */

#include <linux/version.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/mfd/io373x.h>
#include <linux/i2c.h>
#include <asm/mach/irq.h>
//#include <mach/board-ns2816.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include "eneec_ioc.h"
#include "io373x_pw.h"
//#define	 PRINT_DBG_INFO
#define  MFD_DEVS

#ifdef	PRINT_DBG_INFO 
	#define DBG_PRINT(fmt, args...) printk( KERN_INFO "io3730: " fmt, ## args) 
#else 
	#define DBG_PRINT(fmt, args...) /* not debugging: nothing */ 
#endif 

#define	IO373X_REG_POWER	0x806e

#define RAM_SIZE                128
#define IO373X_REG_FIFO         0x81E0
#define IO373X_FIFO_LEN         29
#define NEXT_FIFO_POS(p)        (((p) + 1 == IO373X_FIFO_LEN + 1) ? 0 : ((p) + 1))
#define TO_ID(byte)             ((byte) >> 4)
#define TO_COUNT(byte)          ((byte) & 0x0F)
#define TO_BYTE(id, cnt)        ((id) << 4 | (cnt))
enum { STATE_EXPECT_ID, STATE_GATHER_DATA };

// fifo layout. total 32 bytes
struct fifo {
    unsigned char rp;
    unsigned char wp;
    unsigned char buf[IO373X_FIFO_LEN + 1]; // '+1' for fifo-full condition.
};

struct io373x {
    struct device *dev; // spi_device.dev or i2c_client.dev
    struct io373x_bus_ops *bus_ops;
    struct mutex lock;
    struct blocking_notifier_head notifier_list;
    int irq;
    struct workqueue_struct *work_queue;
    struct work_struct read_work;
    unsigned short chip_id;
    unsigned char rev_id;
    struct fifo fifo;
    int fifo_parsing_state;
    int data_idx;
    struct io373x_subdev_data subdev_data;

// For exposing our userspace API.
    // The main reason to have this class is to make mdev/udev create the
    // /dev/io373x character device nodes exposing our userspace API.
    struct class *io373x_class;
    struct cdev cdev;
    dev_t devt;
// ~
};
static struct io373x  *  g_io373x = NULL;

struct workqueue_struct *io373x_wq = NULL;
EXPORT_SYMBOL_GPL(io373x_wq);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
    #define IO373X_SUBDEV(_name, _id) \
    {                                 \
	    .name   = _name,              \
    	.id     = _id,                \
    }
#else
    #define IO373X_SUBDEV(_name, _id) \
    {                                 \
	    .name   = _name,              \
        .driver_data = (void *) _id   \
    }
#endif

static struct mfd_cell io373x_subdevs[] = {
    IO373X_SUBDEV("io373x-ps2", PDEV_ID_PS2_KBD),
    IO373X_SUBDEV("io373x-pw", PDEV_ID_ONLY_ONE),
    IO373X_SUBDEV("io373x-pwrbutton", PDEV_ID_ONLY_ONE),
    IO373X_SUBDEV("io373x-battery", PDEV_ID_ONLY_ONE),
  /*remove devices not implemented.
    IO373X_SUBDEV("io373x-ps2", PDEV_ID_PS2_MOU),
    IO373X_SUBDEV("io373x-gps", PDEV_ID_ONLY_ONE),
    IO373X_SUBDEV("io373x-hotkey", PDEV_ID_PS2_KBD),*/
};
struct class * getclass(struct io373x *io373x)
{
	return io373x->io373x_class;
} 
//EXPORT_SYMBOL(getclass);
static bool fifo_is_empty(struct fifo *fifo)
{
    return (fifo->rp == fifo->wp);
}

static bool fifo_is_full(struct fifo *fifo)
{
    return (NEXT_FIFO_POS(fifo->wp) == fifo->rp);
}

static unsigned char fifo_pop(struct fifo *fifo)
{
    unsigned char data_byte = fifo->buf[fifo->rp];
    fifo->rp = NEXT_FIFO_POS(fifo->rp);
    return data_byte;
}

static int fifo_get_cur_count(struct fifo *fifo)
{
    if(fifo->wp >= fifo->rp)
        return (fifo->wp - fifo->rp);
    else
        return (IO373X_FIFO_LEN + 1) - fifo->rp + fifo->wp;
}

static unsigned char fifo_get_free_count(struct fifo *fifo)
{
    return IO373X_FIFO_LEN - fifo_get_cur_count(fifo);
}

static bool fifo_is_pos_valid(struct fifo *fifo)
{
    // NOTE: can be equal to fifo-len
    return (0 <= fifo->rp && fifo->rp <= IO373X_FIFO_LEN &&
            0 <= fifo->wp && fifo->wp <= IO373X_FIFO_LEN);
}

int io373x_register_notifier(struct io373x *io373x, struct notifier_block *nb)
{
    DBG_PRINT("io373x_register_notifier io373x = %x, nb = %x\n", io373x, nb);
    return blocking_notifier_chain_register(&io373x->notifier_list, nb);
}
EXPORT_SYMBOL(io373x_register_notifier);

int io373x_unregister_notifier(struct io373x *io373x, struct notifier_block *nb)
{
    return blocking_notifier_chain_unregister(&io373x->notifier_list, nb);
}
EXPORT_SYMBOL(io373x_unregister_notifier);

int io373x_read_regs(struct io373x *io373x, unsigned short start_reg, unsigned char *buf, int byte_cnt)
{    
    int ret = 0;
    mutex_lock(&io373x->lock); 
    ret =  io373x->bus_ops->read_regs(io373x->dev, start_reg, buf, byte_cnt);
    mutex_unlock(&io373x->lock);
    return ret;
}
EXPORT_SYMBOL(io373x_read_regs);

int io373x_write_regs(struct io373x *io373x, unsigned short start_reg, unsigned char *buf, int byte_cnt)
{
    int ret = 0;
    mutex_lock(&io373x->lock); 
    ret = io373x->bus_ops->write_regs(io373x->dev, start_reg, buf, byte_cnt);
    mutex_unlock(&io373x->lock);

    return ret;
}
EXPORT_SYMBOL(io373x_write_regs);

static int io373x_read_fifo(struct io373x *io373x)
{
    // Read the whole fifo including fifo read/write positions.
    int err = io373x_read_regs(io373x, IO373X_REG_FIFO, (unsigned char *) &io373x->fifo, sizeof(struct fifo));
    int i;
    if (err == 0) {
        if (fifo_is_pos_valid(&io373x->fifo)) { // valid fifo positions ?
            if (!fifo_is_empty(&io373x->fifo)) { // read pos != write pos
                // Tell device fifo is now empty.
                err = io373x_write_regs(io373x, IO373X_REG_FIFO + offsetof(struct fifo, rp), &io373x->fifo.wp, 1);
#ifdef PRINT_DBG_INFO                
	 	i = 0;
	 	printk(KERN_INFO " rp = %d, wp = %d\n", io373x->fifo.rp, io373x->fifo.wp);
	 	while(i < 29) {
	 	printk("%02X ",io373x->fifo.buf[i++]);
	 	}
	 	printk(KERN_INFO "\n");
#endif
            }

        } else {
            DBG_PRINT("Invalid io373x read/write positions: %d/%d\n", io373x->fifo.rp, io373x->fifo.wp);
            err = -ENXIO;
        }
    }

    return err;
}

static int io373x_read_fifo_check_header(struct io373x *io373x)
{
	int err = io373x_read_regs(io373x, IO373X_REG_FIFO, (unsigned char *) &io373x->fifo, 2);
	if (err == 0) {
		if (fifo_is_pos_valid(&io373x->fifo)) { // valid fifo positions ?
			if (!fifo_is_empty(&io373x->fifo)) { // read pos != write pos
				return io373x_read_fifo(io373x);
			}
		}
	}
	return -EINVAL;
}

static int ioctl_read_regs(struct io373x *io373x, unsigned long arg)
{
    RWREGS_PARAM param;
    unsigned int bytes_left;
    unsigned char __user *user_buf;
    unsigned short reg;
    unsigned char tmp_buf[RAM_SIZE]; // for buffering user data; user might read a whole RAM.
    int ret;
  
    if (copy_from_user(&param, (void __user *) arg, sizeof(RWREGS_PARAM)))
        return -EFAULT; // some data not copied.

    bytes_left = param.byte_cnt;
    user_buf = (unsigned char __user *) param.buf;
    reg = param.start_reg;

    while (bytes_left) {

        int bytes_this_xfer = min(bytes_left, sizeof(tmp_buf));

        ret = io373x_read_regs(io373x, reg, tmp_buf, bytes_this_xfer);
        if (ret < 0)
            return ret;

        if (copy_to_user(user_buf, tmp_buf, bytes_this_xfer))
            return -EFAULT; // some data not copied.

        bytes_left -= bytes_this_xfer;
        user_buf += bytes_this_xfer;
        reg += bytes_this_xfer;
    }
    
    return 0;  
}

static int ioctl_write_regs(struct io373x *io373x, unsigned long arg)
{
    RWREGS_PARAM param;
    unsigned int bytes_left;
    unsigned char __user *user_buf;
    unsigned short reg;
    unsigned char tmp_buf[RAM_SIZE]; // for buffering user data; user might write a whole RAM.
    int ret;

    if (copy_from_user(&param, (void __user *) arg, sizeof(RWREGS_PARAM)))
        return -EFAULT; // some data not copied.

    bytes_left = param.byte_cnt;
    user_buf = (unsigned char __user *) param.buf;
    reg = param.start_reg;

    while (bytes_left) {

        int bytes_this_xfer = min(bytes_left, sizeof(tmp_buf));

        if (copy_from_user(tmp_buf, user_buf, bytes_this_xfer))
            return -EFAULT; // some data not copied.

        ret = io373x_write_regs(io373x, reg, tmp_buf, bytes_this_xfer);
        if (ret < 0)
            return ret;

        bytes_left -= bytes_this_xfer;
        user_buf += bytes_this_xfer;
        reg += bytes_this_xfer;
    }
    
    return 0; 
}

static int ioctl_enter_code_in_rom(struct io373x *io373x)
{
    return 0;
}

static int ioctl_exit_code_in_rom(struct io373x *io373x)
{
    return 0;
}

static long io373x_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{    
    struct io373x *io373x = filp->private_data;
    int ret = 0;

    //DBG_PRINT("io373x_ioctl()\n");

    // Check type and command number
    if (_IOC_TYPE(cmd) != ENEEC_IOC_MAGIC) {
        DBG_PRINT("Not ENEEC_IOC_MAGIC\n");
        return -ENOTTY;
    }

    switch (cmd)
    {
    case ENEEC_IOC_READ_REGS:
        ret = ioctl_read_regs(io373x, arg);
        break;
    case ENEEC_IOC_WRITE_REGS:
        ret = ioctl_write_regs(io373x, arg);
        break;
    case ENEEC_IOC_GET_CHIPID:
        //ret = ioctl_get_chipid(io373x, arg);
        break;
    case ENEEC_IOC_ENTER_CODE_IN_ROM:
        ret = ioctl_enter_code_in_rom(io373x);        
        break;
    case ENEEC_IOC_EXIT_CODE_IN_ROM:
        ret = ioctl_exit_code_in_rom(io373x);
        break;
    default:
        DBG_PRINT("Unsupported ioctl\n");
        ret = -ENOTTY;
        break;
    }
    
    return ret;
}

static int io373x_open(struct inode *inode, struct file *filp)
{
    struct io373x *io373x = container_of(inode->i_cdev, struct io373x, cdev);

    DBG_PRINT("io373x_open()\n");

    filp->private_data = io373x;

    return 0;    
}

static int io373x_release(struct inode *inode, struct file *filp)
{
    DBG_PRINT("io373x_release()\n");
    return 0;
}

static const struct file_operations io373x_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = io373x_ioctl,
    .open           = io373x_open,
    .release        = io373x_release,
};

static int io373x_create_cdev_node(struct io373x *io373x)
{
    int status;
    dev_t devt;
    struct device *dev;
    struct class *io373x_class;
    bool is_class_created = false, is_region_allocated = false, is_cdev_added = false, is_device_created = false;

    DBG_PRINT("io373x_create_cdev_node .. \n");

    // Create class
    io373x_class = class_create(THIS_MODULE, "io373x");
    status = IS_ERR(io373x_class) ? PTR_ERR(io373x_class) : 0;
    if (status < 0) {
        DBG_PRINT("class_create() failed -- %d\n", status);
        goto error_exit;
    }
    is_class_created = true;

    // Alloc chrdev region.
    status = alloc_chrdev_region(&devt, 0, 1, "io373x");
    if (status < 0) {
        DBG_PRINT("alloc_chrdev_region() failed -- %d\n", status);
        goto error_exit;
    }
    is_region_allocated = true;

    // Add cdev.
    cdev_init(&io373x->cdev, &io373x_fops);
    status = cdev_add(&io373x->cdev, devt, 1);
    if (status < 0) {
        DBG_PRINT("cdev_add() failed -- %d\n", status);
        goto error_exit;
    }
    is_cdev_added = true;

    // Create device
    dev = device_create
                (
                io373x_class, 
                io373x->dev,          // parent device (struct device *)
                devt, 
                io373x,               // caller's context
                "io373x"
                );
    status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    if (status < 0) {
        DBG_PRINT("device_create() failed -- %d\n", status);
        goto error_exit;
    }
    is_device_created = true;

    // Succeed.
    io373x->io373x_class = io373x_class;
    io373x->devt = devt;

    return 0;

error_exit:

    if (is_device_created)
        device_destroy(io373x_class, devt);
    if (is_cdev_added)
        cdev_del(&io373x->cdev);
    if (is_region_allocated)
        unregister_chrdev_region(devt, 1);
    if (is_class_created)
        class_destroy(io373x_class);

    return status;
}

//
// Undo io373x_create_cdev_node().
// Call this only if the char device node was ever created successfully.
static void io373x_destroy_cdev_node(struct io373x *io373x)
{
    device_destroy(io373x->io373x_class, io373x->devt);
    cdev_del(&io373x->cdev);
    unregister_chrdev_region(io373x->devt, 1);
    class_destroy(io373x->io373x_class);
}

static int io373x_add_subdevs(struct io373x *io373x)
{
    int err, i;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
    // 2.6.32's mfd_add_devices(parent, id...) make all platform_device.id = 
    //   (id + mfd_cell.id), here = 0 + -1 or PS2_ID_XXX.
    err = mfd_add_devices(io373x->dev, 0 /*base_id*/, io373x_subdevs, 
                ARRAY_SIZE(io373x_subdevs), NULL, 0);
#else
    // 2.6.31 has no mfd_cell.id field, and mfd_add_devices(parent, id...) make
    //   all platform_device.id = id; So we call mfd_add_devices() many times with
    //   desired id (-1 or PS2_ID_XXX), which we put in mfd_cell.driver_data.
    for (i = 0; i < ARRAY_SIZE(io373x_subdevs); i++) {
        int id = (int) io373x_subdevs[i].driver_data;
        err = mfd_add_devices(io373x->dev, id, &io373x_subdevs[i], 1, NULL, 0);
        if (err)
            break;
    }
#endif

    return err;
}

static void io373x_read_work(struct work_struct *work)
{ 
    struct io373x *io373x = container_of(work, struct io373x, read_work);
//    struct io373x *io373x = (struct io373x *)work;//container_of(work, struct io373x, read_work);
    unsigned char byte;

    DBG_PRINT("io373x_read_work\n");

    if(io373x_read_fifo_check_header(io373x) < 0)
	goto exit_readwork;
   	
    while (!fifo_is_empty(&io373x->fifo)) {

        // Parse the fifo.
        do
        {
            switch(io373x->fifo_parsing_state)
            {
            case STATE_EXPECT_ID:
                byte = fifo_pop(&io373x->fifo);
                io373x->subdev_data.dev_id = TO_ID(byte);
                io373x->subdev_data.count = TO_COUNT(byte);
                io373x->data_idx = 0;
		if(io373x->subdev_data.count > 0) 
                	io373x->fifo_parsing_state = STATE_GATHER_DATA;
		DBG_PRINT("STATE_EXPECT_ID,dev_id: %02X, count: %02X\n", io373x->subdev_data.dev_id, io373x->subdev_data.count);
                break;

            case STATE_GATHER_DATA:
                io373x->subdev_data.buf[io373x->data_idx++] = fifo_pop(&io373x->fifo);
		DBG_PRINT("STATE_GATHER_DATA %02x \n", io373x->subdev_data.buf[io373x->data_idx - 1] );
                if(io373x->data_idx == io373x->subdev_data.count) {
                    io373x->fifo_parsing_state = STATE_EXPECT_ID;
                    blocking_notifier_call_chain(&io373x->notifier_list, 
                        io373x->subdev_data.dev_id, &io373x->subdev_data);
                }
                break;    
            }

        } while(!fifo_is_empty(&io373x->fifo));

        // Read the fifo AGAIN. Maybe new data come during parsing.
    // if(io373x_read_fifo(io373x) < 0)
    //	goto exit_readwork;
    }

exit_readwork:
    enable_irq(io373x->irq);
}

static irqreturn_t io373x_interrupt(int irq, void *dev_id)
{
    struct io373x *io373x = dev_id;

    disable_irq_nosync(irq);
    queue_work(io373x->work_queue, &io373x->read_work);

    return IRQ_HANDLED;
}
static void ns2816_power_off(void)
{
	int err = 0;
	int pm_status = 0;
	unsigned char pm_off[3] = {150, 0, 0};

	printk(KERN_EMERG "ns2816_power_off ...\n");
/*
	if(g_io373x == NULL)
		return ;
*/
	//write 0x806e -> 9
	//write 0x8051 = 150
	//write 0x8052 = 0
	//write 0x8053 = 0
	io373x_poweroff();
#if 0
    	err = io373x_read_regs(g_io373x, IO373X_REG_POWER, &pm_status, 1);
	if(err == 0) {
		printk(KERN_EMERG "power register %x:%d\n", IO373X_REG_POWER, pm_status);
		pm_status = 9;	
		io373x_write_regs(g_io373x, 0x8051, pm_off, 3); 	
		io373x_write_regs(g_io373x, IO373X_REG_POWER, &pm_status, 1); 	
    		err = io373x_read_regs(g_io373x, IO373X_REG_POWER, &pm_status, 1);
		printk(KERN_EMERG "power register %x:%d\n", IO373X_REG_POWER, pm_status);
	}
#endif
}


struct io373x *io373x_probe(struct device *dev, int irq, struct io373x_bus_ops *bus_ops)
{
    struct io373x *io373x = NULL;
    bool irq_requested = false, subdevs_added = false, cdev_node_created = false, work_queue_created = false;
    int err = 0,i;

    DBG_PRINT("io373x_probe with irq = %d\n", irq);

    if (irq <= 0) {
        DBG_PRINT("ERROR: io373x_probe(), irq is not specified.\n");
        return ERR_PTR(-ENODEV);
    }

    if (!(io373x = kzalloc(sizeof(*io373x), GFP_KERNEL)))
        return ERR_PTR(-ENOMEM);

    io373x->dev = dev;
    io373x->irq = irq;
    io373x->bus_ops = bus_ops;

    if(g_io373x == NULL)
	g_io373x = io373x;


    mutex_init(&io373x->lock);

    i2c_set_clientdata(to_i2c_client(dev), io373x);

#if 0 
        io373x_read_regs(io373x, 0x8434,io373x->subdev_data.buf,12);
	for(i =0; i < 12; i++)
		printk("%04X: %02X \n", 0x8434 + i, io373x->subdev_data.buf[i]);	
#if 0	
	 io373x_read_regs(io373x, 0x8400,io373x->subdev_data.buf,16);
	for(i =0; i < 16; i++)
		printk("%04X: %02X \n", 0x8400 + i, io373x->subdev_data.buf[i]);	
#endif	
	io373x_read_regs(io373x, 0xfd10,io373x->subdev_data.buf,4);
	for(i =0; i < 4; i++)
		printk("%04x: %02X \n", 0xfd10 + i, io373x->subdev_data.buf[i]);		
	
#endif

    if((io373x->work_queue = create_singlethread_workqueue("io373x"))) {
        work_queue_created = true;
        INIT_WORK(&io373x->read_work, io373x_read_work);
    } else {
        err = -ENOMEM;
        DBG_PRINT("Failed to create workqueue\n");
        goto error_exit;
    }
    if(io373x_wq == NULL)
	io373x_wq = io373x->work_queue;


    err = request_irq(irq, io373x_interrupt, IRQF_TRIGGER_FALLING | IRQF_SHARED |IRQF_DISABLED /*IRQF_TRIGGER_FALLING*/, "io373x", io373x);
    if (!err) {
        irq_requested = true;
        DBG_PRINT("IRQ %d requested\n", irq);
        BLOCKING_INIT_NOTIFIER_HEAD(&io373x->notifier_list);
    } else {
        DBG_PRINT("Failed to request IRQ %d -- %d\n", irq, err);
        goto error_exit;
    }
    err = gpio_request(irq_to_gpio(irq),"EC_IRQ");
    if(err<0) {
	    DBG_PRINT("failed to request gpio %d -- %d\n",irq_to_gpio(irq),err);
	    goto error_exit;
    }
    DBG_PRINT("request gpio %d ok -- %d\n",irq_to_gpio(irq),err);
#ifdef MFD_DEVS
    err = io373x_add_subdevs(io373x);
    if (!err)
        subdevs_added = true;
    else {
        DBG_PRINT("Failed to add sub devices\n");
        goto error_exit;
    }
    DBG_PRINT("io373x_add_subdevs success\n");
#endif

    err = io373x_create_cdev_node(io373x);
    if (!err)
        cdev_node_created = true;
    else
        goto error_exit;
    pm_power_off = ns2816_power_off;

	DBG_PRINT("start init io3730 fifo reg\n");
	char io3x30_buf[IO373X_FIFO_LEN] = {0};
	err = io373x_write_regs(io373x, IO373X_REG_FIFO, io3x30_buf, IO373X_FIFO_LEN);
	if(err){
		printk("init io3730 fifo reg failed\n");
	}

    return io373x;

error_exit:

    if (cdev_node_created)
        io373x_destroy_cdev_node(io373x);

#ifdef MFD_DEVS
    if (subdevs_added)
        mfd_remove_devices(io373x->dev);
#endif

    if (irq_requested)
        free_irq(io373x->irq, io373x);

    if (work_queue_created)
        destroy_workqueue(io373x->work_queue);

    kfree(io373x);

    return ERR_PTR(err);
}

int io373x_remove(struct io373x *io373x)
{
    DBG_PRINT("io373x_remove\n");
    io373x_destroy_cdev_node(io373x);


#ifdef MFD_DEVS
    mfd_remove_devices(io373x->dev);
#endif

    free_irq(io373x->irq, io373x);

    destroy_workqueue(io373x->work_queue);

    kfree(io373x);

    return 0;
}

MODULE_AUTHOR("flychen");
MODULE_DESCRIPTION("IO373X driver");
MODULE_LICENSE("GPL");

