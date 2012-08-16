/*
 * Base driver for ENE IO373X chip (I2C)
 *
 * Copyright (C) 2010 ENE TECHNOLOGY INC.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mfd/io373x.h>

/*
 * Be sure system is added with one i2c board info for one of the KBC chip
 * at mainboard arch_initcall() code, for example,
 *
 * static struct i2c_board_info system_i2c_devs[] = {
 *     { .type = "io373x-i2c", // one of i2c_device_id.name below;
 *       .addr = 0x18,     // i2c slave address for EC, which must be the same as EC's SMBRSA register(offset 0xFFA4) bit7:1.
 *       .irq  = 220,      // irq used for request_irq().
 *     },
 * };
 *
 * static void __init mini2440_machine_init()
 * {
 *     i2c_register_board_info(0, system_i2c_devs, ARRAY_SIZE(system_i2c_devs));
 * }
 */
static const struct i2c_device_id io373x_idtable[] = {
    //  name          driver_data
    { "io373x-i2c",   0x37300000 },
    { }
};

MODULE_DEVICE_TABLE(i2c, io373x_idtable);

#define CMD_SET_ADR     0x00
#define CMD_READ_BYTE   0x81
#define CMD_READ_WORD   0x82
#define CMD_READ_BLOCK  0x80
#define CMD_WRITE_BYTE  0x01
#define CMD_WRITE_WORD  0x02
#define CMD_WRITE_BLOCK 0x03

static int io373x_i2c_set_adr(struct i2c_client *client, unsigned short reg)
{
    unsigned short data = (reg << 8) | (reg >> 8);
    return i2c_smbus_write_word_data(client, CMD_SET_ADR, data);
}

static int io373x_read_reg_1byte(struct i2c_client *client, unsigned char *buf)
{
    buf[0] = i2c_smbus_read_byte_data(client, CMD_READ_BYTE);
    return 0;
}

static int io373x_read_reg_2byte(struct i2c_client *client, unsigned short *buf)
{
    buf[0] = i2c_smbus_read_word_data(client, CMD_READ_WORD);
    return 0;
}

static int io373x_i2c_read_regs(struct device *dev, unsigned short start_reg, unsigned char *buf, int byte_cnt)
{
    struct i2c_client *client = to_i2c_client(dev);
    unsigned char tmp_buf[I2C_SMBUS_BLOCK_MAX];
    unsigned char cmd;
    int bytes_this_read;
    int bytes_left = byte_cnt;
    int ret;

    ret = io373x_i2c_set_adr(client, start_reg);
    if (ret < 0)
        return ret;

    while (bytes_left) {
        if (bytes_left == 1) // should use CMD_READ_BYTE to read 1 byte ...
            return io373x_read_reg_1byte(client, buf);
        else if (bytes_left == 2) // should use CMD_READ_WORD to read 2 bytes ...
            return io373x_read_reg_2byte(client, (unsigned short *) buf);
        else { // use CMD_BLOCK_READ to read 3~32 bytes.

            bytes_this_read = min(32, bytes_left); // we can do block read for max 32 bytes.
          //  bytes_this_read = min(I2C_SMBUS_BLOCK_MAX - 1, bytes_this_read); // i2c_smbus_read_i2c_block_data() supports max I2C_SMBUS_BLOCK_MAX, and 1st byte is our CNT, so only I2C_SMBUS_BLOCK_MAX-1 valid.
            bytes_this_read = min(4, bytes_this_read); // i2c_smbus_read_i2c_block_data() supports max I2C_SMBUS_BLOCK_MAX, and 1st byte is our CNT, so only I2C_SMBUS_BLOCK_MAX-1 valid.
            cmd = CMD_READ_BLOCK | (bytes_this_read & 0x1F); // cmd=0x80: read 32 bytes, cmd=0x80+cnt: read cnt bytes, cnt=3~31

            // read one more byte for our CNT.
            if (i2c_smbus_read_i2c_block_data(client, cmd, bytes_this_read + 1, tmp_buf) != bytes_this_read + 1) {
		printk("i2c_smbus_read_i2c_block_data fail\n");
                return -EIO;
	    }

            memcpy(buf, tmp_buf + 1, bytes_this_read); // first byte is CNT returned by 3730.

            bytes_left -= bytes_this_read;
            buf += bytes_this_read;
        }
    }

    return 0;
}

static int io373x_i2c_write_regs(struct device *dev, unsigned short start_reg, unsigned char *buf, int byte_cnt)
{
    struct i2c_client *client = to_i2c_client(dev);
    int bytes_this_write;
    int bytes_left = byte_cnt;
    int ret;

    ret = io373x_i2c_set_adr(client, start_reg);
    if (ret < 0)
        return ret;

    while (bytes_left) {

        bytes_this_write = min(32, bytes_left); // we can do block write for max 32 bytes.
        bytes_this_write = min(I2C_SMBUS_BLOCK_MAX, bytes_this_write); // i2c_smbus_write_block_data() supports max this many.

        ret = i2c_smbus_write_block_data(client, CMD_WRITE_BLOCK, bytes_this_write, buf);
        if (ret < 0)
            return ret;

        bytes_left -= bytes_this_write;
        buf += bytes_this_write;
    }

    return 0;
}

static struct io373x_bus_ops io373x_i2c_bus_ops = {
    .bustype    = IO373X_BUS_I2C,
    .read_regs  = io373x_i2c_read_regs,
    .write_regs = io373x_i2c_write_regs,
};


static int io373x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct io373x *io373x = 0;

    printk("io373x_i2c_probe name = %s, addr = %d, irq = %d\n", client->name, client->addr, client->irq);

    io373x = io373x_probe(&client->dev, client->irq, &io373x_i2c_bus_ops);

    if (IS_ERR(io373x))
        return PTR_ERR(io373x);

    i2c_set_clientdata(client, io373x);

    printk("io373x = %x, client = %x, client->dev = %x\n", 
		(unsigned int)io373x, 
		(unsigned int)client, 
		(unsigned int)&client->dev);
    return 0;
}

static int io373x_i2c_remove(struct i2c_client *client)
{
    struct io373x *io373x = i2c_get_clientdata(client);

    io373x_remove(io373x);

    return 0;
}

static struct i2c_driver io373x_i2c_driver = {
    .driver = {
        .name   = "io373x-i2c",
        .owner  = THIS_MODULE,
    },
    .probe      = io373x_i2c_probe,
    .remove     = __devexit_p(io373x_i2c_remove),
    .id_table   = io373x_idtable,
};

static int __init io373x_i2c_init(void)
{
    printk("io373x_i2c_init\n");

    return i2c_add_driver(&io373x_i2c_driver);
}

static void __exit io373x_i2c_exit(void)
{
    printk("io373x_i2c_exit\n");
    
    i2c_del_driver(&io373x_i2c_driver);
}

module_init(io373x_i2c_init);
module_exit(io373x_i2c_exit);
MODULE_AUTHOR("flychen");
MODULE_DESCRIPTION("IO373X I2C driver");
MODULE_LICENSE("GPL");

