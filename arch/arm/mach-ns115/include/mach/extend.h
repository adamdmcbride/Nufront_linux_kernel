#ifndef __MACH_NS115_EXTEND_H_
#define __MACH_NS115_EXTEND_H_

#define EXT_IRQ_NOTSPEC	(-1)
#define USE_DEFAULT (-1)
#define EXT_I2C_DEV(bus,dev,pdata,nirq,naddr) \
	{.bus_id = (bus), .bd = dev, .data = pdata, \
	 .irq = (nirq), .new_addr = (naddr)}

struct extend_i2c_device {
	int bus_id;
	struct i2c_board_info *bd;	/*can not be NULL */
	struct platfrom_data *data;
	int irq;
	int new_addr;
};

struct touch_panel_platform_data {
	int irq_reset;
};

extern struct i2c_board_info __initdata ns115_gs_kxud9;
extern struct i2c_board_info __initdata ns115_gs_kxtj9;
extern struct i2c_board_info __initdata ns115_ls_cm3212;
extern struct i2c_board_info __initdata ns115_tp_goodix;
extern struct i2c_board_info __initdata ns115_tp_gt811;
extern struct i2c_board_info __initdata ns115_tp_ft5x06;
extern struct i2c_board_info __initdata ns115_ec_io373x;
extern struct i2c_board_info __initdata ns115_snd_wm8960;
extern struct i2c_board_info __initdata ns115_snd_alc5631;
extern struct i2c_board_info __initdata ns115_snd_alc3261;
extern struct i2c_board_info __initdata ns115_hdmi_7033;
extern struct i2c_board_info __initdata ns115_hdmi_7033_audio;
extern struct i2c_board_info __initdata ns115_tp_sis;
extern struct i2c_board_info __initdata ns115_tp_zt2083;
extern struct i2c_board_info __initdata ns115_tp_ilitek;
extern struct i2c_board_info __initdata ns115_cs_ami30x;
extern struct i2c_board_info __initdata tps80032_i2c_dev;
extern struct i2c_board_info __initdata ns115_snesor_tmd;
/*support ns115_phone*/
extern struct i2c_board_info __initdata ns115_sensor_mpu3050;
extern struct i2c_board_info __initdata ns115_sensor_bma250;
/*support ns115_pad*/
extern struct i2c_board_info __initdata ns115_gs_dmard06;
extern struct i2c_board_info __initdata ns115_gs_bma250;
extern int __init ext_i2c_register_devices(struct extend_i2c_device *devs,
					   int size);

#endif
