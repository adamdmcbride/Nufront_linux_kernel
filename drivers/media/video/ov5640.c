/*
 * A V4L2 driver for Micron OV5640 cameras.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>

#include "ov5640.h"

//#define DEBUG_EN
#ifdef DEBUG_EN
#define  PDBG(format,...)               printk("[%d:%s]:"format, \
		__LINE__,__func__ , ##__VA_ARGS__ )
#else
#define  PDBG(format,...)               do{}while(0)
#endif

#define V4L2_CID_CAMERA_SPECIAL_EFFECT  (V4L2_CID_PRIVATE_BASE + 124)
#define V4L2_CID_CAMERA_FACING		(V4L2_CID_PRIVATE_BASE + 125)
#define V4L2_CID_AEC_GAIN               (V4L2_CID_PRIVATE_BASE + 126)
#define V4L2_CID_PRE_CAP		(V4L2_CID_PRIVATE_BASE + 127)

#define AFC_CMD_MASK 		0x00ff0000
#define AFC_X_MASK		0x000000ff
#define AFC_Y_MASK		0x0000ff00

#define AFC_CMD_SHIFT		16
#define AFC_X_SHIFT		0
#define AFC_Y_SHIFT		8

#define AFC_GET_CMD(_v)		(( _v & AFC_CMD_MASK ) >> AFC_CMD_SHIFT)
#define AFC_GET_X(_v)		(( _v & AFC_X_MASK ) >> AFC_X_SHIFT)
#define AFC_GET_Y(_v)		(( _v & AFC_Y_MASK ) >> AFC_Y_SHIFT)

#define AFC_SET_CMD(_v,_a) 	\
	do {\
		_v |= (_a & (AFC_CMD_MASK >> AFC_CMD_SHIFT)) << AFC_CMD_SHIFT;\
	} while(0)

#define AFC_SET_X(_v,_a) 	\
	do {\
		_v |= (_a & (AFC_X_MASK >> AFC_X_SHIFT)) << AFC_X_SHIFT;\
	} while(0)

#define AFC_SET_Y(_v,_a) 	\
	do {\
		_v |= (_a & (AFC_Y_MASK >> AFC_Y_SHIFT)) << AFC_Y_SHIFT;\
	} while(0)

#define DRIVER_NAME	"ov5640"

#define XVCLK		2400

#define to_ov5640(sd)		container_of(sd, struct ov5640_priv, subdev)

enum {
	AFC_CMD_SING,	// single
	AFC_CMD_CONT,	// continuous
	AFC_CMD_PAUS,	// pause
	AFC_CMD_RELE,	// release
	AFC_CMD_RELA,	// re-launch
};

enum {
	AFC_RES_RUNNING,
	AFC_RES_SUCC,
	AFC_RES_FAIL,
};

enum v4l2_camera_facing {
	CAMERA_FACING_BACK  = 0,
	CAMERA_FACING_FRONT = 1,
};

enum {
	SPEC_EFF_NORMAL = 0, 	// "Normal",
	SPEC_EFF_BLUE,		//"Blueish",
	SPEC_EFF_RED,		//"Redish",
	SPEC_EFF_GREEN,		//"Greenish",
	SPEC_EFF_BLWH,		//"Black and white",
	SPEC_EFF_SEPIA,		//"Sepia",
	SPEC_EFF_NEG,		//"Negative",
	SPEC_EFF_OVEX,		//"Over exposure",
	SPEC_EFF_SOL,		//"Solarize",
};

/* supported resolutions */
enum {
	OV5640_176x144,
	OV5640_QVGA,
	OV5640_352x288,
	OV5640_VGA,
	//OV5640_720x480,
	//OV5640_1024x768,
	OV5640_720P,
	OV5640_2M,
	OV5640_3M,
	OV5640_5M,
};

struct ov5640_resolution {
	unsigned int width;
	unsigned int height;
};

static struct ov5640_resolution ov5640_resolutions[] = {
	[OV5640_176x144] = {
		.width	= 176,
		.height	= 144,
	},
	[OV5640_QVGA] = {
		.width	= 320,
		.height	= 240,
	},
	[OV5640_352x288] = {
		.width	= 352,
		.height	= 288,
	},
	[OV5640_VGA] = {
		.width	= 640,
		.height	= 480,
	},
	//[OV5640_720x480] = {
	//	.width	= 720,
	//	.height	= 480,
	//},
	//[OV5640_1024x768] = {
	//	.width	= 1024,
	//	.height	= 768,
	//},
	[OV5640_720P] = {
		.width	= 1280,
		.height	= 720,
	},
	[OV5640_2M] = {
		.width	= 1600,
		.height	= 1200,
	},
	[OV5640_3M] = {
		.width	= 2048,
		.height	= 1536,
	},
	[OV5640_5M] = {
		.width	= 2592,
		.height	= 1944,
	},
};

struct ov5640_priv {
	struct v4l2_subdev			subdev;

	int					ident;
	u16					model;
	u8					revision;

	bool					afc_fw_ready;
	bool					af_result;
	bool					flag_vflip;
	bool					flag_hflip;
	int					brightness;
	int					contrast;
	int					effect;
	int					wh_bal;		//white balance
	int					ev;
	int					sat;		//saturation
	int					res;
	int					fps;

	int					f_cap;
	int					shutter;
	int					gain16;
	int					average;
};

static int preview_sysclk, preview_HTS;
static int AE_Target = 52;
static int AE_high, AE_low;

static int ov5640_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = (u8 *)&reg,
		},
		{
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 1,
			.buf    = val,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int ov5640_write(struct i2c_client *client, u16 reg, u8 val)
{
	struct i2c_msg msg;
	struct {
		u16 reg;
		u8 val;
	} __packed buf;
	int ret;

	reg = swab16(reg);

	buf.reg = reg;
	buf.val = val;

	msg.addr        = client->addr;
	msg.flags       = 0;
	msg.len         = 3;
	msg.buf         = (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x, val:0x%02x!\n",
				reg, val);
		return ret;
	}

	return 0;
}

static int ov5640_group_write(struct i2c_client *client, u16 reg, u8 * data, int len)
{
	struct i2c_msg msg;
	struct {
		u16 reg;
		u8 buf[0];
	} __packed * buf;
	int ret = 0;

	buf = kmalloc(sizeof(*buf) + len, GFP_KERNEL);
	if(!buf) {
		dev_err(&client->dev, "No memery\n");
		return -ENOMEM;
	}

	reg = swab16(reg);

	buf->reg = reg;
	memcpy(buf->buf, data, len);

	msg.addr        = client->addr;
	msg.flags       = 0;
	msg.len         = len + 2;
	msg.buf         = (u8 *)buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing group register 0x%04x!\n",
				reg);
		goto _exit;
	}

_exit:
	kfree(buf);
	return ret;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov5640_write_array(struct i2c_client *client, struct regval *regs)
{
	int ret;

	while (regs->reg != REG_END) {
		if( regs->reg == REG_MDELAY){
			udelay(regs->value * 1000);
		} else {
			ret = ov5640_write(client, regs->reg, regs->value);
			if (ret < 0)
				return ret;
		}
		regs++;
	}
	return 0;
}

/* Alter bus settings on camera side */
static int ov5640_set_bus_param(struct soc_camera_device *icd,
		unsigned long flags)
{
	return 0;
}

/* Request bus settings on camera side */
static unsigned long ov5640_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	unsigned long flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER |
		SOCAM_VSYNC_ACTIVE_LOW | SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8;

	return soc_camera_apply_sensor_flags(icl, flags);
}

static const struct v4l2_queryctrl ov5640_controls[] = {
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Vertically",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Horizontally",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_BRIGHTNESS,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Brightness",
		.minimum	= -4,
		.maximum	= 4,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_CONTRAST,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "CONTRAST",
		.minimum	= -3,
		.maximum	= 3,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_CAMERA_SPECIAL_EFFECT,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Special Effect",
		.minimum	= 0,
		.maximum	= 8,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_FOCUS_AUTO,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Auto focus",
		.minimum	= 0,
		.maximum	= 0x7fffffff,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_CAMERA_FACING,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Camera Facing",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_AEC_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "AEC gain",
		.minimum	= 0,
		.maximum	= 0x0ff,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_DO_WHITE_BALANCE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "White balance(light mode)",
		.minimum	= 0,
		.maximum	= 4,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "EV",
		.minimum	= -3,
		.maximum	= 3,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_SATURATION,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Saturation",
		.minimum	= -3,
		.maximum	= 3,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_PRE_CAP,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "pre-capture",
		.minimum	= 0,
		.maximum	= 0,
		.step		= 0,
		.default_value	= 0,
	},

};

static int ov5640_suspend(struct soc_camera_device *icd, pm_message_t state)
{
	struct v4l2_subdev *sd;
	struct ov5640_priv *priv;
	struct i2c_client *client;

	sd = soc_camera_to_subdev(icd);
	priv = to_ov5640(sd);
	client = (struct i2c_client *) v4l2_get_subdevdata(sd);
	ov5640_write(client, 0x3017, 0x00);
	ov5640_write(client, 0x3018, 0x00);

	return 0;
}

static int download_afc_firmware(struct i2c_client * client)
{
#define GROUP_BLOCK	0x100
	int ret;
	int i;
	char val;
	int len;

	ret = ov5640_write(client, 0x3000, 0x20);

	len = sizeof(afc_firmware);
	i = 0;
	while(1) {
		int l;

		l = len - i;
		l = l > GROUP_BLOCK ? GROUP_BLOCK : l;
		ret |= ov5640_group_write(client, 0x8000 + i, &afc_firmware[i], l);
		i += l;
		if (i >= len) {
			break;
		}
	}

	ret |= ov5640_write_array(client, post_afc_firmware);
	if (ret < 0) {
		dev_err(&client->dev, "%s() : download_afc_firmware failed!!\n",
				__func__);
		return ret;
	}
	mdelay(10);
#if 1
	for(i = 0; i < 5; i++) {
		ov5640_read(client, 0x3029, &val);
		if(val == 0x70){
			printk("firmware is running\n");
			break;
		} else {
			printk("val:%x\n",val);
			mdelay(10);
		}
	}
	if (i >= 5) {
		dev_err(&client->dev, "%s() : start afc_firmware failed!!\n",
				__func__);
		return -1;
	}
#endif
	return 0;
}

static int ov5640_resume(struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd;
	struct ov5640_priv *priv;
	struct i2c_client *client;

	sd = soc_camera_to_subdev(icd);
	priv = to_ov5640(sd);
	client = (struct i2c_client *) v4l2_get_subdevdata(sd);

	priv->fps = -1;
	priv->res = -1;
	priv->flag_vflip = 0;
	priv->flag_hflip = 0;
	priv->brightness = 0;
	priv->contrast = 0;
	priv->effect = 0;
	priv->wh_bal = 0;
	priv->ev = 0;
	priv->sat = 0;

	priv->afc_fw_ready = 0;
	priv->f_cap = 0;

	return 0;
}

static struct soc_camera_ops ov5640_ops = {
	.set_bus_param		= ov5640_set_bus_param,
	.query_bus_param	= ov5640_query_bus_param,
	.suspend		= ov5640_suspend,
	.resume			= ov5640_resume,
	.controls		= ov5640_controls,
	.num_controls		= ARRAY_SIZE(ov5640_controls),
};

static int ov5640_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov5640_priv *priv = to_ov5640(sd);
	struct i2c_client *client =
		(struct i2c_client *) v4l2_get_subdevdata(sd);
	int ret = 0;
	u8 val;

	switch (ctrl->id) {
		case V4L2_CID_VFLIP:
			ctrl->value = priv->flag_vflip;
			break;
		case V4L2_CID_HFLIP:
			ctrl->value = priv->flag_hflip;
			break;
		case V4L2_CID_BRIGHTNESS:
			ctrl->value = priv->brightness;
			break;
		case V4L2_CID_CONTRAST:
			ctrl->value = priv->contrast;
			break;
		case V4L2_CID_CAMERA_SPECIAL_EFFECT:
			ctrl->value = priv->effect;
			break;
		case V4L2_CID_FOCUS_AUTO:
			ret = ov5640_read(client, 0x3029, &val);
			if(ret)
				break;
			if(val != 0x10){
				ctrl->value = AFC_RES_RUNNING;
				break;
			}
			ret = ov5640_read(client, 0x3028, &val);
			if(ret)
				break;
			if(val) {
				ctrl->value = AFC_RES_SUCC;
			} else {
				ctrl->value = AFC_RES_FAIL;
			}
			break;
		case V4L2_CID_CAMERA_FACING:
			ctrl->value = CAMERA_FACING_BACK;
			break;
		case V4L2_CID_DO_WHITE_BALANCE:
			ctrl->value = priv->wh_bal;
			break;
		case V4L2_CID_EXPOSURE:
			ctrl->value = priv->ev;
			break;
		case V4L2_CID_SATURATION:
			ctrl->value = priv->sat;
			break;
		case V4L2_CID_AEC_GAIN:
			{
				u8 val;
				ret = ov5640_read(client, 0x350b, &val);
				if(!ret) {
					ctrl->value = val;
				}
				PDBG("AEC GAIN:%x\n", val);
			}
			break;
		default:
			return -EINVAL;
	}

	return ret;
}

static int set_flip(struct i2c_client *client, int is_on)
{
	u8 val;
	int ret = 0;

	ret = ov5640_read(client, 0x3820, &val);
	if (ret < 0) {
		return ret;
	}

	if (is_on) {
		val |= 0x06;
	} else {
		val &= 0xf9;
	}

	return ov5640_write(client, 0x3820, val);
}

static int set_mirror(struct i2c_client *client, int is_on)
{
	u8 val;
	int ret = 0;

	ret = ov5640_read(client, 0x3821, &val);
	if (ret < 0) {
		return ret;
	}

	if (!is_on) {
		val |= 0x06;
	} else {
		val &= 0xf9;
	}

	return ov5640_write(client, 0x3821, val);
}

static int set_brightness(struct i2c_client *client, int val)
{
	if(val < -4 || val > 4)
		return -EINVAL;

	val += 4;
	return ov5640_write_array(client, brightness_regs[val]);
}

static int set_contrast(struct i2c_client *client, int val)
{
	if(val < -3 || val > 3)
		return -EINVAL;

	val += 3;
	return ov5640_write_array(client, contrast_regs[val]);
}

static int set_effect(struct i2c_client *client, int val)
{
	if(val < 0 || val > 8)
		return -EINVAL;

	return ov5640_write_array(client, effect_regs[val]);
}

static int set_wh_bal(struct i2c_client *client, int val)
{
	if(val < 0 || val > 4)
		return -EINVAL;

	return ov5640_write_array(client, light_mode_regs[val]);
}

static int set_ev(struct i2c_client *client, int val)
{
	if(val < -3 || val > 3)
		return -EINVAL;

	val += 3;
	return ov5640_write_array(client, ev_regs[val]);
}

static int set_sat(struct i2c_client *client, int val)
{
	if(val < -3 || val > 3)
		return -EINVAL;

	val += 3;
	return ov5640_write_array(client, saturation_regs[val]);
}

static int auto_focus(struct i2c_client *client, u32 dat)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5640_priv *priv = to_ov5640(sd);
	int x = AFC_GET_X(dat);
	int y = AFC_GET_Y(dat);
	int cmd = AFC_GET_CMD(dat);
	int ret = 0;
	char val;

	PDBG("cmd:%d, x:%d, y:%d\n", cmd, x, y);

	if(x > 100) {
		x = 100;
	}
	x = x * 80 / 100;

	if(y > 100) {
		y = 100;
	}
	switch(priv->res) {
		case OV5640_720P:
			y = y * 45 / 100;
			break;
		default:
			y = y * 60 /100;
	}

	switch(cmd) {
		case AFC_CMD_RELA:
			ret = ov5640_write(client, 0x3023, 0x01);
			ret |= ov5640_write(client, 0x3022, 0x80);
			ret |= ov5640_read(client, 0x3022, &val);
			if(ret)
				break;
			if(val != 0)
				ret = -1;
			break;
		case AFC_CMD_SING:
			ret = ov5640_write(client, 0x3023, 0x01);
			ret |= ov5640_write(client, 0x3024, x);
			ret |= ov5640_write(client, 0x3025, y);
			ret |= ov5640_write(client, 0x3022, 0x81);
			ret |= ov5640_read(client, 0x3022, &val);
			if(ret)
				break;
			if(val != 0)
				ret = -1;
			mdelay(5);
			ret = ov5640_write(client, 0x3023, 0x01);
			ret |= ov5640_write(client, 0x3022, 0x03);
			ret |= ov5640_read(client, 0x3022, &val);
			if(ret)
				break;
			if(val != 0)
				ret = -1;
			break;
		case AFC_CMD_CONT:
			ret = ov5640_write(client, 0x3023, 0x01);
			ret |= ov5640_write(client, 0x3024, x);
			ret |= ov5640_write(client, 0x3025, y);
			ret |= ov5640_write(client, 0x3022, 0x81);
			ret |= ov5640_read(client, 0x3022, &val);
			if(ret)
				break;
			if(val != 0)
				ret = -1;
			mdelay(5);
			ret = ov5640_write(client, 0x3023, 0x01);
			ret |= ov5640_write(client, 0x3022, 0x04);
			ret |= ov5640_read(client, 0x3022, &val);
			if(ret)
				break;
			if(val != 0)
				ret = -1;
			break;
		case AFC_CMD_PAUS:
			ret = ov5640_write(client, 0x3023, 0x01);
			ret |= ov5640_write(client, 0x3022, 0x06);
			ret |= ov5640_read(client, 0x3022, &val);
			if(ret)
				break;
			if(val != 0)
				ret = -1;
			mdelay(5);
			break;
		case AFC_CMD_RELE:
			ret = ov5640_write(client, 0x3023, 0x01);
			ret |= ov5640_write(client, 0x3022, 0x08);
			ret |= ov5640_read(client, 0x3022, &val);
			if(ret)
				break;
			if(val != 0)
				ret = -1;
			break;
		default:
			return -EINVAL;
	}

	return ret;
}

static int get_sysclk(struct i2c_client * client)
{
	// calculate sysclk
	u8 temp1, temp2;
	int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, Bit_div2x, sclk_rdiv, sysclk;
	int sclk_rdiv_map[] = {1, 2, 4, 8};

	//temp1 = OV5640_read_i2c(0x3034);
	//temp2 = temp1 & 0x0f;
	ov5640_read(client, 0x3034, &temp2);
	temp2 &= 0x0f;
	if (temp2 == 8 || temp2 == 10) {
		Bit_div2x = temp2 / 2;
	} else {
		dev_err(&client->dev, "%s() : regs error!!\n", __func__);
	}

	//temp1 = OV5640_read_i2c(0x3035);
	ov5640_read(client, 0x3035, &temp1);
	SysDiv = temp1>>4;
	if(SysDiv == 0) {
		SysDiv = 16;
	}

	//temp1 = OV5640_read_i2c(0x3036);
	ov5640_read(client, 0x3036, &temp1);
	Multiplier = temp1;
	//temp1 = OV5640_read_i2c(0x3037);
	ov5640_read(client, 0x3037, &temp1);
	PreDiv = temp1 & 0x0f;
	Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;

	//temp1 = OV5640_read_i2c(0x3108);
	//temp2 = temp1 & 0x03;
	ov5640_read(client, 0x3108, &temp2);
	temp2 &= 0x03;
	sclk_rdiv = sclk_rdiv_map[temp2];
	VCO = XVCLK * Multiplier / PreDiv;
	sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

	return sysclk;
}

static int get_HTS(struct i2c_client * client)
{
	// read HTS from register settings
	int HTS;
	u8 val;

	//HTS = OV5640_read_i2c(0x380c);
	//HTS = (HTS<<8) + OV5640_read_i2c(0x380d);
	ov5640_read(client, 0x380c, &val);
	HTS = val;
	ov5640_read(client, 0x380d, &val);
	HTS = (HTS << 8) + val;

	return HTS;
}

static int get_VTS(struct i2c_client * client)
{
	// read VTS from register settings
	int VTS;
	u8 val;

	//VTS = OV5640_read_i2c(0x380e);
	//VTS = (VTS<<8) + OV5640_read_i2c(0x380f);
	ov5640_read(client, 0x380e, &val);
	VTS = val;
	ov5640_read(client, 0x380f, &val);
	VTS = (VTS << 8) + val;

	return VTS;
}

static int set_VTS(struct i2c_client * client, int VTS)
{
	// write VTS to registers
	u8 temp;

	temp = VTS & 0xff;
	//OV5640_write_i2c(0x380f, temp);
	ov5640_write(client, 0x380f, temp);

	temp = VTS >> 8;
	//OV5640_write_i2c(0x380e, temp);
	ov5640_write(client, 0x380e, temp);

	return 0;
}

static int get_shutter(struct i2c_client * client)
{
	// read shutter, in number of line period
	int shutter;
	u8 val;

	//shutter = (OV5640_read_i2c(0x03500) & 0x0f);
	ov5640_read(client, 0x3500, &val);
	shutter = val & 0x0f;
	//shutter = (shutter<<8) + OV5640_read_i2c(0x3501);
	ov5640_read(client, 0x3501, &val);
	shutter = (shutter << 8) + val;
	//shutter = (shutter<<4) + (OV5640_read_i2c(0x3502)>>4);
	ov5640_read(client, 0x3502, &val);
	shutter = (shutter << 4) + (val >> 4);

	return shutter;
}

static int set_shutter(struct i2c_client * client, int shutter)
{
	// write shutter, in number of line period
	u8 temp;

	shutter = shutter & 0xffff;
	temp = shutter & 0x0f;
	temp = temp<<4;
	//OV5640_write_i2c(0x3502, temp);
	ov5640_write(client, 0x3502, temp);
	temp = (shutter & 0xfff) >> 4;
	//OV5640_write_i2c(0x3501, temp);
	ov5640_write(client, 0x3501, temp);
	temp = shutter>>12;
	//OV5640_write_i2c(0x3500, temp);
	ov5640_write(client, 0x3500, temp);

	return 0;
}

static int get_gain16(struct i2c_client * client)
{
	// read gain, 16 = 1x
	int gain16;
	u8 val;

	//gain16 = OV5640_read_i2c(0x350a) & 0x03;
	ov5640_read(client, 0x350a, &val);
	gain16 = val & 0x03;
	//gain16 = (gain16<<8) + OV5640_read_i2c(0x350b);
	ov5640_read(client, 0x350b, &val);
	gain16 = (gain16 << 8) + val;

	return gain16;
}

static int set_gain16(struct i2c_client * client, int gain16)
{
	// write gain, 16 = 1x
	u8 temp;

	gain16 = gain16 & 0x3ff;
	temp = gain16 & 0xff;
	//OV5640_write_i2c(0x350b, temp);
	ov5640_write(client, 0x350b, temp);
	temp = gain16>>8;
	//OV5640_write_i2c(0x350a, temp);
	ov5640_write(client, 0x350a, temp);

	return 0;
}

static int get_light_frequency(struct i2c_client * client)
{
	// get banding filter value
	u8 val;

	//temp = OV5640_read_i2c(0x3c01);
	ov5640_read(client, 0x3c01, &val);
	if (val & 0x80) {
		// manual
		//temp1 = OV5640_read_i2c(0x3c00);
		ov5640_read(client, 0x3c00, &val);
		if (val & 0x04) {
			// 50Hz
			return 50;
		} else {
			// 60Hz
			return 60;
		}
	} else {
		// auto
		//temp1 = OV5640_read_i2c(0x3c0c);
		ov5640_read(client, 0x3c0c, &val);
		if (val & 0x01) {
			// 50Hz
			return 50;
		} else {
			// 60Hz
			return 60;
		}
	}
}

static void set_bandingfilter(struct i2c_client * client)
{
	int preview_VTS;
	int band_step60, max_band60, band_step50, max_band50;

	// read preview PCLK
	preview_sysclk = get_sysclk(client);
	// read preview HTS
	preview_HTS = get_HTS(client);
	// read preview VTS
	preview_VTS = get_VTS(client);
	// calculate banding filter
	// 60Hz
	band_step60 = preview_sysclk * 100/preview_HTS * 100/120;
	//OV5640_write_i2c(0x3a0a, (band_step60 >> 8));
	//OV5640_write_i2c(0x3a0b, (band_step60 & 0xff));
	ov5640_write(client, 0x3a0a, (band_step60 >> 8));
	ov5640_write(client, 0x3a0b, (band_step60 & 0xff));
	max_band60 = (preview_VTS-4)/band_step60;
	//OV5640_write_i2c(0x3a0d, max_band60);
	ov5640_write(client, 0x3a0d, max_band60);
	// 50Hz
	band_step50 = preview_sysclk * 100/preview_HTS;
	//OV5640_write_i2c(0x3a08, (band_step50 >> 8));
	//OV5640_write_i2c(0x3a09, (band_step50 & 0xff));
	ov5640_write(client, 0x3a08, (band_step50 >> 8));
	ov5640_write(client, 0x3a09, (band_step50 & 0xff));
	max_band50 = (preview_VTS-4)/band_step50;
	//OV5640_write_i2c(0x3a0e, max_band50);
	ov5640_write(client, 0x3a0e, max_band50);
}

static int set_AE_target(struct i2c_client * client, int target)
{
	// stable in high
	int fast_high, fast_low;

	AE_low = target * 23 / 25; // 0.92
	AE_high = target * 27 / 25; // 1.08
	fast_high = AE_high << 1;
	if(fast_high>255)
		fast_high = 255;
	fast_low = AE_low >> 1;

	//OV5640_write_i2c(0x3a0f, AE_high);
	ov5640_write(client, 0x3a0f, AE_high);
	//OV5640_write_i2c(0x3a10, AE_low);
	ov5640_write(client, 0x3a10, AE_low);
	//OV5640_write_i2c(0x3a1b, AE_high);
	ov5640_write(client, 0x3a1b, AE_high);
	//OV5640_write_i2c(0x3a1e, AE_low);
	ov5640_write(client, 0x3a1e, AE_low);
	//OV5640_write_i2c(0x3a11, fast_high);
	ov5640_write(client, 0x3a11, fast_high);
	//OV5640_write_i2c(0x3a1f, fast_low);
	ov5640_write(client, 0x3a1f, fast_low);

	return 0;
}

static void pre_capture(struct i2c_client *client, struct ov5640_priv *priv)
{
	u8 val;

	priv->shutter = get_shutter(client);
	priv->gain16 = get_gain16(client);
	ov5640_read(client, 0x56a1, &val);
	priv->average = val;
	priv->f_cap = 1;
}

static void setup_capture(struct i2c_client *client, struct ov5640_priv *priv)
{
	int capture_shutter, capture_gain16;
	int capture_sysclk, capture_HTS, capture_VTS;
	int light_frequency, capture_bandingfilter, capture_max_band;
	long capture_gain16_shutter;

	// read capture VTS
	capture_VTS = get_VTS(client);
	capture_HTS = get_HTS(client);
	capture_sysclk = get_sysclk(client);

	// calculate capture banding filter
	light_frequency = get_light_frequency(client);
	if (light_frequency == 60) {
		// 60Hz
		capture_bandingfilter = capture_sysclk * 100 / capture_HTS * 100 / 120;
	} else {
		// 50Hz
		capture_bandingfilter = capture_sysclk * 100 / capture_HTS;
	}
	capture_max_band = ((capture_VTS - 4)/capture_bandingfilter);

	// calculate capture shutter/gain16
	if (priv->average > AE_low && priv->average < AE_high) {
		// in stable range
		capture_gain16_shutter = priv->gain16 * priv->shutter
			* capture_sysclk/preview_sysclk
			* preview_HTS/capture_HTS
			* AE_Target / priv->average;
	} else {
		capture_gain16_shutter = priv->gain16 * priv->shutter
			* capture_sysclk/preview_sysclk
			* preview_HTS/capture_HTS;
	}

	capture_gain16_shutter = capture_gain16_shutter * 3 / 4;

	// gain to shutter
	if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
		// shutter < 1/100
		capture_shutter = capture_gain16_shutter/16;
		if(capture_shutter <1)
			capture_shutter = 1;
		capture_gain16 = capture_gain16_shutter/capture_shutter;
		if(capture_gain16 < 16)
			capture_gain16 = 16;
	} else {
		if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
			// exposure reach max
			capture_shutter = capture_bandingfilter*capture_max_band;
			capture_gain16 = capture_gain16_shutter / capture_shutter;
		} else {
			// 1/100 < capture_shutter =< max, capture_shutter = n/100
			capture_shutter = (capture_gain16_shutter/16/capture_bandingfilter) * capture_bandingfilter;
			capture_gain16 = capture_gain16_shutter / capture_shutter;
		}
	}
	// write capture gain
	set_gain16(client, capture_gain16);
	// write capture shutter
	if (capture_shutter > (capture_VTS - 4)) {
		capture_VTS = capture_shutter + 4;
		set_VTS(client, capture_VTS);
	}
	set_shutter(client, capture_shutter);

	PDBG("capture_shutter:%d, capture_gain16:%d, preview_shutter:%d, preview_gain16:%d, preview_average:%d\n",
			capture_shutter, capture_gain16, priv->shutter, priv->gain16, priv->average);
}

/* Set status of additional camera capabilities */
static int ov5640_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov5640_priv *priv = to_ov5640(sd);
	struct i2c_client *client =
		(struct i2c_client *) v4l2_get_subdevdata(sd);
	int ret = 0;

	switch (ctrl->id) {
		case V4L2_CID_VFLIP:
			priv->flag_vflip = ctrl->value;
			return set_flip(client, priv->flag_vflip);
			break;
		case V4L2_CID_HFLIP:
			priv->flag_hflip = ctrl->value;
			return set_mirror(client, priv->flag_hflip);
			break;
		case V4L2_CID_BRIGHTNESS:
			if (ctrl->value < -4 || ctrl->value > 4) {
				return -EINVAL;
			}
			priv->brightness = ctrl->value;
			return set_brightness(client, priv->brightness);
			break;
		case V4L2_CID_CONTRAST:
			if (ctrl->value < -3 || ctrl->value > 3) {
				return -EINVAL;
			}
			priv->contrast = ctrl->value;
			return set_contrast(client, priv->contrast);
			break;
		case V4L2_CID_CAMERA_SPECIAL_EFFECT:
			if (ctrl->value < 0 || ctrl->value > 8) {
				return -EINVAL;
			}
			priv->effect = ctrl->value;
			return set_effect(client, priv->effect);
			break;
		case V4L2_CID_FOCUS_AUTO:
			return auto_focus(client, ctrl->value);
		case V4L2_CID_DO_WHITE_BALANCE:
			if (ctrl->value < 0 || ctrl->value > 4) {
				return -EINVAL;
			}
			priv->wh_bal = ctrl->value;
			return set_wh_bal(client, priv->wh_bal);
			break;
		case V4L2_CID_EXPOSURE:
			if (ctrl->value < -3 || ctrl->value > 3) {
				return -EINVAL;
			}
			priv->ev = ctrl->value;
			return set_ev(client, priv->ev);
			break;
		case V4L2_CID_SATURATION:
			if (ctrl->value < -3 || ctrl->value > 3) {
				return -EINVAL;
			}
			priv->sat = ctrl->value;
			return set_sat(client, priv->sat);
			break;
		case V4L2_CID_PRE_CAP:
			PDBG("V4L2_CID_PRE_CAP\n");
			pre_capture(client, priv);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

/* Get chip identification */
static int ov5640_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *id)
{
	struct ov5640_priv *priv = to_ov5640(sd);

	id->ident = priv->ident;
	id->revision = priv->revision;

	return 0;
}

static struct v4l2_subdev_core_ops ov5640_core_ops = {
	.g_ctrl			= ov5640_g_ctrl,
	.s_ctrl			= ov5640_s_ctrl,
	.g_chip_ident		= ov5640_g_chip_ident,
};

/* Start/Stop streaming from the device */
static int ov5640_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640_priv *priv = to_ov5640(sd);
	int ret = 0;

	/* Program orientation register. */
	ret = set_flip(client, priv->flag_vflip);
	if (ret < 0)
		return ret;

	ret = set_mirror(client, priv->flag_hflip);
	if (ret < 0)
		return ret;

	ret = set_wh_bal(client, priv->wh_bal);
	if (ret < 0)
		return ret;

	ret = set_brightness(client, priv->brightness);
	if (ret < 0)
		return ret;

	ret = set_contrast(client, priv->contrast);
	if (ret < 0)
		return ret;

	ret = set_ev(client, priv->ev);
	if (ret < 0)
		return ret;

	ret = set_sat(client, priv->sat);
	if (ret < 0)
		return ret;

	ret = set_effect(client, priv->effect);
	if (ret < 0)
		return ret;

	if (enable) {
		dev_dbg(&client->dev, "Enabling Streaming\n");
		/* Start Streaming */
		ret = ov5640_write(client, 0x4202, 0x00);
		if (ret)
			goto out;
	} else {
		dev_dbg(&client->dev, "Disabling Streaming\n");
		/* enter standby mode */
		ret = ov5640_write(client, 0x4202, 0x0f);
		if (ret)
			goto out;
	}

out:
	return ret;
}

/* select nearest higher resolution for capture */
static void ov5640_res_roundup(u32 *width, u32 *height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5640_resolutions); i++) {
		if ((ov5640_resolutions[i].width >= *width) &&
				(ov5640_resolutions[i].height >= *height)) {
			*width = ov5640_resolutions[i].width;
			*height = ov5640_resolutions[i].height;
			return;
		}
	}

	*width = ov5640_resolutions[OV5640_5M].width;
	*height = ov5640_resolutions[OV5640_5M].height;
}

/* Setup registers according to resolution and color encoding */
static int ov5640_set_res(struct i2c_client *client, u32 width)
{
	int ret;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5640_priv *priv = to_ov5640(sd);

	/* select register configuration for given resolution */
	if (width == ov5640_resolutions[OV5640_VGA].width) {
		dev_dbg(&client->dev, "Setting image size to 640x480\n");
		priv->res = OV5640_VGA;
		ret = ov5640_write_array(client, fmt_yuv422_vga);
	} else if (width == ov5640_resolutions[OV5640_176x144].width) {
		dev_dbg(&client->dev, "Setting image size to 176x144\n");
		priv->res = OV5640_176x144;
		ret = ov5640_write_array(client, fmt_yuv422_176x144);
	} else if (width == ov5640_resolutions[OV5640_QVGA].width) {
		dev_dbg(&client->dev, "Setting image size to 320x240\n");
		priv->res = OV5640_QVGA;
		ret = ov5640_write_array(client, fmt_yuv422_320x240);
	} else if (width == ov5640_resolutions[OV5640_352x288].width) {
		dev_dbg(&client->dev, "Setting image size to 352x288\n");
		priv->res = OV5640_352x288;
		ret = ov5640_write_array(client, fmt_yuv422_352x288);
		//} else if (width == ov5640_resolutions[OV5640_720x480].width) {
		//	dev_dbg(&client->dev, "Setting image size to 720x480\n");
		//	priv->res = OV5640_720x480;
		//	ret = ov5640_write_array(client, fmt_yuv422_720x480);
		//} else if (width == ov5640_resolutions[OV5640_1024x768].width) {
		//	dev_dbg(&client->dev, "Setting image size to 1024x768\n");
		//	priv->res = OV5640_1024x768;
		//	ret = ov5640_write_array(client, fmt_yuv422_1024x768);
	} else if (width == ov5640_resolutions[OV5640_720P].width) {
		dev_dbg(&client->dev, "Setting image size to 1280x720\n");
		priv->res = OV5640_720P;
		ret = ov5640_write_array(client, fmt_yuv422_720p);
	} else if (width == ov5640_resolutions[OV5640_2M].width) {
		dev_dbg(&client->dev, "Setting image size to 1600x1200\n");
		priv->res = OV5640_2M;
		ret = ov5640_write_array(client, fmt_yuv422_1600x1200);
	} else if (width == ov5640_resolutions[OV5640_3M].width) {
		dev_dbg(&client->dev, "Setting image size to 2048x1536\n");
		priv->res = OV5640_3M;
		ret = ov5640_write_array(client, fmt_yuv422_2048x1536);
	} else if (width == ov5640_resolutions[OV5640_5M].width) {
		dev_dbg(&client->dev, "Setting image size to 2592x1944\n");
		priv->res = OV5640_5M;
		ret = ov5640_write_array(client, fmt_yuv422_2592x1944);
	} else {
		dev_err(&client->dev, "Failed to select resolution!\n");
		return -EINVAL;
	}

	priv->res = -1;
	return ret;
}

/* set the format we will capture in */
static int ov5640_s_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640_priv *priv = to_ov5640(sd);
	enum v4l2_colorspace cspace;
	enum v4l2_mbus_pixelcode code = mf->code;
	int ret;

	ov5640_res_roundup(&mf->width, &mf->height);

	switch (code) {
		case V4L2_MBUS_FMT_YUYV8_2X8:
			cspace = V4L2_COLORSPACE_SRGB;
			break;
		default:
			return -EINVAL;
	}

	if (!priv->afc_fw_ready) {
		ret = ov5640_write_array(client, init_regs);
		if (ret < 0) {
			dev_err(&client->dev, "%s() : write ov5640_init_regs failed!!",
					__func__);
			return ret;
		}
	}

	ret = ov5640_set_res(client, mf->width);
	if (ret < 0)
		return ret;

	if (!priv->afc_fw_ready) {
		ret = download_afc_firmware(client);
		if(ret < 0) {
			return ret;
		}
		priv->afc_fw_ready = 1;
	}

	if (!priv->f_cap) {	// preview
		set_bandingfilter(client);
		set_AE_target(client, AE_Target);
	} else { 	// capture
		PDBG("Capture.\n");
		ov5640_write(client, 0x3503, 0x03);  // AEC/AGC off
		setup_capture(client, priv);
		priv->f_cap = 0;
	}

	mf->code	= code;
	mf->colorspace	= cspace;

	return ret;
}

static int ov5640_try_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	ov5640_res_roundup(&mf->width, &mf->height);

	mf->field = V4L2_FIELD_NONE;
	mf->code = V4L2_MBUS_FMT_YUYV8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static enum v4l2_mbus_pixelcode ov5640_codes[] = {
	V4L2_MBUS_FMT_YUYV8_2X8,
};

static int ov5640_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov5640_codes))
		return -EINVAL;

	*code = ov5640_codes[index];

	return 0;
}

static int ov5640_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left		= 0;
	a->bounds.top		= 0;
	a->bounds.width		= ov5640_resolutions[OV5640_720P].width;
	a->bounds.height	= ov5640_resolutions[OV5640_720P].width;
	a->defrect		= a->bounds;
	a->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int ov5640_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	a->c.left		= 0;
	a->c.top		= 0;
	a->c.width		= ov5640_resolutions[OV5640_720P].width;
	a->c.height		= ov5640_resolutions[OV5640_720P].height;
	a->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static struct v4l2_subdev_video_ops ov5640_video_ops = {
	.s_stream		= ov5640_s_stream,
	.s_mbus_fmt		= ov5640_s_fmt,
	.try_mbus_fmt		= ov5640_try_fmt,
	.enum_mbus_fmt		= ov5640_enum_fmt,
	.cropcap		= ov5640_cropcap,
	.g_crop			= ov5640_g_crop,
};

static struct v4l2_subdev_ops ov5640_subdev_ops = {
	.core			= &ov5640_core_ops,
	.video			= &ov5640_video_ops,
};

#ifdef DEBUG_EN
static u16 dbg_addr = 0;

static ssize_t show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%04X\n", dbg_addr);
}

static ssize_t store_addr(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	dbg_addr = simple_strtol(buf, NULL, 16);
	return n;
}



static ssize_t show_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 val;
	int ret = 0;

	ret = ov5640_read(client, dbg_addr, &val);

	if(ret < 0){
		dev_err(&client->dev, "OV5640: show_val: reg_read failed\n");
		return ret;
	}

	return sprintf(buf, "0x%02X\n", val);
}

static ssize_t store_val(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 val;
	int ret = 0;

	val = simple_strtol(buf, NULL, 16);

	ret = ov5640_write(client, dbg_addr, val);

	if(ret < 0){
		dev_err(&client->dev, "OV5640: store_val: reg_write failed\n");
		return ret;
	}

	return n;
}

static ssize_t store_test(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5640_priv *priv = to_ov5640(sd);
	int ret ;
	u32 cmd;

	//download_afc_firmware(client);

	cmd = 0;
	AFC_SET_CMD(cmd,AFC_CMD_RELA);
	ret = auto_focus(client, cmd);
	if(ret < 0) {
		dev_err(&client->dev, "Re-launch AF zone failed!\n");
		return ret;
	}

	return n;

	cmd = 0;
	AFC_SET_CMD(cmd,AFC_CMD_SING);
	AFC_SET_X(cmd,50);
	AFC_SET_Y(cmd,50);
	ret = auto_focus(client, cmd);
	if(ret) {
		printk("single focus failed\n");
	} else {
		printk("auto focus result:%d\n", priv->af_result);
	}

	return n;
}

static DEVICE_ATTR(addr, S_IRUGO | S_IWUGO, show_addr, store_addr);
static DEVICE_ATTR(val, S_IRUGO | S_IWUGO, show_val, store_val);
static DEVICE_ATTR(test, S_IWUGO, NULL, store_test);

static struct attribute *dev_attributes[] = {
	&dev_attr_addr.attr,
	&dev_attr_val.attr,
	&dev_attr_test.attr,
	NULL
};

static struct attribute_group dev_attr_group = {
	.attrs = dev_attributes,
};
#endif

static int ov5640_video_probe(struct soc_camera_device *icd,
		struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5640_priv *priv = to_ov5640(sd);
	char modelhi, modello;
	int ret;

	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */
	if (!icd->dev.parent ||
			to_soc_camera_host(icd->dev.parent)->nr != icd->iface) {
		dev_err(&client->dev, "Parent missing or invalid!\n");
		ret = -ENODEV;
		goto err;
	}

	/*
	 * check and show product ID and manufacturer ID
	 */
	ret = ov5640_read(client, MODEL_ID_HI, &modelhi);
	if (ret < 0)
		goto err;

	ret = ov5640_read(client, MODEL_ID_LO, &modello);
	if (ret < 0)
		goto err;

	priv->model = (modelhi << 8) | modello;

	ret = ov5640_read(client, REVISION_NUMBER, &priv->revision);
	if (ret < 0)
		goto err;

	if (priv->model != 0x5640) {
		ret = -ENODEV;
		goto err;
	}

	priv->ident = V4L2_IDENT_OV5640;

	dev_info(&client->dev, "ov5640 Model ID 0x%04x, Revision 0x%02x\n ",
			priv->model, priv->revision);
err:
	return ret;
}

/*
 * i2c_driver function
 */
static int ov5640_probe(struct i2c_client *client,
		const struct i2c_device_id *did)
{
	struct ov5640_priv *priv;
	struct soc_camera_device *icd	= client->dev.platform_data;
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "Missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(struct ov5640_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "Failed to allocate private data!\n");
		return -ENOMEM;
	}

	priv->res = -1;
	priv->fps = -1;

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov5640_subdev_ops);

	icd->ops = &ov5640_ops;

	ret = ov5640_video_probe(icd, client);
	if (ret < 0) {
		goto err_probe;
	}

#ifdef DEBUG_EN
	sysfs_create_group(&client->dev.kobj, &dev_attr_group);
#endif

	return 0;

err_probe:
	icd->ops = NULL;
	kfree(priv);

	return ret;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct ov5640_priv *priv = i2c_get_clientdata(client);

	if(!priv) {
		return 0;
	}

#ifdef DEBUG_EN
	sysfs_remove_group(&client->dev.kobj, &dev_attr_group);
#endif

	kfree(priv);

	return 0;
}

static const struct i2c_device_id ov5640_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5640_id);

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe    = ov5640_probe,
	.remove   = ov5640_remove,
	.id_table = ov5640_id,
};

static int __init ov5640_module_init(void)
{
	return i2c_add_driver(&ov5640_i2c_driver);
}

static void __exit ov5640_module_exit(void)
{
	i2c_del_driver(&ov5640_i2c_driver);
}

module_init(ov5640_module_init);
module_exit(ov5640_module_exit);

MODULE_DESCRIPTION("SoC Camera driver for OV5640");
MODULE_AUTHOR("guojianxin@nusmart.com");
MODULE_LICENSE("GPL v2");

