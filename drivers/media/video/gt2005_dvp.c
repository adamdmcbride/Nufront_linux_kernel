/*
 * A V4L2 driver for Micron GT2005 cameras.
 *
 * Based on GT2005 camera driver.
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

#include "gt2005_dvp.h"

#define DRIVER_NAME	"gt2005"

//#define DEBUG_EN
#ifdef DEBUG_EN
#define  PDBG(format,...)               printk("[%d:%s]:"format, \
		__LINE__,__func__ , ##__VA_ARGS__ )
#else
#define  PDBG(format,...)               do{}while(0)
#endif

#define V4L2_CID_CAMERA_SPECIAL_EFFECT  (V4L2_CID_PRIVATE_BASE + 124)
#define V4L2_CID_CAMERA_FACING		(V4L2_CID_PRIVATE_BASE + 125)

#define to_gt2005(sd)		container_of(sd, struct gt2005_priv, subdev)

u16 DGain_shutter, AGain_shutter, shutter;
u8 DGain_shutterH, DGain_shutterL, AGain_shutterH, AGain_shutterL, shutterH,
    shutterL;
int UXGA_Cap = 0;

enum v4l2_camera_facing {
	CAMERA_FACING_BACK = 0,
	CAMERA_FACING_FRONT = 1,
};

/*enum {
	SPEC_EFF_NORMAL = 0, 	// "Normal",
    SPEC_EFF_BLUE,
	SPEC_EFF_BLWH,		//"Black and white",
    SPEC_EFF_NEG,		//"Negative",
	SPEC_EFF_SEPIA,		//"Sepia",
    SPEC_EFF_BLUE,		//"Blueish",
	SPEC_EFF_GREEN,		//"Greenish",
};
*/
enum {
	SPEC_EFF_NORMAL = 0,	// "Normal",
	SPEC_EFF_BLUE,		//"Blueish",
	SPEC_EFF_RED,		//"Redish",
	SPEC_EFF_GREEN,		//"Greenish",
	SPEC_EFF_BLWH,		//"Black and white",
	SPEC_EFF_SEPIA,		//"Sepia",
	SPEC_EFF_NEG,		//"Negative",
	SPEC_EFF_OVEX,		//"Over exposure",
	SPEC_EFF_SOL,		//"Solarize",
};

enum {
	WB_AUTO = 0,
	WB_TUNGSTEN_LAMP1,
	WB_TUNGSTEN_LAMP2,
	WB_CLEAR_DAY,
	WB_CLOUDY,
};

enum {
	SCENE_AUTO,
	SCENE_NIGHT,
};

/* supported resolutions */
enum {
	GT2005_QVGA,
	GT2005_VGA,
	GT2005_480P,
	GT2005_SVGA,
	GT2005_XGA,
	GT2005_720P,
	GT2005_SXGA,
	GT2005_UXGA,
};

struct gt2005_resolution {
	unsigned int width;
	unsigned int height;
};

static struct gt2005_resolution gt2005_resolutions[] = {
	[GT2005_QVGA] = {
			 .width = 320,
			 .height = 240,
			 },
	[GT2005_VGA] = {
			.width = 640,
			.height = 480,
			},
	[GT2005_480P] = {
			 .width = 720,
			 .height = 480,
			 },
	[GT2005_SVGA] = {
			 .width = 800,
			 .height = 600,
			 },
	[GT2005_XGA] = {
			.width = 1024,
			.height = 768,
			},
	[GT2005_720P] = {
			 .width = 1280,
			 .height = 720,
			 },
	[GT2005_SXGA] = {
			 .width = 1280,
			 .height = 960,
			 },
	[GT2005_UXGA] = {
			 .width = 1600,
			 .height = 1200,
			 },
};

struct gt2005_priv {
	struct v4l2_subdev subdev;

	int ident;

	bool flag_vflip;
	bool flag_hflip;
	int brightness;
	int contrast;
	int sat;
	int effect;
	int wh_bal;		//white balance
	int scene;
	int exposure;

	int inited;
};

static int gt2005_read(struct i2c_client *client, u16 reg, u8 * val)
{
	int err, cnt;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = client->flags,
		 .buf = (u8 *) & reg,
		 .len = 2,
		 },
		{
		 .addr = client->addr,
		 .flags = client->flags | I2C_M_RD,
		 .buf = val,
		 .len = 1,
		 },
	};

	reg = swab16(reg);

	cnt = 3;
	err = -EAGAIN;
	while ((cnt-- > 0) && (err < 0)) {	/* ddl@rock-chips.com :  Transfer again if transent is failed   */
		err = i2c_transfer(client->adapter, msg, 2);

		if (err >= 0) {
			return 0;
		} else {
			PDBG("\n %s read reg(0x%x val:0x%x) failed, try to read again! \n", SENSOR_NAME_STRING(), reg, *val);
			udelay(10);
		}
	}

	return err;
}

static int gt2005_write(struct i2c_client *client, u16 reg, u8 val)
{
	int err, cnt;
	u8 buf[3];
	struct i2c_msg msg[1];

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	msg->addr = client->addr;
	msg->flags = client->flags;
	msg->buf = buf;
	msg->len = sizeof(buf);

	cnt = 3;
	err = -EAGAIN;
	while ((cnt-- > 0) && (err < 0)) {	/* ddl@rock-chips.com :  Transfer again if transent is failed   */
		err = i2c_transfer(client->adapter, msg, 1);

		if (err >= 0) {
			return 0;
		} else {
			PDBG("\n %s write reg(0x%x, val:0x%x) failed, try to write again!\n", SENSOR_NAME_STRING(), reg, val);
			udelay(10);
		}
	}

	return err;
}

/*
 * Write a list of register settings; 0/0 stops the process.
 */
static int gt2005_write_array(struct i2c_client *sd, struct regval_list *vals)
{
	while (vals->reg_num != 0x0 || vals->value != 0x0) {
		int ret = gt2005_write(sd, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}

/* Alter bus settings on camera side */
static int gt2005_set_bus_param(struct soc_camera_device *icd,
				unsigned long flags)
{
	return 0;
}

/* Request bus settings on camera side */
static unsigned long gt2005_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	unsigned long flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER |
	    SOCAM_VSYNC_ACTIVE_LOW | SOCAM_HSYNC_ACTIVE_HIGH |
	    SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8;

	return soc_camera_apply_sensor_flags(icl, flags);
}

static const struct v4l2_queryctrl gt2005_controls[] = {
	{
	 .id = V4L2_CID_VFLIP,
	 .type = V4L2_CTRL_TYPE_BOOLEAN,
	 .name = "Flip Vertically",
	 .minimum = 0,
	 .maximum = 1,
	 .step = 1,
	 .default_value = 0,
	 },
	{
	 .id = V4L2_CID_HFLIP,
	 .type = V4L2_CTRL_TYPE_BOOLEAN,
	 .name = "Flip Horizontally",
	 .minimum = 0,
	 .maximum = 1,
	 .step = 1,
	 .default_value = 0,
	 },
	{
	 .id = V4L2_CID_CAMERA_FACING,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Camera Facing",
	 .minimum = 0,
	 .maximum = 1,
	 .step = 1,
	 .default_value = 0,
	 },
	{
	 .id = V4L2_CID_BRIGHTNESS,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Brightness",
	 .minimum = -2,
	 .maximum = 3,
	 .step = 1,
	 .default_value = 0,
	 },
	{
	 .id = V4L2_CID_CONTRAST,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Contrast",
	 .minimum = -3,
	 .maximum = 3,
	 .step = 1,
	 .default_value = 0,
	 },
	{
	 .id = V4L2_CID_SATURATION,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Saturation",
	 .minimum = 0,
	 .maximum = 2,
	 .step = 1,
	 .default_value = 0,
	 },
	{
	 .id = V4L2_CID_CAMERA_SPECIAL_EFFECT,
	 .type = V4L2_CTRL_TYPE_MENU,
	 .name = "Special Effect",
	 .minimum = 0,
	 .maximum = 6,
	 .step = 1,
	 .default_value = 0,
	 },
	{
	 .id = V4L2_CID_DO_WHITE_BALANCE,
	 .type = V4L2_CTRL_TYPE_MENU,
	 .name = "White balance(light mode)",
	 .minimum = 0,
	 .maximum = 4,
	 .step = 1,
	 .default_value = 0,
	 },
	{
	 .id = V4L2_CID_EXPOSURE,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Exposure Control",
	 .minimum = -3,
	 .maximum = 3,
	 .step = 1,
	 .default_value = 0,
	 },
};

static int gt2005_suspend(struct soc_camera_device *icd, pm_message_t state)
{
	struct v4l2_subdev *sd;
	struct gt2005_priv *priv;

	sd = soc_camera_to_subdev(icd);
	priv = container_of(sd, struct gt2005_priv, subdev);

	return 0;
}

static int gt2005_resume(struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd;
	struct gt2005_priv *priv;

	PDBG("++\n");
	sd = soc_camera_to_subdev(icd);
	priv = container_of(sd, struct gt2005_priv, subdev);

	priv->flag_vflip = 0;
	priv->flag_hflip = 0;
	priv->brightness = 0;
	priv->contrast = 0;
	priv->sat = 0;
	priv->effect = 0;
	priv->wh_bal = 0;
	priv->scene = 0;
	priv->exposure = 0;

	priv->inited = 0;

	return 0;
}

static struct soc_camera_ops gt2005_ops = {
	.set_bus_param = gt2005_set_bus_param,
	.query_bus_param = gt2005_query_bus_param,
	.suspend = gt2005_suspend,
	.resume = gt2005_resume,
	.controls = gt2005_controls,
	.num_controls = ARRAY_SIZE(gt2005_controls),
};

static int gt2005_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct gt2005_priv *priv = to_gt2005(sd);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		ctrl->value = priv->flag_vflip;
		break;
	case V4L2_CID_HFLIP:
		ctrl->value = priv->flag_hflip;
		break;
	case V4L2_CID_CAMERA_FACING:
		ctrl->value = CAMERA_FACING_BACK;
		break;
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = priv->brightness;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = priv->contrast;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = priv->sat;
		break;
	case V4L2_CID_CAMERA_SPECIAL_EFFECT:
		ctrl->value = priv->effect;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		ctrl->value = priv->wh_bal;
		break;
	case V4L2_CID_EXPOSURE:
		ctrl->value = priv->exposure;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int set_flip(struct i2c_client *client, int is_on)
{
	int ret = 0;

	ret = gt2005_write_array(client, sensor_FlipSeqe[is_on]);

	return ret;
}

static int set_mirror(struct i2c_client *client, int is_on)
{
	int ret = 0;

	ret = gt2005_write_array(client, sensor_MirrorSeqe[is_on]);

	return ret;
}

static int set_brightness(struct i2c_client *client, int val)
{
	int ret = 0;

	if (val < -2 || val > 3)
		return -EINVAL;

	ret = gt2005_write_array(client, sensor_BrightnessSeqe[val + 2]);

	return ret;
}

static int set_contrast(struct i2c_client *client, int val)
{
	int ret = 0;

	if (val < -3 || val > 3)
		return -EINVAL;

	ret = gt2005_write_array(client, sensor_ContrastSeqe[val + 3]);

	return ret;
}

static int set_sat(struct i2c_client *client, int val)
{
	int ret = 0;

	if (val < 0 || val > 3)
		return -EINVAL;

	ret = gt2005_write_array(client, sensor_SaturationSeqe[val]);

	return ret;
}

static int set_effect(struct i2c_client *client, int val)
{
	int ret = 0;

	if (val < 0 || val > 6)
		return 0;

	ret = gt2005_write_array(client, sensor_EffectSeqe[val]);

	return ret;
}

static int set_wh_bal(struct i2c_client *client, int val)
{
	int ret = 0;

	if (val < 0 || val > 4)
		return -EINVAL;

	ret = gt2005_write_array(client, sensor_WhiteBalanceSeqe[val]);

	return ret;
}

static int set_exposure(struct i2c_client *client, int val)
{
	int ret = 0;

	if (val < -3 || val > 3)
		return -EINVAL;

	ret = gt2005_write_array(client, sensor_ExposureSeqe[val + 3]);

	return ret;
}

static int set_scene(struct i2c_client *client, int val)
{
	int ret = 0;

	if (val < 0 || val > 1)
		return -EINVAL;

	ret = gt2005_write_array(client, sensor_SceneSeqe[val]);

	return ret;
}

/* Set status of additional camera capabilities */
static int gt2005_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct gt2005_priv *priv = to_gt2005(sd);
	struct i2c_client *client =
	    (struct i2c_client *)v4l2_get_subdevdata(sd);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		priv->flag_vflip = ctrl->value;
		return set_flip(client, priv->flag_vflip);
	case V4L2_CID_HFLIP:
		priv->flag_hflip = ctrl->value;
		PDBG("set H-FLIP:%d\n", priv->flag_hflip);
		return set_mirror(client, priv->flag_hflip);
	case V4L2_CID_BRIGHTNESS:
		if (ctrl->value < -2 || ctrl->value > 3) {
			return -EINVAL;
		}
		priv->brightness = ctrl->value;
		return set_brightness(client, priv->brightness);
	case V4L2_CID_CONTRAST:
		if (ctrl->value < -3 || ctrl->value > 3) {
			return -EINVAL;
		}
		priv->contrast = ctrl->value;
		return set_contrast(client, priv->contrast);
	case V4L2_CID_SATURATION:
		if (ctrl->value < 0 || ctrl->value > 2) {
			return -EINVAL;
		}
		priv->sat = ctrl->value;
		return set_sat(client, priv->sat);
	case V4L2_CID_CAMERA_SPECIAL_EFFECT:
		if (ctrl->value < 0 || ctrl->value > 6) {
			return -EINVAL;
		}
		priv->effect = ctrl->value;
		return set_effect(client, priv->effect);
	case V4L2_CID_DO_WHITE_BALANCE:
		if (ctrl->value < 0 || ctrl->value > 4) {
			return -EINVAL;
		}
		priv->wh_bal = ctrl->value;
		return set_wh_bal(client, priv->wh_bal);
	case V4L2_CID_EXPOSURE:
		if (ctrl->value < -3 || ctrl->value > 3) {
			return -EINVAL;
		}
		priv->exposure = ctrl->value;
		return set_exposure(client, priv->exposure);
	default:
		return -EINVAL;
	}

	return 0;
}

/* Get chip identification */
static int gt2005_g_chip_ident(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct gt2005_priv *priv = to_gt2005(sd);

	id->ident = priv->ident;

	return 0;
}

static struct v4l2_subdev_core_ops gt2005_core_ops = {
	.g_ctrl = gt2005_g_ctrl,
	.s_ctrl = gt2005_s_ctrl,
	.g_chip_ident = gt2005_g_chip_ident,
};

/* Start/Stop streaming from the device */
static int gt2005_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gt2005_priv *priv = to_gt2005(sd);
	int ret = 0;

	/* Program orientation register. */
	ret = set_flip(client, priv->flag_vflip);
	if (ret < 0)
		return ret;

	ret = set_brightness(client, priv->brightness);
	if (ret < 0)
		return ret;

	ret = set_contrast(client, priv->contrast);
	if (ret < 0)
		return ret;

	ret = set_sat(client, priv->sat);
	if (ret < 0)
		return ret;

	ret = set_effect(client, priv->effect);
	if (ret < 0)
		return ret;

	ret = set_wh_bal(client, priv->wh_bal);
	if (ret < 0)
		return ret;

	ret = set_mirror(client, priv->flag_hflip);
	if (ret < 0)
		return ret;

	ret = set_scene(client, priv->scene);
	if (ret < 0)
		return ret;

	ret = set_exposure(client, priv->exposure);
	if (ret < 0)
		return ret;

	if (enable) {
		dev_dbg(&client->dev, "Enabling Streaming\n");
		/* Start Streaming */
		gt2005_write(client, 0x0104, 0x3);
	} else {
		dev_dbg(&client->dev, "Disabling Streaming\n");
		/* enter standby mode */
		gt2005_write(client, 0x0104, 0x0);
	}

	return ret;
}

/* select nearest higher resolution for capture */
static void gt2005_res_roundup(u32 * width, u32 * height)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(gt2005_resolutions); ++i) {
		if ((gt2005_resolutions[i].width >= *width) &&
		    (gt2005_resolutions[i].height >= *height)) {
			*width = gt2005_resolutions[i].width;
			*height = gt2005_resolutions[i].height;
			return;
		}
	}

	*width = gt2005_resolutions[GT2005_UXGA].width;
	*height = gt2005_resolutions[GT2005_UXGA].height;
}

/* Setup registers according to resolution and color encoding */
static int gt2005_set_res(struct i2c_client *client, u32 width, u32 height)
{
	int ret;

	/* select register configuration for given resolution */
	if (width == gt2005_resolutions[GT2005_QVGA].width) {
		dev_dbg(&client->dev, "Setting image size to 320x240\n");
		ret = gt2005_write_array(client, gt2005_qvga);
	} else if (width == gt2005_resolutions[GT2005_VGA].width) {
		dev_dbg(&client->dev, "Setting image size to 640x480\n");
		ret = gt2005_write_array(client, gt2005_vga);
	} else if (width == gt2005_resolutions[GT2005_480P].width) {
		dev_dbg(&client->dev, "Setting image size to 720x480\n");
		ret = gt2005_write_array(client, gt2005_480p);
	} else if (width == gt2005_resolutions[GT2005_SVGA].width) {
		dev_dbg(&client->dev, "Setting image size to 800x600\n");
		ret = gt2005_write_array(client, gt2005_svga);
	} else if (width == gt2005_resolutions[GT2005_XGA].width) {
		dev_dbg(&client->dev, "Setting image size to 1024x768\n");
		ret = gt2005_write_array(client, gt2005_xga);
	} else if (width == gt2005_resolutions[GT2005_720P].width
		   && height == gt2005_resolutions[GT2005_720P].height) {
		dev_dbg(&client->dev, "Setting image size to 1280x720\n");
		ret = gt2005_write_array(client, gt2005_hd);
	} else if (width == gt2005_resolutions[GT2005_SXGA].width
		   && height == gt2005_resolutions[GT2005_SXGA].height) {
		dev_dbg(&client->dev, "Setting image size to 1280x960\n");
		ret = gt2005_write_array(client, gt2005_sxga);
	} else if (width == gt2005_resolutions[GT2005_UXGA].width) {
		dev_dbg(&client->dev, "Setting image size to 1600x1200\n");
		UXGA_Cap = 1;
		gt2005_write(client, 0x0300, 0xC1);

		ret = gt2005_read(client, 0x0012, &shutterH);
		ret = gt2005_read(client, 0x0013, &shutterL);

		//shutter = shutter/2;

		msleep(50);

		ret = gt2005_read(client, 0x0014, &AGain_shutterH);
		ret = gt2005_read(client, 0x0015, &AGain_shutterL);

		ret = gt2005_read(client, 0x0016, &DGain_shutterH);
		ret = gt2005_read(client, 0x0017, &DGain_shutterL);

		//AGain_shutter = ((AGain_shutterH<<8)|(AGain_shutterL&0xff));
		DGain_shutter = (DGain_shutterH << 8 | (DGain_shutterL & 0xff));
		DGain_shutter = DGain_shutter >> 2;
		shutter = ((shutterH << 8) | (shutterL & 0xff));

		ret = gt2005_write_array(client, gt2005_uxga);

		ret = gt2005_write(client, 0x0304, shutter >> 8);
		ret = gt2005_write(client, 0x0305, shutter & 0xff);

		ret = gt2005_write(client, 0x0307, AGain_shutterL);
		ret = gt2005_write(client, 0x0306, AGain_shutterH);
		ret = gt2005_write(client, 0x0308, DGain_shutter & 0xFF);
		ret = gt2005_write(client, 0x0300, 0x01);

		msleep(50);
	} else {
		dev_dbg(&client->dev, "Failed to select resolution!\n");
		return -EINVAL;
	}

	return ret;
}

/* set the format we will capture in */
static int gt2005_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gt2005_priv *priv = to_gt2005(sd);
	enum v4l2_colorspace cspace;
	enum v4l2_mbus_pixelcode code = mf->code;
	int ret;

	PDBG("++\n");

	switch (code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
		cspace = V4L2_COLORSPACE_SRGB;
		break;
	default:
		return -EINVAL;
	}

	if (priv->inited == 0) {
		PDBG("write init regs\n");
		ret = gt2005_write_array(client, gt2005_init_regs);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s() : write gt2005_init_regs failed!!",
				__func__);
			return ret;
		}
		priv->inited = 1;
	}

	ret = gt2005_set_res(client, mf->width, mf->height);
	if (ret < 0)
		return ret;

	mf->code = code;
	mf->colorspace = cspace;

	return ret;
}

static int gt2005_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	gt2005_res_roundup(&mf->width, &mf->height);

	mf->field = V4L2_FIELD_NONE;
	mf->code = V4L2_MBUS_FMT_YUYV8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static enum v4l2_mbus_pixelcode gt2005_codes[] = {
	V4L2_MBUS_FMT_YUYV8_2X8,
};

static int gt2005_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(gt2005_codes))
		return -EINVAL;

	*code = gt2005_codes[index];

	return 0;
}

static int gt2005_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left = 0;
	a->bounds.top = 0;
	a->bounds.width = gt2005_resolutions[GT2005_UXGA].width;
	a->bounds.height = gt2005_resolutions[GT2005_UXGA].height;
	a->defrect = a->bounds;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator = 1;
	a->pixelaspect.denominator = 1;

	return 0;
}

static int gt2005_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	a->c.left = 0;
	a->c.top = 0;
	a->c.width = gt2005_resolutions[GT2005_UXGA].width;
	a->c.height = gt2005_resolutions[GT2005_UXGA].height;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static struct v4l2_subdev_video_ops gt2005_video_ops = {
	.s_stream = gt2005_s_stream,
	.s_mbus_fmt = gt2005_s_fmt,
	.try_mbus_fmt = gt2005_try_fmt,
	.enum_mbus_fmt = gt2005_enum_fmt,
	.cropcap = gt2005_cropcap,
	.g_crop = gt2005_g_crop,
};

static struct v4l2_subdev_ops gt2005_subdev_ops = {
	.core = &gt2005_core_ops,
	.video = &gt2005_video_ops,
};

static int gt2005_video_probe(struct soc_camera_device *icd,
			      struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gt2005_priv *priv = to_gt2005(sd);
	u8 id_hi, id_lo;
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
	 * check and show product ID
	 */
	ret = gt2005_read(client, 0x0000, &id_hi);
	if (ret < 0)
		goto err;

	ret = gt2005_read(client, 0x0001, &id_lo);
	if (ret < 0)
		goto err;

	if (id_hi != 0x51 || id_lo != 0x38) {	/* GT2005 product id. */
		dev_err(&client->dev,
			"got id = 0x%02X%02X, GT2005's id is 0x5138\n", id_hi,
			id_lo);
		ret = -ENODEV;
		goto err;
	}

	priv->ident = V4L2_IDENT_GT2005;

err:
	return ret;
}

#ifdef DEBUG_EN
static u8 dbg_addr = 0;

static ssize_t show_addr(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "0x%02X\n", dbg_addr);
}

static ssize_t store_addr(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t n)
{
	dbg_addr = simple_strtol(buf, NULL, 16);
	return n;
}

static ssize_t show_val(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 val;
	int ret = 0;

	ret = gt2005_read(client, dbg_addr, &val);

	if (ret < 0) {
		dev_err(&client->dev, "show_val: reg_read failed\n");
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

	ret = gt2005_write(client, dbg_addr, val);

	if (ret < 0) {
		dev_err(&client->dev, " store_val: reg_write failed\n");
		return ret;
	}

	return n;
}

static ssize_t store_test(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gt2005_priv *priv = to_gt2005(sd);
	int ret;
	u32 cmd;

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

/*
 * i2c_driver function
 */
static int gt2005_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct gt2005_priv *priv;
	struct soc_camera_device *icd = client->dev.platform_data;
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

	priv = kzalloc(sizeof(struct gt2005_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "Failed to allocate private data!\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &gt2005_subdev_ops);

	icd->ops = &gt2005_ops;

	ret = gt2005_video_probe(icd, client);
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

static int gt2005_remove(struct i2c_client *client)
{
	struct gt2005_priv *priv = i2c_get_clientdata(client);

	if (!priv) {
		return 0;
	}
#ifdef DEBUG_EN
	sysfs_remove_group(&client->dev.kobj, &dev_attr_group);
#endif

	kfree(priv);

	return 0;
}

static const struct i2c_device_id gt2005_id[] = {
	{DRIVER_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, gt2005_id);

static struct i2c_driver gt2005_i2c_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   },
	.probe = gt2005_probe,
	.remove = gt2005_remove,
	.id_table = gt2005_id,
};

static int __init gt2005_module_init(void)
{
	return i2c_add_driver(&gt2005_i2c_driver);
}

static void __exit gt2005_module_exit(void)
{
	i2c_del_driver(&gt2005_i2c_driver);
}

module_init(gt2005_module_init);
module_exit(gt2005_module_exit);

MODULE_DESCRIPTION("SoC Camera driver for GT2005");
MODULE_AUTHOR("huangrentong@nusmart.com");
MODULE_LICENSE("GPL v2");
