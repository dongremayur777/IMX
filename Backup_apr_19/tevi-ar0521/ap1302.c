// b/drivers/media/platform/imx8/imx_ap1302.c
//@ -0,0 +1,959 @@
/*
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 * Copyright 2022 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/firmware.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/version.h>
#include "ap1302.h"

#define AP1302_XCLK_MIN 10000000
#define AP1302_XCLK_MAX 48000000

#define to_ap1302_device(sub_dev) \
		container_of(sub_dev, struct ap1302_device, subdev)

static char *fw_file = "AP1302_AR0234.bin";
static int fw_fmt = 0;
static int fw_fps = 30;
static int fw_af_ctrl = 0;
static int fw_af_manual_pos = 100;

module_param(fw_file, charp, 0444);
module_param(fw_fmt, int, 0444);
module_param(fw_fps, int, 0444);
module_param(fw_af_ctrl, int, 0444);
module_param(fw_af_manual_pos, int, 0444);

struct ap1302_preview_out_fmt {
	u32 mbus_code;
	u16 fmt_reg_val;
};

struct ap1302_preview_out_fmt ap1302_preview_fmt[] = {
	{
		.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
		.fmt_reg_val = 0x50,
	},
	{
		.mbus_code = MEDIA_BUS_FMT_RGB888_1X24,
		.fmt_reg_val = 0x40,
	},
	{
		.mbus_code = MEDIA_BUS_FMT_RGB565_1X16,
		.fmt_reg_val = 0x41,
	},
};

/* Static definitions */
static struct regmap_config ap1302_reg16_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static struct ap1302_res_struct ap1302_preview_res[] = {
	{
		.width = 1920,
		.height = 1200,
	},
	{
		.width = 1920,
		.height = 1080,
	},
	{
		.width = 1280,
		.height = 720,
	},
	{
		.width = 640,
		.height = 480,
	},
};

static struct ap1302_res_struct ap1302_snapshot_res[] = {
	{
		.width = 640,
		.height = 480,
	},
	{
		.width = 720,
		.height = 480,
	},
	{
		.width = 1280,
		.height = 720,
	},
	{
		.width = 1920,
		.height = 1080,
	},
};

static struct ap1302_res_struct ap1302_video_res[] = {
	{
		.width = 640,
		.height = 480,
	},
	{
		.width = 720,
		.height = 480,
	},
	{
		.width = 1280,
		.height = 720,
	},
	{
		.width = 1920,
		.height = 1080,
	},
};

static int ap1302_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ap1302_remove(struct i2c_client *client);

static const struct i2c_device_id ap1302_id[] = {
	{AP1302_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ap1302_id);

static struct i2c_driver ap1302_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AP1302_NAME,
	},
	.probe  = ap1302_probe,
	.remove = ap1302_remove,
	.id_table = ap1302_id,
};

static inline void ap1302_power_down(struct ap1302_device *ap1302_dev, int enable)
{
	gpio_set_value_cansleep(ap1302_dev->pwn_gpio, enable);

	udelay(2000);
}

/* FIXME: Reset procedure should follow spec */
static inline void ap1302_reset(struct ap1302_device *ap1302_dev)
{
	gpio_set_value_cansleep(ap1302_dev->pwn_gpio, 1);
	gpio_set_value_cansleep(ap1302_dev->rst_gpio, 0);
	udelay(5000);

	gpio_set_value_cansleep(ap1302_dev->pwn_gpio, 0);
	udelay(1000);

	gpio_set_value_cansleep(ap1302_dev->rst_gpio, 1);
	msleep(20);
}

static int ap1302_write_reg(struct v4l2_subdev *sd, u16 reg,  u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 au8Buf[4] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >> 8;
	au8Buf[3] = val & 0xff;

	if (i2c_master_send(client, au8Buf, 4) < 0) {
		dev_err(&client->dev, "Write reg error: reg=0x%x, val=0x%x\n", reg, val);
		return -1;
	}

	return 0;
}

static int ap1302_read_reg(struct v4l2_subdev *sd, u16 reg, u16 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	u8 RegBuf[2] = {0};
	u8 ValBuf[2] = {0};

	RegBuf[0] = reg >> 8;
	RegBuf[1] = reg & 0xff;

	if (i2c_master_send(client, RegBuf, 2) != 2) {
		dev_err(&client->dev, "Read reg error: reg=0x%x\n", reg);
		return -1;
	}

	if (i2c_master_recv(client, ValBuf, 2) != 2) {
	dev_err(&client->dev, "Read reg error: reg=0x%x\n", reg);
		return -1;
	}

	*val = ((u16)ValBuf[0] << 8) | (u16)ValBuf[1];

	return 0;
}

static int ap1302_request_firmware(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);
	int ret;

	ret = request_firmware(&ap1302_dev->fw, fw_file, &client->dev);
	if (ret)
		dev_err(&client->dev, "%s failed. ret=%d\n", __func__, ret);
	return ret;
}

/* When loading firmware, host writes firmware data from address 0x8000.
 * When the address reaches 0x9FFF, the next address should return to 0x8000.
 * This function handles this address window and load firmware data to AP1302.
 * win_pos indicates the offset within this window. Firmware loading procedure
 * may call this function several times. win_pos records the current position
 * that has been written to.
 */

static int ap1302_write_fw_window(struct v4l2_subdev *sd,
				  u16 *win_pos, const u8 *buf, u32 len)
{
	struct ap1302_device *dev = to_ap1302_device(sd);
	int ret;
	u32 pos;
	u32 sub_len;

	for (pos = 0; pos < len; pos += sub_len) {
		if (len - pos < AP1302_FW_WINDOW_SIZE - *win_pos)
			sub_len = len - pos;
		else
			sub_len = AP1302_FW_WINDOW_SIZE - *win_pos;

		ret = regmap_raw_write(dev->regmap16,
					*win_pos + AP1302_FW_WINDOW_OFFSET,
					buf + pos, sub_len);

		if (ret)
			return ret;
		*win_pos += sub_len;
		if (*win_pos >= AP1302_FW_WINDOW_SIZE)
			*win_pos = 0;
	}
	return 0;
}

static int ap1302_stream_on(struct v4l2_subdev *sd)
{
	int ret = 0;
#if 0
	//u16 reg_val;

	printk("enter %s\n", __func__);
	ret = ap1302_write_reg(sd, 0x601A, 0x8340);
	mdelay(50);
	//ret = ap1302_read_reg(sd, 0x601A, &reg_val);
	//printk("%s, R0x601A val:0x%x\n", __func__, reg_val);
#endif
	return ret;
}
static int ap1302_stream_off(struct v4l2_subdev *sd)
{
	int ret = 0;
	//u16 reg_val;
#if 0
	printk("enter %s\n", __func__);
	ret = ap1302_write_reg(sd, 0x601A, 0x8040);
	ret = ap1302_write_reg(sd, 0x601A, 0x8140);
	mdelay(50);
	//ret = ap1302_read_reg(sd, 0x601A, &reg_val);
	//printk("%s, R0x601A val:0x%x\n", __func__, reg_val);

	ret = ap1302_write_reg(sd, 0xF038, 0x0023);
	ret = ap1302_write_reg(sd, 0xF040, 0x0000);
	ret = ap1302_write_reg(sd, 0xE000, 0x0000);
	ret = ap1302_write_reg(sd, 0xE002, 0x00C8);
	mdelay(10);
#endif
	return ret;
}

static int ap1302_load_firmware(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);
	struct ap1302_firmware *fw;
	const u8 *fw_data;
	u16 reg_val = 0;
	u16 win_pos = 0;
	int ret;

	dev_info(&client->dev, "Start to load firmware.\n");
	if (!ap1302_dev->fw) {
		dev_err(&client->dev, "firmware not requested.\n");
		return -EINVAL;
	}
	fw = (struct ap1302_firmware *)ap1302_dev->fw->data;
	if (ap1302_dev->fw->size != (sizeof(*fw) + fw->total_size)) {
		dev_err(&client->dev, "firmware size does not match.\n");
		return -EINVAL;
	}

	/* The fw binary contains a header of struct ap1302_firmware.
	 * Following the header is the bootdata of AP1302.
	 * The bootdata pointer can be referenced as &fw[1].
	 */
	fw_data = (u8 *)&fw[1];

	/* Clear crc register. */
	ret = ap1302_write_reg(sd, REG_SIP_CRC, 0xffff);
	if (ret)
		return ret;

	/* Load FW data for PLL init stage. */
	ret = ap1302_write_fw_window(sd, &win_pos, fw_data, fw->pll_init_size);
	if (ret)
		return ret;

	/* Write 2 to bootdata_stage register to apply basic_init_hp
	 * settings and enable PLL.
	 */
	ret = ap1302_write_reg(sd, REG_BOOTDATA_STAGE, 0x0002);
	if (ret)
		return ret;

	/* Wait 1ms for PLL to lock. */
	msleep(20);

	/* Load the rest of bootdata content. */
	ret = ap1302_write_fw_window(sd, &win_pos, fw_data + fw->pll_init_size,
				     fw->total_size - fw->pll_init_size);
	if (ret)
		return ret;

#if 0
	/* Check crc. */
	ret = ap1302_read_reg(sd, REG_SIP_CRC, &reg_val);
	if (ret)
		return ret;

	if (reg_val != fw->crc)
		dev_err(&client->dev, "crc does not match. T:0x%04X F:0x%04X\n", fw->crc, reg_val);
	else
		dev_info(&client->dev, "crc is right.\n");
#endif

	/* Write 0xFFFF to bootdata_stage register to indicate AP1302 that
	 * the whole bootdata content has been loaded.
	 */

	ret = ap1302_write_reg(sd, REG_BOOTDATA_STAGE, 0xFFFF);
	if (ret)
		return ret;

/* Delay 50ms */
	msleep(50);
	
	ret = ap1302_read_reg(sd, REG_BOOTDATA_CHECKSUM, &reg_val);
	if (ret)
		return ret;
	if (reg_val != fw->checksum) {
		dev_err(&client->dev, "checksum does not match. T:0x%04X F:0x%04X\n", fw->checksum, reg_val);
	dev_err(&client->dev, "please reload the driver\n");
		return -EAGAIN;
	} else
		dev_info(&client->dev, "checksum is right.\n");

	dev_info(&client->dev, "Load firmware successfully.\n");

	ap1302_stream_off(sd);

	return 0;
}

static int ap1302_set_clk_rate(struct ap1302_device *sensor)
{
	u32 tgt_xclk;	/* target xclk */
	int ret;

	/* mclk */
	tgt_xclk = sensor->mclk;
	tgt_xclk = min_t(u32, tgt_xclk, (u32)AP1302_XCLK_MAX);
	tgt_xclk = max_t(u32, tgt_xclk, (u32)AP1302_XCLK_MIN);
	sensor->mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(sensor->sensor_clk, sensor->mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", sensor->mclk);
	return ret;
}

/*!
 * ap1302_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ap1302_s_power(struct v4l2_subdev *sd, int on)
{
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);

	mutex_lock(&ap1302_dev->input_lock);
	if (on)
		clk_prepare_enable(ap1302_dev->sensor_clk);
	else
		clk_disable_unprepare(ap1302_dev->sensor_clk);
	mutex_unlock(&ap1302_dev->input_lock);

	ap1302_dev->power_on = on;
	return 0;
}


static enum ap1302_contexts ap1302_get_context(struct v4l2_subdev *sd)
{
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);

	return ap1302_dev->cur_context;


#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int ap1302_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int ap1302_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	//if (code->index)
	//	return -EINVAL;

	if (code->index >= ARRAY_SIZE(ap1302_preview_fmt))
                return -EINVAL;

	code->code = ap1302_preview_fmt[code->index].mbus_code;

	return 0;
}

static int ap1302_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	return 0;
}

static int ap1302_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	return 0;
}

static int get_capturemode(int width, int height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ap1302_preview_res); i++) {
		if ((ap1302_preview_res[i].width == width) &&
						(ap1302_preview_res[i].height == height))
			return i;
	}

	return -1;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int ap1302_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
#else
static int ap1302_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
#endif
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	enum ap1302_contexts context;
	struct ap1302_res_struct *res_table;
	s32 cur_res;
	int capturemode;
	int i;

	for (i = 0; i < ARRAY_SIZE(ap1302_preview_fmt); i++)
		if (ap1302_preview_fmt[i].mbus_code == fmt->code)
			break;
	if (i >= ARRAY_SIZE(ap1302_preview_fmt))
		printk("%s, unsupported fmt", __func__);

	fw_fmt = i;
	printk("%s, fw_fmt=%d\n", __func__, fw_fmt);
	fmt->code = ap1302_preview_fmt[fw_fmt].mbus_code;

	context = ap1302_get_context(sd);
	res_table = ap1302_dev->cntx_res[context].res_table;
	cur_res = ap1302_dev->streamcap.capturemode;

	capturemode = get_capturemode(fmt->width, fmt->height);
	printk("set capturemode:%d, cur_res:%d\n", capturemode, cur_res);
	if (capturemode >= 0) {
		if (capturemode != cur_res)
		{
			ap1302_dev->mode_change = true;
			ap1302_dev->streamcap.capturemode = capturemode;
			ap1302_dev->pix.width = fmt->width;
			ap1302_dev->pix.height = fmt->height;
			return 0;
		}
		else
		{
			ap1302_dev->mode_change = false;
			return 0;
		}
	}

	dev_err(&client->dev, "Set format failed wxh: %dx%d\n", fmt->width, fmt->height);
	return -EINVAL;

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int ap1302_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
#else
static int ap1302_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
#endif
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	enum ap1302_contexts context;
	struct ap1302_res_struct *res_table;
	s32 cur_res;

	if (format->pad)
		return -EINVAL;

	memset(fmt, 0, sizeof(struct v4l2_mbus_framefmt));
	mutex_lock(&ap1302_dev->input_lock);

	context = ap1302_get_context(sd);
	res_table = ap1302_dev->cntx_res[context].res_table;
	cur_res = ap1302_dev->streamcap.capturemode;

	fmt->code = ap1302_preview_fmt[fw_fmt].mbus_code;
	fmt->width = res_table[cur_res].width;
	fmt->height = res_table[cur_res].height;
	fmt->field = V4L2_FIELD_NONE;
	//fmt->reserved[1] = 0x7; //hs_settle

	mutex_unlock(&ap1302_dev->input_lock);

	dev_dbg(&client->dev, "%s code=0x%x, w/h=(%d,%d), colorspace=%d, field=%d\n",
		__func__, fmt->code, fmt->width, fmt->height, fmt->colorspace, fmt->field);

	return 0;
}

/*!
 * ap1302_enum_frame_size - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int ap1302_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_size_enum *fse)
#else
static int ap1302_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_size_enum *fse)
#endif
{
	struct ap1302_device *dev = to_ap1302_device(sd);
	enum ap1302_contexts context;
	struct ap1302_res_struct *res_table;
	int index = fse->index;

	mutex_lock(&dev->input_lock);
	context = ap1302_get_context(sd);
	if (index >= dev->cntx_res[context].res_num) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	res_table = dev->cntx_res[context].res_table;
	fse->min_width = res_table[index].width;
	fse->min_height = res_table[index].height;
	fse->max_width = res_table[index].width;
	fse->max_height = res_table[index].height;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ap1302_s_stream(struct v4l2_subdev *sd, int enable)
{
#if 1
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);
	int ret;
	u16 reg_val;
	//printk("enter %s, enable:%d\n", __func__, enable);
	if (enable) {
		ret = ap1302_stream_on(sd);
		printk("%s, mode:%d, mode_change:%d, width:%d, height:%d\n", __func__,
				ap1302_dev->streamcap.capturemode, ap1302_dev->mode_change,
				ap1302_dev->pix.width, ap1302_dev->pix.height);
		ap1302_dev->mode_change = 1;
		if (ap1302_dev->mode_change) {
			ap1302_write_reg(sd, 0x1184, 0x1);
			ap1302_write_reg(sd, REG_PREVIEW_BASE + CNTX_WIDTH, ap1302_dev->pix.width);
			ap1302_write_reg(sd, REG_PREVIEW_BASE + CNTX_HEIGHT, ap1302_dev->pix.height);
			ap1302_write_reg(sd, 0x2012, ap1302_preview_fmt[fw_fmt].fmt_reg_val);
			ap1302_write_reg(sd, 0x1184, 0xB);
			mdelay(20);
		}

		if (ret < 0)
			return ret;
	}
	else {
		ret = ap1302_stream_off(sd);
		if (ret < 0)
			return ret;
	}	
#endif
	return 0;
}

static int ap1302_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	return 0;
}

static int ap1302_pin_init(struct device *dev, struct ap1302_device *ap1302_dev)
{
	struct pinctrl *pinctrl;
	int retval;

	/* ap1302 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin, H_EN? */
	ap1302_dev->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(ap1302_dev->pwn_gpio))
		dev_warn(dev, "No sensor pwdn pin available");
	else {
		retval = devm_gpio_request_one(dev, ap1302_dev->pwn_gpio,
					GPIOF_OUT_INIT_HIGH, "ap1302_mipi_pwdn");
		if (retval < 0) {
			dev_warn(dev, "Failed to set power pin\n");
			dev_warn(dev, "retval=%d\n", retval);
			return retval;
		}
	}

	/* request reset pin, RST_B */
	ap1302_dev->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(ap1302_dev->rst_gpio))
		dev_warn(dev, "No sensor reset pin available");
	else {
		retval = devm_gpio_request_one(dev, ap1302_dev->rst_gpio,
					GPIOF_OUT_INIT_HIGH, "ap1302_mipi_reset");
		if (retval < 0) {
			dev_warn(dev, "Failed to set reset pin\n");
			return retval;
		}
	}

	return 0;
}

static int ap1302_clock_init(struct device *dev, struct ap1302_device *ap1302_dev)
{
	int retval;

	/* Set initial values for the sensor struct. */
	ap1302_dev->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ap1302_dev->sensor_clk)) {
		/* assuming clock enabled by default */
		ap1302_dev->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(ap1302_dev->sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(ap1302_dev->mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *)&(ap1302_dev->mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ap1302_dev->csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	/* Set mclk rate before clk on */
	ap1302_set_clk_rate(ap1302_dev);

	retval = clk_prepare_enable(ap1302_dev->sensor_clk);
	if (retval < 0) {
		dev_err(dev, "%s: enable sensor clk fail\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static struct v4l2_subdev_video_ops ap1302_subdev_video_ops = {
	.s_stream = ap1302_s_stream,
	.g_parm = ap1302_g_parm,
	.s_parm = ap1302_s_parm,
};

static struct v4l2_subdev_core_ops ap1302_subdev_core_ops = {
	.s_power	= ap1302_s_power,
};

static const struct v4l2_subdev_pad_ops ap1302_subdev_pad_ops = {
	.enum_mbus_code        = ap1302_enum_mbus_code,
	.enum_frame_size       = ap1302_enum_frame_size,
	.get_fmt               = ap1302_get_fmt,
	.set_fmt               = ap1302_set_fmt,
};

static struct v4l2_subdev_ops ap1302_subdev_ops = {
	.core	= &ap1302_subdev_core_ops,
	.video	= &ap1302_subdev_video_ops,
	.pad	= &ap1302_subdev_pad_ops,
};

static const struct media_entity_operations ap1302_sd_media_ops = {
	.link_setup = ap1302_link_setup,
};

/*!
 * ap1302 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ap1302_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	int retval = 0;
	struct ap1302_device *ap1302_dev;
	struct device *dev = &client->dev;
	struct ap1302_res_struct *res_table;
	s32 cur_res;
	u16 reg_val = 0;

	dev_info(dev, "ap1302 probe called.\n");

	/* allocate device & init sub device */
	ap1302_dev = devm_kmalloc(dev, sizeof(*ap1302_dev), GFP_KERNEL);
	if (!ap1302_dev) {
		dev_err(dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	/* Set initial values for the sensor struct. */
	memset(ap1302_dev, 0, sizeof(*ap1302_dev));

	/* ap1302 pin init */
	retval = ap1302_pin_init(dev, ap1302_dev);
	if (retval)
		return retval;

	ap1302_dev->i2c_client = client;

	/* clock init */
	retval = ap1302_clock_init(dev, ap1302_dev);
	if (retval)
		return retval;

	if (fw_fmt >= ARRAY_SIZE(ap1302_preview_fmt))
	{
		dev_err(&client->dev, "fw_fmt not supported\n");
		return -EINVAL;
	}

	ap1302_dev->streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ap1302_dev->streamcap.capturemode = 0;
	ap1302_dev->streamcap.timeperframe.denominator = fw_fps;
	ap1302_dev->streamcap.timeperframe.numerator = 1;

	mutex_init(&ap1302_dev->input_lock);
	mutex_lock(&ap1302_dev->input_lock);
	ap1302_reset(ap1302_dev);
	mutex_unlock(&ap1302_dev->input_lock);

	sd = &ap1302_dev->subdev;
	v4l2_i2c_subdev_init(sd, client, &ap1302_subdev_ops);

	retval = ap1302_request_firmware(sd);
	if (retval) {
		dev_err(&client->dev, "Cannot request ap1302 firmware.\n");
		goto out_free;
	}

	ap1302_dev->regmap16 = devm_regmap_init_i2c(client, &ap1302_reg16_config);
	if (IS_ERR(ap1302_dev->regmap16)) {
		retval = PTR_ERR(ap1302_dev->regmap16);
		dev_err(&client->dev,
			"Failed to allocate 16bit register map: %d\n", retval);
		return retval;
	}

	/* chip id */
	ap1302_read_reg(sd, REG_CHIP_VERSION, &reg_val);
	if (reg_val != AP1302_CHIP_ID) {
		dev_err(&client->dev,
			"Chip version does no match. ret=%d ver=0x%04x\n",
			retval, reg_val);
		goto out_free;
	}
	dev_info(&client->dev, "AP1302 Chip ID is 0x%X\n", reg_val);

	/* Load firmware after power on. */
	retval = ap1302_load_firmware(sd);
	if (retval) {
		dev_err(&client->dev,
			"ap1302_load_firmware failed. ret=%d\n", retval);
		goto out_free;
	}
#if 0
	if (fw_af_ctrl == 0) {
		ap1302_write_reg(sd, 0x5058, 0x0186);
		dev_info(dev, "AF Mode\n");
	}
	else {
		ap1302_write_reg(sd, 0x5058, 0x0183);
		ap1302_write_reg(sd, 0x505C, fw_af_manual_pos);
		dev_info(dev, "Manual Focuse Mode, pos: %d\n", fw_af_manual_pos);
	}
#endif
	ap1302_dev->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ap1302_dev->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ap1302_dev->pads[AP1302_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ap1302_dev->cntx_res[CONTEXT_PREVIEW].res_num = ARRAY_SIZE(ap1302_preview_res);
	ap1302_dev->cntx_res[CONTEXT_PREVIEW].res_table = ap1302_preview_res;
	ap1302_dev->cntx_res[CONTEXT_SNAPSHOT].res_num = ARRAY_SIZE(ap1302_snapshot_res);
	ap1302_dev->cntx_res[CONTEXT_SNAPSHOT].res_table = ap1302_snapshot_res;
	ap1302_dev->cntx_res[CONTEXT_VIDEO].res_num = ARRAY_SIZE(ap1302_video_res);
	ap1302_dev->cntx_res[CONTEXT_VIDEO].res_table = ap1302_video_res;

	/* Only Video is needed */
	ap1302_dev->cur_context = CONTEXT_PREVIEW;
	res_table = ap1302_dev->cntx_res[ap1302_dev->cur_context].res_table;
	cur_res = ap1302_dev->streamcap.capturemode;
	ap1302_dev->pix.width = res_table[cur_res].width;
	ap1302_dev->pix.height = res_table[cur_res].height;

	retval = media_entity_pads_init(&sd->entity, AP1302_SENS_PADS_NUM, ap1302_dev->pads);
	sd->entity.ops = &ap1302_sd_media_ops;
	if (retval)
		goto out_free;

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
	retval = v4l2_async_register_subdev_sensor(sd);
#else
	//retval = v4l2_async_register_subdev_sensor_common(sd);
	retval = v4l2_async_register_subdev(sd);
#endif
	if (retval < 0) {
		dev_err(&client->dev,
					"%s--Async register failed, ret=%d\n", __func__, retval);
		media_entity_cleanup(&sd->entity);
		goto out_free;
	}

	pr_info("%s ap1302, is found\n", __func__);
	return retval;

out_free:
	release_firmware(ap1302_dev->fw);
	ap1302_power_down(ap1302_dev, 1);
	clk_disable_unprepare(ap1302_dev->sensor_clk);
	return retval;
}

/*!
 * ap1302 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ap1302_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ap1302_device *ap1302_dev = to_ap1302_device(sd);

	pr_info("enter %s\n", __func__);
	release_firmware(ap1302_dev->fw);
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	ap1302_power_down(ap1302_dev, 1);
	clk_disable_unprepare(ap1302_dev->sensor_clk);
	mutex_destroy(&ap1302_dev->input_lock);
	return 0;
}

module_i2c_driver(ap1302_i2c_driver);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX AP1302 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("MIPI CSI");
