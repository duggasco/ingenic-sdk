// SPDX-License-Identifier: GPL-2.0+
/*
 * sc3332p.c
 * Ported SC3332P sensor driver for Ingenic T23 SoC with DTS mode selection
 * Copyright (C) 2012-2025 Ingenic Semiconductor Co., Ltd.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/proc_fs.h>
#include <tx-isp-common.h>
#include <sensor-common.h>
#include <sensor-info.h>

#define SENSOR_NAME "sc3332p"
#define SENSOR_CHIP_ID 0xcc44
#define SENSOR_CHIP_ID_H (0xcc)
#define SENSOR_CHIP_ID_L (0x44)
#define SENSOR_BUS_TYPE TX_SENSOR_CONTROL_INTERFACE_I2C
#define SENSOR_I2C_ADDRESS 0x30
#define SENSOR_MAX_WIDTH 2304
#define SENSOR_MAX_HEIGHT 1296
#define SENSOR_REG_END 0xffff
#define SENSOR_REG_DELAY 0xfffe
#define SENSOR_SUPPORT_30FPS_SCLK (74250000)
#define SENSOR_OUTPUT_MAX_FPS 120
#define SENSOR_OUTPUT_MIN_FPS 5
#define SENSOR_VERSION "H20250624a"

static int reset_gpio = GPIO_PA(18);
module_param(reset_gpio, int, S_IRUGO);
MODULE_PARM_DESC(reset_gpio, "Reset GPIO NUM");

static int pwdn_gpio = -1;
module_param(pwdn_gpio, int, S_IRUGO);
MODULE_PARM_DESC(pwdn_gpio, "Power down GPIO NUM");

static int data_interface = TX_SENSOR_DATA_INTERFACE_MIPI;
module_param(data_interface, int, S_IRUGO);
MODULE_PARM_DESC(data_interface, "Sensor Data interface");

static int sensor_mode_index = 0;
module_param(sensor_mode_index, int, S_IRUGO);
MODULE_PARM_DESC(sensor_mode_index, "Sensor Mode Index (0: 2304x1296@30fps, 1: 1080p@60fps, 2: 720p@120fps)");

static struct sensor_info sensor_info = {
	.name = SENSOR_NAME,
	.chip_id = SENSOR_CHIP_ID,
	.version = SENSOR_VERSION,
	.min_fps = SENSOR_OUTPUT_MIN_FPS,
	.max_fps = SENSOR_OUTPUT_MAX_FPS,
	.chip_i2c_addr = SENSOR_I2C_ADDRESS,
	.width = SENSOR_MAX_WIDTH,
	.height = SENSOR_MAX_HEIGHT,
};

struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

// Analog gain LUT placeholder (to be filled in with real values if available)
struct again_lut {
	unsigned int value;
	unsigned int gain;
};
static struct again_lut sensor_again_lut[] = {
	{0x80, 0},
	{0x84, 2886},
	{0x88, 5776},
	{0x8c, 8494},
	{0x90, 11136},
	{0x94, 13706},
	{0x98, 16287},
	{0x9c, 18723},
	{0xa0, 21097},
	// Extend as needed for real SC3332P gain values
};

static struct regval_list sc3332p_init_regs_mode0[] = {
	{0x0103, 0x01}, {0x36e9, 0x80}, {0x37f9, 0x80}, {0x301f, 0x01},
	{0x30b8, 0x44}, {0x3253, 0x10}, {0x3301, 0x08}, {0x3302, 0xff},
	{0x3305, 0x00}, {0x3306, 0x90}, {0x3308, 0x18}, {0x330a, 0x01},
	{0x330b, 0xc0}, {0x330d, 0x70}, {0x330e, 0x30}, {0x3314, 0x15},
	{0x3333, 0x10}, {0x3334, 0x40}, {0x335e, 0x06}, {0x335f, 0x0a},
	{0x3e00, 0x00}, {0x3e01, 0xa8}, {0x3e02, 0x60},
	{0x36e9, 0x54}, {0x37f9, 0x27}, {0x0100, 0x01},
	{SENSOR_REG_DELAY, 0x01},
	{SENSOR_REG_END, 0x00},
};
static struct regval_list sc3332p_init_regs_mode1[] = {
	{0x0103, 0x01}, {0x36e9, 0x80}, {0x37f9, 0x80}, {0x301f, 0x01},
	{0x30b8, 0x44}, {0x3253, 0x10}, {0x3301, 0x08}, {0x3302, 0xff},
	{0x3305, 0x00}, {0x3306, 0x90}, {0x3308, 0x18}, {0x330a, 0x01},
	{0x330b, 0xc0}, {0x330d, 0x70}, {0x330e, 0x30}, {0x3314, 0x15},
	{0x3333, 0x10}, {0x3334, 0x40}, {0x335e, 0x06}, {0x335f, 0x0a},
	{0x320c, 0x06}, {0x320d, 0x40}, // HTS = 1600
	{0x320e, 0x04}, {0x320f, 0x65}, // VTS = 1125
	{0x3e00, 0x00}, {0x3e01, 0x8c}, {0x3e02, 0x00},
	{0x36e9, 0x54}, {0x37f9, 0x27}, {0x0100, 0x01},
	{SENSOR_REG_DELAY, 0x01},
	{SENSOR_REG_END, 0x00},
};
static struct regval_list sc3332p_init_regs_mode2[] = {
	{0x0103, 0x01}, {0x36e9, 0x80}, {0x37f9, 0x80}, {0x301f, 0x01},
	{0x30b8, 0x44}, {0x3253, 0x10}, {0x3301, 0x08}, {0x3302, 0xff},
	{0x3305, 0x00}, {0x3306, 0x90}, {0x3308, 0x18}, {0x330a, 0x01},
	{0x330b, 0xc0}, {0x330d, 0x70}, {0x330e, 0x30}, {0x3314, 0x15},
	{0x3333, 0x10}, {0x3334, 0x40}, {0x335e, 0x06}, {0x335f, 0x0a},
	{0x320c, 0x05}, {0x320d, 0xdc}, // HTS = 1500
	{0x320e, 0x02}, {0x320f, 0x58}, // VTS = 600
	{0x3e00, 0x00}, {0x3e01, 0x60}, {0x3e02, 0x00},
	{0x36e9, 0x54}, {0x37f9, 0x27}, {0x0100, 0x01},
	{SENSOR_REG_DELAY, 0x01},
	{SENSOR_REG_END, 0x00},
};

static struct regval_list* sensor_modes[] = {
	sc3332p_init_regs_mode0,
	sc3332p_init_regs_mode1,
	sc3332p_init_regs_mode2,
};

static struct regval_list sc3332p_stream_on[] = {
	{0x0100, 0x01},
	{SENSOR_REG_END, 0x00},
};

static struct regval_list sc3332p_stream_off[] = {
	{0x0100, 0x00},
	{SENSOR_REG_END, 0x00},
};

static int sc3332p_write_array(struct tx_isp_subdev *sd, struct regval_list *vals)
{
	int ret = 0;
	while (vals->reg_num != SENSOR_REG_END) {
		if (vals->reg_num == SENSOR_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = tx_isp_sensor_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
				return ret;
		}
		vals++;
	}
	return 0;
}

static int sc3332p_detect(struct tx_isp_subdev *sd, unsigned int *ident)
{
	unsigned char high = 0, low = 0;
	if (tx_isp_sensor_read(sd, 0x3107, &high) < 0)
		return -ENODEV;
	if (tx_isp_sensor_read(sd, 0x3108, &low) < 0)
		return -ENODEV;
	*ident = (high << 8) | low;
	return (*ident == SENSOR_CHIP_ID) ? 0 : -ENODEV;
}

static int sc3332p_set_expo(struct tx_isp_subdev *sd, int value)
{
	int ret = 0;
	int it = value & 0xffff;
	int again = (value >> 16) & 0xffff;

	ret += tx_isp_sensor_write(sd, 0x3e00, (it >> 12) & 0x0f);
	ret += tx_isp_sensor_write(sd, 0x3e01, (it >> 4) & 0xff);
	ret += tx_isp_sensor_write(sd, 0x3e02, (it & 0xf) << 4);
	ret += tx_isp_sensor_write(sd, 0x3e07, again & 0xff);
	ret += tx_isp_sensor_write(sd, 0x3e09, (again >> 8) & 0xff);

	return ret;
}

static int sc3332p_set_fps(struct tx_isp_subdev *sd, int fps)
{
	unsigned int hts = 2500;
	unsigned int vts;
	unsigned int fps_m = (fps >> 16) & 0xffff;
	unsigned int fps_d = fps & 0xffff;
	unsigned int sclk = hts * 2700 * 15;
	if (fps_d == 0 || fps_m == 0) return -EINVAL;

	if ((fps_m * 100 / fps_d) > SENSOR_OUTPUT_MAX_FPS * 100 || (fps_m * 100 / fps_d) < SENSOR_OUTPUT_MIN_FPS * 100)
		return -EINVAL;

	vts = sclk * fps_d / hts / fps_m;
	tx_isp_sensor_write(sd, 0x320e, (vts >> 8) & 0xff);
	tx_isp_sensor_write(sd, 0x320f, vts & 0xff);
	return 0;
}

static int sc3332p_s_stream(struct tx_isp_subdev *sd, struct tx_isp_initarg *init)
{
	if (init->enable)
		return sc3332p_write_array(sd, sc3332p_stream_on);
	else
		return sc3332p_write_array(sd, sc3332p_stream_off);
}

static int sc3332p_init(struct tx_isp_subdev *sd, struct tx_isp_initarg *init)
{
	if (!init->enable)
		return 0;
	return sc3332p_write_array(sd, sensor_modes[sensor_mode_index]);
}

static int sc3332p_ioctl(struct tx_isp_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct tx_isp_sensor_value *val = arg;

	switch (cmd) {
	case TX_ISP_EVENT_SENSOR_RESIZE:
		if (val->value < ARRAY_SIZE(sensor_modes)) {
			sensor_mode_index = val->value;
			sc3332p_write_array(sd, sc3332p_stream_off);
			sc3332p_write_array(sd, sensor_modes[sensor_mode_index]);
			sc3332p_write_array(sd, sc3332p_stream_on);
			pr_info("SC3332P: Switched to mode index %d\n", sensor_mode_index);
			ret = 0;
		} else {
			pr_err("SC3332P: Invalid mode index: %d\n", val->value);
			ret = -EINVAL;
		}
		break;
	case TX_ISP_EVENT_SENSOR_EXPO:
		ret = sc3332p_set_expo(sd, val->value);
		break;
	case TX_ISP_EVENT_SENSOR_FPS:
		ret = sc3332p_set_fps(sd, val->value);
		break;
	case TX_ISP_EVENT_SENSOR_PREPARE_CHANGE:
		ret = sc3332p_write_array(sd, sc3332p_stream_off);
		break;
	case TX_ISP_EVENT_SENSOR_FINISH_CHANGE:
		ret = sc3332p_write_array(sd, sc3332p_stream_on);
		break;
	default:
		break;
	}
	return ret;
}

static struct tx_isp_subdev_core_ops sc3332p_core_ops = {
	.g_chip_ident = sc3332p_detect,
	.init = sc3332p_init,
};

static struct tx_isp_subdev_video_ops sc3332p_video_ops = {
	.s_stream = sc3332p_s_stream,
};

static struct tx_isp_subdev_sensor_ops sc3332p_sensor_ops = {
	.ioctl = sc3332p_ioctl,
};

static struct tx_isp_subdev_ops sc3332p_ops = {
	.core = &sc3332p_core_ops,
	.video = &sc3332p_video_ops,
	.sensor = &sc3332p_sensor_ops,
};

static int sc3332p_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tx_isp_subdev *sd;
	struct device_node *np = client->dev.of_node;
	u32 mode_val = 0;

	if (np && !of_property_read_u32(np, "ingenic,mode", &mode_val)) {
		if (mode_val < ARRAY_SIZE(sensor_modes)) {
			sensor_mode_index = mode_val;
		} else {
			pr_warn("SC3332P: Invalid ingenic,mode value %u, defaulting to mode 0\n", mode_val);
			sensor_mode_index = 0;
		}
	} else {
		pr_info("SC3332P: ingenic,mode not specified, defaulting to mode 0\n");
		sensor_mode_index = 0;
	}

	sd = devm_kzalloc(&client->dev, sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;
	tx_isp_subdev_init(NULL, sd, &sc3332p_ops);
	tx_isp_set_subdevdata(sd, client);
	pr_info("SC3332P: Sensor probed with mode index %d\n", sensor_mode_index);
	return 0;
}

static const struct i2c_device_id sc3332p_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc3332p_id);

static struct i2c_driver sc3332p_driver = {
	.driver = {
		.name = SENSOR_NAME,
	},
	.probe = sc3332p_probe,
	.id_table = sc3332p_id,
};

module_i2c_driver(sc3332p_driver);

MODULE_DESCRIPTION("SC3332P sensor driver for T23 with DTS mode selection");
MODULE_LICENSE("GPL");