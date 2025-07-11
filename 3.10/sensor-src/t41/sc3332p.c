// SPDX-License-Identifier: GPL-2.0+
/*
 * sc3332p.c
 * Copyright (C) 2022 Ingenic Semiconductor Co., Ltd.
 *
 * Settings:
 * sboot        resolution      fps       interface              mode
 *   0          2304*1296       30        mipi_2lane            linear
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
#define SENSOR_CHIP_ID_H (0xcc)
#define SENSOR_CHIP_ID_L (0x44)
#define SENSOR_REG_END 0xffff
#define SENSOR_REG_DELAY 0xfffe


#define SENSOR_OUTPUT_MAX_FPS 15
#define SENSOR_OUTPUT_MIN_FPS 5
#define SENSOR_VERSION "H20231115a"

static int reset_gpio = GPIO_PA(18);
module_param(reset_gpio, int, S_IRUGO);
MODULE_PARM_DESC(reset_gpio, "Reset GPIO NUM");

static int pwdn_gpio = GPIO_PA(19);
module_param(pwdn_gpio, int, S_IRUGO);
MODULE_PARM_DESC(pwdn_gpio, "Power down GPIO NUM");

static int data_interface = TX_SENSOR_DATA_INTERFACE_MIPI;
module_param(data_interface, int, S_IRUGO);
MODULE_PARM_DESC(data_interface, "Sensor Date interface");

struct regval_list {
    uint16_t reg_num;
    unsigned char value;
};

struct again_lut {
	unsigned int value;
	unsigned int gain;
};

struct again_lut sensor_again_lut[] = {
	{0x0080, 0},
	{0x0084, 2886},
	{0x0088, 5776},
	{0x008c, 8494},
	{0x0090, 11136},
	{0x0094, 13706},
	{0x0098, 16287},
	{0x009c, 18723},
	{0x00a0, 21097},
	{0x00a4, 23414},
	{0x00a8, 25746},
	{0x00ac, 27953},
	{0x00b0, 30109},
	{0x00b4, 32217},
	{0x00b8, 34345},
	{0x00bc, 36361},
	{0x00c0, 38336},
	{0x00c4, 40270},
	{0x00c8, 42226},
	{0x00cc, 44082},
	{0x00d0, 45904},
	{0x00d4, 47690},
	{0x00d8, 49500},
	{0x00dc, 51220},
	{0x00e0, 52910},
	{0x00e4, 54571},
	{0x00e8, 56254},
	{0x00ec, 57857},
	{0x00f0, 59433},
	{0x00f4, 60984},
	{0x00f8, 62558},
	{0x00fc, 64059},
	{0x0880, 65536},
	{0x0884, 68422},
	{0x0888, 71312},
	{0x088c, 74030},
	{0x0890, 76672},
	{0x0894, 79242},
	{0x0898, 81823},
	{0x089c, 84259},
	{0x08a0, 86633},
	{0x08a4, 88950},
	{0x08a8, 91282},
	{0x08ac, 93489},
	{0x08b0, 95645},
	{0x08b4, 97753},
	{0x08b8, 99881},
	{0x08bc, 101897},
	{0x08c0, 103872},
	{0x08c4, 105806},
	{0x08c8, 107762},
	{0x08cc, 109618},
	{0x08d0, 111440},
	{0x08d4, 113226},
	{0x08d8, 115036},
	{0x08dc, 116756},
	{0x08e0, 118446},
	{0x08e4, 120107},
	{0x08e8, 121790},
	{0x08ec, 123393},
	{0x08f0, 124969},
	{0x08f4, 126520},
	{0x08f8, 128094},
	{0x08fc, 129595},
	{0x0980, 131072},
	{0x0984, 133958},
	{0x0988, 136848},
	{0x098c, 139566},
	{0x0990, 142208},
	{0x0994, 144778},
	{0x0998, 147359},
	{0x099c, 149795},
	{0x09a0, 152169},
	{0x09a4, 154486},
	{0x09a8, 156818},
	{0x09ac, 159025},
	{0x09b0, 161181},
	{0x09b4, 163289},
	{0x09b8, 165417},
	{0x09bc, 167433},
	{0x09c0, 169408},
	{0x09c4, 171342},
	{0x09c8, 173298},
	{0x09cc, 175154},
	{0x09d0, 176976},
	{0x09d4, 178762},
	{0x09d8, 180572},
	{0x09dc, 182292},
	{0x09e0, 183982},
	{0x09e4, 185643},
	{0x09e8, 187326},
	{0x09ec, 188929},
	{0x09f0, 190505},
	{0x09f4, 192056},
	{0x09f8, 193630},
	{0x09fc, 195131},
	{0x0b80, 196608},
	{0x0b84, 199494},
	{0x0b88, 202384},
	{0x0b8c, 205102},
	{0x0b90, 207744},
	{0x0b94, 210314},
	{0x0b98, 212895},
	{0x0b9c, 215331},
	{0x0ba0, 217705},
	{0x0ba4, 220022},
	{0x0ba8, 222354},
	{0x0bac, 224561},
	{0x0bb0, 226717},
	{0x0bb4, 228825},
	{0x0bb8, 230953},
	{0x0bbc, 232969},
	{0x0bc0, 234944},
	{0x0bc4, 236878},
	{0x0bc8, 238834},
	{0x0bcc, 240690},
	{0x0bd0, 242512},
	{0x0bd4, 244298},
	{0x0bd8, 246108},
	{0x0bdc, 247828},
	{0x0be0, 249518},
	{0x0be4, 251179},
	{0x0be8, 252862},
	{0x0bec, 254465},
	{0x0bf0, 256041},
	{0x0bf4, 257592},
	{0x0bf8, 259166},
	{0x0bfc, 260667},
	{0x0f80, 262144},
	{0x0f84, 265030},
	{0x0f88, 267920},
	{0x0f8c, 270638},
	{0x0f90, 273280},
	{0x0f94, 275850},
	{0x0f98, 278431},
	{0x0f9c, 280867},
	{0x0fa0, 283241},
	{0x0fa4, 285558},
	{0x0fa8, 287890},
	{0x0fac, 290097},
	{0x0fb0, 292253},
	{0x0fb4, 294361},
	{0x0fb8, 296489},
	{0x0fbc, 298505},
	{0x0fc0, 300480},
	{0x0fc4, 302414},
	{0x0fc8, 304370},
	{0x0fcc, 306226},
	{0x0fd0, 308048},
	{0x0fd4, 309834},
	{0x0fd8, 311644},
	{0x0fdc, 313364},
	{0x0fe0, 315054},
	{0x0fe4, 316715},
	{0x0fe8, 318398},
	{0x0fec, 320001},
	{0x0ff0, 321577},
	{0x0ff4, 323128},
	{0x0ff8, 324702},
	{0x0ffc, 326203},
	{0x1f80, 327680},

};
struct tx_isp_sensor_attribute sensor_attr;

unsigned int sensor_alloc_again(unsigned int isp_gain, unsigned char shift, unsigned int *sensor_again)
{
	struct again_lut *lut = sensor_again_lut;
	while (lut->gain <= sensor_attr.max_again) {
		if (isp_gain == 0) {
			*sensor_again = lut[0].value;
			return lut[0].gain;
		} else if (isp_gain < lut->gain) {
			*sensor_again = (lut - 1)->value;
			return (lut - 1)->gain;
		} else {
			if ((lut->gain == sensor_attr.max_again) && (isp_gain >= lut->gain)) {
				*sensor_again = lut->value;
				return lut->gain;
			}
		}

		lut++;
	}

	return isp_gain;
}

unsigned int sensor_alloc_dgain(unsigned int isp_gain, unsigned char shift, unsigned int *sensor_dgain)
{
	return 0;
}

struct tx_isp_sensor_attribute sensor_attr={
	.name = SENSOR_NAME,
	.chip_id = 0xcc44,
	.cbus_type = TX_SENSOR_CONTROL_INTERFACE_I2C,
	.cbus_mask = V4L2_SBUS_MASK_SAMPLE_8BITS | V4L2_SBUS_MASK_ADDR_16BITS,
	.cbus_device = 0x30,
	.dbus_type = TX_SENSOR_DATA_INTERFACE_MIPI,
	.mipi = {
		.mode = SENSOR_MIPI_OTHER_MODE,
		.clk = 510,
		.lans = 2,
		.settle_time_apative_en = 1,
		.mipi_sc.sensor_csi_fmt = TX_SENSOR_RAW10,
		.mipi_sc.hcrop_diff_en = 0,
		.mipi_sc.mipi_vcomp_en = 0,
		.mipi_sc.mipi_hcomp_en = 0,
		.mipi_sc.line_sync_mode = 0,
		.mipi_sc.work_start_flag = 0,
		.image_twidth = 2304,
		.image_theight = 1296,
		.mipi_sc.mipi_crop_start0x = 0,
		.mipi_sc.mipi_crop_start0y = 0,
		.mipi_sc.mipi_crop_start1x = 0,
		.mipi_sc.mipi_crop_start1y = 0,
		.mipi_sc.mipi_crop_start2x = 0,
		.mipi_sc.mipi_crop_start2y = 0,
		.mipi_sc.mipi_crop_start3x = 0,
		.mipi_sc.mipi_crop_start3y = 0,
		.mipi_sc.data_type_en = 0,
		.mipi_sc.data_type_value = RAW10,
		.mipi_sc.del_start = 0,
		.mipi_sc.sensor_frame_mode = TX_SENSOR_DEFAULT_FRAME_MODE,
		.mipi_sc.sensor_fid_mode = 0,
		.mipi_sc.sensor_mode = TX_SENSOR_DEFAULT_MODE,
	},
	.data_type = TX_SENSOR_DATA_TYPE_LINEAR,
	.max_again = 327680,
	.max_dgain = 0,
	.min_integration_time = 2,
	.min_integration_time_native = 2,





	.integration_time_apply_delay = 2,
	.again_apply_delay = 2,
	.dgain_apply_delay = 0,
	.sensor_ctrl.alloc_again = sensor_alloc_again,
	.sensor_ctrl.alloc_dgain = sensor_alloc_dgain,
};


static struct regval_list sensor_init_regs_2304_1296_30fps_mipi_2lane[] = {
	{0x0103, 0x01},
	{0x36e9, 0x80},
	{0x37f9, 0x80},
	{0x301f, 0x01},
	{0x30b8, 0x44},
	{0x3253, 0x10},
	{0x3301, 0x08},
	{0x3302, 0xff},
	{0x3305, 0x00},
	{0x3306, 0x90},
	{0x3308, 0x18},
	{0x330a, 0x01},
	{0x330b, 0xc0},
	{0x330d, 0x70},
	{0x330e, 0x30},
	{0x3314, 0x15},
	{0x3333, 0x10},
	{0x3334, 0x40},
	{0x335e, 0x06},
	{0x335f, 0x0a},
	{0x3364, 0x5e},
	{0x337d, 0x0e},
	{0x3390, 0x08},
	{0x3391, 0x09},
	{0x3392, 0x0f},
	{0x3393, 0x10},
	{0x3394, 0x80},
	{0x3395, 0xff},
	{0x33a2, 0x04},
	{0x33ad, 0x2c},
	{0x33b0, 0x0f},
	{0x33b3, 0x48},
	{0x33f8, 0x00},
	{0x33f9, 0xc0},
	{0x33fa, 0x00},
	{0x33fb, 0xf0},
	{0x33fc, 0x0b},
	{0x33fd, 0x0f},
	{0x349f, 0x03},
	{0x34a6, 0x0b},
	{0x34a7, 0x0f},
	{0x34a8, 0x40},
	{0x34a9, 0x30},
	{0x34aa, 0x01},
	{0x34ab, 0xf0},
	{0x34ac, 0x02},
	{0x34ad, 0x10},
	{0x34f8, 0x1f},
	{0x34f9, 0x30},
	{0x3630, 0xf0},
	{0x3631, 0x8c},
	{0x3632, 0x78},
	{0x3633, 0x33},
				
				
				
	{0x363a, 0xcc},
	{0x363c, 0x0f},
	{0x363f, 0xc0},
	{0x3641, 0x20},
	{0x3670, 0x5e},
	{0x3674, 0xf0},
	{0x3675, 0xf0},
	{0x3676, 0xd0},
	{0x3677, 0x87},
	{0x3678, 0x8a},
	{0x3679, 0x8d},
	{0x367c, 0x08},
	{0x367d, 0x0f},
	{0x367e, 0x08},
	{0x367f, 0x0f},
	{0x3690, 0x44},
	{0x3691, 0x48},
	{0x3692, 0x48},
	{0x3696, 0x34},
	{0x3697, 0x34},
	{0x3698, 0x45},
	{0x369c, 0x0b},
	{0x369d, 0x0f},
	{0x36a0, 0x09},
	{0x36a1, 0x0f},
	{0x36b0, 0x88},
	{0x36b1, 0x91},
	{0x36b2, 0xa4},
	{0x36b3, 0xcf},
	{0x36b4, 0x09},
	{0x36b5, 0x0b},
	{0x36b6, 0x0f},
	{0x370f, 0x01},
	{0x3714, 0x81},
	{0x3722, 0x05},
	{0x3724, 0x31},
	{0x3771, 0x09},
	{0x3772, 0x05},
	{0x3773, 0x05},
	{0x377a, 0x0b},
	{0x377b, 0x0f},
				
				
	{0x3900, 0x0d},
	{0x3903, 0x08},
				
				
	{0x3904, 0x04},
	{0x3905, 0x8c},
	{0x391d, 0x01},
	{0x391f, 0x49},
	{0x3922, 0x1f},
	{0x3925, 0x0f},
	{0x3926, 0x21},
				
				
				
				
				
				
				
	{0x3933, 0x80},
	{0x3934, 0x03},
	{0x3935, 0x00},
	{0x3936, 0xa0},
	{0x3937, 0x78},
	{0x3938, 0x76},
	{0x393b, 0x00},
	{0x393c, 0x18},
	{0x39dc, 0x02},
	{0x3e00, 0x00},
	{0x3e01, 0x54},
	{0x3e02, 0x00},
	{0x440e, 0x02},
	{0x4509, 0x28},
	{0x450d, 0x32},
	{0x5000, 0x06},
	{0x5002, 0x06},
	{0x5780, 0x66},
	{0x5784, 0x08},
	{0x5785, 0x04},
	{0x5787, 0x08},
	{0x5788, 0x03},
	{0x5789, 0x01},
	{0x578a, 0x08},
	{0x578b, 0x03},
	{0x578c, 0x01},
	{0x578d, 0x40},
	{0x5790, 0x08},
	{0x5791, 0x04},
	{0x5792, 0x01},
	{0x5793, 0x08},
	{0x5794, 0x04},
	{0x5795, 0x01},//15fps
	{0x320e, 0x0a},//2700
	{0x320f, 0x8c},
	{0x3e00, 0x00},
	{0x3e01, 0xa8},
	{0x3e02, 0x60},
	{0x36e9, 0x54},
	{0x37f9, 0x27},
	{0x0100, 0x01},
	{SENSOR_REG_DELAY, 0x01},

	{SENSOR_REG_END, 0x00},
};

static struct tx_isp_sensor_win_setting sensor_win_sizes[] = {
	{
		.width = 2304,
		.height = 1296,
		.fps = 15 << 16 | 1,
		.mbus_code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.regs = sensor_init_regs_2304_1296_30fps_mipi_2lane,
	},
};
struct tx_isp_sensor_win_setting *wsize = &sensor_win_sizes[0];

static struct regval_list sensor_stream_on_mipi[] = {
	{0x0100, 0x01},
	{SENSOR_REG_END, 0x00},
};

static struct regval_list sensor_stream_off_mipi[] = {
	{0x0100, 0x00},
	{SENSOR_REG_END, 0x00},
};

int sensor_read(struct tx_isp_subdev *sd, uint16_t reg, unsigned char *value)
{
	struct i2c_client *client = tx_isp_get_subdevdata(sd);
	unsigned char buf[2] = {reg >> 8, reg & 0xff};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		[1] = {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = value,
		}
	};
	int ret;
	ret = private_i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}

int sensor_write(struct tx_isp_subdev *sd, uint16_t reg, unsigned char value)
{
	struct i2c_client *client = tx_isp_get_subdevdata(sd);
	uint8_t buf[3] = {(reg >> 8) & 0xff, reg & 0xff, value};
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 3,
		.buf = buf,
	};
	int ret;
	ret = private_i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int sensor_read_array(struct tx_isp_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != SENSOR_REG_END) {
		if (vals->reg_num == SENSOR_REG_DELAY) {
			private_msleep(vals->value);
		} else {
			ret = sensor_read(sd, vals->reg_num, &val);
			if (ret < 0)
				return ret;
		}
		vals++;
	}

	return 0;
}
static int sensor_write_array(struct tx_isp_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != SENSOR_REG_END) {
		if (vals->reg_num == SENSOR_REG_DELAY) {
			private_msleep(vals->value);
		} else {
			ret = sensor_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
				return ret;
		}
		vals++;
	}

	return 0;
}

static int sensor_reset(struct tx_isp_subdev *sd, int val)
{
	return 0;
}

static int sensor_detect(struct tx_isp_subdev *sd, unsigned int *ident)
{
	int ret;
	unsigned char v;

	ret = sensor_read(sd, 0x3107, &v);
	ISP_WARNING("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SENSOR_CHIP_ID_H)
		return -ENODEV;
	*ident = v;

	ret = sensor_read(sd, 0x3108, &v);
	ISP_WARNING("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0)
		return ret;
	if (v != SENSOR_CHIP_ID_L)
		return -ENODEV;
	*ident = (*ident << 8) | v;

	return 0;
}

static int sensor_set_expo(struct tx_isp_subdev *sd, int value)
{
	int ret = -1;
	int it = (value & 0xffff);
	int again = (value & 0xffff0000) >> 16;

	ret += sensor_write(sd, 0x3e00, (unsigned char) ((it >> 12) & 0xf));
	ret += sensor_write(sd, 0x3e01, (unsigned char) ((it >> 4) & 0xff));
	ret += sensor_write(sd, 0x3e02, (unsigned char) ((it & 0x0f) << 4));
	ret = sensor_write(sd, 0x3e07, (unsigned char) (again & 0xff));
	ret += sensor_write(sd, 0x3e09, (unsigned char) (((again >> 8) & 0xff)));

























	return 0;
}

static int sensor_set_integration_time(struct tx_isp_subdev *sd, int value)
{
       int ret = 0;
       ret = sensor_write(sd, 0x3e00, (unsigned char)((value >> 12) & 0x0f));
       ret += sensor_write(sd, 0x3e01, (unsigned char)((value >> 4) & 0xff));
       ret += sensor_write(sd, 0x3e02, (unsigned char)((value & 0x0f) << 4));
       if (ret < 0)
	       return ret;
	   
	   
	   
	   
	   
	   
	   
	   

	return 0;
}

static int sensor_set_analog_gain(struct tx_isp_subdev *sd, int value)
{
       int ret = 0;

       ret += sensor_write(sd, 0x3e07, (unsigned char)(value & 0xff));
       ret += sensor_write(sd, 0x3e09, (unsigned char)((value & 0xff00) >> 8));
       if (ret < 0)
	       return ret;






















															   
																		 
			 
       return 0;

}

static int sensor_set_digital_gain(struct tx_isp_subdev *sd, int value)
{
	return 0;
}

static int sensor_get_black_pedestal(struct tx_isp_subdev *sd, int value)
{
	return 0;
}

static int sensor_init(struct tx_isp_subdev *sd, int enable)
{
	struct tx_isp_sensor *sensor = sd_to_sensor_device(sd);
	int ret = 0;

	if (!enable)
		return ISP_SUCCESS;

	sensor->video.mbus.width = wsize->width;
	sensor->video.mbus.height = wsize->height;
	sensor->video.mbus.code = wsize->mbus_code;
	sensor->video.mbus.field = V4L2_FIELD_NONE;
	sensor->video.mbus.colorspace = wsize->colorspace;
	sensor->video.fps = wsize->fps;
	sensor->video.max_fps = wsize->fps;
	sensor->video.min_fps = SENSOR_OUTPUT_MIN_FPS << 16 | 1;		 

	ret = sensor_write_array(sd, wsize->regs);

	if (ret)
		return ret;
	ret = tx_isp_call_subdev_notify(sd, TX_ISP_EVENT_SYNC_SENSOR_ATTR, &sensor->video);
	sensor->priv = wsize;

	return 0;
}

static int sensor_s_stream(struct tx_isp_subdev *sd, int enable)
{
	int ret = 0;

	if (enable) {
		if (data_interface == TX_SENSOR_DATA_INTERFACE_MIPI) {
			ret = sensor_write_array(sd, sensor_stream_on_mipi);
		} else {
			ISP_ERROR("Don't support this Sensor Data interface\n");
		}
		ISP_WARNING("%s stream on\n", SENSOR_NAME);

	}
	else {
		if (data_interface == TX_SENSOR_DATA_INTERFACE_MIPI) {
			ret = sensor_write_array(sd, sensor_stream_off_mipi);
		} else {
			ISP_ERROR("Don't support this Sensor Data interface\n");
		}
		ISP_WARNING("%s stream off\n", SENSOR_NAME);
	}

	return ret;
}

static int sensor_set_fps(struct tx_isp_subdev *sd, int fps)
{
	struct tx_isp_sensor *sensor = sd_to_sensor_device(sd);
	unsigned int sclk = 0;
	unsigned int hts = 0;
	unsigned int vts = 0;
	unsigned char tmp = 0;

	unsigned int newformat = 0; //the format is 24.8
	int ret = 0;

	newformat = (((fps >> 16) / (fps & 0xffff)) << 8) + ((((fps >> 16) % (fps & 0xffff)) << 8) / (fps & 0xffff));
	if (newformat > (SENSOR_OUTPUT_MAX_FPS << 8) || newformat < (SENSOR_OUTPUT_MIN_FPS << 8)) {
		ISP_ERROR("warn: fps(%d) not in range\n", fps);
		return -1;
	}
	

	ret = sensor_read(sd, 0x320c, &tmp);
	hts = tmp;
	ret += sensor_read(sd, 0x320d, &tmp);
	if (0 != ret) {
		ISP_ERROR("Error: %s read error\n", SENSOR_NAME);
		return ret;
	}
	hts = ((hts << 8) + tmp);
	vts = sclk * (fps & 0xffff) / hts / ((fps & 0xffff0000) >> 16);

//	ret = sensor_write(sd,0x3812,0x00);
	ret += sensor_write(sd, 0x320f, (unsigned char)(vts & 0xff));
	ret += sensor_write(sd, 0x320e, (unsigned char)(vts >> 8));
//	ret += sensor_write(sd,0x3812,0x30);
	if (0 != ret) {
		ISP_ERROR("Error: %s write error\n", SENSOR_NAME);
		return ret;
	}

	sensor->video.fps = fps;
	sensor->video.attr->max_integration_time_native = vts - 8;
	sensor->video.attr->integration_time_limit = vts - 8;
	sensor->video.attr->total_height = vts;
	sensor->video.attr->max_integration_time = vts - 8;
	ret = tx_isp_call_subdev_notify(sd, TX_ISP_EVENT_SYNC_SENSOR_ATTR, &sensor->video);

	return ret;
}

static int sensor_set_mode(struct tx_isp_subdev *sd, int value)
{
	struct tx_isp_sensor *sensor = sd_to_sensor_device(sd);
	int ret = ISP_SUCCESS;

	if (wsize) {
		sensor->video.mbus.width = wsize->width;
		sensor->video.mbus.height = wsize->height;
		sensor->video.mbus.code = wsize->mbus_code;
		sensor->video.mbus.field = V4L2_FIELD_NONE;
		sensor->video.mbus.colorspace = wsize->colorspace;
		sensor->video.fps = wsize->fps;
		ret = tx_isp_call_subdev_notify(sd, TX_ISP_EVENT_SYNC_SENSOR_ATTR, &sensor->video);
	}

	return ret;
}

static int sensor_g_chip_ident(struct tx_isp_subdev *sd,
			       struct tx_isp_chip_ident *chip)
{
	struct i2c_client *client = tx_isp_get_subdevdata(sd);
	unsigned int ident = 0;
	int ret = ISP_SUCCESS;
	if (reset_gpio != -1) {
		ret = private_gpio_request(reset_gpio,"sensor_reset");
		if (!ret) {
			private_gpio_direction_output(reset_gpio, 1);
			private_msleep(5);
			private_gpio_direction_output(reset_gpio, 0);
			private_msleep(5);
			private_gpio_direction_output(reset_gpio, 1);
			private_msleep(5);
		} else {
			ISP_ERROR("gpio request fail %d\n",reset_gpio);
		}
	}
	if (pwdn_gpio != -1) {
		ret = private_gpio_request(pwdn_gpio,"sensor_pwdn");
		if (!ret) {
			private_gpio_direction_output(pwdn_gpio, 1);
			private_msleep(5);
			private_gpio_direction_output(pwdn_gpio, 0);
			private_msleep(5);
		} else {
			ISP_ERROR("gpio request fail %d\n",pwdn_gpio);
		}
	}
	ret = sensor_detect(sd, &ident);
	if (ret) {
		ISP_ERROR("chip found @ 0x%x (%s) is not an %s chip.\n",
			  client->addr, client->adapter->name, SENSOR_NAME);
		return ret;
	}
	ISP_WARNING("%s chip found @ 0x%02x (%s)\n",
		    SENSOR_NAME, client->addr, client->adapter->name);
	ISP_WARNING("sensor driver version %s\n",SENSOR_VERSION);
	if (chip) {
		memcpy(chip->name, SENSOR_NAME, sizeof(SENSOR_NAME));
		chip->ident = ident;
		chip->revision = SENSOR_VERSION;
	}

	return 0;
}

static int sensor_sensor_ops_ioctl(struct tx_isp_subdev *sd, unsigned int cmd, void *arg)
{
	long ret = 0;
	if (IS_ERR_OR_NULL(sd)) {
		ISP_ERROR("[%d]The pointer is invalid!\n", __LINE__);
		return -EINVAL;
	}
	switch(cmd) {
	case TX_ISP_EVENT_SENSOR_EXPO:
		if (arg)
			ret = sensor_set_expo(sd, *(int*)arg);
		break;
	case TX_ISP_EVENT_SENSOR_INT_TIME:
//		if (arg)
		//ret = sensor_set_integration_time(sd, *(int*)arg);
		break;
	case TX_ISP_EVENT_SENSOR_AGAIN:
//		if (arg)
//			ret = sensor_set_analog_gain(sd, *(int*)arg);
		break;
	case TX_ISP_EVENT_SENSOR_DGAIN:
		if (arg)
			ret = sensor_set_digital_gain(sd, *(int*)arg);
		break;
	case TX_ISP_EVENT_SENSOR_BLACK_LEVEL:
		if (arg)
			ret = sensor_get_black_pedestal(sd, *(int*)arg);
		break;
	case TX_ISP_EVENT_SENSOR_RESIZE:
		if (arg)
			ret = sensor_set_mode(sd, *(int*)arg);
		break;
	case TX_ISP_EVENT_SENSOR_PREPARE_CHANGE:
		if (data_interface == TX_SENSOR_DATA_INTERFACE_MIPI) {
			ret = sensor_write_array(sd, sensor_stream_off_mipi);

		} else {
			ISP_ERROR("Don't support this Sensor Data interface\n");
		}
		break;
	case TX_ISP_EVENT_SENSOR_FINISH_CHANGE:
		if (data_interface == TX_SENSOR_DATA_INTERFACE_MIPI) {
			ret = sensor_write_array(sd, sensor_stream_on_mipi);

		} else {
			ISP_ERROR("Don't support this Sensor Data interface\n");
			ret = -1;
		}
		break;
	case TX_ISP_EVENT_SENSOR_FPS:
		if (arg)
			ret = sensor_set_fps(sd, *(int*)arg);
		break;
	default:
		break;
	}

	return ret;
}

static int sensor_g_register(struct tx_isp_subdev *sd, struct tx_isp_dbg_register *reg)
{
	unsigned char val = 0;
	int len = 0;
	int ret = 0;

	len = strlen(sd->chip.name);
	if (len && strncmp(sd->chip.name, reg->name, len)) {
		return -EINVAL;
	}
	if (!private_capable(CAP_SYS_ADMIN))
		return -EPERM;
	ret = sensor_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 2;

	return ret;
}

static int sensor_s_register(struct tx_isp_subdev *sd, const struct tx_isp_dbg_register *reg)
{
	int len = 0;

	len = strlen(sd->chip.name);
	if (len && strncmp(sd->chip.name, reg->name, len)) {
		return -EINVAL;
	}
	if (!private_capable(CAP_SYS_ADMIN))
		return -EPERM;

	sensor_write(sd, reg->reg & 0xffff, reg->val & 0xff);

	return 0;
}

static struct tx_isp_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.reset = sensor_reset,
	.init = sensor_init,
	/*.ioctl = sensor_ops_ioctl,*/
	.g_register = sensor_g_register,
	.s_register = sensor_s_register,
};

static struct tx_isp_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_s_stream,
};

static struct tx_isp_subdev_sensor_ops sensor_sensor_ops = {
	.ioctl = sensor_sensor_ops_ioctl,
};

static struct tx_isp_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.sensor = &sensor_sensor_ops,
};

/* It's the sensor device */
static u64 tx_isp_module_dma_mask = ~(u64)0;
struct platform_device sensor_platform_device = {
	.name = SENSOR_NAME,
	.id = -1,
	.dev = {
		.dma_mask = &tx_isp_module_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.num_resources = 0,
};

static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tx_isp_subdev *sd;
	struct tx_isp_video_in *video;
	struct tx_isp_sensor *sensor;

	sensor = (struct tx_isp_sensor *)kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (!sensor) {
		ISP_ERROR("Failed to allocate sensor subdev.\n");
		return -ENOMEM;
	}
	memset(sensor, 0 ,sizeof(*sensor));

	sensor->mclk = clk_get(NULL, "cgu_cim");
	if (IS_ERR(sensor->mclk)) {
		ISP_ERROR("Cannot get sensor input clock cgu_cim\n");
		goto err_get_mclk;
	}
	private_clk_set_rate(sensor->mclk, 24000000);
	private_clk_enable(sensor->mclk);

	/*
	  convert sensor-gain into isp-gain,
	*/
	sd = &sensor->sd;
	video = &sensor->video;
	sensor->video.attr = &sensor_attr;
	sensor->video.vi_max_width = wsize->width;
	sensor->video.vi_max_height = wsize->height;
	sensor->video.mbus.width = wsize->width;
	sensor->video.mbus.height = wsize->height;
	sensor->video.mbus.code = wsize->mbus_code;
	sensor->video.mbus.field = V4L2_FIELD_NONE;
	sensor->video.mbus.colorspace = wsize->colorspace;
	sensor->video.fps = wsize->fps;
	tx_isp_subdev_init(&sensor_platform_device, sd, &sensor_ops);
	tx_isp_set_subdevdata(sd, client);
	tx_isp_set_subdev_hostdata(sd, sensor);
	private_i2c_set_clientdata(client, sd);

	pr_debug("probe ok ------->%s\n", SENSOR_NAME);

	return 0;

err_get_mclk:
	private_clk_disable(sensor->mclk);
	private_clk_put(sensor->mclk);
	kfree(sensor);

	return -1;
}

static int sensor_remove(struct i2c_client *client)
{
	struct tx_isp_subdev *sd = private_i2c_get_clientdata(client);
	struct tx_isp_sensor *sensor = tx_isp_get_subdev_hostdata(sd);

	if (reset_gpio != -1)
		private_gpio_free(reset_gpio);
	if (pwdn_gpio != -1)
		private_gpio_free(pwdn_gpio);

	private_clk_disable(sensor->mclk);
	private_clk_put(sensor->mclk);
	tx_isp_subdev_deinit(sd);
	kfree(sensor);

	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static __init int init_sensor(void)
{
	int ret = 0;
	ret = private_driver_get_interface();
	if (ret) {
		ISP_ERROR("Failed to init %s driver.\n", SENSOR_NAME);
		return -1;
	}
	return private_i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	private_i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);

MODULE_DESCRIPTION("A low-level driver for "SENSOR_NAME" sensor");
MODULE_LICENSE("GPL");
