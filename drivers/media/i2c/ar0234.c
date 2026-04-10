// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the onsemi AR0234 camera sensor
 *
 * Copyright (C) 2026 Alexander Shiyan <eagle.alexander923@gmail.com>
 *
 * Some parts of code taken from Raspberry Pi driver ar0234.c by:
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd
 * Copyright (C) 2025-2026, UAB Kurokesu
 * Author: Dave Stevenson <dave.stevenson@raspberrypi.com>
 * Author: Danius Kalvaitis <danius@kurokesu.com>
 *
 * Some parts of code taken from imx290.c by:
 * Copyright (C) 2019 FRAMOS GmbH.
 * Copyright (C) 2019 Linaro Ltd.
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "ccs-pll.h"

#define AR0234_NATIVE_WIDTH				(1940U)
#define AR0234_NATIVE_HEIGHT				(1220U)
#define AR0234_PIXEL_ARRAY_LEFT				(8U)
#define AR0234_PIXEL_ARRAY_TOP				(8U)
#define AR0234_PIXEL_ARRAY_WIDTH			(1920U)
#define AR0234_PIXEL_ARRAY_HEIGHT			(1200U)
#define AR0234_MIN_CROP_WIDTH				(4U)
#define AR0234_MIN_CROP_HEIGHT				(2U)
#define AR0234_CROP_WIDTH_STEP				(4U)
#define AR0234_CROP_HEIGHT_STEP				(2U)

#define AR0234_REG_CHIP_VERSION				CCI_REG16(0x3000)
#	define AR0234_CHIP_ID				(0x0a56)
#	define AR0234_CHIP_ID_MONO			(0x1a56)
#define AR0234_REG_Y_ADDR_START				CCI_REG16(0x3002)
#define AR0234_REG_X_ADDR_START				CCI_REG16(0x3004)
#define AR0234_REG_Y_ADDR_END				CCI_REG16(0x3006)
#define AR0234_REG_X_ADDR_END				CCI_REG16(0x3008)
#define AR0234_REG_FRAME_LENGTH_LINES			CCI_REG16(0x300a)
#	define AR0234_FRAME_LENGTH_LINES_MIN		(16)
#	define AR0234_VBLANK_MAX			(0xf000)
#define AR0234_REG_LINE_LENGTH_PCK			CCI_REG16(0x300c)
#	define AR0234_LINE_LENGTH_PCK_MIN		(612)
#	define AR0234_HBLANK_MIN			\
		(AR0234_LINE_LENGTH_PCK_MIN - AR0234_PIXEL_ARRAY_WIDTH / 4)
#	define AR0234_HBLANK_MAX			(0xf000)
#define AR0234_REG_REVISION_NUMBER			CCI_REG16(0x300e)
#define AR0234_REG_COARSE_INTEGRATION_TIME		CCI_REG16(0x3012)
#	define AR0234_EXPOSURE_MIN			(2)
#	define AR0234_EXPOSURE_STEP			(1)
#define AR0234_REG_FINE_INTEGRATION_TIME		CCI_REG16(0x3014)
#define AR0234_REG_RESET				CCI_REG16(0x301a)
#define AR0234_REG_MODE_SELECT				CCI_REG8(0x301c)
#define AR0234_REG_IMAGE_ORIENTATION			CCI_REG8(0x301d)
#define AR0234_REG_DATA_PEDESTAL			CCI_REG16(0x301e)
#	define AR0234_DATA_PEDESTAL_MIN			(0x0000)
#	define AR0234_DATA_PEDESTAL_MAX			(0x03ff)
#	define AR0234_DATA_PEDESTAL_DEFAULT		(0x002a)
#define AR0234_REG_GROUPED_PARAMETER_HOLD		CCI_REG8(0x3022)
#define AR0234_REG_VT_PIX_CLK_DIV			CCI_REG16(0x302a)
#define AR0234_REG_VT_SYS_CLK_DIV			CCI_REG16(0x302c)
#define AR0234_REG_PRE_PLL_CLK_DIV			CCI_REG16(0x302e)
#define AR0234_REG_PLL_MULTIPLIER			CCI_REG16(0x3030)
#define AR0234_REG_OP_PIX_CLK_DIV			CCI_REG16(0x3036)
#define AR0234_REG_OP_SYS_CLK_DIV			CCI_REG16(0x3038)
#define AR0234_REG_BLUE_GAIN				CCI_REG16(0x3058)
#define AR0234_REG_RED_GAIN				CCI_REG16(0x305a)
#define AR0234_REG_GLOBAL_GAIN				CCI_REG16(0x305e)
#	define AR0234_GAIN_MIN				(0x0080)
#	define AR0234_GAIN_MAX				(0x07ff)
#	define AR0234_GAIN_DEFAULT			(0x0080)
#define AR0234_REG_ANALOG_GAIN				CCI_REG16(0x3060)
#	define AR0234_ANA_GAIN_BASE			(64)
#	define AR0234_ANA_GAIN_MIN			(AR0234_ANA_GAIN_BASE)
#	define AR0234_ANA_GAIN_MAX			(16 * AR0234_ANA_GAIN_BASE)
#	define AR0234_ANA_GAIN_DEFAULT			(AR0234_ANA_GAIN_BASE)
#define AR0234_REG_TEST_PATTERN_MODE			CCI_REG16(0x3070)
#	define AR0234_TEST_PATTERN_DISABLED		(0)
#	define AR0234_TEST_PATTERN_SOLID_COLOR		(1)
#	define AR0234_TEST_PATTERN_VERTICAL_COLOR_BARS	(2)
#	define AR0234_TEST_PATTERN_FADE_TO_GREY		(3)
#	define AR0234_TEST_PATTERN_WALKING_1S		(256)
#define AR0234_REG_TEST_DATA_RED			CCI_REG16(0x3072)
#define AR0234_REG_TEST_DATA_GREENR			CCI_REG16(0x3074)
#define AR0234_REG_TEST_DATA_BLUE			CCI_REG16(0x3076)
#define AR0234_REG_TEST_DATA_GREENB			CCI_REG16(0x3078)
#	define AR0234_TESTP_COLOUR_MIN			(0)
#	define AR0234_TESTP_COLOUR_MAX			(0x3ff)
#	define AR0234_TESTP_COLOUR_STEP			(1)
#define AR0234_REG_MFR_30BA				CCI_REG16(0x30ba)
#	define AR0234_MFR_30BA_GAIN_BITS(x)		(0x7620 | (x))
#define AR0234_REG_DATA_FORMAT_BITS			CCI_REG16(0x31ac)
#	define DATA_FORMAT_BITS(x, y)			(((x) << 8) | (y))
#define AR0234_REG_SERIAL_FORMAT			CCI_REG16(0x31ae)
#	define DATA_FORMAT_LANES(x)			(0x200 | (x))
#define AR0234_REG_COMPANDING				CCI_REG16(0x31d0)
#	define COMPANDING_DPCM_EN			BIT(0)
#define AR0234_REG_MIPI_CNTRL				CCI_REG16(0x3354)

static const struct cci_reg_sequence ar0234_common_init[] = {
	{ AR0234_REG_FINE_INTEGRATION_TIME, 0 },
};

static const char *const ar0234_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"Vertical Color Bars",
	"Fade to Grey Vertical Color Bars",
	"Walking 1s",
};

static const unsigned int ar0234_test_pattern_val[] = {
	AR0234_TEST_PATTERN_DISABLED,
	AR0234_TEST_PATTERN_SOLID_COLOR,
	AR0234_TEST_PATTERN_VERTICAL_COLOR_BARS,
	AR0234_TEST_PATTERN_FADE_TO_GREY,
	AR0234_TEST_PATTERN_WALKING_1S,
};

static const char *const ar0234_supply_names[] = {
	"vaa",
	"vdd",
	"vddio",
};

enum ar0234_colour_variant {
	AR0234_VARIANT_MONO,
	AR0234_VARIANT_COLOUR,
	AR0234_VARIANT_MAX
};

enum ar0234_link_freq_index {
	AR0234_LINK_FREQ_IDX_BPP_8,
	AR0234_LINK_FREQ_IDX_BPP_10,
	AR0234_LINK_FREQ_IDX_MAX
};

struct ar0234_mode {
	u8 bpp_in;
	u8 bpp_out;
	u8 dpcm;
	u8 mipi_dt;
	int link_freq_index;
	u32 code[AR0234_VARIANT_MAX];
};

static const struct ar0234_mode ar0234_modes[] = {
	{
		.bpp_in = 8,
		.bpp_out = 8,
		.dpcm = 0,
		.mipi_dt = MIPI_CSI2_DT_RAW8,
		.link_freq_index = AR0234_LINK_FREQ_IDX_BPP_8,
		.code = {
			[AR0234_VARIANT_MONO] = MEDIA_BUS_FMT_Y8_1X8,
			[AR0234_VARIANT_COLOUR] = MEDIA_BUS_FMT_SGRBG8_1X8,
		},
	},
	{
		.bpp_in = 10,
		.bpp_out = 10,
		.dpcm = 0,
		.mipi_dt = MIPI_CSI2_DT_RAW10,
		.link_freq_index = AR0234_LINK_FREQ_IDX_BPP_10,
		.code = {
			[AR0234_VARIANT_MONO] = MEDIA_BUS_FMT_Y10_1X10,
			[AR0234_VARIANT_COLOUR] = MEDIA_BUS_FMT_SGRBG10_1X10,
		},
	},
	{
		.bpp_in = 10,
		.bpp_out = 8,
		.dpcm = COMPANDING_DPCM_EN,
		.mipi_dt = MIPI_CSI2_DT_RAW8,
		.link_freq_index = AR0234_LINK_FREQ_IDX_BPP_8,
		.code = {
			[AR0234_VARIANT_COLOUR] =
				MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8,
		},
	},
};

struct ar0234 {
	struct clk *clk;
	struct regmap *regmap;

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regulator_bulk_data supplies[ARRAY_SIZE(ar0234_supply_names)];
	struct gpio_desc *reset;

	unsigned int num_data_lanes;

	s64 link_freqs[AR0234_LINK_FREQ_IDX_MAX];

	enum ar0234_colour_variant variant;

	struct ccs_pll pll;

	struct ar0234_mode const *mode;

	struct v4l2_ctrl_handler ctrls;

	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct {
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};
};

static inline struct ar0234 *to_ar0234(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct ar0234, sd);
}

static const struct ccs_pll_limits ar0234_pll_limits = {
	.min_ext_clk_freq_hz = 6000000,
	.max_ext_clk_freq_hz = 54000000,
	.vt_fr = {
		.min_pre_pll_clk_div = 1,
		.max_pre_pll_clk_div = 63,
		.min_pll_ip_clk_freq_hz = 6000000,
		.max_pll_ip_clk_freq_hz = 12000000,
		.min_pll_multiplier = 2,
		.max_pll_multiplier = 254,
		.min_pll_op_clk_freq_hz = 384000000,
		.max_pll_op_clk_freq_hz = 768000000,
	},
	.vt_bk = {
		.min_sys_clk_div = 1,
		.max_sys_clk_div = 31,
		.min_sys_clk_freq_hz = 6000000,
		.max_sys_clk_freq_hz = 768000000,
		.min_pix_clk_div = 1,
		.max_pix_clk_div = 31,
		.min_pix_clk_freq_hz = 6000000,
		.max_pix_clk_freq_hz = 90000000,
	},
	.op_bk = {
		.min_sys_clk_div = 1,
		.max_sys_clk_div = 31,
		.min_sys_clk_freq_hz = 6000000,
		.max_sys_clk_freq_hz = 768000000,
		.min_pix_clk_div = 1,
		.max_pix_clk_div = 31,
		.min_pix_clk_freq_hz = 6000000,
		.max_pix_clk_freq_hz = 90000000,
	},
};

static int ar0234_calculate_pll(struct ar0234 *ar0234,
				const struct ar0234_mode *mode)
{
	struct ccs_pll pll = { 0 };
	int ret;

	pll.bus_type = CCS_PLL_BUS_TYPE_CSI2_DPHY;
	pll.op_lanes = ar0234->num_data_lanes;
	pll.vt_lanes = 1;
	pll.csi2.lanes = ar0234->num_data_lanes;
	pll.binning_horizontal = 1;
	pll.binning_vertical = 1;
	pll.scale_m = 1;
	pll.scale_n = 1;
	pll.bits_per_pixel = mode->bpp_out;
	pll.flags = CCS_PLL_FLAG_LANE_SPEED_MODEL |
		    CCS_PLL_FLAG_EVEN_PLL_MULTIPLIER |
		    CCS_PLL_FLAG_FIFO_DERATING |
		    CCS_PLL_FLAG_FIFO_OVERRATING |
		    CCS_PLL_FLAG_EXT_IP_PLL_DIVIDER;
	pll.link_freq = ar0234->link_freqs[mode->link_freq_index] / 2;
	pll.ext_clk_freq_hz = clk_get_rate(ar0234->clk);

	ret = ccs_pll_calculate(ar0234->sd.dev, &ar0234_pll_limits, &pll);
	if (!ret)
		ar0234->pll = pll;

	return ret;
}

static u32 ar0234_calc_analog_gain(u32 req_gain_q6)
{
	u32 s, t, best = 0, min_diff = U32_MAX;
	u32 coarse_mult, fine_gain_q6, total_gain_q6, diff;

	for (s = 0; s <= 4; s++) {
		coarse_mult = (1 << s) * AR0234_ANA_GAIN_BASE;

		for (t = 0; t <= 15; t++) {
			if (s == 0 || s == 2) {
				fine_gain_q6 =
					(AR0234_ANA_GAIN_BASE * 32) / (32 - t);
			} else if (s == 1 || s == 3) {
				fine_gain_q6 = (AR0234_ANA_GAIN_BASE * 16) /
					       (16 - (t / 2));
			} else {
				fine_gain_q6 = (AR0234_ANA_GAIN_BASE * 8) /
					       (8 - (t / 4));
			}

			total_gain_q6 = (coarse_mult * fine_gain_q6) /
					AR0234_ANA_GAIN_BASE;

			if (req_gain_q6 > total_gain_q6)
				diff = req_gain_q6 - total_gain_q6;
			else
				diff = total_gain_q6 - req_gain_q6;

			if (diff < min_diff) {
				min_diff = diff;
				best = (s << 4) | t;
			}
		}
	}

	return best;
}

static int ar0234_set_mfr_30ba(struct ar0234 *ar0234, u32 analog_reg_val)
{
	u16 mfr_30ba_val;
	u32 coarse_idx = (analog_reg_val >> 4) & 0x7;

	if (ar0234->pll.pixel_rate_pixel_array <= 45000000) {
		if (coarse_idx < 3)
			mfr_30ba_val = AR0234_MFR_30BA_GAIN_BITS(6);
		else
			mfr_30ba_val = AR0234_MFR_30BA_GAIN_BITS(0);
	} else {
		if (coarse_idx == 0)
			mfr_30ba_val = AR0234_MFR_30BA_GAIN_BITS(2);
		else if (coarse_idx == 1)
			mfr_30ba_val = AR0234_MFR_30BA_GAIN_BITS(1);
		else
			mfr_30ba_val = AR0234_MFR_30BA_GAIN_BITS(0);
	}

	return cci_write(ar0234->regmap, AR0234_REG_MFR_30BA,
			 mfr_30ba_val, NULL);
}

static int ar0234_set_analog_gain(struct ar0234 *ar0234, u32 val)
{
	u32 reg_val;
	int ret;

	reg_val = ar0234_calc_analog_gain(val);

	ret = cci_write(ar0234->regmap, AR0234_REG_GROUPED_PARAMETER_HOLD,
			1, NULL);
	if (ret)
		return ret;

	ret = ar0234_set_mfr_30ba(ar0234, reg_val);

	cci_write(ar0234->regmap, AR0234_REG_ANALOG_GAIN, reg_val, &ret);

	cci_write(ar0234->regmap, AR0234_REG_GROUPED_PARAMETER_HOLD, 0, NULL);

	return ret;
}

static int ar0234_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0234 *ar0234 = container_of(ctrl->handler,
					     struct ar0234, ctrls);
	struct v4l2_subdev *sd = &ar0234->sd;
	struct v4l2_subdev_state *state;
	struct v4l2_rect *crop;
	int ret = 0;

	if (ctrl->flags & V4L2_CTRL_FLAG_READ_ONLY)
		return 0;

	state = v4l2_subdev_get_locked_active_state(sd);
	crop = v4l2_subdev_state_get_crop(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max = crop->height + ctrl->val - 1;
		int exposure_val = clamp(ar0234->exposure->val,
					 AR0234_EXPOSURE_MIN, exposure_max);

		ret = __v4l2_ctrl_modify_range(ar0234->exposure,
					       AR0234_EXPOSURE_MIN,
					       exposure_max,
					       AR0234_EXPOSURE_STEP,
					       exposure_val);
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_in_use(sd->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HBLANK:
		cci_write(ar0234->regmap, AR0234_REG_LINE_LENGTH_PCK,
			  (crop->width / 4) + ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(ar0234->regmap, AR0234_REG_FRAME_LENGTH_LINES,
			  crop->height + ctrl->val, &ret);
		if (ret)
			break;
		ctrl = ar0234->exposure;
		fallthrough;
	case V4L2_CID_EXPOSURE:
		cci_write(ar0234->regmap, AR0234_REG_COARSE_INTEGRATION_TIME,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ar0234_set_analog_gain(ar0234, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(ar0234->regmap, AR0234_REG_IMAGE_ORIENTATION,
			  (ar0234->vflip->val << 1) | ar0234->hflip->val, &ret);
		break;
	case V4L2_CID_BLUE_BALANCE:
		cci_write(ar0234->regmap, AR0234_REG_BLUE_GAIN,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_RED_BALANCE:
		cci_write(ar0234->regmap, AR0234_REG_RED_GAIN,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(ar0234->regmap, AR0234_REG_GLOBAL_GAIN,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_BRIGHTNESS:
		cci_write(ar0234->regmap, AR0234_REG_DATA_PEDESTAL,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(ar0234->regmap, AR0234_REG_TEST_PATTERN_MODE,
			  ar0234_test_pattern_val[ctrl->val], &ret);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		cci_write(ar0234->regmap, AR0234_REG_TEST_DATA_RED,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		cci_write(ar0234->regmap, AR0234_REG_TEST_DATA_GREENR,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		cci_write(ar0234->regmap, AR0234_REG_TEST_DATA_BLUE,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		cci_write(ar0234->regmap, AR0234_REG_TEST_DATA_GREENB,
			  ctrl->val, &ret);
		break;
	default:
		dev_err(sd->dev, "Invalid control %d\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(sd->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0234_ctrl_ops = {
	.s_ctrl = ar0234_set_ctrl,
};

static int ar0234_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	if (code->index >= ARRAY_SIZE(ar0234_modes))
		return -EINVAL;

	if (!ar0234_modes[code->index].code[ar0234->variant])
		return -EINVAL;

	code->code = ar0234_modes[code->index].code[ar0234->variant];

	return 0;
}

static int ar0234_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	fse->min_width = AR0234_PIXEL_ARRAY_WIDTH;
	fse->max_width = fse->min_width;
	fse->min_height = AR0234_PIXEL_ARRAY_HEIGHT;
	fse->max_height = fse->min_height;

	return 0;
}

static void ar0234_set_link_limits(struct ar0234 *ar0234)
{
	u64 pixel_rate = ar0234->link_freqs[ar0234->mode->link_freq_index] * 2;

	pixel_rate *= ar0234->num_data_lanes;
	do_div(pixel_rate, ar0234->mode->bpp_out);

	__v4l2_ctrl_s_ctrl_int64(ar0234->pixel_rate, pixel_rate);

	__v4l2_ctrl_s_ctrl(ar0234->link_freq, ar0234->mode->link_freq_index);
}

static void ar0234_set_framing_limits(struct ar0234 *ar0234, u32 width)
{
	int hblank;

	__v4l2_ctrl_s_ctrl(ar0234->vblank, AR0234_FRAME_LENGTH_LINES_MIN);

	hblank = AR0234_LINE_LENGTH_PCK_MIN - width / 4;
	__v4l2_ctrl_modify_range(ar0234->hblank, AR0234_HBLANK_MIN,
				 AR0234_HBLANK_MAX, 2, hblank);
	__v4l2_ctrl_s_ctrl(ar0234->hblank, hblank);
}

static int ar0234_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct ar0234_mode const *mode = &ar0234_modes[0];
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct v4l2_rect *crop;
	int i;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	for (i = 0; i < ARRAY_SIZE(ar0234_modes); i++) {
		if (!ar0234_modes[i].code[ar0234->variant])
			continue;
		if (ar0234_modes[i].code[ar0234->variant] == fmt->format.code) {
			mode = &ar0234_modes[i];
			break;
		}
	}

	crop = v4l2_subdev_state_get_crop(state, fmt->pad);

	fmt->format.width = crop->width;
	fmt->format.height = crop->height;
	fmt->format.code = mode->code[ar0234->variant];
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;

	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;

	if (ar0234->mode != mode && fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		int ret = ar0234_calculate_pll(ar0234, mode);

		if (ret) {
			dev_err(sd->dev, "PLL recalculation failed: %d\n", ret);
			return -EINVAL;
		}

		ar0234->mode = mode;

		ar0234_set_link_limits(ar0234);
		ar0234_set_framing_limits(ar0234, crop->width);
	}

	return 0;
}

static int ar0234_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, sel->pad);
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = AR0234_PIXEL_ARRAY_LEFT;
		sel->r.top = AR0234_PIXEL_ARRAY_TOP;
		sel->r.width = AR0234_PIXEL_ARRAY_WIDTH;
		sel->r.height = AR0234_PIXEL_ARRAY_HEIGHT;
		break;
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = AR0234_NATIVE_WIDTH;
		sel->r.height = AR0234_NATIVE_HEIGHT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ar0234_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct v4l2_rect *crop;
	struct v4l2_rect rect;

	if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
		v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	/* Align the requested rectangle to the sensor's cropping granularity */
	rect.left = round_up(sel->r.left, AR0234_CROP_WIDTH_STEP);
	rect.top = round_up(sel->r.top, AR0234_CROP_HEIGHT_STEP);
	rect.width = round_down(sel->r.width, AR0234_CROP_WIDTH_STEP);
	rect.height = round_down(sel->r.height, AR0234_CROP_HEIGHT_STEP);

	/* Clamp the top-left corner so that the whole rectangle stays */
	/* inside the active pixel array */
	rect.left =
		clamp_t(u32, rect.left, AR0234_PIXEL_ARRAY_LEFT,
			AR0234_PIXEL_ARRAY_LEFT + AR0234_PIXEL_ARRAY_WIDTH -
			rect.width);
	rect.top =
		clamp_t(u32, rect.top, AR0234_PIXEL_ARRAY_TOP,
			AR0234_PIXEL_ARRAY_TOP + AR0234_PIXEL_ARRAY_HEIGHT -
			rect.height);

	/* Clamp width and height to the allowed minima and to the */
	/* remaining space after fixing the top-left corner */
	rect.width =
		clamp_t(u32, rect.width, AR0234_MIN_CROP_WIDTH,
			AR0234_PIXEL_ARRAY_LEFT + AR0234_PIXEL_ARRAY_WIDTH -
			rect.left);
	rect.height =
		clamp_t(u32, rect.height, AR0234_MIN_CROP_HEIGHT,
			AR0234_PIXEL_ARRAY_TOP + AR0234_PIXEL_ARRAY_HEIGHT -
			rect.top);

	crop = v4l2_subdev_state_get_crop(state, sel->pad);

	if (rect.width != crop->width || rect.height != crop->height) {
		struct v4l2_mbus_framefmt *format;

		format = v4l2_subdev_state_get_format(state, sel->pad);
		format->width = rect.width;
		format->height = rect.height;
	}

	*crop = rect;
	sel->r = rect;

	if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ar0234_set_framing_limits(ar0234, crop->width);

	return 0;
}

static int ar0234_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_selection sel = {
		.target = V4L2_SEL_TGT_CROP,
		.r.left = AR0234_PIXEL_ARRAY_LEFT,
		.r.top = AR0234_PIXEL_ARRAY_TOP,
		.r.width = AR0234_PIXEL_ARRAY_WIDTH,
		.r.height = AR0234_PIXEL_ARRAY_HEIGHT,
	};
	struct v4l2_subdev_format format = {
		.format = {
			.width = AR0234_PIXEL_ARRAY_WIDTH,
			.height = AR0234_PIXEL_ARRAY_HEIGHT,
		},
	};

	ar0234_set_selection(sd, state, &sel);
	ar0234_set_pad_format(sd, state, &format);

	return 0;
}

static int ar0234_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct v4l2_rect *crop = v4l2_subdev_state_get_crop(state, pad);
	int x_addr_start, x_addr_end, y_addr_start, y_addr_end, ret;

	if (streams_mask != 1)
		return -EINVAL;

	ret = pm_runtime_resume_and_get(sd->dev);
	if (ret)
		return ret;

	cci_write(ar0234->regmap, AR0234_REG_PRE_PLL_CLK_DIV,
		  ar0234->pll.vt_fr.pre_pll_clk_div, &ret);
	cci_write(ar0234->regmap, AR0234_REG_PLL_MULTIPLIER,
		  ar0234->pll.vt_fr.pll_multiplier, &ret);
	cci_write(ar0234->regmap, AR0234_REG_VT_SYS_CLK_DIV,
		  ar0234->pll.vt_bk.sys_clk_div, &ret);
	cci_write(ar0234->regmap, AR0234_REG_VT_PIX_CLK_DIV,
		  ar0234->pll.vt_bk.pix_clk_div, &ret);
	cci_write(ar0234->regmap, AR0234_REG_OP_SYS_CLK_DIV,
		  ar0234->pll.op_bk.sys_clk_div, &ret);
	cci_write(ar0234->regmap, AR0234_REG_OP_PIX_CLK_DIV,
		  ar0234->pll.op_bk.pix_clk_div, &ret);

	cci_multi_reg_write(ar0234->regmap, ar0234_common_init,
			    ARRAY_SIZE(ar0234_common_init), &ret);

	cci_write(ar0234->regmap, AR0234_REG_COMPANDING,
		  ar0234->mode->dpcm, &ret);

	cci_write(ar0234->regmap, AR0234_REG_DATA_FORMAT_BITS,
		  DATA_FORMAT_BITS(ar0234->mode->bpp_in, ar0234->mode->bpp_out),
		  &ret);

	cci_write(ar0234->regmap, AR0234_REG_SERIAL_FORMAT,
		  DATA_FORMAT_LANES(ar0234->num_data_lanes), &ret);

	cci_write(ar0234->regmap, AR0234_REG_MIPI_CNTRL,
		  ar0234->mode->mipi_dt, &ret);

	x_addr_start = crop->left;
	y_addr_start = crop->top;
	x_addr_end = crop->left + crop->width - 1;
	y_addr_end = crop->top + crop->height - 1;

	cci_write(ar0234->regmap, AR0234_REG_X_ADDR_START, x_addr_start, &ret);
	cci_write(ar0234->regmap, AR0234_REG_Y_ADDR_START, y_addr_start, &ret);
	cci_write(ar0234->regmap, AR0234_REG_X_ADDR_END, x_addr_end, &ret);
	cci_write(ar0234->regmap, AR0234_REG_Y_ADDR_END, y_addr_end, &ret);

	if (ret)
		goto start_err;

	ret = __v4l2_ctrl_handler_setup(ar0234->sd.ctrl_handler);

	cci_write(ar0234->regmap, AR0234_REG_MODE_SELECT, 1, &ret);
	if (!ret)
		return 0;

start_err:
	pm_runtime_put_autosuspend(sd->dev);

	dev_err(sd->dev, "Failed to setup sensor\n");

	return ret;
}

static int ar0234_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	int ret;

	if (streams_mask != 1)
		return -EINVAL;

	ret = cci_write(ar0234->regmap, AR0234_REG_MODE_SELECT, 0, NULL);

	pm_runtime_put_autosuspend(sd->dev);

	return ret;
}

static int ar0234_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->bus.mipi_csi2.flags = 0;
	config->bus.mipi_csi2.num_data_lanes = ar0234->num_data_lanes;

	return 0;
}

static const struct v4l2_subdev_video_ops ar0234_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops ar0234_pad_ops = {
	.enum_mbus_code = ar0234_enum_mbus_code,
	.enum_frame_size = ar0234_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ar0234_set_pad_format,
	.get_selection = ar0234_get_selection,
	.set_selection = ar0234_set_selection,
	.enable_streams = ar0234_enable_streams,
	.disable_streams = ar0234_disable_streams,
	.get_mbus_config = ar0234_g_mbus_config,
};

static const struct v4l2_subdev_ops ar0234_subdev_ops = {
	.video = &ar0234_video_ops,
	.pad = &ar0234_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0234_internal_ops = {
	.init_state = ar0234_init_state,
};

static const struct media_entity_operations ar0234_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ar0234_ctrls_init(struct ar0234 *ar0234)
{
	struct v4l2_fwnode_device_properties props;
	int i, ret;

	ret = v4l2_ctrl_handler_init(&ar0234->ctrls, 17 + 2);
	if (ret)
		return ret;

	ar0234->pixel_rate = v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, 1,
					       INT_MAX, 1, 1);

	ar0234->link_freq =
		v4l2_ctrl_new_int_menu(&ar0234->ctrls, &ar0234_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       AR0234_LINK_FREQ_IDX_MAX - 1, 0,
				       ar0234->link_freqs);
	if (ar0234->link_freq)
		ar0234->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ar0234->hblank = v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
					   V4L2_CID_HBLANK, AR0234_HBLANK_MIN,
					   AR0234_HBLANK_MAX, 2,
					   AR0234_HBLANK_MIN);

	ar0234->vblank = v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
					   V4L2_CID_VBLANK,
					   AR0234_FRAME_LENGTH_LINES_MIN,
					   AR0234_VBLANK_MAX, 1,
					   AR0234_FRAME_LENGTH_LINES_MIN);

	ar0234->exposure = v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     AR0234_EXPOSURE_MIN, U16_MAX,
					     AR0234_EXPOSURE_STEP, 200);

	v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, AR0234_ANA_GAIN_MIN,
			  AR0234_ANA_GAIN_MAX, 1, AR0234_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops, V4L2_CID_BRIGHTNESS,
			  AR0234_DATA_PEDESTAL_MIN, AR0234_DATA_PEDESTAL_MAX, 1,
			  AR0234_DATA_PEDESTAL_DEFAULT);

	ar0234->hflip = v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	ar0234->vflip = v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_cluster(2, &ar0234->hflip);

	v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
			  V4L2_CID_BLUE_BALANCE, AR0234_GAIN_MIN,
			  AR0234_GAIN_MAX, 1, AR0234_GAIN_DEFAULT);

	v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
			  V4L2_CID_RED_BALANCE, AR0234_GAIN_MIN,
		   AR0234_GAIN_MAX, 1, AR0234_GAIN_DEFAULT);

	v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
			  V4L2_CID_DIGITAL_GAIN, AR0234_GAIN_MIN,
		   AR0234_GAIN_MAX, 1, AR0234_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(&ar0234->ctrls, &ar0234_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ar0234_test_pattern_menu) - 1,
				     0, 0, ar0234_test_pattern_menu);

	for (i = 0; i < 4; i++) {
		v4l2_ctrl_new_std(&ar0234->ctrls, &ar0234_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  AR0234_TESTP_COLOUR_MIN,
				  AR0234_TESTP_COLOUR_MAX,
				  AR0234_TESTP_COLOUR_STEP,
				  AR0234_TESTP_COLOUR_MAX);
	}

	ret = v4l2_fwnode_device_parse(ar0234->sd.dev, &props);
	if (ret)
		return ret;

	ret = v4l2_ctrl_new_fwnode_properties(&ar0234->ctrls, &ar0234_ctrl_ops,
					      &props);
	if (ret)
		return ret;

	ar0234->sd.ctrl_handler = &ar0234->ctrls;

	ar0234_set_link_limits(ar0234);
	ar0234_set_framing_limits(ar0234, AR0234_PIXEL_ARRAY_WIDTH);

	return 0;
}

static int ar0234_parse_hw_config(struct ar0234 *ar0234)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ar0234->supplies); i++)
		ar0234->supplies[i].supply = ar0234_supply_names[i];

	ret = devm_regulator_bulk_get(ar0234->sd.dev,
				      ARRAY_SIZE(ar0234->supplies),
				      ar0234->supplies);
	if (ret)
		return dev_err_probe(ar0234->sd.dev, ret,
				     "Failed to get supplies\n");

	ar0234->reset = devm_gpiod_get_optional(ar0234->sd.dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(ar0234->reset))
		return dev_err_probe(ar0234->sd.dev, PTR_ERR(ar0234->reset),
				     "Failed to get reset GPIO\n");

	ar0234->clk = devm_v4l2_sensor_clk_get(ar0234->sd.dev, NULL);
	if (IS_ERR(ar0234->clk))
		return dev_err_probe(ar0234->sd.dev, PTR_ERR(ar0234->clk),
				     "Failed to get clock\n");

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(ar0234->sd.dev), NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	switch (bus_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		ar0234->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		ret = dev_err_probe(ar0234->sd.dev, -EINVAL,
				    "Invalid number of CSI2 data lanes %d\n",
		      bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto done_endpoint_free;
	}

	if (bus_cfg.nr_of_link_frequencies != AR0234_LINK_FREQ_IDX_MAX) {
		ret = dev_err_probe(ar0234->sd.dev, -EINVAL,
				    "Invalid number of link freq items %d\n",
				    bus_cfg.nr_of_link_frequencies);
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++) {
		s64 freq = bus_cfg.link_frequencies[i];

		if (freq < 360000000LL || freq > 450000000LL) {
			ret = dev_err_probe(ar0234->sd.dev, -EINVAL,
					    "Invalid link frequency %lli\n",
					    freq);
			goto done_endpoint_free;
		}

		ar0234->link_freqs[i] = freq;
	}

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int ar0234_identify_module(struct ar0234 *ar0234)
{
	u64 id, rev;
	int ret;

	ret = cci_read(ar0234->regmap, AR0234_REG_CHIP_VERSION, &id, NULL);
	ret = cci_read(ar0234->regmap, AR0234_REG_REVISION_NUMBER, &rev, &ret);
	if (ret)
		return dev_err_probe(ar0234->sd.dev, ret,
				     "Failed to read chip id\n");

	if (id == AR0234_CHIP_ID_MONO)
		ar0234->variant = AR0234_VARIANT_MONO;
	else if (id == AR0234_CHIP_ID)
		ar0234->variant = AR0234_VARIANT_COLOUR;
	else
		return dev_err_probe(ar0234->sd.dev, -ENODEV,
				     "Invalid chip id: 0x%04x\n", (u16)id);

	dev_info(ar0234->sd.dev, "Success reading chip id: 0x%04x, Rev.%lld\n",
		 (u16)id, (rev >> 12) & 0xf);

	return ret;
}

static int ar0234_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0234 *ar0234 = to_ar0234(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ar0234->supplies),
				    ar0234->supplies);
	if (ret) {
		dev_err(dev, "Failed to enable regulators\n");
		return ret;
	}

	ret = clk_prepare_enable(ar0234->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		regulator_bulk_disable(ARRAY_SIZE(ar0234->supplies),
				       ar0234->supplies);
		return ret;
	}

	gpiod_set_value_cansleep(ar0234->reset, 0);

	/* ~160000 EXTCLKs */
	fsleep(DIV_ROUND_UP(160000ULL * 1000000, clk_get_rate(ar0234->clk)));

	return 0;
}

static int ar0234_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0234 *ar0234 = to_ar0234(sd);

	gpiod_set_value_cansleep(ar0234->reset, 1);

	regulator_bulk_disable(ARRAY_SIZE(ar0234->supplies), ar0234->supplies);

	clk_disable_unprepare(ar0234->clk);

	/* 100ms PwrDown until next PwrUp */
	fsleep(100000);

	return 0;
}

static void ar0234_subdev_cleanup(struct ar0234 *ar0234)
{
	media_entity_cleanup(&ar0234->sd.entity);
	v4l2_ctrl_handler_free(&ar0234->ctrls);
}

static int ar0234_soft_reset(struct ar0234 *ar0234)
{
	int ret;

	ret = cci_write(ar0234->regmap, AR0234_REG_RESET, 0x0001, NULL);
	fsleep(2000);
	cci_write(ar0234->regmap, AR0234_REG_RESET, 0x2018, &ret);
	fsleep(2000);

	return ret;
}

static int ar0234_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ar0234 *ar0234;
	int ret;

	ar0234 = devm_kzalloc(dev, sizeof(*ar0234), GFP_KERNEL);
	if (!ar0234)
		return -ENOMEM;

	ar0234->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ar0234->regmap))
		return PTR_ERR(ar0234->regmap);

	v4l2_i2c_subdev_init(&ar0234->sd, client, &ar0234_subdev_ops);

	ret = ar0234_parse_hw_config(ar0234);
	if (ret)
		goto error_subdev;

	ret = ar0234_power_on(dev);
	if (ret)
		goto error_subdev;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = ar0234_soft_reset(ar0234);
	if (ret)
		goto error_pm;

	ret = ar0234_identify_module(ar0234);
	if (ret)
		goto error_pm;

	ar0234->mode = &ar0234_modes[0];

	ret = ar0234_calculate_pll(ar0234, ar0234->mode);
	if (ret) {
		dev_err_probe(dev, ret, "PLL calculations failed\n");
		goto error_pm;
	}

	ar0234->sd.internal_ops = &ar0234_internal_ops;
	ar0234->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ar0234->sd.entity.ops = &ar0234_subdev_entity_ops;
	ar0234->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ar0234->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ar0234->sd.entity, 1, &ar0234->pad);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to init entity pads\n");
		goto error_pm;
	}

	ret = v4l2_subdev_init_finalize(&ar0234->sd);
	if (ret) {
		dev_err_probe(dev, ret, "Subdev init error\n");
		goto error_media;
	}

	ret = ar0234_ctrls_init(ar0234);
	if (ret)
		goto error_media;

	ar0234->sd.state_lock = ar0234->ctrls.lock;

	ret = v4l2_async_register_subdev_sensor(&ar0234->sd);
	if (ret) {
		dev_err_probe(dev, ret,
			      "Failed to register sensor sub-device\n");
		goto error_media;
	}

	pm_runtime_put_autosuspend(dev);

	return 0;

error_media:
	media_entity_cleanup(&ar0234->sd.entity);

error_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	ar0234_power_off(dev);

error_subdev:
	ar0234_subdev_cleanup(ar0234);

	return ret;
}

static void ar0234_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	v4l2_async_unregister_subdev(sd);
	ar0234_subdev_cleanup(ar0234);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ar0234_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct acpi_device_id ar0234_acpi_ids[] __maybe_unused = {
	{ "INTC10C0" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, ar0234_acpi_ids);

static const struct of_device_id ar0234_dt_ids[] __maybe_unused = {
	{ .compatible = "onnn,ar0234cs" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar0234_dt_ids);

static DEFINE_RUNTIME_DEV_PM_OPS(ar0234_pm_ops, ar0234_power_off,
				 ar0234_power_on, NULL);

static struct i2c_driver ar0234_i2c_driver = {
	.driver = {
		.name = "ar0234",
		.acpi_match_table = ACPI_PTR(ar0234_acpi_ids),
		.of_match_table	= of_match_ptr(ar0234_dt_ids),
		.pm = pm_ptr(&ar0234_pm_ops),
	},
	.probe = ar0234_probe,
	.remove = ar0234_remove,
};
module_i2c_driver(ar0234_i2c_driver);

MODULE_DESCRIPTION("onsemi AR0234 Camera Sensor Driver");
MODULE_AUTHOR("Alexander Shiyan <eagle.alexander923@gmail.com>");
MODULE_LICENSE("GPL");
