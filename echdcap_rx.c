/*
 *  Minimal driver for the HDMI receiver path exposed at I2C address
 *  0x48 on the StarTech ECHDCAP (SAA7160, subsystem f50a:12ab).
 *
 *  This chip is not MST3367: on this board 0x4e/0x9c never answers
 *  after a cold power-cycle, while 0x48 exposes a stable HDMI
 *  status/timing register map (see TODO.md for the reverse-engineering
 *  notes and register snapshots this driver is derived from).
 *
 *  Timing/status registers answer right after reset, but the receiver only
 *  drives its video output after the init sequence below, which replays the
 *  I2C writes the vendor Windows driver makes (captured from a QEMU MMIO
 *  trace, see TODO.md). Register 0x0f is a bank select for 0x80-0xff.
 *
 *  Pixel clock and the VSYNC/VBACKPORCH split of register 0x62 are
 *  not decoded yet, so detected geometry is matched against a small
 *  table of known modes and the corresponding standard CEA/DMT
 *  v4l2_dv_timings is returned.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  SPDX-License-Identifier: GPL-2.0-only
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/v4l2-dv-timings.h>

#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dv-timings.h>

#include "echdcap_rx.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

static int init = 1;
module_param(init, int, 0644);
MODULE_PARM_DESC(init, "run the receiver init sequence at probe (default 1)");

#define dprintk(sd, lvl, fmt, arg...) \
	do { if (debug >= (lvl)) v4l2_info(sd, fmt, ## arg); } while (0)

struct echdcap_rx_reg {
	u8 reg;
	u8 val;
	u16 delay_ms;	/* sleep after this write */
};

/* Windows driver init, in its order; the base part it runs twice. */
static const struct echdcap_rx_reg echdcap_rx_init_base[] = {
	{ 0x06, 0x00 }, { 0x05, 0x10 }, { 0x05, 0x81 }, { 0x07, 0x0c },
	{ 0x16, 0x0f }, { 0x17, 0x85 }, { 0x18, 0x07 }, { 0x8c, 0x20 },
	{ 0x0f, 0x00 }, { 0x08, 0xae }, { 0x3b, 0x40 }, { 0x68, 0x03 },
	{ 0x6b, 0x11 }, { 0x6c, 0x00 }, { 0x93, 0x43 }, { 0x94, 0x4f },
	{ 0x95, 0x87 }, { 0x96, 0x33 }, { 0x1d, 0x30 }, { 0x75, 0x60 },
	{ 0x3d, 0x40 }, { 0x1b, 0x20 }, { 0x1c, 0x50 }, { 0x78, 0xc1 },
	{ 0x89, 0xff }, { 0x05, 0x00 }, { 0x11, 0x89, 200 },
};

/* After the 0x9b clock burst. */
static const struct echdcap_rx_reg echdcap_rx_init_post9b[] = {
	{ 0x9b, 0x00 }, { 0x05, 0x01 }, { 0x89, 0xff }, { 0x05, 0x00 },
	{ 0x07, 0x0c, 500 }, { 0x07, 0x00 }, { 0x73, 0x01 },
};

/* Mode setup the Windows driver does once a signal is detected. */
static const struct echdcap_rx_reg echdcap_rx_init_mode[] = {
	{ 0x19, 0x31, 35 }, { 0x19, 0x30, 35 },
	{ 0x16, 0x07 }, { 0xa8, 0x03 }, { 0x20, 0x80 }, { 0x89, 0x9f },
	{ 0x8c, 0x00 }, { 0x16, 0x00 }, { 0x97, 0x20 }, { 0x05, 0x82 },
	{ 0x73, 0x09 }, { 0x97, 0x00 }, { 0x05, 0x00 }, { 0x73, 0x01 },
	{ 0x8c, 0x20 }, { 0x16, 0x07 },
	{ 0x19, 0x31, 17 }, { 0x19, 0x30, 17 }, { 0x19, 0xf2, 12 }, { 0x19, 0x30, 150 },
	{ 0x19, 0x31, 12 }, { 0x19, 0x30, 35 },
	{ 0xa8, 0x03 }, { 0x16, 0x2f, 110 },
	{ 0x19, 0x31, 16 }, { 0x19, 0x30, 300 },
	/* final pass */
	{ 0x0f, 0x00 }, { 0x08, 0xae }, { 0x3b, 0x40 }, { 0x68, 0x03 },
	{ 0x6b, 0x11 }, { 0x6c, 0x00 }, { 0x93, 0x43 }, { 0x94, 0x4f },
	{ 0x95, 0x87 }, { 0x96, 0x33 }, { 0xa8, 0x81 }, { 0x20, 0x80, 8 },
	{ 0x20, 0x82 }, { 0x1c, 0x50 }, { 0x0f, 0x00 },
	{ 0x18, 0x04 }, { 0x17, 0x85 }, { 0x16, 0x2f }, { 0x8c, 0x20 },
	{ 0x1c, 0x52 }, { 0x1c, 0x50 }, { 0x89, 0x1f }, { 0x89, 0x5f },
	{ 0x89, 0x1f }, { 0x20, 0x82 }, { 0x20, 0x02 }, { 0x77, 0x20 },
	{ 0x78, 0xc1 }, { 0x0f, 0x00 },
};

static const u8 echdcap_rx_csc21[] = { 0x10, 0x80, 0x10 };
static const u8 echdcap_rx_csc24[] = {
	0x09, 0x04, 0x0e, 0x02, 0xc8, 0x00, 0x0e, 0x3d, 0x84,
	0x03, 0x6e, 0x3f, 0xac, 0x3d, 0xd0, 0x3e, 0x84, 0x03,
};

/* Timing/status register map, see TODO.md */
#define REG_HTOTAL_LO		0x59
#define REG_H_GEOM_HI		0x5a	/* [3:0]=htotal[11:8], [7:4]=hactive[11:8] */
#define REG_HACTIVE_LO		0x5b
#define REG_HSYNC_LO		0x5c
#define REG_H_SYNC_FP_HI	0x5d	/* [3:0]=hsync[11:8], [7:4]=hfp[11:8] */
#define REG_HFP_LO		0x5e
#define REG_VTOTAL_LO		0x5f
#define REG_V_GEOM_HI		0x60	/* [3:0]=vtotal[11:8], [7:4]=vactive[11:8] */
#define REG_VACTIVE_LO		0x61
#define REG_VSYNC_VBP		0x62	/* vsync + vbackporch, split not decoded yet */
#define REG_VFP			0x63
#define REG_SIGNAL_STATUS	0x65	/* bit0: 0=no signal, 1=signal present/locked */

struct echdcap_rx_state {
	struct v4l2_subdev sd;
};

static inline struct echdcap_rx_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct echdcap_rx_state, sd);
}

struct echdcap_rx_mode {
	u32 hactive, vactive, htotal, vtotal;
	struct v4l2_dv_timings timings;
};

/*
 * Known modes, matched purely on detected geometry (pixel clock is not
 * decoded yet). Values come from register snapshots recorded in TODO.md.
 */
static const struct echdcap_rx_mode echdcap_rx_modes[] = {
	{ 1920, 1080, 2200, 1125, V4L2_DV_BT_CEA_1920X1080P30 },
	{ 1280,  720, 1650,  750, V4L2_DV_BT_CEA_1280X720P60 },
	{ 1280,  720, 1980,  750, V4L2_DV_BT_CEA_1280X720P50 },
	{  800,  600, 1048,  631, V4L2_DV_BT_DMT_800X600P85 },
};

static int echdcap_rx_read(struct v4l2_subdev *sd, u8 reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int val = i2c_smbus_read_byte_data(client, reg);

	if (val < 0)
		v4l2_err(sd, "i2c read failed at reg 0x%02x (%d)\n", reg, val);
	return val;
}

static int echdcap_rx_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	int val = echdcap_rx_read(sd, REG_SIGNAL_STATUS);

	if (val < 0)
		return val;

	if (val & 0x01) {
		*status &= ~V4L2_IN_ST_NO_SIGNAL;
	} else {
		*status |= V4L2_IN_ST_NO_SIGNAL;
	}

	return 0;
}

static int echdcap_rx_query_dv_timings(struct v4l2_subdev *sd,
					struct v4l2_dv_timings *timings)
{
	int sig, r59, r5a, r5b, r5c, r5d, r5e, r5f, r60, r61, r62, r63;
	u32 htotal, hactive, hsync, hfp, hbp, vtotal, vactive, vfp;
	int i;

	memset(timings, 0, sizeof(*timings));

	sig = echdcap_rx_read(sd, REG_SIGNAL_STATUS);
	if (sig < 0)
		return sig;
	if (!(sig & 0x01))
		return -ENOLINK;

	r59 = echdcap_rx_read(sd, REG_HTOTAL_LO);
	r5a = echdcap_rx_read(sd, REG_H_GEOM_HI);
	r5b = echdcap_rx_read(sd, REG_HACTIVE_LO);
	r5c = echdcap_rx_read(sd, REG_HSYNC_LO);
	r5d = echdcap_rx_read(sd, REG_H_SYNC_FP_HI);
	r5e = echdcap_rx_read(sd, REG_HFP_LO);
	r5f = echdcap_rx_read(sd, REG_VTOTAL_LO);
	r60 = echdcap_rx_read(sd, REG_V_GEOM_HI);
	r61 = echdcap_rx_read(sd, REG_VACTIVE_LO);
	r62 = echdcap_rx_read(sd, REG_VSYNC_VBP);
	r63 = echdcap_rx_read(sd, REG_VFP);

	if ((r59 | r5a | r5b | r5c | r5d | r5e | r5f | r60 | r61 | r62 | r63) < 0)
		return -EIO;

	htotal  = ((r5a & 0x0f) << 8) | r59;
	hactive = ((r5a & 0xf0) << 4) | r5b;

	hsync = ((r5d & 0x0f) << 8) | r5c;
	hfp   = ((r5d & 0xf0) << 4) | r5e;
	hbp   = htotal - hactive - hsync - hfp;

	vtotal  = ((r60 & 0x0f) << 8) | r5f;
	vactive = ((r60 & 0xf0) << 4) | r61;
	vfp     = r63;

	dprintk(sd, 1,
		"geometry: htotal=%u hactive=%u hsync=%u hfp=%u hbp=%u vtotal=%u vactive=%u vfp=%u (r62=%u)\n",
		htotal, hactive, hsync, hfp, hbp, vtotal, vactive, vfp, r62);

	if (!hactive || !vactive || htotal <= hactive || vtotal <= vactive)
		return -ENOLCK;

	for (i = 0; i < ARRAY_SIZE(echdcap_rx_modes); i++) {
		const struct echdcap_rx_mode *m = &echdcap_rx_modes[i];

		if (m->hactive == hactive && m->vactive == vactive &&
		    m->htotal == htotal && m->vtotal == vtotal) {
			*timings = m->timings;
			return 0;
		}
	}

	v4l2_info(sd,
		  "unrecognized mode: hactive=%u vactive=%u htotal=%u vtotal=%u\n",
		  hactive, vactive, htotal, vtotal);
	return -ERANGE;
}

/*
 * This receiver free-runs to whatever HDMI source is connected and has no
 * known register writes to program a specific mode (see TODO.md item 8), so
 * there is nothing to do here. It exists so that callers of s_dv_timings
 * (i.e. the saa716x bridge driver's VIDIOC_S_DV_TIMINGS handler) succeed and
 * can update their own idea of the current timings from query_dv_timings().
 */
static int echdcap_rx_s_dv_timings(struct v4l2_subdev *sd,
				    struct v4l2_dv_timings *timings)
{
	return 0;
}

static const struct v4l2_subdev_video_ops echdcap_rx_video_ops = {
	.g_input_status = echdcap_rx_g_input_status,
	.query_dv_timings = echdcap_rx_query_dv_timings,
	.s_dv_timings = echdcap_rx_s_dv_timings,
};

static const struct v4l2_subdev_ops echdcap_rx_ops = {
	.video = &echdcap_rx_video_ops,
};

static int echdcap_rx_write_seq(struct i2c_client *client,
				const struct echdcap_rx_reg *seq, size_t n)
{
	size_t i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = i2c_smbus_write_byte_data(client, seq[i].reg, seq[i].val);
		if (ret < 0) {
			dev_err(&client->dev, "init write 0x%02x=0x%02x failed: %d\n",
				seq[i].reg, seq[i].val, ret);
			return ret;
		}
		if (seq[i].delay_ms)
			msleep(seq[i].delay_ms);
	}
	return 0;
}

static int echdcap_rx_hw_init(struct i2c_client *client)
{
	int pass, i, ret;

	for (pass = 0; pass < 2; pass++) {
		ret = echdcap_rx_write_seq(client, echdcap_rx_init_base,
					   ARRAY_SIZE(echdcap_rx_init_base));
		if (ret)
			return ret;
		for (i = 0; i < 16; i++) {
			i2c_smbus_write_byte_data(client, 0x9b, 0x80);
			i2c_smbus_write_byte_data(client, 0x9b, 0xc0);
		}
		ret = echdcap_rx_write_seq(client, echdcap_rx_init_post9b,
					   ARRAY_SIZE(echdcap_rx_init_post9b));
		if (ret)
			return ret;
	}
	ret = echdcap_rx_write_seq(client, echdcap_rx_init_mode,
				   ARRAY_SIZE(echdcap_rx_init_mode));
	if (ret)
		return ret;

	/* Colour space conversion; Windows writes these as two burst transfers. */
	ret = i2c_smbus_write_i2c_block_data(client, 0x21, sizeof(echdcap_rx_csc21),
					     echdcap_rx_csc21);
	if (!ret)
		ret = i2c_smbus_write_i2c_block_data(client, 0x24, sizeof(echdcap_rx_csc24),
						     echdcap_rx_csc24);
	if (ret)
		dev_err(&client->dev, "CSC write failed: %d\n", ret);
	return ret;
}

static int echdcap_rx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct echdcap_rx_state *state;
	struct v4l2_subdev *sd;
	int val;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &echdcap_rx_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	val = i2c_smbus_read_byte_data(client, REG_SIGNAL_STATUS);
	if (val < 0) {
		v4l2_err(sd, "no response at 0x%02x (%s): %d\n",
			 client->addr, client->adapter->name, val);
		return val;
	}

	if (init) {
		int ret = echdcap_rx_hw_init(client);

		if (ret)
			return ret;
		v4l2_info(sd, "receiver init done\n");
	}

	v4l2_info(sd, "echdcap_rx found @ 0x%02x (%s), status reg=0x%02x\n",
		  client->addr, client->adapter->name, val);

	return 0;
}

static int echdcap_rx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	return 0;
}

static const struct i2c_device_id echdcap_rx_id[] = {
	{ "echdcap_rx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, echdcap_rx_id);

static struct i2c_driver echdcap_rx_driver = {
	.driver = {
		.name = "echdcap_rx",
	},
	.probe    = echdcap_rx_probe,
	.remove   = echdcap_rx_remove,
	.id_table = echdcap_rx_id,
};

module_i2c_driver(echdcap_rx_driver);

MODULE_DESCRIPTION("StarTech ECHDCAP HDMI receiver (0x48) minimal subdev");
MODULE_LICENSE("GPL");
