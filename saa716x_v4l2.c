#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/videodev2.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/i2c-algo-bit.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/random.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-sg.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/i2c/adv7604.h>

#include "saa716x_priv.h"
#include "saa716x_mod.h"
#include "saa716x_dma.h"
#include "saa716x_spi.h"
#include "saa716x_i2c.h"
#include "saa716x_gpio.h"
#include "saa716x_cap.h"
#include "saa716x_v4l2.h"
#include "saa716x_vip_reg.h"
#include "mst3367-drv.h"
#include "echdcap_rx.h"
#include "saa716x_gpio_reg.h"
#include "saa716x_greg_reg.h"

static int video_vip_get_stream_params_tda19978(struct saa716x_stream *s);
static int video_vip_get_stream_params_adv7611(struct saa716x_stream *s);
static int video_vip_get_stream_params_echdcap(struct saa716x_stream *s);

/* one VIP DMA channel: 512 page-table entries of 4 KiB */
#define ECHDCAP_DMA_WINDOW	(512 * SAA716x_PAGE_SIZE)

/*
 * Debug-only overrides for live experimentation on the ECHDCAP board.
 * Defaults reproduce the confirmed-correct values (0x06080F8C is what this
 * card's own EEPROM STREAM_DEVICE record encodes; 0 means "don't do the
 * extra 4th GPIO_WR write"). Only applied on the f50a:12ab subsystem path.
 */
static uint echdcap_vi_ctrl = 0x06080F8C;
module_param(echdcap_vi_ctrl, uint, 0644);
MODULE_PARM_DESC(echdcap_vi_ctrl, "debug override for GREG_VI_CTRL on ECHDCAP");

static uint echdcap_board_init = 1;
module_param(echdcap_board_init, uint, 0644);
MODULE_PARM_DESC(echdcap_board_init, "debug: 0 = skip ECHDCAP GPIO reset/board init (keeps receiver state)");

static uint echdcap_gpio_wr_extra;
module_param(echdcap_gpio_wr_extra, uint, 0644);
MODULE_PARM_DESC(echdcap_gpio_wr_extra, "debug: extra GPIO_WR write after reset sequence on ECHDCAP (0=skip)");

static uint echdcap_pad = 1;
module_param(echdcap_pad, uint, 0644);
MODULE_PARM_DESC(echdcap_pad, "debug: pad lines so frames span both VIP DMA channels (0=off)");

static uint echdcap_flags_clear;
module_param(echdcap_flags_clear, uint, 0644);
MODULE_PARM_DESC(echdcap_flags_clear, "debug: VIP stream flags to clear on ECHDCAP");

static uint dma_poison;
module_param(dma_poison, uint, 0644);
MODULE_PARM_DESC(dma_poison, "debug: fill each queued buffer with 0xee");

static uint echdcap_mcu;
module_param(echdcap_mcu, uint, 0444);
MODULE_PARM_DESC(echdcap_mcu, "ECHDCAP: arm and kick the 0x51 watchdog MCU (default off: suspected host hangs)");

static const u32 vi_ch[] = {
    VI0,
    VI1
};

static u8 edid[256] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
	0x3b, 0x10, 0x0a, 0x4e, 0x01, 0x00, 0x00, 0x00,
	0x26, 0x11, 0x01, 0x03, 0x80, 0x00, 0x00, 0x00,
	0x1a, 0xee, 0x91, 0xa3, 0x54, 0x4c, 0x99, 0x26,
	0x0f, 0x50, 0x54, 0x20, 0x00, 0x00, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1d,
	0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e, 0x28,
	0x55, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x1e,
	0x00, 0x00, 0x00, 0xfc, 0x00, 0x44, 0x45, 0x4d,
	0x4f, 0x20, 0x54, 0x44, 0x41, 0x31, 0x39, 0x39,
	0x37, 0x38, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x32,
	0x3c, 0x0f, 0x44, 0x0f, 0x00, 0x0a, 0x20, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfe,
	0x00, 0x31, 0x30, 0x38, 0x30, 0x70, 0x36, 0x30,
	0x26, 0x35, 0x30, 0x78, 0x76, 0x43, 0x01, 0xa1,

	0x02, 0x03, 0x2a, 0xf0, 0x6c, 0x03, 0x0c, 0x00,
	0x10, 0x00, 0xb8, 0x2d, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x4c, 0x04, 0x13, 0x03, 0x12, 0x05, 0x14,
	0x07, 0x16, 0x01, 0x20, 0x21, 0x22, 0x23, 0x09,
	0x07, 0x01, 0x83, 0x01, 0x00, 0x00, 0xe3, 0x05,
	0x03, 0x01, 0x01, 0x1d, 0x00, 0xbc, 0x52, 0xd0,
	0x1e, 0x20, 0xb8, 0x28, 0x55, 0x40, 0xc4, 0x8e,
	0x21, 0x00, 0x00, 0x1e, 0x02, 0x3a, 0x80, 0x18,
	0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00,
	0xc4, 0x8e, 0x21, 0x00, 0x00, 0x1e, 0x02, 0x3a,
	0x80, 0xd0, 0x72, 0x38, 0x2d, 0x40, 0x10, 0x2c,
	0x45, 0x80, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x1e,
	0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10,
	0x10, 0x3e, 0x96, 0x00, 0xc4, 0x8e, 0x21, 0x00,
	0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66  
};

static inline struct saa716x_cap_buffer *to_saa716x_cap_buffer(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct saa716x_cap_buffer, vb);
}

/*
 * HDTV: this structure has the capabilities of the HDTV receiver.
	- V4L2_DV_BT_CEA_640X480P59_94
 */
static const struct v4l2_dv_timings_cap saa716x_cap_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	V4L2_INIT_BT_TIMINGS(
		640, 1920,		/* min/max width */
		480, 1080,		/* min/max height */
		25175000, 74250000,	/* min/max pixelclock*/
		V4L2_DV_BT_STD_CEA861,	/* Supported standards */
		/* capabilities */
		V4L2_DV_BT_CAP_INTERLACED | V4L2_DV_BT_CAP_PROGRESSIVE
	)
};

/* ECHDCAP: receiver also locks to VESA modes up to 1360x768@60 (85.5 MHz). */
static const struct v4l2_dv_timings_cap echdcap_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	.reserved = { 0 },
	V4L2_INIT_BT_TIMINGS(
		640, 1920,
		480, 1080,
		25175000, 85500000,
		V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT,
		V4L2_DV_BT_CAP_PROGRESSIVE
	)
};

static const struct v4l2_dv_timings_cap *saa716x_timings_cap(struct saa716x_stream *s)
{
	if (s->saa716x->config->capture_config.subdev == SAA716x_SUBDEV_ECHDCAP_RX)
		return &echdcap_timings_cap;
	return &saa716x_cap_timings_cap;
}

/*
 * Supported SDTV standards.
 */
#define SAA716X_TVNORMS V4L2_STD_ALL

/* for DMA debug */
static void sg_info(char* info, struct scatterlist *sg) {
	int npages = 0, i;
	struct scatterlist *sg_temp;

	printk("%s: %s :sg_nents(sg) = %d", __func__, info, sg_nents(sg));
	for_each_sg(sg, sg_temp, sg_nents(sg), i) {
		/* page counting code (lib/scatterlist.c:724) */
		npages += PAGE_ALIGN(sg_temp->offset + sg_temp->length) >> PAGE_SHIFT;
		printk("%s: sg[%d]:dma_addr = 0x%llx, offset = 0x%x, dma_len = 0x%x",
			__func__, i, sg_dma_address(sg_temp), sg_temp->offset, sg_dma_len(sg_temp));
		}
	printk("%s: npages = %d", __func__, npages);
}

/*
* Setup the constraints of the queue: besides setting the number of planes
* per buffer and the size and allocation context of each plane, it also
* checks if sufficient buffers have been allocated. Usually 3 is a good
* minimum number: many DMA engines need a minimum of 2 buffers in the
* queue and you need to have another available for userspace processing.
*/
static int queue_setup(struct vb2_queue *vq,
			  unsigned int *nbuffers, unsigned int *nplanes,
			  unsigned int sizes[], struct device *alloc_devs[])
{
   struct saa716x_stream *s = vb2_get_drv_priv(vq);

	printk("%s: called", __func__);
	s->field = s->format.field;
	if (s->field == V4L2_FIELD_ALTERNATE) {
		/*
			* You cannot use read() with FIELD_ALTERNATE since the field
			* information (TOP/BOTTOM) cannot be passed back to the user.
			*/
		if (vb2_fileio_is_active(vq))
			return -EINVAL;
		//s->field = V4L2_FIELD_TOP;
	}

	if (vq->num_buffers + *nbuffers < 3)
		*nbuffers = 3 - vq->num_buffers;

	if (*nplanes)
		return sizes[0] < s->format.sizeimage ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = s->format.sizeimage;
	printk("%s: field = 0x%x", __func__, s->format.field);
	printk("%s: sizeimage = 0x%x", __func__, s->format.sizeimage);

	return 0;
}

/*
* Prepare the buffer for queueing to the DMA engine: check and set the
* payload size.
*/
static int buffer_prepare(struct vb2_buffer *vb)
{
   struct saa716x_stream *s = vb2_get_drv_priv(vb->vb2_queue);
   unsigned long size = s->format.sizeimage;
   
   //printk("%s: called", __func__);
   if (vb2_plane_size(vb, 0) < size) {
	   dev_err(&s->saa716x->pdev->dev, "buffer too small (%lu < %lu)\n",
			vb2_plane_size(vb, 0), size);
	   return -EINVAL;
   }

   vb2_set_plane_payload(vb, 0, size);
   return 0;
}

/*
* Queue this buffer to the DMA engine.
*/
/*
 * ECHDCAP: after a short signal loss (source reboot, BIOS <-> OS switch with
 * an unchanged mode) the VIP keeps raising frame interrupts but the frames
 * handed to userspace stay frozen; only a stop/start of the VIP recovers.
 * The worker flags the input errors latched in INT_STATUS and, once they
 * have stopped for 300 ms, the VIP is restarted with the queued buffers
 * re-assigned to MMU slots from 0.
 */
static void saa716x_cap_map_buffer(struct saa716x_stream *s, struct vb2_buffer *vb, int slot);

#define VIP_INPUT_ERRORS	(VI_STAT_VID_OVRFLW | VI_STAT_WIN_SEQBRK | VI_STAT_FID_SEQBRK)

static uint echdcap_auto_restart = 1;
module_param(echdcap_auto_restart, uint, 0644);
MODULE_PARM_DESC(echdcap_auto_restart, "ECHDCAP: restart capture after input errors (0=off, write 2 to force one)");

void saa716x_cap_check_input_errors(struct saa716x_stream *s)
{
	struct saa716x_dev *saa716x = s->saa716x;
	u32 st;

	if (!echdcap_auto_restart || !s->streaming ||
	    saa716x->config->capture_config.subdev != SAA716x_SUBDEV_ECHDCAP_RX)
		return;
	s->last_frame = jiffies;
	st = SAA716x_EPRD(vi_ch[s->vip_port], INT_STATUS);
	if (echdcap_auto_restart == 2) {	/* debug: force one restart */
		echdcap_auto_restart = 1;
		st |= VI_STAT_WIN_SEQBRK;
	}
	if (!(st & VIP_INPUT_ERRORS))
		return;
	SAA716x_EPWR(vi_ch[s->vip_port], INT_CLR_STATUS, st & VIP_INPUT_ERRORS);
	mod_delayed_work(system_wq, &s->restart_work, msecs_to_jiffies(300));
}
EXPORT_SYMBOL(saa716x_cap_check_input_errors);

static void saa716x_cap_restart_work(struct work_struct *work)
{
	struct saa716x_stream *s = container_of(to_delayed_work(work), struct saa716x_stream,
						restart_work);
	struct saa716x_dev *saa716x = s->saa716x;
	struct saa716x_cap_buffer *buf;
	unsigned long flags;
	int port = s->vip_port, slot = 0, ret;

	if (!s->streaming)
		return;
	/* stop_streaming holds video_lock and cancels us: never block on it */
	if (!mutex_trylock(&s->video_lock)) {
		schedule_delayed_work(&s->restart_work, msecs_to_jiffies(20));
		return;
	}
	if (!s->streaming)
		goto out;

	tasklet_disable(&saa716x->vip[port].tasklet);
	saa716x_vip_stop(saa716x, port);
	msleep(50);	/* let the frame in flight drain */
	SAA716x_EPWR(vi_ch[port], VI_MODE, SOFT_RESET);

	spin_lock_irqsave(&s->qlock, flags);
	list_for_each_entry(buf, &s->buf_list, list) {
		saa716x_cap_map_buffer(s, &buf->vb.vb2_buf, slot);
		slot = (slot + 1) & 7;
	}
	s->mmu_q_index = slot;
	spin_unlock_irqrestore(&s->qlock, flags);

	SAA716x_EPWR(vi_ch[port], INT_CLR_STATUS, 0x3ff);
	ret = saa716x_vip_start(saa716x, port, 0, &s->vip_params);
	s->last_frame = jiffies;
	tasklet_enable(&saa716x->vip[port].tasklet);
	s->restarts++;
	dev_info(&saa716x->pdev->dev, "ECHDCAP: capture restarted (#%u)%s\n",
		 s->restarts, ret ? ", VIP start FAILED" : "");
out:
	mutex_unlock(&s->video_lock);
}

/*
 * Frames can also stop altogether: with RST_ON_ERR the VIP clears its own
 * capture enable on an input error, and then no interrupt ever reaches the
 * check above. Poll while streaming; if capture got disabled or no frame
 * arrived for 500 ms although the receiver is locked, restart.
 */
#define ECHDCAP_WATCH_PERIOD	msecs_to_jiffies(250)
#define ECHDCAP_FRAME_TIMEOUT	msecs_to_jiffies(500)

static void saa716x_cap_watch_work(struct work_struct *work)
{
	struct saa716x_stream *s = container_of(to_delayed_work(work), struct saa716x_stream,
						watch_work);
	struct saa716x_dev *saa716x = s->saa716x;
	u32 mode, status = 0;

	if (!s->streaming)
		return;
	mode = SAA716x_EPRD(vi_ch[s->vip_port], VI_MODE);
	if (echdcap_auto_restart && !delayed_work_pending(&s->restart_work) &&
	    (!(mode & VID_CFEN) || time_after(jiffies, s->last_frame + ECHDCAP_FRAME_TIMEOUT)) &&
	    !v4l2_subdev_call(s->sd_receiver, video, g_input_status, &status) && !status) {
		dev_info(&saa716x->pdev->dev, "ECHDCAP: capture stalled (VI_MODE=%08x, last frame %u ms ago)\n",
			 mode, jiffies_to_msecs(jiffies - s->last_frame));
		s->last_frame = jiffies;	/* one restart per timeout */
		mod_delayed_work(system_wq, &s->restart_work, 0);
	}
	schedule_delayed_work(&s->watch_work, ECHDCAP_WATCH_PERIOD);
}

/* point MMU slot @slot at @vb */
static void saa716x_cap_map_buffer(struct saa716x_stream *s, struct vb2_buffer *vb, int slot)
{
	struct sg_table *sg_desc = vb2_dma_sg_plane_desc(vb, 0);
	struct saa716x_dmabuf *dmabuf;

	dmabuf = &s->saa716x->vip[s->vip_port].dma_buf[0][slot];
	saa716x_dmabuf_sgpagefill(dmabuf, sg_desc->sgl, sg_desc->nents, 0);
	if (s->format.sizeimage > SAA716x_PAGE_SIZE * 512) {
		/* dual channel DMA */
		dmabuf = &s->saa716x->vip[s->vip_port].dma_buf[1][slot];
		saa716x_dmabuf_sgpagefill(dmabuf, sg_desc->sgl, sg_desc->nents, SAA716x_PAGE_SIZE * 512);
	}
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct saa716x_stream *s = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct saa716x_cap_buffer *buf = to_saa716x_cap_buffer(vbuf);
	unsigned long flags;

	//printk("%s: called", __func__);
	spin_lock_irqsave(&s->qlock, flags);
	list_add_tail(&buf->list, &s->buf_list);
	spin_unlock_irqrestore(&s->qlock, flags);

	//sg_info("sg_desc->sgl", sg_desc->sgl);

	pr_debug("%s: vb2_buffer(%d) enqued at PTE %d\n", __func__, vb->index, s->mmu_q_index);
	if (dma_poison) {
		void *va = vb2_plane_vaddr(vb, 0);

		if (va)
			memset(va, 0xee, vb2_plane_size(vb, 0));
	}
	saa716x_cap_map_buffer(s, vb, s->mmu_q_index);
	if ((s->vip_params.stream_flags & VIP_FIELD_SEQ) ||
		(s->vip_params.stream_flags & (VIP_EVEN_FIELD | VIP_ODD_FIELD))) {
		s->mmu_q_index = (s->mmu_q_index + 1) & 3;
	} else {
		s->mmu_q_index = (s->mmu_q_index + 1) & 7;
	}
}

static void return_all_buffers(struct saa716x_stream *s,
				  enum vb2_buffer_state state)
{
   struct saa716x_cap_buffer *buf, *node;
   unsigned long flags;

	printk("%s: called", __func__);
	spin_lock_irqsave(&s->qlock, flags);
	list_for_each_entry_safe(buf, node, &s->buf_list, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&s->qlock, flags);
}

/*
* Start streaming. First check if the minimum number of buffers have been
* queued. If not, then return -ENOBUFS and the vb2 framework will call
* this function again the next time a buffer has been queued until enough
* buffers are available to actually start the DMA engine.
*/
static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct saa716x_stream *s = vb2_get_drv_priv(vq);
	struct saa716x_dev *saa716x = s->saa716x;
	int ret = 0;

	printk("%s: called", __func__);
	s->sequence = 0;

	/* the timings may have changed since the last S_FMT (see follow_source) */
	if (saa716x->config->capture_config.subdev == SAA716x_SUBDEV_ECHDCAP_RX &&
	    video_vip_get_stream_params_echdcap(s)) {
		return_all_buffers(s, VB2_BUF_STATE_QUEUED);
		return -EINVAL;
	}

	/* start DMA */
	SAA716x_EPWR(vi_ch[s->vip_port], INT_CLR_STATUS, 0x3ff);
	ret = saa716x_vip_start(saa716x, s->vip_port, 0, &s->vip_params);
	s->streaming = !ret;
	s->last_frame = jiffies;
	if (s->streaming && saa716x->config->capture_config.subdev == SAA716x_SUBDEV_ECHDCAP_RX)
		schedule_delayed_work(&s->watch_work, ECHDCAP_WATCH_PERIOD);

	if (ret) {
		/*
		* In case of an error, return all active buffers to the
		* QUEUED state
		*/
		return_all_buffers(s, VB2_BUF_STATE_QUEUED);
	}
	return ret;
}

/*
* Stop the DMA engine. Any remaining buffers in the DMA queue are dequeued
* and passed on to the vb2 framework marked as STATE_ERROR.
*/
static void stop_streaming(struct vb2_queue *vq)
{
	struct saa716x_stream *s = vb2_get_drv_priv(vq);
	struct saa716x_dev *saa716x = s->saa716x;
	int vip_port = s->vip_port;
	int val;

	printk("%s: called", __func__);

	/* the restart work backs off while we hold video_lock */
	s->streaming = false;
	cancel_delayed_work_sync(&s->watch_work);
	cancel_delayed_work_sync(&s->restart_work);

	/* stop DMA */
	saa716x_vip_stop(saa716x, vip_port);
	s->mmu_q_index = 0;

	/* Release all active buffers */
	return_all_buffers(s, VB2_BUF_STATE_ERROR);

	/* Report VIP status */
    val = SAA716x_EPRD(vi_ch[vip_port], INT_STATUS);
    printk("%s: [VI%d] INT_STATUS=0x%x", __func__, vip_port, val);
    val &= VI_STAT_LINE_COUNT;
    printk("%s: [VI%d] LINE_COUNT=%d", __func__, vip_port, val >> 16);
	
	SAA716x_EPWR(vi_ch[vip_port], VI_MODE, SOFT_RESET);
}

static const struct vb2_ops saa716x_cap_qops = {
	.queue_setup		= queue_setup,
	.buf_prepare		= buffer_prepare,
	.buf_queue			= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

/*
 * Helper function to check and correct struct v4l2_pix_format. It's used
 * not only in VIDIOC_TRY/S_FMT, but also elsewhere if changes to the SDTV
 * standard, HDTV timings or the video input would require updating the
 * current format.
 */
static void saa716x_cap_fill_pix_format(struct saa716x_stream *s,
				     struct v4l2_pix_format *pix)
{
	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	if (s->input == 0) {
		/* HDMI input */
		pix->width = s->timings.bt.width;
		pix->height = s->timings.bt.height;
		if (s->timings.bt.interlaced) {
			switch (pix->field) {
				case V4L2_FIELD_SEQ_TB:
					break;	
				case V4L2_FIELD_ALTERNATE:
					pix->height /= 2;
					break;
				default:
					pix->field = V4L2_FIELD_INTERLACED;
			}
		} else {
			pix->field = V4L2_FIELD_NONE;
		}
		pix->colorspace = V4L2_COLORSPACE_REC709;
		/*
		 * ECHDCAP: the receiver CSC (coefficients from the vendor driver)
		 * produces BT.601 limited-range YCbCr regardless of the mode
		 * (measured with colour bars: yellow = 210/16/146).
		 */
		if (s->saa716x->config->capture_config.subdev == SAA716x_SUBDEV_ECHDCAP_RX) {
			pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
			pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		}
	} else {
		/* YPbPr input */
		pix->width = 720;
		pix->height = (s->std & V4L2_STD_525_60) ? 480 : 576;
		pix->field = V4L2_FIELD_INTERLACED;
		pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	}

	/*
	 * The YUYV format is four bytes for every two pixels, so bytesperline
	 * is width * 2.
	 */
	pix->bytesperline = pix->width * 2;
	/*
	 * ECHDCAP: a VIP DMA channel only switches to its next buffer in time
	 * when the frame crosses the end of the channel's 2 MB window; frames
	 * that fit into one window lose their first ~768 bytes to the previous
	 * buffer (and once to host low memory). Pad the lines so that every
	 * progressive frame spans both channels like 1080p does.
	 */
	if (echdcap_pad &&
	    s->saa716x->config->capture_config.subdev == SAA716x_SUBDEV_ECHDCAP_RX &&
	    pix->field == V4L2_FIELD_NONE &&
	    pix->bytesperline * pix->height <= ECHDCAP_DMA_WINDOW + SAA716x_PAGE_SIZE)
		pix->bytesperline = round_up(DIV_ROUND_UP(ECHDCAP_DMA_WINDOW + SAA716x_PAGE_SIZE,
							  pix->height), 64);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
}

/* ------------------------------------------------------------------
	video ioctls for the device
   ------------------------------------------------------------------*/

static int saa716x_cap_s_dv_timings(struct file *file, void *_fh,
				 struct v4l2_dv_timings *timings)
{
	struct saa716x_stream *s = video_drvdata(file);
	struct v4l2_subdev *sd = s->sd_receiver;
	int err;

	printk("%s: called", __func__);
	/* S_DV_TIMINGS is not supported on the Analog input */
	if (s->input == 1)
		return -ENODATA;

	if (!v4l2_valid_dv_timings(timings, saa716x_timings_cap(s), NULL, NULL))
		return -EINVAL;

	/* Check if the timings are part of the supported timings. */
	if (!v4l2_find_dv_timings_cap(timings, saa716x_timings_cap(s),
				      0, NULL, NULL))
		return -EINVAL;

	/* Return 0 if the new timings are the same as the current timings. */
	if (v4l2_match_dv_timings(timings, &s->timings, 0, false))
		return 0;

	/*
	 * Changing the timings implies a format change, which is not allowed
	 * while buffers for use with streaming have already been allocated.
	 */
	if (vb2_is_busy(&s->queue))
		return -EBUSY;

	err = v4l2_subdev_call(sd, video, s_dv_timings, timings);
	if (err) {
		return err;
	}

	s->timings = *timings;
	/* Update the internal format */
	saa716x_cap_fill_pix_format(s, &s->format);

	return 0;
}

static int saa716x_cap_g_dv_timings(struct file *file, void *_fh,
				 struct v4l2_dv_timings *timings)
{
	struct saa716x_stream *s = video_drvdata(file);
	//struct v4l2_subdev *sd = s->sd_receiver;

	printk("%s: called", __func__);
	if (s->input == 1)
		return -ENODATA;

	*timings = s->timings;
	return 0;
	//return v4l2_subdev_call(sd, video, g_dv_timings, timings);
}

static int saa716x_cap_enum_dv_timings(struct file *file, void *_fh,
				    struct v4l2_enum_dv_timings *timings)
{
	struct saa716x_stream *s = video_drvdata(file);
	struct v4l2_subdev *sd = s->sd_receiver;

	printk("%s: called", __func__);
	if (s->input == 1)
		return -ENODATA;

	return v4l2_enum_dv_timings_cap(timings, saa716x_timings_cap(s), NULL, NULL);
	//return v4l2_subdev_call(sd, pad, enum_dv_timings, timings);
}

/*
 * Query the current timings as seen by the hardware. This function shall
 * never actually change the timings, it just detects and reports.
 * If no signal is detected, then return -ENOLINK. If the hardware cannot
 * lock to the signal, then return -ENOLCK. If the signal is out of range
 * of the capabilities of the system (e.g., it is possible that the receiver
 * can lock but that the DMA engine it is connected to cannot handle
 * pixelclocks above a certain frequency), then -ERANGE is returned.
 */
static int saa716x_cap_query_dv_timings(struct file *file, void *_fh,
				     struct v4l2_dv_timings *timings)
{	
	struct saa716x_stream *s = video_drvdata(file);
	struct v4l2_subdev *sd = s->sd_receiver;
	int ret;
	
	printk("%s: called", __func__);
	if (s->input == 1)
		return -ENODATA;

	ret = v4l2_subdev_call(sd, video, query_dv_timings, timings);
	return ret;

#if 0
	if (no_signal)
		return -ENOLINK;
	if (cannot_lock_to_signal)
		return -ENOLCK;
	if (signal_out_of_range_of_capabilities)
		return -ERANGE;

	/* Useful for debugging */
	v4l2_print_dv_timings(s->v4l2_dev.name, "query_dv_timings:",
			timings, true);
#endif
	return 0;
}

static int saa716x_cap_dv_timings_cap(struct file *file, void *fh,
				   struct v4l2_dv_timings_cap *cap)
{
	struct saa716x_stream *s = video_drvdata(file);
	struct v4l2_subdev *sd = s->sd_receiver;

	if (s->input == 1)
		return -ENODATA;
	
	return v4l2_subdev_call(sd, pad, dv_timings_cap, cap);
}

static int saa716x_cap_enum_input(struct file *file, void *priv,
			       struct v4l2_input *i)
{
	if (i->index > 1)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	if (i->index == 0) {
		i->std = 0;
		strlcpy(i->name, "HDMI", sizeof(i->name));
		i->capabilities = V4L2_IN_CAP_DV_TIMINGS;
	} else {
		i->std = SAA716X_TVNORMS;
		strlcpy(i->name, "YPbPr", sizeof(i->name));
		i->capabilities = V4L2_IN_CAP_STD;
	}
	return 0;
}

static int saa716x_cap_s_input(struct file *file, void *priv, unsigned int i)
{
	struct saa716x_stream *s = video_drvdata(file);

	if (i > 1)
		return -EINVAL;

	/*
	 * Changing the input implies a format change, which is not allowed
	 * while buffers for use with streaming have already been allocated.
	 */
	if (vb2_is_busy(&s->queue))
		return -EBUSY;

	s->input = i;
	/*
	 * Update tvnorms. The tvnorms value is used by the core to implement
	 * VIDIOC_ENUMSTD so it has to be correct. If tvnorms == 0, then
	 * ENUMSTD will return -ENODATA.
	 */
	s->vdev.tvnorms = i ? SAA716X_TVNORMS : 0;

	/* Update the internal format */
	saa716x_cap_fill_pix_format(s, &s->format);
	return 0;
}

static int saa716x_cap_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct saa716x_stream *s = video_drvdata(file);

	*i = s->input;
	return 0;
}

/*
 * Required ioctl querycap. Note that the version field is prefilled with
 * the version of the kernel.
 */
static int saa716x_cap_querycap(struct file *file, void *priv,
			     struct v4l2_capability *cap)
{
	struct saa716x_stream *s = video_drvdata(file);
	struct saa716x_capture_config *config = &s->saa716x->config->capture_config;

	strlcpy(cap->driver, "saa716x", sizeof(cap->driver));
	strlcpy(cap->card, config->board_name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", pci_name(s->saa716x->pdev));
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_READWRITE | 
						V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_DEVICE_CAPS;
	return 0;
}


static int saa716x_cap_try_fmt_vid_cap(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct saa716x_stream *s = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	printk("%s: called", __func__);
	/*
	 * Due to historical reasons providing try_fmt with an unsupported
	 * pixelformat will return -EINVAL for video receivers. Webcam drivers,
	 * however, will silently correct the pixelformat. Some video capture
	 * applications rely on this behavior...
	 */
	if (pix->pixelformat != V4L2_PIX_FMT_YUYV)
		return -EINVAL;
	saa716x_cap_fill_pix_format(s, pix);
	return 0;
}

static int saa716x_cap_s_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct saa716x_stream *s = video_drvdata(file);
    enum saa716x_capture_subdev sd_type = s->saa716x->config->capture_config.subdev;
	int ret;

	printk("%s: called", __func__);
	ret = saa716x_cap_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	/*
	 * It is not allowed to change the format while buffers for use with
	 * streaming have already been allocated.
	 */
	if (vb2_is_busy(&s->queue))
		return -EBUSY;

	/* TODO: change format */
	s->format = f->fmt.pix;

	/* set vip parameters */
	if (sd_type == SAA716x_SUBDEV_TDA19978) {
		ret = video_vip_get_stream_params_tda19978(s);
	} else if (sd_type == SAA716x_SUBDEV_ECHDCAP_RX) {
		ret = video_vip_get_stream_params_echdcap(s);
	} else {
		ret = video_vip_get_stream_params_adv7611(s);
	}
	if (ret)
		return -EINVAL;
	return 0;
}

static int saa716x_cap_g_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct saa716x_stream *s = video_drvdata(file);

	printk("%s: called", __func__);
	f->fmt.pix = s->format;
	return 0;
}

static int saa716x_cap_enum_fmt_vid_cap(struct file *file, void *priv,
				     struct v4l2_fmtdesc *f)
{
	if (f->index != 0)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_YUYV;
	return 0;
}

static int saa716x_cap_s_std(struct file *file, void *priv, v4l2_std_id std)
{
	struct saa716x_stream *s = video_drvdata(file);

	/* S_STD is not supported on the HDMI input */
	if (!s->input)
		return -ENODATA;

	/*
	 * No change, so just return. Some applications call S_STD again after
	 * the buffers for streaming have been set up, so we have to allow for
	 * this behavior.
	 */
	if (std == s->std)
		return 0;

	/*
	 * Changing the standard implies a format change, which is not allowed
	 * while buffers for use with streaming have already been allocated.
	 */
	if (vb2_is_busy(&s->queue))
		return -EBUSY;

	/* TODO: handle changing std */

	s->std = std;

	/* Update the internal format */
	saa716x_cap_fill_pix_format(s, &s->format);
	return 0;
}

static int saa716x_cap_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct saa716x_stream *s = video_drvdata(file);

	/* G_STD is not supported on the HDMI input */
	if (!s->input)
		return -ENODATA;

	*std = s->std;
	return 0;
}

static int saa716x_cap_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
	struct saa716x_stream *s = video_drvdata(file);

	/* QUERY_STD is not supported on the HDMI input */
	if (!s->input)
		return -ENODATA;

#ifdef TODO
	/*
	 * Query currently seen standard. Initial value of *std is
	 * V4L2_STD_ALL. This function should look something like this:
	 */
	get_signal_info();
	if (no_signal) {
		*std = 0;
		return 0;
	}
	/* Use signal information to reduce the number of possible standards */
	if (signal_has_525_lines)
		*std &= V4L2_STD_525_60;
	else
		*std &= V4L2_STD_625_50;
#endif
	return 0;
}

/* When ffmpeg/vlc opens v4l2 video device, it call this ioctl to get fps */
static int saa716x_cap_g_parm(struct file *file, void *fh, struct v4l2_streamparm *parm)
{
	struct saa716x_stream *s = video_drvdata(file);
	struct v4l2_fract fps;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;

	fps = v4l2_calc_timeperframe(&s->timings);
	parm->parm.capture.timeperframe.numerator = fps.numerator;
	parm->parm.capture.timeperframe.denominator = fps.denominator;
	parm->parm.capture.readbuffers = VIP_BUFFERS;
	
	return 0;
}

static int saa716x_cap_g_edid(struct file *file, void *fh, struct v4l2_edid *edid)
{
	struct saa716x_stream *s = video_drvdata(file);
	int ret;

	if (edid->pad >= 1)
		return -EINVAL;
	ret = v4l2_subdev_call(s->sd_receiver, pad, get_edid, edid);
	return ret;
}

static int saa716x_cap_s_edid(struct file *file, void *fh, struct v4l2_edid *edid)
{
	struct saa716x_stream *s = video_drvdata(file);
	int ret;

	if (edid->pad >= 1)
		return -EINVAL;
	ret = v4l2_subdev_call(s->sd_receiver, pad, set_edid, edid);
	return ret;
}

static int saa716x_cap_log_status(struct file *file, void *fh)
{
	struct saa716x_stream *s = video_drvdata(file);

	v4l2_subdev_call(s->sd_receiver, core, log_status);

	return v4l2_ctrl_log_status(file, fh);
}


/* The control handler. */
static int saa716x_cap_s_ctrl(struct v4l2_ctrl *ctrl)
{
	/*struct saa716x_stream *s =
		container_of(ctrl->handler, struct seton, ctrl_handler);*/

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		/* TODO: set brightness to ctrl->val */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops saa716x_cap_ctrl_ops = {
	.s_ctrl = saa716x_cap_s_ctrl,
};


static const struct v4l2_ioctl_ops saa716x_cap_ioctl_ops = {
	.vidioc_querycap = saa716x_cap_querycap,
	.vidioc_try_fmt_vid_cap = saa716x_cap_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = saa716x_cap_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = saa716x_cap_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = saa716x_cap_enum_fmt_vid_cap,

	.vidioc_s_std = saa716x_cap_s_std,
	.vidioc_g_std = saa716x_cap_g_std,
	.vidioc_querystd = saa716x_cap_querystd,

	.vidioc_s_dv_timings = saa716x_cap_s_dv_timings,
	.vidioc_g_dv_timings = saa716x_cap_g_dv_timings,
	.vidioc_query_dv_timings = saa716x_cap_query_dv_timings,
	.vidioc_enum_dv_timings = saa716x_cap_enum_dv_timings,
	.vidioc_dv_timings_cap = saa716x_cap_dv_timings_cap,

	.vidioc_enum_input = saa716x_cap_enum_input,
	.vidioc_s_input = saa716x_cap_s_input,
	.vidioc_g_input = saa716x_cap_g_input,

	.vidioc_g_parm = saa716x_cap_g_parm,
	.vidioc_s_edid = saa716x_cap_s_edid,
	.vidioc_g_edid = saa716x_cap_g_edid,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_log_status = saa716x_cap_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/*
 * The set of file operations. 
 */
/*
 * ECHDCAP: follow the source. Applications such as VLC never call
 * QUERY/S_DV_TIMINGS, so on open (while nobody is streaming) pick up the
 * mode the receiver currently detects; reopening the device after the
 * source changes resolution is then enough.
 */
static void saa716x_cap_follow_source(struct saa716x_stream *s)
{
	struct v4l2_dv_timings timings;

	if (s->saa716x->config->capture_config.subdev != SAA716x_SUBDEV_ECHDCAP_RX ||
	    s->input != 0 || !s->sd_receiver)
		return;
	if (mutex_lock_interruptible(&s->video_lock))
		return;
	if (vb2_is_busy(&s->queue))
		goto out;
	if (v4l2_subdev_call(s->sd_receiver, video, query_dv_timings, &timings))
		goto out;
	if (v4l2_match_dv_timings(&timings, &s->timings, 0, false) ||
	    !v4l2_valid_dv_timings(&timings, saa716x_timings_cap(s), NULL, NULL))
		goto out;
	if (v4l2_subdev_call(s->sd_receiver, video, s_dv_timings, &timings))
		goto out;
	s->timings = timings;
	saa716x_cap_fill_pix_format(s, &s->format);
	v4l2_print_dv_timings(s->saa716x->v4l2_dev.name, "source changed: ", &timings, false);
out:
	mutex_unlock(&s->video_lock);
}

static int saa716x_cap_open(struct file *file)
{
	int ret = v4l2_fh_open(file);

	if (!ret)
		saa716x_cap_follow_source(video_drvdata(file));
	return ret;
}

static const struct v4l2_file_operations saa716x_cap_fops = {
	.owner = THIS_MODULE,
	.open = saa716x_cap_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

/* ------------------------------------------------------------------
	v4l2-subdev 
   ------------------------------------------------------------------*/
/*
	VIP parameters for given dv-timings.
	VIP_FMT_TYPE2 should be set for interlaced video.
	VIC code definitions are here: include/uapi/linux/v4l2-dv-timings.h
*/
static int video_vip_get_stream_params_tda19978(struct saa716x_stream *s)
{
	struct vip_stream_params *params = &s->vip_params;
	struct v4l2_dv_timings *timings = &s->timings;
	u8 cea861_vic;

	if (timings->type == V4L2_DV_BT_656_1120 && (timings->bt.flags & V4L2_DV_FL_HAS_CEA861_VIC)){
		cea861_vic = timings->bt.cea861_vic;
	} else {
		return -1;
	}

	params->source_format = VIP_FMT_TYPE2;
	/* offset_x/offset_y decides PSU_WINDOW */
	switch (cea861_vic)
	{
	case 1: /* 640x480p59.94 */
		params->source_format = VIP_FMT_DEFAULT;
		params->bits = 16;
		params->samples = 640;
		params->lines = 480;
		params->pitch = 640 * 2;
		params->offset_x = 0;
		params->offset_y = 44;
		break;
	case 2: /* 720x480p59.94 */
		params->source_format = VIP_FMT_DEFAULT;
		params->bits = 16;
		params->samples = 720;
		params->lines = 480;
		params->pitch = 720 * 2;
		params->offset_x = 0;
		params->offset_y = 48;
		break;
	case 17: /* 720x576p50 */
		params->source_format = VIP_FMT_DEFAULT;
		params->bits = 16;
		params->samples = 720;
		params->lines = 576;
		params->pitch = 720 * 2;
		params->offset_x = 0;
		params->offset_y = 48;
		break;
	case 4: /* 1280x720p60 */
		params->bits = 16;
		params->samples = 1280;
		params->lines = 720;
		params->pitch = 1280 * 2;
		params->offset_x = 366;
		params->offset_y = 0;
		params->stream_flags = VIP_HD;
		break;
	case 19: /* 1280x720p50 */
		params->bits = 16;
		params->samples = 1280;
		params->lines = 720;
		params->pitch = 1280 * 2;
		params->offset_x = 696;
		params->offset_y = 0;
		params->stream_flags = VIP_HD;
		break;
	case 5: /* 1920x1080i60 */
		params->bits = 16;
		params->samples = 1920;
		params->lines = 1080;
		params->pitch = 1920 * 2;
		params->offset_x = 276;
		params->offset_y = 0;
		params->stream_flags = VIP_INTERLACED | VIP_HD;
		break;
	case 20: /* 1920x1080i50 */
		params->bits = 16;
		params->samples = 1920;
		params->lines = 1080;
		params->pitch = 1920 * 2;
		params->offset_x = 716;
		params->offset_y = 0;
		params->stream_flags = VIP_INTERLACED | VIP_HD;
		break;
	case 32: /* 1920x1080p24 */
	case 33: /* 1920x1080p25 */
	case 34: /* 1920x1080p30 */
		params->source_format = VIP_FMT_DEFAULT;
		params->bits = 16;
		params->samples = 1920;
		params->lines = 1080;
		params->pitch = 1920 * 2;
		params->offset_x = 0;
		params->offset_y = 44;
		params->stream_flags = VIP_HD;
		break;
	default:
		return -1;
	}

	if (s->format.field == V4L2_FIELD_ALTERNATE) {
		params->stream_flags |= VIP_FIELD_ALTERNATE;
	} else if (s->format.field == V4L2_FIELD_SEQ_TB) {
		params->stream_flags |= VIP_FIELD_SEQ;
	} else if (s->format.field == V4L2_FIELD_INTERLACED) {
		params->stream_flags |= (VIP_ODD_FIELD | VIP_EVEN_FIELD);
	}

	return 0;
}

static int video_vip_get_stream_params_adv7611(struct saa716x_stream *s)
{
	struct vip_stream_params *params = &s->vip_params;
	struct v4l2_dv_timings *timings = &s->timings;
	u8 cea861_vic;

	if (timings->type == V4L2_DV_BT_656_1120 && (timings->bt.flags & V4L2_DV_FL_HAS_CEA861_VIC)){
		cea861_vic = timings->bt.cea861_vic;
	} else {
		return -1;
	}

	params->source_format = VIP_FMT_DEFAULT;
	switch (cea861_vic)
	{
	case 1: /* 640x480p59.94 */
		params->bits = 16;
		params->samples = 640;
		params->lines = 480;
		params->pitch = 640 * 2;
		params->offset_x = 0;
		params->offset_y = 44;
		break;
	case 2: /* 720x480p60 */
		params->bits = 16;
		params->samples = 720;
		params->lines = 480;
		params->pitch = 720 * 2;
		params->offset_x = 0;
		params->offset_y = 48;
		break;
	case 17: /* 720x576p50 */
		params->bits = 16;
		params->samples = 720;
		params->lines = 576;
		params->pitch = 720 * 2;
		params->offset_x = 0;
		params->offset_y = 48;
		break;
	case 4:	 /* 1280x720p60 */
	case 19: /* 1280x720p50 */
		params->bits = 16;
		params->samples = 1280;
		params->lines = 720;
		params->pitch = 1280 * 2;
		params->offset_x = 0;
		params->offset_y = 30;
		params->stream_flags = VIP_HD;
		break;
	case 5:	 /* 1920x1080i60 */
		params->source_format = VIP_FMT_TYPE2;
		params->bits = 16;
		params->samples = 1920;
		params->lines = 1080;
		params->pitch = 1920 * 2;
		params->offset_x = 276;  // 0
		params->offset_y = 0; // 20
		params->stream_flags = VIP_INTERLACED | VIP_HD;
		break;
	case 20: /* 1920x1080i50 */
		params->source_format = VIP_FMT_TYPE2;
		params->bits = 16;
		params->samples = 1920;
		params->lines = 1080;
		params->pitch = 1920 * 2;
		params->offset_x = 716;
		params->offset_y = 0;
		params->stream_flags = VIP_INTERLACED | VIP_HD;
		break;
	case 32: /* 1920x1080p24 */
	case 33: /* 1920x1080p25 */
	case 34: /* 1920x1080p30 */ 
		params->bits = 16;
		params->samples = 1920;
		params->lines = 1080;
		params->pitch = 1920 * 2;
		params->offset_x = 0;
		params->offset_y = 45;
		params->stream_flags = VIP_HD;
		break;
	default:
		return -1;
	}

	if (s->format.field == V4L2_FIELD_ALTERNATE) {
		params->stream_flags |= VIP_FIELD_ALTERNATE;
	} else if (s->format.field == V4L2_FIELD_SEQ_TB) {
		params->stream_flags |= VIP_FIELD_SEQ;
	} else if (s->format.field == V4L2_FIELD_INTERLACED) {
		params->stream_flags |= (VIP_ODD_FIELD | VIP_EVEN_FIELD);
	}

	return 0;
}

/*
 * VIP window parameters for the ECHDCAP receiver (0x48).
 *
 * Values are taken from a register snapshot of the vendor Windows driver
 * while it was streaming 1920x1080p30 (VM passthrough, see TODO.md):
 *   VI_MODE=0xd0020000 VIN_FORMAT=0x01004201 WIN_XYSTART=0x00040000
 *   WIN_XYEND=0x07840437 PSU_FORMAT=0x000020a0 PSU_WINDOW=0x07800438
 *   INT_ENABLE=0x3ff
 * i.e. VIP_FMT_TYPE2, offset_x=4, offset_y=0, inclusive window end.
 * The same window origin is used for every progressive mode.
 */
static int video_vip_get_stream_params_echdcap(struct saa716x_stream *s)
{
	struct vip_stream_params *params = &s->vip_params;
	struct v4l2_bt_timings *bt = &s->timings.bt;

	if (s->timings.type != V4L2_DV_BT_656_1120 || bt->interlaced)
		return -1;

	/* The receiver output window starts at x=4, y=0 for every mode. */
	params->source_format = VIP_FMT_TYPE2;
	params->bits = 16;
	params->samples = bt->width;
	params->lines = bt->height;
	params->pitch = s->format.bytesperline;
	params->offset_x = 4;
	params->offset_y = 0;
	params->stream_flags = VIP_HD | VIP_WIN_END_INCL | VIP_PSU_FMT_NO_BIT31 |
			       VIP_RST_ON_ERR | VIP_INT_SEQBRK;
	params->stream_flags &= ~echdcap_flags_clear;

	if (s->format.field == V4L2_FIELD_ALTERNATE) {
		params->stream_flags |= VIP_FIELD_ALTERNATE;
	} else if (s->format.field == V4L2_FIELD_SEQ_TB) {
		params->stream_flags |= VIP_FIELD_SEQ;
	} else if (s->format.field == V4L2_FIELD_INTERLACED) {
		params->stream_flags |= (VIP_ODD_FIELD | VIP_EVEN_FIELD);
	}

	return 0;
}

/* Subdev & Platform data */
static struct adv76xx_platform_data adv7611_pdata = {
	.disable_cable_det_rst = 0,
	.int1_config = ADV76XX_INT1_CONFIG_ACTIVE_LOW,
	.alt_gamma = 0,
	.blank_data = 1,
	.insert_av_codes = 1,
	.replicate_av_codes = 0,
	.inv_hs_pol = 1, 
	.inv_vs_pol = 1,
	.inv_llc_pol = 1,
	.dr_str_data = ADV76XX_DR_STR_HIGH,
	.dr_str_clk = ADV76XX_DR_STR_HIGH,
	.dr_str_sync = ADV76XX_DR_STR_HIGH,
	.hdmi_free_run_mode = 0,
};

static struct i2c_board_info adv7611_info = {
	.type = "adv7611",
	.addr = 0x4c,
	.platform_data = &adv7611_pdata,
};

static struct i2c_board_info tda19978_info = {
	.type = "tda19978",
	.addr = 0x4c,
};

static struct mst3367_platform_data mst3367_pdata = {
	.some_value = 0,
};

static struct i2c_board_info mst3367_info = {
	.type = "mst3367",
	.addr = 0x4e,
	.platform_data = &mst3367_pdata,
};

static struct i2c_board_info echdcap_rx_info = {
	.type = "echdcap_rx",
	.addr = ECHDCAP_RX_I2C_ADDR,
};

/*
	Register subdevice and initialize it
*/
static int saa716x_subdev_init(struct saa716x_dev *saa716x)
{
	struct saa716x_stream *s = &saa716x->saa716x_stream[0];
	struct saa716x_i2c *i2c;
	struct i2c_adapter *i2cadapter;
	struct i2c_board_info *board_info = NULL;
	struct v4l2_subdev *sd;
	enum saa716x_capture_subdev sd_type = saa716x->config->capture_config.subdev;

	int err = 0;

	if (sd_type == SAA716x_SUBDEV_MST3367 || sd_type == SAA716x_SUBDEV_ECHDCAP_RX)
		i2c = &saa716x->i2c[1]; /* SAA716x I2C Core 1 */
	else
		i2c = &saa716x->i2c[0];

	i2cadapter = &i2c->i2c_adapter;

	printk(KERN_INFO
		"saa716x_subdev_init: type=%d, using I2C adapter %d (%s)\n",
		sd_type, i2cadapter->nr, i2cadapter->name);

    switch (sd_type)
	{
	case SAA716x_SUBDEV_TDA19978:
		board_info = &tda19978_info;
		break;
	case SAA716x_SUBDEV_ADV7611:
		board_info = &adv7611_info;
		break;
	case SAA716x_SUBDEV_ADV7611_AD9983:
		board_info = &adv7611_info;
		break;
	case SAA716x_SUBDEV_MST3367:
		board_info = &mst3367_info;
		break;
	case SAA716x_SUBDEV_ECHDCAP_RX:
		board_info = &echdcap_rx_info;
		break;
	default:
		break;
	}
	if (!board_info)
		return -ENODEV;

	sd = v4l2_i2c_new_subdev_board(&saa716x->v4l2_dev, i2cadapter, board_info, NULL);
	if (sd == NULL) {
		printk("%s: v4l2 subdev register error", __func__);
		return -ENODEV;
	}
	s->sd_receiver = sd;

	switch (sd_type)
	{
		case SAA716x_SUBDEV_TDA19978: {
			video_vip_get_stream_params_tda19978(s);
			
			struct v4l2_edid tda19978_edid = {
				.pad = 0,
				.start_block = 0,
				.blocks = 2,
				.edid = edid,
			};
			err = v4l2_subdev_call(sd, pad, set_edid, &tda19978_edid);
			if (err)
				return err;
			break;
		}
		case SAA716x_SUBDEV_ADV7611_AD9983: {
			/* AD9983 power down */
			u8 pw_down[] = {0x1E, 0x08};
			struct i2c_msg msg[] = {
				{ .addr = 0x4d, .flags = 0, .buf = pw_down, .len = sizeof(pw_down) }, 
			};
			err = i2c_transfer(i2cadapter, msg, 1);
			fallthrough;
		}
		case SAA716x_SUBDEV_ADV7611: {
			video_vip_get_stream_params_adv7611(s);
			
			/* enable DLL_LLC mux */
			u8 dll_llc_cfg[] = {0x19, 0x83};
			u8 dll_llc_mux[] = {0x33, 0x40};
			struct i2c_msg msg1[] = {
				{ .addr = 0x4c,	.flags = 0,	.buf = dll_llc_cfg,	.len = sizeof (dll_llc_cfg) },
			};
			struct i2c_msg msg2[] = {
				{ .addr = 0x4c,	.flags = 0,	.buf = dll_llc_mux,	.len = sizeof (dll_llc_mux) },
			};

			err = i2c_transfer(i2cadapter, msg1, 1);
			err = i2c_transfer(i2cadapter, msg2, 1);

			struct v4l2_subdev_format adv7611_fmt = {
				.pad = ADV7611_PAD_SOURCE,
				.which = V4L2_SUBDEV_FORMAT_ACTIVE,
				.format.code = MEDIA_BUS_FMT_YUYV8_1X16,
			};
			struct v4l2_edid adv7611_edid = {
				.pad = ADV76XX_PAD_HDMI_PORT_A,
				.start_block = 0,
				.blocks = 2,
				.edid = edid,
			};
			
			err = v4l2_subdev_call(sd, video, s_routing, ADV76XX_PAD_HDMI_PORT_A, 0, 0);
			if (err)
				return err;
			err = v4l2_subdev_call(sd, pad, set_edid, &adv7611_edid);
			if (err)
				return err;
			err = v4l2_subdev_call(sd, pad, set_fmt, NULL, &adv7611_fmt);
			if (err)
				return err;
			
			break;
		}
		default:
			break;
	}
	if (sd_type != SAA716x_SUBDEV_MST3367 && sd_type != SAA716x_SUBDEV_ECHDCAP_RX) {
		err = v4l2_subdev_call(sd, video, s_dv_timings, &s->timings);
	if (err)
		return err;
	}
	return 0;
}

/*
 * StarTech ECHDCAP board bring-up. GPIO_WR/GREG end values are the ones the
 * vendor Windows driver leaves while streaming (register snapshot, TODO.md).
 * The reset pulse on GPIO 16 re-initialises the 0x48 receiver; with
 * echdcap_board_init=0 it is skipped so a receiver configured by another
 * driver keeps its state.
 */
#define ECHDCAP_GPIO_OEN	0xfbec3f93
#define ECHDCAP_GPIO_WR		0x0001c024

/*
 * ECHDCAP board MCU at 7-bit 0x51 on a bit-banged I2C bus (GPIO14 = SDA,
 * GPIO15 = SCL). Traced from the vendor driver: a challenge write to 0x13,
 * a 7-byte read ~130 ms later, then 0x11=b4, 0x12=5a, 0x10=5a, and 0x12=5a
 * repeated every 10-15 s. SA7160.X64.SYS: the 0x13 reply is only compared
 * against a hash inside the driver (anti-clone check); 0x11 is the watchdog
 * timeout in seconds (CustomSoftwareWatchdogDuration, 180), 0x10=5a arms it
 * and 0x12=5a kicks it. The vendor driver never disarms it; once kicks stop
 * the MCU fires after the timeout (suspected cause of a host hang after the
 * Windows VM released the card). On unload we write 0x10=00, which is a guess
 * at "disarm", not something seen in the vendor driver.
 */
#define ECHDCAP_GPIO_SDA	BIT(14)
#define ECHDCAP_GPIO_SCL	BIT(15)
#define ECHDCAP_MCU_ADDR	0x51
#define ECHDCAP_MCU_KICK	(10 * HZ)

struct echdcap_mcu {
	struct saa716x_dev		*saa716x;
	struct i2c_adapter		adap;
	struct i2c_algo_bit_data	bit;
	struct delayed_work		kick;
};

static void echdcap_gpio_update(struct saa716x_dev *saa716x, u32 reg, u32 mask, u32 val)
{
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&saa716x->gpio_lock, flags);
	v = SAA716x_EPRD(GPIO, reg);
	SAA716x_EPWR(GPIO, reg, (v & ~mask) | (val & mask));
	spin_unlock_irqrestore(&saa716x->gpio_lock, flags);
}

/* SDA is open-drain: drive low, or release (OEN=1 is input) to the pull-up. */
static void echdcap_mcu_setsda(void *data, int state)
{
	struct saa716x_dev *saa716x = ((struct echdcap_mcu *)data)->saa716x;

	if (state) {
		echdcap_gpio_update(saa716x, GPIO_OEN, ECHDCAP_GPIO_SDA, ECHDCAP_GPIO_SDA);
		echdcap_gpio_update(saa716x, GPIO_WR, ECHDCAP_GPIO_SDA, ECHDCAP_GPIO_SDA);
	} else {
		echdcap_gpio_update(saa716x, GPIO_WR, ECHDCAP_GPIO_SDA, 0);
		echdcap_gpio_update(saa716x, GPIO_OEN, ECHDCAP_GPIO_SDA, 0);
	}
}

/* SCL is push-pull, as the vendor driver does it. */
static void echdcap_mcu_setscl(void *data, int state)
{
	struct saa716x_dev *saa716x = ((struct echdcap_mcu *)data)->saa716x;

	echdcap_gpio_update(saa716x, GPIO_WR, ECHDCAP_GPIO_SCL, state ? ECHDCAP_GPIO_SCL : 0);
}

static int echdcap_mcu_getsda(void *data)
{
	struct saa716x_dev *saa716x = ((struct echdcap_mcu *)data)->saa716x;

	return !!(SAA716x_EPRD(GPIO, GPIO_RD) & ECHDCAP_GPIO_SDA);
}

static int echdcap_mcu_write(struct echdcap_mcu *mcu, const u8 *buf, int len)
{
	struct i2c_msg msg = { .addr = ECHDCAP_MCU_ADDR, .len = len, .buf = (u8 *)buf };
	int ret, i;

	for (i = 0; i < 3; i++) {
		ret = i2c_transfer(&mcu->adap, &msg, 1);
		if (ret == 1)
			return 0;
		msleep(20);
	}
	return ret < 0 ? ret : -EIO;
}

static void echdcap_mcu_kick(struct work_struct *work)
{
	struct echdcap_mcu *mcu = container_of(to_delayed_work(work), struct echdcap_mcu, kick);
	static const u8 kick[] = { 0x12, 0x5a };
	int ret;

	ret = echdcap_mcu_write(mcu, kick, sizeof(kick));
	if (ret)
		dev_warn(&mcu->saa716x->pdev->dev, "ECHDCAP: MCU kick failed (%d)\n", ret);
	schedule_delayed_work(&mcu->kick, ECHDCAP_MCU_KICK);
}

static void echdcap_mcu_init(struct saa716x_dev *saa716x)
{
	static const u8 cfg[][2] = { { 0x11, 0xb4 }, { 0x12, 0x5a }, { 0x10, 0x5a } };
	struct device *dev = &saa716x->pdev->dev;
	struct echdcap_mcu *mcu;
	struct i2c_msg rd;
	u8 chal[5], resp[7];
	int ret, i;

	mcu = kzalloc(sizeof(*mcu), GFP_KERNEL);
	if (!mcu)
		return;
	mcu->saa716x = saa716x;
	mcu->bit.data = mcu;
	mcu->bit.setsda = echdcap_mcu_setsda;
	mcu->bit.setscl = echdcap_mcu_setscl;
	mcu->bit.getsda = echdcap_mcu_getsda;
	mcu->bit.udelay = 10;
	mcu->bit.timeout = msecs_to_jiffies(100);
	mcu->adap.owner = THIS_MODULE;
	mcu->adap.algo_data = &mcu->bit;
	mcu->adap.dev.parent = dev;
	strscpy(mcu->adap.name, "ECHDCAP MCU (GPIO14/15)", sizeof(mcu->adap.name));
	INIT_DELAYED_WORK(&mcu->kick, echdcap_mcu_kick);

	ret = i2c_bit_add_bus(&mcu->adap);
	if (ret) {
		dev_err(dev, "ECHDCAP: MCU i2c bus failed (%d)\n", ret);
		kfree(mcu);
		return;
	}
	saa716x->echdcap_mcu = mcu;

	chal[0] = 0x13;
	get_random_bytes(&chal[1], 4);
	ret = echdcap_mcu_write(mcu, chal, sizeof(chal));
	if (ret)
		dev_warn(dev, "ECHDCAP: MCU challenge failed (%d)\n", ret);
	msleep(130);

	rd.addr = ECHDCAP_MCU_ADDR;
	rd.flags = I2C_M_RD;
	rd.len = sizeof(resp);
	rd.buf = resp;
	ret = i2c_transfer(&mcu->adap, &rd, 1);
	if (ret == 1)
		dev_info(dev, "ECHDCAP: MCU challenge %*ph -> %*ph\n", 4, &chal[1], 7, resp);
	else
		dev_warn(dev, "ECHDCAP: MCU response read failed (%d)\n", ret);

	for (i = 0; i < ARRAY_SIZE(cfg); i++) {
		msleep(20);
		ret = echdcap_mcu_write(mcu, cfg[i], 2);
		if (ret)
			dev_warn(dev, "ECHDCAP: MCU write %02x=%02x failed (%d)\n",
				 cfg[i][0], cfg[i][1], ret);
	}
	schedule_delayed_work(&mcu->kick, ECHDCAP_MCU_KICK);
}

static void echdcap_mcu_exit(struct saa716x_dev *saa716x)
{
	struct echdcap_mcu *mcu = saa716x->echdcap_mcu;
	static const u8 disarm[] = { 0x10, 0x00 };

	if (!mcu)
		return;
	cancel_delayed_work_sync(&mcu->kick);
	echdcap_mcu_write(mcu, disarm, sizeof(disarm));
	i2c_del_adapter(&mcu->adap);
	kfree(mcu);
	saa716x->echdcap_mcu = NULL;
}

static void echdcap_board_setup(struct saa716x_dev *saa716x)
{
	if (echdcap_board_init) {
		SAA716x_EPWR(GPIO, GPIO_WR_MODE, 0);
		SAA716x_EPWR(GPIO, GPIO_WR, 0);
		SAA716x_EPWR(GPIO, GPIO_OEN, ECHDCAP_GPIO_OEN);
		SAA716x_EPWR(GPIO, GPIO_WR, 0x00010004);
		msleep(50);
		SAA716x_EPWR(GPIO, GPIO_WR, 0x00000004);
		msleep(50);
		SAA716x_EPWR(GPIO, GPIO_WR, 0x00010004);
		msleep(50);
	}
	/* Output value first, then direction, so no pin glitches low. */
	SAA716x_EPWR(GPIO, GPIO_WR_MODE, 0);
	SAA716x_EPWR(GPIO, GPIO_WR, echdcap_gpio_wr_extra ? echdcap_gpio_wr_extra : ECHDCAP_GPIO_WR);
	SAA716x_EPWR(GPIO, GPIO_OEN, ECHDCAP_GPIO_OEN);

	SAA716x_EPWR(GREG, GREG_PMCSR_DATA_2, 0x00000c00);
	SAA716x_EPWR(GREG, GREG_VIDEO_IN_CTRL, 0);
	if (echdcap_vi_ctrl != 0x06080F8C)
		SAA716x_EPWR(GREG, GREG_VI_CTRL, echdcap_vi_ctrl);

	printk(KERN_INFO "ECHDCAP: board init%s: GPIO WR=%08x OEN=%08x GREG VIDEO_IN_CTRL=%08x +0c=%08x\n",
	       echdcap_board_init ? "" : " (no reset pulse)",
	       SAA716x_EPRD(GPIO, GPIO_WR), SAA716x_EPRD(GPIO, GPIO_OEN),
	       SAA716x_EPRD(GREG, GREG_VIDEO_IN_CTRL), SAA716x_EPRD(GREG, GREG_PMCSR_DATA_2));

	if (echdcap_mcu)
		echdcap_mcu_init(saa716x);
}

/*
 * The initial setup of this device instance. Note that the initial state of
 * the driver should be complete. So the initial format, standard, timings
 * and video input should all be initialized to some reasonable value.
 */
int saa716x_v4l2_init(struct saa716x_dev *saa716x)
{
	static const struct v4l2_dv_timings timings_def = V4L2_DV_BT_CEA_1280X720P60;
	struct saa716x_stream *stream = &saa716x->saa716x_stream[0];
	struct video_device *vdev;
	struct v4l2_ctrl_handler *hdl;
	struct vb2_queue *q;
	int ret;

	//saa716x->num_adapters = 1;	
	mutex_init(&stream->video_lock);
	spin_lock_init(&stream->qlock);
	INIT_DELAYED_WORK(&stream->restart_work, saa716x_cap_restart_work);
	INIT_DELAYED_WORK(&stream->watch_work, saa716x_cap_watch_work);

	stream->saa716x = saa716x;
	stream->vip_port = saa716x->config->capture_config.vip_port;
	/* Default input is HDMI */
	stream->input = 0;

	/* Fill in the initial format-related settings */
	stream->timings = timings_def;
	saa716x_cap_fill_pix_format(stream, &stream->format);

	/* Initialize the top-level structure */
	ret = v4l2_device_register(&saa716x->pdev->dev, &saa716x->v4l2_dev);
	snprintf(saa716x->v4l2_dev.name, sizeof(saa716x->v4l2_dev.name), "saa716x");
	
	/* Make subdev ctrl accessible */
	hdl = &stream->ctrl_handler;
	v4l2_ctrl_handler_init(hdl, 0);
	if (hdl->error) {
		ret = hdl->error;
		goto free_hdl;
	}
	saa716x->v4l2_dev.ctrl_handler = hdl;

	/* Initialize the vb2 queue */
	INIT_LIST_HEAD(&stream->buf_list);
	q = &stream->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	q->dev = &saa716x->pdev->dev;
	q->drv_priv = stream;
	q->buf_struct_size = sizeof(struct saa716x_cap_buffer);
	q->ops = &saa716x_cap_qops;
	q->mem_ops = &vb2_dma_sg_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	/*
	 * Assume that this DMA engine needs to have at least two buffers
	 * available before it can be started. The start_streaming() op
	 * won't be called until at least this many buffers are queued up.
	 */
	q->min_buffers_needed = VIP_BUFFERS;
	/*
	 * The serialization lock for the streaming ioctls. This is the same
	 * as the main serialization lock, but if some of the non-streaming
	 * ioctls could take a long time to execute, then you might want to
	 * have a different lock here to prevent VIDIOC_DQBUF from being
	 * blocked while waiting for another action to finish. This is
	 * generally not needed for PCI devices, but USB devices usually do
	 * want a separate lock here.
	 */
	q->lock = &stream->video_lock;
	q->gfp_flags = GFP_DMA32;
	ret = vb2_queue_init(q);
	if (ret)
		goto free_hdl;

	/* Initialize the video_device structure */
	vdev = &stream->vdev;
	strlcpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));
	/*
	 * There is nothing to clean up, so release is set to an empty release
	 * function. The release callback must be non-NULL.
	 */
	vdev->release = video_device_release_empty;
	vdev->fops = &saa716x_cap_fops,
	vdev->ioctl_ops = &saa716x_cap_ioctl_ops,
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
	/*
	 * The main serialization lock. All ioctls are serialized by this
	 * lock. Exception: if q->lock is set, then the streaming ioctls
	 * are serialized by that separate lock.
	 */
	vdev->lock = &stream->video_lock;
	vdev->queue = q;
	vdev->v4l2_dev = &saa716x->v4l2_dev;
	/* Supported SDTV standards, if any */
	vdev->tvnorms = SAA716X_TVNORMS;
	video_set_drvdata(vdev, stream);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto free_hdl;

	if (saa716x->pdev->subsystem_vendor == 0xf50a &&
	    saa716x->pdev->subsystem_device == 0x12ab)
		echdcap_board_setup(saa716x);

	saa716x_subdev_init(saa716x);

	dev_info(&saa716x->pdev->dev, "SAA716x Capture V4L2 Driver loaded\n");
	return 0;

free_hdl:
	v4l2_ctrl_handler_free(&stream->ctrl_handler);
	v4l2_device_unregister(&saa716x->v4l2_dev);
	return ret;
}
EXPORT_SYMBOL(saa716x_v4l2_init);

int saa716x_v4l2_exit(struct saa716x_dev *saa716x)
{
	struct saa716x_stream *stream = &saa716x->saa716x_stream[0];

	echdcap_mcu_exit(saa716x);
	video_unregister_device(&stream->vdev);
	v4l2_ctrl_handler_free(&stream->ctrl_handler);
	v4l2_device_unregister(&saa716x->v4l2_dev);
	mutex_destroy(&saa716x->adap_lock);

	return 0;
}
EXPORT_SYMBOL(saa716x_v4l2_exit);

