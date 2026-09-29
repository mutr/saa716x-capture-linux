#ifndef __SAA716x_VIP_H
#define __SAA716x_VIP_H

#include "saa716x_dma.h"

#define VIP_BUFFERS	8

/*
 * Stream port flags
 */
enum vip_stream_flags {
	VIP_ODD_FIELD		= 0x0001,
	VIP_EVEN_FIELD		= 0x0002,
	VIP_FIELD_SEQ		= 0x0004,
	VIP_FIELD_ALTERNATE	= 0x0008,
	VIP_INTERLACED	= 0x0010,
	VIP_HD			= 0x0100,
	VIP_NO_SCALER		= 0x1000,
	VIP_WIN_END_INCL	= 0x2000,	/* WIN_XYEND line = offset_y + lines - 1 */
	VIP_PSU_FMT_NO_BIT31	= 0x4000,	/* PSU_FORMAT = 0x000020A0 */
	VIP_RST_ON_ERR		= 0x8000,	/* VI_MODE |= RST_ON_ERR */
	VIP_INT_SEQBRK		= 0x10000	/* also enable FID/WIN sequence-break IRQs */
};

/*
 * Source format type
 */
enum vip_source_format {
	VIP_FMT_DEFAULT		= 0x1,
	VIP_FMT_TYPE2		= 0x2,
};

/*
 * Stream port parameters
 * bits: Bits per sample
 * samples: samples perline
 * lines: number of lines
 * pitch: stream pitch in bytes
 * offset_x: offset to first valid pixel
 * offset_y: offset to first valid line
 */
struct vip_stream_params {
	u32			bits;
	u32			samples;
	u32			lines;
	s32			pitch;
	u32			offset_x;
	u32			offset_y;
	enum vip_stream_flags	stream_flags;
	enum vip_source_format	source_format;
};

struct saa716x_vip_stream_port {
	u8			dual_channel;
	u8			read_index;
	u8			dma_channel[2];
	struct saa716x_dmabuf	dma_buf[2][VIP_BUFFERS];
	struct saa716x_dev	*saa716x;
	struct tasklet_struct	tasklet;
};

extern void saa716x_vipint_disable(struct saa716x_dev *saa716x);
extern void saa716x_vip_disable(struct saa716x_dev *saa716x);
extern int saa716x_vip_get_write_index(struct saa716x_dev *saa716x, int port);
extern int saa716x_vip_start(struct saa716x_dev *saa716x, int port,
			    int one_shot,
			    struct vip_stream_params *stream_params);
extern int saa716x_vip_stop(struct saa716x_dev *saa716x, int port);
extern int saa716x_vip_init(struct saa716x_dev *saa716x, int port,
				void (*worker)(unsigned long));
extern int saa716x_vip_init2(struct saa716x_dev *saa716x, int port,
				void (*worker)(unsigned long));
extern int saa716x_vip_exit(struct saa716x_dev *saa716x, int port);
extern int saa716x_vip_exit2(struct saa716x_dev *saa716x, int port);

#endif /* __SAA716x_VIP_H */
