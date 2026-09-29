// SPDX-License-Identifier: GPL-2.0-only

#include "saa716x_priv.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <sound/control.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#define SAA716X_ALSA_CAP_AI_PORT		0
#define SAA716X_ALSA_PERIOD_TDA19978	0x640
#define SAA716X_ALSA_PERIOD_ADV7611	0x5c0

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static int enable[SNDRV_CARDS] = { 1, [1 ... (SNDRV_CARDS - 1)] = 1 };

struct snd_saa716x_card {
	struct saa716x_dev *saa716x;
	struct snd_card *sc;
	struct snd_pcm_substream *capture_pcm_substream;
	spinlock_t slock;
	unsigned int hwptr_done_capture;
	bool capture_running;
};

unsigned int snd_saa716x_periods(struct saa716x_dev *saa716x)
{
	enum saa716x_capture_subdev sd_type = saa716x->config->capture_config.subdev;

	if (sd_type == SAA716x_SUBDEV_TDA19978) {
		return SAA716X_ALSA_PERIOD_TDA19978;
	}
	return SAA716X_ALSA_PERIOD_ADV7611;
}

static struct snd_pcm_hardware snd_saa716x_capture_hdmi(struct saa716x_dev *saa716x)
{
	unsigned int period;

	period = snd_saa716x_periods(saa716x);

	struct snd_pcm_hardware hw = {
		.info = SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_MMAP_VALID,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rates = SNDRV_PCM_RATE_44100,
		.rate_min = 44100,
		.rate_max = 44100,
		.channels_min = 2,
		.channels_max = 2,
		.buffer_bytes_max = period * 4 * AIP_BUFFERS,
		.period_bytes_min = period * 4,
		.period_bytes_max = period * 4,
		.periods_min = AIP_BUFFERS,
		.periods_max = AIP_BUFFERS,
	};

	return hw;
}

static void snd_saa716x_announce_pcm_data(struct snd_saa716x_card *saa716x_sc,
					struct saa716x_aip_stream_port *aip)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned long flags;

	spin_lock_irqsave(&saa716x_sc->slock, flags);
	substream = saa716x_sc->capture_pcm_substream;
	if (!saa716x_sc->capture_running || !substream) {
		spin_unlock_irqrestore(&saa716x_sc->slock, flags);
		return;
	}

	runtime = substream->runtime;
	if (!runtime || !runtime->dma_area) {
		spin_unlock_irqrestore(&saa716x_sc->slock, flags);
		return;
	}
	
	memcpy(runtime->dma_area + (frames_to_bytes(runtime, runtime->period_size) * aip->read_index),
			aip->dma_buf[aip->read_index].mem_virt,
			frames_to_bytes(runtime, runtime->period_size));
	/* hwptr is frame position, not bytes position */
	saa716x_sc->hwptr_done_capture = ((aip->read_index + 1) & 7) * runtime->period_size;

	spin_unlock_irqrestore(&saa716x_sc->slock, flags);
	
	snd_pcm_period_elapsed(substream);
}

void saa716x_alsa_deliver_buffer(struct saa716x_dev *saa716x)
{
	struct snd_saa716x_card *saa716x_sc = saa716x->alsa;

	if (!saa716x_sc)
		return;

	snd_saa716x_announce_pcm_data(saa716x_sc, &saa716x->aip[SAA716X_ALSA_CAP_AI_PORT]);
}

static int snd_saa716x_pcm_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_saa716x_card *saa716x_sc = snd_pcm_substream_chip(substream);
	unsigned long flags;

	printk("%s: called", __func__);

	runtime->hw = snd_saa716x_capture_hdmi(saa716x_sc->saa716x);
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	spin_lock_irqsave(&saa716x_sc->slock, flags);
	saa716x_sc->capture_pcm_substream = substream;
	spin_unlock_irqrestore(&saa716x_sc->slock, flags);

	return 0;
}

static int snd_saa716x_pcm_capture_close(struct snd_pcm_substream *substream)
{
	struct snd_saa716x_card *saa716x_sc = snd_pcm_substream_chip(substream);
	unsigned long flags;

	printk("%s: called", __func__);

	if (saa716x_sc->capture_running)
		saa716x_aip_stop(saa716x_sc->saa716x, SAA716X_ALSA_CAP_AI_PORT);

	spin_lock_irqsave(&saa716x_sc->slock, flags);
	saa716x_sc->capture_running = false;
	saa716x_sc->capture_pcm_substream = NULL;
	spin_unlock_irqrestore(&saa716x_sc->slock, flags);

	return 0;
}

static int snd_saa716x_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_saa716x_card *saa716x_sc = snd_pcm_substream_chip(substream);
	unsigned long flags;
	
	printk("%s: called", __func__);

	spin_lock_irqsave(&saa716x_sc->slock, flags);
	saa716x_sc->hwptr_done_capture = 0;
	spin_unlock_irqrestore(&saa716x_sc->slock, flags);

	return 0;
}

static int snd_saa716x_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_saa716x_card *saa716x_sc = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aip_stream_params params = {
		.bits = snd_pcm_format_width(runtime->format),
		.sample_rate = runtime->rate,
		.ai_size = snd_saa716x_periods(saa716x_sc->saa716x),
	};
	unsigned long flags;
	int ret = 0;

	printk("%s: called", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = saa716x_aip_start(saa716x_sc->saa716x,
					SAA716X_ALSA_CAP_AI_PORT, &params);
		if (!ret) {
			spin_lock_irqsave(&saa716x_sc->slock, flags);
			saa716x_sc->capture_running = true;
			spin_unlock_irqrestore(&saa716x_sc->slock, flags);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		spin_lock_irqsave(&saa716x_sc->slock, flags);
		saa716x_sc->capture_running = false;
		spin_unlock_irqrestore(&saa716x_sc->slock, flags);
		ret = saa716x_aip_stop(saa716x_sc->saa716x, SAA716X_ALSA_CAP_AI_PORT);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static snd_pcm_uframes_t
snd_saa716x_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_saa716x_card *saa716x_sc = snd_pcm_substream_chip(substream);
	unsigned long flags;
	snd_pcm_uframes_t hwptr_done;

	spin_lock_irqsave(&saa716x_sc->slock, flags);
	hwptr_done = saa716x_sc->hwptr_done_capture;
	spin_unlock_irqrestore(&saa716x_sc->slock, flags);

	return hwptr_done;
}

static const struct snd_pcm_ops snd_saa716x_pcm_capture_ops = {
	.open = snd_saa716x_pcm_capture_open,
	.close = snd_saa716x_pcm_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.prepare = snd_saa716x_pcm_prepare,
	.trigger = snd_saa716x_pcm_trigger,
	.pointer = snd_saa716x_pcm_pointer,
};

static int snd_saa716x_pcm_create(struct snd_saa716x_card *saa716x_sc)
{
	struct snd_pcm *pcm;
	int ret;

	ret = snd_pcm_new(saa716x_sc->sc, "saa716x HDMI Audio", 0, 0, 1, &pcm);
	if (ret)
		return ret;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_saa716x_pcm_capture_ops);
	//snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_VMALLOC, NULL, 0x2000*8, 0x2000*8);
        snd_pcm_lib_preallocate_pages_for_all(
          pcm,
          SNDRV_DMA_TYPE_CONTINUOUS,
          snd_dma_continuous_data(GFP_KERNEL),
          0x2000 * 8,
          0x2000 * 8
        );
	pcm->private_data = saa716x_sc;
	strscpy(pcm->name, "saa716x HDMI Capture", sizeof(pcm->name));

	return 0;
}

static void snd_saa716x_card_set_names(struct snd_saa716x_card *saa716x_sc)
{
	struct snd_card *sc = saa716x_sc->sc;
	struct saa716x_dev *saa716x = saa716x_sc->saa716x;

	strscpy(sc->driver, "saa716x", sizeof(sc->driver));
	snprintf(sc->shortname, sizeof(sc->shortname), "saa716x-%d",
		 saa716x->num);
	snprintf(sc->longname, sizeof(sc->longname), "%s HDMI Audio",
		 saa716x->config->capture_config.board_name);
}

static void snd_saa716x_card_free(struct snd_saa716x_card *saa716x_sc)
{
	if (!saa716x_sc)
		return;

	saa716x_sc->saa716x->alsa = NULL;
	kfree(saa716x_sc);
}

static void snd_saa716x_card_private_free(struct snd_card *sc)
{
	if (!sc)
		return;

	snd_saa716x_card_free(sc->private_data);
	sc->private_data = NULL;
	sc->private_free = NULL;
}

static int snd_saa716x_card_create(struct saa716x_dev *saa716x,
				   struct snd_card *sc,
				   struct snd_saa716x_card **saa716x_sc)
{
	*saa716x_sc = kzalloc(sizeof(struct snd_saa716x_card), GFP_KERNEL);
	if (!*saa716x_sc)
		return -ENOMEM;

	(*saa716x_sc)->saa716x = saa716x;
	(*saa716x_sc)->sc = sc;
	spin_lock_init(&(*saa716x_sc)->slock);

	sc->private_data = *saa716x_sc;
	sc->private_free = snd_saa716x_card_private_free;

	return 0;
}

int saa716x_alsa_init(struct saa716x_dev *saa716x)
{
	struct snd_card *sc = NULL;
	struct snd_saa716x_card *saa716x_sc;
	unsigned int devno = min_t(unsigned int, saa716x->num, SNDRV_CARDS - 1);
	int ret;

	if (!enable[devno])
		return 0;

	ret = snd_card_new(&saa716x->pdev->dev, index[devno], id[devno],
			   THIS_MODULE, 0, &sc);
	if (ret)
		return ret;

	ret = snd_saa716x_card_create(saa716x, sc, &saa716x_sc);
	if (ret)
		goto err_free_card;

	snd_saa716x_card_set_names(saa716x_sc);

	ret = snd_saa716x_pcm_create(saa716x_sc);
	if (ret)
		goto err_free_card;

	saa716x->alsa = saa716x_sc;

	ret = snd_card_register(sc);
	if (ret) {
		saa716x->alsa = NULL;
		goto err_free_card;
	}

	return 0;

err_free_card:
	snd_card_free(sc);
	return ret;
}

void saa716x_alsa_exit(struct saa716x_dev *saa716x)
{
	struct snd_saa716x_card *saa716x_sc = saa716x->alsa;

	if (saa716x_sc)
		snd_card_free(saa716x_sc->sc);
	saa716x->alsa = NULL;
}
