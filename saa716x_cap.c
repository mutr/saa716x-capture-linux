#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/version.h>

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <linux/i2c.h>

#include "saa716x_mod.h"

#include "saa716x_gpio_reg.h"
#include "saa716x_greg_reg.h"
#include "saa716x_msi_reg.h"

#include "saa716x_i2c.h"
#include "saa716x_msi.h"
#include "saa716x_cap.h"
#include "saa716x_gpio.h"
#include "saa716x_rom.h"
#include "saa716x_spi.h"
#include "saa716x_priv.h"
#include "saa716x_v4l2.h"
#include "saa716x_debugfs.h"

unsigned int verbose = 4;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "verbose startup messages, default is 1 (ERROR)");

unsigned int int_type = MODE_MSI;
module_param(int_type, int, 0644);
MODULE_PARM_DESC(int_type, "force Interrupt Handler type: 0=INT-A, 1=MSI, 2=MSI-X. default INT-A mode");

#define DRIVER_NAME	"SAA716x Capture"

static void video_vip_worker(unsigned long data);
static void audio_aip_worker(unsigned long data);
void saa716x_irq_work_handler(struct work_struct *work);

/*
	1~4 HDMI input.
	DRECAP DC-HB1, DC-HC1
	Regia ONE, Regia TWO
	KEIAN DM626 H3
	THANKO HDMVC4UC
*/
static struct saa716x_capture_config generic_tda19978 = {
	.board_name = "HDMI(Generic TDA19978)",
	.vip_port 	= 0,
	.vi_ctrl 	= 0x06080F8C,
	.subdev 	= SAA716x_SUBDEV_TDA19978,
};

/*
	1 HDMI input.
	SKNet Monstar X3
*/
static struct saa716x_capture_config sknet = {
	.board_name = "HDMI(SKNet)",
	.vip_port 	= 0,
	.vi_ctrl 	= 0x04080F89,
	.subdev 	= SAA716x_SUBDEV_ADV7611,
};

/*
	1 HDMI + 1 YPbPr input.
	DRECAP DC-HD1
	KEIAN KHE660
*/
static struct saa716x_capture_config hd1 = {
	.board_name = "HDMI/YPbPr",
	.vip_port 	= 1,
	.vi_ctrl 	= 0x2C688C41,
	.subdev 	= SAA716x_SUBDEV_ADV7611_AD9983,
};

static int saa716x_cap_init_reg(struct saa716x_dev *saa716x)
{
	struct saa716x_capture_config *cap_config = &saa716x->config->capture_config;
	
	SAA716x_EPWR(GREG, GREG_VI_CTRL, cap_config->vi_ctrl);

	SAA716x_EPWR(GREG, GREG_FGPI_CTRL, 0x321);
	SAA716x_EPWR(GREG, GREG_VIDEO_IN_CTRL, 0x0C);

	SAA716x_EPWR(MSI, MSI_INT_ENA_SET_L, MSI_INT_TAGACK_AI_0);

	/* tda19978 interrupt config */
	SAA716x_EPWR(MSI, MSI_INT_ENA_SET_H, MSI_INT_EXTINT_5);
	SAA716x_EPWR(MSI, MSI_CONFIG38, MSI_INT_POL_EDGE_FALL);
	/* adv7611 interrupt config */
	SAA716x_EPWR(MSI, MSI_INT_ENA_SET_H, MSI_INT_EXTINT_6);
	SAA716x_EPWR(MSI, MSI_CONFIG39, MSI_INT_POL_EDGE_FALL);
	/* ad9983? */
	SAA716x_EPWR(MSI, MSI_INT_ENA_SET_H, MSI_INT_EXTINT_8);
	SAA716x_EPWR(MSI, MSI_CONFIG41, MSI_INT_POL_EDGE_FALL);

	//SAA716x_EPWR(MSI, MSI_INT_ENA_SET_L, MSI_INT_OVRFLW_VI0_0);
	//SAA716x_EPWR(MSI, MSI_INT_ENA_SET_H, MSI_INT_I2CINT_0);
	//SAA716x_EPWR(MSI, MSI_INT_ENA_SET_H, MSI_INT_I2CINT_1);
	
	return 0;
}

static struct saa716x_capture_config startech_echdcap = {
    .board_name = "StarTech ECHDCAP",
    .vip_port = 0,
    .vi_ctrl = 0x06080F8C,
    .subdev = SAA716x_SUBDEV_ECHDCAP_RX,
};

/* Identify capture board type */
static int saa716x_cap_board_identify(struct saa716x_dev *saa716x)
{
	switch (saa716x->revision) {
		case 2:	/* SAA7160E */
			saa716x->config->capture_config = hd1;
			break;
		case 3:	/* SAA7160ET */
                        if (saa716x->pdev->subsystem_vendor == 0xf50a &&
                            saa716x->pdev->subsystem_device == 0x12ab) {
                            saa716x->config->capture_config = startech_echdcap;
                        } else if(saa716x->pdev->subsystem_vendor == SKNET){
				saa716x->config->capture_config = sknet;
			} else {
				saa716x->config->capture_config = generic_tda19978;
			}
			break;
		default:
			// Unknown board type.
			dprintk(SAA716x_ERROR, 1, "Unknown capture board type.");
			saa716x->config->capture_config = generic_tda19978;
			break;
	}
	
	printk("%s: Board type: %s", __func__, saa716x->config->capture_config.board_name);
	return 0;
}

static int saa716x_cap_pci_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct saa716x_dev *saa716x;
	int err = 0;

	saa716x = kzalloc(sizeof (struct saa716x_dev), GFP_KERNEL);
	if (saa716x == NULL) {
		printk(KERN_ERR "saa716x_cap_pci_probe ERROR: out of memory\n");
		err = -ENOMEM;
		goto fail0;
	}

	saa716x->verbose	= verbose;
	saa716x->int_type	= int_type;
	saa716x->pdev		= pdev;
	saa716x->module		= THIS_MODULE;
	saa716x->config		= (struct saa716x_config *) pci_id->driver_data;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
	saa716x->irq_work_queue = create_singlethread_workqueue(saa716x->v4l2_dev.name);
#else
	saa716x->irq_work_queue = alloc_workqueue(saa716x->v4l2_dev.name, WQ_UNBOUND, 1);
#endif
	if (saa716x->irq_work_queue == NULL) {
		dprintk(SAA716x_ERROR, 1, "Could not create workqueue");
		err = -ENOMEM;
		goto fail1;
	}

	INIT_WORK(&saa716x->irq_work, saa716x_irq_work_handler);

	err = saa716x_pci_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x PCI Initialization failed");
		goto fail1;
	}

	err = saa716x_cgu_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x CGU Init failed");
		goto fail1;
	}

	err = saa716x_core_boot(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x Core Boot failed");
		goto fail2;
	}
	dprintk(SAA716x_DEBUG, 1, "SAA716x Core Boot Success");

	err = saa716x_msi_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x MSI Init failed");
		goto fail2;
	}

	err = saa716x_jetpack_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x Jetpack core initialization failed");
		goto fail2;
	}

	err = saa716x_i2c_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x I2C Initialization failed");
		goto fail2;
	}

	saa716x_gpio_init(saa716x);
	err = saa716x_check_eeprom(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x EEPROM check failed");
	}

	err = saa716x_parse_eeprom(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x EEPROM parse failed");
	}

	err = saa716x_cap_board_identify(saa716x);
	err = saa716x_cap_init_reg(saa716x);

	err = saa716x_vip_init2(saa716x, saa716x->config->capture_config.vip_port, video_vip_worker);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x VIP initialization failed");
		goto fail3;
	}
	err = saa716x_aip_init2(saa716x, 0, audio_aip_worker);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x AIP initialization failed");
		goto fail4;
	}

	err = saa716x_v4l2_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x V4L2 initialization failed");
		goto fail5;
	}

	err = saa716x_alsa_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x ALSA initialization failed");
		goto fail6;
	}

	err = saa716x_make_debugfs(saa716x);

	return 0;

fail6:
	saa716x_v4l2_exit(saa716x);
fail5:
	saa716x_aip_exit2(saa716x, 0);
fail4:
	saa716x_vip_exit2(saa716x, saa716x->config->capture_config.vip_port);
fail3:
	saa716x_i2c_exit(saa716x);
fail2:
	saa716x_pci_exit(saa716x);
fail1:
	kfree(saa716x);
fail0:
	return err;
}

static void saa716x_cap_pci_remove(struct pci_dev *pdev)
{
	struct saa716x_dev *saa716x = pci_get_drvdata(pdev);

	saa716x_remove_debugfs(saa716x);
	flush_workqueue(saa716x->irq_work_queue);
	saa716x_alsa_exit(saa716x);
	saa716x_v4l2_exit(saa716x);
	saa716x_aip_exit2(saa716x, 0);
	saa716x_vip_exit2(saa716x, saa716x->config->capture_config.vip_port);
	saa716x_i2c_exit(saa716x);
	destroy_workqueue(saa716x->irq_work_queue);
	saa716x_pci_exit(saa716x);
	kfree(saa716x);
}

static irqreturn_t saa716x_cap_pci_irq(int irq, void *dev_id)
{
	struct saa716x_dev *saa716x	= (struct saa716x_dev *) dev_id;

	u32 stat_h, stat_l, mask_h, mask_l;

	if (unlikely(saa716x == NULL)) {
		printk("%s: saa716x=NULL", __func__);
		return IRQ_NONE;
	}

	stat_l = SAA716x_EPRD(MSI, MSI_INT_STATUS_L);
	stat_h = SAA716x_EPRD(MSI, MSI_INT_STATUS_H);
	mask_l = SAA716x_EPRD(MSI, MSI_INT_ENA_L);
	mask_h = SAA716x_EPRD(MSI, MSI_INT_ENA_H);

	dprintk(SAA716x_DEBUG, 1, "MSI STAT L=<%02x> H=<%02x>, CTL L=<%02x> H=<%02x>",
		stat_l, stat_h, mask_l, mask_h);

	if (!((stat_l & mask_l) || (stat_h & mask_h)))
		return IRQ_NONE;

	if (stat_l)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_L, stat_l);

	if (stat_h)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_H, stat_h);

	// saa716x_msi_event(saa716x, stat_l, stat_h);
#if 0
	dprintk(SAA716x_DEBUG, 1, "VI STAT 0=<%02x> 1=<%02x>, CTL 1=<%02x> 2=<%02x>",
		SAA716x_EPRD(VI0, INT_STATUS),
		SAA716x_EPRD(VI1, INT_STATUS),
		SAA716x_EPRD(VI0, INT_ENABLE),
		SAA716x_EPRD(VI1, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 0=<%02x> 1=<%02x>, CTL 1=<%02x> 2=<%02x>",
		SAA716x_EPRD(FGPI0, INT_STATUS),
		SAA716x_EPRD(FGPI1, INT_STATUS),
		SAA716x_EPRD(FGPI0, INT_ENABLE),
		SAA716x_EPRD(FGPI0, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 2=<%02x> 3=<%02x>, CTL 2=<%02x> 3=<%02x>",
		SAA716x_EPRD(FGPI2, INT_STATUS),
		SAA716x_EPRD(FGPI3, INT_STATUS),
		SAA716x_EPRD(FGPI2, INT_ENABLE),
		SAA716x_EPRD(FGPI3, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "AI STAT 0=<%02x> 1=<%02x>, CTL 0=<%02x> 1=<%02x>",
		SAA716x_EPRD(AI0, AI_STATUS),
		SAA716x_EPRD(AI1, AI_STATUS),
		SAA716x_EPRD(AI0, AI_CTL),
		SAA716x_EPRD(AI1, AI_CTL));

	dprintk(SAA716x_DEBUG, 1, "I2C STAT 0=<%02x> 1=<%02x>, CTL 0=<%02x> 1=<%02x>",
		SAA716x_EPRD(I2C_A, INT_STATUS),
		SAA716x_EPRD(I2C_B, INT_STATUS),
		SAA716x_EPRD(I2C_A, INT_ENABLE),
		SAA716x_EPRD(I2C_B, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "DCS STAT=<%02x>, CTL=<%02x>",
		SAA716x_EPRD(DCS, DCSC_INT_STATUS),
		SAA716x_EPRD(DCS, DCSC_INT_ENABLE));
#endif

	if (stat_l) {
		if (stat_l & MSI_INT_TAGACK_VI0_0) {
			tasklet_schedule(&saa716x->vip[0].tasklet);
		}
		if (stat_l & MSI_INT_TAGACK_VI1_0) {
			tasklet_schedule(&saa716x->vip[1].tasklet);
		}
		if (stat_l & MSI_INT_TAGACK_AI_0) {
			tasklet_schedule(&saa716x->aip[0].tasklet);
		}
	}

	if (stat_h) {
		if (stat_h & MSI_INT_EXTINT_5) {
			queue_work(saa716x->irq_work_queue, &saa716x->irq_work);
		}
		if (stat_h & MSI_INT_EXTINT_6) {
			queue_work(saa716x->irq_work_queue, &saa716x->irq_work);
		}
	}

	return IRQ_HANDLED;
}

/* Service routine to handle interrupts from receiver */
void saa716x_irq_work_handler(struct work_struct *work)
{
	struct saa716x_dev *saa716x = container_of(work, struct saa716x_dev, irq_work);
	
	v4l2_subdev_call(saa716x->saa716x_stream[0].sd_receiver, core, interrupt_service_routine, 0, NULL);
}

static void video_vip_worker(unsigned long data)
{
	struct saa716x_vip_stream_port *vip_entry = (struct saa716x_vip_stream_port *)data;
	struct saa716x_dev *saa716x = vip_entry->saa716x;
	struct saa716x_stream *s = &saa716x->saa716x_stream[0];
	struct saa716x_cap_buffer *cb;
	u32 vi_port, dma_ch_num;
	u32 write_index;

	dma_ch_num = vip_entry->dma_channel[0];
	if (dma_ch_num == 0) {
		vi_port = 0;
	} else if (dma_ch_num == 3) {
		vi_port = 1;
	} else {
		printk(KERN_ERR "%s: unexpected channel %u\n",
		       __func__, vip_entry->dma_channel[0]);
		return;
	}

	write_index = saa716x_vip_get_write_index(saa716x, vi_port);
	if (write_index < 0)
		return;

	dprintk(SAA716x_DEBUG, 1, "dma buffer = %d", write_index);

	if (write_index == vip_entry->read_index) {
		printk(KERN_DEBUG "%s: called but nothing to do\n", __func__);
		return;
	}

	do {	
		if ((s->vip_params.stream_flags & VIP_FIELD_SEQ) ||
			(s->vip_params.stream_flags & (VIP_EVEN_FIELD | VIP_ODD_FIELD))) {
			if (vip_entry->read_index % 2 == 0) {
				vip_entry->read_index = (vip_entry->read_index + 1) & 7;
				continue;
			}
		}

		spin_lock(&s->qlock);
		if (list_empty(&s->buf_list)) {
			printk("%s: vb2_queue is empty !", __func__);
			spin_unlock(&s->qlock);
			return;
		}
		cb = list_first_entry(&s->buf_list, struct saa716x_cap_buffer, list);
		list_del(&cb->list);
		spin_unlock(&s->qlock);
		
		cb->vb.field = s->format.field;
		cb->vb.vb2_buf.timestamp = ktime_get_ns();
		cb->vb.sequence = s->sequence++;
		vb2_buffer_done(&cb->vb.vb2_buf, VB2_BUF_STATE_DONE);
		printk("%s: vb2_buffer(%d) returned", __func__, cb->vb.vb2_buf.index);

		vip_entry->read_index = (vip_entry->read_index + 1) & 7;
	} while (write_index != vip_entry->read_index);
}

static void audio_aip_worker(unsigned long data)
{
	struct saa716x_aip_stream_port *aip_entry = (struct saa716x_aip_stream_port *)data;
	struct saa716x_dev *saa716x = aip_entry->saa716x;
	u32 ai_port, dma_ch_num;
	u32 write_index;

	dma_ch_num = aip_entry->dma_channel;
	if (dma_ch_num == 10) {
		ai_port = 0;
	} else if (dma_ch_num == 11) {
		ai_port = 1;
	} else {
		printk(KERN_ERR "%s: unexpected channel %u\n",
		       __func__, aip_entry->dma_channel);
		return;
	}

	write_index = saa716x_aip_get_write_index(saa716x, ai_port);
	dprintk(SAA716x_DEBUG, 1, "write_index = %d", write_index);

	if (write_index == aip_entry->read_index) {
		printk(KERN_DEBUG "%s: called but nothing to do\n", __func__);
		return;
	}

	do {
		pci_dma_sync_sg_for_cpu(saa716x->pdev,
			aip_entry->dma_buf[aip_entry->read_index].sg_list,
			aip_entry->dma_buf[aip_entry->read_index].list_len,
			PCI_DMA_FROMDEVICE);
		saa716x_alsa_deliver_buffer(saa716x);
		aip_entry->read_index = (aip_entry->read_index + 1) & 7;
	} while (write_index != aip_entry->read_index);
}


/* Driver data */
static struct saa716x_config saa716x_cap_generic_config = {
	.model_name		= "SAA7160 Capture",
	.dev_type		= "HDMI Capture",
	.boot_mode		= SAA716x_EXT_BOOT,
	.irq_handler	= saa716x_cap_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
};

/* Driver Registration*/
static struct pci_device_id saa716x_cap_pci_table[] = {
	MAKE_ENTRY(NXP_SEMICONDUCTOR,	0x0002, 	SAA7160, &saa716x_cap_generic_config),
	MAKE_ENTRY(SKNET,				MONSTAR_X3, SAA7160, &saa716x_cap_generic_config),
	MAKE_ENTRY(KWORLD,				KHE660,		SAA7160, &saa716x_cap_generic_config),

        MAKE_ENTRY(0xf50a, 0x12ab, SAA7160, &saa716x_cap_generic_config),

	{ }
};
MODULE_DEVICE_TABLE(pci, saa716x_cap_pci_table);

static struct pci_driver saa716x_cap_pci_driver = {
	.name			= DRIVER_NAME,
	.id_table		= saa716x_cap_pci_table,
	.probe			= saa716x_cap_pci_probe,
	.remove			= saa716x_cap_pci_remove,
};

module_pci_driver(saa716x_cap_pci_driver);

MODULE_DESCRIPTION("SAA716x Capture driver");
MODULE_AUTHOR("notsugi");
MODULE_LICENSE("GPL");
