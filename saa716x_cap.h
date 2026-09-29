#ifndef __SAA716x_CAPTURE_H
#define __SAA716x_CAPTURE_H

#define SKNET               0x3275
#define MONSTAR_X3          0x9090

#define KWORLD              0x17de
#define KHE660              0x7554

enum saa716x_capture_subdev
{
	SAA716x_SUBDEV_TDA19978 = 0,
	SAA716x_SUBDEV_ADV7611,
	SAA716x_SUBDEV_ADV7611_AD9983,
	SAA716x_SUBDEV_MST3367,
	SAA716x_SUBDEV_ECHDCAP_RX,
};

struct saa716x_capture_config
{
	char        *board_name;
	int         vip_port;
	u32         vi_ctrl;
	enum saa716x_capture_subdev    subdev;
};


#endif /* __SAA716x_CAPTURE_H */
