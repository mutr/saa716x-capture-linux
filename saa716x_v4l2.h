#ifndef __SAA716x_V4L2_H
#define __SAA716x_V4L2_H

struct saa716x_dev;
struct saa716x_stream;


extern int saa716x_v4l2_init(struct saa716x_dev *saa716x);
extern int saa716x_v4l2_exit(struct saa716x_dev *saa716x);
extern void saa716x_cap_check_input_errors(struct saa716x_stream *s);

#endif /* __SAA716x_V4l2_H */
