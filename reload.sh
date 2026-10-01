#!/bin/bash -xe
for m in videodev v4l2-dv-timings videobuf2-v4l2 videobuf2-dma-sg dvb-core snd-pcm i2c-algo-bit; do sudo modprobe $m; done
sudo rmmod saa716x_capture 2>/dev/null || true
sudo rmmod saa716x_core 2>/dev/null || true
sudo rmmod mst3367_drv 2>/dev/null || true
sudo rmmod echdcap_rx 2>/dev/null || true

sudo insmod ./mst3367-drv.ko
sudo insmod ./echdcap_rx.ko
sudo insmod ./saa716x_core.ko
sudo insmod ./saa716x_capture.ko
