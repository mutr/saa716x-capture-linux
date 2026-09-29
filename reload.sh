#!/bin/bash -xe
sudo rmmod saa716x_capture 2>/dev/null || true
sudo rmmod saa716x_core 2>/dev/null || true
sudo rmmod mst3367_drv 2>/dev/null || true
sudo rmmod echdcap_rx 2>/dev/null || true

sudo insmod ./mst3367-drv.ko debug=1
sudo insmod ./echdcap_rx.ko debug=1
sudo insmod ./saa716x_core.ko
sudo insmod ./saa716x_capture.ko verbose=4
