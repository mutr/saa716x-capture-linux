#!/bin/bash -xe
sudo rmmod saa716x_capture 2>/dev/null || true
sudo rmmod saa716x_core 2>/dev/null || true
sudo rmmod mst3367_drv 2>/dev/null || true

