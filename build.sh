#!/bin/bash -xe
sudo make -C /lib/modules/$(uname -r)/build M=$(pwd) clean
sudo bash ./build-module.sh
