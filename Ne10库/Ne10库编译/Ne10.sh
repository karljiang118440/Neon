#!/bin/bash
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
export PATH=$PATH:/home/djiango/bsp15.0/gcc-linaro-4.9-2015.05-x86_64_aarch64-linux-gnu/bin


cd /home/djiango/Ne10/projectNe10



mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/home/djiango/Ne10/projectNe10/GNUlinux_config.cmake ..
make
