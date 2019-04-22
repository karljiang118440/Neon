#!/bin/bash
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
export LINUX_S32V234_DIR=/home/djiango/bsp15.0/linux
export PATH=$PATH:/home/djiango/bsp15.0/gcc-linaro-4.9-2015.05-x86_64_aarch64-linux-gnu/bin
export PATH=$PATH:/home/djiango/NXP/S32DS_Vision_v2.0/S32DS/APUC/bin
export APU_TOOLS=/home/djiango/NXP/S32DS_Vision_v2.0/S32DS/APUC

#cd /home/djiango/NXP/S32DS_Vision_v2.0/S32DS/s32v234_sdk/isp/graphs/mipi_simple_ov9716_karl/build-v234ce-gnu-linux-d
#make clean & make

cd /home/djiango/NXP/S32DS_Vision_v2.0/S32DS/s32v234_sdk/demos/isp/isp_ov9716_csi_dcu_karl/build-v234ce-gnu-linux-d
#make cleansub & make APU_COMP=nxp allsub
make clean & make

cp isp_ov9716_csi_dcu.elf /media/djiango/rootfs/home/root/karl

sync

umount /dev/sdb2 /mnt

echo "ok"
