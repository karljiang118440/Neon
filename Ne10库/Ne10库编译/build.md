


#1、cmake 操作

##1.1、cmake 相关

cmake --version

cmake-gui



#二、编译 Ne10 库

#1、

https://github.com/projectNe10/Ne10/blob/master/doc/building.md

cd $NE10_PATH                       # Change directory to the location of the Ne10 source
mkdir build && cd build             # Create the `build` directory and navigate into it
export NE10_LINUX_TARGET_ARCH=armv7 # Set the target architecture (can also be "aarch64")
cmake -DGNULINUX_PLATFORM=ON ..     # Run CMake to generate the build files
make 

> 修改为 armv8 或者 aarch64 的编译器

cd $NE10_PATH                       # Change directory to the location of the Ne10 source
mkdir build && cd build             # Create the `build` directory and navigate into it

export NE10_LINUX_TARGET_ARCH=armv8 # Set the target architecture (can also be "aarch64")
export NE10_LINUX_TARGET_ARCH=aarch64 # Set the target architecture (can also be "aarch64")

cmake -DGNULINUX_PLATFORM=ON ..     # Run CMake to generate the build files
cmake -DGNULINUX_PLATFORM=ON .. -DCMAKE_SYSTEM_NAME=Linux
cmake -DCMAKE_TOOLCHAIN_FILE=../GNUlinux_config.cmake ..
make 




#2、其他版本的

CMake Error: The source directory "/home/karljiang/Neon" does not appear to contain CMakeLists.txt.
Specify --help for usage, or press the help button on the CMake GUI.




projectNe10-Ne10-v1.2.1-72-g1f059a7.zip 该文件包中Ne10并不支持 aarch64 的操作。








#3、交叉编译工具指定

指定工具选项为 cmake -DGNULINUX_PLATFORM=ON ..

You are trying to compile for non-ARM (CMAKE_SYSTEM_PROCESSOR='x86_64')!










#4、参考文档

https://www.jianshu.com/p/83da8280cc2d





>:指定编译器

export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabi-
export PATH=/个人交叉编译器路径/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabi/bin/:$PATH

>:修改 Ubuntu 中的路径

export ARCH=arm
export CROSS_COMPILE=aarch64-linux-gnu-
export PATH=/home/karljiang/VisionSDK_S32V2_RTM_1_2_0/compilers/gcc-6.3.1-linaro-Xarmv8-linux/i686-linux/bin/:$PATH

/home/karljiang/VisionSDK_S32V2_RTM_1_2_0/compilers/gcc-6.3.1-linaro-Xarmv8-linux/i686-linux/bin






###1、编译方式1
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnueabi-
export PATH=/home/djiango/Ne10/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabi/bin/:$PATH

/home/djiango/Ne10/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabi/bin

该编译器应该没有问题

/home/djiango/Ne10/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabi/arm-linux-gnueabi/bin






###2、编译方式2
export ARCH=arm
export CROSS_COMPILE=aarch64-linux-gnu- // 
export PATH=/home/djiango/bsp15.0/gcc-linaro-4.9-2015.05-x86_64_aarch64-linux-gnu/bin/:$PATH





#5、windows 下类似的




























#三、关于 Ne10 库的使用

包含的模块有哪些：

1、（信号处理）DSP 包含常用的FFT FIR IIR等函数
2、（数学计算）Math
3、（图像处理）ImgprocI
4、physics



https://community.arm.com/cn/f/discussions/10409/ne10-armv8/33143

在这里有强调关于ne10对armv8不支持math和physics modules。

我刚刚试了一下也是，编译modules下dsp相关内容以及使用都没问题。但是math和physics不行。

>:能够使用的库函数并不太多，估计最后能够使用的无非就是 ImgprocI 部分。



Ne10.sh 脚本

#!/bin/bash
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
export PATH=$PATH:/home/djiango/bsp15.0/gcc-linaro-4.9-2015.05-x86_64_aarch64-linux-gnu/bin


cd /home/djiango/Ne10/projectNe10



mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/home/djiango/Ne10/projectNe10/GNUlinux_config.cmake ..
make


