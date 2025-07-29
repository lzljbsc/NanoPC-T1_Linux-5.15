#!/bin/bash

set -x

# 交叉编译器路径
CC_USED=/opt/toolchain/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabi/bin/arm-linux-gnueabi-

if [ ! -f ".config" ]; then
    make CROSS_COMPILE=$CC_USED nanopc-t1_defconfig
fi

make CROSS_COMPILE=$CC_USED -j32 all

make CROSS_COMPILE=$CC_USED -j32 dtbs modules uImage LOADADDR=0x40008000


# 使用以下命令 menuconfig savedefconfig
# make CROSS_COMPILE=$CC_USED menuconfig
# make CROSS_COMPILE=$CC_USED savedefconfig

