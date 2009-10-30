#!/bin/sh
#
# Auto create android system and userdata images.
#
# 20081029:
#	Lox		: create
#

IMG_DIR_NAME=arch/arm/boot
BOOT_RAMDISK=build/boot.img-ramdisk

if [ ! -f ${IMG_DIR_NAME}/zImage ]; then
	echo "Cannot find \"${IMG_DIR_NAME}/zImage\"!" ;
	exit 1
fi

echo "Building ramdisk"

mkbootfs ${BOOT_RAMDISK} | gzip > build/out/ramdisk.img
if [ ! -f build/out/ramdisk.img ]; then
	echo "Cannot build ramdisk!" ;
	exit 1
fi


echo "Building boot.img"
mkbootimg --kernel ${IMG_DIR_NAME}/zImage --ramdisk build/out/ramdisk.img --cmdline "no_console_suspend=1 console=null" -o build/out/boot.img-hero --base 0x19200000

rm build/out/ramdisk.img

if [ ! -f build/out/boot.img-hero ]; then
	echo "Cannot build boot.img!" ;
	exit 1
fi

echo "Done. (build/out/boot.img-hero)"

