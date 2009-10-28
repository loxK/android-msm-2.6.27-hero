#!/bin/sh
#
# Auto create android system and userdata images.
#
# 20081029:
#	Lox		: create
#

UPDATE_SKEL=build/update-skel

mkimg/mkboot-hero.sh

if [ ! -f build/out/boot.img-hero ]; then
	echo "Failed building boot.img!" ;
	exit 1
fi

echo "Copying boot.img"
cp build/out/boot.img-hero ${UPDATE_SKEL}/boot.img
echo "Done."

echo "Copying modules"
if [ -f crypto/lzo.ko ]; then
    echo "[lzo.ko]"
    cp crypto/lzo.ko ${UPDATE_SKEL}/system/lib/modules/
fi

if [ -f lib/lzo/lzo_compress.ko ]; then
    echo "[lzo_compress.ko]"
    cp lib/lzo/lzo_compress.ko ${UPDATE_SKEL}/system/lib/modules/
fi

if [ -f lib/lzo/lzo_decompress.ko ]; then
    echo "[lzo_decompress.ko]"
    cp lib/lzo/lzo_decompress.ko ${UPDATE_SKEL}/system/lib/modules/
fi

if [ -f drivers/net/tun.ko ]; then
    echo "[tun.ko]"
    cp drivers/net/tun.ko ${UPDATE_SKEL}/system/lib/modules/
fi

if [ -f net/ipv4/ip_gre.ko ]; then
    echo "[ip_gre.ko]"
    cp net/ipv4/ip_gre.ko ${UPDATE_SKEL}/system/lib/modules/
fi

echo "Building update.zip"

CURRENT_DIR=${PWD}

cd ${UPDATE_SKEL}
ls | zip -r ../update -x system/lib/modules/.\* .placeholder -@
cd ${CURRENT_DIR}

mv build/update.zip  build/out
	
echo "Signing update.zip"

androsign build/out/update.zip
	
echo "update-signed.zip ready in build/out folder. Ready to flash."

exit 0

echo mkbootfs ${BOOT_RAMDISK} | gzip > build/boot.img-ramdisk.cpio.gz
if [ ! -f build/boot.img-ramdisk.cpio.gz ]; then
	echo "Cannot build ramdisk!" ;
	exit 1
fi


echo "Building boot.img"
mkbootimg --kernel ${IMG_DIR_NAME}/zImage --ramdisk build/boot.img-ramdisk.cpio.gz --cmdline "no_console_suspend=1 console=null" -o build/hero-boot.img --base 0x19200000

rm build/boot.img-ramdisk.cpio.gz

if [ ! -f build/hero-boot.img ]; then
	echo "Cannot build boot.img!" ;
	exit 1
fi

echo "Done. (${IMG_DIR_NAME}/hero-boot.img)"

