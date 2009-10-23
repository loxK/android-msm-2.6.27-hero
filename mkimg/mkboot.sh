#!/bin/bash
#
# Auto find kernel and rootfs to create boot image.
#
# 20080220:
#	Charlie		: create
# 20080312:
#	Charlie		: allow local kernel image
# 20080416:
#	Charlie		: use bash
# 20080606:
#	Charlie		: update command line
# 20080819:
#	Charlie		: auto grab command line from device
#

ROOT_DIR_NAME=root
ROOT_IMG_NAME=ramdisk.img
BOOT_IMG_NAME=boot.img
KERNEL_IMAGE=../arch/arm/boot/zImage
PRJ_PATH=${ANDROID_BUILD_TOP}
CMD_LINE=

IMGDIR=mkimg
IMGTOOL=${PRJ_PATH}/out/host/linux-x86/bin/mkbootimg
FSTOOL=${PRJ_PATH}/out/host/linux-x86/bin/mkbootfs
ROOTFS=${OUT}/${ROOT_DIR_NAME}

if [ -d ${IMGDIR} ]; then
	cd ${IMGDIR}
fi

[ ! -f func.sh ] && echo "This script needs func.sh!" && exit

source func.sh

if [ ! -f ${IMGTOOL} ]; then
	echo "Cannot find \"${IMGTOOL}\"!" ;
	exit 1
fi

if [ ! -f ${FSTOOL} ]; then
	echo "Cannot find \"${FSTOOL}\"!" ;
	exit 1
fi

if [ -f ./kernel ]; then
	echo "[Use local kernel]"
	local_kernel=1
fi

if [ -f ./${ROOT_IMG_NAME} ]; then
	echo "[Use local ramdisk]"
	local_ramdisk=1
fi

if [ x$local_ramdisk == x ]; then
	if [ -d ./${ROOT_DIR_NAME} ]; then
		echo "[Use local rootfs]"
		ROOTFS=./${ROOT_DIR_NAME}
	fi

	if [ ! -d ${ROOTFS} ]; then
		echo "No root fs found!" ;
		exit 1
	fi
fi

echo "Remove old images..."
rm -f ${BOOT_IMG_NAME}	> /dev/null 2>&1

echo =====
if [ x$local_kernel == x ]; then
	echo KERNEL : [`get_full_path ${KERNEL_IMAGE}`]
else
	echo KERNEL : [`get_full_path ./kernel`]
fi
if [ x$local_ramdisk == x ]; then
	echo ROOTFS : [`get_full_path ${ROOTFS}`]
else
	echo RAMDISK: [`get_full_path ./${ROOT_IMG_NAME}`]
fi
echo =====

if [ x$local_kernel == x ]; then
	echo "Copy kernel..."
	cp -f ${KERNEL_IMAGE} kernel	> /dev/null 2>&1
fi

if [ x$local_ramdisk == x ]; then
	echo "Create ramdisk image..."
	${FSTOOL} ${ROOTFS} | gzip > ${ROOT_IMG_NAME}
fi

if [ ! -f kernel ] || [ ! -f ${ROOT_IMG_NAME} ]; then
	echo "No kernel or ramdisk found!" ;
	exit 1
fi

if [ "${CMD_LINE}" == "" ]; then
	CMD_LINE=`get_command`
fi

if [ "${CMD_LINE}" == "" ]; then
	echo "Create boot image without command line..."
	${IMGTOOL} --kernel kernel --ramdisk ${ROOT_IMG_NAME} --output ${BOOT_IMG_NAME}
else
	echo "Create boot image with command line [${CMD_LINE}]..."
	${IMGTOOL} --kernel kernel --ramdisk ${ROOT_IMG_NAME} --cmdline "${CMD_LINE}" --output ${BOOT_IMG_NAME}
fi

if [ -f ${BOOT_IMG_NAME} ]; then
	echo "Done."
else
	echo "Failed!"
fi

# keep images?
if [ "$1" != "-k" ]; then
	echo "Remove intermediate images..."
	if [ x$local_kernel == x ]; then
		rm -f kernel	> /dev/null 2>&1
	fi
	if [ x$local_ramdisk == x ]; then
		rm -f ${ROOT_IMG_NAME}	> /dev/null 2>&1
	fi
fi

