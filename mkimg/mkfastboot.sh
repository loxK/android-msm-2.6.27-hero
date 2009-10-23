#!/bin/bash
#
# Create fastboot recovery image.
#
# 20080220:
#	Charlie		: create
# 20080416:
#	Charlie		: use bash
# 20080606:
#	Charlie		: change command line
# 20080819:
#	Charlie		: auto grab kernel command line
#

REC_IMG_NAME=recovery.img
REC_LDR_NAME=usbloader
PRJ_PATH=${ANDROID_BUILD_TOP}

IMGDIR=mkimg
IMGTOOL=${PRJ_PATH}/out/host/linux-x86/bin/mkbootimg

if [ -d ${IMGDIR} ]; then
	cd ${IMGDIR}
fi

[ ! -f func.sh ] && echo "This script needs func.sh!" && exit

source func.sh

if [ ! -f ${IMGTOOL} ]; then
	echo "Cannot find \"${IMGTOOL}\"!" ;
	exit 1
fi

echo "Remove old recovery image..."
rm -f ${REC_IMG_NAME} > /dev/null 2>&1

echo =====
echo BOOT : [`get_full_path ${OUT}/${REC_LDR_NAME}`]
echo =====

echo "Copy ${REC_LDR_NAME}..."
cp -f ${OUT}/${REC_LDR_NAME} .	> /dev/null 2>&1

if [ ! -f ${REC_LDR_NAME} ]; then
	echo "Cannot find \"${REC_LDR_NAME}\"!" ;
	exit 1
fi

CMD_LINE=`get_command`

if [ "${CMD_LINE}" == "" ]; then
	echo "Create recovery image without command line..."
	${IMGTOOL} --kernel ${REC_LDR_NAME} --ramdisk NONE --output ${REC_IMG_NAME}
else
	echo "Create recovery image with command line [${CMD_LINE}]..."
	${IMGTOOL} --kernel ${REC_LDR_NAME} --ramdisk NONE --cmdline "${CMD_LINE}" --output ${REC_IMG_NAME}
fi

if [ -f ${REC_IMG_NAME} ]; then
	echo "Done."
else
	echo "Failed!"
fi

# keep images?
if [ "$1" != "-k" ]; then
	echo "Remove intermediate stuff..."
	rm -f ${REC_LDR_NAME} > /dev/null 2>&1
fi

