#!/bin/bash
#
# Auto create android system and userdata images.
#
# 20080220:
#	Charlie		: create
# 20080416:
#	Charlie		: use bash
#

SYS_DIR_NAME=system
DAT_DIR_NAME=data
SYS_IMG_NAME=system.img
DAT_IMG_NAME=userdata.img
PRJ_PATH=${ANDROID_BUILD_TOP}

IMGDIR=mkimg
IMGTOOL=${PRJ_PATH}/out/host/linux-x86/bin/mkyaffs2image
SYS_IMG_DIR=${OUT}/${SYS_DIR_NAME}
DAT_IMG_DIR=${OUT}/${DAT_DIR_NAME}

if [ -d ${IMGDIR} ]; then
	cd ${IMGDIR}
fi

[ ! -f func.sh ] && echo "This script needs func.sh!" && exit

source func.sh

if [ ! -f ${IMGTOOL} ]; then
	echo "Cannot find \"${IMGTOOL}\"!" ;
	exit 1
fi

if [ -d ./${SYS_DIR_NAME} ]; then
	echo "[Use local ${SYS_DIR_NAME} directory]"
	SYS_IMG_DIR=./${SYS_DIR_NAME}
fi

if [ ! -d ${SYS_IMG_DIR} ]; then
	echo "No ${SYS_DIR_NAME} directory found!" ;
	exit 1
fi

if [ -d ./${DAT_DIR_NAME} ]; then
	echo "[Use local ${DAT_DIR_NAME} directory]"
	DAT_IMG_DIR=./${DAT_DIR_NAME}
fi

if [ ! -d ${DAT_IMG_DIR} ]; then
	echo "No ${DAT_DIR_NAME} directory found!" ;
	exit 1
fi

echo "Remove old images..."
rm -f ${SYS_IMG_NAME}	> /dev/null 2>&1
rm -f ${DAT_IMG_NAME}	> /dev/null 2>&1

echo =====
echo "SYSTEM   : [`get_full_path ${SYS_IMG_DIR}`]"
echo "USERDATA : [`get_full_path ${DAT_IMG_DIR}`]"
echo =====

echo "Create ${SYS_DIR_NAME} image..."
${IMGTOOL} -f ${SYS_IMG_DIR} ${SYS_IMG_NAME}

echo "Create ${DAT_DIR_NAME} image..."
${IMGTOOL} -f ${DAT_IMG_DIR} ${DAT_IMG_NAME}

if [ -f ${SYS_IMG_NAME} ] && [ -f ${DAT_IMG_NAME} ]; then
	chmod a+r ${SYS_IMG_NAME} ${DAT_IMG_NAME}
	echo "Done."
else
	echo "Failed!"
fi

