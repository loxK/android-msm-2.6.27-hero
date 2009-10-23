#!/bin/bash
#
# Auto upload kernel and ramdisk image via fastboot.
#
# 20080219:
#	Charlie		: override kernel command line
# 20080416:
#	Charlie		: use bash
# 20080606:
#	Charlie		: change command line
# 20080819:
#	Charlie		: auto grab kernel command line
#

IMGDIR=mkimg

if [ -d ${IMGDIR} ]; then
	cd ${IMGDIR}
fi

if [ -f func.sh ]; then
	source func.sh
	CMD_LINE=`get_command`
fi

./mkboot.sh

if [ ! -f boot.img ]; then
	echo "No Image!"
	exit 1
fi

if [ "$1" == "-f" ]; then
	echo "Update boot partition:"
	fastboot flash boot boot.img
	fastboot reboot
else
	if [ "${CMD_LINE}" == "" ]; then
		echo "Download and reboot:"
		fastboot boot boot.img
	else
		echo "Download and reboot with command line [${CMD_LINE}]:"
		fastboot -c "${CMD_LINE}" boot boot.img
	fi
fi

