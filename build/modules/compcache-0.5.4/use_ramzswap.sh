#!/bin/bash

#
# use_ramzswap (run as *root*)
# - Loads ramzswap and related modules.
# - Sets up swap device.
#
# Usage: use_ramzswap.sh [disksize(KB)|memlimit(KB)] [backing swap device]
#
#	If backing swap device is not given then first parameter is taken
#	as ramzswap disk size. Otherwise, its taken as ramzswap memory
#	limit (see NOTES below).
#
#	e.g. 1) use_ramzswap.sh 10240 /dev/sda2
#		sets ramzswap limit as 10MB and /dev/sda2 as backing
#		swap device. NOTE: /dev/sda2 *must* be a valid swap partition.
#
#	     2) use_ramzswap.sh 10240
#		sets ramzswap disk size as 10MB.
#
#	     3) use_ramzswap.sh
#		same as (2) - ramzswap disk size will be set to default
#		(25% of RAM size)
#
#
# NOTES on parameters:
#
# -- disksize:
# This size refers to amount of (uncompressed) data it can hold.
# For e.g. disksize_kb=1024 means it can hold 1024kb worth of
# uncompressed data even if this data compresses to just, say, 100kb.
#
# -- memlimit:
# This refers to limit on amount of (compressed) data it can hold in
# memory. Any additional data is forwarded to backing swap device.
#
# -- backing_swap:
# This is block device to be used as backing store for ramzswap.
# When pages more than memlimit_kb as swapped to ramzswap, we store
# any additional pages in this device. We may also move some pages
# from ramzswap to this device in case system is really low on
# memory (TODO).
# This device is not directly visible to kernel as a swap device
# (/proc/swaps will only show /dev/ramzswap0 and not this device).
# Managing this backing device is the job of ramzswap module.
#

##
# Script begin
##

# ramzswap module params
BACKING_DEV="$2"
RZS_PARAM_STR=""

if [ -z "$BACKING_DEV" ]; then
	if [ -n "$1" ]; then
		RZS_PARAM_STR="disksize_kb=$1"
	fi
else
	RZS_PARAM_STR="memlimit_kb=$1 backing_swap=$BACKING_DEV"
fi

LSMOD_BIN=/sbin/lsmod
INSMOD_BIN=/sbin/insmod
MODPROBE_BIN=/sbin/modprobe
SWAPON_BIN=/sbin/swapon
UDEVADM_BIN=/sbin/udevadm

# Maximum of two parameters allowed
INSMOD()
{
	MOD_NAME="$1"

	# insmod program treats all params as a single param if
	# these are not passed separately. This causes incorrect
	# parsing of options if multiple params are given.
	PARAM1="$2"
	PARAM2="$3"

	EXIST=`$LSMOD_BIN | grep "$MOD_NAME"`
	if [ "$EXIST" != "" ]; then
		echo "Module: $MOD_NAME already loaded."
		return 0
	else
		if [ ! -f $MOD_NAME.ko ]; then
			echo "Module: $MOD_NAME not found in current directory"
			return 1
		else
			#echo "INSMOD: P1: [$PARAM1] P2: [$PARAM2]"
			$INSMOD_BIN $MOD_NAME.ko "$PARAM1" "$PARAM2"
			RET="$?"
			[ "$RET" != 0 ] && echo "Failed to load $MOD_NAME module"
			return $RET

		fi
	fi
}


EXIST=`cat /proc/swaps | grep ramzswap`
if [ -n "$EXIST" ]; then
	echo "ramzswap swap device already active."
	exit 0
fi

echo "Loading modules ..."
$MODPROBE_BIN -q lzo_compress || (echo "LZO compress module not found"; exit 0)
$MODPROBE_BIN -q lzo_decompress || (echo "LZO decompress module not found"; exit 0)
INSMOD xvmalloc
[ "$?" != "0" ] && exit $?

#echo "PARAMS: [$RZS_PARAM_STR]"

# RZS_PARAM_STR must be passed to INSMOD() *without* quotes
# so individual ramzswap params, separated by spaces, are
# treated as different INSMOD() parameters.
INSMOD ramzswap $RZS_PARAM_STR
[ "$?" != 0 ] && exit $?

# /dev/ramzswap0 is not available immediately after insmod returns
# So, let udev complete its work before we do swapon
if [ -f "$UDEVADM_BIN" ]; then
	$UDEVADM_BIN settle
else
	sleep 2
fi

# Set it as swap device with highest priority
echo "Setting up swap device ..."
$SWAPON_BIN /dev/ramzswap0 -p 100
RET="$?"
if [ "$RET" = "0" ]; then
	echo "Done!"
else
	echo "Could not add ramzswap swap device."
fi

exit $RET
