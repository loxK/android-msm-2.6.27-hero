#!/bin/bash

# unuse_ramzswap (run as *root*)
# - Unloads ramzswap and related modules.
# - Turns off swap device. 
#

# FIXME: make sure modprobe etc. are in PATH and then use 'which'
#LSMOD_BIN=`which lsmod`
#RMMOD_BIN=`which rmmod`
#SWAPOFF_BIN=`which swapoff`

LSMOD_BIN=/sbin/lsmod
RMMOD_BIN=/sbin/rmmod
SWAPOFF_BIN=/sbin/swapoff

EXIST=`$LSMOD_BIN | grep ramzswap`
if [ "$EXIST" = "" ]; then
	echo "ramzswap module not loaded"
	exit 0
fi

EXIST=`cat /proc/swaps | grep ramzswap`
if [ "$EXIST" != "" ]; then
	echo "Turning off compache swap device ..."
	$SWAPOFF_BIN /dev/ramzswap0
fi

echo "Unloading modules ..."
$RMMOD_BIN ramzswap
$RMMOD_BIN xvmalloc
echo "Done!"
