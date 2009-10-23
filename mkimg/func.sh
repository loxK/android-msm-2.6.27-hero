
function get_command ()
{
	product=`echo ${ANDROID_PRODUCT_OUT} | awk -F/ '{print $NF}'`

	file=${ANDROID_BUILD_TOP}/partner/htc/${product}/board_config.mk

	[ ! -f $file ] && return

	cat $file | grep BOARD_KERNEL_CMDLINE | tail -1 | sed "s|BOARD_KERNEL_CMDLINE := *||"
}

function get_full_path ()
{
	[ x$1 == x ] && pwd && return

	if [ -d $1 ]; then
		d=$1
		f=
	else
		d=`dirname $1`
		f=`basename $1`
	fi

	pushd $d > /dev/null 2>&1

	if [ $? == 0 ]; then
		d=`pwd`
		popd > /dev/null 2>&1
	else
		if [ ${d:0:1} == . ]; then
			if [ ${d:1:1} == / ]; then
				d=`pwd`/${d:2}
			else
				d=`pwd`/$d
			fi
		fi
	fi

	echo $d/$f
}

