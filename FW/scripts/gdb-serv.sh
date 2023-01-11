#!/bin/bash

function usage()
{
	cat << HEREDOC
	Usage: $progname <openocd-stlink, openocd-jlink, openocd-mculink, jlink>"
HEREDOC
}

# save current dir
cwd=$(pwd)
# cd to script dir
DIR="$(cd "$(dirname "$0")" && pwd)"
cd $DIR

if [ "$#" -ne 1 ]; then
	print_usage
fi

if [ "$1" == "openocd-stlink" ]; then
	echo "start" $1
	openocd -f interface/stlink.cfg -c "transport select hla_swd" -f openocd.cfg

elif [ "$1" == "openocd-jlink" ]; then
	echo "start" $1
	openocd -f interface/jlink.cfg -c "transport select swd" -f openocd.cfg \
			-c "rtt setup 0x20000410 1024 \"SEGGER RTT\"" \
			-c "rtt start" \
			-c "rtt server start 1237 0"

elif [ "$1" == "openocd-mculink" ]; then
	echo "start" $1
	openocd -f interface/cmsis-dap.cfg -c "transport select swd" -f openocd.cfg
	
elif [ "$1" == "jlink" ]; then
	echo "start" $1
	while read line; do
		FLAG+='-'$line' '
	done < jlink.cfg
	echo $FLAG
	JLinkGDBServer $FLAG

else
	usage
fi

# restore current dir
cd $cwd
