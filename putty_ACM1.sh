#!/bin/bash
PORT=/dev/ttyACM1
while :
do
	sleep 1
	if [ -e "$PORT" ] && [ -z "$(pgrep -f "putty -load ACM1")" ] ; then
		echo "restart"
		putty -load ACM1
	fi
done
