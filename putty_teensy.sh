#!/bin/bash
PORT=/dev/ttyACM0
while :
do
	sleep 1
	if [ -e "$PORT" ] && [ -z "$(pgrep -f "putty -load ACM0_Teensy")" ] ; then
		echo "restart"
		putty -load ACM0_Teensy
	fi
done
