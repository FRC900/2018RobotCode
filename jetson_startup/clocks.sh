#!/bin/bash

for i in `seq 10`
do
	sleep 15
	nvpmodel -m 0
	/home/ubuntu/jetson_clocks.sh
done

