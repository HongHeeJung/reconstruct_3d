#!/bin/sh
pushd .

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1

cd ~/catkin_ws_3
catkin_make
source ./devel/setup.bash

sudo notilus rosrun rosserial_python serial_node.py /dev/ttyUSB1
if [ "$?" != "0" ]; then
	echo "Cannot run Arduino" 1>&2
    popd
	exit 1
fi

sudo notilus terminal roslaunch realsense2_camera rs_t265.launch
if [ "$?" != "0" ]; then
	echo "Cannot run Real-sense camera" 1>&2
    popd
	exit 1
fi

sudo notilus terminal roslaunch loam_rotating_2d ALL.launch
if [ "$?" != "0" ]; then
	echo "Cannot launch loam_rotating_2d" 1>&2
    popd
	exit 1
fi

popd