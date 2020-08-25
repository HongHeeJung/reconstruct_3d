#!/bin/sh
pushd .

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1

cd ~/catkin_ws_3
catkin_make
source ./devel/setup.bash

echo "============================== Running loam_rotating_2d =============================="
gnome-terminal -- rosrun rosserial_python serial_node.py /dev/ttyUSB1
echo $0
if [ "$?" != "0" ]; then
	echo "Cannot run Arduino" 1>&2
    popd
	exit 1
fi
sleep 1

gnome-terminal -- roslaunch realsense2_camera rs_t265.launch
echo $0
if [ "$?" != "0" ]; then
	echo "Cannot run Real-sense camera" 1>&2
    popd
	exit 1
fi
# Wait until real-sense node is on.
sleep 3

gnome-terminal -- roslaunch loam_rotating_2d ALL.launch
echo $0
if [ "$?" != "0" ]; then
	echo "Cannot launch loam_rotating_2d" 1>&2
    popd
	exit 1
fi

echo "============================== All process is ON! =============================="

popd