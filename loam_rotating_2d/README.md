# Reconstruct 3d map using 2d Lidar

rplidar_ros: rplidar a3
  
  
## running code  
```
roscore  
```
  
### Use commands individually
```
catkin_make  
source ./devel/setup.bash  
```
  
```
#Arduino  
rosrun rosserial_python serial_node.py /dev/ttyUSB1  
```
  
```
#Realsense Camera  
roslaunch realsense2_camera rs_t265.launch  
```
  
```
#Lidar  
roslaunch loam_rotating_2d ALL.launch  
```
  
### Use script
```
bash loam_rotating_2d.sh
```
