# Reconstruct 2.5d map using 2d Lidar

catkin_ws: ros, mavros, mavlink, px4 firmware  
rplidar_ros: rplidar a3
  
  
###### running code  
'''  
source ./devel/setup.bash  
catkin_make  
roscore  
#Arduino  
rosrun rosserial_python serial_node.py /dev/ttyUSB1  
#Lidar  
roslaunch rplidar_ros ALL.launch  
'''  
