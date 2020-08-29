# Get 3d point cloud using rotating 2d Lidar

catkin_ws: ros, rplidar_ros  
rplidar_ros: rplidar a3
  
  
###### running code  
'''  

catkin_make  
source ./devel/setup.bash  
  
  
roscore  
  
#Arduino  
rosrun rosserial_python serial_node.py /dev/ttyUSB1  
  
#Lidar  
roslaunch rplidar_ros ALL.launch  
'''  
