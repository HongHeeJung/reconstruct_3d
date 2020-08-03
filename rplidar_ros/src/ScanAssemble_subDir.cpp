/* 
lidar control - 1.5 degree division + subscribe direction Version

      This code was written by Jonathan Wheadon and Ijaas at Plymouth university for module ROCO318
      DATE of last update : 8/11/2018

      2020.07.09 Revise 360 degree laser scanning
      2020.07.20 Change laser scan division to 1.5 degree
      Affine transformation matrix for [approx 1 degree]
      2020.07.21 For Arduino ROS comm.
      2020.07.24 Change Scan data & Vector3f::UnitX() -> [X] axis -> if(direction == 0) CW
      2020.07.31 Add subscriber to get the direction

*/

#include <ros/ros.h>
#include <tf/transform_listener.h> // ROS library
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

using namespace Eigen; //space transformation 
using namespace ros;

// Variable to count how many 2d scans have been taken
int ScanNo = 0; // Devide 180 deg -> ScanNO: counter
int direction = 5; // 5 -> Not working
int start_motor = 0;
const float space_radian = 0.0261799;
const int degree_offset = 120;

// Variables store the previous cloud and fully assembled cloud
pcl::PointCloud<pcl::PointXYZ> oldcloud;
pcl::PointCloud<pcl::PointXYZ> assembledCloud;
sensor_msgs::PointCloud2 OutPutCloud;

class ScanDirection {
    public:
        ScanDirection();
        int dirCallback(const std_msgs::Int16::ConstPtr& dir);
    private:
        NodeHandle node_;
        Subscriber direction_sub_;
};

ScanDirection::ScanDirection(){
    direction_sub_ = node_.subscribe<std_msgs::Int16> ("/direction", 10, boost::bind(&ScanDirection::dirCallback, this, _1));
}

int ScanDirection::dirCallback(const std_msgs::Int16::ConstPtr& dir){
    direction = dir->data;
    ROS_INFO("Scan Direction: [%d]", dir->data);
}

// Create ScanAssembler class
class ScanAssembler {
    public:
        ScanAssembler();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    private:
        NodeHandle node_;
        tf::TransformListener tfListener_;
        laser_geometry::LaserProjection projector_;

	Publisher start_pub_;
        Subscriber scan_sub_;
        Publisher full_point_cloud_publisher_;
};

ScanAssembler::ScanAssembler(){
    start_pub_ = node_.advertise<std_msgs::Int16> ("/start", 1, false);
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1000, &ScanAssembler::scanCallback, this);
    // Point cloud on RVIZ
    full_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/fullcloud", 1000, false);
}

void ScanAssembler::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    // Create start variable
    std_msgs::Int16 Start;
    start_motor = 1;
    Start.data = start_motor;
    start_pub_.publish(Start);

    // Convert laser scan to point cloud
    sensor_msgs::PointCloud2 cloud;
    // Transform a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2 in target frame
    projector_.transformLaserScanToPointCloud("/laser", *scan, cloud, tfListener_);

    // Convert ROS point cloud into PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> TempCloud;
    pcl::fromROSMsg(cloud, TempCloud);

    // Initialise PCL point cloud for storing rotated cloud
    pcl::PointCloud<pcl::PointXYZ>  RotatedCloud;

    // Create Affine transformation matrix for 0.0174533 radians around Z axis (approx 1 degree) of rotation with no translation.
    Affine3f RotateMatrix = Affine3f::Identity();
    RotateMatrix.translation() << 0.0, 0.0, 0.0;

    // Rotate matrix has offset of 90 degrees to set start pos
    if(direction == 0){
        RotateMatrix.rotate (AngleAxisf (((ScanNo) * space_radian), Vector3f::UnitX()));
    } else if(direction == 1){
        RotateMatrix.rotate (AngleAxisf (((ScanNo + degree_offset) * space_radian), Vector3f::UnitX()));
    } else if(direction == 2 || direction == 3){
	ROS_INFO("Reset ScanNo");
    } else {
	// wait topic
	ROS_INFO("No Direction Signal");
    }

    // Rotate Tempcloud by rotation matrix timesed by the scan number, AKA how many scan have been taken.
    pcl::transformPointCloud (TempCloud, RotatedCloud, RotateMatrix);

    //  If this is first scan save rotated cloud as old cloud
    //  if this is not first scan concatanate rotated cloud and oldcloud
    //  together save new cloud as old cloud and output to rviz
    if(direction == 0 || direction == 1){
        if(ScanNo > 0){
            assembledCloud = oldcloud + RotatedCloud;
            pcl::copyPointCloud(assembledCloud, oldcloud);
            ScanNo += 1.5;
        } else {
            oldcloud = RotatedCloud;
            assembledCloud = RotatedCloud;
            ScanNo += 1.5;
        }
    } else if(direction == 2 || direction == 3){
	ScanNo = 0;
    } else {
	ROS_INFO("WAIT...");
    }

    // Convert Assembled cloud to ROS cloud and publish all
    pcl::toROSMsg(assembledCloud, OutPutCloud);
    full_point_cloud_publisher_.publish(OutPutCloud);

    // Return to spin until next laser scan taken
    return;
}

int main(int argc, char** argv)
{
    // You must call one of the versions of ros::init() before using any other part of the ROS system
    init(argc, argv, "ScanAssembler");
    ScanAssembler assembler;
    ScanDirection direct;
    spin();

    return 0;
}
