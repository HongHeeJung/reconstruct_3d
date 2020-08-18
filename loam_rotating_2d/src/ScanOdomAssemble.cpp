/* 
lidar control - 1.5 degree division + subscribe direction Version
      
      This code was written by Jonathan Wheadon and Ijaas at Plymouth university for module ROCO318
      DATE of last update : 8/11/2018
      +
      Revised by Hee Jung Hong for LOAM
      
      2020.07.09 Revise 360 degree laser scanning
      2020.07.20 Change laser scan division to 1.5 degree
      Affine transformation matrix for [approx 1 degree]
      2020.07.21 For Arduino ROS comm.
      2020.07.24 Change Scan data & Vector3f::UnitX() -> [X] axis -> if(direction == 0) CW
      2020.07.31 Add subscriber to get the direction
      2020.08.04 Initialized by direction topic
      2020.08.05 Done for scanning 3d
      2020.08.12 Add odometry
*/

// Laser 2 Point cloud
#include <ros/ros.h>
#include <tf/transform_listener.h>
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

// Odometry
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace Eigen;
using namespace ros;

// Variable to count how many 2d scans have been taken
int ScanNo = 0;
int direction = -1; // Not working
int start_motor = 0;
const float space_radian = 0.0261799;
const int degree_offset = 120;

// Variables store the previous cloud and fully assembled cloud
pcl::PointCloud<pcl::PointXYZ> oldcloud;
pcl::PointCloud<pcl::PointXYZ> assembledCloud;
sensor_msgs::PointCloud2 OutPutCloud;

// Array  to get odometry data
double timeLaserOdometry;
bool newLaserOdometry = false;
float odom[10] = {0};

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  timeLaserOdometry = laserOdometry->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  odom[0] = -pitch;
  odom[1] = -yaw;
  odom[2] = roll;

  odom[3] = laserOdometry->pose.pose.position.x;
  odom[4] = laserOdometry->pose.pose.position.y;
  odom[5] = laserOdometry->pose.pose.position.z;

  odom[6] = laserOdometry->pose.pose.orientation.x;
  odom[7] = laserOdometry->pose.pose.orientation.y;
  odom[8] = laserOdometry->pose.pose.orientation.z;
  odom[9] = laserOdometry->pose.pose.orientation.w;

  newLaserOdometry = true;
  ROS_INFO("Subscribe Odometry");
  ROS_INFO("P: [%f], Y: [%f], R: [%f]", odom[0],odom[1],odom[2]);
  ROS_INFO("position -> x: [%f], y: [%f], z: [%f]", odom[3],odom[4],odom[5]);
  ROS_INFO("orientation -> x: [%f], y: [%f], z: [%f], w: [%f]", odom[6],odom[7],odom[8],odom[9]);
}

class ScanDirection {
    public:
        ScanDirection();
        int dirCallback(const std_msgs::Int16::ConstPtr& dir);
    private:
        NodeHandle node_;
        Subscriber direction_sub_;
};

ScanDirection::ScanDirection(){
    direction_sub_ = node_.subscribe<std_msgs::Int16> 
                                ("/direction", 10, boost::bind(&ScanDirection::dirCallback, this, _1));
}

int ScanDirection::dirCallback(const std_msgs::Int16::ConstPtr& dir){
    int new_direction = dir->data;
    if(new_direction != direction){
        ScanNo = 0;
        ROS_INFO("========== Initialize [ScanNo] ==========");
        direction = new_direction;
        ROS_INFO("Scan Direction: [%d]", dir->data);
    }
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
        Subscriber subLaserOdometry;
        Publisher full_point_cloud_publisher_;
};

ScanAssembler::ScanAssembler(){
    start_pub_ = node_.advertise<std_msgs::Int16> ("/start", 1, false);
    subLaserOdometry = node_.subscribe<nav_msgs::Odometry> 
                                ("/camera/odom/sample", 5, laserOdometryHandler);
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> 
                                ("/scan", 1000, &ScanAssembler::scanCallback, this);
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

    // Create Affine transformation matrix(3x3) for space_radians around X axis (approx 1 degree) of rotation with no translation.
    Affine3f RotateMatrix = Affine3f::Identity();
    RotateMatrix.translation() << odom[3], odom[4], odom[5];
    ROS_INFO("=============== Translated ===============");

    // Rotate matrix has offset of 90 degrees to set start pos
    if(direction == 0){
        RotateMatrix.rotate (AngleAxisf (((ScanNo) * space_radian), Vector3f::UnitX()));
    } else if(direction == 1){
        RotateMatrix.rotate (AngleAxisf (((ScanNo + degree_offset) * space_radian), Vector3f::UnitX()));
    } else {
	    // wait topic
	    ROS_INFO("No Direction Signal.");
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
            ScanNo += 1;
        } else {
            oldcloud = RotatedCloud;
            assembledCloud = RotatedCloud;
            ScanNo += 1;
        }
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
