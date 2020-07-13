/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rplidar_ros/Control.h"

#define RAD2DEG(x) ((x)*180./M_PI)

rplidar_ros::Control pubToArdu;

float angle_detected[500] = {0.0,}; // 360개만 해도 충분하지만. 넉넉하게
float distance_detected[500] = {0.0,}; 

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment; // 한바퀴에 얻는 포인트 갯수
    // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count); //%s = laser // cont는 358, 359 360과 같이 다양하다.
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max)); // 감지한 angle의 최대 최솟 값. 계속 바뀜
  
    // 그 한바퀴에 대한 포인트 값을 출력한다. 즉 360개의 point에 대한 정보를 print한다.
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]); // degree //  scan->ranges[i]
        angle_detected[i] = degree;
        distance_detected[i] = scan->ranges[i];
    }
    /*
    사용할 수 있는 변수
    1. count : 한 바퀴 회전하면서, 얻은 포인트의 갯수
    2. scan->angle_mim : 아래의 angle_detected 내부의 값 중 최솟값
    3. scan->angle_max : 아래의 angle_detected 내부의 값 중 최댓값
    4. angle_detected[i] : 바로 위의 for에 의해, [0~count-1]까지 값이 수정된다. 
    5. distance_detected[i] : 바로 위의 for에 의해, [0~count-1]까지 값이 수정된다. 
    주의! 직전 count가 359이고, 지금의 count가 358이라면 배열의 [358]는 이전 값이 저장되어 있다.
    */
    // 알고리즘 구현 START ********************************
    
    // 알고리즘 구현 END **********************************
    // 위의 구현 부분에 다음의 변수에 값을 대입해야 한다.
    pubToArdu.velocity_on = true;           // 이 변수의 type = Bool
    pubToArdu.streer_on = false;            // 이 변수의 type = Bool
    pubToArdu.velocity = 0;                 // 이 변수의 type = signed int (4byte)
    pubToArdu.streer_angle = 0;             // 이 변수의 type = signed int (4byte)
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::Publisher pub = n.advertise<rplidar_ros::Control>("/ToArdu", 1000);	

    while(ros::ok()){  
        pub.publish(pubToArdu);		
        ros::spinOnce();
    }

    // ros::spin();

    return 0;
}
