/*
2020.07.03 Connections to A4988를 built-in driver version으로 수정
2020.07.07 SBD 기준 COM이 VCC
2020.07.08 enable이 HIGH일 때 작동
2020.07.15 ROS code 추가
*/

#include <ros.h>
#include <std_msgs/Int16.h>
const int dirPin = 2;  // Direction 회전 방향
const int stepPin = 3; // Step  클럭을 만들어 주면 펄스 수마큼 속도가 변함
const int enPin = 4;   // Enable 1 or 연결x: WORK, 0 or GND: OFF 

const int STEPS_PER_REV = 360; // Motor steps per rotation
const int stepDelayMicros = 4750;

void CallBack(const std_msgs::Int16& control);

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Int16> sub("/control", &CallBack);

void setup() {
  // Setup the pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);
  //digitalWrite(enPin,HIGH); //for motor only test

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  // Spin motor one rotation
  for(int x = 0; x < STEPS_PER_REV; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(stepDelayMicros); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(stepDelayMicros);
  }
  digitalWrite(enPin,LOW);
}

void CallBack(const std_msgs::Int16& control)
{
  // direction of rotation
  if(control.data == 0) {
    digitalWrite(enPin,HIGH);
    digitalWrite(dirPin,HIGH); // Set motor direction clockwise
  } else {
    digitalWrite(enPin,HIGH);
    digitalWrite(dirPin,LOW); // Set motor direction counterclockwise
  }
}
