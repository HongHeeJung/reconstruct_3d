/*
stepper motor control - publish direction version

2020.07.03 Revise 'Connections to A4988' to 'built-in driver version'
2020.07.07 COM is VCC(5V) for SBD stepper motor
2020.07.08 if(enable == HIGH) ON
2020.07.15 Add ROS code for ros serial comm.
2020.07.31 Publish direction node to control the Start point
*/

#include <ros.h>
#include <std_msgs/Int16.h>
const int dirPin = 2;  // Direction 회전 방향
const int stepPin = 3; // Step  클럭을 만들어 주면 펄스 수마큼 속도가 변함
const int enPin = 4;   // Enable 1 or 연결x: WORK, 0 or GND: OFF 

const int STEPS_PER_REV = 800; // Motor steps per rotation (1.5 degree per step)
const int stepDelayMicros = 4800;
const int direction_controller = 0;

ros::NodeHandle node_;
// Create StepNo message
std_msgs::Int16 Direction;
ros::Publisher pub_direction("/direction", &Direction);

void setup() {
  // Setup the pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);

  node_.initNode();
  node_.advertise(pub_direction);
}

void loop() {
  node_.spinOnce();
  // Spin motor one rotation
  Direction.data = direction_controller;
  pub_direction.Publish(&Direction);
  digitalWrite(enPin,High);
  //revolve 180 deg.
  for(int x = 0; x < STEPS_PER_REV; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(stepDelayMicros); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(stepDelayMicros);
  }
  direction_controller = (direction_controller+1)%2;
}
