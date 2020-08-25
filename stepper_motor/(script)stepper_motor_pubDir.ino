/*
stepper motor control - publish direction version

    2020.07.03 Revise 'Connections to A4988' to 'built-in driver version'
    2020.07.07 COM is VCC(5V) for SBD stepper motor
    2020.07.08 if(enable == HIGH) ON
    2020.07.15 Add ROS code for ros serial comm.
    2020.07.31 Publish direction node to control the Start point
    2020.08.01 Add delay microseconds before runnung the motor
    2020.08.24 Faster version
    2020.08.25 Add command for exit when revolution is up to 180 deg. - X
    2020.08.25 Add one_revolution variable to control motor position.
*/

#include <ros.h>
#include <std_msgs/Int16.h>

/*
#include <stdio.h>
#include <coino.h>
#define MAX_LEN_LINE    10
*/

const int dirPin = 2;  // Direction of revolution
const int stepPin = 3; // Step makes clock that control pulse.
const int enPin = 4;   // Enable 1 or connection X: WORK, 0 or GND: OFF 

const int STEPS_PER_REV = 800; // Motor steps per rotation (1.5 degree per step)
const int stepDelayMicros = 3500;
int direction_controller = -1;
int s = 0; // Global variable of start.data
int one_revolution = 1; // Alert motor revolve 180 degree.

void startCallback(const std_msgs::Int16& start);

ros::NodeHandle node_;
// Create Direction message
std_msgs::Int16 Direction;
ros::Subscriber<std_msgs::Int16> sub_start("/start", &startCallback);
ros::Publisher pub_direction("/direction", &Direction);

void setup() {
  // Setup the pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);
  //digitalWrite(dirPin,LOW); //CCW
  digitalWrite(dirPin,HIGH); //CW
  
  
  Direction.data = direction_controller;
  pub_direction.publish(&Direction);
 
  node_.initNode();
  node_.advertise(pub_direction);
  node_.subscribe(sub_start);
}

void startCallback(const std_msgs::Int16& start){
  s = start.data;
  if((start.data == 0) && (one_revolution == 1)){
    digitalWrite(enPin,LOW);
    Serial.print("========== WAITTING... ==========");
  } else {
    digitalWrite(enPin,HIGH);
    Serial.print("========== START! ==========");
  }
}

void loop() {
  node_.spinOnce();
  // Spin motor one rotation
  if(s == 1){
    Direction.data = direction_controller;
    pub_direction.publish(&Direction);
    printf("========== Direction: [%d] ==========", direction_controller);
  }
  
  //revolve 180 deg.
  one_revolution = 0;
  for(int x = 0; x < STEPS_PER_REV; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(stepDelayMicros); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(stepDelayMicros);
  }
  one_revolution = 1;
  direction_controller = (direction_controller+1)%2;
}
/*
// 이거 안됨 ㅜㅜ
int main(){
  char command[MAX_LEN_LINE];
  char *args[] = {command, NULL};
  
  while (true) {
    char *s;
    int len;
    printf("================================= My Shell is ON =================================");
    s = fgets(command, MAX_LEN_LINE, stdin);
    if (s == NULL){
      fprintf(stderr, "Running...\n");
      exit(1);
    }
      
    len = strlen(command);
    printf("%d\n", len);
    if(command[len - 1] == '\n'){
      command[len - 1] = '\0'; 
    } 

    // exit 입력시 shell 종료 
    if((!strcmp("exit",command)) && (StepNo.data == 120)){
      printf("exit!");
      return 0;
    }
    printf("[%s]\n", command);
  }
  return 0;
}
*/
