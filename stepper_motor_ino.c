/*
2020.07.03 Connections to A4988를 built-in driver version으로 수정
2020.07.07 SBD 기준 COM이 VCC
2020.07.08 enable이 HIGH일 때 작동
*/

const int dirPin = 2;  // Direction 회전 방향
const int stepPin = 3; // Step  클럭을 만들어 주면 펄스 수마큼 속도가 변함
const int enPin = 4; // Enable 1 or 연결x: WORK, 0 or GND: OFF 

// Motor steps per rotation
const int STEPS_PER_REV = 180;
const int stepDelayMicros = 1000;

void setup() {
  
  // Setup the pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  
  digitalWrite(enPin,HIGH);
}

void loop() {
  
  digitalWrite(dirPin,HIGH); // Set motor direction clockwise
  
  // Spin motor one rotation slowly
  for(int x = 0; x < STEPS_PER_REV; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(stepDelayMicros); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(stepDelayMicros); 
  }
  
  // Pause for one second
  //delay(1000); //딜레이 없앰
  
  // clockwise로만 연속적으로 회전하도록 주석처리 함
    /*
  // Set motor direction counterclockwise
  digitalWrite(dirPin,LOW);

  // Spin motor two rotations quickly
  for(int x = 0; x < (STEPS_PER_REV * 2); x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(1000);
  }
  
  // Pause for one second
  delay(1000);
  */
}
