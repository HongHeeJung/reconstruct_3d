// Connections to A4988를 built-in driver version으로 수정
const int dirPin = 2;  // Direction
const int stepPin = 3; // Step
const int enPin = 4; // Enable

// Motor steps per rotation
const int STEPS_PER_REV = 200;
 
void setup() {
  
  // Setup the pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
}
void loop() {
  
  // Set motor direction clockwise
  digitalWrite(dirPin,HIGH); 
  digitalW
  
  // Spin motor one rotation slowly
  for(int x = 0; x < STEPS_PER_REV; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(2000); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(2000); 
  }
  
  // Pause for one second
  delay(1000); 
  
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
}
