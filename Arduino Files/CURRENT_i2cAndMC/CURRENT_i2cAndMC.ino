//*************************************************************************//
// Rover 3 Motor Controller Control Code - Arduino
// Author: Alexander Vanden Bussche, Kelly Low 
// Spring 2020
//*************************************************************************//

// Function: To read data from i2c and send commands to the motor controllers

// Drive Commands (0-15):
// 0 - kill, dead stop
// 1 - Normal Forward 
// 2 - Normal Reverse 
// 3 - Normal Forward Left
// 4 - Normal Forward Right
// 5 - Normal Reverse Left
// 6 - Normal Reverse Right
// 7 - Fast Forward
// 8 - Fast Reverse
// 9 - Fast Forward Left
// 10 - Fast Forward Right
// 11 - Fast Reverse Left
// 12 - Fast Reverse Right
// 13 - Slow-stop Forward
// 14 - Slow-stop Backward (Ramp-down of speed instead of instant stop)
// 15 - Spin clockwise

#include <Wire.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h"

#define ARDUINO_ADDRESS 0x04

// Motor Outputs
// 2 left side motors and 2 right side motors' directions are tied together
// (left side motors always both in reverse or forward and same goes to the right side motors) 
#define FOR_EN_RIGHT 0 // R_EN on controller unit
#define FOR_EN_LEFT 1
#define REV_EN_LEFT 2 // L_EN on controller unit
#define REV_EN_RIGHT 4

// PWM controls speed
#define FOR_PWM_RIGHT 5
#define FOR_PWM_LEFT 9
#define REV_PWM_RIGHT 6
#define REV_PWM_LEFT 10

// Initialize variables
int state = 0;// On/off state
int commandReceived = 0;// Drive commands received from i2c
int n = 0;// Drive commands executed by motor controller
int speed = 0; // Current speed 

// Set speed value from 0 to 255 (slow-fast) 
int goNormalSpeed = 120;
int goFastSpeed = 200;
int stopSpeed = 0;

void setup() {
  // Pin modes
  // Enable pins for motor controllers (on/off control)
  pinMode(FOR_EN_RIGHT, OUTPUT);
  pinMode(FOR_EN_LEFT, OUTPUT);
  pinMode(REV_EN_LEFT, OUTPUT);
  pinMode(REV_EN_RIGHT, OUTPUT);

  // Enable pin for LED
  pinMode(13, OUTPUT); 

  // PWM pins for motor controllers (speed control)
  pinMode(FOR_PWM_RIGHT, OUTPUT);
  pinMode(FOR_PWM_LEFT, OUTPUT);
  pinMode(REV_PWM_RIGHT, OUTPUT);
  pinMode(REV_PWM_LEFT, OUTPUT);

  // Initialize all motor controllers to HIGH to start
  digitalWrite(FOR_EN_RIGHT, HIGH);
  digitalWrite(FOR_EN_LEFT, HIGH);
  digitalWrite(REV_EN_LEFT, HIGH);
  digitalWrite(REV_EN_RIGHT, HIGH);
  
  Serial.begin(9600);

  // Initialize i2c connection
  Wire.begin(ARDUINO_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready");
  //driveForward();
  Serial.println("done");
}

//*************************************************************************//
// Receive data from i2c, read and execute drive function based on data
//*************************************************************************//
void receiveData(int byteCount){
  // When data is sent by i2c and received by Arduino, read data
  while(Wire.available()){
    commandReceived = Wire.read();
    Serial.print("Data received: ");
    Serial.println(commandReceived);

    // Switch cases based on command received from i2c
    switch (commandReceived){
      //LED for testing
      //n = 1;
      //  if(state==0){
      //    digitalWrite(13, HIGH);
      //    state=1;
      //  }
      //  else{
      //    digitalWrite(13, LOW);
      //    state=0;
      //  }
      //  break;

      case 0:   // Kill, dead stop
        stopMotion();
        n=0;
        break;
      case 1:   // Forward with normal speed
        normalForward();
        Serial.println("Going Forward");
        n=1;
        break;
      case 2:   // Reverse with normal speed
        normalBackward();
        n=2;
        break;
      case 3:   // Forward turn left with normal speed
        normalForwardLeft();
        break;
        n=3;
      case 4:   // Forward turn right with normal speed
        normalForwardRight();
        n=4;
        break;
      case 5:   // Reverse turn left with normal speed
        normalReverseLeft();
        n=5;
        break;
      case 6:   // Reverse turn right with normal speed
        normalReverseRight();
        n=6;
        break;
      case 7:   // Forward with fast speed
        fastForward();
        n=7;
        break;
      case 8:   // Reverse with fast speed
        fastReverse();
        n=8;
        break;
      case 9:   // Forward turn left with fast speed
        fastForwardLeft();
        n=9;
        break;
      case 10:  // Forward turn right with fast speed
        fastForwardRight();
        n=10;
        break;
      case 11:  // Reverse turn left with fast speed
        fastReverseLeft();
        n=11;
        break;
      case 12:  // Reverse turn right with fast speed
        fastReverseRight();
        n=12;
        break;
      case 13:  // Slow down and stop when moving forward
        slowForwardStop();
        n=13;
        break;
      case 14:  // Slow down and stop when moving backward
        slowBackwardStop();
        n=14;
        break;
      case 15:
        spinCW(); // Spin clockwise
        n = 15;
      default:
        stopMotion(); // Default is set to stop to ensure Rover is in a stop position when no key is pressed
      break;
    }

  }
}
// Send data to i2c 
void sendData(){
  Wire.write(n);
}

void loop() {
  delay(10);
}

//*************************************************************************//
// Drive functions definition
//*************************************************************************//
// Stop by setting all speed to 0
void stopMotion(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
  Serial.println("stopped");
}

// Forward with normal speed
void normalForward(){
  analogWrite(FOR_PWM_RIGHT, goNormalSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goNormalSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
  //Serial.println("end of drive forward");
}

// Backward with normal speed
void normalBackward(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goNormalSpeed);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goNormalSpeed);
}

// Forward turn left with normal speed
void normalForwardLeft(){
  analogWrite(FOR_PWM_RIGHT, goNormalSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goNormalSpeed/3);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}

// Forward turn right with normal speed
void normalForwardRight(){
  analogWrite(FOR_PWM_RIGHT, goNormalSpeed/3);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goNormalSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}

// Reverse turn left with normal speed
void normalReverseLeft(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goNormalSpeed);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goNormalSpeed/3);
}

// Reverse turn right with normal speed
void normalReverseRight(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goNormalSpeed/3);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goNormalSpeed);
}

// Forward with fast speed
void fastForward(){
  analogWrite(FOR_PWM_RIGHT, goFastSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goFastSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}

// Reverse with fast speed
void fastReverse(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goFastSpeed);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goFastSpeed);
}

// Forward turn left with fast speed
void fastForwardLeft(){
  analogWrite(FOR_PWM_RIGHT, goFastSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goFastSpeed/3);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}

// Forward turn right with fast speed
void fastForwardRight(){
  analogWrite(FOR_PWM_RIGHT, goFastSpeed/3);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goFastSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}

// Reverse turn left with fast speed
void fastReverseLeft(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goFastSpeed);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goFastSpeed/3);
}

// Reverse turn right with fast speed
void fastReverseRight(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goFastSpeed/3);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goFastSpeed);
}

// Forward slow down and stop
void slowForwardStop(){
  // Speed slows down in every 500ms from normal speed to stop
  for (speed = goNormalSpeed; speed >= 0; speed--){
    // If stop key is pressed, rover brakes immediately
    if(commandReceived==0){
      break;
    }
      analogWrite(FOR_PWM_RIGHT, speed);
      analogWrite(REV_PWM_RIGHT, stopSpeed);
      analogWrite(FOR_PWM_LEFT, speed);
      analogWrite(REV_PWM_LEFT, stopSpeed);
      delay(500);
  }
}

// Backward slow down and stop
void slowBackwardStop(){
  // Speed slows down in every 500ms from normal speed to stop
  for (speed = goNormalSpeed; speed >= 0; speed--){
    // If stop key is pressed, rover brakes immediately
    if(commandReceived==0){
      break;
    }
      analogWrite(FOR_PWM_RIGHT, stopSpeed);
      analogWrite(REV_PWM_RIGHT, speed);
      analogWrite(FOR_PWM_LEFT, stopSpeed);
      analogWrite(REV_PWM_LEFT, speed);
      delay(500);
  }
}

// Spins clockwise with fast speed
void spinCW(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goFastSpeed);
  analogWrite(FOR_PWM_LEFT, goFastSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}
