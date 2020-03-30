#include <Wire.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h"

#define ARDUINO_ADDRESS 0x04


int n = 0;
int state = 0;
int distance = 0;
int commandReceived = 0;


//Motor Outputs
//There are 4 motors
//Left side and right side directions are tied together(left side always both in reverese or both forward, same for right) 
#define FOR_EN_RIGHT 0 //R_EN on controller unit
#define FOR_EN_LEFT 1
#define REV_EN_LEFT 2 //L_EN on controller unit
#define REV_EN_RIGHT 4

//each side is linked together
//pwm gives speed
#define FOR_PWM_RIGHT 5
#define FOR_PWM_LEFT 9
#define REV_PWM_RIGHT 6
#define REV_PWM_LEFT 10


void setup() {
  // pin modes
  //enable pins
  pinMode(FOR_EN_RIGHT, OUTPUT);
  pinMode(FOR_EN_LEFT, OUTPUT);
  pinMode(REV_EN_LEFT, OUTPUT);
  pinMode(REV_EN_RIGHT, OUTPUT);
  //pwm pins
  pinMode(FOR_PWM_RIGHT, OUTPUT);
  pinMode(FOR_PWM_LEFT, OUTPUT);
  pinMode(REV_PWM_RIGHT, OUTPUT);
  pinMode(REV_PWM_LEFT, OUTPUT);


  //initialize everything to HIGH tostart
  digitalWrite(FOR_EN_RIGHT, HIGH);
  digitalWrite(FOR_EN_LEFT, HIGH);
  digitalWrite(REV_EN_LEFT, HIGH);
  digitalWrite(REV_EN_RIGHT, HIGH);
  
  pinMode(13, OUTPUT); // LED
  Serial.begin(9600);

  //i2c stuff
  Wire.begin(ARDUINO_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready");
  driveForward();
  Serial.println("done");
}

void loop() {
  delay(1000);
}
void stopMotion(){
  //set all motors to off
  digitalWrite(FOR_EN_RIGHT, LOW);
  digitalWrite(FOR_EN_LEFT, LOW);
  digitalWrite(REV_EN_LEFT, LOW);
  digitalWrite(REV_EN_RIGHT, LOW);

  //set speed value from 0 to 255 based on input

}
void driveForward(){
  //set speed value from 0 to 255 based on input
  analogWrite(FOR_PWM_RIGHT, 150);
  analogWrite(REV_PWM_RIGHT, 0);
   analogWrite(FOR_PWM_LEFT, 150);
  analogWrite(REV_PWM_LEFT, 0);
  //Serial.println(mc2_pwm_R);
}
void driveBackward(){

  //set speed value from 0 to 255 based on input   
}

void turnLeft(){


  //set speeds froom 0 to 255 based on input
}
void turnRight(){


  //set speeds froom 0 to 255 based on input
}
void receiveData(int byteCount){
  while(Wire.available()){
    commandReceived = Wire.read();
    Serial.print("Data received: ");
    Serial.println(commandReceived);
    
    //1 flips LED state
    //2 drives motor forward
    //3 drives motor backward
    //4 turns left
    //5 turns right
    //6 stops rover

    switch (commandReceived){
      //LED stuff for testing
      case 1:
        n = 1;
        if(state==0){
          digitalWrite(13, HIGH);
          state=1;
        }
        else{
          digitalWrite(13, LOW);
          state=0;
        }
        break;
      case 2:   //drive forward state
        //driveForward();
        n=2;
        break;
      case 3:   //drive back state
        //driveBackward();
        n=3;
        break;
      case 4:   //turn left
        //turnLeft();
        break;
        n=4;
      case 5:   //turn Right
        //turnRight();
        n=5;
        break;
      case 6:   //stop
        //stopMotion();
        n=6;
        break;
      default:
        //stopMotion(); 
      break;
    }
        
    if(n==1){
      
    }
  }
}
void sendData(){
  Wire.write(n);
}
