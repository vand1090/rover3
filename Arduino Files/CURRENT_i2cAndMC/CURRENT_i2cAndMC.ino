#include <Wire.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h"

#define ARDUINO_ADDRESS 0x04
#define LIDAR_ADDRESS 0x29
SFEVL53L1X distanceSensor;

int n = 0;
int state = 0;
int distance = 0;


//Motor Outputs
//There are 4 motors
//Left side and right side directions are tied together(left side always both in reverese or both forward, same for right) 
int FOR_EN_RIGHT = 0; //R_EN on controller unit
int FOR_EN_LEFT = 0;
int REV_EN_LEFT = 0; //L_EN on controller unit
int REV_EN_RIGHT = 0;

//Same pin will control left and right PWM, assuming the motor controllers are okay with this
//pwm gives speed
int mc1_pwm = 0;
int mc2_pwm = 0;
int mc3_pwm = 0;
int mc4_pwm = 0;

void setup() {
  // pin modes
  //enable pins
  pinMode(FOR_EN_RIGHT, OUTPUT);
  pinMode(FOR_EN_LEFT, OUTPUT);
  pinMode(REV_EN_LEFT, OUTPUT);
  pinMode(REV_EN_RIGHT, OUTPUT);
  //pwm pins
  pinMode(mc1_pwm, OUTPUT);
  pinMode(mc2_pwm, OUTPUT);
  pinMode(mc3_pwm, OUTPUT);
  pinMode(mc4_pwm, OUTPUT);

  //initialize everything to LOW
  digitalWrite(FOR_EN_RIGHT, LOW);
  digitalWrite(FOR_EN_LEFT, LOW);
  digitalWrite(REV_EN_LEFT, LOW);
  digitalWrite(REV_EN_RIGHT, LOW);
  digitalWrite(mc1_pwm, LOW);
  digitalWrite(mc2_pwm, LOW);
  digitalWrite(mc3_pwm, LOW);
  digitalWrite(mc4_pwm, LOW);
  
  pinMode(13, OUTPUT); // LED
  Serial.begin(9600);

  
  //i2c stuff
  Wire.begin(ARDUINO_ADDRESS);
  //Wire.begin(LIDAR_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready");
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
  //make sure reverse pins are disabled
  digitalWrite(REV_EN_LEFT, LOW);
  digitalWrite(REV_EN_RIGHT, LOW);
  //enable forward drive 
  digitalWrite(FOR_EN_RIGHT, HIGH);
  digitalWrite(FOR_EN_LEFT, HIGH);

  //set speed value from 0 to 255 based on input

}
void driveBackward(){
  //make sure forward pins are disabled
  digitalWrite(FOR_EN_RIGHT, LOW);
  digitalWrite(FOR_EN_LEFT, LOW);

  //enable reverse pins
  digitalWrite(REV_EN_RIGHT, HIGH);
  digitalWrite(REV_EN_LEFT, HIGH);

  //set speed value from 0 to 255 based on input   
}

void turnLeft(){
  //make all pins low first, to avoid errors
  digitalWrite(FOR_EN_RIGHT, LOW);
  digitalWrite(FOR_EN_LEFT, LOW);
  digitalWrite(REV_EN_LEFT, LOW);
  digitalWrite(REV_EN_RIGHT, LOW);

  //set left pins to reverse
  digitalWrite(REV_EN_LEFT, HIGH);
  //set right pins to forward
  digitalWrite(FOR_EN_RIGHT, HIGH);  

  //set speeds froom 0 to 255 based on input
}
void turnRight(){
  //make all pins low first, to avoid errors
  digitalWrite(FOR_EN_RIGHT, LOW);
  digitalWrite(FOR_EN_LEFT, LOW);
  digitalWrite(REV_EN_LEFT, LOW);
  digitalWrite(REV_EN_RIGHT, LOW);

  //set right pins to reverse
  digitalWrite(REV_EN_RIGHT, HIGH);
  //set right pins to forward
  digitalWrite(FOR_EN_LEFT, HIGH);  

  //set speeds froom 0 to 255 based on input
}
void receiveData(int byteCount){
  while(Wire.available()){
    n = Wire.read();
    Serial.print("Data received: ");
    Serial.println(n);
    
    //1 flips LED state
    //2 drives motor forward
    //3 drives motor backward
    //4 turns left
    //5 turns right
    //6 stops rover
    if(n==1){
      if(state==0){
        digitalWrite(13, HIGH);
        state=1;
      }
      else{
        digitalWrite(13, LOW);
        state=0;
      }
    }
  }
}
void sendData(){
  Wire.write(state);
}
void sendDistance(){
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.stopRanging();
  
  Serial.print("Distance(mm): ");
  Serial.println(distance);
  Wire.write(distance);
}
