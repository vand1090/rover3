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
//set speed value from 0 to 255   
int goNormalSpeed = 120;
int goFastSpeed = 220;
int stopTurnSpeed = 255;
int stopSpeed = 0;
int speed = 0;


//Motor Outputs
//There are 4 motors
//Left side and right side directions are tied together(left side always both in reverse or both forward, same for right) 
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

  //initialize everything to HIGH to start
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
  //driveForward();
  Serial.println("done");
}

void loop() {
  delay(100);
}
void stopMotion(){
  //set speed value from 0 to 255 based on input
  analogWrite(FOR_PWM_RIGHT, 0);
  analogWrite(REV_PWM_RIGHT, 0);
  analogWrite(FOR_PWM_LEFT, 0);
  analogWrite(REV_PWM_LEFT, 0);
  Serial.println("stopped");
}
void normalForward(){
  analogWrite(FOR_PWM_RIGHT, goNormalSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goNormalSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
  Serial.println("end of drive forward");
}
void normalBackward(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goNormalSpeed);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goNormalSpeed);
}
void normalForwardLeft(){
  analogWrite(FOR_PWM_RIGHT, goNormalSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goNormalSpeed/3);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}
void normalForwardRight(){
  analogWrite(FOR_PWM_RIGHT, goNormalSpeed/3);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goNormalSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}
void normalReverseLeft(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goNormalSpeed);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goNormalSpeed/3);
}
void normalReverseRight(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goNormalSpeed/3);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goNormalSpeed);
}
void fastForward(){
  analogWrite(FOR_PWM_RIGHT, goFastSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goFastSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}
void fastReverse(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goFastSpeed);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goFastSpeed);
}
void fastForwardLeft(){
  analogWrite(FOR_PWM_RIGHT, goFastSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goFastSpeed/3);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}
void fastForwardRight(){
  analogWrite(FOR_PWM_RIGHT, goFastSpeed/3);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, goFastSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}
void fastReverseLeft(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goFastSpeed);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goFastSpeed/3);
}
void fastReverseRight(){
  analogWrite(FOR_PWM_RIGHT, stopSpeed);
  analogWrite(REV_PWM_RIGHT, goFastSpeed/3);
  analogWrite(FOR_PWM_LEFT, stopSpeed);
  analogWrite(REV_PWM_LEFT, goFastSpeed);
}
void stopForwardRight(){//SAME AS NORMAL RIGHT
  analogWrite(FOR_PWM_RIGHT, stopTurnSpeed/3);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, stopTurnSpeed);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}
void stopForwardLeft(){// SAME AS NORMAL LEFT
  analogWrite(FOR_PWM_RIGHT, stopTurnSpeed);
  analogWrite(REV_PWM_RIGHT, stopSpeed);
  analogWrite(FOR_PWM_LEFT, stopTurnSpeed/3);
  analogWrite(REV_PWM_LEFT, stopSpeed);
}
void slowStop(){
  for (speed = goNormalSpeed; speed <= 0; i--)
    if(commandReceived==0)//if stop key is pressed
      break;
    analogWrite(FOR_PWM_RIGHT, speed);
    analogWrite(REV_PWM_RIGHT, stopSpeed);
    analogWrite(FOR_PWM_LEFT, speed);
    analogWrite(REV_PWM_LEFT, stopSpeed); 
}
void receiveData(int byteCount){
  while(Wire.available()){
    commandReceived = Wire.read();
    Serial.print("Data received: ");
    Serial.println(commandReceived);
    
      //0-15
      //0 - kill, dead stop
      //1 - Normal Forward 
      //2 - Normal Reverse 
      //3 - Normal Forward Left
      //4 - Normal Forward Right
      //5 - Normal Reverse Left
      //6 - Normal Reverse Right
      //7 - Fast Forward
      //8 - Fast Reverse
      //9 - Fast Forward Left
      //10 - Fast Forward Right
      //11 - Fast Reverse Left
      //12 - Fast Reverse Right
      //13 - Stop Forward Left
      //14 - Stop Forward Right
      //15 - Slow-stop (Ramp-down of speed instead of instant stop)

    switch (commandReceived){
      //LED stuff for testing
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

      case 0:    //kill, dead stop
        stopMotion();
        n=0;
        break;
      case 1:   //Normal Forward 
        normalForward();
        Serial.println("Going Forward");
        n=1;
        break;
      case 2:   //Normal Reverse 
        normalBackward();
        n=2;
        break;
      case 3:   //Normal Forward Left
        normalForwardLeft();
        break;
        n=3;
      case 4:   //Normal Forward Right
        normalForwardRight();
        n=4;
        break;
      case 5:   //Normal Reverse Left
        normalReverseLeft();
        n=5;
        break;
      case 6:   //Normal Reverse Right
        normalReverseRight();
        n=6;
        break;
      case 7:   //Fast Forward
        fastForward();
        n=7;
        break;
      case 8:   //Fast Reverse
        fastReverse();
        n=8;
        break;
      case 9:   //Fast Forward Left
        fastForward();
        n=9;
        break;
      case 10:  //Fast Forward Right
        fastForwardRight();
        n=10;
        break;
      case 11:  //Fast Reverse Left
        fastReverseLeft();
        n=11;
        break;
      case 12:  //Fast Reverse Right
        fastReverseRight();
        n=12;
        break;
      case 13:  //Stop Forward Left
        stopForwardLeft();
        n=13;
        break;
      case 14:  //Stop Forward Right
        stopForwardRight();
        n=14;
        break;
      case 15:  //Slow-stop
        slowStop();
        n=15;
        break;
      default:
        stopMotion(); 
      break;
    }

  }
}
void sendData(){
  Wire.write(n);
}
