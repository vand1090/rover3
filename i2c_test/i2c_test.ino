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

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);

  Wire.begin(ARDUINO_ADDRESS);
  Wire.begin(LIDAR_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendDistance);

  Serial.println("Ready");
}

void loop() {
  
  delay(1000);

}
void receiveData(int byteCount){
  while(Wire.available()){
    n = Wire.read();
    Serial.print("Data received: ");
    Serial.println(n);

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
void sendDistance(){
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.stopRanging();
  
  Serial.print("Distance(mm): ");
  Serial.println(distance);
  Wire.write(distance);
}
