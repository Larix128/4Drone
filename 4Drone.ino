#include <Servo.h>
#include "MPU9250.h"

MPU9250 mpu;

Servo esc1; //yellow wire
Servo esc2; //green wire
Servo esc3; //orange wire
Servo esc4; //blue wire

byte esc1PIN = 10;
byte esc2PIN = 11;
byte esc3PIN = 12;
byte esc4PIN = 13;

float gyroX; // -179.5 to 180.5
float gyroY; // -178.5 to 181.5
float gyroZ; // -180 to 180

volatile int thro_input_raw; //throttle value; min 1104 max 1912 
float thro_input; //throttle value corrected to 0-1 float
volatile int prev_thro_time = 0; // time of last thro reading

void setup() {
  Serial.begin(115200);
  Wire.begin();

  esc1.attach(esc1PIN);
  esc2.attach(esc2PIN);
  esc3.attach(esc3PIN);
  //esc4.attach(esc4PIN);

  esc1.write(30);
  esc2.write(30);
  esc3.write(30);
  //esc4.write(30);
  delay(2000);

  mpu.setup(0x68);  // change to your own address
  attachInterrupt(0, thro_rising, RISING); //calls thro_rising() when pin goes up
}

void loop() {
  if (mpu.update()) {
    gyroX = (mpu.getEulerX()+0.5);
    gyroY = (mpu.getEulerY()+1.5);
    gyroZ = mpu.getEulerZ()/180;
    //Serial.print(gyroX);
    //Serial.print(" ");
    //Serial.print(gyroY);
    //Serial.println("");
  }
 
  esc1.write(thro_input*100 + thro_input*(-gyroY) + thro_input*gyroX);
  esc2.write(thro_input*100 + thro_input*(-gyroY) + thro_input*(-gyroX));
  esc3.write(thro_input*100 + thro_input*gyroY + thro_input*(-gyroX));
  //esc4.write(thro_input*100 + thro_input*gyroY + thro_input*gyroX);
}




void thro_rising() {
  attachInterrupt(0, thro_falling, FALLING);
  prev_thro_time = micros();
}
void thro_falling() {
  attachInterrupt(0, thro_rising, RISING);
  thro_input_raw = micros()-prev_thro_time;
  thro_input = abs((float(thro_input_raw) - 1108)/804); //abs is just bodge, need to replace by ifs
  //Serial.println(thro_input);
}
