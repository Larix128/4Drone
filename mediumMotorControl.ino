#include <Servo.h>
#include "MPU9250.h"

MPU9250 mpu;



Servo esc1; //yellow
Servo esc2; //green
Servo esc3; //orange


void setup() {
  Serial.begin(115200);
  Wire.begin();
  esc1.attach(8);
  esc2.attach(9);
  esc3.attach(10);
  esc1.write(30);
  esc2.write(30);
  esc3.write(30);
  delay(2000);

  mpu.setup(0x68);  // change to your own address
}

void loop() {
  if (mpu.update()) {
    Serial.print(mpu.getYaw()); Serial.print(", ");
    Serial.print(mpu.getPitch()); Serial.print(", ");
    Serial.println(mpu.getRoll());
    
    
    esc1.write((mpu.getYaw()+180)/4);
    esc2.write((mpu.getPitch()+180)/4);
    esc3.write((mpu.getRoll()+180)/4);

    
  
  }
}
