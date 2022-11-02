#include <Servo.h>
#include "MPU9250.h"
#include "4Drone.h"

MPU9250 mpu;
//escs calibrated to servowrite(50-135)
//need to implement angular bias to flight calc
//need to correct/calibrate rc readings to be exact(0=lowest, 0.5=middle, 1=highest)
//       F
//    1     2
//      \ /
// L     X     R
//      / \
//    3     4
//       B
Servo esc1; //  green wire 
Servo esc2; // yellow wire golden motor
Servo esc3; //   blue wire
Servo esc4; // orange wire

//#define
#define esc1PIN 4
#define esc2PIN 5
#define esc3PIN 6
#define esc4PIN 7
#define gyroInfl 25  //how much does gyro influence motors
#define throInfl 73  //how much does thro influence motors
#define deltaThro 720
#define deltaElev 744
#define deltaAile 704
#define deltaRudd 728
#define failsafeTreshold 10




int failure = 0;//if something goes wrong add 1 (failsafe triggers when >= failsafeTreshold)


//gyro
struct Vector3 gyro;//-0.5 to 0.5
struct Vector3 gyroOffset;
float angBias;//thrust multiplier to keep the upward force constant
int gyroCalibrated = 0;

struct Vector3 desiredAngle; 
struct Vector3 levelAngle;
struct Vector3 currentAngle;

//RC inputs
volatile int ThroStart;
volatile float Thro;//brown
volatile int ElevStart;
volatile float Elev;//yellow
volatile int AileStart;
volatile float Aile;//green
volatile int RuddStart;
volatile float Rudd;//orange


int currentTime;


















void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  //Serial.println("script iniciated");
  
  esc1.attach(esc1PIN);
  esc2.attach(esc2PIN);
  esc3.attach(esc3PIN);
  esc4.attach(esc4PIN);


  //batteryWait();
  calibrateESCs();
  
  digitalWrite(LED_BUILTIN, HIGH);
  mpu.setup(0x68);  //gyro: change to my own address
  delay(2000); 


  //RC read 
  attachInterrupt(0, calcThro, CHANGE);
  attachInterrupt(1, calcElev, CHANGE);
  attachInterrupt(4, calcAile, CHANGE);
  attachInterrupt(5, calcRudd, CHANGE); 
}



void loop() {
  Serial.println(micros());
  calcGyro();
  //printGyro();
  struct ESCs escs = {50, 50, 50, 50};
  
  if (Thro != 0 && failure < failsafeTreshold) {  //if throttle is 0 do not calculate (send 50 instead)
    escs.e1 = (50 + Thro*throInfl + gyroInfl*(+ gyro.x - gyro.y));
    escs.e2 = (50 + Thro*throInfl + gyroInfl*(+ gyro.x + gyro.y));
    escs.e3 = (50 + Thro*throInfl + gyroInfl*(- gyro.x - gyro.y));
    escs.e4 = (50 + Thro*throInfl + gyroInfl*(- gyro.x + gyro.y));
  }
  //printRC();
  //printESC(escs);
  esc1.write(escs.e1);
  esc2.write(escs.e2);
  esc3.write(escs.e3);
  esc4.write(escs.e4);
}
















void calcThro() {
    if(digitalRead(2) == HIGH) {
    ThroStart = micros();
  }
  else {
    Thro = float((uint16_t)(micros() - ThroStart));
    
    if (Thro < 1125) {//avoid unexpected underflow behavior & bottom cap
      Thro = 0;
    }
    else {
      Thro = Thro - 1125;
    }

    if(Thro > deltaThro + 100) { //failsafe
      Thro = 0;
      failure = failure + 1;
    }
    else if (Thro > deltaThro) { //top cap
      Thro = 1;
    }
    else {
      Thro = Thro/deltaThro; 
    }
  }
}

void calcElev() {
  if(digitalRead(3) == HIGH) {
    ElevStart = micros();
  }
  else {
    Elev = float((uint16_t)(micros() - ElevStart));
    
    if (Elev < 1125) {//avoid unexpected underflow behavior & bottom cap
      Elev = 0;
    }
    else {
      Elev = Elev - 1125;
    }

    if(Elev > deltaElev + 100) { //failsafe
      Elev = 0;
      failure = failure + 1;
    }
    else if (Elev > deltaElev) { //top cap
      Elev = 1;
    }
    else {
      Elev = Elev/deltaElev; 
    }
  }
}

void calcAile() {
  if(digitalRead(19) == HIGH) {
    AileStart = micros();
  }
  else {
    Aile = float((uint16_t)(micros() - AileStart));
    
    if (Aile < 1125) {//avoid unexpected underflow behavior & bottom cap
      Aile = 0;
    }
    else {
      Aile = Aile - 1125;
    }

    if(Aile > deltaAile + 100) { //failsafe
      Aile = 0;
      failure = failure + 1;
    }
    else if (Aile > deltaAile) { //top cap
      Aile = 1;
    }
    else {
      Aile = Aile/deltaAile; 
    }
  }
}

void calcRudd() {
  if(digitalRead(18) == HIGH) {
    RuddStart = micros();
  }
  else {
    Rudd = float((uint16_t)(micros() - RuddStart));
    
    if (Rudd < 1096) {//avoid unexpected underflow behavior & bottom cap
      Rudd = 0;
    }
    else {
      Rudd = Rudd - 1096;
    }

    if(Rudd > deltaRudd + 100) { //failsafe
      Rudd = 0;
      failure = failure + 1;
    }
    else if (Rudd > deltaRudd) { //top cap
      Rudd = 1;
    }
    else {
      Rudd = Rudd/deltaRudd; 
    }
  }
}

void calibrateESCs() {
  Serial.println("calibration iniciated");
  pinMode(52, OUTPUT);
  digitalWrite(52, LOW);
  pinMode(53, INPUT_PULLUP);
  int maxPower = 135;
  
  esc1.write(maxPower);
  esc2.write(maxPower);
  esc3.write(maxPower);
  esc4.write(maxPower);
  
  batteryWait();
  Serial.println("motors powered");
  
  //this section has to be cca 36673 millis long
  delay(1500);
  while(maxPower > 50) {
    maxPower = maxPower - 5;
    esc1.write(maxPower);
    esc3.write(maxPower);
    esc4.write(maxPower);
    delay(100);
  }
  Serial.println("esc1, esc3, esc4 calibrated");
  delay(33459);
  //end of the section

  esc2.write(50);//exit programming mode
  Serial.println("esc2 calibrated");
  delay(12000);
  Serial.println("calibration successful");
}

void batteryWait() {   //wait until main powersource is connected
  while(analogRead(A0)<800) {
    delay(10);
  }
}

void calcGyro() { //get gyro data, 63rd measurement is used for calibration
  if (mpu.update()) {
    //printRC();
    if (gyroCalibrated == 64) {      //normal mode <64>
      gyro.x = (mpu.getEulerX()-gyroOffset.x) * 0.0174533; //convert to rad
      gyro.y = (mpu.getEulerY()-gyroOffset.y) * 0.0174533;
      gyro.z = mpu.getEulerZ() * 0.0174533;
      
      //calculate thrust multiplier to keep it konstant
      if (abs(gyro.x) > 0.5 || abs(gyro.y) > 0.5) {
        gyro.x = 0;
        gyro.y = 0;
        angBias = 0; //failsafe -> angle limit 0.5rad (28.6deg)
        failure = failure + 1;;
      }
      else {
        angBias = 1/cos(gyro.x)*1/cos(gyro.y);
      }              
    }
    else if (gyroCalibrated < 63) {    //before calibration <0;62>
      gyroCalibrated = gyroCalibrated + 1;
    }
    else {    //calibration <63>
      gyroOffset.x = mpu.getEulerX();
      gyroOffset.y = mpu.getEulerY();
      gyroCalibrated = 64;
      digitalWrite(LED_BUILTIN, LOW);
      //Serial.println("gyro offsets calculated");
    }
  }
}

void printRC() {
  Serial.print(Thro);
  Serial.print(" ");
  Serial.print(Elev);
  Serial.print(" ");
  Serial.print(Aile);
  Serial.print(" ");
  Serial.println(Rudd);
}
void printESC(struct ESCs escs) {
  Serial.print(escs.e1);
  Serial.print(" ");
  Serial.print(escs.e2);
  Serial.print(" ");
  Serial.print(escs.e3);
  Serial.print(" ");
  Serial.println(escs.e4);
}
void printGyro() {
  Serial.print(gyro.x);
  Serial.print(" ");
  Serial.println(gyro.y);
  //Serial.print(" ");
  //Serial.println(gyro.z);  
}
