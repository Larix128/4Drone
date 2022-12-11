#include <Servo.h>
#include "MPU9250.h"
#include "4Drone.h"

MPU9250 mpu;
//escs calibrated to servowrite(50-135)
//need to correct/calibrate rc readings to be exact(0=lowest, 0.5=middle, 1=highest)
//need to add failsafe to RC readings
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
volatile int ElevStart;
volatile int AileStart;
volatile int RuddStart;

struct RCInputs rc;


struct RCThreshold thro;
struct RCThreshold elev;
struct RCThreshold aile;
struct RCThreshold rudd;



int currentTime;















//=================================================================================================================================================================
//  SETUP
//=================================================================================================================================================================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("script iniciated");
  
  esc1.attach(esc1PIN);
  esc2.attach(esc2PIN);
  esc3.attach(esc3PIN);
  esc4.attach(esc4PIN);


  //calibrateESCs();
  
  digitalWrite(LED_BUILTIN, HIGH);
  mpu.setup(0x68);  //gyro: change to my own address
  delay(2000); 


  //RC read 
  attachInterrupt(0, calcThro, CHANGE);
  attachInterrupt(1, calcElev, CHANGE);
  attachInterrupt(4, calcAile, CHANGE);
  attachInterrupt(5, calcRudd, CHANGE); 
  calibrateRC();
}


//=================================================================================================================================================================
//  LOOP
//=================================================================================================================================================================
void loop() {
  if (failure >= failsafeTreshold) {
    failsafe(2);
  }
  Serial.println(micros());
  calcGyro();
  
  struct ESCs escs = {50, 50, 50, 50};
  if (rc.Thro != 0) {  //if throttle is 0 do not calculate (send 50 instead)
    
    //===================================================================================
    // put flight calculation here; write results to escs.e1, escs.e2, escs.e3, escs.e4
    //===================================================================================
  
  }
 

  esc1.write(escs.e1);
  esc2.write(escs.e2);
  esc3.write(escs.e3);
  esc4.write(escs.e4);
}















//=================================================================================================================================================================
//  RC INTERRUPTS
//=================================================================================================================================================================
void calcThro() {
    if(digitalRead(2) == HIGH) {
    ThroStart = micros();
  }
  else {
    rc.Thro = (uint16_t)(micros() - ThroStart);
    //Serial.println(rc.Thro);
  }
}

void calcElev() {
  if(digitalRead(3) == HIGH) {
    ElevStart = micros();
  }
  else {
    rc.Elev = (uint16_t)(micros() - ElevStart);
  }
}

void calcAile() {
  if(digitalRead(19) == HIGH) {
    AileStart = micros();
  }
  else {
    rc.Aile = (uint16_t)(micros() - AileStart);
  }
}

void calcRudd() {
  if(digitalRead(18) == HIGH) {
    RuddStart = micros();
  }
  else {
    rc.Rudd = (uint16_t)(micros() - RuddStart);
  }
}


//============================================================
//  CALIBRATION
//============================================================
void calibrateESCs() {
  Serial.println("calibration iniciated");
  int maxPower = 135;
  

  if (analogRead(A0)>800) {  //failsafe
    failsafe(1);
  }
  
  esc1.write(maxPower);
  esc2.write(maxPower);
  esc3.write(maxPower);
  esc4.write(maxPower);

  Serial.println("please connect powersource");
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
  esc2.write(50);//exit programming mode
  Serial.println("esc2 calibrated");
  //end of the timed section 

  delay(12000);
  Serial.println("calibration successful");
}






void batteryWait() {   //wait until main powersource is connected
  while(analogRead(A0)<800) {
    delay(10);
  }
}

//============================================================
// CALIBRATE RC
//============================================================
void calibrateRC() {
  thro.minmin = 10000;
  thro.minmax = -10000;
  Serial.println(thro.minmax);
  while (true){
    Serial.println(thro.minmax);
    calibrateRCx (rc.Thro, &thro.minmin, &thro.minmax);

  } 
  delay(1000); 
  calibrateRCx (rc.Thro, &thro.medmin, &thro.medmax);
  delay(1000);
  calibrateRCx (rc.Thro, &thro.maxmin, &thro.maxmax);
}

void calibrateRCx (int rcValue, int *min, int *max){
  delay(1000);
  Serial.println(rcValue);
  Serial.print(*min);
  Serial.print(" ");
  Serial.println(*max);
  if (rcValue < *min) {
    min = rcValue;
    Serial.println("A");
  }
  else if (rcValue > *max) {
    max = rcValue;
        Serial.println("B");

  }
    
    delay(10);
}

//============================================================
// CALCULATE GYRO
//============================================================
void calcGyro() { //get gyro data, 63rd measurement used for calibration
  if (mpu.update()) {
    if (gyroCalibrated == 64) {      //normal mode <64>
      gyro.x = (mpu.getEulerX()-gyroOffset.x) * 0.0174533; //convert to rad
      gyro.y = (mpu.getEulerY()-gyroOffset.y) * 0.0174533;
      gyro.z = mpu.getEulerZ() * 0.0174533;
      
      if (abs(gyro.x) > 0.5 || abs(gyro.y) > 0.5) {  //failsafe - angle limit 0.5rad (28.6deg)    
        failure = failure + 1;
        if (failure >= failsafeTreshold) {
          failsafe(3);
        }
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
      Serial.println("gyro offsets calculated");
    }
  }
}



//============================================================
//  FAILSAFE
//============================================================
void failsafe(int cause) {
  while (true) {
    esc1.write(0);
    esc2.write(0);
    esc3.write(0);
    esc4.write(0);
    Serial.print("failsafe triggered, ERROR ");
    Serial.println(cause);
    delay(10);
  }
}

//============================================================
//  DEBUG
//============================================================
void printRC() {
  Serial.print(rc.Thro);
  Serial.print(" ");
  Serial.print(rc.Elev);
  Serial.print(" ");
  Serial.print(rc.Aile);
  Serial.print(" ");
  Serial.println(rc.Rudd);
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
