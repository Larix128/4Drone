struct ESCs {
  float e1;
  float e2;
  float e3;
  float e4;
};

struct Vector3 {
  float x;
  float y;
  float z;
};

struct RCThreshold {
  int minmin;
  int minmax;
  int medmin;
  int medmax;
  int maxmin;
  int maxmax;
};


//print
void printGyro(); //print(gyro.x gyro.y gyro.z)
void printESC(); // print(escs.e1 escs.e2 escs.e3 escs.e4)
void printRC(); // print(Thro Elev Aile Rudd)


//RC
void calcThro(); //interrupt, calculate input from RC
void calcElev(); //interrupt, calculate input from RC
void calcAile(); //interrupt, calculate input from RC
void calcRudd(); //interrupt, calculate input from RC

//calibration & setup
void calibrateRC();
void calibrateRCmin ();
void calibrateESCs(); // calibrate ESC to <50; 135> value range
void batteryWait(); //wait until main powersource is connected

void failsafe(int cause); //failsafe - set motors to 0, trigger infinite loop, print error code
//1   esc calibration: powersource connected too soon
//2   loop: unknown error
//3   gyro read: angle limit exceeded

//void flightCTRL();
