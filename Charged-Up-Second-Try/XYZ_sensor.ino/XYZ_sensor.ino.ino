#include "FastIMU.h"
#include "SPI.h"

#define IMU_ADDRESS 0x69    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
#define WINDOW_SIZE 100

int INDEX = 0;
float VALUE = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;

BMI160 IMU;               //Change to the name of any supported IMU! 

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  int err = IMU.init(calib, IMU_ADDRESS);
  while (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    err = IMU.init(calib, IMU_ADDRESS);
  }
  
#ifdef PERFORM_CALIBRATION
  //Serial.println("Keep IMU level.");
  delay(1000);
  IMU.calibrateAccelGyro(&calib);
  IMU.init(calib, IMU_ADDRESS);
#endif
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);
  
  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = accelData.accelX*100;        // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result
  Serial.println(map(AVERAGED,-100,100,-90,90));
  delay(10);
}
