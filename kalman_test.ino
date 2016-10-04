#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <L3G.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

L3G gyro;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Kalman kalmanX;
Kalman kalmanY;

#define gyroAddress 0x68
#define adxlAddress 0x53

double zeroValue[5] = { -200, 44, 660, 52.3, -18.5}; // Found by experimenting

/* All the angles start at 180 degrees */
double gyroXangle = 180;
double gyroYangle = 180;

double compAngleX = 180;
double compAngleY = 180;

unsigned long timer;

uint8_t buffer[2]; // I2C buffer

void setup() {
  //Serial.begin(115200);
  Serial.begin(9600);
  
  // Initialise the ADXL345
  if(!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  //Set the range
  accel.setRange(ADXL345_RANGE_16_G);

  // Initialise the L3G4200D
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();
  kalmanX.setAngle(180); // Set starting angle
  kalmanY.setAngle(180);
  timer = micros();
}

void loop() {

  //Get a new ADXL345 event to pull data
  sensors_event_t event;
  accel.getEvent(&event);
  //Read the gyro data
  gyro.read();
  
  double gyroXrate = -(((double)gyro.g.x - zeroValue[3]) / 14.375);
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Without any filter

  double gyroYrate = (((double)gyro.g.y - zeroValue[4]) / 14.375);
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000); // Without any filter

  double accXangle = (atan2(event.acceleration.x,(sqrt(pow(event.acceleration.y,2)+pow(event.acceleration.z,2)))) * 180.0) / PI;
  double accYangle = (atan2(event.acceleration.y,(sqrt(pow(event.acceleration.x,2)+pow(event.acceleration.z,2)))) * 180.0) / PI;

  compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle);
  compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

  double xAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer)); // calculate the angle using a Kalman filter
  double yAngle = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer)); // calculate the angle using a Kalman filter

  timer = micros();

  /* print data to processing */
  Serial.print("Gyro Angles: ");
  Serial.print(gyroXangle); Serial.print(",");
  Serial.print(gyroYangle); Serial.print("\t");
  
  Serial.print("Acc Angles: ");
  Serial.print(accXangle); Serial.print(",");
  Serial.print(accYangle); Serial.print("\t");Serial.print("\t");

  Serial.print("Comp Angles: ");
  Serial.print(compAngleX); Serial.print(",");
  Serial.print(compAngleY); Serial.print("\t");Serial.print("\t");

  Serial.print("X and Y Angles: ");
  Serial.print(xAngle); Serial.print(",");
  Serial.print(yAngle); Serial.print("\t");

  Serial.print("\n");

  delay(10);
}
