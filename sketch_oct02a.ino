/*
    Kalman Filter Example for ADXL345 & L3G4200D. Output for processing.
    Read more: http://www.jarzebski.pl/arduino/rozwiazania-i-algorytmy/odczyty-pitch-roll-oraz-filtr-kalmana.html
    GIT: https://github.com/jarzebski/Arduino-KalmanFilter
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <L3G.h>
#include <KalmanFilter.h>

L3G gyro;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

void setup() 
{
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
}

void loop()
{
  //Get a new ADXL345 event to pull data
  sensors_event_t event;
  accel.getEvent(&event);
  //Read the gyro data
  gyro.read();
  
  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = (atan2(event.acceleration.x,(sqrt(pow(event.acceleration.y,2)+pow(event.acceleration.z,2)))) * 180.0) / PI;
  accRoll  = (atan2(event.acceleration.y,(sqrt(pow(event.acceleration.x,2)+pow(event.acceleration.z,2)))) * 180.0) / PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyro.g.y);
  kalRoll = kalmanX.update(accRoll, gyro.g.x);
  
  Serial.print("ACC PITCH: ");
  Serial.print(accPitch);
  Serial.print("   ACC ROLL:  ");
  Serial.print(accRoll);
  Serial.print("   KAL PITCH:  ");
  Serial.print(kalPitch);
  Serial.print("   KAL ROLL:   ");
  Serial.println(kalRoll);
  /*Serial.print(":");
  Serial.print(event.acceleration.x);
  Serial.print(":");
  Serial.print(event.acceleration.y);
  Serial.print(":");
  Serial.print(event.acceleration.z);
  Serial.print(":");
  Serial.print(gyro.g.x);
  Serial.print(":");
  Serial.print(gyro.g.y);
  Serial.print(":");
  Serial.print(gyro.g.z);*/

  //Serial.println();
}


