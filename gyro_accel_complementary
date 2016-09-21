#include <Wire.h>
#include <TimerOne.h>
 
#include "ADXL345.h"
#include "L3G4200D.h"
//Gyro sampling rate in micro seconds
#define SampleRate 10000
ADXL345 accel;
L3G4200D gyro;
bool readGyro;
double angleF_roll;
double angleF_pitch;
unsigned long pT; //previous time
void setup()
{
   
  Serial.begin(115200);
   
  Serial.println("Ready.");
  Wire.begin();
   
  accel.init(-1, 0, 8);
  gyro.init(-0.18891-0.04382, -0.09454-0.05530, -0.31720-0.04951); 
  accel.setSoftwareOffset(-0.023, 0, 0.03577027);
  //accel.printCalibrationValues(40);
  //gyro.printCalibrationValues(40);
 
   
  pT = 0;
   readGyro = false;
   angleF_roll = angleF_pitch = 0;
    
  Timer1.initialize(SampleRate);
  Timer1.attachInterrupt( timerIsr );
}
 
void timerIsr()
{
  readGyro = true;
}
 
void loop()
{
   
  //ACCEL
  AccelRotation accelRot;
   
  accelRot = accel.readPitchRoll();
  Serial.print("{P0|Pitch|127,255,0|");
  Serial.print(accelRot.pitch);
   
   
  Serial.print("|Roll|255,255,0|");
  Serial.print(accelRot.roll);
  Serial.println("}");
   
 
  AccelG accelG;
  accelG = accel.readAccelG();
  Serial.print("{P1|Xg|255,0,0|");
  Serial.print(accelG.x);
   
   
  Serial.print("|Yg|0,255,0|");
  Serial.print(accelG.y);
   
  Serial.print("|Zg|0,0,255|");
  Serial.print(accelG.z);
  Serial.println("}");
  //END ACCEL
 
//GYRO
  if(readGyro)
  {
    readGyro = false;
    unsigned long cT = micros();
     
    GyroDPS gDPS;
    gDPS = gyro.readGyroDPS();
     
    unsigned long dT = cT - pT;
    pT = cT;
     
    Serial.print("{P2|X|255,0,0|");
    Serial.print(gDPS.x);
     
     
    Serial.print("|Y|0,255,0|");
    Serial.print(gDPS.y);
     
    Serial.print("|Z|0,0,255|");
    Serial.print(gDPS.z);
    Serial.println("}");
    
    
     // Complementary Filter
     
    Serial.print("{P4|Roll|0,255,0|");
    angleF_roll = 0.95*(angleF_roll + gDPS.x*(dT/1000000.0)) +0.05*accelRot.roll; 
    Serial.print(angleF_roll);
    angleF_pitch = 0.95*(angleF_pitch - gDPS.y*(dT/1000000.0)) +0.05*accelRot.pitch;
    Serial.print("|Pitch|255,0,0|");
    Serial.print(angleF_pitch);
    Serial.println("}");  
 
  }
  //END GYRO
}
