// Name: L3G4200D_Gyro_Init_and_Test
// Written by: William Martin
// Developed for: The University of Tennessee Graduate Robotics Team
// Created: 3/1/2013
// Last Modified: 3/3/2013
// Contact: wmartin8@utk.edu
// License: open-source, but I do appreciate the above lines being included in your code
// Based on: http://bildr.org/2011/06/l3g4200d-arduino/ and the MIT License

/**** GENERAL NOTES ****
- this code is only converned with the yaw axis (z) of the gyro. To work with
- the roll (x) and pitch (y) axes, you can copy this code with a slight change of variable names. 

- boolean variable 'calibrate' should only need to set to 'true' if you are getting 
- unexpected behavior from code. This may happen if your use requires the gyro to be rotated
- either extremely quickly or extremly slowly. 
******* END NOTES ********/

#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

// This character buffer holds all of the Serial.print statements.
char p_buffer[115];
// By wrapping all of our Serial.print statements in P, the character strings are stored in the
// PROGMEM (flash) memory (32k) instead of the EPROM (2k). This fixes a weird bug that was causing my
// code to crash  during runtime because I had too many character strings embedded in the print statements
// that was causing the SRAM to overflow
#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)

// These are basically define statements used in the funtion calls below
const boolean CLOCKWISE = true;
const boolean COUNTERCLOCKWISE = false;

int L3G4200D_Address = 105; //I2C address of the L3G4200D

// These hold the instantaneous values from the gryo for pitch, roll, and yaw
int rollGyroVal;
int pitchGyroVal;
int yawGyroVal;

const int gyroNoiseThresh = 25; // Ignore gyroscope values below this value
const int gyroDelayTime = 20; // refresh rate of gyroscope. 
double yawGyroValDouble; // running sums get very large. We need to poll the int from the register and convert it to a double
double yawGyroValRunSum = 0; // when turning, we poll the gyro thousands of times and accumulate the values help in yawGyroVal

double clicksPerDegCW = -663.33; // a constant used when turning X degrees clockwise. This value can be re-set by turning the calibrate variable to TRUE and following on-screen instructions
double clicksPerDegCCW = 647.67; // a constant used when turning X degrees counter clockwise. This value can be re-set by turning the calibrate variable to TRUE and following on-screen instructions

// Boolean program controllers for the gyroscope
boolean calibrate = false; // set to true if you want to recalibrate the gyro (doesn't seem like gyro is detecting accurate number of degrees turned
boolean testMode = true; // set to true if you want to test the gyro to see if it needs to be recalibrated

boolean gyroTurnTimeoutError = false; // keeps track of if the gyro has timed out on a turn
int turnTimeout = 8000; // maximum time required for a turn (ms). Error above is thrown if turn takes longer than this

void setup(){

  Wire.begin();
  Serial.begin(9600);

  Serial.println(P("\n\n\nStarting up L3G4200D"));
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec

  delay(1500); //wait for the sensor to be ready 
  
  if(calibrate) {

    // In this block, we perform calibration to reset the values stored in clicksPerDegCW and clicksPerDegCCW.
    // IMPORTANT NOTE - after calibration, you will need to copy and paste the values displayed on the serial moniter into the 
    // variable declarations above!!!! 
    
    recalibrateGyroscope();
    
    
  }
  
  if (testMode) {
    
    // This function tests to see that the current values in clicksPerDegCW and clicksPerDegCCW
    // are currently working or need to be reset
    testCalibrationResults();
    
  }
    
  

}

void loop(){
  // The main loop is left empty so that you can incorporate the code developed here into
  // your project. Here is an example of how you may like to use this code in some robotics application...
  
  /*
  boolean turnDirection;
  double turnDegrees;
  
  turnDirection = getDirectionToTurn(); // some function you wrote
  turnDegrees = getDegreesToTurn(); // some function you wrote
  
  startTurning(turnDirection); // make your robot start turning (another function you wrote)
  gyroTurnTimeoutError = gyroBasedTurns(turnDirection, turnDegrees); // function returns once gyro has been rotated far enough
  stopTurning(); // tell motors to stop turning based on gyro data
  
  if (gyroTurnTimeoutError) { // gryo not working? startTurning function not working? 
  }
  else { // success!
  }
  */
  
  
}







/******** BELOW THIS LINE IS ALL FUNCTIONS FOR THE L3G4200D GYRO UNIT *********/ 
boolean gyroBasedTurns(boolean dir, double angleDegrees) {
 // dir = 1 -> clockwise, dir = 0 -> counterClockWise 
 
 // This function continuously moniters the gyroscope's yaw reading until
 // it has turned angleDegrees in dir direction. It returns true if the turn is successful
 // and false if the turn event has timed out.
 
 double clicksToTurn = 0;
 if (dir == CLOCKWISE) {
   clicksToTurn = angleDegrees * clicksPerDegCW;
 }
 
 else if (dir == COUNTERCLOCKWISE) {
   clicksToTurn = angleDegrees * clicksPerDegCCW;
 }
 
 double yawGyroValDouble = 0;
 double totalClicks = 0;
 
 int starttime = millis(); // get start time
 int endtime = starttime; // init end time
 
 while (abs(totalClicks) < abs(clicksToTurn)) {
  getGyroValues();  // This will update rollGyroVal, pitchGyroVal, and yawGyroVal with new values

  yawGyroValDouble =yawGyroVal;
  if(abs(yawGyroValDouble) > abs(gyroNoiseThresh)){ // ignore noise
      totalClicks+=yawGyroValDouble; // update runsum
    }
  
  delay (gyroDelayTime);
  endtime = millis();
  if ((endtime - starttime) > turnTimeout) {
    return true; // turn timed out. Return an error
  }

 }
 
 return false; // we've accumulated anough clicks to be able to say that the turn has completed. Return
}


void testCalibrationResults() {
  
    // This function runs a few simple tests to determine if the gyro is correctly calibrated
  
    Serial.println(P("Now to test. After you type a key, the program will wait for "));
    Serial.println(P("you to rotate the gyro 90 degrees clockwise. "));
    Serial.println(P("It will inform you once you have done so."));
    
    while(Serial.available() == 0){} // wait for user to enter a button
    Serial.read(); Serial.println(P("Go!"));

    gyroTurnTimeoutError = gyroBasedTurns(CLOCKWISE, 90.0);
    if (gyroTurnTimeoutError) {
      Serial.println(P("Error - turn timed out."));
    } else {
    Serial.println(P("Turn Complete.\n"));}
    
    Serial.println(P("Now type a key, then rotate the gyro 90 degrees counter-clockwise..."));
    
    while(Serial.available() == 0){}
    Serial.read(); Serial.println(P("Go!"));
    gyroTurnTimeoutError = gyroBasedTurns(COUNTERCLOCKWISE, 90.0);
    if (gyroTurnTimeoutError) {
      Serial.println(P("Error - turn timed out."));
    } else {
    Serial.println(P("Turn Complete.\n"));}
    
    Serial.println(P("Now type a key, then rotate the gyro 180 degrees clockwise..."));
    
    while(Serial.available() == 0){}
    Serial.read(); Serial.println(P("Go!"));
    gyroTurnTimeoutError = gyroBasedTurns(CLOCKWISE, 180.0);
    if (gyroTurnTimeoutError) {
      Serial.println(P("Error - turn timed out."));
    } else {
    Serial.println(P("Turn Complete.\n"));}
    
    Serial.println(P("Now type a key, then rotate the gyro 180 degrees counter-clockwise..."));
    
    while(Serial.available() == 0){}
    Serial.read(); Serial.println(P("Go!"));
    gyroTurnTimeoutError = gyroBasedTurns(COUNTERCLOCKWISE, 180.0);
    if (gyroTurnTimeoutError) {
      Serial.println(P("Error - turn timed out."));
    } else {
    Serial.println(P("Turn Complete.\n"));}
    
    Serial.println(P("Testing complete. If these tests were satisfactory, "));
    Serial.println(P("change 'boolean testMode' valraible initialization to 'FALSE'"));
    Serial.println(P("And begin using your gyroscope!"));
}


void recalibrateGyroscope() {
  
  // This function determines the 'clicks per degree' variable values. Follow onscreen instructions. 
  
  double CWaverages[5] = {0}; // init arrays to hold test values
  double CCWaverages[5] = {0};
  int NUMTESTS = 3; // increase this number to eliminate noise in calibration tests. Decrease to be less annoyed with calibration
  
  Serial.println(P("Welcome to the gyroscope calibration routine! Let's get started...\n"));
  Serial.println(P("You are going to be rotating the gryoscope back and forth a few times to set the "));
  Serial.println(P(" clicksPerDegCW and clicksPerDegCCW variable values. Follow the on-screen instructions.\n\n")); 
  
  for(int i=0; i<NUMTESTS; i++) {
  CWaverages[i] = setCountPerDegCW(clicksPerDegCW, gyroDelayTime); 
  CCWaverages[i] = setCountPerDegCCW(clicksPerDegCCW, gyroDelayTime);
  Serial.print(P("(Test set "));
  Serial.print(i+1);
  Serial.print(P(" of "));
  Serial.print(NUMTESTS);
  Serial.println(P(" completed.)"));
  }
  
  Serial.println(P("\n\nTesting complete! Here is the data collected: \n (clicksPerDegCW,\tclicksPerDegCW)"));
  
  clicksPerDegCW = 0; // reset these values
  clicksPerDegCCW = 0;
  for(int i=0; i<NUMTESTS; i++) {
    Serial.print(CWaverages[i]);
    Serial.print(",\t");
    Serial.println(CCWaverages[i]);
    clicksPerDegCW+=CWaverages[i];
    clicksPerDegCCW+=CCWaverages[i];
  }
  clicksPerDegCW = clicksPerDegCW/(double)NUMTESTS;
  clicksPerDegCCW = clicksPerDegCCW/(double)NUMTESTS;
  
  Serial.println(P("(Numbers in the two columns above should be very similar. If numbers in the same "));
  Serial.println(P("column have different signs, then you have confused clockwise with counterclockwise.)"));
  Serial.println(P("You will now need to change some values\n in the initializaion at the top of the code."));
  Serial.print(P("\nChange the line 'double clicksPerDegCW = XXXX;' to 'double clicksPerDegCW = "));
  Serial.print(clicksPerDegCW);
  Serial.println("; ");
  Serial.print(P("Change the line 'double clicksPerDegCCW = XXXX;' to 'double clicksPerDegCCW = "));
  Serial.print(clicksPerDegCCW);
  Serial.println(P("; "));
  Serial.println(P("\nNow, change the boolean variable calibrate to 'false' \nand re-upload the code to the Arduino."));
  Serial.print(P("You should also set the boolean variable testMode to 'true' \nto verify "));
  Serial.println(P("that the re-calibration is giving expected results."));
  
}


int setCountPerDegCW(int currentVal, int gyroDelayTime) {
  
  Serial.println(P("After hitting any key, you will have 5 seconds"));
  Serial.println(P(" to rotate the gyro 180 degrees clockwise."));
  while(Serial.available() == 0){} // wiat for user to type a key
  Serial.read(); // throw away result
  double yawGyroValDouble=0;
  double yawGyroValRunSum=0;
  int starttime = millis();
  Serial.println(P("Go!"));
  int endtime = starttime;
  while ((endtime - starttime) <=5000) // do this loop for up to 5000mS
  {
    getGyroValues();  // This will update rollGyroVal, pitchGyroVal, and yawGyroVal with new values
    yawGyroValDouble =yawGyroVal; // convert to double
    if(abs(yawGyroValDouble) > abs(gyroNoiseThresh)){ // ignore noise
      yawGyroValRunSum += yawGyroValDouble; // add to running sum
    }
    delay (gyroDelayTime); // let gyro refresh
    endtime = millis(); // get elapsed time of test
  }
  
  Serial.println(P("Time's up!"));
  Serial.print(P("Click count found was: "));
  Serial.println(yawGyroValRunSum);
  Serial.print(P("The variable 'clicksPerDegCW in this test is: "));
  Serial.println(yawGyroValRunSum/180);
  
  return yawGyroValRunSum/180;
   
}

int setCountPerDegCCW(int currentVal, int gyroDelayTime) {
  
  Serial.println(P("After hitting any key, you will have 5 seconds"));
  Serial.println(P(" to rotate the gyro 180 degrees counter clockwise."));
  while(Serial.available() == 0){}
  Serial.read();
  double yawGyroValDouble=0;
  double yawGyroValRunSum=0;
  int starttime = millis();
  Serial.println("Go!");
  int endtime = starttime;
  while ((endtime - starttime) <=5000) // do this loop for up to 5000mS
  {
    getGyroValues();  // This will update rollGyroVal, pitchGyroVal, and yawGyroVal with new values
    yawGyroValDouble =yawGyroVal; // convert to double
    if(abs(yawGyroValDouble) > abs(gyroNoiseThresh)){ // ignore noise
      yawGyroValRunSum += yawGyroValDouble; // update running sum
    }
    delay (gyroDelayTime); // allow gyro to refresh
    endtime = millis(); // get elapsed time
  }
  
  Serial.println(P("Time's up!"));
  Serial.print(P("Click count found was: "));
  Serial.println(yawGyroValRunSum);
  Serial.print(P("The variable 'clicksPerDegCCW in this test is: "));
  Serial.println(yawGyroValRunSum/180);
  
  return yawGyroValRunSum/180;
   
}

void getGyroValues(){

  // Get instantaneous roll, pitch and yaw values from gyro
  byte rollGyroValMSB = readRegister(L3G4200D_Address, 0x29);
  byte rollGyroValLSB = readRegister(L3G4200D_Address, 0x28);
  rollGyroVal = ((rollGyroValMSB << 8) | rollGyroValLSB);

  byte pitchGyroValMSB = readRegister(L3G4200D_Address, 0x2B);
  byte pitchGyroValLSB = readRegister(L3G4200D_Address, 0x2A);
  pitchGyroVal = ((pitchGyroValMSB << 8) | pitchGyroValLSB);

  byte yawGyroValMSB = readRegister(L3G4200D_Address, 0x2D);
  byte yawGyroValLSB = readRegister(L3G4200D_Address, 0x2C);
  yawGyroVal = ((yawGyroValMSB << 8) | yawGyroValLSB);
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable rollGyroVal, pitchGyroVal, yawGyroVal and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}
