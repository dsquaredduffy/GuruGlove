#include <Wire.h>
#include <ADXL345.h>
#include <HMC5883L.h>
#include <MovingAverageFilter.h>
#define  STOPPER 0              /* Smaller than any datum */
#define  MEDIAN_FILTER_SIZE (17)

HMC5883L compass;     //Create magnetomere objects
ADXL345 w_acc, s_acc; //Create accelerometer objects
MovingAverageFilter movingAverageFilter(10);

//Demiliters used for serial communication between microcontrollers
const char startOfDegreeDelimiter    = '<';
const char endOfDegreeDelimiter      = '>';

const char startOfNumberDelimiter    = '{';
const char endOfNumberDelimiter      = '}';

const char startOfCharacterDelimiter = '[';
const char endOfCharacterDelimiter   = ']';

// Pin values for flex sensor inputs
const int finger_flexion_pin = 0;   //PIN 40
const int downward_wrist_pin = 1;   //PIN 39
const int upward_wrist_pin   = 2;   //PIN 38
const int elbow_flexion_pin  = 3;   //PIN 37

const int dataPointsCount = 10;
//Pointers used to hold min and max data for sensors / only written to once
//Because these are static and global they are initliazed to zero
static int *wrist_data, *shoulder_data, *heading_data;
static int *finger_data, *elbow_data, *upward_wrist_data, *downward_wrist_data;
static int previous_heading = 200;

static int finger_average[dataPointsCount], elbow_average[dataPointsCount], wrist_average[dataPointsCount];
static int f,w,e;

//////////////////////////////
// PITCH AND ROLL STRUCTURE //
//////////////////////////////

//This structure is used because pitch and roll are constantly written to
//This avoids using pointers to return pitch and roll values which would require
//allocating and deallocating new memory
struct Angles {int pitch, roll;};

//Calculates roll and pitch
struct Angles getAngles(Vector norm) {
    int pitch = -(atan2(norm.XAxis, sqrt(norm.YAxis*norm.YAxis + norm.ZAxis*norm.ZAxis))*180.0)/M_PI;
    int roll  = (atan2(norm.YAxis, norm.ZAxis)*180.0)/M_PI;
    Angles angles = { pitch, roll };
    return angles;    
}

///////////////////////////////////////
// SETUP AND INITALIZATION FUNCTIONS //
///////////////////////////////////////

void setup() {
  Serial.begin(115200);
  w_acc.setAddress(0x1D);
  s_acc.setAddress(0x53);

 /// Initialise the ADXL345
  if(!w_acc.begin())
  {
    Serial.print (startOfCharacterDelimiter);    
    Serial.print ("Ooops, Wrist ADXL345 not detected ... Check your wiring!");
    Serial.print (endOfCharacterDelimiter);   
    Serial.println (); 
    delay(500);
  }
  //Set the range
  w_acc.setRange(ADXL345_RANGE_16G);

 /// Initialise the ADXL345
  if(!s_acc.begin())
  {
    Serial.print (startOfCharacterDelimiter);    
    Serial.print ("Ooops, Shoulder ADXL345 not detected ... Check your wiring!");
    Serial.print (endOfCharacterDelimiter);   
    Serial.println (); 
    delay(500);
  }
  //Set the range
  s_acc.setRange(ADXL345_RANGE_16G);  

  // Initialize HMC5883L
  while (!compass.begin())
  {
    Serial.print (startOfCharacterDelimiter);    
    Serial.println("Failed to begin compass sensor!");
    Serial.print (endOfCharacterDelimiter);   
    Serial.println (); 
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  Calibrate_Sensors();
}

//Calls the calibration function for all sensors 
//Waits three seconds after informing user which sensor will be calibrated
void Calibrate_Sensors()
{
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("About to begin calibrating finger");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(3000);
  finger_data = Calibrate_Flex(finger_flexion_pin);
  PrintMinMaxFlex(finger_data);

  Serial.print (startOfCharacterDelimiter);        
  Serial.print ("About to begin calibrating upward wrist motion");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(3000);
  upward_wrist_data = Calibrate_Flex(upward_wrist_pin);
  PrintMinMaxFlex(upward_wrist_data);

  Serial.print (startOfCharacterDelimiter);        
  Serial.print ("About to begin calibrating downward wrist motion");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(3000);
  downward_wrist_data = Calibrate_Flex(downward_wrist_pin);
  PrintMinMaxFlex(downward_wrist_data);

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibrating wrist, rotate wrist from side to side");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(3000);
  wrist_data = Calibrate_Acc(w_acc);
  PrintMinMaxAngles(wrist_data);

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("About to begin calibrating elbow, flex up and down");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(3000);
  elbow_data = Calibrate_Flex(elbow_flexion_pin);
  PrintMinMaxFlex(elbow_data);
  
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibrating shoulder, raise arm up and down from your side");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(3000);
  shoulder_data = Calibrate_Acc(s_acc);
  PrintMinMaxAngles(shoulder_data);

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibrating shoulder, swing arm back and forth");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(3000);
  heading_data = Calibrate_Heading(s_acc);
  PrintMinMaxHeading(heading_data);  
}

//////////////////////////////////
// CALIBRATING SENSOR FUNCTIONS //
//////////////////////////////////

//Return min and max values of an accelerometer sensor
int *Calibrate_Acc(ADXL345 accelerometer){
  
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibration Begin");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  
  struct Angles a;
  static int acc_data[4];
  //Clear array so that the next sensor will not be affected by previous data
  memset(acc_data, 0, sizeof(acc_data));

  //Takes 800 samples in 8 seconds
  for(int i=0; i<800; i++)
  {
    Vector norm = accelerometer.readNormalize();    
    a = getAngles(norm);
  
    acc_data[0] = min(acc_data[0], a.pitch);
    acc_data[1] = max(acc_data[1], a.pitch);
    
    acc_data[2] = min(acc_data[2], a.roll);
    acc_data[3] = max(acc_data[3], a.roll);

    //Roll minimum kept coming out to 0 degrees because static variables are automatically zero
    //This if statement accounts for that because the minimum can be a negative value
    if(acc_data[3] == 0)
      acc_data[3] = a.roll;
    
    delay(10);
  }
  
  Serial.print (startOfCharacterDelimiter);    
  Serial.print ("Calibration Ended");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 

  return acc_data;
}

//Return min and max values of a flex sensor
int *Calibrate_Flex(int flexpin){

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibration Began");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  
  static int simple_flex_data[2];
  //Clear array so that the next sensor will not be affected by previous data
  memset(simple_flex_data, 0, sizeof(simple_flex_data));
  int flex_value;
  
  //Takes 800 samples in 8 seconds
  for(int i=0; i<800; i++)
  {

     flex_value = analogRead(flexpin);
     
    /*Serial.print (startOfCharacterDelimiter);      
    Serial.print ("Analog Value: ");
    Serial.print (endOfCharacterDelimiter);   
    Serial.println (); 
  
    Serial.print (startOfNumberDelimiter);      
    Serial.print (flex_value);
    Serial.print (endOfNumberDelimiter);   
    Serial.println (); */
     
     simple_flex_data[0] = min(simple_flex_data[0], flex_value);
     simple_flex_data[1] = max(simple_flex_data[1], flex_value);     
     //Due to all initial values being zero, we take in the minimum right above zero
     if(simple_flex_data[0] == 0)
        simple_flex_data[0] = flex_value;
     //No flex sensor should have a max of 1023 because it will never provide a voltage
     //of 5V
     if(simple_flex_data[1] == 1023 && flex_value != simple_flex_data[0])
        simple_flex_data[1] = flex_value;  
        
     delay(8);
  }
  
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibration Ended");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  
  return simple_flex_data;  
}

//Return min and max values of accelerometer sensor
int *Calibrate_Heading(ADXL345 accelerometer){

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibration Began");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  //No need to clear this array because it is only written to once
  static int data[2];
  float heading_value;
  
  for(int i=0; i<800; i++)
  {
    Vector norm = accelerometer.readScaled();    
    heading_value = Calculate_Accurate_Headings(norm);
    //Due to limitations of HMC5883L NAN values are possible so check for them
    if(!isnan(heading_value)) {
      data[0] = min(data[0], heading_value);
      data[1] = max(data[1], heading_value);      

    //Minimum heading should never be zero
    if(data[0] == 0)
      data[0] = heading_value; 
    }
    
    delay(10);
  }

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibration Ended");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  
  return data;
}

/////////////////////////
// FILTERING FUNCTIONS //
/////////////////////////

int MovingAverage(int input)
{ 
  int output = movingAverageFilter.process(input);
  return output;
}

float MovingAverageFilter(int data, char option)
{
  int out = 0;
  if(option == 'f'){
  finger_average[f] = data;
  f = (f+1) % dataPointsCount;

  for(int i=0; i<dataPointsCount; i++)
    out += finger_average[i];
  }

  if(option == 'e'){
  elbow_average[e] = data;
  e = (e+1) % dataPointsCount;

  for(int i=0; i<dataPointsCount; i++)
    out += elbow_average[i];
  }

  if(option == 'w'){
  wrist_average[w] = data;
  w = (w+1) % dataPointsCount;

  for(int i=0; i<dataPointsCount; i++)
    out += wrist_average[i];
  }
  return out/dataPointsCount;
}

//Applies median filter to the sensor value passed into it
uint16_t median_filter(uint16_t datum)
{
 struct pair
 {
   struct pair *point;                                /* Pointers forming list linked in sorted order */
   uint16_t value;                                    /* Values to sort */
 };
 static struct pair buffer[MEDIAN_FILTER_SIZE] = {0}; /* Buffer of nwidth pairs */
 static struct pair *datpoint = buffer;               /* Pointer into circular buffer of data */
 static struct pair small = {NULL, STOPPER};          /* Chain stopper */
 static struct pair big = {&small, 0};                /* Pointer to head (largest) of linked list.*/

 struct pair *successor;                              /* Pointer to successor of replaced data item */
 struct pair *scan;                                   /* Pointer used to scan down the sorted list */
 struct pair *scanold;                                /* Previous value of scan */
 struct pair *median;                                 /* Pointer to median */
 uint16_t i;

 if (datum == STOPPER)
 {
   datum = STOPPER + 1;                             /* No stoppers allowed. */
 }

 if ( (++datpoint - buffer) >= MEDIAN_FILTER_SIZE)
 {
   datpoint = buffer;                               /* Increment and wrap data in pointer.*/
 }

 datpoint->value = datum;                           /* Copy in new datum */
 successor = datpoint->point;                       /* Save pointer to old value's successor */
 median = &big;                                     /* Median initially to first in chain */
 scanold = NULL;                                    /* Scanold initially null. */
 scan = &big;                                       /* Points to pointer to first (largest) datum in chain */

 /* Handle chain-out of first item in chain as special case */
 if (scan->point == datpoint)
 {
   scan->point = successor;
 }
 scanold = scan;                                     /* Save this pointer and   */
 scan = scan->point ;                                /* step down chain */

 /* Loop through the chain, normal loop exit via break. */
 for (i = 0 ; i < MEDIAN_FILTER_SIZE; ++i)
 {
   /* Handle odd-numbered item in chain  */
   if (scan->point == datpoint)
   {
     scan->point = successor;                      /* Chain out the old datum.*/
   }

   if (scan->value < datum)                        /* If datum is larger than scanned value,*/
   {
     datpoint->point = scanold->point;             /* Chain it in here.  */
     scanold->point = datpoint;                    /* Mark it chained in. */
     datum = STOPPER;
   };

   /* Step median pointer down chain after doing odd-numbered element */
   median = median->point;                       /* Step median pointer.  */
   if (scan == &small)
   {
     break;                                      /* Break at end of chain  */
   }
   scanold = scan;                               /* Save this pointer and   */
   scan = scan->point;                           /* step down chain */

   /* Handle even-numbered item in chain.  */
   if (scan->point == datpoint)
   {
     scan->point = successor;
   }

   if (scan->value < datum)
   {
     datpoint->point = scanold->point;
     scanold->point = datpoint;
     datum = STOPPER;
   }

   if (scan == &small)
   {
     break;
   }

   scanold = scan;
   scan = scan->point;
 }
 return median->value;
}

/////////////////////////////////////////
// TILT COMPENSATION HEADING FUNCTIONS //
/////////////////////////////////////////

// Calculate Heading
float noTiltCompensate(Vector mag) {
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}
 
// Calculate Titl Compensated Heading
float tiltCompensate(Vector mag, Vector normAccel) {
  
  float roll;
  float pitch;

  roll = asin(normAccel.YAxis);
  pitch = asin(-normAccel.XAxis);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}

// Correct angle
float correctAngle(float heading) {
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }

  return heading;
}

//Used the previous three functions to get an accurate heading
float Calculate_Accurate_Headings(Vector norm) {
  float heading1, heading2;
  
  // Read vector
  Vector mag = compass.readNormalize();
  // Calculate headings
  heading1 = noTiltCompensate(mag);
  heading2 = tiltCompensate(mag, norm);

  if (heading2 == -1000)
  {
    heading2 = heading1;
  }

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (1.0 + (-0.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * 180/M_PI; 
  heading2 = heading2 * 180/M_PI; 
  
  return heading2;
}

///////////////////////////////
// RETURNING ANGLE FUNCTIONS //
///////////////////////////////

// Returns finger degree angle
int Finger_Flex() {
  int flex_value, flex_deg;
  flex_value = analogRead(finger_flexion_pin); // Read finger motion
  // 0   --> Closed Hand
  // 180 --> Open Hand
  flex_deg = map(flex_value,finger_data[1],finger_data[0], 180, 0);
  //flex_deg = MovingAverage(flex_deg);
  return flex_deg;
}

// Returns elbow degree angle
int Elbow_Flex() {
  int flex_value, flex_deg;
  flex_value = analogRead(elbow_flexion_pin); // Read elbow motion
  // 0   --> Elbow bent up fully
  // 125 --> Elbow not bent
  flex_deg = map(flex_value,elbow_data[1],elbow_data[0], 125, 0);
  //Elbow can not be bent backwards
  flex_deg = constrain(flex_deg, 125, 0);
  //flex_deg = MovingAverage(flex_deg);
  return flex_deg;
} 

// Returns wrist degree angle for flexion
int Wrist_Flex() {
  int up_flex_value, up_flex_deg, down_flex_value, down_flex_deg, final_degree;
   
  up_flex_value = analogRead(upward_wrist_pin); // Read upward wrist motion
  down_flex_value = analogRead(downward_wrist_pin); // Read downward wrist motion

  // 90  --> Palm Down
  // 180 --> Palm Facing Forward
  // 0   --> Palm Facing User
  up_flex_deg = map(up_flex_value,upward_wrist_data[1],upward_wrist_data[0], 90, 180);
  down_flex_deg = map(down_flex_value,downward_wrist_data[1],downward_wrist_data[0], 90, 0);

  //Motor can only move from 0 to 180
  up_flex_deg = constrain(up_flex_deg, 90, 180);
  down_flex_deg = constrain(down_flex_deg, 0, 90);

  //Determines which flex sensor to use (up or down)
  if (up_flex_deg > 100)
    final_degree = up_flex_deg;
  else
    final_degree = down_flex_deg;

  //final_degree = MovingAverage(final_degree);
  
  return final_degree;
} 

//Returns the shoudler pitch degree
int Wrist_Roll(Vector norm) {
  struct Angles a;
  int max_angle,min_angle;
  a = getAngles(norm);

  //This check is in case the person rotates past -180 to +180
  if(wrist_data[2] > -180)
    min_angle = wrist_data[2];
  else
    min_angle = -180;

  //This is to make the flat placement on the palm the flat placement on robotic hand
  if(wrist_data[3] > -20)
    max_angle = wrist_data[3];
  else
    max_angle = -20;

  // 0 --> palm facing up
  // 180 --> palm facing down
  int roll = map(a.roll, min_angle, max_angle, 0, 180);
  //Motor can not move more than 180 degrees, it will stop if made to go further
  roll = constrain(roll, 0, 180);

  //This is to prevent the flipping of the motor when going from -179 to +179
  if(a.roll > 100 || a.roll < min_angle)
    roll = 0;
      
  return roll;
  
}

//Returns the shoulder yaw degree
int Shoulder_Yaw(Vector scaled) {
  float heading_value = Calculate_Accurate_Headings(scaled);
  //Account for NAN values
  if(isnan(heading_value))
    heading_value = previous_heading;
  int yaw = map(heading_value, heading_data[1], heading_data[0], 0, 135);
  yaw = constrain(yaw, 0, 135);
  //Save heading in case a NAN value is found in next iteration
  previous_heading = heading_value;
  return yaw;
}

//Returns the shoudler pitch degree
int Shoulder_Pitch(Vector norm) {
  struct Angles a;
  int max_angle,min_angle;
  a = getAngles(norm);

  //Checks for highest shoulder lift
  if(shoulder_data[0] > -90)
    min_angle = shoulder_data[0];
  else
    min_angle = -90;

  //Checks for lowest shoulder lift
  if(shoulder_data[1] < 90)
    max_angle = shoulder_data[1];
  else
    max_angle = 90;

  // 0   --> Shoulder lifted straight up
  // 180 --> Shoulder flat down with no raising
  int pitch = map(a.pitch, max_angle, min_angle, 180, 0);
  // Motor can not move more than 0 to 180
  pitch = constrain(pitch, 0, 180);
  
  return pitch;
}

/////////////////////////
// DEBUGGING FUNCTIONS //
/////////////////////////

//Print min and max for accelerometers
void PrintMinMaxAngles(int *min_max){
  
  Serial.print (startOfCharacterDelimiter);    
  Serial.print ("------------------------------------");
  Serial.print (" Minimum Pitch = ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
  
  Serial.print (startOfNumberDelimiter);        
  Serial.print (min_max[0]);    
  Serial.print (endOfNumberDelimiter);   
  Serial.println ();
  
  Serial.print (startOfCharacterDelimiter);    
  Serial.print (" Maximum Pitch = ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
  
  Serial.print (startOfNumberDelimiter);       
  Serial.print (min_max[1]);
  Serial.print (endOfNumberDelimiter);   
  Serial.println ();
  
  Serial.print (startOfCharacterDelimiter);      
  Serial.print (" Minimum Roll = ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
  
  Serial.print (startOfNumberDelimiter);      
  Serial.print (min_max[2]);
  Serial.print (endOfNumberDelimiter);   
  Serial.println ();
  
  Serial.print (startOfCharacterDelimiter);      
  Serial.print (" Maximum Roll = ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
  
  Serial.print (startOfNumberDelimiter);      
  Serial.print (min_max[3]);
  Serial.print (endOfNumberDelimiter);   
  Serial.println ();
  
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("------------------------------------");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
}


//Print min and max for flex sensors
void PrintMinMaxFlex(int *data){
  
  Serial.print (startOfCharacterDelimiter);        
  Serial.print ("------------------------------------");
  Serial.print (" Minimum Flex Value= ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();

  Serial.print (startOfNumberDelimiter);      
  Serial.print (data[0]);    
  Serial.print (endOfNumberDelimiter);   
  Serial.println ();
  
  Serial.print (startOfCharacterDelimiter);        
  Serial.print (" Maximum Flex Value= ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();

  Serial.print (startOfNumberDelimiter);        
  Serial.print (data[1]);
  Serial.print (endOfNumberDelimiter);   
  Serial.println ();
  
  Serial.print (startOfCharacterDelimiter);        
  Serial.print ("------------------------------------");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
}

//Print min and max heading
void PrintMinMaxHeading(int *data){
  
  Serial.print (startOfCharacterDelimiter);        
  Serial.print ("------------------------------------");
  Serial.print (" Minimum Heading= ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();

  Serial.print (startOfNumberDelimiter);      
  Serial.print (data[0]);    
  Serial.print (endOfNumberDelimiter);   
  Serial.println ();
  
  Serial.print (startOfCharacterDelimiter);        
  Serial.print (" Maximum Heading= ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();

  Serial.print (startOfNumberDelimiter);        
  Serial.print (data[1]);
  Serial.print (endOfNumberDelimiter);   
  Serial.println ();
  
  Serial.print (startOfCharacterDelimiter);        
  Serial.print ("------------------------------------");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
}

void loop(){

   int shoulder_pitch, shoulder_yaw, elbow_flexion;
   int wrist_roll, wrist_flexion, finger_flexion;
   
   //Read data from wrist accelerometer
   Vector wrist_norm = w_acc.readNormalize();
   //Read data from shoulder accelerometer
   Vector shoulder_norm = s_acc.readNormalize();
   //Used for tilt compensation
   Vector shoulder_scaled = s_acc.readScaled(); 

   //Obtain all six angles needed for movement
   shoulder_pitch = Shoulder_Pitch(shoulder_norm);
   shoulder_yaw = Shoulder_Yaw(shoulder_scaled);
   elbow_flexion = Elbow_Flex();
   wrist_roll = Wrist_Roll(wrist_norm);
   wrist_flexion = Wrist_Flex();
   finger_flexion = Finger_Flex(); 

   //Serial.print ("Shoulder Pitch Degree: "); //FOR DEBUGGING 
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (shoulder_pitch);          // send shoulder pitch angle
   Serial.print (endOfDegreeDelimiter);  
   Serial.println ();
   delay(2);

   //Serial.print ("Shoulder Yaw Degree: "); //FOR DEBUGGING 
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (shoulder_yaw);            // send shoulder yaw angle
   Serial.print (endOfDegreeDelimiter);  
   Serial.println ();
   delay(2);

   //Serial.print ("Elbow Flex Degree: "); //FOR DEBUGGING 
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (elbow_flexion);           // send elbow flexion angle
   Serial.print (endOfDegreeDelimiter);
   Serial.println ();
   delay(5);
   
   //Serial.print ("Wrist Roll Degree: "); //FOR DEBUGGING 
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (wrist_roll);              // send wrist roll angle
   Serial.print (endOfDegreeDelimiter);
   Serial.println ();
   delay(2);

   //Serial.print ("Wrist Flex Degree: "); //FOR DEBUGGING   
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (wrist_flexion);           // send wrist flexion angle
   Serial.print (endOfDegreeDelimiter);
   Serial.println ();
   delay(5);
   
   //Serial.print ("Finger Flex Degree: "); //FOR DEBUGGING 
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (finger_flexion);          // send finger flexion angle
   Serial.print (endOfDegreeDelimiter);
   Serial.println ();
   delay(5);

   //Takes a total of 21 milliseconds all degree values to be sent
}
