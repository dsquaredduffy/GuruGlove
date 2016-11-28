#include <Wire.h>
#include <ADXL345.h>
#include <HMC5883L.h>
#define  STOPPER 0              /* Smaller than any datum */
#define  MEDIAN_FILTER_SIZE (17)

HMC5883L compass;
ADXL345 wrist_accelerometer, shoulder_accelerometer;

const char startOfDegreeDelimiter    = '<';
const char endOfDegreeDelimiter      = '>';

const char startOfNumberDelimiter    = '{';
const char endOfNumberDelimiter      = '}';

const char startOfCharacterDelimiter = '[';
const char endOfCharacterDelimiter   = ']';

// Pin values for flex sensor inputs
const int finger_flexion_pin = 1;   //PIN 39
const int elbow_flexion_pin  = 2;   //PIN 38
const int upward_wrist_pin = 3;     //PIN 37
const int downward_wrist_pin = 4;   //PIN 36

//Float pointers to hold address to data location of min/max values
static float *wrist_data, *shoulder_data;
static int *finger_data, *elbow_data, *upward_wrist_data, *backward_wrist_data;
int shoulder_pitch, shoulder_yaw, elbow_flexion, wrist_roll, wrist_flexion, finger_flexion;

struct Angles {
    float pitch, roll;
    float min_pitch, max_pitch, min_roll, max_roll;
    float fpitch, froll;
};

struct Angles getAngles(Vector norm) {
    float pitch = -(atan2(norm.XAxis, sqrt(norm.YAxis*norm.YAxis + norm.ZAxis*norm.ZAxis))*180.0)/M_PI;
    float roll  = (atan2(norm.YAxis, norm.ZAxis)*180.0)/M_PI;
    
    Angles angles = { pitch, roll };
    return angles;    
}

struct Angles getMinMaxAngles(float *min_max) {
  
    float min_pitch = -(atan2(min_max[0], sqrt(min_max[1]*min_max[1] + min_max[2]*min_max[2]))*180.0)/M_PI;
    float min_roll  = (atan2(min_max[1], min_max[2])*180.0)/M_PI;
    
    float max_pitch = -(atan2(min_max[3], sqrt(min_max[4]*min_max[4] + min_max[5]*min_max[5]))*180.0)/M_PI;
    float max_roll  = (atan2(min_max[4], min_max[5])*180.0)/M_PI;
    
    Angles min_max_angles = { min_pitch, min_roll, max_pitch, max_roll };
    return min_max_angles;    
}

// Returns filtered pitch and roll
struct Angles getFilteredAngles(float angles[1])
{
  float fpitch = median_filter(angles[0]);
  float froll = median_filter(angles[1]);

  Angles fangles = { fpitch, froll };
  return fangles;  
}

// This is ran once at the beginning of the program
void setup(void) 
{
  Serial.begin(57600);

  // Address for SDO high
  wrist_accelerometer.setAddress(0x1D);
  // Address for SDO low
  shoulder_accelerometer.setAddress(0x53);
    
 /// Initialise the ADXL345
  if(!wrist_accelerometer.begin())
  {
    Serial.print (startOfCharacterDelimiter);    
    Serial.print ("Ooops, no wrist ADXL345 detected ... Check your wiring!");
    Serial.print (endOfCharacterDelimiter);   
    Serial.println (); 
    delay(500);
  }
  //Set the range
  wrist_accelerometer.setRange(ADXL345_RANGE_16G);

  // Initialise the ADXL345
  if(!shoulder_accelerometer.begin())
  {
    Serial.print (startOfCharacterDelimiter);    
    Serial.println("Ooops, no shoulder ADXL345 detected ... Check your wiring!");
    Serial.print (endOfCharacterDelimiter);   
    Serial.println (); 
    delay(500);
  }
  //Set the range
  shoulder_accelerometer.setRange(ADXL345_RANGE_16G);
  
  // Initialize HMC5883L
  while (!compass.begin())
  {
    Serial.print (startOfCharacterDelimiter);    
    Serial.println("Failed to begin compass sensor!");
    Serial.print (endOfCharacterDelimiter);   
    Serial.println (); 
    while(1);
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
void Calibrate_Sensors()
{
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("About to begin calibrating finger");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 

  delay(2000);
  finger_data = Calibrate_Flex(finger_flexion_pin);
  
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("About to begin calibrating elbow");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(2000);
  elbow_data = Calibrate_Flex(elbow_flexion_pin);

  Serial.print (startOfCharacterDelimiter);        
  Serial.print ("About to begin calibrating forward wrist motion");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(2000);
  upward_wrist_data = Calibrate_Flex(upward_wrist_pin);

  Serial.print (startOfCharacterDelimiter);        
  Serial.print ("About to begin calibrating backward wrist motion");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(2000);
  backward_wrist_data = Calibrate_Flex(downward_wrist_pin);

  /*Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibrating wrist, twist wrist from one side to another");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(2000);
  wrist_data = Calibrate_Acc(wrist_accelerometerelerometer);

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibrating shoulder, move arm back and forth then rotate side to side");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  delay(2000);
  shoulder_data = Calibrate_Acc(shoulder_acc);*/
}

//Return min and max values of flex sensor
int *Calibrate_Flex(int flexpin){
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibration Began");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  static int simple_flex_data[1];
  memset(simple_flex_data, 0, sizeof(simple_flex_data)); // clear the array
  int flex_value;
  //Take in a 800 readings and take the min and max values of the readings
  //Will take 8 seconds to complete
  for(int i=0; i<800; i++)
  {
     flex_value = analogRead(flexpin); // Read flex sensor
     simple_flex_data[0] = min(simple_flex_data[0], flex_value);
     simple_flex_data[1] = max(simple_flex_data[1], flex_value);
     //Due to all initial values being zero, we take in the minimum right above zero
     if(simple_flex_data[0] == 0)
        simple_flex_data[0] = flex_value;
     if(simple_flex_data[1] == 1023 && flex_value != simple_flex_data[0])
        simple_flex_data[1] = flex_value;  
     delay(10);
  }
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Minimum Value: ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();

  Serial.print (startOfNumberDelimiter);    
  Serial.print (simple_flex_data[0]);
  Serial.print (endOfNumberDelimiter);
  Serial.println ();
   
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Maximum Value: ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
  
  Serial.print (startOfNumberDelimiter);        
  Serial.print (simple_flex_data[1]);  
  Serial.print (endOfNumberDelimiter);
  Serial.println ();
  
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibration Ended");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 

  return simple_flex_data;  
  
}

//Return min and max values of accelerometer sensor
float *Calibrate_Acc(ADXL345 accelerometer){
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibration Began");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  
  static float acc_data[5];
  Vector norm = accelerometer.readNormalize();
  float z;
  //Take in a 800 readings and take the min and max values of the readings
  //Will take 8 seconds to complete
  for(int i=0; i<800; i++)
  {
  Vector norm = accelerometer.readNormalize();
  z = norm.ZAxis;
  acc_data[0] = min(acc_data[0], norm.XAxis);
  acc_data[1] = min(acc_data[1], norm.YAxis);
  acc_data[2] = min(acc_data[2], norm.ZAxis);

  acc_data[3] = max(acc_data[3], norm.XAxis);
  acc_data[4] = max(acc_data[4], norm.YAxis);
  acc_data[5] = max(acc_data[5], norm.ZAxis);

  if(acc_data[5] > 20)
     acc_data[5] = z;
  
  delay(10);
  }

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Minimum X Axis Value: ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();

  Serial.print (startOfNumberDelimiter);    
  Serial.print (acc_data[0]);
  Serial.print (endOfNumberDelimiter);
  Serial.println ();
   
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Maximum X Axis Value: ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
  
  Serial.print (startOfNumberDelimiter);        
  Serial.print (acc_data[3]);  
  Serial.print (endOfNumberDelimiter);
  Serial.println ();

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Minimum Y Axis Value: ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();

  Serial.print (startOfNumberDelimiter);    
  Serial.print (acc_data[1]);
  Serial.print (endOfNumberDelimiter);
  Serial.println ();
   
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Maximum Y Axis Value: ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
  
  Serial.print (startOfNumberDelimiter);        
  Serial.print (acc_data[4]);  
  Serial.print (endOfNumberDelimiter);
  Serial.println ();

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Minimum Z Axis Value: ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();

  Serial.print (startOfNumberDelimiter);    
  Serial.print (acc_data[2]);
  Serial.print (endOfNumberDelimiter);
  Serial.println ();
   
  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Maximum Z Axis Value: ");
  Serial.print (endOfCharacterDelimiter);   
  Serial.println ();
  
  Serial.print (startOfNumberDelimiter);        
  Serial.print (acc_data[5]);  
  Serial.print (endOfNumberDelimiter);
  Serial.println ();

  Serial.print (startOfCharacterDelimiter);      
  Serial.print ("Calibration Ended"); 
  Serial.print (endOfCharacterDelimiter);   
  Serial.println (); 
  
  return acc_data;
}

// No tilt compensation
float noTiltCompensate(Vector mag)
{
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}
 
// Tilt compensation
float tiltCompensate(Vector mag, Vector normAccel)
{
  // Pitch & Roll 

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
float correctAngle(float heading)
{
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }

  return heading;
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

float Calculate_Accurate_Headings(Vector norm)
{
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
  // For Bytom / Poland declination angle is 4'26E (positive)
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

// Returns finger degree angle
int Finger_Flex()
{
  int flex_deg;  
  flex_deg = analogRead(finger_flexion_pin); // Read finger motion
  //flex_deg = map(flexposition,finger_data[1],finger_data[0], 115, 300);
  flex_deg = map(flex_deg,finger_data[1],finger_data[0], 0, 180);
  return flex_deg;
}

// Returns elbow degree angle
int Elbow_Flex()
{
  int flex_deg;
  flex_deg = analogRead(elbow_flexion_pin); // Read elbow motion
  //flex_deg = map(flexposition,elbow_data[1],elbow_data[0], 90, 240);
  flex_deg = map(flex_deg,elbow_data[1],elbow_data[0], 90, 0);
  return flex_deg;
} 

// Returns wrist degree angle for flexion
int Wrist_Flex()
{
  int up_deg, down_deg, final_degree;
  
  up_deg = analogRead(upward_wrist_pin); // Read upward wrist motion
  down_deg = analogRead(downward_wrist_pin); // Read downward wrist motion
  
  up_deg = map(up_deg,upward_wrist_data[1],upward_wrist_data[0], 90, 180);
  up_deg = constrain(up_deg, 90, 180);
  
  down_deg = map(down_deg,backward_wrist_data[0],backward_wrist_data[1], 0, 90);
  down_deg = constrain(down_deg, 0, 90);

  /*flex_deg[0] = map(f_pos,upward_wrist_data[1],upward_wrist_data[0], 135, 180);
  flex_deg[0] = constrain(flex_deg[0], 90, 180);
  flex_deg[1] = map(b_pos,backward_wrist_data[0],backward_wrist_data[1], 45, 135);
  flex_deg[1] = constrain(flex_deg[1], 0, 90);*/


  if(up_deg > 90 && up_deg < 180)
    final_degree = up_deg;
  else if(down_deg > 0 && down_deg < 90)
    final_degree = down_deg;
  else if(up_deg == 180)
    final_degree = 180;
  else if(up_deg == 90 || down_deg == 90)
    final_degree = 90;
  else if(down_deg == 0)
    final_degree = 0;
    
  return final_degree;
} 

//Returns the wrist roll degree
int Wrist_Roll(Vector norm)
{
  /*int max_angle,min_angle;
  static float *min_max_angle;
  static int *min_max_angle = MaxAccPitchRoll(wrist_data);
  
  if(min_max_angle[1] > -180)
    min_angle = min_max_angle[1];
  else
    min_angle = 0;
    
  if(min_max_angle[3] < 180)
    max_angle = min_max_angle[3];
  else
    max_angle = 180;*/
  
  
  struct Angles a;
  a = getAngles(norm);
  a = getMinMaxAngles(shoulder_data);
        
  //roll = map(roll, a.min_roll, a.max_roll, 0, 360);
  int roll = map(a.roll, -180, 180, 0, 360);
  if(320 < roll  && roll < 359)
    roll = 0;
    
  return roll;
}

//Returns the shoulder yaw degree
int Shoulder_Yaw(Vector norm)
{
  float heading = Calculate_Accurate_Headings(norm);
  int yaw = map(yaw, 0, 360, 0, 360);
  return yaw;
}

//Returns the shoudler pitch degree
int Shoulder_Pitch(Vector norm)
{
  /*int max_angle,min_angle;
  static int *min_max_angle = MaxAccPitchRoll(shoulder_data);  

  if(min_max_angle[0] > 0)
    min_angle = min_max_angle[0];
  else
    min_angle = 0;

  if(min_max_angle[2] < 90)
    max_angle = min_max_angle[2];
  else
    max_angle = 90;*/
  
  struct Angles a;
  float angles[1];
  a = getAngles(norm);
  angles[0] = a.pitch;
  angles[1] = a.roll;
  a = getMinMaxAngles(shoulder_data);
  a = getFilteredAngles(angles);
  
  //int pitch =  map(pitch, a.min_pitch, a.max_pitch, 90, 180);    
  int pitch =  map(a.pitch, 0, 90, 90, 180);
  
  return pitch;
}

void loop(void) 
{
  //Read data from wrist accelerometer
  Vector wrist_norm = wrist_accelerometer.readNormalize();
  //Read data from shoulder accelerometer
  Vector shoulder_norm = shoulder_accelerometer.readNormalize();
  //Used for tilt compensation
  Vector shoulder_scaled = shoulder_accelerometer.readScaled();
  
  //Obtain all six angles needed for movement
  shoulder_pitch = Shoulder_Pitch(shoulder_norm);
  shoulder_yaw = Shoulder_Yaw(shoulder_scaled);
  elbow_flexion = Elbow_Flex();
  wrist_roll = Wrist_Roll(wrist_norm);
  wrist_flexion = Wrist_Flex();
  finger_flexion = Finger_Flex();

   //Serial.print ("Shoulder Pitch Degree: ");  
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (shoulder_pitch);          // send shoulder pitch angle
   Serial.print (endOfDegreeDelimiter);  
   Serial.println ();
   delay(2);

   //Serial.print ("Shoulder Yaw Degree: ");
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (shoulder_yaw);            // send shoulder yaw angle
   Serial.print (endOfDegreeDelimiter);  
   Serial.println ();
   delay(2);

   //Serial.print ("Elbow Flex Degree: ");   
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (elbow_flexion);           // send elbow flexion angle
   Serial.print (endOfDegreeDelimiter);
   Serial.println ();
   delay(5);
   
   //Serial.print ("Wrist Roll Degree: ");   
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (wrist_roll);              // send wrist roll angle
   Serial.print (endOfDegreeDelimiter);
   Serial.println ();
   delay(2);

   //Serial.print ("Wrist Flex Degree: ");    
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (wrist_flexion);           // send wrist flexion angle
   Serial.print (endOfDegreeDelimiter);
   Serial.println ();
   delay(5);
   
   //Serial.print ("Finger Flex Degree: ");    
   Serial.print (startOfDegreeDelimiter);    
   Serial.print (finger_flexion);          // send finger flexion angle
   Serial.print (endOfDegreeDelimiter);
   Serial.println ();
   delay(5);
}
