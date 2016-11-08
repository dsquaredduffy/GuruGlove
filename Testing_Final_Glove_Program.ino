#include <Wire.h>
#include <ADXL345.h>
#include <HMC5883L.h>
#define  STOPPER 0              /* Smaller than any datum */
#define  MEDIAN_FILTER_SIZE (17)

HMC5883L compass;
ADXL345 wrist_acc, shoulder_acc;

const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter   = '>';

// Pin values for flex sensor inputs
const int finger_pin = 0; 
const int elbow_pin = 1; 
const int fwrist_pin = 2; 
const int bwrist_pin = 3; 

//Float pointers to hold address to data location of min/max values
static float *finger_data, *elbow_data, *fwrist_data, *bwrist_data, *wrist_data, *shoulder_data;

// This is ran once at the beginning of the program
void setup(void) 
{
  Serial.begin(115200);
  // Initialise the ADXL345
  /*if(!wrist_acc.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    delay(500);
  }
  //Set the range
  wrist_acc.setRange(ADXL345_RANGE_16G);

  // Initialise the ADXL345
  if(!shoulder_acc.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    delay(500);
  }
  //Set the range
  shoulder_acc.setRange(ADXL345_RANGE_16G);
  
  // Initialize HMC5883L
  while (!compass.begin())
  {
    Serial.println("Failed to begin compass sensor!");
    while(1);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);*/

  Calibrate_Sensors();
}

//Calls the calibration function for all sensors 
void Calibrate_Sensors()
{
  Serial.println("About to begin calibrating finger");
  delay(3000);
  finger_data = Calibrate_Flex(finger_pin);
  
  Serial.println("About to begin calibrating elbow");
  delay(3000);
  elbow_data = Calibrate_Flex(elbow_pin);
  
  Serial.println("About to begin calibrating forward wrist motion");
  delay(3000);
  fwrist_data = Calibrate_Flex(fwrist_pin);
  
  Serial.println("About to begin calibrating backward wrist motion");
  delay(3000);
  bwrist_data = Calibrate_Flex(bwrist_pin);

 /* Serial.println("Calibrating wrist, twist wrist from one side to another");
  delay(3000);
  wrist_data = Calibrate_Acc(wrist_acc);

  Serial.println("Calibrating shoulder, move arm back and forth then rotate side to side");
  delay(3000);
  shoulder_data = Calibrate_Acc(shoulder_acc);*/
}

//Return min and max values of flex sensor
float *Calibrate_Flex(int flexpin){
  Serial.println("Calibration Began");
  static float simple_flex_data[1];
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
  
  Serial.print("Minimum Value: ");
  Serial.print(simple_flex_data[0]);
  Serial.print("        ");
  Serial.print("Maximum Value: ");
  Serial.println(simple_flex_data[1]);   
  Serial.println("Calibration Ended");

  simple_flex_data[0] = 0;
  simple_flex_data[1] = 1023;
  
  return simple_flex_data;  
}

//Return min and max values of accelerometer sensor
float *Calibrate_Acc(ADXL345 accelerometer){
  Serial.println("Calibration Began");
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
  delay(10);
  }
  
  if(acc_data[5] > 20)
  {
     acc_data[5] = z;
  }
  Serial.println("Calibration Ended"); 
  return acc_data;
}

// Return the current time
float time() {
  return float( micros() ) * 1e-6;
}

// Return pitch and roll
float *DetermineAccPitchRoll(Vector norm)
{
  static float angles[1];
  //Pitch Calculation -- Check the negative pitch values
  angles[0] = -(atan2(norm.XAxis, sqrt(norm.YAxis*norm.YAxis + norm.ZAxis*norm.ZAxis))*180.0)/M_PI;
  //Roll Calculation
  angles[1] = (atan2(norm.YAxis, norm.ZAxis)*180.0)/M_PI;
  return angles;
}

// Return max pitch and roll angle for accelerometer
float *MaxAccPitchRoll(float *data)
{
  static float angles[1];
  //Minimum Pitch and Roll
  //Pitch Calculation -- Check the negative pitch values
  angles[0] = -(atan2(data[0], sqrt(data[1]*data[1] + data[2]*data[2]))*180.0)/M_PI;
  //Roll Calculation
  angles[1] = (atan2(data[1], data[2])*180.0)/M_PI;

  //Maximum Pitch and Roll
  //Pitch Calculation -- Check the negative pitch values
  angles[2] = -(atan2(data[3], sqrt(data[4]*data[4] + data[5]*data[5]))*180.0)/M_PI;
  //Roll Calculation
  angles[3] = (atan2(data[4], data[5])*180.0)/M_PI;
  return angles;
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
  static int *angles = Filtered_Acc_Roll_Pitch(normAccel);
  
  if (angles[1] > 0.78 || angles[1] < -0.78 || angles[0] > 0.78 || angles[0] < -0.78)
  {
    return -1000;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(angles[1]);
  float sinRoll = sin(angles[1]);  
  float cosPitch = cos(angles[0]);
  float sinPitch = sin(angles[0]);

  // Tilt compensation ultizing the acceleromter angle values
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

// Returns filtered pitch and roll
int *Filtered_Acc_Roll_Pitch(Vector norm)
{
  static int filt_acc_data[1];
  //Retrive accelerometer angle values
  float *medata = DetermineAccPitchRoll(norm);
  //Pass angle values to get filtered
  filt_acc_data[0] = median_filter(medata[0]);
  filt_acc_data[1] = median_filter(medata[1]);
  return filt_acc_data;
}

int Calculate_Accurate_Headings(Vector norm)
{
  int tc_heading;
  float c_heading[1];
  // Read vector
  Vector mag = compass.readNormalize();
  // Calculate headings
  c_heading[0] = noTiltCompensate(mag);
  c_heading[1] = tiltCompensate(mag, norm);

  if (c_heading[1] == -1000)
  {
    c_heading[1] = c_heading[0];
  }

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (1.0 + (-0.0 / 60.0)) / (180 / M_PI);
  c_heading[0] += declinationAngle;
  c_heading[1] += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  c_heading[0] = correctAngle(c_heading[0]);
  c_heading[1] = correctAngle(c_heading[1]);

  // Convert to degrees
  c_heading[0] = c_heading[0] * 180/M_PI; 
  c_heading[1] = c_heading[1] * 180/M_PI; 
  tc_heading = c_heading[1];
  
  return tc_heading;
}

// Returns finger degree angle
int Finger_Flex()
{
  int flexposition, flex_deg;    // Input value from the analog pin.
  flexposition = analogRead(finger_pin); // Read finger motion
  flex_deg = map(flexposition,finger_data[0],finger_data[1], 115, 300);
  return flex_deg;
}

// Returns elbow degree angle
int Elbow_Flex()
{
  int flexposition, flex_deg;    // Input value from the analog pin.
  flexposition = analogRead(elbow_pin); // Read elbow motion
  flex_deg = map(flexposition,elbow_data[1],elbow_data[0], 90, 240);
  return flex_deg;
} 

// Returns wrist degree angle for flexion
int Wrist_Flex()
{
  int f_pos,b_pos;                // Input value from the analog pin.
  int final_degree;
  static int flex_deg[1];
  f_pos = analogRead(fwrist_pin); // Read forward wrist motion
  b_pos = analogRead(bwrist_pin); // Read backward wrist motion
  flex_deg[0] = map(f_pos,fwrist_data[0],fwrist_data[1], 90, 180);
  flex_deg[1] = map(b_pos,bwrist_data[1],bwrist_data[0], 0, 90);
  if(flex_deg[0] == 90)
    final_degree = flex_deg[1];
  else if(flex_deg[1] == 0)
    final_degree = flex_deg[0];
  return final_degree;
} 

//Returns the wrist roll degree
int Wrist_Roll(Vector norm)
{
  int max_angle,min_angle;
  static float *min_max_angle;
  min_max_angle = MaxAccPitchRoll(wrist_data);
  static int *angles;
  angles = Filtered_Acc_Roll_Pitch(norm);
  int roll = angles[1];

  if(min_max_angle[1] > 0)
    min_angle = min_max_angle[1];
  else
    min_angle = 0;

  if(min_max_angle[3] < 180)
    max_angle = min_max_angle[3];
  else
    max_angle = 180;
    
  roll = map(roll, min_angle, max_angle, 180, 0);
  //roll = map(roll, 0, 180, 180, 0);
  if(320 < roll  && roll < 359)
    roll = 0;
  return roll;
}

//Returns the shoulder yaw degree
int Shoulder_Yaw(Vector norm)
{
  int max_angle,min_angle;
  int yaw = Calculate_Accurate_Headings(norm);
  return yaw;
}

//Returns the shoudler pitch degree
int Shoulder_Pitch(Vector norm)
{
  int max_angle,min_angle;
  static float *min_max_angle;
  min_max_angle = MaxAccPitchRoll(shoulder_data);
  static int *angles;
  angles = Filtered_Acc_Roll_Pitch(norm);
  int pitch = angles[0];
  
  if(min_max_angle[0] > 0)
    min_angle = min_max_angle[1];
  else
    min_angle = 0;

  if(min_max_angle[2] < 90)
    max_angle = min_max_angle[3];
  else
    max_angle = 90;
    
  pitch =  map(pitch, min_angle, max_angle, 90, 180);    
  //pitch =  map(pitch, 0, 90, 90, 180);
  return pitch;
}

void loop(void) 
{
  int shoulder_pitch, shoulder_yaw, elbow_flexion, wrist_roll, wrist_flexion, finger_flexion;
  //Read data from wrist accelerometer
  //Vector wrist_norm = wrist_acc.readNormalize();
  //Read data from shoulder accelerometer
  //Vector shoulder_norm = shoulder_acc.readNormalize();

  //Obtain all six angles needed for movement
  //shoulder_pitch = Shoulder_Pitch(shoulder_norm);
  //shoulder_yaw = Shoulder_Yaw(shoulder_norm);
  elbow_flexion = Elbow_Flex();
  //wrist_roll = Wrist_Roll(wrist_norm);
  wrist_flexion = Wrist_Flex();
  finger_flexion = Finger_Flex();

   /*Serial.print (startOfNumberDelimiter);    
   Serial.print (shoulder_pitch);          // send shoulder pitch angle
   Serial.print (endOfNumberDelimiter);  
   Serial.println ();

   Serial.print (startOfNumberDelimiter);    
   Serial.print (shoulder_yaw);            // send shoulder yaw angle
   Serial.print (endOfNumberDelimiter);  
   Serial.println ();*/

   Serial.print ("Elbow Flex Degree: ");
   Serial.print (startOfNumberDelimiter);    
   Serial.print (elbow_flexion);           // send elbow flexion angle
   Serial.print (endOfNumberDelimiter);
   Serial.println ();

   /*Serial.print (startOfNumberDelimiter);    
   Serial.print (wrist_roll);              // send wrist roll angle
   Serial.print (endOfNumberDelimiter);
   Serial.println ();*/
   
   Serial.print ("Wrist Flex Degree: ");
   Serial.print (startOfNumberDelimiter);    
   Serial.print (wrist_flexion);           // send wrist flexion angle
   Serial.print (endOfNumberDelimiter);
   Serial.println ();

   Serial.print ("Finger Flex Degree: ");
   Serial.print (startOfNumberDelimiter);    
   Serial.print (finger_flexion);          // send finger flexion angle
   Serial.print (endOfNumberDelimiter);
   Serial.println ();

   delay(1000);
}
