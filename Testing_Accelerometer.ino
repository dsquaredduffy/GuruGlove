#include <Wire.h>
#include <ADXL345.h>

ADXL345 accelerometer;
static float *min_max;

const char startOfDegreeDelimiter    = '<';
const char endOfDegreeDelimiter      = '>';

const char startOfNumberDelimiter    = '{';
const char endOfNumberDelimiter      = '}';

const char startOfCharacterDelimiter = '[';
const char endOfCharacterDelimiter   = ']';

struct Angles {int pitch, roll;};
struct MinMaxAngles {int min_pitch, max_pitch, min_roll, max_roll;};

struct Angles getAngles(Vector norm) {
    int pitch = -(atan2(norm.XAxis, sqrt(norm.YAxis*norm.YAxis + norm.ZAxis*norm.ZAxis))*180.0)/M_PI;
    int roll  = (atan2(norm.YAxis, norm.ZAxis)*180.0)/M_PI;
    
    Angles angles = { pitch, roll };
    return angles;    
}

struct MinMaxAngles getMinMaxAngles() {    
    int min_pitch = (atan2(min_max[0], sqrt(min_max[1]*min_max[1] + min_max[2]*min_max[2]))*180.0)/M_PI;
    int min_roll  = (atan2(min_max[1], min_max[2])*180.0)/M_PI;
    
    int max_pitch = (atan2(min_max[3], sqrt(min_max[4]*min_max[4] + min_max[5]*min_max[5]))*180.0)/M_PI;
    int max_roll  = (atan2(min_max[4], min_max[5])*180.0)/M_PI;

    Serial.println("------------------------------------");
    Serial.print(" Minimum Pitch = ");
    Serial.print(min_pitch);    
    Serial.print("        ");
    Serial.print(" Maximum Pitch = ");
    Serial.println(max_pitch);
    Serial.print(" Minimum Roll = ");
    Serial.print(min_roll);
    Serial.print("        ");
    Serial.print(" Maximum Roll = ");
    Serial.println(max_roll);
    Serial.println("------------------------------------");
    
    MinMaxAngles min_max_angles = { min_pitch, max_pitch, min_roll, max_roll };
    return min_max_angles;    
}

void setup() {
  Serial.begin(115200);
  memset(min_max, 0, sizeof(min_max));
  accelerometer.setAddress(0x53);

 /// Initialise the ADXL345
  if(!accelerometer.begin())
  {
    Serial.print (startOfCharacterDelimiter);    
    Serial.print ("Ooops, no ADXL345 detected ... Check your wiring!");
    Serial.print (endOfCharacterDelimiter);   
    Serial.println (); 
    delay(500);
  }
  //Set the range
  accelerometer.setRange(ADXL345_RANGE_16G);

  min_max = Calibrate_Acc_Angles(accelerometer);
  PrintMinMaxAngles(); 
}

//Return min and max values of accelerometer sensor
float *Calibrate_Acc(ADXL345 accelerometer){
  
  static float acc_data[5];
  //Take in a 800 readings and take the min and max values of the readings
  //Will take 8 seconds to complete
  
  Serial.println("Countdown Begin");
  
  for(int i=0; i<800; i++)
  {
    Serial.print("Inside --> loop number ");
    Serial.println(i);
    Vector norm = accelerometer.readNormalize();
  
    acc_data[0] = min(acc_data[0], norm.XAxis);
    acc_data[1] = min(acc_data[1], norm.YAxis);
    acc_data[2] = min(acc_data[2], norm.ZAxis);
  
    acc_data[3] = max(acc_data[3], norm.XAxis);
    acc_data[4] = max(acc_data[4], norm.YAxis);
    acc_data[5] = max(acc_data[5], norm.ZAxis);
    
    delay(10);
  }

  return acc_data;
}

//Return min and max values of accelerometer sensor
float *Calibrate_Acc_Angles(ADXL345 accelerometer){
    
  Serial.println("Countdown Begin");
  struct Angles a;
  static float acc_data[4];
  
  for(int i=0; i<800; i++)
  {
    Serial.print("Inside --> loop number ");
    Serial.println(i);
    Vector norm = accelerometer.readNormalize();    
    a = getAngles(norm);

    Serial.print (" Pitch = ");
    Serial.print (a.pitch);     
    Serial.print("        ");    
    Serial.print (a.roll);
    Serial.println (); 
  
    acc_data[0] = min(acc_data[0], a.pitch);
    acc_data[1] = max(acc_data[1], a.pitch);
    
    acc_data[2] = min(acc_data[2], a.roll);
    acc_data[3] = max(acc_data[3], a.roll);
    
    delay(10);
  }

  Serial.println("------------------------------------");
  Serial.print(" Minimum Pitch = ");
  Serial.print(acc_data[0]);    
  Serial.print("        ");
  Serial.print(" Maximum Pitch = ");
  Serial.println(acc_data[1]);
  Serial.print(" Minimum Roll = ");
  Serial.print(acc_data[2]);
  Serial.print("        ");
  Serial.print(" Maximum Roll = ");
  Serial.println(acc_data[3]);
  Serial.println("------------------------------------");
  
  return acc_data;
}

void PrintMinMaxAngles(){
  
  //struct MinMaxAngles a;
  //a = getMinMaxAngles();
  
  Serial.println("------------------------------------");
  Serial.print(" Minimum Pitch = ");
  Serial.print(min_max[0]);    
  Serial.print("        ");
  Serial.print(" Maximum Pitch = ");
  Serial.println(min_max[1]);
  Serial.print(" Minimum Roll = ");
  Serial.print(min_max[2]);
  Serial.print("        ");
  Serial.print(" Maximum Roll = ");
  Serial.println(min_max[3]);
  Serial.println("------------------------------------");
}

void loop(){


  Vector norm = accelerometer.readNormalize();    
  struct Angles a;
  int max_angle,min_angle;
  a = getAngles(norm);

  if(min_max[2] > -180)
    min_angle = min_max[0];
  else
    min_angle = -180;
    
  if(min_max[3] < 180)
    max_angle = min_max[1];
  else
    max_angle = 180;


  int o_roll = map(a.roll, -180, 180, 0, 360);
  int m_roll = map(a.roll, min_angle, max_angle, 0, 360);

  Serial.print (" Original Roll = "); 
  Serial.print (a.roll);    
  Serial.print("        ");
  Serial.print (" Constant Mapped Roll = ");
  Serial.print (o_roll);
  Serial.print("        ");
  Serial.print (" Min Max Mapped Roll = ");   
  Serial.print (m_roll);
  Serial.println (); 
      
  delay(400);

}

