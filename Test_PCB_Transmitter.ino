#include <Wire.h>
#include <Servo.h>
#include <L3G.h>
#include <ADXL345.h>

ADXL345 accelerometer;
const int flexpin = 38;
static int simple_flex_data[1];

const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter   = '>';

void setup() {
  Serial.begin(115200);
  Serial.println("Initialize ADXL345");
  if (!accelerometer.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    delay(500);
  }
  accelerometer.setRange(ADXL345_RANGE_16G);
    
}

int *Calibrate_Flex(int flexposition){
  for(int i=0; i<20; i++)
  {
     simple_flex_data[0] = min(simple_flex_data[0], flexposition);
     simple_flex_data[1] = max(simple_flex_data[1], flexposition);
     delay(20);
  }
  if(simple_flex_data[0] == 0)
  {
     simple_flex_data[0] = flexposition;
  } 
  return simple_flex_data;  
}

void PrintCaliFlex(int flexposition)
{
  int *cali_flex = Calibrate_Flex(flexposition);
  Serial.println("Flex Sensor Calibration Data");
  Serial.print(" Min: ");
  Serial.print(cali_flex[0]);
  Serial.print("     ");
  Serial.print(" Max: ");
  Serial.println(cali_flex[1]);
}

void loop() {
  int flexposition = analogRead(flexpin);
  Serial.print(flexposition);
  
  if(flexposition > 500)
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    
  Vector norm = accelerometer.readNormalize();
  Vector filtered = accelerometer.lowPassFilter(norm, 0.5);
  int pitch = -(atan2(norm.XAxis, sqrt(norm.YAxis*norm.YAxis + norm.ZAxis*norm.ZAxis))*180.0)/M_PI;
  int roll  = (atan2(norm.YAxis, norm.ZAxis)*180.0)/M_PI;
  int fpitch = -(atan2(filtered.XAxis, sqrt(filtered.YAxis*filtered.YAxis + filtered.ZAxis*filtered.ZAxis))*180.0)/M_PI;
  int froll  = (atan2(filtered.YAxis, filtered.ZAxis)*180.0)/M_PI;
  //fpitch = map(fpitch, 0,90,90,180);
  
  if(fpitch > 0)
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

   Serial.print (startOfNumberDelimiter);    
   Serial.print (flexposition);    // send the number
   Serial.print (endOfNumberDelimiter);  
   Serial.print (fpitch);    // send the number
   Serial.print (endOfNumberDelimiter);
   Serial.print (froll);    // send the number
   Serial.print (endOfNumberDelimiter);
}
