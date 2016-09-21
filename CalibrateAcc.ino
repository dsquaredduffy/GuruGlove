#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
char report[80];
int min_x,min_y,min_z,max_x,max_y,max_z;

void setup() {
  Serial.begin(9600);
  //Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);

  Serial.println("");
}

void loop() {  
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
  
  min_x = min(min_x, event.acceleration.x);
  min_y = min(min_y, event.acceleration.y);
  min_z = min(min_z, event.acceleration.z);

  max_x = max(max_x, event.acceleration.x);
  max_y = max(max_y, event.acceleration.y);
  max_z = max(max_z, event.acceleration.z);
  
  snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
    min_x, min_y, min_z,
    max_x, max_y, max_z);
  Serial.println(report);
  
  delay(100);
}
