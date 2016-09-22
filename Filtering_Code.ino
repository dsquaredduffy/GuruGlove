#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <MedianFilter.h>
#include <MovingAverageFilter.h>
#include <Filters.h>
#include <L3G.h>
#define  STOPPER 0                                      /* Smaller than any datum */
#define  MEDIAN_FILTER_SIZE (17)

L3G gyro;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
MovingAverageFilter movingAverageFilter(5);

const int flexpin = 0, movesize = 20, maxsize = 20;
float values[movesize], accx[maxsize], accy[maxsize], accz[maxsize];
int counter = 0;

void setup(void) 
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

// return the current time
float time() {
  return float( micros() ) * 1e-6;
}

float *DetermineAccPitchRoll(sensors_event_t event)
{
  static float angles[1];
  //Pitch Calculation
  angles[0] = (atan2(event.acceleration.x,(sqrt(pow(event.acceleration.y,2)+pow(event.acceleration.z,2)))) * 180.0) / PI;
  //Roll Calculation
  angles[1] = (atan2(event.acceleration.y,(sqrt(pow(event.acceleration.x,2)+pow(event.acceleration.z,2)))) * 180.0) / PI;

  return angles;
}

float MovingAverage(float input)
{ 
  float output = movingAverageFilter.process(input);
  return output;
}

void updateArrays(sensors_event_t event)
{
  if(counter >= maxsize)
  {
    for (int k = 1; k > maxsize-1; k++)
    {   
      accx[k-1]=accx[k];
      accy[k-1]=accy[k];
      accz[k-1]=accz[k];
    }
    accx[maxsize-1] = event.acceleration.x;
    accy[maxsize-1] = event.acceleration.y;
    accz[maxsize-1] = event.acceleration.z;
  }
  else
  {
    accx[counter] = event.acceleration.x;
    accy[counter] = event.acceleration.y;
    accz[counter] = event.acceleration.z;
  }
  counter++;
}

int *quicksort(int x[],int first,int last)
{
  int pivot,j,temp,i;
   if(first<last){
       pivot=first;
       i=first;
       j=last;
       while(i<j){
           while(x[i]<=x[pivot]&&i<last)
               i++;
           while(x[j]>x[pivot])
               j--;
           if(i<j){
               temp=x[i];
                x[i]=x[j];
                x[j]=temp;
           }
       }
       temp=x[pivot];
       x[pivot]=x[j];
       x[j]=temp;
       quicksort(x,first,j-1);
       quicksort(x,j+1,last);
  }
  return x;
}

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

void PrintRawAcc(sensors_event_t event)
{
  //See Regular Data
  /*Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");*/

  //Used For Graphing
  Serial.print(event.acceleration.x); Serial.print(",");Serial.print(event.acceleration.y); Serial.print(",");
  Serial.print(event.acceleration.z);
}

void PrintRawGryo()
{
  //See Regular Data
  /*Serial.print("X: "); Serial.print((int)gyro.g.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print((int)gyro.g.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print((int)gyro.g.z); Serial.print("  ");Serial.println("m/s^2 ");*/

  //Used For Graphing
  Serial.print((int)gyro.g.x); Serial.print(",");Serial.print((int)gyro.g.y); Serial.print(",");
  Serial.println((int)gyro.g.z);
}

void PrintRawGryoandAcc(sensors_event_t event)
{
    //Used For Graphing
  Serial.print(event.acceleration.x); Serial.print(",");Serial.print(event.acceleration.y); Serial.print(",");
  Serial.print(event.acceleration.z);Serial.print(",");
  Serial.print((int)gyro.g.x); Serial.print(",");Serial.print((int)gyro.g.y); Serial.print(",");
  Serial.println((int)gyro.g.z);
}
void PrintAccPitchRoll(sensors_event_t event)
{
  float *data = DetermineAccPitchRoll(event);
  
  //See Regular Data
  /*Serial.print("Pitch:: "); Serial.print(data[0],2); Serial.print("  ");
  Serial.print("Roll: "); Serial.print(data[1],2); Serial.println("  ");*/

  //Used for graphing
  Serial.print((int)data[0]); Serial.print(","); Serial.println((int)data[1]);
}

void PrintAccAndAngles(sensors_event_t event)
{
  float *data = DetermineAccPitchRoll(event);
  int pitch = data[0];
  int roll = data[1];
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.print("m/s^2 ");
  Serial.print("                  ");
  Serial.print("Pitch:: "); Serial.print(pitch); Serial.print("  "); Serial.print("Roll: "); Serial.print(roll); Serial.println("  ");
}

void PrintMedFiltAcc(sensors_event_t event)
{
  float *medata = DetermineAccPitchRoll(event);
  int pitch = median_filter(medata[0]);
  int roll = median_filter(medata[1]);
  
  //See Regular Data
  /*Serial.print("Pitch:: "); Serial.print(medata[0],2); Serial.print("  ");
  Serial.print("Roll: "); Serial.print(medata[1],2); Serial.println("  ");*/

  //See Data Versus Filtered Data
  Serial.print("Pitch:: "); Serial.print(medata[0],2); Serial.print("  "); Serial.print("F_Pitch:: "); Serial.print(pitch); Serial.print("      ");
  Serial.print("Roll: "); Serial.print(medata[1],2); Serial.print("  "); Serial.print("F_Roll:: "); Serial.println(roll);

  //Used for graphing
  /*Serial.print((int)medata[0]); Serial.print(","); Serial.print((int)medata[1]); Serial.print(",");
  Serial.print(pitch);Serial.print(","); Serial.println(roll);*/
}

void PrintMovAvgFiltAcc(sensors_event_t event)
{
  float *movavg = DetermineAccPitchRoll(event);
  int pitch = median_filter(movavg[0]);
  int roll = median_filter(movavg[1]);

  //See Regular Data
  /*Serial.print("Pitch:: "); Serial.print(movavg[0],2); Serial.print("  ");
  Serial.print("Roll: "); Serial.print(movavg[1],2); Serial.println("  ");*/

  //See Data Versus Filtered Data
  /*Serial.print("Pitch:: "); Serial.print(movavg[0],2); Serial.print("  "); Serial.print("F_Pitch:: "); Serial.print(pitch); Serial.print("      ");
  Serial.print("Roll: "); Serial.print(movavg[1],2); Serial.print("  "); Serial.print("F_Roll:: "); Serial.println(roll);*/

  //Used for graphing
  /*Serial.print((int)movavg[0]); Serial.print(","); Serial.print((int)movavg[1]); Serial.print(",");
  Serial.print(pitch);Serial.print(","); Serial.println(roll);*/
}

void loop(void) 
{
  //Get a new ADXL345 event to pull data
  sensors_event_t event;
  accel.getEvent(&event);
  //Read the gyro data
  gyro.read();

  PrintRawGryoandAcc(event);

  //Flex Sensor Output
  /*int flexposition, flex_deg;    // Input value from the analog pin.
  flexposition = analogRead(flexpin); // Read flex sensor
  flex_deg = map(flexposition,500 ,850, 20, 170);
  flex_deg = constrain(flex_deg, 20, 170);
  flexservo.write(flex_deg);*/
  
  delay(15);
}
