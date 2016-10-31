const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter   = '>';

enum { FlexDegree, AccPitch, AccRoll };

int whichNumber = flex_deg;

float flex_deg, acc_pitch, acc_roll;

void setup ()
 { 
 Serial.begin (115200);
 Serial.println ("Starting ...");
 }
 
void processNumber (const long n)
 {
  switch (whichNumber)
    {
    case FlexDegree: 
      flex_deg = x;
      whichNumber = AccPitch;
      Serial.print ("Flex Degree = ");
      break;
      
    case AccPitch: 
      acc_pitch = x;
      whichNumber = AccRoll;
      Serial.print ("Accelerometer Pitch = ");
      break;

    case AccRoll: 
      acc_roll = x;
      whichNumber = FlexDegree;
      Serial.print ("Accelerometer Roll = ");
      break;
    }
    
  Serial.println (n);
 }
 
void processInput ()
 {
 static long receivedNumber = 0;
 static boolean negative = false;
 
 byte c = Serial.read ();
 
 switch (c)
   {
     case endOfNumberDelimiter:  
       if (negative) 
         processNumber (- receivedNumber); 
       else
         processNumber (receivedNumber); 
  
     // fall through to start a new number
     case startOfNumberDelimiter: 
       receivedNumber = 0; 
       negative = false;
       break;
       
     case '0' ... '9': 
       receivedNumber *= 10;
       receivedNumber += c - '0';
       break;
       
     case '-':
       negative = true;
       break;
     
   } // end of switch  
 }  
 
void loop ()
 {
 
 if (Serial.available ())
   processInput ();
      
 } 
