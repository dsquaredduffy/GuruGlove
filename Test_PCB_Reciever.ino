const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter   = '>';

void setup ()
 { 
 Serial.begin (115200);
 Serial.println ("Starting ...");
 }
 
void processNumber (const long n)
 {
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
   
 // do other stuff here
 } 
