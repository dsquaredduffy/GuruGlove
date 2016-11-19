#include<Servo.h>
Servo shoulder_lift_servo, shoulder_rotate_servo;
Servo wrist_rotation_servo, wrist_flexion_servo;
Servo elbow_servo, finger_servo;

const char startOfDegreeDelimiter = '<';
const char endOfDegreeDelimiter   = '>';

const char startOfNumberDelimiter = '{';
const char endOfNumberDelimiter   = '}';

const char startOfCharacterDelimiter = '[';
const char endOfCharacterDelimiter   = ']';

enum { Shoulder_Pitch, Shoulder_Yaw, Elbow_Flexion, Wrist_Roll, Wrist_Flexion, Finger_Flexion };
int whichNumber = Shoulder_Pitch;
int shoulder_pitch, shoulder_yaw,elbow_flexion,wrist_roll,wrist_flexion,finger_flexion;

const byte numChars = 100;
char receivedChars[numChars]; // an array to store the received data

void setup() {
  Serial.begin(115200);
  Serial.flush();
  Serial.println ("WAITING FOR DATA ...");
  shoulder_lift_servo.attach(11);    
  shoulder_rotate_servo.attach(12);   
  elbow_servo.attach(8);  
  wrist_rotation_servo.attach(9); 
  wrist_flexion_servo.attach(10); 
  finger_servo.attach(7);

}

void processDegrees (const long degree)
 {
  switch (whichNumber)
    {
     case Shoulder_Pitch: 
      shoulder_pitch = degree;
      whichNumber = Shoulder_Yaw;
      shoulder_lift_servo.write(shoulder_pitch);
      Serial.print ("Shoulder Pitch = ");
      break;
      
    case Shoulder_Yaw: 
      shoulder_yaw = degree;
      whichNumber = Elbow_Flexion;
      //shoulder_rotate_servo.write(shoulder_yaw);
      Serial.print ("Shoulder Yaw = ");
      break;

    case Elbow_Flexion: 
      elbow_flexion = degree;
      whichNumber = Wrist_Roll;
      //elbow_servo.write(elbow_flexion);
      Serial.print ("Elbow Flexion Angle = ");
      break;

     case Wrist_Roll: 
      wrist_roll = degree;
      whichNumber = Wrist_Flexion;
      //wrist_rotation_servo.write(wrist_roll);
      Serial.print ("Wrist Roll = ");
      break;
      
    case Wrist_Flexion: 
      wrist_flexion = degree;
      whichNumber = Finger_Flexion;
      //wrist_flexion_servo.write(wrist_flexion);
      Serial.print ("Forward Wrist Motion = ");
      break;

    case Finger_Flexion: 
      finger_flexion = degree;
      whichNumber = Shoulder_Pitch;
      //finger_servo.write(finger_flexion);
      Serial.print ("Finger Flexion Angle = ");
      break; 
    }
    
  Serial.println (degree);
 }

void processNumber (const long n)
{
 Serial.println(n);
}  
 
void processCharacter()
{
  //String myString = String(receivedChars);
  Serial.println(receivedChars);
}

void processInput ()
 {
  static long receivedNumber = 0;
  byte receivedCharacter;
  static boolean negative = false;
  static byte ndx = 0;
   
  byte c = Serial.read ();
  switch (c)
  {
    // Starting new string
    case startOfCharacterDelimiter: 
       memset(receivedChars, 0, sizeof(receivedChars)); // clear the array
    break;

    case 'a' ... 'z' :
     receivedChars[ndx] = c;
     ndx++;
     break;

   case 'A' ... 'Z':
     receivedChars[ndx] = c;
     ndx++;
     break;

   case ' ':
     receivedChars[ndx] = c;
     ndx++;
     break;  

   case ':':
     receivedChars[ndx] = c;
     ndx++;
     break;  

    // Reached the end of the string
    case endOfCharacterDelimiter:  
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      processCharacter();
    break;
      
    // Starting new number
    case startOfNumberDelimiter: 
      receivedNumber = 0; 
      negative = false;
    break;

     // End of number
    case endOfNumberDelimiter:  
      if (negative) 
        processNumber (- receivedNumber); 
      else
        processNumber (receivedNumber);
      receivedNumber = 0;
      break; 
        
    // Starting new degree value
    case startOfDegreeDelimiter: 
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

    // End of degree value
    case endOfDegreeDelimiter:  
      if (negative) 
        processDegrees (- receivedNumber); 
      else
        processDegrees (receivedNumber); 
      receivedNumber = 0;
    break;
  }
 }
 
void loop ()
{
   if (Serial.available ())
      processInput ();
   //Serial.flush();
}
