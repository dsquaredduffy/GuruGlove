#include<Servo.h>
Servo shoulder_lift_servo;  
Servo shoulder_rotate_servo;
Servo elbow_servo;
Servo wrist_rotation_servo;
Servo wrist_flexion_servo;
Servo finger_servo;

const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter   = '>';
enum { Shoulder_Pitch, Shoulder_Yaw, Elbow_Flexion, Wrist_Roll, Wrist_Flexion, Finger_Flexion };
int whichNumber = Shoulder_Pitch;
int shoulder_pitch, shoulder_yaw,elbow_flexion,wrist_roll,wrist_flexion,finger_flexion;

void setup() {
  Serial.begin(115200);
  Serial.println ("Starting ...");
  shoulder_lift_servo.attach(3);    
  shoulder_rotate_servo.attach(5);   
  elbow_servo.attach(6);  
  wrist_rotation_servo.attach(9); 
  wrist_flexion_servo.attach(10); 
  finger_servo.attach(11);
}

void processNumber (const long degree)
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
      shoulder_rotate_servo.write(shoulder_yaw);
      Serial.print ("Shoulder Yaw = ");
      break;

    case Elbow_Flexion: 
      elbow_flexion = degree;
      whichNumber = Wrist_Roll;
      elbow_servo.write(elbow_flexion);
      Serial.print ("Elbow Flexion Angle = ");
      break;

     case Wrist_Roll: 
      wrist_roll = degree;
      whichNumber = Wrist_Flexion;
      wrist_rotation_servo.write(wrist_roll);
      Serial.print ("Wrist Roll = ");
      break;
      
    case Wrist_Flexion: 
      wrist_flexion = degree;
      whichNumber = Finger_Flexion;
      wrist_flexion_servo.write(wrist_flexion);
      Serial.print ("Wrist Flexion Angle = ");
      break;

    case Finger_Flexion: 
      finger_flexion = degree;
      whichNumber = Shoulder_Pitch;
      finger_servo.write(finger_flexion);
      Serial.print ("Finger Flexion Angle = ");
      break;     
    }
    
  Serial.println (degree);
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
  }
}
 
void loop ()
 {
   if (Serial.available ())
     processInput ();
     
   delay(1);
 }
