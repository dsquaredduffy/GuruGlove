static int *flex_data;
const int flex_pin = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  flex_data = Calibrate_Flex(flex_pin);
  PrintMinMaxFlex(flex_data);
}

int *Calibrate_Flex(int flexpin){
  Serial.print ("Calibration Began");

  static int simple_flex_data[2];
  int flex_value;
  
  //Take in a 800 readings and take the min and max values of the readings
  //Will take 8 seconds to complete
  for(int i=0; i<800; i++)
  {
     Serial.print("Inside --> loop number ");
     Serial.println(i);
     flex_value = analogRead(flexpin); // Read flex sensor
     simple_flex_data[0] = min(simple_flex_data[0], flex_value);
     simple_flex_data[1] = max(simple_flex_data[1], flex_value);
     //Due to all initial values being zero, we take in the minimum right above zero
     if(simple_flex_data[0] == 0)
        simple_flex_data[0] = flex_value;
//     if(simple_flex_data[1] == 1023 && flex_value != simple_flex_data[0])
//        simple_flex_data[1] = flex_value;
     delay(10);
  }
  Serial.println("------------------------------------");  
  Serial.print ("Minimum Value: ");
  Serial.print (simple_flex_data[0]);
  Serial.print ("        ");     
  Serial.print ("Maximum Value: ");      
  Serial.println (simple_flex_data[1]); 
  Serial.println ("Calibration Ended");
  Serial.println("------------------------------------");

  return simple_flex_data;  
}

void PrintMinMaxFlex(int *data){

  Serial.println("------------------------------------");
  Serial.print(" Minimum Value: ");
  Serial.print(data[0]);    
  Serial.print("        ");
  Serial.print(" Maximum Value: ");
  Serial.println(data[1]);
  Serial.println("------------------------------------");
}

void loop() {
  // put your main code here, to run repeatedly:
  int flex_value = analogRead(flex_pin); 
  int flex_deg = map(flex_value,flex_data[1],flex_data[0], 90, 0);
  Serial.print("Analog Value: ");
  Serial.print(flex_value);
  Serial.print("      ");
  Serial.print("Degree Value: ");
  Serial.println(flex_deg);
}
