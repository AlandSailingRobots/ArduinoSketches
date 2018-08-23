
int analogPin = 1;     
int value = 0;  
float angle_in_degree = 0;
char dataString[8] = {0};

void setup() {
  Serial.begin(9600);
  Serial.println("SETUP COMPLETE");
}

void loop() {
  value = analogRead(analogPin);
  Serial.print("Value read: ");
  Serial.println(value);
  angle_in_degree = 360.0*value/1024.0;  //analogRead ouput is from 0 to 1023
  Serial.print("Value_in_degree: ");
  Serial.println(angle_in_degree);

  sprintf(dataString,"%02X",angle_in_degree); // convert a value to hexa 
  Serial.println(dataString);   // send the data
  
  delay(500);

}

