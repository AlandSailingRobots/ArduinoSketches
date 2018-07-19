// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

int latestReceice2 = -1;
int latestReceive1 = -1;
uint8_t compasData[6] = {0, 0, 0, 0, 0, 0};
int hedingReqNr = 0;
int serData = 0;
int serId = 0;
int serNr = 0;
int serDataLen = 4;
bool serIsMessageData = false;
int serIds[1] = {0x41};

void setup() {
  Wire.begin(0x19);                // join i2c bus with address #8
  Wire.onRequest(reqEvent);     // register event
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);           // start serial for output
  Serial.println("I2C SLAVE");
}


void loop() {
  setHeading(serData);
  delay(10);
}
  


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (0 < Wire.available()) { // loop through all but the last
                    // receive byte as a int
    latestReceice2 = latestReceive1;
    latestReceive1= Wire.read();
    //Serial.print(latestReceive1);
    //Serial.print ("    ");
    //Serial.println(latestReceice2);
  }
  //int x = Wire.read();    // receive byte as an integer
  //Serial.println(x);         // print the integer
}

void reqEvent(){
  if (latestReceive1 == 0 and latestReceice2 == 0xE1){
    Wire.write(0x32);
  }
  else if(latestReceive1 == 0x50){

     
     Wire.write(compasData[hedingReqNr]);
     hedingReqNr ++;
     hedingReqNr = hedingReqNr%6;
      
    //}
    //Serial.println("POST HEADING");
  }
}

void setHeading(int16_t heading){
  compasData[0] = (heading & (0xff << 8)) >> 8;
  compasData[1] = heading& 0xff;
  
}


void serialEvent() {
  int raw;
  while (Serial.available()) {

    if (!serIsMessageData){
        serId = Serial.read();
          if (serId == 0x41){        
            serIsMessageData = true;
          }
        
    } else {
      if (serNr == 0){
        serData = 0;
        serData =Serial.read();
        serNr ++;
        serNr = serNr%serDataLen; 
      } else {
        raw =   Serial.read();
        serData = (serData << 8) + raw;
        serNr ++;
        serNr = serNr%serDataLen;
        if (serNr == 0){
           serIsMessageData = false;    
        }
      } 
    }
  }
}


