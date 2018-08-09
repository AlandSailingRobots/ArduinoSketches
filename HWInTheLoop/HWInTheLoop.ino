

#include <Wire.h>
#include <Canbus.h>
#include <Arduino.h>

#define CHIP_SELECT_PIN 10

int latestReceice2 = -1;
int latestReceive1 = -1;

int heading = 0;
uint8_t compasData[6] = {0, 0, 0, 0, 0, 0};
int hedingReqNr = 0;

int windSpeed = 1000;
int windAngle = 20000;


int serData = 0;
int serId = 0;
int serNr = 0;
int serDataLen = 4;
bool serIsMessageData = false;
int serIds[3] = {0x11, 0x21, 0x22};

uint8_t actuatorWrapper[3] = {0xfb, 0xfa, 0xcb};
int rudderAngle = 0;
int wingsailAngle = 0;


CanbusClass Canbus;


void setup() {
  Wire.begin(0x19);                // join i2c bus with address #x19
  Wire.onRequest(reqEvent);     // register event
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);           // start serial for output
  Serial.println("I2C SLAVE");

   if(Canbus.Init(CHIP_SELECT_PIN)) {
    Serial.println("CAN bus initialized.");
  }

  Serial.println("SETUP COMPLETE");
}


void loop() {
  Serial.print("Heading: ");
  Serial.println (heading);
  Serial.print("Windspeed: ");
  Serial.println (windSpeed);
  Serial.print("Windangle: ");
  Serial.println (windAngle);
  Serial.println("");
  //setHeading(heading);
  //sendWindData();
  delay(100);
}
  

void receiveEvent(int howMany) {
  while (0 < Wire.available()) { // loop through all but the last
                    // receive byte as a int
    latestReceice2 = latestReceive1;
    latestReceive1= Wire.read();
    Serial.println ("HELLO");

  }

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
  compasData[0] = (heading & 0xff00) >> 8;
  compasData[1] = heading& 0xff;
  
}


void serialEvent() {
  int raw;
  while (Serial.available()) {

    if (!serIsMessageData){
        serId = Serial.read();
          if (isSerId(serId)){        
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
          switch (serId){
            case 0x11:
              heading = serData;
              break;
            case 0x21:
              windSpeed = serData;
              break;
            case 0x22:
              windAngle = serData;
              break;
            default:
              break;
          }
           serIsMessageData = false;    
        }
      } 
    }
  }
}

bool isSerId(int id){
  for (int i; i < (sizeof(serIds)/sizeof(int)); i++){
    if (id == serIds[i]){
      return true; 
    }
  }
  return false;
}


bool sendWindData (){

  N2kMsgArd Nmsg;
  CanMsg Cmsg;
  Nmsg.PGN = 130306;
  N2kMsgToId(Nmsg, Cmsg.id);
  Cmsg.header.ide = 1;
  Cmsg.header.length = 8;
  setWindData(Cmsg, windSpeed, windAngle);

  if (!Canbus.SendMessage(&Cmsg)){
    return false;
  }

  Nmsg.PGN = 130311;
  N2kMsgToId(Nmsg, Cmsg.id);
  if (!Canbus.SendMessage(&Cmsg)){
    return false;
  }

  return true;
}

void setWindData(CanMsg &Cmsg, int speed, int direction){
  Cmsg.data[1] = speed & 0xff;
  Cmsg.data[2] = (speed & 0xff00) >> 8;
  Cmsg.data[3] = direction & 0xff;
  Cmsg.data[4] = (direction & 0xff00) >> 8;
}

void writeRuderAngle(){
  Serial.write(actuatorWrapper[0]);
  Serial.write(actuatorWrapper[1]);
  Serial.write(actuatorWrapper[2]);
  Serial.write(rudderAngle);
  Serial.write(wingsailAngle);
}



