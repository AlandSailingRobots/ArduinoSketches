
#include <Wire.h>

int latestReceice2 = -1;
int latestReceive1 = -1;
uint8_t compasData[6] = {0, 0, 0, 0, 0, 0};
int hedingReqNr = 0;


void setup() {
  Wire.begin(0x19);                // join i2c bus with address #0x19
  Wire.onRequest(reqEvent);     // register event
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);           // start serial for output
  Serial.println("I2C SLAVE");
}


void loop() {
  for (int heading = 0; heading < 3600; heading +=10){
    setHeading(heading);
    delay(1000);
  }
  

}
  



void receiveEvent(int howMany) {
  while (0 < Wire.available()) { 
 
    latestReceice2 = latestReceive1;
    latestReceive1= Wire.read();

  }

}

void reqEvent(){
  if (latestReceive1 == 0 and latestReceice2 == 0xE1){ //for init
    Wire.write(0x32);
  }
  else if(latestReceive1 == 0x50){ //for post heading

     
     Wire.write(compasData[hedingReqNr]);
     hedingReqNr ++;
     hedingReqNr = hedingReqNr%6;
      

  }
}

void setHeading(int16_t heading){
  compasData[0] = (heading & 0xff00) >> 8;
  compasData[1] = heading & 0xff;
  
}




