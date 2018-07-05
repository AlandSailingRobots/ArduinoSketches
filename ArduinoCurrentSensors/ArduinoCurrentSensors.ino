#include <ArduinoSTL.h>

/*Purpose: Main Arduino file for current sensor data
 *         Reads sensors and sends data on canbus
 *         Uses CAN-bus to receive and send data.
 *
 *
 */

#include <Wire.h>
#include <SoftwareSerial.h>

#include <Canbus.h>
#include <MsgParsing.h>
#include <CanMessageHandler.h>

#include <string>

// Fill in with current sensors addresses
// Add a case for the new sensor in getCurValue() and getVolValue()
#define NUMBER_OF_SENSORS 2

#define PIN_CUR_SENSOR_0 A0 //solar panel current
#define PIN_VOL_SENSOR_0 A2 //solar panel voltage
#define PIN_VOL_SENSOR_1 A3 //battery voltage
#define PIN_CUR_SENSOR_1 A1 //battery current 

// Don't forget changing this depending on the arduino board
// Mega -> 53, Nano or Uno -> 10
#define CHIP_SELECT_PIN 10


// Do we want response status?
const int RESPONSE_STATUS_NOT_CONNECTED = 0;

// Might be useful later
const int SENSOR_READ_TIME[] = {
        900,    // Time for PH sensor to read
        600,     // Time for Conductivety sensor to read
        600    // Time for Temperature sensor to read
};

const int SENSOR_INPUT_SIZE = 20;
const int SENSOR_READING_TRIES = 5;

CanbusClass Canbus;
unsigned long lastReadingTimeInSeconds = 0;

long int sensorReadingIntervalInSeconds = 1;

// Half precision float converter, IEEE754 standard
Float16Compressor fltCompressor;

// Header variable, occupy 1 byte as follow: id| roll_num | error
unsigned int sensor_id = 0;      // 3 bits
unsigned int rolling_number = 0; // 2 bits
unsigned int error_flag = 0;     // 3 bits

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    if(Canbus.Init(CHIP_SELECT_PIN)) {
        Serial.println("CAN bus initialized.");
    }

    Serial.println("SETUP COMPLETE");

}

void loop()
{
    checkCanbusFor (400);
    handleSensorReadingTimer();
    
}

void handleSensorReadingTimer() {
    unsigned long timeNowInSeconds = millis()/1000;

    if(sensorReadingIntervalInSeconds != -1 && lastReadingTimeInSeconds + sensorReadingIntervalInSeconds < timeNowInSeconds) {
        lastReadingTimeInSeconds = timeNowInSeconds;
        sendCurrentSensorData(sensor_id);
        sensor_id = (sensor_id+1)%NUMBER_OF_SENSORS;
    }
}

void sendCurrentSensorData (unsigned int sensor_id){

    // Init Canbus message
    CanMessageHandler messageHandler(MSG_ID_CURRENT_SENSOR_DATA);

    // Encode current sensor message
    Serial.println("########################################");
    Serial.print("Sensor encoding: id = ");
    Serial.print(sensor_id);
    Serial.print(", rol_num = ");
    Serial.println(rolling_number);

    uint16_t cur_data = getCurrentValue(sensor_id);
    uint16_t vol_data = getVoltageValue(sensor_id);

    Serial.println("Encoding... ");
    messageHandler.encodeMessage( cur_data,       CURRENT_SENSOR_CURRENT_START, CURRENT_SENSOR_CURRENT_DATASIZE, CURRENT_SENSOR_CURRENT_IN_BYTE);
    messageHandler.encodeMessage( vol_data,       CURRENT_SENSOR_VOLTAGE_START, CURRENT_SENSOR_VOLTAGE_DATASIZE, CURRENT_SENSOR_VOLTAGE_IN_BYTE);
    messageHandler.encodeMessage( sensor_id,           CURRENT_SENSOR_ID_START,      CURRENT_SENSOR_ID_DATASIZE,      CURRENT_SENSOR_ID_IN_BYTE);
    messageHandler.encodeMessage( rolling_number, CURRENT_SENSOR_ROL_NUM_START, CURRENT_SENSOR_ROL_NUM_DATASIZE, CURRENT_SENSOR_ROL_NUM_IN_BYTE);
    Serial.println("After encoding data: ");
    Serial.println(messageHandler.getMessageInBitset().to_string<char, std::string::traits_type, std::string::allocator_type>().c_str());

    
    Serial.println("Bitset To CanMsg call");
    Serial.println(messageHandler.bitsetToCanMsg());

    
    uint8_t message[8] = {0,0,0,0,0,0,0,0};
    Serial.print("message.data after bitsetToCanMsg(): ");
    for(int i=0;i<8;i++) {
      message[i] = messageHandler.getMessage().data[i];
      Serial.print(message[i]);
      Serial.print(" | ");
    }
    Serial.println(" ");


    // TO DO: FIX THIS PART AS ITS NOT CODING EVEN NUMBER FOR SOME REASON
    // FIRST FIX : get rid of the mask, problem should be there then...
    //sensor_id = (sensor_id+1)%NUMBER_OF_SENSORS;    // Uses 3 bits.
    rolling_number = (rolling_number+1)%4;          // Uses 2 bits.
    //error_flag = 0;


    // Send messages over Canbus
    CanMsg currentSensorData = messageHandler.getMessage();
    Canbus.SendMessage(&currentSensorData);

}

void checkCanbusFor (int timeMs){
    int startTime= millis();
    int timer = 0;
    while (timer < timeMs){
        if (Canbus.CheckForMessages()) {
            CanMsg msg;
            Canbus.GetMessage(&msg);
        }
        timer = millis() - startTime;
    }
}

uint16_t getCurrentValue(unsigned int sensorID) { 

    int pinToReadCur;
    switch (sensorID) {
      case 0:
          pinToReadCur = PIN_CUR_SENSOR_0;
          break;
      case 1:
          pinToReadCur = PIN_CUR_SENSOR_1;
          break;
      default:
          break;
      }
    float value = analogRead(pinToReadCur)*0.0186; 
    Serial.print("Current reading: ");
    Serial.println(value);
    uint16_t output = fltCompressor.compress(value);
    
    return output;
}

uint16_t getVoltageValue(unsigned int sensorID) { 

    int pinToReadVol;
    switch (sensorID) {
      case 0:
          pinToReadVol = PIN_VOL_SENSOR_0;
          break;
      case 1:
          pinToReadVol = PIN_VOL_SENSOR_1;
          break;
      default:
          break;
      }
    float value = analogRead(pinToReadVol)*0.0258;
    Serial.print("Voltage reading: ");
    Serial.println(value);
    uint16_t output = fltCompressor.compress(value);
    
    return output;
}

