#include <ArduinoSTL.h>

/*Purpose: Main Arduino file for current sensor data
 *         Gets requests and reads sensors and sends data on canbus
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
#define PIN_CUR_SENSOR_1 A0 //solar panel current
#define PIN_VOL_SENSOR_1 A2 //solar panel voltage
#define PIN_VOL_SENSOR_2 A3 //battery voltage
#define PIN_CUR_SENSOR_2 A1 //battery current 

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
        sendCurrentSensorData();
    }
}

void sendCurrentSensorData (){
    // Modify this block before flashing the arduino depending
    // on how many current sensors are plugged in

    CanMessageHandler messageHandler(MSG_ID_CURRENT_SENSOR_DATA);
    //CanMessageHandler messageHandlerPU(MSG_ID_CURRENT_SENSOR_DATA_POWER_UNIT);
    //CanMessageHandler messageHandlerB(MSG_ID_CURRENT_SENSOR_DATA_BOX);

    // Create new encodeMessage func? -> trying encodeCSMessage / aborted for now
    Serial.println("########################################");
    Serial.print("Sensor enconding: id = ");
    Serial.print(sensor_id);
    Serial.print(", rol_num = ");
    Serial.println(rolling_number);
    //Serial.print(messageHandler.encodeMessage(CURRENT_SENSOR_CURRENT_DATASIZE, getCurrentValue()));
    //Serial.println(messageHandler.encodeMessage(CURRENT_SENSOR_VOLTAGE_DATASIZE, getVoltageValue()));

    // Encode left byte, header for current sensors
    uint16_t cur_data = getCurrentValue();
    uint16_t vol_data = getVoltageValue();
    uint start = 2;
    uint length = 2;
    
    //Serial.println("Encoding current: ");
    Serial.println(messageHandler.encodeMessage( cur_data, 2, 2)); // default values in bytes
    //Serial.println(messageHandler.getMessageInBitset().to_string<char, std::string::traits_type, std::string::allocator_type>().c_str());
    Serial.println(messageHandler.encodeMessage( vol_data, 0, 2));
    Serial.println(messageHandler.encodeMessage( sensor_id, 7*8 + 5, 3, false)); // false --> values in bits
    Serial.println(messageHandler.encodeMessage( rolling_number, 7*8 + 3, 2, false));
    Serial.println("Bitset To CanMsg call");
    Serial.println(messageHandler.bitsetToCanMsg());

    /*uint8_t a_byte = 0;
    uint8_t *p_byte = &a_byte;
    uint32_t arduino_max_int = 0;
    uint32_t *p_max = &arduino_max_int;
    std::bitset<64> a_bitset(4294967295);
    Serial.println(a_bitset.to_string<char, std::string::traits_type, std::string::allocator_type>().c_str()+32);
    std::bitset<32> another_bitset(static_cast<std::string>(a_bitset.to_string<char, std::string::traits_type, std::string::allocator_type>().c_str()+32));
    Serial.println(another_bitset.to_string<char, std::string::traits_type, std::string::allocator_type>().c_str());
    //another_bitset = static_cast<std::bitset<32>>(a_bitset);
    *p_max = another_bitset.to_ulong();
    Serial.print("*p_max value: ");
    Serial.println(*p_max);
    Serial.print("Going to uint8_t a_byte: ");
    a_byte = static_cast<uint8_t>(*p_max);
    Serial.println(*p_byte);*/
    
    
    //std::bitset<64> test(4294967295);
    //Serial.println("Test value: ");
    //Serial.println(test.to_string<char, std::string::traits_type, std::string::allocator_type>().c_str());
    //int numberof1 = test.count();
    //Serial.println(numberof1);

    /*Serial.print("Bitset encoded: ");
    std::bitset<64> bitset_message = messageHandler.getMessageInBitset();
    std::string str_msg = bitset_message.to_string<char, std::string::traits_type, std::string::allocator_type>(); 
    // compiler is lost there, that's why we have to specify everything in the template
    String arduino_wants_this_one = (String)str_msg.c_str();
    Serial.println(arduino_wants_this_one);*/
    
    sensor_id = (sensor_id+1)%8;           // Uses 3 bits.
    rolling_number = (rolling_number+1)%4; // Uses 2 bits.
    //error_flag = 0;

    



    //Serial.print("Second sensor enconding: ");
    //Serial.print(messageHandlerPU.encodeMessage(CURRENT_SENSOR_CURRENT_DATASIZE, getCurrentValuePU()));
    //Serial.println(messageHandlerPU.encodeMessage(CURRENT_SENSOR_VOLTAGE_DATASIZE, getVoltageValuePU()));

    //Serial.print("Third sensor enconding: ")
    //Serial.print(messageHandlerB.encodeMessage(CURRENT_SENSOR_CURRENT_DATASIZE, getCurrentValue()));
    //Serial.println(messageHandlerB.encodeMessage(CURRENT_SENSOR_VOLTAGE_DATASIZE, getVoltageValue()));

    // Send messages over Canbus
    CanMsg currentSensorData = messageHandler.getMessage();
    Canbus.SendMessage(&currentSensorData);

    //CanMsg currentSensorDataPU = messageHandlerPU.getMessage();
    //Canbus.SendMessage(&currentSensorDataPU);

    //CanMsg currentSensorDataB = messageHandlerB.getMessage();
    //Canbus.SendMessage(&currentSensorDataB);
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

uint16_t getCurrentValue() { //need to add uint8_t sensor variable, to change the pin we read

// Put the analog read and everything here
    float value = analogRead(PIN_CUR_SENSOR_1)*0.0186; 
    Serial.print("solar panel current: ");
    Serial.println(value);
    uint16_t output = fltCompressor.compress(value);
    
    return output;
}

uint16_t getVoltageValue() { //need to add uint8_t sensor variable, to change the pin we read

// Put the analog read and everything here
    float value = analogRead(PIN_VOL_SENSOR_1)*0.0258;
    Serial.print("solar panel voltage: ");
    Serial.println(value);
    uint16_t output = fltCompressor.compress(value);
    
    return output;
}

uint16_t getCurrentValuePU() { //need to add uint8_t sensor variable, to change the pin we read

// Put the analog read and everything here
    float value = analogRead(PIN_CUR_SENSOR_2)*0.0186;
    uint16_t output = fltCompressor.compress(value);
    
    return output;
}

uint16_t getVoltageValuePU() { //need to add uint8_t sensor variable, to change the pin we read

// Put the analog read and everything here
    float value = analogRead(PIN_VOL_SENSOR_2)*0.0258;
    uint16_t output = fltCompressor.compress(value);
    
    return output;
}
