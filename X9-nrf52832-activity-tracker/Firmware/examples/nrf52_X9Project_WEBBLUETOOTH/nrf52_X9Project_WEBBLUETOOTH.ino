/********************************************************************************************************/
/************************ INCLUDES **********************************************************************/
/********************************************************************************************************/
#define NRF52

#include <SPI.h>
#include <BLEPeripheral.h>    //bluetooth
#include <BLEUtil.h>
#include "KX126_SPI.h"        //accelerometer

/********************************************************************************************************/
/************************ CONSTANTS / SYSTEM VAR ********************************************************/
/********************************************************************************************************/

//normal  loop speed is about 35 ms or 28Hz
float   speedLow  = 1000 / 4;  //2Hz 
float   speedMed  = 1000 / 8;  //8Hz 
float   speedHigh = 1000 / 26; //16Hz 

//this is max speed, actual speed may be much slower
float   speedMs = speedHigh;

/********************************************************************************************************/
/************************ DEFINITIONS *******************************************************************/
/********************************************************************************************************/

#define HR_LED_PIN         4
#define HR_DETECTOR        29
#define TOUCH_BUTTON       30
#define VIBRATE_PIN        8
#define BATTERY_PIN        28

#define OLED_CS            15
#define OLED_RES           14
#define OLED_DC            13
#define OLED_SCL           12
#define OLED_SDA           11

//Accelerometer GPIO
#define KX126_CS           24

#define KX126_SDI          19
#define KX126_SDO          20
#define KX126_SCL          18
#define KX126_INT          23

#define PIN_SPI_MISO         (KX126_SDO)
#define PIN_SPI_MOSI         (KX126_SDI)
#define PIN_SPI_SCK          (KX126_SCL)
#define CS_PIN                (KX126_CS)

// Serial dummy NC GPIO - turning on hardware UART Serial fixes some BLE Serial bugs
#define PIN_SERIAL_RX       25 
#define PIN_SERIAL_TX       26

/********************************************************************************************************/
/************************ VARIABLES *********************************************************************/
/********************************************************************************************************/
bool debug = true;
  //Button - raw capacitive touch button value
    int buttonValue;     

  //Heart Rate PPG
    int HRRawVal, HRbpm;

  //Battery Power
    int batteryValue;

  //Timestamp
    float  clocktime = 0;

  //Bluetooth
    int     received_command = 99; //from browser or app
 
  //KX126 Accelerometer
    const int dataReadyPin = 6;
    const int chipSelectPin = 7;
    float     acc[3];
    double    pitch;
    double    roll;

/********************************************************************************************************/
/************************ DECLARATIONS ******************************************************************/
/********************************************************************************************************/

//Accelerometer
KX126_SPI kx126(KX126_CS);

//Bluetooth
// create peripheral instance, see pinouts above
BLEPeripheral blePeripheral = BLEPeripheral();

// create service
BLEService customService =    BLEService("a000");

// create command i/o characteristics
BLECharCharacteristic    ReadOnlyArrayGattCharacteristic  = BLECharCharacteristic("a001", BLERead);
BLECharCharacteristic    WriteOnlyArrayGattCharacteristic = BLECharCharacteristic("a002", BLEWrite);

//create streaming data characteristic
BLECharacteristic        DataCharacteristic("a003", BLERead | BLENotify, 20);  //@param data - an Uint8Array.


/********************************************************************************************************/
/************************ SETUP *************************************************************************/
/********************************************************************************************************/

void setup() 
{
    Serial.begin(115200);
    delay(50);

  /************ INIT KX126 ACCELEROMETER *****************************/
    kx126.init();
    delay(100);

  /************ I/O BUTTON, LED, HAPTIC FEEDBACK *********************/
    //Set HR LED pin 
    pinMode(HR_LED_PIN, OUTPUT);  digitalWrite(HR_LED_PIN, 1);

   //configure haptic feedback pin
    pinMode(VIBRATE_PIN, OUTPUT);  digitalWrite(VIBRATE_PIN, 0);

    /************ CONFIGURE & START BLUETOOTH *********************/
    setupBluetooth();

  delay(500);  
}

/********************************************************************************************************/
/************************ LOOP **************************************************************************/
/********************************************************************************************************/

void loop()
{     
 
 /*************** LOOP SPEED CONTROL **********************************/
if(clocktime + speedMs < millis()){

  /*************** TIMESTAMP *******************************************/
   clocktime = millis();


  /*************** HEART RATE *****************************************/
   HRRawVal = analogRead(HR_DETECTOR);
   //TO DO: calculate BPM based on raw values from photoreceptor

  /*************** BATTERY LEVEL **************************************/
   //Converting raw value to max 100 but this is not a true percentile reflection of battery power, more work 
   //has to be done before raw value can be converted to true measurement of remaining battery power
   batteryValue = ( analogRead(BATTERY_PIN) / 1022) * 100; //max A read is 1023

  /*************** BUTTON MGMT *****************************************/
  //get capacitive touch button value from ADC
  buttonValue = analogRead(TOUCH_BUTTON); //no touch 100-400 , touch 500-1023

  /*************** TEST BUTTON, HR LED AND VIBRATION MOTOR *************/
  //touching button turns on green HR LED for one second - comment out if touch is used over Web Bluetooth
 /* if(buttonValue < 350){
    digitalWrite(HR_LED_PIN, 1);
    delay(1000);
    digitalWrite(HR_LED_PIN, 0);
    delay(1000);
  } */

   /************** READ KX126 ACCELEROMETER *****************************/
   sampleAngularPosition();

   /************** TRANSMIT SENSOR DATA OVER BLUETOOTH ******************/ 
   transmitSensorData();


//end timed loop
 }

//end infinate loop
} 



/********************************************************************************************************/
/************************ FUNCTIONS *********************************************************************/
/********************************************************************************************************/

/*********************************************************************
*************** READ KX126 ACCELEROMETER *****************************
*********************************************************************/
void sampleAngularPosition(){
    //KX022 ACCELEROMETER I2C
    acc[0] = (float)(kx126.getAccel(0));
    acc[1] = (float)(kx126.getAccel(1));
    acc[2] = (float)(kx126.getAccel(2));
    float eulerX, eulerY, eulerZ;
    
    eulerX = acc[0]; eulerY = acc[1]; eulerZ = acc[2]; 
    pitch = (180/3.141592) * ( atan2( eulerX, sqrt( eulerY * eulerY + eulerZ * eulerZ)) );
    roll = (180/3.141592) * ( atan2(-eulerY, -eulerZ) );

    //adjust to minimize gimbal lock issues
    pitch = pitch + 180;
    if(roll < -90 ){
      roll = 450 + roll;
      if(roll > 360){roll = roll - 360;}
    } else { roll = roll + 90; }
}


/*********************************************************************
*************** TRANSMIT SENSOR DATA OVER BLUETOOTH ****************** 
*********************************************************************/
void transmitSensorData(){
    BLECentral central = blePeripheral.central();
    
    if(central){ // if a central is connected to peripheral
              const unsigned char imuCharArray[20] = {
                  (uint8_t)( (acc[0] + 1.00) * 100.00),
                  (uint8_t)( (acc[1] + 1.00) * 100.00),
                  (uint8_t)( (acc[2] + 1.00) * 100.00),  
                  (uint8_t)(roll / 1.41),              //360 --> 256   
                  (uint8_t)(pitch / 1.41),             //360 --> 256
                  (uint8_t)(HRRawVal / 4),      //1023 --> 256
                  (uint8_t)(buttonValue / 4),   //1023 --> 256
                  (uint8_t)(batteryValue),
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,             
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,  
                  (uint8_t)0                   //empty
              }; 
              //send data over bluetooth
              DataCharacteristic.setValue(imuCharArray,20);
              delay(5);
          }
}



/********************************************************************************************************/
/************************ BLUETOOTH BLE FUNCTIONS *************************************************/
/********************************************************************************************************/
void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler

  //do something when connection starts
  delay(5);
}

void blePeripheralDisconnectHandler(BLECentral& central) {

    //do something on disconnect

  delay(5);
}

void blePeripheralServicesDiscoveredHandler(BLECentral& central) {
  // central  services discovered event handler

  delay(5);
}

void bleCharacteristicValueUpdatedHandle(BLECentral& central, BLECharacteristic& characteristic) {
  
    
  const unsigned char* the_buffer = characteristic.value();
  unsigned char the_length = characteristic.valueLength();
  
  String bleRawVal = "";
  for (byte i = 0; i < the_length; i++){ 
    bleRawVal += String(the_buffer[i], HEX); 
  }

  char *char_buf = const_cast<char*>(bleRawVal.c_str());
  
  received_command = (int)strtol(char_buf, NULL, 16);

  BLEUtil::printBuffer(characteristic.value(), characteristic.valueLength());
  delay(5);
}


void setupBluetooth(){
  /************ INIT BLUETOOTH BLE instantiate BLE peripheral *********/
   // set advertised local name and service UUID
    blePeripheral.setLocalName("Tingle");
    blePeripheral.setDeviceName("Tingle");
    blePeripheral.setAdvertisedServiceUuid(customService.uuid());
    blePeripheral.setAppearance(0xFFFF);
  
    // add attributes (services, characteristics, descriptors) to peripheral
    blePeripheral.addAttribute(customService);
    
    blePeripheral.addAttribute(ReadOnlyArrayGattCharacteristic);
    blePeripheral.addAttribute(WriteOnlyArrayGattCharacteristic);
    
    blePeripheral.addAttribute(DataCharacteristic); //streaming data for app graph
    
    // assign event handlers for connected, disconnected to peripheral
    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

    // assign event handlers for characteristic
    ReadOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);
    WriteOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);

    // assign initial values
    char readValue[10] = {0,0,0,0,0,0,0,0,0,0};
    ReadOnlyArrayGattCharacteristic.setValue(0);
    char writeValue[10] = {0,0,0,0,0,0,0,0,0,0};
    WriteOnlyArrayGattCharacteristic.setValue(0);
  
    // begin initialization
    blePeripheral.begin();
  
}

/********************************************************************************************************/
/************************ UTILITY FUNCTIONS *************************************************/
/********************************************************************************************************/
float differenceBetweenAngles(float firstAngle, float secondAngle)
  {
        double difference = secondAngle - firstAngle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
 }
 
int hex_to_int(char c){
  int first;
  int second;
  int value;
  
  if (c >= 97) {
    c -= 32;
  }
  first = c / 16 - 3;
  second = c % 16;
  value = first * 10 + second;
  if (value > 9) {
    value--;
  }
  return value;
}

int hex_to_ascii(char c, char d){
  int high = hex_to_int(c) * 16;
  int low = hex_to_int(d);
  return high+low;
}


