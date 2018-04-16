
/********************************************************************************************************/
/************************ INCLUDE ***********************************************************************/
/********************************************************************************************************/
#define NRF52

#include <stdio.h>
#include <stdint.h>
#include <SPI.h>
#include <BLEPeripheral.h>    //bluetooth
#include <BLEUtil.h>
#include "BLESerial.h"
#include <KX126_SPI.h>        //accelerometer

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

//BLESerial
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9

// Serial dummy NC GPIO
#define PIN_SERIAL_RX       25 
#define PIN_SERIAL_TX       26

/********************************************************************************************************/
/************************ VARIABLES *********************************************************************/
/********************************************************************************************************/

  //Button - raw capacitive touch button value
    int buttonValue;     

  //Heart Rate PPG
    int HRRawVal, HRbpm;

  //Battery Power
    int batteryValue;

  //Timestamp
    float  clocktime = 0;

  //Bluetooth
    unsigned long millisPerTransmit; 
    unsigned long millisPrevious = 0;
 
  //KX126 Accelerometer
    const int dataReadyPin = 6;
    const int chipSelectPin = 7;
    float     acc[3];
    double    pitch;
    double    roll;

/********************************************************************************************************/
/************************ DECLARATIONS ******************************************************************/
/********************************************************************************************************/
//KX126 Accelerometer
KX126_SPI kx126(KX126_CS);

//Bluetooth - create peripheral instance, see pinout definitions above
BLESerial bleSerial(BLE_REQ, BLE_RDY, BLE_RST);

//Permanent storage on nRF52832 FLASH
BLEBondStore bleBondStore;


/********************************************************************************************************/
/************************ SETUP *************************************************************************/
/********************************************************************************************************/

void setup() 
{
    // custom services and characteristics can be added as well
    bleSerial.setLocalName("X9nRF52832");

    //Bluetooth UART
    bleSerial.begin();
    
    // acceptable values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
    int power = -4;
    sd_ble_gap_tx_power_set(power);

    //SET BLE SERIAL TRANSMIT FREQUENCY
    unsigned long millisPerTransmit = 1000; // 1HZ
    
    delay(50);

    //Hardware UART - BLESerial is more stable with HW UART enabled but it does consume power
    Serial.begin(115200); 
  //  Serial.println("SETUP");

  /************ INIT KX126 ACCELEROMETER *****************************/
    kx126.init();
    delay(100);

  /************ I/O BUTTON, LED, HAPTIC FEEDBACK *********************/
    //Set HR LED pin high/off 
    pinMode(HR_LED_PIN, OUTPUT);  digitalWrite(HR_LED_PIN, 1);

   //configure haptic feedback pin
    pinMode(VIBRATE_PIN, OUTPUT);  digitalWrite(VIBRATE_PIN, 0);

  delay(500);  
}

/********************************************************************************************************/
/************************ LOOP **************************************************************************/
/********************************************************************************************************/

void loop()
{     
  //BLUETOOTH SERIAL
  bleSerial.poll();
  delay(50);

  /*************** LOOP SPEED CONTROL **********************************/
if(clocktime + speedMs < millis()){

  /*************** TIMESTAMP *******************************************/
   clocktime = millis();
  // Serial.println(clocktime);

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
  //touching button turns on green HR LED and vibration motor for one second
  if(buttonValue < 350){
    digitalWrite(VIBRATE_PIN, 1);
    digitalWrite(HR_LED_PIN, 1);
    delay(1000);
    digitalWrite(VIBRATE_PIN, 0);
    digitalWrite(HR_LED_PIN, 0);
    delay(1000);
  }

   /************** READ KX126 ACCELEROMETER *****************************/
   sampleAngularPosition();

   /************** TRANSMIT SENSOR DATA OVER BLUETOOTH ******************/ 
    unsigned long millisNow;
    millisNow = millis(); 
    //check to see if enough time has elapsed
    if(millisNow - millisPrevious >= millisPerTransmit){
      // increment previous time, so we keep proper pace
      millisPrevious = millisPrevious + millisPerTransmit;
      
      transmitSensorData();
    }
 } //end timed loop

} //end infinate loop

/********************************************************************************************************/
/************************ FUNCTIONS *********************************************************************/
/********************************************************************************************************/

/*********************************************************************
*************** READ KX126 ACCELEROMETER *****************************
*********************************************************************/
void sampleAngularPosition(){
    //KX126 ACCELEROMETER I2C
    acc[0] = (float)(kx126.getAccel(0) );
    acc[1] = (float)(kx126.getAccel(1) );
    acc[2] = (float)(kx126.getAccel(2) );
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
    if (bleSerial) {
      //use itoa to convert float->char and sprintf to convert int->char

      char buffPitch[8], buffPitch2[4];
      itoa(pitch, buffPitch, 10);
      strcat(buffPitch, ".");                   //append decimal point
      uint16_t i = (pitch - (int)pitch) * 100;  //subtract to get the decimals, and multiply by 100
      itoa(i, buffPitch2, 10);                  //convert to a second string
      strcat(buffPitch, buffPitch2);            //and append to the first

      char buffRoll[8], buffRoll2[4];
      itoa(roll, buffRoll, 10);
      strcat(buffRoll, ".");                    //append decimal point
      uint16_t j = (roll - (int)roll) * 100;    //subtract to get the decimals, and multiply by 100
      itoa(j, buffRoll2, 10);                   //convert to a second string
      strcat(buffRoll, buffRoll2);              //and append to the first
    //   bleSerial.println(buffRoll);

       char buffButton[16] = "";
       snprintf(buffButton, sizeof(buffButton), "%s %d", "bt:", buttonValue);
    //   bleSerial.println(buffButton);

       char buffHeart[16] = "";
       snprintf(buffHeart, sizeof(buffHeart), "%s %d", "hr:", HRRawVal);
     //  bleSerial.println(buffHeart);

       char buffBattery[16] = "";
       snprintf(buffBattery, sizeof(buffBattery), "%s %d", "ba:", batteryValue);
     //  bleSerial.println(buffBattery);

       char buffCombined[88];
       strcpy(buffCombined, buffPitch);
       strcat(buffCombined, " ");
       strcat(buffCombined, buffRoll);
       strcat(buffCombined, " ");
       strcat(buffCombined, buffButton);
       strcat(buffCombined, " ");
       strcat(buffCombined, buffHeart);
       strcat(buffCombined, " ");
       strcat(buffCombined, buffBattery);

       //send the combined char array
       bleSerial.println(buffCombined);
    }
}

/*********************************************************************
*************** ETC **************************************************
*********************************************************************/

// echo all received data back
void loopback() {
  if (bleSerial) {
    int byte;
    while ((byte = bleSerial.read()) > 0) bleSerial.write(byte);
  }
}

// periodically sent time stamps
void spam() {
  if (bleSerial) {
    bleSerial.print(millis());
    bleSerial.println(" Salamander!");
  }
}






