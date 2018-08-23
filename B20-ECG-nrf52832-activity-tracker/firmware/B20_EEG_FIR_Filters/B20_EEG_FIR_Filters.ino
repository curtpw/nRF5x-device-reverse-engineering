/*
---------------------------------------------------------------------------------
ADS1292 for Arduino Code

ADS1292 code Inspired by:
https://github.com/ericherman/eeg-mouse
http://www.mccauslandcenter.sc.edu/CRNL/ads1298

FIR Filters by Leeman Geophysical:
https://github.com/LeemanGeophysicalLLC/FIR_Filter_Arduino_Library

FFT for ARM Cortex by Adafruit:
https://github.com/adafruit/Adafruit_ZeroFFT

nRF5x core and Bluetooth comms by Sandeep Mistry:
https://github.com/sandeepmistry/arduino-nRF5
https://github.com/sandeepmistry/arduino-BLEPeripheral

Useful information about using FIR with the ADS1292 for EEG by bois083:
https://bois083.wordpress.com

---------------------------------------------------------------------------------
-----------------------------------------------------------
| Module | Signal         |   B20 nRF52 Pin               |
|--------+----------------+-------------------------------+
|        |  CS            |  15                           |
|  SPI   |  MOSI (DOUT)   |  18                           |    
|        |  MISO (DIN)    |  16                           |     
|        |  SCK           |  17                           |  
|--------+----------------+-------------------------------+
|        |  START         |  20                           | 
|  ADS   |  RESET         |  22                           |
|        |  DRDY          |  wired 3.3v                   | 
-----------------------------------------------------------



SCLK   |  Device attempts to decode and execute commands every eight serial clocks. 
       |  It is recommended that multiples of 8 SCLKs be presented every serial transfer to keep the interface in a normal operating mode.
       |
DRDY   |  hard wired high so no way of knowing if there is new data. Get data from the ADS1292 at a rate which is lower than its sample rate
       |
START  |  keep low if using start opcode. The START pin or the START command is used to place the device either in normal data capture mode or pulse data capture mode.
       |
PWDN   |  When PWDN is pulled low, all on-chip circuitry is powered down
       |
CS     |  low = SPI is active, must remain low during communication, then wait 4-5 tCLK cycles before returning to high, set CS high then low to reset the communication
       |  connect to J3.1 if JP21 = <**o], or J3.7 if JP21 = <o**]
       |
RESET  |  low = force a reset, RESET is automatically issued to the digital filter whenever registers CONFIG1 and RESP are set to new values with a WREG command


Normal EEG Waveforms
https://emedicine.medscape.com/article/1139332-overview
Roy Sucholeiki, MD Director, Comprehensive Seizure and Epilepsy Program, The Neurosciences Institute at Central DuPage Hospital
Delta waves - 3 Hz or less
Theta waves - 3.5-7.5 Hz
Alpha waves - 8-13 Hz
Beta waves - Greater than 13 Hz
EMG signal - generally peaks at 200 Hz

EMG differes greately based on body location and electrode placement 

alpha and theta are highly correlated with attention
*/

#define NRF52

/********************************************************************************************************/
/************************ INCLUDES **********************************************************************/
/********************************************************************************************************/
#include <SPI.h>
#include <BLEPeripheral.h>    //bluetooth
#include <BLEUtil.h>
#include "FIR.h"

//utilities
#include <math.h>
#include <stdarg.h>

/********************************************************************************************************/
/************************ CONSTANTS / SYSTEM VAR ********************************************************/
/********************************************************************************************************/
bool    debug = false;                //turn serial on/off to get data or turn up sample rate
bool    debug_time = false;           //turn loop component time debug on/off
boolean debug_msg     = false;         //debug msg
bool    IS_CONNECTED = false;


/********************************************************************************************************/
/************************ VARIABLES *********************************************************************/
/********************************************************************************************************/

//the sample rate
#define FS 80    

//BLE send runs over sample time so we shorten next sample time to keep things more or less in order. Super hacky but what ya gunna do. 
float bluetoothTimeOverflow = 0;
bool  timeCorrection = false;

float deltaWave = 1, thetaWave = 1, alphaWave = 1, betaWave = 1, EMGWave = 1, deltaWave2 = 1, thetaWave2 = 1, alphaWave2 = 1, betaWave2 = 1, EMGWave2 = 1;

float deltaWaveOld[10] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
float thetaWaveOld[10] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
float alphaWaveOld[10] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
float betaWaveOld[10] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
float EMGWaveOld[10] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

//data at point between BLE sends so we only have to send over BLE half as much
float deltaWaveAncient[10];// = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100}; 
float thetaWaveAncient[10];// = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100}; 
float alphaWaveAncient[10];// = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100}; 
float betaWaveAncient[10];// = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100}; 
float EMGWaveAncient[10];// = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100}; 

float transmitFloatArray[10]= {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

unsigned char transmitCharArray[20] = {
    (uint8_t)(0), 
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0), 
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),
    (uint8_t)(0),                 //empty
}; 

FIR<float, 25> fir_filter_deltawave; 
FIR<float, 31> fir_filter_thetawave; 
FIR<float, 25> fir_filter_alphawave; 
FIR<float, 19> fir_filter_betawave; 
FIR<float, 17> fir_filter_emgwave; 

float correctSampleTime;
int trackSamples = 0;
bool samplesReadyFlag = false;

//***Bluetooth variables
unsigned long microsPerReading, microsPrevious;
int     command_value = 99; //controlls how device and app talk to each other

//***ADS1292 variables
String sep = "---------------------------------------------------------";
int n_channels        = 0;
int blank             = 0;

// SPI
#define DOUT       23 //MOSI
#define DIN        16 //MISO
#define SPICK      17 //SCK

// pins
#define PIN_CS     15
#define PIN_RESET  22
#define PIN_START  20
#define PIN_DRDY    9 //(wired high, sub NC dummy pin)


// register commands
#define READ  0x20
#define WRITE  0x40

// other
int gMaxChan = 2;
int gNumActiveChan = 2;
int activeSerialPort = 0; //data will be sent to serial port that last sent commands. E.G. bluetooth or USB port
boolean gActiveChan [2]; // reports whether channels 1..2 are active
boolean isRDATAC = true;

enum spi_command {
  // system commands
  WAKEUP =    0x02,
  STANDBY =   0x04,
  RESET =     0x06,
  START =     0x08,
  STOP =      0x0a,

  // read commands
  RDATAC =    0x10,
  SDATAC =    0x11,
  RDATA =     0x12,

  // register commands
  RREG =      0x20,
  WREG =      0x40
};

enum reg {
  // device settings
  ID =        0x00,

  // global settings
  CONFIG1 =   0x01,
  CONFIG2 =   0x02,
  LOFF =      0x03,

  // channel specific settings
  CHnSET =    0X03,
  CH1SET = CHnSET + 1,
  CH2SET = CHnSET + 2,

  // lead off status
  RLD_SENS =  0x06,
  LOFF_SENS = 0x07,
  LOFF_STAT = 0x08,

  // other
  GPIO =      0x0B,
  RESP1 =     0x09,
  RESP2 =     0x0A

};

enum ID_bits {
  DEV_ID7 = 0x80,
  DEV_ID6 = 0x40,
  DEV_ID5 = 0x20,
  DEV_ID2 = 0x04,
  DEV_ID1 = 0x02,
  DEV_ID0 = 0x01,

  ID_const = 0x10,
  ID_ADS129x = DEV_ID7,
  ID_ADS129xR = (DEV_ID7 | DEV_ID6),

  ID_4CHAN = 0,
  ID_6CHAN = DEV_ID0,
  ID_8CHAN = DEV_ID1,

  ID_ADS1294 = (ID_ADS129x | ID_4CHAN),
  ID_ADS1296 = (ID_ADS129x | ID_6CHAN),
  ID_ADS1298 = (ID_ADS129x | ID_8CHAN),
  ID_ADS1294R = (ID_ADS129xR | ID_4CHAN),
  ID_ADS1296R = (ID_ADS129xR | ID_6CHAN),
  ID_ADS1298R = (ID_ADS129xR | ID_8CHAN)
};

enum CONFIG1_bits {
  SINGLE_SHOT = 0x80,
  DR2 = 0x04,
  DR1 = 0x02,
  DR0 = 0x01,

  CONFIG1_const = 0x00,
   SAMP_125_SPS = CONFIG1_const,
   SAMP_250_SPS = (CONFIG1_const | DR0),
   SAMP_500_SPS = (CONFIG1_const | DR1),
   SAMP_1_KSPS = (CONFIG1_const | DR1 | DR0),
   SAMP_2_KSPS = (CONFIG1_const | DR2), 
   SAMP_4_KSPS = (CONFIG1_const | DR2 | DR0),
   SAMP_8_KSPS = (CONFIG1_const | DR2 | DR1)
  
};

enum CONFIG2_bits {
  PDB_LOFF_COMP = 0x40,
   PDB_REFBUF = 0X20,
   VREF_4V = 0x10,
   CLK_EN = 0X08,
  INT_TEST = 0x02, //amplitude = Â±(VREFP â€“ VREFN) / 2400
  TEST_FREQ = 0x01,

  CONFIG2_const = 0x80,
  INT_TEST_1HZ = (CONFIG2_const | INT_TEST | TEST_FREQ), 
  INT_TEST_DC = (CONFIG2_const | INT_TEST)
};

enum LOFF_bits {
  COMP_TH2 = 0x80,
  COMP_TH1 = 0x40,
  COMP_TH0 = 0x20,
  VLEAD_OFF_EN = 0x10,
  ILEAD_OFF1 = 0x08,
  ILEAD_OFF0 = 0x04,
  FLEAD_OFF1 = 0x02,
  FLEAD_OFF0 = 0x01,

  LOFF_const = 0x00,

  COMP_TH_95 = 0x00,
  COMP_TH_92_5 = COMP_TH0,
  COMP_TH_90 = COMP_TH1,
  COMP_TH_87_5 = (COMP_TH1 | COMP_TH0),
  COMP_TH_85 = COMP_TH2,
  COMP_TH_80 = (COMP_TH2 | COMP_TH0),
  COMP_TH_75 = (COMP_TH2 | COMP_TH1),
  COMP_TH_70 = (COMP_TH2 | COMP_TH1 | COMP_TH0),

  ILEAD_OFF_6nA = 0x00,
  ILEAD_OFF_12nA = ILEAD_OFF0,
  ILEAD_OFF_18nA = ILEAD_OFF1,
  ILEAD_OFF_24nA = (ILEAD_OFF1 | ILEAD_OFF0),

  FLEAD_OFF_AC = FLEAD_OFF0,
  FLEAD_OFF_DC = (FLEAD_OFF1 | FLEAD_OFF0)
};

enum CHnSET_bits {
  PDn =        0x80,
  PD_n =       0x80,
  GAINn2 =     0x40,
  GAINn1 =     0x20,
  GAINn0 =     0x10,
  MUXn2 =      0x04,
  MUXn1 =      0x02,
  MUXn0 =      0x01,

  CHnSET_const =  0x00,

  GAIN_1X = GAINn0,
  GAIN_2X = GAINn1,
  GAIN_3X = (GAINn1 | GAINn0),
  GAIN_4X = GAINn2,
  GAIN_6X = 0x00,
  GAIN_8X = (GAINn2 | GAINn0),
  GAIN_12X = (GAINn2 | GAINn1),

  ELECTRODE_INPUT = 0x00,
  SHORTED = MUXn0,
  RLD_INPUT = MUXn1,
  MVDD = (MUXn1 | MUXn0),
  TEMP = MUXn2,
  TEST_SIGNAL = (MUXn2 | MUXn0),
  RLD_DRP = (MUXn2 | MUXn1),
  RLD_DRN = (MUXn2 | MUXn1 | MUXn0)
};

enum CH1SET_bits {
  PD_1 = 0x80,
  GAIN12 = 0x40,
  GAIN11 = 0x20,
  GAIN10 = 0x10,
  MUX12 = 0x04,
  MUX11 = 0x02,
  MUX10 = 0x01,

  CH1SET_const = 0x00
};

enum CH2SET_bits {
  PD_2 = 0x80,
  GAIN22 = 0x40,
  GAIN21 = 0x20,
  GAIN20 = 0x10,
  MUX22 = 0x04,
  MUX21 = 0x02,
  MUX20 = 0x01,

  CH2SET_const = 0x00
};

enum RLD_SENSP_bits {
  RLD8P = 0x80,
  RLD7P = 0x40,
  RLD6P = 0x20,
  RLD5P = 0x10,
  RLD4P = 0x08,
  RLD3P = 0x04,
  RLD2P = 0x02,
  RLD1P = 0x01,

  RLD_SENSP_const = 0x00
};

enum RLD_SENSN_bits {
  RLD8N = 0x80,
  RLD7N = 0x40,
  RLD6N = 0x20,
  RLD5N = 0x10,
  RLD4N = 0x08,
  RLD3N = 0x04,
  RLD2N = 0x02,
  RLD1N = 0x01,

  RLD_SENSN_const = 0x00
};

enum LOFF_SENSP_bits {
  LOFF8P = 0x80,
  LOFF7P = 0x40,
  LOFF6P = 0x20,
  LOFF5P = 0x10,
  LOFF4P = 0x08,
  LOFF3P = 0x04,
  LOFF2P = 0x02,
  LOFF1P = 0x01,

  LOFF_SENSP_const = 0x00
};

enum LOFF_SENSN_bits {
  LOFF8N = 0x80,
  LOFF7N = 0x40,
  LOFF6N = 0x20,
  LOFF5N = 0x10,
  LOFF4N = 0x08,
  LOFF3N = 0x04,
  LOFF2N = 0x02,
  LOFF1N = 0x01,

  LOFF_SENSN_const = 0x00
};

enum LOFF_FLIP_bits {
  LOFF_FLIP8 = 0x80,
  LOFF_FLIP7 = 0x40,
  LOFF_FLIP6 = 0x20,
  LOFF_FLIP5 = 0x10,
  LOFF_FLIP4 = 0x08,
  LOFF_FLIP3 = 0x04,
  LOFF_FLIP2 = 0x02,
  LOFF_FLIP1 = 0x01,

  LOFF_FLIP_const = 0x00
};

enum LOFF_STATP_bits {
  IN8P_OFF = 0x80,
  IN7P_OFF = 0x40,
  IN6P_OFF = 0x20,
  IN5P_OFF = 0x10,
  IN4P_OFF = 0x08,
  IN3P_OFF = 0x04,
  IN2P_OFF = 0x02,
  IN1P_OFF = 0x01,

  LOFF_STATP_const = 0x00
};

enum LOFF_STATN_bits {
  IN8N_OFF = 0x80,
  IN7N_OFF = 0x40,
  IN6N_OFF = 0x20,
  IN5N_OFF = 0x10,
  IN4N_OFF = 0x08,
  IN3N_OFF = 0x04,
  IN2N_OFF = 0x02,
  IN1N_OFF = 0x01,

  LOFF_STATN_const = 0x00
};

enum GPIO_bits {
  GPIOC2 = 0x08,
  GPIOC1 = 0x04,
  GPIOD2 = 0x02,
  GPIOD1 = 0x01,
  GPIO_const = 0x00
};

enum RESP_bits {
  RESP_DEMOD_EN1 = 0x80,
  RESP_MOD_EN1 = 0x40,
  RESP_PH2 = 0x10,
  RESP_PH1 = 0x08,
  RESP_PH0 = 0x04,
  RESP_CTRL1 = 0x02,
  RESP_CTRL0 = 0x01,

  RESP_const = 0x20,

  RESP_PH_22_5 = 0x00,
  RESP_PH_45 = RESP_PH0,
  RESP_PH_67_5 = RESP_PH1,
  RESP_PH_90 = (RESP_PH1 | RESP_PH0),
  RESP_PH_112_5 = RESP_PH2,
  RESP_PH_135 = (RESP_PH2 | RESP_PH0),
  RESP_PH_157_5 = (RESP_PH2 | RESP_PH1),

  RESP_NONE = 0x00,
  RESP_EXT = RESP_CTRL0,
  RESP_INT_SIG_INT = RESP_CTRL1,
  RESP_INT_SIG_EXT = (RESP_CTRL1 | RESP_CTRL0)
};

// serial api
boolean read_ads_data     = false;
boolean serial_send_data  = false;
boolean valid_cmd         = false;
enum SERIAL_API {
  S_READ_ADS = 0x67,//      = 0xBF,   // read from the ads
  S_STOP_READ_ADS = 0x73, //  = 0xFD,   // stop reading freom the ads
  S_SEND_SERIAL     = 0xDF,   // start sending to the serial interface
  S_STOP_SEND_SERIAL  = 0xFB    // stop streaming to the serial interface
};


/********************************************************************************************************/
/************************ DECLARATIONS ******************************************************************/
/********************************************************************************************************/
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
/************************ BLUETOOTH BLE FUNCTIONS *******************************************************/
/********************************************************************************************************/
void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler

  //CONNECTED
  IS_CONNECTED = true;
  
  if(debug){
    Serial.print(F("Connected event, central: "));
    Serial.println(central.address());
  }
  delay(5);
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler

    //NOT CONNECTED
    IS_CONNECTED = false;
  
    if(debug){
        Serial.print(F("Disconnected event, central: "));
        Serial.println(central.address());
    }
  delay(5);
}

void blePeripheralServicesDiscoveredHandler(BLECentral& central) {
  // central  services discovered event handler
  if(debug){
    Serial.print(F(" services discovered event, central: "));
    Serial.println(central.address());
  }

  delay(5);
  //delay(2000);
}

void bleCharacteristicValueUpdatedHandle(BLECentral& central, BLECharacteristic& characteristic) {
  
    if(debug){ Serial.print(F(" Begin bleCharacteristicValueUpdatedHandle: ")); }
    
  const unsigned char* the_buffer = characteristic.value();
  unsigned char the_length = characteristic.valueLength();
 // char char_buf[2]={0,0};
  //int command_value;
  
  String bleRawVal = "";
  for (byte i = 0; i < the_length; i++){ 
    bleRawVal += String(the_buffer[i], HEX); 
  }

  char *char_buf = const_cast<char*>(bleRawVal.c_str());
  command_value = (int)strtol(char_buf, NULL, 16);
  if(debug) Serial.print("APP COMMAND: "); Serial.println( command_value );

  BLEUtil::printBuffer(characteristic.value(), characteristic.valueLength());
 // if(debug) delay(1000);
  delay(5);
}

void switchCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  if(debug) Serial.print(F("Characteristic event, written: "));

  if (ReadOnlyArrayGattCharacteristic.value()) {
    if(debug) Serial.println(F("LED on"));
 //   digitalWrite(LED_PIN, HIGH);
  } else {
    if(debug) Serial.println(F("LED off"));
 //   digitalWrite(LED_PIN, LOW);
  }
}

void setupBluetooth(){
  /************ INIT BLUETOOTH BLE instantiate BLE peripheral *********/
   // set advertised local name and service UUID
    blePeripheral.setLocalName("EEGlasses");
    blePeripheral.setDeviceName("EEGlasses");
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
  //  blePeripheral.setEventHandler(BLEWritten, blePeripheralServicesDiscoveredHandler);

    // assign event handlers for characteristic
    ReadOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);
    WriteOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);

    // assign initial values
    char readValue[10] = {0,0,0,0,0,0,0,0,0,0};
    ReadOnlyArrayGattCharacteristic.setValue(0);
    char writeValue[10] = {0,0,0,0,0,0,0,0,0,0};
    WriteOnlyArrayGattCharacteristic.setValue(0);

    // initialize variables to pace updates to correct rate
    microsPerReading = 1000000 / 25;
    microsPrevious = micros();
  
    // begin initialization
    blePeripheral.begin();
  
    if(debug) Serial.println("BLE MOBILE APP PERIPHERAL");
}


/********************************************************************************************************/
/************************ ADS1292 FUNCTIONS *************************************************************/
/********************************************************************************************************/

String hex_to_char(int hex_in) {
  int precision = 2;
  char tmp[16];
  char format[128];
  sprintf(format, "0x%%.%dX", precision);
  sprintf(tmp, format, hex_in);
  //Serial.print(tmp);
  return(String(tmp));
}

void write_byte(int reg_addr, int val_hex) {
  //see pages 40,43 of datasheet - 
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(5);
  SPI.transfer(0x40 | reg_addr);
  delayMicroseconds(5);
  SPI.transfer(0x00); // number of registers to be read/written â€“ 1
  delayMicroseconds(5);
  SPI.transfer(val_hex);
  delayMicroseconds(10);
  digitalWrite(PIN_CS, HIGH);
  
  if(debug_msg){
    Serial.println(sep);
    Serial.print( "sent:\t" + hex_to_char(reg_addr) + "\t" + hex_to_char(val_hex) + "\t" );
    Serial.println(val_hex, BIN);
  }
}

int read_byte(int reg_addr){
  int out = 0;
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x20 | reg_addr);
  delayMicroseconds(5);
  SPI.transfer(0x00); // number of registers to be read/written â€“ 1
  delayMicroseconds(5);
  out = SPI.transfer(0x00);
        delayMicroseconds(1);
  digitalWrite(PIN_CS, HIGH);

  if(debug_msg){
    Serial.println(sep);
    Serial.println( "sent:\t" + hex_to_char(reg_addr) );
    Serial.println( "recieved:\t" + hex_to_char(out) );
  }
  
  return(out);
}

void send_command(uint8_t cmd) {
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(5); // 5 = 6us at 32
  SPI.transfer(cmd);
  delayMicroseconds(10);
  digitalWrite(PIN_CS, HIGH);
}

// initialization

void init_pins(){
  pinMode(SPICK, OUTPUT);
  pinMode(DIN, INPUT); 
  pinMode(DOUT, OUTPUT);
  pinMode(PIN_LED,   OUTPUT);
  pinMode(PIN_CS,    OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_START, OUTPUT);
  pinMode(PIN_DRDY, INPUT);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_START, LOW);
  delay(1);
}

void init_serial(){
  Serial.begin(115200);
  Serial.flush();
  delayMicroseconds(100);
}

void init_spi(){
  
  // initializes the SPI bus by setting SCK and MOSI low
  SPI.begin();
  
  // spi data mode
  // sets clock polarity and phase
  // CPOL = 0 (clock polarity, clock is idle when low)
  // CPHA = 1 (clock phase , data is shifted in and out on the rising of the data clock signal )
  SPI.setDataMode(SPI_MODE1);
  
  // spi clock divider
  // sets relative to the system clock
  // n transitions per cycles (SPI_CLOCK_DIV2 = 1 transition / 2 cycles)
  // DIV4 is arduino default, override to make faster
  // needs to be at least 32, or the clock is too slow, 64 to be safe
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  
  // spi bit order
  // sets the order of the bits shifted in and out
  // MSBFIRST = most-significant bit first
  SPI.setBitOrder(MSBFIRST);
  
  // Pause
  delay(1);
}

void init_ads(){
  int chSet;
  // see page 77 for boot sequence

  // Issue Reset Pulse
  digitalWrite(PIN_RESET, HIGH);
  delay(1000);
  digitalWrite(PIN_RESET, LOW);
  delay(1000);
  digitalWrite(PIN_RESET, HIGH);
  delay(100); 

  // Reset communication
  digitalWrite(PIN_CS, LOW);
  delay(1000);
  digitalWrite(PIN_CS, HIGH);
  
  // Wait longer for TI chip to start
  delay(500);
        
  // Send SDATAC Command (Stop Read Data Continuously mode)
  send_command(SDATAC);
  delay(10);
        
  chSet = read_byte(READ | ID);
  Serial.print("-- ID" + String(chSet) + "--");
                  
  // All GPIO set to output 0x0000: (floating CMOS inputs can flicker on and off, creating noise)
  write_byte(GPIO,0x00);
  
  if(debug_msg){
    Serial.println(sep);
    Serial.println("CONFIGs 1 2");
  }

  /****** SET ADS1292 SAMPLE RATE - MUST BE HIGHER THAN FFT or FIR SAMPLE RATE ******/
  //lower sample rate means more oversampling ie more sensativity 
  //write_byte(CONFIG1, SAMP_125_SPS);
  write_byte(CONFIG1, SAMP_250_SPS);
  //write_byte(CONFIG1, SAMP_500_SPS); 
  //write_byte(CONFIG1, SAMP_1_KSPS);
  
  write_byte(CONFIG2, 0xA0);
  delay(1000);
  // write_byte(CONFIG2, 0xA3); //Actives test signal. Comment this line for normal electrodes

  if(debug_msg){
    Serial.println(sep);
    Serial.println("Check Configs");
      chSet = read_byte(CONFIG1);
      Serial.println("CONFIG1: " + String(chSet) + "\t\t" + hex_to_char(chSet) );
      chSet = read_byte(CONFIG2);
      Serial.println("CONFIG2: " + String(chSet) + "\t\t" + hex_to_char(chSet) );
  }
  
  if(debug_msg){
    Serial.println(sep);
    Serial.println("Set Channels");
  }
  // Set channels to input signal
  for (int i = 1; i <= gMaxChan; ++i) {
    write_byte(CHnSET + i, ELECTRODE_INPUT | GAIN_12X);
    write_byte(CHnSET + i, 0x00); //For test signal 0x05. For normal electrodes 0x00
    //write_byte(CHnSET + i,SHORTED);
  }
  
  // Start
  digitalWrite(PIN_START, HIGH);
  delay(150);
  
  // get device id

  //detect active channels
  if(debug_msg){
    Serial.println(sep);
    Serial.println("detecting active channels:");
  }
  gNumActiveChan = 0;
  
  for (int i = 1; i <= gMaxChan; i++) {
    delayMicroseconds(1); 
    chSet = read_byte(CHnSET + i);
    gActiveChan[i] = ((chSet & 7) != SHORTED);          // SHORTED = 0x01
    if ( (chSet & 7) != SHORTED) gNumActiveChan++;  
    if(debug_msg) Serial.println(String(i) + ": " + String(chSet) + "\t\t" + hex_to_char(chSet) ); 
  }
  if(debug_msg) Serial.println("detected " + String(gNumActiveChan) + " active channels.");

  // start reading
  //send_command(RDATAC); // can't read registers when in rdatac mode!
        
}

// get data
int32_t channelData[2];  //combined

void read_ADS1292_data(){
  
  //Serial.println(sep);
  
  //vars
  int numSerialBytes = 1 + (3 * gNumActiveChan); //8-bits header plus 24-bits per ACTIVE channel
  unsigned char serialBytes[numSerialBytes];
  int i = 0;
  
  int values[gNumActiveChan];
  
  unsigned int a, b, c;
         
  // start
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);
        //Serial.println("RD");
        SPI.transfer(RDATA);
        delayMicroseconds(1);
  // get bytes 1-3
  serialBytes[i++] = SPI.transfer(0x00); // get 1st byte of header
        //delayMicroseconds(1);
  SPI.transfer(0x00); //skip 2nd byte of header
        //delayMicroseconds(1);
  SPI.transfer(0x00); //skip 3rd byte of header
        delayMicroseconds(1);
  
  // get channels
  for (int ch = 1; ch <= gMaxChan; ch++) {
    
    a = SPI.transfer(0x00);
                //delayMicroseconds(1);
    b = SPI.transfer(0x00);
                //delayMicroseconds(1);
    c = SPI.transfer(0x00);
                //delayMicroseconds(1);

   /******** START CONVERT TO SINGLE  int32 *************/
   //from https://bois083.wordpress.com/lucid-dreaming-device/version-5-0/ads1292-firmware/

      channelData[ch] = 0;
      
      // read 24 bits of channel data in 3 byte chunks
      channelData[ch] = ((channelData[ch]) << 8) | a;
      channelData[ch] = ((channelData[ch]) << 8) | b;
      channelData[ch] = ((channelData[ch]) << 8) | c;

      // convert 3 byte 2's complement to 4 byte 2's complement
      if((channelData[ch] >> 23) == 1)
      {
         channelData[ch] |= 0xFF000000;
      }
      else
      {
         channelData[ch] &= 0x00FFFFFF;
      }
  }
  
  // end
  delayMicroseconds(2);
  digitalWrite(PIN_CS, HIGH);        
}

// main

void setup(){
  

  // ------------------------------------------------------------
  // init serial
  init_serial();
  delay(50); 
  if(debug_msg) Serial.println("Starting....");

  // ------------------------------------------------------------
  // init ADS1292 bioimdedance sensor
  // set ADS1292 pins
  init_pins();
  
  // spi
  init_spi();
  
  // init ADS1292
  init_ads();

  // ------------------------------------------------------------
  // init FIR filters  [80Hz, 240Hz, 500Hz]

  float filter_coef_deltawave[25]={267,264,378,509,654,806,960,1108,1242,1355,1440,1494,1512,1494,1440,1355,1242,1108,960,806,654,509,378,264,267}; //80Hz
  // float filter_coef_deltawave[55]={-63,-23,-20,-11,7,36,78,135,210,303,417,551,705,879,1070,1276,1493,1716,1942,2165,2379,2579,2759,2914,3040,3132,3189,3208,3189,3132,3040,2914,2759,2579,2379,2165,1942,1716,1493,1276,1070,879,705,551,417,303,210,135,78,36,7,-11,-20,-23,-63}; //80Hz

  float filter_coef_thetawave[31]={274,366,468,445,241,-152,-671,-1186,-1536,-1572,-1218,-505,423,1343,2018,2265,2018,1343,423,-505,-1218,-1572,-1536,-1186,-671,-152,241,445,468,366,274}; //80Hz
  //float filter_coef_thetawave[51]={-46,-179,-265,-391,-445,-395,-190,177,670,1195,1618,1790,1590,968,-26,-1235,-2417,-3294,-3620,-3250,-2186,-593,1229,2908,4088,4512,4088,2908,1229,-593,-2186,-3250,-3620,-3294,-2417,-1235,-26,968,1590,1790,1618,1195,670,177,-190,-395,-445,-391,-265,-179,-46}; //80Hz
  
  float filter_coef_alphawave[25]={-405,-473,-141,678,1473,1459,254,-1596,-2805,-2286,-108,2377,3462,2377,-108,-2286,-2805,-1596,254,1459,1473,678,-141,-473,-405}; //80Hz
  //float filter_coef_alphawave[53]={-80,-23,97,245,267,37,-383,-705,-580,99,989,1429,898,-493,-1903,-2253,-1031,1180,2939,2897,835,-2030,-3773,-3099,-322,2765,4094,2765,-322,-3099,-3773,-2030,835,2897,2939,1180,-1031,-2253,-1903,-493,898,1429,989,99,-580,-705,-383,37,267,245,97,-23,-80}; //80Hz

  float filter_coef_betawave[19]={550,19,-1678,-1035,2716,3012,-2742,-5246,1153,6207,1153,-5246,-2742,3012,2716,-1035,-1678,19,550}; //80Hz
  // float filter_coef_betawave[55]={77,-45,-241,-127,343,467,-176,-747,-277,611,582,-130,-235,28,-568,-1071,655,2951,1202,-3935,-4749,2102,7744,2652,-7453,-7868,3096,10136,3096,-7868,-7453,2652,7744,2102,-4749,-3935,1202,2951,655,-1071,-568,28,-235,-130,582,611,-277,-747,-176,467,343,-127,-241,-45,77}; //80Hz
   
  float filter_coef_emgwave[17]={620,-2043,2476,-733,-2260,2461,2565,-10060,13656,-10060,2565,2461,-2260,-733,2476,-2043,620}; //80Hz
  //float filter_coef_emgwave[39]={-268,461,7,-1485,3160,-3399,1484,1065,-1684,-261,2253,-1343,-1861,3156,219,-4964,3875,6291,-19751,25986,-19751,6291,3875,-4964,219,3156,-1861,-1343,2253,-261,-1684,1065,1484,-3399,3160,-1485,7,461,-268}; //80Hz

  
  // Set the coefficients
  fir_filter_deltawave.setFilterCoeffs(filter_coef_thetawave);
  fir_filter_thetawave.setFilterCoeffs(filter_coef_thetawave);
  fir_filter_alphawave.setFilterCoeffs(filter_coef_alphawave);
  fir_filter_betawave.setFilterCoeffs(filter_coef_betawave);
  fir_filter_emgwave.setFilterCoeffs(filter_coef_emgwave);
  
    // Set the gain
  Serial.print("Alpha Wave Band Pass Filter Gain: ");
  Serial.println(fir_filter_alphawave.getGain());
  fir_filter_deltawave.getGain();
  fir_filter_thetawave.getGain();
  fir_filter_betawave.getGain();
  fir_filter_emgwave.getGain();


  // ------------------------------------------------------------
  // init bluetooth
  setupBluetooth();



//prep for loop
 correctSampleTime = (1 / (float)(FS + 7)) * 1000000; //add 5 because timing is off for some weird reason.. always check
 Serial.print("correct ind sample time: "); Serial.println(correctSampleTime);
      
}

void loop() {
    float startSampleTime = micros();

    //to keep track of when events should be triggered
    trackSamples++;
    if(trackSamples > FS) trackSamples = 1;  
        
    /******************* Read ADS1292 Bioimpedance Sensor *************/
    read_ADS1292_data();           //get ADS1292 data

    /******************* Process Finite Impulse Response (FIR) ********/
    int16_t val;

    //DELTA WAVE
    val = (double)channelData[1] / 10000; 
    val = fir_filter_deltawave.processReading(val);
    deltaWave = val;
//Serial.print("Raw data, prepped data, filtered alpha: \t\t"); Serial.print(channelData[1]); Serial.print("\t\t"); Serial.print((double)channelData[1] / 10000); Serial.print("\t\t"); Serial.println(val);

    //THETA WAVE
    val = (double)channelData[1] / 10000; 
    val = fir_filter_thetawave.processReading(val);
    thetaWave = val;

    //ALPHA WAVE
    val = (double)channelData[1] / 10000; 
    val = fir_filter_alphawave.processReading(val);
    alphaWave = val;

    //BETA WAVE
    val = (double)channelData[1] / 10000; 
    val = fir_filter_betawave.processReading(val);
    betaWave = val;

    //EMG WAVE
    val = (double)channelData[1] / 10000; 
    val = fir_filter_emgwave.processReading(val);
    EMGWave = val;

    /******************* Save Data ************************************/
    deltaWaveOld[9] = deltaWave;
    thetaWaveOld[9] = thetaWave;
    alphaWaveOld[9] = alphaWave;
    betaWaveOld[9] = betaWave; 
    EMGWaveOld[9] = EMGWave; 
    
    for(int k = 0; k < 9; k++){
        deltaWaveOld[k] = deltaWaveOld[k+1];
        thetaWaveOld[k] = thetaWaveOld[k+1];
        alphaWaveOld[k] = alphaWaveOld[k+1];
        betaWaveOld[k] = betaWaveOld[k+1];
        EMGWaveOld[k] = EMGWaveOld[k+1];
    }
  

    /******************* Bluetooth BLE GAP/GATT Scanning **************/
    if( (trackSamples == 10 ) || (trackSamples == 35 ) || (trackSamples == 60 )/*&& IS_CONNECTED == false*/){
        blePeripheral.poll();  //every x samples
        
        //track how much we run over allowed sample time
        bluetoothTimeOverflow = (micros() - startSampleTime) - correctSampleTime; 
        if(bluetoothTimeOverflow > 0){ 
            timeCorrection = true;
        } else if( (micros() - startSampleTime) < correctSampleTime){ delayMicroseconds( (correctSampleTime - (micros() - startSampleTime) ) ); } //regulate sample rate
    }

    /******************* Store Data Between Transmittions *************/
    else if( ( trackSamples == 13 ) || (trackSamples == 39 ) || (trackSamples == 65 ) ){
        for(int w = 0; w < 10; w++){
            deltaWaveAncient[w] = deltaWaveOld[w];
            thetaWaveAncient[w] = thetaWaveOld[w];
            alphaWaveAncient[w] = alphaWaveOld[w];
            betaWaveAncient[w] = betaWaveOld[w];
            EMGWaveAncient[w] = EMGWaveOld[w];
        }

    //    Serial.print("delta wave ancient , delta wave old:\t"); 
   //     for(int q = 0; q < 10; q++){  Serial.print(deltaWaveAncient[q], 6); Serial.print("\t"); Serial.print(deltaWaveOld[q], 6);} Serial.println(" ");
        
        if( (micros() - startSampleTime) < correctSampleTime){ delayMicroseconds( (correctSampleTime - (micros() - startSampleTime) ) ); } //regulate sample rate
    }

    /******************* Process Data *********************************/
    else if( ( (trackSamples == 23 ) || (trackSamples == 49 ) || (trackSamples == 77 ))){
        processData();
    
        //track how much we run over allowed sample time
        bluetoothTimeOverflow = (micros() - startSampleTime) - correctSampleTime; 
        
        if(bluetoothTimeOverflow > 0){ 
            timeCorrection = true;
        } else if( (micros() - startSampleTime) < correctSampleTime){ delayMicroseconds( (correctSampleTime - (micros() - startSampleTime) ) ); } //regulate sample rate
    }
    
    /******************* Bluetooth BLE GAP/GATT Sending ***************/
    else if( ( (trackSamples == 26 ) || (trackSamples == 52 ) || (trackSamples == 80 )) && IS_CONNECTED == true){
        sendBluetooth();  
        
        //track how much we run over allowed sample time
        bluetoothTimeOverflow = (micros() - startSampleTime) - correctSampleTime; 
        
        if(bluetoothTimeOverflow > 0){ 
            timeCorrection = true;
        } else if( (micros() - startSampleTime) < correctSampleTime){ delayMicroseconds( (correctSampleTime - (micros() - startSampleTime) ) ); } //regulate sample rate
    }



    /******************* MANAGE SAMPLE SPEED AND CORRECT FOR OVERFLOW **/
    else if(timeCorrection){
        float modifiedSampleTime = correctSampleTime - bluetoothTimeOverflow;
        if( (micros() - startSampleTime) < modifiedSampleTime){ delayMicroseconds( (modifiedSampleTime - (micros() - startSampleTime) ) ); } //regulate sample rate
        timeCorrection = false;
    } else {
        if( (micros() - startSampleTime) < correctSampleTime){ delayMicroseconds( (correctSampleTime - (micros() - startSampleTime) ) ); } //regulate sample rate
    }


 
 Serial.print("sample time: "); Serial.println(micros() - startSampleTime);
} //end infinate loop


/********************************************************************************************************/
/************************ PRIMARY LOOP FUNCTIONS ********************************************************/
/********************************************************************************************************/
void sendBluetooth(){
    //send data over bluetooth BLE GATT notification characteristic
    DataCharacteristic.setValue(transmitCharArray,20);
    //time to send
    // delay(5);
}

void processData(){

if(debug_msg){
    Serial.print("Old delta, ancient delta:\t");
    for(int q = 0; q < 10; q++){  Serial.print(deltaWaveOld[q], 6); Serial.print("\t"); }  
    for(int q = 0; q < 10; q++){  Serial.print(deltaWaveAncient[q], 6); Serial.print("\t"); } Serial.println(" ");
    Serial.print("Old alpha, ancient alpha:\t");
    for(int q = 0; q < 10; q++){  Serial.print(alphaWaveOld[q], 6); Serial.print("\t"); }  
    for(int q = 0; q < 10; q++){  Serial.print(alphaWaveAncient[q], 6); Serial.print("\t"); } Serial.println(" ");
}

    //front load inter-trasnmition sample
    deltaWave2 = deltaWaveAncient[9]; thetaWave2 = thetaWaveAncient[9]; alphaWave2 = alphaWaveAncient[9]; betaWave2 = betaWaveAncient[9]; EMGWave2 = EMGWaveAncient[9];
    
    //get average based on last 6 samples
    for(int j = 0; j < 10; j++){
        transmitFloatArray[0] = deltaWave + deltaWaveOld[j];
        transmitFloatArray[1] = thetaWave + thetaWaveOld[j];
        transmitFloatArray[2] = alphaWave + alphaWaveOld[j];
        transmitFloatArray[3] = betaWave + betaWaveOld[j];
        transmitFloatArray[4] = EMGWave + EMGWaveOld[j];

        transmitFloatArray[5] = deltaWave2 + deltaWaveAncient[j];
        transmitFloatArray[6] = thetaWave2 + thetaWaveAncient[j];
        transmitFloatArray[7] = alphaWave2 + alphaWaveAncient[j];
        transmitFloatArray[8] = betaWave2 + betaWaveAncient[j];
        transmitFloatArray[9] = EMGWave2 + EMGWaveAncient[j];
    }

    //divide total for average
    for(int g = 0; g < 10; g++){
      transmitFloatArray[g] = transmitFloatArray[g] / 11;
    }

    if(debug_msg) Serial.print("pre-normed delta, theta, alpha, beta, emg, delta2, theta2, alpha2, beta2, emg2:\t"); 
    if(debug_msg) for(int q = 0; q < 10; q++){  Serial.print(transmitFloatArray[q], 6); Serial.print("\t"); } Serial.println(" ");

    //normalize
    for(int m = 0; m < 10; m++){
      transmitFloatArray[m] = (transmitFloatArray[m] + 8000) / 16000;  
    }

    if(debug_msg) Serial.print("normed delta, theta, alpha, beta, emg, delta2, theta2, alpha2, beta2, emg2:   \t"); 
    if(debug_msg) for(int q = 0; q < 10; q++){  Serial.print(transmitFloatArray[q], 6); Serial.print("\t"); } Serial.println(" ");

    //bounds
    for(int k = 0; k < 10; k++){
      transmitFloatArray[k] = min(max(transmitFloatArray[k] , 0.00000001), 0.99999999); 
    }

    if(debug_msg) Serial.print("sending delta, theta, alpha, beta, emg, delta2, theta2, alpha2, beta2, emg2:  \t"); 
    if(debug_msg) for(int q = 0; q < 10; q++){  Serial.print(transmitFloatArray[q], 6); Serial.print("\t"); } Serial.println(" ");
    
    //impedence signals have a big spread so we will take log before sending then inverse log in our target application
    //float   deltaWave = min(max(1e-3,log10( deltaWave + offset )),10)/10;  
  

    /******************* Bluetooth BLE Data Transmittion **********/
    BLECentral central = blePeripheral.central();

    if(central){ // if a central is connected to peripheral

        //we will send six significant digits from each signal log value  example: log10(565629) --> 5.75253166 --> 0.575253166 --> ["57"],["52"],["53"] 
       // const unsigned char transmitCharArray[20] = {
       const unsigned char tempTransmitCharArray[20] = {
            (uint8_t)( int(transmitFloatArray[0] * 100) ),  
            (uint8_t)( int(transmitFloatArray[0] * 10000) - int(transmitFloatArray[0] * 100) ),
            (uint8_t)( int(transmitFloatArray[1] * 100) ),  
            (uint8_t)( int(transmitFloatArray[1] * 10000) - int(transmitFloatArray[1] * 100) ),
            (uint8_t)( int(transmitFloatArray[2] * 100) ),  
            (uint8_t)( int(transmitFloatArray[2] * 10000) - int(transmitFloatArray[2] * 100) ),
            (uint8_t)( int(transmitFloatArray[3] * 100) ),  
            (uint8_t)( int(transmitFloatArray[3] * 10000) - int(transmitFloatArray[3] * 100) ),
            (uint8_t)( int(transmitFloatArray[4] * 100) ),  
            (uint8_t)( int(transmitFloatArray[4] * 10000) - int(transmitFloatArray[4] * 100) ),
            (uint8_t)( int(transmitFloatArray[5] * 100) ),  
            (uint8_t)( int(transmitFloatArray[5] * 10000) - int(transmitFloatArray[5] * 100) ),
            (uint8_t)( int(transmitFloatArray[6] * 100) ),  
            (uint8_t)( int(transmitFloatArray[6] * 10000) - int(transmitFloatArray[6] * 100) ),
            (uint8_t)( int(transmitFloatArray[7] * 100) ),  
            (uint8_t)( int(transmitFloatArray[7] * 10000) - int(transmitFloatArray[7] * 100) ),
            (uint8_t)( int(transmitFloatArray[8] * 100) ),  
            (uint8_t)( int(transmitFloatArray[8] * 10000) - int(transmitFloatArray[8] * 100) ),
            (uint8_t)( int(transmitFloatArray[9] * 100) ),  
            (uint8_t)( int(transmitFloatArray[9] * 10000) - int(transmitFloatArray[9] * 100) ),
        }; 
        memcpy( transmitCharArray, tempTransmitCharArray, 20 );

    }
}



