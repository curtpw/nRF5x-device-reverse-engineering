/*
---------------------------------------------------------------------------------
ADS1292 for Arduino Code

Inspired by:
https://github.com/ericherman/eeg-mouse
http://www.mccauslandcenter.sc.edu/CRNL/ads1298

---------------------------------------------------------------------------------
-----------------------------------------------------------
| Module | Signal         |   B20 nRF52 Pin               |
|--------+----------------+-------------------------------+
|        |  CS            |  15                           |
|  SPI   |  MOSI (DOUT)   |  18  (not library default)    |
|        |  MISO (DIN)    |  16  (not library default)    |
|        |  SCK           |  17 (not library default)     |
|--------+----------------+-------------------------------+
|        |  START         |  20                           | 
|        |  RESET         |  22                           |
|  ADS   |  DRDY          |  ?                            | 
|        |  CLKSEL        |                               |    
|        |  STOP          |                               |           
|        |  PWDN          |                               |            
-----------------------------------------------------------
|        |  +5V           |  +5V                          | 
| Power  |  +3.3V         |  +3.3V                        |  
|        |  GND           |  GND                          |  
-----------------------------------------------------------


SCLK   |  Device attempts to decode and execute commands every eight serial clocks. 
       |  It is recommended that multiples of 8 SCLKs be presented every serial transfer to keep the interface in a normal operating mode.
       |
DRDY   |  low = new data are available, regardless of CS
       |
START  |  keep low if using start opcode. The START pin or the START command is used to place the device either in normal data capture mode or pulse data capture mode.
       |
PWDN   |  When PWDN is pulled low, all on-chip circuitry is powered down
       |
CS     |  low = SPI is active, must remain low during communication, then wait 4-5 tCLK cycles before returning to high, set CS high then low to reset the communication
       |  connect to J3.1 if JP21 = <**o], or J3.7 if JP21 = <o**]
       |
RESET  |  low = force a reset, RESET is automatically issued to the digital filter whenever registers CONFIG1 and RESP are set to new values with a WREG command
       |
CLKSEL |  internal clock: JP23 is <o**], external clock: JP23 is <**o]


SPI library
the spi clock divider (setClockDivider) sets relative to the system clock, the frequency of SLK relative to the chip frequency
it needs to be at least 32, or the clock is too slow, 64 to be safe
SPI.setClockDivider(SPI_CLOCK_DIV32);

uint8_t = unsigned char with 8 bits

*/

#define NRF52

/********************************************************************************************************/
/************************ INCLUDES **********************************************************************/
/********************************************************************************************************/
#include <SPI.h>
#include <BLEPeripheral.h>    //bluetooth
#include <BLEUtil.h>
#include "Adafruit_ZeroFFT.h"

//utilities
#include <math.h>
#include <stdarg.h>

/********************************************************************************************************/
/************************ CONSTANTS / SYSTEM VAR ********************************************************/
/********************************************************************************************************/
bool    debug = false;                //turn serial on/off to get data or turn up sample rate
bool    debug_time = false;           //turn loop component time debug on/off
boolean debug_msg     = true;         //ads1292 debug msg
bool    IS_CONNECTED = false;


/********************************************************************************************************/
/************************ VARIABLES *********************************************************************/
/********************************************************************************************************/
//#define SAMPLES 512             //Must be a power of 2
//#define SAMPLING_FREQUENCY 585  //Hz, must be less than 10000 due to ADC

//***FFT variables
//the signal in signal.h has 2048 samples. Set this to a value between 16 and 2048 inclusive.
//this must be a power of 2
#define DATA_SIZE 512

//the sample rate
//#define FS 558 //FOR EEG
#define FS 600    
int16_t data[DATA_SIZE];
float deltaWave, thetaWave, alphaWave, betaWave, EMGWave;

float deltaWaveOld[5] = {100, 100, 100, 100, 100};
float thetaWaveOld[5] = {100, 100, 100, 100, 100};
float alphaWaveOld[5] = {100, 100, 100, 100, 100};
float betaWaveOld[5] = {100, 100, 100, 100, 100};
float EMGWaveOld[5] = {100, 100, 100, 100, 100};

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
#define PIN_LED    13

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
  //lower sample rate means more oversampling ie more sensativity but we need higher rate for FFT
  // write_byte(CONFIG1, SAMP_250_SPS);
  write_byte(CONFIG1, SAMP_500_SPS); 
 // write_byte(CONFIG1, SAMP_1_KSPS);
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

void read_and_send_data(){
  
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
  // init application
  delay(1000);  
        
  // set ADS1292 pins
  init_pins();
  
  // serial
  init_serial();
  delay(50);  

  // on!
  if(debug_msg) Serial.println("on");
  
  // spi
  init_spi();
  
  // init ADS1292
  init_ads();
  
  // off!
  //digitalWrite(PIN_LED, LOW);
  if(debug_msg) Serial.println("off");
  //blinkyblink();

  // CONFIGURE & START BLUETOOTH /
  setupBluetooth();

  //FOR FFT
 // sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
 // Serial.print("sampling_period_us: "); Serial.println(sampling_period_us);

//prep for loop
 correctSampleTime = (1 / (float)FS) * 1000000;
 Serial.print("correct ind sample time: "); Serial.println(correctSampleTime);
      
}

void loop() {
    float startSampleTime = micros();

    //to keep track of when inter-sample events should be triggered
    trackSamples++;
    if(trackSamples > DATA_SIZE){ trackSamples = 1; samplesReadyFlag = true; }
    
    
    /******************* Read ADS1292 Bioimpedance Sensor *************/
   // int32_t avg = 0;
    read_and_send_data();           //get ADS1292 data
    int16_t val;
    val = (double)channelData[1] / 100; 
   // avg += val;
    addData(val);


    //processing and sending events distributed across data collection
    if(samplesReadyFlag){

        /******************* Bluetooth BLE GAP/GATT Scanning **************/
        if(trackSamples == 1 && IS_CONNECTED == false){  //run on any loop count divisible by 10
            blePeripheral.poll(); 

            //duplicate sample to fill time
            addData(val);
            addData(val);
            if( (micros() - startSampleTime) < (correctSampleTime * 3)){ delayMicroseconds( ((correctSampleTime * 3) - (micros() - startSampleTime) ) ); } //triple sample
       //     Serial.print("BLE poll sample time: "); Serial.println(micros() - startSampleTime); 
        } 

        //process time series data sample with FFT to get frequency domain bin series data
     //   if(trackSamples == 32 || trackSamples == 96 || trackSamples == 163 || trackSamples == 224 || trackSamples == 288 || trackSamples == 352 || trackSamples == 416 || trackSamples == 483){
     //   if(trackSamples == 16 || trackSamples == 48 || trackSamples == 82 || trackSamples == 112 || trackSamples == 144 || trackSamples == 176 || trackSamples == 208 || trackSamples == 242){
        if(trackSamples == 64 || trackSamples == 320){
            processFFT();
            
            //duplicate sample to fill time
            addData(val);
            addData(val);
            addData(val);
            if( (micros() - startSampleTime) < (correctSampleTime * 4)){ delayMicroseconds( ((correctSampleTime * 4) - (micros() - startSampleTime) ) ); } //quadruple sample
       //     Serial.print("FFT sample time: "); Serial.println(micros() - startSampleTime); 
        }
        
        //send frequency domain data over BLE notification characteristic
      //  if(trackSamples == 32 || trackSamples == 64 || trackSamples == 96 || trackSamples == 128 || trackSamples == 162 || trackSamples == 192 || trackSamples == 224 || trackSamples == 256){
        if(trackSamples == 128 || trackSamples == 384){
            sendBluetooth();
            
            //duplicate sample to fill time
            addData(val);
            addData(val);
            if( (micros() - startSampleTime) < (correctSampleTime * 3)){ delayMicroseconds( ((correctSampleTime * 3) - (micros() - startSampleTime) ) ); } //triple sample
        //    Serial.print("BLE send sample time: "); Serial.println(micros() - startSampleTime); 
        }

    } else {
      if( (micros() - startSampleTime) < correctSampleTime){ delayMicroseconds( (correctSampleTime - (micros() - startSampleTime) ) ); } //single sample
    }
    
    //  Serial.print("Raw sample: "); Serial.print(channelData[1]); Serial.print("Mod sample: "); Serial.println((double)channelData[1] / 100);

    // Serial.print("Raw Ind Sample time: "); Serial.println( micros() - startSampleTime );
    // Serial.print("delay time: "); Serial.println( (correctSampleTime - (micros() - startSampleTime) ) );
    //push down sample rate to capture lower frequencies
    
    
 
} //end infinate loop


/********************************************************************************************************/
/************************ PRIMARY LOOP FUNCTIONS ********************************************************/
/********************************************************************************************************/

void addData(int16_t val){
    //move data forward in sample array
    for(int i=0; i<DATA_SIZE; i++){  data[i] = data[i + 1];  }

    //add latest data to sample array
    data[DATA_SIZE - 1] = val;   
}

void processFFT(){
    //run the FFT
  ZeroFFT(data, DATA_SIZE);

  //get the maximum value
//  float maxVal = 0;
//  float maxFrequency = 0;
  
  //data is only meaningful up to sample rate/2, discard the other half
 /* for(int i=0; i<DATA_SIZE/2; i++) if(data[i] > maxVal){ 
      maxVal = data[i]; 
      maxFrequency = FFT_BIN(i, FS, DATA_SIZE); 
  } */
//  Serial.print(maxFrequency); Serial.print(" Hz MAX: "); Serial.println(maxVal); 
}

void sendBluetooth(){

    deltaWave = 0; thetaWave = 0; alphaWave = 0; betaWave = 0; EMGWave = 0;

  //data is only meaningful up to sample rate/2, discard the other half
  for(int i=0; i<DATA_SIZE/2; i++){
    float currentFrequency = FFT_BIN(i, FS, DATA_SIZE);
    //print the frequency
 /*   Serial.print(currentFrequency);
    Serial.print(" Hz: ");

    //print the corresponding FFT output
    Serial.println(data[i]); */

    if(currentFrequency > 1 && currentFrequency < 4 ){ deltaWave = deltaWave + data[i]; }
    if(currentFrequency > 3 && currentFrequency < 8){ thetaWave = thetaWave + data[i]; }
    if(currentFrequency > 7 && currentFrequency < 13){ alphaWave = alphaWave + data[i]; }
    if(currentFrequency > 12 && currentFrequency < 25){ betaWave = betaWave + data[i]; }
    if(currentFrequency > 180 && currentFrequency < 220){ EMGWave = EMGWave + data[i]; }
  }


  for(int j = 0; j < 5; j++){
    deltaWave = deltaWave + deltaWaveOld[j];
    thetaWave = thetaWave + thetaWaveOld[j];
    alphaWave = alphaWave + alphaWaveOld[j];
    betaWave = betaWave + betaWaveOld[j];
    EMGWave = EMGWave + EMGWaveOld[j];
  }

    deltaWave = deltaWave / 6;
    thetaWave = thetaWave / 6;
    alphaWave = alphaWave / 6;
    betaWave = betaWave / 6;
    EMGWave = EMGWave  / 6;
  
  for(int k = 0; k < 4; k++){
    deltaWaveOld[k] = deltaWaveOld[k+1];
    thetaWaveOld[k] = thetaWaveOld[k+1];
    alphaWaveOld[k] = alphaWaveOld[k+1];
    betaWaveOld[k] = betaWaveOld[k+1];
    EMGWaveOld[k] = EMGWaveOld[k+1];
  }
  
  deltaWaveOld[4] = deltaWave;
  thetaWaveOld[4] = thetaWave;
  alphaWaveOld[4] = alphaWave;
  betaWaveOld[4] = betaWave; 
  EMGWaveOld[4] = EMGWave; 
    
 /*   Serial.print("Delta: "); Serial.print(deltaWave);
    Serial.print("\tTheta: "); Serial.print(thetaWave);
    Serial.print("\tAlpha: "); Serial.print(alphaWave);
    Serial.print("\tBeta: "); Serial.println(betaWave);

    Serial.print("log10(delta): "); Serial.println(log10( deltaWave ), 5); */

    //impedence signals have a big spread so we will take log before sending then inverse log in our target application
    float   deltaLog = min(max(1e-3,log10( deltaWave )),10)/10;
    float   thetaLog = min(max(1e-3,log10( thetaWave )),10)/10; 
    float   alphaLog = min(max(1e-3,log10( alphaWave )),10)/10;
    float   betaLog = min(max(1e-3,log10( betaWave )),10)/10; 
    float   EMGLog = min(max(1e-3,log10( EMGWave )),10)/10; 
  
 /*   Serial.print("lgD: "); Serial.print(deltaLog, 5);
    Serial.print("\tlgT: "); Serial.print(thetaLog, 5);
    Serial.print("\tlgA: "); Serial.print(alphaLog, 5);
    Serial.print("\tlgB: "); Serial.println(betaLog, 5); Serial.println(" "); */

    /******************* Bluetooth BLE Data Transmittion **********/
    BLECentral central = blePeripheral.central();

    if(central){ // if a central is connected to peripheral

        //we will send six significant digits from each signal log value  example: log10(565629) --> 5.75253166 --> 0.575253166 --> ["57"],["52"],["53"] 
        const unsigned char imuCharArray[20] = {
            (uint8_t)( int(deltaLog * 100) ),  
            (uint8_t)( int(deltaLog * 10000) - int(deltaLog * 100) ),
            (uint8_t)( int(deltaLog * 1000000) - int(deltaLog * 10000) - int(deltaLog * 100) ),
            (uint8_t)( int(thetaLog * 100) ),  
            (uint8_t)( int(thetaLog * 10000) - int(thetaLog * 100) ),
            (uint8_t)( int(thetaLog * 1000000) - int(thetaLog * 10000) - int(thetaLog * 100) ),
            (uint8_t)( int(alphaLog * 100) ),  
            (uint8_t)( int(alphaLog * 10000) - int(alphaLog * 100) ),
            (uint8_t)( int(alphaLog * 1000000) - int(alphaLog * 10000) - int(alphaLog * 100) ),
            (uint8_t)( int(betaLog * 100) ),  
            (uint8_t)( int(betaLog * 10000) - int(betaLog * 100) ),
            (uint8_t)( int(betaLog * 1000000) - int(betaLog * 10000) - int(betaLog * 100) ),
            (uint8_t)( int(EMGLog * 100) ),  
            (uint8_t)( int(EMGLog * 10000) - int(EMGLog * 100) ),
            (uint8_t)( int(EMGLog * 1000000) - int(EMGLog * 10000) - int(EMGLog * 100) ),
            (uint8_t)(0),
            (uint8_t)(0),
            (uint8_t)(0),
            (uint8_t)(0),  
            (uint8_t)(0)                   //empty
        }; 
        //send data over bluetooth BLE GATT notification characteristic
        DataCharacteristic.setValue(imuCharArray,20);
        //time to send
       // delay(5);
    }
}



