/********************************************************************************************************/
/************************ INCLUDED FILES ****************************************************************/
/********************************************************************************************************/

#include <stdio.h>
#include "ssd1306.h"          //OLED display
#include "ssd1331_api.h"
#include <SPI.h>
#include <BLEPeripheral.h>    //bluetooth
#include <BLEUtil.h>
#include "BLESerial.h"

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
#define OLED_LED_POW       16
#define OLED_IC_POW        17
#define KX126_CS           24
#define KX126_SDI          19
#define KX126_SDO          20
#define KX126_SCL          18
#define KX126_INT          23

//BLESerial
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9

//Hardware UART dummy NC GPIO
#define PIN_SERIAL_RX       25 
#define PIN_SERIAL_TX       26


/********************************************************************************************************/
/************************ VARIABLES *********************************************************************/
/********************************************************************************************************/

  //Button - raw capacitive touch button value
    int buttonValue;     

  //Battery Power
    int batteryValue;

  //Timestamp
    float  clocktime = 0;

  //Bluetooth
    unsigned long millisPerTransmit; 
    unsigned long millisPrevious = 0;
    char readTextArray[13];
    char writeTextArray[13];
    
  //OLED display
  //scroll place holder and index in array of RGB colors
    int placeHolder, colorNumber;

/********************************************************************************************************/
/************************ DECLARATIONS ******************************************************************/
/********************************************************************************************************/

  //Bluetooth - create peripheral instance, see pinout definitions above
  BLESerial bleSerial(BLE_REQ, BLE_RDY, BLE_RST);


/********************************************************************************************************/
/************************ SETUP *************************************************************************/
/********************************************************************************************************/

void setup()
{
  //OLED display
    pinMode(OLED_IC_POW, OUTPUT);  digitalWrite(OLED_IC_POW, 1);
    pinMode(OLED_LED_POW, OUTPUT);  digitalWrite(OLED_LED_POW, 0);
    ssd1331_96x64_spi_init(OLED_RES, OLED_CS, OLED_DC);  
    
    delay(100);
    
    //start at the top
    placeHolder = 0;


  //BLE Serial
  
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
    Serial.begin(9600); 

    delay(50);

    //clear screen
    hardClear();
}

void loop()
{
    //monitor Bluetooth serial
    bleSerial.poll();

    //read any data received over Bluetooth serial from Nordic UART app or other source
    readBLE();
    delay(80);
}


//read BLE serial and respond if indicated
void readBLE() {
  //do we have a connection?
  if (bleSerial) {
    int byte;
    int count = 0;
    //max message size is 13 characters, one line
    for(int q = 0; q < 13; q++){ readTextArray[q] = '\0'; }
    //while we have more serial data to look at
    while ((byte = bleSerial.read()) > 0){ 
      //Serial.write((char)byte);
      if(count < 13){
        readTextArray[count] = (char)byte;
        count++;
      }
    } 
    //while ((byte = bleSerial.read()) > 0) bleSerial.write((char)byte);
    

    //if we processed at least one character of serial data
    if(count > 0){
      //clear screen
      hardClear();

      //if we recieved text naming a color, display message in that color and reply with a custom message related to the color 
      if( StrContains(readTextArray, "red") ){
        bleSerial.println("Like a ruby!");
        printOLED(readTextArray, 10, 10, 255, 0, 0);
      }
      else if( StrContains(readTextArray, "green") ){
        bleSerial.println("Like an emerald!");
        printOLED(readTextArray, 10, 10, 0, 255, 0);
      }
      else if( StrContains(readTextArray, "blue") ){
        bleSerial.println("Like the ocean!");
        printOLED(readTextArray, 10, 10, 0, 0, 255);
      }
      else if( StrContains(readTextArray, "yellow") ){
        bleSerial.println("Like the sun!");
        printOLED(readTextArray, 10, 10, 255, 255, 0);
      }
      else if( StrContains(readTextArray, "purple") ){
        bleSerial.println("Like an eggplant!");
        printOLED(readTextArray, 10, 10, 255, 0, 255);
      }
      else{
        //default no color detected
        printOLED(readTextArray, 10, 10, 255, 255, 255);
        delay(5000);
      }
    }
  }
}

//clear screen by writing over it
static void hardClear()
{
    ssd1306_clearScreen();
    for (uint8_t y = 0; y < 64; y++)
    {
        ssd1331_drawLine(0,y, 95, y, RGB_COLOR8(0,0,0));
    }
}

//extremely simple 10x6 pixel font
const uint8_t symbolMap[433][6] = {
{0,1,1,1,1,0}, //0
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{0,0,0,1,1,0}, //1
{0,0,1,1,1,0},
{0,1,1,1,1,0},
{0,0,0,1,1,0},
{0,0,0,1,1,0},
{0,0,0,1,1,0},
{0,0,0,1,1,0},
{0,0,0,1,1,0},
{0,0,0,1,1,0},
{0,0,0,1,1,0},

{0,0,0,0,0,0}, //2
{0,1,1,1,1,0},
{1,1,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,1,1,0},
{0,0,1,1,0,0},
{0,1,1,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,1,1,1,1},

{1,1,1,1,1,1}, //3
{0,0,0,1,1,0},
{0,0,1,1,0,0},
{0,1,1,1,1,0},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{0,0,0,1,1,0}, //4
{0,0,1,1,1,0},
{0,1,1,1,1,0},
{0,1,0,1,1,0},
{1,0,0,1,1,0},
{1,0,0,1,1,0},
{1,0,0,1,1,0},
{1,1,1,1,1,1},
{0,0,0,1,1,0},
{0,0,0,1,1,0},

{1,1,1,1,1,1}, //5
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,1,1,1,0},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{0,0,1,1,1,0}, //6
{0,1,1,0,0,0},
{1,1,0,0,0,0},
{1,1,1,1,1,0},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{1,1,1,1,1,1}, //7
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,1,1,0},
{0,0,0,1,1,0},
{0,0,0,1,1,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},

{0,1,1,1,1,0}, //8
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{0,1,1,1,1,0}, //9
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,1},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,1,1,0},
{0,1,1,1,0,0},

{0,0,1,1,0,0}, //A
{0,0,1,1,0,0},
{0,1,1,1,1,0},
{0,1,1,1,1,0},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,1,1,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},

{1,1,1,1,1,0}, //B
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,1,1,1,0},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,1,1,1,0},

{0,1,1,1,1,0}, //C
{1,1,0,0,1,1},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{1,1,1,1,0,0}, //D
{1,1,0,1,1,0},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,1,1,0},
{1,1,1,1,0,0},

{1,1,1,1,1,1}, //E
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,1,1,1,1},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,1,1,1,1},

{1,1,1,1,1,1}, //F
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,1,1,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},

{0,1,1,1,1,0}, //G
{1,1,0,0,1,1},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,1,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{1,1,0,0,1,1}, //H
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,1,1,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},

{0,0,1,1,0,0}, //I
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},

{0,0,0,0,1,1}, //J
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{1,1,0,0,0,1}, //K
{1,1,0,0,1,1},
{1,1,0,1,1,0},
{1,1,1,1,0,0},
{1,1,1,0,0,0},
{1,1,1,1,0,0},
{1,1,0,1,1,0},
{1,1,0,0,1,1},
{1,1,0,0,0,1},

{1,1,0,0,0,0}, //L
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,1,1,1,0},

{1,0,0,0,0,1}, //M
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,1,1,1,1},
{1,1,1,1,1,1},
{1,0,1,1,0,1},
{1,0,1,1,0,1},
{1,0,0,0,0,1},
{1,0,0,0,0,1},

{1,1,0,0,1,1}, //N
{1,1,0,0,1,1},
{1,1,1,0,1,1},
{1,1,1,0,1,1},
{1,1,1,1,1,1},
{1,1,1,1,1,1},
{1,1,0,1,1,1},
{1,1,0,1,1,1},
{1,1,0,0,1,1},

{0,1,1,1,1,0}, //O
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{1,1,1,1,1,0}, //P
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,1,1,1,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},

{0,1,1,1,1,0}, //Q
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},
{0,0,0,0,1,1},

{1,1,1,1,1,0}, //R
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,1,1,1,1},
{1,1,0,1,1,0},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},

{0,1,1,1,1,0}, //S
{1,1,0,0,1,1},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{0,1,1,1,1,0},
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{1,1,1,1,1,1}, //T
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},

{1,1,0,0,1,1}, //U
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},

{1,1,0,0,1,1}, //V
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,0,1,1},
{0,1,1,0,1,1},
{0,1,1,1,1,0},
{0,0,1,1,0,0},
{0,0,0,1,0,0},

{1,0,0,0,0,1}, //W
{1,0,0,0,0,1},
{1,0,0,0,0,1},
{1,0,0,0,0,1},
{1,0,1,1,0,1},
{1,0,1,1,0,1},
{1,0,1,1,0,1},
{1,1,1,1,1,1},
{0,1,1,1,1,0},

{1,1,0,0,1,1}, //X
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},
{0,0,1,1,0,0},
{0,1,1,1,1,0},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},

{1,1,0,0,1,1}, //Y
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{1,1,0,0,1,1},
{0,1,1,1,1,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0},

{1,1,1,1,1,1}, //Z
{0,0,0,0,1,1},
{0,0,0,0,1,1},
{0,0,0,1,1,0},
{0,0,1,1,0,0},
{0,1,1,0,0,0},
{1,1,0,0,0,0},
{1,1,0,0,0,0},
{1,1,1,1,1,1},

{0,0,0,0,0,0}, //.
{0,0,0,0,0,0},
{0,0,0,0,0,0},
{0,0,0,0,0,0},
{0,0,0,0,0,0},
{0,0,0,0,0,0},
{0,0,0,0,0,0},
{0,0,1,1,0,0},
{0,0,1,1,0,0}};

//print an individual character to the OLED display using the basic font system
void printCharOLED(char symbol, int Xpos, int Ypos, int r, int g, int b)
{
  int mapOffset = 999;
  if(symbol == '0'){ mapOffset = 0; } 
  else if(symbol == '1'){ mapOffset = 10; }
  else if(symbol == '2'){ mapOffset = 20; }
  else if(symbol == '3'){ mapOffset = 30; }
  else if(symbol == '4'){ mapOffset = 40; }
  else if(symbol == '5'){ mapOffset = 50; }
  else if(symbol == '6'){ mapOffset = 60; }
  else if(symbol == '7'){ mapOffset = 70; }
  else if(symbol == '8'){ mapOffset = 80; }
  else if(symbol == '9'){ mapOffset = 90; }
  else if(symbol == 'a' || symbol == 'a'){ mapOffset = 100 + 0*9; }
  else if(symbol == 'B' || symbol == 'b'){ mapOffset = 100 + 1*9; }
  else if(symbol == 'C' || symbol == 'c'){ mapOffset = 100 + 2*9; }
  else if(symbol == 'D' || symbol == 'd'){ mapOffset = 100 + 3*9; }
  else if(symbol == 'E' || symbol == 'e'){ mapOffset = 100 + 4*9; }
  else if(symbol == 'F' || symbol == 'f'){ mapOffset = 100 + 5*9; }
  else if(symbol == 'G' || symbol == 'g'){ mapOffset = 100 + 6*9; }
  else if(symbol == 'H' || symbol == 'h'){ mapOffset = 100 + 7*9; }
  else if(symbol == 'I' || symbol == 'i'){ mapOffset = 100 + 8*9; }
  else if(symbol == 'J' || symbol == 'j'){ mapOffset = 100 + 9*9; }
  else if(symbol == 'K' || symbol == 'k'){ mapOffset = 100 + 10*9; }
  else if(symbol == 'L' || symbol == 'l'){ mapOffset = 100 + 11*9; }
  else if(symbol == 'M' || symbol == 'm'){ mapOffset = 100 + 12*9; }
  else if(symbol == 'N' || symbol == 'n'){ mapOffset = 100 + 13*9; }
  else if(symbol == 'O' || symbol == 'o'){ mapOffset = 100 + 14*9; }
  else if(symbol == 'P' || symbol == 'p'){ mapOffset = 100 + 15*9; }
  else if(symbol == 'Q' || symbol == 'q'){ mapOffset = 100 + 16*9; }
  else if(symbol == 'R' || symbol == 'r'){ mapOffset = 100 + 17*9; }
  else if(symbol == 'S' || symbol == 's'){ mapOffset = 100 + 18*9; }
  else if(symbol == 'T' || symbol == 't'){ mapOffset = 100 + 19*9; }
  else if(symbol == 'U' || symbol == 'u'){ mapOffset = 100 + 20*9; }
  else if(symbol == 'V' || symbol == 'v'){ mapOffset = 100 + 21*9; }
  else if(symbol == 'W' || symbol == 'w'){ mapOffset = 100 + 22*9; }
  else if(symbol == 'X' || symbol == 'x'){ mapOffset = 100 + 23*9; }
  else if(symbol == 'Y' || symbol == 'y'){ mapOffset = 100 + 24*9; }
  else if(symbol == 'Z' || symbol == 'z'){ mapOffset = 100 + 25*9; }

  if( mapOffset < 900){ //is there a character to print?
  //number height or letter height
  int symbolHeight = 10;
  if(mapOffset >= 100){ symbolHeight = 9; Ypos = Ypos + 1; }
  
  //traverse and draw image
  for (uint8_t y = 0; y < symbolHeight; y++)
    {   
      for (uint8_t x = 0; x < 6; x++)
      {
          if(symbolMap[y + mapOffset][x]){
              ssd1331_drawLine( (x + Xpos), (y + Ypos), (x + Xpos), (y + Ypos), RGB_COLOR8(b, g, r));
          }
      }
  }
  }
}

//print one line of characters to the OLED display
void printOLED(char printText[],int Xpos, int Ypos, int r, int g, int b)
{
  char printTextArray[13];
  for(int q = 0; q < 12; q++){ printTextArray[q] = '\0'; }
  strcpy(printTextArray, printText); //copy to 12 element array
  
  for (int i = 0; i < 12; i++){
    printCharOLED(printTextArray[i], i*8, Ypos, r, g, b);
  }
}

// searches for the string sfind in the string str
// returns 1 if string found
// returns 0 if string not found
int StrContains(char *str, char *sfind)
{
    char found = 0;
    char index = 0;
    char len;

    len = strlen(str);
    
    if (strlen(sfind) > len) {
        return 0;
    }
    while (index < len) {
        if (str[index] == sfind[found]) {
            found++;
            if (strlen(sfind) == found) {
                return 1;
            }
        }
        else {
            found = 0;
        }
        index++;
    }

    return 0;
}



