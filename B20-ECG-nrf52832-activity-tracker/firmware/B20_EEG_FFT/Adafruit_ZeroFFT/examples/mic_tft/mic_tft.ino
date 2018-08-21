#include "Adafruit_ZeroFFT.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define STMPE_CS 6
#define TFT_CS   9
#define TFT_DC   10
#define SD_CS    5

//set this to 0 to disable autoranging and scale to FFT_MAX
#define AUTOSCALE 1

#if AUTOSCALE == 0
#define FFT_MAX 512
#endif

/*set this to a power of 2. 
 * Lower = faster, lower resolution
 * Higher = slower, higher resolution
  */
#define DATA_SIZE 1024

//the sample rate.
#define FS 2360

#define NUM_REFERENCE_LINES 6

#define GRAPH_OFFSET 30

#define GRAPH_WIDTH (tft.width() - 3)
#define GRAPH_HEIGHT (tft.height() - GRAPH_OFFSET)

#define GRAPH_MIN (tft.height() - 2)
#define GRAPH_MAX (tft.height() - GRAPH_HEIGHT)
static float xScale;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

int16_t data[DATA_SIZE];

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  delay(10);
  //while(!Serial);
  Serial.println("FFT with TFT featherwing!");

  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
  pinMode(A1, OUTPUT);
  digitalWrite(A1, LOW);
  
  tft.begin();
  tft.setRotation(1);
}

void drawReference(){
  //draw some reference lines
  uint16_t refStep = DATA_SIZE / 2 / NUM_REFERENCE_LINES;
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_RED);
  for(int i=0; i<DATA_SIZE/2 - refStep; i+=refStep){
    uint16_t fc = FFT_BIN(i, FS, DATA_SIZE);
    uint16_t xpos = map(i, 0, DATA_SIZE/2, 0, GRAPH_WIDTH);
    tft.setCursor(xpos, 0);
    tft.drawLine(xpos, GRAPH_MIN, xpos, GRAPH_MAX, ILI9341_RED);
    
    tft.print(fc);
  }
}

void loop() {
  int32_t avg = 0;
  for(int i=0; i<DATA_SIZE; i++){
    int16_t val = analogRead(A2);
    avg += val;
    data[i] = val;
  }

  //remove DC offset and gain up to 16 bits
  avg = avg/DATA_SIZE;
  for(int i=0; i<DATA_SIZE; i++) data[i] = (data[i] - avg) * 64;
    
  //run the FFT
  ZeroFFT(data, DATA_SIZE);

#if AUTOSCALE
  //get the maximum value
  float maxVal = 0;
  //data is only meaningful up to sample rate/2, discard the other half
  for(int i=0; i<DATA_SIZE/2; i++) if(data[i] > maxVal) maxVal = data[i];

  //normalize to the maximum returned value
  for(int i=0; i<DATA_SIZE/2; i++)
    data[i] =(float)data[i] / maxVal * GRAPH_HEIGHT;
#endif

  tft.fillScreen(ILI9341_BLACK);
  drawReference();

  //draw all points
  int16_t lasty, thisy;
  thisy = GRAPH_MIN;
  for(int i=1; i<GRAPH_WIDTH; i++){
    uint16_t ix = map(i, 0, GRAPH_WIDTH, 0, DATA_SIZE/2);
    
#if AUTOSCALE
    thisy = constrain(map(data[ix], 0, GRAPH_HEIGHT, GRAPH_MIN, GRAPH_MAX), GRAPH_OFFSET, GRAPH_MIN);
#else
    thisy = constrain(map(data[ix], 0, FFT_MAX, GRAPH_MIN, GRAPH_MAX), GRAPH_OFFSET, GRAPH_MIN);
#endif
 
    tft.drawLine(i - 1, lasty, i, thisy, ILI9341_GREEN);
    lasty = thisy;
  }
}