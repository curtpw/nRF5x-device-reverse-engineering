#include "Adafruit_ZeroFFT.h"
#include "Adafruit_ZeroPDM.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define SAMPLERATE_HZ 22000
#define DECIMATION    64

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

#define NUM_REFERENCE_LINES 6

#define GRAPH_OFFSET 30

#define GRAPH_WIDTH (tft.width() - 3)
#define GRAPH_HEIGHT (tft.height() - GRAPH_OFFSET)

#define GRAPH_MIN (tft.height() - 2)
#define GRAPH_MAX (tft.height() - GRAPH_HEIGHT)
static float xScale;

Adafruit_ZeroPDM pdm = Adafruit_ZeroPDM(1, 4); 
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

int16_t data[DATA_SIZE];

// a windowed sinc filter for 44 khz, 64 samples
uint16_t sincfilter[DECIMATION] = {0, 2, 9, 21, 39, 63, 94, 132, 179, 236, 302, 379, 467, 565, 674, 792, 920, 1055, 1196, 1341, 1487, 1633, 1776, 1913, 2042, 2159, 2263, 2352, 2422, 2474, 2506, 2516, 2506, 2474, 2422, 2352, 2263, 2159, 2042, 1913, 1776, 1633, 1487, 1341, 1196, 1055, 920, 792, 674, 565, 467, 379, 302, 236, 179, 132, 94, 63, 39, 21, 9, 2, 0, 0};

// a manual loop-unroller!
#define ADAPDM_REPEAT_LOOP_16(X) X X X X X X X X X X X X X X X X

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  delay(10);
  //while(!Serial);
  Serial.println("FFT with TFT featherwing!");

  // Initialize the PDM/I2S receiver
  if (!pdm.begin()) {
    Serial.println("Failed to initialize I2S/PDM!");
    while (1);
  }

  // Configure PDM receiver, sample rate
  if (!pdm.configure(SAMPLERATE_HZ * DECIMATION / 16, true)) {
    Serial.println("Failed to configure PDM");
    while (1);
  }
  Serial.println("PDM configured");
  
  tft.begin();
  tft.setRotation(1);
}

void drawReference(){
  //draw some reference lines
  uint16_t refStep = DATA_SIZE / 2 / NUM_REFERENCE_LINES;
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_RED);
  for(int i=0; i<DATA_SIZE/2 - refStep; i+=refStep){
    uint16_t fc = FFT_BIN(i, SAMPLERATE_HZ, DATA_SIZE);
    uint16_t xpos = map(i, 0, DATA_SIZE/2, 0, GRAPH_WIDTH);
    tft.setCursor(xpos, 0);
    tft.drawLine(xpos, GRAPH_MIN, xpos, GRAPH_MAX, ILI9341_RED);
    
    tft.print(fc);
  }
}

void loop() {
  int32_t avg = 0;
  for(int i=0; i<DATA_SIZE; i++){
    uint16_t runningsum = 0;
    uint16_t *sinc_ptr = sincfilter;
    
    for (uint8_t samplenum=0; samplenum < (DECIMATION/16) ; samplenum++) {
       uint16_t sample = pdm.read() & 0xFFFF;    // we read 16 bits at a time, by default the low half
    
       ADAPDM_REPEAT_LOOP_16(      // manually unroll loop: for (int8_t b=0; b<16; b++) 
         {
           // start at the LSB which is the 'first' bit to come down the line, chronologically 
           // (Note we had to set I2S_SERCTRL_BITREV to get this to work, but saves us time!)
           if (sample & 0x1) {
             runningsum += *sinc_ptr;     // do the convolution
           }
           sinc_ptr++;
           sample >>= 1;
        }
      )
    }

    avg += runningsum;
    data[i] = runningsum;
  }

  //remove DC offset
  avg = avg/DATA_SIZE;
  for(int i=0; i<DATA_SIZE; i++) data[i] = (data[i] - avg);
    
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

  //add a bit of delay because we redraw the screen each time and it gets a bit blinky
  delay(250);
}
