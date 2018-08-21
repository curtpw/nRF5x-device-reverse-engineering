/* This example shows how to use the FFT library with a circuit playground express.
 *  
 *  The LEDs will map around the circle when frequencies between FREQ_MIN and FREQ_MAX are detected
 */

#include <Adafruit_CircuitPlayground.h>
#include "Adafruit_ZeroFFT.h"

//this must be a power of 2
#define DATA_SIZE 256

#define NUM_PIXELS 12

//the sample rate
#define FS 22000

//the lowest frequency that will register on the meter
#define FREQ_MIN 600

//the highest frequency that will register on the meter
#define FREQ_MAX 3000

#define MIN_INDEX FFT_INDEX(FREQ_MIN, FS, DATA_SIZE)
#define MAX_INDEX FFT_INDEX(FREQ_MAX, FS, DATA_SIZE)

#define SCALE_FACTOR 32

int16_t pixelData[NUM_PIXELS];
int16_t inputData[DATA_SIZE];

// the setup routine runs once when you press reset:
void setup() {
  CircuitPlayground.begin();
}

void loop() {
  CircuitPlayground.mic.capture(inputData, DATA_SIZE);

  /*******************************
   *   REMOVE DC OFFSET
   ******************************/
  int32_t avg = 0;
  int16_t *ptr = inputData;
  for(int i=0; i<DATA_SIZE; i++) avg += *ptr++;
  avg = avg/DATA_SIZE;

  ptr = inputData;
  for(int i=0; i<DATA_SIZE; i++){
    *ptr -= avg;
    *ptr++ = *ptr*SCALE_FACTOR;
  }
  
  //run the FFT
  ZeroFFT(inputData, DATA_SIZE);
  
  //set all to 0
  memset(pixelData, 0, NUM_PIXELS*sizeof(int16_t));

  //downsample into NUM_PIXELS bins
  for(int i=MIN_INDEX; i<MAX_INDEX; i++){
    int ix = map(i, MIN_INDEX, MAX_INDEX, 0, NUM_PIXELS);
    pixelData[ix] += inputData[i];
  }

  //display the data
  for(int i=0; i<NUM_PIXELS; i++)
    CircuitPlayground.strip.setPixelColor(i, Wheel(pixelData[i]));

  CircuitPlayground.strip.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
// values below 20 will not register
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 20) {
    return CircuitPlayground.strip.Color(0, 0, 0);
  } 
  else if(WheelPos < 85) {
    return CircuitPlayground.strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return CircuitPlayground.strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return CircuitPlayground.strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
