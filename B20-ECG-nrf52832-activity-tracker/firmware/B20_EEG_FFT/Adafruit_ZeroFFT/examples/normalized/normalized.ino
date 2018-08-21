/* This example shows how to calculate an FFT an normalize it.
 * 
 * The signal.h file constains a 200hz sine wave mixed with a weaker 800hz sine wave.
 * The signal was generated at a sample rate of 8000hz.
 */

#include "Adafruit_ZeroFFT.h"
#include "signal.h"

//the signal in signal.h has 2048 values. Set this to a value from 16 to 2048.
//this must be a power of 2
#define DATA_SIZE 1024

//the sample rate
#define FS 8000

//this array will hold the normalized data
float normalized[DATA_SIZE/2];

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  while(!Serial); //wait for serial to be ready

  //run the FFT
  ZeroFFT(signal, DATA_SIZE);

  //get the maximum value
  float maxVal = 0;
  //data is only meaningful up to sample rate/2, discard the other half
  for(int i=0; i<DATA_SIZE/2; i++) if(signal[i] > maxVal) maxVal = signal[i];

  //normalize to the maximum returned value
  for(int i=0; i<DATA_SIZE/2; i++)
    normalized[i] = (float)signal[i] / maxVal;

  for(int i=0; i<DATA_SIZE/2; i++){
    
    //print the frequency
    Serial.print(FFT_BIN(i, FS, DATA_SIZE));
    Serial.print(" Hz: ");

    //print the corresponding FFT output
    Serial.println(normalized[i]);
  }
}

void loop() {
  //don't even do anything
}
