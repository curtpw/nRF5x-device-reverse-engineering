/* This example shows the most basic usage of the Adafruit ZeroFFT library.
 * it calculates the FFT and prints out the results along with their corresponding frequency
 * 
 * The signal.h file constains a 200hz sine wave mixed with a weaker 800hz sine wave.
 * The signal was generated at a sample rate of 8000hz.
 * 
 * Note that you can print only the value (coment out the other two print statements) and use
 * the serial plotter tool to see a graph.
 */

#include "Adafruit_ZeroFFT.h"
#include "signal.h"

//the signal in signal.h has 2048 samples. Set this to a value between 16 and 2048 inclusive.
//this must be a power of 2
#define DATA_SIZE 256

//the sample rate
#define FS 8000

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  while(!Serial); //wait for serial to be ready

  //run the FFT
  ZeroFFT(signal, DATA_SIZE);

  //data is only meaningful up to sample rate/2, discard the other half
  for(int i=0; i<DATA_SIZE/2; i++){
    
    //print the frequency
    Serial.print(FFT_BIN(i, FS, DATA_SIZE));
    Serial.print(" Hz: ");

    //print the corresponding FFT output
    Serial.println(signal[i]);
  }
}

void loop() {
  //don't even do anything
}