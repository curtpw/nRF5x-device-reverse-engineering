// Moving Average Example
// Shows how to use an FIR filter as a moving average on a simple
// set of data that can be easily verified by hand.

#include <FIR.h>

// Make an instance of the FIR filter. In this example we'll use
// floating point values and an 8 element filter. For a moving average
// that means an 8 point moving average.
FIR<float, 8> fir;

void setup() {
  Serial.begin(115200); // Start a serial port

  // For a moving average we want all of the coefficients to be unity.
  float coef[8] = { 1., 1., 1., 1., 1., 1., 1., 1.};

  // Set the coefficients
  fir.setFilterCoeffs(coef);

  Serial.print("Gain set: ");
  Serial.println(fir.getGain());
}

void loop() {
  // Calculate the moving average for a time series with the elements.
  // 0, 1, 2, ...., 13, 14, 15
  for (float i=0; i < 16; i++) {
    Serial.println(fir.processReading(i));
  }

  while (true) {}; // Spin forever
}

