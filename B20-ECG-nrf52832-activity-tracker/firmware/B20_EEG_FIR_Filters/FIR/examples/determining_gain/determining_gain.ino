// Gain Setting Example
// Demonstrates the filter response with unity input to
// get the appropriate value for the filter gain setting.

#include <FIR.h>

// Make an instance of the FIR filter. In this example we'll use
// floating point values and a 13 element filter.
FIR<long, 13> fir;

void setup() {
  Serial.begin(115200); // Start a serial port

  // Use an online tool to get these such as http://t-filter.engineerjs.com
  // This filter rolls off after 2 Hz for a 10 Hz sampling rate.
  long coef[13] = { 660, 470, -1980, -3830, 504, 10027, 15214,
                    10027, 504, -3830, -1980, 470, 660};
 
  // Set the coefficients
  fir.setFilterCoeffs(coef);

  // The filter automatically determines the gain
  Serial.print("Automatically calculated gain: ");
  Serial.println(fir.getGain());

  // Set the gain to 1 to find the actual gain.
  // After running this sketch you'll see the gain
  // value sould be 26916.
  Serial.println("Setting gain to 1 to show how to manually determine gain");
  long gain = 1;
  fir.setGain(gain);
}

void loop() {
  // Need to run at least the length of the filter.
  for (float i=0; i < 14; i++) {
    Serial.print("Iteration ");
    Serial.print(i+1);
    Serial.print(" -> ");
    Serial.println(fir.processReading(1)); // Input all unity values
  }

  while (true) {}; // Spin forever
}

