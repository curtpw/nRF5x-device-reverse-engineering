// Multiple Filters Example
// Show how to make multiple filter objects in a single sketch.

#include <FIR.h>

// Make two instances of the FIR filter. The first is a 13 element float
// and the second is a 10 point float. The 13 element filter is a 2 Hz low-pass
// for a 10 SPS signal and the 10 element is a moving average over one second.

FIR<float, 13> fir_lp;
FIR<float, 10> fir_avg;

// Some data that is a 0.2 Hz sine wave with a 0.3 Hz sine wave noise on it.
float data[51] = { 0.2, -0.01757378,  0.25261   ,  0.50501472,  0.28169386,
        0.73591371,  0.67214444,  0.63916107,  1.04340099,  0.75089683,
        0.97104406,  1.1070092 ,  0.79944834,  1.15681282,  0.95424177,
        0.83291634,  1.1026499 ,  0.68138434,  0.80743739,  0.79725188,
        0.39288974,  0.65097057,  0.32531128,  0.14508086,  0.32099195,
       -0.17069916, -0.07172355, -0.14896219, -0.55850617, -0.30387694,
       -0.64596943, -0.77404044, -0.57958674, -1.0231335 , -0.8365037 ,
       -0.86670651, -1.16875508, -0.81455928, -1.07313216, -1.05899594,
       -0.76797161, -1.09233578, -0.76336743, -0.70354745, -0.86712553,
       -0.40092413, -0.57401086, -0.4319664 , -0.07473948, -0.32007654,
        0.0936594};

void setup() {
  Serial.begin(115200); // Start a serial port

  // Use an online tool to get these such as http://t-filter.engineerjs.com
  // This filter rolls off after 2 Hz for a 10 Hz sampling rate.
  float coef_lp[13] = { 660, 470, -1980, -3830, 504, 10027, 15214,
                    10027, 504, -3830, -1980, 470, 660};

  // For a moving average we use all ones as coefficients.
  float coef_avg[10] = {1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};

  // Set the coefficients
  fir_lp.setFilterCoeffs(coef_lp);
  fir_avg.setFilterCoeffs(coef_avg);

  // Set the gain
  Serial.print("Low Pass Filter Gain: ");
  Serial.println(fir_lp.getGain());
  Serial.print("Moving Average Filter Gain: ");
  Serial.println(fir_avg.getGain());
}

void loop() {
  // Run through our simulated data in "real time" and compare the outputs.
  // You can paste the output in your favorite graphing program if you want
  // to see how the different filters modify the output.

  Serial.println("Input, Moving_Avg, Lowpass");

  for (int i=0; i < 51; i++) {
    Serial.print(data[i]);
    Serial.print(", ");
    Serial.print(fir_avg.processReading(data[i]));
    Serial.print(", ");
    Serial.println(fir_lp.processReading(data[i]));
    delay(100); // Simulate real-time 10 Hz data collection
  }

  while (true) {}; // Spin forever
}
