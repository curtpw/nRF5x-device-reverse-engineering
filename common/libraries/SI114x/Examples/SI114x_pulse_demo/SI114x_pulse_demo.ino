/* SI114_Pulse_Demo.ino
 * demo code for the Modern Device SI1143-based pulse sensor
 * http://moderndevice.com/product/pulse-heartbeat-sensor-pulse-sensor-1x/
 * Paul Badger 2013 with plenty of coding help from Jean-Claude Wippler
 * Modified by Toby Corkindale Feb 2016 to use Wire library.
 * Hardware setup - please read the chart below and set the appropriate options
 */
 
#include <SI114x.h> 
 

const int SAMPLES_TO_AVERAGE = 5;             // samples for smoothing 1 to 10 seem useful 5 is default
// increase for smoother waveform (with less resolution - slower!) 


// #define SEND_TOTAL_TO_PROCESSING      // Use this option exclusive of other options
                                      // for sending data to HeartbeatGraph in Processing
// #define POWERLINE_SAMPLING         // samples on an integral of a power line period [eg 1/60 sec]
#define AMBIENT_LIGHT_SAMPLING     // also samples ambient slight (slightly slower)
#define PRINT_LED_VALS             // print LED raw values
#define GET_PULSE_READING          // prints HB and signal size


int binOut;     // 1 or 0 depending on state of heartbeat
int BPM;
unsigned long red;        // read value from visible red LED
unsigned long _IR1;        // read value from infrared LED1
unsigned long _IR2;       // read value from infrared LED2
unsigned long total;     // all three LED reads added together
int signalSize;          // the heartbeat signal minus the offset

SI114x pulse;
int c=0;
void setup () {

    pulse.init();
    Serial.begin(115200);

    Serial.println("\n Pulse_demo ");

    if (pulse.isPresent())
    {
        Serial.println("SI114x Pulse Sensor found");
    }
    else 
    {
      Serial.println("No SI114x found");
    }

    pulse.initSensor();
}


void loop(){
  
   // Serial.println("loop");
    readPulseSensor();
    //delay(1);
}



// simple smoothing function for  heartbeat detection and processing
float smooth(float data, float filterVal, float smoothedVal){

    if (filterVal > 1){      // check to make sure param's are within range
        filterVal = .99;
    }
    else if (filterVal <= 0.0){
        filterVal = 0.01;
    }

    smoothedVal = (data * (1.0 - filterVal)) + (smoothedVal  *  filterVal);
    return smoothedVal;
}


void readPulseSensor(){

    static int foundNewFinger;
    static  int valley=0, peak=0, smoothPeak, smoothValley, binOut, lastBinOut, BPM;
    static unsigned long lastTotal, lastMillis,  valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime=millis(), lastBeat, beat;
    static float baseline, HFoutput, HFoutput2, shiftedOutput, LFoutput, hysterisis;

    unsigned long total=0, start;
    int i=0;
    int signalSize;
    red = 0;
    _IR1 = 0;
    _IR2 = 0;
    total = 0;

    #ifdef AMBIENT_LIGHT_SAMPLING
    int als_vis, als_ir;
    als_vis = 0;
    als_ir = 0;
    #endif

    start = millis();
         
    
     #ifdef POWERLINE_SAMPLING
     
     while (millis() - start < 16){   // 60 hz - or use 33 for two cycles
     // 50 hz in Europe use 20, or 40
       Serial.print("sample");
     #else     
     while (i < SAMPLES_TO_AVERAGE){      
     #endif


     #ifdef AMBIENT_LIGHT_SAMPLING
     uint16_t* ambientLight = pulse.fetchALSData();
     als_vis += ambientLight[0];
     als_ir += ambientLight[1];
     #endif

     uint16_t* ledValues = pulse.fetchLedData();

     red += ledValues[0];
     _IR1 += ledValues[1];
     _IR2 += ledValues[2];
     i++;
     }
     
    red = red / i;  // get averages
    _IR1 = _IR1 / i;
    _IR2 = _IR2 / i;
    total = red + _IR1 + _IR2;
    
   

#ifdef AMBIENT_LIGHT_SAMPLING

    als_vis = als_vis / i;
    als_ir = als_ir / i;

    Serial.print(als_vis);       //  ambient visible
    Serial.print("\t");
    Serial.print(als_ir);        //  ambient IR
    Serial.print("\t");

#endif


#ifdef PRINT_LED_VALS

    Serial.print(red);
    Serial.print("\t");
    Serial.print(_IR1);
    Serial.print("\t");
    Serial.print(_IR2);
    Serial.print("\t");
    Serial.println((long)total);   

#endif

 #ifdef SEND_TOTAL_TO_PROCESSING
    Serial.println(total);
 #endif

 #ifdef GET_PULSE_READING

    // except this one for Processing heartbeat monitor
    // comment out all the bottom print lines

    if (lastTotal < 20000L && total > 20000L) {
        foundNewFinger = 1;  // found new finger!
        Serial.println("found new finger");
    }

    lastTotal = total;

    // if found a new finger prime filters first 20 times through the loop
    if (++foundNewFinger > 25) foundNewFinger = 25;   // prevent rollover 

    if ( foundNewFinger < 20){
        baseline = total - 200;   // take a guess at the baseline to prime smooth filter
    }
    
    else if(total > 20000L) {    // main running function
    
    
        // baseline is the moving average of the signal - the middle of the waveform
        // the idea here is to keep track of a high frequency signal, HFoutput and a 
        // low frequency signal, LFoutput
        // The HF signal is shifted downward slightly (heartbeats are negative peaks)
        // The high freq signal has some hysterisis added. When the HF signal is less than the 
        // shifted LF signal, we have found a heartbeat.
        baseline = smooth(total, 0.99, baseline);   // 
        HFoutput = smooth((total - baseline), 0.2, HFoutput);    // recycling output - filter to slow down response
        HFoutput2 = HFoutput + hysterisis;     
        LFoutput = smooth((total - baseline), 0.95, LFoutput);
        // heartbeat signal is inverted - we are looking for negative peaks
        shiftedOutput = LFoutput - (signalSize * .05);

        // We need to be able to keep track of peaks and valleys to scale the output for 
        // user convenience. Hysterisis is also scaled.
        if (HFoutput  > peak) peak = HFoutput; 
        if (peak > 1500) peak = 1500; 

        if (millis() - lastPeakTime > 1800){  // reset peak detector slower than lowest human HB
            smoothPeak =  smooth((float)peak, 0.6, (float)smoothPeak);  // smooth peaks
            peak = 0;
            lastPeakTime = millis();
        }

        if (HFoutput  < valley)   valley = HFoutput;
        if (valley < -1500) valley = -1500;

        if (millis() - lastValleyTime > 1800){  // reset valleys detector slower than lowest human HB
            smoothValley =  smooth((float)valley, 0.6, (float)smoothValley);  // smooth valleys
            valley = 0;
            lastValleyTime = millis();           
        }

        signalSize = smoothPeak - smoothValley;  // this the size of the smoothed HF heartbeat signal
        
        // Serial.print(" T  ");
        // Serial.print(signalSize); 

        if(HFoutput2 < shiftedOutput){
            lastBinOut = binOut;
            binOut = 1;
         //   Serial.println("\t1");
            hysterisis = - constrain((signalSize / 15), 35, 120) ;   // you might want to divide by smaller number
            // if you start getting "double bumps"
        } 
        else {
         //   Serial.println("\t0");
            lastBinOut = binOut;
            binOut = 0;
            hysterisis = constrain((signalSize / 15), 35, 120);    // ditto above
            
        } 

  if (lastBinOut == 1 && binOut == 0){
      Serial.println(binOut);
  }

        if (lastBinOut == 0 && binOut == 1){
            lastBeat = beat;
            beat = millis();
            BPM = 60000 / (beat - lastBeat);
            Serial.print(binOut);
            Serial.print("\t BPM ");
            Serial.print(BPM);  
            Serial.print("\t signal size ");
            Serial.println(signalSize);  
        }

    }
 #endif

}


