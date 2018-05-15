/* SI114_pulse_PSO2.ino 
 * A version of SI114_Pulse_Demo.ino with PSO2 sensing added
 * demo code for the Modern Device SI1143-based pulse sensor
 * http://moderndevice.com/product/pulse-heartbeat-sensor-pulse-sensor-1x/
 * Paul Badger 2013 with plenty of coding help from Jean-Claude Wippler
 * Modified by Toby Corkindale Feb 2016 to use Wire library.
 * Hardware setup - please read the chart below and set the appropriate options
 * See the #defines for various printing options
 * This is experimental software (and pre-beta) at that, it is not suitable 
 * for any particular purpose. No life-critical devices should
 * be based on this software.
 * Code in the public domain but credit for the software is nice :)
 */
 
 #include <SI114x.h>

/*
 For Arduino users, use the SDA and SCL pins on your controller.
 For Teensy 3.x/LC users, likewise.
 Typically pin 18 is SCL, and 19 is SDA.

 The original docs here said to use 10k resisters in series. (Why?)
 I note that you should additionally have 5k pull-up resistors going to a 3V3 source.


 JeeNode users just set the appropriate port

 JeeNode Port  SCL ('duino pin)  SDA ('duino pin)
 0             18 (A5)       19 (A4)
 1             4             14 (A0)
 2             5             15 (A1)
 3             6             16 (A2)
 4             7             17 (A3)
 */


const int SAMPLES_TO_AVERAGE = 5;             // samples for smoothing 1 to 10 seem useful 5 is default
// increase for smoother waveform (with less resolution - slower!) 


//#define SEND_TOTAL_TO_PROCESSING   // Use this option exclusive of other options
                                      // for sending data to HeartbeatGraph in Processing
// #define POWERLINE_SAMPLING         // samples on an integral of a power line period [eg 1/60 sec]
#define AMBIENT_LIGHT_SAMPLING     // also samples ambient slight (slightly slower)
// #define PRINT_LED_VALS             // print LED raw values
#define GET_PULSE_READING            // prints HB, signal size, PSO2 ratio


int binOut;     // 1 or 0 depending on state of heartbeat
int BPM;
unsigned long red;        // read value from visible red LED
unsigned long IR1;        // read value from infrared LED1
unsigned long IR2;       // read value from infrared LED2
unsigned long IR_total;     // IR LED reads added together


SI114x pulse; 

void setup () {
    Serial.begin(57600);
    for (int i = 0; i < 5; i++) {
      Serial.println("Pulse monitor");
      delay(1000);
    }

    if (pulse.isPresent()) {
        Serial.println("SI114x Pulse Sensor found");
        pulse.id();
    }
    else {
      while (1) {
        Serial.println("No SI114x found");
        delay(1000);
      }
    }

    pulse.initSensor();
}


void loop(){
    readPulseSensor();
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

    static int foundNewFinger, red_signalSize, red_smoothValley;
    static long red_valley, red_Peak, red_smoothRedPeak, red_smoothRedValley, 
               red_HFoutput, red_smoothPeak; // for PSO2 calc
    static  int IR_valley=0, IR_peak=0, IR_smoothPeak, IR_smoothValley, binOut, lastBinOut, BPM;
    static unsigned long lastTotal, lastMillis, IRtotal, valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime=millis(), lastBeat, beat;
    static float IR_baseline, red_baseline, IR_HFoutput, IR_HFoutput2, shiftedOutput, LFoutput, hysterisis;

    unsigned long total=0, start;
    int i=0;
    int IR_signalSize;
    red = 0;
    IR1 = 0;
    IR2 = 0;
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
     IR1 += ledValues[1];
     IR2 += ledValues[2];
     i++;
     }
     
    red = red / i;  // get averages
    IR1 = IR1 / i;
    IR2 = IR2 / i;
    total =  IR1 + IR2 + red;  // red excluded
    IRtotal = IR1 + IR2;
    
   

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
    Serial.print(IR1);
    Serial.print("\t");
    Serial.print(IR2);
    Serial.print("\t");
    Serial.println((long)total);   

#endif

 #ifdef SEND_TOTAL_TO_PROCESSING
    Serial.println(total);
 #endif

 #ifdef GET_PULSE_READING

    // except this one for Processing heartbeat monitor
    // comment out all the bottom print lines

    if (lastTotal < 20000L && total > 20000L) foundNewFinger = 1;  // found new finger!

    lastTotal = total;
     
    // if found a new finger prime filters first 20 times through the loop
    if (++foundNewFinger > 25) foundNewFinger = 25;   // prevent rollover 

    if ( foundNewFinger < 20){
        IR_baseline = total - 200;   // take a guess at the baseline to prime smooth filter
   Serial.println("found new finger");     
    }
    
    else if(total > 20000L) {    // main running function
    
    
        // baseline is the moving average of the signal - the middle of the waveform
        // the idea here is to keep track of a high frequency signal, HFoutput and a 
        // low frequency signal, LFoutput
        // The LF signal is shifted downward slightly downward (heartbeats are negative peaks)
        // The high freq signal has some hysterisis added. 
        // When the HF signal crosses the shifted LF signal (on a downward slope), 
        // we have found a heartbeat.
        IR_baseline = smooth(IRtotal, 0.99, IR_baseline);   // 
        IR_HFoutput = smooth((IRtotal - IR_baseline), 0.2, IR_HFoutput);    // recycling output - filter to slow down response
        
        red_baseline = smooth(red, 0.99, red_baseline); 
        red_HFoutput = smooth((red - red_HFoutput), 0.2, red_HFoutput);
        
        // beat detection is performed only on the IR channel so 
        // fewer red variables are needed
        
        IR_HFoutput2 = IR_HFoutput + hysterisis;     
        LFoutput = smooth((IRtotal - IR_baseline), 0.95, LFoutput);
        // heartbeat signal is inverted - we are looking for negative peaks
        shiftedOutput = LFoutput - (IR_signalSize * .05);

        if (IR_HFoutput  > IR_peak) IR_peak = IR_HFoutput; 
        if (red_HFoutput  > red_Peak) red_Peak = red_HFoutput;
        
        // default reset - only if reset fails to occur for 1800 ms
        if (millis() - lastPeakTime > 1800){  // reset peak detector slower than lowest human HB
            IR_smoothPeak =  smooth((float)IR_peak, 0.6, (float)IR_smoothPeak);  // smooth peaks
            IR_peak = 0;
            
            red_smoothPeak =  smooth((float)red_Peak, 0.6, (float)red_smoothPeak);  // smooth peaks
            red_Peak = 0;
            
            lastPeakTime = millis();
        }

        if (IR_HFoutput  < IR_valley)   IR_valley = IR_HFoutput;
        if (red_HFoutput  < red_valley)   red_valley = red_HFoutput;
        
  /*      if (IR_valley < -1500){
            IR_valley = -1500;  // ditto above
            Serial.println("-1500");
        } 
        if (red_valley < -1500) red_valley = -1500;  // ditto above  */



        if (millis() - lastValleyTime > 1800){  // insure reset slower than lowest human HB
            IR_smoothValley =  smooth((float)IR_valley, 0.6, (float)IR_smoothValley);  // smooth valleys
            IR_valley = 0;
            lastValleyTime = millis();           
        }

   //     IR_signalSize = IR_smoothPeak - IR_smoothValley;  // this the size of the smoothed HF heartbeat signal
        hysterisis = constrain((IR_signalSize / 15), 35, 120) ;  // you might want to divide by smaller number
                                                                // if you start getting "double bumps"
            
        // Serial.print(" T  ");
        // Serial.print(IR_signalSize); 

        if  (IR_HFoutput2 < shiftedOutput){
            // found a beat - pulses are valleys
            lastBinOut = binOut;
            binOut = 1;
         //   Serial.println("\t1");
            hysterisis = -hysterisis;
            IR_smoothValley =  smooth((float)IR_valley, 0.99, (float)IR_smoothValley);  // smooth valleys
            IR_signalSize = IR_smoothPeak - IR_smoothValley;
            IR_valley = 0x7FFF;
            
            red_smoothValley =  smooth((float)red_valley, 0.99, (float)red_smoothValley);  // smooth valleys
            red_signalSize = red_smoothPeak - red_smoothValley;
            red_valley = 0x7FFF;
            
            lastValleyTime = millis();
             
        } 
        else{
         //   Serial.println("\t0");
            lastBinOut = binOut;
            binOut = 0;
            IR_smoothPeak =  smooth((float)IR_peak, 0.99, (float)IR_smoothPeak);  // smooth peaks
            IR_peak = 0;
            
            red_smoothPeak =  smooth((float)red_Peak, 0.99, (float)red_smoothPeak);  // smooth peaks
            red_Peak = 0;
            lastPeakTime = millis();      
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
            Serial.print("\t IR ");
            Serial.print(IR_signalSize);
            Serial.print("\t PSO2 ");         
            Serial.println(((float)red_baseline / (float)(IR_baseline/2)), 3);                     
        }

    }
 #endif
}



