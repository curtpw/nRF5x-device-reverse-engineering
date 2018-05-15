// HeartbeatGraphFromArduino
// Graphs data from the SI1143 based pulse / PSO2 sensor
// The data can be either integer or float format (I think :) ), but must be followed by a carriage return.
// Note 57600 baud, Serial port set to 0 where I usually see MY BBB or JeeNode :)
// This code is a collage of some of the Arduino Communication examples. Too bad Arduino won't include some more mo better examples ;)
// There is code in here for the Modern Device Pulse Sensor that many people may not want - it's kind of like an AC coupled oscilloscope trace
// It might be handy for some folks. 
// There are still a few drawing bugs.

// This example code is in the public domain.
// Paul Badger 2012 

import processing.serial.*;
import processing.serial.*;

Serial myPort;            // The serial port
int xPos = 1;             // horizontal position of the graph
float lastXPos = 1;
float lastYPos, lastYPosIR;
float smoothData, rawData, smoothDataIR, rawDataIR;
int drawing = 1;
float data, lastPeak = 600;
float lastValleyReset, lastPeakReset, peak = 600, valley = 0, lastValley  =  0, lastXPeak, lastPeakPos =0;
float peak2, lastPeak2, valley2, lastValley2; 
float lastPeakResetTime = millis(), lastValleyResetTime = millis(), HBflag = 0;
boolean peakCycleFlag = false, valleyCycleFlag = false;
float level, lastData, lastThresh, redVal, lastRedVal, IRval, lastIRval;   // variables for heartbeat function
PFont myFont;
int time, lastTime, hrvCounter;
int HRVoffset = 400;



void setup() {
  size(1200, 800);

  // List all the available serial ports
  println(Serial.list());
  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[0], 57600);
  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');


  background(0);


  // Uncomment the following two lines to see the available fonts 
  String[] fontList = PFont.list();
  println(fontList);
  myFont = createFont("Helvetica-Light", 12);
  textFont(myFont);
  text("Begin", 6, 20);
}

void draw() {
  // serial drives everything
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    // convert to an int and map to the screen height:


    // split the string on the commas and convert the 
    // resulting substrings into an integer array:
    float[] data = float(split(inString, ","));
    // if the array has at least three elements, you know
    // you got the whole thing.  Put the numbers in the
    // color variables:
    if (data.length >=1) {
      redVal = data[0];
      //   IRval = data[1];
    }


    // there appears to be some issue in Processing with receiving serial data from Arduino at 57600 badu
    // some rudimentary attempts at filtering it were not really sucessful
    // below is code left from the effort

    float iterVariance = (abs(lastRedVal - redVal) / redVal);
    if ( iterVariance > .03 ) {
      print("\t\t\t");
      println(iterVariance);
    }

    lastRedVal = redVal;

    smoothData = smooth( redVal, .97, smoothData);    // note the recirculation of smoothData, won't work without it
    rawData = smooth( redVal, .3, rawData);

    smoothDataIR = smooth( IRval, .97, smoothDataIR);    // note the recirculation of smoothData, won't work without it
    rawDataIR = smooth( IRval, .3, rawDataIR);


    //    println(redVal);         // for debugging input
    //    print("    ");
    //    println(IRval);


    /*
//  // smoothedPeak = smooth( peak, .8, smoothedPeak); 
     print(data);
     print("\t");
     print(smoothData); 
     print("\t");
     print(peak);
     print("\t");
     print(lastPeak);
     print("\t");
     print(valley);
     print("\t");
     println(lastValley);
     */


    // subtract the movign baseline to keep the waveform centered in a DC sense
    redVal = ((redVal - smoothData) / 2) + 350 ;
    IRval = ((IRval - smoothDataIR) / 2) + 350 ;

    // limit the waveform to the window
    redVal = constrain((int)redVal, 0, (int)600);
    IRval = constrain((int)IRval, 0, (int)600);


    getPeakAndFloor((int) redVal);
    getHeartBeatAndDraw((int) redVal);

    if (drawing == 1) {

      redVal = constrain(redVal, 0.0, 600.0);

      // draw the line:
      stroke(255, 120, 0, 180);
      line(lastXPos, lastYPos, xPos, redVal);

      //       stroke(255, 0, 180, 180);                     // second channel for Infrared data
      //  line(lastXPos, lastYPosIR, xPos, IRval);

      lastXPos = xPos;
      lastYPos = redVal;


      lastYPosIR = IRval;

      // at the edge of the screen, go back to the beginning reset screen
      if (xPos >= width) {
        xPos = 1;
        lastXPos = 1;
        // background(0);
      } 
      else {
        // increment the horizontal position:
        xPos +=1;
      }
    }
  }
}


int smooth(float data, float filterVal, float smoothedVal) {

  if (filterVal > 1) {      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0) {
    filterVal = 0;
  }
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
  return (int)smoothedVal;
}


// peakAndFloor looks for waveform peaks and floors to set trigger levels for the next heartbeat

void  getPeakAndFloor(int dataP) {
  // println(dataP);

  peak = constrain(peak, 0, 600);      // keep things in range
  valley = constrain(valley, 0, 600); 

  if (dataP < peak) {     // found a new peak
    peak = (int)dataP;    // peaks on Processing are SMALLER numbers - standard CG coordinate system (2nd quadrant)
    lastPeakResetTime = millis();
  }

  else if ( peak < ( lastPeak2 - 10 ))  peak = lastPeak2;

  peak += .4 ;    // bleed off the peak detector


  peak2 = constrain(peak2, 0, 600);      // keep things in range
  valley2 = constrain(valley2, 0, 600); 

  if (dataP < peak2) {     // found a new peak
    peak2 = (int)dataP;    // peaks on Processing are SMALLER numbers - standard CG coordinate system (2nd quadrant)
  }

  if ( (valley2) > 600 ) valley2 = 600;       // keep things in range

  if (dataP > valley2) {     // found a new valley
    valley2 = (int)dataP;    // valleys on Processing are SMALLER numbers - standard CG coordinate system (2nd quadrant)
    lastValley2 = valley2;
    lastValleyResetTime = millis();
  }


  if (millis() - lastValleyResetTime > 3000) {
    peak = 600;
    valley = 0;
    lastValleyResetTime = millis();
  }




  if (dataP > valley) {        // found a new peak
    valley = (int)dataP;       // peaks on Processing are SMALLER numbers - standard CG coordinate system (2nd quadrant)
    if (valley2 > lastValley2 + 10) valley2 = lastValley2;
    lastValleyResetTime = millis();
  }

  valley -= 1.0;    // bleed off the peak detector

  if (drawing == 1) {

    peak = constrain(peak, 0, 600);      // keep things in range
    valley = constrain(valley, 0, 600); 
    lastPeak2 = constrain(lastPeak2, 0, 600);      // keep things in range


    // draw the A:
    stroke(200, 255, 0);
    line(lastXPos, lastPeak, xPos, peak);

    stroke(0, 255, 255);
    line(lastXPos, lastValley, xPos, valley);

    //   stroke(100, 255, 100);
    //   line(lastXPos, lastPeak2, xPos, lastPeak2);


    lastPeak = peak;
    lastValley = valley;


    // at the edge of the screen, go back to the beginning reset screen
    if (xPos == 1) {
      lastPeak = 1;
      lastValley = 1;
      fill(0);
      rect(0, 0, width, 600);
    }
  }
}  

void getHeartBeatAndDraw(int data) {
  float percent = .75;    // percent is the trigger for sensing a heartbeat - percent of the way from the last valley to the last peak

  float thresh = (peak * percent) + (valley * (1 - percent ));   // thresh is percent of the way from the last valley to the last peak

  if (drawing == 1) {

    if (data < thresh && HBflag == 0  && ((millis() - lastTime) > 280 )) {
      //   println("vertical line");
      time = millis();
      lastValleyResetTime = time;                    // for peak / valley reset
      stroke(255, 0, 0, 100.0);
      line(xPos, 40, xPos, lastValley);              // draws the feint red heartbeat lines
      fill(255);
      text(str(time - lastTime), xPos + 2, 20);      // draws the ms data label
      float elapsed = (float)(time - lastTime);
      float bpm = (1000 /  elapsed) * 60;
      text(str((int)bpm), xPos + 2, 40);             // draws the bpm data label
      doHRVgraph();     
      HBflag = 1;
      lastTime = time;
    }
  }

  if (data > thresh && HBflag == 1 && ((millis() - lastTime) > 280 ) ) {     // reset the flag w a delay to avoid false triggers
    HBflag = 0;
  }

  lastData = data;
  lastThresh = thresh;

  // at the edge of the screen, go back to the beginning reset screen
  if (xPos == 1) {
    lastThresh = 1;
  }
}



void   doHRVgraph() {
  stroke(0);

  if (drawing == 1) {
    fill(255, 255, 100);
    int rectHeight = height - (((time - lastTime) / 2 ) - (HRVoffset - 100)) ;
    rectHeight = constrain(rectHeight, 600, height);
    rect( hrvCounter, rectHeight, 3, rectHeight);      //

    hrvCounter += 3;

    // reset bottom rectangle
    if (hrvCounter > width - 2) {
      fill(0);
      rect(0, 600, width, height);
      hrvCounter = 0;
    }
  }
}



void mousePressed() {
  if (drawing == 1) {
    drawing = 0;                   // pause drawing if drawing
    delay(10);
    mouseX = -1;
  }
  else  if (drawing == 0 && mouseY > .5 * height ) {      // reset the screen if clicked in the bottom of screen
    drawing = -1;                                           

    // reset the top drawing area
    lastPeak = 1;
    lastValley = 1;
    fill(0);
    rect(0, 0, width, 600);
    xPos = 1;
    lastXPos = 1;

    // reset the graph
    hrvCounter = 0;
    rect(0, 600, width, height);
  }

  else  if (drawing == 0 && mouseY < .5 * height ) {       // click top of screen to resume drawing from pause
    drawing = 1;
  }

  else if (drawing == -1 ) {                              // if stopped start drawing
    drawing = 1;
  }
}

