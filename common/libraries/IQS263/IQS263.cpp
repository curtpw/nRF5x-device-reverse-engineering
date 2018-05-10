/*******************************************************************************
 *
 *		Azoteq IQS263_Example_Code
 *
 *              IQS263_Handler - IQS263 device settings and events
 *
*******************************************************************************/
#include "Arduino.h"
#include "iqs263.h"







/*************************** IQS263 DEVICE SETTINGS ***************************/

// constructor
/*
IQS263::IQS263(int sclPin,int sdaPin,int rdyPin):_sclPin(sclPin),_sdaPin(sdaPin),_rdyPin(rdyPin)
{
}
*/
IQS263::IQS263(int rdyPin):_rdyPin(rdyPin)
{
}
/*

// destructor
IQS263::~IQS263()
{
}
*/
int IQS263::init(void)
{
	Wire.begin();
	pinMode(_rdyPin,INPUT);
    // Read the product number
    CommsIQS_start();
    CommsIQS_Read(IQS263_ADDR, DEVICE_INFO, &data_buffer[0], 2);
    CommsIQS_stop();
	if (data_buffer[0]!=0x3C)
	{
		return -2;
	}
//	Serial.print("DEVICE_INFO 0x");	Serial.println(data_buffer[0],HEX);

    // Switch the IQS263 into projection mode - if necessary
    data_buffer[0] = SYSTEM_FLAGS_VAL;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, SYS_FLAGS, &data_buffer[0], 1);
    CommsIQS_stop();

    // Set active channels
    data_buffer[0] = ACTIVE_CHS;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, ACTIVE_CHANNELS, &data_buffer[0], 1);
    CommsIQS_stop();

    // Setup touch and prox thresholds for each channel
    data_buffer[0] = PROX_THRESHOLD;
    data_buffer[1] = TOUCH_THRESHOLD_CH1;
    data_buffer[2] = TOUCH_THRESHOLD_CH2;
    data_buffer[3] = TOUCH_THRESHOLD_CH3;
    data_buffer[4] = MOVEMENT_THRESHOLD;
    data_buffer[5] = RESEED_BLOCK;
    data_buffer[6] = HALT_TIME;
    data_buffer[7] = I2C_TIMEOUT;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, THRESHOLDS, &data_buffer[0], 8);
    CommsIQS_stop();

    // Set the ATI Targets (Target Counts)
    data_buffer[0] = ATI_TARGET_TOUCH;
    data_buffer[1] = ATI_TARGET_PROX;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, TIMINGS_AND_TARGETS, &data_buffer[0], 2);
    CommsIQS_stop();

    // Set the BASE value for each channel
    data_buffer[0] = MULTIPLIERS_CH0;
    data_buffer[1] = MULTIPLIERS_CH1;
    data_buffer[2] = MULTIPLIERS_CH2;
    data_buffer[3] = MULTIPLIERS_CH3;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, MULTIPLIERS, &data_buffer[0], 4);
    CommsIQS_stop();

    // Setup prox settings
    data_buffer[0] = PROXSETTINGS0_VAL;
    data_buffer[1] = PROXSETTINGS1_VAL;
    data_buffer[2] = PROXSETTINGS2_VAL;
    data_buffer[3] = PROXSETTINGS3_VAL;
    data_buffer[4] = EVENT_MASK_VAL;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, PROX_SETTINGS, &data_buffer[0], 5);
    CommsIQS_stop();

    // Setup Compensation (PCC)
    data_buffer[0] = COMPENSATION_CH0;
    data_buffer[1] = COMPENSATION_CH1;
    data_buffer[2] = COMPENSATION_CH2;
    data_buffer[3] = COMPENSATION_CH3;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, COMPENSATION, &data_buffer[0], 4);
    CommsIQS_stop();

    // Set timings on the IQS263
    data_buffer[0] = LOW_POWER;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, TIMINGS_AND_TARGETS, &data_buffer[0], 1);
    CommsIQS_stop();

    // Set gesture timers on IQS263
    data_buffer[0] = TAP_TIMER;
    data_buffer[1] = FLICK_TIMER;
    data_buffer[2] = FLICK_THRESHOLD;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, GESTURE_TIMERS, &data_buffer[0], 3);
    CommsIQS_stop();

    // Redo ati
    data_buffer[0] = 0x10;
    CommsIQS_start();
    CommsIQS_Write(IQS263_ADDR, PROX_SETTINGS, &data_buffer[0], 1);
    CommsIQS_stop();

    // Wait untill the ATI algorithm is done
	int timeoutCount=10;
    do
    {
		delay(10);// Roger Clark. Note this delay does not seem to be needed 
        CommsIQS_start();
        CommsIQS_Read(IQS263_ADDR, SYS_FLAGS, &data_buffer[0], 1);
        CommsIQS_stop();
    }
    while ((data_buffer[0] & 0b00000100) && (timeoutCount-- > 0) );

	if (data_buffer[0] & 0b00000100)
	{
		Serial.println("Timed out");
		return -1;
	}
//	Serial.print("ATI algorithm  SYS_FLAGS = 0x");		Serial.println(data_buffer[0],HEX);	
	
    // read the error bit to determine if ATI error occured
    CommsIQS_start();
    CommsIQS_Read(IQS263_ADDR, PROX_SETTINGS, &data_buffer[0], 2);
    CommsIQS_stop();

    if (data_buffer[1] & 0x02)
	{
		
		return 1;//Serial.println("Automatic Tuning Implementation occured");
    }

	return 0;
}
/*
 * THIS FUNCTION DOES NOT WORK.
 * Its and attempt to fix the issue where we can't read more than one register at a time
 */
void IQS263::handleMultiRead()
 {
	int NoOfBytes;
	int i;
	Serial.println("handleMultiRead");
    CommsIQS_start();   	// Start the communication session
	Serial.println("STARTED");
	Wire.beginTransmission(IQS263_ADDR); 	
    Wire.write(SYS_FLAGS); 	
	Wire.endTransmission(false);     // stop transmitting send restart
	NoOfBytes=2;
	Wire.requestFrom(IQS263_ADDR, NoOfBytes,false);    // if doing mutiple reads the connection needs to be left open
	i=0;

	while(Wire.available() < NoOfBytes)    // slave may send less than requested
	{

		delay(1);
	}
	for(int i=0;i<NoOfBytes;i++)
	{
		data_buffer[i] = Wire.read();    // receive a byte 
	}	
	
	return;
	Serial.println("Got SYS_FLAGS");
	NoOfBytes=2;		
	Wire.write(IQS263_ADDR<<1); 	
    Wire.write(TOUCH_BYTES);
	Wire.endTransmission(false);     // stop transmitting send restart	
	Wire.requestFrom(IQS263_ADDR, NoOfBytes,false);    // if doing mutiple reads the connection needs to be left open
	i=0;
	NoOfBytes=1;
	while(Wire.available() < NoOfBytes)    // slave may send less than requested
	{
		delay(1);
	}
	for(int i=0;i<NoOfBytes;i++)
	{
		data_buffer[i+2] = Wire.read();    // receive a byte 
	}	

 }
 
iqs263EventData IQS263::getEvents(void)
{
	uint8_t events;
	iqs263EventData evt;
	
	/**************************************************************************************************
	 *	NOTE. This function need to be improved
	 *	Currently it reads each of the 3 registers separately, as its the only way I could get it to work
	 *  However, they are supposed to be read in same RDY = LOW, pulse, so the vales relate to the same event
	 *	
	 *	if the CommsIQS_stop(); CommsIQS_start(); between each read is removed, only data in the SYS_FLAGS
	 *  is valid, the other two registers return 0xff 
	 *
	 *	The original code uses some sort of repeat start system to prevent a STOP bit being sent as this causes
	 *  the device to do its own processing and not be available for further I2C comms until RDY is LOW again. 
	 *
	 * Update 2017/07/15	The problem appears to be caused buy a bug in the Wire library where 1 byte of additional
	 * clock pulses are sent (8 clock pulses). This causes the IQS263 to return 0xFF and sometimes a NAK as this read 
	 * is for 1 more bytes than in the register
	 * I've posted an issue to Sandeep's repo about this
	 * https://github.com/sandeepmistry/arduino-nRF5/issues/176
	 *
	 ***************************************************************************************************/
	

    CommsIQS_start();   	// Start the communication session
    CommsIQS_Read(IQS263_ADDR, SYS_FLAGS, &data_buffer[0], 2);                  // Read the system flags register to enable all events
	evt.eventFlags=data_buffer[1];
	
	CommsIQS_stop(); // Workaround for problem in Wire lib
	CommsIQS_start();// Workaround for problem in Wire lib

	CommsIQS_Read(IQS263_ADDR, TOUCH_BYTES, &data_buffer[2], 1);             // Read from the touch bytes register to enable touch events
	evt.touchFlags=data_buffer[2];

	CommsIQS_stop(); // Workaround for problem in Wire lib
	CommsIQS_start();// Workaround for problem in Wire lib

    CommsIQS_Read(IQS263_ADDR, COORDINATES, &data_buffer[3], 3);                // Read the coordinates register to get slider coordinates
	evt.wheelPos=data_buffer[3];
	evt.relativePos = (int16_t)(data_buffer[4]  | data_buffer[5]<<8);	
	CommsIQS_stop();                                                            // Stop the communication session
	
    events = data_buffer[1];
	
	return evt;

	
    /******************************* PROXIMITY ********************************/

    if (events & 0x01)
	{
        proxEvent();
	}
    /******************************** TOUCH ***********************************/
    if (events & 0x02)
    {
		touchEvent();
	}
	
    /******************************** SLIDE ***********************************/

    if (events & 0x04)
    {
		slideEvent();
	}
    /******************************* MOVEMENT *********************************/

    if (events & 0x10)
    {
		movementEvent();
	}
	else
	{
		//Serial.println("ISQ263 - movement flag not set");
		// NOT SURE WHAT THIS CONDITION MEANS
		// Perhaps movement has stopped ???
	}
    /********************************* TAP ************************************/

    if(events & 0x20)
	{
        tapEvent();
	}
    /******************************* FLICK (LEFT) *****************************/

    if(events & 0x40)
    {
		flickLeft();
	}
    /******************************* FLICK (RIGHT) ****************************/

    if(events & 0x80)
    {
		flickRight();
	}
}

/*******************************************************************************
 *      This function handles touch events. If a touch event occurs
 ******************************************************************************/
void IQS263::touchEvent(void)
{
    touch0 = data_buffer[2];
    if (touch0 != 0)
    {
        /* CHANNEL 1*/
        if (touch0 & 0x02)                  // If a touch event occurs on Channel 1
        {
           Serial.println("Touch event 1 ON");  
        }
        else
        {
            Serial.println("Touch event 1 OFF");  
        }

        /* CHANNEL 2 */
        if (touch0 & 0x04)                    // If a touch event occurs on Channel 2
        {
            Serial.println("Touch event 2 ON");  
        }
        else
        {
           Serial.println("Touch event 2 OFF");  
        }

        /* CHANNEL 3 */
        if (touch0 & 0x08)                   // If a touch event occurs on Channel 3
        {
            Serial.println("Touch event 3 ON");  
        }
        else
        {
           Serial.println("Touch event 3 OFF"); 
        }
    }
}
/*******************************************************************************
 *      This function handles a slide event, if a slide event is triggered
 *      Showing the position that the finger of the
 *      user is currently on including the coordinate value thereof.
 ******************************************************************************/
void IQS263::slideEvent(void)
{
    sliderCoords = data_buffer[3];
    touch0 = data_buffer[2];

    if (touch0 != 0)
    {
        if ((sliderCoords > 0 && sliderCoords < 85)&& (touch0 & 0x02))
        {
			Serial.println("Slide event 1");
        }
        else if ((sliderCoords > 0 && sliderCoords < 170)&& (touch0 & 0x04) )
        {
			Serial.println("Slide event 2");
        }
        else if ((sliderCoords > 170 && sliderCoords < 255)&& (touch0 & 0x08))
        {
			Serial.println("Slide event 3");
        }
        else
        {
			Serial.println("slide event 'else' condition");
        }
    }
}

/*******************************************************************************
 *      This function handles a prox event
 ******************************************************************************/
void IQS263::proxEvent(void)
{
    prox = data_buffer[2];

    if (prox & 0x01)              // If a prox event occures
    {
		Serial.println("Proximity event");
    }
    else
    {
		Serial.println("Proximity event 'else' condition");
    }
}

/*******************************************************************************
 *      This function handles a movement event, each time movement is
 *      sensed
 ******************************************************************************/
void IQS263::movementEvent(void)
{
    movement = data_buffer[1];

    if (movement & 0x10)          // If a movement event occurs
    {
		Serial.println("Movement event");
    }
    else
    {
		Serial.println("Movement event 'else' condition");
    }
}

/*******************************************************************************
 *      This function handles a tap event, each time this event is
 *      triggered
 ******************************************************************************/
void IQS263::tapEvent(void)
{
    uint8_t i;
    tap = data_buffer[1];

    if (tap & 0x20)              // If a tap event occurs
    {
		Serial.println("Tap event");
		delay(300);// Not sure why it needs a delay
    }
}

/*******************************************************************************
 *     This function handles a right flick event, each time this event is
 *     is triggered
 ******************************************************************************/
void IQS263::flickRight(void)
{
    uint8_t i;
    rightFlick = data_buffer[1];

    if (rightFlick & 0x80)      // If a right flick event occurs
    {
        tap = 0x00;                     // avoid tap event coliding with flicker event		
		Serial.println("Flick right");
    }
    else
    {
		Serial.println("Flick right 'else' condition");
    }
}

/*******************************************************************************
 *     This function handles a left flick event, each time this event is
 *     is triggered
 ******************************************************************************/
void IQS263::flickLeft(void)
{
    uint8_t i;
    leftFlick = data_buffer[1];

    if (leftFlick & 0x40)        // If a left click event occurs
    {
        tap = 0x00;                     // avoid tap event coliding with flicker event		
		Serial.println("Flick left");
    }
    else
    {
		Serial.println("Flick left 'else' condition");
    }
}


void IQS263::CommsIQS_Read(unsigned char i2c_addr, unsigned char read_addr, unsigned char *data, unsigned char NoOfBytes,bool leaveConnectionOpen)
{
    unsigned char i;
	
	Wire.beginTransmission(i2c_addr); 	
    Wire.write(read_addr); 	
	Wire.endTransmission(false);     // stop transmitting send restart
	Wire.requestFrom(i2c_addr, NoOfBytes,!leaveConnectionOpen);    // if doing mutiple reads the connection needs to be left open
	i=0;
	while(Wire.available() < NoOfBytes)    // slave may send less than requested
	{
		delay(1);
	}
	for(int i=0;i<NoOfBytes;i++)
	{
		data[i] = Wire.read();    // receive a byte 
	}
}

void IQS263::CommsIQS_Write(unsigned char i2c_addr, unsigned char write_addr, unsigned char *data, unsigned char NoOfBytes)
{
    unsigned char i;

	Wire.beginTransmission(i2c_addr); 
    Wire.write(write_addr);    
	for (i = 0; i < NoOfBytes; i++)
	{
		Wire.write(data[i]); 	
	}
	Wire.endTransmission();     // stop transmitting
}


void IQS263::CommsIQS_stop(void)
{

    //wait for the IQS device to become ready
    while(digitalRead(_rdyPin))                       //wait for the IQS to change from ready to not ready
    {}
    while(!digitalRead(_rdyPin))                       // wait for the IQS to change from not ready to ready
    {}
    //wait for IQS device to become ready
}

void IQS263::CommsIQS_start(void)
{
    // IQ263 starts in full streaming mode
    while(digitalRead(_rdyPin))                       //wait for the IQS to change from ready to not ready
    {}
}
