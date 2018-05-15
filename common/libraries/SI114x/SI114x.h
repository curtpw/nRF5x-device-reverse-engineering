// SI114.h
// Code for the Modern Device Pulse Sensor
// Based on the SI1143 chip
// Heavily updated by Toby Corkindale to use the Wire library,
// amongst other changes. February 2016.
// https://github.com/TJC
// Original version by:
// paul@moderndevice.com 6-27-2012

#ifndef SI114x_h
#define SI114x_h

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>


class SI114x {
public:
    enum {     // register values
        /* 0x00 */        PART_ID, REV_ID, SEQ_ID, INT_CFG,
        /* 0x04 */        IRQ_ENABLE, IRQ_MODE1, IRQ_MODE2, HW_KEY,
        /* 0x08 */        MEAS_RATE, ALS_RATE, PS_RATE, ALS_LOW_TH,
        /* 0x0C */        RESERVED_0C, ALS_HI_TH, RESERVED_0E, PS_LED21,
        /* 0x10 */        PS_LED3, PS1_TH, RESERVED_12, PS2_TH,
        /* 0x14 */        RESERVED_14, PS3_TH, RESERVED_16, PARAM_WR,
        /* 0x18 */        COMMAND, /* gap: 0x19..0x1F */ RESERVED_1F = 0x1F,
        /* 0x20 */        RESPONSE, IRQ_STATUS, ALS_VIS_DATA0, ALS_VIS_DATA1,
        /* 0x24 */        ALS_IR_DATA0, ALS_IR_DATA1, PS1_DATA0, PS1_DATA1,
        /* 0x28 */        PS2_DATA0, PS2_DATA1, PS3_DATA0, PS3_DATA1,
        /* 0x2C */        AUX_DATA0, AUX_DATA1, PARAM_RD, RESERVED_2F,
        /* 0x30 */        CHIP_STAT, /* gap: 0x31..0x3A */ RESERVED_3A = 0x3A,
        /* 0x3B */        ANA_IN_KEY1, ANA_IN_KEY2, ANA_IN_KEY3, ANA_IN_KEY4,
    };

    enum {       // Parmeter RAM values
        // Parameter Offsets
        PARAM_I2C_ADDR     =       0x00,
        PARAM_CH_LIST      =       0x01,
        PARAM_PSLED12_SELECT  =    0x02,
        PARAM_PSLED3_SELECT   =    0x03,
        PARAM_FILTER_EN       =    0x04,
        PARAM_PS_ENCODING     =    0x05,
        PARAM_ALS_ENCODING    =    0x06,
        PARAM_PS1_ADCMUX      =    0x07,
        PARAM_PS2_ADCMUX      =    0x08,
        PARAM_PS3_ADCMUX      =    0x09,
        PARAM_PS_ADC_COUNTER  =    0x0A,
        PARAM_PS_ADC_CLKDIV   =    0x0B,
        PARAM_PS_ADC_GAIN     =    0x0B,
        PARAM_PS_ADC_MISC     =    0x0C,
        PARAM_ALS1_ADC_MUX    =    0x0D,
        PARAM_ALS2_ADC_MUX    =    0x0E,
        PARAM_ALS3_ADC_MUX    =    0x0F,
        PARAM_ALSVIS_ADC_COUNTER = 0x10,
        PARAM_ALSVIS_ADC_CLKDIV =  0x11,
        PARAM_ALSVIS_ADC_GAIN   =  0x11,
        PARAM_ALSVIS_ADC_MISC   =  0x12,
        PARAM_ALS_HYST          =  0x16,
        PARAM_PS_HYST           =  0x17,
        PARAM_PS_HISTORY        =  0x18,
        PARAM_ALS_HISTORY       =  0x19,
        PARAM_ADC_OFFSET        =  0x1A,
        PARAM_SLEEP_CTRL        =  0x1B,
        PARAM_LED_RECOVERY      =  0x1C,
        PARAM_ALSIR_ADC_COUNTER =  0x1D,
        PARAM_ALSIR_ADC_CLKDIV  =  0x1E,
        PARAM_ALSIR_ADC_GAIN    =  0x1E,
        PARAM_ALSIR_ADC_MISC    =  0x1F
    };


    enum{      // Command Register Values
        NOP_cmd     =     B00000000,    // Forces a zero into the RESPONSE register
        RESET_cmd   =     B00000001,    // Performs a software reset of the firmware
        BUSADDR_cmd =     B00000010,    // Modifies I2C address
        PS_FORCE_cmd =    B00000101,    // Forces a single PS measurement
        PSALS_FORCE_cmd = B00000111,    // Forces a single PS and ALS measurement
        PS_PAUSE_cmd   =  B00001001,    // Pauses autonomous PS
        ALS_PAUSE_cmd  =  B00001010,    // Pauses autonomous ALS
        PSALS_PAUSE_cmd = B00001011,    // Pauses PS and ALS
        PS_AUTO_cmd     = B00001101,    // Starts/Restarts an autonomous PS Loop
        ALS_AUTO_cmd    = B00001110,    // Starts/Restarts an autonomous ALS Loop
        PSALS_AUTO_Cmd  = B00001111     // Starts/Restarts autonomous ALS and PS loop
    };

    // The Wire library expects 7 bit addresses with the 8th for indicating R/W

    const uint8_t i2cAddr = 0x5A;

    SI114x();

	/* Roger Clark.
	 * Added init function as Wire.begin can't be called in the constructor on the nRF52 as it seems to crash the code.
	 */
	void init()
	{
		Wire.begin();
	}
    void beginTransmission() const
    { 
		Wire.beginTransmission(i2cAddr); 
	}

    // Note, could optionally take a boolean(false) to say don't release the bus.
    byte endTransmission() const
      { return Wire.endTransmission(); }

    // Note, can also take a 3rd parameter; set false to say don't release the bus.
    void requestData(uint8_t count) const
      { Wire.requestFrom(i2cAddr, count); }

    bool isPresent();
    void initSensor();
    void id();
    byte getReg (byte reg);
    void setReg (byte reg, byte val);
    uint16_t* fetchALSData ();
    void setLEDcurrents(byte LED1, byte LED2, byte LED3);
    void setLEDdrive(byte LED1pulse, byte LED2pulse, byte LED3pulse);
    uint16_t* fetchLedData();
    byte readParam (byte addr);
    void writeParam (byte addr, byte val);

   // variables for output
   unsigned int resp, als_vis, als_ir, aux, more; // keep in this order!
};
#endif
