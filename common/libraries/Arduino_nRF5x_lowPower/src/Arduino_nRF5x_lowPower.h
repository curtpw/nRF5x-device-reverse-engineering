/**************************************************************************/
/*!
    @file		Arduino_nRF5x_lowPower.h
	@author		Marc-Oliver Ristau <marc.ristau@mristau.eu>
	@version	0.1.0
	
	Arduino library for nRF5x lowPower modes
	
	@section HISTORY
	
	v0.1.0	- First Release
*/
/**************************************************************************/

#ifndef ARDUINO_NRF5X_LOWPOWER_H
#define ARDUINO_NRF5X_LOWPOWER_H

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <nrf_sdm.h>
#include <nrf_soc.h>

typedef enum {
    POWER_MODE_OFF              = 0x01, // shuts down the nRF5x
    POWER_MODE_LOW_POWER        = 0x02, // nRF5x low power mode
    POWER_MODE_CONSTANT_LATENCY = 0x03  // nRF5x constant latency mode
} nRF5x_powermodes_t;

class Arduino_nRF5x_lowPower {
    public:
        void powerMode(nRF5x_powermodes_t mode);
        void enableDCDC();
        void disableDCDC();
        void enableWakeupByInterrupt(uint32_t pin, uint32_t mode);
        void disableWakeupByInterrupt(uint32_t pin);
    private:
        void powerOff(void);
        void constLat(void);
        void lowPower(void);
        uint8_t checkForSoftDevice(void);
};

extern Arduino_nRF5x_lowPower nRF5x_lowPower;

#endif