/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  ADD THE FOLLOWING TO boards.txt:
  
Wearable-nRF52-B20.name=Wearable nrf52 B20 ECG

Wearable-nRF52-B20.upload.tool=sandeepmistry:openocd
Wearable-nRF52-B20.upload.target=nrf52
Wearable-nRF52-B20.upload.maximum_size=524288

Wearable-nRF52-B20.bootloader.tool=sandeepmistry:openocd

Wearable-nRF52-B20.build.mcu=cortex-m4
Wearable-nRF52-B20.build.f_cpu=16000000
Wearable-nRF52-B20.build.board=WEARABLE_NRF52_B20
Wearable-nRF52-B20.build.core=nRF5
Wearable-nRF52-B20.build.variant=Wearable-nRF52-B20
Wearable-nRF52-B20.build.variant_system_lib=
Wearable-nRF52-B20.build.extra_flags=-DNRF52
Wearable-nRF52-B20.build.float_flags=-mfloat-abi=hard -mfpu=fpv4-sp-d16
Wearable-nRF52-B20.build.ldscript=nrf52_xxaa.ld

Wearable-nRF52-B20.menu.softdevice.none=None
Wearable-nRF52-B20.menu.softdevice.none.softdevice=none

Wearable-nRF52-B20.menu.softdevice.s132=S132
Wearable-nRF52-B20.menu.softdevice.s132.softdevice=s132
Wearable-nRF52-B20.menu.softdevice.s132.softdeviceversion=2.0.1
Wearable-nRF52-B20.menu.softdevice.s132.upload.maximum_size=409600
Wearable-nRF52-B20.menu.softdevice.s132.build.extra_flags=-DNRF52 -DS132 -DNRF51_S132
Wearable-nRF52-B20.menu.softdevice.s132.build.ldscript=armgcc_s132_nrf52832_xxaa.ld

Wearable-nRF52-B20.menu.lfclk.lfxo=Crystal Oscillator
Wearable-nRF52-B20.menu.lfclk.lfxo.build.lfclk_flags=-DUSE_LFXO
Wearable-nRF52-B20.menu.lfclk.lfrc=RC Oscillator
Wearable-nRF52-B20.menu.lfclk.lfrc.build.lfclk_flags=-DUSE_LFRC
Wearable-nRF52-B20.menu.lfclk.lfsynt=Synthesized
Wearable-nRF52-B20.menu.lfclk.lfsynt.build.lfclk_flags=-DUSE_LFSYNT
*/

#define NRF52
#ifndef _VARIANT_WEARABLE_NRF52_B20_
#define _VARIANT_WEARABLE_NRF52_B20_

/** Master clock frequency */
#ifdef NRF52
#define VARIANT_MCK       (64000000ul)
#else
#define VARIANT_MCK       (16000000ul)
#endif

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
#define PIN_LED              (8) //vibration motor
#define LED_BUILTIN          PIN_LED

/*
 * Analog pins
 */
#define PIN_A0               (2)
#define PIN_A1               (3)
#define PIN_A2               (4)
#define PIN_A3               (5)
#define PIN_A4               (28)
#define PIN_A5               (29)
#define PIN_A6               (30)
#define PIN_A7               (31)

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
#ifdef NRF52
#define ADC_RESOLUTION    14
#else
#define ADC_RESOLUTION    10
#endif

/*
 * Serial interfaces
 */

// Serial Cannibalized from HR sensor I2C
#define PIN_SERIAL_RX       (25) 
#define PIN_SERIAL_TX       (26)

// Serial dummy NC GPIO
//#define PIN_SERIAL_RX       (22) 
//#define PIN_SERIAL_TX       (9)

/*
 * SPI Interfaces
 * IMPORTANT: The accelerometer/flash and the OLED are on seperate SPI interfaces ie separate GPIO.
 * It is possible to have two SPI interfaces working at once, but this is not currently supported 
 * by Sandeep's Nordic ArduinoCore SPI library. I will write a modified SPI library that supports
 * two interfaces as soon as I can, but for now you must choose between the accelerometer and display.
 * Comment out the SPI GPIO and select GPIO for the SPI device you choose not to use.
 */

#define SPI_INTERFACES_COUNT 1

//SPI GPIO for KX126 Accelerometer
/*
#define PIN_SPI_MISO         ()
#define PIN_SPI_MOSI         ()
#define PIN_SPI_SCK          ()
static const uint8_t SS     = 24 ;
*/


//SPI for ADS1292 ECG
#define PIN_SPI_MISO         (18)
#define PIN_SPI_MOSI         (16)
#define PIN_SPI_SCK          (17)
static const uint8_t SS     = 15 ;


static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (11u)
#define PIN_WIRE_SCL         (12u)

#ifdef __cplusplus
}
#endif

#endif
