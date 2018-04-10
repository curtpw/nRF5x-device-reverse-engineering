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
*/

#ifndef _VARIANT_CHILDMIND_ID107_
#define _VARIANT_CHILDMIND_ID107_

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
#define PIN_LED              (25)   //vibration motor
#define LED_BUILTIN          PIN_LED

/*
 * Analog pins
 */
#define PIN_A0               (1)
#define PIN_A1               (2)
#define PIN_A2               (3)
#define PIN_A3               (4)
#define PIN_A4               (5)
#define PIN_A5               (6)

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
#ifdef NRF52
#define ADC_RESOLUTION    14
#else
#define ADC_RESOLUTION    10
#endif

/*
 * Serial interfaces
 */
// Serial
//#define PIN_SERIAL_RX       (24)
//#define PIN_SERIAL_TX       (23)

//change for audio integration
#define PIN_SERIAL_RX       (8)
#define PIN_SERIAL_TX       (10)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MISO         (4u)
#define PIN_SPI_MOSI         (3u)
#define PIN_SPI_SCK          (5u)

static const uint8_t SS   = 0xFFFFFFFF ;// Use not connected  //7u ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

#define PIN_SPI1_MISO         (4u)
#define PIN_SPI1_MOSI         (3u)
#define PIN_SPI1_SCK          (5u)

static const uint8_t SS1   = 0xFFFFFFFF ;// Use not connected  //7u ;
static const uint8_t MOSI1 = PIN_SPI1_MOSI ;
static const uint8_t MISO1 = PIN_SPI1_MISO ;
static const uint8_t SCK1  = PIN_SPI1_SCK ; 

#define PIN_SPI2_MISO         (4u)
#define PIN_SPI2_MOSI         (3u)
#define PIN_SPI2_SCK          (5u)

static const uint8_t SS2   = 0xFFFFFFFF ;// Use not connected  //7u ;
static const uint8_t MOSI2 = PIN_SPI2_MOSI ;
static const uint8_t MISO2 = PIN_SPI2_MISO ;
static const uint8_t SCK2  = PIN_SPI2_SCK ; 

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (15u)
#define PIN_WIRE_SCL         (22u)
//alternate SDA=10 SCL=9 for built in SI1143 optical sensor

#ifdef __cplusplus
}
#endif

#endif
