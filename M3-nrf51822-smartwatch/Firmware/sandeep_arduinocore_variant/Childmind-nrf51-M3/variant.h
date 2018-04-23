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

#ifndef _CHILDMIND_NRF51_M3_
#define _CHILDMIND_NRF51_M3_

/** Master clock frequency */
#define VARIANT_MCK       (16000000ul)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (29u)
#define NUM_DIGITAL_PINS     (29u)
//#define NUM_ANALOG_INPUTS    (6u)
  #define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
#define PIN_LED              (10)   
#define LED_BUILTIN          PIN_LED

/*
 * Analog pins
 */
#define PIN_A0               (26)
#define PIN_A1               (27)
#define PIN_A2               (1)
#define PIN_A3               (2)
#define PIN_A4               (3)
#define PIN_A5               (4)
#define PIN_A6               (5)
#define PIN_A7               (6)

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
#define ADC_RESOLUTION    10

/*
 * Serial interfaces
 */
// Serial
#define PIN_SERIAL_RX       (12)
#define PIN_SERIAL_TX       (13)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (30)
#define PIN_SPI_MOSI         (0)
#define PIN_SPI_SCK          (21)

#define PIN_SPI1_MISO         (30) //not used
#define PIN_SPI1_MOSI         (0)
#define PIN_SPI1_SCK          (21)

#define PIN_SPI2_MISO         (30) //not used
#define PIN_SPI2_MOSI         (0)
#define PIN_SPI2_SCK          (21)

static const uint8_t SS   = 0xFFFFFFFF ;// Use not connected  //5u ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

static const uint8_t SS1   = 0xFFFFFFFF ;// Use not connected  //7u ;
static const uint8_t MOSI1 = PIN_SPI1_MOSI ;
static const uint8_t MISO1 = PIN_SPI1_MISO ;
static const uint8_t SCK1  = PIN_SPI1_SCK ; 

static const uint8_t SS2   = 0xFFFFFFFF ;// Use not connected  //7u ;
static const uint8_t MOSI2 = PIN_SPI2_MOSI ;
static const uint8_t MISO2 = PIN_SPI2_MISO ;
static const uint8_t SCK2  = PIN_SPI2_SCK ; 

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (2u)
#define PIN_WIRE_SCL         (18u)

// TEMP DUMMY
//#define PIN_WIRE_SDA         (19u)
//#define PIN_WIRE_SCL         (20u)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#ifdef __cplusplus
}
#endif

#endif
