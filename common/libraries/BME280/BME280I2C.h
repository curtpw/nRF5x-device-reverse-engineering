/*
BME280I2C.h
This code records data from the BME280 sensor and provides an API.
This file is part of the Arduino BME280 library.
Copyright (C) 2016  Tyler Glenn

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Written: Sep 19 2016.
Last Updated: Oct 07 2017.

This code is licensed under the GNU LGPL and is open for ditrbution
and copying in accordance with the license.
This header must be included in any derived code or copies of the code.

Based on the data sheet provided by Bosch for the Bme280 environmental sensor.
 */

#ifndef TG_BME_280_I2C_H
#define TG_BME_280_I2C_H

#include "BME280.h"

//////////////////////////////////////////////////////////////////
/// BME280I2C - I2C Implementation of BME280.
class BME280I2C: public BME280
{

public:

   enum I2CAddr
   {
      I2CAddr_0x76 = 0x76,
      I2CAddr_0x77 = 0x77
   };


   struct Settings : public BME280::Settings
   {
      Settings(
         OSR _tosr       = OSR_X1,
         OSR _hosr       = OSR_X1,
         OSR _posr       = OSR_X1,
         Mode _mode      = Mode_Forced,
         StandbyTime _st = StandbyTime_1000ms,
         Filter _filter  = Filter_16,
         SpiEnable _se   = SpiEnable_False,
         I2CAddr _addr   = I2CAddr_0x76
        ): BME280::Settings(_tosr, _hosr, _posr, _mode, _st, _filter, _se),
           bme280Addr(_addr) {}

      I2CAddr bme280Addr;
   };

   ///////////////////////////////////////////////////////////////
   /// Constructor used to create the class. All parameters have
   /// default values.
   BME280I2C(
      const Settings& settings = Settings());


/*****************************************************************/
/* ACCESSOR FUNCTIONS                                            */
/*****************************************************************/

   /////////////////////////////////////////////////////////////////
   virtual void setSettings(
      const Settings& settings);

   /////////////////////////////////////////////////////////////////
   const Settings& getSettings() const;


protected:

private:

   Settings m_settings;

   //////////////////////////////////////////////////////////////////
   /// Write values to BME280 registers.
   virtual bool WriteRegister(
      uint8_t addr,
      uint8_t data);

   /////////////////////////////////////////////////////////////////
   /// Read values from BME280 registers.
   virtual bool ReadRegister(
      uint8_t addr,
      uint8_t data[],
      uint8_t length);

};
#endif // TG_BME_280_I2C_H
