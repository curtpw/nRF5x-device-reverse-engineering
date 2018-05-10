/*

BME280.h

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

Written: Dec 30 2015.
Last Updated: Oct 07 2017.

This code is licensed under the GNU LGPL and is open for ditrbution
and copying in accordance with the license.
This header must be included in any derived code or copies of the code.

 */

#ifndef TG_BME_280_H
#define TG_BME_280_H

#include "Arduino.h"


//////////////////////////////////////////////////////////////////
/// BME280 - Driver class for Bosch Bme280 sensor
///
/// Based on the data sheet provided by Bosch for
/// the Bme280 environmental sensor.
///
class BME280
{
public:

/*****************************************************************/
/* ENUMERATIONS                                                  */
/*****************************************************************/

   enum TempUnit
   {
      TempUnit_Celsius,
      TempUnit_Fahrenheit
   };

   enum PresUnit
   {
      PresUnit_Pa,
      PresUnit_hPa,
      PresUnit_inHg,
      PresUnit_atm,
      PresUnit_bar,
      PresUnit_torr,
      PresUnit_psi
   };

   enum OSR
   {
      OSR_Off =  0,
      OSR_X1  =  1,
      OSR_X2  =  2,
      OSR_X4  =  3,
      OSR_X8  =  4,
      OSR_X16 =  5
   };

   enum Mode
   {
      Mode_Sleep  = 0,
      Mode_Forced = 1,
      Mode_Normal = 3
   };

   enum StandbyTime
   {
      StandbyTime_500us   = 0,
      StandbyTime_62500us = 1,
      StandbyTime_125ms   = 2,
      StandbyTime_250ms   = 3,
      StandbyTime_50ms    = 4,
      StandbyTime_1000ms  = 5,
      StandbyTime_10ms    = 6,
      StandbyTime_20ms    = 7
   };

   enum Filter
   {
      Filter_Off = 0,
      Filter_2   = 1,
      Filter_4   = 2,
      Filter_8   = 3,
      Filter_16  = 4
   };

   enum SpiEnable
   {
      SpiEnable_False = 0,
      SpiEnable_True = 1
   };

   enum ChipModel
   {
      ChipModel_UNKNOWN = 0,
      ChipModel_BMP280 = 0x58,
      ChipModel_BME280 = 0x60
   };

/*****************************************************************/
/* STRUCTURES                                                  */
/*****************************************************************/

   struct Settings
   {
      Settings(
         OSR _tosr       = OSR_X1,
         OSR _hosr       = OSR_X1,
         OSR _posr       = OSR_X1,
         Mode _mode      = Mode_Forced,
         StandbyTime _st = StandbyTime_1000ms,
         Filter _filter  = Filter_Off,
         SpiEnable _se   = SpiEnable_False
      ): tempOSR(_tosr),
         humOSR(_hosr),
         presOSR(_posr),
         mode(_mode),
         standbyTime(_st),
         filter(_filter),
         spiEnable(_se) {}

      OSR tempOSR;
      OSR humOSR;
      OSR presOSR;
      Mode mode;
      StandbyTime standbyTime;
      Filter filter;
      SpiEnable spiEnable;
   };

/*****************************************************************/
/* INIT FUNCTIONS                                                */
/*****************************************************************/

   /////////////////////////////////////////////////////////////////
   /// Constructor used to create the class.
   /// All parameters have default values.
   BME280(
      const Settings& settings);

   /////////////////////////////////////////////////////////////////
   /// Method used to initialize the class.
   bool begin();

/*****************************************************************/
/* ENVIRONMENTAL FUNCTIONS                                       */
/*****************************************************************/

   //////////////////////////////////////////////////
   /// Read the temperature from the BME280 and return a float.
   float temp(
      TempUnit unit = TempUnit_Celsius);

   /////////////////////////////////////////////////////////////////
   /// Read the pressure from the BME280 and return a float with the
   /// specified unit.
   float pres(
      PresUnit unit = PresUnit_hPa);

   /////////////////////////////////////////////////////////////////
   /// Read the humidity from the BME280 and return a percentage
   /// as a float.
   float hum();

   /////////////////////////////////////////////////////////////////
   /// Read the data from the BME280 in the specified unit.
   void   read(
      float&    pressure,
      float&    temperature,
      float&    humidity,
      TempUnit  tempUnit    = TempUnit_Celsius,
      PresUnit  presUnit    = PresUnit_hPa);


/*****************************************************************/
/* ACCESSOR FUNCTIONS                                            */
/*****************************************************************/

   ////////////////////////////////////////////////////////////////
   /// Method used to return ChipModel.
   ChipModel chipModel();

protected:

/*****************************************************************/
/* CONSTRUCTOR INIT FUNCTIONS                                    */
/*****************************************************************/

   //////////////////////////////////////////////////////////////////
   /// Write configuration to BME280, return true if successful.
   virtual bool Initialize();


/*****************************************************************/
/* ACCESSOR FUNCTIONS                                            */
/*****************************************************************/

   /////////////////////////////////////////////////////////////////
   virtual void setSettings(
      const Settings& settings);

   /////////////////////////////////////////////////////////////////
   virtual const Settings& getSettings() const;


private:

/*****************************************************************/
/* CONSTANTS                                                     */
/*****************************************************************/

   static const uint8_t CTRL_HUM_ADDR   = 0xF2;
   static const uint8_t CTRL_MEAS_ADDR  = 0xF4;
   static const uint8_t CONFIG_ADDR     = 0xF5;
   static const uint8_t PRESS_ADDR      = 0xF7;
   static const uint8_t TEMP_ADDR       = 0xFA;
   static const uint8_t HUM_ADDR        = 0xFD;
   static const uint8_t TEMP_DIG_ADDR   = 0x88;
   static const uint8_t PRESS_DIG_ADDR  = 0x8E;
   static const uint8_t HUM_DIG_ADDR1   = 0xA1;
   static const uint8_t HUM_DIG_ADDR2   = 0xE1;
   static const uint8_t ID_ADDR         = 0xD0;

   static const uint8_t TEMP_DIG_LENGTH         = 6;
   static const uint8_t PRESS_DIG_LENGTH        = 18;
   static const uint8_t HUM_DIG_ADDR1_LENGTH    = 1;
   static const uint8_t HUM_DIG_ADDR2_LENGTH    = 7;
   static const uint8_t DIG_LENGTH              = 32;
   static const uint8_t SENSOR_DATA_LENGTH      = 8;


/*****************************************************************/
/* VARIABLES                                                     */
/*****************************************************************/
   Settings m_settings;

   uint8_t m_dig[32];
   ChipModel m_chip_model;

   bool m_initialized;


/*****************************************************************/
/* ABSTRACT FUNCTIONS                                            */
/*****************************************************************/

   /////////////////////////////////////////////////////////////////
   /// Write values to BME280 registers.
   virtual bool WriteRegister(
      uint8_t addr,
      uint8_t data)=0;

   /////////////////////////////////////////////////////////////////
   /// Read values from BME280 registers.
   virtual bool ReadRegister(
      uint8_t addr,
      uint8_t data[],
      uint8_t length)=0;


/*****************************************************************/
/* WORKER FUNCTIONS                                              */
/*****************************************************************/

   /////////////////////////////////////////////////////////////////
   /// Calculates registers based on settings.
   void CalculateRegisters(
      uint8_t& ctrlHum,
      uint8_t& ctrlMeas,
      uint8_t& config);

   /////////////////////////////////////////////////////////////////
   /// Write the settings to the chip.
   bool WriteSettings();


   /////////////////////////////////////////////////////////////////
   /// Read the the chip id data from the BME280, return true if
   /// successful and the id matches a known value.
   bool ReadChipID();

   /////////////////////////////////////////////////////////////////
   /// Read the the trim data from the BME280, return true if
   /// successful.
   bool ReadTrim();

   /////////////////////////////////////////////////////////////////
   /// Read the raw data from the BME280 into an array and return
   /// true if successful.
   bool ReadData(
      int32_t data[8]);


   /////////////////////////////////////////////////////////////////
   /// Calculate the temperature from the BME280 raw data and
   /// BME280 trim, return a float.
   float CalculateTemperature(
      int32_t raw,
      int32_t& t_fine,
      TempUnit unit = TempUnit_Celsius);

   /////////////////////////////////////////////////////////////////
   /// Calculate the humidity from the BME280 raw data and BME280
   /// trim, return a float.
   float CalculateHumidity(
      int32_t raw,
      int32_t t_fine);

   /////////////////////////////////////////////////////////////////
   /// Calculate the pressure from the BME280 raw data and BME280
   /// trim, return a float.
   float CalculatePressure(
      int32_t raw,
      int32_t t_fine,
      PresUnit unit = PresUnit_hPa);

};

#endif // TG_BME_280_H
