/*****************************************************************************
  KX022.cpp

 Copyright (c) 2016 ROHM Co.,Ltd.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/
//#include <avr/pgmspace.h>
#include <Arduino.h>
#include <Wire.h>
#include <KX022.h>

KX022::KX022(int slave_address)
{
  _device_address = slave_address ;
}

byte KX022::init(void)
{
  byte rc;
  unsigned char reg;
  unsigned char gsel;
  int i;

  rc = read(KX022_WHO_AM_I, &reg, sizeof(reg));
  if (rc != 0) {
    Serial.println(F("Can't access KX022"));
    return (rc);
  } 
  Serial.print(F("KX022_WHO_AMI Register Value = 0x"));
  Serial.println(reg, HEX);
  
  if (reg != KX022_WAI_VAL) {
    Serial.println(F("Can't find KX022"));
    return (rc);
  }

  reg = KX022_CNTL1_VAL;
  rc = write(KX022_CNTL1, &reg, sizeof(reg));
  if (rc != 0) {
    Serial.println("Can't write KX022 CNTL1 register at first");
    return (rc);
  }

  reg = KX022_ODCNTL_VAL;
  rc = write(KX022_ODCNTL, &reg, sizeof(reg));
  if (rc != 0) {
    Serial.println("Can't write KX022 ODCNTL register");
    return (rc);
  }

  rc = read(KX022_CNTL1, &reg, sizeof(reg));
  if (rc != 0) {
    Serial.println(F("Can't read KX022 CNTL1 register"));
    return (rc);
  }
  gsel = reg & KX022_CNTL1_GSELMASK;

  reg |= KX022_CNTL1_PC1;
  rc = write(KX022_CNTL1, &reg, sizeof(reg));
  if (rc != 0) {
    Serial.println(F("Can't write KX022 CNTL1 register at second"));
    return (rc);
  }
  
  switch(gsel) {
    case KX022_CNTL1_GSEL_2G : _g_sens = 16384; break;
    case KX022_CNTL1_GSEL_4G : _g_sens = 8192;  break;
    case KX022_CNTL1_GSEL_8G : _g_sens = 4096;  break;
    default: break;
  }
}

byte KX022::get_rawval(unsigned char *data)
{
  byte rc;

  //DEBUG TRIPLE
  rc = read(KX022_XOUT_L, data, 6);
  if (rc != 0) {
    Serial.println(F("Can't get KX022 accel value"));
  }

  return (rc);
}

byte KX022::get_val(float *data)
{
  byte rc;
  unsigned char val[6];
  signed short acc[3];

  rc = get_rawval(val);
  if (rc != 0) {
    return (rc);
  }

  //DEBUG
//Serial.print(val[0]); Serial.print(val[1]); Serial.print(val[2]); Serial.print(val[3]); Serial.print(val[4]); Serial.print(val[5]);  Serial.println(" are the raw values");

  acc[0] = ((signed short)val[1] << 8) | (val[0]);
  acc[1] = ((signed short)val[3] << 8) | (val[2]);
  acc[2] = ((signed short)val[5] << 8) | (val[4]);

  // Convert LSB to g
  data[0] = (float)acc[0] / _g_sens;
  data[1] = (float)acc[1] / _g_sens;
  data[2] = (float)acc[2] / _g_sens;

  return (rc);  
}

byte KX022::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;
  unsigned int cnt;

  Wire.beginTransmission(_device_address);
        //DEBUG
    delay(3);
  Wire.write(memory_address);
        //DEBUG
    delay(3);
  Wire.write(data, size);
      //DEBUG
    delay(5);
  rc = Wire.endTransmission(true);
  return (rc);
}

byte KX022::read(unsigned char memory_address, unsigned char *data, int size)
{
  byte rc;
  unsigned char cnt;

  Wire.beginTransmission(_device_address);
        //DEBUG
    delay(3);
  Wire.write(memory_address);
      //DEBUG
    delay(5);
  rc = Wire.endTransmission(false);
  if (rc != 0) {
    return (rc);
  }

  Wire.requestFrom(_device_address, size, true);
  cnt = 0;
  while(Wire.available()) {
    data[cnt] = Wire.read();
    //DEBUG
    delay(4);
    cnt++;
  }

  return (0);
}
