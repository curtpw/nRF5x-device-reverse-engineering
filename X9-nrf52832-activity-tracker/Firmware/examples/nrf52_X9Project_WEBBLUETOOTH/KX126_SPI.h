/*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*
*  KX126_SPI Accelerometer library
*
*  @author  Goran Mahovlić
*  @version 1.0
*  @date    26-May-2017
*
*  Library for "KX126_SPI Accelerometer library" from Kionix a Rohm group
*    http://www.kionix.com/product/KX126_SPI-1020
*
*/

#ifndef KX126_SPI_H
#define KX126_SPI_H
#include <SPI.h>
//#include "KX022_registers.h"



#define KX126_WHO_AM_I      (0x0F)
#define KX126_CNTL1_1       (0x1A)
#define KX126_CNTL1_2       (0x41) //?
#define KX126_ODCNTL_1      (0x1F)
#define KX126_ODCNTL_2      (0x02) //?
#define KX126_CNTL3_1       (0x1C)
#define KX126_CNTL3_2       (0xD8) //?
#define KX126_TILT_TIMER_1  (0x27)
#define KX126_TILT_TIMER_2  (0x01) //?
#define KX126_CNTL2_1       (0x1B)
#define KX126_CNTL2_2       (0xC1) //?

// output register x
#define KX122_XOUT_L 0x08
#define KX122_XOUT_H 0x09
// output register y
#define KX122_YOUT_L 0x0A
#define KX122_YOUT_H 0x0B
// output register z
#define KX122_ZOUT_L 0x0C
#define KX122_ZOUT_H 0x0D

#define DATA_OUT_BASE 0x02

class KX126_SPI
{
public:
    /**
    * KX126_SPI constructor
    */
    KX126_SPI(int cs);
    /**
    * KX126_SPI constructor with SPI channel
    */	
	KX126_SPI(SPIClass &spi,int cs);
    /**
    * KX126_SPI destructor
    */
    ~KX126_SPI();
 
    /** Initializa KX126_SPI sensor
     *
     *  Configure sensor setting
     *
     */
    int init(void);
    float getAccel(int channelNum);
	void readBytes (int address,uint8_t *buff,int len);

	

private:
	void writeTwoBytes (int address, int data);
	int getByte (int address);
	uint16_t getTwoBytes(int address);	
	SPIClass &_spi;
	int _cs;
	uint8_t _buf[16];// faster to permanently allocate a SPI transfer buffer
};
 
#endif