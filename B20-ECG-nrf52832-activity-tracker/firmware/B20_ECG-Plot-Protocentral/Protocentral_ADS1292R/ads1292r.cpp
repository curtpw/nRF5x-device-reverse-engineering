//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//   
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT). 
//   
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   Requires g4p_control graphing library for processing.  Built on V4.1
//   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ads1292r.h>
#include <SPI.h>

char* ads1292r::ads1292_Read_Data()
{

   static char SPI_Dummy_Buff[10];
   
   digitalWrite(ADS1292_CS_PIN, LOW);
   
	for (int i = 0; i < 9; ++i)
	{
		SPI_Dummy_Buff[i] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
		
	}
	
    digitalWrite(ADS1292_CS_PIN, HIGH);
	
	return SPI_Dummy_Buff;
}

void ads1292r::ads1292_Init()
{ 
  // start the SPI library:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST); 
  //CPOL = 0, CPHA = 1
  SPI.setDataMode(SPI_MODE1);
  // Selecting 1Mhz clock for SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  ads1292_Reset();
  delay(100);
  ads1292_Disable_Start();
  ads1292_Enable_Start();
  
  ads1292_Hard_Stop();
  ads1292_Start_Data_Conv_Command();
  ads1292_Soft_Stop();
  delay(50);
  ads1292_Stop_Read_Data_Continuous();					// SDATAC command
  delay(300);
  

/*
  // write_byte(CONFIG1, SAMP_250_SPS);
  write_byte(CONFIG1, SAMP_500_SPS); 
 // write_byte(CONFIG1, SAMP_1_KSPS);
   CONFIG1_const = 0x00,
   SAMP_125_SPS = CONFIG1_const,
   SAMP_250_SPS = (CONFIG1_const | DR0),
   SAMP_500_SPS = (CONFIG1_const | DR1),
      SAMP_2_KSPS = (CONFIG1_const | DR2), 
     SINGLE_SHOT = 0x80,
  DR2 = 0x04,
  DR1 = 0x02,
  DR0 = 0x01,
 */

  //ads1292_Reg_Write(ADS1292_REG_CONFIG1, 0x00); 		//Set sampling rate to 125 SPS
  ads1292_Reg_Write(ADS1292_REG_CONFIG1, 0x02);     //Set sampling rate to 500 SPS
  //ads1292_Reg_Write(ADS1292_REG_CONFIG1, 0x04);     //Set sampling rate to 2k SPS
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CONFIG2, 0b10100000);	//Lead-off comp off, test signal disabled
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFF, 0b00010000);		//Lead-off defaults
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH1SET, 0b01000000);	//Ch 1 enabled, gain 6, connected to electrode in
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH2SET, 0b01100000);	//Ch 2 enabled, gain 6, connected to electrode in
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_RLDSENS, 0b00101100);	//RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFFSENS, 0x00);		//LOFF settings: all disabled
  delay(10);
														//Skip register 8, LOFF Settings default
  ads1292_Reg_Write(ADS1292_REG_RESP1, 0b11110010);		//Respiration: MOD/DEMOD turned only, phase 0
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_RESP2, 0b00000011);		//Respiration: Calib OFF, respiration freq defaults
  delay(10);
  ads1292_Start_Read_Data_Continuous();
  delay(10);
  ads1292_Enable_Start();
}

void ads1292r::ads1292_Reset()
{
  digitalWrite(ADS1292_PWDN_PIN, HIGH);
  delay(100);					// Wait 100 mSec
  digitalWrite(ADS1292_PWDN_PIN, LOW);
  delay(100);
  digitalWrite(ADS1292_PWDN_PIN, HIGH);
  delay(100);
}

void ads1292r::ads1292_Disable_Start()
{
  digitalWrite(ADS1292_START_PIN, LOW);
  delay(20);
}

void ads1292r::ads1292_Enable_Start()
{
  digitalWrite(ADS1292_START_PIN, HIGH);
  delay(20);
}

void ads1292r::ads1292_Hard_Stop (void)
{
  digitalWrite(ADS1292_START_PIN, LOW);
  delay(100);
}


void ads1292r::ads1292_Start_Data_Conv_Command (void)
{
  ads1292_SPI_Command_Data(START);					// Send 0x08 to the ADS1x9x
}

void ads1292r::ads1292_Soft_Stop (void)
{
  ads1292_SPI_Command_Data(STOP);                   // Send 0x0A to the ADS1x9x
}

void ads1292r::ads1292_Start_Read_Data_Continuous (void)
{
  ads1292_SPI_Command_Data(RDATAC);					// Send 0x10 to the ADS1x9x
}

void ads1292r::ads1292_Stop_Read_Data_Continuous (void)
{
  ads1292_SPI_Command_Data(SDATAC);					// Send 0x11 to the ADS1x9x
}

void ads1292r::ads1292_SPI_Command_Data(unsigned char data_in)
{
  byte data[1];
  //data[0] = data_in;
  digitalWrite(ADS1292_CS_PIN, LOW);
  delay(2);
  digitalWrite(ADS1292_CS_PIN, HIGH);
  delay(2);
  digitalWrite(ADS1292_CS_PIN, LOW);
  delay(2);
  SPI.transfer(data_in);
  delay(2);
  digitalWrite(ADS1292_CS_PIN, HIGH);
}

//Sends a write command to SCP1000
void ads1292r::ads1292_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{
  switch (READ_WRITE_ADDRESS)
  {
    case 1:
            DATA = DATA & 0x87;
	    break;
    case 2:
            DATA = DATA & 0xFB;
	    DATA |= 0x80;		
	    break;
    case 3:
	    DATA = DATA & 0xFD;
	    DATA |= 0x10;
	    break;
    case 7:
	    DATA = DATA & 0x3F;
	    break;
    case 8:
    	    DATA = DATA & 0x5F;
	    break;
    case 9:
	    DATA |= 0x02;
	    break;
    case 10:
	    DATA = DATA & 0x87;
	    DATA |= 0x01;
	    break;
    case 11:
	    DATA = DATA & 0x0F;
	    break;
    default:
	    break;		
  }
  
  // now combine the register address and the command into one byte:
  byte dataToSend = READ_WRITE_ADDRESS | WREG;
  
   digitalWrite(ADS1292_CS_PIN, LOW);
   delay(2);
   digitalWrite(ADS1292_CS_PIN, HIGH);
   delay(2);
  // take the chip select low to select the device:
  digitalWrite(ADS1292_CS_PIN, LOW);
  delay(2);
  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(0x00);		//number of register to wr
  SPI.transfer(DATA);		//Send value to record into register
  
  delay(2);
  // take the chip select high to de-select:
  digitalWrite(ADS1292_CS_PIN, HIGH);
}