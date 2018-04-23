/*
The MIT License (MIT)
Copyright (c) 2016 Kionix Inc.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __KX022_REGISTERS_H__
#define __KX022_REGISTERS_H__
/* registers */
// x- hp filter output
#define KX022_XHP_L 0x00
#define KX022_XHP_H 0x01
// y- hp filter output
#define KX022_YHP_L 0x02
#define KX022_YHP_H 0x03
// z- hpfilteroutput
#define KX022_ZHP_L 0x04
#define KX022_ZHP_H 0x05
// output register x
#define KX022_XOUT_L 0x06
#define KX022_XOUT_H 0x07
// output register y
#define KX022_YOUT_L 0x08
#define KX022_YOUT_H 0x09
// output register z
#define KX022_ZOUT_L 0x0A
#define KX022_ZOUT_H 0x0B
// communication selftest
#define KX022_COTR 0x0C
#define KX022_WHO_AM_I 0x0F
// current sixfacet posititions
#define KX022_TSCP 0x10
// previous six facet positions
#define KX022_TSPP 0x11
// This register indicates the triggering axis when a tap/double tap interrupt occurs.
#define KX022_INS1 0x12
// This register tells witch function caused an interrupt.
#define KX022_INS2 0x13
// This register reports the axis and direction of detected motion.
#define KX022_INS3 0x14
// This register reports the status of the interrupt.
#define KX022_STATUS_REG 0x15
// Latched interrupt source information (INS1,INS2, INS3 except WMI/BFI and INT when WMI/BFI is zero) is cleared and physical interrupt latched pin is changed to its inactive state when this register is read. Read value is dummy.
#define KX022_INT_REL 0x17
// Read/write control register that controls the main feature set.
#define KX022_CNTL1 0x18
// 2' control register
#define KX022_CNTL2 0x19
// 3' controlregister
#define KX022_CNTL3 0x1A
// This register is responsible for configuring ODR (output data rate) and filter settings
#define KX022_ODCNTL 0x1B
// This register controls the settings for the physical interrupt pin INT1
#define KX022_INC1 0x1C
// This register controls which axis and direction of detected motion can cause an interrupt.
#define KX022_INC2 0x1D
// This register controls which axis and direction of tap/double tap can cause an interrup
#define KX022_INC3 0x1E
// This register controls routing of an interrupt reporting to physical interrupt pin INT1
#define KX022_INC4 0x1F
// This register controls the settings for the physical interrupt pin INT2.
#define KX022_INC5 0x20
// This register controls routing of interrupt reporting to physical interrupt pin INT2
#define KX022_INC6 0x21
// This register is the initial count register for the tilt position state timer
#define KX022_TILT_TIMER 0x22
// This register is the initial count register for the motion detection timer
#define KX022_WUFC 0x23
// This register is responsible for enableing/disabling reporting of Tap/Double Tap.
#define KX022_TDTRC 0x24
// This register contains counter information for the detection of a double tap event.
#define KX022_TDTC 0x25
// This register represents the 8-bit jerk high threshold to determine if a tap is detected.
#define KX022_TTH 0x26
// This register represents the 8-bit (0d 255d) jerk low threshold to determine if a tap is detected.
#define KX022_TTL 0x27
// This register contains counter information for the detection of any tap event.
#define KX022_FTD 0x28
// This register contains counter information for the detection of a double tap event
#define KX022_STD 0x29
// This register contains counter information for the detection of a tap event.
#define KX022_TLT 0x2A
// This register contains counter information for the detection of single and double taps.
#define KX022_TWS 0x2B
// This register sets the threshold for wake-up (motion detect) interrupt is set.
#define KX022_ATH 0x30
// This register sets the low level threshold for tilt angle detection.
#define KX022_TILT_ANGLE_LL 0x32
// This register sets the high level threshold for tilt angle detection.
#define KX022_TILT_ANGLE_HL 0x33
// This register sets the Hysteresis that is placed in between the Screen Rotation states
#define KX022_HYST_SET 0x34
// Low Power Control sets the number of samples of accelerometer output to be average
#define KX022_LP_CNTL 0x35
// Read/write control register that controls the buffer sample threshold
#define KX022_BUF_CNTL1 0x3A
// Read/write control register that controls sample buffer operation
#define KX022_BUF_CNTL2 0x3B
// This register reports the status of the sample buffer
#define KX022_BUF_STATUS_1 0x3C
// This register reports the status of the sample buffer trigger function
#define KX022_BUF_STATUS_2 0x3D
// Latched buffer status information and the entire sample buffer are cleared when any data is written to this register.
#define KX022_BUF_CLEAR 0x3E
// Buffer output register
#define KX022_BUF_READ 0x3F
// When 0xCA is written to this register, the MEMS self-test function is enabled. Electrostatic-actuation of the accelerometer, results in a DC shift of the X, Y and Z axis outputs. Writing 0x00 to this register will return the accelerometer to normal operation
#define KX022_SELF_TEST 0x60
#define KX012_WHO_AM_I 0x0F
#define KX023_WHO_AM_I 0x0F
/* registers bits */
// before set
#define KX022_COTR_DCSTR_BEFORE (0x55 << 0)
// after set
#define KX022_COTR_DCSTR_AFTER (0xAA << 0)
// WHO_AM_I -value for KX022
#define KX022_WHO_AM_I_WIA_ID (0x14 << 0)
// x-left
#define KX022_TSCP_LE (0x01 << 5)
// x+right
#define KX022_TSCP_RI (0x01 << 4)
// y-down
#define KX022_TSCP_DO (0x01 << 3)
// y+up
#define KX022_TSCP_UP (0x01 << 2)
// z-facedown
#define KX022_TSCP_FD (0x01 << 1)
// z+faceup
#define KX022_TSCP_FU (0x01 << 0)
// x-left
#define KX022_TSPP_LE (0x01 << 5)
// x+right
#define KX022_TSPP_RI (0x01 << 4)
// y-down
#define KX022_TSPP_DO (0x01 << 3)
// y+up
#define KX022_TSPP_UP (0x01 << 2)
// z-facedown
#define KX022_TSPP_FD (0x01 << 1)
// z+faceup
#define KX022_TSPP_FU (0x01 << 0)
// x-
#define KX022_INS1_TLE (0x01 << 5)
// x+
#define KX022_INS1_TRI (0x01 << 4)
// y-
#define KX022_INS1_TDO (0x01 << 3)
// y+
#define KX022_INS1_TUP (0x01 << 2)
// z-
#define KX022_INS1_TFD (0x01 << 1)
// z+
#define KX022_INS1_TFU (0x01 << 0)
// indicates buffer full interrupt. Automatically cleared when buffer is read.
#define KX022_INS2_BFI (0x01 << 6)
// Watermark interrupt, bit is set to one when FIFO has filled up to the value stored in the sample bits.This bit is automatically cleared when FIFO/FILO is read and the content returns to a value below the value stored in the sample bits.
#define KX022_INS2_WMI (0x01 << 5)
// indicates that new acceleration data (0x06h to 0x0Bh) is available. This bit is cleared when acceleration data is read or the interrupt release register INT_REL is read.
#define KX022_INS2_DRDY (0x01 << 4)
// no tap
#define KX022_INS2_TDTS_NOTAP (0x00 << 2)
// single tap event
#define KX022_INS2_TDTS_SINGLE (0x01 << 2)
// double tap event
#define KX022_INS2_TDTS_DOUBLE (0x02 << 2)
// do not exist
#define KX022_INS2_TDTS_NA (0x03 << 2)
// Status of Wake up. This bit is cleared when the interrupt release register INT_REL is read.
#define KX022_INS2_WUFS (0x01 << 1)
// Tilt Position status. This bit is cleared when the interrupt release register INT_REL is read.
#define KX022_INS2_TPS (0x01 << 0)
// x-
#define KX022_INS3_XNWU (0x01 << 5)
// x+
#define KX022_INS3_XPWU (0x01 << 4)
// y-
#define KX022_INS3_YNWU (0x01 << 3)
// y+
#define KX022_INS3_YPWU (0x01 << 2)
// z-
#define KX022_INS3_ZNWU (0x01 << 1)
// z+
#define KX022_INS3_ZPWU (0x01 << 0)
// INT reports the combined (OR) interrupt information of all features.
#define KX022_STATUS_REG_INT (0x01 << 4)
// controls the operating mode of the KX022.
#define KX022_CNTL1_PC1 (0x01 << 7)
// determines the performance mode of the KX022. The noise varies with ODR, RES and different LP_CNTL settings possibly reducing the effective resolution.
#define KX022_CNTL1_RES (0x01 << 6)
// enables the reporting of the availability of new acceleration data as an interrupt
#define KX022_CNTL1_DRDYE (0x01 << 5)
// 2g range
#define KX022_CNTL1_GSEL_2G (0x00 << 3)
// 4g range
#define KX022_CNTL1_GSEL_4G (0x01 << 3)
// 8g range
#define KX022_CNTL1_GSEL_8G (0x02 << 3)
// not valid settings
#define KX022_CNTL1_GSEL_NA (0x03 << 3)
// enables the Directional Tap function that will detect single and double tap events.
#define KX022_CNTL1_TDTE (0x01 << 2)
// enables the Wake Up (motion detect) function
#define KX022_CNTL1_WUFE (0x01 << 1)
// enables the Tilt Position function that will detect changes in device orientation.
#define KX022_CNTL1_TPE (0x01 << 0)
// initiates software reset, which performs the RAM reboot routine
#define KX022_CNTL2_SRST (0x01 << 7)
// command test control
#define KX022_CNTL2_COTC (0x01 << 6)
// x-
#define KX022_CNTL2_LEM (0x01 << 5)
// x+
#define KX022_CNTL2_RIM (0x01 << 4)
// y-
#define KX022_CNTL2_DOM (0x01 << 3)
// y+
#define KX022_CNTL2_UPM (0x01 << 2)
// z-
#define KX022_CNTL2_FDM (0x01 << 1)
// z+
#define KX022_CNTL2_FUM (0x01 << 0)
// 1.5Hz
#define KX022_CNTL3_OTP_1P563 (0x00 << 6)
// 6.25Hz
#define KX022_CNTL3_OTP_6P25 (0x01 << 6)
// 12.5Hz
#define KX022_CNTL3_OTP_12P5 (0x02 << 6)
// 50Hz
#define KX022_CNTL3_OTP_50 (0x03 << 6)
// 50Hz
#define KX022_CNTL3_OTDT_50 (0x00 << 3)
// 100Hz
#define KX022_CNTL3_OTDT_100 (0x01 << 3)
// 200Hz
#define KX022_CNTL3_OTDT_200 (0x02 << 3)
// 400Hz
#define KX022_CNTL3_OTDT_400 (0x03 << 3)
// 12.5Hz
#define KX022_CNTL3_OTDT_12P5 (0x04 << 3)
// 25Hz
#define KX022_CNTL3_OTDT_25 (0x05 << 3)
// 800Hz
#define KX022_CNTL3_OTDT_800 (0x06 << 3)
// 1600Hz
#define KX022_CNTL3_OTDT_1600 (0x07 << 3)
// 0.78Hz
#define KX022_CNTL3_OWUF_0P781 (0x00 << 0)
// 1.563Hz
#define KX022_CNTL3_OWUF_1P563 (0x01 << 0)
// 3.125Hz
#define KX022_CNTL3_OWUF_3P125 (0x02 << 0)
// 6.25Hz
#define KX022_CNTL3_OWUF_6P25 (0x03 << 0)
// 12.5Hz
#define KX022_CNTL3_OWUF_12P5 (0x04 << 0)
// 25Hz
#define KX022_CNTL3_OWUF_25 (0x05 << 0)
// 50Hz
#define KX022_CNTL3_OWUF_50 (0x06 << 0)
// 100Hz
#define KX022_CNTL3_OWUF_100 (0x07 << 0)
// low-pass filter roll off control
#define KX022_ODCNTL_IIR_BYPASS (0x01 << 7)
// low pass filter enable
#define KX022_ODCNTL_LPRO (0x01 << 6)
// 12.5Hz
#define KX022_ODCNTL_OSA_12P5 (0x00 << 0)
// 25Hz
#define KX022_ODCNTL_OSA_25 (0x01 << 0)
// 50Hz
#define KX022_ODCNTL_OSA_50 (0x02 << 0)
// 100Hz
#define KX022_ODCNTL_OSA_100 (0x03 << 0)
// 200Hz
#define KX022_ODCNTL_OSA_200 (0x04 << 0)
// 400Hz
#define KX022_ODCNTL_OSA_400 (0x05 << 0)
// 800Hz
#define KX022_ODCNTL_OSA_800 (0x06 << 0)
// 1600Hz
#define KX022_ODCNTL_OSA_1600 (0x07 << 0)
// 0.78Hz
#define KX022_ODCNTL_OSA_0P781 (0x08 << 0)
// 1.563Hz
#define KX022_ODCNTL_OSA_1P563 (0x09 << 0)
// 3.125Hz
#define KX022_ODCNTL_OSA_3P125 (0x0A << 0)
// 6.25Hz
#define KX022_ODCNTL_OSA_6P25 (0x0B << 0)
// enables/disables the physical interrupt
#define KX022_INC1_IEN1 (0x01 << 5)
// sets the polarity of the physical interrupt pin
#define KX022_INC1_IEA1 (0x01 << 4)
// sets the response of the physical interrupt pin
#define KX022_INC1_IEL1 (0x01 << 3)
// sets the polarity of Self Test
#define KX022_INC1_STPOL (0x01 << 1)
// sets the 3-wire SPI interface
#define KX022_INC1_SPI3E (0x01 << 0)
// x negative (x-): 0 = disabled, 1 = enabled
#define KX022_INC2_XNWUE (0x01 << 5)
// x positive (x+): 0 = disabled, 1 = enabled
#define KX022_INC2_XPWUE (0x01 << 4)
// y negative (y-): 0 = disabled, 1 = enabled
#define KX022_INC2_YNWUE (0x01 << 3)
// y positive (y+): 0 = disabled, 1 = enabled
#define KX022_INC2_YPWUE (0x01 << 2)
// z negative (z-): 0 = disabled, 1 = enabled
#define KX022_INC2_ZNWUE (0x01 << 1)
// z positive (z+): 0 = disabled, 1 = enabled
#define KX022_INC2_ZPWUE (0x01 << 0)
// x negative (x-): 0 = disabled, 1 = enabled
#define KX022_INC3_TLEM (0x01 << 5)
// x positive (x+): 0 = disabled, 1 = enabled
#define KX022_INC3_TRIM (0x01 << 4)
// y negative (y-): 0 = disabled, 1 = enabled
#define KX022_INC3_TDOM (0x01 << 3)
// y positive (y+): 0 = disabled, 1 = enabled
#define KX022_INC3_TUPM (0x01 << 2)
// z negative (z-): 0 = disabled, 1 = enabled
#define KX022_INC3_TFDM (0x01 << 1)
// z positive (z+): 0 = disabled, 1 = enabled
#define KX022_INC3_TFUM (0x01 << 0)
// Buffer full interrupt reported on physical interrupt pin INT1
#define KX022_INC4_BFI1 (0x01 << 6)
// Watermark interrupt reported on physical interrupt pin INT1
#define KX022_INC4_WMI1 (0x01 << 5)
// Data ready interrupt reported on physical interrupt pin INT1
#define KX022_INC4_DRDYI1 (0x01 << 4)
// Tap/Double Tap interrupt reported on physical interrupt pin INT1
#define KX022_INC4_TDTI1 (0x01 << 2)
// Wake-Up (motion detect) interrupt reported on physical interrupt pin INT1
#define KX022_INC4_WUFI1 (0x01 << 1)
// Tilt position interrupt reported on physical interrupt pin INT1
#define KX022_INC4_TPI1 (0x01 << 0)
// enables/disables the physical interrupt
#define KX022_INC5_IEN2 (0x01 << 5)
// sets the polarity of the physical interrupt pin
#define KX022_INC5_IEA2 (0x01 << 4)
// sets the response of the physical interrupt pin
#define KX022_INC5_IEL2 (0x01 << 3)
// BFI2  Buffer full interrupt reported on physical interrupt pin INT2
#define KX022_INC6_BFI2 (0x01 << 6)
// WMI2 - Watermark interrupt reported on physical interrupt pin INT2
#define KX022_INC6_WMI2 (0x01 << 5)
// DRDYI2  Data ready interrupt reported on physical interrupt pin INT2
#define KX022_INC6_DRDYI2 (0x01 << 4)
// TDTI2 - Tap/Double Tap interrupt reported on physical interrupt pin INT2
#define KX022_INC6_TDTI2 (0x01 << 2)
// WUFI2  Wake-Up (motion detect) interrupt reported on physical interrupt pin INT2
#define KX022_INC6_WUFI2 (0x01 << 1)
// TPI2  Tilt position interrupt reported on physical interrupt pin INT2
#define KX022_INC6_TPI2 (0x01 << 0)
// enables/disables the double tap interrupt
#define KX022_TDTRC_DTRE (0x01 << 1)
// enables/disables single tap interrupt
#define KX022_TDTRC_STRE (0x01 << 0)
// No Averaging
#define KX022_LP_CNTL_AVC_NO_AVG (0x00 << 4)
// 2 Samples Averaged
#define KX022_LP_CNTL_AVC_2_SAMPLE_AVG (0x01 << 4)
// 4 Samples Averaged
#define KX022_LP_CNTL_AVC_4_SAMPLE_AVG (0x02 << 4)
// 8 Samples Averaged
#define KX022_LP_CNTL_AVC_8_SAMPLE_AVG (0x03 << 4)
// 16 Samples Averaged (default)
#define KX022_LP_CNTL_AVC_16_SAMPLE_AVG (0x04 << 4)
// 32 Samples Averaged
#define KX022_LP_CNTL_AVC_32_SAMPLE_AVG (0x05 << 4)
// 64 Samples Averaged
#define KX022_LP_CNTL_AVC_64_SAMPLE_AVG (0x06 << 4)
// 128 Samples Averaged
#define KX022_LP_CNTL_AVC_128_SAMPLE_AVG (0x07 << 4)
// count of samples to buffer
#define KX022_BUF_CNTL1_SMP_TH0_6 (0x7F << 0)
// controls activation of the sample buffer
#define KX022_BUF_CNTL2_BUFE (0x01 << 7)
// determines the resolution of the acceleration data samples collected by the sample
#define KX022_BUF_CNTL2_BRES (0x01 << 6)
// buffer full interrupt enable bit
#define KX022_BUF_CNTL2_BFIE (0x01 << 5)
// The buffer collects 681 sets of 8-bit low resolution values or 339 sets of 16-bit high resolution values and then stops collecting data, collecting new data only when the buffer is not full
#define KX022_BUF_CNTL2_BUF_M_FIFO (0x00 << 0)
// The buffer holds the last 681 sets of 8-bit low resolution values or 339 sets of 16-bit high resolution values. Once the buffer is full, the oldest data is discarded to make room for newer data.
#define KX022_BUF_CNTL2_BUF_M_STREAM (0x01 << 0)
// When a trigger event occurs, the buffer holds the last data set of SMP[9:0] samples before the trigger event and then continues to collect data until full. New data is collected only when the buffer is not full.
#define KX022_BUF_CNTL2_BUF_M_TRIGGER (0x02 << 0)
// The buffer holds the last 681 sets of 8-bit low resolution values or 339 sets of 16-bit high resolution values. Once the buffer is full, the oldest data is discarded to make room for newer data. Reading from the buffer in this mode will return the most recent data first.
#define KX022_BUF_CNTL2_BUF_M_FILO (0x03 << 0)
#define KX022_BUF_STATUS_1_SMP_LEV0_7 (0xFF << 0)
// reports the status of the buffers trigger function if this mode has been selected
#define KX022_BUF_STATUS_2_BUF_TRIG (0x01 << 7)
// MEMS Test OFF
#define KX022_SELF_TEST_MEMS_TEST_OFF (0x00 << 0)
// MEMS Test ON
#define KX022_SELF_TEST_MEMS_TEST_ON (0xCA << 0)
// WHO_AM_I -value for KX012
#define KX012_WHO_AM_I_WIA_ID (0x1A << 0)
// WHO_AM_I -value for KX023
#define KX023_WHO_AM_I_WIA_ID (0x15 << 0)
 /*registers bit masks */
#define KX022_COTR_DCSTR_MASK 0xFF

#define KX022_WHO_AM_I_WIA_MASK 0xFF
// status of tap/double tap, bit is released when interrupt release register INT_REL is read.
#define KX022_INS2_TDTS_MASK 0x0C
// selects the acceleration range of the accelerometer outputs
#define KX022_CNTL1_GSEL_MASK 0x18
// sets the output data rate for the Tilt Position function
#define KX022_CNTL3_OTP_MASK 0xC0
// sets the output data rate for the Directional TapTM function
#define KX022_CNTL3_OTDT_MASK 0x38
// sets the output data rate for the general motion detection function and the high-pass filtered outputs
#define KX022_CNTL3_OWUF_MASK 0x07
// acceleration output data rate.
#define KX022_ODCNTL_OSA_MASK 0x0F
#define KX022_INC2_WUE_MASK 0x3F
#define KX022_INC3_TM_MASK 0x3F
#define KX022_HYST_SET_HYST_MASK 0x3F
// Averaging Filter Control
#define KX022_LP_CNTL_AVC_MASK 0x70

#define KX022_BUF_CNTL1_SMP_TH0_MASK 0x7F
#define KX022_BUF_CNTL1_SMP_TH0_6_MASK 0x7F
// selects the operating mode of the sample buffer
#define KX022_BUF_CNTL2_BUF_M_MASK 0x03

#define KX022_BUF_STATUS_1_SMP_LEV0_MASK 0xFF
#define KX022_BUF_STATUS_1_SMP_LEV0_7_MASK 0xFF

#define KX022_SELF_TEST_MEMS_TEST_MASK 0xFF

#define KX012_WHO_AM_I_WIA_MASK 0xFF

#define KX023_WHO_AM_I_WIA_MASK 0xFF
#endif

