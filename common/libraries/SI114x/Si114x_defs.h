
// A Silicon Laboratories Definitions File 
// just included for reference with Modern Device's SI114 Pulse Sensor

//-----------------------------------------------------------------------------
// Si114x_defs.hPARAM
//-----------------------------------------------------------------------------
// Copyright 2011 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// File Description:  
//
// Register/bit definitions for the Si114x family.
//
// Target:         Si114x 
// Command Line:   None
//
// Engineering Version
// 
//-----------------------------------------------------------------------------

#ifndef SI114x_DEFS_H
#define SI114x_DEFS_H

// I2C Registers
#define SI114_REG_PART_ID               0x00
#define SI114_REG_REV_ID                0x01
#define SI114_REG_SEQ_ID                0x02
#define SI114_REG_IRQ_CFG               0x03
#define SI114_REG_IRQ_ENABLE            0x04
#define SI114_REG_IRQ_MODE1             0x05
#define SI114_REG_IRQ_MODE2             0x06
#define SI114_REG_HW_KEY                0x07
#define SI114_REG_MEAS_RATE             0x08
#define SI114_REG_ALS_RATE              0x09
#define SI114_REG_PS_RATE               0x0A
#define SI114_REG_ALS_LO_TH             0x0B
#define SI114_REG_ALS_HI_TH             0x0D
#define SI114_REG_ALS_IR_ADCMUX         0x0E
#define SI114_REG_PS_LED21              0x0F
#define SI114_REG_PS_LED3               0x10
#define SI114_REG_PS1_TH                0x11
#define SI114_REG_PS2_TH                0x12
#define SI114_REG_PS3_TH                0x13
#define SI114_REG_PS_LED3_TH0           0x15
#define SI114_REG_PARAM_IN              0x17
#define SI114_REG_PARAM_WR              0x17
#define SI114_REG_COMMAND               0x18
#define SI114_REG_RESPONSE              0x20
#define SI114_REG_IRQ_STATUS            0x21
#define SI114_REG_ALS_VIS_DATA0         0x22
#define SI114_REG_ALS_VIS_DATA1         0x23
#define SI114_REG_ALS_IR_DATA0          0x24
#define SI114_REG_ALS_IR_DATA1          0x25
#define SI114_REG_PS1_DATA0             0x26
#define SI114_REG_PS1_DATA1             0x27
#define SI114_REG_PS2_DATA0             0x28
#define SI114_REG_PS2_DATA1             0x29
#define SI114_REG_PS3_DATA0             0x2A
#define SI114_REG_PS3_DATA1             0x2B
#define SI114_REG_AUX_DATA0             0x2C
#define SI114_REG_AUX_DATA1             0x2D
#define SI114_REG_PARAM_OUT             0x2E
#define SI114_REG_PARAM_RD              0x2E
#define SI114_REG_CHIP_STAT             0x30


// Parameter Offsets
#define SI114_PARAM_I2C_ADDR            0x00
#define SI114_PARAM_CH_LIST             0x01
#define SI114_PARAM_PSLED12_SELECT      0x02
#define SI114_PARAM_PSLED3_SELECT       0x03
#define SI114_PARAM_FILTER_EN           0x04
#define SI114_PARAM_PS_ENCODING         0x05
#define SI114_PARAM_ALS_ENCODING        0x06
#define SI114_PARAM_PS1_ADC_MUX         0x07
#define SI114_PARAM_PS2_ADC_MUX         0x08
#define SI114_PARAM_PS3_ADC_MUX         0x09
#define SI114_PARAM_PS_ADC_COUNTER      0x0A
#define SI114_PARAM_PS_ADC_CLKDIV       0x0B
#define SI114_PARAM_PS_ADC_GAIN         0x0B
#define SI114_PARAM_PS_ADC_MISC         0x0C
#define SI114_PARAM_ALS1_ADC_MUX        0x0D
#define SI114_PARAM_ALS2_ADC_MUX        0x0E
#define SI114_PARAM_ALS3_ADC_MUX        0x0F
#define SI114_PARAM_ALSVIS_ADC_COUNTER  0x10
#define SI114_PARAM_ALSVIS_ADC_CLKDIV   0x11
#define SI114_PARAM_ALSVIS_ADC_GAIN     0x11
#define SI114_PARAM_ALSVIS_ADC_MISC     0x12
#define SI114_PARAM_ALS_HYST            0x16
#define SI114_PARAM_PS_HYST             0x17
#define SI114_PARAM_PS_HISTORY          0x18
#define SI114_PARAM_ALS_HISTORY         0x19
#define SI114_PARAM_ADC_OFFSET          0x1A
#define SI114_PARAM_SLEEP_CTRL          0x1B
#define SI114_PARAM_LED_RECOVERY        0x1C
#define SI114_PARAM_ALSIR_ADC_COUNTER   0x1D
#define SI114_PARAM_ALSIR_ADC_CLKDIV    0x1E
#define SI114_PARAM_ALSIR_ADC_GAIN      0x1E
#define SI114_PARAM_ALSIR_ADC_MISC      0x1F



// REG_IRQ_CFG 
#define SI114_ ICG_INTOE                0x01
#define SI114_ ICG_INTMODE              0x02


// REG_IRQ_ENABLE
// REG_IRQ_STATUS
#define SI114_IE_NONE                   0x00

#define SI114_IE_ALS_NONE               0x00
#define SI114_IE_ALS_EVRYSAMPLE         0x01
#define SI114_IE_ALS_EXIT_WIN           0x01
#define SI114_IE_ALS_ENTER_WIN          0x02

#define SI114_IE_PS1_NONE               0x00
#define SI114_IE_PS1_EVRYSAMPLE         0x04
#define SI114_IE_PS1_CROSS_TH           0x04
#define SI114_IE_PS1_EXCEED_TH          0x04
#define SI114_IE_PS1                    0x04

#define SI114_IE_PS2_NONE               0x00
#define SI114_IE_PS2_EVRYSAMPLE         0x08
#define SI114_IE_PS2_CROSS_TH           0x08
#define SI114_IE_PS2_EXCEEED_TH         0x08
#define SI114_IE_PS2                    0x08

#define SI114_IE_PS3_NONE               0x00
#define SI114_IE_PS3_EVRYSAMPLE         0x10
#define SI114_IE_PS3_CROSS_TH           0x10
#define SI114_IE_PS3_EXCEED_TH          0x10
#define SI114_IE_PS3                    0x10

#define SI114_IE_CMD                    0x20

#define SI114_IE_ALL                    0x3F

// REG_IRQ_MODE1
#define SI114_IM1_NONE                  0x00
#define SI114_IM1_ALS_NONE              0x00
#define SI114_IM1_ALS_EVRYSAMPLE        0x00
#define SI114_IM1_ALS_EXIT_WIN          0x01
#define SI114_IM1_ALS_ENTER_WIN         0x40
#define SI114_IM1_ALS_CHOOSE_IR         0x20

#define SI114_IM1_PS1_NONE              0x00
#define SI114_IM1_PS1_EVRYSAMPLE        (0x0<<4)
#define SI114_IM1_PS1_CROSS_TH          (0x1<<4)
#define SI114_IM1_PS1_EXCEED_TH         (0x3<<4)

#define SI114_IM1_PS2_NONE              0x00
#define SI114_IM1_PS2_EVRYSAMPLE        (0x0<<6)
#define SI114_IM1_PS2_CROSS_TH          (0x1<<6)
#define SI114_IM1_PS2_EXCEED_TH         (0x3<<6)


// REG_IRQ_MODE1
#define SI114_IM2_PS3_NONE              0x00
#define SI114_IM2_PS3_EVRYSAMPLE        (0x0)
#define SI114_IM2_PS3_CROSS_TH          (0x1)
#define SI114_IM2_PS3_EXCEED_TH         (0x3)


//
// REG_PS_LED21   LED2 Current is upper nibble
//                LED1 Current is lower nibble 
//
// REG_PS_LED3    LED3 Current is lower nibble
#define SI114_LEDI_000                  0x00
#define SI114_LEDI_006                  0x01
#define SI114_LEDI_011                  0x02
#define SI114_LEDI_022                  0x03
#define SI114_LEDI_045                  0x04
#define SI114_LEDI_067                  0x05
#define SI114_LEDI_090                  0x06
#define SI114_LEDI_112                  0x07
#define SI114_LEDI_135                  0x08
#define SI114_LEDI_157                  0x09
#define SI114_LEDI_180                  0x0A
#define SI114_LEDI_202                  0x0B
#define SI114_LEDI_224                  0x0C
#define SI114_LEDI_269                  0x0D
#define SI114_LEDI_314                  0x0E
#define SI114_LEDI_359                  0x0F
#define SI114_MIN_LED_CURRENT           LEDI_006  // Do not set this to LEDI_000
#define SI114_MAX_LED_CURRENT           LEDI_359




// PARAM_CH_LIST
#define SI114_PS1_TASK                  0x01
#define SI114_PS2_TASK                  0x02
#define SI114_PS3_TASK                  0x04
#define SI114_ALS_VIS_TASK              0x10
#define SI114_ALS_IR_TASK               0x20
#define SI114_AUX_TASK                  0x40

//
// ADC Counters
// PARAM_PS_ADC_COUNTER      
// PARAM_ALSVIS_ADC_COUNTER  
// PARAM_ALSIR_ADC_COUNTER  
//
#define SI114_RECCNT_001                0x00
#define SI114_RECCNT_007                0x10
#define SI114_RECCNT_015                0x20
#define SI114_RECCNT_031                0x30
#define SI114_RECCNT_063                0x40
#define SI114_RECCNT_127                0x50
#define SI114_RECCNT_255                0x60
#define SI114_RECCNT_511                0x70

//
// Proximity LED Selection
// PARAM_PSLED12_SELECT  PS2 LED Choice is Upper Nibble
//                       PS1 LED Choice is Lower Nibble 
//
// PARAM_PSLED3_SELECT   PS3 LED Choice is Lower Nibble
//
// Each of the three PS measurements can choose whichever
// irLED to light up during the measurement, with whichever
// combination desired.
#define SI114_NO_LED                    0x00
#define SI114_LED1_EN                   0x01
#define SI114_LED2_EN                   0x02
#define SI114_LED3_EN                   0x04
#define SI114_SEL_LED1_PS1              (LED1_EN)
#define SI114_SEL_LED2_PS1              (LED2_EN)
#define SI114_SEL_LED3_PS1              (LED3_EN)
#define SI114_SEL_LED1_PS2              (LED1_EN<<4)
#define SI114_SEL_LED2_PS2              (LED2_EN<<4)
#define SI114_SEL_LED3_PS2              (LED3_EN<<4)
#define SI114_SEL_LED1_PS3              (LED1_EN)
#define SI114_SEL_LED2_PS3              (LED2_EN)
#define SI114_SEL_LED3_PS3              (LED3_EN)

//
// PARAM_PS_ENCODING  
// When these bits are set the corresponding measurement 
// will report the least significant bits of the
// ADC is used instead of the most significant bits
#define SI114_PS1_LSB                   0x10
#define SI114_PS2_LSB                   0x20
#define SI114_PS3_LSB                   0x40
#define SI114_PS_ENCODING_MASK          0x70

//
// PARAM_ALS_ENCODING 
// When these bits are set the corresponding measurement 
// will report the least significant bits of the
// ADC is used instead of the most significant bits
#define SI114_ALS_VIS_LSB               0x10
#define SI114_ALS_IR_LSB                0x20
#define SI114_AUX_LSB                   0x40
#define SI114_ALS_ENCODING_MASK         0xCF


//
// PARAM_PS_ADC_MISC         
// PARAM_ALS_VIS_ADC_MISC         
// PARAM_ALS_IR_ADC_MISC         
//
// PS_MODE_MEAS_MODE and NOT_PS_MEAS_MODE are applicable only
// for PARAM_PS_ADC_MISC.
//
//    PS_MEAS_MODE is used to perform normal Proximity measurements. 
//    While in this operatinal mode, it is possible to choose either
//    the small IR photodiode or big IR photodiode. The big IR
//    photodiode is the default for normal operation. The small IR
//    photodiode is typically not used, but it is possible to use it
//    for proximity measurements.
//
//    NOT_PS_MEAS_MODE can be applied to PARAM_PS_ADC_MISC.
//
//    This allows three PS channels to perform RAW ADC measurements 
//    on any source (no irLED driven). These measurements will have no
//    reference, and will have an offset of 0x4000 if the most 
//    significant 16 bits of the 17-bit ADC is reported. Otherwise, 
//    if the PARAM_xxx_ENCODING _LSB settings are set, then the 
//    offset will be 0x8000 (due to bit shifting). 
//
//    When performing voltage measurements (INT, LED1, LED2, TEMP), 
//    a separate VSS measurement should be subtracted from the reading.
//
//    When performing measurement with visible photodiode, first take
//    a no-led measurement and subtract away the visible light measurement.
//
//    When performing an optical measurement using either two IR 
//    photodiodes, one should subtract away the no-led measurement from 
//    the IR measurement.
//
//    Note that the subtraction ordering is significant between
//    taking visible light photodiode vs IR photodiode
//    measurement. The raw ADC reading of visible light photodiode
//    decreases with increasing light levels while the raw ADC reading 
//    of IR light photodiode increases with increasing light levels.
//
// HSIG_EN means 'high signal range enable'. The ADC would
// be able to operate with higher light levels, but at the
// expense of sensitivity. This setting can be used for 
// operation under direct sunlight.
//
// PARAM_PS_ADC_MISC, PARAM_ALSVIS_ADC_MISC and PARAM_ALSIR_ADC_MISC    
// can use HSIG_EN also.
// 
//
#define SI114_NOT_PS_MEAS_MODE          0x00 
#define SI114_PS_MEAS_MODE              0x04
#define SI114_HSIG_EN                   0x20
#define SI114_RANGE_EN                  0x20

#define SI114_ALS_IR_ADC_MISC_MASK      0x20
#define SI114_ALS_VIS_ADC_MISC_MASK     0x20

//
// ADC Mux Settings
// PARAM_PS1_ADC_MUX    See PARAM_PS_ADC_MISC also 
// PARAM_PS2_ADC_MUX    See PARAM_PS_ADC_MISC also
// PARAM_PS3_ADC_MUX    See PARAM_PS_ADC_MISC also
//
// PARAM_ALS1_ADC_MUX   MUX_ALS_VIS or MUX_NONE only
// PARAM_ALS2_ADC_MUX   MUX_ALS_IR, MUX_PS_IR or MUX_NONE only
// PARAM_ALS3_ADC_MUX   MUX_VTEMP, MUX_LED1, MUX_LED2, MUX_INT
//                      to use anything other than MUX_VTEMP, 
//                      ANA_IN_KEY should be unlocked first.
//
#define SI114_MUX_SMALL_IR              0x00
#define SI114_MUX_VIS                   0x02
#define SI114_MUX_LARGE_IR              0x03
#define SI114_MUX_NO_PHOTO_DIODE        0x06
#define SI114_MUX_VTEMP      	          0x65
#define SI114_MUX_INT      	          0x05
#define SI114_MUX_LED1      	          0x15
#define SI114_MUX_VSS      	          0x25
#define SI114_MUX_LED2      	          0x35
#define SI114_MUX_VDD      	          0x75

//
// ADC Dividers
// PARAM_PS_ADC_GAIN       
// PARAM_ALSVIS_ADC_GAIN  
// PARAM_ALSIR_ADC_GAIN 
//
#define SI114_ADC_NORM                  0x00
#define SI114_ADC_DIV2                  0x01
#define SI114_ADC_DIV4                  0x02
#define SI114_ADC_DIV8                  0x03
#define SI114_ADC_DIV16                 0x04
#define SI114_ADC_DIV32                 0x05
#define SI114_ADC_DIV64                 0x06
#define SI114_ADC_DIV128                0x07
#define SI114_ADC_DIV256                0x08
#define SI114_ADC_DIV512                0x09
#define SI114_ADC_DIV1024               0x0A
#define SI114_ADC_DIV2048               0x0B


// Hardware Key value
// REG_HW_KEY
#define SI114_HW_KEY_VAL0               0x17

// Sleep Control
// PARAM_SLEEP_CTRL 
#define SI114_SLEEP_DISABLED            0x01

// ANA_IN_KEY value
#define SI114_ANA_KEY_38                0x10
#define SI114_ANA_KEY_39                0x40
#define SI114_AMA_KEY_3A                0x62
#define SI114_ANA_KEY_3B                0x3b


#define SI114_VIS_STATIC_WINDOW_SCALE    0 /* max is 7 */ 


//
// Macro Definitions that should be modified when
// Different Si114x ADC Settings are used
//

#define SI114_IRAMB_SUNLIGHT_LOWER_THRESH               1300
#define SI114_IRAMB_HIGH_SIGNAL_UPPER_THRESH           22000
#define SI114_IRAMB_HIGH_SIGNAL_LOWER_THRESH            1300
#define SI114_IRAMB_HIGH_SENSITIVITY_UPPER_THRESH      22000
#define SI114_IRAMB_HIGH_SENSITIVITY_LOWER_THRESH       1300
#define SI114_IRAMB_LOWLIGHT_UPPER_THRESH              22000

#define SI114_VISAMB_SUNLIGHT_LOWER_THRESH               700  
#define SI114_VISAMB_HIGH_SIGNAL_UPPER_THRESH          22000
#define SI114_VISAMB_HIGH_SIGNAL_LOWER_THRESH           5000
#define SI114_VISAMB_HIGH_SENSITIVITY_UPPER_THRESH     22000
#define SI114_VISAMB_HIGH_SENSITIVITY_LOWER_THRESH      5000
#define SI114_VISAMB_LOWLIGHT_UPPER_THRESH             22000

#define SI114_IR_SCALING_SUNLIGHT                       2940 // 1*16*12.5 // 3058 1*16*13*14.7   
#define SI114_IR_SCALING_HIGH_SIGNAL                     200 // 1*16*12.5 //  208 1*16*13    
#define SI114_IR_SCALING_HIGH_SENSITIVITY                 16 // 1*16
#define SI114_IR_SCALING_LOWLIGHT                          1 // 1

#define SI114_VIS_SCALING_SUNLIGHT                       470   
#define SI114_VIS_SCALING_HIGH_SIGNAL                     16     
#define SI114_VIS_SCALING_HIGH_SENSITIVITY                 4   
#define SI114_VIS_SCALING_LOWLIGHT                         1    



#endif
