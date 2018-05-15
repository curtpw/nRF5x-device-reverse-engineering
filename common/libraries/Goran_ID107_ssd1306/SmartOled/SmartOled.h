/****************************************************************************** 
SFE_MicroOLED.cpp
Main source code for the MicroOLED mbed Library
Jim Lindblom @ SparkFun Electronics
October 26, 2014
https://github.com/sparkfun/Micro_OLED_Breakout/tree/master/Firmware/Arduino/libraries/SFE_MicroOLED
Adapted for mbed by Nenad Milosevic
March, 2015
Adapted for Arduino and 64x32 oled by Goran Mahovlić
Getting all to work Roger Clark
This file defines the hardware SPI interface for the Micro OLED Breakout.
Development environment specifics:
This code was heavily based around the MicroView library, written by GeekAmmo
(https://github.com/geekammo/MicroView-Arduino-Library), and released under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later 
version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#ifndef SmartOled_h
#define SmartOled_h

static inline void swap(uint8_t &a, uint8_t &b)
{
    
}
 
#ifndef _BV
#define _BV(bit) (1<<(bit))
#endif

#define BLACK 0
#define WHITE 1

#define LCDWIDTH            64
#define LCDHEIGHT           32
#define LCDCOLUMNOFFSET   0 // Visible start column within SSD1306 controller memory
#define FONTHEADERSIZE    6

#define LCDTOTALWIDTH   128  // Full width of SSD1306 controller memory
#define LCDTOTALHEIGHT   64  // Full height of SSD1306 controller memory

#define LCDTOTALPAGES   (LCDTOTALHEIGHT / 8)
#define LCDPAGES    (LCDHEIGHT / 8)

#define NORM                0
#define XOR                 1

#define PAGE                0
#define ALL                 1

#define SETCONTRAST         0x81
#define DISPLAYALLONRESUME  0xA4
#define DISPLAYALLON        0xA5
#define NORMALDISPLAY       0xA6
#define INVERTDISPLAY       0xA7
#define DISPLAYOFF          0xAE
#define DISPLAYON           0xAF
#define SETDISPLAYOFFSET    0xD3
#define SETCOMPINS          0xDA
#define SETVCOMDESELECT     0xDB
#define SETDISPLAYCLOCKDIV  0xD5
#define SETPRECHARGE        0xD9
#define SETMULTIPLEX        0xA8
#define SETLOWCOLUMN        0x00
#define SETHIGHCOLUMN       0x12
#define SETSTARTLINE        0x00
#define MEMORYMODE          0x20
#define SETCOLUMNBOUNDS     0x21
#define SETPAGEBOUNDS       0x22
#define SETPAGE             0xB0
#define COMSCANINC          0xC0
#define COMSCANDEC          0xC8
#define SEGREMAP            0xA1
#define CHARGEPUMP          0x8D
#define EXTERNALVCC         0x01
#define SWITCHCAPVCC        0x02

// Scroll
#define ACTIVATESCROLL                  0x2F
#define DEACTIVATESCROLL                0x2E
#define SETVERTICALSCROLLAREA           0xA3
#define RIGHTHORIZONTALSCROLL           0x26
#define LEFTHORIZONTALSCROLL            0x27
#define VERTICALRIGHTHORIZONTALSCROLL   0x29
#define VERTICALLEFTHORIZONTALSCROLL    0x2A


    class SmartOled
    {
        public:
            SmartOled(uint8_t pin_rst, uint8_t pin_dc, uint8_t pin_cs);
            void init(int spi_mode, int spi_freq);
    
            // Standard text output functions
            void putc2(char c);
            void puts(const char *cstring);
            void printf(const char *format, ...);

            // RAW LCD functions
            void command(uint8_t c);
            void command(uint8_t c1, uint8_t c2);
            void command(uint8_t c1, uint8_t c2, uint8_t c3);
            void command(uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5, uint8_t c6, uint8_t c7, uint8_t c8);
            void command(uint8_t cmds[],int length);    
            void setColumnAddress(uint8_t add);
            void setPageAddress(uint8_t add);

            // LCD Draw functions
            void clear(uint8_t mode);
            void clear(uint8_t mode, uint8_t c);
            void invert(boolean inv);
            void contrast(uint8_t contrast);
            void display(void);
            void setCursor(uint8_t x, uint8_t y);
            void pixel(uint8_t x, uint8_t y);
            void pixel(uint8_t x, uint8_t y, uint8_t color, uint8_t mode);
            void line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
            void line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color, uint8_t mode);
            void lineH(uint8_t x, uint8_t y, uint8_t width);
            void lineH(uint8_t x, uint8_t y, uint8_t width, uint8_t color, uint8_t mode);
            void lineV(uint8_t x, uint8_t y, uint8_t height);
            void lineV(uint8_t x, uint8_t y, uint8_t height, uint8_t color, uint8_t mode);
            void rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
            void rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode);
            void rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
            void rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode);
            void circle(uint8_t x, uint8_t y, uint8_t radius);
            void circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color, uint8_t mode);
            void circleFill(uint8_t x0, uint8_t y0, uint8_t radius);
            void circleFill(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color, uint8_t mode);
            void drawChar(uint8_t x, uint8_t y, uint8_t c);
            void drawChar(uint8_t x, uint8_t y, uint8_t c, uint8_t color, uint8_t mode);
            void drawBitmap(const uint8_t * bitArray);
            uint8_t getLCDWidth(void);
            uint8_t getLCDHeight(void);
            void setColor(uint8_t color);
            void setDrawMode(uint8_t mode);
            uint8_t *getScreenBuffer(void);

            // Font functions
            uint8_t getFontWidth(void);
            uint8_t getFontHeight(void);
            uint8_t getTotalFonts(void);
            uint8_t getFontType(void);
            uint8_t setFontType(uint8_t type);
            uint8_t getFontStartChar(void);
            uint8_t getFontTotalChar(void);

            // LCD Rotate Scroll functions  
            void scrollRight(uint8_t start, uint8_t stop);
            void scrollLeft(uint8_t start, uint8_t stop);
            void scrollStop(void);
            void flipVertical(boolean flip);
            void flipHorizontal(boolean flip);             
        private:
        	uint8_t _pin_rst;
            uint8_t _pin_dc;
            uint8_t _pin_cs;
            uint8_t foreColor, drawMode, fontWidth, fontHeight, fontType, fontStartChar, fontTotalChar, cursorX, cursorY;
            uint16_t fontMapWidth;
            static const unsigned char *fontsPointer[];
    };
#endif