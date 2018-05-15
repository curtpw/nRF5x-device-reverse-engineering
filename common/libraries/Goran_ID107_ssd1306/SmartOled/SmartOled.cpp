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


#include <stdarg.h>
#include <SPI.h>
#include "SmartOled.h"

// Add header of the fonts here.
#include "font5x7.h"
#include "font8x16.h"
#include "fontlargenumber.h"
#include "7segment.h"

// Change the total fonts included
#define TOTALFONTS    4

// Add the font name as declared in the header file.
unsigned const char *SmartOled::fontsPointer[]={
  font5x7
  ,font8x16
  ,sevensegment
  ,fontlargenumber
};

/** \brief SmartOled screen buffer.
Page buffer LCDWIDTH x LCDHEIGHT divided by 8
Page buffer is required because in SPI mode, the host cannot read the SSD1306's GDRAM of the controller.  This page buffer serves as a scratch RAM for graphical functions.  All drawing function will first be drawn on this page buffer, only upon calling display() function will transfer the page buffer to the actual LCD controller's memory.
*/
static uint8_t screenmemory [LCDWIDTH * LCDHEIGHT / 8]; 
  /* SSD1306 Memory organised in 128 horizontal pixel and 8 rows of byte
   B  B .............B  -----
   y  y .............y        \
   t  t .............t         \
   e  e .............e          \
   0  1 .............127         \
                                  \
   D0 D0.............D0            \
   D1 D1.............D1            / ROW 0
   D2 D2.............D2           /
   D3 D3.............D3          /
   D4 D4.............D4         /
   D5 D5.............D5        /
   D6 D6.............D6       /
   D7 D7.............D7  ----
  */

/** \brief Initialisation of SmartOled Library.
    Setup IO pins and parameters for SPI then send initialisation commands to the SSD1306 controller inside the OLED. 
*/

  SmartOled::SmartOled(uint8_t pin_rst, uint8_t pin_dc, uint8_t pin_cs)
  {
    pinMode(pin_rst, OUTPUT);
    digitalWrite(pin_rst,HIGH);
    pinMode(pin_dc, OUTPUT);
    digitalWrite(pin_dc,LOW);
    pinMode(pin_cs, OUTPUT);
    digitalWrite(pin_cs,HIGH);
    //
    _pin_rst = pin_rst;
    _pin_dc = pin_dc;
    _pin_cs = pin_cs;
   	SPI.begin();
  }

  void SmartOled::init(int spi_mode, int spi_freq) 
{

  // default 5x7 font
  setFontType(0);
  setColor(WHITE);
  setDrawMode(NORM);
  setCursor(0,0);
  
  memset(screenmemory,0,(LCDWIDTH * LCDHEIGHT / 8));  // initially clear Page buffer


	digitalWrite(_pin_dc,LOW);
	digitalWrite(_pin_cs,HIGH);
	SPI.begin();
	//SPI.format(8, spi_mode);	// 8 Bit wide SPI and Mode (0 - 3)
	//SPI.frequency(spi_freq);	// SPI speed in Hz

	// Display reset routine
	digitalWrite(_pin_rst,HIGH);
	delay(5);	// VDD (3.3V) goes high at start, lets just chill for 5 ms
	digitalWrite(_pin_rst,LOW);
	delay(10);	// wait 10ms
	digitalWrite(_pin_rst,HIGH);
	delay(5); // wait 5ms



//  Original Espruino commands
//  uint8_t initCmds[] = { 0xAe,0xD5,0x80,0xA8,31,0xD3,0x0,0x40,0x8D,0x14,0x20,0x00,0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,0xDb,0x40,0xA4,0xA6,0xAf};
 
//  Latest Espruino commands
  uint8_t initCmds[] = { 0xAe,0xD5,0x80,0xA8,31,0xD3,0x0,0x40,0x8D,0x14,0x20,0x01,0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,0xDb,0x40,0xA4,0xA6,0xAf };

// Init commands from the OLED Display datasheet
  uint8_t initCmdsOLED_SPEC[] = { 0xAE,    /*display off*/        
                  0x00,    /*set lower column address*/        
                  0x12,    /*set higher column address*/ 
                  0x00,    /*set display start line*/ 
                  0xB0,    /*set page address*/ 
                  0x81,    /*contract control*/ 
                  0x4f,        /*128*/  
                  0xA1,    /*set segment remap*/ 
                  0xA6,    /*normal / reverse*/ 
                  0xA8,    /*multiplex ratio*/ 
                  0x1F,        /*duty  =  1/32*/  
                  0xC8,    /*Com scan direction*/ 
                  0xD3,    /*set display offset*/ 
                  0x00, 
                  0xD5,    /*set osc division*/ 
                  0x80,     
                  0xD9,    /*set pre-charge period*/ 
                  0x1f, 
                  0xDA,    /*set COM pins*/ 
                  0x12, 
                  0xdb,    /*set vcomh*/ 
                  0x40, 
                  0x8d,    /*set charge pump enable*/ 
                  0x10, 
                  0xAF    /*display ON*/   }; 

  
  command (initCmdsOLED_SPEC,sizeof(initCmds));
/*  
  SPI.transfer(0x21);
  SPI.transfer(0);
  SPI.transfer(127);
  SPI.transfer(0x22);
  SPI.transfer(0);
  SPI.transfer(7); 
  
*/
  command(MEMORYMODE, 0);
  clear(ALL);           // Erase hardware memory inside the OLED controller to avoid random data in memory.
 // command(DISPLAYON);

}

/** \brief Send the display command byte(s)
    
    Send command(s) via SPI to SSD1306 controller.
*/
void SmartOled::command(uint8_t c) {

    digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
    digitalWrite(_pin_cs,LOW);  // SS LOW to initialize transfer
    SPI.transfer(c);          // Transfer the command byte
    digitalWrite(_pin_cs,HIGH);  // SS HIGH to end transfer
 
}
 
void SmartOled::command(uint8_t c1, uint8_t c2) {
    
    digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
    digitalWrite(_pin_cs,LOW);  // SS LOW to initialize transfer
    SPI.transfer(c1);         // Transfer the command byte
    SPI.transfer(c2);         // Transfer the first parameter
    digitalWrite(_pin_cs,HIGH);  // SS HIGH to end transfer
 
}
 
void SmartOled::command(uint8_t c1, uint8_t c2, uint8_t c3) {
    
    digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
    digitalWrite(_pin_cs,LOW);  // SS LOW to initialize transfer
    SPI.transfer(c1);         // Transfer the command byte
    SPI.transfer(c2);         // Transfer the first parameter
    SPI.transfer(c3);         // Transfer the second parameter
    digitalWrite(_pin_cs,HIGH);  // SS HIGH to end transfer
 
}
 
void SmartOled::command(uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5, uint8_t c6, uint8_t c7, uint8_t c8) {
    
    digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
    digitalWrite(_pin_cs,LOW);  // SS LOW to initialize transfer
    SPI.transfer(c1);         // Transfer the command byte
    SPI.transfer(c2);         // Transfer the first parameter
    SPI.transfer(c3);         // Transfer the second parameter
    SPI.transfer(c4);         // Transfer the third parameter
    SPI.transfer(c5);         // Transfer the fourth parameter
    SPI.transfer(c6);         // Transfer the fifth parameter
    SPI.transfer(c7);         // Transfer the sixth parameter
    SPI.transfer(c8);         // Transfer the seventh parameter
    digitalWrite(_pin_cs,HIGH);  // SS HIGH to end transfer
 
}
 
void SmartOled::command(uint8_t cmds[],int length) {
    
    digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
    digitalWrite(_pin_cs,LOW);  // SS LOW to initialize transfer
  for(int i=0;i<length;i++)
  {
    SPI.transfer(cmds[i]);
  }
    digitalWrite(_pin_cs,HIGH);  // SS HIGH to end transfer
 
} 
 
/** \brief Clear screen buffer or SSD1306's memory.
 
    To clear all GDRAM inside the LCD controller, pass in the variable mode = ALL and to clear screen page buffer pass in the variable mode = PAGE.
*/
void SmartOled::clear(uint8_t mode) {
    if (mode==ALL) 
  {
    digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
    digitalWrite(_pin_cs,LOW);  // SS LOW to initialize transfer
    
    for (int b=0;b<4;b++) 
    { 
      digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
      SPI.transfer(0xb0+b);
      SPI.transfer(0x00);
      SPI.transfer(0x12);
      digitalWrite(_pin_dc,HIGH);  // DC pin HIGH for a command
      for (int i = 0; i < 64; i++) 
      {
        SPI.transfer(0);
      }
    }
    digitalWrite(_pin_cs,HIGH);  // SS HIGH to end transfer
    }
    else
    {
        memset(screenmemory,0,(LCDWIDTH * LCDPAGES));
        //display();
    }
}
 
/** \brief Clear or replace screen buffer or SSD1306's memory with a character. 
 
    To clear GDRAM inside the LCD controller, pass in the variable mode = ALL with c character and to clear screen page buffer, pass in the variable mode = PAGE with c character.
*/
void SmartOled::clear(uint8_t mode, uint8_t c) {
    if (mode==ALL) {
    digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
    digitalWrite(_pin_cs,LOW);  // SS LOW to initialize transfer
    
    for (int b=0;b<4;b++) 
    { 
      digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
      SPI.transfer(0xb0+b);
      SPI.transfer(0x00);
      SPI.transfer(0x12);
      digitalWrite(_pin_dc,HIGH);  // DC pin LOW for a command
      for (int i = 0; i < 64; i++) 
      {
        SPI.transfer(c);
      }
    }
    digitalWrite(_pin_cs,HIGH);  // SS HIGH to end transfer

    }
    else
    {
        memset(screenmemory,c,LCDWIDTH * LCDPAGES);
        display();
    }   
}
 
/** \brief Invert display.
 
    The WHITE color of the display will turn to BLACK and the BLACK will turn to WHITE.
*/
void SmartOled::invert(boolean inv) {
    if (inv)
    command(INVERTDISPLAY);
    else
    command(NORMALDISPLAY);
}
 
/** \brief Set contrast.
 
    OLED contract value from 0 to 255. Note: Contrast level is not very obvious.
*/
void SmartOled::contrast(uint8_t contrast) {
    command(SETCONTRAST, contrast);         // 0x81
}
 
/** \brief Transfer display memory.
 
    Bulk move the screen buffer to the SSD1306 controller's memory so that images/graphics drawn on the screen buffer will be displayed on the OLED.
*/
void SmartOled::display(void) {

    digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
    digitalWrite(_pin_cs,LOW);  // SS LOW to initialize transfer
  

  
  for (int b=0;b<4;b++) 
  { 
    digitalWrite(_pin_dc,LOW);  // DC pin LOW for a command
    SPI.transfer(0xb0+b);
    SPI.transfer(0x00);
    SPI.transfer(0x12);
    digitalWrite(_pin_dc,HIGH);  // DC pin LOW for a command
    for (int i = 0; i < 64; i++) 
    {
      SPI.transfer(screenmemory[i+64*b]);
    }
  }
    digitalWrite(_pin_cs,HIGH);  // SS HIGH to end transfer
}
 
/*
    Classic text print functions.
*/
 
void SmartOled::putc2(char c) {
    if (c == '\n') {
        cursorY += fontHeight;
        cursorX  = 0;
    } else if (c == '\r') {
        // skip 
    } else {
        drawChar(cursorX, cursorY, (uint8_t)c, foreColor, drawMode);
        cursorX += fontWidth+1;
        if ((cursorX > (LCDWIDTH - fontWidth))) {
            cursorY += fontHeight;
            cursorX = 0;
        }
    }
}
 
void SmartOled::puts(const char *cstring) {
    while (*cstring != 0) {
        putc2(*cstring++);
    }
}
 
void SmartOled::printf(const char *format, ...)
{
    static char buffer[128];
    
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
    
    char *c = (char *)&buffer;
    while (*c != 0)
    {
        putc2(*c++);
    }
}
 
/** \brief Set cursor position.
 
SmartOled's cursor position to x,y.
*/
void SmartOled::setCursor(uint8_t x, uint8_t y) {
    cursorX=x;
    cursorY=y;
}
 
/** \brief Draw pixel.
 
Draw pixel using the current fore color and current draw mode in the screen buffer's x,y position.
*/
void SmartOled::pixel(uint8_t x, uint8_t y) {
    pixel(x,y,foreColor,drawMode);
}
 
/** \brief Draw pixel with color and mode.
 
Draw color pixel in the screen buffer's x,y position with NORM or XOR draw mode.
*/
void SmartOled::pixel(uint8_t x, uint8_t y, uint8_t color, uint8_t mode) {
    if ((x>=LCDWIDTH) || (y>=LCDHEIGHT))
    return;
    
    if (mode==XOR) {
        if (color==WHITE)
        screenmemory[x+ (y/8)*LCDWIDTH] ^= _BV((y%8));
    }
    else {
        if (color==WHITE)
        screenmemory[x+ (y/8)*LCDWIDTH] |= _BV((y%8));
        else
        screenmemory[x+ (y/8)*LCDWIDTH] &= ~_BV((y%8)); 
    }
}
 
/** \brief Draw line.
 
Draw line using current fore color and current draw mode from x0,y0 to x1,y1 of the screen buffer.
*/
void SmartOled::line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    line(x0,y0,x1,y1,foreColor,drawMode);
}
 
/** \brief Draw line with color and mode.
 
Draw line using color and mode from x0,y0 to x1,y1 of the screen buffer.
*/
void SmartOled::line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color, uint8_t mode) {
    uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        swap(x0, y0);
        swap(x1, y1);
    }
 
    if (x0 > x1) {
        swap(x0, x1);
        swap(y0, y1);
    }
 
    uint8_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);
 
    int8_t err = dx / 2;
    int8_t ystep;
 
    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;}
 
    for (; x0<x1; x0++) {
        if (steep) {
            pixel(y0, x0, color, mode);
        } else {
            pixel(x0, y0, color, mode);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }   
}
 
/** \brief Draw horizontal line.
 
Draw horizontal line using current fore color and current draw mode from x,y to x+width,y of the screen buffer.
*/
void SmartOled::lineH(uint8_t x, uint8_t y, uint8_t width) {
    line(x,y,x+width,y,foreColor,drawMode);
}
 
/** \brief Draw horizontal line with color and mode.
 
Draw horizontal line using color and mode from x,y to x+width,y of the screen buffer.
*/
void SmartOled::lineH(uint8_t x, uint8_t y, uint8_t width, uint8_t color, uint8_t mode) {
    line(x,y,x+width,y,color,mode);
}
 
/** \brief Draw vertical line.
 
Draw vertical line using current fore color and current draw mode from x,y to x,y+height of the screen buffer.
*/
void SmartOled::lineV(uint8_t x, uint8_t y, uint8_t height) {
    line(x,y,x,y+height,foreColor,drawMode);
}
 
/** \brief Draw vertical line with color and mode.
 
Draw vertical line using color and mode from x,y to x,y+height of the screen buffer.
*/
void SmartOled::lineV(uint8_t x, uint8_t y, uint8_t height, uint8_t color, uint8_t mode) {
    line(x,y,x,y+height,color,mode);
}
 
/** \brief Draw rectangle.
 
Draw rectangle using current fore color and current draw mode from x,y to x+width,y+height of the screen buffer.
*/
void SmartOled::rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
    rect(x,y,width,height,foreColor,drawMode);
}
 
/** \brief Draw rectangle with color and mode.
 
Draw rectangle using color and mode from x,y to x+width,y+height of the screen buffer.
*/
void SmartOled::rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode) {
    uint8_t tempHeight;
    
    lineH(x,y, width, color, mode);
    lineH(x,y+height-1, width, color, mode);
    
    tempHeight=height-2;
    
    // skip drawing vertical lines to avoid overlapping of pixel that will 
    // affect XOR plot if no pixel in between horizontal lines      
    if (tempHeight<1) return;           
 
    lineV(x,y+1, tempHeight, color, mode);
    lineV(x+width-1, y+1, tempHeight, color, mode);
}
 
/** \brief Draw filled rectangle.
 
Draw filled rectangle using current fore color and current draw mode from x,y to x+width,y+height of the screen buffer.
*/
void SmartOled::rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
    rectFill(x,y,width,height,foreColor,drawMode);
}
 
/** \brief Draw filled rectangle with color and mode.
 
Draw filled rectangle using color and mode from x,y to x+width,y+height of the screen buffer.
*/  
void SmartOled::rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode) {
    // TODO - need to optimise the memory map draw so that this function will not call pixel one by one
    for (int i=x; i<x+width;i++) {
        lineV(i,y, height, color, mode);
    }
}
 
/** \brief Draw circle.
 
    Draw circle with radius using current fore color and current draw mode at x,y of the screen buffer.
*/
void SmartOled::circle(uint8_t x0, uint8_t y0, uint8_t radius) {
    circle(x0,y0,radius,foreColor,drawMode);
}
 
/** \brief Draw circle with color and mode.
 
Draw circle with radius using color and mode at x,y of the screen buffer.
*/
void SmartOled::circle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color, uint8_t mode) {
    //TODO - find a way to check for no overlapping of pixels so that XOR draw mode will work perfectly 
    int8_t f = 1 - radius;
    int8_t ddF_x = 1;
    int8_t ddF_y = -2 * radius;
    int8_t x = 0;
    int8_t y = radius;
 
    pixel(x0, y0+radius, color, mode);
    pixel(x0, y0-radius, color, mode);
    pixel(x0+radius, y0, color, mode);
    pixel(x0-radius, y0, color, mode);
 
    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
 
        pixel(x0 + x, y0 + y, color, mode);
        pixel(x0 - x, y0 + y, color, mode);
        pixel(x0 + x, y0 - y, color, mode);
        pixel(x0 - x, y0 - y, color, mode);
        
        pixel(x0 + y, y0 + x, color, mode);
        pixel(x0 - y, y0 + x, color, mode);
        pixel(x0 + y, y0 - x, color, mode);
        pixel(x0 - y, y0 - x, color, mode);
        
    }
}
 
/** \brief Draw filled circle.
 
    Draw filled circle with radius using current fore color and current draw mode at x,y of the screen buffer.
*/
void SmartOled::circleFill(uint8_t x0, uint8_t y0, uint8_t radius) {
    circleFill(x0,y0,radius,foreColor,drawMode);
}
 
/** \brief Draw filled circle with color and mode.
 
    Draw filled circle with radius using color and mode at x,y of the screen buffer.
*/
void SmartOled::circleFill(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color, uint8_t mode) {
    // TODO - - find a way to check for no overlapping of pixels so that XOR draw mode will work perfectly 
    int8_t f = 1 - radius;
    int8_t ddF_x = 1;
    int8_t ddF_y = -2 * radius;
    int8_t x = 0;
    int8_t y = radius;
 
    // Temporary disable fill circle for XOR mode.
    if (mode==XOR) return;
    
    for (uint8_t i=y0-radius; i<=y0+radius; i++) {
        pixel(x0, i, color, mode);
    }
 
    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
 
        for (uint8_t i=y0-y; i<=y0+y; i++) {
            pixel(x0+x, i, color, mode);
            pixel(x0-x, i, color, mode);
        } 
        for (uint8_t i=y0-x; i<=y0+x; i++) {
            pixel(x0+y, i, color, mode);
            pixel(x0-y, i, color, mode);
        }    
    }
}
 
/** \brief Get LCD height.
 
    The height of the LCD return as byte.
*/
uint8_t SmartOled::getLCDHeight(void) {
    return LCDHEIGHT;
}
 
/** \brief Get LCD width.
 
    The width of the LCD return as byte.
*/  
uint8_t SmartOled::getLCDWidth(void) {
    return LCDWIDTH;
}
 
/** \brief Get font width.
 
    The cucrrent font's width return as byte.
*/  
uint8_t SmartOled::getFontWidth(void) {
    return fontWidth;
}
 
/** \brief Get font height.
 
    The current font's height return as byte.
*/
uint8_t SmartOled::getFontHeight(void) {
    return fontHeight;
}
 
/** \brief Get font starting character.
 
    Return the starting ASCII character of the currnet font, not all fonts start with ASCII character 0. Custom fonts can start from any ASCII character.
*/
uint8_t SmartOled::getFontStartChar(void) {
    return fontStartChar;
}
 
/** \brief Get font total characters.
 
    Return the total characters of the current font.
*/
uint8_t SmartOled::getFontTotalChar(void) {
    return fontTotalChar;
}
 
/** \brief Get total fonts.
 
    Return the total number of fonts loaded into the SmartOled's flash memory.
*/
uint8_t SmartOled::getTotalFonts(void) {
    return TOTALFONTS;
}
 
/** \brief Get font type.
 
    Return the font type number of the current font.
*/
uint8_t SmartOled::getFontType(void) {
    return fontType;
}
 
/** \brief Set font type.
 
    Set the current font type number, ie changing to different fonts base on the type provided.
*/
uint8_t SmartOled::setFontType(uint8_t type) {
    if (type>=TOTALFONTS)
    return false;
 
    fontType = type;
    fontWidth = *(fontsPointer[fontType]+0);
    fontHeight = *(fontsPointer[fontType]+1);
    fontStartChar = *(fontsPointer[fontType]+2);
    fontTotalChar = *(fontsPointer[fontType]+3);
    fontMapWidth = (*(fontsPointer[fontType]+4) * 100) + *(fontsPointer[fontType]+5); // two bytes values into integer 16
    return true;
}
 
/** \brief Set color.
 
    Set the current draw's color. Only WHITE and BLACK available.
*/
void SmartOled::setColor(uint8_t color) {
    foreColor=color;
}
 
/** \brief Set draw mode.
 
    Set current draw mode with NORM or XOR.
*/
void SmartOled::setDrawMode(uint8_t mode) {
    drawMode=mode;
}
 
/** \brief Draw character.
 
    Draw character c using current color and current draw mode at x,y.
*/
void  SmartOled::drawChar(uint8_t x, uint8_t y, uint8_t c) {
    drawChar(x,y,c,foreColor,drawMode);
}
 
/** \brief Draw character with color and mode.
 
    Draw character c using color and draw mode at x,y.
*/
void  SmartOled::drawChar(uint8_t x, uint8_t y, uint8_t c, uint8_t color, uint8_t mode) {
    // TODO - New routine to take font of any height, at the moment limited to font height in multiple of 8 pixels
 
    uint8_t rowsToDraw,row, tempC;
    uint8_t i,j,temp;
    uint16_t charPerBitmapRow,charColPositionOnBitmap,charRowPositionOnBitmap,charBitmapStartPosition;
    
    if ((c<fontStartChar) || (c>(fontStartChar+fontTotalChar-1)))       // no bitmap for the required c
    return;
    
    tempC=c-fontStartChar;
 
    // each row (in datasheet is call page) is 8 bits high, 16 bit high character will have 2 rows to be drawn
    rowsToDraw=fontHeight/8;    // 8 is LCD's page size, see SSD1306 datasheet
    if (rowsToDraw<=1) rowsToDraw=1;
 
    // the following draw function can draw anywhere on the screen, but SLOW pixel by pixel draw
    if (rowsToDraw==1) {
        for  (i=0;i<fontWidth+1;i++) {
            if (i==fontWidth) // this is done in a weird way because for 5x7 font, there is no margin, this code add a margin after col 5
            temp=0;
            else
            temp = *(fontsPointer[fontType]+FONTHEADERSIZE+(tempC*fontWidth)+i);
            
            for (j=0;j<8;j++) {         // 8 is the LCD's page height (see datasheet for explanation)
                if (temp & 0x1) {
                    pixel(x+i, y+j, color,mode);
                }
                else {
                    pixel(x+i, y+j, !color,mode);
                }
                
                temp >>=1;
            }
        }
        return;
    }
 
    // font height over 8 bit
    // take character "0" ASCII 48 as example
    charPerBitmapRow=fontMapWidth/fontWidth;  // 256/8 =32 char per row
    charColPositionOnBitmap=tempC % charPerBitmapRow;  // =16
    charRowPositionOnBitmap=int(tempC/charPerBitmapRow); // =1
    charBitmapStartPosition=(charRowPositionOnBitmap * fontMapWidth * (fontHeight/8)) + (charColPositionOnBitmap * fontWidth) ;
 
    // each row on LCD is 8 bit height (see datasheet for explanation)
    for(row=0;row<rowsToDraw;row++) {
        for (i=0; i<fontWidth;i++) {
            temp = *(fontsPointer[fontType]+FONTHEADERSIZE+(charBitmapStartPosition+i+(row*fontMapWidth)));
            for (j=0;j<8;j++) {         // 8 is the LCD's page height (see datasheet for explanation)
                if (temp & 0x1) {
                    pixel(x+i,y+j+(row*8), color, mode);
                }
                else {
                    pixel(x+i,y+j+(row*8), !color, mode);
                }
                temp >>=1;
            }
        }
    }
 
}
 
/** \brief Stop scrolling.
 
    Stop the scrolling of graphics on the OLED.
*/
void SmartOled::scrollStop(void){
    command(DEACTIVATESCROLL);
}
 
/** \brief Right scrolling.
 
Set row start to row stop on the OLED to scroll right. Refer to http://learn.microview.io/intro/general-overview-of-microview.html for explanation of the rows.
*/
void SmartOled::scrollRight(uint8_t start, uint8_t stop){
    if (stop<start)     // stop must be larger or equal to start
    return;
    scrollStop();       // need to disable scrolling before starting to avoid memory corrupt
    command(RIGHTHORIZONTALSCROLL, 0x00, start, 0x07, stop, 0x00, 0xFF, ACTIVATESCROLL); // scroll speed frames , TODO
}
 
/** \brief Left scrolling.
    Set row start to row stop on the OLED to scroll left. Refer to http://learn.microview.io/intro/general-overview-of-microview.html for explanation of the rows.
*/
void SmartOled::scrollLeft(uint8_t start, uint8_t stop){
    if (stop<start)     // stop must be larger or equal to start
    return;
    scrollStop();       // need to disable scrolling before starting to avoid memory corrupt
    command(LEFTHORIZONTALSCROLL, 0x00, start, 0x07, stop, 0x00, 0xFF, ACTIVATESCROLL); // scroll speed frames , TODO
}
 
/** \brief Vertical flip.
 
Flip the graphics on the OLED vertically.
*/
void SmartOled::flipVertical(boolean flip) {
    if (flip) {
        command(COMSCANINC);
    }
    else {
        command(COMSCANDEC);
    }
}
 
/** \brief Horizontal flip.
 
    Flip the graphics on the OLED horizontally.
*/  
void SmartOled::flipHorizontal(boolean flip) {
    if (flip) {
        command(SEGREMAP | 0x0);
    }
    else {
        command(SEGREMAP | 0x1);
    }
}
 
/*
    Return a pointer to the start of the RAM screen buffer for direct access.
*/
uint8_t *SmartOled::getScreenBuffer(void) {
    return screenmemory;
}
 
/*
Draw Bitmap image on screen. The array for the bitmap can be stored in main program file, so user don't have to mess with the library files. 
To use, create const uint8_t array that is LCDWIDTH x LCDHEIGHT pixels (LCDWIDTH * LCDHEIGHT / 8 bytes). Then call .drawBitmap and pass it the array. 
*/  
void SmartOled::drawBitmap(const uint8_t * bitArray)
{
    for (int i=0; i<(LCDWIDTH * LCDHEIGHT / 8); i++)
        screenmemory[i] = bitArray[i];
}

/** \brief Set SSD1306 page address.
  Send page address command and address to the SSD1306 OLED controller.
*/
void SmartOled::setPageAddress(uint8_t add) {
  command(SETPAGE|add);
  return;
}

/** \brief Set SSD1306 column address.
  Send column address command and address to the SSD1306 OLED controller.
*/
void SmartOled::setColumnAddress(uint8_t add) {
  command((SETHIGHCOLUMN|(add>>4))+0x02, SETLOWCOLUMN|(0x0f&add));
  return;
}