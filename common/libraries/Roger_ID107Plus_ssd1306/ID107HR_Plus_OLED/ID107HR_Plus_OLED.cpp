#include "ID107HR_Plus_OLED.h"

static uint8_t buffer[SH1106_LCDHEIGHT * SH1106_LCDWIDTH / 8];


void ID107HR_Plus_OLED::drawPixel(int16_t x, int16_t y, uint16_t color) {
	if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
		return;
	switch (color) 
	{
		case WHITE:   buffer[x+ (y/8)*SH1106_LCDWIDTH] |=  (1 << (y&7)); break;
		case BLACK:   buffer[x+ (y/8)*SH1106_LCDWIDTH] &= ~(1 << (y&7)); break; 
		case INVERSE: buffer[x+ (y/8)*SH1106_LCDWIDTH] ^=  (1 << (y&7)); break; 
	}
}


ID107HR_Plus_OLED::ID107HR_Plus_OLED(int8_t DC, int8_t RST, int8_t CS, int8_t PWR) :_spi(SPI), Adafruit_GFX(SH1106_LCDWIDTH, SH1106_LCDHEIGHT) {
	dc = DC;
	rst = RST;
	cs = CS;
	pwr = PWR;
}

ID107HR_Plus_OLED::ID107HR_Plus_OLED(SPIClass &spi,int8_t DC, int8_t RST, int8_t CS, int8_t PWR) :_spi(spi), Adafruit_GFX(SH1106_LCDWIDTH, SH1106_LCDHEIGHT) {
	dc = DC;
	rst = RST;
	cs = CS;
	pwr = PWR;
}

void ID107HR_Plus_OLED::begin(void) {
	// power on
	if (pwr != -1) {
		pinMode(pwr, OUTPUT);
		digitalWrite(pwr, HIGH);
	}
	pinMode(dc, OUTPUT);
	pinMode(cs, OUTPUT);
	_spi.begin();
	// reset OLED
	pinMode(rst, OUTPUT);
	digitalWrite(rst, HIGH);
	delay(10);
	digitalWrite(rst, LOW);
	delay(10);
	digitalWrite(rst, HIGH);

	SH1106_command(SH1106_DISPLAYOFF);
	SH1106_command(SH1106_SETDISPLAYCLOCKDIV);
	SH1106_command(0x50); //100Hz
	//SH1106_command(SH1106_SETMULTIPLEX);  ==> not sure about this one, ID107HR Plus driver enables this with 0x1F
	//SH1106_command(0x1F); 
	SH1106_command(SH1106_SETDISPLAYOFFSET); SH1106_command(0x00); 
	SH1106_command(SH1106_SETSTARTLINE | 0x00);
	SH1106_command(0xad); SH1106_command(0x8b); // DC-DC Control Mode, 8b = DC-DC converter on when display on
	SH1106_command(0x32); // POR value related
	SH1106_command(SH1106_SEGREMAP | 0x01);
	SH1106_command(SH1106_COMSCANDEC);
	SH1106_command(SH1106_SETCOMPINS); SH1106_command(0x12); 
	SH1106_command(SH1106_SETCONTRAST); SH1106_command(0xcf); 
	SH1106_command(SH1106_SETPRECHARGE); SH1106_command(0x28); 
	SH1106_command(SH1106_SETVCOMDETECT); SH1106_command(0x35); 
	SH1106_command(SH1106_DISPLAYALLON_RESUME);
	SH1106_command(SH1106_NORMALDISPLAY);
	SH1106_command(SH1106_DISPLAYON);
	clearDisplay();
}


void ID107HR_Plus_OLED::invertDisplay(const bool onOff) {
	SH1106_command(onOff ? SH1106_INVERTDISPLAY : SH1106_NORMALDISPLAY);
}

void ID107HR_Plus_OLED::SH1106_command(const uint8_t data) { 
	_spi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
	digitalWrite(cs, HIGH);
	digitalWrite(dc, LOW);
	digitalWrite(cs, LOW);
	fastSPIwrite(data);
	digitalWrite(cs, HIGH);
	_spi.endTransaction();
}

void ID107HR_Plus_OLED::display(void) {
	
	SH1106_command(SH1106_SETLOWCOLUMN | 0x0);
	SH1106_command(SH1106_SETHIGHCOLUMN | 0x0);
	SH1106_command(SH1106_SETSTARTLINE | 0x0);

	const uint8_t height= SH1106_LCDHEIGHT / 8;
	const uint8_t width= SH1106_LCDWIDTH / 8;
	const uint8_t m_row = 0;
	const uint8_t m_col = SH1106_LCDWIDTH / 2;

	uint16_t p = 0;
			
	for (uint8_t i = 0; i < height; i++) {
		SH1106_command(SH1106_SETPAGEADDR + i + m_row);//set page address
		SH1106_command(m_col & 0xf);//set lower column address
		SH1106_command(0x10 | (m_col >> 4));//set higher column address
		
		for(uint8_t j = 0; j < width; j++){
			for (uint8_t k = 0; k < width; k++, p++) {
					fastSPIwrite(buffer[p]);
			}
		}
	}

}

void ID107HR_Plus_OLED::clearDisplay(void) {
	(void)memset(buffer, 0, (SH1106_LCDWIDTH * SH1106_LCDHEIGHT / 8));
}


inline void ID107HR_Plus_OLED::fastSPIwrite(const uint8_t data) {
	(void)_spi.transfer(data);
}

void ID107HR_Plus_OLED::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
	drawFastHLineInternal(x, y, w, color);
}

void ID107HR_Plus_OLED::drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) {
  if(y < 0 || y >= HEIGHT) { return; }

  // make sure we don't try to draw below 0
  if(x < 0) { 
    w += x;
    x = 0;
  }

  // make sure we don't go off the edge of the display
  if( (x + w) > WIDTH) { 
    w = (WIDTH - x);
  }

  // if our width is now negative, punt
  if(w <= 0) { return; }

  // set up the pointer for movement through the buffer
  register uint8_t *pBuf = buffer;
  // adjust the buffer pointer for the current row
  pBuf += ((y/8) * SH1106_LCDWIDTH);
  // and offset x columns in
  pBuf += x;

  register uint8_t mask = 1 << (y&7);

  switch (color) 
  {
  case WHITE:         while(w--) { *pBuf++ |= mask; }; break;
    case BLACK: mask = ~mask;   while(w--) { *pBuf++ &= mask; }; break;
  case INVERSE:         while(w--) { *pBuf++ ^= mask; }; break;
  }
}

void ID107HR_Plus_OLED::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
	drawFastVLineInternal(x, y, h, color);
}


void ID107HR_Plus_OLED::drawFastVLineInternal(int16_t x, int16_t __y, int16_t __h, uint16_t color) {

  // do nothing if we're off the left or right side of the screen
  if(x < 0 || x >= WIDTH) { return; }

  // make sure we don't try to draw below 0
  if(__y < 0) { 
    // __y is negative, this will subtract enough from __h to account for __y being 0
    __h += __y;
    __y = 0;

  } 

  // make sure we don't go past the height of the display
  if( (__y + __h) > HEIGHT) { 
    __h = (HEIGHT - __y);
  }

  // if our height is now negative, punt 
  if(__h <= 0) { 
    return;
  }

  // this display doesn't need ints for coordinates, use local byte registers for faster juggling
  register uint8_t y = __y;
  register uint8_t h = __h;


  // set up the pointer for fast movement through the buffer
  register uint8_t *pBuf = buffer;
  // adjust the buffer pointer for the current row
  pBuf += ((y/8) * SH1106_LCDWIDTH);
  // and offset x columns in
  pBuf += x;

  // do the first partial byte, if necessary - this requires some masking
  register uint8_t mod = (y&7);
  if(mod) {
    // mask off the high n bits we want to set 
    mod = 8-mod;

    // note - lookup table results in a nearly 10% performance improvement in fill* functions
    // register uint8_t mask = ~(0xFF >> (mod));
    static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
    register uint8_t mask = premask[mod];

    // adjust the mask if we're not going to reach the end of this byte
    if( h < mod) { 
      mask &= (0XFF >> (mod-h));
    }

  switch (color) 
    {
    case WHITE:   *pBuf |=  mask;  break;
    case BLACK:   *pBuf &= ~mask;  break;
    case INVERSE: *pBuf ^=  mask;  break;
    }
  
    // fast exit if we're done here!
    if(h<mod) { return; }

    h -= mod;

    pBuf += SH1106_LCDWIDTH;
  }


  // write solid bytes while we can - effectively doing 8 rows at a time
  if(h >= 8) { 
    if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
      do  {
      *pBuf=~(*pBuf);

        // adjust the buffer forward 8 rows worth of data
        pBuf += SH1106_LCDWIDTH;

        // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
        h -= 8;
      } while(h >= 8);
      }
    else {
      // store a local value to work with 
      register uint8_t val = (color == WHITE) ? 255 : 0;

      do  {
        // write our value in
      *pBuf = val;

        // adjust the buffer forward 8 rows worth of data
        pBuf += SH1106_LCDWIDTH;

        // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
        h -= 8;
      } while(h >= 8);
      }
    }

  // now do the final partial byte, if necessary
  if(h) {
    mod = h & 7;
    // this time we want to mask the low bits of the byte, vs the high bits we did above
    // register uint8_t mask = (1 << mod) - 1;
    // note - lookup table results in a nearly 10% performance improvement in fill* functions
    static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
    register uint8_t mask = postmask[mod];
    switch (color) 
    {
      case WHITE:   *pBuf |=  mask;  break;
      case BLACK:   *pBuf &= ~mask;  break;
      case INVERSE: *pBuf ^=  mask;  break;
    }
  }
}

void ID107HR_Plus_OLED::powerOn()
{
	digitalWrite(pwr,HIGH);
}

void ID107HR_Plus_OLED::powerOff()
{
	digitalWrite(pwr,LOW);
}