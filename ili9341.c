#include "ili9341.h"

volatile uint16_t LCD_W=ILI9341_TFTWIDTH;
volatile uint16_t LCD_H=ILI9341_TFTHEIGHT;

#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
uint16_t _width = 320;
uint16_t _height = 240;


void ili9341_hard_init(void)//init hardware
{
rstddr=0xFF;//output for reset
rstport |=(1<<rst);//pull high for normal operation
controlddr|=(1<<dc);//D/C as output
}


void ili9341_spi_init(void)//set spi speed and settings 
{
DDRB |=(1<<1)|(1<<2)|(1<<3)|(1<<5);//CS,SS,MOSI,SCK as output(although SS will be unused throughout the program)
SPCR=(1<<SPE)|(1<<MSTR);//mode 0,fosc/4
SPSR |=(1<<SPI2X);//doubling spi speed.i.e final spi speed-fosc/2
PORTB |=(1<<1);//cs off during startup

}


void ili9341_spi_send(unsigned char spi_data)//send spi data to display
{
SPDR=spi_data;//move data into spdr
while(!(SPSR & (1<<SPIF)));//wait till the transmission is finished
}


void ili9341_writecommand8(uint8_t com)//command write
{
controlport &=~((1<<dc)|(1<<cs));//dc and cs both low to send command
_delay_us(5);//little delay
ili9341_spi_send(com);
controlport |=(1<<cs);//pull high cs
}


void ili9341_writedata8(uint8_t data)//data write
{
controlport |=(1<<dc);//st dc high for data
_delay_us(1);//delay
controlport &=~(1<<cs);//set cs low for operation
ili9341_spi_send(data);
controlport |=(1<<cs);
}


void ili9341_setaddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)//set coordinate for print or other function
{
ili9341_writecommand8(0x2A);
ili9341_writedata8(x1>>8);
ili9341_writedata8(x1);
ili9341_writedata8(x2>>8);
ili9341_writedata8(x2);

ili9341_writecommand8(0x2B);
ili9341_writedata8(y1>>8);
ili9341_writedata8(y1);
ili9341_writedata8(y2);
ili9341_writedata8(y2);

ili9341_writecommand8(0x2C);//meory write
}


void ili9341_hard_reset(void)//hard reset display
{
rstport |=(1<<rst);//pull high if low previously
_delay_ms(200);
rstport &=~(1<<rst);//low for reset
_delay_ms(200);
rstport |=(1<<rst);//again pull high for normal operation
_delay_ms(200);
}


void ili9341_init(void)//set up display using predefined command sequence
{
ili9341_hard_init();
ili9341_spi_init();
ili9341_hard_reset();
ili9341_writecommand8(0x01);//soft reset
_delay_ms(1000);
//power control A
ili9341_writecommand8(0xCB);
ili9341_writedata8(0x39);
ili9341_writedata8(0x2C);
ili9341_writedata8(0x00);
ili9341_writedata8(0x34);
ili9341_writedata8(0x02);

//power control B
ili9341_writecommand8(0xCF);
ili9341_writedata8(0x00);
ili9341_writedata8(0xC1);
ili9341_writedata8(0x30);

//driver timing control A
ili9341_writecommand8(0xE8);
ili9341_writedata8(0x85);
ili9341_writedata8(0x00);
ili9341_writedata8(0x78);

//driver timing control B
ili9341_writecommand8(0xEA);
ili9341_writedata8(0x00);
ili9341_writedata8(0x00);

//power on sequence control
ili9341_writecommand8(0xED);
ili9341_writedata8(0x64);
ili9341_writedata8(0x03);
ili9341_writedata8(0x12);
ili9341_writedata8(0x81);

//pump ratio control
ili9341_writecommand8(0xF7);
ili9341_writedata8(0x20);

//power control,VRH[5:0]
ili9341_writecommand8(0xC0);
ili9341_writedata8(0x23);

//Power control,SAP[2:0];BT[3:0]
ili9341_writecommand8(0xC1);
ili9341_writedata8(0x10);

//vcm control
ili9341_writecommand8(0xC5);
ili9341_writedata8(0x3E);
ili9341_writedata8(0x28);

//vcm control 2
ili9341_writecommand8(0xC7);
ili9341_writedata8(0x86);

//memory access control
ili9341_writecommand8(0x36);
ili9341_writedata8(0x48);

//pixel format
ili9341_writecommand8(0x3A);
ili9341_writedata8(0x55);

//frameration control,normal mode full colours
ili9341_writecommand8(0xB1);
ili9341_writedata8(0x00);
ili9341_writedata8(0x18);

//display function control
ili9341_writecommand8(0xB6);
ili9341_writedata8(0x08);
ili9341_writedata8(0x82);
ili9341_writedata8(0x27);

//3gamma function disable
ili9341_writecommand8(0xF2);
ili9341_writedata8(0x00);

//gamma curve selected
ili9341_writecommand8(0x26);
ili9341_writedata8(0x01);

//set positive gamma correction
ili9341_writecommand8(0xE0);
ili9341_writedata8(0x0F);
ili9341_writedata8(0x31);
ili9341_writedata8(0x2B);
ili9341_writedata8(0x0C);
ili9341_writedata8(0x0E);
ili9341_writedata8(0x08);
ili9341_writedata8(0x4E);
ili9341_writedata8(0xF1);
ili9341_writedata8(0x37);
ili9341_writedata8(0x07);
ili9341_writedata8(0x10);
ili9341_writedata8(0x03);
ili9341_writedata8(0x0E);
ili9341_writedata8(0x09);
ili9341_writedata8(0x00);

//set negative gamma correction
ili9341_writecommand8(0xE1);
ili9341_writedata8(0x00);
ili9341_writedata8(0x0E);
ili9341_writedata8(0x14);
ili9341_writedata8(0x03);
ili9341_writedata8(0x11);
ili9341_writedata8(0x07);
ili9341_writedata8(0x31);
ili9341_writedata8(0xC1);
ili9341_writedata8(0x48);
ili9341_writedata8(0x08);
ili9341_writedata8(0x0F);
ili9341_writedata8(0x0C);
ili9341_writedata8(0x31);
ili9341_writedata8(0x36);
ili9341_writedata8(0x0F);

//exit sleep
ili9341_writecommand8(0x11);
_delay_ms(120);
//display on
ili9341_writecommand8(0x29);

}

//set colour for drawing
void ili9341_pushcolour(uint16_t colour)
{
ili9341_writedata8(colour>>8);
ili9341_writedata8(colour);
}


//clear lcd and fill with colour
void ili9341_clear(uint16_t colour)
{
uint16_t i,j;
ili9341_setaddress(0,0,LCD_W-1,LCD_H-1);

for(i=0;i<LCD_W;i++)
{
for(j=0;j<LCD_H;j++)
{
ili9341_pushcolour(colour);
}
}

}



//draw pixel
void ili9341_drawpixel(uint16_t x3,uint16_t y3,uint16_t colour1) //pixels will always be counted from right side.x is representing LCD width which will always be less tha 240.Y is representing LCD height which will always be less than 320
{
if((x3 < 0) ||(x3 >=LCD_W) || (y3 < 0) || (y3 >=LCD_H)) return;

ili9341_setaddress(x3,y3,x3+1,y3+1);

ili9341_pushcolour(colour1);
}


//draw vertical line
void ili9341_drawvline(uint16_t x,uint16_t y,uint16_t h,uint16_t colour)//basically we will see this line horizental if we see the display 320*240
{
if((x >=LCD_W) || (y >=LCD_H)) return;
if((y+h-1)>=LCD_H)
h=LCD_H-y;
ili9341_setaddress(x,y,x,y+h-1);
while(h--)
{
ili9341_pushcolour(colour);
}
}


//draw horizental line

void ili9341_drawhline(uint16_t x,uint16_t y,uint16_t w,uint16_t colour)
{
if((x >=LCD_W) || (y >=LCD_H)) return;
if((x+w-1)>=LCD_W)
w=LCD_W-x;
ili9341_setaddress(x,y,x+w-1,y);
while(w--)
{
ili9341_pushcolour(colour);
}
}

//draw colour filled rectangle
void ili9341_fillrect(uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t colour)
{
if((x >=LCD_W) || (y >=LCD_H)) return;
if((x+w-1)>=LCD_W)
w=LCD_W-x;
if((y+h-1)>=LCD_H)
h=LCD_H-y;

ili9341_setaddress(x, y, x+w-1, y+h-1);

for(y=h; y>0; y--) 
{
for(x=w; x>0; x--)
{
ili9341_pushcolour(colour);
}
}
}
//rotate screen at desired orientation
void ili9341_setRotation(uint8_t m) 
{
uint8_t rotation;
ili9341_writecommand8(0x36);
rotation=m%4;
switch (rotation) 
{
case 0:
ili9341_writedata8(0x40|0x08);
LCD_W = 240;
LCD_H = 320;
break;
case 1:
ili9341_writedata8(0x20|0x08);
LCD_W  = 320;
LCD_H = 240;
break;
case 2:
ili9341_writedata8(0x80|0x08);
LCD_W  = 240;
LCD_H = 320;
break;
case 3:
ili9341_writedata8(0x40|0x80|0x20|0x08);
LCD_W  = 320;
LCD_H = 240;
break;
}
}

// Draw a circle outline
void ili9341_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) 
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  ili9341_drawpixel(x0  , y0+r, color);
  ili9341_drawpixel(x0  , y0-r, color);
  ili9341_drawpixel(x0+r, y0  , color);
  ili9341_drawpixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    ili9341_drawpixel(x0 + x, y0 + y, color);
    ili9341_drawpixel(x0 - x, y0 + y, color);
    ili9341_drawpixel(x0 + x, y0 - y, color);
    ili9341_drawpixel(x0 - x, y0 - y, color);
    ili9341_drawpixel(x0 + y, y0 + x, color);
    ili9341_drawpixel(x0 - y, y0 + x, color);
    ili9341_drawpixel(x0 + y, y0 - x, color);
    ili9341_drawpixel(x0 - y, y0 - x, color);
  }
}

//DRaw Circle Helper
void ili9341_drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color) 
{
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      ili9341_drawpixel(x0 + x, y0 + y, color);
      ili9341_drawpixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      ili9341_drawpixel(x0 + x, y0 - y, color);
      ili9341_drawpixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      ili9341_drawpixel(x0 - y, y0 + x, color);
      ili9341_drawpixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      ili9341_drawpixel(x0 - y, y0 - x, color);
      ili9341_drawpixel(x0 - x, y0 - y, color);
    }
  }
}

void ili9341_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) 
{
  ili9341_drawvline(x0, y0-r, 2*r+1, color);
  ili9341_fillCircleHelper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void ili9341_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color) 
{

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      ili9341_drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      ili9341_drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      ili9341_drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      ili9341_drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}


void ili9341_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) 
{
  // Update in subclasses if desired!
  ili9341_drawLine(x, y, x, y+h-1, color);
}

void ili9341_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) 
{
  // Update in subclasses if desired!
  ili9341_drawLine(x, y, x+w-1, y, color);
}


// Bresenham's algorithm - thx wikpedia
void ili9341_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) 
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      ili9341_drawpixel(y0, x0, color);
    } else {
      ili9341_drawpixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// Draw a rectangle
void ili9341_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
{
  ili9341_drawFastHLine(x, y, w, color);
  ili9341_drawFastHLine(x, y+h-1, w, color);
  ili9341_drawFastVLine(x, y, h, color);
  ili9341_drawFastVLine(x+w-1, y, h, color);
}


void ili9341_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
{
  // Update in subclasses if desired!
  for (int16_t i=x; i<x+w; i++) {
    ili9341_drawFastVLine(i, y, h, color);
  }
}
// Draw a rounded rectangle
void ili9341_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) 
{
  // smarter version
  ili9341_drawFastHLine(x+r  , y    , w-2*r, color); // Top
  ili9341_drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  ili9341_drawFastVLine(x    , y+r  , h-2*r, color); // Left
  ili9341_drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  ili9341_drawCircleHelper(x+r    , y+r    , r, 1, color);
  ili9341_drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  ili9341_drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  ili9341_drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

// Fill a rounded rectangle
void ili9341_fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) 
{
  // smarter version
  ili9341_fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  ili9341_fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  ili9341_fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

// Draw a 1-bit image (bitmap) at the specified (x,y) position from the
// provided bitmap buffer (must be PROGMEM memory) using the specified
// foreground color (unset bits are transparent).
void ili9341_drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) 
{

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte=0;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++) {
      if(i & 7) byte <<= 1;
      else      byte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if(byte & 0x80) ili9341_drawpixel(x+i, y+j, color);
    }
  }
}


//Draw XBitMap Files (*.xbm), exported from GIMP,
//Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
//C Array can be directly used with this function
void ili9341_drawXBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) 
{

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte=0;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(i & 7) byte >>= 1;
      else      byte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if(byte & 0x01) ili9341_drawpixel(x+i, y+j, color);
    }
  }
}
