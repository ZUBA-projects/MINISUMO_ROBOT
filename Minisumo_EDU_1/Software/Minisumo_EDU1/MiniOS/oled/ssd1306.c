/*
 * ssd1306.h
 *
 *  Created on: 05.04.2020
 *      Author: zuba1
 */

#include "../../hal.h"
#include "ssd1306.h"


static uint8_t _vccstate;
static int16_t _width, _height, WIDTH, HEIGHT, cursor_x, cursor_y;
static uint8_t textsize, rotation;
static uint16_t textcolor, textbgcolor;
uint8_t _oledsdapin = DEF_GPIO;
uint8_t _oledsclpin = DEF_GPIO;

uint8_t wrap,   // If set, 'wrap' text at right edge of display
     _cp437; // If set, use correct CP437 charset (default is off)

// the memory buffer for the LCD
static uint8_t buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8];

// Return the size of the display (per current rotation)
/*
int16_t ssd1306_width(void)
{
    return _width;
}
*/
/*
int16_t ssd1306_height(void)
{
    return _height;
}*/

void set_rotation(uint8_t x)
{
    rotation = (x & 3);
    switch (rotation) {
    case 0:
    case 2:
        _width  = WIDTH;
        _height = HEIGHT;
        break;
    case 1:
    case 3:
        _width  = HEIGHT;
        _height = WIDTH;
        break;
    }
}




void ssd1306_init_i2c(uint8_t _sdapin,uint8_t _sclpin)
{
    //_oledi2caddr = SSD1306_I2C_ADDRESS;
     _oledsdapin = _sdapin;
	 _oledsclpin = _sclpin;
}
/*
void ssd1306_command(uint8_t c)
{
        i2c_Start(_oledsclpin,_oledsdapin,(_oledi2caddr | I2C_WRITE_HAL));
        i2c_Write(OLED_REG_COMMAND);
        i2c_Write(c);
        i2c_Stop();
}*/

void ssd1306_begin(uint8_t vccstate, uint8_t i2caddr, uint8_t reset)
{
	i2c_set_reg_instance(_oledsclpin,_oledsdapin,SSD1306_I2C_ADDRESS);
    _vccstate = vccstate;
    //_oledi2caddr = i2caddr;
    //UNUSED_VARIABLE(_oledi2caddr);

    _width    = WIDTH;
    _height   = HEIGHT;
    cursor_y  = cursor_x    = 0;
    textsize  = 1;
    textcolor = textbgcolor = 0xFFFF;
    wrap      = 1;
    _cp437    = 0;

    _width = WIDTH = SSD1306_LCDWIDTH;
    _height = HEIGHT = SSD1306_LCDHEIGHT;
    rotation  = 0;

#if defined SSD1306_128_32
    // Init sequence for 128x32 OLED module
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_DISPLAYOFF);                    // 0xAE
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    i2c_writeReg(OLED_REG_COMMAND,0x80);                                  // the suggested ratio 0x80

    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETMULTIPLEX);                  // 0xA8
    i2c_writeReg(OLED_REG_COMMAND,0x1F);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETDISPLAYOFFSET);              // 0xD3
    i2c_writeReg(OLED_REG_COMMAND,0x0);                                   // no offset
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETSTARTLINE | 0x0);            // line #0
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC) {
        i2c_writeReg(OLED_REG_COMMAND,0x10);
    }
    else {
        i2c_writeReg(OLED_REG_COMMAND,0x14);
    }
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_MEMORYMODE);                    // 0x20
    i2c_writeReg(OLED_REG_COMMAND,0x00);                                  // 0x0 act like ks0108
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SEGREMAP | 0x1);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_COMSCANDEC);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETCOMPINS);                    // 0xDA
    i2c_writeReg(OLED_REG_COMMAND,0x02);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETCONTRAST);                   // 0x81
    i2c_writeReg(OLED_REG_COMMAND,0x8F);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC) {
        i2c_writeReg(OLED_REG_COMMAND,0x22);
    }
    else {
        i2c_writeReg(OLED_REG_COMMAND,0xF1);
    }
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETVCOMDETECT);                 // 0xDB
    i2c_writeReg(OLED_REG_COMMAND,0x40);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_NORMALDISPLAY);                 // 0xA6
#endif

#if defined SSD1306_128_64
    // Init sequence for 128x64 OLED module
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_DISPLAYOFF);                    // 0xAE
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    i2c_writeReg(OLED_REG_COMMAND,0x80);                                  // the suggested ratio 0x80
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETMULTIPLEX);                  // 0xA8
    i2c_writeReg(OLED_REG_COMMAND,0x3F);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETDISPLAYOFFSET);              // 0xD3
    i2c_writeReg(OLED_REG_COMMAND,0x0);                                   // no offset
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETSTARTLINE | 0x0);            // line #0
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC) {
        i2c_writeReg(OLED_REG_COMMAND,0x10);
    }
    else {
        i2c_writeReg(OLED_REG_COMMAND,0x14);
    }
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_MEMORYMODE);                    // 0x20
    i2c_writeReg(OLED_REG_COMMAND,0x00);                                  // 0x0 act like ks0108
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SEGREMAP | 0x1);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_COMSCANDEC);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETCOMPINS);                    // 0xDA
    i2c_writeReg(OLED_REG_COMMAND,0x12);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC) {
        i2c_writeReg(OLED_REG_COMMAND,0x9F);
    }
    else {
        i2c_writeReg(OLED_REG_COMMAND,0xCF);
    }
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC) {
        i2c_writeReg(OLED_REG_COMMAND,0x22);
    }
    else {
        i2c_writeReg(OLED_REG_COMMAND,0xF1);
    }
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETVCOMDETECT);                 // 0xDB
    i2c_writeReg(OLED_REG_COMMAND,0x40);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_NORMALDISPLAY);                 // 0xA6
#endif

#if defined SSD1306_96_16
    // Init sequence for 96x16 OLED module
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_DISPLAYOFF);                    // 0xAE
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    i2c_writeReg(OLED_REG_COMMAND,0x80);                                  // the suggested ratio 0x80
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETMULTIPLEX);                  // 0xA8
    i2c_writeReg(OLED_REG_COMMAND,0x0F);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETDISPLAYOFFSET);              // 0xD3
    i2c_writeReg(OLED_REG_COMMAND,0x00);                                   // no offset
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETSTARTLINE | 0x0);            // line #0
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC) {
        i2c_writeReg(OLED_REG_COMMAND,0x10);
    }
    else {
        i2c_writeReg(OLED_REG_COMMAND,0x14);
    }
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_MEMORYMODE);                    // 0x20
    i2c_writeReg(OLED_REG_COMMAND,0x00);                                  // 0x0 act like ks0108
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SEGREMAP | 0x1);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_COMSCANDEC);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETCOMPINS);                    // 0xDA
    i2c_writeReg(OLED_REG_COMMAND,0x2);	//ada x12
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC) {
        i2c_writeReg(OLED_REG_COMMAND,0x10);
    }
    else {
        i2c_writeReg(OLED_REG_COMMAND,0xAF);
    }
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC) {
        i2c_writeReg(OLED_REG_COMMAND,0x22);
    }
    else {
        i2c_writeReg(OLED_REG_COMMAND,0xF1);
    }
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETVCOMDETECT);                 // 0xDB
    i2c_writeReg(OLED_REG_COMMAND,0x40);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_NORMALDISPLAY);                 // 0xA6
#endif

    i2c_writeReg(OLED_REG_COMMAND,SSD1306_DISPLAYON);//--turn on oled panel
}



// the most basic function, set a single pixel
void ssd1306_draw_pixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
        return;

    // check rotation, move pixel around if necessary
    switch (rotation) {
    case 1:
        swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        break;
    case 3:
        swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    // x is which column
    switch (color) {
    case WHITE:
        buffer[x + (y / 8)*SSD1306_LCDWIDTH] |=  (1 << (y & 7));
        break;
    case BLACK:
        buffer[x + (y / 8)*SSD1306_LCDWIDTH] &= ~(1 << (y & 7));
        break;
    case INVERSE:
        buffer[x + (y / 8)*SSD1306_LCDWIDTH] ^=  (1 << (y & 7));
        break;
    }
}


void ssd1306_invert_display(uint8_t i)
{
	i2c_set_reg_instance(_oledsclpin,_oledsdapin,SSD1306_I2C_ADDRESS);
    if (i) {
        i2c_writeReg(OLED_REG_COMMAND,SSD1306_INVERTDISPLAY);
    }
    else {
        i2c_writeReg(OLED_REG_COMMAND,SSD1306_NORMALDISPLAY);
    }
}

/*
// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_start_scroll_right(uint8_t start, uint8_t stop)
{
	i2c_set_reg_instance(_oledsclpin,_oledsdapin,SSD1306_I2C_ADDRESS);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_RIGHT_HORIZONTAL_SCROLL);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,start);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,stop);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,0XFF);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_ACTIVATE_SCROLL);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_start_scroll_left(uint8_t start, uint8_t stop)
{
	i2c_set_reg_instance(_oledsclpin,_oledsdapin,SSD1306_I2C_ADDRESS);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_LEFT_HORIZONTAL_SCROLL);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,start);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,stop);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,0XFF);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_start_scroll_diag_right(uint8_t start, uint8_t stop)
{
	i2c_set_reg_instance(_oledsclpin,_oledsdapin,SSD1306_I2C_ADDRESS);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SET_VERTICAL_SCROLL_AREA);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_LCDHEIGHT);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,start);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,stop);
    i2c_writeReg(OLED_REG_COMMAND,0X01);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_start_scroll_diag_left(uint8_t start, uint8_t stop)
{
	i2c_set_reg_instance(_oledsclpin,_oledsdapin,SSD1306_I2C_ADDRESS);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SET_VERTICAL_SCROLL_AREA);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_LCDHEIGHT);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,start);
    i2c_writeReg(OLED_REG_COMMAND,0X00);
    i2c_writeReg(OLED_REG_COMMAND,stop);
    i2c_writeReg(OLED_REG_COMMAND,0X01);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_ACTIVATE_SCROLL);
}

void ssd1306_stop_scroll(void)
{
	i2c_set_reg_instance(_oledsclpin,_oledsdapin,SSD1306_I2C_ADDRESS);
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_DEACTIVATE_SCROLL);
}
*/
/*
// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void ssd1306_dim(uint8_t dim)
{
	i2c_set_reg_instance(_oledsclpin,_oledsdapin,SSD1306_I2C_ADDRESS);
    uint8_t contrast;

    if (dim) {
        contrast = 0; // Dimmed display
    }
    else {
        if (_vccstate == SSD1306_EXTERNALVCC) {
            contrast = 0x9F;
        }
        else {
            contrast = 0xCF;
        }
    }
    // the range of contrast to too small to be really useful
    // it is useful to dim the display
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_SETCONTRAST);
    i2c_writeReg(OLED_REG_COMMAND,contrast);
}
*/
/*
void ssd1306_data(uint8_t c)
{

		i2c_Start(_oledsclpin,_oledsdapin,(_oledi2caddr | I2C_WRITE_HAL));
        i2c_Write(OLED_REG_DATA);
        i2c_Write(c);
        i2c_Stop();
}*/

void ssd1306_display(void)
{
	i2c_set_reg_instance(_oledsclpin,_oledsdapin,SSD1306_I2C_ADDRESS);

    i2c_writeReg(OLED_REG_COMMAND,SSD1306_COLUMNADDR);
    i2c_writeReg(OLED_REG_COMMAND,0);   // Column start address (0 = reset)
    i2c_writeReg(OLED_REG_COMMAND,SSD1306_LCDWIDTH - 1); // Column end address (127 = reset)

    i2c_writeReg(OLED_REG_COMMAND,SSD1306_PAGEADDR);
    i2c_writeReg(OLED_REG_COMMAND,0); // Page start address (0 = reset)
#if SSD1306_LCDHEIGHT == 64
    i2c_writeReg(OLED_REG_COMMAND,7); // Page end address
#endif
#if SSD1306_LCDHEIGHT == 32
    i2c_writeReg(OLED_REG_COMMAND,3); // Page end address
#endif
#if SSD1306_LCDHEIGHT == 16
    i2c_writeReg(OLED_REG_COMMAND,1); // Page end address
#endif

/*

    	i2c_Start(_oledsclpin,_oledsdapin,(SSD1306_I2C_ADDRESS | I2C_WRITE_HAL));
        i2c_Write(0x40);//to do: continuous

        i2c_Write(SSD1306_I2C_ADDRESS);

        uint16_t len=(SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT) / 8;
        for(uint16_t i=0;i<len;i++){
        	 i2c_Write(buffer[i]);//to do: continuous
        }

        i2c_Stop();
*/

     i2c_writeMulti(OLED_REG_DATA, buffer,((SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT) / 8),1);



}

/****************************************************---GFX---***********************************************/

// clear everything
void ssd1306_clear_display(void)
{
    memset(buffer, 0, (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8));
}



void ssd1306_draw_fast_hline(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    uint8_t __swap = 0;
    switch (rotation) {
    case 0:
        // 0 degree rotation, do nothing
        break;
    case 1:
        // 90 degree rotation, swap x & y for rotation, then invert x
        __swap = 1;
        swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
        // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        x -= (w - 1);
        break;
    case 3:
        // 270 degree rotation, swap x & y for rotation, then invert y  and adjust y for w (not to become h)
        __swap = 1;
        swap(x, y);
        y = HEIGHT - y - 1;
        y -= (w - 1);
        break;
    }

    if (__swap) {
        ssd1306_draw_fast_vline_internal(x, y, w, color);
    }
    else {
        ssd1306_draw_fast_hline_internal(x, y, w, color);
    }
}

void ssd1306_draw_fast_hline_internal(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    // Do bounds/limit checks
    if (y < 0 || y >= HEIGHT) {
        return;
    }

    // make sure we don't try to draw below 0
    if (x < 0) {
        w += x;
        x = 0;
    }

    // make sure we don't go off the edge of the display
    if ( (x + w) > WIDTH) {
        w = (WIDTH - x);
    }

    // if our width is now negative, punt
    if (w <= 0) {
        return;
    }

    // set up the pointer for  movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    register uint8_t mask = 1 << (y & 7);

    switch (color) {
    case WHITE:
        while (w--) {
            *pBuf++ |= mask;
        };
        break;
    case BLACK:
        mask = ~mask;
        while (w--) {
            *pBuf++ &= mask;
        };
        break;
    case INVERSE:
        while (w--) {
            *pBuf++ ^= mask;
        };
        break;
    }
}

void ssd1306_draw_fast_vline(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    uint8_t __swap = 0;
    switch (rotation) {
    case 0:
        break;
    case 1:
        // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
        __swap = 1;
        swap(x, y);
        x = WIDTH - x - 1;
        x -= (h - 1);
        break;
    case 2:
        // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        y -= (h - 1);
        break;
    case 3:
        // 270 degree rotation, swap x & y for rotation, then invert y
        __swap = 1;
        swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    if (__swap) {
        ssd1306_draw_fast_hline_internal(x, y, h, color);
    }
    else {
        ssd1306_draw_fast_vline_internal(x, y, h, color);
    }
}


void ssd1306_draw_fast_vline_internal(int16_t x, int16_t __y, int16_t __h, uint16_t color)
{

    // do nothing if we're off the left or right side of the screen
    if (x < 0 || x >= WIDTH) {
        return;
    }

    // make sure we don't try to draw below 0
    if (__y < 0) {
        // __y is negative, this will subtract enough from __h to account for __y being 0
        __h += __y;
        __y = 0;

    }

    // make sure we don't go past the height of the display
    if ( (__y + __h) > HEIGHT) {
        __h = (HEIGHT - __y);
    }

    // if our height is now negative, punt
    if (__h <= 0) {
        return;
    }

    // this display doesn't need ints for coordinates, use local byte registers for faster juggling
    register uint8_t y = __y;
    register uint8_t h = __h;


    // set up the pointer for fast movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    // do the first partial byte, if necessary - this requires some masking
    register uint8_t mod = (y & 7);
    if (mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;

        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        register uint8_t mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if ( h < mod) {
            mask &= (0XFF >> (mod - h));
        }

        switch (color) {
        case WHITE:
            *pBuf |=  mask;
            break;
        case BLACK:
            *pBuf &= ~mask;
            break;
        case INVERSE:
            *pBuf ^=  mask;
            break;
        }

        // fast exit if we're done here!
        if (h < mod) {
            return;
        }

        h -= mod;

        pBuf += SSD1306_LCDWIDTH;
    }


    // write solid bytes while we can - effectively doing 8 rows at a time
    if (h >= 8) {
        if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
            do  {
                *pBuf = ~(*pBuf);

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            }
            while (h >= 8);
        }
        else {
            // store a local value to work with
            register uint8_t val = (color == WHITE) ? 255 : 0;

            do  {
                // write our value in
                *pBuf = val;

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            }
            while (h >= 8);
        }
    }

    // now do the final partial byte, if necessary
    if (h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
        register uint8_t mask = postmask[mod];
        switch (color) {
        case WHITE:
            *pBuf |=  mask;
            break;
        case BLACK:
            *pBuf &= ~mask;
            break;
        case INVERSE:
            *pBuf ^=  mask;
            break;
        }
    }
}


// Draw a circle outline
void ssd1306_draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ssd1306_draw_pixel(x0  , y0 + r, color);
    ssd1306_draw_pixel(x0  , y0 - r, color);
    ssd1306_draw_pixel(x0 + r, y0  , color);
    ssd1306_draw_pixel(x0 - r, y0  , color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ssd1306_draw_pixel(x0 + x, y0 + y, color);
        ssd1306_draw_pixel(x0 - x, y0 + y, color);
        ssd1306_draw_pixel(x0 + x, y0 - y, color);
        ssd1306_draw_pixel(x0 - x, y0 - y, color);
        ssd1306_draw_pixel(x0 + y, y0 + x, color);
        ssd1306_draw_pixel(x0 - y, y0 + x, color);
        ssd1306_draw_pixel(x0 + y, y0 - x, color);
        ssd1306_draw_pixel(x0 - y, y0 - x, color);
    }
}

void ssd1306_draw_circle_helper(int16_t x0, int16_t y0,
                                int16_t r, uint8_t cornername, uint16_t color)
{
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
            ssd1306_draw_pixel(x0 + x, y0 + y, color);
            ssd1306_draw_pixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            ssd1306_draw_pixel(x0 + x, y0 - y, color);
            ssd1306_draw_pixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            ssd1306_draw_pixel(x0 - y, y0 + x, color);
            ssd1306_draw_pixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            ssd1306_draw_pixel(x0 - y, y0 - x, color);
            ssd1306_draw_pixel(x0 - x, y0 - y, color);
        }
    }
}

void ssd1306_fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    ssd1306_draw_fast_vline(x0, y0 - r, 2 * r + 1, color);
    ssd1306_fill_circle_helper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void ssd1306_fill_circle_helper(int16_t x0, int16_t y0, int16_t r,
                                uint8_t cornername, int16_t delta, uint16_t color)
{

    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;

        if (cornername & 0x1) {
            ssd1306_draw_fast_vline(x0 + x, y0 - y, 2 * y + 1 + delta, color);
            ssd1306_draw_fast_vline(x0 + y, y0 - x, 2 * x + 1 + delta, color);
        }
        if (cornername & 0x2) {
            ssd1306_draw_fast_vline(x0 - x, y0 - y, 2 * y + 1 + delta, color);
            ssd1306_draw_fast_vline(x0 - y, y0 - x, 2 * x + 1 + delta, color);
        }
    }
}

// Bresenham's algorithm - thx wikpedia
void ssd1306_draw_line(int16_t x0, int16_t y0,
                       int16_t x1, int16_t y1,
                       uint16_t color)
{
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
    	swap(x0, y0);
    	swap(x1, y1);
    }

    if (x0 > x1) {
    	swap(x0, x1);
    	swap(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    }
    else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (steep) {
            ssd1306_draw_pixel(y0, x0, color);
        }
        else {
            ssd1306_draw_pixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

// Draw a rectangle
void ssd1306_draw_rect(int16_t x, int16_t y,
                       int16_t w, int16_t h,
                       uint16_t color)
{
    ssd1306_draw_fast_hline(x, y, w, color);
    ssd1306_draw_fast_hline(x, y + h - 1, w, color);
    ssd1306_draw_fast_vline(x, y, h, color);
    ssd1306_draw_fast_vline(x + w - 1, y, h, color);
}

#if 0
void Adafruit_GFX::drawFastVLine(int16_t x, int16_t y,
                                 int16_t h, uint16_t color)
{
    // Update in subclasses if desired!
    drawLine(x, y, x, y + h - 1, color);
}

void Adafruit_GFX::drawFastHLine(int16_t x, int16_t y,
                                 int16_t w, uint16_t color)
{
    // Update in subclasses if desired!
    drawLine(x, y, x + w - 1, y, color);
}
#endif

void ssd1306_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    // Update in subclasses if desired!
    for (int16_t i = x; i < x + w; i++) {
        ssd1306_draw_fast_vline(i, y, h, color);
    }
}

void ssd1306_fill_screen(uint16_t color)
{
    ssd1306_fill_rect(0, 0, _width, _height, color);
}

// Draw a rounded rectangle
void ssd1306_draw_round_rect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    // smarter version
    ssd1306_draw_fast_hline(x + r  , y    , w - 2 * r, color); // Top
    ssd1306_draw_fast_hline(x + r  , y + h - 1, w - 2 * r, color); // Bottom
    ssd1306_draw_fast_vline(x    , y + r  , h - 2 * r, color); // Left
    ssd1306_draw_fast_vline(x + w - 1, y + r  , h - 2 * r, color); // Right
    // draw four corners
    ssd1306_draw_circle_helper(x + r    , y + r    , r, 1, color);
    ssd1306_draw_circle_helper(x + w - r - 1, y + r    , r, 2, color);
    ssd1306_draw_circle_helper(x + w - r - 1, y + h - r - 1, r, 4, color);
    ssd1306_draw_circle_helper(x + r    , y + h - r - 1, r, 8, color);
}

// Fill a rounded rectangle
void ssd1306_fill_round_rect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    // smarter version
    ssd1306_fill_rect(x + r, y, w - 2 * r, h, color);

    // draw four corners
    ssd1306_fill_circle_helper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
    ssd1306_fill_circle_helper(x + r    , y + r, r, 2, h - 2 * r - 1, color);
}

// Draw a triangle
void ssd1306_draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    ssd1306_draw_line(x0, y0, x1, y1, color);
    ssd1306_draw_line(x1, y1, x2, y2, color);
    ssd1306_draw_line(x2, y2, x0, y0, color);
}

// Fill a triangle
void ssd1306_fill_triangle( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    int16_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
    	swap(y0, y1);
    	swap(x0, x1);
    }
    if (y1 > y2) {
    	swap(y2, y1);
    	swap(x2, x1);
    }
    if (y0 > y1) {
    	swap(y0, y1);
    	swap(x0, x1);
    }

    if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if (x1 < a)      a = x1;
        else if (x1 > b) b = x1;
        if (x2 < a)      a = x2;
        else if (x2 > b) b = x2;
        ssd1306_draw_fast_hline(a, y0, b - a + 1, color);
        return;
    }

    int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
    int32_t
    sa   = 0,
    sb   = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if (y1 == y2) last = y1;  // Include y1 scanline
    else         last = y1 - 1; // Skip it

    for (y = y0; y <= last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if (a > b) swap(a, b);
        ssd1306_draw_fast_hline(a, y, b - a + 1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
        a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if (a > b) swap(a, b);
        ssd1306_draw_fast_hline(a, y, b - a + 1, color);
    }
}

/*
void ssd1306_draw_bitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color)
{
    int16_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                ssd1306_draw_pixel(x + i, y + j, color);
            }
        }
    }
}*/

// Draw a 1-bit color bitmap at the specified x, y position from the
// provided bitmap buffer (must be PROGMEM memory) using color as the
// foreground color and bg as the background color.
/*
void ssd1306_draw_bitmap_bg(int16_t x, int16_t y,
                            const uint8_t *bitmap, int16_t w, int16_t h,
                            uint16_t color, uint16_t bg)
{
    int16_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                ssd1306_draw_pixel(x + i, y + j, color);
            }
            else {
                ssd1306_draw_pixel(x + i, y + j, bg);
            }
        }
    }
}*/

//Draw XBitMap Files (*.xbm), exported from GIMP,
//Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
//C Array can be directly used with this function
/*
void ssd1306_draw_xbitmap(int16_t x, int16_t y,
                          const uint8_t *bitmap, int16_t w, int16_t h,
                          uint16_t color)
{

    int16_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (1 << (i % 8))) {
                ssd1306_draw_pixel(x + i, y + j, color);
            }
        }
    }
}*/

size_t ssd1306_write(uint8_t c)
{
    if (c == '\n') {
        cursor_y += textsize * 8;
        cursor_x  = 0;
    }
    else if (c == '\r') {
        // skip em
    }
    else {
        ssd1306_draw_char(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
        cursor_x += textsize * 6;
        if (wrap && (cursor_x > (_width - textsize * 6))) {
            cursor_y += textsize * 8;
            cursor_x = 0;
        }
    }

    return 1;
}

// Draw a character
void ssd1306_draw_char(int16_t x, int16_t y, uint8_t c, uint16_t color, uint16_t bg, uint8_t size)
{

    if ((x >= _width)            || // Clip right
            (y >= _height)           || // Clip bottom
            ((x + 6 * size - 1) < 0) || // Clip left
            ((y + 8 * size - 1) < 0))   // Clip top
        return;

    if (!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

    for (int8_t i = 0; i < 6; i++ ) {
        uint8_t line;
        if (i == 5)
            line = 0x0;
        else
            line = my_pgm_read_byte(my_font + (c * 5) + i);
        for (int8_t j = 0; j < 8; j++) {
            if (line & 0x1) {
                if (size == 1) // default size
                    ssd1306_draw_pixel(x + i, y + j, color);
                else {  // big size
                    ssd1306_fill_rect(x + (i * size), y + (j * size), size, size, color);
                }
            }
            else if (bg != color) {
                if (size == 1) // default size
                    ssd1306_draw_pixel(x + i, y + j, bg);
                else {  // big size
                    ssd1306_fill_rect(x + i * size, y + j * size, size, size, bg);
                }
            }
            line >>= 1;
        }
    }
}

void ssd1306_set_cursor(int16_t x, int16_t y)
{
    cursor_x = x;
    cursor_y = y;
}

int16_t ssd1306_get_cursor_x(void)
{
    return cursor_x;
}

int16_t ssd1306_get_cursor_y(void)
{
    return cursor_y;
}

void ssd1306_set_textsize(uint8_t s)
{
    textsize = (s > 0) ? s : 1;
}

void ssd1306_set_textcolor(uint16_t c)
{
    // For 'transparent' background, we'll set the bg
    // to the same as fg instead of using a flag
    textcolor = textbgcolor = c;
}

void ssd1306_set_textcolor_bg(uint16_t c, uint16_t b)
{
    textcolor   = c;
    textbgcolor = b;
}

void ssd1306_set_textwrap(uint8_t w)
{
    wrap = w;
}

uint8_t ssd1306_get_rotation(void)
{
    return rotation;
}

void ssd1306_set_rotation(uint8_t x)
{
    rotation = (x & 3);
    switch (rotation) {
    case 0:
    case 2:
        _width  = WIDTH;
        _height = HEIGHT;
        break;
    case 1:
    case 3:
        _width  = HEIGHT;
        _height = WIDTH;
        break;
    }
}

// Enable (or disable) Code Page 437-compatible charset.
// There was an error in glcdfont.c for the longest time -- one character
// (#176, the 'light shade' block) was missing -- this threw off the index
// of every character that followed it.  But a TON of code has been written
// with the erroneous character indices.  By default, the library uses the
// original 'wrong' behavior and old sketches will still work.  Pass 'true'
// to this function to use correct CP437 character values in your code.
void ssd1306_cp437(uint8_t x)
{
    _cp437 = x;
}


void ssd1306_putstring(char* buffer)
{
    while (*buffer) {
        ssd1306_write((uint8_t)*buffer);
        buffer++;
    }
}

void ssd1306_puts(char* buffer)
{
    ssd1306_putstring(buffer);
    ssd1306_write('\n');
}


