#ifndef ST7565_H
#define ST7565_H


/***********
 * Module for Electronic Assembly's DOGL128-6 display module
 * Should be compatible with other modules using ST7565 controller
 ***********/


/*

 Each page is 8 lines, one byte per column

         Col0
        +---+--
        | 0 |
Page 0  | 1 |
        | 2 |
        | 3 |
        | 4 |
        | 5 |
        | 6 |
        | 7 |
        +---+--
*/

/*
  LCD interface class.
  Usage:
    DogLCD dog(spi, pin_power, pin_cs, pin_a0, pin_reset);
    where spi is an instance of SPI class
*/

/*
    DogLCD(SPI& spi, PinName power, PinName cs, PinName a0, PinName reset):
    _spi(spi), _cs(cs), _a0(a0), _reset(reset), _power(power), _updating(0)
    {
    }
*/

typedef enum {
	ST7565_RESET = 0,
	ST7565_SET = 1,
} ST7565_PixelState_t;


// initialize and turn on the display
void ST7565_init();
// send a 128x64 picture for the whole screen
void ST7565_send_pic(const unsigned char* data);
// clear screen
void ST7565_clear_screen();
// turn all pixels on
void ST7565_all_on(ST7565_PixelState_t state);

int ST7565_getWidth(void);
int ST7565_getHeight(void);
void ST7565_pixel(int x, int y, int colour);
void ST7565_fill(int x, int y, int width, int height, int colour);
void ST7565_beginupdate();
void ST7565_endupdate();


#endif
