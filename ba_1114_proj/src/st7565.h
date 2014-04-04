#ifndef MBED_DOGLCD_H
#define MBED_DOGLCD_H

#include "AbstractLCD.h"

/***********
 * Module for Electronic Assembly's DOGL128-6 display module
 * Should be compatible with other modules using ST7565 controller
 ***********/

#define LCDWIDTH 128
#define LCDHEIGHT 64
#define LCDPAGES  (LCDHEIGHT+7)/8

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

class DogLCD: public AbstractLCD
{
    SPI& _spi;
    DigitalOut _cs, _a0, _reset, _power;
    int _updating;
    void _send_commands(const unsigned char* buf, size_t size);
    void _send_data(const unsigned char* buf, size_t size);
    void _set_xy(int x, int y);
    unsigned char _framebuffer[LCDWIDTH*LCDPAGES];
public:
    DogLCD(SPI& spi, PinName power, PinName cs, PinName a0, PinName reset):
    _spi(spi), _cs(cs), _a0(a0), _reset(reset), _power(power), _updating(0)
    {
    }
    // initialize and turn on the display
    void init();
    // send a 128x64 picture for the whole screen
    void send_pic(const unsigned char* data);
    // clear screen
    void clear_screen();
    // turn all pixels on
    void all_on(bool on = true);

    // AbstractLCD methods
    virtual int width()  {return LCDWIDTH;};
    virtual int height() {return LCDHEIGHT;};
    virtual void pixel(int x, int y, int colour);
    virtual void fill(int x, int y, int width, int height, int colour);
    virtual void beginupdate();
    virtual void endupdate();
};

#endif
