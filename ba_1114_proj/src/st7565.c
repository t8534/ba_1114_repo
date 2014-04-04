#include "DogLCD.h"

// macro to make sure x falls into range from low to high (inclusive)
#define CLAMP(x, low, high) { if ( (x) < (low) ) x = (low); if ( (x) > (high) ) x = (high); } while (0);

void DogLCD::_send_commands(const unsigned char* buf, size_t size)
{
    // for commands, A0 is low
    _spi.format(8,0);
    _spi.frequency(10000000);
    _cs = 0;
    _a0 = 0;
    while ( size-- > 0 )
        _spi.write(*buf++);
    _cs = 1;
}

void DogLCD::_send_data(const unsigned char* buf, size_t size)
{
    // for data, A0 is high
    _spi.format(8,0);
    _spi.frequency(10000000);
    _cs = 0;
    _a0 = 1;
    while ( size-- > 0 )
        _spi.write(*buf++);
    _cs = 1;
    _a0 = 0;
}

// set column and page number
void DogLCD::_set_xy(int x, int y)
{
    //printf("_set_xy(%d,%d)\n", x, y);
    CLAMP(x, 0, LCDWIDTH-1);
    CLAMP(y, 0, LCDPAGES-1);
    unsigned char cmd[3];
    cmd[0] = 0xB0 | (y&0xF);
    cmd[1] = 0x10 | (x&0xF);
    cmd[2] = (x>>4)&0xF;
    _send_commands(cmd, 3);
}

// initialize and turn on the display
void DogLCD::init()
{
    const unsigned char init_seq[] = {
        0x40,    //Display start line 0
        0xa1,    //ADC reverse
        0xc0,    //Normal COM0...COM63
        0xa6,    //Display normal
        0xa2,    //Set Bias 1/9 (Duty 1/65)
        0x2f,    //Booster, Regulator and Follower On
        0xf8,    //Set internal Booster to 4x
        0x00,
        0x27,    //Contrast set
        0x81,
        0x16,
        0xac,    //No indicator
        0x00,
        0xaf,    //Display on
    };
    //printf("Reset=L\n");
    _reset = 0;
    //printf("Power=H\n");
    _power = 1;
    //wait_ms(1);
    //printf("Reset=H\n");
    _reset = 1;
    //wait(5);
    //printf("Sending init commands\n");
    _send_commands(init_seq, sizeof(init_seq));
}

void DogLCD::send_pic(const unsigned char* data)
{
    //printf("Sending picture\n");
    for (int i=0; i<LCDPAGES; i++)
    {
        _set_xy(0, i);
        _send_data(data + i*LCDWIDTH, LCDWIDTH);
    }
}

void DogLCD::clear_screen()
{
    //printf("Clear screen\n");
    memset(_framebuffer, 0, sizeof(_framebuffer));
    if ( _updating == 0 )
    {
        send_pic(_framebuffer);
    }
}

void DogLCD::all_on(bool on)
{
    //printf("Sending all on %d\n", on);
    unsigned char cmd = 0xA4 | (on ? 1 : 0);
    _send_commands(&cmd, 1);
}

void DogLCD::pixel(int x, int y, int colour)
{
    CLAMP(x, 0, LCDWIDTH-1);
    CLAMP(y, 0, LCDHEIGHT-1);
    int page = y / 8;
    unsigned char mask = 1<<(y%8);
    unsigned char *byte = &_framebuffer[page*LCDWIDTH + x];
    if ( colour == 0 )
        *byte &= ~mask; // clear pixel
    else
        *byte |= mask; // set pixel
    if ( !_updating )
    {
        _set_xy(x, page);
        _send_data(byte, 1);
    }
}

void DogLCD::fill(int x, int y, int width, int height, int colour)
{
    /*
      If we need to fill partial pages at the top:

      ......+---+---+.....
       ^    | = | = |     = : don't touch
       |    | = | = |     * : update
      y%8   | = | = |
       |    | = | = |
       v    | = | = |
    y---->  | * | * |
            | * | * |
            | * | * |
            +---+---+
    */
    //printf("fill(x=%d, y=%d, width=%d, height=%d, colour=%x)\n",  x, y, width, height, colour);
    CLAMP(x, 0, LCDWIDTH-1);
    CLAMP(y, 0, LCDHEIGHT-1);
    CLAMP(width, 0, LCDWIDTH - x);
    CLAMP(height, 0, LCDHEIGHT - y);
    int page = y/8;
    int firstpage = page;
    int partpage = y%8;
    if ( partpage != 0 )
    {
        // we need to process partial bytes in the top page
        unsigned char mask = (1<<partpage) - 1; // this mask has 1s for bits we need to leave
        unsigned char *bytes = &_framebuffer[page*LCDWIDTH + x];
        for ( int i = 0; i < width; i++, bytes++ )
        {
          // clear "our" bits
          *bytes &= mask;
          if ( colour != 0 )
            *bytes |= ~mask; // set our bits
        }
        height -= partpage;
        page++;
    }
    while ( height >= 8 )
    {
        memset(&_framebuffer[page*LCDWIDTH + x], colour == 0 ? 0 : 0xFF, width);
        page++;
        height -= 8;
    }
    if ( height != 0 )
    {
        // we need to process partial bytes in the bottom page
        unsigned char mask = ~((1<<partpage) - 1); // this mask has 1s for bits we need to leave
        unsigned char *bytes = &_framebuffer[page*LCDWIDTH + x];
        for ( int i = 0; i < width; i++, bytes++ )
        {
          // clear "our" bits
          *bytes &= mask;
          if ( colour != 0 )
            *bytes |= ~mask; // set our bits
        }
        page++;
    }
    //printf("_updating=%d\n", _updating);
    if ( !_updating )
    {
        int laststpage = page;
        for ( page = firstpage; page < laststpage; page++)
        {
            //printf("setting x=%d, page=%d\n", x, page);
            _set_xy(x, page);
            //printf("sending %d bytes at offset %x\n", width, page*LCDWIDTH + x);
            _send_data(&_framebuffer[page*LCDWIDTH + x], width);
        }
    }
}

void DogLCD::beginupdate()
{
    _updating++;
    //printf("beginupdate: %d\n", _updating);
}

void DogLCD::endupdate()
{
    _updating--;
    //printf("endupdate: %d\n", _updating);
    if ( _updating == 0 )
    {
        send_pic(_framebuffer);
    }
}
