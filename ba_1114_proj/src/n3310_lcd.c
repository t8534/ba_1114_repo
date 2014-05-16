



#include "n3310_lcd.h"
#include "n3310_fonts.h"

N3310LCD::N3310LCD (PinName mosi, PinName miso, PinName sck,
                    PinName ce, PinName dat_cmd, PinName lcd_rst, PinName bl_on) :
                    lcdPort(mosi, miso, sck),
                    ceWire(ce), dcWire(dat_cmd), rstWire(lcd_rst), blWire(bl_on)
{
}

void N3310LCD::init()
{
    // use default SPI format
    lcdPort.format(8,0);
    lcdPort.frequency(1000000);

    // lcd reset
    wait_ms(1);
    rstWire = 0;
    wait_ms(1);
    rstWire = 1;

    write(0x21, CMD);
    write(0xc8, CMD);
    write(0x06, CMD);
    write(0x13, CMD);
    write(0x20, CMD);
    cls();
    write(0x0c, CMD);
}

void N3310LCD::cls()
{
    write(0x0c, CMD);
    write(0x80, CMD);

    for (int i = 0; i < 504; i++)
    {
        write(0, DATA);
    }
}

void N3310LCD::backlight(eBacklight state)
{
    // switch off/on back light
    blWire = state;
}

void N3310LCD::write(BYTE data, eRequestType req_type)
{
    // bring CS low for write
    ceWire = 0;

    if (CMD == req_type)
        dcWire = 0;
    else // DATA
        dcWire = 1;

    lcdPort.write(data);

    // write finished
    ceWire = 1;
}

void N3310LCD::locate(BYTE xPos, BYTE yPos)
{
    write(0x40 | yPos, CMD);      // column
    write(0x80 | xPos, CMD);      // row
}

void N3310LCD::drawBitmap(BYTE xPos, BYTE yPos, BYTE* bitmap, BYTE bmpXSize, BYTE bmpYSize)
{
    BYTE row;

    if (0 == bmpYSize % 8)
        row = bmpYSize/8;
    else
        row = bmpYSize/8 + 1;

    for (BYTE n = 0; n < row; n++)
    {
        locate(xPos, yPos);
        for(BYTE i = 0; i < bmpXSize; i++)
        {
            write(bitmap[i + (n * bmpXSize)], DATA);
        }
        yPos++;
    }
}

void N3310LCD::writeString(BYTE xPos, BYTE yPos, char* string, eDisplayMode mode)
{
    locate(xPos, yPos);

    while (*string)
    {
        writeChar(*string++, mode);
    }
}

void N3310LCD::writeStringBig(BYTE xPos, BYTE yPos, char* string, eDisplayMode mode)
{
    while (*string)
    {
        writeCharBig(xPos, yPos, *string , mode);

        if('.' == *string++)
            xPos += 5;
        else
            xPos += 12;
    }
}

void N3310LCD::writeChar(BYTE ch, eDisplayMode mode)
{
    BYTE sendByte;

    unsigned char* pFont = (unsigned char*)font6_8;
    ch -= 32;

    for (BYTE line = 0; line < 6; line++)
    {
        sendByte = *(pFont + ch*6 + line);
        write((mode == NORMAL)? sendByte: (sendByte ^ 0xff) , DATA);
    }
}

void N3310LCD::writeCharBig(BYTE xPos, BYTE yPos, BYTE ch, eDisplayMode mode)
{
    BYTE sendByte;

    unsigned char* pFont = (unsigned char *) big_number;

    if('.' == ch)
        ch = 10;
    else if ('+' == ch)
        ch = 11;
    else if ('-' == ch)
        ch = 12;
    else
        ch = ch & 0x0f;

    for(BYTE i = 0; i < 3; i++)
    {
        locate(xPos, yPos + i);

        for(BYTE j = 0; j < 16; j++)
        {
            sendByte =  *(pFont + ch*48 + i*16 + j);
            write((mode == NORMAL)? sendByte : (sendByte^0xff), DATA);
        }
    }
}


