#ifndef __N3310_LCD_H_
#define __N3310_LCD_H_



enum eDisplayMode {NORMAL, HIGHLIGHT};
enum eRequestType {CMD, DATA};
enum eBacklight {OFF, ON};


class N3310LCD
{
public:
    N3310LCD(PinName mosi, PinName miso, PinName sck,
             PinName ce, PinName dat_cmd, PinName lcd_rst, PinName bl_on);

    void init();
    void cls();
    void backlight(eBacklight state);
    void write(BYTE data, eRequestType req_type);
    void locate(BYTE xPos, BYTE yPos);

    void drawBitmap(BYTE xPos, BYTE yPos, BYTE* bitmap, BYTE bmpXSize, BYTE bmpYSize);
    void writeString(BYTE xPos, BYTE yPos, char* string, eDisplayMode mode);
    void writeStringBig(BYTE xPos, BYTE yPos, char* string, eDisplayMode mode);
    void writeChar(BYTE ch, eDisplayMode mode);
    void writeCharBig(BYTE xPos, BYTE yPos, BYTE ch, eDisplayMode mode);

private:
    // I/O
    SPI lcdPort;            // does SPI MOSI, MISO and SCK
    DigitalOut ceWire;      // does SPI CE
    DigitalOut dcWire;      // does 3310 DAT_CMD
    DigitalOut rstWire;     // does 3310 LCD_RST
    DigitalOut blWire;      // does 3310 BL_ON (backlight)
};







#endif /* DISPLAY_PC_H_ */
