#ifndef __N3310_LCD_H_
#define __N3310_LCD_H_


enum eDisplayMode {NORMAL, HIGHLIGHT};
enum eRequestType {CMD, DATA};
enum eBacklight {OFF, ON};


void N3310LCD_init(void);
void N3310LCD_cls(void);
void N3310LCD_backlight(eBacklight state);
void N3310LCD_write(uint8_t data, eRequestType req_type);
void N3310LCD_locate(uint8_t xPos, uint8_t yPos);

void N3310LCD_drawBitmap(uint8_t xPos, uint8_t yPos, uint8_t* bitmap, uint8_t bmpXSize, uint8_t bmpYSize);
void N3310LCD_writeString(uint8_t xPos, uint8_t yPos, char* string, eDisplayMode mode);
void N3310LCD_writeStringBig(uint8_t xPos, uint8_t yPos, char* string, eDisplayMode mode);
void N3310LCD_writeChar(uint8_t ch, eDisplayMode mode);
void N3310LCD_writeCharBig(uint8_t xPos, uint8_t yPos, uint8_t ch, eDisplayMode mode);


#endif /* DISPLAY_PC_H_ */
