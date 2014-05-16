

#include <string.h>
#include "driver_config.h"
#include "timer16.h"
#include "gpio.h"
#include "ssp.h"
#include "n3310_lcd.h"
#include "n3310_fonts.h"

static SSP_Dev_t SSP_Dev;


/* todo
 *
 *   Pinout:
 *
 *   spi, pin_power, pin_cs, pin_a0, pin_reset
 *
 *
 * The display N3310 is connected by SPI1 on lpcexpresso 1114 board.
 * SPI0 is shared with debugger pins.
 *
 * SPI                                         N3310
 *
 * SPI MOSI1 - PIO2_3  - PIO2_3 J6-45   -----  mosi       - pin xx
 * SPI MISO1 - PIO2_2  - PIO2_2 J6-14   -----  miso
 * SPI SCK1  - PIO2_1  - PIO2_1 J16-13  -----  sck        - pin xx
 * SPI SSEL1 - PIO2_0  - PIO2_0 J6-12   -----  ce         - pin xx    -- CS
 * IO_PIN    - PIO2_6  - PIO2_6         -----  dat_cmd    - pin xx
 * IO_PIN    - PIO2_7  - PIO2_7         -----  lcd_rst    - pin xx
 * IO_PIN    - PIO2_8  - PIO2_8         -----  bl_on      - pin xx
 *
 *
 */


/* old
N3310LCD::N3310LCD (PinName mosi, PinName miso, PinName sck,
                    PinName ce, PinName dat_cmd, PinName lcd_rst, PinName bl_on) :
                    lcdPort(mosi, miso, sck),
                    ceWire(ce), dcWire(dat_cmd), rstWire(lcd_rst), blWire(bl_on)
{
}
*/

typedef enum {
	N3310_PIN_RESET = 0,
	N3310_PIN_SET = 1
} PinState_t;


// SSEL1 is controlled by SSP driver
static void CE_Pin(PinState_t state)
{
	if (state == N3310_PIN_SET)
    {
		SSP_SSEL1_GPIO_High();
    }
	else if (state == N3310_PIN_RESET)
	{
    	SSP_SSEL1_GPIO_Low();
    }

    return;
}


static void DataCmd_Pin(PinState_t state)
{
	if (state == N3310_PIN_SET)
    {
		GPIOSetValue(PORT2, 6, 1);
    }
	else if (state == N3310_PIN_RESET)
	{
		GPIOSetValue(PORT2, 6, 0);
    }

	return;
}


static void LcdRst_Pin(PinState_t state)
{
	if (state == N3310_PIN_SET)
    {
		GPIOSetValue(PORT2, 7, 1);
    }
	else if (state == N3310_PIN_RESET)
	{
		GPIOSetValue(PORT2, 7, 0);
    }

	return;
}


static void B_ON_Pin(PinState_t state)
{
	if (state == N3310_PIN_SET)
    {
		GPIOSetValue(PORT2, 8, 1);
    }
	else if (state == N3310_PIN_RESET)
	{
		GPIOSetValue(PORT2, 8, 0);
    }

	return;
}



static void ioInit(void)
{
    // SPI SSEL pin -> CE is controlled at SSP driver
	CE_Pin(N3310_PIN_SET);    // Chip is inactive

	/* Enable AHB clock to the GPIO domain. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

    /* DataCmd pin:
     * Data/Command
     * 0 - Command
     * 1 - Data
     */
    //LPC_IOCON->PIO2_6 &= ~0x07;
    //LPC_IOCON->PIO2_6 |= 0x00;   /* GPIO pin mode = 0x00 */
    LPC_IOCON->PIO2_6 = 0x00;      /* GPIO pin mode = 0x00 */
    GPIOSetDir(PORT2, 6, 1);       /* Output */
    GPIOSetValue(PORT2, 6, 0);     /* Low - Command mode */


    /* LcdRst pin:
     * RESET pin, active Low
     */
    //LPC_IOCON->PIO2_7 &= ~0x07;
    //LPC_IOCON->PIO2_7 |= 0x00;   /* GPIO pin mode = 0x00 */
    LPC_IOCON->PIO2_7 = 0x00;      /* GPIO pin mode = 0x00 */
    GPIOSetDir(PORT2, 7, 1);       /* Output */
    GPIOSetValue(PORT2, 7, 1);     /* High - Reset not active */


    /* B_ON pin:
     * Backlight pin, active Low todo ???
     */
    //LPC_IOCON->PIO2_8 &= ~0x07;
    //LPC_IOCON->PIO2_8 |= 0x00;   /* GPIO pin mode = 0x00 */
    LPC_IOCON->PIO2_8 = 0x00;      /* GPIO pin mode = 0x00 */
    GPIOSetDir(PORT2, 8, 1);       /* Output */
    GPIOSetValue(PORT2, 8, 1);     /* High - Reset not active */

}


//todo
static void spiInit(void)
{
	SSP_Dev.Device = LPC_SSP1;
	SSP_Dev.FrameFormat = SSP_FRAME_SPI;
	SSP_Dev.DataSize = SSP_DATABITS_8;
	SSP_Dev.CPOL = SSP_SPI_CPOL_HI;    //todo
	SSP_Dev.CPHA = SSP_SPI_CPHA_FIRST; //todo
	SSP_Dev.LoopBackMode = SSP_LOOPBACK_OFF;
	SSP_Dev.Mode = SSP_MASTER_MODE;
                                     //todo what a frequency
	SSP_Dev.SCR = 0x07;              /* CR0->SerialClockRate */
	SSP_Dev.CPSDVSR = 0x02;          /* SSPxCPSR->CPSDVSR */
	SSP_Dev.DIV = 0x02;              /* SSPxCLKDIV->DIV */

	SSP_Dev.SlaveOutputDisable = SSP_SLAVE_OUTPUT_ENABLE;
	SSP_Dev.transferType = SSP_TRANSFER_POLLING;
	SSP_Dev.InterruptCondition = SSP_ISR_NOFLAG_SET;
	SSP_Dev.ISR_Processing = NULL;
	SSP_Dev.SSEL_Mode = SSP_SSEL_GPIO;  // Manually controlled
	SSP_Dev.IO_pins.MOSI_pin = SSP_MOSI1_PIN_2_3;
	SSP_Dev.IO_pins.MISO_pin = SSP_MISO1_PIN_2_2;
	SSP_Dev.IO_pins.SCK_pin = SSP_SCK1_PIN_2_1;
	SSP_Dev.IO_pins.SSEL_pin = SSP_NO_PIN;  // Manually controlled

	SSP_Init(&SSP_Dev);
}


// Send always 1 byte
static void spi_send(const uint8_t val)
{
    uint16_t rxVal16 = 0;
    uint16_t txVal16 = 0;


   	txVal16 = val;
   	SSP_WriteRead(&SSP_Dev, &txVal16, &rxVal16, 1);

   	return;
}


void N3310LCD_write(BYTE data, eRequestType req_type)
{
    // bring CS low for write
	CE_Pin(N3310_PIN_RESET);    //ceWire = 0;

    if (CMD == req_type)
    	DataCmd_Pin(N3310_PIN_RESET);    //dcWire = 0;
    else // DATA
    	DataCmd_Pin(N3310_PIN_SET);    //dcWire = 1;

    spi_send(data);    //lcdPort.write(data);

    // write finished
    CE_Pin(N3310_PIN_SET);    //ceWire = 1;
}


void N3310LCD_init()
{
    // use default SPI format
    //lcdPort.format(8,0);
    //lcdPort.frequency(1000000);

    ioInit();
    spiInit();

    // lcd reset
    delayMs(0, 1);    //wait_ms(1);    // todo check is clock ok.
    LcdRst_Pin(N3310_PIN_RESET);   // rstWire = 0;
    delayMs(0, 1);    //wait_ms(1);
    LcdRst_Pin(N3310_PIN_SET);   //rstWire = 1;

    N3310LCD_write(0x21, CMD);
    N3310LCD_write(0xc8, CMD);
    N3310LCD_write(0x06, CMD);
    N3310LCD_write(0x13, CMD);
    N3310LCD_write(0x20, CMD);
    N3310LCD_cls();
    N3310LCD_write(0x0c, CMD);
}

void N3310LCD_cls()
{
	N3310LCD_write(0x0c, CMD);
	N3310LCD_write(0x80, CMD);

    for (int i = 0; i < 504; i++)
    {
    	N3310LCD_write(0, DATA);
    }
}

void N3310LCD_backlight(eBacklight state)
{
	//todo check is state match
    // switch off/on back light
	B_ON_Pin(state);    //blWire = state;
}


void N3310LCD_locate(uint8_t xPos, uint8_t yPos)
{
	N3310LCD_write(0x40 | yPos, CMD);      // column
	N3310LCD_write(0x80 | xPos, CMD);      // row
}

void N3310LCD_drawBitmap(uint8_t xPos, uint8_t yPos, uint8_t* bitmap, uint8_t bmpXSize, uint8_t bmpYSize)
{
	uint8_t row;

    if (0 == bmpYSize % 8)
        row = bmpYSize/8;
    else
        row = bmpYSize/8 + 1;

    for (uint8_t n = 0; n < row; n++)
    {
    	N3310LCD_locate(xPos, yPos);
        for(uint8_t i = 0; i < bmpXSize; i++)
        {
        	N3310LCD_write(bitmap[i + (n * bmpXSize)], DATA);
        }
        yPos++;
    }
}

void N3310LCD_writeString(uint8_t xPos, uint8_t yPos, char* string, eDisplayMode mode)
{
	N3310LCD_locate(xPos, yPos);

    while (*string)
    {
    	N3310LCD_writeChar(*string++, mode);
    }
}

void N3310LCD_writeStringBig(uint8_t xPos, uint8_t yPos, char* string, eDisplayMode mode)
{
    while (*string)
    {
    	N3310LCD_writeCharBig(xPos, yPos, *string , mode);

        if('.' == *string++)
            xPos += 5;
        else
            xPos += 12;
    }
}

void N3310LCD_writeChar(uint8_t ch, eDisplayMode mode)
{
	uint8_t sendByte;

    unsigned char* pFont = (unsigned char*)font6_8;
    ch -= 32;

    for (uint8_t line = 0; line < 6; line++)
    {
        sendByte = *(pFont + ch*6 + line);
        N3310LCD_write((mode == NORMAL)? sendByte: (sendByte ^ 0xff) , DATA);
    }
}

void N3310LCD_writeCharBig(uint8_t xPos, uint8_t yPos, uint8_t ch, eDisplayMode mode)
{
	uint8_t sendByte;

    unsigned char* pFont = (unsigned char *) big_number;

    if('.' == ch)
        ch = 10;
    else if ('+' == ch)
        ch = 11;
    else if ('-' == ch)
        ch = 12;
    else
        ch = ch & 0x0f;

    for(uint8_t i = 0; i < 3; i++)
    {
    	N3310LCD_locate(xPos, yPos + i);

        for(uint8_t j = 0; j < 16; j++)
        {
            sendByte =  *(pFont + ch*48 + i*16 + j);
            N3310LCD_write((mode == NORMAL)? sendByte : (sendByte^0xff), DATA);
        }
    }
}


