/*
 *
 *   Pinout:


 *
 *   spi, pin_power, pin_cs, pin_a0, pin_reset
 *
 *
 * The display ST7565 is connected by SPI1 on lpcexpresso 1114 board.
 * SPI0 is shared with debugger pins.
 *
 * SPI                                         ST7565
 *
 * SPI MOSI1 - PIO2_3  - PIO2_3 J6-45   -----  SI   - pin 16
 * SPI MISO1 - PIO2_2  - PIO2_2 J6-14   -----
 * SPI SCK1  - PIO2_1  - PIO2_1 J16-13  -----  SCL  - pin 17
 * SPI SSEL1 - PIO2_0  - PIO2_0 J6-12   -----  CS1B - pin 20                 (chip active - low)
 * IO_PIN    - PIO2_7  - PIO2_7         -----  A0   - pin 18 --- A0_PIN      (low - control data mode)
 * IO_PIN    - PIO2_8  - PIO2_8         -----  RST  - pin 19 --- RESET_PIN   (low force reset)
 *
 */

#include <string.h>
#include "driver_config.h"
#include "timer16.h"
#include "gpio.h"
#include "ssp.h"
#include "st7565.h"

#define LCDWIDTH 128
#define LCDHEIGHT 64
#define LCDPAGES  (LCDHEIGHT+7)/8

// macro to make sure x falls into range from low to high (inclusive)
#define CLAMP(x, low, high) { if ( (x) < (low) ) x = (low); if ( (x) > (high) ) x = (high); } while (0);

typedef enum {
	ST7565_PIN_RESET = 0,
	ST7565_PIN_SET = 1
} PinState_t;


static unsigned char framebuffer[LCDWIDTH*LCDPAGES];
static int _updating;
static SSP_Dev_t SSP_Dev;


// SSEL1 is controlled by SSP driver
static void CS_Pin(PinState_t state)
{
	if (state == ST7565_PIN_SET)
    {
		SSP_SSEL1_GPIO_High();
    }
	else if (state == ST7565_PIN_RESET)
	{
    	SSP_SSEL1_GPIO_Low();
    }

    return;
}


static void A0_Pin(PinState_t state)
{
	if (state == ST7565_PIN_SET)
    {
		GPIOSetValue(PORT2, 7, 1);
    }
	else if (state == ST7565_PIN_RESET)
	{
		GPIOSetValue(PORT2, 7, 0);
    }

	return;
}


static void RESET_Pin(PinState_t state)
{
	if (state == ST7565_PIN_SET)
    {
		GPIOSetValue(PORT2, 8, 1);
    }
	else if (state == ST7565_PIN_RESET)
	{
		GPIOSetValue(PORT2, 8, 0);
    }

	return;
}




static void send_commands(const unsigned char* buf, size_t size)
{
    uint16_t rxVal16 = 0;
    uint16_t txVal16 = 0;

	// for commands, A0 is low
    //_spi.format(8,0);
    //_spi.frequency(10000000);

	CS_Pin(ST7565_PIN_RESET);    //_cs = 0;
	A0_Pin(ST7565_PIN_RESET);    //_a0 = 0;

    while ( size-- > 0 )
    {
        //_spi.write(*buf++);
    	txVal16 = *buf++;
    	SSP_WriteRead(&SSP_Dev, &txVal16, &rxVal16, 1);
    }

    CS_Pin(ST7565_PIN_SET);      //_cs = 1;

}

static void send_data(const unsigned char* buf, size_t size)
{
    uint16_t rxVal16 = 0;
    uint16_t txVal16 = 0;

	// for data, A0 is high
    //_spi.format(8,0);
    //_spi.frequency(10000000);

	CS_Pin(ST7565_PIN_RESET);    //_cs = 0;
	A0_Pin(ST7565_PIN_SET);      //_a0 = 1;

    while ( size-- > 0 )
    {
        //_spi.write(*buf++);
    	txVal16 = *buf++;
    	SSP_WriteRead(&SSP_Dev, &txVal16, &rxVal16, 1);
    }

    CS_Pin(ST7565_PIN_SET);      //_cs = 1;
    A0_Pin(ST7565_PIN_RESET);    //_a0 = 0;
}

// set column and page number
static void set_xy(int x, int y)
{
    unsigned char cmd[3];


    //printf("_set_xy(%d,%d)\n", x, y);
    CLAMP(x, 0, LCDWIDTH-1);
    CLAMP(y, 0, LCDPAGES-1);
    cmd[0] = 0xB0 | (y&0xF);
    cmd[1] = 0x10 | (x&0xF);
    cmd[2] = (x>>4)&0xF;
    send_commands(cmd, 3);
}


static void ioInit(void)
{
    // SPI SSEL pin -> CS is controlled at SSP driver
	CS_Pin(ST7565_PIN_SET);    // Chip is inactive

	/* Enable AHB clock to the GPIO domain. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

    /* A0 pin */
    //LPC_IOCON->PIO2_7 &= ~0x07;
    //LPC_IOCON->PIO2_7 |= 0x00;   /* GPIO pin mode = 0x00 */
    LPC_IOCON->PIO2_7 = 0x00;   /* GPIO pin mode = 0x00 */
    GPIOSetDir(PORT2, 7, 1);     /* Output */
    GPIOSetValue(PORT2, 7, 0);   /* Low - Control Data mode */

    /* RESET pin */
    //LPC_IOCON->PIO2_8 &= ~0x07;
    //LPC_IOCON->PIO2_8 |= 0x00;   /* GPIO pin mode = 0x00 */
    LPC_IOCON->PIO2_8 = 0x00;   /* GPIO pin mode = 0x00 */
    GPIOSetDir(PORT2, 8, 1);     /* Output */
    GPIOSetValue(PORT2, 8, 1);   /* High - Reset not active */

}

//todo
static void spiInit(void)
{
	SSP_Dev.Device = LPC_SSP1;
	SSP_Dev.FrameFormat = SSP_FRAME_SPI;
	SSP_Dev.DataSize = SSP_DATABITS_8;
	SSP_Dev.CPOL = SSP_SPI_CPOL_HI;
	SSP_Dev.CPHA = SSP_SPI_CPHA_FIRST;
	SSP_Dev.LoopBackMode = SSP_LOOPBACK_OFF;
	SSP_Dev.Mode = SSP_MASTER_MODE;

	SSP_Dev.SCR = 0x07;              /* CR0->SerialClockRate */
	SSP_Dev.CPSDVSR = 0x02;          /* SSPxCPSR->CPSDVSR */
	SSP_Dev.DIV = 0x02;              /* SSPxCLKDIV->DIV */

	SSP_Dev.SlaveOutputDisable = SSP_SLAVE_OUTPUT_ENABLE;
	SSP_Dev.transferType = SSP_TRANSFER_POLLING;
	SSP_Dev.InterruptCondition = SSP_ISR_NOFLAG_SET;
	SSP_Dev.ISR_Processing = NULL;
	SSP_Dev.SSEL_Mode = SSP_SSEL_GPIO;
	SSP_Dev.IO_pins.MOSI_pin = SSP_MOSI1_PIN_2_3;
	SSP_Dev.IO_pins.MISO_pin = SSP_MISO1_PIN_2_2;
	SSP_Dev.IO_pins.SCK_pin = SSP_SCK1_PIN_2_1;
	SSP_Dev.IO_pins.SSEL_pin = SSP_NO_PIN;

	SSP_Init(&SSP_Dev);
}



// initialize and turn on the display
void ST7565_init()
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


    ioInit();
    spiInit();



    //printf("Reset=L\n");

    RESET_Pin(ST7565_PIN_RESET);    //_reset = 0;

    //printf("Power=H\n");
    //_power = 1;

    //todo ? wait_ms(1);
    delayMs(0, 1);

    //printf("Reset=H\n");
    RESET_Pin(ST7565_PIN_SET);    //_reset = 1;


    //todo ? wait(5);
    delayMs(0, 5000);

    //printf("Sending init commands\n");
    send_commands(init_seq, sizeof(init_seq));
}


int ST7565_getWidth(void)
{
    return LCDWIDTH;
}


int ST7565_getHeight(void)
{
    return LCDHEIGHT;
}


void ST7565_send_pic(const unsigned char* data)
{
	int i = 0;


    //printf("Sending picture\n");
    for (i = 0; i < LCDPAGES; i++)
    {
        set_xy(0, i);
        send_data(data + i*LCDWIDTH, LCDWIDTH);
    }
}

void ST7565_clear_screen()
{
    //printf("Clear screen\n");
    memset(framebuffer, 0, sizeof(framebuffer));
    if ( _updating == 0 )
    {
    	ST7565_send_pic(framebuffer);
    }
}

void ST7565_all_on(ST7565_PixelState_t state)
{
    //printf("Sending all on %d\n", on);
    unsigned char cmd = 0xA4 | (state ? 1 : 0);
    send_commands(&cmd, 1);
}

void ST7565_pixel(int x, int y, int colour)
{
    CLAMP(x, 0, LCDWIDTH-1);
    CLAMP(y, 0, LCDHEIGHT-1);
    int page = y / 8;
    unsigned char mask = 1<<(y%8);
    unsigned char *byte = &framebuffer[page*LCDWIDTH + x];

    if ( colour == 0 )
        *byte &= ~mask; // clear pixel
    else
        *byte |= mask; // set pixel
    if ( !_updating )
    {
        set_xy(x, page);
        send_data(byte, 1);
    }
}

void ST7565_fill(int x, int y, int width, int height, int colour)
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
    int i = 0;

    if (partpage != 0)
    {
        // we need to process partial bytes in the top page
        unsigned char mask = (1 << partpage) - 1; // this mask has 1s for bits we need to leave
        unsigned char *bytes = &framebuffer[page*LCDWIDTH + x];
        for (i = 0; i < width; i++, bytes++)
        {
          // clear "our" bits
          *bytes &= mask;
          if ( colour != 0 )
            *bytes |= ~mask; // set our bits
        }
        height -= partpage;
        page++;
    }
    while (height >= 8)
    {
        memset(&framebuffer[page*LCDWIDTH + x], colour == 0 ? 0 : 0xFF, width);
        page++;
        height -= 8;
    }
    if (height != 0)
    {
        // we need to process partial bytes in the bottom page
        unsigned char mask = ~((1<<partpage) - 1); // this mask has 1s for bits we need to leave
        unsigned char *bytes = &framebuffer[page*LCDWIDTH + x];
        for ( i = 0; i < width; i++, bytes++ )
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
            set_xy(x, page);
            //printf("sending %d bytes at offset %x\n", width, page*LCDWIDTH + x);
            send_data(&framebuffer[page*LCDWIDTH + x], width);
        }
    }
}

void ST7565_beginupdate()
{
    _updating++;
    //printf("beginupdate: %d\n", _updating);
}

void ST7565_endupdate()
{
    _updating--;
    //printf("endupdate: %d\n", _updating);
    if ( _updating == 0 )
    {
    	ST7565_send_pic(framebuffer);
    }
}
