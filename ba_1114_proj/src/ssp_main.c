/****************************************************************************
 *   $Id:: ssp_main.c 4785 2010-09-03 22:39:27Z nxp21346                    $



 *   Project: NXP LPC11xx SSP example
 *
 *   Description:
 *     This file contains SSP test modules, main entry, to test SSP APIs.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/

/* Test conditions:
 *
 * SPI:
 *
 * SPITESTS_LoopbackInternalTest() is used for internal loopback test.
 * SPI0 or SPI1 canbe tested on LPCExpresso LPC1114 board.
 *
 * SPI0 and SPI1 are used connected together for base hardware interconnection
 * test - SPITESTS_LoopbackHardwareISPTest()
 *
 * LPCExpresso LPC1114 board pinout:
 *
 *             uC pin    PCB
 *
 * SPI MOSI0 - PIO0_9  - (MOSI/SWO) PIO0_9 J6-5
 * SPI MISO0 - PIO0_8  - (MISO) PIO0_8 J6-6
 * SPI SCK0  - PIO2_11 - (SCK) PIO2_11 J6-7
 * SPI SSEL0 - PIO0_2  - (SSEL0) PIO0_2 J6-8
 *
 *
 * SPI MOSI1 - PIO2_3  - PIO2_3 J6-45
 * SPI MISO1 - PIO2_2  - PIO2_2 J6-14
 * SPI SCK1  - PIO2_1  - PIO2_1 J16-13
 * SPI SSEL1 - PIO2_0  - PIO2_0 J6-12
 *
 *
 * UART1-TX/I2C1-SDA - PIO1_7 - J6-9
 * UART1-RX/I2C1-SCl - PIO1_6 - J6-10
 *
 *
 * UART3-TX/I2C2-SDA - PIO0_5 - J6-40
 * UART3-RX/I2C2-SCL - PIO0_4 - J6-41
 *
 *
 * The clock settings:
 * -------------------
 *
 * See p.22 UM10389 manual.
 *
 * XTAL = 12 MHz
 *
 * PLL is used. The sys_pllclkout = 48 MHz
 * main clock = 48 MHz
 *
 * System Clock Divider = 1 ->  system clock = 48 MHz
 *
 *
 * SSP0 peripherial clock divider = 2 -> SPI0 Peripherial Clock SPI0_PCLK = 24 MHz
 * Following BitFreq  formula for SPI0:
 *
 * BitFreq = SPI0_PCLK / (CPSDVSR x (SCR + 1)) = SPI0_PCLK/16 = 24/16 = 1.5 Mbps
 * CPSDVSR = 2
 * SCR = 7
 *
 *
 *
 *
 * Additional notes:
 * =================
 *
 * 1.
 * If, for test purposes both SPI0 and SPI1 are connected together in the same
 * uC, and receive timeout flag SSPMIS_RTMIS is tested, the main function must
 * be finished with while(1) {}, in other case the code will be executed and
 * finished before the timeout occurs.
 *
 * Also this is not possible to generate SPI1 IRQ before SPI0 buffer will
 * be sent.
 * It means if SPI0 send 7 bytes (there is 7 writes to FIFO buffer).
 * SPI1 will not generate IRQ before all 7 bytes income to receive FIFO.
 * There will be no 7 separate ISR call raised by SPI1.
 * TODO: check it again.
 *
 *
 * 2.
 * LPC1114 has no IRQ flag when single byte received at FIFO.
 * There is a receive timeout flag SSPMIS_RTMIS, set when byte is not read from
 * FIFO in a timeout defined time.
 * If the byte is received at the FIFO and timeout expires than ISR will
 * be called.
 * This flag can be used to call ISR with every byte received.
 *
 * If the byte was received, IRS called with SSPMIS_RTMIS set, this is not
 * enough to clear the flag inside of ISR routine. The byte must be read from
 * FIFO, if not ISR will be continuously called despite the flag was cleared.
 *
 *
 * SSPMIS_RXMIS - IRQ flag - FIFO is at least half full.
 *
 * This is set when >= 4 but < 9 bytes is received.
 * This is possible to push in one pass more than 4 bytes into FIFO, and IRQ
 * will be called after all bytes will be received.
 * It happens if SPI0 and SPI1 are connected together at the same uC.
 *
 * If only this flag is configured to rise up IRQ, than if FIFO is not read
 * inside of ISR, the IRQ will be called continuously with the flag set,
 * despite the flag is cleared every time inside of ISR.
 * If both SSPMIS_RXMIS and SSPMIS_RTMIS are active, and FIFO will be not
 * read inside of ISR, than ISR will be called continuously, despite of flags
 * are cleared every time inside of ISR.
 * The flags will set alternatively:
 *
 *    SSPMIS_RXMIS
 *    SSPMIS_RXMIS | SSPMIS_RTMIS
 *    SSPMIS_RXMIS
 *    SSPMIS_RXMIS | SSPMIS_RTMIS
 *
 *
 * With 9 bytes send from SPI0 to SPI1, and FIFO is not read:
 *
 * SSPMIS_RXMIS - if only received timeout flag is configured the ISR will
 * be continuously raised with this flag set.
 *
 * SSPMIS_RORMIS - if only overrun timeout flag is configured the ISR will
 * be (continuously - or one time ?) raised with this flag set.
 *
 * SSPMIS_RXMIS | SSPMIS_RTMIS | SSPMIS_RORMIS
 * if all FIFO received at least full, timeout, overrun flags are configured,
 * and FIFO will be not read, then ISR will be called with alternatively
 * SSPMIS_RXMIS and SSPMIS_RTMIS set as below. The flag Overrrun wil be not
 * set.
 *
 *    SSPMIS_RXMIS
 *    SSPMIS_RXMIS | SSPMIS_RTMIS
 *    SSPMIS_RXMIS
 *    SSPMIS_RXMIS | SSPMIS_RTMIS
 *
 * It seems to be FIFO at least full flag has highest priority.
 * Than Timeout flag, and Overrun in this case.
 *
 *
 * Changelog.
 * ==========
 *
 *
 * TODO:
 *
 */

//#include <LPC11xx.h>

#include "driver_config.h"
#include "target_config.h"

#include "gpio.h"
#include "ssp.h"
#include "spi_tests.h"
#include "mpl115a.h"

// n3310 lcd
#include "N3310LCD.h"
#include "mbed_bmp.h"



// menu starting points
#define MENU_X    10        // 0-83
#define MENU_Y    1        // 0-5

#define DEMO_ITEMS 4

// menu definition
char menu_items[DEMO_ITEMS][12] =
{
    "TEMPERATURE",
    "CHAR MAP",
    "BITMAP",
    "ABOUT"
};






/******************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void)
{
	boolean_t res = FALSE;
	double pressure;
	uint32_t i = 0;
	uint32_t k = 0;


    SystemInit();
    /* LED test output*/
    GPIOInit();    /* Set up clock */
    GPIOSetDir(LED_PORT, LED_BIT, 1);
    //GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);



#if 0
    MPL115AIntitalize();
    MPL115AReadCoeffs();
    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    // delay
    for (i = 0; i < 500000; i++)
    {
        k += 1;
    }
    MPL115AReadPressureAndTempADC();
//    MPL115ACalculatePressure(&pressure);
    while (1) {}
#endif



#if 0
    SPITESTS_Init();
    res = SPITESTS_LoopbackInternalTest(&SPITESTS_Dev);
    if (FALSE == res)
    {
    	GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    }
#endif


    // N3310 lcd tests
    N3310LCD_init();
    N3310LCD_cls();
    N3310LCD_backlight(1);





    //while (1) {};  // For tests, to wait until received timeout ISR will be generated.


    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);


    ///////////////////////////////////////////////////////////////////////
    // 3310 section - begin
    ///////////////////////////////////////////////////////////////////////



void temperature(N3310LCD* lcd)
{
    lcd->writeStringBig(5, 1, "+21.12", NORMAL);
    lcd->writeString(73, 2, "C", NORMAL);
}

void charmap(N3310LCD* lcd)
{
    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 14; j++)
        {
          lcd->locate(j*6, i);
          lcd->writeChar(i*14 + j + 32, NORMAL);
        }
    }
}

void bitmap(N3310LCD* lcd)
{
    lcd->drawBitmap(20, 1, mbed_bmp, 48, 24);
}

void about(N3310LCD* lcd)
{
    lcd->writeString(0, 1, "Nokia 3310 LCD", NORMAL);
    lcd->writeString(15, 2, "driven by", NORMAL);
    lcd->writeString(30, 3, "mbed", NORMAL);
}

void (*menu_funcs[DEMO_ITEMS])(N3310LCD*) =
{
    temperature,
    charmap,
    bitmap,
    about
};

void initMenu(N3310LCD* lcd)
{
    lcd->writeString(MENU_X, MENU_Y, menu_items[0], HIGHLIGHT );

    for (int i = 1; i < DEMO_ITEMS; i++)
    {
        lcd->writeString(MENU_X, MENU_Y + i, menu_items[i], NORMAL);
    }
}

void waitforOKKey(N3310LCD* lcd, Joystick* jstick)
{
    lcd->writeString(38, 5, "OK", HIGHLIGHT );

    int key = 0xFF;
    while (key != CENTER_KEY)
    {
        for (int i = 0; i < NUM_KEYS; i++)
        {
            if (jstick->getKeyState(i) !=0)
            {
                jstick->resetKeyState(i);  // reset
                if (CENTER_KEY == i) key = CENTER_KEY;
            }
        }
    }
}

void autoDemo(N3310LCD* lcd)
{
    while (true)
    {
        for (int i = 0; i < DEMO_ITEMS; i++)
        {
            lcd->cls();
            lcd->backlight(ON);
            wait(1);

            (*menu_funcs[i])(lcd);

            wait(3);

            lcd->backlight(OFF);
            wait(3);
        }
    }
}

int main()
{
    Joystick jstick(N3310SPIPort::AD0);
    N3310LCD lcd(N3310SPIPort::MOSI, N3310SPIPort::MISO, N3310SPIPort::SCK,
                 N3310SPIPort::CE, N3310SPIPort::DAT_CMD, N3310SPIPort::LCD_RST,
                 N3310SPIPort::BL_ON);
    lcd.init();
    lcd.cls();
    lcd.backlight(ON);

    // demo stuff
    // autoDemo(&lcd);

    initMenu(&lcd);
    int currentMenuItem = 0;
    Ticker jstickPoll;
    jstickPoll.attach(&jstick, &Joystick::updateADCKey, 0.01);    // check ever 10ms


    while (true)
    {
    for (int i = 0; i < NUM_KEYS; i++)
    {
        if (jstick.getKeyState(i) != 0)
        {
            jstick.resetKeyState(i);  // reset button flag
            switch(i)
            {
                case UP_KEY:
                    // current item to normal display
                    lcd.writeString(MENU_X, MENU_Y + currentMenuItem, menu_items[currentMenuItem], NORMAL);
                    currentMenuItem -=1;
                    if (currentMenuItem <0)  currentMenuItem = DEMO_ITEMS -1;
                    // next item to highlight display
                    lcd.writeString(MENU_X, MENU_Y + currentMenuItem, menu_items[currentMenuItem], HIGHLIGHT);
                    break;

                case DOWN_KEY:
                    // current item to normal display
                    lcd.writeString(MENU_X, MENU_Y + currentMenuItem, menu_items[currentMenuItem], NORMAL);
                    currentMenuItem +=1;
                    if(currentMenuItem >(DEMO_ITEMS - 1))  currentMenuItem = 0;
                    // next item to highlight display
                    lcd.writeString(MENU_X, MENU_Y + currentMenuItem, menu_items[currentMenuItem], HIGHLIGHT);
                    break;

                case LEFT_KEY:
                    initMenu(&lcd);
                    currentMenuItem = 0;
                    break;

                case RIGHT_KEY:
                    lcd.cls();
                    (*menu_funcs[currentMenuItem])(&lcd);
                    waitforOKKey(&lcd, &jstick);
                    lcd.cls();
                    initMenu(&lcd);
                    currentMenuItem = 0;
                    break;
            }
        }
    }
    }

    return EXIT_SUCCESS;

    ///////////////////////////////////////////////////////////////////////
    // 3310 section - end off
    ///////////////////////////////////////////////////////////////////////




    while (1) {}

    return 0;
}

/******************************************************************************
**                            End Of File
******************************************************************************/

