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

#include "timer16.h"
#include "gpio.h"
#include "ssp.h"
#include "spi_tests.h"
#include "mpl115a.h"

#include "st7565.h"
#include "graphics.h"
#include "hellombed.h"




/******************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void)
{
	//boolean_t res = FALSE;
	//double pressure;
	//uint32_t i = 0;
	//uint32_t k = 0;


    SystemInit();
    /* LED test output*/
    GPIOInit();    /* Set up clock */
    GPIOSetDir(LED_PORT, LED_BIT, 1);
    //GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);


// Pressure sensor test
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


// Loopback test
#if 0
    SPITESTS_Init();
    res = SPITESTS_LoopbackInternalTest(&SPITESTS_Dev);
    if (FALSE == res)
    {
    	GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    }
#endif

    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);


    //while (1) {};  // For tests, to wait until received timeout ISR will be generated.

    // Display test

    ST7565_init();
    // draw "hello mbed"
    ST7565_send_pic(pic_hellombed);

    //wait(5);
    //delayMs(uint8_t timer_num, uint32_t delayInMs)
    delayMs(0, 5000);

    // draw rectangle around the screen
    GRAPH_line(0, 0, ST7565_getWidth()-1, 0, 0xFFFFFF);

    //wait(2);
    delayMs(0, 2000);

    GRAPH_line(ST7565_getWidth()-1, 0, ST7565_getWidth()-1, ST7565_getHeight()-1, 0xFFFFFF);

    //wait(2);
    delayMs(0, 2000);

    GRAPH_line(ST7565_getWidth()-1, ST7565_getHeight()-1, 0, ST7565_getHeight()-1, 0xFFFFFF);

    //wait(2);
    delayMs(0, 2000);

    GRAPH_line(0, ST7565_getHeight()-1, 0, 0, 0xFFFFFF);

    //wait(5);
    delayMs(0, 5000);

    ST7565_clear_screen();


    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);

/*
    float rotx = 0, roty = 0, rotz = 0;

    Timer timer;
    timer.start();
    int frameno = 0;
    const int pollcount = 10;
    // shift 1/4th of screen to the left
    tf.position(-dog.width() / 4, 0, 0);
    tf.colour(0xffffff);
    // shift 1/4th of screen to the right
    cube.position(+dog.width() / 4, 0, 0);
    cube.colour(0xffffff);
    while (1)
    {
        rotx += 0.1;
        roty += 0.08;
        rotz += 0.05;

        // set rotation angles
        tf.rotate(rotx, roty, rotz);
        cube.rotate(rotx, roty, rotz);
        // lock update
        dog.beginupdate();
            dog.clear_screen();
            // render TieFighter
            tf.render(g);
            // and the cube
            cube.render(g);
        // unlock update (and draw framebuffer)
        dog.endupdate();
        if ( ++frameno == pollcount )
        {
            // output fps to serial
            int end = timer.read_ms();
            float fps = pollcount*1000.0/end;
            printf("\r%d frames, %d ms, FPS: %f", pollcount, end, fps);
            frameno = 0;
            timer.reset();
        }
        //dog.fill(40, 40, 52, 52, 0x000000);
    }
*/

    while (1) {}

    return 0;
}

/******************************************************************************
**                            End Of File
******************************************************************************/

