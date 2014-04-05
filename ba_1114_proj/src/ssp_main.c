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


#include "driver_config.h"
#include "target_config.h"

#include "gpio.h"
#include "ssp.h"
#include "spi_tests.h"




/******************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void)
{
	boolean_t res = FALSE;


    SystemInit();
    /* LED test output*/
    GPIOInit();    /* Set up clock */
    GPIOSetDir(LED_PORT, LED_BIT, 1);
    //GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);

#if 0
    res = SPITESTS_LoopbackInternalTest(SPI1);
    if (FALSE == res)
    {
    	GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    }
#endif




    //while (1) {};  // For tests, to wait until received timeout ISR will be generated.


    return 0;
}

/******************************************************************************
**                            End Of File
******************************************************************************/

