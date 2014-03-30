/*
 * spi_tests.c

 *
 * rev. 140111_0916
 *
 *
 * Test conditions:
 * ================
 *
 * SPI:
 *
 * SPITESTS_LoopbackInternalTest() is used for internal loopback test.
 * SPI0 or SPI1 can be tested on LPCExpresso LPC1114 board.
 *
 * SPI0 and SPI1 are used connected together for base hardware interconnection
 * test - SPITESTS_LoopbackHardwareISPTest().
 * SPI0 and SPI1 are available on LPCExpresso LPC1114 board.
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
 *
 *
 * TODO:
 *
 */


#include "driver_config.h"  //todo describe this include MUST BE, and if SSP driver will be not ON in there,
                            // than all the ssp.h and .c file will be not add to compile, so during compilation
                            // of this file, there will be error. This approach is very buggy.
#include "gpio.h"
#include "ssp.h"
#include "spi_tests.h"


#define SSP_BUFSIZE    8
#define SSP_FIFOSIZE    8

static uint8_t src_addr[SSP_BUFSIZE];
static uint8_t dest_addr[SSP_BUFSIZE];

// Replace both with something better finally
static uint8_t pseudo_mutex;
static uint8_t buffIdx;

void SPITESTS_SPI1DataProcessingISP(uint8_t val)
{
    if (buffIdx < SSP_BUFSIZE)
    {
    	dest_addr[buffIdx++] = val;
    }
    else
    {
    	pseudo_mutex = 1;
    }

    return;
}




/*****************************************************************************
** Function name:		SPITESTS_LoopbackInternalTest
**
** Descriptions:		Internal Loopback test. The data are send and received
**                      by the internal SSP logic.
**
**                      Please define at ssp.h:
**
**                      Example for SPI0:
**
**                      SSP_LOOPBACK_MODE0  1
**                      SSP_SLAVE0          0
**                      SSP_TX_RX_ONLY0     0
**                      SSP_USE_CS0         1
**
**
** Parameters:			SPI port number
**
** Returned value:		TRUE is test passed, FALSE if not.
**
*****************************************************************************/
boolean_t SPITESTS_LoopbackInternalTest(uint8_t portNum)
{
    boolean_t res = TRUE;
    uint8_t i = 0;

    //todo
    // why USE_CS, FIFOSIZE, BUFFSIZE, what about IRS

    SSP_IOConfig(portNum);
    SSP_Init(portNum);
    for (i = 0; i < SSP_BUFSIZE; i++)
    {
  	    src_addr[i] = (uint8_t)i;
  	    dest_addr[i] = 0;
    }


    /* Set SPI0 SSEL pin to output low. */
    if ( SPI0 == portNum )
    {  //todo is it not automatically set by SSP periph
#if !SSP_USE_CS
        GPIOSetValue(PORT0, 2, 0);
#endif
    }
    else if ( SPI1 == portNum )
    {
#if !SSP_USE_CS
	    GPIOSetValue(PORT2, 0, 0);
#endif
    }
    else
    {}


    i = 0;
    while (i < SSP_BUFSIZE)
    {
  	    /* to check the RXIM and TXIM interrupt, I send a block data at
  	     * one time based on the FIFOSIZE(8).
  	     */
        SSP_Send( portNum, (uint8_t *)&src_addr[i], SSP_FIFOSIZE );

  	    /* If RX interrupt is enabled, below receive routine can be
  	     * also handled inside the ISR.
  	     */
  	    SSP_Receive( portNum, (uint8_t *)&dest_addr[i], SSP_FIFOSIZE );

  	    i += SSP_FIFOSIZE;
    }


    /* Set SSEL pin to output high. */
    if ( SPI0 == portNum )
    {
#if !SSP_USE_CS
        GPIOSetValue( PORT0, 2, 1 );
#endif
    }
    else if ( SPI1 == portNum )
    {
#if !SSP_USE_CS
        GPIOSetValue( PORT2, 0, 1 );
#endif
    }
    else
    {}


    for (i = 0; i < SSP_BUFSIZE; i++)
    {
        if (src_addr[i] != dest_addr[i])
        {
            res = FALSE;
            break;
        }
    }


	return res;
}


/*****************************************************************************
** Function name:		SPITESTS_LoopbackHardwarePoolingTest
**
** Descriptions:		Loopback test between hardwired SPI0 (master)
**                      and SPI1 (slave).
**                      This is not possible in an easy and clear way to send
**                      and received more than FIFO length data, without ISP.
**                      Therefore the 8 bytes of data is send by SPI0, and
**                      received by SPI1.
**
**                      Pinout on LPCExpresso LPC1114 board:
**
**                                  uC pin    PCB
**
**                      SPI MOSI0 - PIO0_9  - (MOSI/SWO) PIO0_9 J6-5
**                      SPI MISO0 - PIO0_8  - (MISO) PIO0_8 J6-6
**                      SPI SCK0  - PIO2_11 - (SCK) PIO2_11 J6-7
**                      SPI SSEL0 - PIO0_2  - (SSEL0) PIO0_2 J6-8
**
**                      SPI MOSI1 - PIO2_3  - PIO2_3 J6-45
**                      SPI MISO1 - PIO2_2  - PIO2_2 J6-14
**                      SPI SCK1  - PIO2_1  - PIO2_1 J16-13
**                      SPI SSEL1 - PIO2_0  - PIO2_0 J6-12
**
**
**                      Please define at ssp.h:
**
**                      SSP_LOOPBACK_MODE0  0
**                      SSP_SLAVE0          0 - SPI0 Master
**                      SSP_TX_RX_ONLY0     1
**                      SSP_USE_CS0         1
**
**                      SSP_LOOPBACK_MODE1  0
**                      SSP_SLAVE1          1 - SPI1 Slave
**                      SSP_TX_RX_ONLY1     1
**                      SSP_USE_CS1         1
**
**
** Parameters:			None
**
** Returned value:      TRUE is test passed, FALSE if not.
**
*****************************************************************************/
boolean_t SPITESTS_LoopbackHardwarePoolingTest(void)
{
	boolean_t res = TRUE;
    uint8_t i = 0;

    SSP_IOConfig(SPI0);
    SSP_Init(SPI0);

    SSP_IOConfig(SPI1);
    SSP_Init(SPI1);

    for (i = 0; i < SSP_FIFOSIZE; i++)
    {
  	    src_addr[i] = (uint8_t)i;
  	    dest_addr[i] = 0;
    }

    SSP_Send(SPI0, (uint8_t *)src_addr, SSP_FIFOSIZE);
    SSP_Receive(SPI1, (uint8_t *)dest_addr, SSP_FIFOSIZE);

    for (i = 0; i < SSP_FIFOSIZE; i++)
    {
  	    if (src_addr[i] != dest_addr[i])
  	    {
            res = FALSE;
            break;
  	    }
    }

	return res;
}


/*****************************************************************************
** Function name:		SPITESTS_LoopbackHardwareISPTest
**
** Descriptions:		Loopback test between hardwired SPI0 (master)
**                      and SPI1 (slave).
**                      The data are received by routine called by ISR when
**                      receive a byte of data. Please see notes above.
**
**                      Pinout on LPCExpresso LPC1114 board:
**
**                                  uC pin    PCB
**
**                      SPI MOSI0 - PIO0_9  - (MOSI/SWO) PIO0_9 J6-5
**                      SPI MISO0 - PIO0_8  - (MISO) PIO0_8 J6-6
**                      SPI SCK0  - PIO2_11 - (SCK) PIO2_11 J6-7
**                      SPI SSEL0 - PIO0_2  - (SSEL0) PIO0_2 J6-8
**
**                      SPI MOSI1 - PIO2_3  - PIO2_3 J6-45
**                      SPI MISO1 - PIO2_2  - PIO2_2 J6-14
**                      SPI SCK1  - PIO2_1  - PIO2_1 J16-13
**                      SPI SSEL1 - PIO2_0  - PIO2_0 J6-12
**
**
**                      Please define at ssp.h:
**
**                      SSP_LOOPBACK_MODE0  0
**                      SSP_SLAVE0          0 - SPI0 Master
**                      SSP_TX_RX_ONLY0     1
**                      SSP_USE_CS0         1
**
**                      SSP_LOOPBACK_MODE1  0
**                      SSP_SLAVE1          1 - SPI1 Slave
**                      SSP_TX_RX_ONLY1     1
**                      SSP_USE_CS1         1
**
**
** Parameters:			None
**
** Returned value:      TRUE is test passed, FALSE if not.
**
*****************************************************************************/
boolean_t SPITESTS_LoopbackHardwareISPTest(void)
{
	boolean_t res = TRUE;
    uint16_t i = 0;

    SSP_IOConfig(SPI0);
    SSP_Init(SPI0);

    SSP_IOConfig(SPI1);
    SSP_Init(SPI1);

    for (i = 0; i < SSP_BUFSIZE; i++)
    {
  	    src_addr[i] = (uint8_t)(i + 1);
  	    dest_addr[i] = 0;
    }
    pseudo_mutex = 0;
    buffIdx = 0;

    SSP_Send(SPI0, (uint8_t *)src_addr, SSP_BUFSIZE);

    // Wait until all data are received by SPI1 ISR
    //while (0 == pseudo_mutex) {};
    for (i = 0; i < SSP_BUFSIZE; i++)
    {
  	    if (src_addr[i] != dest_addr[i])
  	    {
            res = FALSE;
            break;
  	    }
    }

	return res;
}
