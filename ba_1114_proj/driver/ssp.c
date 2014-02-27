/****************************************************************************
 *   $Id:: ssp.c 4785 2010-09-03 22:39:27Z nxp21346                         $


 *   Project: NXP LPC11xx SSP example
 *
 *   Description:
 *     This file contains SSP code example which include SSP 
 *     initialization, SSP interrupt handler, and APIs for SSP
 *     reading.
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

/****************************************************************************
 *
 * Arkadiusz Liberda 2013
 *
 * Additional info and updates.
 * ----------------------------
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
 */



#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_SSP==1  //todo: check is it really still need if we manage linked obj at eclipse source dir.
                                 // No - this is not need. Remove in all driver files.
#include "gpio.h"
#include "ssp.h"
#include "spi_tests.h"


/* statistics of all the interrupts */
volatile uint32_t interruptRxStat0 = 0;
volatile uint32_t interruptOverRunStat0 = 0;
volatile uint32_t interruptRxTimeoutStat0 = 0;

volatile uint32_t interruptRxStat1 = 0;
volatile uint32_t interruptOverRunStat1 = 0;
volatile uint32_t interruptRxTimeoutStat1 = 0;

/*****************************************************************************
** Function name:	SSP0_IRQHandler
**
** Descriptions:	SSP port is used for SPI communication.
**					SSP interrupt handler
**					The algorithm is, if RXFIFO is at least half full,
**					start receive until it's empty; if TXFIFO is at least
**					half empty, start transmit until it's full.
**					This will maximize the use of both FIFOs and performance.
**
**                  Remember to configure active flags at SSP_Init().
**                  LPC_SSP1->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM | SSPIMSC_RXIM;
**
** parameters:		None
** Returned value:	None
** 
*****************************************************************************/
void SSP0_IRQHandler(void) 
{
    uint32_t regValue;

    regValue = LPC_SSP0->MIS;
    if ( regValue & SSPMIS_RORMIS )	/* Receive overrun interrupt */
    {
	    interruptOverRunStat0++;
	    LPC_SSP0->ICR = SSPICR_RORIC;	/* clear interrupt */
    }
    if ( regValue & SSPMIS_RTMIS )	/* Receive timeout interrupt */
    {
	    interruptRxTimeoutStat0++;
	    LPC_SSP0->ICR = SSPICR_RTIC;	/* clear interrupt */
    }

    /* please be aware that, in main and ISR, CurrentRxIndex and CurrentTxIndex
    are shared as global variables. It may create some race condition that main
    and ISR manipulate these variables at the same time. SSPSR_BSY checking (polling)
    in both main and ISR could prevent this kind of race condition */
    if ( regValue & SSPMIS_RXMIS )	/* Rx at least half full */
    {
	    interruptRxStat0++;		/* receive until it's empty */
    }
    return;
}

/*****************************************************************************
** Function name:	SSP1_IRQHandler
**
** Descriptions:	SSP port is used for SPI communication.
**					SSP interrupt handler
**					The algorithm is, if RXFIFO is at least half full,
**					start receive until it's empty; if TXFIFO is at least
**					half empty, start transmit until it's full.
**					This will maximize the use of both FIFOs and performance.
**
**                  Remember to configure active flags at SSP_Init().
**                  LPC_SSP1->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM | SSPIMSC_RXIM;
**
** parameters:		None
** Returned value:	None
** 
*****************************************************************************/
void SSP1_IRQHandler(void) 
{
  uint32_t regValue;


  regValue = LPC_SSP1->MIS;


  if ( regValue & SSPMIS_RORMIS )	/* Receive overrun interrupt */
  {
	interruptOverRunStat1++;
	LPC_SSP1->ICR = SSPICR_RORIC;	/* clear interrupt */
  }


  if ( regValue & SSPMIS_RTMIS )	/* Receive timeout interrupt */
  {
	interruptRxTimeoutStat1++;
	LPC_SSP1->ICR = SSPICR_RTIC;	/* clear interrupt */

	//arek tests
#if 0
    while (LPC_SSP1->SR & SSPSR_RNE)
    {
        SPITESTS_SPI1DataProcessingISP((uint8_t)(LPC_SSP1->DR));
    }
#endif

  }


#if 0
  /* please be aware that, in main and ISR, CurrentRxIndex and CurrentTxIndex
  are shared as global variables. It may create some race condition that main
  and ISR manipulate these variables at the same time. SSPSR_BSY checking (polling)
  in both main and ISR could prevent this kind of race condition */
  if ( regValue & SSPMIS_RXMIS )	/* Rx at least half full */
  {
	interruptRxStat1++;		/* receive until it's empty */

    while (LPC_SSP1->SR & SSPSR_RNE)
    {
    	SPITESTS_SPI1DataProcessingISP((uint8_t)(LPC_SSP1->DR));
    }

  }
#endif

  return;
}

/*****************************************************************************
** Function name:		SSP_IOConfig
**
** Descriptions:		SSP port initialization routine
**				
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void SSP_IOConfig( uint8_t portNum )
{
    if (portNum == 0)
    {
    	// SSP0
    	// SPI MOSI0 - PIO0_9  - (MOSI/SWO) PIO0_9 J6-5
    	// SPI MISO0 - PIO0_8  - (MISO) PIO0_8 J6-6
    	// SPI SCK0  - PIO2_11 - (SCK) PIO2_11 J6-7
    	// SPI SSEL0 - PIO0_2  - (SSEL0) PIO0_2 J6-8

        LPC_SYSCON->PRESETCTRL |= (0x1<<0);
        LPC_SYSCON->SYSAHBCLKCTRL |= (0x1<<11);
        LPC_SYSCON->SSP0CLKDIV = 0x02;  /* Divided by 2 */
        LPC_IOCON->PIO0_8 &= ~0x07;     /* SSP I/O config */
        LPC_IOCON->PIO0_8 |= 0x01;		/* MISO0 */
        LPC_IOCON->PIO0_9 &= ~0x07;
        LPC_IOCON->PIO0_9 |= 0x01;		/* MOSI0 */

#ifdef __JTAG_DISABLED
        LPC_IOCON->SCK_LOC = 0x00;
        LPC_IOCON->SWCLK_PIO0_10 &= ~0x07;
        LPC_IOCON->SWCLK_PIO0_10 |= 0x02;    /* SCK0 */
#else

        // This is original, and wrong on LPC1114 LPCExpresso board
        //#if 0
        ///* On HummingBird/Candiru 1(HB1/CD1), SSP CLK can be routed to different
        //pins, other than JTAG TCK, it's either P2.11 func. 1 or P0.6 func. 2. */
        //LPC_IOCON->SCK_LOC = 0x01;
        //LPC_IOCON->PIO2_11 = 0x01;	/* P2.11 function 1 is SSP clock, need to
        //							combined with IOCONSCKLOC register setting */
        //#else
        //LPC_IOCON->SCK_LOC = 0x02;
        //LPC_IOCON->PIO0_6 = 0x02;	/* P0.6 function 2 is SSP clock, need to
        //							combined with IOCONSCKLOC register setting */


        // LPCExpresso LPC114 use hardwired PIO2_11 as SCK0
        LPC_IOCON->SCK_LOC = 0x01;
        LPC_IOCON->PIO2_11 = 0x01;

        // Optionally with SWD you can use SCK0 on PIO0_6
        //LPC_IOCON->SCK_LOC = 0x02;
        //LPC_IOCON->PIO0_6 = 0x02;

#endif	/* endif __JTAG_DISABLED */  


#if SSP_USE_CS0
        LPC_IOCON->PIO0_2 &= ~0x07;
        LPC_IOCON->PIO0_2 |= 0x01;		/* SSEL0 */
#else
        /* Enable AHB clock to the GPIO domain. */
        LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

        LPC_IOCON->PIO0_2 &= ~0x07;		/* SSEL0 is a GPIO pin */
        /* port0, bit 2 is set to GPIO output and high */
        GPIOSetDir( PORT0, 2, 1 );
        GPIOSetValue( PORT0, 2, 1 );
#endif

    }
    else
    {
	    // SSP1
    	//
    	// SPI MOSI1 - PIO2_3  - PIO2_3 J6-45
    	// SPI MISO1 - PIO2_2  - PIO2_2 J6-14
    	// SPI SCK1  - PIO2_1  - PIO2_1 J16-13
    	// SPI SSEL1 - PIO2_0  - PIO2_0 J6-12

        LPC_SYSCON->PRESETCTRL |= (0x1<<2);
        LPC_SYSCON->SYSAHBCLKCTRL |= (1<<18);
        LPC_SYSCON->SSP1CLKDIV = 0x02; /* Divided by 2 */
        LPC_IOCON->PIO2_2 &= ~0x07;    /* SSP I/O config */
        LPC_IOCON->PIO2_2 |= 0x02;     /* MISO1 */
        LPC_IOCON->PIO2_3 &= ~0x07;
        LPC_IOCON->PIO2_3 |= 0x02;     /* MOSI1 */
        LPC_IOCON->PIO2_1 &= ~0x07;
        LPC_IOCON->PIO2_1 |= 0x02;     /* SCLK1 */
 
#if SSP_USE_CS1
        LPC_IOCON->PIO2_0 &= ~0x07;
        LPC_IOCON->PIO2_0 |= 0x02;		/* SSEL1 */
#else
        /* Enable AHB clock to the GPIO domain. */
        LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);
		
        LPC_IOCON->PIO2_0 &= ~0x07;		/* SSP SSEL is a GPIO pin */
        /* port2, bit 0 is set to GPIO output and high */
        GPIOSetDir( PORT2, 0, 1 );
        GPIOSetValue( PORT2, 0, 1 );
#endif

    }

    return;
}

/*****************************************************************************
** Function name:		SSP_Init
**
** Descriptions:		SSP port initialization routine
**				
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void SSP_Init( uint8_t portNum )
{
    uint8_t i, Dummy=Dummy;

    if ( portNum == 0 )
    {
        // SSP0

        /* Set DSS data to 8-bit, Frame format SPI, CPOL = 0, CPHA = 0, and SCR is 15 */
        LPC_SSP0->CR0 = 0x0707;

        /* SSPCPSR clock prescale register, master mode, minimum divisor is 0x02 */
        LPC_SSP0->CPSR = 0x2;

        for ( i = 0; i < SSP_FIFOSIZE; i++ )
        {
            Dummy = LPC_SSP0->DR;		/* clear the RxFIFO */
        }

        /* Enable the SSP Interrupt */
        NVIC_EnableIRQ(SSP0_IRQn);
	
        /* Device select as master, SSP Enabled */
#if SSP_LOOPBACK_MODE0
        LPC_SSP0->CR1 = SSPCR1_LBM | SSPCR1_SSE;
#else
#if SSP_SLAVE0
        /* Slave mode */
        if ( LPC_SSP0->CR1 & SSPCR1_SSE )
        {
            /* The slave bit can't be set until SSE bit is zero. */
	        LPC_SSP0->CR1 &= ~SSPCR1_SSE;
        }
        LPC_SSP0->CR1 = SSPCR1_MS;		/* Enable slave bit first */
        LPC_SSP0->CR1 |= SSPCR1_SSE;	/* Enable SSP */
#else
            /* Master mode */
        LPC_SSP0->CR1 = SSPCR1_SSE;
#endif
#endif

        /* Set SSPINMS registers to enable interrupts
         * enable all error related interrupts
         */
        //LPC_SSP0->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM;
        LPC_SSP0->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM | SSPIMSC_RXIM; //todo maybe before you switch on isr ?
    }
    else
    {

	    //SSP1

        /* Set DSS data to 8-bit, Frame format SPI, CPOL = 0, CPHA = 0, and SCR is 15 */
        LPC_SSP1->CR0 = 0x0707;

        /* SSPCPSR clock prescale register, master mode, minimum divisor is 0x02 */
        LPC_SSP1->CPSR = 0x2;

        for ( i = 0; i < SSP_FIFOSIZE; i++ )
        {
	        Dummy = LPC_SSP1->DR;		/* clear the RxFIFO */
        }

        /* Enable the SSP Interrupt */
        NVIC_EnableIRQ(SSP1_IRQn);
	
        /* Device select as master, SSP Enabled */
#if SSP_LOOPBACK_MODE1
        LPC_SSP1->CR1 = SSPCR1_LBM | SSPCR1_SSE;
#else
#if SSP_SLAVE1
        /* Slave mode */
        if ( LPC_SSP1->CR1 & SSPCR1_SSE )
        {
	        /* The slave bit can't be set until SSE bit is zero. */
	        LPC_SSP1->CR1 &= ~SSPCR1_SSE;
        }
        LPC_SSP1->CR1 = SSPCR1_MS;		/* Enable slave bit first */
        LPC_SSP1->CR1 |= SSPCR1_SSE;	/* Enable SSP */
#else
        /* Master mode */
        LPC_SSP1->CR1 = SSPCR1_SSE;
#endif
#endif

        /* Set SSPINMS registers to enable interrupts */
        /* enable all error related interrupts */
        //LPC_SSP1->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM;


        LPC_SSP1->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM | SSPIMSC_RXIM;
        //LPC_SSP1->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM;  //arek temp
        //LPC_SSP1->IMSC = SSPIMSC_RTIM; // timeout
        //LPC_SSP1->IMSC = SSPIMSC_RORIM;  // overrun
        //LPC_SSP1->IMSC = SSPIMSC_RXIM;

    }

    return;
}

/*****************************************************************************
** Function name:		SSP_Send
**
** Descriptions:		Send a block of data to the SSP port, the 
**						first parameter is the buffer pointer, the 2nd 
**						parameter is the block length.
**
** parameters:			port #, buffer pointer, and the block length
** Returned value:		None
** 
*****************************************************************************/
void SSP_Send(uint8_t portNum, uint8_t *buf, uint32_t Length)
{
    uint32_t i;
    uint8_t Dummy = Dummy;
    
    for (i = 0; i < Length; i++)
    {
        if (portNum == 0)
        {
            // SSP0

            /* Move on only if NOT busy and TX FIFO not full. */
	        while ( (LPC_SSP0->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF );
	        LPC_SSP0->DR = *buf;
            buf++;
#if !SSP_LOOPBACK_MODE0
	        while ( (LPC_SSP0->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
            /* Whenever a byte is written, MISO FIFO counter increments, Clear FIFO
             * on MISO. Otherwise, when SSP0Receive() is called, previous data byte
             * is left in the FIFO.
             */
	        Dummy = LPC_SSP0->DR;
#else
            /* Wait until the Busy bit is cleared. */
	        while ( LPC_SSP0->SR & SSPSR_BSY );
#endif
        }
        else
        {
            // SSP1

	        /* Move on only if NOT busy and TX FIFO not full. */
	        while ( (LPC_SSP1->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF );
	        LPC_SSP1->DR = *buf;
	        buf++;
#if !SSP_LOOPBACK_MODE1
	        while ( (LPC_SSP1->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
	        /* Whenever a byte is written, MISO FIFO counter increments, Clear FIFO
	         * on MISO. Otherwise, when SSP0Receive() is called, previous data byte
	         * is left in the FIFO.
	         */
            Dummy = LPC_SSP1->DR;
#else
	        /* Wait until the Busy bit is cleared. */
	        while ( LPC_SSP1->SR & SSPSR_BSY );
#endif
        }
    }

    return;
}

/*****************************************************************************
** Function name:		SSP_Receive
** Descriptions:		the module will receive a block of data from 
**						the SSP, the 2nd parameter is the block 
**						length.
** parameters:			port #, buffer pointer, and block length
** Returned value:		None
** 
*****************************************************************************/
void SSP_Receive(uint8_t portNum, uint8_t *buf, uint32_t Length)
{
    uint32_t i;
 
    for (i = 0; i < Length; i++)
    {
        /* As long as Receive FIFO is not empty, I can always receive.
	     * If it's a loopback test, clock is shared for both TX and RX,
	     * no need to write dummy byte to get clock to get the data
	     * if it's a peer-to-peer communication, SSPDR needs to be written
	     * before a read can take place.
	     */

        if (portNum == 0)
        {
            // SSP0

#if !SSP_LOOPBACK_MODE0

#if SSP_SLAVE0
	        while ( !(LPC_SSP0->SR & SSPSR_RNE) );
#else
	        LPC_SSP0->DR = 0xFF;
	        /* Wait until the Busy bit is cleared */
	        while ( (LPC_SSP0->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
#endif

#else
	        while ( !(LPC_SSP0->SR & SSPSR_RNE) );
#endif
	        *buf = LPC_SSP0->DR;
	        buf++;
        }
        else
        {
            // SSP1

#if !SSP_LOOPBACK_MODE1
#if SSP_SLAVE1
	        while ( !(LPC_SSP1->SR & SSPSR_RNE) );
#else
	        LPC_SSP1->DR = 0xFF;
	        /* Wait until the Busy bit is cleared */
	        while ( (LPC_SSP1->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
#endif
#else
	        while ( !(LPC_SSP1->SR & SSPSR_RNE) );
#endif
	        *buf = LPC_SSP1->DR;
            buf++;
        }
    }

    return;
}
#endif



/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/* SSP Status register */
#define SSPSR_TFE       ((uint32_t)(0x1<<0))
#define SSPSR_TNF       ((uint32_t)(0x1<<1))
#define SSPSR_RNE       ((uint32_t)(0x1<<2))
#define SSPSR_RFF       ((uint32_t)(0x1<<3))
#define SSPSR_BSY       ((uint32_t)(0x1<<4))


#define FIFOSIZE		8

#define SSP_ENABLE       ((uint32_t)(1<<1))
#define SSP_DISABLE      // todo disable ((uint32_t)(1<<3))




void SSP_Init(SSP_Dev_t *SSP_Dev)
{
	uint32_t regVal = 0;
	uint32_t regSRCVal = 0;

	regVal |= SSP_Dev->DataSize | SSP_Dev->FrameFormat | SSP_Dev->CPOL | SSP_Dev->CPHA;
	regSRCVal = GetSerialClockRate(SSP_Dev->ClockRateHz);
	regVal |= regSRCVal; //todo move it to right position.
	SSP_Dev->Device->CR0 = regVal;

	regVal = 0;
	regVal |= SSP_Dev->LoopBackMode | SSP_Dev->Mode | SSP_Dev->SlaveOutputDisable;
	SSP_Dev->Device->CR1 = regVal;

	regVal = GetClockPrescaleFactor(SSP_Dev->ClockRateHz);
	SSP_Dev->Device->CPSR = regVal;

	/* Set Interrupt Mask Register */
	regVal = 0;
	regval = SSP_Dev->InterruptCondition;
	SSP_Dev->Device->IMSC;

	/* clear the RxFIFO */
    for (i = 0; i < FIFOSIZE; i++)
    {
    	regVal = SSP_Dev->Device->DR;
    }

    //todo: enable interrupts at
    //NVIC_EnableIRQ(SSP0_IRQn);


    //todo: enable power at SYSAHBCLKCTRL bit 11 and 18

    //todo: enable peripherial clock SSP0/1CLKDIV

    //todo: reset - SSP_RST_N bits (0 and 2) in PRESETCTRL are set to 1.


    /* Enable SSP */
	SSP_Dev->Device->CR0 |= SSP_ENABLE;

}


void SSP_DeInit(SSP_Dev_t *SSP_Dev)
{
    /* Disble SSP */
	SSP_Dev->Device->CR0 |= SSP_DISABLE;


	//todo: disable power at SYSAHBCLKCTRL bit 11 and 18

    //todo: disable peripherial clock SSP0/1CLKDIV

    //todo: set - SSP_RST_N bits (0 and 2) in PRESETCTRL are set to 0.


}


void SSP_ConfigUpdate(SSP_Dev_t *SSP_Dev)
{

	//todo disable SPI






	//todo enable SPI

}


//todo below see Status Register and Raw Interrupt Status Register


void SSP_Send(SSP_Dev_t *SSP_Dev, uint8_t *buff, uint32_t len)
{
	uint32_t tmp = 0;
	uint8_t *buffIdx = buff;


	while (len--)
	{
		// Wait until Tx FIFO not full - there is a room for one frame.
		// TNF = 0 -> FIFO full
	    while ( !(SSP_Dev->Device->SR & SSPSR_TNF) );
        SSP_Dev->Device->SR = *(buffIdx++);
	}

    // Clear RxFIFO buffer
    while ( SSP_Dev->Device->SR & SSPSR_RNE )
    {
    	tmp = SSP_Dev->Device->SR;
    }

}


int32_t SSP_RecvBlock(SSP_Dev_t *SSP_Dev, uint8_t *buff, uint32_t len)
{
	int32_t ret = 0;
	uint8_t *buffIdx = buff;


	//todo wrong you have to wait until this is not empty and than go
	// check this kind of bug in other places.

    // If RxFIFO is not empty
	while (len != 0 &&  SSP_Dev->Device->SR & SSPSR_RNE)
	{
		*(buffIdx++) = SSP_Dev->Device->SR;
	}

	return ret;
}


//todo: add description
// rxBuff must be not shorter than txBuff.
//
int32_t SSP_SendRecvBlock(SSP_Dev_t *SSP_Dev, uint8_t *txBuff, uint32_t txLen, uint8_t *rxBuff)
{
	int32_t ret = 0;
	uint32_t tmp = 0;
	uint8_t *txBuffIdx = txBuff;
	uint8_t *rxBuffIdx = rxBuff;


    //todo: Improve. There is no check for timeout
	// Wait until Tx FIFO empty and SSP not busy.
    while ( SSP_Dev->Device->SR & (SSPSR_TFE | SSPSR_BSY) );

    // Read Rx FIFO until empty.
    while ( SSP_Dev->Device->SR & SSPSR_RNE )
    {
    	tmp = SSP_Dev->Device->SR;
    }

    while (txLen--)
    {
    	// Is TxFIFO full.
    	// Tx FIFO full -> SSPSR_TNF = 0
    	if ( !(SSP_Dev->Device->SR & SSPSR_TNF) )
    	{
    		// wait until SSP not busy
    		// todo: is it really need if FIFO full ?
    		while ( SSP_Dev->Device->SR & SSPSR_BSY );

    		// all bytes are transmitted and received
    	    // Read Rx FIFO if not empty.
    	    while ( SSP_Dev->Device->SR & SSPSR_RNE )
    	    {
    	    	*(rxBuffIdx++) = SSP_Dev->Device->SR;
    	    }
    	}

    	// Rx FIFO full
    	// When RxFIFO can be full, what about TxFIFO full above ?
        if ( SSP_Dev->Device->SR & SSPSR_RFF )
        {
    		// wait until SSP not busy
    		// todo: is it really need if FIFO full ?
    		while ( SSP_Dev->Device->SR & SSPSR_BSY );

    		// Read Rx FIFO if not empty.
    	    while ( SSP_Dev->Device->SR & SSPSR_RNE )
    	    {
    	    	*(rxBuffIdx++) = SSP_Dev->Device->SR;
    	    }
        }

        SSP_Dev->Device->SR = *(txBuffIdx++);
    }

	// wait until SSP not busy
	while ( SSP_Dev->Device->SR & SSPSR_BSY );

	// Read Rx FIFO if not empty.
    while ( SSP_Dev->Device->SR & SSPSR_RNE )
    {
    	*(rxBuffIdx++) = SSP_Dev->Device->SR;
    }


    return ret;
}


// txBuff must equals rxBuff
bool_t SSP_LoopbackTest(SSP_Dev_t *SSP_Dev, uint8_t *txBuff, uint32_t txLen, uint8_t *rxBuff)
{
    bool_t res = FALSE;
	uint8_t *txBuffIdx = txBuff;
	uint8_t *rxBuffIdx = rxBuff;


    // send
    for (i = 0; i < txLen; i++)
    {
        /* Move on only if NOT busy and TX FIFO not full. */
        while ( LPC_SSP0->SR & (SSPSR_TNF | SSPSR_BSY) );
        LPC_SSP0->DR = *(txBuff++);
        while ( LPC_SSP0->SR & SSPSR_BSY ); // Wait until send a frame
                                            // todo is it really needed if we check FIFO above ?
    }

    // read
    for (i = 0; i < txLen; i++)
    {
        /* As long as Receive FIFO is not empty, the frame can be always received.
	     * If it's a loopback test, clock is shared for both TX and RX,
	     * no need to write dummy byte to get clock to get the data
	     * if it's a peer-to-peer communication, SSPDR needs to be written
	     * before a read can take place.
	     */
        while ( !(LPC_SSP0->SR & SSPSR_RNE) ); // wait if RxFIFO empty
        *(rxBuff++) = LPC_SSP0->DR;
    }

    //todo: compare buffer


    res = TRUE;

    return res;
}


/******************************************************************************
**                            End Of File
******************************************************************************/

