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
#include "gpio.h"
#include "ssp.h"
#include "spi_tests.h"


/* SSP0 Masked Interrupt register */
#define SSPMIS_RORMIS   ((uint32_t)(1U<<0))
#define SSPMIS_RTMIS    ((uint32_t)(1U<<1))
#define SSPMIS_RXMIS    ((uint32_t)(1U<<2))
#define SSPMIS_TXMIS    ((uint32_t)(1U<<3))

/* SSP0 Interrupt clear register */
#define SSPICR_RORIC    ((uint32_t)(1U<<0))
#define SSPICR_RTIC     ((uint32_t)(1U<<1))

/* SSP Status register */
#define SSPSR_TFE       ((uint32_t)(1U<<0))
#define SSPSR_TNF       ((uint32_t)(1U<<1))
#define SSPSR_RNE       ((uint32_t)(1U<<2))
#define SSPSR_RFF       ((uint32_t)(1U<<3))
#define SSPSR_BSY       ((uint32_t)(1U<<4))


#define FIFOSIZE		8


/********************** SSP Control *****************************
 * Used for control functionality
 */

#define SSP_CTRL_ENABLE       ((uint32_t)(1U<<1))
#define SSP_CTRL_DISABLE      ~((uint32_t)(1U<<1))

#define SSP_CTRL_LOOPBACK_ON     ((uint32_t)(1U))
#define SSP_CTRL_LOOPBACK_OFF    ~((uint32_t)(1U))



void SSP0_IRQHandler(void)
{
    uint32_t regValue;

    regValue = LPC_SSP0->MIS;
    if ( regValue & SSPMIS_RORMIS )	    /* Receive overrun interrupt */
    {
	    LPC_SSP0->ICR = SSPICR_RORIC;	/* clear interrupt */
    }
    if ( regValue & SSPMIS_RTMIS )	    /* Receive timeout interrupt */
    {
	    LPC_SSP0->ICR = SSPICR_RTIC;	/* clear interrupt */
    }

    /* Please be aware, in main and ISR, CurrentRxIndex and CurrentTxIndex
     * are shared as global variables. It may create some race condition that
     * main and ISR manipulate these variables at the same time. SSPSR_BSY
     * checking (polling) in both main and ISR could prevent this kind of race
     * condition.
     */
    if ( regValue & SSPMIS_RXMIS )	/* Rx at least half full */
    {
    }

    return;
}


void SSP1_IRQHandler(void)
{
    uint32_t regValue;

    regValue = LPC_SSP1->MIS;
    if ( regValue & SSPMIS_RORMIS )	    /* Receive overrun interrupt */
    {
	    LPC_SSP1->ICR = SSPICR_RORIC;	/* clear interrupt */
    }
    if ( regValue & SSPMIS_RTMIS )	    /* Receive timeout interrupt */
    {
	    LPC_SSP1->ICR = SSPICR_RTIC;	/* clear interrupt */
    }

    /* Please be aware that, in main and ISR, CurrentRxIndex and CurrentTxIndex
     * are shared as global variables. It may create some race condition that
     * main and ISR manipulate these variables at the same time. SSPSR_BSY
     * checking (polling) in both main and ISR could prevent this kind of race
     * condition.
     */
    if ( regValue & SSPMIS_RXMIS )	/* Rx at least half full */
    {
    }

    return;
}



/* This is a set of function to define SSEL pins controlled not automatically,
 * but by hand.
 */
void SSP_SSEL0_GPIO_Init(void)
{
    /* Enable AHB clock to the GPIO domain. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

    LPC_IOCON->PIO0_2 &= ~0x07;
    LPC_IOCON->PIO0_2 |= 0x00;   /* GPIO pin mode = 0x00 */
    GPIOSetDir(PORT0, 2, 1);     /* Output */
    GPIOSetValue(PORT0, 2, 1);   /* High */
}

void SSP_SSEL0_GPIO_High(void)
{
    GPIOSetValue(PORT0, 2, 1);
}

void SSP_SSEL0_GPIO_Low(void)
{
    GPIOSetValue(PORT0, 2, 0);
}

void SSP_SSEL1_GPIO_Init(void)
{
    /* Enable AHB clock to the GPIO domain. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

    LPC_IOCON->PIO2_0 &= ~0x07;
    LPC_IOCON->PIO2_0 |= 0x00;   /* GPIO pin mode = 0x00 */
    GPIOSetDir(PORT2, 0, 1);     /* Output */
    GPIOSetValue(PORT2, 0, 1);   /* High */
}

void SSP_SSEL1_GPIO_High(void)
{
	GPIOSetValue(PORT2, 0, 1);
}

void SSP_SSEL1_GPIO_Low(void)
{
	GPIOSetValue(PORT2, 0, 0);
}



void SSP_IO_Init(SSP_IO_pins_t pin)
{
    switch (pin) {

        case SSP_MOSI0_PIN_0_9:
        {
            LPC_IOCON->PIO0_9 &= ~0x07;
            LPC_IOCON->PIO0_9 |= 0x01;
        	break;
        }

        case SSP_MISO0_PIN_0_8:
        {
            LPC_IOCON->PIO0_8 &= ~0x07;
            LPC_IOCON->PIO0_8 |= 0x01;
        	break;
        }

        /* The SCK0 function is multiplexed to three different pin locations
         * (two locations on the HVQFN package). Use the IOCON_LOC register
         * (see Section 7.4) to select a physical location for the SCK0
         * function in addition to selecting the function in the IOCON
         * registers. The SCK1 pin is not multiplexed.
         */

        case SSP_SCK0_PIN_0_6:
        {
        	LPC_IOCON->SCK_LOC &= ~0x03;
        	LPC_IOCON->SCK_LOC |= 0x02;
            LPC_IOCON->PIO0_6 &= ~0x07;
            LPC_IOCON->PIO0_6 |= 0x02;
        	break;
        }

        case SSP_SCK0_PIN_0_10:
        {
        	LPC_IOCON->SCK_LOC &= ~0x03;
        	LPC_IOCON->SCK_LOC |= 0x00;
        	LPC_IOCON->SWCLK_PIO0_10 &= ~0x03;
        	LPC_IOCON->SWCLK_PIO0_10 |= 0x02;
        	break;
        }

        case SSP_SCK0_PIN_2_11:
        {
        	LPC_IOCON->SCK_LOC &= ~0x03;
        	LPC_IOCON->SCK_LOC |= 0x01;
            LPC_IOCON->PIO2_11 &= ~0x07;
            LPC_IOCON->PIO2_11 |= 0x01;
        	break;
        }

        case SSP_SSEL0_PIN_0_2:
        {
            LPC_IOCON->PIO0_2 &= ~0x07;
            LPC_IOCON->PIO0_2 |= 0x01;
        	break;
        }


        case SSP_MOSI1_PIN_2_3:
        {
            LPC_IOCON->PIO2_3 &= ~0x07;
            LPC_IOCON->PIO2_3 |= 0x02;
        	break;
        }

        case SSP_MOSI1_PIN_1_9:
        {
            LPC_IOCON->PIO1_9 &= ~0x07;
            LPC_IOCON->PIO1_9 |= 0x02;
        	break;
        }

        case SSP_MISO1_PIN_2_2:
        {
            LPC_IOCON->PIO2_2 &= ~0x07;
            LPC_IOCON->PIO2_2 |= 0x02;
        	break;
        }

        case SSP_MISO1_PIN_1_10:
        {
            LPC_IOCON->PIO1_10 &= ~0x07;
            LPC_IOCON->PIO1_10 |= 0x03;
        	break;
        }

        case SSP_SCK1_PIN_2_1:
        {
            LPC_IOCON->PIO2_1 &= ~0x07;
            LPC_IOCON->PIO2_1 |= 0x02;
        	break;
        }

        case SSP_SCK1_PIN_3_2:
        {
            LPC_IOCON->PIO3_2 &= ~0x07;
            LPC_IOCON->PIO3_2 |= 0x03;
        	break;
        }

        case SSP_SSEL1_PIN_2_0:
        {
            LPC_IOCON->PIO2_0 &= ~0x07;
            LPC_IOCON->PIO2_0 |= 0x02;
        	break;
        }

        case SSP_SSEL1_PIN_2_4:
        {
            LPC_IOCON->PIO2_4 &= ~0x07;
            LPC_IOCON->PIO2_4 |= 0x02;
        	break;
        }

        default:
        	break;

    }

}


void SSP_Init(SSP_Dev_t *SSP_Dev)
{

	uint32_t regVal = 0;
	uint8_t i = 0;


	/* Before write to reg the clock must be enabled */
    if (SSP_Dev->Device == LPC_SSP0)
    {
        /* Enable SSP0 clock */
    	LPC_SYSCON->SYSAHBCLKCTRL |= ((uint32_t)(1<<11));

        /* Set peripherial clock divider.
    	 *
    	 * 0: Disable SPI0_PCLK.
  	     * 1 - 255: Divide by 1 - 255.
  	     */
        LPC_SYSCON->SSP0CLKDIV = SSP_Dev->DIV;

        /* Reset SSP0
         * Before accessing the SPI and I2C peripherals, write a one to this
         * register to ensure that the reset signals to the SPI and I2C are
         * de-asserted.
         */
        LPC_SYSCON->PRESETCTRL &= ~((uint32_t)(1<<0));
        LPC_SYSCON->PRESETCTRL |= ((uint32_t)(1<<0));

    }
    else  /* LPC_SSP1 */
    {

        /* Enable SSP1 clock */
    	LPC_SYSCON->SYSAHBCLKCTRL |= ((uint32_t)(1<<18));

        /* Set peripherial clock divider.
    	 *
    	 * 0: Disable SPI1_PCLK.
  	     * 1 - 255: Divide by 1 - 255.
  	     */
        LPC_SYSCON->SSP1CLKDIV = SSP_Dev->DIV;

        /* Reset SSP1
         * Before accessing the SPI and I2C peripherals, write a one to this
         * register to ensure that the reset signals to the SPI and I2C are
         * de-asserted.
         */
        LPC_SYSCON->PRESETCTRL &= ~((uint32_t)(1<<2));
        LPC_SYSCON->PRESETCTRL |= ((uint32_t)(1<<2));

    }

	SSP_IO_Init(SSP_Dev->IO_pins.MOSI_pin);
	SSP_IO_Init(SSP_Dev->IO_pins.MISO_pin);
	SSP_IO_Init(SSP_Dev->IO_pins.SCK_pin);
	if (SSP_Dev->SSEL_Mode == SSP_SSEL_AUTO)
	{
	    SSP_IO_Init(SSP_Dev->IO_pins.SSEL_pin);
	}
	else
	{
		/* SSEL is controlled by GPIO by the user. */
		if (SSP_Dev->Device == LPC_SSP0)
		{
			SSP_SSEL0_GPIO_Init();
		}
		else
		{
			SSP_SSEL1_GPIO_Init();

		}
	}

	regVal |= SSP_Dev->DataSize | SSP_Dev->FrameFormat | SSP_Dev->CPOL | SSP_Dev->CPHA;
	regVal |= ((uint32_t)(SSP_Dev->SCR << 8));
	SSP_Dev->Device->CR0 = regVal;

	regVal = 0;
	regVal |= SSP_Dev->LoopBackMode | SSP_Dev->Mode | SSP_Dev->SlaveOutputDisable;
	SSP_Dev->Device->CR1 = regVal;
	SSP_Dev->Device->CPSR = SSP_Dev->CPSDVSR;

	/* Set Interrupt Mask Register */
	regVal = 0;
	regVal = SSP_Dev->InterruptCondition;
	SSP_Dev->Device->IMSC;

    if (SSP_Dev->Device == LPC_SSP0)
    {
        NVIC_EnableIRQ(SSP0_IRQn);
    }
    else  /* LPC_SSP1 */
    {
        NVIC_EnableIRQ(SSP1_IRQn);
    }


	/* clear the RxFIFO */
    for (i = 0; i < FIFOSIZE; i++)
    {
    	regVal = SSP_Dev->Device->DR;
    }

    /* Enable SSP */
	SSP_Dev->Device->CR1 |= SSP_CTRL_ENABLE;

}


void SSP_DeInit(SSP_Dev_t *SSP_Dev)
{
    /* Disable SSP */
	SSP_Dev->Device->CR1 &= SSP_CTRL_DISABLE;

    if (SSP_Dev->Device == LPC_SSP0)
    {
        NVIC_DisableIRQ(SSP0_IRQn);

        /* Disable peripherial clock */
        LPC_SYSCON->SSP0CLKDIV = 0;

        /* Disable SSP0 clock */
        LPC_SYSCON->SYSAHBCLKCTRL &= ~((uint32_t)(1<<11));
    }
    else  /* SSP1 */
    {
        NVIC_DisableIRQ(SSP1_IRQn);

        /* Disable peripherial clock */
        LPC_SYSCON->SSP1CLKDIV = 0;

        /* Disable SSP1 clock */
        LPC_SYSCON->SYSAHBCLKCTRL &= ~((uint32_t)(1<<18));
    }

}


void SSP_ConfigUpdate(SSP_Dev_t *SSP_Dev)
{
	SSP_DeInit(SSP_Dev);
	SSP_Init(SSP_Dev);
}


boolean_t SSP_LoopbackTest(SSP_Dev_t *SSP_Dev)
{
	boolean_t res = TRUE;
    uint8_t buffLen = FIFOSIZE;
	uint8_t txBuff[FIFOSIZE] = {1, 2, 3, 4, 5, 6, 7, 8};
	uint8_t rxBuff[FIFOSIZE];
	uint16_t i = 0;


    for (i = 0; i < buffLen; i++)
    {
        /* Move on only if NOT busy and TX FIFO not full. */
        while ( SSP_Dev->Device->SR & (SSPSR_TNF | SSPSR_BSY) );
        SSP_Dev->Device->DR = txBuff[i];
    }

	// wait until SSP not busy
	while ( SSP_Dev->Device->SR & SSPSR_BSY );

    for (i = 0; i < buffLen; i++)
    {
        /* As long as Receive FIFO is not empty, the frame can be always
         * received. If it's a loopback test, clock is shared for both
         * TX and RX, no need to write dummy byte to get clock to get the data
	     * if it's a peer-to-peer communication, SSPDR needs to be written
	     * before a read can take place.
	     */

    	/* Read Rx FIFO if not empty. */
        while ( SSP_Dev->Device->SR & SSPSR_RNE )
        {
        	rxBuff[i] = SSP_Dev->Device->DR;
        }
    }

    for (i = 0; i < buffLen; i++)
    {
    	if (txBuff[i] != rxBuff[i])
    	{
    		res = FALSE;
    		break;
    	}
    }

    return res;
}


/* tx_buff, rx_buff must be 16 bit !
 *
 */
void SSP_WriteRead(SSP_Dev_t *SSP_Dev, uint16_t *tx_buff, uint16_t *rx_buff, uint16_t len)
{
	uint16_t t_idx = 0;
	uint16_t tmp = 0;


	uint16_t *tx_buff_ptr = tx_buff;
	uint16_t *rx_buff_ptr = rx_buff;

	/* Wait until Tx FIFO empty and SSP not busy. */
    while ( !(SSP_Dev->Device->SR & SSPSR_TFE) || (SSP_Dev->Device->SR & SSPSR_BSY) );

    /* Read Rx FIFO until empty and wait until SSP not busy. */
    while ( SSP_Dev->Device->SR & (SSPSR_RNE | SSPSR_BSY) )
    {
    	tmp = SSP_Dev->Device->DR;
    }

	while (len)
	{
		t_idx = 0;

		while (len > 0 && t_idx < FIFOSIZE)
		{
            SSP_Dev->Device->DR = *tx_buff_ptr++;
            t_idx++;
            len--;
		}

		/* wait until Tx FIFO will be completely sent (empty) and SSP not Busy */
		while ( !(SSP_Dev->Device->SR & SSPSR_TFE) || (SSP_Dev->Device->SR & SSPSR_BSY) );

		while (t_idx)
		{
			/* wait until rx_fifo not empty */
		    while ( (SSP_Dev->Device->SR & SSPSR_RNE) != SSPSR_RNE );
			*rx_buff_ptr++ = SSP_Dev->Device->DR;
            t_idx--;
		}

	}

}




// Only for SSP1 !
//
uint8_t SSPSendRecvByte(uint8_t outb)
{
	uint8_t res = 0;

	//todo you can fifo buffer here, to be sure this is empty

    /* Move on only if NOT busy and TX FIFO not full. */
    while ( (LPC_SSP1->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF );

    LPC_SSP1->DR = outb;

    // wait until fifo not empty and periph no busy
    while ( (LPC_SSP1->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );

    res = LPC_SSP1->DR;


  	return res;
}



/******************************************************************************
**                            End Of File
******************************************************************************/

