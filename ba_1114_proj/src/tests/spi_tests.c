/*
 * spi_tests.c

 *
 *  Created on: 26-12-2013
 *      Author: arek
 */


#include "driver_config.h"  //todo describe that it MUST BE, and SSP driver will be not ON in there,
                            // than all the ssp.h and .c file will be not add to compile, so during compilation
                            // of this file, there will be error. This approach is very buggy.
#include "gpio.h"
#include "ssp.h"
#include "spi_tests.h"



static uint8_t src_addr[SSP_BUFSIZE];
static uint8_t dest_addr[SSP_BUFSIZE];

//todo description, describe hot to set driver settings
/*****************************************************************************
** Function name:		LoopbackTest
**
** Descriptions:		Loopback test
**
** Parameters:			SPI port number
** Returned value:		None
**
*****************************************************************************/
boolean_t SPI_LoopbackInternalTest(uint8_t portNum)
{
    boolean_t res = TRUE;
    uint8_t i = 0;

    //todo
    // why USE_CS, FIFOSIZE, BUFFSIZE, what about IRS use

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
#if !USE_CS
        GPIOSetValue(PORT0, 2, 0);
#endif
    }
    else if ( SPI1 == portNum )
    {
#if !USE_CS
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
        SSP_Send( portNum, (uint8_t *)&src_addr[i], FIFOSIZE );

  	    /* If RX interrupt is enabled, below receive routine can be
  	     * also handled inside the ISR.
  	     */
  	    SSP_Receive( portNum, (uint8_t *)&dest_addr[i], FIFOSIZE );

  	    i += FIFOSIZE;
    }


    /* Set SSEL pin to output high. */
    if ( SPI0 == portNum )
    {
#if !USE_CS
        GPIOSetValue( PORT0, 2, 1 );
#endif
    }
    else if ( SPI1 == portNum )
    {
#if !USE_CS
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


//todo
// description
boolean_t SPI_LoopbackHardwareTest(void)
{
	boolean_t res = TRUE;
    uint8_t i = 0;

    SSP_IOConfig(SPI0);
    SSP_Init(SPI0);

    SSP_IOConfig(SPI1);
    SSP_Init(SPI1);

    for (i = 0; i < SSP_BUFSIZE; i++)
    {
  	    src_addr[i] = (uint8_t)i;
  	    dest_addr[i] = 0;
    }

    SSP_Send(SPI0, (uint8_t *)src_addr, SSP_BUFSIZE);
    SSP_Receive(SPI1, (uint8_t *)dest_addr, SSP_BUFSIZE);
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

