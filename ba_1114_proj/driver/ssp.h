/****************************************************************************
 *   $Id:: ssp.h 5796 2010-12-03 00:08:56Z nxp21346                         $
 *   Project: NXP LPC11xx SSP example
 *
 *   Description:
 *     This file contains SSP code header definition.
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
#ifndef __SSP_H__
#define __SSP_H__
#if CONFIG_ENABLE_DRIVER_SSP==1

/* There are there modes in SSP: loopback, master or slave. */
/* Here are the combination of all the tests. 

(1) LOOPBACK test:		LOOPBACK_MODE=1, TX_RX_ONLY=0, USE_CS=1;
(2) Serial EEPROM test:	LOOPBACK_MODE=0, TX_RX_ONLY=0, USE_CS=0; (default)
(3) TX(Master) Only:	LOOPBACK_MODE=0, SSP_SLAVE=0, TX_RX_ONLY=1, USE_CS=1;
(4) RX(Slave) Only:		LOOPBACK_MODE=0, SSP_SLAVE=1, TX_RX_ONLY=1, USE_CS=1 */


/********* SPI0 specify section *********/

#define SSP_LOOPBACK_MODE0  0		/* 1 is loopback, 0 is normal operation. */
#define SSP_SLAVE0          0		/* 1 is SLAVE mode, 0 is master mode */
#define SSP_TX_RX_ONLY0     1		/* 1 is TX or RX only depending on SSP_SLAVE
								 * flag, 0 is either loopback mode or communicate
								 * with a serial EEPROM.
								 */

/* If USE_CS is zero, set SSEL as GPIO that you have total control
 * of the sequence. When test serial SEEPROM(LOOPBACK_MODE=0, TX_RX_ONLY=0),
 * set USE_CS to 0. When LOOPBACK_MODE=1 or TX_RX_ONLY=1, set USE_CS to 1.
 */
#define SSP_USE_CS0			1


/********* SPI1 specify section *********/

#define SSP_LOOPBACK_MODE1	0		/* 1 is loopback, 0 is normal operation. */
#define SSP_SLAVE1		1		/* 1 is SLAVE mode, 0 is master mode */
#define SSP_TX_RX_ONLY1		1		/* 1 is TX or RX only depending on SSP_SLAVE
								 * flag, 0 is either loopback mode or communicate
								 * with a serial EEPROM.
								 */

/* If USE_CS is zero, set SSEL as GPIO that you have total control
 * of the sequence. When test serial SEEPROM(LOOPBACK_MODE=0, TX_RX_ONLY=0),
 * set USE_CS to 0. When LOOPBACK_MODE=1 or TX_RX_ONLY=1, set USE_CS to 1.
 */
#define SSP_USE_CS1			1




#define SSP_DEBUG		0  // todo: move out, it not belongs to the driver

/* SPI read and write buffer size */
//#define SSP_BUFSIZE		16
//#define SSP_BUFSIZE		256
#define SSP_BUFSIZE		9

#define SSP_FIFOSIZE		8

#define SSP_DELAY_COUNT		10
#define SSP_MAX_TIMEOUT		0xFF

/* Port0.2 is the SSP select pin */
#define SSP0_SEL        (0x1<<2)
	
/* SSP Status register */
#define SSPSR_TFE       (0x1<<0)
#define SSPSR_TNF       (0x1<<1) 
#define SSPSR_RNE       (0x1<<2)
#define SSPSR_RFF       (0x1<<3) 
#define SSPSR_BSY       (0x1<<4)

/* SSP CR0 register */
#define SSPCR0_DSS      (0x1<<0)
#define SSPCR0_FRF      (0x1<<4)
#define SSPCR0_SPO      (0x1<<6)
#define SSPCR0_SPH      (0x1<<7)
#define SSPCR0_SCR      (0x1<<8)

/* SSP CR1 register */
#define SSPCR1_LBM      (0x1<<0)
#define SSPCR1_SSE      (0x1<<1)
#define SSPCR1_MS       (0x1<<2)
#define SSPCR1_SOD      (0x1<<3)

/* SSP Interrupt Mask Set/Clear register */
#define SSPIMSC_RORIM   (0x1<<0)
#define SSPIMSC_RTIM    (0x1<<1)
#define SSPIMSC_RXIM    (0x1<<2)
#define SSPIMSC_TXIM    (0x1<<3)

/* SSP0 Interrupt Status register */
#define SSPRIS_RORRIS   (0x1<<0)
#define SSPRIS_RTRIS    (0x1<<1)
#define SSPRIS_RXRIS    (0x1<<2)
#define SSPRIS_TXRIS    (0x1<<3)

/* SSP0 Masked Interrupt register */
#define SSPMIS_RORMIS   (0x1<<0)
#define SSPMIS_RTMIS    (0x1<<1)
#define SSPMIS_RXMIS    (0x1<<2)
#define SSPMIS_TXMIS    (0x1<<3)

/* SSP0 Interrupt clear register */
#define SSPICR_RORIC    (0x1<<0)
#define SSPICR_RTIC     (0x1<<1)

/* ATMEL SEEPROM command set */
#define WREN		0x06		/* MSB A8 is set to 0, simplifying test */
#define WRDI		0x04
#define RDSR		0x05
#define WRSR		0x01
#define READ		0x03
#define WRITE		0x02

/* RDSR status bit definition */
#define RDSR_RDY	0x01
#define RDSR_WEN	0x02

/* If RX_INTERRUPT is enabled, the SSP RX will be handled in the ISR
SSPReceive() will not be needed. */
extern void SSP0_IRQHandler (void);
extern void SSP1_IRQHandler (void);
extern void SSP_IOConfig( uint8_t portNum );
extern void SSP_Init( uint8_t portNum );
extern void SSP_Send( uint8_t portNum, uint8_t *Buf, uint32_t Length );
extern void SSP_Receive( uint8_t portNum, uint8_t *buf, uint32_t Length );


/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

//
// 1.
// Configure pins
//
// 2.
// Fill the table.
//


/********************** Configuration ******************************
 * Used only for configuration
 */

/* Frame format */
#define SSP_FRAME_SPI          0x00000000
#define SSP_FRAME_TI           0x00000010    /* Not supported */
#define SSP_FRAME_MICROWIRE    0x00000020    /* Not supported */

/* Data size select. The numbers of bits transfered in each frame */
#define SSP_DATABITS_4    0x00000003
#define SSP_DATABITS_5    0x00000004
#define SSP_DATABITS_6    0000000005
#define SSP_DATABITS_7    0x00000006
#define SSP_DATABITS_8    0x00000007
#define SSP_DATABITS_9    0x00000008
#define SSP_DATABITS_10   0x00000009
#define SSP_DATABITS_11   0x0000000A
#define SSP_DATABITS_12   0x0000000B
#define SSP_DATABITS_13   0x0000000C
#define SSP_DATABITS_14   0x0000000D
#define SSP_DATABITS_15   0x0000000E
#define SSP_DATABITS_16   0x0000000F


/* Clock polarity, should be:
 *
 * - SPI_CPOL_HI: high level
 * - SPI_CPOL_LO: low level
 */
#define SSP_SPI_CPOL_HI    ((uint32_t)(1U<<6))
#define SSP_SPI_CPOL_LO    ((uint32_t)(0))


/* Clock phase, used only in SPI
 *
 * - SPI_CPHA_FIRST: first clock edge
 * - SPI_CPHA_SECOND: second clock edge
 *
 */
#define SSP_SPI_CPHA_FIRST     ((uint32_t)(0))
#define SSP_SPI_CPHA_SECOND    ((uint32_t)(1U<<7))


/* Loop Back Mode
 */
#define SSP_LOOPBACK_ON     ((uint32_t)(1U))
#define SSP_LOOPBACK_OFF    ((uint32_t)(0))


/* Master/Slave mode
 */
#define SSP_SLAVE_MODE			((uint32_t)(0))
#define SSP_MASTER_MODE			((uint32_t)(1U<<2))


/* Slave Output Disable
 */
#define SSP_SLAVE_OUTPUT_ENABLE     ((uint32_t)(0))
#define SSP_SLAVE_OUTPUT_DISABLE    ((uint32_t)(1U<<3))


/* SSP Transfer Type definitions */
typedef enum {
	SSP_TRANSFER_POLLING = 0,
	SSP_TRANSFER_INTERRUPT
} SSP_TransferType_t;


/* SSP Interrupt Condition */
#define SSP_ISR_RORIM    ((uint32_t)(1U))
#define SSP_ISR_RTIM     ((uint32_t)(1U<<1))
#define SSP_ISR_RXIM     ((uint32_t)(1U<<2))
#define SSP_ISR_RXIM     ((uint32_t)(1U<<3))






/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

/* x=target variable, y=mask */
#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK(x,y) ((x) & (y))



/* SPI configuration structure. See defines above */
typedef struct {

	LPC_SSP_TypeDef *Device;  // LPC_SSP0 or LPC_SSP1
    uint32_t FrameFormat;
	uint32_t DataSize;
	uint32_t CPOL;
	uint32_t CPHA;
	uint32_t LoopBackMode;
	uint32_t Mode;
	uint32_t ClockRateHz;	/* Clock rate,in Hz, should not exceed TODO: (SPI peripheral clock)/8 */
	uint32_t SlaveOutputDisable;
	SSI_TransferType_t transferType;
	uint32_t InterruptCondition;
	void (*ISR_Processing)( void );

} SSP_Dev_t;



void SSP_Init(SSP_Dev_t *SSP_Dev);
void SSP_DeInit(SSP_Dev_t *SSP_Dev);
void SSP_ConfigUpdate(SSP_Dev_t *SSP_Dev);

void SSP_Send(SSP_Dev_t *SSP_Dev, uint8_t *buff, uint32_t len);
int32_t SSP_RecvBlock(SSP_Dev_t *SSP_Dev, uint8_t *buff, uint32_t len);
int32_t SSP_SendRecvBlock(SSP_Dev_t *SSP_Dev, uint8_t *txBuff, uint32_t txLen, uint8_t *rxBuff);
bool_t SSP_LoopbackTest(SSP_Dev_t *SSP_Dev);


#endif
#endif  /* __SSP_H__ */
/*****************************************************************************
**                            End Of File
******************************************************************************/

