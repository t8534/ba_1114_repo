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

// TODO
//
// 1.
// Update comments
//
//

#include "type.h"


/* IO pins
 *
 * Please check are pin is available on uP model.
 *
 */

typedef enum
{

	SSP_NO_PIN = 0,

	/* SSP0 */

	SSP_MOSI0_PIN_0_9,

	SSP_MISO0_PIN_0_8,

	SSP_SCK0_PIN_0_6,
	SSP_SCK0_PIN_0_10,
	SSP_SCK0_PIN_2_11,

	SSP_SSEL0_PIN_0_2,

	/* SSP1 */

	SSP_MOSI1_PIN_2_3,
	SSP_MOSI1_PIN_1_9,

	SSP_MISO1_PIN_2_2,
	SSP_MISO1_PIN_1_10,

	SSP_SCK1_PIN_2_1,
	SSP_SCK1_PIN_3_2,

	SSP_SSEL1_PIN_2_0,
	SSP_SSEL1_PIN_2_4

} SSP_IO_pins_t;



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
#define SSP_MASTER_MODE			((uint32_t)(0))
#define SSP_SLAVE_MODE			((uint32_t)(1U<<2))


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
#define SSP_ISR_NOFLAG_SET    ((uint32_t)(0U))
#define SSP_ISR_RORIM         ((uint32_t)(1U))
#define SSP_ISR_RTIM          ((uint32_t)(1U<<1))
#define SSP_ISR_RXIM          ((uint32_t)(1U<<2))
#define SSP_ISR_TXIM          ((uint32_t)(1U<<3))


typedef enum {

	SSP_SSEL_AUTO,
	SSP_SSEL_GPIO,

} SSP_SSEL_Mode_t;


typedef struct {

	SSP_IO_pins_t MOSI_pin;
	SSP_IO_pins_t MISO_pin;
	SSP_IO_pins_t SCK_pin;
	SSP_IO_pins_t SSEL_pin;

} SSP_HW_IO_t;


/* SPI configuration structure. See defines above
 * If SSEL pin is not automatic, but configured as GPIO you have to
 * initialize and control it on your side using:
 *
 * SSP_SSEL0_GPIO_Init()
 * SSP_SSEL0_GPIO_High()
 * SSP_SSEL0_GPIO_Low()
 *
 * SSP_SSEL1_GPIO_Init()
 * SSP_SSEL1_GPIO_High()
 * SSP_SSEL1_GPIO_Low()
 *
 *
 *
 * Clock Settings.
 *
 * CR0->SerialClockRate
 *
 * Serial Clock Rate. The number of prescaler output clocks per bit on the bus,
 * minus one. The CPSDVSR is the prescale divider, and the APB clock
 * PCLK clocks the prescaler, the bit frequency is PCLK / (CPSDVSR x [SCR+1]).
 *
 *
 * SSP0CPSR->CPSDVSR
 * SSP1CPSR->CPSDVSR
 *
 * This even value between 2 and 254, by which SPI_PCLK is divided to yield
 * the prescaler output clock. Bit 0 always reads as 0.
 *
 *
 * SSP0CLKDIV->DIV
 * SSP1CLKDIV->DIV
 *
 * SPIx_PCLK clock divider values
 * 0: Disable SPI0_PCLK.
 * 1 - 255: Divide by 1 to 255
 *
 *
 *
 */
typedef struct {

	LPC_SSP_TypeDef *Device;  /* LPC_SSP0 or LPC_SSP1 */
    uint32_t FrameFormat;
	uint32_t DataSize;
	uint32_t CPOL;
	uint32_t CPHA;
	uint32_t LoopBackMode;
	uint32_t Mode;

	uint8_t SCR;              /* CR0->SerialClockRate */
    uint8_t CPSDVSR;          /* SSPxCPSR->CPSDVSR */
    uint8_t DIV;              /* SSPxCLKDIV->DIV */

	uint32_t SlaveOutputDisable;
	SSP_TransferType_t transferType;
	uint32_t InterruptCondition;
	void (*ISR_Processing)( void );

	SSP_SSEL_Mode_t SSEL_Mode;    /* SSEL pin automatic or controlled by user */
	SSP_HW_IO_t IO_pins;

} SSP_Dev_t;



void SSP_Init(SSP_Dev_t *SSP_Dev);
void SSP_DeInit(SSP_Dev_t *SSP_Dev);
void SSP_ConfigUpdate(SSP_Dev_t *SSP_Dev);

boolean_t SSP_LoopbackTest(SSP_Dev_t *SSP_Dev);
void SSP_WriteRead(SSP_Dev_t *SSP_Dev, uint16_t *tx_buff, uint16_t *rx_buff, uint16_t len);

void SSP_SSEL0_GPIO_Init(void);
void SSP_SSEL0_GPIO_High(void);
void SSP_SSEL0_GPIO_Low(void);
void SSP_SSEL1_GPIO_Init(void);
void SSP_SSEL1_GPIO_High(void);
void SSP_SSEL1_GPIO_Low(void);

#endif  /* __SSP_H__ */
/*****************************************************************************
**                            End Of File
******************************************************************************/

