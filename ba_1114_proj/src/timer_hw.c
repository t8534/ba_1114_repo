/*
 * timer.c
 *
 *
 * rev. 140111_0916
 *
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

#include "hwtimer.h"



void HwTimerInit(void)
{
    /* Enable TPU channels 5-0. */
    MSTP.CRA.BIT._TPUL = 0;

    /* Stop TPU1 and TPU2 before any changes to its registers. */
    TPU.TSTR.BIT.CST1 = 0;
    TPU.TSTR.BIT.CST2 = 0;

    #if defined(PCLK_MHZ)
        #if (PCLK_MHZ == 32)
            /* TCNT2 counts on peripherial modules clock 32 MHz divided by 1024.
             * TCNT1 counts on TCNT2 overflow.
             * 1 count - 32 us
             * TCNT2 overflow ~2.1s
             * TCNT1 overflow ~38h
             */
            TPU1.TCR.BYTE = TPU_CLEARING_DISABLED |
                            TPU_COUNT_ON_RISING_EDGE |
                            TPU1_COUNT_ON_TCNT2_OVERFLOW;

            TPU2.TCR.BYTE = TPU_CLEARING_DISABLED |
                            TPU_COUNT_ON_RISING_EDGE |
                            TPU2_PRESCALER_1024;
        #elif(PCLK_MHZ == 40)
            /* TCNT2 counts on peripherial modules clock 40 MHz divided by 1024.
             * TCNT1 counts on TCNT2 overflow.
              * 1 count - 25.6 us
              * TCNT2 overflow ~1.67s
              * TCNT1 overflow ~30.5h
              */
             TPU1.TCR.BYTE = TPU_CLEARING_DISABLED |
                             TPU_COUNT_ON_RISING_EDGE |
                             TPU1_COUNT_ON_TCNT2_OVERFLOW;

             TPU2.TCR.BYTE = TPU_CLEARING_DISABLED |
                             TPU_COUNT_ON_RISING_EDGE |
                             TPU2_PRESCALER_1024;
        #else
            #error "PCLK_MHZ changed!"
        #endif
    #else
        #error "Missing defines: PLCK_MHZ."
    #endif

    TPU1.TIOR.BYTE = TPU_OUTPUT_DISABLED;

    TPU2.TIOR.BYTE = TPU_OUTPUT_DISABLED;

	TPU1.TMDR.BYTE = 0;
	TPU2.TMDR.BYTE = 0;

    /* Start TPU1 and TPU2 chanells. */
    TPU.TSTR.BYTE |= TPU1_START_BIT | TPU2_START_BIT;
}

TmrCnt_t HwTimerGetCount(void)
{
    TmrCnt_t hiWord1, hiWord2, loWord;

    /* Read the higer part of the counter twice to detect
     * overflows from lo to hi part of the counter.
     */
    hiWord1 = TPU1.TCNT;
    loWord = TPU2.TCNT;
    hiWord2 = TPU1.TCNT;

    if (hiWord2 != hiWord1) {
        loWord = TPU2.TCNT;
    }

    return (TmrCnt_t)((hiWord2 << 16) | loWord);
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


