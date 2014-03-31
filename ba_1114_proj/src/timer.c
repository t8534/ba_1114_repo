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
#include "timer.h"
#include "timer_hw.h"

void TimerStart(Timer_t * tmr, TmrCnt_t timeout)
{
    tmr->start = HwTimerGetCount();
    tmr->timeout = timeout;
    tmr->state = TIMER_RUNNING;
}

void TimerStop(Timer_t * tmr)
{
    tmr->state = TIMER_STOPPED;
}

int TimerPause(Timer_t * tmr)
{
    int result = -1;
    TmrCnt_t elap = TimerGetElap(tmr);

    /* We can pause the timer only if the timer hasn't elapsed. */
    if (elap < tmr->timeout) {
       tmr->timeout -= elap;
       tmr->state = TIMER_PAUSED;
       result = 0;
    }

    return result;
}

int TimerResume(Timer_t * tmr)
{
    int result = -1;

    if (TIMER_PAUSED == tmr->state) {
        TimerStart(tmr, tmr->timeout);
        result = 0;
    }

    return result;
}

TimerState_t TimerGetState(Timer_t * tmr)
{
    if (TIMER_RUNNING == tmr->state) {
        TmrCnt_t elap;

        /* This is wrap around arithmetic, no need to test if cur < start, etc.
         * Please don't try to improve this.
         * The only problematic situation would be if timeout is set to TIMER_MAX
         * or close to that. We may have no chance to catch the elapsed condition
         * before the cur wraps around (all depends how ofte you call the TimerGetState().
         */
        elap = TimerGetElap(tmr);

        if (elap >= tmr->timeout) {
            tmr->state = TIMER_ELAPSED;
        }
    }

    return tmr->state;
}

TmrCnt_t TimerGetElap(Timer_t * tmr)
{
    return HwTimerGetCount() - tmr->start;
}

TmrCnt_t TimerGetRemaining(Timer_t * tmr)
{
    return tmr->timeout - TimerGetElap(tmr);
}

TmrCnt_t TimerGetTimeout(Timer_t * tmr)
{
    return tmr->timeout;
}

void Wait(TmrCnt_t timeout)
{
    Timer_t tmr;

    TimerStart(&tmr, timeout);
    while(TIMER_RUNNING == TimerGetState(&tmr));
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


