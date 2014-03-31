/*
 * timer.h
 *
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "type.h"
#include "timer_hw.h"


/** Converts time expressed in [us] to timer counts.
 * \param us Time expressed in [us].
 * \retval Timer counts.
 */
#define TIMER_US_2_CNT(us)   HWTIMER_US_2_CNT(us)
/** Converts time expressed in [ms] to timer counts.
 * \param us Time expressed in [ms].
 * \retval Timer counts.
 */
#define TIMER_MS_2_CNT(ms)   HWTIMER_MS_2_CNT(ms)

/** Timer states. */
typedef enum {
   TIMER_STOPPED,
   TIMER_RUNNING,
   TIMER_PAUSED,
   TIMER_ELAPSED
} TimerState_t;

/** Timer representation. */
typedef struct {
   TmrCnt_t       start;
   TmrCnt_t       timeout;
   TimerState_t   state;
} Timer_t;

/** Starts timer for a given timeout. */
void TimerStart(
    Timer_t * tmr, /**< Timer to be started. */
    TmrCnt_t timeout /**< Timeout expressed in counts, use the TIMER_US_2_CNT(), TIMER_MS_2_CNT() to make convertion from [us], [ms] respectively. */
);

/** Stops timer. */
void TimerStop(
    Timer_t * tmr /**< Timer to be stopped. */
);

/** Pause the timer. Counting down can be later resumed.
 * \retval 0 on success.
 * \retval -1 on failure, e.g. already elapsed timer can not be paused.
 */
int TimerPause(
    Timer_t * tmr /**< Pointer to the timer. */
);

/** Resumes previously paused timer.
 * \retval 0 on success.
 * \retval -1 on failure, when timer is not in paused state.
 */
int TimerResume(
    Timer_t * tmr /**< Pointer to the timer. */
);

/** Returns curren state of given timer. */
TimerState_t TimerGetState(
    Timer_t * tmr /**< Pointer to the timer. */
);

/** Returns time elapsed (in counts) on given timer. */
TmrCnt_t TimerGetElap(
    Timer_t * tmr /**< Pointer to the timer. */
);

/** Returns remaining time for specified timer to elapse (in counts). */
TmrCnt_t TimerGetRemaining(
    Timer_t * tmr /**< Pointer to the timer. */
);

/** Returns timeout value set to given timer. */
TmrCnt_t TimerGetTimeout(
    Timer_t * tmr /**< Pointer to the timer. */
);

/**
 * Waits in a tight loop for a given time, use the TIMER_US_2_CNT(),
 * TIMER_MS_2_CNT() to make convertion from [us], [ms] respectively.
 * CAUTION! This function doesn't take into account multitasking.
 */
void Wait(
    TmrCnt_t timeout
);

/** Initializes the timer library. */
#define  TimerInit() HwTimerInit()




#endif /* TIMER_H_ */
