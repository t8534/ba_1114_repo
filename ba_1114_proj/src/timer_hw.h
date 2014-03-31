/*
 * timer_hw.h
 *
 *
 */

#ifndef TIMER_HW_H_
#define TIMER_HW_H_

#include "type.h"

/*****************************************************************************
                  Exported symbolic constants
*****************************************************************************/
/* The hwtimer.h exports TIMER_MAX and TmrCnt_t type because the hwtimer module
 * knows size of underlying counter.
 */
#define TIMER_MAX           0xFFFFFFFFUL
/*****************************************************************************
                  Exported function-like macros
*****************************************************************************/
#if (ECLK == 32000UL)
    /* NOTE! Resolution 32[us]. */
    #define  HWTIMER_US_2_CNT(x)    ((TmrCnt_t)((x)>>5))
    #define  HWTIMER_MS_2_CNT(x)    ((TmrCnt_t)(((x)*1000UL)>>5))
    #define  HWTIMER_CNT_2_MS(x)    ((uint32_t)(((x)<<5)/1000UL))
#elif(ECLK == 40000UL)
       /* !!! Tus >= 25,6 [us] */
    #define  HWTIMER_US_2_CNT(x)    ((TmrCnt_t)(((x)*10UL)>>8))
    #define  HWTIMER_MS_2_CNT(x)    ((TmrCnt_t)(((x)*10000UL)>>8))
    #define  HWTIMER_CNT_2_MS(x)    ((uint32_t)(((x)<<8)/10000UL))
#else
    #error "Please recalculate the WTIMER_US_2_CNT(x) and HWTIMER_MS_2_CNT(x) macros in hwtimer.h."
#endif




/*****************************************************************************
                  Exported defined macros
*****************************************************************************/

/*****************************************************************************
                  Exported types, enums definitions
*****************************************************************************/
typedef uint32_t TmrCnt_t;

/*****************************************************************************
                  Exported function prototypes
*****************************************************************************/
/*----------------------------------------------------------------------------
* FUNCTION ARGUMENTS:
* None
*
* RETURN VALUE:
* None
*
* FUNCTION DESCRIPTION:
* Initalize and start free running counter.
*---------------------------------------------------------------------------*/
void HwTimerInit( void );

/*----------------------------------------------------------------------------
* FUNCTION ARGUMENTS:
* None
*
* RETURN VALUE:
* Current number of counts of the free running counter.
*
* FUNCTION DESCRIPTION:
* Returns number of counts of the free running counter.
*---------------------------------------------------------------------------*/
uint32_t HwTimerGetCount( void );

/*****************************************************************************
                  Exported object declarations
*****************************************************************************/



#endif /* TIMER_HW_H_ */
