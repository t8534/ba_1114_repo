/****************************************************************************
 *   $Id:: timer16.h 4785 2010-09-03 22:39:27Z nxp21346                     $
 *   Project: NXP LPC11xx 16-bit timer example
 *
 *   Description:
 *     This file contains 16-bit timer code example which include timer 
 *     initialization, timer interrupt handler, and related APIs for 
 *     timer setup.
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
#ifndef __TIMER16_H 
#define __TIMER16_H
#if CONFIG_ENABLE_DRIVER_TIMER16==1

/* TIMER_CLOCKFREQ is the clock rate into the timer prescaler */
#define TIMER_CLOCKFREQ SystemCoreClock

/* MHZ_PRESCALE is a value to set the prescaler to in order to
   clock the timer at 1 MHz. Clock needs to be a multiple of 1 MHz or
   this will not work. */
#define MHZ_PRESCALE    (TIMER_CLOCKFREQ/1000000)

/* TIME_INTERVALmS is a value to load the timer match register with
   to get a 1 mS delay */
#define TIME_INTERVALmS	1000

/* The test is either MAT_OUT or CAP_IN. Default is MAT_OUT. */
#define TIMER_MATCH		0


void delayMs(uint8_t timer_num, uint32_t delayInMs);

#define EMC0	4
#define EMC1	6
#define EMC2	8
#define EMC3	10

#define MATCH0	(1<<0)
#define MATCH1	(1<<1)
#define MATCH2	(1<<2)
#define MATCH3	(1<<3)


void TIMER16_0_IRQHandler(void);
void TIMER16_1_IRQHandler(void);

void TMR16_Enable(uint8_t timer_num);
void TMR16_Disable(uint8_t timer_num);
void TMR16_Reset(uint8_t timer_num);
void TMR16_Init(uint8_t timer_num, uint16_t timerInterval);
void TMR16_InitPWM(uint8_t timer_num, uint32_t period, uint8_t match_enable, uint8_t cap_enabled);
void TMR16_SetMatchPWM (uint8_t timer_num, uint8_t match_nr, uint32_t value);

////////////////////////////////////////////////////////////
// PWM section

// Channels output pin:
//
// CT16B0 timer:
//
//    Label            Pint description    Pin
//
//    TMR16_PWM_CH0    CT16B0_MAT0         IOCON_PIO0_8
//    TMR16_PWM_CH1    CT16B0_MAT1         IOCON_PIO0_9
//    TMR16_PWM_CH2    CT16B0_MAT2         IOCON_SWCLK_PIO0_10  !! Not clear it is Timer0 or 1 ???
//
//
// CT16B1 timer:
//
//    TMR16_PWM_CH0    CT16B1_MAT0         IOCON_PIO1_9
//    TMR16_PWM_CH1    CT16B1_MAT1         IOCON_PIO1_10
//
//
// Rules for single edge controlled PWM outputs:
//
// 1. All single edge controlled PWM outputs go LOW at the beginning of each PWM cycle
// (timer is set to zero) unless their match value is equal to zero.
//
// 2. Each PWM output will go HIGH when its match value is reached. If no match occurs
// (i.e. the match value is greater than the PWM cycle length), the PWM output remains
// continuously LOW.
//
// 3. If a match value larger than the PWM cycle length is written to the match register, and
// the PWM signal is HIGH already, then the PWM signal will be cleared on the next start
// of the next PWM cycle.
//
// 4. If a match register contains the same value as the timer reset value (the PWM cycle
// length), then the PWM output will be reset to LOW on the next clock tick. Therefore,
// the PWM output will always consist of a one clock tick wide positive pulse with a
// period determined by the PWM cycle length (i.e. the timer reload value).
//
// 5. If a match register is set to zero, then the PWM output will go to HIGH the first time the
// timer goes back to zero and will stay HIGH continuously.
//


typedef enum {
	TMR16_PWM_CH0,
	TMR16_PWM_CH1,
	TMR16_PWM_CH2    // Not used on Timer 1
} TMR16_PWMChannel_t;


// PWM output pin behaviour (EMR reg)
typedef enum {
	TMR16_PWM_OUT_DO_NOTHING = 0x00,
	TMR16_PWM_OUT_LOWH = 0x01,
	TMR16_PWM_OUT_HIGH = 0x02,
	TMR16_PWM_OUT_TOGGLE = 0x03
} TMR16_PWMPinMode_t;


// Enable PWM output pin (EMR reg)
typedef enum {
	TMR16_PWM_CH0_PIN_ENABLE = (1 << 0),
	TMR16_PWM_CH1_PIN_ENABLE = (1 << 1),
	TMR16_PWM_CH2_PIN_ENABLE = (1 << 2)
} TMR16_PWMChannelPinEnable_t;


// Set output pin as PWM mode, not a Timer.
typedef enum {
	TMR16_PWM_CH0_PIN_PWM_MODE = (1 << 0),
	TMR16_PWM_CH1_PIN_PWM_MODE = (1 << 1),
	TMR16_PWM_CH2_PIN_PWM_MODE = (1 << 2)
} TMR16_PWMChannelPinPWMMode_t;


//todo MCR reg
// Control Timer Counter (TC) action when Match Register match (MCR).
// Example:
// MCR |= ((TMR16_PWM_TC_IRQ | TMR16_PWM_TC_RESET | TMR16_PWM_TC_STOP) << TMR16_PWM_TC_CTRL_MR3)
typedef enum {
    TMR16_PWM_TC_IRQ = (1 << 0),
    TMR16_PWM_TC_RESET = (1 << 1),
    TMR16_PWM_TC_STOP = (1 << 2)
} TMR16_PWM_TC_Action_t;

// Select Match Register which control Timer Counter action on match (MCR).
typedef enum {
    TMR16_PWM_TC_CTRL_MR0 = 0,
    TMR16_PWM_TC_CTRL_MR1 = 3,
    TMR16_PWM_TC_CTRL_MR2 = 6,
    TMR16_PWM_TC_CTRL_MR3 = 9
} TMR16_PWM_TC_Ctrl_Action_t;


typedef struct {

	LPC_TMR_TypeDef *Device;   /* LPC_TMR16B0 or LPC_TMR16B1 */
	TMR16_PWMChannel_t PWMChannel;
	TMR16_PWMChannelPinEnable_t PWMChannelPinEnable;
	TMR16_PWMPinMode_t PWMPinMode;
	TMR16_PWMChannelPinPWMMode_t PWMChannelPinPWMMode;

	// Control Timer Counter (TC) action when Match Register match (MCR).
	// Select Match Register which control Timer Counter action on match (MCR).
	// Example:
	// MCR |= ((TMR16_PWM_TC_IRQ | TMR16_PWM_TC_RESET | TMR16_PWM_TC_STOP) << TMR16_PWM_TC_CTRL_MR3)
    //
	// Note: When the match outputs are selected to serve as PWM outputs, the timer reset
	// (MRnR) and timer stop (MRnS) bits in the Match Control Register MCR must be set to 0
	// except for the match register setting the PWM cycle length. For this register, set the
	// MRnR bit to 1 to enable the timer reset when the timer value matches the value of the
	// corresponding match register.

	uint32_t  timerCounterActionOnMatch;


	uint8_t tmrPrescaler;    //todo add description how to calculate.

	uint16_t pwmCycle_us;
	uint16_t pwmDuty_us;


} TMR16_PWMConfig_t;


void TMR16_PWMInit(TMR16_PWMConfig_t *PWMCfg);
void TMR16_PWMDeInit(TMR16_PWMConfig_t *PWMCfg);
void TMR16_PWMSetCycle(TMR16_PWMConfig_t *PWMCfg);
void TMR16_PWMSetDuty(TMR16_PWMConfig_t *PWMCfg);
void TMR16_PWMEnable(TMR16_PWMConfig_t *PWMCfg);
void TMR16_PWMDisable(TMR16_PWMConfig_t *PWMCfg);





#endif
#endif /* end __TIMER16_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
