#include "timer16.h"
#include "pwm_user.h"





//
// The Timer1 16 bit is used for sound generation.

// MAT0 reg is used to define PWM cycle
// MAT1 define duration time.
// Duration is volume level. Initialized to 1/2 of cycle.
// MAT1 define second part of cycle. The PWM cycle start
// always with Low.
//
// TIMER1 MAT1 is output pin - P1.10 (FUNC = 0x02, MODE = 0x02 (default pull-up resistor) )
//
// PWM signal starts always from Low level. On all configured
// MATx outputs this is set to Low. Than after reach any MATx register,
// the assigned pin is going High, until timer counter is reset.
// Than it goes again to Low after match of MATx assigned to the given pin.
// For example let MAT0 is set to 1000 ticks, and reset Timer after
// match. Let MAT1 is set to 400 ticks.
// When Timer start, out pin related to MAT1 goes Low, like all the
// other. Than when Timer counts 400 ticks it match MAT1, so pin
// related to MAT1 goes High. But after Timer counts to 1000, it will
// match MAT0, the Timer will be reset. And because of Timer reset,
// all MATx pins goes back to Low.
//

// The Timer driver is hard set and this is need to reconfigure is as well.



// The PWM cycle should be 1000 ms on init, todo is it 1 Hz ?
// duration 1/2 cycle - maximum voice level
void PWMUSR_Init(uint16_t cycle_ms)
{

	init_timer16PWM(uint8_t timer_num, uint32_t period, uint8_t match_enable, uint8_t cap_enabled);
	setMatch_timer16PWM (uint8_t timer_num, uint8_t match_nr, uint32_t value);

	disable_timer16(uint8_t timer_num);
    // this is disabled here
}


void PWMUSR_Enable(void)
{

}


void PWMUSR_Disable(void)
{

}


void PWMUSR_Set(uint16_t cycle_ms, uint16_t duration_ms)
{
	// stop pwm

	// update registers

	// start pwm


	void enable_timer16(uint8_t timer_num);
	void disable_timer16(uint8_t timer_num);
	void reset_timer16(uint8_t timer_num);
	void init_timer16(uint8_t timer_num, uint16_t timerInterval);
	void setMatch_timer16(uint8_t timer_num, uint8_t match_nr, uint32_t value);

	void init_timer16PWM(uint8_t timer_num, uint32_t period, uint8_t match_enable, uint8_t cap_enabled);
	void setMatch_timer16PWM (uint8_t timer_num, uint8_t match_nr, uint32_t value);

}


void PWMUSR_DeInit(void)
{



}
