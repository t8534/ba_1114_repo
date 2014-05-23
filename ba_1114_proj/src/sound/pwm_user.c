#include "timer16.h"
#include "pwm_user.h"





//
// The Timer1 16 bit is used for sound generation.
// MAT0 reg is used to define PWM cycle
// MAT1 define duration time.
// Duration is volume level. Initialized to 1/2 of cycle.
//
// MAT1 Output ????
// PWM signal starts always from Low level.
//




// The PWM cycle should be 1000 ms on init,
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
