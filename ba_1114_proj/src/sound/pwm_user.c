#include "timer16.h"
#include "pwm_user.h"



// The PWM cycle should be 1000 ms
void PWMUSR_Init(uint16_t cycle_ms)
{



}


void PWMUSR_Set(uint16_t cycle_ms, uint16_t duration_ms, uint16_t silence_ms)
{
	// stop pwm

	// update registers

	// start pwm


	void enable_timer16(uint8_t timer_num);
	void disable_timer16(uint8_t timer_num);
	void reset_timer16(uint8_t timer_num);
	void init_timer16(uint8_t timer_num, uint16_t timerInterval);
	void setMatch_timer16(uint8_t timer_num, uint8_t match_nr, uint32_t value);

}


void PWMUSR_DeInit(void)
{



}
