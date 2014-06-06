#include "driver_config.h"  //todo describe this include MUST BE, and if SSP driver will be not ON in there,
                            // than all the ssp.h and .c file will be not add to compile, so during compilation
                            // of this file, there will be error. This approach is very buggy.

#include "timer16.h"
#include "pwm_user.h"



// The output current is 3-4 mA, max 45-50 mA

/////////////////////////////////////////////////////////////////////////////
//
// The Timer1 16 bit is used for sound generation.
//
// The Timer1 input clock = SystemClock = 48 MHz
// With prescaler register PR = 48 it means the Timer clock cycle, after divide
// by 48 equals 1 us.
// So the Timer cycle is 1 us.
// This is enough resolution for sound generator.
// 1 kHz this is 1 ms cycle.
// With 1 us cycle resolution this is 0.001 kHz -> 1 Hz resolution.
// Also with PWM cycle = 1 us, the frequency (max) is 1 MHz
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
// In the PWM mode, following on UM10398 this is working as below:
//
// Note [UM10398, p.343]: It is recommended to use match channel 3 to set
// the PWM cycle because it is not pinned out.
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
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//
// Finally settings.
//
// MAT3 register is used to define PWM cycle because it is not related to any
// output pin.
// PWM Channel 0 is used (PWMEN0) - MAT0 define duration time.
// Duration is related to the volume level. This is initialized to 1/2 of cycle.
// MAT0 define second part of cycle. The PWM cycle starting always with Low.
//
// TIMER1 MAT0 is output pin - P1.9 (FUNC = 0x01, MODE = 0x02 (default pull-up resistor) )
//
///////////////////////////////////////////////////////////////////////////////




//todo The Timer driver is hard set and this is need to reconfigure is as well.



// The PWM cycle should be 0Hz on init.
// duration 1/2 cycle - maximum voice level
// For tests it could be 1kHz on init.
void PWMUSR_Init(uint16_t cycle_ms)
{

	//init_timer16PWM(uint8_t timer_num, uint32_t period, uint8_t match_enable, uint8_t cap_enabled);
	//setMatch_timer16PWM(uint8_t timer_num, uint8_t match_nr, uint32_t value);

	//disable_timer16(uint8_t timer_num);
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


	//void enable_timer16(uint8_t timer_num);
	//void disable_timer16(uint8_t timer_num);
	//void reset_timer16(uint8_t timer_num);
	//void init_timer16(uint8_t timer_num, uint16_t timerInterval);
	//void setMatch_timer16(uint8_t timer_num, uint8_t match_nr, uint32_t value);

	//void init_timer16PWM(uint8_t timer_num, uint32_t period, uint8_t match_enable, uint8_t cap_enabled);
	//void setMatch_timer16PWM (uint8_t timer_num, uint8_t match_nr, uint32_t value);

}


void PWMUSR_DeInit(void)
{



}
