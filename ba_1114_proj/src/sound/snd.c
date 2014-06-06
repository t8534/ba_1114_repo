#include "driver_config.h"  //todo describe this include MUST BE, and if SSP driver will be not ON in there,
                            // than all the ssp.h and .c file will be not add to compile, so during compilation
                            // of this file, there will be error. This approach is very buggy.
#include "snd.h"


void SND_Init(void)
{

	//
	// timer16
	// match0 - pwm cycle
	// match1 = match2 = duty period
	// match1 - PWM1 output
	// match2 - PWM2 output
	// PWM1, PWM2 outputs w przeciwfazie.
	// Open drain outputs ?
	//



}
