//***********************************************************************************
// Include files
//***********************************************************************************
#include "src/cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void cmu_init(void){
	// Peripheral clocks enabled
	CMU_ClockEnable(cmuClock_GPIO, true);

	// Configure CMU for LETIMER0
	#if CLOCK == cmuSelect_LFXO						//if because ULFRCO is automatically enabled
	  CMU_OscillatorEnable(CLOCK, true, true);		//ENABLE OSCILLATOR
	#endif
	CMU_ClockSelectSet(cmuClock_LFA, CLOCK);		//SELECT OSC ONTO BUS
	CMU->LFAPRESC0 = (prescale & 0b00001111);		//Clock Prescaler Set
	CMU_ClockEnable(cmuClock_HFLE,true);			//Clock to enable register coms
	CMU_ClockEnable(cmuClock_LETIMER0,true);		//Enable Letimer0 clock
}

