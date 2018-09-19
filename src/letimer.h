#ifndef LETIMER_H
#define LETIMER_H

// LETIMER Setup Routine Header
// Author D.Doty
// Compiled in Simplicity IDE

//***********************************************************************************
// Include files
//***********************************************************************************

#include "em_core.h"
#include "em_letimer.h"
#include "src/sleep.h"
#include "src/gpio.h"
#include "em_cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************

#define BLOCK_SLEEP		EM4		//EM0-4 (Note 4 isn't actually implemented)
#define PULSE_LENGTH	175		//mS
#define PERIOD			2250	//mS

// Choose the clock based on energy mode
#if (BLOCK_SLEEP == EM4)
	#define CLOCK	cmuSelect_ULFRCO
	#define FREQ	1000	//Hz
//	#if PERIOD/1000 > 2147000
//		#error Period too big, max is 2,147,000 S
//	#endif
#else
	#define CLOCK	cmuSelect_LFXO
	#define FREQ	32768	//Hz
//	#if PERIOD/1000 > 65535
//		#error Period too big, max is 65,535 S
//	#endif
#endif

//***********************************************************************************
// global variables
//***********************************************************************************

uint8_t prescale;

//***********************************************************************************
// function prototypes
//***********************************************************************************

void LETIMER0_IRQHandler();

void letimer_init();

#endif
