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
#include "main.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************

/*
 * LETIMER0 Configuration Structure, passed to letimer_init();
*/
struct letimer_config
{
	uint8_t block_sleep;	//sleep mode you cannot go down to
	uint16_t period;		//mS
	uint16_t pulse_width;	//mS
	bool oneshot;			//oneshot timer or free run
};

//***********************************************************************************
// function prototypes
//***********************************************************************************

void letimer_init(struct letimer_config fig);

#endif
