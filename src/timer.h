#ifndef TIMER_H
#define TIMER_H

// HFTIMER Setup Routine Source
// Author D.Doty
// Compiled in Simplicity IDE

//***********************************************************************************
// Include files
//***********************************************************************************

#include "main.h"
#include "src/sleep.h"
#include "em_core.h"
#include "em_cmu.h"
#include "em_timer.h"


//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************

/*
 * HFTimer Configuration Structure, passed to hftimer_init();
*/
struct hftimer_config
{
	uint16_t period;		//mS
	uint16_t pulse_width;	//mS
	bool oneshot;			//oneshot timer or free run
};

//***********************************************************************************
// function prototypes
//***********************************************************************************

void hftimer_init(struct hftimer_config fig);

#endif
