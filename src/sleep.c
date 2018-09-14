// Sleep Routine Source
// Author D.Doty
// Adapted from Silicon Labs Code
// Compiled in Simplicity IDE
// SI Labs Copyright:

/* @section License*
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 **********************************************************************************
 * Permission is granted to anyone to use this software for any purpose,* including commercial applications,
 * and to alter it and redistribute it* freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not*    claim that you wrote the original software.*
 * 2. Altered source versions must be plainly marked as such, and must not be*    misrepresented as being the original software.*
 * 3. This notice may not be removed or altered from any source distribution.**
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no* obligation to support this Software.
 * Silicon Labs is providing the* Software "AS IS", with no express or implied warranties of any kind,*
 * including, but not limited to, any implied warranties of merchantability* or fitness for any particular purpose
 * or warranties against infringement* of any proprietary rights of a third party.**
 * Silicon Labs will not be liable for any consequential, incidental, or* special damages,
 * or any other relief, or for any claim by any third party,* arising from your use of this Software.
 ***********************************************************************************/

//***********************************************************************************
// Include files
//***********************************************************************************
#include "src/sleep.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************

int sleep_block_counter[4];

//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************

void blockSleepMode(int8_t minimumMode){
	//Disable interrupts
	//Increment the minmode
	//Enable interrupts
	CORE_ATOMIC_IRQ_DISABLE();
	sleep_block_counter[minimumMode]++;
	CORE_ATOMIC_IRQ_ENABLE();
}

void unblockSleepMode(int8_t minimumMode){
	//Disable interrupts
	//If minmode is greater than 0, decrement
	//If minmode is 0, go into infinite loop, blocking all execution (for debugging, might disable this later)
	//Enable interrupts
	CORE_ATOMIC_IRQ_DISABLE();
	if(sleep_block_counter[minimumMode] > 0){
		sleep_block_counter[minimumMode]--;
	}else{
		while(1);
	}
	CORE_ATOMIC_IRQ_ENABLE();
}

void sleep(void){
	if(sleep_block_counter[0]>0){		//stay in EM0
		return;
	}
	else if(sleep_block_counter[1]>0){	//EM1 blocked, go to EM0
		return;
	}
	else if(sleep_block_counter[2]>0){	//EM2 blocked, go to EM1
		EMU_EnterEM1();
	}
	else if(sleep_block_counter[3]>0){	//EM3 blocked, go to EM2
		EMU_EnterEM2(true);				//true means restore HF clock state to pre-sleep condition
	}
	else {								//Else go to EM3, (ignoring EM4 signals)
		EMU_EnterEM3(true);				//true means restore HF clock state to pre-sleep condition
	}
}






