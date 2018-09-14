#ifndef SLEEP_H
#define SLEEP_H

// Sleep Routine Header
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

#include "em_core.h"
#include "em_emu.h"

//***********************************************************************************
// defined files
//***********************************************************************************

#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
#define EM4 4

//***********************************************************************************
// global variables
//***********************************************************************************

int sleep_block_counter[4];

//***********************************************************************************
// function prototypes
//***********************************************************************************

void blockSleepMode(int8_t minimumMode);

void unblockSleepMode(int8_t minimumMode);

void sleep();

#endif
