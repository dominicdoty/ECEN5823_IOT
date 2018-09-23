/***************************************************************************************************
 * \file   msc.c
 * \brief  D.Doty Memory Configuration Routine
 **************************************************************************************************/

//***********************************************************************************
// Include files
//***********************************************************************************

#include "msc.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************

/*
 * Sets the IFC Read Clear Flag
 * Interrupt Flag Clear register will mirror the Interrupt Flag register
 * When Interrupt Flag Clear register is read, it will automatically clear all set flags after read
*/
void msc_ifc_autoclear()
{
	MSC->LOCK = MSC_UNLOCK_CODE;		//Unlock the memory controller registers to allow writing
	MSC->CTRL |= MSC_CTRL_IFCREADCLEAR;	//Set the memory controller to auto clear interrupt flags after reading them
	MSC->LOCK = 1;						//Re-Lock the memory controller registers
}
