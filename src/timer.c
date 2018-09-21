// HFTIMER Setup Routine Source
// Author D.Doty
// Compiled in Simplicity IDE

//***********************************************************************************
// Include files
//***********************************************************************************
#include "src/timer.h"

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
 * Takes a configuration structure and sets up the timer to produce
 * interrupts at a given period and pulse length. AKA, int at period,
 * int at period + pulse length, int at 2*period, etc.
 * Always blocks EM2 sleep (must stay in EM1 for HF)
*/
void hftimer_init(struct hftimer_config fig){
	uint32_t frequency;
	if(fig.period > 1740)	//Max time you can get in the HFtimer without prescaling the hf bus
	{
		while(1);
	}

	//check that we're using the HFXO for the timer, otherwise jam the program from proceeding
	if(CMU->HFCLKSTATUS == CMU_HFCLKSTATUS_SELECTED_HFXO)
	{
		frequency = 38400000;	//Hz
	}
	else
	{
		while(1);
	}

	// Calculate Prescaler
	uint64_t frac = (fig.period*(frequency/65536))/1000;	//Calculate the fraction of desired counts to max timer counts // WORK verify non overflow
	uint8_t prescale = 0;
	while(frac != 0){		//bit shift right until fraction value is zero
		frac = frac >> 1;	//this rounds the value up to the next power of two
		prescale++;			//prescale counts the number of divisions by two required to get the desired period in the LETIMER max counts
	}						//prescale register translates this value to powers of two (ie 0->div1, 1->div2, 2->div4, etc.)

	if (prescale > 10)
	{
		while(1);			//period is too big, jam program here
	}

	// Calculate Timer Comp Values
	const uint32_t period_cnts = (fig.period*(frequency/(1<<prescale)))/1000;
	const uint32_t pulse_cnts = (fig.pulse_width*(frequency/(1<<prescale)))/1000;

	// Configure CMU for TIMER0
	CMU->CTRL |= CMU_CTRL_HFPERCLKEN;				//Enable HF Peripheral Clock
	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;		//Enable Timer0 clock

	// Stop TIMER0
	TIMER0->CMD = TIMER_CMD_STOP;				//Make sure TIMER0 is stopped

	// Configure TIMER0 Registers				//Clock Prescaler Set
	TIMER0->CTRL |= ((uint32_t)(prescale & 0b00001111))<<_TIMER_CTRL_PRESC_SHIFT;
	TIMER0->TOP = period_cnts;					//Set upper timer limit
	TIMER0->CTRL |= TIMER_CTRL_MODE_DOWN;		//Set for down counting

	// Interrupt Configuration
	TIMER0->IFC = TIMER_IFC_UF;					//clear int flags
	TIMER0->IEN |= TIMER_IEN_UF;				//enable int flags
	NVIC_EnableIRQ(TIMER0_IRQn);				//enable interrupt vector

	if (fig.pulse_width != 0)					//If the pulse width is non-zero, configure comp1 ints
	{
		TIMER0->IFC = TIMER_IFC_CC0;			//clear comp0 flag
		TIMER0->IEN |= TIMER_IFC_CC0;			//set comp0 enabled
		TIMER0->CC[0].CCV = pulse_cnts;			//set comp0 value
		TIMER0->CC[0].CTRL |=					//set mode to output compare
		(((uint32_t)(timerCCModeCompare)) << _TIMER_CC_CTRL_MODE_SHIFT);

	}

	// Configure Oneshot Mode and Start (only start if in free run mode
	if (fig.oneshot == true)
	{
		TIMER0->CTRL |= TIMER_CTRL_OSMEN;		//Set to oneshot mode
	}
	else
	{
	blockSleepMode(EM2);						//set max sleep mode
	TIMER0->CMD = TIMER_CMD_START;				//start the TIMER0
	}
}
