// LETIMER Setup Routine Source
// Author D.Doty
// Compiled in Simplicity IDE

//***********************************************************************************
// Include files
//***********************************************************************************
#include "src/letimer.h"

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
 * Blocks sleep to specified level, EM4 -> ULFRCO, EM0-3 -> LFXO
*/
void letimer_init(struct letimer_config fig){
	// Choose the Oscillator
	// Choose the clock based on energy mode
	uint32_t oscillator;
	uint16_t frequency;
	if (fig.block_sleep == EM4)
	{
		oscillator = cmuSelect_ULFRCO;
		frequency = 1000;	//Hz
		if(fig.period/1000 > 2147000)
		{
			while(1); //period is too big, jam the program here
		}
	}
	else
	{
		oscillator = cmuSelect_LFXO;
		frequency = 32768;	//Hz
		if(fig.period/1000 > 65535)
		{
			while(1); //period is too big, jam the program here
		}
	}

	// Calculate Prescaler
	uint64_t frac = (fig.period*frequency)/(65536*1000);	//Calculate the fraction of desired counts to max timer counts
	uint8_t prescale = 0;
	while(frac != 0){		//bit shift right until fraction value is zero
		frac = frac >> 1;	//this rounds the value up to the next power of two
		prescale++;			//prescale counts the number of divisions by two required to get the desired period in the LETIMER max counts
	}						//prescale register translates this value to powers of two (ie 0->div1, 1->div2, 2->div4, etc.)


	// Calculate Timer Comp Values
	const uint16_t period_cnts = (fig.period*frequency)/((1<<prescale) *1000);
	const uint16_t pulse_cnts = (fig.pulse_width*frequency)/((1<<prescale) *1000);

	// Configure CMU for LETIMER0
	if (oscillator == cmuSelect_LFXO)									//if because ULFRCO is automatically enabled
	{
		CMU_OscillatorEnable((CMU_Select_TypeDef)oscillator, true, true);//ENABLE OSCILLATOR
	}
	CMU_ClockSelectSet(cmuClock_LFA, (CMU_Select_TypeDef)oscillator);	//SELECT OSC ONTO BUS
	CMU->LFAPRESC0 = (prescale & 0b00001111);								//Clock Prescaler Set
	CMU_ClockEnable(cmuClock_HFLE,true);									//Clock to enable register coms
	CMU_ClockEnable(cmuClock_LETIMER0,true);								//Enable Letimer0 clock


	// Initialize LETIMER0
	LETIMER0->CMD = LETIMER_CMD_STOP;	//Make sure LETIMER0 is stopped


	// Set up LETIMER0 Config Struct
	LETIMER_Init_TypeDef timer0 = {.enable = false,
										  .comp0Top = true};

	// Configure LETIMER0 Registers
	if (fig.pulse_width != 0)					//If the pulse width is non-zero, configure comp1
	{
		LETIMER_CompareSet(LETIMER0,1,pulse_cnts);
	}

	if (fig.oneshot == true)
	{
		timer0.repMode = letimerRepeatOneshot;		//Set rep mode to one shot
	}
	else
	{
		timer0.repMode = letimerRepeatFree;			//Set rep mode to free running
	}
	LETIMER_RepeatSet(LETIMER0,0,1);			//Set rep counter to 1 for oneshot (also needs to be non zero for free run)
	LETIMER_CompareSet(LETIMER0,0,period_cnts);	//Set the period comparator
	LETIMER_Init(LETIMER0,&timer0);				//Initialize timer with set values from above


	// Interrupt Configuration
	if (fig.pulse_width != 0)					//If the pulse width is non-zero, configure comp1 ints
	{
		LETIMER0->IFC = LETIMER_IFC_COMP1;		//clear comp1 flag
		LETIMER0->IEN |= LETIMER_IEN_COMP1;		//set comp1 enabled
	}

	LETIMER0->IFC = LETIMER_IFC_UF;				//clear int flags
	LETIMER0->IEN |= LETIMER_IEN_UF;				//enable int flags
	blockSleepMode(fig.block_sleep);			//set max sleep mode
	NVIC_EnableIRQ(LETIMER0_IRQn);				//enable interrupt vector


	// Start the Timer
	LETIMER0->CMD = LETIMER_CMD_START;	//start the LETIMER0
	while((LETIMER0->SYNCBUSY) & 1);	//twiddle thumbs while LETIMER registers sync
}
