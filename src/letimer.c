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
void LETIMER0_IRQHandler(){
	//disable peripheral call interrupt
	CORE_ATOMIC_IRQ_DISABLE();

	//copy the interrupt register (auto clears flag)
	uint32_t intreg = LETIMER0->IFC;

	//determine which interrupt triggered
	if(intreg & LETIMER_IFC_UF){
		//turn the LED off
		GPIO_PinOutClear(LED0_port, LED0_pin);
	}
	else{
		//turn the LED on
		GPIO_PinOutSet(LED0_port, LED0_pin);
	}

	//enable peripheral call interrupt
	CORE_ATOMIC_IRQ_ENABLE();
}


void letimer_init(){
	// Calculate Prescaler
	// Note that compiler optimization just deletes this whole section and stores the values in temporary variables
	// You will not see this run in debugger. The variables also will not show up. But the registers do get set correctly
	uint64_t frac = (PERIOD*FREQ)/(65536*1000);	//Calculate the fraction of desired counts to max timer counts
	prescale = 0;
	while(frac != 0){		//bit shift right until fraction value is zero
		frac = frac >> 1;	//this rounds the value up to the next power of two
		prescale++;			//prescale counts the number of divisions by two required to get the desired period in the LETIMER max counts
	}						//prescale register translates this value to powers of two (ie 0->div1, 1->div2, 2->div4, etc.)


	// Calculate Timer Comp Values
	const uint16_t period_cnts = (PERIOD*FREQ)/((1<<prescale) *1000);
	const uint16_t pulse_cnts = (PULSE_LENGTH*FREQ)/((1<<prescale) *1000);


	// Configure CMU for LETIMER0
	#if CLOCK == cmuSelect_LFXO						//if because ULFRCO is automatically enabled
	  CMU_OscillatorEnable(CLOCK, true, true);		//ENABLE OSCILLATOR
	#endif
	CMU_ClockSelectSet(cmuClock_LFA, CLOCK);		//SELECT OSC ONTO BUS
	CMU->LFAPRESC0 = (prescale & 0b00001111);		//Clock Prescaler Set
	CMU_ClockEnable(cmuClock_HFLE,true);			//Clock to enable register coms
	CMU_ClockEnable(cmuClock_LETIMER0,true);		//Enable Letimer0 clock


	// Initialize LETIMER0
	MSC->LOCK = MSC_UNLOCK_CODE;		//Unlock the memory controller registers to allow writing
	MSC->CTRL |= MSC_CTRL_IFCREADCLEAR;	//Set the memory controller to auto clear interrupt flags after reading them
	MSC->LOCK = 1;						//Re-Lock the memory controller registers
	LETIMER0->CMD = LETIMER_CMD_STOP;	//Make sure LETIMER0 is stopped


	// Set up LETIMER0 Config Struct
	const LETIMER_Init_TypeDef timer0 = {.enable = false,
										  .comp0Top = true};

	// Configure LETIMER0 Registers
	LETIMER_CompareSet(LETIMER0,0,period_cnts);	//Set the period comparator
	LETIMER_CompareSet(LETIMER0,1,pulse_cnts);	//Set the pulse length comparator
	LETIMER_RepeatSet(LETIMER0,0,1);			//Set rep counter to non 0 for free running mode
	LETIMER_Init(LETIMER0,&timer0);				//Initialize timer with set values from above


	// Interrupt Configuration
	LETIMER0->IFC = LETIMER_IFC_UF | LETIMER_IFC_COMP1;	//clear int flags
	LETIMER0->IEN = LETIMER_IEN_UF | LETIMER_IEN_COMP1;	//enable int flags
	blockSleepMode(BLOCK_SLEEP);						//set max sleep mode
	NVIC_EnableIRQ(LETIMER0_IRQn);						//enable interrupt vector


	// Start the Timer
	LETIMER0->CMD = LETIMER_CMD_START;	//start the LETIMER0
	while((LETIMER0->SYNCBUSY) & 1);	//twiddle thumbs while LETIMER registers sync
}
