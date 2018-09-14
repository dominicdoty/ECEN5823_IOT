
/***********************************************************************************************//**
 * \file   main.c
 * \brief  D.Doty Energy Modes Assignment
 *
 * Project for ECEN 5823 - Managing Energy Modes
 * Based on the Silicon Labs Example Code
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;
//***********************************************************************************
// Include files
//***********************************************************************************

#include "src/main.h"
#include "src/gpio.h"
#include "src/cmu.h"
#include "em_core.h"
#include "src/sleep.h"
#include "em_letimer.h"

//***********************************************************************************
// defined files
//***********************************************************************************

#define SLEEPMODE		EM2		//EM0-4 (Note 4 isn't actually implemented)
#define PULSE_LENGTH	175		//mS
#define PERIOD			2250	//mS

// Choose the clock based on energy mode
#if (SLEEPMODE == EM3) | (SLEEPMODE == EM4)
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

//***********************************************************************************
// main
//***********************************************************************************

/**
 * @brief  Main function
 */

int main(void)
{
  // Initialize device
  initMcu();

  // Initialize board
  initBoard();

  // Initialize GPIO
  gpio_init();

  // Initialize clocks
  cmu_init();

  // Initialize stack
  gecko_init(&config);

  // Calculate Prescaler
  // Note that compiler optimization just deletes this whole section and stores the values in temporary variables
  // You will not see this run in debugger. The variables also will not show up. But the registers do get set correctly
  long int frac = (PERIOD*FREQ)/(65535*1000);	//Calculate the fraction of desired counts to max timer counts
  int prescale = 0;
  while(frac != 0){			//bit shift right until fraction value is zero
	  frac = frac >> 1;		//this rounds the value up to the next power of two
	  prescale++;			//prescale counts the number of divisions by two required to get the desired period in the LETIMER max counts
  }							//prescale register translates this value to powers of two (ie 0->div1, 1->div2, 2->div4, etc.)

  // Calculate Timer Comp Values
  uint16_t period_cnts = (PERIOD*FREQ)/((1<<prescale) *1000);
  uint16_t pulse_cnts = (PULSE_LENGTH*FREQ)/((1<<prescale) *1000);

  // Configure CMU for LETIMER0
#if CLOCK == cmuSelect_LFXO						//if because ULFRCO is automatically enabled
  CMU_OscillatorEnable(CLOCK, true, true);		//ENABLE OSCILLATOR
#endif
  CMU_ClockSelectSet(cmuClock_LFA, CLOCK);		//SELECT OSC ONTO BUS
  CMU_ClockDivSet(cmuClock_LETIMER0, prescale);	//Clock Prescaler Set
  CMU_ClockEnable(cmuClock_HFLE,true);			//Clock to enable register coms
  CMU_ClockEnable(cmuClock_LETIMER0,true);		//Enable Letimer0 clock


  // Initialize LETIMER0
  MSC->LOCK = MSC_UNLOCK_CODE;			//Unlock the memory controller registers to allow writing
  MSC->CTRL |= MSC_CTRL_IFCREADCLEAR;	//Set the memory controller to auto clear interrupt flags after reading them
  MSC->LOCK = 1;						//Re-Lock the memory controller registers
  LETIMER0->CMD = LETIMER_CMD_STOP;		//Make sure LETIMER0 is stopped

  //Set up LETIMER0 config struct
  const LETIMER_Init_TypeDef timer0 = {.enable = false,
		  	  	  	  	  	  	  	  .comp0Top = true};

  LETIMER_CompareSet(LETIMER0,0,period_cnts);	//Set the period comparator
  LETIMER_CompareSet(LETIMER0,1,pulse_cnts);	//Set the pulse length comparator
  LETIMER_RepeatSet(LETIMER0,0,1);				//Set rep counter to non 0 for free running mode
  LETIMER_Init(LETIMER0,&timer0);				//Initialize timer with set values from above

  // Interrupt Configuration
  LETIMER0->IFC = LETIMER_IFC_UF | LETIMER_IFC_COMP1;	//clear int flags
  LETIMER0->IEN = LETIMER_IEN_UF | LETIMER_IEN_COMP1;	//enable int flags
  blockSleepMode(SLEEPMODE);							//set max sleep mode
  NVIC_EnableIRQ(LETIMER0_IRQn);						//enable interrupt vector

  // Start the Timer
  LETIMER0->CMD = LETIMER_CMD_START;	//start the LETIMER0
  while((LETIMER0->SYNCBUSY) & 1);		//twiddle thumbs while LETIMER registers sync

  // Sleep
  while (1) {
	  sleep();
  }
}


//int main(void)
//{
//  int i;
//
//  // Initialize device
//  initMcu();
//  // Initialize board
//  initBoard();
//
//  /* Initialize GPIO */
//  gpio_init();
//
//  // Initialize clocks
//  cmu_init();
//
//  // Initialize stack
//  gecko_init(&config);
//
//
//  while (1) {
//		for (i = 0; i < 5000000; i++);
//		GPIO_PinOutClear(LED0_port, LED0_pin);
//
//		for (i = 0; i < 5000000; i++);
//		GPIO_PinOutClear(LED1_port, LED1_pin);
//
//		for (i = 0; i < 20000000; i++);
//		GPIO_PinOutSet(LED0_port, LED0_pin);
//		GPIO_PinOutSet(LED1_port, LED1_pin);
//  }
//}

//for 2.25 prescaler should be div 2, counts should be 36864
//prescaler isn't setting correctly
//comp0 setting to 8044, not correct


/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
