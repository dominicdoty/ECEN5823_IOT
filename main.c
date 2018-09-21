
/***********************************************************************************************//**
 * \file   main.c
 * \brief  D.Doty
 *
 * Project for ECEN 5823
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
#include "src/letimer.h"
#include "src/msc.h"
#include "src/timer.h"

//***********************************************************************************
// defined files
//***********************************************************************************

#define FLAG_I2C_BRINGUP	1
#define FLAG_I2C_REQ		2
#define FLAG_I2C_READ		4

//***********************************************************************************
// global variables
//***********************************************************************************

uint16_t flags = 0;

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

	//set the flag
	flags |= FLAG_I2C_BRINGUP;

	//enable peripheral call interrupt
	CORE_ATOMIC_IRQ_ENABLE();
}

void TIMER0_IRQHandler(){
	//disable peripheral call interrupt
	CORE_ATOMIC_IRQ_DISABLE();

	//copy the interrupt register (auto clears flag)
	uint32_t intreg = TIMER0->IFC;

	//set the flag
	flags |= FLAG_I2C_REQ;

	//unblock sleep mode (only necessary because of oneshot)
	unblockSleepMode(EM2);

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

  // MSC setup
  msc_ifc_autoclear();

  // Initialize LETIMER
  struct letimer_config fig =
  {
  	.block_sleep = EM3,	//sleep mode you cannot go down to
  	.period = 3000,		//mS
  	.pulse_width = 0,	//mS
	.oneshot = false
  };

  letimer_init(fig);

  // Initialize HFTIMER
  struct hftimer_config hf_fig =
  {
  	.period = 80,
  	.pulse_width = 0,
  	.oneshot = true
  };

  hftimer_init(hf_fig);



  // Sleep
  while (1) {
	  uint16_t clear_flags = 0;

	  if(flags & FLAG_I2C_BRINGUP)
	  {
		  GPIO_PinOutSet(LED0_port, LED0_pin);
//		  temp_sensor_on();
		  blockSleepMode(EM2);
		  TIMER_Enable(TIMER0, true);
		  clear_flags |= FLAG_I2C_BRINGUP;
	  }

	  if(flags & FLAG_I2C_REQ)
	  {
		  GPIO_PinOutSet(LED1_port, LED1_pin);
//		  i2c_start_bus();
//		  i2c_write();
		  clear_flags |= FLAG_I2C_REQ;
	  }

	  if(flags & FLAG_I2C_READ)
	  {
		  GPIO_PinOutClear(LED0_port, LED0_pin);
		  GPIO_PinOutClear(LED1_port, LED1_pin);
//		  i2c_read();
//		  temp_sensor_off();
		  clear_flags |= FLAG_I2C_READ;
	  }

	  // Atomic section to clear global flags
	  CORE_ATOMIC_IRQ_DISABLE();
	  flags &= ~clear_flags;
	  CORE_ATOMIC_IRQ_ENABLE();

	  // If all flags have been serviced go to sleep, otherwise start over
	  if(flags == 0)
	  {
	  sleep();
	  }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
