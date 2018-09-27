
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
#include "src/load_pow_manage.h"
#include "src/i2c.h"

//***********************************************************************************
// defined files
//***********************************************************************************

#define FLAG_I2C_BRINGUP	0b0000000000000001
#define FLAG_I2C_REQ		0b0000000000000010
#define FLAG_I2C_READ		0b0000000000000100

#define I2C_READ_PERIOD		3000	//mS
#define TEMP_RESET_TIME		80		//mS

#define I2C_ADDR			0x40	//slave address
#define I2C_TEMP_CMD		0xE3	//command to measure temp. slave expected to bus hold till response ready

#define THRESHOLD_TEMP		27		//temperature below which to turn on LED1

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

// all interrupts are declared here as this is where
// custom stuff for the timers and I2C is implemented

void LETIMER0_IRQHandler()
{
	//disable peripheral call interrupt
	CORE_ATOMIC_IRQ_DISABLE();

	//copy the interrupt register (auto clears flag)
	//this produces an unused variable warning, but its faster than R IF -> W IFC
	uint32_t intreg = LETIMER0->IFC;

	//set the flag
	flags |= FLAG_I2C_BRINGUP;

	//enable peripheral call interrupt
	CORE_ATOMIC_IRQ_ENABLE();
}

void TIMER0_IRQHandler()
{
	//disable peripheral call interrupt
	CORE_ATOMIC_IRQ_DISABLE();

	//copy the interrupt register (auto clears flag)
	//this produces an unused variable warning, but its faster than R IF -> W IFC
	uint32_t intreg = TIMER0->IFC;

	//set the flag
	flags |= FLAG_I2C_REQ;

	//enable peripheral call interrupt
	CORE_ATOMIC_IRQ_ENABLE();
}

void I2C0_IRQHandler()
{
	//disable peripheral call interrupt
	CORE_ATOMIC_IRQ_DISABLE();

	//copy the interrupt register (auto clears flag)
	//this produces an unused variable warning, but its faster than R IF -> W IFC
	uint32_t intreg = I2C0->IFC;

	//clear the interrupt enable
	//(this is necessary since RXDATAV can only be cleared by
	// reading the RX buffer and I don't want to do that in the interrupt)
	I2C0->IEN &= ~I2C_IEN_RXDATAV;

	//set the flag
	flags |= FLAG_I2C_READ;

	//enable peripheral call interrupt
	CORE_ATOMIC_IRQ_ENABLE();
}

//***********************************************************************************
// main
//***********************************************************************************

int main(void)
{
  // Initialize device //
  initMcu();


  // Initialize board //
  initBoard();


  // Initialize GPIO //
  gpio_init();


  // Initialize clocks //
  cmu_init();


  // Initialize stack //
  gecko_init(&config);


  // MSC setup //
  msc_ifc_autoclear();


  // Initialize LETIMER //
  struct letimer_config fig =
  {
  	.block_sleep = EM4,			//sleep mode you cannot go down to
  	.period = I2C_READ_PERIOD,	//mS
  	.pulse_width = 0,			//mS
	.oneshot = false
  };
  letimer_init(fig);


  // Initialize HFTIMER //
  struct hftimer_config hf_fig =
  {
  	.period = TEMP_RESET_TIME,
  	.pulse_width = 0,
  	.oneshot = true
  };
  hftimer_init(hf_fig);


  // Initialize the I2C Peripheral //
  i2c_init();


  // Initialize the Temp Sensor Power Pin //
  temp_sensor_init();


  // Scheduler //
  while (1) {
	  uint16_t clear_flags = 0;

	  if(flags & FLAG_I2C_BRINGUP)
	  {
		  temp_sensor_on();						//turn on the temp sensor
		  blockSleepMode(EM2);					//block us into EM1 to run high freq timer
		  TIMER_Enable(TIMER0, true);			//start the timer (set for 80mS in config above)
		  clear_flags |= FLAG_I2C_BRINGUP;		//store to clear later
	  }

	  if(flags & FLAG_I2C_REQ)
	  {
		  unblockSleepMode(EM2);					//unblock EM2 from high freq timer
		  i2c_start_bus();							//enable bus, toggle to reset slaves, send abort, block off EM2
		  i2c_read_command(I2C_ADDR, I2C_TEMP_CMD);	//request the temperature from the sensor,
		  clear_flags |= FLAG_I2C_REQ;				//does not wait for the result to return (handled by interrupt)
	  }

	  if(flags & FLAG_I2C_READ)
	  {
		  float reading = i2c_read_temp();		//take the reading and convert to float deg C
		  temp_sensor_off();					//conversion complete, data received. shut down sensor
		  i2c_stop_bus();						//disable bus, abort, disable, remove block on EM2

		  if(reading < THRESHOLD_TEMP)			//enable the LED based on temperature
		  {
			  GPIO_PinOutSet(LED1_port, LED1_pin);
		  }
		  else
		  {
			  GPIO_PinOutClear(LED1_port, LED1_pin);
		  }

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

  // Bluetooth Scheduler
  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:
        /* Set advertising parameters. 100ms advertisement interval.
         * The first two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6). */
        gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        break;

      /* This event is generated when a connected client has either
       * 1) changed a Characteristic Client Configuration, meaning that they have enabled
       * or disabled Notifications or Indications, or
       * 2) sent a confirmation upon a successful reception of the indication. */
      case gecko_evt_gatt_server_characteristic_status_id:
        /* Check that the characteristic in question is temperature - its ID is defined
         * in gatt.xml as "temperature_measurement". Also check that status_flags = 1, meaning that
         * the characteristic client configuration was changed (notifications or indications
         * enabled or disabled). */
        if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
            && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)) {
          if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02) {
            /* Indications have been turned ON - start the repeating timer. The 1st parameter '32768'
             * tells the timer to run for 1 second (32.768 kHz oscillator), the 2nd parameter is
             * the timer handle and the 3rd parameter '0' tells the timer to repeat continuously until
             * stopped manually.*/
            gecko_cmd_hardware_set_soft_timer(32768, 0, 0);
          } else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x00) {
            /* Indications have been turned OFF - stop the timer. */
            gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          }
        }
        break;

      /* This event is generated when the software timer has ticked. In this example the temperature
       * is read after every 1 second and then the indication of that is sent to the listening client. */
      case gecko_evt_hardware_soft_timer_id:
        /* Measure the temperature as defined in the function temperatureMeasure() */
        temperatureMeasure();
        break;

      case gecko_evt_le_connection_closed_id:
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Stop timer in case client disconnected before indications were turned off */
          gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        }
        break;

      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Checks if the user-type OTA Control Characteristic was written.
       * If written, boots the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:
        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      default:
        break;
    }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
