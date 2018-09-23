#ifndef I2C_H
#define I2C_H

// I2C Setup/Use Routine Source
// Author D.Doty
// Compiled in Simplicity IDE

//***********************************************************************************
// Include files
//***********************************************************************************

#include "main.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "em_core.h"
#include "src/sleep.h"
#include "em_cmu.h"


//***********************************************************************************
// defined files
//***********************************************************************************

// I2C pin definitions and default states
#define I2C_SCL_PORT	gpioPortC
#define I2C_SCL_PIN		10
#define I2C_SCL_LOC		14
#define I2C_SCL_DEFAULT	true

#define I2C_SDA_PORT	gpioPortC
#define I2C_SDA_PIN		11
#define I2C_SDA_LOC		16
#define I2C_SDA_DEFAULT	true

#define I2C_RW_BIT		0b00000001
#define I2C_CLOCK_DIV	4 	//9 works with standard pulse aspect ratio
							//4 with fast pulse aspect ratio produces ~400kHz I2C clock

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************

void i2c_init();

void i2c_start_bus();

void i2c_read_command(uint8_t slave_address, uint8_t command);

void i2c_command(uint8_t slave_address, uint8_t command);

float i2c_read_temp();

void i2c_stop_bus();

#endif
