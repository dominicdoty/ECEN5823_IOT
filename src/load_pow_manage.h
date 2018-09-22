#ifndef LOAD_POW_MANAGE_H
#define LOAD_POW_MANAGE_H

// LPM Setup/Use Routine Source
// Author D.Doty
// Compiled in Simplicity IDE

//***********************************************************************************
// Include files
//***********************************************************************************

#include "main.h"
#include "em_core.h"
#include "em_gpio.h"
#include "em_cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************

#define LOAD_PIN		15
#define LOAD_PORT		gpioPortD
#define LOAD_DEFAULT	false

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************

void temp_sensor_init();

void temp_sensor_on();

void temp_sensor_off();

#endif
