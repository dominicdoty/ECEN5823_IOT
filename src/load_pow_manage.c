// LPM Setup/Use Routine Source
// Author D.Doty
// Compiled in Simplicity IDE

//***********************************************************************************
// Include files
//***********************************************************************************
#include "src/load_pow_manage.h"

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

void temp_sensor_init()
{
	//verify the clock tree has been set up, jam program if not
	ASSERT(CMU_ClockFreqGet(cmuClock_GPIO)!=0);

	//set drive strength
	GPIO_DriveStrengthSet(LOAD_PORT, gpioDriveStrengthStrongAlternateStrong);
	//set pin mode push pull
	GPIO_PinModeSet(LOAD_PORT, LOAD_PIN, gpioModePushPull, LOAD_DEFAULT);
}

void temp_sensor_on()
{
	//turn on power to the pin
	GPIO_PinOutSet(LOAD_PORT, LOAD_PIN);
}

void temp_sensor_off()
{
	//turn off power to the pin
	GPIO_PinOutClear(LOAD_PORT, LOAD_PIN);
}
