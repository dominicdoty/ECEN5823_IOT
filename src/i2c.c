// I2C Setup/Use Routine Source
// Author D.Doty
// Compiled in Simplicity IDE

//***********************************************************************************
// Include files
//***********************************************************************************
#include "src/i2c.h"

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
 * Initializes the I2C bus (pins defined in header)
 * Assumes HF clock is enabled, checks/enables HFPERCLK
 * Configures I2C pins (but does not enable them, to allow manual toggling)
 * Sets TXBIL flag to happen on half full TX register (useful since we send 3 bytes, 2 byte register)
*/
void i2c_init()
{
	//make sure clock branch is on
	if (CMU_ClockFreqGet(cmuClock_HFPER)==0)
	{
		CMU_ClockEnable(cmuClock_HFPER, true);
	}

	//enable I2C clock
	CMU_ClockEnable(cmuClock_I2C0, true);

	//configure I2C pins
	GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeWiredAndPullUpFilter, I2C_SDA_DEFAULT);
	GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeWiredAndPullUpFilter, I2C_SCL_DEFAULT);

	//configure I2C registers
	I2C0->ROUTELOC0 |= I2C_SDA_LOC;
	I2C0->ROUTELOC0 |= (I2C_SCL_LOC << _I2C_ROUTELOC0_SCLLOC_SHIFT);
	I2C0->CTRL |= I2C_CTRL_TXBIL_HALFFULL;	//set the tx buffer flag to change when buffer is half empty
	I2C0->CTRL |= I2C_CTRL_CLHR_FAST;
	I2C0->CLKDIV = I2C_CLOCK_DIV;

	//configure I2C interrupts
	NVIC_EnableIRQ(I2C0_IRQn);
}

/*
 * Toggles I2C bus to reset slaves
 * Enables I2C pin connections to peripheral
 * Enables I2C
 * Sends Abort to clear bus
 * Blocks EM2
*/
void i2c_start_bus()
{
	// Toggle the clock pin to reset all the slaves
	for(int i=0; i<9; i++)
	{
		GPIO_PinOutClear(I2C_SCL_PORT, I2C_SCL_PIN);
		GPIO_PinOutSet(I2C_SCL_PORT, I2C_SCL_PIN);
	}
	I2C0->ROUTEPEN = (I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN);
	I2C0->CTRL |= I2C_CTRL_EN;			//enable I2c
	I2C0->CMD = I2C_CMD_ABORT;			//abort makes bus not busy
	blockSleepMode(EM2);
}

/*
 * Sends the given slave a read command for the given address:
 * START
 * SLAVE_ADDR + W
 * COMMAND
 * RSTART
 * SlAVE_ADDR + R
 * Enables RX data interrupt (to accomodate ACKing)
 * Does not wait for response
*/
void i2c_read_command(uint8_t slave_address, uint8_t command)
{
	I2C0->CMD = I2C_CMD_START;							//start
	I2C0->TXDATA = (slave_address << 1);				//send address with W bit
	I2C0->TXDATA = command;								//send command
	while(!(I2C0->STATUS & I2C_STATUS_TXC));			//idle till TX complete
	I2C0->CMD = I2C_CMD_START;							//restart
	I2C0->TXDATA = (slave_address << 1) | I2C_RW_BIT;	//send address with R bit
	I2C0->IEN |= I2C_IEN_RXDATAV;						//enable recieve interrupt
}

/*
 * Sends the given slave a written command not expecting a response:
 * START
 * SLAVE_ADDR + W
 * COMMAND
 * STOP
*/
void i2c_command(uint8_t slave_address, uint8_t command)
{
	I2C0->CMD = I2C_CMD_START;						//start
	I2C0->TXDATA = (slave_address << 1);			//send address with W bit
	I2C0->TXDATA = command;							//send command
	while(~(I2C0->STATUS & I2C_STATUS_TXC));		//idle till TX done
	I2C0->CMD = I2C_CMD_STOP;						//stop
}

/*
 * Read temperature from the RX buffer (2 bytes) with an ACK,NACK,STOP:
 * READ BYTE 1
 * ACK
 * READ BYTE 2
 * NACK
 * STOP
 * Convert read bytes to temperature in C
*/
float i2c_read_temp()
{
	uint16_t reading = (I2C0->RXDATA) << 8;				//read the MSB
	I2C0->CMD = I2C_CMD_ACK;							//ack it
	while(!(I2C0->STATUS & I2C_STATUS_RXDATAV));		//wait for new data
	reading |= I2C0->RXDATA;							//read the LSB
	I2C0->CMD = I2C_CMD_NACK;							//nack it
	while(I2C0->STATUS & I2C_STATUS_PNACK);				//wait for nack
	I2C0->CMD = I2C_CMD_STOP;							//send stop
	float temp = (((reading*175.72)/65536) - 46.85);	//convert to temp while waiting
	while(I2C0->STATUS & I2C_STATUS_PSTOP);				//wait for stop (if necessary)
	return temp;										//return temperature
}

/*
 * Stop the bus by issuing an abort, disabling I2C, and disabling the pins
 * pins are diabled so they're ready for manual toggling next time
*/
void i2c_stop_bus()
{
	I2C0->CMD = I2C_CMD_ABORT;
	I2C0->CTRL &= ~I2C_CTRL_EN;
	I2C0->ROUTEPEN = 0;
	unblockSleepMode(EM2);
}
