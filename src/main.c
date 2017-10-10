/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include "IncFile1.h"
#include <asf.h>
#include "ioport.h"
#include "delay.h"
#include "time.h"
#include "iom644pa.h"
#include "sfr_defs.h"
#include <string.h>
#include <stdlib.h>
#include "macros.h"
#include "sc18is600.h"

//SPI
void SPI_MasterInit(void) {
	ioport_set_pin_dir( MOSI,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir( SCK,  IOPORT_DIR_OUTPUT);// Set MOSI and SCK output, all others input
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); //Enable SPI, Master, set clock rate fck/16
}

char SPI_MasterTransmit(char cData) {	//send message
	SPDR = cData; // Start transmission
	while(!( SPSR & _BV(SPIF) )); // Wait for transmission complete
	return SPDR;
}

/* SC18IS750 version
char SPI_send (char byte) { // mcu sends a byte to spi bus
	SPDAT = byte; // data is sent
	while(!SPI_tx_completed); // wait end of transmission
	SPSTAT &= ~0x80; // clear mcu spi interrupt flag (SPIF)
	SPI_tx_completed = 0; // clear transmit spi interrupt flag
	return SPDAT; // receive data on spi read
}
*/

void SC_read_I2C2 (char numofbytes, char slaveaddr) { // mcu reads a register from SC16IS750
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18	
	SPI_MasterTransmit(SC18IS600_CMD_RDBLK);	//Read N bytes from I2C-bus slave device
	SPI_MasterTransmit(numofbytes); // number of bytes to read
	slaveaddr = slaveaddr | 0b00000001;	//SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
	SPI_MasterTransmit(slaveaddr); // slave address
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
}

char SPI_read_buf (char register) { // mcu reads a register from SC16IS600
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
	SPI_MasterTransmit(SC18IS600_CMD_RDBUF);	//Read buffer
	register = SPI_MasterTransmit(0); // dummy data is sent for spi read
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
	return register; // receive the read data
}

void SPI_write (char address, char numofbytes, char[] data) { // mcu writes data to slave through SC16IS600
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
	SPI_MasterTransmit(SC18IS600_CMD_WRBLK);	//Write N bytes to I2C-bus slave device
	SPI_MasterTransmit(numofbytes); //number of bytes to write
	address = address & 0b11111110;	//SC18IS600 ignore the least significant bit of slave address and set it to 0 to write
	SPI_MasterTransmit(address); //slave address
	for (int i = 0; i < numofbytes; i++) {
		SPI_MasterTransmit(data[i]);
	}
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
}


void SC18_set_rgst(char reg, char cmd) {
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
	SPI_MasterTransmit(SC18IS600_CMD_WRREG); // Write to sc18 internal register
	SPI_MasterTransmit(reg); // register select
	SPI_MasterTransmit(cmd); // command
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
}

// SC18IS800 Initial (unfinished)
void init_SC16IS600 (void) {
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
	SC18_set_rgst(SC18IS600_I2CCLOCK,5);	//set clock decimal as 5
	SC18_set_rgst(SC18IS600_I2CTO,0b01111111);	//set time out as half osc
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
}


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	
//	clock();
	
//	TCCR0A
	
	SPI_MasterInit();
	
	ioport_init();
	ioport_set_pin_dir	( LED,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_level( LED,	IOPORT_PIN_LEVEL_HIGH);
	
	while(1){
		ioport_toggle_pin_level(LED);
		_delay_ms(200);
		ioport_toggle_pin_level(LED);
		_delay_ms(200);			
	}
	
	/* Insert application code here, after the board has been initialized. */
}

/*
int main(void) {
	BAT_LOW_LED(0X00); //Make sure it is off before changing direction
	_delay_ms(10);
	BAT_LOW_LED_DIR(0xFF);//Set BATTERY LED I/Os as outputs.
	BAT_LOW_LED(0xFF);
	_delay_ms(500);
}
*/
