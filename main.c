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
#include <stdio.h>
#include "ASF.h"
#include "time.h"
#include "ioport.h"
#include "sfr_defs.h"
#include <string.h>
#include <stdlib.h>
#include "macros.h"
#include "sc18is600.h"

bool interup;

//SPI
void SPI_MasterInit(void) {
	ioport_set_pin_dir( MOSI,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir( SCK,  IOPORT_DIR_OUTPUT);// Set MOSI and SCK output, all others input
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); //Enable SPI, Master, set clock rate fck/16
}

uint8_t SPI_MasterTransmit(char cData) {	//send message
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

void SC_read_I2C2 (uint8_t numofbytes, uint8_t slaveaddr) { // mcu reads a register from SC16IS600
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18	
	
	SPI_MasterTransmit(SC18IS600_CMD_RDBLK);	//Read N bytes from I2C-bus slave device
	SPI_MasterTransmit(numofbytes); // number of bytes to read
	slaveaddr = slaveaddr | 0b00000001;	//SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
	SPI_MasterTransmit(slaveaddr); // slave address
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
}

// the checking loop may be changed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void SC_read_af_write (uint8_t numofwrite, uint8_t numofread, uint8_t slaveaddr, char data1[]) { // mcu reads slace after write
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
	SPI_MasterTransmit(SC18IS600_CMD_RDAWR);	//Read N bytes from I2C-bus slave device
	SPI_MasterTransmit(numofwrite); // number of bytes to write
	SPI_MasterTransmit(numofread); // number of bytes to read
	slaveaddr = slaveaddr | 0b00000001;	//SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
	SPI_MasterTransmit(slaveaddr); // slave address
	for (int i = 0; i < numofwrite; i++) {
		SPI_MasterTransmit(data1[i]);
	}
	slaveaddr = slaveaddr | 0b00000001;	//SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
	while(ioport_get_pin_level(INT_SC));	//maybe should change here!!!!!!!!!! watch the register state
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
}

char SPI_read_buf (void) { // mcu reads a register from SC16IS600
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
	SPI_MasterTransmit(SC18IS600_CMD_RDBUF);	//Read buffer
	char readdata;
	readdata = SPI_MasterTransmit(0); // dummy data is sent for spi read
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
	return readdata; // receive the read data
}


void SPI_write (uint8_t address, int numofbytes, char data1[]) { // mcu writes data to slave through SC16IS600
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
	SPI_MasterTransmit(SC18IS600_CMD_WRBLK);	//Write N bytes to I2C-bus slave device
	SPI_MasterTransmit(numofbytes); //number of bytes to write
	address = address & 0b11111110;	//SC18IS600 ignore the least significant bit of slave address and set it to 0 to write
	SPI_MasterTransmit(address); //slave address
	for (int i = 0; i < numofbytes; i++) {
		SPI_MasterTransmit(data1[i]);
	}
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
}


void SC18_set_rgst(char reg, char cmd) {	//set sc18 register
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
	SPI_MasterTransmit(SC18IS600_CMD_WRREG); // Write to sc18 internal register
	SPI_MasterTransmit(reg); // register select
	SPI_MasterTransmit(cmd); // command
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
}

// SC18IS800 Initial (unfinished)
void init_SC18IS600 (void) {
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
	SC18_set_rgst(SC18IS600_I2CCLOCK,5);	//set clock decimal as 5
	SC18_set_rgst(SC18IS600_I2CTO,0b01111111);	//set time out as half osc
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
}


// MPU6050 Initial
void init_MPU6050 (void) {
	char PW[] = {0x6B, 0x00};
	SPI_write (0b1101000, 2, PW);
//  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
//  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
//  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
//  Wire.endTransmission();
	char GC[] = {0x1B, 0x00}; 
	SPI_write (0b1101000, 2, GC);
//  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
//  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
//  Wire.endTransmission();
	char AC[] = {0x1C, 0x00}; 
	SPI_write (0b1101000, 2, AC);	
//  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
//  Wire.write(0b00000000); //Setting the accel to +/- 2g
//  Wire.endTransmission();
}

void recordAccelRegisters() {
	char AR = 0x3B;
	SPI_write (0b1101000, 1, AR);
//	Wire.beginTransmission(0b1101000); //I2C address of the MPU
//	Wire.write(0x3B); //Starting register for Accel Readings
//	Wire.endTransmission();
	SC_read_I2C2 (6, 0b1101000);
//	Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
	interup = ioport_get_pin_level(INT_SC);
	while(ioport_get_pin_level(INT_SC));
//	while(Wire.available() < 6);
	accelX = SPI_read_buf()<<8|SPI_read_buf(); //Store first two bytes into accelX
	accelY = SPI_read_buf()<<8|SPI_read_buf(); //Store middle two bytes into accelY
	accelZ = SPI_read_buf()<<8|SPI_read_buf(); //Store last two bytes into accelZ
	gForceX = accelX / 16384.0;
	gForceY = accelY / 16384.0;
	gForceZ = accelZ / 16384.0;	//processAccelData
}



int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	
//	clock();
	
//	TCCR0A
	
	
	
	ioport_init();
	delay_ms(40);
	ioport_set_pin_dir	( LED,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_level( LED,	IOPORT_PIN_LEVEL_HIGH);
	
	ioport_set_pin_dir	( SPI_SS,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_level( SPI_SS,  IOPORT_PIN_LEVEL_HIGH);
	
	SPI_MasterInit();
	delay_ms(40);
	init_SC18IS600();
	delay_ms(40);
	init_MPU6050 ();
	delay_ms(40);
	
//	SC_read_I2C2 (50, 0x00000000);
	
	

	while(1){
		ioport_toggle_pin_level(LED);
		delay_ms(500);
		ioport_toggle_pin_level(LED);
		delay_ms(500);	
		recordAccelRegisters();
		delay_ms(500);
	}
	
	/* Insert application code here, after the board has been initialized. */
}


