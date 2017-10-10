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

// Initialises SPI bus.
void SPI_MasterInit(void) {
	// Set MOSI and SCK output, all others input.
	ioport_set_pin_dir(MOSI,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SCK,  IOPORT_DIR_OUTPUT);

	// Enable SPI, Master, set clock rate fck/16.
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
}

// Send and read data from SPI bus.
byte SPI_MasterTransmit(byte cData) {
	// Start transmission
	SPDR = cData;

	// Wait for transmission complete
	while(!( SPSR & _BV(SPIF) ));
	return SPDR;
}

// Chip select SC18IS600.
void SC_chip_select(void) {
	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);
}

// Chip unselect SC18IS600.
void SC_chip_unselect(void) {
	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);
}

// SC reads N bytes from sensors into its receiver buffer.
void SC_read_I2C(uint8_t num_of_bytes, byte slave_addr) {
	// Chip select SC18IS600.
	SC_chip_select();
	
	// First commend byte for reading from I2C-bus slave device.
	SPI_MasterTransmit(SC18IS600_CMD_RDBLK);
	// Second commend byte that specifies the number of bytes to read.
	SPI_MasterTransmit(num_of_bytes);
	// Thrid commend byte that specifies the slave address.
	SPI_MasterTransmit(SC_get_read_address(slave_addr));

	// Unselect sc18.
	SC_chip_unselect();
}

// Reads the receiver buffer from SC18IS600
char MCU_SC_read_buffer(uint8_t num_of_bytes) {
	// Chip select SC18IS600.
	SC_chip_select();

	// Actually reading the buffer.
	SPI_MasterTransmit(SC18IS600_CMD_RDBUF);

	// Construct an array to host the returned data.
	byte data[num_of_bytes];
	
	uint8_t i = 0;
	for (i = 0; i < num_of_bytes; i++) {
		// Dummy data is sent for SPI read.
		data[i] = SPI_MasterTransmit(0);
	}
	
	// Unselect sc18.
	SC_chip_unselect();

	// Return the received data.
	return data;
}

// Writes data to slave through SC18IS600
void MCU_SC_write (byte address, int num_of_bytes, char data[]) {
	// Chip select SC18IS600.
	SC_chip_select();

	// Commend for writing N bytes to I2C-bus slave device.
	SPI_MasterTransmit(SC18IS600_CMD_WRBLK);
	// Specify the number of bytes to write.
	SPI_MasterTransmit(num_of_bytes);
	// Specify the slave address to write.
	SPI_MasterTransmit(SC_get_write_address(address));

	// Go.
	for (int i = 0; i < num_of_bytes; i++) {
		SPI_MasterTransmit(data[i]);
	}
	
	// Unselect sc18.
	SC_chip_unselect();
}

// Sets SC18IS600 registers.
void SC_set_register(byte register, byte value) {	//set sc18 register
	// Chip select SC18IS600.
	SC_chip_select();

	// Commend for writing to SC18IS600 internal register.
	SPI_MasterTransmit(SC18IS600_CMD_WRREG);
	// Select which register to write.
	SPI_MasterTransmit(register);
	// Write the value to the sepcified address.
	SPI_MasterTransmit(value);
	
	// Unselect sc18.
	SC_chip_unselect();
}

// TODO: SC18IS600 Initialisation
void SC_init (void) {
	// Chip select SC18IS600.
	SC_chip_select();

	// Set clock decimal to 5.
	SC_set_register(SC18IS600_I2CCLOCK,5);
	// Set timeout as 1/2 second.
	SC_set_register(SC18IS600_I2CTO,0b01111111);
	
	// Unselect sc18.
	SC_chip_unselect();
}


// MPU6050 Initial
void init_MPU6050 (void) {
	char PW[] = {0x6B, 0x00};
	MCU_SC_write (0b1101000, 2, PW);
//  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
//  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
//  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
//  Wire.endTransmission();
	char GC[] = {0x1B, 0x00};
	MCU_SC_write (0b1101000, 2, GC);
//  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
//  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
//  Wire.endTransmission();
	char AC[] = {0x1C, 0x00};
	MCU_SC_write (0b1101000, 2, AC);
//  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
//  Wire.write(0b00000000); //Setting the accel to +/- 2g
//  Wire.endTransmission();
}

void recordAccelRegisters() {
	char AR = 0x3B;
	MCU_SC_write (0b1101000, 1, AR);
//	Wire.beginTransmission(0b1101000); //I2C address of the MPU
//	Wire.write(0x3B); //Starting register for Accel Readings
//	Wire.endTransmission();
	SC_read_I2C (6, 0b1101000);
//	Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
	interup = ioport_get_pin_level(INT_SC);
	while(ioport_get_pin_level(INT_SC));
//	while(Wire.available() < 6);
	accelX = MCU_SC_read_buffer()<<8|MCU_SC_read_buffer(); //Store first two bytes into accelX
	accelY = MCU_SC_read_buffer()<<8|MCU_SC_read_buffer(); //Store middle two bytes into accelY
	accelZ = MCU_SC_read_buffer()<<8|MCU_SC_read_buffer(); //Store last two bytes into accelZ
	gForceX = accelX / 16384.0;
	gForceY = accelY / 16384.0;
	gForceZ = accelZ / 16384.0;	//processAccelData
}

// SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
byte SC_get_read_address(address) {
	return address | 0b00000001;
}

// SC18IS600 ignore the least significant bit of slave address and set it to 0 to write
byte SC_get_write_address(address) {
	return address & 0b11111110;
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
	init_MPU6050();
	delay_ms(40);
	
//	SC_read_I2C (50, 0x00000000);
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


/* SC18IS750 version
char SPI_send (char byte) { // mcu sends a byte to spi bus
	SPDAT = byte; // data is sent
	while(!SPI_tx_completed); // wait end of transmission
	SPSTAT &= ~0x80; // clear mcu spi interrupt flag (SPIF)
	SPI_tx_completed = 0; // clear transmit spi interrupt flag
	return SPDAT; // receive data on spi read
}
*/

// // the checking loop may be changed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// void SC_read_after_write(uint8_t numofwrite, uint8_t numofread, uint8_t slaveaddr, char data1[]) { // mcu reads slace after write
// 	ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_LOW);	//select sc18
// 	SPI_MasterTransmit(SC18IS600_CMD_RDAWR);	//Read N bytes from I2C-bus slave device
// 	SPI_MasterTransmit(numofwrite); // number of bytes to write
// 	SPI_MasterTransmit(numofread); // number of bytes to read
// 	slaveaddr = slaveaddr | 0b00000001;	//SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
// 	SPI_MasterTransmit(slaveaddr); // slave address
// 	for (int i = 0; i < numofwrite; i++) {
// 		SPI_MasterTransmit(data1[i]);
// 	}
// 	slaveaddr = slaveaddr | 0b00000001;	//SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
// 	while(ioport_get_pin_level(INT_SC));	//maybe should change here!!!!!!!!!! watch the register state
// 	ioport_set_pin_level(CS_SC,	IOPORT_PIN_LEVEL_HIGH);	//unselect sc18
// }

