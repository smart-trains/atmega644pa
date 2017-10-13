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
 * System libraries.
 */
#include "ASF.h"
#include "sfr_defs.h"

#include "iom644pa.h"


/*
 * IC libraries.
 */
#include "sc18is600.h"

/*
 * Project libraries.
 */
#include "base.h"
#include "functions.h"


// Initialisations.
void init() {
    ioport_init();
	
	SPI_MasterInit();
    delay_ms(40);
    ioport_set_pin_dir(LED, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(LED, IOPORT_PIN_LEVEL_HIGH);

    ioport_set_pin_level(SS, IOPORT_PIN_LEVEL_HIGH);
    delay_ms(40);

  
    SC_init();
    delay_ms(40);
    init_MPU6050();

    delay_ms(40);
}

// Initialises SPI bus.
void SPI_MasterInit(void) {
    // Set MOSI and SCK output, all others input.
    ioport_set_pin_dir(MISO, IOPORT_DIR_INPUT);
    ioport_set_pin_dir(MOSI, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(SCK, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(SS, IOPORT_DIR_OUTPUT);

    // Enable SPI, Master, set clock rate fck/128.
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0) | _BV(SPR1) ;
}

// Send and read data from SPI bus.
byte SPI_MasterTransmit(byte cData) {
    // Start transmission
    SPDR = cData;

    // Wait for transmission complete
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}
byte testfuck;
// SC reads N bytes from sensors into its receiver buffer.
void SC_read_I2C(uint8_t num_of_bytes, byte slave_addr) {
    // Chip select SC18IS600.
    SC_chip_select();

    // First commend byte for reading from I2C-bus slave device.
    SPI_MasterTransmit(SC18IS600_CMD_RDBLK);
    // Second commend byte that specifies the number of bytes to read.
    testfuck = SPI_MasterTransmit(num_of_bytes);
    // Thrid commend byte that specifies the slave address.
    SPI_MasterTransmit(SC_get_read_address(slave_addr));

    // Unselect sc18.
    SC_chip_unselect();
}

// Reads the receiver buffer from SC18IS600
void MCU_SC_read_buffer(uint8_t num_of_bytes, byte data[]) {
    // Chip select SC18IS600.
    SC_chip_select();

    // Actually reading the buffer.
    SPI_MasterTransmit(SC18IS600_CMD_RDBUF);

    // Construct an array to host the returned data.
    byte temp;

    uint8_t i = 0;
    for (i = 0; i < num_of_bytes; i++) {
        // Dummy data is sent for SPI read.
        temp = SPI_MasterTransmit(0);
        data[i] = temp;                                 
    }
	byte testt = data[0];
	byte testt2 = data[1];

    // Unselect sc18.
    SC_chip_unselect();
}


// Writes data to slave through SC18IS600
void MCU_SC_write(byte address, uint8_t num_of_bytes, byte data[]) {
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
void SC_set_register(byte reg_address, byte value) {    //set sc18 register
    // Chip select SC18IS600.
    SC_chip_select();

    // Commend for writing to SC18IS600 internal register.
    SPI_MasterTransmit(SC18IS600_CMD_WRREG);
    // Select which register to write.
    SPI_MasterTransmit(reg_address);
    // Write the value to the specified address.
    SPI_MasterTransmit(value);

    // Unselect sc18.
    SC_chip_unselect();
}

// TODO: SC18IS600 Initialisation
void SC_init(void) {
    // Chip select SC18IS600.
    ioport_set_pin_dir(INT_SC, IOPORT_DIR_INPUT);
	
	ioport_set_pin_dir(IO4_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(IO4_SC, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_dir(RST_SC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(RST_SC, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(RST_SC, IOPORT_PIN_LEVEL_HIGH);


    SC_chip_select();

    // Set clock decimal to 32. Set clock rate fck/128.
    SC_set_register(SC18IS600_I2CCLOCK, 32);
    // Set timeout as 1/2 second.
    SC_set_register(SC18IS600_I2CTO, 0b01111111);
	
	// Set timeout as 1/2 second.
    SC_set_register(SC18IS600_I2CADR, 0b01010101);

    // Unselect sc18.
    SC_chip_unselect();
}


// MPU6050 Initial
void init_MPU6050(void) {
    byte PW[] = {0x6B, 0x00};
    MCU_SC_write(0b1101000, 2, PW);

    byte GC[] = {0x1B, 0x00};
    MCU_SC_write(0b1101000, 2, GC);

    byte AC[] = {0x1C, 0x00};
    MCU_SC_write(0b1101000, 2, AC);

    // The working example from Arudino:
    //  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
    //  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
    //  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
    //  Wire.endTransmission();
    //  Wire.beginTransmission(0b1101000); //I2C address of the MPU
    //  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
    //  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
    //  Wire.endTransmission();
    //  Wire.beginTransmission(0b1101000); //I2C address of the MPU
    //  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
    //  Wire.write(0b00000000); //Setting the accel to +/- 2g
    //  Wire.endTransmission();
}

void MPU_6050_read(void) {
    uint8_t bytes_to_read = 6;

    bool interrupt = ioport_get_pin_level(INT_SC);
    long accelX, accelY, accelZ;
    float gForceX, gForceY, gForceZ;

    long gyroX, gyroY, gyroZ;
    float rotX, rotY, rotZ;

    byte AR[] = {0x3B};
    MCU_SC_write(0b1101000, 1, AR);
//	Wire.beginTransmission(0b1101000); //I2C address of the MPU
//	Wire.write(0x3B); //Starting register for Accel Readings
//	Wire.endTransmission();
    SC_read_I2C(bytes_to_read, 0b1101000);
//	Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)

	delay_ms(10);
//	while (ioport_get_pin_level(INT_SC));
//	while(Wire.available() < 6);
    byte data[6] = {0};
    MCU_SC_read_buffer(bytes_to_read, data);

    accelX = data[0] << 8 | data[1]; //Store first two bytes into accelX
    accelY = data[2] << 8 | data[3]; //Store middle two bytes into accelY
    accelZ = data[4] << 8 | data[5]; //Store last two bytes into accelZ
    gForceX = accelX / 16384.0f;
    gForceY = accelY / 16384.0f;
    gForceZ = accelZ / 16384.0f;    //processAccelData
}

// Chip select SC18IS600.
void SC_chip_select(void) {
    ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(CS_SC, IOPORT_PIN_LEVEL_LOW);
}

// Chip unselect SC18IS600.
void SC_chip_unselect(void) {
    ioport_set_pin_level(CS_SC, IOPORT_PIN_LEVEL_HIGH);
}

// SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
byte SC_get_read_address(byte address) {
    return address | (byte) 0b00000001;
}

// SC18IS600 ignore the least significant bit of slave address and set it to 0 to write
byte SC_get_write_address(byte address) {
    return address & (byte) 0b11111110;
}

int main(void) {
    /* Insert system clock initialization code here (sysclk_init()). */
    //	clock();
    //	TCCR0A

    init();

    //	SC_read_I2C (50, 0x00000000);
    while (1) {
        ioport_toggle_pin_level(LED);
        delay_ms(500);
        ioport_toggle_pin_level(LED);
        delay_ms(500);
        MPU_6050_read();
        delay_ms(500);
    }
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