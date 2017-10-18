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
 * RS485 libraries.
 */
#include "RS485.h"


/*
 * IC libraries.
 */
#include "sc18is600.h"

/*
 * Project libraries.
 */
#include "base.h"
#include "functions.h"

//Cherry
#include <util/twi.h>
#include <htu21d.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <i2c_master.h>
#include <AMG8853.h>


// Initialisations.
void init() {
    MCU_init();
    delay_ms(40);

    LED_init();
    delay_ms(40);

    SPI_MasterInit();
    delay_ms(40);

    SC_init();
    delay_ms(40);

    init_MPU6050();
    delay_ms(40);
    
    HTU21D_init();
	delay_ms(40);
	
	I2C_init();
	delay_ms(40);
	
	interruptInit();
	
	AMG8853_init();
	delay_ms(40);
}

// MCU initialisations.
void MCU_init(void) {
    ioport_init();
    delay_ms(40);

    // Disable the SS pin to this MCU.
    ioport_set_pin_dir(SS, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(SS, IOPORT_PIN_LEVEL_HIGH);
}

// LED initialisations.
void LED_init(void) {
    ioport_set_pin_dir(LED, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(LED, IOPORT_PIN_LEVEL_HIGH);
}

// Initialises SPI bus.
void SPI_MasterInit(void) {
    // Set MOSI, SCK and CS_SC output, all others input.
    ioport_set_pin_dir(MISO, IOPORT_DIR_INPUT);
    ioport_set_pin_dir(MOSI, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(SCK, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);

    // Enable SPI as Master, set clock rate fck/128, and operate as SPI mode 3.
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(CPHA) | _BV(SPR1) | _BV(SPR0) ;
}

//Interrupt
void interruptInit(void) {
	sei();	// set global interrupt enable
	SREG = _BV(INT0); //Interrupt 0 .  PD2
	MCUCR = _BV(ISC00)|_BV(ISC01); //rising edge interrupt
}

// TODO: SC18IS600 Initialisation
void SC_init(void) {
    // Chip select SC18IS600.
    ioport_set_pin_dir(INT_SC, IOPORT_DIR_INPUT);

    ioport_set_pin_dir(IO4_SC, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(IO4_SC, IOPORT_PIN_LEVEL_LOW);

    ioport_set_pin_dir(RST_SC, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(RST_SC, IOPORT_PIN_LEVEL_LOW);
    delay_ms(100);
    ioport_set_pin_level(RST_SC, IOPORT_PIN_LEVEL_HIGH);


    SC_chip_select();

    // Set clock decimal to 32. Set clock rate fck/128.
    SC_set_register(SC18IS600_I2CCLOCK, 32);
    // Set timeout as 1/2 second.
    SC_set_register(SC18IS600_I2CTO, 0b01111111);

    // Set SC address.
    SC_set_register(SC18IS600_I2CADR, 0b01010101);
}

// MPU6050 Initial
void init_MPU6050(void) {
    byte PW[] = {0x6B, 0x00};
    SC_write_I2C(0b1101000, 2, PW);

    byte GC[] = {0x1B, 0x00};
    SC_write_I2C(0b1101000, 2, GC);

    byte AC[] = {0x1C, 0x00};
    SC_write_I2C(0b1101000, 2, AC);

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

<<<<<<< HEAD
void RS485_init(void) {
	// set USART Baud Rate 0 Register Low and High byte
	UBRR0H = (BRC >> 8);
	UBRR0L = BRC;
	// USART Transmitter Enable; TX Complete Interrupt Enable;
	// USART Receiver Enable; RX Complete Interrupt Enable.
	UCSR0B = _BV(TXEN0) | _BV(TXCIE0) | _BV(RXEN0) | _BV(RXCIE0);
	// Set Character Size to 8-bit
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
	// set direction of XCK for USART
	ioport_set_pin_dir(IO5_SC, IOPORT_DIR_OUTPUT);
	// enable interrupt
	sei();
}

=======
>>>>>>> b418d85ecf606962e213682204f29bca394ef25d
// Send and read data from SPI bus.
byte SPI_MasterTransmit(byte cData) {
    // Start transmission
    SPDR = cData;

    // Wait for transmission complete
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}

// SC reads N bytes from sensors into its receiver buffer.
void SC_read_I2C(uint8_t num_of_bytes, byte slave_addr) {
    // First commend byte for reading from I2C-bus slave device.
    SPI_MasterTransmit(SC18IS600_CMD_RDBLK);
    // Second commend byte that specifies the number of bytes to read.
    SPI_MasterTransmit(num_of_bytes);
    // Thrid commend byte that specifies the slave address.
    SPI_MasterTransmit(SC_get_read_address(slave_addr));
}

// Reads the receiver buffer from SC18IS600
void SC_read_buffer(uint8_t num_of_bytes, byte data[]) {
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
}


// Writes data to slave through SC18IS600
void SC_write_I2C(byte address, uint8_t num_of_bytes, byte data[]) {
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
}

// Reads SC18IS600 registers.
byte SC_read_register(byte reg_address) {
    // Commend for writing to SC18IS600 internal register.
    SPI_MasterTransmit(SC18IS600_CMD_RDREG);
    // Select which register to write.
    SPI_MasterTransmit(reg_address);
    // Write a dummy data to receive the returned data.
    return SPI_MasterTransmit((byte) 0x00);
}

// Sets SC18IS600 registers.
void SC_set_register(byte reg_address, byte value) {
    // Commend for writing to SC18IS600 internal register.
    SPI_MasterTransmit(SC18IS600_CMD_WRREG);
    // Select which register to write.
    SPI_MasterTransmit(reg_address);
    // Write the value to the specified address.
    SPI_MasterTransmit(value);
}

void MPU_6050_read(void) {
    uint8_t bytes_to_read = 6;

    bool interrupt = ioport_get_pin_level(INT_SC);
    long accelX, accelY, accelZ;
    float gForceX, gForceY, gForceZ;

    long gyroX, gyroY, gyroZ;
    float rotX, rotY, rotZ;

    byte AR[] = {0x3B};
    SC_write_I2C(0b1101000, 1, AR);
//	Wire.beginTransmission(0b1101000); //I2C address of the MPU
//	Wire.write(0x3B); //Starting register for Accel Readings
//	Wire.endTransmission();
    SC_read_I2C(bytes_to_read, 0b1101000);
//	Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)

	delay_ms(10);
//	while (ioport_get_pin_level(INT_SC));
//	while(Wire.available() < 6);
    byte data[6] = {0};
    SC_read_buffer(bytes_to_read, data);

    accelX = data[0] << 8 | data[1]; //Store first two bytes into accelX
    accelY = data[2] << 8 | data[3]; //Store middle two bytes into accelY
    accelZ = data[4] << 8 | data[5]; //Store last two bytes into accelZ
    gForceX = accelX / 16384.0f;
    gForceY = accelY / 16384.0f;
    gForceZ = accelZ / 16384.0f;    //processAccelData
}

// Chip select SC18IS600.
void SC_chip_select(void) {
    ioport_set_pin_level(CS_SC, IOPORT_PIN_LEVEL_LOW);
}

// Chip unselect SC18IS600.
void SC_chip_unselect(void) {
    ioport_set_pin_level(CS_SC, IOPORT_PIN_LEVEL_HIGH);
}

// SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
byte SC_get_read_address(byte address) {
	address << 1;
    return address | (byte) 0b00000001;
}

// SC18IS600 ignore the least significant bit of slave address and set it to 0 to write
byte SC_get_write_address(byte address) {
	address << 1;
    return address & (byte) 0b11111110;
}

byte grid_message[65]={0};
int main(void) {
    /* Insert system clock initialization code here (sysclk_init()). */
    //	clock();
    //	TCCR0A

//    init();
//	int xx=0;
//	xx=AMG8853_therm_temp();

	


    //	SC_read_I2C (50, 0x00000000);
    while (1) {
        ioport_toggle_pin_level(LED);
        delay_ms(500);
        ioport_toggle_pin_level(LED);
        delay_ms(500);
//        MPU_6050_read();
//        delay_ms(500);
		
		int xx=0;
		xx=AMG8853_therm_temp();
		delay_ms(10);
		AMG8853_generate_message(grid_message);
//		delay_ms(1000);
		
		
		
		
    }
}


/*
// SC18IS750 version
char SPI_send (char byte) { // mcu sends a byte to spi bus
	SPDAT = byte; // data is sent
	while(!SPI_tx_completed); // wait end of transmission
	SPSTAT &= ~0x80; // clear mcu spi interrupt flag (SPIF)
	SPI_tx_completed = 0; // clear transmit spi interrupt flag
	return SPDAT; // receive data on spi read
}
*/

// TODO: the checking loop may be changed
void SC_read_after_write(uint8_t numofwrite, uint8_t numofread, uint8_t slaveaddr,
                         char data1[]) { // mcu reads slace after write
    ioport_set_pin_dir(CS_SC, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(CS_SC, IOPORT_PIN_LEVEL_LOW);    //select sc18
    SPI_MasterTransmit(SC18IS600_CMD_RDAWR);    //Read N bytes from I2C-bus slave device
    SPI_MasterTransmit(numofwrite); // number of bytes to write
    SPI_MasterTransmit(numofread); // number of bytes to read
    slaveaddr = slaveaddr |
                0b00000001;    //SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
    SPI_MasterTransmit(slaveaddr); // slave address
    for (int i = 0; i < numofwrite; i++) {
        SPI_MasterTransmit(data1[i]);
    }
    slaveaddr = slaveaddr |
                0b00000001;    //SC18IS600 ignore the least significant bit of slave address and set it to 1 to read
    while (ioport_get_pin_level(INT_SC));    //TODO: maybe should change here, watch the register state
    ioport_set_pin_level(CS_SC, IOPORT_PIN_LEVEL_HIGH);    //unselect sc18
}


//Cherry
#define F_CPU 16000000
#define F_SCL 100000// SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

//I2C
void I2C_init(void)
{
	ioport_set_pin_dir(SCL, IOPORT_DIR_OUTPUT);
	TWBR = (uint8_t)TWBR_val;
	//TWSR = 0;
	TWCR = _BV(TWEN);
}

uint8_t I2C_Start(uint8_t Device_ADDRESS)
{
	//reset TWI control register
	TWCR = 0;
	// transmit START condition 
	TWCR = _BV(TWEN) | _BV(TWSTA) | _BV(TWINT) | _BV(TWEA);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the start condition was successfully transmitted
	if((TWSR & 0xF8) != TW_START){ 
		return 1;
	 }
	
	// load slave address into data register
	TWDR = Device_ADDRESS;
	// start transmission of address
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	//TWCR = TWCR & 0b11011111;
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) );
	uint8_t twst =TWSR & 0xF8;
	// check if the device has acknowledged the READ / WRITE mode	uint8_t twst = TWSR & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) {
		return 1;
		}
	/*if ((TWSR & 0xF8) != TW_MT_SLA_ACK) {
		return 1;}*/
	
	return 0;
}

void I2C_Stop(void)
{
	// transmit STOP condition
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

uint8_t I2C_Write(uint8_t DATA) {
	/*TWCR = _BV(TWEN) | _BV(TWSTA) | _BV(TWINT);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the start condition was successfully transmitted
	if((TWSR & 0xF8) != TW_START){
		return 1;
	}*/
	// load data into data register
	TWDR = DATA;
	// start transmission of data
	TWCR = _BV(TWINT) | _BV(TWEN);
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) );
	
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
	return 0;
}

uint8_t I2C_READ_ACK(void) {
	// start TWI module and acknowledge data after reception
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA); 
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) );
	// return received data from TWDR
	return TWDR;
}

uint8_t I2C_READ_NACK(void) {
	// start receiving without acknowledging reception
	TWCR = _BV(TWINT) | _BV(TWEN);
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) );
	// return received data from TWDR
	return TWDR;
}


uint8_t I2C_Write_register(uint8_t devaddr, uint8_t regaddr, uint8_t* DATA, uint16_t length)
{
	if (I2C_Start(devaddr & 0b11111110)) return 1;

	I2C_Write(regaddr);
	/*// load data into data register
	TWDR = regaddr;
	// start transmission of data
	TWCR = _BV(TWINT) | _BV(TWEN);
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) );
	
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }*/

	for (uint16_t i = 0; i < length; i++)
	{
		if (I2C_Write(DATA[i])) return 1;
	}

	I2C_Stop();

	return 0;
}

uint8_t I2C_Read_register(uint8_t devaddr, uint8_t regaddr, uint8_t* DATA, uint16_t length)
{
	if (I2C_Start(devaddr & 0b11111110)) return 1;

	I2C_Write(regaddr);
	/*// load data into data register
	TWDR = regaddr;
	// start transmission of data
	TWCR = _BV(TWINT) | _BV(TWEN);
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) );
	
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }*/

	if (I2C_Start(devaddr | 0x01)) return 1;

	for (uint16_t i = 0; i < (length-1); i++)
	{
		DATA[i] = I2C_READ_ACK();
	}
	DATA[(length-1)] = I2C_READ_NACK();

	I2C_Stop();

	return 0;
}

//Grid Eye AMG8853
void AMG8853_init(void){
	//ioport_set_pin_dir(SDA, IOPORT_DIR_OUTPUT);
	//reset AMG8853
	I2C_Write_register(AMG8853_address, REG_RST, 0x3f, 1);
	//set frame rate
	I2C_Write_register(AMG8853_address, REG_FPSC, 0x00, 1);
	
}

int AMG8853_therm_temp(void){
	byte buf[2]={2,4};
	//byte buf2[1];
	I2C_Read_register(AMG8853_address, REG_TOOL, buf, 2);
	//I2C_Read_register(AMG8853_address, REG_TOOH, buf2, 1);
	uint16_t AMG_temperature = ((buf[1] & 0x0f)<<8) | (buf[0]);
	if (AMG_temperature>2047){
		AMG_temperature=AMG_temperature-2048;
		AMG_temperature=-AMG_temperature;
	}
	return AMG_temperature;
}

void AMG8853_pixel_out(int *pixel_buffer){
	int temp;
	byte i, j, buffer[32];
	for(i=0;i<4;i++){
		I2C_Read_register(AMG8853_address,REG_PIXL+(i*0x20), buffer, sizeof(buffer)/sizeof(buffer[0]));
		for(j=0; j<32; j+=2){
			temp = ((buffer[j+1]&0x0f)<<8)|buffer[j];
			if (temp>2047){
				temp = temp-4096;
			}
			*pixel_buffer++ = temp;
		}
	}
}

void AMG8853_generate_message(byte message[]){
	//construct message
	message[MSG_ENV_INDEX]=AMG8853_therm_temp();
	AMG8853_pixel_out(grid_lv);
	for(int i=0; i<NUM_CELLS; i++){
		message[MSG_GRID_INDEX+i]=(byte) grid_lv[i];
	}
}

//HTU21D
void HTU21D_init(void){
	SC_write_I2C(HTU21D_Address, 1, SOFT_RESET);//soft reset the HTU21D
	delay_ms(15);//reset time 15ms
}

void HTU21D_set_resolution(char resolution){
	SC_set_register(HTU21D_Address, resolution);//write to register for setting resolution
}

void HTU21D_measure_temp(void){
	SC_write_I2C(HTU21D_Address, 1, MEASURE_TEMPERATURE);//send measure temperature command
	delay_ms(T11bit_measure_time);
}

uint8_t MSB=0;
uint8_t LSB=0;
float HTU21D_read_temp(void){
	SC_read_I2C(3, HTU21D_Address);//send read temperature command
//	uint8_t LSB, MSB=0;
	float t=0, TEMP=0;
	byte info[2]={1,1,1};
	SC_read_buffer(3, info);
	MSB = info[0];
	LSB = info[1];
	t=MSB*256+LSB;
	TEMP=175.72*t/65536-46.85;
//	return MSB;
}

