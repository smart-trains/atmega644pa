void init(void);
void MCU_init(void);
void LED_init(void);
void init_MPU6050(void);
void SC_init(void);
void SPI_MasterInit(void);

byte SPI_MasterTransmit(byte cData);

void SC_chip_select(void);
void SC_chip_unselect(void);
void SC_set_register(byte register, byte value);
byte SC_read_register(byte register);
void SC_write_I2C(byte address, uint8_t num_of_bytes, byte data[]);
void SC_read_I2C(uint8_t num_of_bytes, byte slave_addr);
void SC_read_buffer(uint8_t num_of_bytes, byte data[]);

byte SC_get_read_address(byte address);
byte SC_get_write_address(byte address);

void MPU_6050_read(void);