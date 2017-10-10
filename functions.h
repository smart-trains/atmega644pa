void SPI_MasterInit(void);
byte SPI_MasterTransmit(byte cData);
void SC_chip_select(void);
void SC_chip_unselect(void);
void SC_read_I2C(uint8_t num_of_bytes, byte slave_addr);
void MCU_SC_read_buffer(uint8_t num_of_bytes, byte data[]);
void MCU_SC_write (byte address, int num_of_bytes, char data[]);
void SC_set_register(byte register, byte value);
void SC_init (void);
void init_MPU6050 (void);
void recordAccelRegisters();
byte SC_get_read_address(byte address);
byte SC_get_write_address(byte address);