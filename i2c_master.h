#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#define I2C_READ 0x01
#define I2C_WRITE 0x00

void I2C_init(void);
uint8_t I2C_Start(uint8_t SLA_ADDRESS);
uint8_t I2C_Write(uint8_t DATA);
uint8_t I2C_READ_ACK(void);
uint8_t I2C_READ_NACK(void);
uint8_t I2C_Transmit(uint8_t SLA_Address, uint8_t* DATA, uint16_t length);
uint8_t I2C_Receive(uint8_t SLA_Address, uint8_t* DATA, uint16_t length);
uint8_t I2C_Write_register(uint8_t devaddr, uint8_t regaddr, uint8_t* DATA, uint16_t length);
uint8_t I2C_Read_register(uint8_t devaddr, uint8_t regaddr, uint8_t* DATA, uint16_t length);
void I2C_Stop(void);

#endif // I2C_MASTER_H
