/*
 * base.h
 *
 * Created: 5/09/2017 4:33:30 PM
 *  Author: SoraJ
 */

#include "ioport.h"
#include <avr/io.h>
//#include "iom644pa.h"

#ifndef INCFILE1_H_
#define INCFILE1_H_

#endif /* INCFILE1_H_ */

#define SET(PORT,MASK,VALUE) PORT = ((MASK & VALUE) | (PORT & ~MASK))
#define GET(PORT,MASK) PORT & MASK

typedef uint8_t byte;

// LED
#define LED IOPORT_CREATE_PIN(PORTB, 3)

// SPI Bus
#define SS IOPORT_CREATE_PIN(PORTB, 4)
#define MOSI IOPORT_CREATE_PIN(PORTB, 5)
#define MISO IOPORT_CREATE_PIN(PORTB, 6)
#define SCK IOPORT_CREATE_PIN(PORTB, 7)

// SC16IS600
#define CS_SC IOPORT_CREATE_PIN(PORTA, 6)
#define RST_SC IOPORT_CREATE_PIN(PORTA, 7)
// IO5 is not used.
#define IO5_SC IOPORT_CREATE_PIN(PORTB, 0)
#define IO4_SC IOPORT_CREATE_PIN(PORTB, 1)
#define GPIO3_SC IOPORT_CREATE_PIN(PORTA, 3)
#define GPIO2_SC IOPORT_CREATE_PIN(PORTA, 2)
#define GPIO1_SC IOPORT_CREATE_PIN(PORTA, 1)
#define GPIO0_SC IOPORT_CREATE_PIN(PORTA, 0)
#define INT_SC IOPORT_CREATE_PIN(PORTD, 3)

// RS485
#define F_CPU 16000000
#define BUAD 38400
#define BRC ((F_CPU/16/BUAD)-1) //  Baud Rate Calculate
#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 128

char serialBuffer[TX_BUFFER_SIZE];
uint8_t serialReadPos = 0;
uint8_t serialWritePos = 0;

char rxBuffer[RX_BUFFER_SIZE];
uint8_t rxReadPos = 0;
uint8_t rxWritePos = 0;

//