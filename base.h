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
#define INT_SC IOPORT_CREATE_PIN(PORTD, 3)