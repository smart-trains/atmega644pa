/*
 * IncFile1.h
 *
 * Created: 5/09/2017 4:33:30 PM
 *  Author: SoraJ
 */ 

/*SET and GET MACRO*/
#include "ioport.h"
#include <avr/io.h> 
//#include "iom644pa.h"


#ifndef INCFILE1_H_
#define INCFILE1_H_

#endif /* INCFILE1_H_ */

#define SET(PORT,MASK,VALUE) PORT = ((MASK & VALUE) | (PORT & ~MASK))
#define GET(PORT,MASK) PORT & MASK



#define  LED     IOPORT_CREATE_PIN(PORTB, 3)//LED pin: PB3

#define MOSI		IOPORT_CREATE_PIN(PORTB, 5)
#define MISO		IOPORT_CREATE_PIN(PORTB, 6)
#define SCK			IOPORT_CREATE_PIN(PORTB, 7)

#define CS_SC		IOPORT_CREATE_PIN(PORTA, 6)

#define SPI_SS		IOPORT_CREATE_PIN(PORTB, 4)

#define INT_SC		IOPORT_CREATE_PIN(PORTD, 3)

//for MPU6050
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;


