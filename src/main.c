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
#include <asf.h>
#include "ioport.h"
#include "delay.h"
#include "time.h"
#include "iom644pa.h"
#include "sfr_defs.h"
#include <string.h>?
#include <stdio.h>

#define  LED     IOPORT_CREATE_PIN(PORTB, 3)//LED定义为PB3

#define MOSI		IOPORT_CREATE_PIN(MOSI_PORT, MOSI_BIT)
#define MISO		IOPORT_CREATE_PIN(MISO_PORT, MISO_BIT)
#define SCK			IOPORT_CREATE_PIN(SCK_PORT, SCK_BIT)

//SPI
void SPI_MasterInit(void) {
	ioport_set_pin_dir( MOSI,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir( MISO,  IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir( SCK,  IOPORT_DIR_OUTPUT);// Set MOSI and SCK output, all others input
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); //Enable SPI, Master, set clock rate fck/16
}

void SPI_MasterTransmit(char cData) {
	SPDR = cData; // Start transmission
	while(!( SPSR & _BV(SPIF) )); // Wait for transmission complete
}


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	
//	clock();
	
//	TCCR0A

	system_init(); //系统初始化
	
	
	SPI_MasterInit();
	
	ioport_init();//初始化IO端口
	ioport_set_pin_dir( LED,  IOPORT_DIR_OUTPUT);//LED端口为输出
	ioport_set_pin_level( LED, IOPORT_PIN_LEVEL_HIGH);//LED端口输出高电平
	

	while(1){
		ioport_toggle_pin_level(LED);
		_delay_ms(200);
		ioport_toggle_pin_level(LED);
		_delay_ms(200);			
	}
	
	/* Insert application code here, after the board has been initialized. */
}

/*
int main(void) {
	BAT_LOW_LED(0X00); //Make sure it is off before changing direction
	_delay_ms(10);
	BAT_LOW_LED_DIR(0xFF);//Set BATTERY LED I/Os as outputs.
	BAT_LOW_LED(0xFF);
	_delay_ms(500);
}
*/
