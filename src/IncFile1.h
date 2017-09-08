/*
 * IncFile1.h
 *
 * Created: 5/09/2017 4:33:30 PM
 *  Author: SoraJ
 */ 

/*SET and GET MACRO*/
#define SET(PORT,MASK,VALUE) PORT = ((MASK & VALUE) | (PORT & ~MASK))
#define GET(PORT,MASK) PORT & MASK

#define BAT_LOW_LED_DIR(DIR) 	SET(DDRB, _BV(PB3), DIR)
#define BAT_LOW_LED(STATE) 		SET(PORTB, _BV(PB3),~STATE)

#ifndef INCFILE1_H_
#define INCFILE1_H_





#endif /* INCFILE1_H_ */