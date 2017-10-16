/*
 * IncFile2.h
 *
 * Created: 4/10/2017 11:42:46 PM
 *  Author: ChenxuGong
 */ 


#ifndef INCFILE2_H_
#define INCFILE2_H_
#endif /* INCFILE2_H_ */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// HTU21 device address
#define HTU21D_Address											0x80 //0b1000000

// HTU21 device commands
#define SOFT_RESET									        0xfe
#define MEASURE_TEMPERATURE				                    0xf3
#define MEASURE_HUMIDITY					                    0xf5

#define WRITE_USER_REGISTER						0xE6


// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL								(175.72)
#define TEMPERATURE_COEFF_ADD								(-46.85)

// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL									(125)
#define HUMIDITY_COEFF_ADD									(-6)

// Conversion timings

#define T11bit_measure_time		     7
#define H8bit_measure_time			 3


// HTU21 User Register masks and bit position
#define HTU21_USER_REG_RESOLUTION_MASK						0x81
// Resolution
#define T11bit_H8bit				                        0x81

