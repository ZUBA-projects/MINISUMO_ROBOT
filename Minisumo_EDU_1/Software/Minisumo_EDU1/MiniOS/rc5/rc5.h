/*
 * rc5.h
 *
 *  Created on: 05.04.2020
 *      Author: zuba1
 */

#ifndef RC5_H_
#define RC5_H_

#include "../../hal.h"


// Define number of Timer1 ticks (with a prescaler of 1/8)
#define RC5_SHORT_TIME   	 700                      // Used as a minimum time for short pulse or short space ( ==>  700 us)
#define RC5_MED_TIME	     1200                      // Used as a maximum time for short pulse or short space ( ==> 1200 us)
#define RC5_LONG_TIME     	 2000                      // Used as a maximum time for long pulse or long space   ( ==> 2000 us)

#define RC5_TOUT		33						//mS

typedef struct{
	//private
	uint8_t rc5_ok;
	uint8_t rc5_state;
	uint8_t j;
	uint32_t tout;
	uint32_t timer;
	//public
	uint16_t rc5_code;
	uint8_t toggle_bit;
	uint8_t address;
	uint8_t command;
	//cfg
	uint8_t _gpio;
}rc5struct_t;



void rc5_init(rc5struct_t * _struct);

#endif /* RC5_H_ */
