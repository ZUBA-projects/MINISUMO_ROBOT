/*
 * hal.h
 *
 *  Created on: 07.04.2020
 *      Author: zuba1
 */

#ifndef HAL_H_
#define HAL_H_

#include <avr/io.h> // Contains all the I/O Register Macros
#include "stdio.h"
#include "util/delay.h"
#include "string.h"
#include <avr/interrupt.h>
#include "avr/pgmspace.h"
#include "stdlib.h"


#define USE_PROGMEM		1


#ifdef USE_PROGMEM
#define MY_PROGMEM 		PROGMEM
#define my_pgm_read_byte(addr) 		pgm_read_byte(addr)
#else
#define MY_PROGMEM					()
#define my_pgm_read_byte(addr) 		(*(const unsigned char *)(addr))
#endif




//GPIO

//digital
typedef enum {D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13,D_LAST} digitalgpionames_t;

//analog
typedef enum {A0=D_LAST,A1,A2,A3,A4,A5,A6,A7,A_LAST} analoggpionames_t;





#define NO_GPIO		0xFF		//no gpio
#define DEF_GPIO	0xFE		//default gpio ie. hardware i2c or spi

typedef enum{
	GPIO_INPUT,
	GPIO_ADC,
	GPIO_OUTPUT,
	GPIO_PWM
}gpioportmode_t;

void gpioSet(uint8_t _gpio, uint8_t _state);
uint8_t gpioGet(uint8_t _gpio);
void pwmSet(uint8_t _gpio, uint8_t _value);
uint16_t adcGet(uint8_t _gpio);
void gpioInit(uint8_t _gpio, gpioportmode_t _mode);


//Time
uint32_t hal_millis();
void hal_delay(uint32_t _time);



//I2C
typedef enum{
	I2C_NACK_HAL,
	I2C_ACK_HAL
}i2cack_t;

typedef enum{
	I2C_WRITE_HAL,
	I2C_READ_HAL
}i2crw_t;

void i2c_Start(uint8_t _scl, uint8_t _sda, uint8_t _addr);
void i2c_Write(uint8_t _data);
uint8_t i2c_Read(i2cack_t _ack);
void i2c_Stop();


void init_hal();



#endif /* HAL_H_ */
