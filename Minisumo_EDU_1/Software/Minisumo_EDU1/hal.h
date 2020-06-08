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
#include "compat/twi.h"
//#include "math.h"

#define PROGRAM_V_MAJOR		0
#define PROGRAM_V_MINOR		32

#define USE_PROGMEM		1


#ifdef USE_PROGMEM
#define MY_PROGMEM 		PROGMEM
#define my_pgm_read_byte(addr) 		pgm_read_byte(addr)
#else
#define MY_PROGMEM					()
#define my_pgm_read_byte(addr) 		(*(const unsigned char *)(addr))
#endif


#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )


#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define swap(a, b) { int16_t t = a; a = b; b = t; }

#define sq(x) ((x)*(x))

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define clearbit(sfr, b) (sfr &= ~(1<<b))
#define setbit(sfr, b) (sfr |= (1<<b))
#define togglebit(sfr, b) (sfr ^= (1<<b))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)


//#define ASM_IMPLEMENTATION		1		//USE USER ASM COMMANDS

//https://forum.arduino.cc/index.php?topic=587329.0
#ifdef ASM_IMPLEMENTATION

#define cbi(sfr, b) __asm__ __volatile__ (		\
	"cbi %0, %1\n\t"							\
	:	/* no outputs */					\
	:"I" (_SFR_IO_ADDR(sfr)), "I" (b)			\
	:"memory")


#define sbi(sfr, b)  __asm__ __volatile__ (		    \
		"sbi %0, %1\n\t"							\
		:	/* no outputs */				\
		:"I" (_SFR_IO_ADDR(sfr)), "I" (b)			\
		:"memory")

/*
#define sbi(sfr, b)  __asm__ __volatile__ (		    \
		"lds r24, %0\n\t"							\
		:				\
		: "r" (b)			\
		:)
*/



#define cbi(sfr, b) clearbit(sfr, b)
#define tbi(sfr, b) (sfr ^= (1<<b))


#define interrupts() 	__asm__ __volatile__ ("sei\n\t" :/* no outputs */ :/* no inputs */ : "memory")
#define noInterrupts() 	__asm__ __volatile__ ("cli\n\t" :/* no outputs */ :/* no inputs */ : "memory")





#else




#define cbi(sfr, b) clearbit(sfr, b)
#define sbi(sfr, b) setbit(sfr, b)
#define tbi(sfr, b) togglebit(sfr, b)

#define interrupts() sei()
#define noInterrupts() cli()




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

typedef void (*call_backINT) (uint8_t,uint8_t); //_gpio,_state
void gpioSetInt(uint8_t _gpio, void * callback_ev);
void gpioRemoveInt(uint8_t _gpio);

//Time
uint32_t hal_millis(); //return system time in ms
uint32_t hal_micros();//return system time in us
void hal_delay(uint32_t _time); //delay (ms)


//math
int16_t hal_sin_deg_int16(int16_t _deg);
int16_t hal_cos_deg_int16(int16_t _deg);




//I2C

#define HAL_I2C_TIMEOUT		1

typedef enum{
	I2C_NACK_HAL,
	I2C_ACK_HAL
}i2cack_t;

typedef enum{
	I2C_WRITE_HAL,
	I2C_READ_HAL
}i2crw_t;

uint8_t i2c_Start(uint8_t _scl, uint8_t _sda, uint8_t _addr);
uint8_t i2c_Write(uint8_t _data);
uint8_t i2c_Read(i2cack_t _ack);
void i2c_Stop();



// setup instance for i2c
void i2c_set_reg_instance(uint8_t _scl, uint8_t _sda, uint8_t _addr);
// Write an 8-bit register
void i2c_writeReg(uint8_t reg, uint8_t value);
// Write a 16-bit register
void i2c_writeReg16Bit(uint8_t reg, uint16_t value);

// Write a 32-bit register
void i2c_writeReg32Bit(uint8_t reg, uint32_t value);
// Read an 8-bit register
uint8_t i2c_readReg(uint8_t reg);
// Read a 16-bit register
uint16_t i2c_readReg16Bit(uint8_t reg);

// Read a 32-bit register
uint32_t i2c_readReg32Bit(uint8_t reg);

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void i2c_writeMulti(uint8_t reg, uint8_t const *src, uint16_t count,uint8_t isrestart);


// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void i2c_readMulti(uint8_t reg, uint8_t * dst, uint16_t count);




void init_hal();



#endif /* HAL_H_ */
