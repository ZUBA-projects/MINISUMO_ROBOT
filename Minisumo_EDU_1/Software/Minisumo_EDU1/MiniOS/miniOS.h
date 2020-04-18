/*
 * miniOS.h
 *
 *  Created on: 05.04.2020
 *      Author: zuba1
 */

#ifndef MINISUMO_ARDUINO_EXAMPLE_EDU_1_MINIOS_H_
#define MINISUMO_ARDUINO_EXAMPLE_EDU_1_MINIOS_H_

#include "../hal.h"
#include "oled/ssd1306.h"
#include "vl53l0x/VL53L0X.h"
#include "motor/motor.h"
#include "rc5/rc5.h"
typedef struct{
	//start module cfg
	uint8_t ringid;

	//fight cfg
	uint8_t isfighting;
	uint8_t fightmode;
	uint8_t startwaittime;
	uint8_t startangle;

	//sensor cfg
	uint8_t colortreshold;
	uint8_t rangetreshold;

	//motion cfg
	uint8_t searchspeed;
	uint8_t atackspeed;

}miniOSmemory_t;


typedef struct{
	uint8_t _gpio0pin;
	uint8_t _gpio1pin;
	uint8_t _i2caddr;
	uint16_t _lastrange;
	vl53l0x_t _vl53l0xSensor;
}rangesensorinstance_t;

typedef struct{
	uint16_t _timmingbudget;
	uint16_t _timmingpetroid;
	uint32_t _SignalRateLimit;
	uint8_t _VcselPulsePeriodFinal;
	uint8_t _VcselPulsePeriodPre;
}rangesensorconfig_t;

typedef struct{
	uint8_t test1;
	uint8_t test2;

}miniOSconfig_t;


void oled_init(uint8_t _sdapin,uint8_t _sclpin,uint8_t rstpin);
//void oled_print_oponentring(uint8_t x,uint8_t y,float deg,float dist);
void oled_print(char* data,uint8_t size,uint8_t color,uint8_t x,uint8_t y,uint8_t update);
void oled_clear();
void oled_fill();

void preinit_range_sensor(rangesensorinstance_t * sensors);
void init_range_sensor(rangesensorinstance_t * _sensors,rangesensorconfig_t *_cfg);
uint16_t get_distance(rangesensorinstance_t * _sensors);


void init_analog_sensor(uint8_t _gpio);
uint16_t get_analog_value(uint8_t _gpio);


void led(uint8_t _state,uint8_t _gpio);
void init_led(uint8_t _gpio);



void miniOS_init(miniOSconfig_t * _config);


#endif /* MINISUMO_ARDUINO_EXAMPLE_EDU_1_MINIOS_H_ */
