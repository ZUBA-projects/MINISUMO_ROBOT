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
	int16_t _opponentdegree;
	uint16_t _opponentdistance;
	uint8_t _issighted;
	uint8_t _isnew;
}sensors_t;



//66,2/31.3 -as pgm
//66,4/40,2 -as ram

typedef struct{
	uint8_t test1;
	uint8_t test2;

}miniOSconfig_t;


void oled_init(uint8_t _sdapin,uint8_t _sclpin,uint8_t rstpin);
void oled_print_ring(uint8_t x,uint8_t y,int16_t deg,int32_t dist,uint16_t maxdist);
void oled_print(char* data,uint8_t size,uint8_t color,uint8_t x,uint8_t y,uint8_t update);
void oled_clear();
void oled_fill();

//for instances
uint8_t init_range_sensor_stack(uint8_t _localsensornum);
uint8_t add_range_sensor(uint8_t _id, uint8_t _i2caddr, int16_t _angle, uint8_t _dispposx, uint8_t _dispposy, uint8_t _gpio0, uint8_t _gpio1, uint8_t _scl, uint8_t _sda);
void set_max_sensor_range(uint16_t _mrng);
uint16_t get_max_sensor_range();
void init_range_sensors_instances(rangesensorconfig_t *_cfg);
uint8_t get_range_sensor_instance(vl53l0x_t *_sensorinstance ,uint8_t _id);
uint8_t check_distance(sensors_t* _sensors);


//for all of range sensors
//void compare_distance(sensors_t * _sensall, rangesensorinstance_t * _sensinstances[],uint8_t _numofsensors,uint16_t _maxrange);


void set_analog_sensor_treshold(uint16_t _antresh);
uint16_t get_analog_sensor_treshold();

void init_analog_sensor(uint8_t _gpio);
uint16_t get_analog_value(uint8_t _gpio);

uint8_t get_analog_sensor(uint8_t _gpio);
uint32_t get_battery_lvl_mv(uint8_t _gpio,uint16_t _coef,uint16_t _maxrang);


void led(uint8_t _state,uint8_t _gpio);
void init_led(uint8_t _gpio);



void miniOS_init(miniOSconfig_t * _config);


#endif /* MINISUMO_ARDUINO_EXAMPLE_EDU_1_MINIOS_H_ */
