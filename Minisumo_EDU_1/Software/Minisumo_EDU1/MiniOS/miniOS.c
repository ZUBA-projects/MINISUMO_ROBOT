/*
 * miniOS.c
 *
 *  Created on: 05.04.2020
 *      Author: zuba1
 */

#include "../hal.h"
#include "miniOS.h"
#include "oled/ssd1306.h"
#include "vl53l0x/VL53L0X.h"
#include "motor/motor.h"
#include "rc5/rc5.h"



/*********************************************----OLED----**************************************************************/

void oled_init(uint8_t _sdapin,uint8_t _sclpin,uint8_t rstpin){

	gpioInit(rstpin, GPIO_OUTPUT);

	gpioSet(rstpin, 0);
	hal_delay(10);
	gpioSet(rstpin, 1);
	hal_delay(10);

	ssd1306_init_i2c(_sdapin, _sclpin);
	//ssd1306_set_rotation(2);
	ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, 0);
	//ssd1306_display();
}


void oled_print_ring(uint8_t x,uint8_t y,int16_t deg,int32_t dist,uint16_t maxdist){


	if(dist>maxdist) dist=maxdist;
	dist=maxdist-dist;

	uint8_t dispmaxrange=15;

	dist=(dist*dispmaxrange)/maxdist;

	//scale

	deg-=90;

	//calc vector

	int32_t x2=(dist*hal_cos_deg_int16(deg))/0x7FFF; // hal_cos_deg_int16 returns values from - 0x7FFF to 0x7FFF (-1 to 1)
	int32_t y2=(dist*hal_sin_deg_int16(deg))/0x7FFF;

	ssd1306_draw_circle(x,y, dispmaxrange,1);

	ssd1306_draw_line(x, y, x2+x, y2+y,1);
}


void oled_print(char* data,uint8_t size,uint8_t color,uint8_t x,uint8_t y,uint8_t update){


    ssd1306_set_textsize(size);
    //ssd1306_set_textcolor(color);
    ssd1306_set_textcolor_bg(color,1-color);
    ssd1306_set_cursor(x, y);

    for(uint16_t i=0; i<strlen(data);i++){
    	  ssd1306_write(data[i]);
    }

    if(update)ssd1306_display();
}

void oled_clear(){
	   ssd1306_clear_display();
}

void oled_fill(){
	ssd1306_fill_screen(WHITE);
}


/*********************************************----LV53l0x----**************************************************************/

static vl53l0x_t *_rangesensorinstances;//[__rangesensornum];

uint8_t __rangesensornum;
uint16_t _maxrangeofsensor;

uint8_t init_range_sensor_stack(uint8_t _localsensornum){
	//create instance
	__rangesensornum=_localsensornum;
	_rangesensorinstances=malloc(sizeof(vl53l0x_t)*__rangesensornum);

	if(_rangesensorinstances!=NULL) return 0;
	return 1;

}

uint8_t add_range_sensor(uint8_t _id, uint8_t _i2caddr, int16_t _angle, uint8_t _dispposx, uint8_t _dispposy, uint8_t _gpio0, uint8_t _gpio1, uint8_t _scl, uint8_t _sda){

	if(_id>__rangesensornum) return 1;

	//configure individual settings for range sensors
	_rangesensorinstances[_id].g_i2cAddr=_i2caddr;
	_rangesensorinstances[_id]._positionangle=_angle;
	_rangesensorinstances[_id]._gpio0pin=_gpio0;
	_rangesensorinstances[_id]._gpio1pin=_gpio1;
	_rangesensorinstances[_id]._sclpin=_scl;
	_rangesensorinstances[_id]._sdapin=_sda;
	_rangesensorinstances[_id]._dpposx=_dispposx;
	_rangesensorinstances[_id]._dpposy=_dispposy;


	setvl53l0xinstance(&_rangesensorinstances[_id]);
	preinitVL53L0X();
	return 0;
}

void set_max_sensor_range(uint16_t _mrng){
	_maxrangeofsensor=_mrng;
}

uint16_t get_max_sensor_range(){
	return _maxrangeofsensor;
}

uint8_t get_range_sensor_instance(vl53l0x_t * _sensorinstance ,uint8_t _id){
	if(_id>__rangesensornum) return 1;
	*_sensorinstance=_rangesensorinstances[_id];
	return 0;
}

void init_range_sensors_instances(rangesensorconfig_t *_cfg){

	  //initialize all of range sensors and setup new address
	  for(uint8_t i=0; i<__rangesensornum;i++){
			setvl53l0xinstance(&_rangesensorinstances[i]);
			initVL53L0X_to_continous(_cfg);
	  }
}


uint8_t check_distance(sensors_t* _sensors){

	uint8_t readystensorscount=0;
	uint8_t initedstensorscount=0;
	uint8_t i=0;
	uint8_t smallerid1=__rangesensornum;
	uint8_t smallerid2=__rangesensornum;
	uint16_t temprange=_maxrangeofsensor;

	for(i=0; i<__rangesensornum;i++){
		//handle continuous conversion
		setvl53l0xinstance(&_rangesensorinstances[i]);
		checkRangeContinousSensor();
		//normalize sensor range
		if(_rangesensorinstances[i]._lastrange > _maxrangeofsensor) _rangesensorinstances[i]._lastrange=_maxrangeofsensor;
		if(bitRead(_rangesensorinstances[i]._sensorstatebus, VL53L0X_DONE))readystensorscount++;
		if(bitRead(_rangesensorinstances[i]._sensorstatebus, VL53L0X_IS_INITED))initedstensorscount++;
	}

	if(!(((initedstensorscount<2)&&(readystensorscount))||(readystensorscount>(initedstensorscount/2)))) return 0;


		i=0;
		_sensors->_isnew=1; //data complete. set up new data flag
		_sensors->_opponentdistance=_maxrangeofsensor;
		_sensors->_issighted=0;
		//find one or two sensors where distance is lowest and compare ranges

		//find smaller one
		while(i<__rangesensornum){
			if(bitRead(_rangesensorinstances[i]._sensorstatebus, VL53L0X_DONE)){
				//check distance value
				if(_rangesensorinstances[i]._lastrange<_maxrangeofsensor){
					//write coordinates of opponent
					if(_rangesensorinstances[i]._lastrange<temprange){
						temprange=_rangesensorinstances[i]._lastrange;
						smallerid1=i;
					}
				}
			}

			i++;
		}

		if(smallerid1!=__rangesensornum){
			_sensors->_opponentdegree=_rangesensorinstances[smallerid1]._positionangle;
			_sensors->_opponentdistance=_rangesensorinstances[smallerid1]._lastrange;
			_sensors->_issighted=1;
		}else{
			_sensors->_opponentdistance=_maxrangeofsensor;
			_sensors->_issighted=0;
		}

		if(readystensorscount>1){
			i=0;
			temprange=_maxrangeofsensor;

			//find second smaller value
			while(i<__rangesensornum){
				if(bitRead(_rangesensorinstances[i]._sensorstatebus, VL53L0X_DONE)){
					//clear range flag
					cbi(_rangesensorinstances[i]._sensorstatebus, VL53L0X_DONE);
					//check distance value
					if(_rangesensorinstances[i]._lastrange<_maxrangeofsensor){

						//write coordinates of opponent
						if(_rangesensorinstances[i]._lastrange<temprange){
							temprange=_rangesensorinstances[i]._lastrange;
							smallerid2=i;
						}
					}

				}
				i++;
				if(i==smallerid1) i++;
			}

			if(smallerid2!=__rangesensornum){
				_sensors->_opponentdegree=(_rangesensorinstances[smallerid1]._positionangle +_rangesensorinstances[smallerid2]._positionangle)/2;
				_sensors->_opponentdistance=(_rangesensorinstances[smallerid1]._lastrange +_rangesensorinstances[smallerid2]._lastrange)/2;

			}
		}
	return 1;
}






/*********************************************----ADC----**************************************************************/

uint16_t _analogtresholdvalue=500; //set as def 500

void set_analog_sensor_treshold(uint16_t _antresh){
	 _analogtresholdvalue=_antresh;
}

uint16_t get_analog_sensor_treshold(){
	 return _analogtresholdvalue;
}


void init_analog_sensor(uint8_t _gpio){
	gpioInit(_gpio, GPIO_ADC);
}

uint16_t get_analog_value(uint8_t _gpio){
	uint16_t _value;
	_value=adcGet(_gpio);
	return _value;
}

uint8_t get_analog_sensor(uint8_t _gpio){

	return (get_analog_value(_gpio) < _analogtresholdvalue);
}

uint32_t get_battery_lvl_mv(uint8_t _gpio,uint16_t _coef,uint16_t _maxrang){
	return (get_analog_value(_gpio)*_coef)/_maxrang;
}




/*********************************************----RC5----**************************************************************/




/*****************************************----LED AND BUTTON----*******************************************************/

void led(uint8_t _state,uint8_t _gpio){
	gpioSet(_gpio, _state);

}

void init_led(uint8_t _gpio){
	gpioInit(_gpio, GPIO_OUTPUT);
}




void miniOS_init(miniOSconfig_t * _config){

	_config->test1=_config->test2;


}

/*



#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>





#define ROBOT_NAME    "EDU1"




#define UART_SPEED         115200
#define KEY_OF_MEMORY      33


typedef enum{
   SEARCH_STOP,
   SEARCH_ROTATE,
   SEARCH_PINGPONG,
   SEARCH_HEXAGON,
}searchmode_t;






//RC5 "SUMO REMOTE" gpio
#define RC5_PIN            3     //RC5 gpio -int0 -D3

//led and button gpio
#define LED_PIN            13     //Led gpio
#define BUTTON1_PIN        11     //Button 1 gpio
#define BUTTON2_PIN        10     //Button 2 gpio

//uart -hardwre defined by Serial
//#define UART_TX_PIN      0     //Txd pin of uart
//#define UART_RX_PIN      1     //Rxd pin of uart

//I2C -Hardware defined
//#define I2C_SCL_PIN      A5     //SCL pin of I2C
//#define I2C_SDA_PIN      A4     //SDA pin of I2C

*/




