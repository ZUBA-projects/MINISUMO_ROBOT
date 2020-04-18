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
	//ssd1306_set_rotation(0);
	ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, 0);
	//ssd1306_display();
}

/*
void oled_print_oponentring(uint8_t x,uint8_t y,float deg,float dist){

	deg-=90;
	deg=(deg/57.29577951) ; //Convert degrees to radians

	dist=dist/12;

	float x2=dist*cos(deg);
	float y2=dist*sin(deg);

	ssd1306_draw_circle(x,y, 21,1);

	ssd1306_draw_line(x, y, x2+x, y2+y,1);
}
*/

void oled_print(char* data,uint8_t size,uint8_t color,uint8_t x,uint8_t y,uint8_t update){


    ssd1306_set_textsize(size);
    ssd1306_set_textcolor(color);
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

void preinit_range_sensor(rangesensorinstance_t * sensors){
	gpioInit(sensors->_gpio0pin, GPIO_OUTPUT);
	gpioInit(sensors->_gpio1pin, GPIO_INPUT);

	gpioSet(sensors->_gpio0pin, 0);
}


void init_range_sensor(rangesensorinstance_t * _sensors, rangesensorconfig_t *_cfg){

	setvl53l0xinstance(&_sensors->_vl53l0xSensor);

	gpioSet(_sensors->_gpio0pin, 1);

	initVL53L0X(1);
	// lower the return signal rate limit (default is 0.25 MCPS)
	setSignalRateLimit(_cfg->_SignalRateLimit);	//0.1 as 10
	// increase laser pulse periods (defaults are 14 and 10 PCLKs)
	setVcselPulsePeriod(VcselPeriodPreRange, _cfg->_VcselPulsePeriodPre);	//18
	setVcselPulsePeriod(VcselPeriodFinalRange, _cfg->_VcselPulsePeriodFinal);	//14
	setMeasurementTimingBudget(  _cfg->_timmingbudget * 1000UL );		// integrate over x ms per measurement

	//set up new i2c addr
	setAddress(_sensors->_i2caddr);

	startContinuous(_cfg->_timmingpetroid);
}


uint16_t get_distance(rangesensorinstance_t * _sensors){
	//set up current addr
	setvl53l0xinstance(&_sensors->_vl53l0xSensor);
	//read value

	//gpioGet(sensors->_gpio1pin);

	if(checkRangeContinousSensor()){
		_sensors->_lastrange=readRangeContinousSensor();
	}

	return _sensors->_lastrange;
}



/*********************************************----ADC----**************************************************************/



void init_analog_sensor(uint8_t _gpio){

	gpioInit(_gpio, GPIO_ADC);


}


uint16_t get_analog_value(uint8_t _gpio){
	uint16_t _value;

	_value=adcGet(_gpio);


	return _value;
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




