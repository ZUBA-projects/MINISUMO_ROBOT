#include "MiniOS/miniOS.h"
#include "hal.h"


#define ROBOT_NAME    "EDU1"


//RC5 "SUMO REMOTE" gpio
#define RC5_PIN            D3     //RC5 gpio -int0 -D3

//led and button gpio
#define LED_PIN            D13     //Led gpio
#define BUTTON1_PIN        D11     //Button 1 gpio
#define BUTTON2_PIN        D10     //Button 2 gpio

//uart -hardwre defined by Serial
//#define UART_TX_PIN      0     //Txd pin of uart
//#define UART_RX_PIN      1     //Rxd pin of uart

//I2C -Hardware defined
#define I2C_SCL_PIN      A5     //SCL pin of I2C
#define I2C_SDA_PIN      A4     //SDA pin of I2C

/********************************************---DISTANCE SENSOR---************************************/

//VL53l0x sensors gpio
#define VL53L0X_L_RST_PIN   A1     //Left TOF sensor Reset pin
#define VL53L0X_LF_RST_PIN  D12     //Left-Forward TOF sensor Reset pin
#define VL53L0X_RF_RST_PIN  A3     //Right-Forward TOF sensor Reset pin
#define VL53L0X_R_RST_PIN   A0     //Right TOF sensor Reset pin

typedef enum{
  RANGE_SENSOR_L,   //Left range sensor
  RANGE_SENSOR_LF,  //Left Front range sensor
  RANGE_SENSOR_RF,  //Right Front range sensor
  RANGE_SENSOR_R,   //Right range sensor
  RANGE_SENSOR_NUM  //number of range sensors
}distancesensors_t;

static rangesensorinstance_t _sensors[RANGE_SENSOR_NUM];

void init_range_sensors(){

  //global config of distance sensor
  rangesensorconfig_t rangesensorconfig;
  rangesensorconfig._SignalRateLimit=10;// lower the return signal rate limit (default is 0.25 MCPS) where 10 is equal of 0.1
  rangesensorconfig._VcselPulsePeriodFinal=14;// increase laser pulse periods (defaults are 14 and 10 PCLKs)
  rangesensorconfig._VcselPulsePeriodPre=18;
  rangesensorconfig._timmingbudget=100;   // integrate over 500 ms per measurement
  rangesensorconfig._timmingpetroid=0;    // petroid for continous

  //config sensors gpio and i2c addr
  _sensors[RANGE_SENSOR_L]._i2caddr=0x50;
  _sensors[RANGE_SENSOR_L]._gpio0pin=VL53L0X_L_RST_PIN;
  _sensors[RANGE_SENSOR_L]._gpio1pin=NO_GPIO;
  _sensors[RANGE_SENSOR_L]._vl53l0xSensor._sclpin=I2C_SCL_PIN;
  _sensors[RANGE_SENSOR_L]._vl53l0xSensor._sdapin=I2C_SDA_PIN;

  _sensors[RANGE_SENSOR_LF]._i2caddr=0x51;
  _sensors[RANGE_SENSOR_LF]._gpio0pin=VL53L0X_LF_RST_PIN;
  _sensors[RANGE_SENSOR_LF]._gpio1pin=NO_GPIO;
  _sensors[RANGE_SENSOR_LF]._vl53l0xSensor._sclpin=I2C_SCL_PIN;
  _sensors[RANGE_SENSOR_LF]._vl53l0xSensor._sdapin=I2C_SDA_PIN;

  _sensors[RANGE_SENSOR_RF]._i2caddr=0x52;
  _sensors[RANGE_SENSOR_RF]._gpio0pin=VL53L0X_RF_RST_PIN;
  _sensors[RANGE_SENSOR_RF]._gpio1pin=NO_GPIO;
  _sensors[RANGE_SENSOR_RF]._vl53l0xSensor._sclpin=I2C_SCL_PIN;
  _sensors[RANGE_SENSOR_RF]._vl53l0xSensor._sdapin=I2C_SDA_PIN;

  _sensors[RANGE_SENSOR_R]._i2caddr=0x53;
  _sensors[RANGE_SENSOR_R]._gpio0pin=VL53L0X_R_RST_PIN;
  _sensors[RANGE_SENSOR_R]._gpio1pin=NO_GPIO;
  _sensors[RANGE_SENSOR_R]._vl53l0xSensor._sclpin=I2C_SCL_PIN;
  _sensors[RANGE_SENSOR_R]._vl53l0xSensor._sdapin=I2C_SDA_PIN;

  //pre initialize all of range sensors (init gpio0 and disable sensor)
  for(uint8_t i=0; i<RANGE_SENSOR_NUM;i++){
    preinit_range_sensor(&_sensors[i]);
  }

  //initialize all of range sensors and setup new address
  for(uint8_t i=0; i<RANGE_SENSOR_NUM;i++){
    init_range_sensor(&_sensors[i],&rangesensorconfig); //init sensor
    get_distance(&_sensors[i]); //run continuous conversion
  }
}

void check_range_sensors(){
  //initialize all of range sensors and setup new address
  for(uint8_t i=0; i<RANGE_SENSOR_NUM;i++){
    get_distance(&_sensors[i]); //handle continuous conversion
  }
}



/********************************************---ANALOG SENSOR---************************************/

//Analog Sensors
#define BATTERY_PIN         A2     //Pin for Battery voltage measurment
#define COLLOR1_PIN         A6     //Pin for Dohyo Color 1 measurment
#define COLLOR2_PIN         A7     //Pin for Dohyo Color 2 measurment

typedef enum{
  ANALOG_SENSOR_LF, //Left Front color sensor
  ANALOG_SENSOR_RF, //Right Front color sensor
  ANALOG_SENSOR_BATTERY,  //Battery lvl check
  ANALOG_SENSOR_NUM //number of color sensor
}analogsensors_t;

void init_analog_sensors(){
	init_analog_sensor(BATTERY_PIN);
}

void check_analog_sensors(){

	static uint16_t _oldval=0;
	uint16_t _aval=get_analog_value(BATTERY_PIN);

	if(_aval!=_oldval){
		_oldval=_aval;
		/*
		   oled_clear();
		   char data[16];

		   sprintf(data,"AN:%d",_aval);
		   oled_print((char*)data,1,1,10,10,1);
		   */

		if(_aval>500){
			led(1,LED_PIN);
		}else{
			led(0,LED_PIN);
		}

	}


}

/********************************************---MOTOR---************************************/

//Motor Left
#define LF_PIN             D2     //Left forward gpio
#define LB_PIN             D4     //Left Backward gpio
#define LPWM_PIN           D5     //Left PWM gpio

//Motor Right
#define RF_PIN             D8     //Right forward gpio
#define RB_PIN             D7     //Right Backward gpio
#define RPWM_PIN           D6     //Right PWM gpio

static motorconfig_t motorcfg;

void motorcfg_init(){

   motorcfg.lpwm = LPWM_PIN;
   motorcfg.lf = LF_PIN;
   motorcfg.lb = LB_PIN;
   motorcfg.rpwm = RPWM_PIN;
   motorcfg.rf = RF_PIN;
   motorcfg.rb = RB_PIN;

     //MOVE_STOP
     //MOVE_FORWARD
     //MOVE_BACKWARD
     //MOVE_RIGHT
     //MOVE_LEFT
     //MOVE_HOLD

   Motor_Init(&motorcfg);
   Motor(MOVE_STOP, 0, 0);
}



/********************************************---MAIN---************************************/

void my_init(){

   init_hal();
  

   init_led(LED_PIN);
   led(1,LED_PIN);


   static miniOSconfig_t _config;

   _config.test1=1;
   _config.test2=3;

   miniOS_init(&_config);


   //init motor
   motorcfg_init();

   //init oled
   oled_init(I2C_SCL_PIN,I2C_SDA_PIN,NO_GPIO);

   oled_clear();
   oled_print((char*)"WAIT_1_S",2,1,10,10,1);

   hal_delay(1000);

   //init vl53l0x sensors
   init_range_sensors();


   //init analog sensors
   init_analog_sensors();


  // NO_GPIO
  // DEF_GPIO
}


void my_loop(){

   //handle sensors in idle time
   check_range_sensors();
   check_analog_sensors();




   //core speed meter


   static uint32_t _lasttime=0;
   uint32_t _nowtime=hal_millis();
   static uint32_t _timeafterdisplay=0;
   oled_clear();
   char data[16];

   sprintf(data,"T:%lu ms  ",(_nowtime-_lasttime));
   oled_print((char*)data,1,1,10,10,1);

   sprintf(data,"D:%lu ms  ",(_nowtime- _timeafterdisplay));
   oled_print((char*)data,1,1,10,20,1);

   _lasttime=_nowtime;
   _timeafterdisplay=hal_millis();

   //handle OS in idle time
}


#ifndef ARDUINO

int main(void)
{
  my_init();
  while(1){
    my_loop();
  }
}

#else

void setup() {
  my_init();
}

void loop() {
  my_loop();
}

#endif
