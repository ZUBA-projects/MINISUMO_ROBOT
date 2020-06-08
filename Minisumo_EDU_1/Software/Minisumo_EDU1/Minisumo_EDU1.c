#include "MiniOS/miniOS.h"
#include "hal.h"

//displaying values
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

//VL53l0x sensors gpio0
#define VL53L0X_L_RST_PIN   A1     //Left TOF sensor Reset pin
#define VL53L0X_LF_RST_PIN  D12     //Left-Forward TOF sensor Reset pin
#define VL53L0X_RF_RST_PIN  A3     //Right-Forward TOF sensor Reset pin
#define VL53L0X_R_RST_PIN   A0     //Right TOF sensor Reset pin


#define MAX_RANGE_OF_DISTANCE_SENSOR	400			//400mm -40cm

typedef enum{
  RANGE_SENSOR_L,   //Left range sensor
  RANGE_SENSOR_LF,  //Left Front range sensor
  RANGE_SENSOR_RF,  //Right Front range sensor
  RANGE_SENSOR_R,   //Right range sensor
  RANGE_SENSOR_NUM  //number of range sensors
}distancesensors_t;

sensors_t sensors;

void init_range_sensors(){

  //init range sensor display module
  init_range_sensor_stack(RANGE_SENSOR_NUM);

  add_range_sensor(RANGE_SENSOR_L, 0x10, -90, 1, 1, VL53L0X_L_RST_PIN ,NO_GPIO, I2C_SCL_PIN, I2C_SDA_PIN);
  add_range_sensor(RANGE_SENSOR_LF,0x11, -15, 1, 10, VL53L0X_LF_RST_PIN,NO_GPIO, I2C_SCL_PIN, I2C_SDA_PIN);
  add_range_sensor(RANGE_SENSOR_RF,0x12,  15, 100, 10, VL53L0X_RF_RST_PIN,NO_GPIO, I2C_SCL_PIN, I2C_SDA_PIN);
  add_range_sensor(RANGE_SENSOR_R, 0x13,  90, 100, 1, VL53L0X_R_RST_PIN ,NO_GPIO, I2C_SCL_PIN, I2C_SDA_PIN);

  //config of distance sensor (hardware config)
  rangesensorconfig_t rangesensorconfig;
  rangesensorconfig._SignalRateLimit=25;// lower the return signal rate limit (default is 0.25 MCPS) where 10 is equal of 0.1
  rangesensorconfig._VcselPulsePeriodFinal=10;// increase laser pulse periods (defaults are 14 and 10 PCLKs) 14
  rangesensorconfig._VcselPulsePeriodPre=14;//18
  rangesensorconfig._timmingbudget=5;   // integrate over x ms per measurement
  rangesensorconfig._timmingpetroid=0;    // petroid for continous

  hal_delay(100);

  //setup max range of sensors
  set_max_sensor_range(MAX_RANGE_OF_DISTANCE_SENSOR);

  //setup added sensors
  init_range_sensors_instances(&rangesensorconfig);
}


/********************************************---ANALOG SENSOR---************************************/

//Analog Sensors
#define BATTERY_PIN         A2     //Pin for Battery voltage measurment
#define COLLOR1_PIN         A6     //Pin for Dohyo Color 1 measurment
#define COLLOR2_PIN         A7     //Pin for Dohyo Color 2 measurment

#define ANALOG_DEF_TRESHOLD	500

#define BATTERY_LVL_GAIN	10		//1:10
#define ADC_MAX_VTG			5000	//mV
#define ADC_MAX_VALUE		1023	//10b
#define ADC_COEF	(ADC_MAX_VTG*BATTERY_LVL_GAIN)



void init_analog_sensors(){
	set_analog_sensor_treshold(ANALOG_DEF_TRESHOLD);
	init_analog_sensor(COLLOR1_PIN);
	init_analog_sensor(COLLOR2_PIN);
	init_analog_sensor(BATTERY_PIN);
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

/********************************************---RC5---************************************/


/********************************************---MAIN---************************************/

void my_init(){

   init_hal();
  

   init_led(LED_PIN);
   led(0,LED_PIN);


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

}


void update_display_main(){
	  oled_clear(); //clear display

	   char data[16];
/*
	   //display ringid
	   sprintf(data,"Id:23");
	   oled_print((char*)data,1,1,28,21,0); //update display flag

	   //display battery state
	   uint16_t bat=get_battery_lvl_mv(BATTERY_PIN,ADC_COEF,ADC_MAX_VALUE);
	   uint16_t vt=bat/1000;
	   uint16_t dotvt=(bat/100)-(vt*10);

	   sprintf(data,"%d.%dV", vt,dotvt);
	   oled_print((char*)data,1,1,62,21,1);
*/

	   //display opponent ring
	   oled_print_ring(64,16,sensors._opponentdegree,sensors._opponentdistance,get_max_sensor_range());



	   //display range sensors
	   vl53l0x_t *_sensorinstance=malloc(sizeof(vl53l0x_t));

	   for(uint8_t id=0; id<RANGE_SENSOR_NUM; id++){

		   get_range_sensor_instance(_sensorinstance ,id);

		   if(_sensorinstance!=NULL){
			   if(bitRead(_sensorinstance->_sensorstatebus, VL53L0X_IS_INITED)){
				   sprintf(data,"%d", _sensorinstance->_lastrange);
				   oled_print((char*)data,1,(_sensorinstance->_lastrange==get_max_sensor_range()),_sensorinstance->_dpposx,_sensorinstance->_dpposy,0);
			   }else{
				   oled_print("ERR",1,1,_sensorinstance->_dpposx,_sensorinstance->_dpposy,0);
			   }
		   }
	   }

	   free(_sensorinstance);


	   //display color sensors
	   uint16_t _temp=get_analog_value(COLLOR1_PIN);	//get_analog_sensor(COLLOR1_PIN);
	   sprintf(data,"%d", _temp);
	   oled_print((char*)data,1,(_temp>get_analog_sensor_treshold()),1,21,0);

	   _temp=get_analog_value(COLLOR2_PIN);				//get_analog_sensor(COLLOR2_PIN);
	   sprintf(data,"%d", _temp);
	   oled_print((char*)data,1,(_temp>get_analog_sensor_treshold()),100,21,0);

	   oled_print("",1,1,1,1,1); //update display
}






void my_loop(){

   //handle sensors in idle time
	check_distance(&sensors);

/*
   static uint32_t _lasttime=0;
   uint32_t _nowtime=hal_millis();

   oled_clear();
   char data[16];

   static uint32_t delta=0;
   static uint32_t lastdelta=0;

  if(_rangesensorinstances[RANGE_SENSOR_R]._isnewrange){
	  _rangesensorinstances[RANGE_SENSOR_R]._isnewrange=0;
	  delta=_nowtime-lastdelta;
	  lastdelta=_nowtime;
  }

   sprintf(data,"T:%lu ms  ",(_nowtime-_lasttime));
   oled_print((char*)data,1,1,10,1,1);

   if(_rangesensorinstances[RANGE_SENSOR_R]._sensorisinited){
   	   sprintf(data,"R:%d", _rangesensorinstances[RANGE_SENSOR_R]._lastrange);
   	   oled_print((char*)data,1,1,10,10,1);
   }else{
	   oled_print("R:ERR",1,1,10,10,1);
   }

   sprintf(data,"TM:%lu ",delta);
   oled_print((char*)data,1,1,10,20,1);

   _lasttime=_nowtime;

*/

   if(sensors._isnew){
	   sensors._isnew=0;

   }
   update_display_main();




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
