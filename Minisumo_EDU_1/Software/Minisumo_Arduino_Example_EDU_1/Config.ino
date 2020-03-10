/********************************   Robot Movement Config  *******************************************/







/*******************************   Robot Components Config  ******************************************/

#define UART_SPEED         115200



/*******************************    Memory Adress Config  ********************************************/
enum{
  MEMORY_RING_ID,                 //Saves Ring ID for start module component
  MEMORY_IS_FIGHTING,             //Saves Is fighting status
  MEMORY_COLOR_TRESHOLD,          //Saves Color treshold
  MEMORY_FRONT_RANGE_TRESHOLD,    //Saves Front Range sensors treshold in cm
  MEMORY_SIDE_RANGE_TRESHOLD,     //Saves Side Range sensors treshold in cm
  MEMORY_SEARCH_MODE,             //Saves Search mode 
  MEMORY_SEARCH_SPEED,            //Saves search max speed
  MEMORY_ATACK_SPEED,             //Saves atack max speed



  MEMORY_CHECK_SUM,               //Saves check sum of written data
  MEMORY_END                      //end addres -used to declare size of buffor
}

/************************************   GPIO Config   ************************************************/

//VL53l0x sensors gpio
#define VL53L0X_L_RST_PIN   0     //Left TOF sensor Reset pin 
#define VL53L0X_LF_RST_PIN  0     //Left-Forward TOF sensor Reset pin 
#define VL53L0X_RF_RST_PIN  0     //Right-Forward TOF sensor Reset pin 
#define VL53L0X_R_RST_PIN   0     //Right TOF sensor Reset pin 

//Analog Sensors
#define BATTERY_PIN         0     //Pin for Battery voltage measurment
#define COLLOR1_PIN         0     //Pin for Dohyo Color 1 measurment
#define COLLOR2_PIN         0     //Pin for Dohyo Color 2 measurment

//Motor Left 
#define LF_PIN             0     //Left forward gpio
#define LB_PIN             0     //Left Backward gpio
#define LPWM_PIN           0     //Left PWM gpio

//Motor Right
#define RF_PIN             0     //Right forward gpio
#define RB_PIN             0     //Right Backward gpio
#define RPWM_PIN           0     //Right PWM gpio

//RC5 "SUMO REMOTE" gpio    
#define RC5_PIN            0     //RC5 gpio

//led and button gpio
#define LED_PIN            0     //Led gpio
#define BUTTON1_PIN        0     //Button 1 gpio  
#define BUTTON2_PIN        0     //Button 2 gpio 

//uart -hardwre defined by Serial
//#define UART_TX_PIN      0     //Txd pin of uart
//#define UART_RX_PIN      0     //Rxd pin of uart

//I2C -Hardware defined 
//#define I2C_SCL_PIN      0     //SCL pin of I2C
//#define I2C_SDA_PIN      0     //SDA pin of I2C

 
