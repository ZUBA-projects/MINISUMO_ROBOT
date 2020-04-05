/********************************   Includes  *******************************************/
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <EEPROM.h>






//Motor Left 
#define LF_PIN             2     //Left forward gpio
#define LB_PIN             4     //Left Backward gpio
#define LPWM_PIN           5     //Left PWM gpio

//Motor Right
#define RF_PIN             8     //Right forward gpio
#define RB_PIN             7     //Right Backward gpio
#define RPWM_PIN           6     //Right PWM gpio


typedef enum{
  MOVE_STOP,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  MOVE_RIGHT,
  MOVE_LEFT,
  MOVE_HOLD
}motor_t;


void Motor_Drive(uint8_t _state, uint8_t _ml, uint8_t _mr){
  
  switch (_state){
    case MOVE_STOP:
      digitalWrite(LF_PIN, LOW);
      digitalWrite(LB_PIN, LOW);
      digitalWrite(RF_PIN, LOW);
      digitalWrite(RB_PIN, LOW);
      break;
    
    case MOVE_FORWARD:
      digitalWrite(LF_PIN, HIGH);
      digitalWrite(LB_PIN, LOW);
      digitalWrite(RF_PIN, HIGH);
      digitalWrite(RB_PIN, LOW);
      break;

    case MOVE_BACKWARD:
      digitalWrite(LF_PIN, LOW);
      digitalWrite(LB_PIN, HIGH);
      digitalWrite(RF_PIN, LOW);
      digitalWrite(RB_PIN, HIGH);
      break;
      
    case MOVE_RIGHT:
      digitalWrite(LF_PIN, HIGH);
      digitalWrite(LB_PIN, LOW);
      digitalWrite(RF_PIN, LOW);
      digitalWrite(RB_PIN, HIGH);
      break;
    
    case MOVE_LEFT:
      digitalWrite(LF_PIN, LOW);
      digitalWrite(LB_PIN, HIGH);
      digitalWrite(RF_PIN, HIGH);
      digitalWrite(RB_PIN, LOW);
      break;
    
    case MOVE_HOLD:
      digitalWrite(LF_PIN, HIGH);
      digitalWrite(LB_PIN, HIGH);
      digitalWrite(RF_PIN, HIGH);
      digitalWrite(RB_PIN, HIGH);
      break;
    
    default:
      digitalWrite(LF_PIN, LOW);
      digitalWrite(LB_PIN, LOW);
      digitalWrite(RF_PIN, LOW);
      digitalWrite(RB_PIN, LOW);
      break;
  }

  analogWrite(LPWM_PIN,_ml);
  analogWrite(RPWM_PIN,_mr);
  
}


void Motor_Init(){
  
  //initialize motor drivers gpio 
  pinMode(LF_PIN, OUTPUT);
  pinMode(LB_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(RF_PIN, OUTPUT);
  pinMode(RB_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);

  Motor_Drive(MOVE_STOP, 0, 0);
}



/********************************   Robot Config  *******************************************/

#define ROBOT_NAME    "EDU1"



/*******************************   Robot Components Config And Other  ******************************************/

#define UART_SPEED         115200
#define KEY_OF_MEMORY      33


typedef enum{
   SEARCH_STOP,
   SEARCH_ROTATE,
   SEARCH_PINGPONG,
   SEARCH_HEXAGON,
}searchmode_t;


/************************************   GPIO Config   ************************************************/




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

/********************************   Obiects  *******************************************/
U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 


/********************   Global Vars for Menu and Fight  *************************************/
//vars for menu and display
uint8_t MenuView=0;
uint8_t DisplayUpdated=0;

/*******************************    Memory EEPROM ********************************************/

enum{
  MEMORY_RING_ID,                 //Saves Ring ID for start module component
  MEMORY_IS_FIGHTING,             //Saves Is fighting status
  MEMORY_COLOR_TRESHOLD,          //Saves Color treshold
  MEMORY_FRONT_RANGE_TRESHOLD,    //Saves Front Range sensors treshold in cm
  MEMORY_SIDE_RANGE_TRESHOLD,     //Saves Side Range sensors treshold in cm
  MEMORY_SEARCH_MODE,             //Saves Search mode 
  MEMORY_SEARCH_SPEED,            //Saves search max speed
  MEMORY_ATACK_SPEED,             //Saves atack max speed

  MEMORY_KEY,                     //Saves Key of written data
  MEMORY_END                      //end addres -used to declare size of buffor
};



//vars storaged in memory
uint8_t RingId=0;                   //Ring ID for start module component
uint8_t IsFightingFlag=0;           //Is fighting status
uint8_t ColorTreshold=128;          //Color treshold
uint8_t FrontRangeTreshold=128;     //Front Range sensors treshold in cm 
uint8_t FrontSideTreshold=128;      //Front Range sensors treshold in cm
searchmode_t SearchMode=SEARCH_ROTATE; //Search mode 
uint8_t SearchSpeed=128;            //Search max speed
uint8_t AtackSpeed=128;             //Atack max speed

uint8_t Memory_Read(){
  if(KEY_OF_MEMORY==EEPROM.read(MEMORY_KEY)){
    //key is correct, read all storaged data
    RingId=EEPROM.read(MEMORY_RING_ID);
    IsFightingFlag=EEPROM.read(MEMORY_IS_FIGHTING);
    ColorTreshold=EEPROM.read(MEMORY_COLOR_TRESHOLD);
    FrontRangeTreshold=EEPROM.read(MEMORY_FRONT_RANGE_TRESHOLD);
    FrontSideTreshold=EEPROM.read(MEMORY_SIDE_RANGE_TRESHOLD);
    SearchMode=(searchmode_t)EEPROM.read(MEMORY_SEARCH_MODE);
    SearchSpeed=EEPROM.read(MEMORY_SEARCH_SPEED);
    AtackSpeed=EEPROM.read(MEMORY_ATACK_SPEED);
    return 1;
  }
  return 0;
}




void Memory_Complete_Write(){
    EEPROM.write(MEMORY_RING_ID, RingId);
    EEPROM.write(MEMORY_IS_FIGHTING, IsFightingFlag);
    EEPROM.write(MEMORY_COLOR_TRESHOLD, ColorTreshold);
    EEPROM.write(MEMORY_FRONT_RANGE_TRESHOLD, FrontRangeTreshold);
    EEPROM.write(MEMORY_SIDE_RANGE_TRESHOLD, FrontSideTreshold);
    EEPROM.write(MEMORY_SEARCH_MODE, (uint8_t)SearchMode);
    EEPROM.write(MEMORY_SEARCH_SPEED, SearchSpeed);
    EEPROM.write(MEMORY_ATACK_SPEED, AtackSpeed);
    
    EEPROM.write(MEMORY_KEY,KEY_OF_MEMORY); 
}

void Memory_Update_Short(){
    EEPROM.write(MEMORY_RING_ID, RingId);
    EEPROM.write(MEMORY_IS_FIGHTING, IsFightingFlag);
}


/************************************   BUTTON AND LED   ************************************************/

void LED(uint8_t _state){
    digitalWrite(LED_PIN, _state);
}

typedef enum{
  SW_NONE,
  SW_SHORT,
  SW_LONG
}sw_t;


uint8_t Switch1(){
  uint16_t _t=0;
  while(digitalRead(BUTTON1_PIN)==0){
    _t++;
    delay(1);
  }

  if(_t>1500) return SW_LONG;
  if(_t>30) return SW_SHORT;
  return SW_NONE;
}

uint8_t Switch2(){
  uint16_t _t=0;
  while(digitalRead(BUTTON2_PIN)==0){
    _t++;
    delay(1);
  }

  if(_t>1500) return SW_LONG;
  if(_t>30) return SW_SHORT;
  return SW_NONE;
}




/************************************   RC5 Sumo Remote   ************************************************/


#include <Wire.h>
#include <VL53L0X.h>

//Analog Sensors
#define BATTERY_PIN         A2     //Pin for Battery voltage measurment
#define COLLOR1_PIN         A6     //Pin for Dohyo Color 1 measurment
#define COLLOR2_PIN         A7     //Pin for Dohyo Color 2 measurment


//VL53l0x sensors gpio
#define VL53L0X_L_RST_PIN   A1     //Left TOF sensor Reset pin 
#define VL53L0X_LF_RST_PIN  12     //Left-Forward TOF sensor Reset pin 
#define VL53L0X_RF_RST_PIN  A3     //Right-Forward TOF sensor Reset pin 
#define VL53L0X_R_RST_PIN   A0     //Right TOF sensor Reset pin 


VL53L0X vl53l0x[4];


/************************************   vl53l0x Range Sensor   ************************************************/

typedef enum{
  SENSOR_L=0,
  SENSOR_LF=1,
  SENSOR_RF=2,
  SENSOR_R=3, 
  SENSOR_COLOR_L,
  SENSOR_COLOR_R,
  SENSOR_BATTERY_PERCENT,
  SENSOR_ANY,
  SENSOR_NUM,
}sensorid_t;


typedef enum{
  STATE_SENSOR_IDLE=254,
  STATE_SENSOR_ERROR=255,
}sensorstate_t;

uint8_t LastSensorsStates[SENSOR_NUM];
uint8_t CurrentSensorsStates[SENSOR_NUM];


void Sensor_Handle(uint8_t _id){

  switch(_id){
    case SENSOR_L:
    case SENSOR_LF:
    case SENSOR_RF:
    case SENSOR_R:
        if(CurrentSensorsStates[_id]!=STATE_SENSOR_ERROR){
              if (vl53l0x[_id].timeoutOccurred()) {
                CurrentSensorsStates[_id]=STATE_SENSOR_IDLE;
              }else{
                CurrentSensorsStates[_id]=vl53l0x[_id].readRangeContinuousMillimeters()/10;
                if(CurrentSensorsStates[_id]==STATE_SENSOR_ERROR) CurrentSensorsStates[_id]=STATE_SENSOR_IDLE;
              }
        }
        break;
        
    case SENSOR_COLOR_L:   
        CurrentSensorsStates[SENSOR_COLOR_L]=analogRead(COLLOR1_PIN)/4;
        break;
        
    case SENSOR_COLOR_R:
         CurrentSensorsStates[SENSOR_COLOR_R]=analogRead(COLLOR2_PIN)/4;
        break;
        
    case SENSOR_BATTERY_PERCENT:
        CurrentSensorsStates[SENSOR_BATTERY_PERCENT]=analogRead(BATTERY_PIN)/4;
        break;
        
    default:
    
        break;
  }
}

uint8_t Sensor_is_New(uint8_t _id){
  
  if(LastSensorsStates[_id]==CurrentSensorsStates[_id]) return 0;
  
  return 1;
}

uint8_t Sensor_Get(uint8_t _id){
  
    LastSensorsStates[_id]=CurrentSensorsStates[_id];

  return CurrentSensorsStates[_id];
}


void Sensor_Init(){
  //initialize analog signal inputs

  //initialize analog signal inputs
  pinMode(BATTERY_PIN, INPUT);
  pinMode(COLLOR1_PIN, INPUT);
  pinMode(COLLOR2_PIN, INPUT);
  
  //initialize TOF gpio
  pinMode(VL53L0X_L_RST_PIN, OUTPUT);      //Left TOF sensor Reset pin 
  pinMode(VL53L0X_LF_RST_PIN, OUTPUT);     //Left-Forward TOF sensor Reset pin 
  pinMode(VL53L0X_RF_RST_PIN, OUTPUT);     //Right-Forward TOF sensor Reset pin 
  pinMode(VL53L0X_R_RST_PIN, OUTPUT);      //Right TOF sensor Reset pin 



  Sensor_Handle(SENSOR_BATTERY_PERCENT);
  Sensor_Handle(SENSOR_COLOR_L);
  Sensor_Handle(SENSOR_COLOR_R);

  digitalWrite(VL53L0X_L_RST_PIN, 0);  
  digitalWrite(VL53L0X_LF_RST_PIN, 0);  
  digitalWrite(VL53L0X_RF_RST_PIN, 0);  
  digitalWrite(VL53L0X_R_RST_PIN, 0);  

  digitalWrite(VL53L0X_L_RST_PIN, 1);
  vl53l0x[SENSOR_L].setTimeout(500);
  if (!vl53l0x[SENSOR_L].init()) {
    CurrentSensorsStates[SENSOR_L]=STATE_SENSOR_ERROR;
  }else{
    CurrentSensorsStates[SENSOR_L]=STATE_SENSOR_IDLE;
    vl53l0x[SENSOR_L].setAddress(0x20);
    vl53l0x[SENSOR_L].startContinuous();
    Sensor_Handle(SENSOR_L);
  }
  
  digitalWrite(VL53L0X_LF_RST_PIN, 1);  
  vl53l0x[SENSOR_LF].setTimeout(500);
  if (!vl53l0x[SENSOR_LF].init()){
    CurrentSensorsStates[SENSOR_LF]=STATE_SENSOR_ERROR;
  }else{
    CurrentSensorsStates[SENSOR_LF]=STATE_SENSOR_IDLE;
    vl53l0x[SENSOR_LF].setAddress(0x21);  
    vl53l0x[SENSOR_LF].startContinuous();
    Sensor_Handle(SENSOR_LF);  
  }

  digitalWrite(VL53L0X_RF_RST_PIN, 1);  
  vl53l0x[SENSOR_RF].setTimeout(500);
  if (!vl53l0x[SENSOR_RF].init()){
    CurrentSensorsStates[SENSOR_RF]=STATE_SENSOR_ERROR;
  }else{
    CurrentSensorsStates[SENSOR_RF]=STATE_SENSOR_IDLE;
    vl53l0x[SENSOR_RF].setAddress(0x22);
    vl53l0x[SENSOR_RF].startContinuous();
    Sensor_Handle(SENSOR_RF);
  }

  digitalWrite(VL53L0X_R_RST_PIN, 1);  
  vl53l0x[SENSOR_R].setTimeout(500);
  if (!vl53l0x[SENSOR_R].init()){
    CurrentSensorsStates[SENSOR_R]=STATE_SENSOR_ERROR;
  }else{
    CurrentSensorsStates[SENSOR_R]=STATE_SENSOR_IDLE;
    vl53l0x[SENSOR_R].setAddress(0x23);
    vl53l0x[SENSOR_R].startContinuous();
    Sensor_Handle(SENSOR_R);
  }

}

/************************************   Setup function for init all of modules   ************************************************/
void setup() {
  
  //initialize serial 
  Serial.begin(UART_SPEED);
  
   //initialize gpio for leds and buttons
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);



  //check memory for ring id and last robot config
  
  if(Memory_Read()==0){
    Memory_Complete_Write(); //vrite current pre set values
  }


  //initialize rc5 signal decoder

  //initialize display
  u8x8.begin();
  //u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);     //fat font
  u8x8.setFont(u8x8_font_chroma48medium8_r);           //slim font
  u8x8.noInverse();
  u8x8.clear();
  /*
     u8x8.inverse();
     u8x8.noInverse();
  u8x8.setCursor(0,1);
   */
  //initialize sensors
  Sensor_Init();

  //initial is done
}


/************************************   loop for fight   ************************************************/

void fight(){
  
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);     //set up fat font
      u8x8.inverse();
      //u8x8.clear();
      u8x8.fillDisplay();

      //calculate position of displayed name
      uint8_t xpos=strlen(ROBOT_NAME)/2;
      
      if(xpos<6){
        xpos=6-xpos; //calculate start position for centric place
      }else{
        xpos=0;
      }
      
      u8x8.draw2x2String(xpos,1,ROBOT_NAME);

      
  while(1){

      //chcek rc5 codes

    if(Switch1()==SW_LONG) break;
    
      
  }

      u8x8.noInverse();
      u8x8.setFont(u8x8_font_chroma48medium8_r);           //set up again slim font
      DisplayUpdated=0;
}




/************************************   main loop for system runloop   ************************************************/


void loop() {

  //chcek rc5 codes

  //handle menu
  if(MenuView==0){
    //display name of robot
    if(DisplayUpdated==0){
      
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);     //set up fat font
      u8x8.clear();

      //calculate position of displayed name
      uint8_t xpos=strlen(ROBOT_NAME)/2;
      
      if(xpos<6){
        xpos=6-xpos; //calculate start position for centric place
      }else{
        xpos=0;
      }
      u8x8.draw2x2String(xpos,1,ROBOT_NAME);

      u8x8.setFont(u8x8_font_chroma48medium8_r);           //set up again slim font
      DisplayUpdated=1;
    }

    switch (Switch1()){
      case SW_NONE:
        break;
      case SW_SHORT:
      case SW_LONG:
        fight();
         DisplayUpdated=0;
        break;
    }

    switch (Switch2()){
      case SW_NONE:
        break;
      case SW_SHORT:
      case SW_LONG:
        DisplayUpdated=0;
        MenuView=1;
        u8x8.clear();
        break;
    }
    
  }

  if(MenuView==1){
    //Sensor View
    if(DisplayUpdated==0){
      
      u8x8.setCursor(0, 0);
      u8x8.print("LF:");  
      u8x8.print(Sensor_Get(SENSOR_LF)); 
      u8x8.print("  ");  
      
      u8x8.setCursor(0, 1);
      u8x8.print(" L:");  
      u8x8.print(Sensor_Get(SENSOR_L)); 
      u8x8.print("  "); 
      
      u8x8.setCursor(8, 0);
      u8x8.print("RF:");  
      u8x8.print(Sensor_Get(SENSOR_RF)); 
      u8x8.print("  "); 

      u8x8.setCursor(8, 1);
      u8x8.print(" R:");  
      u8x8.print(Sensor_Get(SENSOR_R)); 
      u8x8.print("  "); 
      
      u8x8.setCursor(8, 1);
      u8x8.print(" R:");  
      u8x8.print(Sensor_Get(SENSOR_R)); 

      u8x8.setCursor(0, 2);
      u8x8.print("CL:");  
      u8x8.print(Sensor_Get(SENSOR_COLOR_L)); 
      u8x8.print("  "); 
      
      u8x8.setCursor(8, 2);
      u8x8.print("CR:");  
      u8x8.print(Sensor_Get(SENSOR_COLOR_R)); 
      u8x8.print("  "); 
      
      u8x8.setCursor(0, 3);
      u8x8.print("Battery:");  
      u8x8.print(Sensor_Get(SENSOR_BATTERY_PERCENT)); 
      u8x8.print("%  ");   
      
      DisplayUpdated=1;
    }

    //handle sensors
    for (uint8_t i=0;i<SENSOR_NUM; i++){
      Sensor_Handle(i);
      if(Sensor_is_New(i))  DisplayUpdated=0; //if any new data, update display
    }

    switch (Switch2()){
      case SW_NONE:
        break;
      case SW_SHORT:
      case SW_LONG:
        DisplayUpdated=0;
        MenuView=2;
        u8x8.clear();
        break;
    }
    
  }


  if(MenuView==2){
    //Sensor View
    if(DisplayUpdated==0){
      u8x8.drawString(0,0,"Settings...");
      DisplayUpdated=1;
    }

    switch (Switch2()){
      case SW_NONE:
        break;
      case SW_SHORT:
      case SW_LONG:
        DisplayUpdated=0;
        MenuView=0;
        u8x8.clear();
        break;
    }
    
  }

//menu 

  
//Memory_Update_Short()
//Memory_Complete_Write();

  
}
