#indlude "arduino.h"



//Setup function for init all of modules
void setup() {
  
  //initialize serial 
  Serial.init(UART_SPEED);
  
  init_hardware(); //set up gpio, led 

  init_sumoremote_decoder();

  //check memory for ring id and last robot config

  //initialize rc5 signal decoder

  //initialize display

  //initialize vl53l0x distance sensor

  //initial is done
}



//main loop for system runloop
void loop() {
  
  
  
}




void fight_mode(){
  //check switch state
  //check last sumo remote state
  //check vl53l0x sensor state 

  //decision 
  

  
}


void menu_mode(){
  
}
