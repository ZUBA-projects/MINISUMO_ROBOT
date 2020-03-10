

void init_hardware(){
    
  //initialize gpio for leds and buttons
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);

  //initialize motor drivers gpio 
  pinMode(LF_PIN, OUTPUT);
  pinMode(LB_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(RF_PIN, OUTPUT);
  pinMode(RB_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);

  //initialize analog signal inputs
  pinMode(BATTERY_PIN, INPUT);
  pinMode(COLLOR1_PIN, INPUT);
  pinMode(COLLOR2_PIN, INPUT);
    
}
