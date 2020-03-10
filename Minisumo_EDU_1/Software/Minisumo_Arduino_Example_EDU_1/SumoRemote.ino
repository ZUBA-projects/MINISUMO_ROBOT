// RC5 Protocol decoder Arduino code
 
// Define number of Timer1 ticks (with a prescaler of 1/8)
#define short_time     1400                      // Used as a minimum time for short pulse or short space ( ==>  700 us)
#define   med_time     2400                      // Used as a maximum time for short pulse or short space ( ==> 1200 us)
#define  long_time     4000                      // Used as a maximum time for long pulse or long space   ( ==> 2000 us)

boolean rc5_ok = 0, toggle_bit;
byte rc5_state = 0, j, address, command;
unsigned int rc5_code;
 
void setup() {
  // Timer1 module configuration
  TCCR1A = 0;
  TCCR1B = 0;                                    // Disable Timer1 module
  TCNT1  = 0;                                    // Set Timer1 preload value to 0 (reset)
  TIMSK1 = 1;                                    // enable Timer1 overflow interrupt
  attachInterrupt(0, RC5_read, CHANGE);          // Enable external interrupt (INT0)
   Serial.begin(9600);
   Serial.println("READY.");
}
 
void RC5_read() {
unsigned int timer_value;
  if(rc5_state != 0){
    timer_value = TCNT1;                         // Store Timer1 value
    TCNT1 = 0;                                   // Reset Timer1
  }
  switch(rc5_state){
   case 0 :                                      // Start receiving IR data (initially we're at the beginning of mid1)
    TCNT1  = 0;                                  // Reset Timer1
    TCCR1B = 2;                                  // Enable Timer1 module with 1/8 prescaler ( 2 ticks every 1 us)
    rc5_state = 1;                               // Next state: end of mid1
    j = 0;
    return;
   case 1 :                                      // End of mid1 ==> check if we're at the beginning of start1 or mid0
    if((timer_value > long_time) || (timer_value < short_time)){         // Invalid interval ==> stop decoding and reset
      rc5_state = 0;                             // Reset decoding process
      TCCR1B = 0;                                // Disable Timer1 module
      return;
    }
    bitSet(rc5_code, 13 - j);
    j++;
    if(j > 13){                                  // If all bits are received
      rc5_ok = 1;                                // Decoding process is OK
      detachInterrupt(0);                        // Disable external interrupt (INT0)
      return;
    }
      if(timer_value > med_time){                // We're at the beginning of mid0
        rc5_state = 2;                           // Next state: end of mid0
        if(j == 13){                             // If we're at the LSB bit
          rc5_ok = 1;                            // Decoding process is OK
          bitClear(rc5_code, 0);                 // Clear the LSB bit
          detachInterrupt(0);                    // Disable external interrupt (INT0)
          return;
        }
      }
      else                                       // We're at the beginning of start1
        rc5_state = 3;                           // Next state: end of start1
      return;
   case 2 :                                      // End of mid0 ==> check if we're at the beginning of start0 or mid1
    if((timer_value > long_time) || (timer_value < short_time)){
      rc5_state = 0;                             // Reset decoding process
      TCCR1B = 0;                                // Disable Timer1 module
      return;
    }
    bitClear(rc5_code, 13 - j);
    j++;
    if(timer_value > med_time)                   // We're at the beginning of mid1
      rc5_state = 1;                             // Next state: end of mid1
    else                                         // We're at the beginning of start0
      rc5_state = 4;                             // Next state: end of start0
    return;
   case 3 :                                      // End of start1 ==> check if we're at the beginning of mid1
    if((timer_value > med_time) || (timer_value < short_time)){           // Time interval invalid ==> stop decoding
      TCCR1B = 0;                                // Disable Timer1 module
      rc5_state = 0;                             // Reset decoding process
      return;
    }
    else                                         // We're at the beginning of mid1
      rc5_state = 1;                             // Next state: end of mid1
    return;
   case 4 :                                      // End of start0 ==> check if we're at the beginning of mid0
    if((timer_value > med_time) || (timer_value < short_time)){           // Time interval invalid ==> stop decoding
      TCCR1B = 0;                                // Disable Timer1 module
      rc5_state = 0;                             // Reset decoding process
      return;
    }
    else                                         // We're at the beginning of mid0
      rc5_state = 2;                             // Next state: end of mid0
    if(j == 13){                                 // If we're at the LSB bit
      rc5_ok = 1;                                // Decoding process is OK
      bitClear(rc5_code, 0);                     // Clear the LSB bit
      detachInterrupt(0);                        // Disable external interrupt (INT0)
    }
  }
}
 
ISR(TIMER1_OVF_vect) {                           // Timer1 interrupt service routine (ISR)
  rc5_state = 0;                                 // Reset decoding process
  TCCR1B = 0;                                    // Disable Timer1 module
}

typedef enum{
  //sumo remote reg address
  PILOT_PROGRAM =0x0B, //(11 in decimal) (Programming address)
  PILOT_REMOTE =0x07, //(Competition system address, also defined in the standard as Experimental)

}ir_adr;



{
  if(rc5_ok){                                    // If the mcu receives RC5 message with successful
    rc5_ok = 0;                                  // Reset decoding process
    rc5_state = 0;
    TCCR1B = 0;                                  // Disable Timer1 module
    toggle_bit = bitRead(rc5_code, 11);          // Toggle bit is bit number 11
    address = (rc5_code >> 6) & 0x1F;            // Next 5 bits are for address
    command = rc5_code & 0x3F;                   // The 6 LSBits are command bits

    //rc5 data
    Serial.print("\t\tADDR:");
    Serial.print(command);
    Serial.print("\t\tCOMMAND:");
    Serial.print(address);
    Serial.print("\t\tTOGGLE:");
    Serial.println(toggle_bit);
    //decoded values

    switch(address){
      case PILOT_PROGRAM:
            Serial.print("SUMO PROGRAM RING ID:");
            Serial.println((command& 0xFE)>>1);
        break;
        
      case PILOT_REMOTE:
            Serial.print("SUMO REMOTE RING ID:");
            Serial.print((command& 0xFE)>>1);
            if(command& 0x01){
              Serial.println("\t\t START!");
            }else{
              Serial.println("\t\t STOP!");
            }
        break;
    }
    attachInterrupt(0, RC5_read, CHANGE);        // Enable external interrupt (INT0)
  }
}
