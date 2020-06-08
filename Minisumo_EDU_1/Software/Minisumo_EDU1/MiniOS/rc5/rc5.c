/*
 * rc5.c
 *
 *  Created on: 05.04.2020
 *      Author: zuba1
 */

#include "../../hal.h"
#include "rc5.h"

rc5struct_t * _rc5struct;


static void RC5_read() {
uint16_t timer_value=0;
  if(_rc5struct->rc5_state != 0){
    timer_value = hal_micros()-_rc5struct->timer;                         // Store Timer1 value
    _rc5struct->timer=hal_micros();                                // Reset Timer1
  }
  switch(_rc5struct->rc5_state){
   case 0 :                                      // Start receiving IR data (initially we're at the beginning of mid1)
	_rc5struct->timer=hal_micros();                                 // Reset Timer1

	// Enable Timer1 module with 1/8 prescaler ( 2 ticks every 1 us)
	//set up new tout
	_rc5struct->tout=RC5_TOUT+hal_millis();

	_rc5struct->rc5_state = 1;                               // Next state: end of mid1
    _rc5struct->j = 0;
    return;
   case 1 :                                      // End of mid1 ==> check if we're at the beginning of start1 or mid0
    if((timer_value > RC5_LONG_TIME) || (timer_value < RC5_SHORT_TIME)){         // Invalid interval ==> stop decoding and reset
    	_rc5struct->rc5_state = 0;                             // Reset decoding process
    	_rc5struct->tout=0;                                 // Disable Timer1 module
      return;
    }
    setbit(_rc5struct->rc5_code, (13 - _rc5struct->j));
    _rc5struct->j++;
    if(_rc5struct->j > 13){                                  // If all bits are received
    	_rc5struct->rc5_ok = 1;                                // Decoding process is OK
    	gpioRemoveInt(_rc5struct->_gpio);                        // Disable external interrupt (INT0)
      return;
    }
      if(timer_value > RC5_MED_TIME){                // We're at the beginning of mid0
    	  _rc5struct->rc5_state = 2;                           // Next state: end of mid0
        if(_rc5struct->j == 13){                             // If we're at the LSB bit
        	_rc5struct->rc5_ok = 1;                            // Decoding process is OK
        	clearbit(_rc5struct->rc5_code, 0);                 // Clear the LSB bit
          gpioRemoveInt(_rc5struct->_gpio);                   // Disable external interrupt (INT0)
          return;
        }
      }
      else                                       // We're at the beginning of start1
    	  _rc5struct->rc5_state = 3;                           // Next state: end of start1
      return;
   case 2 :                                      // End of mid0 ==> check if we're at the beginning of start0 or mid1
    if((timer_value > RC5_LONG_TIME) || (timer_value < RC5_SHORT_TIME)){
    	_rc5struct->rc5_state = 0;                             // Reset decoding process
    	_rc5struct->tout=0;                                 // Disable Timer1 module
      return;
    }
    clearbit(_rc5struct->rc5_code, (13 - _rc5struct->j));
    _rc5struct->j++;
    if(timer_value > RC5_MED_TIME)                   // We're at the beginning of mid1
    	_rc5struct->rc5_state = 1;                             // Next state: end of mid1
    else                                         // We're at the beginning of start0
    	_rc5struct->rc5_state = 4;                             // Next state: end of start0
    return;
   case 3 :                                      // End of start1 ==> check if we're at the beginning of mid1
    if((timer_value > RC5_MED_TIME) || (timer_value < RC5_SHORT_TIME)){           // Time interval invalid ==> stop decoding
    	_rc5struct->tout=0;                                 // Disable Timer1 module
      _rc5struct->rc5_state = 0;                             // Reset decoding process
      return;
    }
    else                                         // We're at the beginning of mid1
    	_rc5struct->rc5_state = 1;                             // Next state: end of mid1
    return;
   case 4 :                                      // End of start0 ==> check if we're at the beginning of mid0
    if((timer_value > RC5_MED_TIME) || (timer_value < RC5_SHORT_TIME)){           // Time interval invalid ==> stop decoding
    	_rc5struct->tout=0;                                 // Disable Timer1 module
      _rc5struct->rc5_state = 0;                             // Reset decoding process
      return;
    }
    else                                         // We're at the beginning of mid0
    	_rc5struct->rc5_state = 2;                             // Next state: end of mid0
    if(_rc5struct->j == 13){                                 // If we're at the LSB bit
    	_rc5struct->rc5_ok = 1;                                // Decoding process is OK
      clearbit(_rc5struct->rc5_code, 0);                     // Clear the LSB bit
      gpioRemoveInt(_rc5struct->_gpio);                        // Disable external interrupt (INT0)
    }
  }
}

void rc5_check() {
	//check tout
		if((_rc5struct->tout!=0)&&(hal_millis()>_rc5struct->tout)){
			_rc5struct->rc5_state = 0;                                 // Reset decoding process
			_rc5struct->tout=0;                                    // Disable Timer1 module
		}


	//check rc5 state
	if(_rc5struct->rc5_ok){                                    // If the mcu receives RC5 message with successful
		_rc5struct->rc5_ok = 0;                                  // Reset decoding process
		_rc5struct->rc5_state = 0;
		//_rc5struct->timeren=0;                                 // Disable Timer1 module
		_rc5struct->toggle_bit = bitRead(_rc5struct->rc5_code, 11);          // Toggle bit is bit number 11
		_rc5struct->address = (_rc5struct->rc5_code >> 6) & 0x1F;            // Next 5 bits are for address
		_rc5struct->command = _rc5struct->rc5_code & 0x3F;                   // The 6 LSBits are command bits
		gpioSetInt(_rc5struct->_gpio, &RC5_read);	// Enable external interrupt (on pin)
	}

}


void rc5_init(rc5struct_t * _struct) {
  _rc5struct=_struct;
  gpioSetInt(_rc5struct->_gpio, &RC5_read);	// Enable external interrupt (on pin)
}

