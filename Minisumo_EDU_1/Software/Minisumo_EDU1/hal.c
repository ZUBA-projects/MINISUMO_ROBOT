/*
 * hal.c
 *
 *  Created on: 07.04.2020
 *      Author: zuba1
 */

#include "hal.h"

//HAl LIBIARY FOR ATMEGA328P

void yield(){} //not used


/***********************************---TRIGONOMETRY---*********************************/

//NO float mode

static const unsigned char trigTab[] MY_PROGMEM ={
	    0x00,0x00,0x02,0x3b,0x04,0x77,0x06,0xb2,0x08,0xed,0x0b,0x27,0x0d,0x61,0x0f,0x99,0x11,0xd0,0x14,0x05,
	    0x16,0x39,0x18,0x6c,0x1a,0x9c,0x1c,0xca,0x1e,0xf7,0x21,0x20,0x23,0x47,0x25,0x6c,0x27,0x8d,0x29,0xab,
	    0x2b,0xc6,0x2d,0xde,0x2f,0xf2,0x32,0x03,0x34,0x0f,0x36,0x17,0x38,0x1c,0x3a,0x1b,0x3c,0x17,0x3e,0x0d,
	    0x3f,0xff,0x41,0xec,0x43,0xd3,0x45,0xb6,0x47,0x93,0x49,0x6a,0x4b,0x3b,0x4d,0x07,0x4e,0xcd,0x50,0x8c,
	    0x52,0x46,0x53,0xf9,0x55,0xa5,0x57,0x4b,0x58,0xe9,0x5a,0x81,0x5c,0x12,0x5d,0x9c,0x5f,0x1e,0x60,0x99,
	    0x62,0x0c,0x63,0x78,0x64,0xdc,0x66,0x38,0x67,0x8d,0x68,0xd9,0x6a,0x1d,0x6b,0x58,0x6c,0x8b,0x6d,0xb6,
	    0x6e,0xd9,0x6f,0xf2,0x71,0x03,0x72,0x0b,0x73,0x0a,0x74,0x00,0x74,0xee,0x75,0xd2,0x76,0xad,0x77,0x7e,
	    0x78,0x46,0x79,0x05,0x79,0xbb,0x7a,0x67,0x7b,0x09,0x7b,0xa2,0x7c,0x31,0x7c,0xb7,0x7d,0x32,0x7d,0xa4,
	    0x7e,0x0d,0x7e,0x6b,0x7e,0xc0,0x7f,0x0a,0x7f,0x4b,0x7f,0x82,0x7f,0xaf,0x7f,0xd2,0x7f,0xeb,0x7f,0xfa,
	    0x7f,0xff,
	};

uint16_t read_sine_tab(uint16_t _deg){

	_deg=_deg*2;

	uint16_t _ret=(my_pgm_read_byte(trigTab + _deg) << 8) | my_pgm_read_byte(trigTab + _deg + 1);
	return _ret;
}


int16_t hal_sin_deg_int16(int16_t _deg){

    //normalize deg to range from 0 to 360*

    while(_deg<0) {_deg+=360;}
    while(_deg>360) {_deg-=360;}

    //normalize to tab size and find quarter of sine

    uint8_t n=0;

    while(_deg>90){
        _deg=_deg-90;
        n++;
    }

    //conv deg to radians
    //_deg=_deg/57.29577951 -we dont need to convert deg to radians because tabele is allready organized as degrees values

    //lookup to tabele in correct mode

    switch(n){
        case 0:
        	return read_sine_tab(_deg);

        case 1:
        	return read_sine_tab(90 - _deg);

        case 2:
            return 0 - read_sine_tab(_deg);

        case 3:
        	return 0 - read_sine_tab(90 - _deg);
    }

return 0;
}


int16_t hal_cos_deg_int16(int16_t _deg){
    return hal_sin_deg_int16(_deg+90);
}



/***********************************---GPIO---*********************************/

//62,3/31.3
typedef enum{
	TIMER0A,
	TIMER0B,
	TIMER1A,
	TIMER1B,
	TIMER1C,
	TIMER2,
	TIMER2A,
	TIMER2B,
	TIMER3A,
	TIMER3B,
	TIMER3C,
	TIMER4A,
	TIMER4B,
	TIMER4C,
	TIMER4D,
	TIMER5A,
	TIMER5B,
	TIMER5C,
	NOT_ON_TIMER,
}timers_t;

typedef enum{
	PORTAX,
	PORTBX,
	PORTCX,
	PORTDX,
	PORTEX,
	PORTFX,
	NOT_ON_PORT,
}ports_t;


timers_t gpio_to_timer(uint8_t _gpio){

	switch(_gpio){
		case D3:
			return TIMER2B;
			break;

		case D5:
			return TIMER0B;
			break;

		case D6:
			return TIMER0A;
			break;

		case D9:
			return TIMER1A;
			break;

		case D10:
			return TIMER1B;
			break;

		case D11:
			return TIMER2A;
			break;

		default:
			return NOT_ON_TIMER;
			break;
	}
}

ports_t gpio_to_info(uint8_t _gpio){

	if(_gpio<D8) return PORTAX;
	if(_gpio<D_LAST) return PORTBX;
	if(_gpio<A_LAST) return PORTCX;
	return NOT_ON_PORT;
}


uint8_t gpio_to_num(uint8_t _gpio){

    if(_gpio>D13) _gpio+=2;

	if(_gpio)
	while(_gpio>7){
		_gpio-=8;
	}
	return (_gpio);
}

void gpioSet(uint8_t _gpio, uint8_t _state){

	if(_gpio!=NO_GPIO){

		switch(gpio_to_info(_gpio)){
			case PORTAX:
				if(_state){
					setbit(PORTD,gpio_to_num(_gpio));//PORTD|=(1UL<<gpio_to_num(_gpio));
				}else{
					clearbit(PORTD,gpio_to_num(_gpio));//PORTD&=~(1UL<<gpio_to_num(_gpio));
				}
				break;
			case PORTBX:
				if(_state){
					setbit(PORTB,gpio_to_num(_gpio));//PORTB|=(1UL<<gpio_to_num(_gpio));
				}else{
					clearbit(PORTB,gpio_to_num(_gpio));//PORTB&=~(1UL<<gpio_to_num(_gpio));
				}
				break;
			case PORTCX:
				if(_state){
					setbit(PORTC,gpio_to_num(_gpio));//PORTC|=(1UL<<gpio_to_num(_gpio));
				}else{
					clearbit(PORTC,gpio_to_num(_gpio));//PORTC&=~(1UL<<gpio_to_num(_gpio));
				}
				break;
			default:
				break;
		}
	}
}

uint8_t gpioGet(uint8_t _gpio){
	if(_gpio!=NO_GPIO){
		switch(gpio_to_info(_gpio)){
			case PORTAX:
				return bitRead(PIND, gpio_to_num(_gpio));//PIND & (1UL<<gpio_to_num(_gpio));
				break;
			case PORTBX:
				return bitRead(PINB, gpio_to_num(_gpio));//PINB & (1UL<<gpio_to_num(_gpio));
				break;
			case PORTCX:
				return bitRead(PINC, gpio_to_num(_gpio));//PINC & (1UL<<gpio_to_num(_gpio));
				break;
			default:
				break;
		}
	}
	return 0;
}

void pwmSet(uint8_t _gpio, uint8_t _value){

	if (_value == 0)
	{
		gpioSet( _gpio, 0);
	}
	else if (_value == 255)
	{
		gpioSet( _gpio, 1);
	}
	else
	{
		switch(gpio_to_timer(_gpio))
		{
			// XXX fix needed for atmega8
			#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
			case TIMER0A:
				// connect pwm to pin on timer 0
				sbi(TCCR0, COM00);
				OCR0 = val; // set pwm duty
				break;
			#endif

			#if defined(TCCR0A) && defined(COM0A1)
			case TIMER0A:
				// connect pwm to pin on timer 0, channel A
				sbi(TCCR0A, COM0A1);
				OCR0A = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR0A) && defined(COM0B1)
			case TIMER0B:
				// connect pwm to pin on timer 0, channel B
				sbi(TCCR0A, COM0B1);
				OCR0B = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR1A) && defined(COM1A1)
			case TIMER1A:
				// connect pwm to pin on timer 1, channel A
				sbi(TCCR1A, COM1A1);
				OCR1A = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR1A) && defined(COM1B1)
			case TIMER1B:
				// connect pwm to pin on timer 1, channel B
				sbi(TCCR1A, COM1B1);
				OCR1B = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR1A) && defined(COM1C1)
			case TIMER1C:
				// connect pwm to pin on timer 1, channel B
				sbi(TCCR1A, COM1C1);
				OCR1C = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR2) && defined(COM21)
			case TIMER2:
				// connect pwm to pin on timer 2
				sbi(TCCR2, COM21);
				OCR2 = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR2A) && defined(COM2A1)
			case TIMER2A:
				// connect pwm to pin on timer 2, channel A
				sbi(TCCR2A, COM2A1);
				OCR2A = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR2A) && defined(COM2B1)
			case TIMER2B:
				// connect pwm to pin on timer 2, channel B
				sbi(TCCR2A, COM2B1);
				OCR2B = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR3A) && defined(COM3A1)
			case TIMER3A:
				// connect pwm to pin on timer 3, channel A
				sbi(TCCR3A, COM3A1);
				OCR3A = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR3A) && defined(COM3B1)
			case TIMER3B:
				// connect pwm to pin on timer 3, channel B
				sbi(TCCR3A, COM3B1);
				OCR3B = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR3A) && defined(COM3C1)
			case TIMER3C:
				// connect pwm to pin on timer 3, channel C
				sbi(TCCR3A, COM3C1);
				OCR3C = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR4A)
			case TIMER4A:
				//connect pwm to pin on timer 4, channel A
				sbi(TCCR4A, COM4A1);
				#if defined(COM4A0)		// only used on 32U4
				cbi(TCCR4A, COM4A0);
				#endif
				OCR4A = _value;	// set pwm duty
				break;
			#endif

			#if defined(TCCR4A) && defined(COM4B1)
			case TIMER4B:
				// connect pwm to pin on timer 4, channel B
				sbi(TCCR4A, COM4B1);
				OCR4B = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR4A) && defined(COM4C1)
			case TIMER4C:
				// connect pwm to pin on timer 4, channel C
				sbi(TCCR4A, COM4C1);
				OCR4C = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR4C) && defined(COM4D1)
			case TIMER4D:
				// connect pwm to pin on timer 4, channel D
				sbi(TCCR4C, COM4D1);
				#if defined(COM4D0)		// only used on 32U4
				cbi(TCCR4C, COM4D0);
				#endif
				OCR4D = _value;	// set pwm duty
				break;
			#endif


			#if defined(TCCR5A) && defined(COM5A1)
			case TIMER5A:
				// connect pwm to pin on timer 5, channel A
				sbi(TCCR5A, COM5A1);
				OCR5A = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR5A) && defined(COM5B1)
			case TIMER5B:
				// connect pwm to pin on timer 5, channel B
				sbi(TCCR5A, COM5B1);
				OCR5B = _value; // set pwm duty
				break;
			#endif

			#if defined(TCCR5A) && defined(COM5C1)
			case TIMER5C:
				// connect pwm to pin on timer 5, channel C
				sbi(TCCR5A, COM5C1);
				OCR5C = _value; // set pwm duty
				break;
			#endif

			case NOT_ON_TIMER:
			default:
				if (_value < 128) {
					gpioSet( _gpio, 0);
				} else {
					gpioSet( _gpio, 1);
				}
		}
	}
}

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  #define DEFAULT 0
  #define EXTERNAL 1
  #define INTERNAL1V1 2
  #define INTERNAL INTERNAL1V1
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  #define DEFAULT 0
  #define EXTERNAL 4
  #define INTERNAL1V1 8
  #define INTERNAL INTERNAL1V1
  #define INTERNAL2V56 9
  #define INTERNAL2V56_EXTCAP 13
#else
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
#define INTERNAL1V1 2
#define INTERNAL2V56 3
#else
#define INTERNAL 3
#endif
#define DEFAULT 1
#define EXTERNAL 0
#endif


uint8_t analog_reference = DEFAULT;

void analogReference(uint8_t mode)
{
	// can't actually set the register here because the default setting
	// will connect AVCC and the AREF pin, which would cause a short if
	// there's something connected to AREF.
	analog_reference = mode;
}

uint16_t adcGet(uint8_t _gpio){

	uint8_t low, high;

#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
	if (pin >= A0) pin -= D_LAST; // allow for channel or pin numbers
#endif
	pin = analogPinToChannel(pin);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	if (pin >= A0) pin -= D_LAST; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
	if (pin >= A0) pin -= D_LAST; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
	if (pin >= A0) pin -= D_LAST; // allow for channel or pin numbers
#else
	if (_gpio >= A0) _gpio -= D_LAST; // allow for channel or pin numbers
#endif

#if defined(ADCSRB) && defined(MUX5)
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif

	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
#if defined(ADMUX)
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = (analog_reference << 4) | (pin & 0x07);
#else
	ADMUX = (analog_reference << 6) | (_gpio & 0x07);
#endif
#endif

	// without a delay, we seem to read from the wrong channel
	//delay(1);

#if defined(ADCSRA) && defined(ADCL)
	// start the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low  = ADCL;
	high = ADCH;
#else
	// we dont have an ADC, return 0
	low  = 0;
	high = 0;
#endif

	// combine the two bytes
	return (high << 8) | low;
}




void gpioInit(uint8_t _gpio, gpioportmode_t _mode){

	if(_gpio!=NO_GPIO){
		switch(_mode){
			case GPIO_ADC:
			case GPIO_INPUT:
				/*
				if(gpio_to_info(_gpio)==PORTAX) DDRD&=~(1UL<<gpio_to_num(_gpio));
				if(gpio_to_info(_gpio)==PORTBX) DDRB&=~(1UL<<gpio_to_num(_gpio));
				if(gpio_to_info(_gpio)==PORTCX) DDRC&=~(1UL<<gpio_to_num(_gpio));
				*/

				if(gpio_to_info(_gpio)==PORTAX) clearbit(DDRD,gpio_to_num(_gpio));
				if(gpio_to_info(_gpio)==PORTBX) clearbit(DDRB,gpio_to_num(_gpio));
				if(gpio_to_info(_gpio)==PORTCX) clearbit(DDRC,gpio_to_num(_gpio));
				break;

			case GPIO_PWM:
			case GPIO_OUTPUT:
				/*
				if(gpio_to_info(_gpio)==PORTAX) DDRD|=(1UL<<gpio_to_num(_gpio));
				if(gpio_to_info(_gpio)==PORTBX) DDRB|=(1UL<<gpio_to_num(_gpio));
				if(gpio_to_info(_gpio)==PORTCX) DDRC|=(1UL<<gpio_to_num(_gpio));
				*/
				if(gpio_to_info(_gpio)==PORTAX) setbit(DDRD,gpio_to_num(_gpio));
				if(gpio_to_info(_gpio)==PORTBX) setbit(DDRB,gpio_to_num(_gpio));
				if(gpio_to_info(_gpio)==PORTCX) setbit(DDRC,gpio_to_num(_gpio));
				break;

			default:
				break;
		}

	}
}


call_backINT INT0_ev_callback=NULL;
call_backINT INT1_ev_callback=NULL;

ISR (INT0_vect)
{
	if(INT0_ev_callback!=NULL){
		INT0_ev_callback(D2, gpioGet(D2));
	}
}

ISR (INT1_vect)
{
	if(INT1_ev_callback!=NULL){
		INT1_ev_callback(D3, gpioGet(D3));
	}
}

void gpioSetInt(uint8_t _gpio, void * callback_ev){


	gpioInit( _gpio, GPIO_INPUT);

	switch(_gpio){

		case D2:
			INT0_ev_callback=callback_ev;
			sbi(EICRA,ISC00);//any chnage ISR
			sbi(EIMSK,INT0);
			sei();
			break;

		case D3:
			INT1_ev_callback=callback_ev;
			sbi(EICRA,ISC00);//any chnage ISR
			sbi(EIMSK,INT1);
			sei();
			break;

		default:
			break;
	}
}

void gpioRemoveInt(uint8_t _gpio){

	switch(_gpio){

		case D2:
			cbi(EIMSK,INT0);
			INT0_ev_callback=NULL;
			break;

		case D3:
			cbi(EIMSK,INT1);
			INT1_ev_callback=NULL;
			break;

		default:
			break;
	}
}



/***********************************---TIME---*********************************/


#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile uint32_t timer0_overflow_count = 0;
volatile uint32_t timer0_millis = 0;
static uint8_t timer0_fract = 0;


#if defined(TIM0_OVF_vect)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
	uint32_t m = timer0_millis;
	uint8_t f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}



uint32_t hal_millis(){
	uint32_t m;
	uint8_t oldSREG = SREG;
	cli();
	m = timer0_millis;
	SREG = oldSREG;
	return m;
}



uint32_t hal_micros() {
	uint32_t m;
	uint8_t oldSREG = SREG, t;

	cli();
	m = timer0_overflow_count;
#if defined(TCNT0)
	t = TCNT0;
#elif defined(TCNT0L)
	t = TCNT0L;
#else
	#error TIMER 0 not defined
#endif

#ifdef TIFR0
	if ((TIFR0 & _BV(TOV0)) && (t < 255))
		m++;
#else
	if ((TIFR & _BV(TOV0)) && (t < 255))
		m++;
#endif

	SREG = oldSREG;

	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

void hal_delay(uint32_t _time){
	uint32_t start = hal_micros();

	while (_time > 0) {
		yield();
		while ( _time > 0 && (hal_micros() - start) >= 1000) {
			_time--;
			start += 1000;
		}
	}
}

/***********************************---I2C---*********************************/

#define Prescaler 1
#define F_SCL	400000
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

uint8_t __scl=0;
uint8_t __sda=0;
uint8_t __addr=0;

//static volatile uint8_t twi_state;
/*
uint8_t TWIGetStatus(void)
{
    //uint8_t statuss;
    //statuss = TWSR & 0xF8;
    //return statuss;
	return TW_STATUS;
}*/
/*
void twi_reply(uint8_t ack)
{
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}
*/
/*
void twi_releaseBus(void)
{
  // release bus
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

  // update twi state
  //twi_state = TWI_READY;
}
*/
/*
 void twi_setAddress(uint8_t address)
{
  // set twi slave address (skip over TWGCE bit)
  TWAR = address << 1;
}

void twi_setFrequency(uint32_t frequency)
{
  TWBR = ((F_CPU / frequency) - 16) / 2;
}

 */



uint8_t i2c_Write(uint8_t _data){
	/*
    TWDR = _data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0){
    	//if(TW_STATUS!=TW_MT_DATA_ACK) error();
    }*/
	// load data into data register
	TWDR = _data;
	// start transmission of data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );

	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }

	return 0;
}

uint8_t i2c_Read(i2cack_t _ack){
  if(_ack==I2C_ACK_HAL){
	  //ack
	  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
  }else{
	  //nack
	  TWCR = (1<<TWINT)|(1<<TWEN);
  }

 // uint32_t _tout=hal_millis();

  // wait for end of transmission
  	while( !(TWCR & (1<<TWINT)) ){
/*
    	if((_tout+HAL_I2C_TIMEOUT)<hal_millis()){
    		return 0;
    	}*/

    	//if((TW_STATUS==TW_MR_DATA_ACK)||(TW_STATUS==TW_MR_DATA_NACK)) return TWDR;
    	//if(TW_STATUS==TW_MR_ARB_LOST) return 0;


    }

    return TWDR;
}


uint8_t i2c_Start(uint8_t _scl, uint8_t _sda, uint8_t _addr){
	/*
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while ((TWCR & (1<<TWINT)) == 0){
	  //if(TW_STATUS!=TW_STAR) error();
  }
  i2c_Write(_addr);*/

	// reset TWI control register
	TWCR = 0;
	// transmit START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );

	// check if the start condition was successfully transmitted
	if((TWSR & 0xF8) != TW_START){ return 1; }

	// load slave address into data register
	TWDR = _addr;
	// start transmission of address
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );

	// check if the device has acknowledged the READ / WRITE mode
	uint8_t twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;
}

void i2c_Stop(){
  TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	/*
	  // send stop condition
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

	  // wait for stop condition to be exectued on bus
	  // TWINT is not set after a stop condition!
	  while(TWCR & _BV(TWSTO)){
	    continue;
	  }

	  // update twi state
	  //twi_state = TWI_READY;*/
}


/*
ISR(TWI_vect)
{
  switch(TW_STATUS){
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
      // copy device address and r/w bit to output register and ack
      TWDR = twi_slarw;
      twi_reply(1);
      break;

    // Master Transmitter
    case TW_MT_SLA_ACK:  // slave receiver acked address
    case TW_MT_DATA_ACK: // slave receiver acked data
      // if there is data to send, send it, otherwise stop
      if(twi_masterBufferIndex < twi_masterBufferLength){
        // copy data to output register and ack
        TWDR = twi_masterBuffer[twi_masterBufferIndex++];
        twi_reply(1);
      }else{
	if (twi_sendStop)
          twi_stop();
	else {
	  twi_inRepStart = 1;	// we're gonna send the START
	  // don't enable the interrupt. We'll generate the start, but we
	  // avoid handling the interrupt until we're in the next transaction,
	  // at the point where we would normally issue the start.
	  TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
	  twi_state = TWI_READY;
	}
      }
      break;
    case TW_MT_SLA_NACK:  // address sent, nack received
      twi_error = TW_MT_SLA_NACK;
      twi_stop();
      break;
    case TW_MT_DATA_NACK: // data sent, nack received
      twi_error = TW_MT_DATA_NACK;
      twi_stop();
      break;
    case TW_MT_ARB_LOST: // lost bus arbitration
      twi_error = TW_MT_ARB_LOST;
      twi_releaseBus();
      break;

    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
      // put byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
      //no break
    case TW_MR_SLA_ACK:  // address sent, ack received
      // ack if more bytes are expected, otherwise nack
      if(twi_masterBufferIndex < twi_masterBufferLength){
        twi_reply(1);
      }else{
        twi_reply(0);
      }
      break;
    case TW_MR_DATA_NACK: // data received, nack sent
      // put final byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
	if (twi_sendStop)
          twi_stop();
	else {
	  twi_inRepStart = 1;	// we're gonna send the START
	  // don't enable the interrupt. We'll generate the start, but we
	  // avoid handling the interrupt until we're in the next transaction,
	  // at the point where we would normally issue the start.
	  TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
	  twi_state = TWI_READY;
	}
	break;
    case TW_MR_SLA_NACK: // address sent, nack received
      twi_stop();
      break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
      // enter slave receiver mode
      twi_state = TWI_SRX;
      // indicate that rx buffer can be overwritten and ack
      twi_rxBufferIndex = 0;
      twi_reply(1);
      break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
      // if there is still room in the rx buffer
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        // put byte in buffer and ack
        twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
        twi_reply(1);
      }else{
        // otherwise nack
        twi_reply(0);
      }
      break;
    case TW_SR_STOP: // stop or repeated start condition received
      // ack future responses and leave slave receiver state
      twi_releaseBus();
      // put a null char after data if there's room
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        twi_rxBuffer[twi_rxBufferIndex] = '\0';
      }
      // callback to user defined callback
      twi_onSlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
      // since we submit rx buffer to "wire" library, we can reset it
      twi_rxBufferIndex = 0;
      break;
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
      // nack back at master
      twi_reply(0);
      break;

    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
      // enter slave transmitter mode
      twi_state = TWI_STX;
      // ready the tx buffer index for iteration
      twi_txBufferIndex = 0;
      // set tx buffer length to be zero, to verify if user changes it
      twi_txBufferLength = 0;
      // request for txBuffer to be filled and length to be set
      // note: user must call twi_transmit(bytes, length) to do this
      twi_onSlaveTransmit();
      // if they didn't change buffer & length, initialize it
      if(0 == twi_txBufferLength){
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }

      // transmit first byte from buffer, fall
      //no break
    case TW_ST_DATA_ACK: // byte sent, ack returned
      // copy data to output register
      TWDR = twi_txBuffer[twi_txBufferIndex++];
      // if there is more to send, ack, otherwise nack
      if(twi_txBufferIndex < twi_txBufferLength){
        twi_reply(1);
      }else{
        twi_reply(0);
      }
      break;
    case TW_ST_DATA_NACK: // received nack, we are done
    case TW_ST_LAST_DATA: // received ack, but we are done already!
      // ack future responses
      twi_reply(1);
      // leave slave receiver state
      twi_state = TWI_READY;
      break;

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      twi_error = TW_BUS_ERROR;
      twi_stop();
      break;
  }
}
*/



void i2c_set_reg_instance(uint8_t _scl, uint8_t _sda, uint8_t _addr){
	__scl=_scl;
	__sda=_sda;
	__addr=_addr;
}

// Write an 8-bit register
void i2c_writeReg(uint8_t reg, uint8_t value) {
	i2c_Start(0,0,__addr | I2C_WRITE_HAL);
	i2c_Write(reg);
	i2c_Write(value);
	i2c_Stop();
}

// Write a 16-bit register
void i2c_writeReg16Bit(uint8_t reg, uint16_t value){
	i2c_Start(0,0,__addr | I2C_WRITE_HAL);
	i2c_Write(reg);
	i2c_Write((value >> 8) & 0xFF);
	i2c_Write((value     ) & 0xFF);
	i2c_Stop();
}

// Write a 32-bit register
void i2c_writeReg32Bit(uint8_t reg, uint32_t value){
	i2c_Start(0,0,__addr | I2C_WRITE_HAL);
	i2c_Write(reg);
	i2c_Write((value >>24) & 0xFF);
	i2c_Write((value >>16) & 0xFF);
	i2c_Write((value >> 8) & 0xFF);
	i2c_Write((value     ) & 0xFF);
	i2c_Stop();
}

// Read an 8-bit register
uint8_t i2c_readReg(uint8_t reg) {
  uint8_t value;
  i2c_Start(0,0,__addr | I2C_WRITE_HAL);
  i2c_Write(reg);
  //i2c_Stop();
  i2c_Start(0,0,__addr | I2C_READ_HAL);
  value = i2c_Read(I2C_NACK_HAL);
  i2c_Stop();
  return value;
}

// Read a 16-bit register
uint16_t i2c_readReg16Bit(uint8_t reg) {
  uint16_t value;
  i2c_Start(0,0,__addr | I2C_WRITE_HAL);
  i2c_Write( reg );
  //i2c_Stop();
  i2c_Start(0,0,__addr | I2C_READ_HAL);
  value  = i2c_Read(I2C_ACK_HAL) << 8;
  value |= i2c_Read(I2C_NACK_HAL);
  i2c_Stop();
  return value;
}

// Read a 32-bit register
uint32_t i2c_readReg32Bit(uint8_t reg) {
  uint32_t value;
  i2c_Start(0,0,__addr | I2C_WRITE_HAL);
  i2c_Write( reg );
  //i2c_Stop();
  i2c_Start(0,0,__addr | I2C_READ_HAL);
  value  = (uint32_t)i2c_Read(I2C_ACK_HAL) <<24;
  value |= (uint32_t)i2c_Read(I2C_ACK_HAL) <<16;
  value |= (uint32_t)i2c_Read(I2C_ACK_HAL) << 8;
  value |= i2c_Read(I2C_NACK_HAL);
  i2c_Stop();
  return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
/*
void i2c_writeMulti(uint8_t reg, uint8_t const *src, uint16_t count){
	i2c_Start(0,0,__addr | I2C_WRITE_HAL);
	i2c_Write( reg );
	while ( count-- > 0 ) {
	i2c_Write( *src++ );
	}
	i2c_Stop();
}*/

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void i2c_writeMulti(uint8_t reg, uint8_t const *src, uint16_t count,uint8_t isrestart){
	i2c_Start(0,0,__addr | I2C_WRITE_HAL);
	i2c_Write( reg );
	if(isrestart) i2c_Write(__addr | I2C_WRITE_HAL);
	while ( count-- > 0 ) {
		i2c_Write( *src++ );
	}
	i2c_Stop();
}


// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void i2c_readMulti(uint8_t reg, uint8_t * dst, uint16_t count) {
	i2c_Start(0,0,__addr | I2C_WRITE_HAL);
  i2c_Write( reg );
  //i2c_Stop();
  i2c_Start(0,0,__addr | I2C_READ_HAL);
  while ( count > 0 ) {
    if ( count > 1 ){
      *dst++ = i2c_Read(I2C_ACK_HAL);
    } else {
      *dst++ = i2c_Read(I2C_NACK_HAL);
    }
    count--;
  }
  i2c_Stop();
}




/***********************************---HAL_INIT---*********************************/

void init_hal(){


	// this needs to be called before setup() or some functions won't
	// work there
	sei();

	// on the ATmega168, timer 0 is also used for fast hardware pwm
	// (using phase-correct PWM would mean that timer 0 overflowed half as often
	// resulting in different millis() behavior on the ATmega8 and ATmega168)
#if defined(TCCR0A) && defined(WGM01)
	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);
#endif

	// set timer 0 prescale factor to 64
#if defined(__AVR_ATmega128__)
	// CPU specific: different values for the ATmega128
	sbi(TCCR0, CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
	// this combination is for the standard atmega8
	sbi(TCCR0, CS01);
	sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
	// this combination is for the standard 168/328/1280/2560
	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
	// this combination is for the __AVR_ATmega645__ series
	sbi(TCCR0A, CS01);
	sbi(TCCR0A, CS00);
#else
	#error Timer 0 prescale factor 64 not set correctly
#endif

	// enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
	sbi(TIMSK, TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
	sbi(TIMSK0, TOIE0);
#else
	#error	Timer 0 overflow interrupt not set correctly
#endif

	// timers 1 and 2 are used for phase-correct hardware pwm
	// this is better for motors as it ensures an even waveform
	// note, however, that fast pwm mode can achieve a frequency of up
	// 8 MHz (with a 16 MHz clock) at 50% duty cycle

#if defined(TCCR1B) && defined(CS11) && defined(CS10)
	TCCR1B = 0;

	// set timer 1 prescale factor to 64
	sbi(TCCR1B, CS11);
#if F_CPU >= 8000000L
	sbi(TCCR1B, CS10);
#endif
#elif defined(TCCR1) && defined(CS11) && defined(CS10)
	sbi(TCCR1, CS11);
#if F_CPU >= 8000000L
	sbi(TCCR1, CS10);
#endif
#endif
	// put timer 1 in 8-bit phase correct pwm mode
#if defined(TCCR1A) && defined(WGM10)
	sbi(TCCR1A, WGM10);
#endif

	// set timer 2 prescale factor to 64
#if defined(TCCR2) && defined(CS22)
	sbi(TCCR2, CS22);
#elif defined(TCCR2B) && defined(CS22)
	sbi(TCCR2B, CS22);
//#else
	// Timer 2 not finished (may not be present on this CPU)
#endif

	// configure timer 2 for phase correct pwm (8-bit)
#if defined(TCCR2) && defined(WGM20)
	sbi(TCCR2, WGM20);
#elif defined(TCCR2A) && defined(WGM20)
	sbi(TCCR2A, WGM20);
//#else
	// Timer 2 not finished (may not be present on this CPU)
#endif

#if defined(TCCR3B) && defined(CS31) && defined(WGM30)
	sbi(TCCR3B, CS31);		// set timer 3 prescale factor to 64
	sbi(TCCR3B, CS30);
	sbi(TCCR3A, WGM30);		// put timer 3 in 8-bit phase correct pwm mode
#endif

#if defined(TCCR4A) && defined(TCCR4B) && defined(TCCR4D) /* beginning of timer4 block for 32U4 and similar */
	sbi(TCCR4B, CS42);		// set timer4 prescale factor to 64
	sbi(TCCR4B, CS41);
	sbi(TCCR4B, CS40);
	sbi(TCCR4D, WGM40);		// put timer 4 in phase- and frequency-correct PWM mode
	sbi(TCCR4A, PWM4A);		// enable PWM mode for comparator OCR4A
	sbi(TCCR4C, PWM4D);		// enable PWM mode for comparator OCR4D
#else /* beginning of timer4 block for ATMEGA1280 and ATMEGA2560 */
#if defined(TCCR4B) && defined(CS41) && defined(WGM40)
	sbi(TCCR4B, CS41);		// set timer 4 prescale factor to 64
	sbi(TCCR4B, CS40);
	sbi(TCCR4A, WGM40);		// put timer 4 in 8-bit phase correct pwm mode
#endif
#endif /* end timer4 block for ATMEGA1280/2560 and similar */

#if defined(TCCR5B) && defined(CS51) && defined(WGM50)
	sbi(TCCR5B, CS51);		// set timer 5 prescale factor to 64
	sbi(TCCR5B, CS50);
	sbi(TCCR5A, WGM50);		// put timer 5 in 8-bit phase correct pwm mode
#endif

#if defined(ADCSRA)
	// set a2d prescaler so we are inside the desired 50-200 KHz range.
	#if F_CPU >= 16000000 // 16 MHz / 128 = 125 KHz
		sbi(ADCSRA, ADPS2);
		sbi(ADCSRA, ADPS1);
		sbi(ADCSRA, ADPS0);
	#elif F_CPU >= 8000000 // 8 MHz / 64 = 125 KHz
		sbi(ADCSRA, ADPS2);
		sbi(ADCSRA, ADPS1);
		cbi(ADCSRA, ADPS0);
	#elif F_CPU >= 4000000 // 4 MHz / 32 = 125 KHz
		sbi(ADCSRA, ADPS2);
		cbi(ADCSRA, ADPS1);
		sbi(ADCSRA, ADPS0);
	#elif F_CPU >= 2000000 // 2 MHz / 16 = 125 KHz
		sbi(ADCSRA, ADPS2);
		cbi(ADCSRA, ADPS1);
		cbi(ADCSRA, ADPS0);
	#elif F_CPU >= 1000000 // 1 MHz / 8 = 125 KHz
		cbi(ADCSRA, ADPS2);
		sbi(ADCSRA, ADPS1);
		sbi(ADCSRA, ADPS0);
	#else // 128 kHz / 2 = 64 KHz -> This is the closest you can get, the prescaler is 2
		cbi(ADCSRA, ADPS2);
		cbi(ADCSRA, ADPS1);
		sbi(ADCSRA, ADPS0);
	#endif
	// enable a2d conversions
	sbi(ADCSRA, ADEN);
#endif

	// the bootloader connects pins 0 and 1 to the USART; disconnect them
	// here so they can be used as normal digital i/o; they will be
	// reconnected in Serial.begin()
#if defined(UCSRB)
	UCSRB = 0;
#elif defined(UCSR0B)
	UCSR0B = 0;
#endif


/*

  //init i2c 

    //set SCL to 400kHz
    TWSR = 0x00;
    TWBR = 0x0C;
    //enable TWI
    TWCR = (1<<TWEN);
*/

	TWBR = (uint8_t)TWBR_val;


/*
	  // initialize state
	  twi_state = TWI_READY;
	  twi_sendStop = 1;		// default value
	  twi_inRepStart = 0;

	  // activate internal pullups for twi.
	  //digitalWrite(SDA, 1);
	  //digitalWrite(SCL, 1);

	  // initialize twi prescaler and bit rate
	  cbi(TWSR, TWPS0);
	  cbi(TWSR, TWPS1);
	  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;



	  // enable twi module, acks, and twi interrupt
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
	  */
}

