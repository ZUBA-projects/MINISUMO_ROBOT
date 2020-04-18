/*
 * motor.c
 *
 *  Created on: 05.04.2020
 *      Author: zuba1
 */

#include "../../hal.h"
#include "motor.h"

motorconfig_t * motorcfglcl;

void Motor(motor_t _move, uint8_t _lpwm, uint8_t _rpwm){

	  switch (_move){
	    case MOVE_STOP:
	      gpioSet(motorcfglcl->lf, 0);
	      gpioSet(motorcfglcl->lb, 0);
	      gpioSet(motorcfglcl->rf, 0);
	      gpioSet(motorcfglcl->rb, 0);
	      break;

	    case MOVE_FORWARD:
	      gpioSet(motorcfglcl->lf, 1);
	      gpioSet(motorcfglcl->lb, 0);
	      gpioSet(motorcfglcl->rf, 1);
	      gpioSet(motorcfglcl->rb, 0);
	      break;

	    case MOVE_BACKWARD:
	      gpioSet(motorcfglcl->lf, 0);
	      gpioSet(motorcfglcl->lb, 1);
	      gpioSet(motorcfglcl->rf, 0);
	      gpioSet(motorcfglcl->rb, 1);
	      break;

	    case MOVE_RIGHT:
	      gpioSet(motorcfglcl->lf, 1);
	      gpioSet(motorcfglcl->lb, 0);
	      gpioSet(motorcfglcl->rf, 0);
	      gpioSet(motorcfglcl->rb, 1);
	      break;

	    case MOVE_LEFT:
	      gpioSet(motorcfglcl->lf, 0);
	      gpioSet(motorcfglcl->lb, 1);
	      gpioSet(motorcfglcl->rf, 1);
	      gpioSet(motorcfglcl->rb, 0);
	      break;

	    case MOVE_HOLD:
	      gpioSet(motorcfglcl->lf, 1);
	      gpioSet(motorcfglcl->lb, 1);
	      gpioSet(motorcfglcl->rf, 1);
	      gpioSet(motorcfglcl->rb, 1);
	      break;

	    default:
	      gpioSet(motorcfglcl->lf, 0);
	      gpioSet(motorcfglcl->lb, 0);
	      gpioSet(motorcfglcl->rf, 0);
	      gpioSet(motorcfglcl->rb, 0);
	      break;
	  }

	  pwmSet(motorcfglcl->lpwm,_lpwm);
	  pwmSet(motorcfglcl->rpwm,_rpwm);

}


void Motor_Init(motorconfig_t * _motorcfg){

	motorcfglcl=_motorcfg;

	//initialize motor drivers gpio
	gpioInit(motorcfglcl->lf, GPIO_OUTPUT);
	gpioInit(motorcfglcl->lb, GPIO_OUTPUT);
	gpioInit(motorcfglcl->lpwm, GPIO_PWM);

	gpioInit(motorcfglcl->rf, GPIO_OUTPUT);
	gpioInit(motorcfglcl->rb, GPIO_OUTPUT);
	gpioInit(motorcfglcl->rpwm, GPIO_PWM);

	Motor(MOVE_STOP, 0, 0);
}
