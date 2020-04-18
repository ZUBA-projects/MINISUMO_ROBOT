/*
 * motor.h
 *
 *  Created on: 05.04.2020
 *      Author: zuba1
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "../../hal.h"

typedef struct{

	//gpio cfg
	uint8_t lpwm;
	uint8_t lf;
	uint8_t lb;

	uint8_t rpwm;
	uint8_t rf;
	uint8_t rb;


}motorconfig_t;

typedef enum{
  MOVE_STOP,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  MOVE_RIGHT,
  MOVE_LEFT,
  MOVE_HOLD
}motor_t;


void Motor_Init(motorconfig_t * _motorcfg);
void Motor(motor_t _move, uint8_t _lpwm, uint8_t _rpwm);


#endif /* MOTOR_H */
