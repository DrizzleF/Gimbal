#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

#define SERVO_PAN   0
#define SERVO_TILT  1

#define SERVO_MIN_ANGLE   0
#define SERVO_MAX_ANGLE   180
#define SERVO_MIN_PULSE   500    // 0.5ms
#define SERVO_MAX_PULSE   2500   // 2.5ms

void Servo_Init(void);
void Servo_SetAngle(uint8_t servo_id, float angle);

#endif
