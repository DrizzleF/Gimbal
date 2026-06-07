#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

#define SERVO_PAN   0
#define SERVO_TILT  1

#define SERVO_MIN_ANGLE   0
#define SERVO_MAX_ANGLE   180
#define SERVO_MIN_PULSE   500    // 0.5ms
#define SERVO_MAX_PULSE   2500   // 2.5ms

// 360° 舵机速度模式 (Pan)
#define SERVO_360_CENTER  1500   // 停止 (us)
#define SERVO_360_RANGE   400    // ±400us 对应满速

void Servo_Init(void);
void Servo_SetAngle(uint8_t servo_id, float angle);
void Servo_SetSpeed(uint8_t servo_id, float speed);  // speed: -1.0..1.0

// 激光笔控制 (PA4)
void Laser_Init(void);
void Laser_On(void);
void Laser_Off(void);

#endif
