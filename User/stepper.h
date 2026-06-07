#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f10x.h"

#define ADDR_PAN   0x02
#define ADDR_TILT  0x03

void Stepper_Init(void);

// Emm 固件 F6 速度模式 (8字节)
// dir: 0=CW, 1=CCW
// vel: 0-5000 RPM
// acc: 0-255 (0=立即加速)
void Stepper_VelControl(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc);

void Stepper_Stop(uint8_t addr);

#endif
