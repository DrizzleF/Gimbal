#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f10x.h"

typedef struct {
    uint8_t valid;      // Z=1 表示有效目标
    int16_t err_x;      // 像素误差 X（有符号，±320）
    int16_t err_y;      // 像素误差 Y（有符号，±240）
} CoordFrame;

void Protocol_Init(void);
uint8_t Protocol_Parse(char ch, CoordFrame *coord);

#endif
