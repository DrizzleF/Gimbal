#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f10x.h"

typedef struct {
    uint8_t valid;
    uint16_t target_x;
    uint16_t target_y;
} CoordFrame;

void Protocol_Init(void);
uint8_t Protocol_Parse(char ch, CoordFrame *coord);

#endif
