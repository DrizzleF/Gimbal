#ifndef __BT_H
#define __BT_H

#include "stm32f10x.h"

void BT_Init(uint32_t baudrate);
uint8_t BT_DataAvailable(void);
char BT_Read(void);
void BT_SendString(const char *str);

#endif
