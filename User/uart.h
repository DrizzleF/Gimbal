#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"

#define UART_RX_BUF_SIZE 64

void UART_Init(void);
uint8_t UART_DataAvailable(void);
char UART_Read(void);
void UART_Flush(void);
void UART_SendString(const char *str);

#endif
