#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"

#define UART_RX_BUF_SIZE 256

// USART1 — K230 通信
void UART_Init(void);
uint8_t UART_DataAvailable(void);
char UART_Read(void);
void UART_Flush(void);
void UART_SendString(const char *str);

// USART2 — Tilt 电机 (PA2 TX, PA3 RX)
void UART2_Init(void);
void UART2_SendBuf(const uint8_t *buf, uint8_t len);

// USART3 — Pan 电机 (PB10 TX, PB11 RX)
void UART3_Init(void);
void UART3_SendBuf(const uint8_t *buf, uint8_t len);
#endif
