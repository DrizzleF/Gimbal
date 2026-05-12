#include "bt.h"

#define BT_RX_BUF_SIZE 64

static char bt_rx_buf[BT_RX_BUF_SIZE];
static volatile uint8_t bt_rx_head = 0;
static volatile uint8_t bt_rx_tail = 0;

void BT_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        char ch = (char)USART_ReceiveData(USART2);
        uint8_t next = (bt_rx_head + 1) % BT_RX_BUF_SIZE;
        if (next != bt_rx_tail)
        {
            bt_rx_buf[bt_rx_head] = ch;
            bt_rx_head = next;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

uint8_t BT_DataAvailable(void)
{
    return (bt_rx_head != bt_rx_tail);
}

char BT_Read(void)
{
    while (bt_rx_head == bt_rx_tail);
    char ch = bt_rx_buf[bt_rx_tail];
    bt_rx_tail = (bt_rx_tail + 1) % BT_RX_BUF_SIZE;
    return ch;
}

void BT_SendString(const char *str)
{
    while (*str)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, *str++);
    }
}
