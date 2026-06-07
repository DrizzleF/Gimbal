#include "stepper.h"
#include "uart.h"

void Stepper_Init(void)
{
    UART2_Init();
    UART3_Init();
}

// Emm 固件 F6 格式: addr F6 dir velH velL acc snF 6B  (8字节)
void Stepper_VelControl(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc)
{
    if (vel > 5000) vel = 5000;
    uint8_t cmd[8];
    cmd[0] = addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel & 0xFF);
    cmd[5] = acc;
    cmd[6] = 0x00;
    cmd[7] = 0x6B;

    if (addr == ADDR_TILT)
        UART2_SendBuf(cmd, 8);
    else
        UART3_SendBuf(cmd, 8);
}

void Stepper_Stop(uint8_t addr)
{
    uint8_t cmd[] = {addr, 0xFE, 0x98, 0x00, 0x6B};

    if (addr == ADDR_TILT)
        UART2_SendBuf(cmd, 5);
    else
        UART3_SendBuf(cmd, 5);
}
