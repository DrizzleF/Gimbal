#include "stm32f10x.h"
#include "servo.h"
#include "uart.h"
#include "protocol.h"
#include "Delay.h"
#include <stdlib.h>

// ==================== 视觉参数 ====================
#define CENTER_X         320
#define CENTER_Y         240

// ==================== P控制参数 ====================
#define DEAD_ZONE        20
#define KP_X             0.025f     // 像素→角度/步
#define KP_Y             0.025f
#define MAX_STEP         3.0f       // 单步最大角度变化
#define EMA_ALPHA        0.30f      // 角度平滑系数

// ==================== 时序参数 ====================
#define CTRL_INTERVAL_MS 10         // 100Hz控制
#define WATCHDOG_MS      200        // 超时停转

// ==================== 全局变量 ====================
volatile uint32_t sys_tick = 0;
static uint16_t target_x = 320, target_y = 240;
static uint8_t  coord_valid = 0;
static uint32_t last_rx_time = 0;
static uint32_t last_ctrl_time = 0;

static float pan_angle = 90.0f;
static float tilt_angle = 90.0f;
static float pan_target = 90.0f;
static float tilt_target = 90.0f;

int main(void)
{
    CoordFrame coord;

    // 配置SysTick 1ms中断
    SysTick_Config(SystemCoreClock / 1000);

    Servo_Init();
    UART_Init();
    Protocol_Init();

    // 舵机归中
    Servo_SetAngle(SERVO_PAN, 90.0f);
    Servo_SetAngle(SERVO_TILT, 90.0f);

    while (1)
    {
        // ---- 接收K230坐标 ----
        while (UART_DataAvailable())
        {
            char ch = UART_Read();
            if (Protocol_Parse(ch, &coord))
            {
                target_x = coord.target_x;
                target_y = coord.target_y;
                coord_valid = 1;
                last_rx_time = sys_tick;
            }
        }

        // ---- 100Hz P控制 ----
        if (sys_tick - last_ctrl_time >= CTRL_INTERVAL_MS)
        {
            last_ctrl_time = sys_tick;

            if (coord_valid)
            {
                coord_valid = 0;

                int16_t err_x = (int16_t)target_x - CENTER_X;
                int16_t err_y = (int16_t)target_y - CENTER_Y;

                // X轴 P控制 → 计算目标角度
                if (abs(err_x) > DEAD_ZONE)
                {
                    float step_x = err_x * KP_X;
                    if (step_x > MAX_STEP)  step_x = MAX_STEP;
                    if (step_x < -MAX_STEP) step_x = -MAX_STEP;
                    pan_target -= step_x;    // 反号匹配底座方向
                }

                // Y轴 P控制
                if (abs(err_y) > DEAD_ZONE)
                {
                    float step_y = err_y * KP_Y;
                    if (step_y > MAX_STEP)  step_y = MAX_STEP;
                    if (step_y < -MAX_STEP) step_y = -MAX_STEP;
                    tilt_target += step_y;
                }

                // 限幅
                if (pan_target < 0.0f)   pan_target = 0.0f;
                if (pan_target > 180.0f) pan_target = 180.0f;
                if (tilt_target < 0.0f)   tilt_target = 0.0f;
                if (tilt_target > 180.0f) tilt_target = 180.0f;
            }

            // EMA平滑 + 输出
            pan_angle  = EMA_ALPHA * pan_target  + (1.0f - EMA_ALPHA) * pan_angle;
            tilt_angle = EMA_ALPHA * tilt_target + (1.0f - EMA_ALPHA) * tilt_angle;

            Servo_SetAngle(SERVO_PAN, pan_angle);
            Servo_SetAngle(SERVO_TILT, tilt_angle);
        }

        // ---- 看门狗：200ms无数据则停转 ----
        if (sys_tick - last_rx_time > WATCHDOG_MS)
        {
            coord_valid = 0;
        }
    }
}
