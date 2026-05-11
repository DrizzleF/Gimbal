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

// ==================== 丢失搜索参数 ====================
#define LOST_THRESHOLD       5      // 连续N个周期无数据后进入搜索 (50ms)
#define EXTRAPOLATE_STEPS    20     // 外推步数 (200ms)
#define EXTRAPOLATE_SPEED    0.7f   // 外推每步角度
#define SEARCH_LEG_STEP      3.0f   // 螺旋每段增量 (°)
#define SEARCH_STEP_SPEED    0.5f   // 搜索每步角度
#define SEARCH_MAX_RANGE     35.0f  // 搜索最大偏移范围 (°)
#define SEARCH_MAX_LEGS      16     // 最大搜索段数
#define RETURN_SPEED         1.2f   // 返回每步角度

typedef enum {
    STATE_TRACKING = 0,
    STATE_EXTRAPOLATE,    // 沿最后方向外推
    STATE_SEARCH,         // 方形螺旋搜索
    STATE_RETURN,         // 返回丢失位置
    STATE_IDLE            // 待机等待
} SearchState;

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

// 搜索状态机
static SearchState search_state = STATE_TRACKING;
static uint8_t  lost_cycles = 0;
static int8_t   last_pan_dir = 0, last_tilt_dir = 0;
static float    lost_pan_target = 90.0f, lost_tilt_target = 90.0f;
static uint8_t  extrapolate_count = 0;

// 螺旋搜索变量
static float    search_origin_pan = 0, search_origin_tilt = 0;
static uint8_t  search_dir = 0;       // 0:右 1:下 2:左 3:上
static float    search_leg_len = 0;
static float    search_leg_done = 0;
static uint8_t  search_leg_count = 0;

static float clamp_angle(float a)
{
    if (a < 0.0f)   return 0.0f;
    if (a > 180.0f) return 180.0f;
    return a;
}

int main(void)
{
    CoordFrame coord;

    SysTick_Config(SystemCoreClock / 1000);

    Servo_Init();
    UART_Init();
    Protocol_Init();

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

        // ---- 100Hz控制循环 ----
        if (sys_tick - last_ctrl_time >= CTRL_INTERVAL_MS)
        {
            last_ctrl_time = sys_tick;

            if (coord_valid)
            {
                coord_valid = 0;
                lost_cycles = 0;
                search_state = STATE_TRACKING;

                int16_t err_x = (int16_t)target_x - CENTER_X;
                int16_t err_y = (int16_t)target_y - CENTER_Y;

                // X轴 P控制，同时记录舵机实际运动方向
                if (abs(err_x) > DEAD_ZONE)
                {
                    float step_x = err_x * KP_X;
                    if (step_x > MAX_STEP)  step_x = MAX_STEP;
                    if (step_x < -MAX_STEP) step_x = -MAX_STEP;
                    pan_target -= step_x;
                    last_pan_dir = (step_x > 0.0f) ? -1 : 1;
                }

                // Y轴 P控制
                if (abs(err_y) > DEAD_ZONE)
                {
                    float step_y = err_y * KP_Y;
                    if (step_y > MAX_STEP)  step_y = MAX_STEP;
                    if (step_y < -MAX_STEP) step_y = -MAX_STEP;
                    tilt_target += step_y;
                    last_tilt_dir = (step_y > 0.0f) ? 1 : -1;
                }

                // 保存最后有效目标角度（搜索结束后的归位点）
                lost_pan_target = pan_target;
                lost_tilt_target = tilt_target;
            }
            else
            {
                lost_cycles++;

                switch (search_state)
                {
                case STATE_TRACKING:
                    if (lost_cycles >= LOST_THRESHOLD)
                    {
                        search_state = STATE_EXTRAPOLATE;
                        extrapolate_count = 0;
                    }
                    break;

                case STATE_EXTRAPOLATE:
                    pan_target  += last_pan_dir  * EXTRAPOLATE_SPEED;
                    tilt_target += last_tilt_dir * EXTRAPOLATE_SPEED;

                    if (++extrapolate_count >= EXTRAPOLATE_STEPS)
                    {
                        search_state = STATE_SEARCH;
                        search_origin_pan = pan_target;
                        search_origin_tilt = tilt_target;
                        search_dir = 0;
                        search_leg_len = SEARCH_LEG_STEP;
                        search_leg_done = 0;
                        search_leg_count = 0;
                    }
                    break;

                case STATE_SEARCH:
                {
                    switch (search_dir) {
                        case 0: pan_target  += SEARCH_STEP_SPEED; break;
                        case 1: tilt_target -= SEARCH_STEP_SPEED; break;
                        case 2: pan_target  -= SEARCH_STEP_SPEED; break;
                        case 3: tilt_target += SEARCH_STEP_SPEED; break;
                    }

                    search_leg_done += SEARCH_STEP_SPEED;

                    if (search_leg_done >= search_leg_len)
                    {
                        search_leg_done = 0;
                        search_dir = (search_dir + 1) % 4;
                        if (search_dir == 0 || search_dir == 2)
                            search_leg_len += SEARCH_LEG_STEP;
                        search_leg_count++;
                    }

                    // 超出搜索范围或段数上限则进入归位
                    float pd = pan_target - search_origin_pan;
                    float td = tilt_target - search_origin_tilt;
                    if ((pd > SEARCH_MAX_RANGE || pd < -SEARCH_MAX_RANGE ||
                         td > SEARCH_MAX_RANGE || td < -SEARCH_MAX_RANGE) ||
                        search_leg_count >= SEARCH_MAX_LEGS)
                    {
                        search_state = STATE_RETURN;
                    }
                    break;
                }

                case STATE_RETURN:
                {
                    float dp = lost_pan_target - pan_target;
                    float dt = lost_tilt_target - tilt_target;

                    if (dp > RETURN_SPEED)       pan_target  += RETURN_SPEED;
                    else if (dp < -RETURN_SPEED) pan_target  -= RETURN_SPEED;
                    else                         pan_target   = lost_pan_target;

                    if (dt > RETURN_SPEED)       tilt_target += RETURN_SPEED;
                    else if (dt < -RETURN_SPEED) tilt_target -= RETURN_SPEED;
                    else                         tilt_target  = lost_tilt_target;

                    if (pan_target == lost_pan_target && tilt_target == lost_tilt_target)
                        search_state = STATE_IDLE;
                    break;
                }

                case STATE_IDLE:
                    break;
                }
            }

            // 限幅
            pan_target  = clamp_angle(pan_target);
            tilt_target = clamp_angle(tilt_target);

            // EMA平滑 + 输出
            pan_angle  = EMA_ALPHA * pan_target  + (1.0f - EMA_ALPHA) * pan_angle;
            tilt_angle = EMA_ALPHA * tilt_target + (1.0f - EMA_ALPHA) * tilt_angle;

            Servo_SetAngle(SERVO_PAN, pan_angle);
            Servo_SetAngle(SERVO_TILT, tilt_angle);
        }

        // ---- 看门狗：200ms无数据强制标记丢失 ----
        if (sys_tick - last_rx_time > WATCHDOG_MS)
        {
            coord_valid = 0;
        }
    }
}
