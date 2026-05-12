#include "stm32f10x.h"
#include "servo.h"
#include "uart.h"
#include "protocol.h"
#include "bt.h"
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
#define LOST_THRESHOLD       5      // 连续无数据周期数后进入搜索 (50ms)
#define EXTRAPOLATE_MAX      50.0f  // 外推最大偏离角度 (°)
#define EXTRAPOLATE_SPEED    0.8f   // 外推每步角度
#define PERIM_PADDING        8.0f   // 边缘搜索矩形扩展量 (°)
#define PERIM_STEP           0.6f   // 边缘搜索每步角度
#define DEFAULT_PERIM_RANGE  20.0f  // 无方向信息时的默认搜索范围 (°)
#define RETURN_SPEED         1.2f   // 返回每步角度

// ==================== 手操参数 ====================
#define MANUAL_STEP          2.0f   // 蓝牙方向键每步角度 (°)

typedef enum {
    STATE_TRACKING = 0,
    STATE_EXTRAPOLATE,
    STATE_PERIMETER,
    STATE_RETURN,
    STATE_IDLE
} SearchState;

typedef enum {
    MODE_TRACKING = 0,
    MODE_MANUAL   = 1
} SystemMode;

// ==================== BT 指令解析 ====================
typedef enum {
    BT_NONE = 0,
    BT_MODE, BT_COLOR, BT_PAN, BT_TILT,
    BT_PAN_L, BT_PAN_R, BT_TILT_U, BT_TILT_D
} BTCmdType;

typedef struct {
    BTCmdType type;
    float     value;
} BTCmd;

static BTCmd bt_parse(const char *line)
{
    BTCmd cmd = {BT_NONE, 0};

    if (line[0] == 'M' && line[1] >= '0' && line[1] <= '1' && line[2] == '\0')
        { cmd.type = BT_MODE;  cmd.value = line[1] - '0'; }
    else if (line[0] == 'C' && line[1] >= '0' && line[1] <= '1' && line[2] == '\0')
        { cmd.type = BT_COLOR; cmd.value = line[1] - '0'; }
    else if (line[0] == 'P')
        { cmd.type = BT_PAN;  float v=0; uint8_t i=1;
          while (line[i]>='0'&&line[i]<='9') { v=v*10+(line[i]-'0'); i++; } cmd.value=v; }
    else if (line[0] == 'T')
        { cmd.type = BT_TILT; float v=0; uint8_t i=1;
          while (line[i]>='0'&&line[i]<='9') { v=v*10+(line[i]-'0'); i++; } cmd.value=v; }
    else if (line[0] == 'L' && line[1] == '\0')
        cmd.type = BT_PAN_L;
    else if (line[0] == 'R' && line[1] == '\0')
        cmd.type = BT_PAN_R;
    else if (line[0] == 'U' && line[1] == '\0')
        cmd.type = BT_TILT_U;
    else if (line[0] == 'D' && line[1] == '\0')
        cmd.type = BT_TILT_D;

    return cmd;
}

static void bt_handle_cmd(BTCmd cmd)
{
    switch (cmd.type)
    {
    case BT_MODE:
        sys_mode = (cmd.value == 0) ? MODE_TRACKING : MODE_MANUAL;
        if (sys_mode == MODE_TRACKING)
            { search_state = STATE_TRACKING; lost_cycles = 0; }
        BT_SendString(sys_mode == MODE_TRACKING ? "OK TRACK\n" : "OK MANUAL\n");
        break;

    case BT_COLOR:
        UART_SendString(cmd.value == 0 ? "C0\n" : "C1\n");
        BT_SendString(cmd.value == 0 ? "OK ORANGE\n" : "OK GREEN\n");
        break;

    case BT_PAN:
        if (sys_mode == MODE_MANUAL) pan_target = clamp_angle(cmd.value);
        break;

    case BT_TILT:
        if (sys_mode == MODE_MANUAL) tilt_target = clamp_angle(cmd.value);
        break;

    case BT_PAN_L:
        if (sys_mode == MODE_MANUAL) pan_target = clamp_angle(pan_target + MANUAL_STEP);
        break;

    case BT_PAN_R:
        if (sys_mode == MODE_MANUAL) pan_target = clamp_angle(pan_target - MANUAL_STEP);
        break;

    case BT_TILT_U:
        if (sys_mode == MODE_MANUAL) tilt_target = clamp_angle(tilt_target - MANUAL_STEP);
        break;

    case BT_TILT_D:
        if (sys_mode == MODE_MANUAL) tilt_target = clamp_angle(tilt_target + MANUAL_STEP);
        break;

    default: break;
    }
}

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
static int16_t  last_err_x = 0, last_err_y = 0;
static float    lost_pan_target = 90.0f, lost_tilt_target = 90.0f;
static uint8_t  extrapolate_count = 0;

// 边缘搜索变量
static float    perim_pan_min = 0, perim_pan_max = 0;
static float    perim_tilt_min = 0, perim_tilt_max = 0;
static uint8_t  perim_corner = 0;

// 系统模式
static SystemMode sys_mode = MODE_TRACKING;

// BT 行缓冲
static char    bt_line_buf[16];
static uint8_t bt_line_idx = 0;

static float clamp_angle(float a)
{
    if (a < 0.0f)   return 0.0f;
    if (a > 180.0f) return 180.0f;
    return a;
}

static void perim_enter(void)
{
    float p_min = lost_pan_target;
    float p_max = pan_target;
    if (p_min > p_max) { float t = p_min; p_min = p_max; p_max = t; }
    perim_pan_min = clamp_angle(p_min - PERIM_PADDING);
    perim_pan_max = clamp_angle(p_max + PERIM_PADDING);

    float t_min = lost_tilt_target;
    float t_max = tilt_target;
    if (t_min > t_max) { float t = t_min; t_min = t_max; t_max = t; }
    perim_tilt_min = clamp_angle(t_min - PERIM_PADDING);
    perim_tilt_max = clamp_angle(t_max + PERIM_PADDING);

    perim_corner = 0;
}

static void perim_enter_default(void)
{
    perim_pan_min = clamp_angle(lost_pan_target - DEFAULT_PERIM_RANGE);
    perim_pan_max = clamp_angle(lost_pan_target + DEFAULT_PERIM_RANGE);
    perim_tilt_min = clamp_angle(lost_tilt_target - DEFAULT_PERIM_RANGE);
    perim_tilt_max = clamp_angle(lost_tilt_target + DEFAULT_PERIM_RANGE);
    perim_corner = 0;
}

int main(void)
{
    CoordFrame coord;

    SysTick_Config(SystemCoreClock / 1000);

    Servo_Init();
    UART_Init();
    BT_Init(9600);
    Protocol_Init();

    Servo_SetAngle(SERVO_PAN, 90.0f);
    Servo_SetAngle(SERVO_TILT, 90.0f);

    while (1)
    {
        // ---- 接收K230坐标 (USART1) ----
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

        // ---- 接收蓝牙指令 (USART2) ----
        while (BT_DataAvailable())
        {
            char ch = BT_Read();
            if (ch == '\n' || ch == '\r')
            {
                if (bt_line_idx > 0)
                {
                    bt_line_buf[bt_line_idx] = '\0';
                    bt_line_idx = 0;
                    BTCmd cmd = bt_parse(bt_line_buf);
                    bt_handle_cmd(cmd);
                }
            }
            else if (bt_line_idx < sizeof(bt_line_buf) - 1)
            {
                bt_line_buf[bt_line_idx++] = ch;
            }
        }

        // ---- 100Hz控制循环 ----
        if (sys_tick - last_ctrl_time >= CTRL_INTERVAL_MS)
        {
            last_ctrl_time = sys_tick;

            if (sys_mode == MODE_TRACKING)
            {
                // ============ 追踪模式 ============
                if (coord_valid)
                {
                    coord_valid = 0;
                    lost_cycles = 0;
                    search_state = STATE_TRACKING;

                    int16_t err_x = (int16_t)target_x - CENTER_X;
                    int16_t err_y = (int16_t)target_y - CENTER_Y;

                    last_err_x = err_x;
                    last_err_y = err_y;

                    if (abs(err_x) > DEAD_ZONE)
                    {
                        float step_x = err_x * KP_X;
                        if (step_x > MAX_STEP)  step_x = MAX_STEP;
                        if (step_x < -MAX_STEP) step_x = -MAX_STEP;
                        pan_target -= step_x;
                    }

                    if (abs(err_y) > DEAD_ZONE)
                    {
                        float step_y = err_y * KP_Y;
                        if (step_y > MAX_STEP)  step_y = MAX_STEP;
                        if (step_y < -MAX_STEP) step_y = -MAX_STEP;
                        tilt_target -= step_y;
                    }

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
                            if (last_err_x == 0 && last_err_y == 0)
                            {
                                perim_enter_default();
                                search_state = STATE_PERIMETER;
                            }
                            else
                            {
                                search_state = STATE_EXTRAPOLATE;
                                extrapolate_count = 0;
                            }
                        }
                        break;

                    case STATE_EXTRAPOLATE:
                        if (last_err_x > 0)      pan_target -= EXTRAPOLATE_SPEED;
                        else if (last_err_x < 0) pan_target += EXTRAPOLATE_SPEED;
                        if (last_err_y > 0)      tilt_target -= EXTRAPOLATE_SPEED;
                        else if (last_err_y < 0) tilt_target += EXTRAPOLATE_SPEED;
                        extrapolate_count++;

                        {
                            float pd = pan_target - lost_pan_target;
                            float td = tilt_target - lost_tilt_target;
                            if ((pd > EXTRAPOLATE_MAX || pd < -EXTRAPOLATE_MAX) ||
                                (td > EXTRAPOLATE_MAX || td < -EXTRAPOLATE_MAX))
                            {
                                if (pd > EXTRAPOLATE_MAX)       pan_target = lost_pan_target + EXTRAPOLATE_MAX;
                                else if (pd < -EXTRAPOLATE_MAX) pan_target = lost_pan_target - EXTRAPOLATE_MAX;
                                if (td > EXTRAPOLATE_MAX)       tilt_target = lost_tilt_target + EXTRAPOLATE_MAX;
                                else if (td < -EXTRAPOLATE_MAX) tilt_target = lost_tilt_target - EXTRAPOLATE_MAX;

                                perim_enter();
                                search_state = STATE_PERIMETER;
                            }
                        }
                        break;

                    case STATE_PERIMETER:
                    {
                        float wp_pan = 0, wp_tilt = 0;
                        switch (perim_corner) {
                            case 0: wp_pan = perim_pan_max; wp_tilt = perim_tilt_max; break;
                            case 1: wp_pan = perim_pan_min; wp_tilt = perim_tilt_max; break;
                            case 2: wp_pan = perim_pan_min; wp_tilt = perim_tilt_min; break;
                            case 3: wp_pan = perim_pan_max; wp_tilt = perim_tilt_min; break;
                            default: search_state = STATE_RETURN; break;
                        }

                        if (search_state == STATE_PERIMETER)
                        {
                            float dp = wp_pan - pan_target;
                            float dt = wp_tilt - tilt_target;

                            if (dp > PERIM_STEP)       pan_target  += PERIM_STEP;
                            else if (dp < -PERIM_STEP) pan_target  -= PERIM_STEP;
                            else                       pan_target   = wp_pan;

                            if (dt > PERIM_STEP)       tilt_target += PERIM_STEP;
                            else if (dt < -PERIM_STEP) tilt_target -= PERIM_STEP;
                            else                       tilt_target  = wp_tilt;

                            if (pan_target == wp_pan && tilt_target == wp_tilt)
                            {
                                perim_corner++;
                                if (perim_corner >= 4)
                                    search_state = STATE_RETURN;
                            }
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
            }
            // else: MODE_MANUAL — pan_target/tilt_target 由 BT 指令直接设置

            // 限幅 + EMA + 输出 (两种模式共用)
            pan_target  = clamp_angle(pan_target);
            tilt_target = clamp_angle(tilt_target);

            pan_angle  = EMA_ALPHA * pan_target  + (1.0f - EMA_ALPHA) * pan_angle;
            tilt_angle = EMA_ALPHA * tilt_target + (1.0f - EMA_ALPHA) * tilt_angle;

            Servo_SetAngle(SERVO_PAN, pan_angle);
            Servo_SetAngle(SERVO_TILT, tilt_angle);
        }

        // ---- 看门狗 ----
        if (sys_tick - last_rx_time > WATCHDOG_MS)
        {
            coord_valid = 0;
        }
    }
}
