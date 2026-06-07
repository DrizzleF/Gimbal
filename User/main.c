#include "stm32f10x.h"
#include "stepper.h"
#include "uart.h"
#include "protocol.h"
#include "Delay.h"
#include "OLED.h"
#include "pid.h"

// ==================== 参数 ====================
#define CENTER_X       320
#define CENTER_Y       240
#define DEAD_ZONE      10         // 宽死区，避免噪声导致不停
#define CTRL_MS        40         // 25Hz
#define OLED_MS        200
// 三段调速阈值
#define ERR_SMALL      8
#define ERR_LARGE      25

// Pan 三段限速
#define PAN_SPD_SMALL  10
#define PAN_SPD_MID    18
#define PAN_SPD_LARGE  28

// Tilt 三段限速
#define TILT_SPD_SMALL 10
#define TILT_SPD_MID   18
#define TILT_SPD_LARGE 28

// 丢目标后向最后方向移动
#define RECOVER_MS      500
#define RECOVER_SPEED   30

volatile uint32_t sys_tick = 0;

static int16_t  err_x = 0, err_y = 0;

// PID
static PID_t yaw_pid = {
    .kp = 0.15f, .ki = 0, .kd = 0.06f,
    .target = 0, .out = 0,
    .out_max = 15, .ki_max = 3,
};
static PID_t pitch_pid = {
    .kp = 0.10f, .ki = 0, .kd = 0.04f,
    .target = 0, .out = 0,
    .out_max = 15, .ki_max = 3,
};
static volatile float dbg_yaw = 0, dbg_pit = 0;

static int16_t  last_err_x = 0, last_err_y = 0;
static uint32_t recover_tick = 0;

// ==================== 辅助函数 ====================
static inline float fabsf(float x) { return x < 0 ? -x : x; }

static void Stepper_Speed(uint8_t addr, float rpm, float acc)
{
    uint8_t dir;
    uint16_t vel;
    if (rpm > 0) { dir = 0; vel = (uint16_t)(rpm + 0.5f); }
    else         { dir = 1; vel = (uint16_t)(-rpm + 0.5f); }
    if (vel > 5000) vel = 5000;
    Stepper_VelControl(addr, dir, vel, (uint8_t)acc);
}

// ==================== 主函数 ====================
int main(void)
{
    SysTick_Config(SystemCoreClock / 1000);
    UART_Init();
    Stepper_Init();
    Protocol_Init();
    OLED_Init();

    // 等驱动上电稳定
    OLED_Clear();
    OLED_ShowString(1, 1, "Wait 2s..");
    Delay_ms(2000);

    // 自检
    OLED_Clear();
    OLED_ShowString(1, 1, "SelfTest");

    OLED_ShowString(2, 1, "Pan CW  ");
    Stepper_VelControl(ADDR_PAN, 0, 30, 0xFF);
    Delay_ms(150);
    Stepper_Stop(ADDR_PAN);
    Delay_ms(100);

    OLED_ShowString(2, 1, "Pan CCW ");
    Stepper_VelControl(ADDR_PAN, 1, 30, 0xFF);
    Delay_ms(150);
    Stepper_Stop(ADDR_PAN);
    Delay_ms(100);

    OLED_ShowString(2, 1, "Tilt CW ");
    Stepper_VelControl(ADDR_TILT, 0, 30, 0xFF);
    Delay_ms(150);
    Stepper_Stop(ADDR_TILT);
    Delay_ms(100);

    OLED_ShowString(2, 1, "Tilt CCW");
    Stepper_VelControl(ADDR_TILT, 1, 30, 0xFF);
    Delay_ms(150);
    Stepper_Stop(ADDR_TILT);
    Delay_ms(100);

    SysTick_Config(SystemCoreClock / 1000);
    OLED_Clear();
    UART_Flush();

    uint32_t last_ctrl = 0, last_oled = 0;
    uint8_t  has_target = 0;
    uint32_t last_rx = 0;
    uint32_t rx_count = 0;

    while (1)
    {
        // ---- 读取 K230 ----
        if (UART_DataAvailable())
        {
            char ch = UART_Read();
            rx_count++;
            CoordFrame cf;
            if (Protocol_Parse(ch, &cf))
            {
                err_x = cf.err_x;
                err_y = cf.err_y;
                last_rx = sys_tick;
                if (cf.valid) has_target = 1;
                else          has_target = 0;
            }
        }

        if (has_target && (sys_tick - last_rx > 200))
            has_target = 0;

        // ---- PID 控制 ----
        if (sys_tick - last_ctrl >= CTRL_MS)
        {
            last_ctrl = sys_tick;

            if (has_target)
            {
                recover_tick = 0;
                last_err_x = err_x;
                last_err_y = err_y;

                // Pan PID + 三段限速
                float yaw_spd = PID_Calc(&yaw_pid, -err_x);
                float abs_yaw = fabsf(err_x);
                float yaw_lim, yaw_acc;
                if (abs_yaw < ERR_SMALL)      { yaw_lim = PAN_SPD_SMALL; }
                else if (abs_yaw < ERR_LARGE) { yaw_lim = PAN_SPD_MID;   }
                else                          { yaw_lim = PAN_SPD_LARGE; }
                yaw_acc = yaw_lim * 2.0f;
                if (yaw_spd > yaw_lim)  yaw_spd = yaw_lim;
                if (yaw_spd < -yaw_lim) yaw_spd = -yaw_lim;

                dbg_yaw = yaw_spd;
                if (abs_yaw < DEAD_ZONE)
                    Stepper_Stop(ADDR_PAN);
                else
                    Stepper_Speed(ADDR_PAN, yaw_spd, yaw_acc);

                // Tilt PID + 三段限速
                float pit_spd = PID_Calc(&pitch_pid, -err_y);
                float abs_pit = fabsf(err_y);
                float pit_lim, pit_acc;
                if (abs_pit < ERR_SMALL)      { pit_lim = TILT_SPD_SMALL; }
                else if (abs_pit < ERR_LARGE) { pit_lim = TILT_SPD_MID;   }
                else                          { pit_lim = TILT_SPD_LARGE; }
                pit_acc = pit_lim * 2.0f;
                if (pit_spd > pit_lim)  pit_spd = pit_lim;
                if (pit_spd < -pit_lim) pit_spd = -pit_lim;

                dbg_pit = pit_spd;
                if (abs_pit < DEAD_ZONE)
                    Stepper_Stop(ADDR_TILT);
                else
                    Stepper_Speed(ADDR_TILT, pit_spd, pit_acc);
            }
            else
            {
                // 丢目标：沿最后误差方向缓慢移动一小段
                if (recover_tick == 0)
                    recover_tick = sys_tick;

                if (sys_tick - recover_tick < RECOVER_MS) {
                    // 方向取反（齿轮反转），速度按误差比例缩放
                    float ry = -(float)last_err_x * 0.04f;
                    float rp = -(float)last_err_y * 0.04f;
                    if (ry > RECOVER_SPEED)  ry = RECOVER_SPEED;
                    if (ry < -RECOVER_SPEED) ry = -RECOVER_SPEED;
                    if (rp > RECOVER_SPEED)  rp = RECOVER_SPEED;
                    if (rp < -RECOVER_SPEED) rp = -RECOVER_SPEED;
                    dbg_yaw = ry; dbg_pit = rp;
                    Stepper_Speed(ADDR_PAN,  ry, RECOVER_SPEED * 2);
                    Stepper_Speed(ADDR_TILT, rp, RECOVER_SPEED * 2);
                } else {
                    dbg_yaw = 0; dbg_pit = 0;
                    Stepper_Stop(ADDR_PAN);
                    Stepper_Stop(ADDR_TILT);
                }
            }
        }

        // ---- OLED ----
        if (sys_tick - last_oled >= OLED_MS)
        {
            last_oled = sys_tick;
            OLED_ShowSignedNum(1, 1, err_x, 5);
            OLED_ShowSignedNum(1, 7, err_y, 5);
            OLED_ShowString(1, 13, "R");
            OLED_ShowNum(1, 14, (uint16_t)(rx_count % 1000), 3);
            if (has_target)
                OLED_ShowString(2, 1, "Tracking");
            else if (recover_tick && (sys_tick - recover_tick < RECOVER_MS))
                OLED_ShowString(2, 1, "Recover ");
            else
                OLED_ShowString(2, 1, "Lost    ");
            OLED_ShowSignedNum(3, 1, (int32_t)dbg_yaw, 4);
            OLED_ShowSignedNum(3, 6, (int32_t)dbg_pit, 4);
            OLED_ShowNum(4, 1, sys_tick / 1000, 4);
        }
    }
}
