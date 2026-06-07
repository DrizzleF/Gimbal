# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

**禁止更新 Claude Code CLI 本身。** 不要运行 `claude update`、`npm update -g @anthropic-ai/claude-code` 或任何升级 Claude Code 版本的命令。

## Project Overview

2025全国大学生电子设计竞赛E题"简易自行瞄准装置"。二维云台控制蓝紫激光笔瞄准目标靶心，光斑直径<0.55cm。

硬件：K230摄像头（亚博智能）+ STM32F103C8 + ZDT_X42S闭环步进电机×2（张大头）+ 蓝紫激光笔。

E题核心要求：
- 小车沿100cm×100cm正方形轨迹逆时针行驶（巡迹用MSPM0，本仓库不含）
- 瞄准模块：二维云台+激光笔，K230视觉检测靶心，STM32控制步进电机
- 静止瞄准：2秒内对准靶心，误差≤1.5cm
- 随机角度瞄准：4秒内完成，误差≤1.8cm
- 动态瞄准（行驶中）：最大误差≤8cm（1圈）/ ≤6cm（2圈）
- 显示瞄准时间和光斑误差

## Architecture

```
K230 (灰度视觉) ──UART1(115200)──→ STM32 (PID+三段限速) ──UART2(115200)──→ ZDT_X42S Tilt (0x03)
                                      │                      UART3(115200)──→ ZDT_X42S Pan  (0x02)
                                   激光笔(直连VCC)                 ↑
                                   OLED(PB6/PB7 I2C)         磁编码器闭环
```

STM32 端:
├── 25Hz 控制循环 (40ms)，主循环轮询
├── F6 速度模式（Emm固件）：像素偏差→PID→三段限速→RPM命令
├── 双 UART 独立控制：USART2→Tilt, USART3→Pan（无总线冲突）
├── PID 参数：Pan Kp=0.3 Kd=0.02, Tilt Kp=0.2 Kd=0.01, 均 Ki=0
├── PID 输出限幅 ±20 RPM，再经三段速度限制
├── 激光笔: 直连VCC（常亮），非GPIO控制
├── OLED: SSD1306 128×64 I2C，4行显示
│   ├── L1: raw_x, raw_y + R计数器（诊断用）
│   ├── L2: 误差(err_x, err_y) 有符号
│   ├── L3: 状态(Tracking/Lost)
│   └── L4: PID输出(yaw_spd, pit_spd) + 运行时间(秒)
├── 开机自检: 30RPM正转150ms→停100ms→反转150ms→停100ms（Pan+Tilt各两方向）
├── 死区用 Stepper_Stop (FE 98) 而非 F6 vel=0
└── 丢失目标时两个轴立即 Stepper_Stop

**关键设计决策：**
- 控制逻辑在STM32端（25Hz），K230只发像素误差（~40Hz）
- 两个电机各占一个UART（USART2/3），消除总线竞争和命令间延时
- F6 速度模式（Emm固件），8字节格式：`Addr F6 dir velH velL acc snF 6B`
- snF=0x00 立即执行（非同步模式，同步模式 snF=0x01 需等 FF66 触发）
- PID 位置式，带积分抗饱和和输出限幅
- 三段调速：误差小→低速精细调节，误差大→高速追击
- 齿轮传动反转方向：两个轴均需方向取反（`-err_x`, `-err_y`）
- 激光笔直连VCC常亮（不经过GPIO控制）
- 协议极简：ASCII行协议，`\n` 结尾，兼容新旧两种格式
- `Delay_us()` 保存/恢复 SysTick 寄存器（LOAD/CTRL/VAL），避免破坏系统时钟

## Build

- **STM32**: Keil MDK v5, ARM Compiler 5 (V5.06). 打开 `Project.uvprojx`，编译目标 STM32F103C8.
- **K230**: 将 `k230/main.py` 通过CanMV IDE或TF卡放到K230开发板运行.

## Hardware Pin Map

> 权威文档：[PINOUT.md](PINOUT.md)，修改引脚须同步更新。

| STM32引脚 | 功能 | 连接 |
|-----------|------|------|
| PA9 (USART1_TX) | K230通信发送 | K230 RX |
| PA10 (USART1_RX) | K230通信接收 | K230 TX |
| PA2 (USART2_TX) | Tilt电机发送 | ZDT_X42S (0x03) RX |
| PA3 (USART2_RX) | Tilt电机接收 | ZDT_X42S (0x03) TX |
| PB6 (I2C_SCL) | OLED 时钟 | SSD1306 SCL |
| PB7 (I2C_SDA) | OLED 数据 | SSD1306 SDA |
| PB10 (USART3_TX) | Pan电机发送 | ZDT_X42S (0x02) RX |
| PB11 (USART3_RX) | Pan电机接收 | ZDT_X42S (0x02) TX |

**接线关键：**
- STM32 TX → 驱动板 RX，STM32 RX → 驱动板 TX，不能接反
- 驱动板 GND 须与 STM32 GND 共地
- 驱动板 COM 引脚须接 GND（选择 UART 模式，浮空默认脉冲模式）
- 驱动板 EN 接 GND（保持使能/锁轴）
- 电机须外部24V/3A+独立供电，不可从STM32取电
- Pan=地址2 (0x02), Tilt=地址3 (0x03)

## Communication Protocol

- **K230 → STM32** (USART1, 115200): 
  - 新格式：`X<err_x>Y<err_y>Z<valid>\n` — 像素误差+有效标志，例 `X15Y-8Z1\n`
  - 旧格式兼容：`X<abs_x>Y<abs_y>\n` — 绝对坐标，自动转为误差
  - 协议解析器（`protocol.c`）自动识别两种格式
- **STM32 → 电机** (USART2/3, 115200): ZDT_X42S Emm固件 F6 速度模式
  - `Addr F6 dir velH velL acc snF 6B`（8字节）
  - vel: 0-5000 (RPM), acc: 0-255 (0=立即加速), dir: 0=CW 1=CCW
  - 停止用 `Addr FE 98 00 6B`（5字节），比 F6 vel=0 更可靠

## Key Tuning Parameters

### PID 参数

| 参数 | Pan (Yaw) | Tilt (Pitch) | 说明 |
|------|-----------|-------------|------|
| Kp | 0.3 | 0.2 | 比例增益 |
| Ki | 0 | 0 | 积分（当前禁用） |
| Kd | 0.02 | 0.01 | 微分增益 |
| out_max | 20 | 20 | PID输出限幅(RPM) |
| ki_max | 3 | 3 | 积分限幅 |

### 三段调速限幅（在 PID 输出之上再次限速）

| 误差范围 | Pan 限速 | Tilt 限速 |
|----------|---------|----------|
| < 8px (ERR_SMALL) | 15 RPM | 15 RPM |
| 8~25px (ERR_LARGE) | 25 RPM | 25 RPM |
| > 25px | 40 RPM | 40 RPM |

### 其他参数

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `DEAD_ZONE` | 10 px | 死区，进入死区用 Stepper_Stop |
| `CTRL_MS` | 40 | 控制周期(25Hz) |
| `OLED_MS` | 200 | OLED刷新周期 |
| `WATCHDOG_MS` | 200 | 丢目标超时(ms) |
| `MAX_VEL` | 10 | 保守上限（被三段限速覆盖） |
| 自检速度 | 30 RPM | 每个方向150ms |
| 加速度 | 限速×2 | 动态计算 |

### Delay_us 关键实现

```c
void Delay_us(uint32_t xus)
{
    uint32_t saved_load = SysTick->LOAD;   // 保存
    uint32_t saved_ctrl = SysTick->CTRL;
    SysTick->LOAD = 72 * xus;
    SysTick->VAL  = 0x00;
    SysTick->CTRL = 0x00000005;            // HCLK, 使能, 无中断
    while (!(SysTick->CTRL & 0x00010000)); // 等待
    SysTick->LOAD = saved_load;            // 恢复
    SysTick->VAL  = 0x00;
    SysTick->CTRL = saved_ctrl;            // 恢复中断使能
}
```

**不保存/恢复会导致 SysTick 被永久禁用，系统时钟冻结。**

## K230 视觉检测

使用灰度矩形检测，通过 `cv_lite.grayscale_find_rectangles_with_corners()` 检测靶纸矩形边框。

**筛选策略（二级验证）：**
1. 几何验证 `validate_rect()`：每条边 ≥30px，对边长度差 <120px，对边平行+邻边垂直（容差30°）
2. 面积阈值 ≥2000 像素

取面积最大的合格矩形，对角线交点取靶心。输出像素误差（相对画面中心320,240）。

**检测参数：** canny(50,150), approx_epsilon=0.04, area_min_ratio=0.001, max_angle_cos=0.3, gaussian_blur=5

## 丢失目标处理

丢目标后沿最后误差方向缓慢移动 300ms（`RECOVER_MS`），速度按 `last_err × 0.04` 缩放、上限 `RECOVER_SPEED=12 RPM`。超时后停住等待目标重新出现。检测到目标立即切回 PID 追踪。

OLED L2：Tracking / Recover / Lost。

## 国一方案分析（2023-2025）

> 文件夹：`2025年电赛E题国一方案/`、`2024年H题电赛国一方案/`、`2023年电赛E题国一方案/`

### 2025 E题国一方案 — HAL层算法结构

**系统架构：** K230(视觉) → MSPM0G3507(云台) + MSPM0G3507(小车)

**视觉 — 矩形检测（非颜色识别）：**
```python
rects = cv_lite.rgb888_find_rectangles_with_corners(image_shape, img_np,
    canny_thresh1, canny_thresh2, approx_epsilon, area_min_ratio, max_angle_cos, gaussian_blur_size)
```
- Canny边缘→多边形拟合→角度余弦校验→对边平行+邻边垂直→对角线交点=靶心
- 面积阈值 S_THRESHOLD 可按键调节（500-10000），对边长度差校验有效性

**中断架构（关键）：**
- `TIMER_0_INST_IRQHandler` — 2ms定时器中断，**PID计算+步进电机驱动全在中断里**
- `SysTick_Handler` — 1ms系统滴答
- 主循环 — 按键扫描 + 串口处理 + OLED显示（非实时任务）
- PID执行频率：`Tick.steppid++ >= 2` 即每4ms执行一次（250Hz）

**PID结构体（位置式）：**
```c
typedef struct {
    double Kp, Ki, Kd;
    double target_val, actual_val;
    double err, err_last, err_sum;
    double output;
} tPid;
// output = Kp*err + Ki*err_sum + Kd*(err - err_last)
// I限幅：I_limit(pid, low, high)
// 输出限幅：constrain_double(output, low, high)
```
- X轴：`Kp=0.084, Ki=0.00047, Kd=0`
- Y轴：`Kp=-0.084, Ki=-0.00047, Kd=0`（取反，机械方向相反）

**步进电机驱动 — RPM→PWM频率映射：**
```c
int Calculate_target(int Target) {
    // Target>0正转, <0反转, =0停止
    float stepsPerSecond = (abs(Target) * 200 * 16) / 60.0f;  // RPM→步/秒
    uint16_t periodValue = (10000000 / stepsPerSecond) - 1;    // PWM时钟/频率
    return direction * periodValue;
}
void Set_PWM(int L, int R) {
    // L>0: 使能+方向置高+设PWM周期+50%占空比
    // L<0: 使能+方向置低+设PWM周期+50%占空比
    // L==0: 失能
}
```

**串口协议解析 — 三状态状态机：**
```c
// 状态机：等待'[' → 接收数据直到'*' → 等待']'完成
if (RxState == 0 && RxData == '[') { RxState = 1; }
if (RxState == 1 && RxData == '*') { RxState = 2; }
if (RxState == 2 && RxData == ']') { RxState = 0; RxFlag = 1; }
// 格式：[±xxx±yyy*]  8字节坐标
```

**主循环逻辑：**
```c
while(1) {
    Timer_work();   // 编码器读取等
    key_work();     // 按键扫描
    uart_work();    // 串口数据解析 → x_site, y_site
    oled_show();    // OLED显示误差/输出
    // 死区判断：x_site∈[-2,2] 且 y_site∈[15,20] → 停机
}
```

### 2024 H题国一方案 — HAL层算法结构

**系统架构：** STM32F103 + MPU6050(DMP) + 8路红外

**PWM初始化（TIM2，20kHz）：**
```c
// ARR=100-1, PSC=36-1 → 72MHz/36/100=20kHz
TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;
TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;
// CH2(PA1) 左电机, CH3(PA2) 右电机
```

**电机驱动 — GPIO方向+PWM速度：**
```c
void Set_Speed(int motor_l, int motor_r) {
    if (motor_l >= 0) GPIO_SetBits(PA3), GPIO_ResetBits(PA4), TIM_SetCompare3(TIM2, motor_l);
    else              GPIO_SetBits(PA4), GPIO_ResetBits(PA3), TIM_SetCompare3(TIM2, -motor_l);
    // 右电机同理，PA0/PA5 + TIM2_CH2
}
void Limit(int *left, int *right) {
    // 限幅 [PWM_MIN, PWM_MAX]
}
```

**循迹 — 红外状态查表（8路→8bit→误差值）：**
```c
u8 Get_Infrared_State(void) {
    // 8个GPIO读取 → 左移拼接为8bit状态码
    return (TRACK1<<7 | TRACK2<<6 | ... | TRACK8<<0);
}
float Track_err(void) {
    // 查表：state→error（±1/±2/±4/±6）
    // 消抖：连续2次相同状态才更新
    if (state == last_state) same_cnt++;
    else { same_cnt = 0; last_state = state; }
    if (same_cnt < 2) return last_error;  // 不够2次，返回旧值
    // 查表映射...
}
```

**PD循迹控制：**
```c
int PID_out(float error, int Target) {
    int err = (int)error;
    int diff = err - last_err;
    if (last_err == 0 && err != 0) diff = err / 2;  // 首次微分减半
    out = Kp * err + Kd * diff;  // Kp=2.5, Kd=3.5
    if (out > 25) out = 25; if (out < -25) out = -25;
    last_err = err;
    return out;
}
void Final_Speed(int pid_out, int base_speed) {
    int left  = base_speed - pid_out;
    int right = base_speed + pid_out;
    // 限幅 [MIN_SPEED=5, MAX_SPEED=40]
    Set_Speed(left, right);
}
```

**陀螺仪角度PD（航向保持）：**
```c
int pid_angle2(int target, int yaw) {
    int yaw0_360 = (yaw < 0) ? yaw + 360 : yaw;  // 归一化到0~360
    int err = target - yaw0_360;
    if (err > 180) err -= 360;    // 角度差最短路径
    if (err < -180) err += 360;
    int diff = err - err_last;
    if (err_last == 0 && err != 0) diff = err / 2;  // 首次微分减半
    out = ka2p * err + ka2d * diff;  // Kp=0.5, Kd=3.5
    // 限幅 ±20
    err_last = err;
    return out;
}
// 用法：left = base_speed - pid_out; right = base_speed + pid_out;
```

**多圈状态机（直行→循迹交替）：**
```
step 1: 陀螺仪直行(航向角0°) → 遇到黑线 → step 2
step 2: 红外循迹 → 离开黑线 → step 3
step 3: 陀螺仪直行(航向角168°) → 遇到黑线 → step 4
step 4: 红外循迹 → 离开黑线 → step 5
...（每圈4步=2段直行+2段循迹，角度目标递增）
```

### 2023 E题国一方案（云台追踪参考）

双云台系统（红激光模拟目标 + 绿激光追踪），STC32G + TC264 + OpenMV，步进电机驱动。

### 控制算法选型经验

| 场合 | 推荐算法 | 关键细节 |
|------|---------|---------|
| 舵机云台 | P/PD + EMA平滑 | I项易震荡，EMA替代D项阻尼 |
| 步进电机 | 位置式PID | RPM→PWM频率映射，中断中执行 |
| 直流电机+编码器 | 串级PID | 位置外环→速度内环，PWM限幅 |
| 循迹小车 | PD方向环 + PI速度环 | 红外查表+消抖，速度差分配 |
| 角度控制 | PD + 归一化 | `±180°`最短路径，首次微分减半 |

### 本系统 vs 国一方案差异

| 维度 | 本系统 | 2025国一 |
|------|--------|---------|
| 视觉 | K230灰度矩形检测 + 几何验证 | K230矩形检测 (`cv_lite.find_rectangles`) |
| 目标定位 | 矩形对角线交点 | 矩形对角线交点 |
| 云台执行器 | ZDT_X42S闭环步进电机（UART Emm F6速度模式） | 步进电机（脉冲+方向） |
| 控制算法 | PID + 三段限速（主循环25Hz） | 位置式PID（中断250Hz） |
| 控制位置 | 主循环轮询(25Hz) | 定时器中断(250Hz) |
| 电机通信 | 双UART独立（USART2+USART3） | 单总线 |
| 通信协议 | `X<err>Y<err>Z<valid>\n` | `[±xxx±yyy*]` |
| 显示 | SSD1306 OLED (I2C) | OLED |

## Project Documents

| 文档 | 内容 |
|------|------|
| [PINOUT.md](PINOUT.md) | 引脚配置（权威来源） |
| [e题.txt](e题.txt) | 2025 E题完整题目 |
| [docs/电赛控制类题目汇总_2022-2024.md](docs/电赛控制类题目汇总_2022-2024.md) | 历年控制题 |
| [docs/电赛控制类优秀方案汇总_2022-2024.md](docs/电赛控制类优秀方案汇总_2022-2024.md) | 优秀方案 |

## ZDT_X42S 步进电机参考

驱动板资料位于 `Downloads/ZDT_XS系列第二代闭环步进电机资料/`，关键文档：
- `3. 说明书/ZDT_X42S第二代闭环步进电机用户手册V1.0.4_260401.pdf` — 协议手册
- `3. 说明书/ZDT闭环步进电机MODBUS协议使用说明V1.0.1_260401.pdf` — MODBUS协议
- `10.例程_STM32F103/` — 官方 STM32 例程（串口/CAN/脉冲三种控制方式）
