# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

2025全国大学生电子设计竞赛E题"简易自行瞄准装置"的瞄准模块——二维舵机云台目标追踪系统。硬件：K230摄像头（亚博智能开发套件）+ STM32F103C8 + 两个MG996R 180°舵机。

## Architecture

```
K230 (MicroPython)                    STM32F103C8 (C, Keil MDK)
├── 橙色目标检测 (Lab blob)  ──UART──→  ├── 中断接收坐标
├── 发送像素坐标 X<val>Y<val>\n        ├── 100Hz P控制循环 (10ms)
└── 画面显示 + OSD                     ├── EMA平滑 → PWM输出舵机
                                       ├── 200ms看门狗超时保活
                                       └── 丢失搜索状态机 (外推→螺旋→归位)
```

**关键设计决策：**
- 控制逻辑在STM32端而非K230端，以获得100Hz控制频率（K230只能20Hz）
- 纯P控制（无I/D），EMA平滑替代D项阻尼
- 协议极简：K230只发原始像素坐标，STM32计算误差和控制量

## Build

- **STM32**: Keil MDK v5, 用 ARM Compiler 5 (V5.06). 打开 `Project.uvprojx`，编译目标ST STM32F103C8.
- **K230**: 将 `k230/main.py` 通过CanMV IDE或TF卡放到K230开发板运行.

## Hardware Pin Map

| STM32引脚 | 功能 | 连接 |
|-----------|------|------|
| PA0 (TIM2_CH1) | PWM Pan舵机 | 下方舵机信号线 |
| PA1 (TIM2_CH2) | PWM Tilt舵机 | 上方舵机信号线 |
| PA9 (USART1_TX) | UART发送 | K230 RX |
| PA10 (USART1_RX) | UART接收 | K230 TX |

MG996R舵机须外部5V/2A+独立供电，不可从STM32取电。电源并联1000μF电容防抖。

## Communication Protocol

K230 → STM32: `X<像素X>Y<像素Y>\n`，例 `X320Y240\n`，坐标基于640×480画面。

## Key Tuning Parameters (main.c)

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `KP_X / KP_Y` | 0.025 | P增益，像素→角度/步 |
| `DEAD_ZONE` | 20 | 死区(px)，防微抖 |
| `MAX_STEP` | 3.0 | 单步最大角度变化 |
| `EMA_ALPHA` | 0.30 | 角度平滑，越小越拖慢但越稳 |
| `CTRL_INTERVAL_MS` | 10 | 控制周期(100Hz) |
| `WATCHDOG_MS` | 200 | 丢目标后保持时间 |

## Lost Target Search (main.c)

目标出画幅后自动执行三段式搜索：

```
TRACKING ──(50ms无数据)──→ EXTRAPOLATE ──(200ms外推)──→ SEARCH ──(螺旋完毕)──→ RETURN ──(到位)──→ IDLE
    ↑                         ↑                        ↑                      ↑
    └──────── 收到坐标即中断，立即切回 TRACKING ────────┘                      │
    └────────────────────── 收到坐标即中断 ─────────────────────────────────────┘
```

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `LOST_THRESHOLD` | 5 | 连续无数据周期数(×10ms)才进入搜索 |
| `EXTRAPOLATE_STEPS` | 20 | 外推步数，沿最后运动方向继续 |
| `EXTRAPOLATE_SPEED` | 0.7 | 外推每步角度(°) |
| `SEARCH_LEG_STEP` | 3.0 | 螺旋每段增量(°)，控制搜索密度 |
| `SEARCH_STEP_SPEED` | 0.5 | 搜索每步角度(°) |
| `SEARCH_MAX_RANGE` | 35.0 | 搜索相对外推终点的最大范围(°) |
| `SEARCH_MAX_LEGS` | 16 | 最大搜索段数，超限进入归位 |
| `RETURN_SPEED` | 1.2 | 归位每步角度(°) |

搜索过程任何时刻收到新坐标，立即退出搜索、回到追踪模式。

## K230 Color Threshold

橙色Lab阈值在 `k230/main.py` 的 `ORANGE_THRESHOLD = (40, 85, 20, 60, 30, 80)`。格式：(L_min, L_max, A_min, A_max, B_min, B_max)。需根据实际光照标定。
