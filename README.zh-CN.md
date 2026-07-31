# Spinphony

[English](README.md) | **简体中文**

一个用 Rust 编写的 `#![no_std]` 磁场定向控制（FOC）库，面向 BLDC / PMSM 电机。

> **注意：** 这个库还处于非常早期的阶段 —— 当前只是一个个人兴趣驱动的项目，有空之后会继续更新 😀

Spinphony 专为裸机嵌入式目标设计：无堆分配，基于 [`libm`](https://crates.io/crates/libm) 的 `f32` 数学运算，日志使用 [`defmt`](https://crates.io/crates/defmt)。所有组件都实现统一的 `DynamicSystem` trait，因此控制器、观测器和滤波器可以以相同的方式组合使用。

> 名字是个双关 —— *spin*（旋转）+ *symphony*（交响乐）。没错，它还能让你的电机播放音乐（见 `constant::cannon_in_d`）。

## 特性

- **FOC 核心** —— Clarke / Park 变换及其逆变换、角度环绕工具、dq 电流测量
- **调制方式** —— SPWM、SVPWM（经典扇区法和 Min-Max 零序注入），带电压限幅和母线电压偏置，输出可直接映射为 PWM 占空比
- **级联控制** —— 位置 → 速度 → 电流（转矩）三环级联，基于带输出限幅和积分抗饱和的 PID 控制器
- **控制模式** —— 位置 / 速度 / 电流闭环、速度开环，以及无感速度控制（V/F 启动，开环到闭环平滑切换）
- **无感观测器** —— 滑模反电动势观测器（SMO）+ 锁相环角度观测器（PLL），以及角速度计算器
- **滤波器** —— 一阶（单极点）低通滤波器
- **其他** —— 振荡器、旋律播放器、MIDI 音高频率表和 `note!` 宏，可以把电机当扬声器驱动
- **高度可移植** —— 得益于 embedded-rust 生态高度可复用的跨平台抽象、`svd2rust` 等 PAC 生成项目，以及 [embassy](https://github.com/embassy-rs/embassy) 这类异步运行时，这个库几乎可以无缝迁移到任何公开了 SVD 文件的 MCU 上运行

## 快速开始

在 `Cargo.toml` 中添加依赖：

```toml
[dependencies]
spinphony = { git = "https://github.com/Carbohydrate-42/spinphony.git" }
```

## 模块概览

| 模块 | 内容 |
|---|---|
| `algorithm` | `FieldOrientedControl` —— 坐标变换、SPWM / SVPWM 调制 |
| `controller` | `MotorController`、`PosVelCurrCascadeController`、`PIDController`、`VoltageFrequencyLauncher`、`Oscillator`、`MelodyPlayer` |
| `observer` | `AngleObserver`（SMO + PLL）、`SlidingModeBackEmfObserver`、`PhaseLockLoopAngleObserver`、`AngularVelocityCalculator` |
| `filter` | `LowPass1Pole` |
| `core` | `DynamicSystem` trait、`FrequencyDivider`、`ExecutionLimiter`、`Note` / `note!` |
| `constant` | MIDI 频率表、测试音符、《D 大调卡农》示例旋律 |

## 使用示例

```rust
use spinphony::core::dynamic_system::DynamicSystem;
use spinphony::controller::motor_controller::{
    MotorController, MotorControllerParam, MotorControllerInput, MotorControlMode,
};
use spinphony::controller::pos_vel_curr_cascade_controller::{
    PosVelCurrCascadeControllerParam, PosVelCurrCascadeControllerInput,
};
use spinphony::controller::pid_controller::PIDControllerParam;
use spinphony::controller::voltage_frequency_launcher::VoltageFrequencyLauncherParam;

// 1. 配置控制器
let param = MotorControllerParam {
    position_velocity_current_cascade_controller_param: PosVelCurrCascadeControllerParam {
        current_loop_pid_param: PIDControllerParam { /* kp, ki, kd, 限幅 .. */ ..Default::default() },
        velocity_loop_pid_param: PIDControllerParam { /* .. */ ..Default::default() },
        position_loop_pid_param: PIDControllerParam { /* .. */ ..Default::default() },
        mode: MotorControlMode::Velocity,
    },
    voltage_frequency_launcher_param: VoltageFrequencyLauncherParam::default(),
    voltage_power_supply: 24.0, // 母线电压
    pole_pairs: 7,
    zero_position_angle: 0.0,
};

let mut motor = MotorController::new(param);

// 2. 在控制循环中（例如 PWM 周期中断）：
loop {
    motor.set_input(MotorControllerInput {
        position_velocity_current_cascade_controller_input:
            PosVelCurrCascadeControllerInput {
                velocity_ref: 10.0,                 // rad/s
                position_meas: read_encoder(),      // rad
                velocity_meas: estimate_velocity(), // rad/s
                current_meas: read_phase_current(), // A
                ..Default::default()
            },
    });

    motor.update(dt);

    // 3. (ua, ub, uc) 已包含母线电压偏置 —— 直接映射为 PWM 占空比
    let (ua, ub, uc) = motor.output();
    set_pwm_duty(ua, ub, uc);
}
```

## 控制模式

| 模式 | 说明 |
|---|---|
| `Position` | 位置 → 速度 → 电流三环级联，编码器闭环 |
| `Velocity` | 速度 → 电流双环级联，编码器闭环 |
| `Current` | 仅电流（转矩）环 |
| `VelocityOpenLoop` | 速度开环控制（无反馈） |
| `VelocitySensorless` | V/F 启动 → SMO + PLL 无感闭环，平滑切换 |
| `PositionSensorless` | 尚未实现 |

## 这个库不做什么

Spinphony 是一个纯控制算法库，与硬件无关。以下外设相关的工作刻意留给你的 HAL / 应用层处理：

- **位置传感器驱动** —— 编码器、霍尔传感器、旋转变压器、磁编码器（ABI / SPI 等）。你需要自己读取传感器，把 `position_meas` 传进来
- **PWM / DAC 输出** —— 带死区的互补 PWM、普通 PWM，甚至 DAC 输出。库只返回三相电压 `(ua, ub, uc)`，如何送到栅极驱动器由你决定
- **电流采样** —— 单 / 双 / 三电阻采样、相电流 ADC 驱动，以及与 PWM 同步的 ADC 触发调度
- **外设与板级配置** —— 时钟、定时器、GPIO、DMA、中断，以及控制循环的调度

一句话：硬件归你管，Spinphony 负责把测量值变成电压指令。

## 路线图

1. 支持定点数计算，基于 [fixed](https://gitlab.com/tspiteri/fixed)
2. 更多先进控制算法，例如：
   - 龙伯格观测器（Luenberger Observer）
   - 扩展卡尔曼滤波（EKF）
   - 高频注入（HFI），用于低速 / 零速无感控制
   - 弱磁控制
   - MTPA（最大转矩电流比控制）
   - 模型预测控制（MPC）
3. 增加更多不同 MCU 下的示例 —— 甚至包括嵌入式 Linux SoC（`std` 模式）

## 许可证

[MIT](LICENSE)
