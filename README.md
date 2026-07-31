# Spinphony

**English** | [简体中文](README.zh-CN.md)

A `#![no_std]` Field-Oriented Control (FOC) library for BLDC / PMSM motors, written in Rust.

> **Note:** This library is still at a very early stage — it's currently a personal hobby project, and I'll keep updating it when I have time 😀

Spinphony is designed for bare-metal embedded targets: no heap allocation, `f32` math backed by [`libm`](https://crates.io/crates/libm), and logging via [`defmt`](https://crates.io/crates/defmt). Every component implements a unified `DynamicSystem` trait, so controllers, observers and filters compose the same way.

> The name is a pun — *spin* + *symphony*. Yes, it can also play music on your motor (see `constant::cannon_in_d`).

## Features

- **FOC core** — Clarke / Park transforms and their inverses, angle wrapping utilities, dq current measurement
- **Modulation** — SPWM, SVPWM (classic sector method and Min-Max zero-sequence injection), with voltage limiting and DC-bus bias output ready for PWM duty cycles
- **Cascade control** — position → velocity → current (torque) loops built on a PID controller with output limiting and integral anti-windup
- **Control modes** — position / velocity / current closed loop, velocity open loop, and sensorless velocity control with V/F launch and a smooth open-loop → closed-loop transition
- **Sensorless observers** — sliding-mode back-EMF observer + PLL angle observer, plus an angular velocity calculator
- **Filters** — first-order (single-pole) low-pass filter
- **Extras** — oscillator, melody player, MIDI note frequency table and a `note!` macro for driving a motor as a speaker
- **Highly portable** — thanks to the embedded-rust ecosystem's reusable cross-platform abstractions, projects like `svd2rust` (PAC generation), and async runtimes like [embassy](https://github.com/embassy-rs/embassy), this library can be ported almost seamlessly to any MCU with a published SVD file

## Getting Started

Add the dependency to your `Cargo.toml`:

```toml
[dependencies]
spinphony = { git = "https://github.com/Carbohydrate-42/spinphony.git" }
```

## Module Overview

| Module | Contents |
|---|---|
| `algorithm` | `FieldOrientedControl` — coordinate transforms, SPWM / SVPWM modulation |
| `controller` | `MotorController`, `PosVelCurrCascadeController`, `PIDController`, `VoltageFrequencyLauncher`, `Oscillator`, `MelodyPlayer` |
| `observer` | `AngleObserver` (SMO + PLL), `SlidingModeBackEmfObserver`, `PhaseLockLoopAngleObserver`, `AngularVelocityCalculator` |
| `filter` | `LowPass1Pole` |
| `core` | `DynamicSystem` trait, `FrequencyDivider`, `ExecutionLimiter`, `Note` / `note!` |
| `constant` | MIDI frequency table, test notes, Canon in D demo melody |

## Quick Start

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

// 1. Configure the controller
let param = MotorControllerParam {
    position_velocity_current_cascade_controller_param: PosVelCurrCascadeControllerParam {
        current_loop_pid_param: PIDControllerParam { /* kp, ki, kd, limits .. */ ..Default::default() },
        velocity_loop_pid_param: PIDControllerParam { /* .. */ ..Default::default() },
        position_loop_pid_param: PIDControllerParam { /* .. */ ..Default::default() },
        mode: MotorControlMode::Velocity,
    },
    voltage_frequency_launcher_param: VoltageFrequencyLauncherParam::default(),
    voltage_power_supply: 24.0, // Vbus
    pole_pairs: 7,
    zero_position_angle: 0.0,
};

let mut motor = MotorController::new(param);

// 2. In your control loop (e.g. a PWM-period interrupt):
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

    // 3. (ua, ub, uc) already include the Vbus bias — map them to PWM duty cycles
    let (ua, ub, uc) = motor.output();
    set_pwm_duty(ua, ub, uc);
}
```

## Control Modes

| Mode | Description |
|---|---|
| `Position` | Position → velocity → current cascade, closed loop with encoder |
| `Velocity` | Velocity → current cascade, closed loop with encoder |
| `Current` | Current (torque) loop only |
| `VelocityOpenLoop` | Open-loop velocity control (no feedback) |
| `VelocitySensorless` | V/F launch → SMO + PLL closed loop, with smooth transition |
| `PositionSensorless` | Not implemented yet |

## What This Library Does NOT Do

Spinphony is a pure control-algorithm library. It is hardware-agnostic and deliberately leaves all peripherals to your HAL / application layer, including:

- **Position sensor drivers** — encoders, Hall sensors, resolvers, magnetic encoders (ABI / SPI / etc.). You read the sensor and pass `position_meas` in
- **PWM / DAC output** — complementary PWM with dead time, plain PWM, or even DAC output. The library only returns the three-phase voltages `(ua, ub, uc)`; how they reach the gate driver is up to you
- **Current sensing** — single / dual / three-shunt sampling, phase-current ADC drivers, and the scheduling of ADC triggers synchronized with PWM
- **Peripheral & board configuration** — clocks, timers, GPIO, DMA, interrupts, and control-loop scheduling

In short: you own the hardware; Spinphony turns measurements into voltage commands.

## Roadmap

1. Fixed-point calculation support, based on [fixed](https://gitlab.com/tspiteri/fixed)
2. More advanced control algorithms, for example:
   - Luenberger observer
   - Extended Kalman filter (EKF)
   - High-frequency injection (HFI) for low-speed / zero-speed sensorless control
   - Field weakening control
   - MTPA (maximum torque per ampere)
   - Model predictive control (MPC)
3. More examples on different MCUs — and even on embedded Linux SoCs (in `std` mode)

## License

[MIT](LICENSE)
