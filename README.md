# La Voiture — Field Tracking & Autonomous Robot Car

Embedded firmware and desktop GUI for an autonomous robot car that tracks magnetic field paths, accepts joystick remote control, and follows waypoint routes drawn on a desktop application.

Built on the **STM32L051K8T6** (ARM Cortex-M0+, 32 MHz) with bare-metal C and a **PyQt5** real-time dashboard.

## System Overview

```
  Laptop (GUI)                Remote Controller              Robot Car (this repo)
 ┌────────────┐   BLE/UART   ┌────────────────── ┐   IR TX/RX  ┌─────────────────┐
 │ PyQt5 App  │◄────────────►│ EFM8LB1 MCU       │◄───────────►│ STM32L051K8T6   │
 │  - 3D viz  │  JDY-23 BLE  │  - LCD + Joystick │  38 kHz     │  - H-Bridge     │
 │  - Path    │  9600 baud   │  - Mode select    │  28-bit     │  - 3x Inductors │
 │    editor  │              │  - IR transceiver │  SIRC       │  - LSM6DS33 IMU │
 │  - Gauges  │              │                   │  protocol   │  - VL53L0X ToF  │
 └────────────┘              └───────────────────┘             └─────────────────┘
```

## Three Driving Modes

### 1. Field Tracking (Fastest in Lab)
Follows wires carrying fixed-frequency current using a 3-inductor array (2 parallel + 1 vertical). A custom-tuned PI controller steers the car along the wire path, with automatic intersection detection and configurable turn sequences.

### 2. Joystick Remote Control
The controller's joystick ADC values (X/Y axes) are mapped to 0–255, transmitted via IR in real-time, and translated to differential motor commands.

### 3. Path Tracking (Autonomous Waypoint Navigation)
Routes drawn on the PyQt5 GUI are transmitted via Bluetooth to the controller, then forwarded via IR to the car. An onboard IMU (LSM6DS33) provides yaw feedback for trajectory following, with a 7-state FSM handling intersection detection, compensation, turning, and settling.

## Key Engineering

### Bidirectional IR Protocol

![IR Protocol Waveform](IR_Transmitter_Side.png)

The custom "TIBO IR Protocol" uses pulse-distance encoding on a 38 kHz carrier:

- **Physical layer**: 10 carrier cycles = 1 burst (~263 µs = 1T unit). The TSOP receiver inverts the signal — bursts become low pulses.
- **Bit encoding**: Bit 0 = 1T burst + 3T silence (falling-to-falling ~3T). Bit 1 = 1T burst + 4T silence (falling-to-falling ~4T). Threshold at 3.5T (~921 µs).
- **Frame structure (28 bits, MSB first)**: Start sign (4T burst + 2T space) → Command byte (8 bits) → High data byte (8 bits) → Low data byte (8 bits) → Address nibble (4 bits) → Stop sign (1T burst).
- **Hardware**: SFH4546 IR LED + TSOP33338 receiver
- **Challenge solved**: High duty cycles caused the receiver's AGC to drop gain mid-packet. Re-engineered frames with larger silence areas to keep duty cycle below 50%, eliminating signal degradation during simultaneous bidirectional transmission.

### Inductive Line Following
- 3 ADC channels sample left/right/front inductors continuously
- Proportional steering: `error = (v_right - v_left) / (v_right + v_left)`
- Front inductor threshold triggers intersection detection
- Tuning: `KP=0.022`, `KS=0.02`, `KF=0.020`, base power `PB=95`

### IMU Odometry
- LSM6DS33 (6-axis accel + gyro) over I2C at address 0x6A/0x6B
- Gyro integration for yaw tracking (calibrated with 200-sample offset)
- Roll/pitch estimation from accelerometer
- Position integration: `pose_x += speed * cos(yaw)`, `pose_y += speed * sin(yaw)`

## Hardware

| Component | Part | Interface | Purpose |
|-----------|------|-----------|---------|
| MCU | STM32L051K8T6 | — | ARM Cortex-M0+ @ 32 MHz, 64 KB Flash, 8 KB RAM |
| IMU | LSM6DS33 | I2C (0x6A) | 6-axis accel + gyro for orientation & odometry |
| ToF Sensor | VL53L0X | I2C (0x29) | Distance measurement |
| IR TX | SFH4546 LED | PA6 (TIM22 PWM) | 38 kHz carrier, pulse-distance encoding |
| IR RX | TSOP33338 | PA7 (EXTI) | Demodulated IR input |
| Motors | Geared DC x2 | H-Bridge + PWM | Right: PA2/PA3, Left: PA15/PB3 |
| Inductors | 3x coils | ADC ch0/1/9 | Left, right, front magnetic field sensing |
| BLE | JDY-23 | UART1 (115200) | Telemetry to PC GUI |

## Desktop GUI (PyQt5)

The GUI provides real-time visualization and control:

- **3D Vehicle Attitude**: Roll/pitch rendering with wheel animation
- **Telemetry Dashboard**: Acceleration, gyro, angles, motor power gauges
- **Path Editor**: Interactive grid canvas for drawing waypoint routes (up to 32 waypoints)
- **BLE Terminal**: Device scanning, connection management, command console
- **Robot Trail**: Real-time position tracking on the path grid (last 2000 points)

```bash
pip install PyQt5 bleak
python Core/Src/gui.py
```

## Project Structure

```
Core/
  Src/
    main.c            Main FSM logic, motor control, sensor fusion (1800 lines)
    ir_rx.c           IR pulse-distance decoder (EXTI + Timer)
    ir_tx.c           IR transmitter (38 kHz carrier + envelope)
    vl53l0x.c         VL53L0X ToF distance sensor driver
    adc.c             ADC for inductive sensors
    i2c.c             I2C peripheral (IMU + ToF)
    tim.c             Timer configuration (PWM, IR, sampling)
    usart.c           UART telemetry output
    gpio.c            GPIO pin configuration
    gui.py            PyQt5 dashboard application
    ble_receiver.py   Async BLE client (Bleak)
    pathfinder.py     Waypoint grid editor widget
  Inc/
    datatypes.h       IMU constants, PI tuning parameters
    config.h          IR protocol definitions, timing, register map
    main.h            GPIO mappings, state machine enums
    ir_rx.h / ir_tx.h IR frame structures and APIs
cmake/
  gcc-arm-none-eabi.cmake   ARM cross-compilation toolchain
  stm32cubemx/              HAL driver integration
Drivers/                    STM32 HAL + CMSIS libraries
CMakeLists.txt              Top-level CMake configuration
build.py                    Cross-platform build entry point
build.sh / build.ps1        Platform-specific build scripts
STM32L051XX_FLASH.ld        Linker script (64 KB Flash, 8 KB RAM)
Porject2_test_1.ioc         STM32CubeMX project file
```

## Build & Flash

### Prerequisites
- `arm-none-eabi-gcc` (ARM GCC toolchain)
- CMake 3.22+
- Ninja (recommended) or Make
- Python 3 (for build script and GUI)

### First-Time Setup
```bash
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake -G Ninja
```

### Build & Flash
```bash
python build.py build
python build.py flash -Port COM3          # Windows
python build.py flash -p /dev/tty.xxx     # macOS/Linux
python build.py clean
```

### Flashing via UART Bootloader
1. Hold the **BOOT0** button
2. Press the **RESET** button
3. Release BOOT0 — MCU enters ROM bootloader mode
4. Run `python build.py flash`

## IR Protocol Reference

| Cmd ID | Name | Value | Direction |
|--------|------|-------|-----------|
| 0 | START | 0x0000 | Controller → Car |
| 1 | PAUSE | 0x0000 | Controller → Car |
| 2 | RESET | 0x0000 | Controller → Car |
| 3 | MODE | 0=field, 1=remote, 2=path | Controller → Car |
| 4 | PATH | 1–3=preset, 4=manual | Controller → Car |
| 5–6 | JOYSTICK_X/Y | 0–255 | Controller → Car |
| 7–24 | IMU_REG[0–17] | 16-bit sensor value | Car → Controller |
| 25 | DATA_RECEIVED | mode confirmation | Car → Controller |
| 26 | CROSSING_ACTION | 0=fwd, 1=left, 2=right, 3=stop | Car → Controller |
| 27–28 | LEFT/RIGHT_POWER | motor power | Car → Controller |
| 39 | ZERO_YAW | calibration trigger | Controller → Car |
| 40 | MANUAL_PATH | packed 2-bit directions | Controller → Car |

## Telemetry Output (UART)

The car streams sensor data over UART1 at 115200 baud:
```
ACC[g] X:0.12 Y:-0.03 Z:0.99 | GYRO[dps] X:1.2 Y:-0.5 Z:0.3
ANGLE roll:6.8 deg | pitch:-1.7 deg
```

## State Machines

### Top-Level Controller FSM
`STATE_CONFIG` → `STATE_DRIVE` → `STATE_PAUSE` (transitioned by IR commands)

### Car Mode FSM
- `STATE_FIELD_TRACKING` — PI-controlled inductive line following
- `STATE_REMOTE` — Direct joystick motor mapping
- `STATE_PATH_TRACKING` — Autonomous waypoint navigation

### Path Tracking Sub-FSM (7 states)
`Resting` → `Running` → `Intersection_encountered` → `Intersection_compensation` → `Intersection_turning` → `Intersection_turn_settle` → `Intersection_stop`
