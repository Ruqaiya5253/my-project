# Self‑Balancing Robot – STM32F3 + Keyestudio Kit

[Project Report](Project_Report/Project_Report.pdf)

## Overview
This project implements a two‑wheeled self‑balancing robot using the
STM32F3 Discovery board (STM32F303VCT6) and the Keyestudio Self‑Balancing
Robot Kit. The microcontroller reads orientation from an MPU‑6050 IMU,
estimates the tilt angle with a complementary filter, and runs a position
PID controller at a fixed rate of 200 Hz (configurable). The controller
output drives two DC motors through the motor shield, keeping the robot
upright and recovering from minor disturbances.

The firmware is written entirely in C using the STM32 HAL libraries,
built with CMake, and flashed from VS Code. It follows a non‑blocking,
interrupt‑driven architecture: all time‑critical sensor reads, angle
calculations, and the PID loop execute inside a hardware timer ISR,
guaranteeing deterministic timing. A separate UART telemetry stream
(also non‑blocking) transmits the robot’s angle, gyro rate, motor
command, and encoder speeds for real‑time plotting and debugging.

---

## Hardware Components
* **Microcontroller**: STM32F3 Discovery (STM32F303VCT6)
* **IMU**: MPU‑6050 (6‑axis accelerometer + gyroscope), on the Keyestudio shield
* **Motor Shield**: Keyestudio Self‑Balancing Car Shield V3
  * 2× DC motors with integrated magnetic encoders
  * L293D‑compatible dual H‑bridge driver
  * On‑board 5 V regulator for the encoder modules
* **Power**: 7–12 V battery or bench supply connected to the shield’s `Vin`
  terminal. The STM32F3 board is powered via its ST‑LINK USB connector.

---

## Pin Mapping
All connections between the STM32F3 Discovery board and the Keyestudio
shield are summarised below. Ground (`GND`) must be shared between the
STM32 board and the shield.

---

### Table 1 — Full Pin Mapping (Detailed)

| STM32 | Shield Pin  | Function          | Description                                    |
| ----- | ----------- | ----------------- | ---------------------------------------------- |
| PA15  | D4 (pin 8)  | R_m encoder input | TIM2_CH1 input capture, measures R_wheel speed |
| PA1   | D5 (pin 9)  | L_m encoder input | TIM2_CH2 input capture, measures L_wheel speed |
| PB10  | D9 (pin 4)  | L_m PWM output    | TIM2_CH3 PWM signal for L_m speed control      |
| PB11  | D10 (pin 3) | R_m PWM output    | TIM2_CH4 PWM signal for R_m speed control      |
| PD8   | D8 (pin 5)  | R_m direction B   | GPIO output (IN2)                              |
| PD9   | D12 (pin 2) | R_m direction A   | GPIO output (IN1)                              |
| PD10  | D7 (pin 6)  | L_m direction A   | GPIO output (IN1)                              |
| PD11  | D6 (pin 7)  | L_m direction B   | GPIO output (IN2)                              |
| PA9   | SCL         | I²C clock for IMU | I2C2 serial clock line to MPU-6050             |
| PA10  | SDA         | I²C data for IMU  | I2C2 serial data line to MPU-6050              |
| PA2   | TX          | UART transmit     | USART2 TX pin                                  |
| PA3   | RX          | UART receive      | USART2 RX pin                                  |

---

### Table 2 — Functional Mapping (Firmware-Oriented)

| STM32 Pin | Shield Function     | Description                               |
| --------- | ------------------- | ----------------------------------------- |
| PA15      | TIM2_CH1            | Right motor encoder input (Input Capture) |
| PA1       | TIM2_CH2            | Left motor encoder input (Input Capture)  |
| PB10      | TIM2_CH3            | Left motor PWM output                     |
| PB11      | TIM2_CH4            | Right motor PWM output                    |
| PD8       | D12 (Right DirA)    | Direction control, right motor A          |
| PD9       | D8 (Right DirB)     | Direction control, right motor B          |
| PD10      | D10 (Left DirA)     | Direction control, left motor A           |
| PD11      | D7 (Left DirB)      | Direction control, left motor B           |
| PA9       | I2C2_SCL            | MPU-6050 serial clock                     |
| PA10      | I2C2_SDA            | MPU-6050 serial data                      |
| PA2       | USART2_TX           | Telemetry UART transmit                   |
| PA3       | USART2_RX           | Telemetry UART receive (optional)         |
| GND       | GND                 | Common ground (must be connected)         |
| 5V        | 5V (shield encoder) | Powers encoder modules (max 100 mA)       |


* **Note:** The `Vin` pin on the shield (motor supply) should **never** be
  connected to the STM32’s 5 V or 3 V pins. Use a separate battery or
  power supply that can deliver at least 1 A for the motors.

### Motor Control Logic
| Direction  | Right Motor (PD8/D12, PD9/D8) | Left Motor (PD10/D10, PD11/D7) |
|------------|-------------------------------|--------------------------------|
| Forward    | PD8 = HIGH, PD9 = LOW         | PD10 = HIGH, PD11 = LOW        |
| Backward   | PD8 = LOW, PD9 = HIGH         | PD10 = LOW, PD11 = HIGH        |
| Stop/Coast | Both LOW                      | Both LOW                       |

The PWM duty cycle for each motor is set via `__HAL_TIM_SET_COMPARE` on
TIM2_CH3 (right) and TIM2_CH4 (left). The signed `motor_cmd` value is
clamped to ±`MOTOR_PWM_MAX` (currently 1599, equal to TIM2 ARR).

---

## System Architecture & Control Loop
### 1. Angle Estimation (Complementary Filter)
The tilt angle is calculated from MPU‑6050 accelerometer and gyroscope
readings using a complementary filter:

```
angle = 0.98 * (angle + gyro_rate * dt) + 0.02 * accel_angle
```

* `gyro_rate`: Angular velocity about the robot’s pitch axis (raw
  gyroscope data scaled to °/s and bias‑corrected).
* `accel_angle`: Tilt angle derived from the Y and Z accelerometer axes
  via `atan2f(acc_y, acc_z) * 180/pi`. The sensor orientation is
  calibrated such that a forward tilt gives a positive angle.
* `dt`: Control loop sampling time = 1 / 200 Hz = 5 ms (fixed by TIM6).

The filter runs exclusively inside the TIM6 ISR, guaranteeing a constant
update rate and eliminating jitter from UART or other software delays.

### 2. PID Controller
A position PID controller keeps the robot upright (setpoint = 0°). The
control law is:

```
output = Kp * error + Ki * integral(error) + Kd * derivative(error)
```

* **Kp** = 55.0, **Ki** = 1.2, **Kd** = 2.8 (initial hand‑tuned values,
  subject to further optimisation).
* Integral clamping limits windup.
* The output is clamped to ±`MOTOR_PWM_MAX`.
* The same command is applied to both motors to maintain forward/backward
  balance. (Differential steering is not yet implemented.)

The PID computation also takes place inside the TIM6 ISR, immediately
after the angle update, so the control loop latency is minimal.

### 3. Motor Driver
The `motor_set_speed(motor, speed)` function translates a signed speed
command into direction GPIO levels and the corresponding PWM duty cycle.
A positive speed drives the robot forward; a negative speed drives it
backward.

### 4. Encoder Monitoring (Extra Feature)
Both motor encoders are connected to TIM2 in Input Capture mode. Each
rising edge stores the counter value, and the period between consecutive
edges is computed inside the capture callback. The frequency (Hz) of the
encoder pulses is stored in volatile variables.

The encoder data is **not** used in the balance controller; it is read
exclusively for telemetry and diagnostics. This allows the user to
observe motor speeds in real time and can later be extended for a
cascaded speed‑control loop.

### 5. Telemetry & Debugging
Every Nth control cycle (currently every 10th, i.e., at 20 Hz), the
TIM6 ISR sets a flag. In the main `while(1)` loop, a non‑blocking UART
transmit routine sends a comma‑separated line containing:

```
angle_deg, gyro_rate_dps, motor_cmd, left_encoder_Hz, right_encoder_Hz
```

A Python script (provided in the repository) reads this stream and plots
the values in real time, enabling live tuning and fault diagnosis.

---

## Firmware Modules
The project is split into separate, well‑defined modules:

| File | Responsibility |
|------|----------------|
| `mpu6050.c/.h` | I²C communication with the MPU‑6050, initialization, calibration, reading raw and scaled accelerometer/gyroscope data. |
| `complementary.c/.h` | Complementary filter for angle estimation; relies on `mpu6050` for sensor access. |
| `pid.c/.h` | Generic PID controller with anti‑windup integral clamping and output limiting. |
| `motor.c/.h` | Motor direction control and PWM duty‑cycle setting via GPIO and TIM2. |
| `encoder.c/.h` | TIM2 input‑capture driver for both motor encoders; provides pulse frequency and count. |
| `main.c` | System initialisation, start of all modules, TIM6 ISR (angle update + PID + motor set), telemetry flag handling, and UART transmit callback. |

All modules are independent and can be tested or replaced individually.
The only shared resources are the I²C handle (`hi2c2`) and the TIM2 handle.

---

## Getting Started
### Prerequisites
* STM32F3 Discovery board
* Keyestudio self‑balancing robot kit (assembled per the
  [official video](https://www.youtube.com/watch?v=PTaSzV7YRuE))
* USB‑to‑UART adapter (e.g., CP2102) for telemetry
* ST‑LINK/V2 (integrated on the Discovery board)
* VS Code with STM32CubeMX extension, ARM GCC toolchain, CMake, and Git

### Build & Flash
1. Clone the repository and open the folder in VS Code.
2. Generate the code using STM32CubeMX (if needed) with the `.ioc` file.
3. Build the project with `Ctrl+Shift+B` (select the CMake target).
4. Connect the ST‑LINK USB cable and the serial adapter.
5. Flash and debug with the STM32Cube debug configuration (ST‑Link GDB
   server).
6. Open a serial terminal (e.g., `screen /dev/ttyUSB0 115200`) or run
   the provided Python plotting script.

### First‑Time Calibration
* Place the robot on a flat, stationary surface in its balancing
  orientation.
* Power on the board; the firmware automatically calibrates the MPU‑6050
  offsets (300 samples are averaged). The calibration runs once during
  `start_robot_modules()` before the control loop starts.
* After calibration the robot will attempt to balance. Hold it firmly
  during initial tests while tuning gains.

### Tuning the PID
1. Start with `Kp` small (e.g., 20) and increase until the robot
   oscillates around the upright position.
2. Add `Kd` to dampen the oscillations. Increase gradually until the
   robot recovers quickly from a nudge without overshooting.
3. If the robot drifts away, add a small `Ki` to eliminate the
   steady‑state error.
4. Use the live telemetry plot to visualise angle and motor command;
   adjust gains iteratively.

**Current gains** (baseline, untuned for all robots):
`Kp=55.0`, `Ki=1.2`, `Kd=2.8`. These may need to be changed depending on
battery voltage, mechanical build, and centre of mass.

---

## Extra Features
Beyond the mandatory self‑balancing functionality, this project includes:

* **Real‑time encoder speed monitoring** – Both motor encoder frequencies
  are captured via hardware input captures and transmitted in the
  telemetry stream. This data is not required for the position‑PID
  balancing task but provides valuable insight into motor behaviour and
  can be used in future speed‑based control layers.
* **Non‑blocking telemetry with busy‑flag** – The UART transmission
  avoids any waiting, preserving the fixed‑rate control loop.
* **Modular, well‑documented codebase** – The separation into independent
  drivers (motor, sensor, encoder, PID, filter) follows the manual’s
  recommendations and simplifies debugging and extension.

---

## Credits & License
Developed as part of the EE/CE 376L/332L “Microcontrollers and
Interfacing Lab” course at Habib University.  
This repository is for educational purposes only. See `LICENSE` for
details (if present).

