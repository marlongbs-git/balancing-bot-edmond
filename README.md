# edmond: the self-balancing robot!


Edmond is a two-wheeled inverted-pendulum robot built around an **STM32F303K8 (Nucleo-32)**. He balances using **LQR state feedback** designed in MATLAB/Simulink, with sensor fusion from an MPU6050, a custom 3D-printed chassis, and an OLED face that reacts when he falls.

<p align="center">
  <img width="480" height="456" alt="robot_balancing_updatedgif" src="https://github.com/user-attachments/assets/99f73a7b-7ec8-4353-b704-47a9899092dd" />
</p>

| | |
|---|---|
| **Controller** | Discrete LQR (`dlqr`) on `[x, ẋ, θ, θ̇]` |
| **MCU** | STM32F303K8 (Cortex-M4F, 64 KB flash) |
| **Sensing** | MPU6050 IMU (I2C) + quadrature encoders |
| **Actuation** | 2x DFRobot FIT0450 geared DC motors, L298N-based driver |
| **Extras** | Fall detection, SSD1306 OLED face |
| **Tools** | STM32CubeIDE, MATLAB/Simulink, SolidWorks |

---

## Contents

- [How edmond works](#how-he-works)
- [Repository layout](#repository-layout)
- [Firmware](#firmware)
- [Electronics](#electronics)
- [Control design](#control-design)
- [Mechanical design](#mechanical-design)
- [What I learned](#what-i-learned)
- [Status and roadmap](#status-and-roadmap)
- [Credits](#credits)

---

## How he works

Every control cycle the firmware:

1. Reads the accelerometer and gyro over I2C and fuses them with a **complementary filter** to get the pitch angle `θ` and rate `θ̇`.
2. Reads the wheel encoders for position `x` and velocity `ẋ`.
3. Computes the control effort `u = -K·[x ẋ θ θ̇]ᵀ`.
4. Converts `u` (a force) to a motor duty cycle using the motor's electrical model (torque, back-EMF, supply voltage).
5. Drives the H-bridge, or cuts power and shows the "fallen" face if the tilt exceeds the fall threshold.

## Repository layout

```
├── firmware/       STM32CubeIDE project (.ioc, Core/Src, Core/Inc)
├── electronics/    Pinout, wiring diagram, bill of materials
├── matlab/         Plant model, LQR design, Simulink model
├── mechanical/     SolidWorks parts and assembly, STEP and STL exports
├── docs/           Longer write-ups (sensor fusion, motor model, I2C debugging)
└── media/          Photos, renders, demo GIF
```

## Firmware

C on STM32 HAL. The code is split into small modules:

| File | Responsibility |
|---|---|
| `mpu6050.c/.h` | I2C driver, register reads, calibration, digital and complementary filter |
|`fit0450encoder.c/.h`| Encoder data, wheel velocity, wheel position |
| `robot_state.c/.h` | Combines IMU and encoder data into one `RobotState` |
| `control.c/.h` | LQR gains, force-to-duty conversion, PWM output |
| `ssd1306.c/.h` | Normal and fallen faces, drawn only on state changes |

See [`firmware/`](firmware/) for build instructions.

## Electronics

Pin assignments and wiring live in [`electronics/pinout.md`](electronics/pinout.md).
<p align="center">
<img width="333" height="358" alt="edmond_electronicsbringup" src="https://github.com/user-attachments/assets/4c21a2d2-5324-4838-9c20-1dbf5a6c9c3a" />
</p>


- The MPU6050 and OLED share one I2C bus (different addresses).
- The robot is powered from a single supply through a 5 V buck converter into the Nucleo.
- I2C reliability took real effort: loose connectors caused intermittent failures, so the final build is soldered. The bus-recovery routine is documented in [`docs/i2c-debugging.md`](docs/i2c-debugging.md).

## Control design

The plant is modeled as a cart-pendulum with the wheels as the cart and the chassis as the pendulum body, including its moment of inertia. The model is linearized about the upright position, discretized, and fed to `dlqr`.

<p align="center">
<img width="410" height="208" alt="cart_pend" src="https://github.com/user-attachments/assets/2eb07105-48cd-4d19-afb7-57c038483cd1" />
<img width="410" height="208" alt="LQR" src="https://github.com/user-attachments/assets/7ee3ae93-6ead-4845-ac50-3fe6c707cdd0" />
</p>

- **State:** `[x, ẋ, θ, θ̇]`
- **Weights:** `Q` penalizes angle and angular rate much more than position; `R` penalizes motor effort.
- **Physical parameters** (masses, center of mass, inertia) come from the SolidWorks mass-properties report.

Details and the derivation are in [`matlab/`](matlab/)

## Mechanical design

The chassis is 3D printed in PLA and holds the two motors, the electronics deck, and the OLED.

<p align="center">
<img width="333" height="358" alt="edmond_exploded2" src="https://github.com/user-attachments/assets/6169db7e-8a90-4e7c-b782-918073b05110" />
<img width="333" height="358" alt="front" src="https://github.com/user-attachments/assets/133204fc-85f2-48fb-bcea-943160f73219" /> 
</p>

STEP and STL exports are in [`mechanical/`](mechanical/).

## What I learned

- Bit-level I2C register handling, and why a silent read failure is worse than a loud one.
- Why unit mismatches (degrees vs radians) are the fastest way to saturate a motor.
- How to get real physical parameters out of CAD instead of guessing them.
- Controllers can be limited by actuation, motor deadband, back-EMF spikes, and why power-supply details matter.

## Status and roadmap

- [x] Balances unaided (standing on business!)
- [x] Recovers from small pushes
- [x] Fall detection with OLED face
- [ ] Extended Kalman filter for attitude estimation
- [ ] Battery power

## Credits

- Phil's Lab (STM32 implementation and digital filter tutorials)
- Steve Brunton's Control Bootcamp (control and state-space modeling lectures), *Data Driven Science & Engineering
  Machine Learning, Dynamical Systems, and Control* 
- Beard and McLain, *Small Unmanned Aircraft* (attitude estimation)
- Brian Douglas (Discrete Control Theory)
- SSD1306 Files from [afiskon/stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306) for OLED Drivers and face from [mjyc/table-robot-face](https://github.com/mjyc/tablet-robot-face)

