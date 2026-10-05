# Pinout

Board: **STM32 Nucleo-F303K8** (Nucleo-32, Arduino Nano footprint).
The "Board pin" column is the label printed on the Nucleo; the "MCU pin" column is the STM32 pin it connects to.

<p align="center">
<img width="416" height="396" alt="stm32pinout" src="https://github.com/user-attachments/assets/0a39a320-662f-44f8-8130-903e8f8ff417" />
</p>

## Connection table

| Board pin | MCU pin | Function | Connected to |
|---|---|---|---|
| A0 | PA0 | `TIM2_CH1` (encoder A) | FIT0450 encoder, channel A |
| A1 | PA1 | `TIM2_CH2` (encoder B) | FIT0450 encoder, channel B |
| D0 | PA10 | GPIO output (direction) | Motor driver M1 |
| D2 | PA12 | GPIO output (direction) | Motor driver M2 |
| D4 | PB7 | `I2C1_SDA` | MPU6050 SDA, OLED SDA |
| D5 | PB6 | `I2C1_SCL` | MPU6050 SCL, OLED SCL |
| D9 | PA8 | `TIM1_CH1` (PWM) | Motor driver E1 (enable / speed) |
| D10 | PA11 | `TIM1_CH4` (PWM) | Motor driver E2 (enable / speed) |
| 3V3 | - | Power | MPU6050 VCC (TODO: confirm whether the OLED and encoder are also on 3V3) |
| 5V | - | Power in | 5 V buck converter output (see below) |
| GND | - | Ground | Common ground for everything |
| PA2 / PA15 | - | `USART2` TX / RX | ST-LINK virtual COM port (debug prints) |

> Only one encoder (on A0/A1) is wired. Position and velocity come from that wheel, even though both motors have encoders.

## I2C bus

| Device | Address (7-bit) | Notes |
|---|---|---|
| MPU6050 | `0x68` | Default address (AD0 low). Confirm against `DEVICE_ADDRESS` in the firmware. |
| SSD1306 OLED | `0x3C` | 128x64, used for the robot face. |

Both devices share the same two wires. Different addresses mean no conflict, but a full OLED refresh sends about 1 KB over the bus and blocks the CPU, so the face is only redrawn when the state changes (see the firmware notes).

## Motor driver (DFRobot DRI0002, L298N-based)

| Signal | Meaning |
|---|---|
| `M1`, `M2` | Direction (HIGH / LOW) for each motor |
| `E1`, `E2` | Enable pins, driven with PWM for speed |

PWM runs on **TIM1**, not TIM3. TIM3's channels collided with the I2C pins during development, which is why the PWM outputs moved to PA8 and PA11.

## Power

```
DC supply 7.1V ────────────────► motor driver (motor supply)
            │
            └─► 5 V buck converter ─► Nucleo +5V pin ─► on-board regulator ─► 3V3 rail
                                                         (MPU6050, OLED, encoder)
```

- One bench supply powers the whole robot.
- The motor driver takes the raw supply voltage. The Nucleo, IMU, OLED and encoder run from a regulated 5 V buck converter output.
- Back-EMF spikes appear on the supply when the motors reverse quickly, so the buck converter is chosen with input headroom.
- **Remove SB9** on the Nucleo if you run it from the +5V pin without USB connected (Already removed).
- All grounds (supply, driver, Nucleo, encoder, sensors) must be common, otherwise the encoder readings are unreliable.

## Wiring notes

- Soldered connections only for Buck Converter and IMU. Vibrations from movement and cheap headers caused I2C Failure for the sensor (acknowledge failures and arbitration-lost errors).
- Keep the I2C wires short and away from the motor wires.
- If the bus ever locks up, the firmware runs a bus-recovery routine (toggle SCL up to 9 times, send a STOP, re-initialize the peripheral).

## Bill of materials (electronics)

| Part | Qty | Notes |
|---|---|---|
| STM32 Nucleo-F303K8 | 1 | Main controller |
| MPU6050 breakout | 1 | I2C IMU |
| DFRobot FIT0450 geared DC motor with encoder | 2 | 120:1 gearbox |
| DFRobot DRI0002 dual motor driver | 1 | L298N-based |
| 0.96" SSD1306 I2C OLED, 128x64 | 1 | I2C OLED Robot face |
| 5 V buck converter (MPM3610 breakout) | 1 | 6-21 V in, 1.2 A out |
| Perfboard, wire, headers | - | Soldered build |

