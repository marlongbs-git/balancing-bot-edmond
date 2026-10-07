# I2C Debugging and Reliable Sensor Reads

Edmond's MPU6050 (IMU) and SSD1306 (OLED face) share one I2C bus on an STM32F303K8 Nucleo. Getting that bus reliable took longer than most of the control work, and the problems were instructive. This page covers:

- what the error codes were and what each one told us,
- the fix that keeps the robot running on the **last good reading** when a transfer fails,
- why putting the display on the same bus as the sensor slowed the control loop, and what to do about it,
- a troubleshooting guide and reference links for I2C on STM32 Nucleo boards.

## Contents

- [Setup](#setup)
- [How the problem showed up](#how-the-problem-showed-up)
- [Reading the error codes](#reading-the-error-codes)
- [What the codes told us about Edmond](#what-the-codes-told-us-about-edmond)
- [The fix: check, retry, recover, keep the last good value](#the-fix-check-retry-recover-keep-the-last-good-value)
- [Sensor and OLED on one bus](#sensor-and-oled-on-one-bus)
- [Lessons learned](#lessons-learned)
- [References](#references)

---

## Setup

| Item | Value |
|---|---|
| MCU | STM32F303K8 (Nucleo-32) |
| Peripheral | `I2C1`, pins `PB6` (SCL) and `PB7` (SDA) |
| Devices | MPU6050 at `0x68`, SSD1306 OLED at `0x3C` |
| HAL calls | Blocking: `HAL_I2C_Mem_Read`, `HAL_I2C_Master_Transmit` |

Pins and wiring are in [`../electronics/pinout.md`](../electronics/pinout.md).

My I2C BUS runs at 100kHz.

## How the problem showed up

The sensor loop read the IMU every cycle, and now and then a read would fail. Sometimes the angle froze, sometimes the robot jerked, sometimes the loop stalled altogether. In the early bring-up the MPU6050 wasn't found at all, and later the failures were intermittent. They were hard to chase because they were intermittent, so it was worth printing the error code instead of just checking for success or failure.


## Reading the error codes

After any failed HAL I2C call, `HAL_I2C_GetError(&hi2c1)` returns a bitmask. The bits are defined in [`stm32f3xx_hal_i2c.h`](https://github.com/STMicroelectronics/stm32f3xx_hal_driver/blob/master/Inc/stm32f3xx_hal_i2c.h):

| Bit | Name | Value | Meaning in plain terms |
|---|---|---|---|
| 0 | `HAL_I2C_ERROR_BERR` | `0x01` | **Bus error.** A START or STOP showed up at an illegal moment, usually from noise or a glitch. |
| 1 | `HAL_I2C_ERROR_ARLO` | `0x02` | **Arbitration lost.** The MCU put a 1 on SDA but read back a 0, so something else was driving the line low. |
| 2 | `HAL_I2C_ERROR_AF` | `0x04` | **Acknowledge failure.** Nobody pulled SDA low to ACK. The address or data byte went unanswered. |
| 3 | `HAL_I2C_ERROR_OVR` | `0x08` | **Overrun/underrun.** Mostly relevant in slave mode. |
| 4 | `HAL_I2C_ERROR_DMA` | `0x10` | DMA transfer error. |
| 5 | `HAL_I2C_ERROR_TIMEOUT` | `0x20` | **Timeout.** The transfer did not finish in the time given. |

Bits can combine, so decode the hex by splitting it into its bits:

```
0x22 = 0x20 + 0x02  =  TIMEOUT + ARLO
0x04                =  AF
```

Print it as hex in your error handler. It costs one line and saves hours:

```c
HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS << 1,
                                            REG_DATA_GYROX, I2C_MEMADD_SIZE_8BIT,
                                            buf, 2, I2C_TIMEOUT_MS);
if (status != HAL_OK) {
    uint32_t err = HAL_I2C_GetError(&hi2c1);
    sprintf(msg, "I2C FAILED status=%d error=0x%lX\r\n", status, err);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}
```

## What the codes told us about Edmond

Two codes appeared in the debugger:

| Code | Decoded | What it means |
|---|---|---|
| `0x4` | `AF` | The bus itself worked, and the MCU sent the address, but no device answered. The sensor was either unpowered, disconnected, or not listening at that moment. |
| `0x22` | `ARLO` + `TIMEOUT` | The MCU lost control of SDA, then the transfer never finished. Something was holding or disturbing the line mid-transaction. |

Taken together: **the I2C peripheral and the firmware configuration were fine.** The two codes point to an *electrical* problem on the wires: connections that broke momentarily. A connector that opens for a moment mid-byte leaves the sensor holding SDA low part way through a transfer. The MCU then reads back the wrong level (`ARLO`) and waits for a transfer that cannot complete (`TIMEOUT`). When the connector is open at the address phase, nobody answers (`AF`).

That matched what we saw physically. The failures were intermittent; they changed when the wires moved, and they went away after **the loose connectors were replaced with soldered joints**.

This is why the code was worth decoding:

| If you see | Suspect first |
|---|---|
| `AF` (`0x4`) on every try | Wrong address, wrong wiring, no power, or a dead device |
| `AF` now and then | Loose or intermittent connection |
| `ARLO`, `BERR` | Noise, a loose wire, a short, or a device stuck holding SDA |
| `TIMEOUT` alone | Bus stuck busy, clock stretching, or a timeout that is too short |
| Nothing works, even after power-up | I2C timing register or pin setup (see below) |

## The fix: check, retry, recover, keep the last good value

The original code issued the reads and used the result without checking. A failed read left a stale or garbage buffer, which turned into a nonsense angle. The fix has four parts:

1. **Check every transaction.** Use a finite timeout (`I2C_TIMEOUT_MS`), never `HAL_MAX_DELAY`. A blocking call with an infinite timeout can hang the whole control loop on a stuck bus.
2. **Report the cause.** Print the status and hex error code, as above.
3. **Recover the bus** if failures keep happening. After more than 5 failures in a row, run `I2C_BusRecovery()`.
4. **Return the last good reading** on a failure, so the filter and controller keep getting a sensible number instead of garbage.

```c
float mpu6050_getGyroRate(float GxOffsetError, float GyOffsetError, float GzOffsetError)
{
    uint8_t datagy_x[2];
    static float    lastGoodGyroLSB     = 0.0f;   // remembered between calls
    static uint16_t consecutiveFailures = 0;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1,
                                  REG_DATA_GYROX, 1, datagy_x, 2, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        uint32_t errorCode = HAL_I2C_GetError(&hi2c1);
        sprintf(printstringArcTan, "I2C FAILED status=%d error=0x%lX \r\n", status, errorCode);
        HAL_UART_Transmit(&huart2, (uint8_t*)printstringArcTan, strlen(printstringArcTan), 100);

        if (++consecutiveFailures > 5) {
            I2C_BusRecovery();
            consecutiveFailures = 0;
        }
        return lastGoodGyroLSB;                   // keep the last good reading
    }
    consecutiveFailures = 0;

    // ... convert, apply offset, low-pass filter ...
    lastGoodGyroLSB = x_gyroLSBFiltered;
    return lastGoodGyroLSB;
}
```

The accelerometer function follows the same pattern with its own `lastGoodPitchFiltered`. Each function keeps its own `static` last-good value and its own failure counter.

### What bus recovery does

If a device is left holding SDA low (a transfer was cut off mid-byte), it stays that way until it sees enough clock pulses to finish the byte it thinks it is sending. `I2C_BusRecovery()` does the standard rescue:

1. Disable the I2C peripheral and switch SCL and SDA to plain GPIO.
2. Toggle SCL up to 9 times until SDA is released.
3. Generate a STOP condition.
4. Reconfigure the pins and re-initialize the peripheral.


## Sensor and OLED on one bus

Adding the OLED face brought a second problem. It worked, but the sensor loop got slower and the balancing worse.

### The two kinds of traffic are very different

| | MPU6050 read | OLED full-screen refresh |
|---|---|---|
| Bytes per transaction | 2 to 6 data bytes | 1024 data bytes (128 × 64 ÷ 8) |
| Bytes on the wire (approx.) | ~5 to 9 | ~1030 |
| Time at 400 kHz | about 0.2 ms | about 23 ms |
| Time at 100 kHz | about 0.8 ms | about 93 ms |
| How often | Every control cycle | Only when the face changes |

Approximate: each byte takes 9 clock cycles (8 bits plus ACK), and the figures ignore start/stop overhead and clock stretching. Your measured times depend on the bus speed you set.

A sensor read is a quick question. A full frame refresh is about **100 times longer**, and it uses the same two wires.

### Why that hurts the control loop

1. **One bus, one conversation at a time.** I2C allows a single transaction at a time. While the framebuffer is going out, the IMU cannot be read.
2. **The HAL calls are blocking.** `HAL_I2C_Master_Transmit` does not return until the whole frame has been sent, so the CPU waits too. The control loop effectively pauses for the length of the refresh.
3. **The loop period jumps.** A normal cycle takes a few milliseconds, and a cycle with a redraw takes tens of milliseconds longer. The complementary filter integrates the gyro using `dt`, so a long cycle both delays the correction and makes that step's gyro integration larger. The balance controller sees an old angle at the moment it matters.
4. **Long transactions meet bus faults.** A long transfer is more exposed to noise than a short one, so the OLED traffic also raised the number of failed reads in a loose-wired build.

### What helps

| Approach | Idea | Cost |
|---|---|---|
| **Redraw only on change** (used in Edmond) | Draw the face when the fallen/normal state flips, not every cycle | Almost free, a few lines of edge detection |
| **Rate-limit** | Redraw at most every few hundred ms | Easy |
| **Partial updates** | Send only the changed page or region instead of all 1024 bytes | More code |
| **DMA or interrupt-driven transfer** | The CPU keeps running while the hardware sends the frame | Moderate, needs careful sequencing so sensor reads wait their turn |
| **A separate I2C bus** for the display | No sharing at all | Needs a free I2C peripheral and pins |
| **Faster bus clock** | Shorter transfers | Check that both devices support the speed |

Edmond's implementation:

```c
static uint8_t lastFaceState = 0xFF;          // forces the first draw

void updateFaceDisplay(uint8_t isFallen) {
    if (isFallen != lastFaceState) {          // draw only when the state changes
        if (isFallen) ssd1306_displayFallenFace();
        else          ssd1306_displayNormalFace();
        lastFaceState = isFallen;
    }
}
```

`updateFaceDisplay()` is called once per loop, outside the tipped branch, so the face also returns to normal after the robot is picked up.

## Lessons learned

- **Never trust a blocking call with `HAL_MAX_DELAY`.** One stuck bus can hang the whole system. Use a timeout and handle the failure.
- **Check every I2C transaction, not just the first one.** An unchecked read hands garbage to the rest of the code.
- **Print the error code in hex.** It turns "I2C is broken" into a specific cause.
- **Intermittent errors are usually electrical.** Solder it, shorten the wires, keep them away from motor wires.
- **Keep the last good value, but plan for a long outage.** It covers glitches, not dead sensors.
- **A shared bus is a shared budget.** A 1 KB display refresh costs about a hundred sensor reads. Update the display only when it changes.
- **Pin and peripheral choices matter early.** Check the pin alternate functions before assigning timers and I2C on the same pins.

## References

- STM32F3 HAL driver source, I2C error definitions: [`stm32f3xx_hal_i2c.h`](https://github.com/STMicroelectronics/stm32f3xx_hal_driver/blob/master/Inc/stm32f3xx_hal_i2c.h) (and the matching `Src/stm32f3xx_hal_i2c.c` in the same repository)
- STM32F303 reference manual (RM0316) and Nucleo-32 user manual (UM1956), from [st.com](https://www.st.com): I2C peripheral and board details
