# Mechanical design

The chassis is designed in SolidWorks and 3D printed in PLA. It holds the two geared motors, the electronics deck (Nucleo, MPU6050, perfboard), and the OLED face. The assembly is `robot_assemblyV4`.

<p align = "center" >
<img width="33%"  alt="isometric" src="https://github.com/user-attachments/assets/b415dcfc-d7a4-4f4b-94d3-b0750596dcb9" />
<img width="33%"  alt="front" src="https://github.com/user-attachments/assets/8f0e75c7-f476-4da7-b65d-713b5f0b9093" />
<img width="33%"  alt="edmond_exploded2" src="https://github.com/user-attachments/assets/29931ba6-6ea5-453c-ac21-7c40e3c0b9ef" />
</p>

## Contents

- [Files](#files)
- [Design overview](#design-overview)
- [Fasteners](#fasteners)
- [Mass properties](#mass-properties)
- [Accelerometer calibration jig](#accelerometer-calibration-jig)
- [Assembly notes](#assembly-notes)

---

## Files

| Folder / file | What it is |
|---|---|
| `*.SLDPRT`, `*.SLDASM` | Native SolidWorks parts and the full assembly |
| `stl/` | STL files for 3D printing |

## Design overview

- **Rigid body.** The legs tilt together with the head. The whole chassis is the pendulum body in the control model, and only the wheels act as the cart.
- **Motors in the legs.** The two DFRobot FIT0450 geared motors sit low, one per leg, with the wheels on their output shafts.
- **Electronics deck on top.** The Nucleo and the MPU6050 sit on a perfboard, and the OLED is mounted at the front so the face is visible.
- **Printed in PLA.** All structural parts are 3D printed.

## Fasteners

Two metric sizes are used throughout:

| Size | Typical use |
|---|---|
| **M3** | Structural joints, motor mounting, wheel and leg hardware |
| **M2** | Small electronics mounts: perfboard, OLED, sensor board |
| **M3 Heat-Set Inserts** | Leg and Head Interfacing

Using only two sizes keeps the build simple: a single hex key and screwdriver set, and no mixing of thread standards.

## Mass properties

Taken from SolidWorks (**Evaluate → Mass Properties**) relative to a coordinate system at the wheel axle, with its **X axis along the axle**, which is the axis the robot tips about.

| Property | Value |
|---|---|
| Mass (pendulum body, without wheels) | 291.35 g |
| Wheel mass (both) | 40 g |
| Center of mass (X, Y, Z) | −65.85, 60.43, −2.22 mm |
| Moment of inertia about the COM, along the axle (`Lxx`) | 604,748 g·mm² |

These values feed directly into the control model. See [`../matlab/README.md`](../matlab/README.md) for how `L` and `I` are derived from them.

## Accelerometer calibration jig

A small 3D-printed jig for calibrating the MPU6050 accelerometer. An accelerometer at rest should read exactly 1 g on whichever axis points along gravity and 0 on the other two, but real boards are off by a small amount per axis. The jig holds the sensor in repeatable, known orientations so the offsets can be measured instead of guessed.

<p align = "center" >

<img width="501" height="613" alt="image" src="https://github.com/user-attachments/assets/a0a0fdf2-2906-4e33-9bda-41042bb156d7" />
  
</p>

### Why a jig

Hand-holding the board against a table gives slightly different angles every time, and a tilt of even a degree or two shows up as an error in the offset. A printed jig with flat, square faces makes each calibration pose repeatable. It also pairs with the firmware routine, which asks for one pose per axis.

### How it is used

The firmware routine `getAccelerometerOffsets()` walks through the three axes over the serial port (USART2). For each axis:

1. It prints a prompt telling you which pose to put the board in. The X axis prompt is *"Lay Flat on Side"* and the Y axis prompt is *"Lay Flat on Back"*, followed by the Z axis pose.
2. A 5-second countdown gives you time to place the jig and let it settle.
3. It reads the accelerometer 10 times at 100 ms intervals and averages the samples.
4. It stores `offset = 1 − average` (in g) for that axis.

The three offsets are printed at the end, and they are added to the raw readings in the pitch calculation.

| Step | Pose | Axis calibrated |
|---|---|---|
| 1 | Jig on its side | X |
| 2 | Jig on its back | Y |
| 3 | Jig flat | Z |

### Axis convention

Using the jig showed that the sensor's X and Y axes are swapped relative to the robot's frame. Pitch is therefore computed with

```c
anglePitch = atan2(y_accel, sqrt(x_accel*x_accel + z_accel*z_accel)) * (180 / M_PI);
```

and the gyro rate about X is used with it. The jig is what made this visible: with the board in a known pose, the axis that read 1 g was not the one expected.

### Is it needed again?

No. The calibration only needs to be done once per sensor. After running it, the offsets were hard-coded in the firmware, so the jig is not needed to run the robot. Re-run it if the MPU6050 is replaced or re-soldered at a different angle.

### Files and print notes

- STEP and STL: `calibration_jig.step`, `calibration_jig.stl`

## Assembly notes

- Keep the center of mass centered between the wheels, so the robot tips straight and doesn't drift sideways.
- Re-run **Mass Properties** whenever you add, move, or change parts. The `m`, `L` and `I` values in the control model depend on it, and a new gain vector may be needed.
- Check that the encoder and motor wires do not rub against the wheels.

