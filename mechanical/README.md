
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
- [Assembly notes](#assembly-notes)

---

## Files

| Folder / file | What it is |
|---|---|
| `*.SLDPRT`, `*.SLDASM` | Native SolidWorks parts and the full assembly |
| `step/` | STEP exports. Open in any CAD package. |
| `stl/` | STL files for 3D printing |

<!-- TODO: adjust names and folders to match what you actually commit -->

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

## Assembly notes

- Keep the center of mass centered between the wheels, so the robot tips straight and doesn't drift sideways.
- Re-run **Mass Properties** whenever you add, move, or change parts. The `m`, `L` and `I` values in the control model depend on it, and a new gain vector may be needed.
- Check that the encoder and motor wires do not rub against the wheels.
