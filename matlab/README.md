# MATLAB / Simulink: model and LQR design

Everything needed to go from the robot's CAD mass properties to the gain vector `Kdlqr` that runs on the STM32.

**Pipeline:** SolidWorks mass properties → linearized cart-pendulum model `(A, B)` → controllability and observability checks → discretize at `Ts = 0.04 s` → discrete LQR (`dlqr`) → closed-loop pole check → Simulink simulation → paste `Kdlqr` into the firmware.

## Contents

- [Files](#files)
- [How to run](#how-to-run)
- [The model](#the-model)
- [Parameters and where they come from](#parameters-and-where-they-come-from)
- [Getting the inertia out of SolidWorks](#getting-the-inertia-out-of-solidworks)
- [LQR design](#lqr-design)
- [Results](#results)
- [Simulink model](#simulink-model)
- [Using the gains in the firmware](#using-the-gains-in-the-firmware)
- [Assumptions and limitations](#assumptions-and-limitations)
- [References](#references)

---

## Files

| File | What it is |
|---|---|
| `implementation.mlx` | **The design script for this robot.** Parameters, `A` and `B`, controllability and observability, `lqr`, `c2d`, `dlqr`, discrete pole plot. Produces `Kdlqr`. |
| `self_balancing_discreteimplementation.slx` | Simulink model of the discrete closed loop. Reads its variables from the workspace, so run `implementation.mlx` first. |
| `documentation.mlx` | Study notes following Steve Brunton's cart-pendulum example: pole placement, LQR, Kalman filter, LQG, S-Function nonlinear simulation. It uses Brunton's textbook values (`m = 1`, `M = 5`, `L = 2`), **not** this robot's parameters. |

**Requirements:** MATLAB with Control System Toolbox (`lqr`, `dlqr`, `c2d`, `ctrb`, `obsv`) and Simulink.

## How to run

1. Open `implementation.mlx` and click **Run**. It defines every variable the Simulink model needs: `F`, `G`, `Kdlqr`, `x0`, `Ts`, `Cd_simulink`, `Dd_simulink`.
2. Open `self_balancing_discreteimplementation.slx` and run it. The Scope shows the four states recovering from the initial tilt `x0`.
3. To change the starting tilt, edit `x0 = [0; 0; deg2rad(10); 0]` in the script and run it again.

## The model

The robot is a cart-pendulum. The **wheels are the cart** (mass `M`), and the **chassis, motors, electronics and head are the pendulum body** (mass `m`). The body has its own center of mass and moment of inertia, so it is not an ideal point mass on a massless rod.

State and input:

```
x = [ x   ẋ   θ   θ̇ ]ᵀ        u = force on the cart (N)
x   cart (wheel) position       θ   body angle from upright (rad)
```

Linearized about the upright position (θ = 0), with `p = I(M + m) + M m L²`:

```
      ⎡ 0    1                   0                 0 ⎤        ⎡ 0           ⎤
A  =  ⎢ 0  -(I + mL²) d / p    m² g L² / p         0 ⎥   B =  ⎢ (I + mL²)/p ⎥
      ⎢ 0    0                   0                 1 ⎥        ⎢ 0           ⎥
      ⎣ 0  -(m L d) / p        m g L (M + m) / p    0 ⎦        ⎣ m L / p     ⎦
```

Sanity check: setting `I = 0` collapses this to Brunton's point-mass cart-pendulum matrices. Open-loop, the model has exactly one unstable pole (the falling pendulum), at about **+13.9 rad/s**.

## Parameters and where they come from

| Symbol | Value | Meaning | Source |
|---|---|---|---|
| `m` | 0.29135 kg | Pendulum body mass | SolidWorks mass of `robot_assemblyV4` (291.35 g) |
| `M` | 0.040 kg | Wheel (cart) mass | Weighed |
| `L` | 0.06047 m | Distance from the wheel axle to the body's center of mass | √(Y² + Z²) of the COM in `Coordinate System1` (60.43 mm, −2.22 mm) |
| `I` | 6.04748e-4 kg·m² | Body moment of inertia **about its own center of mass**, about the axle direction | SolidWorks `Lxx` at the COM = 604,748.13 g·mm² × 1e-9 |
| `d` | 1 | Cart damping | Estimated, a tuning value |
| `g` | 9.81 m/s² | Gravity | |
| `Ts` | 0.04 s | Control loop period | Firmware loop (must match, see below) |

## Getting the inertia out of SolidWorks

The inertia is computed in CAD, not guessed.

1. Put a **coordinate system at the wheel axle**, with its **X axis along the axle**. The robot tips about this X axis.
2. Open **Evaluate → Mass Properties** and set *Report coordinate values relative to* to that coordinate system.
3. Read three things:
   - **Mass** → `m`
   - **Center of mass** (X, Y, Z). X is along the axle (it is about half the track width, because the origin sits on one wheel), so it does not affect the lever arm. The lever arm is `L = √(Y² + Z²)`.
   - **Moments of inertia taken at the center of mass and aligned with the output coordinate system** → `Lxx`, the moment about the axle direction.
4. Convert units: g·mm² × 1e-9 = kg·m².

**Why `Lxx` at the center of mass, and not `Ixx` at the axle?** The model contains `(I + m L²)`. That term is the parallel-axis theorem (the inertia about the axle). If `I` is already the axle value, the `m L²` is counted twice. Check: `6.04748e-4 + 0.29135 × 0.06047² = 1.670e-3 kg·m²`, which matches SolidWorks' `Ixx` at the axle (1,669,992 g·mm²).

## LQR design

Cost: minimize `Σ ( xᵀ Q x + R u² )`.

```matlab
Q = diag([50 1 100 100]);   % penalties on x, xdot, theta, thetadot
R = 100;                    % penalty on control effort
```

- `Q` punishes state error. The large angle and angular-rate weights make tilt the priority. Position is allowed to drift more.
- `R` punishes control effort. A larger `R` gives a gentler, slower controller and a smaller one a more aggressive controller. Only the ratio of `Q` to `R` matters.

Steps in the script:

1. `ss(A, B, C, D)` with `C = eye(4)` (all four states are measured: encoder for `x` and `ẋ`, filtered IMU for `θ` and `θ̇`).
2. `ctrb` and `obsv` checks: both rank 4.
3. `c2d(sysc, Ts, 'zoh')` with `Ts = 0.04`.
4. Re-check controllability and observability on `(F, G)`.
5. `[Kdlqr, Ks, Kp] = dlqr(F, G, Q, R)`, applied directly to the **discrete** system. (`lqr` on the continuous model is also computed, only for comparison.)
6. Verify stability: all eigenvalues of `F − G·Kdlqr` must have magnitude < 1.

## Results

Closed-loop discrete poles, all inside the unit circle:

| Pole | Magnitude |
|---|---|
| 0.0810 | 0.0810 |
| 0.9714 | 0.9714 |
| 0.8970 ± 0.0465i | 0.8982 |

Gain vector (`u = −Kdlqr · [x ẋ θ θ̇]ᵀ`, SI units, angle in radians):

```
Kdlqr ≈ [ -0.2045   -1.4142    5.5034    0.4909 ]
```

<!-- TODO: add images, for example:
![Discrete poles on the unit circle](../media/matlab_poles.png)
![Simulink scope, 10 degree start](../media/simulink_scope_10deg.png)
-->

The Simulink runs start tilted at 10°, 20° and 30° with all states returning to zero.

## Simulink model

`self_balancing_discreteimplementation.slx` is a minimal discrete closed loop:

- **Discrete State Space** block: `A = F`, `B = G`, `C = Cd_simulink` (identity), `D = Dd_simulink`, initial condition `x0`, sample time `Ts`.
- **Gain** block (`−Kdlqr`, matrix multiply): state feedback `u = −K x` fed back into the plant.
- **Scope** and **To Workspace**: for viewing and exporting the state trajectories.

It simulates the **linear** model, so it answers "does the design stabilize the linearized plant?", not "will the real robot behave identically?".

## Using the gains in the firmware

1. Run `implementation.mlx` and read `Kdlqr` (add `disp(Kdlqr)` to print it).
2. Paste the four numbers into `setControlGains(...)` in `firmware/Core/Src/control.c`.
3. Keep the same state order `[x, ẋ, θ, θ̇]`, SI units, and angles in **radians**. The firmware stores the pitch in degrees and converts before applying `K`.
4. `Ts = 0.04` must match the actual control loop period, because the discrete gains depend on it. If the loop runs at a different rate, change `Ts` and re-run the script.
5. Convert force to motor duty with the motor model (see the firmware README).

<!-- TODO: state the sign convention (which tilt direction is positive θ) so the firmware and model are explicitly consistent. -->

## Assumptions and limitations

- The model is **linearized about upright**, valid for small angles.
- The only input is a force on the cart. The motor, driver and deadband are handled afterward by the force-to-duty conversion, not inside the LQR model.
- Damping `d` is an estimate, not a measured value.
- Wheel inertia is represented only through the cart mass `M`.
- No sensor noise or state estimator in the loop. The measured states come from the complementary filter and encoder directly.
- Tuning of `Q` and `R` was done by experiment on the robot, not by an optimization criterion.

## References

- Steven L. Brunton, *Control Bootcamp* lectures and *Data-Driven Science and Engineering* (cart-pendulum, LQR, LQG)
- University of Michigan CTMS, *Inverted Pendulum: System Modeling* (the form with body inertia `I`)
- MathWorks documentation: `lqr`, `dlqr`, `c2d`, `ctrb`, `obsv`
