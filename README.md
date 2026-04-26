# Robot Arm Control — EE 5325 Course Project
**Spring 2026 | University of Texas at Arlington**

A complete modeling, simulation, and control study of a 2-link planar elbow robot arm, progressing from first-principles dynamics derivation through open-loop simulation to advanced closed-loop controllers including adaptive and robust control under uncertainty.

---

## Overview

This project covers the full control engineering lifecycle for a nonlinear robotic manipulator:

- Deriving the equations of motion from the Euler-Lagrange formulation
- Simulating open-loop behavior numerically
- Designing and comparing multiple closed-loop control strategies, from basic PD-gravity compensation through computed-torque, integral action for disturbance rejection, actuator saturation analysis, adaptive control under unknown parameters, and robust control under parametric uncertainty

All simulations were implemented in MATLAB using `ode23` numerical integration.

---

## System Description

The system is a 2-link planar elbow robot arm with the following parameters:

| Parameter | Value |
|-----------|-------|
| Link masses | m₁ = m₂ = 1 kg |
| Link lengths | a₁ = a₂ = 1 m |
| Gravity | g = 9.8 m/s² |

**State vector:** x = [θ₁, θ₂, θ̇₁, θ̇₂]ᵀ

**Desired trajectories (Problems 4–8):**
- q_d1(t) = 0.1 sin(2πt)
- q_d2(t) = 0.1 cos(2πt)

---

## Part 1: Dynamics Modeling

### Problem 1 — Euler-Lagrange Formulation

The nonlinear dynamics of the 2-link manipulator were derived from first principles using the Euler-Lagrange method.

Forward kinematics were used to express the position of each link mass, and their velocities were obtained by differentiation. The kinetic energy T was computed assuming point masses at the link ends, and the potential energy V was derived from gravitational effects. The Lagrangian L = T − V was then formed.

Applying the Euler-Lagrange equations for each joint yielded the standard robot dynamics equation:

```
M(q)q̈ + N(q, q̇) = τ
```

where:
- **M(q)** is the symmetric positive-definite inertia matrix
- **N(q, q̇)** contains Coriolis, centrifugal, and gravitational terms
- **τ** is the joint torque vector

The resulting equations capture the nonlinear coupling between the two joints.

### Problem 2 — State-Space Representation

The nonlinear dynamics were converted into state-space form by defining the state vector x = [θ₁, θ₂, θ̇₁, θ̇₂]ᵀ and expressing the accelerations as:

```
q̈ = M⁻¹(q)(τ − N(q, q̇))
```

This form is suitable for numerical simulation and control design.

---

## Part 2: Open-Loop Simulation

### Problem 3 — Numerical Integration

The open-loop dynamics were simulated in MATLAB using `ode23`. Since the system is nonlinear and coupled, no closed-form analytical solution exists. The solver returns the complete solution x(t) over the simulation interval.

The end-effector position was computed at each time step using forward kinematics:

```
xe = a₁·cos(θ₁) + a₂·cos(θ₁ + θ₂)
ye = a₁·sin(θ₁) + a₂·sin(θ₁ + θ₂)
```

**Outputs:** Joint angles vs time, angular velocities vs time, 3D end-effector trajectory plot.

Without feedback, the arm follows an uncontrolled trajectory driven purely by the applied torques and initial conditions.

**MATLAB file:** `Part_II_Open_Loop_Simulation_Problem_3.m`

---

## Part 3: Closed-Loop Control

### Problem 4 — PD-Gravity Controller

A PD controller with gravity compensation was designed:

```
τ = Kp(qd − q) + Kv(q̇d − q̇) + G(q)
```

where the gravity compensation term G(q) cancels the gravitational load at each joint.

**Gain matrices:**
- Kp = diag(2500, 2500)
- Kv = diag(100, 100)

**Results:** The controller drives the arm from the initial condition to the desired sinusoidal trajectories with small tracking error after a short transient. However, large initial error causes a large initial torque spike, which may exceed actuator limits in practice.

**Limitations:**
- Does not cancel Coriolis or centrifugal terms; tracking may degrade at higher speeds
- Requires accurate knowledge of gravity parameters
- Not robust to disturbances or modeling errors

**MATLAB file:** `Part_III_Closed_Loop_Control_Problem_4.m`

---

### Problem 5 — Computed-Torque Controller

To overcome the limitations of PD-gravity control, a computed-torque controller was designed that explicitly cancels the full nonlinear robot dynamics:

```
τ = M(q)[q̈d + Kv(q̇d − q̇) + Kp(qd − q)] + C(q, q̇)q̇ + G(q)
```

This choice of control law reduces the closed-loop error dynamics to:

```
ë + Kvė + Kpe = 0
```

which is a linear, decoupled, second-order stable system. With positive-definite Kp and Kv, the tracking error converges to zero.

**Results:** Near-zero steady-state tracking error with smooth, repeatable motion. Significantly better tracking than PD-gravity control, though the controller requires accurate knowledge of all robot parameters.

**MATLAB file:** `Part_III_Closed_Loop_Control_Problem_5.m`

---

### Problem 6 — Computed-Torque with Integral Action (Disturbance Rejection)

A constant external disturbance τd = [5; 5] Nm was added to both joints. A standard PD computed-torque controller cannot reject constant disturbances, so integral action was added:

```
τ = M(q)[q̈d + Kv(q̇d − q̇) + Kp(qd − q) + Kiη] + C(q, q̇)q̇ + G(q)
```

where η̇ = e is the integral error state.

**Gain matrices:**
- Kp = diag(2500, 2500)
- Kv = diag(100, 100)
- Ki = diag(300, 300)

The augmented state vector was x = [θ₁, θ₂, θ̇₁, θ̇₂, η₁, η₂]ᵀ.

**Results:** The integral term successfully eliminates steady-state error caused by the constant disturbance. Both joint angles converge to the desired trajectories and tracking errors approach zero. The initial torque spike is larger due to simultaneous disturbance rejection and initial condition correction.

**MATLAB file:** `Part_III_Closed_Loop_Control_Problem_6.m`

---

### Problem 7 — PID Computed-Torque with Actuator Torque Saturation

The same PID computed-torque structure was applied, but with actuator torque limits enforced:

```
−15 ≤ τ₁ ≤ 15 Nm,   −15 ≤ τ₂ ≤ 15 Nm
```

The commanded torque τ_c was saturated element-wise before being applied to the plant:

```
τᵢ = sat(τ_c,i),   i = 1, 2
```

**Gain matrices:**
- Kp = diag(2500, 2500)
- Kv = diag(100, 100)
- Ki = diag(200, 200)

**Results:** The controller performs poorly under these saturation limits. Both joints deviate significantly from the desired trajectories, and tracking errors remain large and oscillatory throughout the simulation. The torques repeatedly saturate at ±15 Nm, preventing the full computed-torque compensation from being realized.

**Key insight:** Torque saturation severely limits the effectiveness of computed-torque control. The controller structure is theoretically sound, but it depends on sufficient actuator authority. When the required torque exceeds the allowable bounds, nonlinear cancellation breaks down and tracking degrades substantially. Actuator sizing is a critical practical constraint in manipulation system design.

**MATLAB file:** `Part_III_Closed_Loop_Problem_7.m`

---

### Problem 8a — Adaptive Control with Unknown Mass Parameters

The link masses m₁ and m₂ were assumed to be unknown. Since computed-torque control requires exact parameter knowledge, an adaptive control law was designed instead.

**Key idea:** Robot dynamics are nonlinear in the states but linear in the unknown physical parameters. This linearity-in-parameters property allows the dynamics to be expressed as:

```
M(q)q̈r + C(q, q̇)q̇r + G(q) = Y(q, q̇, q̇r, q̈r) · [m₁; m₂]
```

where Y is the regressor matrix constructed explicitly in terms of known signals.

A filtered error was introduced:

```
s = ė + Λe,   Λ = diag(15, 15)
```

**Adaptive control law:**

```
τ = Y(q, q̇, q̇r, q̈r)·m̂ − Kd·s
```

**Parameter update law (Lyapunov-based):**

```
ṁ̂ = −Γ·Yᵀ·s,   Γ = diag(8, 8)
```

**Gain matrix:** Kd = diag(120, 120)

**Initial conditions:** m̂₁(0) = m̂₂(0) = 0.5 kg (true values: 1 kg each)

**Results:** Both joint angles converge to the desired trajectories after a short transient. Tracking errors approach zero as the parameter estimates adapt online toward the true mass values. The adaptive mass estimate plot confirms convergence of m̂₁ and m̂₂ close to 1 kg. Large initial control torques are expected since the controller must simultaneously correct the initial condition error and adapt the incorrect mass estimates.

**MATLAB file:** `Part_III_Closed_Control_Problem_8_a.m`

---

### Problem 8b — Robust Computed-Torque Control under Parametric Uncertainty

The controller deliberately used incorrect nominal mass values while the plant used the true values:

| | Value |
|--|--|
| True masses (plant) | m₁ = m₂ = 1 kg |
| Nominal masses (controller) | m̂₁ = m̂₂ = 0.8 kg |

Since the nominal model does not match the true dynamics, a standard computed-torque controller leaves a residual uncertainty. A robust term was added to dominate this mismatch.

A sliding-type error variable was defined:

```
s = ė + Λe,   Λ = diag(20, 20)
```

**Nominal computed-torque part:**

```
τ_nominal = M̂(q)v + Ĉ(q, q̇)q̇ + Ĝ(q),   v = q̈d + Kvė + Kpe
```

**Final controller with robust term:**

```
τ = M̂(q)v + Ĉ(q, q̇)q̇ + Ĝ(q) + Kr·sign(s)
```

**Gain matrices:**
- Kp = diag(2500, 2500)
- Kv = diag(100, 100)
- Kr = diag(25, 25)

The sign-based robust term was selected over a saturation form because it provides stronger compensation and drives the tracking error to zero rather than just bounding it.

**Results:** Both joint angles quickly converge to the desired sinusoidal trajectories despite the 20% mass mismatch. Tracking errors approach zero after a short transient, confirming that the robust term successfully dominates the model uncertainty. The controller satisfies the bounded tracking error requirement and in this simulation also achieves near-zero steady-state error.

**Limitation:** The discontinuous sign-based robust term can produce chattering in practical physical systems. A saturation-based approximation (boundary layer φ = 0.01) was also implemented as an option in the code for smoother operation.

**MATLAB file:** `Part_III_Closed_Loop_Problem_8_b.m`

---

## Controller Comparison Summary

| Controller | Disturbance Rejection | Parameter Uncertainty | Actuator Limits | Tracking Performance |
|---|---|---|---|---|
| PD-Gravity | No | No | Not analyzed | Good (sinusoidal) |
| Computed-Torque | No | No | Not analyzed | Excellent |
| CT + Integral | Yes (constant) | No | Not analyzed | Excellent |
| CT + Saturation | Partial | No | Yes | Poor (degrades severely) |
| Adaptive | N/A | Yes (online) | No | Excellent |
| Robust CT | No | Yes (bounded) | No | Excellent |

---

## Repository Structure

```
├── Part_II_Open_Loop_Simulation_Problem_3.m
├── Part_III_Closed_Loop_Control_Problem_4.m
├── Part_III_Closed_Loop_Control_Problem_5.m
├── Part_III_Closed_Loop_Control_Problem_6.m
├── Part_III_Closed_Loop_Problem_7.m
├── Part_III_Closed_Control_Problem_8_a.m
├── Part_III_Closed_Loop_Problem_8_b.m
└── README.md
```

---

## Key Concepts Covered

- Euler-Lagrange dynamics derivation
- Forward kinematics and state-space modeling
- Inertia matrix, Coriolis/centrifugal terms, gravity vector
- PD-gravity, computed-torque, PID control for nonlinear robotic systems
- Integral action for constant disturbance rejection
- Actuator torque saturation and its practical impact on control authority
- Adaptive control using the linear-in-parameters property and Lyapunov stability theory
- Regressor matrix construction and online parameter estimation
- Robust control using sliding variables and sign/saturation-based robust terms
- Parametric uncertainty and nominal-vs-true model separation

---

## Course

**EE 5325 — Robotics**
The University of Texas at Arlington, Spring 2026
