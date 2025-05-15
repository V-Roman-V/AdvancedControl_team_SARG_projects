# Advanced Control: Motorized Vessel Control with Wind disturbance

## Overview

This project is focused on the **control** of a motorized boat using adaptive-based control under wind disturbance using Backstepping.

<img src="images/Boat types.png" alt="Differential vs. steerable thruster configurations" style="width:50%;">

## Task Definition

Given the initial state $\mathbf{x}_i$ and a desired position $\mathbf{x}_d$, the goal is to design a control law $\mathbf{u} = [u_1, u_2]^T$ such that the boat will reach the desired position with zero velocity, despite wind disturbances.
The boat can finish at any angle. The thruster force can only be applied in the forward direction.

---

**TODO:** add gif of the solution

---

## Updated Wind Fields

To test the control system under varying environmental conditions, we implemented **three distinct wind field models**:

1. **Cosine Wind Field**:
   A spatially varying wind defined by sinusoidal functions, simulating periodic gusts.

   $$
   V(x,y) = A \left(1 + B\cos\left(\frac{2\pi \text{ dist}}{\lambda}\right)\right)
   $$  

   where:
     - $A$, $B$ are `base_speed` and wave `amplitude`
     - $\text{dist}$ is a `distance` aling wind direction
     - $\lambda$ is a `wavelength`.  

2. **Perlin Noise Wind Field**:  
   A procedurally generated turbulent wind using Perlin noise, mimicking natural randomness. 

3. **Constant Directional Wind**:  
   A uniform wind $(V_{wx}, V_{wy})$ with fixed magnitude/direction.  

![alt text](images/Different_wind_types.png)  
*Figure: Visualization of the three wind field types (cosine, Perlin noise, constant) used in simulations.*  


## Boats experience additional **unknown** drag effects from Water

Beyond wind disturbances, the boats are subject to hydrodynamic drag forces that dissipate kinetic energy. These effects are **modeled as unmeasured damping terms** in the dynamics:  

$$
\begin{aligned}
\dot{V}_f &= -D_f \cdot V_f \quad &\text{(Surge damping)}, \\
\dot{V}_s &= -D_s \cdot V_s \quad &\text{(Sway damping)}, \\
\dot{\omega} &= -D_ψ \cdot \omega \quad &\text{(Yaw damping)},
\end{aligned}
$$

where:

- $V_f, V_s$ are surge/sway velocities in the body frame,  
- $\omega$ is the yaw rate,  
- $D_f, D_s, D_ψ$ are unknown damping coefficients.  

Demonstration of the Drag Force:  
![alt text](simulator/gif/Drag_force_without_control.gif)  
![alt text](images/Velocity_decay.png)

## Mathematical Model

### Boat Kinematics

Let the state vector of the boat be represented as:

$$
\mathbf{x} = \begin{bmatrix} x, & y, & \psi, & V_x, & V_y, & \omega \end{bmatrix}^T,
$$

where:

- $x, y$ are the position coordinates,
- $\psi$ is the yaw (heading),
- $V_x, V_y$ are the surge and sway velocities (linear velocities in the body-fixed frame),
- $\omega$ is the yaw rate.

#### Wind Model: Sail Interaction Model

The wind now exerts a **directional force** on the boat's sail, proportional to:
1. The **projected sail area** facing the wind (cosine of relative angle).
2. The **squared speed difference** between wind and boat (Bernoulli principle).

<img src="images/Wind_force.png" alt="Wind force" style="width:40%;">

#### Wind Dynamic:
- **Relative wind velocity** (in boat's body frame):

  $$
  \begin{aligned}
  V_{wx}^b &= \cos(\psi) V_{wx} + \sin(\psi) V_{wy}, \\
  V_{wy}^b &= -\sin(\psi) V_{wx} + \cos(\psi) V_{wy}.
  \end{aligned}
  $$

- **Effective wind angle**:

  $$
  \alpha = \text{atan2}(V_{wy}^b, V_{wx}^b).
  $$

- **Speed difference**:

  $$
  \Delta V = \sqrt{(V_{wx}^b - V_x)^2 + (V_{wy}^b - V_y)^2}.
  $$

#### Sail Force Calculation:
The total wind force in the boat's body frame is:

$$
\begin{aligned}
F_{sail,x} &= \frac{1}{2} \rho C_x A \cdot \Delta V^2 \cos \alpha, \\
F_{sail,y} &= \frac{1}{2} \rho C_y A \cdot \Delta V^2 \sin \alpha,
\end{aligned}
$$

where:
- $C_x$ = Drag coefficient for **surge** (longitudinal force),  
- $C_y$ = Drag coefficient for **sway** (lateral force),  
- $\rho$ = Air density,  
- $A$ = Effective sail area.  

### Dynamics with Wind Disturbance  

The equations of the dynamics of the boat including wind effects have the following matrix form:

$$
\begin{bmatrix}
    \dot{x} \\
    \dot{y} \\
    \dot{\psi} \\
    \dot{V}_x \\
    \dot{V}_y \\
    \dot{\omega}
\end{bmatrix}
= \begin{bmatrix}
0 & 0 & 0 & \cos(\psi) & -\sin(\psi) & 0 \\
0 & 0 & 0 & \sin(\psi) & \cos(\psi) & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & -D_x & 0 & 0 \\
0 & 0 & 0 & 0 & -D_y & 0 \\
0 & 0 & 0 & 0 & 0 & -D_ψ
\end{bmatrix}
\begin{bmatrix}
    x \\
    y \\
    \psi \\
    V_x \\
    V_y \\
    \omega
\end{bmatrix} +
\begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
\frac{1}{m} & 0 & 0 \\
0 & \frac{1}{m} & 0 \\
0 & 0 & \frac{1}{I_z}
\end{bmatrix}
\left(
\begin{bmatrix}
F_x(u) \\
F_y(u) \\
M(u)
\end{bmatrix}
+ 
\underbrace{
\begin{bmatrix}
F_{sail,x} \\
F_{sail,y} \\
0
\end{bmatrix}
}_{\text{Sail contribution}}
\right)
$$

where:

- $m$ is the boat's mass,
- $I_z$ is the moment of inertia about the vertical axis,
- $D_x, D_y, D_\psi$ are the damping coefficients,
- $F_x, F_y$ are the forces due to the thrusters,
- $M$ is the moment generated by the thrusters, and
- $F_{sail,x}, F_{sail,y}$ are the wind forces by the sail.

#### Differential Drive Boat

$$
\begin{aligned}
F_x(u) &= u_1 + u_2\\
F_y(u) &= 0 \\
M(u) &= L(u_1 - u_2)
\end{aligned}
$$

where:

- $u_1$ and $u_2$ are the control inputs for the left and right motors, respectively.
- $L$ is the distance between the motor and the center of the boat.

#### Steerable Drive Boat

$$
\begin{aligned}
F_x(u) &= u_f \cos(u_\phi)\\
F_y(u) &= u_f \sin(u_\phi)\\
M(u) &= L \cdot u_f \sin(u_\phi)
\end{aligned}
$$

where:

- $u_f$ is the control for the motor.
- $u_\phi$ is the control for the steering angle.
- $L$ is the distance between the motor and the center of the boat.

## Control Design

### Previous Energy-Based Control

In the previous project, we developed an energy-based control strategy for motorized boats, using Lyapunov stability theory.

#### **Differential Drive Boat**

The control law for differential drive is derived from an energy function combining position and orientation errors:

$$
E = \frac{1}{2} k_0 (x_e^2 + y_e^2) + \frac{1}{2} k_1 (V_x^2 + V_y^2) + \frac{1}{2} k_2 \psi_e^2,
$$

where:

- $x_e = x - x_d$, $y_e = y - y_d$ are position errors in body frame,
- $\psi_e = \psi - \psi_d$ is the heading error (though $\psi_d$ is arbitrary here),
- $k_0, k_1, k_2$ are positive gains.

Taking the time derivative and substituting dynamics, we obtain the control inputs:

$$
\begin{aligned}
u_1 &= k_0 x_e - k_1 (x_e V_x + y_e V_y - \omega) - k_2 \psi_e, \\
u_2 &= k_0 x_e - k_1 (x_e V_x + y_e V_y + \omega) + k_2 \psi_e.
\end{aligned}
$$

**Implementation Notes:**

1. **Clipping**: Thrusters are unidirectional, so inputs are saturated:

$$
u_1 = \text{clip}(0, u_1, u_{max}), \quad u_2 = \text{clip}(0, u_2, u_{max}).
$$

2. **Turn-around Case**: If both $u_1, u_2 < 0$, the boat must reverse direction. We enforce:

$$
\text{If } u_1 < 0 \text{ and } u_2 < 0, \quad \text{set } u_1 = \frac{1}{2} u_{max}, u_2 = 0 \text{ (or vice versa)}.
$$


#### **Steerable Drive Boat**

For steerable thrusters, the energy function is the same:

$$
E = \frac{1}{2} k_0 (x_e^2 + y_e^2) + \frac{1}{2} k_1 (V_x^2 + V_y^2) + \frac{1}{2} k_2 \psi_e^2.
$$

The resulting control laws:

$$
\begin{aligned}
u_f &= k_0 (x_e^2 + y_e^2) - k_1 (x_e V_x + y_e V_y), \\
u_\phi &= k_2 \psi_e,
\end{aligned}
$$

where $u_f$ is the thrust magnitude and $u_\phi$ the steering angle.

**Implementation Notes:**

1. **Clipping**:

$$
u_f = \text{clip}(0, u_f, u_{f Max}), \quad u_\phi = \text{clip}(-u_{\phi Max}, u_\phi, u_{\phi Max}).
$$

### Old energy-based control with wind:

As we can see the energy-based control that assumes zero wind cannot provide good control to reach zero position.

![alt text](<simulator/gif/simulation_energy_based.gif>)

### Phase plot

![Phase plot for energy based control with wind](images/energy_based_phase_plot.png)

### Adaptive Control

To handle unknown wind disturbances $(V_{wx}, V_{wy})$, we augment the energy-based controller with an adaptation law that adapts and compensates for the wind effects during the work.

#### **1. State Augmentation**

Define the **augmented state vector** to include wind disturbance adaptation:

$$
\mathbf{x_a} = \begin{bmatrix} x, y, \psi, V_x, V_y, \omega, \hat{V_{wx}}, \hat{V_{wy}} \end{bmatrix}^T,
$$

where $\hat{V_{wx}}, \hat{V_{wy}}$ are adaptation parameters of the wind velocities in global frame.

The **main challenges** was to handle the different coordinates systems, as wind given in global coordinates, but boat velocities in local boat frame.

#### **2. Modified Lyapunov Function**  

Introduce a Lyapunov function including adaptation errors:

$$
E_a = E + \frac{1}{2 \gamma_w} \tilde{V_{wx}}^2 + \frac{1}{2 \gamma_w} \tilde{V_{wy}}^2,
$$

where:

- $\gamma_x, \gamma_y > 0$ are adaptation gains.
- $\tilde{V_{wx}} = V_{wx} - \hat{V_{wx}}$ and $\tilde{V_{wy}} = V_{wy} - \hat{V_{wy}}$ are adaptation errors.

#### **3. Adaptation Laws**

Derive adaptation laws by ensuring $\dot{E}_a \leq 0$:

$$
\begin{aligned}
\dot{\hat{V_{wx}}} &= - \gamma_w \left( \hat{V_{wx}} + V_{xGlobal} \right), \\
\dot{\hat{V_{wy}}} &= - \gamma_w \left( \hat{V_{wy}} + V_{yGlobal} \right).
\end{aligned}
$$

where:
- $V_{xGlobal}$ and $V_{yGlobal}$ are boat speed in global coordinates:

$$
\begin{aligned}
V_{xGlobal} &= \cos(\psi) V_x - \sin(\psi) V_y, \\
V_{yGlobal} &= \sin(\psi) V_x + \cos(\psi) V_y.
\end{aligned}
$$

#### **4. Adaptive Control Laws**

**Differential Drive:**

$$
\begin{aligned}
u_1 &= k_0 x_e - k_1 \left( x_e (V_x - \hat{V_{wxLocal}}) + y_e (V_y - \hat{V_{wyLocal}}) - \omega \right) - k_2 \psi_e - k_w \hat{V_{wxLocal}}, \\
u_2 &= k_0 x_e - k_1 \left( x_e (V_x - \hat{V_{wxLocal}}) + y_e (V_y - \hat{V_{wyLocal}}) + \omega \right) + k_2 \psi_e - k_w \hat{V_{wxLocal}}.
\end{aligned}
$$

**Steerable Drive:**

$$
\begin{aligned}
u_f &= k_0 (x_e^2 + y_e^2) - k_1 \left( x_e (V_x - \hat{V_{wxLocal}}) + y_e (V_y - \hat{V_{wyLocal}}) \right) - k_w \hat{V_{wxLocal}}, \\
u_\phi &= k_2 \psi_e.
\end{aligned}
$$

**where:**
- $\hat{V_{wxLocal}}$ and $\hat{V_{wyLocal}}$ are adaptation parameters for wind velocities in the boat frame:

$$
\begin{aligned}
\hat{V_{wxLocal}} &=  \cos(\psi) \dot{\hat{V_{wx}}} + \sin(\psi) \dot{\hat{V_{wy}}}, \\
\hat{V_{wyLocal}} &= -\sin(\psi) \dot{\hat{V_{wx}}} + \cos(\psi) \dot{\hat{V_{wy}}}.
\end{aligned}
$$

## Repository Structure

### `control.py`

This file contains the `Controller` classes for **Differential** and **Steering** boats, which computes the control inputs based on the boat state, desired state.

### `boat.py`

This file contains two boat classes `DifferentialThrustBoat` and `SteerableThrustBoat` that models boat dynamics, given the current control inputs.

### `visualization.py`

This module provides functions to visualize the boat trajectory and desired position.
The visualization has three modes: 'gif', 'realtime', 'final'. The 'gif' mode generates a GIF of the boat motion, 'realtime' shows the simulation in real-time, and 'final' displays the final trajectory after the simulation.

### `main.py`

The `main.py` file is the entry point for the simulation. It initializes the boat, sets up the controller, and runs the simulation loop. At each time step, the control inputs are computed, and the boat state is updated. The trajectory is then visualized.

## How to Run

To run the simulation, simply execute the `main.py` file:

```bash
python main.py
```

## Simulation Setup

### Initial Conditions

- **Initial state ($\mathbf{x}_i$)**:

$$
\mathbf{x_a} = \begin{bmatrix} x, y, \psi, V_x, V_y, \omega, \hat{V_{wx}}, \hat{V_{wy}} \end{bmatrix}^T,
$$

- We have randomly generated 20 differential/steering boats. Each board has zero initial speeds. The initial position and yaw (heading) are arbitrary.

- **Wind disturbance adaptation**: The initial adaptation parameters for wind velocities $\hat{V_{wx}}$ and $\hat{V_{wy}}$ are initialized to zero.

- **The desired reference trajectory** for each boat is defined by a target position $x_d, y_d$ for convinients every target position were set to origin of the coordinate system.

- **Simulation duration**: The simulation runs for a total time of $T = 300$ seconds, with a time step $\Delta t = 1$ second.

## Results

## Adaptive control

### First iteration of adding wind adaptation

![alt text](simulator/gif/simulation_funny_jumps.gif)
![alt text](simulator/gif/simulation_water_slide.gif)

### Successfull implementation

We can see that adaptive control can successfully adapt to the wind:

![trajectories](simulator/gif/simulation_adaptive_control.gif)

### Phase plot

![Phase plot](images/phase_plot_adaptive_control.png)

### Wind Adaptation

![Wind adaptation](images/wind_estimates.png)

### Control by time

#### Control for differential boats

![Control by time](images/control_plot_differential.png)

#### Control for steerable boats

![Control by time](images/control_plot_steerable.png)

## Control of boats inside Wind vector field

**Wind Field:**  
![Wind field](images/Find_field.png)

![Wind simulation](simulator/gif/simulation_with_different_wind.gif)