# Dynamics and Control of a Luffing Crane (2025-2026 Challenge)

> **Lagrangian-based automated sway-control system for high-precision luffing crane positioning.**

This project details the comprehensive modeling and control system design for a Luffing Crane, focusing on managing load sway in a 2D environment. The objective is to precisely control the x-position of the load ($x_l$) by designing a controller to apply the appropriate input force ($F_A$) to the hoist block.

---

## 🏗️ System Description
The crane system is modeled in two dimensions and consists of several key components:

* **Crane Beam**: A rigid rod of length $L_0$.
* **Jib**: A rigid rod of length $L_1$ attached to the crane beam.
* **Hoist Block**: A point mass ($m_2$) that moves along the jib.
* **Load**: A point mass ($m_3$) suspended by a chain of length $L_2$.



### Generalized Coordinates
The system's dynamics are described by the generalized coordinates:
q = [x, $\theta(t)$]

Where:
* $x(t)$ is the position of the hoist block.
* $\theta(t)$ is the absolute angle of the chain.

---

## 🛠️ Technical Workflow

### 1. Dynamics & Modeling
* **Non-linear Equations of Motion (EoMs)**: Derived using the Lagrange method based on kinetic energy, potential energy, and non-conservative forces.
* **Linearization**: The EoMs were linearized around stable equilibrium positions to analyze small-signal behavior.
* **Transfer Functions**: Developed the plant transfer function $G(s)$ relating input force $F_A$ to load displacement $y(s) = x_l(s)$.
* **Plant TF**: $G(s) = \frac{5.1 \cdot 10^{-3}s^2 + 10s + 250}{5 \cdot 10^5s^4 + 2.5 \cdot 10^5s^3 + 6.4 \cdot 10^5s^2 + 4.9 \cdot 10^4s}$.

### 2. Feedback Control Design
The project progressed through several control iterations to meet performance requirements:

* **Proportional (P) Control**: Evaluated gains $P_1=80$, $P_2=400$, and $P_3=700$ to assess stability and steady-state error.
* **Proportional-Derivative (PD) Control**: Implemented $C(s) = 390s + 360$ to improve transient response.
* **Lead/Lag Filters**: Used to refine the open-loop $L(s) = C(s)G(s)$ characteristics for better stability margins.
* **Notch Filtering**: Designed a complex controller $C_2(s)$ to suppress resonance frequencies, which act as bandwidth-limiting factors.

**Controller Structure:**
$$C_2(s) = K \frac{\frac{\alpha}{\omega_c}s + 1}{\frac{1}{\alpha\omega_c}s + 1} \cdot \left(\frac{\omega_p}{\omega_z}\right)^2 \frac{s^2 + 2\beta_z\omega_zs + \omega_z^2}{s^2 + 2\beta_p\omega_ps + \omega_p^2}$$

### 3. Analysis & Robustness
* **Frequency Domain**: Used Nyquist contours and Bode plots to determine Phase Margin (PM), Gain Margin (GM), and Modulus Margin (MM).
* **Parameter Sensitivity**: Tested robustness against a 90% decrease in the load damping coefficient $d_W$.
* **Disturbance Rejection**: Analyzed the sensitivity function $S(s)$ to evaluate the system's ability to suppress external wind disturbances $d(t)$.

---

## 💻 Software Requirements
The following MATLAB toolboxes are required to run the simulations:
* Simulink
* Control Systems Toolbox
* Symbolic Math Toolbox
* Simscape / Simscape Multibody

---

## 📈 Key Results
* Successfully stabilized the load position with a maximum allowed overshoot of 30%.
* Achieved a crossover frequency of $f_c = 0.06$ Hz using advanced loop-shaping with notch filters.
* Verified that the identity $S + T = 1$ holds for the designed closed-loop system, highlighting the trade-off between tracking performance and noise rejection.
