Read [](file:///c%3A/Users/makannan/Downloads/c%2B%2B/project2_pid_controller/include/vehicle_plant.hpp)

Read [](file:///c%3A/Users/makannan/Downloads/c%2B%2B/project2_pid_controller/include/pid_controller.hpp#1-1), lines 1 to 80

Read [](file:///c%3A/Users/makannan/Downloads/c%2B%2B/project2_pid_controller/src/main.cpp#1-1), lines 1 to 120

Read [](file:///c%3A/Users/makannan/Downloads/c%2B%2B/project2_pid_controller/src/main.cpp#120-120), lines 120 to 194

Read [](file:///c%3A/Users/makannan/Downloads/c%2B%2B/project2_pid_controller/visualize.py#40-40), lines 40 to 103

Here is a complete explanation:

---

## What Project 2 Does

It simulates **longitudinal (forward/back) speed control of a car** using a PID controller — the same problem a real cruise-control or ACC system solves.

### The two components

**vehicle_plant.hpp — the simulated car (the "plant")**

The car is modelled as a point mass obeying Newton's second law:

$$m \cdot \frac{dv}{dt} = F_{\text{drive}}(\text{cmd}) - F_{\text{drag}}(v)$$

- **Drive force** scales linearly with the control command (`+1` = full throttle, `-1` = full brake)
- **Drag force** is quadratic: $F_{\text{drag}} = c_d \cdot v^2$ (air resistance grows with speed squared)
- The velocity update uses **RK4** from Project 1 for accuracy
- Terminal velocity at full throttle ≈ $\sqrt{3000 / 0.4}$ ≈ 310 km/h

**pid_controller.hpp — the controller**

A discrete PID with production-grade features:
- **P term** — proportional to current error (fast response)
- **I term** — accumulates error over time (eliminates steady-state offset)
- **D term** — reacts to rate of error change (dampens overshoot)
- **Anti-windup** — clamps the integral so it can't accumulate unboundedly when the actuator is saturated
- **Derivative low-pass filter** (τ = 50 ms) — prevents the D term from amplifying sensor noise

The control loop runs at **100 Hz** (`dt = 0.01 s`) and the target is **100 km/h**.

---

## Figure 1 — P vs PI vs PID Comparison (3 panels, same target)

Each panel plots vehicle speed (km/h) vs time for one controller variant, with the 100 km/h target (red dashed) and ±2% settling band (grey dotted):

### Left — P-only (`Kp=0.3`)
- The controller commands throttle proportional to how far the speed is from target
- As speed rises, error shrinks → throttle drops → the car settles **below** the target
- At equilibrium, drag exactly cancels the (reduced) drive force, leaving a permanent gap
- **This is steady-state error** — a fundamental limitation of P-only control. The controller can only push hard if there's still a large error, so it always "gives up" before reaching the setpoint

### Middle — PI (`Kp=0.3`, `Ki=0.05`)
- The integral term **accumulates the error over time** and keeps increasing the command until the error is exactly zero
- The car reaches and holds 100 km/h — steady-state error is eliminated
- Response is slower than PID because there's no derivative braking

### Right — PID (`Kp=0.4`, `Ki=0.06`, `Kd=0.5`, τ=50 ms)
- The D term detects the **rate at which error is shrinking** and backs off the throttle early, preventing overshoot
- Faster rise time than PI, smoother approach to setpoint, no overshoot
- The low-pass filter on D prevents amplifying noise that would cause throttle chattering

---

## Figure 2 — Step Change 60 → 120 km/h (2 stacked panels)

This tests a harder scenario: the car is already cruising at 60 km/h and the setpoint jumps to 120 km/h at t = 20 s.

**Top panel — speed vs time:**
- Left half: car holds 60 km/h steadily
- At t=20 s: setpoint jumps, PID is reset (prevents a D-term spike from the sudden error jump), and the car accelerates to 120 km/h
- Shows how quickly the controller responds and whether it overshoots

**Bottom panel — control command vs time:**
- Shows the actuator signal (throttle/brake, clamped to [-1, +1])
- At t=20 s you'll see a **full-throttle burst** as the controller sees a large error
- As speed approaches 120 km/h the command tapers down
- The **anti-windup** clamp prevents the integral from over-charging during the hard acceleration phase, which would otherwise cause overshoot when the target is reached