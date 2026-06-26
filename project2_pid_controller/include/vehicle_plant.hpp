// =============================================================================
// vehicle_plant.hpp  —  Simulated 1-D longitudinal vehicle (point-mass model)
// =============================================================================
// Models the longitudinal dynamics of a car:
//
//   m · dv/dt  =  F_drive(cmd) − F_drag(v)
//
// where:
//   F_drive = cmd * max_throttle_force   for cmd ∈ [0, +1]  (throttle)
//           = cmd * max_brake_force      for cmd ∈ [-1, 0)  (brake)
//   F_drag  = drag_coeff · v²            (aerodynamic quadratic drag)
//
// State propagation uses RK4 for accuracy, matching Project 1's integrator.
//
// Terminal velocity (full throttle, no brake):
//   v_max = sqrt(max_throttle_force / drag_coeff)
//         ≈ sqrt(3000 / 0.4) ≈ 86 m/s ≈ 310 km/h
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>

namespace controls {

/// Simulated 1-D longitudinal vehicle plant.
struct VehiclePlant {

    // ── Physical parameters (defaults: a typical mid-size car) ────────────────
    double mass               = 1500.0;   ///< kg
    double drag_coeff         =    0.4;   ///< N/(m/s)²   ≈ ½·ρ·Cd·A
    double max_throttle_force = 3000.0;   ///< N  (engine drive force at wheels)
    double max_brake_force    = 8000.0;   ///< N  (braking force)

    // ── State ─────────────────────────────────────────────────────────────────
    double velocity = 0.0;   ///< m/s (forward only; clamped to ≥ 0)

    // ── Step ──────────────────────────────────────────────────────────────────

    /// Propagate dynamics one timestep using RK4.
    ///
    /// @param cmd   control command ∈ [-1, +1]
    ///              +1 = full throttle,  -1 = full brake,  0 = coast
    /// @param dt    timestep in seconds
    void step(double cmd, double dt) {
        cmd = std::clamp(cmd, -1.0, 1.0);

        // Net acceleration as a function of velocity (cmd held constant over step)
        auto net_accel = [this, cmd](double v) -> double {
            const double drive = (cmd >= 0.0) ? cmd * max_throttle_force
                                              : cmd * max_brake_force;   // negative
            const double drag  = drag_coeff * v * v;                    // always positive
            return (drive - drag) / mass;
        };

        // RK4 for the scalar ODE dv/dt = net_accel(v)
        const double k1 = net_accel(velocity);
        const double k2 = net_accel(velocity + dt * 0.5 * k1);
        const double k3 = net_accel(velocity + dt * 0.5 * k2);
        const double k4 = net_accel(velocity + dt * k3);

        velocity += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        velocity  = std::max(0.0, velocity);   // vehicle cannot reverse
    }
};

} // namespace controls
