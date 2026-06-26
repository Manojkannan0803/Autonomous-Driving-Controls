// =============================================================================
// project1_integrators / src / main.cpp
//
// Demonstrates the integrators library on two ODEs with known analytical
// solutions, then runs a convergence analysis to verify the expected
// O(dt), O(dt²), O(dt⁴) error scaling.
//
// Output files (written to the working directory — i.e. your build folder):
//   exp_decay.csv          — exponential decay trajectory
//   harmonic.csv           — harmonic oscillator trajectory
//   convergence.csv        — error vs step-size table for all three methods
//
// Use:   python visualize.py   (from the build directory) to plot results.
// =============================================================================
#include "integrators.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// ── Analytical solutions ──────────────────────────────────────────────────────

// ODE: dy/dt = -λ·y           Exact: y(t) = y₀·exp(-λt)
struct ExponentialDecay {
    double lambda;

    numerics::DerivFn ode() const {
        return [lam = lambda](double /*t*/, const numerics::State& x) {
            return numerics::State{-lam * x[0]};
        };
    }
    double exact(double t, double y0) const {
        return y0 * std::exp(-lambda * t);
    }
};

// ODE: x'' = -ω²x  →  state = [x, v]    Exact (x(0)=1, v(0)=0): x(t) = cos(ωt)
struct HarmonicOscillator {
    double omega;

    numerics::DerivFn ode() const {
        return [w = omega](double /*t*/, const numerics::State& s) {
            return numerics::State{s[1], -w * w * s[0]};
        };
    }
    double exact_x(double t) const { return std::cos(omega * t); }
    double exact_v(double t) const { return -omega * std::sin(omega * t); }
};

// ── Utility ───────────────────────────────────────────────────────────────────

double euclidean_error(const numerics::State& approx, const numerics::State& exact) {
    double sum = 0.0;
    for (std::size_t i = 0; i < approx.size(); ++i) {
        const double d = approx[i] - exact[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

void write_trajectory_csv(const std::string& path,
                           const numerics::Trajectory& traj,
                           std::function<std::vector<double>(double)> exact_fn) {
    std::ofstream file(path);
    file << std::fixed << std::setprecision(8);

    // Header
    file << "time";
    for (std::size_t j = 0; j < traj.states[0].size(); ++j)
        file << ",numerical_" << j;
    for (std::size_t j = 0; j < traj.states[0].size(); ++j)
        file << ",exact_" << j;
    file << '\n';

    // Rows
    for (std::size_t i = 0; i < traj.size(); ++i) {
        file << traj.times[i];
        for (double v : traj.states[i])
            file << ',' << v;
        for (double v : exact_fn(traj.times[i]))
            file << ',' << v;
        file << '\n';
    }
    std::cout << "  Wrote " << traj.size() << " rows -> " << path << '\n';
}

// ── Convergence analysis ──────────────────────────────────────────────────────
// Vary dt, measure global error at t_end, observe O(dt^p) scaling.

void convergence_analysis(const numerics::DerivFn& ode,
                           double t0, double t_end,
                           const numerics::State& x0,
                           std::function<numerics::State(double)> exact_at_t,
                           const std::string& csv_path) {
    const std::vector<double> steps = {0.1, 0.05, 0.025, 0.01, 0.005, 0.001};
    const numerics::State exact = exact_at_t(t_end);

    std::cout << '\n';
    std::cout << std::setw(10) << "dt"
              << std::setw(16) << "Euler error"
              << std::setw(16) << "RK2 error"
              << std::setw(16) << "RK4 error" << '\n';
    std::cout << std::string(58, '-') << '\n';

    std::ofstream csv(csv_path);
    csv << "dt,euler_error,rk2_error,rk4_error\n";
    csv << std::scientific << std::setprecision(8);

    for (double dt : steps) {
        const auto t_euler = numerics::integrate(ode, t0, t_end, x0, dt, numerics::Method::EULER);
        const auto t_rk2   = numerics::integrate(ode, t0, t_end, x0, dt, numerics::Method::RK2);
        const auto t_rk4   = numerics::integrate(ode, t0, t_end, x0, dt, numerics::Method::RK4);

        const double e_euler = euclidean_error(t_euler.final_state(), exact);
        const double e_rk2   = euclidean_error(t_rk2.final_state(),   exact);
        const double e_rk4   = euclidean_error(t_rk4.final_state(),   exact);

        std::cout << std::scientific << std::setprecision(3)
                  << std::setw(10) << dt
                  << std::setw(16) << e_euler
                  << std::setw(16) << e_rk2
                  << std::setw(16) << e_rk4 << '\n';

        csv << dt << ',' << e_euler << ',' << e_rk2 << ',' << e_rk4 << '\n';
    }
    std::cout << "  Wrote convergence table -> " << csv_path << '\n';
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "==============================================\n";
    std::cout << "   Project 1 — Numerical Integration Demo    \n";
    std::cout << "==============================================\n";

    // ── Experiment 1: Exponential Decay ──────────────────────────────────────
    {
        std::cout << "\n[1] Exponential Decay:  dy/dt = -2·y,  y(0) = 1\n";

        const ExponentialDecay sys{.lambda = 2.0};
        const numerics::State  x0{1.0};
        const double           t_end = 3.0;

        const auto traj = numerics::integrate(sys.ode(), 0.0, t_end, x0, 0.01);

        const double approx = traj.final_state()[0];
        const double exact  = sys.exact(t_end, 1.0);
        std::cout << "  RK4 at t=" << t_end
                  << ":  numerical=" << std::fixed << std::setprecision(8) << approx
                  << "  exact=" << exact
                  << "  |error|=" << std::scientific << std::abs(approx - exact) << '\n';

        write_trajectory_csv("exp_decay.csv", traj,
            [&sys](double t) { return std::vector<double>{sys.exact(t, 1.0)}; });

        std::cout << "  Convergence analysis (t_end=" << t_end << "):\n";
        convergence_analysis(sys.ode(), 0.0, t_end, x0,
            [&sys](double t) -> numerics::State { return {sys.exact(t, 1.0)}; },
            "convergence_exp.csv");
    }

    // ── Experiment 2: Harmonic Oscillator ────────────────────────────────────
    {
        std::cout << "\n[2] Harmonic Oscillator:  x''=-4x,  x(0)=1, v(0)=0\n";

        const HarmonicOscillator sys{.omega = 2.0};
        const numerics::State    x0{1.0, 0.0};
        const double             t_end = 10.0;

        const auto t_euler = numerics::integrate(sys.ode(), 0.0, t_end, x0, 0.01, numerics::Method::EULER);
        const auto t_rk4   = numerics::integrate(sys.ode(), 0.0, t_end, x0, 0.01, numerics::Method::RK4);

        const auto exact_final = numerics::State{sys.exact_x(t_end), sys.exact_v(t_end)};
        std::cout << "  Euler |error| at t=" << t_end << ":  "
                  << std::scientific << euclidean_error(t_euler.final_state(), exact_final) << '\n';
        std::cout << "  RK4   |error| at t=" << t_end << ":  "
                  << euclidean_error(t_rk4.final_state(), exact_final) << '\n';

        write_trajectory_csv("harmonic.csv", t_rk4,
            [&sys](double t) {
                return std::vector<double>{sys.exact_x(t), sys.exact_v(t)};
            });

        std::cout << "  Convergence analysis (t_end=" << t_end << "):\n";
        convergence_analysis(sys.ode(), 0.0, t_end, x0,
            [&sys](double t) -> numerics::State {
                return {sys.exact_x(t), sys.exact_v(t)};
            },
            "convergence_harmonic.csv");
    }

    // ── Bad-weather 1: Euler instability (exp decay, dt=0.6, λ·dt=1.2>1) ──────
    {
        std::cout << "\n[BAD-1] Euler instability:  dy/dt=-2y,  dt=0.6  (lambda*dt=1.2 > 1)\n";

        const ExponentialDecay sys{.lambda = 2.0};
        const numerics::State  x0{1.0};
        const double           t_end = 5.0;

        const auto traj = numerics::integrate(sys.ode(), 0.0, t_end, x0, 0.6, numerics::Method::EULER);

        std::ofstream csv("euler_unstable.csv");
        csv << "time,euler,exact\n" << std::fixed << std::setprecision(8);
        for (std::size_t i = 0; i < traj.size(); ++i) {
            const double t = traj.times[i];
            csv << t << ',' << traj.states[i][0] << ',' << sys.exact(t, 1.0) << '\n';
        }
        std::cout << "  Wrote euler_unstable.csv\n";
    }

    // ── Bad-weather 2: Euler energy drift (harmonic oscillator, long time) ───
    {
        std::cout << "\n[BAD-2] Euler energy drift:  x''=-4x,  dt=0.1,  t_end=30\n";

        const HarmonicOscillator sys{.omega = 2.0};
        const numerics::State    x0{1.0, 0.0};
        const double             t_end = 30.0;
        const double             dt    = 0.1;

        const auto t_euler = numerics::integrate(sys.ode(), 0.0, t_end, x0, dt, numerics::Method::EULER);
        const auto t_rk4   = numerics::integrate(sys.ode(), 0.0, t_end, x0, dt, numerics::Method::RK4);

        auto energy = [](const numerics::State& s, double omega) {
            return 0.5 * s[1] * s[1] + 0.5 * omega * omega * s[0] * s[0];
        };

        std::ofstream csv("euler_drift.csv");
        csv << "time,euler_pos,rk4_pos,exact_pos,euler_energy,rk4_energy\n"
            << std::fixed << std::setprecision(8);
        for (std::size_t i = 0; i < t_euler.size(); ++i) {
            const double t = t_euler.times[i];
            csv << t
                << ',' << t_euler.states[i][0]
                << ',' << t_rk4.states[i][0]
                << ',' << sys.exact_x(t)
                << ',' << energy(t_euler.states[i], sys.omega)
                << ',' << energy(t_rk4.states[i], sys.omega)
                << '\n';
        }
        std::cout << "  Wrote euler_drift.csv\n";
    }

    std::cout << "\nAll done.  Visualise with:  python ../visualize.py\n";
    return 0;
}
