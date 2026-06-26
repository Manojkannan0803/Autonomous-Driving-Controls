"""
project1_integrators / visualize.py
------------------------------------
Runs integrators_demo to generate CSV files, then plots the results.
Run from the build directory:

    cd build
    python ../project1_integrators/visualize.py
"""

import os
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ── Run the demo binary to (re)generate CSVs ─────────────────────────────────

_exe = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "build", "integrators_demo.exe")
_exe = os.path.normpath(_exe)

if not os.path.exists(_exe):
    print(f"[error] integrators_demo.exe not found at {_exe!r}")
    print("  Run build.bat first to compile the demo.")
    sys.exit(1)

print(f"Running {_exe} ...")
result = subprocess.run([_exe], capture_output=True, text=True)
if result.returncode != 0:
    print("[error] integrators_demo failed:")
    print(result.stderr)
    sys.exit(result.returncode)
print(result.stdout.strip())


# ── helpers ───────────────────────────────────────────────────────────────────

def require(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        print(f"  [warn] {path!r} not found — skipping (run integrators_demo first)")
        return None
    return pd.read_csv(path)


# ── Figure 1: trajectory comparisons ─────────────────────────────────────────

fig1, axes = plt.subplots(1, 2, figsize=(13, 4))
fig1.suptitle("Project 1 — Numerical Integration: RK4 vs Exact", fontsize=13)

# Exponential decay
df_exp = require("exp_decay.csv")
if df_exp is not None:
    ax = axes[0]
    ax.plot(df_exp["time"], df_exp["numerical_0"], lw=2, label="RK4 (dt=0.01)")
    ax.plot(df_exp["time"], df_exp["exact_0"], "r--", lw=1.5, label="Exact")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("y(t)")
    ax.set_title(r"Exponential Decay:  $\dot{y} = -2y$,  $y(0)=1$")
    ax.legend()
    ax.grid(True, alpha=0.3)

# Harmonic oscillator
df_harm = require("harmonic.csv")
if df_harm is not None:
    ax = axes[1]
    ax.plot(df_harm["time"], df_harm["numerical_0"], lw=2, label="RK4 position (dt=0.01)")
    ax.plot(df_harm["time"], df_harm["exact_0"], "r--", lw=1.5, label="Exact position")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("x(t)")
    ax.set_title(r"Harmonic Oscillator:  $\ddot{x} = -4x$,  $x(0)=1$")
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("trajectories.png", dpi=150, bbox_inches="tight")
print("Saved trajectories.png")

# ── Figure 2: convergence analysis (log-log) ─────────────────────────────────

fig2, axes2 = plt.subplots(1, 2, figsize=(13, 4))
fig2.suptitle("Project 1 — Convergence Analysis (global error vs step size)", fontsize=13)

for ax, csv_file, title in [
    (axes2[0], "convergence_exp.csv",      "Exponential Decay"),
    (axes2[1], "convergence_harmonic.csv", "Harmonic Oscillator"),
]:
    df = require(csv_file)
    if df is None:
        continue

    dts = df["dt"].values
    ax.loglog(dts, df["euler_error"], "o-", label="Euler  O(h¹)")
    ax.loglog(dts, df["rk2_error"],   "s-", label="RK2    O(h²)")
    ax.loglog(dts, df["rk4_error"],   "^-", label="RK4    O(h⁴)")

    # Reference slopes
    ref_h  = dts[-3] ** 1 / df["euler_error"].iloc[-3] * df["euler_error"].iloc[-3]
    ax.loglog(dts, dts ** 1 * (df["euler_error"].iloc[0] / dts[0] ** 1), "k:", lw=0.8, label="slope 1")
    ax.loglog(dts, dts ** 2 * (df["rk2_error"].iloc[0]   / dts[0] ** 2), "k--", lw=0.8, label="slope 2")
    ax.loglog(dts, dts ** 4 * (df["rk4_error"].iloc[0]   / dts[0] ** 4), "k-", lw=0.8, label="slope 4")

    ax.set_xlabel("Step size dt (s)")
    ax.set_ylabel("Global L2 error")
    ax.set_title(title)
    ax.legend(fontsize=8)
    ax.grid(True, which="both", alpha=0.3)

plt.tight_layout()
plt.savefig("convergence.png", dpi=150, bbox_inches="tight")
print("Saved convergence.png")

# ── Figure 3: bad-weather scenarios ──────────────────────────────────────────

fig3, axes3 = plt.subplots(1, 2, figsize=(13, 4))
fig3.suptitle("Project 1 — Bad-Weather Scenarios: Where Euler Fails", fontsize=13)

# Left: Euler instability (λ·dt > 1 violates stability condition)
df_bad = require("euler_unstable.csv")
if df_bad is not None:
    ax = axes3[0]
    ax.plot(df_bad["time"], df_bad["euler"], "b-o", ms=5, lw=1.5, label="Euler (dt=0.6, λ·dt=1.2)")
    ax.plot(df_bad["time"], df_bad["exact"], "r--",        lw=1.5, label="Exact")
    ax.axhline(0, color="gray", lw=0.8, ls=":")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("y(t)")
    ax.set_title(r"Euler Instability: $\dot{y}=-2y$,  $\lambda \cdot dt=1.2 > 1$")
    ax.legend()
    ax.grid(True, alpha=0.3)

# Right: Euler energy drift on long-time harmonic oscillator
df_drift = require("euler_drift.csv")
if df_drift is not None:
    ax = axes3[1]
    exact_energy = 0.5 * 2.0**2 * 1.0**2  # 0.5 * ω² * x0² = 2.0
    ax.plot(df_drift["time"], df_drift["euler_energy"], "b-", lw=1.5, label="Euler energy (dt=0.1)")
    ax.plot(df_drift["time"], df_drift["rk4_energy"],   "g-", lw=1.5, label="RK4 energy   (dt=0.1)")
    ax.axhline(exact_energy, color="r", lw=1.5, ls="--", label=f"Exact energy = {exact_energy:.1f}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(r"Total energy  $E = \frac{1}{2}v^2 + \frac{1}{2}\omega^2 x^2$")
    ax.set_title(r"Euler Energy Drift: $\ddot{x}=-4x$,  $t \in [0,\,30]$")
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("bad_weather.png", dpi=150, bbox_inches="tight")
print("Saved bad_weather.png")

plt.show()
