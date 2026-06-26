"""
project2_pid_controller / visualize.py
----------------------------------------
Runs pid_demo to generate CSV files, then plots the results.
Run from the build directory:

    cd build
    python ../project2_pid_controller/visualize.py
"""

import os
import subprocess
import sys

import matplotlib.pyplot as plt
import pandas as pd

# ── Run the demo binary to (re)generate CSVs ─────────────────────────────────

_exe = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "build", "pid_demo.exe")
_exe = os.path.normpath(_exe)

if not os.path.exists(_exe):
    print(f"[error] pid_demo.exe not found at {_exe!r}")
    print("  Run build.bat first to compile the demo.")
    sys.exit(1)

print(f"Running {_exe} ...")
result = subprocess.run([_exe], capture_output=True, text=True)
if result.returncode != 0:
    print("[error] pid_demo failed:")
    print(result.stderr)
    sys.exit(result.returncode)
print(result.stdout.strip())


def require(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        print(f"  [warn] {path!r} not found — skipping (run pid_demo first)")
        return None
    return pd.read_csv(path)


TARGET_KPH = 100.0

# ── Figure 1: P vs PI vs PID comparison ──────────────────────────────────────

fig1, axes = plt.subplots(1, 3, figsize=(16, 4), sharey=True)
fig1.suptitle("Project 2 — PID Longitudinal Speed Control  (target: 100 km/h)", fontsize=13)

experiments = [
    ("p_only.csv",     "P-only  (Kp=0.3)",              axes[0]),
    ("pi_control.csv", "PI  (Kp=0.3, Ki=0.05)",         axes[1]),
    ("pid_control.csv","PID  (Kp=0.4, Ki=0.06, Kd=0.5)", axes[2]),
]

for csv_file, label, ax in experiments:
    df = require(csv_file)
    if df is None:
        continue
    ax.plot(df["time"], df["velocity_kph"], lw=2, label="velocity")
    ax.axhline(TARGET_KPH, color="red", ls="--", lw=1.2, label="target")
    ax.axhline(TARGET_KPH * 0.98, color="gray", ls=":", lw=0.8)
    ax.axhline(TARGET_KPH * 1.02, color="gray", ls=":", lw=0.8, label="±2% band")
    ax.set_xlim(0, 40)
    ax.set_ylim(0, 135)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (km/h)")
    ax.set_title(label)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("pid_comparison.png", dpi=150, bbox_inches="tight")
print("Saved pid_comparison.png")

# ── Figure 2: Step change  60 → 120 km/h ─────────────────────────────────────

df_sc = require("speed_change.csv")
if df_sc is not None:
    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig2.suptitle("Project 2 — Step Change:  60 → 120 km/h  (PI with anti-windup)", fontsize=12)

    ax1.plot(df_sc["time"], df_sc["velocity_kph"], lw=2, label="velocity")
    ax1.plot(df_sc["time"], df_sc["target_kph"],   "r--", lw=1.2, label="setpoint")
    ax1.set_ylabel("Speed (km/h)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(df_sc["time"], df_sc["command"], lw=1.5, color="tab:orange", label="command")
    ax2.axhline(0, color="black", lw=0.5)
    ax2.set_ylabel("Control command [-1, +1]")
    ax2.set_xlabel("Time (s)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("speed_change.png", dpi=150, bbox_inches="tight")
    print("Saved speed_change.png")

# ── Figure 3: bad-weather scenarios ──────────────────────────────────────────

fig3, axes3 = plt.subplots(1, 2, figsize=(14, 4))
fig3.suptitle("Project 2 — Bad-Weather Scenarios: Tuning & Windup Failures", fontsize=13)

# Left: aggressive Kp — velocity looks similar but command signal is bang-bang
df_bang = require("bad_kp_bang.csv")
df_p    = require("p_only.csv")
ax = axes3[0]
if df_bang is not None:
    ax.plot(df_bang["time"], df_bang["command"],
            lw=1.2, color="tab:red",   label="P-only Kp=5.0  command (bang-bang)")
if df_p is not None:
    ax.plot(df_p["time"],   df_p["command"],
            lw=1.5, color="tab:blue",  ls="--", label="P-only Kp=0.3  command (smooth)")
ax.axhline( 1.0, color="gray", ls=":", lw=0.8)
ax.axhline(-1.0, color="gray", ls=":", lw=0.8)
ax.axhline( 0.0, color="black", ls="-", lw=0.5)
ax.set_xlim(0, 40)
ax.set_ylim(-1.15, 1.15)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Control command  [-1, +1]")
ax.set_title("Actuator Abuse: Kp=5.0 saturates the command signal")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# Right: integral windup vs nominal PI
df_windup = require("bad_windup.csv")
df_pi     = require("pi_control.csv")
ax = axes3[1]
if df_windup is not None:
    ax.plot(df_windup["time"], df_windup["velocity_kph"],
            lw=2, color="tab:red",  label="PI no anti-windup  (Kp=0.3, Ki=0.05)")
if df_pi is not None:
    ax.plot(df_pi["time"], df_pi["velocity_kph"],
            lw=1.5, color="tab:blue", ls="--", label="PI with anti-windup  (nominal)")
ax.axhline(TARGET_KPH,        color="black", ls=":",  lw=1.2, label="Target 100 km/h")
ax.axhline(TARGET_KPH * 1.02, color="gray",  ls=":",  lw=0.8, label="+2% band")
ax.set_xlim(0, 40)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Speed (km/h)")
ax.set_title("Integral Windup: PI Without Anti-Windup Clamp")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("bad_weather.png", dpi=150, bbox_inches="tight")
print("Saved bad_weather.png")

plt.show()
