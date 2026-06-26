"""
visualize.py — Project 6: LQR Trajectory Tracker Plots

Produces three figures:
  trajectory.png  — True paths for P6 and P4 overlaid on reference
  tracking.png    — CTE + speed error comparison over time
  gains.png       — Gain schedule: how K and v_ref vary with curvature along path

Run from the project6 directory:
  python visualize.py          # saves PNGs only
  python visualize.py --show   # also opens interactive windows
"""

import sys
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
DEMO_EXE   = SCRIPT_DIR.parent / "build" / "lqr_tracker_demo.exe"
TRACK_CSV  = SCRIPT_DIR / "tracking.csv"
GAINS_CSV  = SCRIPT_DIR / "gains.csv"


def ensure_csvs():
    if not TRACK_CSV.exists():
        print("CSVs not found — running lqr_tracker_demo.exe ...")
        subprocess.run([str(DEMO_EXE)], check=True, cwd=str(SCRIPT_DIR))


def load_data():
    track = pd.read_csv(TRACK_CSV)
    gains = pd.read_csv(GAINS_CSV)
    return track, gains


# ── Figure 1: Trajectory comparison ──────────────────────────────────────────
def plot_trajectory(track: pd.DataFrame):
    fig, ax = plt.subplots(figsize=(8, 8))

    # Downsample for clarity
    n = max(1, len(track) // 800)
    t = track.iloc[::n]

    ax.plot(t["p6_x"], t["p6_y"], "g-", linewidth=1.5, label="LQR Tracker (P6)", zorder=4)
    ax.plot(t["p4_x"], t["p4_y"], "b--", linewidth=1.5, label="Fixed LQR (P4)", zorder=3)

    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.set_title("Project 6 — LQR Tracker vs Fixed LQR: Trajectory")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(str(SCRIPT_DIR / "trajectory.png"), dpi=150)
    print("Saved trajectory.png")
    return fig


# ── Figure 2: CTE + speed error over time ────────────────────────────────────
def plot_tracking(track: pd.DataFrame):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    ax = axes[0]
    ax.plot(track["t"], track["p6_cte"].abs(), "g-", linewidth=1.0,
            label="P6 |CTE| (m)")
    ax.plot(track["t"], track["p4_cte"].abs(), "b-", alpha=0.7, linewidth=1.0,
            label="P4 |CTE| (m)")
    p6_rms = np.sqrt(np.mean(track["p6_cte"]**2))
    p4_rms = np.sqrt(np.mean(track["p4_cte"]**2))
    ax.axhline(p6_rms, color="g", linestyle=":", linewidth=1)
    ax.axhline(p4_rms, color="b", linestyle=":", linewidth=1)
    ax.set_ylabel("|CTE| (m)")
    ax.set_title(f"Cross-Track Error   P6 RMS={p6_rms:.3f}m   P4 RMS={p4_rms:.3f}m")
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=0)

    ax2 = axes[1]
    ax2.plot(track["t"], track["p6_v"],    "g-", linewidth=1.2, label="P6 actual speed")
    ax2.plot(track["t"], track["p6_v_ref"], "k--", linewidth=1.0, label="Reference v_ref")
    ax2.plot(track["t"], track["p4_v"],   "b-", linewidth=1.2, alpha=0.7, label="P4 speed (const)")
    ax2.set_ylabel("Speed (m/s)")
    ax2.set_title("Speed Profiles")
    ax2.legend(); ax2.grid(True, alpha=0.3)

    ax3 = axes[2]
    ax3.plot(track["t"], track["p6_ve"].abs(), "g-", linewidth=1.0,
             label="P6 |speed error| (m/s)")
    ax3.set_ylabel("|Speed error| (m/s)")
    ax3.set_xlabel("Time (s)")
    ax3.set_title("Speed Tracking Error (P6 only — P4 has no speed controller)")
    ax3.legend(); ax3.grid(True, alpha=0.3); ax3.set_ylim(bottom=0)

    fig.suptitle("Project 6 — Tracking Performance", fontsize=12)
    fig.tight_layout()
    fig.savefig(str(SCRIPT_DIR / "tracking.png"), dpi=150)
    print("Saved tracking.png")
    return fig


# ── Figure 3: Gain schedule ───────────────────────────────────────────────────
def plot_gains(gains: pd.DataFrame):
    """
    Shows how LQR gains and reference speed vary along the trajectory.
    Key insight: K values increase where curvature is high and speed drops —
    the controller automatically becomes more aggressive where needed.
    """
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    ax = axes[0]
    ax.plot(gains["s"], gains["v_ref"], "k-", linewidth=1.5)
    ax.fill_between(gains["s"], gains["v_ref"], alpha=0.2, color="steelblue")
    ax.set_ylabel("v_ref (m/s)")
    ax.set_title("Reference speed profile (slows on high-curvature segments)")
    ax.grid(True, alpha=0.3)

    ax2 = axes[1]
    ax2.plot(gains["s"], gains["kappa"], "purple", linewidth=1.2)
    ax2.axhline(0, color="k", linewidth=0.5)
    ax2.fill_between(gains["s"], gains["kappa"], 0,
                     where=gains["kappa"] > 0, alpha=0.2, color="green",
                     label="Left curve (κ > 0)")
    ax2.fill_between(gains["s"], gains["kappa"], 0,
                     where=gains["kappa"] < 0, alpha=0.2, color="red",
                     label="Right curve (κ < 0)")
    ax2.set_ylabel("κ (1/m)")
    ax2.set_title("Path curvature")
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    ax3 = axes[2]
    ax3.plot(gains["s"], gains["K00"], "g-",  linewidth=1.2, label="K₀₀  (steer ← CTE)")
    ax3.plot(gains["s"], gains["K01"], "b-",  linewidth=1.2, label="K₀₁  (steer ← heading)")
    ax3.plot(gains["s"], gains["K12"], "r-",  linewidth=1.2, label="K₁₂  (accel ← speed)")
    ax3.set_xlabel("Arc-length s (m)")
    ax3.set_ylabel("Gain value")
    ax3.set_title("LQR gain schedule  (varies with v_ref at each waypoint)")
    ax3.legend(); ax3.grid(True, alpha=0.3)

    fig.suptitle("Project 6 — Time-Varying Gain Schedule", fontsize=12)
    fig.tight_layout()
    fig.savefig(str(SCRIPT_DIR / "gains.png"), dpi=150)
    print("Saved gains.png")
    return fig


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    show = "--show" in sys.argv
    ensure_csvs()
    track, gains = load_data()

    figs = [
        plot_trajectory(track),
        plot_tracking(track),
        plot_gains(gains),
    ]

    print(f"\nAll plots saved to: {SCRIPT_DIR}")
    print("Tip: pass --show to open interactive windows.")

    if show:
        matplotlib.use("TkAgg")
        plt.show()
