"""
visualize.py — Project 5: Kalman Filter + EKF Sensor Fusion Plots

Produces three figures:
  trajectory.png    — True path, GPS scatter, KF track, EKF track
  errors.png        — Position error over time + covariance trace
  innovation.png    — EKF innovation sequence (white-noise test)

Run from the project5 directory:
  python visualize.py          # saves PNGs only
  python visualize.py --show   # also opens interactive windows
"""

import sys
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
DEMO_EXE   = SCRIPT_DIR.parent / "build" / "kalman_demo.exe"
TRAJ_CSV   = SCRIPT_DIR / "trajectory.csv"
ERR_CSV    = SCRIPT_DIR / "errors.csv"
INNOV_CSV  = SCRIPT_DIR / "innovations.csv"


def ensure_csvs():
    if not TRAJ_CSV.exists():
        print("CSVs not found — running kalman_demo.exe ...")
        subprocess.run([str(DEMO_EXE)], check=True, cwd=str(SCRIPT_DIR))


def load_data():
    traj  = pd.read_csv(TRAJ_CSV)
    errs  = pd.read_csv(ERR_CSV)
    innov = pd.read_csv(INNOV_CSV)
    return traj, errs, innov


# ── Figure 1: Trajectory comparison ──────────────────────────────────────────
def plot_trajectory(traj: pd.DataFrame):
    fig, ax = plt.subplots(figsize=(8, 8))

    # GPS scatter (only where gps_valid == 1)
    gps = traj[traj["gps_valid"] == 1]
    ax.scatter(gps["gps_x"], gps["gps_y"],
               s=12, alpha=0.4, color="lightcoral", zorder=2,
               label=f"GPS (σ=3 m, {len(gps)} fixes)")

    ax.plot(traj["true_x"], traj["true_y"],
            "k-", linewidth=2.0, zorder=5, label="Ground truth")
    ax.plot(traj["kf_x"],  traj["kf_y"],
            "b--", linewidth=1.5, zorder=4, label="LinearKF (GPS only)")
    ax.plot(traj["ekf_x"], traj["ekf_y"],
            "g-",  linewidth=1.5, zorder=4, label="EKF (GPS + IMU)")

    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.set_title("Project 5 — Sensor Fusion: Trajectory Comparison")
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = str(SCRIPT_DIR / "trajectory.png")
    fig.savefig(path, dpi=150)
    print(f"Saved trajectory.png")
    return fig


# ── Figure 2: Estimation errors + covariance trace ───────────────────────────
def plot_errors(errs: pd.DataFrame):
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax = axes[0]
    ax.plot(errs["t"], errs["kf_err"],  "b-",  linewidth=1.2,
            label="LinearKF error (m)")
    ax.plot(errs["t"], errs["ekf_err"], "g-",  linewidth=1.2,
            label="EKF error (m)")
    ax.axhline(3.0, color="red", linestyle="--", linewidth=1,
               label="GPS noise σ = 3 m")
    ax.set_ylabel("Position error (m)")
    ax.set_title("Position Error vs Time")
    ax.legend()
    ax.grid(True, alpha=0.3)

    kf_rmse  = np.sqrt(np.mean(errs["kf_err"] ** 2))
    ekf_rmse = np.sqrt(np.mean(errs["ekf_err"] ** 2))
    ax.text(0.02, 0.92,
            f"RMSE  LinearKF: {kf_rmse:.2f} m   EKF: {ekf_rmse:.2f} m",
            transform=ax.transAxes, fontsize=9,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    ax2 = axes[1]
    ax2.plot(errs["t"], errs["kf_P_trace"],  "b-",  linewidth=1.2,
             label="KF covariance trace")
    ax2.plot(errs["t"], errs["ekf_P_trace"], "g-",  linewidth=1.2,
             label="EKF covariance trace")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Tr(P) — total uncertainty")
    ax2.set_title("Filter Uncertainty Over Time (Tr(P))")
    ax2.set_yscale("log")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    fig.suptitle("Project 5 — Estimation Errors and Covariance", fontsize=12)
    fig.tight_layout()
    path = str(SCRIPT_DIR / "errors.png")
    fig.savefig(path, dpi=150)
    print(f"Saved errors.png")
    return fig


# ── Figure 3: Innovation sequence (white-noise test) ─────────────────────────
def plot_innovation(innov: pd.DataFrame):
    """
    A well-tuned Kalman Filter should produce innovations (measurement residuals)
    that look like white noise with zero mean.  If innovations are correlated
    (autocorrelation ≠ 0 at lag > 0) the filter is either mis-tuned or the
    model is wrong.
    """
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))

    for col, label, row in [("innov_x", "Innovation X (m)", 0),
                             ("innov_y", "Innovation Y (m)", 1)]:
        data = innov[col].values

        # Time series
        axes[row][0].plot(innov["t"], data, "g-", linewidth=0.8, alpha=0.7)
        axes[row][0].axhline(0, color="k", linewidth=0.5)
        axes[row][0].set_xlabel("Time (s)")
        axes[row][0].set_ylabel(label)
        axes[row][0].set_title(f"{label} over time")
        axes[row][0].grid(True, alpha=0.3)

        # Histogram — should be roughly Gaussian
        axes[row][1].hist(data, bins=25, color="steelblue", edgecolor="white",
                          density=True, alpha=0.8)
        mu, sigma = np.mean(data), np.std(data)
        x_range = np.linspace(mu - 4*sigma, mu + 4*sigma, 200)
        # Manual Gaussian PDF (avoids scipy dependency)
        gauss_pdf = (1.0 / (sigma * np.sqrt(2 * np.pi))) * \
                    np.exp(-0.5 * ((x_range - mu) / sigma) ** 2)
        axes[row][1].plot(x_range, gauss_pdf,
                          "r-", linewidth=1.5, label=f"N(μ={mu:.1f}, σ={sigma:.1f})")
        axes[row][1].set_xlabel(label)
        axes[row][1].set_ylabel("Density")
        axes[row][1].set_title(f"Innovation distribution")
        axes[row][1].legend(fontsize=8)
        axes[row][1].grid(True, alpha=0.3)

    fig.suptitle("EKF Innovation Sequence  (should be zero-mean white noise)",
                 fontsize=11)
    fig.tight_layout()
    path = str(SCRIPT_DIR / "innovation.png")
    fig.savefig(path, dpi=150)
    print(f"Saved innovation.png")
    return fig


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    show = "--show" in sys.argv
    ensure_csvs()
    traj, errs, innov = load_data()

    figs = [
        plot_trajectory(traj),
        plot_errors(errs),
        plot_innovation(innov),
    ]

    print(f"\nAll plots saved to: {SCRIPT_DIR}")
    print("Tip: pass --show to open interactive windows.")

    if show:
        matplotlib.use("TkAgg")
        plt.show()
