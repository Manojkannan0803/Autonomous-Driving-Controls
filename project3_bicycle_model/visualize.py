"""
project3_bicycle_model / visualize.py
---------------------------------------
Auto-builds + runs bicycle_demo if the CSVs are missing, then plots results.
Can be run from any directory:

    python "c:\\...\\project3_bicycle_model\\visualize.py"

Pass --show to pop up interactive windows (requires a display).
By default only PNG files are saved.
"""

import os
import subprocess
import sys
from pathlib import Path

import matplotlib
# Use non-interactive Agg backend by default so the script never blocks.
# Switch to the default interactive backend only when --show is passed.
_SHOW = "--show" in sys.argv
if not _SHOW:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ── Resolve paths relative to this script, not the CWD ───────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT   = SCRIPT_DIR.parent
BUILD_DIR   = REPO_ROOT / "build"
EXE         = BUILD_DIR / "bicycle_demo.exe"
# CSVs are written to whatever directory the exe runs in — we run it from
# SCRIPT_DIR so they land right next to this file for easy loading.
CSV_DIR     = SCRIPT_DIR

NEEDED_CSVS = ["circle.csv", "s_curve.csv", "figure_eight.csv"]


def ensure_csvs():
    """Run bicycle_demo if any CSV is missing."""
    missing = [c for c in NEEDED_CSVS if not (CSV_DIR / c).exists()]
    if not missing:
        return  # all good

    print(f"  CSVs not found: {missing}")

    if not EXE.exists():
        # Try to build first
        print(f"  bicycle_demo.exe not found at {EXE}")
        print("  Attempting to compile with g++ ...")
        compile_cmd = [
            "g++", "-std=c++20", "-Wall", "-O2",
            f"-I{SCRIPT_DIR / 'include'}",
            str(SCRIPT_DIR / "src" / "main.cpp"),
            f"-o{EXE}",
        ]
        result = subprocess.run(compile_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print("  Compile failed:\n", result.stderr)
            sys.exit(1)
        print("  Compiled successfully.")

    print(f"  Running {EXE.name} ...")
    result = subprocess.run(
        [str(EXE)],
        cwd=str(CSV_DIR),      # ← CSVs will be written next to visualize.py
        capture_output=True,
        text=True,
    )
    print(result.stdout)
    if result.returncode != 0:
        print("  bicycle_demo failed:\n", result.stderr)
        sys.exit(1)


def load(filename):
    path = CSV_DIR / filename
    if not path.exists():
        print(f"  [warn] {filename!r} still not found after running demo — check build.")
        return None
    return pd.read_csv(path)


# ── Auto-run demo if needed ───────────────────────────────────────────────────
ensure_csvs()


# ── Figure 1: Circular motion ─────────────────────────────────────────────────
df_c = load("circle.csv")
if df_c is not None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Project 3 — Scenario 1: Circular Motion Validation", fontsize=13)

    ax = axes[0]
    ax.plot(df_c["x"], df_c["y"], lw=2, label="Simulated path")

    # Reference circle
    R = df_c["dist_from_centre"].mean()
    theta_ref = np.linspace(0, 2 * np.pi, 300)
    ax.plot(R * np.cos(theta_ref), R * np.sin(theta_ref), "r--", lw=1.2, label=f"Reference circle (R={R:.2f}m)")

    ax.plot(df_c["x"].iloc[0], df_c["y"].iloc[0], "go", ms=8, label="Start")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("XY trajectory")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(df_c["time"], df_c["dist_from_centre"], lw=2, label="Actual radius")
    ax.axhline(R, color="red", ls="--", lw=1.2, label=f"Theoretical R = {R:.3f} m")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance from centre (m)")
    ax.set_title("Radius consistency (should be flat)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(str(SCRIPT_DIR / "circle_validation.png"), dpi=150, bbox_inches="tight")
    print("Saved circle_validation.png")

# ── Figure 2: S-curve ─────────────────────────────────────────────────────────
df_s = load("s_curve.csv")
if df_s is not None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 4))
    fig.suptitle("Project 3 — Scenario 2: S-Curve (open-loop, no controller)", fontsize=13)

    ax = axes[0]
    ax.plot(df_s["x"], df_s["y"], lw=2, label="Vehicle path")
    # Overlay reference S-curve
    length = df_s["x"].max()
    s_ref  = np.linspace(0, 1, 300)
    ax.plot(s_ref * length, 1.5 * np.sin(2 * np.pi * s_ref), "r--", lw=1.5, label="Reference path")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("XY trajectory vs reference")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(df_s["time"], df_s["cte"], lw=2, color="tab:orange", label="CTE (m)")
    ax.axhline(0, color="black", lw=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Cross-Track Error (m)")
    ax.set_title("CTE without a controller — non-zero drift")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(str(SCRIPT_DIR / "s_curve_openloop.png"), dpi=150, bbox_inches="tight")
    print("Saved s_curve_openloop.png")

# ── Figure 3: Figure-eight ────────────────────────────────────────────────────
df_f = load("figure_eight.csv")
if df_f is not None:
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(df_f["x"], df_f["y"], lw=2, label="Simulated path")
    ax.plot(df_f["x"].iloc[0], df_f["y"].iloc[0], "go", ms=8, label="Start")
    ax.plot(df_f["x"].iloc[-1], df_f["y"].iloc[-1], "rs", ms=8, label="End")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Project 3 — Scenario 3: Figure-Eight\n(heading wrap stress test)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(str(SCRIPT_DIR / "figure_eight.png"), dpi=150, bbox_inches="tight")
    print("Saved figure_eight.png")

if _SHOW:
    plt.show()
else:
    print("\nAll plots saved to:", SCRIPT_DIR)
    print("Tip: pass --show to open interactive windows.")
