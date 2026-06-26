"""
Project 7 – Path Planner visualiser
Generates:
  map_path.png  — occupancy grid + A* raw path + smoothed spline + start/goal
  curvature.png — κ(s) and v_ref(s) along the smoothed path
"""
import sys
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
GRID_CSV   = SCRIPT_DIR / "grid.csv"
RAW_CSV    = SCRIPT_DIR / "raw_path.csv"
SMOOTH_CSV = SCRIPT_DIR / "smooth_path.csv"
EXE        = SCRIPT_DIR.parent / "build" / "path_planner_demo.exe"

SHOW = "--show" in sys.argv


def ensure_csvs():
    """Run the demo binary if any CSV is missing."""
    if not (GRID_CSV.exists() and RAW_CSV.exists() and SMOOTH_CSV.exists()):
        print(f"CSVs missing — running {EXE.name} ...")
        subprocess.run([str(EXE)], cwd=str(SCRIPT_DIR), check=True)


def load_grid(path: Path):
    rows, cols = None, None
    lines = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("#"):
                # parse  # rows=60 cols=60
                for tok in line.split():
                    if tok.startswith("rows="):
                        rows = int(tok.split("=")[1])
                    elif tok.startswith("cols="):
                        cols = int(tok.split("=")[1])
            else:
                lines.append([int(v) for v in line.split(",")])
    grid = np.array(lines, dtype=np.float32)
    return grid, rows, cols


# ─────────────────────────────────────────────────────────────────────
def plot_map_path(grid, raw, smooth):
    fig, ax = plt.subplots(figsize=(7, 7))
    rows, cols = grid.shape

    # Obstacle map: grey = occupied, white = free
    ax.imshow(grid, origin="lower",
              extent=[0, cols, 0, rows],
              cmap="Greys", vmin=0, vmax=1, alpha=0.55)

    # A* raw path
    ax.plot(raw["x"], raw["y"], "b--", linewidth=1.2,
            label="A* raw path", zorder=3)

    # Smoothed spline
    ax.plot(smooth["x"], smooth["y"], color="crimson", linewidth=2.2,
            label="Spline (C² smooth)", zorder=4)

    # Speed colour overlay on smooth path
    sc = ax.scatter(smooth["x"], smooth["y"], c=smooth["v_ref"],
                    cmap="plasma", s=4, zorder=5,
                    vmin=smooth["v_ref"].min(), vmax=smooth["v_ref"].max())
    cbar = fig.colorbar(sc, ax=ax, fraction=0.035, pad=0.02)
    cbar.set_label("v_ref (m/s)", fontsize=9)

    # Start / Goal markers
    ax.plot(raw["x"].iloc[0],  raw["y"].iloc[0],  "go", markersize=12,
            zorder=6, label="Start")
    ax.plot(raw["x"].iloc[-1], raw["y"].iloc[-1], "r*", markersize=14,
            zorder=6, label="Goal")

    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Project 7 — Path Planner: A* + Cubic Spline Smoothing")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.3, alpha=0.4)

    fig.tight_layout()
    out = SCRIPT_DIR / "map_path.png"
    fig.savefig(out, dpi=150)
    print(f"Saved {out.name}")
    if SHOW:
        plt.show()
    plt.close(fig)


def plot_curvature(smooth):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    s    = smooth["s"]
    kap  = smooth["kappa"]
    vref = smooth["v_ref"]

    # ── κ(s) ──────────────────────────────────────────────────────────
    ax1.plot(s, kap, color="steelblue", linewidth=1.4)
    ax1.axhline(0, color="k", linewidth=0.6, linestyle="--")
    ax1.fill_between(s, kap, 0,
                     where=(kap > 0), alpha=0.25, color="steelblue",
                     label="Left curve (κ>0)")
    ax1.fill_between(s, kap, 0,
                     where=(kap < 0), alpha=0.25, color="tomato",
                     label="Right curve (κ<0)")
    ax1.set_ylabel("κ (1/m)")
    ax1.set_title("Curvature and Speed Profile Along Smoothed Path")
    ax1.legend(fontsize=8, loc="upper right")
    ax1.grid(True, linewidth=0.3, alpha=0.5)

    # Annotate max curvature
    idx_max = kap.abs().idxmax()
    ax1.annotate(f"|κ|_max = {abs(kap[idx_max]):.3f} 1/m",
                 xy=(s[idx_max], kap[idx_max]),
                 xytext=(s[idx_max] + 3, kap[idx_max] * 1.3),
                 fontsize=8, color="navy",
                 arrowprops=dict(arrowstyle="->", color="navy", lw=0.8))

    # ── v_ref(s) ───────────────────────────────────────────────────────
    ax2.plot(s, vref, color="forestgreen", linewidth=1.6)
    ax2.fill_between(s, vref, alpha=0.2, color="forestgreen")
    ax2.set_xlabel("Arc length s (m)")
    ax2.set_ylabel("v_ref (m/s)")
    ax2.set_ylim(0, vref.max() * 1.15)
    ax2.grid(True, linewidth=0.3, alpha=0.5)

    # Stats box
    stats = (f"Path length: {s.iloc[-1]:.1f} m\n"
             f"Max |κ|: {kap.abs().max():.3f} 1/m\n"
             f"Min R: {1/kap.abs().max():.1f} m\n"
             f"v range: {vref.min():.1f}–{vref.max():.1f} m/s")
    ax2.text(0.98, 0.97, stats,
             transform=ax2.transAxes,
             ha="right", va="top", fontsize=8,
             bbox=dict(boxstyle="round,pad=0.3", facecolor="wheat", alpha=0.7))

    fig.tight_layout()
    out = SCRIPT_DIR / "curvature.png"
    fig.savefig(out, dpi=150)
    print(f"Saved {out.name}")
    if SHOW:
        plt.show()
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ensure_csvs()

    grid, rows, cols = load_grid(GRID_CSV)
    raw    = pd.read_csv(RAW_CSV)
    smooth = pd.read_csv(SMOOTH_CSV)

    plot_map_path(grid, raw, smooth)
    plot_curvature(smooth)

    print(f"\nAll plots saved to: {SCRIPT_DIR}")
    print("Tip: pass --show to open interactive windows.")
