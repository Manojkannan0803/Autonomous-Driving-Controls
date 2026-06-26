"""
project4_lateral_controllers / visualize.py
--------------------------------------------
Auto-runs lateral_demo if CSVs are missing, then produces three figures:

  Figure 1 — XY trajectories at 36 km/h: all 3 controllers vs reference
  Figure 2 — CTE over time at 36 km/h: the "before/after Project 3" comparison
  Figure 3 — Speed sensitivity: RMS CTE of each controller at 10 / 36 / 72 km/h
"""

import subprocess
import sys
from pathlib import Path

import matplotlib
_SHOW = "--show" in sys.argv
if not _SHOW:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT   = SCRIPT_DIR.parent
EXE         = REPO_ROOT / "build" / "lateral_demo.exe"
P3_INC      = REPO_ROOT / "project3_bicycle_model" / "include"

SPEEDS_KMH  = [2, 10, 20]     # labels produced by the demo (rounded from m/s)
CONTROLLERS = ["PurePursuit", "Stanley", "LQR"]
COLOURS     = {"PurePursuit": "tab:blue", "Stanley": "tab:orange", "LQR": "tab:green"}


# ── Auto-run demo if CSVs are missing ─────────────────────────────────────────

def ensure_csvs():
    sample = SCRIPT_DIR / "PurePursuit_10kmh.csv"
    if sample.exists():
        return

    print("  CSVs not found — running lateral_demo ...")
    if not EXE.exists():
        print(f"  lateral_demo.exe not found at {EXE}")
        print("  Attempting compile with g++ ...")
        compile_cmd = [
            "g++", "-std=c++20", "-O2",
            f"-I{SCRIPT_DIR / 'include'}",
            f"-I{P3_INC}",
            str(SCRIPT_DIR / "src" / "main.cpp"),
            f"-o{EXE}",
        ]
        r = subprocess.run(compile_cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print("  Compile failed:\n", r.stderr)
            sys.exit(1)
        print("  Compiled.")

    r = subprocess.run([str(EXE)], cwd=str(SCRIPT_DIR),
                       capture_output=True, text=True)
    print(r.stdout)
    if r.returncode != 0:
        print(r.stderr)
        sys.exit(1)


def load(name):
    p = SCRIPT_DIR / name
    if not p.exists():
        return None
    return pd.read_csv(p)


def find_csv(ctrl, speed_kmh):
    """Find the CSV whose filename contains controller name and speed."""
    for f in SCRIPT_DIR.glob(f"{ctrl}_*.csv"):
        if f"_{speed_kmh}kmh" in f.name or f"_{speed_kmh:02d}kmh" in f.name:
            return pd.read_csv(f)
    # Try any speed file for this controller (fallback)
    files = list(SCRIPT_DIR.glob(f"{ctrl}_*.csv"))
    if files:
        return pd.read_csv(files[0])
    return None


ensure_csvs()

# ── Figure 1: XY trajectories (one speed) ────────────────────────────────────

fig1, ax = plt.subplots(figsize=(12, 5))
fig1.suptitle("Project 4 — Lateral Controllers: XY Trajectory on S-Curve", fontsize=13)

# Reference S-curve overlay
length = 80.0
s_ref  = np.linspace(0, 1, 400)
ax.plot(s_ref * length, 1.5 * np.sin(2 * np.pi * s_ref),
        "k--", lw=1.5, label="Reference path", zorder=5)

# Each controller (use first available CSV)
for ctrl in CONTROLLERS:
    files = sorted(SCRIPT_DIR.glob(f"{ctrl}_*.csv"))
    if not files:
        continue
    # Pick medium speed file
    df = None
    for f in files:
        if "36" in f.name or "10" in f.name:
            df = pd.read_csv(f); break
    if df is None:
        df = pd.read_csv(files[0])
    speed_label = files[0].stem.split("_")[-1]
    ax.plot(df["x"], df["y"], lw=2, color=COLOURS[ctrl],
            label=f"{ctrl} ({speed_label})")

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Closed-loop path following (compare with open-loop CTE = 26 m in Project 3)")
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(str(SCRIPT_DIR / "xy_comparison.png"), dpi=150, bbox_inches="tight")
print("Saved xy_comparison.png")

# ── Figure 2: CTE over time ───────────────────────────────────────────────────

fig2, ax = plt.subplots(figsize=(12, 4))
fig2.suptitle("Project 4 — Cross-Track Error over Time (closed-loop)", fontsize=13)

for ctrl in CONTROLLERS:
    files = sorted(SCRIPT_DIR.glob(f"{ctrl}_*.csv"))
    if not files:
        continue
    for f in files:
        if "36" in f.name or "10" in f.name:
            df = pd.read_csv(f)
            ax.plot(df["time"], df["cte"], lw=2, color=COLOURS[ctrl], label=f"{ctrl}")
            break

ax.axhline(0, color="black", lw=0.5)
ax.set_xlabel("Time (s)")
ax.set_ylabel("CTE (m)")
ax.set_title("CTE should stay near zero — compare with 26 m open-loop from Project 3")
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(str(SCRIPT_DIR / "cte_comparison.png"), dpi=150, bbox_inches="tight")
print("Saved cte_comparison.png")

# ── Figure 3: RMS CTE vs speed ────────────────────────────────────────────────

fig3, ax = plt.subplots(figsize=(9, 5))
fig3.suptitle("Project 4 — Speed Sensitivity: RMS CTE per controller", fontsize=13)

for ctrl in CONTROLLERS:
    csvs = sorted(SCRIPT_DIR.glob(f"{ctrl}_*.csv"))
    speeds, rms_ctes = [], []
    for f in csvs:
        df  = pd.read_csv(f)
        spd = int(''.join(filter(str.isdigit, f.stem.split("_")[-1])))
        rms = float(np.sqrt((df["cte"]**2).mean()))
        speeds.append(spd); rms_ctes.append(rms)
    if speeds:
        order = sorted(range(len(speeds)), key=lambda i: speeds[i])
        ax.plot([speeds[i] for i in order], [rms_ctes[i] for i in order],
                "o-", lw=2, color=COLOURS[ctrl], label=ctrl)

ax.set_xlabel("Speed (km/h)")
ax.set_ylabel("RMS CTE (m)")
ax.set_title("All controllers degrade at higher speed — LQR should degrade least")
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(str(SCRIPT_DIR / "speed_sensitivity.png"), dpi=150, bbox_inches="tight")
print("Saved speed_sensitivity.png")

if _SHOW:
    plt.show()
else:
    print(f"\nAll plots saved to: {SCRIPT_DIR}")
    print("Tip: pass --show to open interactive windows.")
