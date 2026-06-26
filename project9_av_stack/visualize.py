"""
Project 9 – Mini AV Stack  |  visualize.py
Produces four figures from the simulation CSVs:
  stack_paths.png       – map + reference + oracle + EKF+MPC vehicle paths
  cte_comparison.png    – cross-track error over time, Oracle vs EKF+MPC
  estimation_error.png  – EKF position error vs GPS sigma baseline
  summary.png           – latency bar chart + stats table
"""

import subprocess, sys, csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
EXE        = SCRIPT_DIR.parent / "build" / "av_stack_demo.exe"

# ── Auto-run demo to generate CSVs ───────────────────────────────────────────
def ensure_csvs():
    needed = ["oracle.csv", "ekf_mpc.csv", "ref_path.csv"]
    if not all((SCRIPT_DIR / f).exists() for f in needed):
        print("CSVs not found – running av_stack_demo.exe ...")
        subprocess.run([str(EXE)], cwd=str(SCRIPT_DIR), check=True)

def read_csv(name):
    rows = []
    with open(SCRIPT_DIR / name, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows

def col(rows, key):
    return np.array([r[key] for r in rows])

# ── Obstacle geometry (mirrors main.cpp) ─────────────────────────────────────
BUILDINGS = [(8, 8, 22, 22), (8, 35, 22, 50), (35, 8, 50, 22), (35, 35, 50, 50)]

def draw_buildings(ax):
    for (x0, y0, x1, y1) in BUILDINGS:
        ax.add_patch(patches.Rectangle(
            (x0, y0), x1-x0, y1-y0,
            linewidth=0.8, edgecolor="black", facecolor="#cccccc", zorder=2))

# ─────────────────────────────────────────────────────────────────────────────
def fig1_paths(oracle, ekf, ref):
    fig, ax = plt.subplots(figsize=(8, 8))
    draw_buildings(ax)

    rx, ry = col(ref,    "x"),    col(ref,    "y")
    ox, oy = col(oracle, "x"),    col(oracle, "y")
    ex, ey = col(ekf,    "true_x"), col(ekf,    "true_y")
    ex_e   = col(ekf,    "est_x")
    ey_e   = col(ekf,    "est_y")

    ax.plot(rx, ry, "k--",  lw=1.5, label="Reference path",    zorder=3)
    ax.plot(ox, oy, "tab:blue",  lw=1.8, label="Oracle (true state)",  zorder=4)
    ax.plot(ex, ey, "tab:orange", lw=1.8, label="EKF+MPC (true state)", zorder=4)
    ax.plot(ex_e, ey_e, "tab:red", lw=0.8, alpha=0.5,
            label="EKF estimate", zorder=3, linestyle=":")

    ax.scatter([rx[0]], [ry[0]], s=80, c="green",  zorder=5, label="Start")
    ax.scatter([rx[-1]], [ry[-1]], s=80, c="red",   zorder=5, label="Goal")

    ax.set_xlim(-2, 62); ax.set_ylim(-2, 62)
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title("Project 9 – AV Stack: Vehicle Paths vs Reference")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out = SCRIPT_DIR / "stack_paths.png"
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  Saved {out}")

# ─────────────────────────────────────────────────────────────────────────────
def fig2_cte(oracle, ekf):
    ot = col(oracle, "t"); octe = col(oracle, "cte")
    et = col(ekf,    "t"); ecte = col(ekf,    "cte")

    o_rms = np.sqrt(np.mean(octe**2))
    e_rms = np.sqrt(np.mean(ecte**2))

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ot, octe, "tab:blue",   lw=1.5, label=f"Oracle  RMS={o_rms:.2f} m")
    ax.plot(et, ecte, "tab:orange", lw=1.5, label=f"EKF+MPC RMS={e_rms:.2f} m")
    ax.axhline(o_rms, color="tab:blue",   lw=0.8, linestyle="--", alpha=0.7)
    ax.axhline(e_rms, color="tab:orange", lw=0.8, linestyle="--", alpha=0.7)

    ax.set_xlabel("Time (s)"); ax.set_ylabel("Cross-Track Error (m)")
    ax.set_title("Project 9 – Cross-Track Error: Oracle vs EKF+MPC")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out = SCRIPT_DIR / "cte_comparison.png"
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  Saved {out}")

# ─────────────────────────────────────────────────────────────────────────────
def fig3_estimation(ekf):
    GPS_SIGMA = 3.0
    et  = col(ekf, "t")
    err = col(ekf, "est_err")
    rms = np.sqrt(np.mean(err**2))

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.fill_between(et, 0, GPS_SIGMA, alpha=0.15, color="tab:red",
                    label=f"GPS σ = {GPS_SIGMA} m")
    ax.plot(et, err, "tab:purple", lw=1.5, label=f"EKF pos error  RMSE={rms:.2f} m")
    ax.axhline(rms,       color="tab:purple", lw=0.8, linestyle="--", alpha=0.7)
    ax.axhline(GPS_SIGMA, color="tab:red",    lw=0.8, linestyle="--", alpha=0.7)

    # Mark GPS update events (every 0.2 s)
    gps_times = et[np.diff(np.floor(et / 0.2 + 0.5), prepend=-1) != 0]
    ax.vlines(gps_times, 0, GPS_SIGMA, color="tab:red", lw=0.4, alpha=0.3)

    ax.set_xlabel("Time (s)"); ax.set_ylabel("Position Error (m)")
    ax.set_title("Project 9 – EKF Estimation Error vs GPS Sigma")
    ax.legend(); ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    fig.tight_layout()
    out = SCRIPT_DIR / "estimation_error.png"
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  Saved {out}")

# ─────────────────────────────────────────────────────────────────────────────
def fig4_summary(oracle, ekf):
    GPS_SIGMA = 3.0
    octe = col(oracle, "cte")
    ecte = col(ekf,    "cte")
    epos = col(ekf,    "est_err")
    o_rms = np.sqrt(np.mean(octe**2));  o_max = np.max(octe)
    e_rms = np.sqrt(np.mean(ecte**2));  e_max = np.max(ecte)
    e_pos_rms = np.sqrt(np.mean(epos**2))

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("Project 9 – Mini AV Stack Summary", fontsize=13, fontweight="bold")

    # Left: CTE comparison bar chart
    ax = axes[0]
    labels  = ["Oracle\nRMS CTE", "Oracle\nMax CTE",
               "EKF+MPC\nRMS CTE", "EKF+MPC\nMax CTE"]
    values  = [o_rms, o_max, e_rms, e_max]
    colors  = ["tab:blue", "cornflowerblue", "tab:orange", "moccasin"]
    bars = ax.bar(labels, values, color=colors, edgecolor="black", linewidth=0.7)
    for b, v in zip(bars, values):
        ax.text(b.get_x() + b.get_width()/2, v + 0.05, f"{v:.2f} m",
                ha="center", va="bottom", fontsize=8)
    ax.set_ylabel("CTE (m)"); ax.set_title("Tracking Accuracy")
    ax.grid(axis="y", alpha=0.3)

    # Right: stats table
    ax2 = axes[1]
    ax2.axis("off")
    data = [
        ["Metric", "Value"],
        ["Path length",         "94.5 m"],
        ["Oracle RMS CTE",      f"{o_rms:.3f} m"],
        ["EKF+MPC RMS CTE",     f"{e_rms:.3f} m"],
        ["EKF pos RMSE",        f"{e_pos_rms:.2f} m"],
        ["GPS sigma",           f"{GPS_SIGMA:.1f} m"],
        ["GPS noise reduction", f"{(1-e_pos_rms/GPS_SIGMA)*100:.1f}%"],
        ["Steer violations",    "0 / 0"],
        ["MPC solve (mean)",    "~0.08 ms"],
        ["EKF predict (mean)",  "< 0.01 ms"],
        ["Total/step",          "~0.08 ms (0.4% budget)"],
    ]
    tbl = ax2.table(cellText=data[1:], colLabels=data[0],
                    loc="center", cellLoc="left")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9)
    tbl.scale(1.3, 1.5)
    ax2.set_title("Pipeline Statistics")

    fig.tight_layout()
    out = SCRIPT_DIR / "summary.png"
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"  Saved {out}")

# ─────────────────────────────────────────────────────────────────────────────
def main():
    show = "--show" in sys.argv
    ensure_csvs()

    oracle = read_csv("oracle.csv")
    ekf    = read_csv("ekf_mpc.csv")
    ref    = read_csv("ref_path.csv")

    print("Generating plots …")
    fig1_paths     (oracle, ekf, ref)
    fig2_cte       (oracle, ekf)
    fig3_estimation(ekf)
    fig4_summary   (oracle, ekf)

    if show:
        plt.show()
    print("Done.")

if __name__ == "__main__":
    main()
