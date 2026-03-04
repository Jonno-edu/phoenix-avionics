#!/usr/bin/env python3
"""
plot_ekf_mission.py — Phoenix Avionics EKF Mission Dashboard

Reads the CSV exported by `cmake --build build_host --target run_ekf_export`
and produces a 5-panel diagnostic figure that visualises:

  Panel 1 — Altitude tracking: truth vs EKF vs raw baro (with 200 m fault)
  Panel 2 — Vertical velocity error with ±3σ covariance bounds
  Panel 3 — Gyro bias X convergence (truth vs EKF)
  Panel 4 — Accel bias X convergence (truth vs EKF)
  Panel 5 — Innovation χ² for baro and GPS Z (log scale, gate at 9.0)

Usage
─────
    # 1. Generate the data (includes 200 m baro fault at T=30–31 s)
    cmake --build build_host --target run_ekf_export

    # 2. Plot interactively (from workspace root)
    python3 scripts/plot_ekf_mission.py

    # 3. Point at a different CSV
    python3 scripts/plot_ekf_mission.py --csv path/to/ekf_mission_data.csv

    # 4. Save to PNG (for CI artefacts / PR comments)
    python3 scripts/plot_ekf_mission.py --save ekf_mission.png
"""

import argparse
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd

# ── Constants ────────────────────────────────────────────────────────────────

# χ² rejection threshold for a 1-DOF measurement at 99.7% confidence
CHI2_GATE = 9.0

# EKF flight mode labels (must match ekf_core.h ekf_flight_mode_t)
MODE_LABELS = {
    0: "UNINITIALIZED",
    1: "LEVELING",
    2: "HEADING_ALIGN",
    3: "ZVU_CALIBRATING",
    4: "FLIGHT",
}

# Fault window (must match test_ekf_export.c)
FAULT_T_START = 30.0
FAULT_T_END   = 31.0


# ── Data loading ─────────────────────────────────────────────────────────────

def load_csv(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)

    # Altitude (positive-up) is more intuitive than NED Z (positive-down)
    df["alt_true"] = -df["true_pz"]
    df["alt_ekf"]  = -df["ekf_pz"]
    # raw_baro is already positive-up altitude

    # Errors: EKF − truth
    df["err_vz"] = df["ekf_vz"] - df["true_vz"]

    # 3σ bounds from covariance diagonal
    df["3sig_vz"] = 3.0 * np.sqrt(df["cov_vz"].clip(lower=0))

    return df


# ── Phase shading ─────────────────────────────────────────────────────────────

def _phase_shade(ax: plt.Axes, df: pd.DataFrame) -> None:
    t0      = df["time_s"].iloc[0]
    t_end   = df["time_s"].iloc[-1]
    t_burn  = df.loc[df["time_s"] >= 15.0, "time_s"].iloc[0]
    t_coast = df.loc[df["time_s"] >= 20.0, "time_s"].iloc[0]

    spans = [
        (t0,      t_burn,  "#BBDEFB", 0.20, "PAD"),
        (t_burn,  t_coast, "#FFCCBC", 0.30, "BURN"),
        (t_coast, t_end,   "#C8E6C9", 0.15, "COAST"),
    ]
    ylim = ax.get_ylim()
    for x0, x1, colour, alpha, label in spans:
        ax.axvspan(x0, x1, facecolor=colour, alpha=alpha, zorder=0)
        ax.text((x0 + x1) / 2, ylim[1], label,
                ha="center", va="top", fontsize=7,
                color="grey", style="italic", zorder=6)


def _shade_all(axes, df):
    for ax in axes:
        _phase_shade(ax, df)


# ── Panel helpers ─────────────────────────────────────────────────────────────

def _grid(ax):
    ax.grid(True, linewidth=0.4, alpha=0.6)


def plot_altitude(ax: plt.Axes, df: pd.DataFrame) -> None:
    """Truth, EKF estimate, and raw baro (shows the 200 m fault spike)."""
    ax.plot(df["time_s"], df["alt_true"],
            color="black", linewidth=1.4, linestyle="--",
            label="True Altitude", zorder=4)

    # Raw baro: only 50 Hz samples are non-NaN — plot as thin scatter
    baro_df = df.dropna(subset=["raw_baro"])
    ax.plot(baro_df["time_s"], baro_df["raw_baro"],
            color="red", linewidth=0.8, alpha=0.45,
            label="Raw Baro (with fault)", zorder=2)

    ax.plot(df["time_s"], df["alt_ekf"],
            color="#1565C0", linewidth=1.2, alpha=0.9,
            label="EKF Estimate", zorder=3)

    # Annotate the fault window
    ax.axvspan(FAULT_T_START, FAULT_T_END,
               facecolor="red", alpha=0.12, zorder=1, label="_nolegend_")
    ymax = ax.get_ylim()[1]
    ax.annotate("200 m fault\n(rejected)",
                xy=((FAULT_T_START + FAULT_T_END) / 2, ymax * 0.55),
                ha="center", fontsize=8, color="darkred",
                arrowprops=dict(arrowstyle="-", color="darkred", alpha=0.5))

    # Shade GPS lock-loss windows (grey hatching) if the column is present
    if "gps_lock" in df.columns:
        # Find contiguous runs of gps_lock == 0
        lock_series = df["gps_lock"].fillna(1).astype(int)
        changes = lock_series.diff().fillna(0)
        loss_starts  = df.loc[changes ==  -1, "time_s"].tolist()  # 1→0
        loss_ends    = df.loc[changes ==  +1, "time_s"].tolist()  # 0→1
        # Handle the case where data starts during a loss
        if lock_series.iloc[0] == 0:
            loss_starts.insert(0, float(df["time_s"].iloc[0]))
        # Handle the case where data ends during a loss
        if lock_series.iloc[-1] == 0:
            loss_ends.append(float(df["time_s"].iloc[-1]))
        for i, (t0, t1) in enumerate(zip(loss_starts, loss_ends)):
            label = "GPS loss of lock" if i == 0 else "_nolegend_"
            ax.axvspan(t0, t1,
                       facecolor="#9E9E9E", alpha=0.30, zorder=1,
                       hatch="//", label=label)

    ax.set_ylabel("Altitude (m)", fontsize=9)
    ax.set_title("Altitude Tracking & Outlier Rejection", fontsize=10, fontweight="bold")
    ax.legend(loc="upper left", fontsize=8)
    _grid(ax)


def plot_vel_error(ax: plt.Axes, df: pd.DataFrame) -> None:
    """Vertical velocity error bounded by ±3σ covariance envelope."""
    ax.plot(df["time_s"], df["err_vz"],
            color="#1565C0", linewidth=1.0,
            label="Vel-Z error (m/s)", zorder=3)
    ax.plot(df["time_s"],  df["3sig_vz"],
            "r--", linewidth=0.9, alpha=0.8, label="+3σ bound", zorder=2)
    ax.plot(df["time_s"], -df["3sig_vz"],
            "r--", linewidth=0.9, alpha=0.8, label="−3σ bound", zorder=2)
    ax.fill_between(df["time_s"], df["3sig_vz"], -df["3sig_vz"],
                    color="red", alpha=0.10, zorder=1)
    ax.axhline(0, color="grey", linewidth=0.6, linestyle=":")

    ax.set_ylabel("Vel-Z error (m/s)", fontsize=9)
    ax.set_title("Vertical Velocity Error  ±3σ Confidence Bounds", fontsize=10, fontweight="bold")
    ax.legend(loc="upper right", fontsize=8)
    _grid(ax)


def plot_gyro_bias(ax: plt.Axes, df: pd.DataFrame) -> None:
    """Gyro bias X convergence — truth vs EKF estimate."""
    ax.plot(df["time_s"], df["true_gbx"],
            color="black", linewidth=1.2, linestyle="--",
            label="True Gyro Bias X", zorder=3)
    ax.plot(df["time_s"], df["ekf_gbx"],
            color="#6A1B9A", linewidth=1.0, alpha=0.9,
            label="EKF Gyro Bias X", zorder=2)

    ax.set_ylabel("Gyro Bias (rad/s)", fontsize=9)
    ax.set_title("Gyro Bias X Convergence", fontsize=10, fontweight="bold")
    ax.legend(loc="upper right", fontsize=8)
    _grid(ax)


def plot_accel_bias(ax: plt.Axes, df: pd.DataFrame) -> None:
    """Accel bias X convergence — truth vs EKF estimate."""
    ax.plot(df["time_s"], df["true_abx"],
            color="black", linewidth=1.2, linestyle="--",
            label="True Accel Bias X", zorder=3)
    ax.plot(df["time_s"], df["ekf_abx"],
            color="#E65100", linewidth=1.0, alpha=0.9,
            label="EKF Accel Bias X", zorder=2)

    ax.set_ylabel("Accel Bias (m/s²)", fontsize=9)
    ax.set_title("Accelerometer Bias X Convergence", fontsize=10, fontweight="bold")
    ax.legend(loc="upper right", fontsize=8)
    _grid(ax)


def plot_chi2(ax: plt.Axes, df: pd.DataFrame) -> None:
    """Innovation χ² for baro and GPS on a log scale.

    During the 200 m fault window the baro χ² rockets to ~10⁸, making the
    rejection gate at 9.0 perfectly visible on the log axis.
    GPS χ² is absent entirely during the lock-loss window.
    """
    baro_df = df.dropna(subset=["baro_chi2"])
    gps_df  = df.dropna(subset=["gps_chi2"])

    ax.plot(baro_df["time_s"], baro_df["baro_chi2"],
            color="#C62828", linewidth=0.9, alpha=0.75,
            label="Baro χ²", zorder=3)
    ax.plot(gps_df["time_s"], gps_df["gps_chi2"],
            color="#2E7D32", linewidth=1.0, alpha=0.8,
            marker=".", markersize=3, linestyle="none",
            label="GPS Z-pos χ²", zorder=3)

    ax.axhline(CHI2_GATE, color="black", linewidth=1.2,
               linestyle=":", label=f"Rejection gate  (χ² = {CHI2_GATE:.0f})", zorder=4)

    # Highlight the fault window
    ax.axvspan(FAULT_T_START, FAULT_T_END,
               facecolor="red", alpha=0.10, zorder=1)

    # Shade GPS lock-loss windows to explain the χ² gap
    if "gps_lock" in df.columns:
        lock_series = df["gps_lock"].fillna(1).astype(int)
        changes = lock_series.diff().fillna(0)
        loss_starts = df.loc[changes == -1, "time_s"].tolist()
        loss_ends   = df.loc[changes == +1, "time_s"].tolist()
        if lock_series.iloc[0] == 0:
            loss_starts.insert(0, float(df["time_s"].iloc[0]))
        if lock_series.iloc[-1] == 0:
            loss_ends.append(float(df["time_s"].iloc[-1]))
        for t0, t1 in zip(loss_starts, loss_ends):
            ax.axvspan(t0, t1, facecolor="#9E9E9E", alpha=0.25, zorder=1)

    ax.set_yscale("log")
    ax.set_ylim(bottom=0.01)
    ax.set_xlabel("Time (s)", fontsize=9)
    ax.set_ylabel("Innovation χ²", fontsize=9)
    ax.set_title("Filter Health — Innovation χ²  (log scale,  gate = 9.0)",
                 fontsize=10, fontweight="bold")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, which="both", linestyle="--", linewidth=0.3, alpha=0.55)


# ── Master plot ───────────────────────────────────────────────────────────────

def plot_ekf_results(csv_path: Path, save_path: Path | None = None) -> None:
    print(f"[plot] Loading: {csv_path}")
    df = load_csv(csv_path)
    n_rows = len(df)
    t_end  = df["time_s"].iloc[-1]
    print(f"[plot] {n_rows:,} rows  ({t_end:.1f} s)")

    # Summary statistics (coast phase only)
    coast = df[df["time_s"] >= 20.0]
    pos_rms = float(np.sqrt(((coast["ekf_pz"] - coast["true_pz"]) ** 2).mean()))
    vel_rms = float(np.sqrt((coast["err_vz"] ** 2).mean()))
    baro_df = df.dropna(subset=["baro_chi2"])
    reject_pct = (baro_df["baro_chi2"] > CHI2_GATE).mean() * 100
    print(f"\n[plot] End-of-mission statistics:")
    print(f"         Final mode:             {MODE_LABELS.get(int(df['mode'].iloc[-1]), '?')}")
    print(f"         Coast RMS pos-Z error:  {pos_rms:.3f} m")
    print(f"         Coast RMS vel-Z error:  {vel_rms:.4f} m/s")
    print(f"         Baro rejection rate:    {reject_pct:.1f}%")
    fault_df = baro_df[(baro_df["time_s"] >= FAULT_T_START) &
                       (baro_df["time_s"] <  FAULT_T_END)]
    if not fault_df.empty:
        print(f"         Fault window χ² peak:   {fault_df['baro_chi2'].max():.2e}")
        print(f"         Fault measurements:     {len(fault_df)}  "
              f"({(fault_df['baro_chi2'] > CHI2_GATE).sum()} / {len(fault_df)} rejected)")
    if "gps_lock" in df.columns:
        lock_series = df["gps_lock"].fillna(1).astype(int)
        loss_pct    = (lock_series == 0).mean() * 100
        loss_dur    = (lock_series == 0).sum() * float(df["time_s"].iloc[1] - df["time_s"].iloc[0])
        print(f"         GPS lock-loss:          {loss_pct:.1f}% of flight  ({loss_dur:.1f} s total)")

    # ── Layout ───────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    fig.suptitle(
        "Phoenix Avionics — EKF Mission Dashboard\n"
        f"60 s  |  250 Hz IMU  |  GPS @ 10 Hz  |  Baro @ 50 Hz  "
        f"|  200 m fault @ T=30–31 s  |  Source: {csv_path.name}",
        fontsize=10, fontweight="bold", y=0.995,
    )

    plot_altitude  (axes[0], df)
    plot_vel_error (axes[1], df)
    plot_gyro_bias (axes[2], df)
    plot_accel_bias(axes[3], df)
    plot_chi2      (axes[4], df)

    # Phase shading
    for ax in axes:
        ax.autoscale(axis="y")
        ax.relim()
        ax.autoscale_view()
    _shade_all(axes, df)

    # Mode-change vertical lines
    mode_changes = df[df["mode"].diff() != 0]
    for _, row in mode_changes.iterrows():
        if row["mode"] > 0:
            for ax in axes:
                ax.axvline(row["time_s"], color="black",
                           linewidth=0.7, linestyle=":", alpha=0.45, zorder=5)

    plt.tight_layout(rect=[0, 0, 1, 0.992])

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"\n[plot] Saved: {save_path}")
    else:
        try:
            fig.canvas.manager.set_window_title("Phoenix Avionics EKF Telemetry")
        except Exception:
            pass
        plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────

def _find_csv() -> Path:
    """Resolve default CSV path relative to this script's workspace root."""
    # Script lives at OBC/scripts/, CSV is typically at OBC/build_host/
    script_dir  = Path(__file__).resolve().parent
    workspace   = script_dir.parent         # OBC/
    candidates  = [
        workspace / "build_host" / "ekf_mission_data.csv",
        Path("build_host") / "ekf_mission_data.csv",
        Path("ekf_mission_data.csv"),
    ]
    for p in candidates:
        if p.exists():
            return p
    print(
        "[plot] ERROR: could not find ekf_mission_data.csv\n"
        "       Run:  cmake --build build_host --target run_ekf_export\n"
        "       Then: python3 scripts/plot_ekf_mission.py",
        file=sys.stderr,
    )
    sys.exit(1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot EKF mission data exported by run_ekf_export.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--csv", type=Path, default=None,
        help="Path to ekf_mission_data.csv (default: auto-detected in build_host/)",
    )
    parser.add_argument(
        "--save", type=Path, default=None,
        metavar="OUT.png",
        help="Save figure to file instead of displaying interactively.",
    )
    args = parser.parse_args()

    csv_path = args.csv if args.csv else _find_csv()
    plot_ekf_results(csv_path, save_path=args.save)


if __name__ == "__main__":
    main()
