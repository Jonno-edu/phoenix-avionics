#!/usr/bin/env python3
"""
plot_csv_results.py — Phoenix Avionics EKF SIL Diagnostic Dashboard
====================================================================
Visualises the output of the test_csv_runner SIL binary against raw sensor
data.  Modelled after plot_ekf_mission.py with the same visual language:
phase shading, mode-change vertical lines, ±3σ confidence ribbons, and a
log-scale χ² health panel.

Usage (run from the OBC/ workspace root):
    python3 src/modules/estimator/tests/plot_csv_results.py [--save]

    --save   Write the figure to tests/plots/ instead of opening an
             interactive window.

Input files (both relative to this script's directory):
    ekf_output.csv             — written by the run_csv_runner cmake target
    phoenix_sensor_stream.csv  — raw sensor stream from the GSU

Layout (5 subplots, shared x-axis):
    1. Altitude      — raw baro, raw GPS, EKF (−p_D) with ±3σ ribbon;
                       GPS rejection windows shaded in red
    2. Velocity Down — raw GPS vel_d, EKF v_D with ±3σ ribbon
    3. Attitude      — EKF roll / pitch / yaw (deg) + gyro bias twinx
    4. Innovations   — baro and GPS-Down residuals (measured − predicted)
    5. Filter Health — χ² test ratios on a log scale; gate and rejection events

Phase shading on every panel:
    UNINIT/LEVEL/ALIGN/ZVU  →  steel-blue tint  (warmup)
    FLIGHT                  →  green tint        (navigation)

CSV columns expected from test_csv_runner (25 columns):
    time, flight_mode,
    p_N, p_E, p_D, v_N, v_E, v_D,
    roll, pitch, yaw, bg_x, bg_y, bg_z,
    std_pN, std_pE, std_pD, std_vD, std_yaw,
    baro_innov, baro_ratio, baro_rej,
    gps_innov_pD, gps_ratio_pD, gps_rej
"""

import argparse
import sys
import pathlib

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = pathlib.Path(__file__).parent.resolve()
EKF_CSV    = SCRIPT_DIR / "ekf_output.csv"
RAW_CSV    = SCRIPT_DIR / "phoenix_sensor_stream.csv"
PLOTS_DIR  = SCRIPT_DIR / "plots"

# χ² gate — the normalised gate used in the C code is 1.0 (test_ratio > 1.0)
CHI2_GATE = 1.0

FLIGHT_MODE_LABELS = {
    0: "UNINIT",
    1: "LEVEL",
    2: "ALIGN",
    3: "ZVU",
    4: "FLIGHT",
}

# Phase shading palette (matches plot_ekf_mission.py colour language)
_MODE_SHADE = {
    0: ("#B0BEC5", 0.18),   # UNINIT  — grey
    1: ("#BBDEFB", 0.22),   # LEVEL   — blue
    2: ("#BBDEFB", 0.22),   # ALIGN   — blue (same warmup family)
    3: ("#FFF9C4", 0.35),   # ZVU     — yellow
    4: ("#C8E6C9", 0.20),   # FLIGHT  — green
}


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def pressure_to_altitude(pressure_pa: np.ndarray) -> np.ndarray:
    return 44330.0 * (1.0 - (pressure_pa / 101325.0) ** 0.190295)


def load_data() -> tuple[pd.DataFrame, pd.DataFrame]:
    if not EKF_CSV.exists():
        sys.exit(f"[error] EKF output not found: {EKF_CSV}\n"
                 "        Run: cmake --build build_host --target run_csv_runner")
    if not RAW_CSV.exists():
        sys.exit(f"[error] Raw sensor CSV not found: {RAW_CSV}\n"
                 "        Adjust RAW_CSV in this script if your data lives elsewhere.")

    ekf_df = pd.read_csv(EKF_CSV)
    raw_df = pd.read_csv(RAW_CSV)
    raw_df["baro_alt"] = pressure_to_altitude(raw_df["pressure_pa"].to_numpy())
    return raw_df, ekf_df


# ---------------------------------------------------------------------------
# Phase shading helpers  (matches plot_ekf_mission.py _phase_shade pattern)
# ---------------------------------------------------------------------------

def _phase_shade(ax: plt.Axes, ekf_df: pd.DataFrame) -> None:
    """Shade each flight-mode span in the background and label it."""
    mode_col = ekf_df["flight_mode"]
    t_col    = ekf_df["time"]

    # Build contiguous runs of the same flight mode
    changes = mode_col.ne(mode_col.shift()).cumsum()
    for _, grp in ekf_df.groupby(changes):
        mode = int(grp["flight_mode"].iloc[0])
        t0   = float(grp["time"].iloc[0])
        t1   = float(grp["time"].iloc[-1])
        colour, alpha = _MODE_SHADE.get(mode, ("#FFFFFF", 0.0))
        ax.axvspan(t0, t1, facecolor=colour, alpha=alpha, zorder=0)

    # Label each distinct mode span at the top of the axis
    ax.autoscale(axis="y")
    ax.relim()
    ax.autoscale_view()
    ylim = ax.get_ylim()
    label_y = ylim[1] - (ylim[1] - ylim[0]) * 0.02

    seen_modes: set[int] = set()
    for _, grp in ekf_df.groupby(changes):
        mode = int(grp["flight_mode"].iloc[0])
        if mode in seen_modes:
            continue
        seen_modes.add(mode)
        t0 = float(grp["time"].iloc[0])
        t1 = float(grp["time"].iloc[-1])
        ax.text((t0 + t1) / 2, label_y,
                FLIGHT_MODE_LABELS.get(mode, str(mode)),
                ha="center", va="top", fontsize=6,
                color="#555555", style="italic", zorder=6, clip_on=True)


def _shade_all(axes: list[plt.Axes], ekf_df: pd.DataFrame) -> None:
    for ax in axes:
        _phase_shade(ax, ekf_df)


def _mode_change_lines(axes: list[plt.Axes], ekf_df: pd.DataFrame) -> None:
    """Draw a dotted vertical line at every flight-mode transition."""
    diffs = ekf_df["flight_mode"].diff()
    transitions = ekf_df.loc[diffs != 0, "time"].iloc[1:]   # skip t=0
    for t in transitions:
        for ax in axes:
            ax.axvline(t, color="black", linewidth=0.7,
                       linestyle=":", alpha=0.5, zorder=5)


def _gps_rej_shade(ax: plt.Axes, ekf_df: pd.DataFrame) -> None:
    """Shade windows where GPS was rejected (consecutive rows with gps_rej=1)."""
    rej = ekf_df["gps_rej"].astype(int)
    starts = ekf_df.loc[rej.diff() == 1, "time"].tolist()
    ends   = ekf_df.loc[rej.diff() == -1, "time"].tolist()
    if rej.iloc[0] == 1:
        starts.insert(0, float(ekf_df["time"].iloc[0]))
    if rej.iloc[-1] == 1:
        ends.append(float(ekf_df["time"].iloc[-1]))
    for i, (t0, t1) in enumerate(zip(starts, ends)):
        lbl = "GPS Rejected" if i == 0 else "_nolegend_"
        ax.axvspan(t0, t1, facecolor="#EF5350", alpha=0.20,
                   zorder=1, label=lbl)


def _grid(ax: plt.Axes) -> None:
    ax.grid(True, linewidth=0.4, alpha=0.55)


# ---------------------------------------------------------------------------
# Panel helpers
# ---------------------------------------------------------------------------

def plot_altitude(ax: plt.Axes, raw_df: pd.DataFrame, ekf_df: pd.DataFrame) -> None:
    """Altitude tracking: raw baro, raw GPS, EKF ±3σ, GPS rejection shading."""
    t_raw    = raw_df["time"]
    t_ekf    = ekf_df["time"]
    alt_ekf  = -ekf_df["p_D"]
    sigma_pD =  ekf_df["std_pD"].clip(lower=0.0)

    _gps_rej_shade(ax, ekf_df)

    ax.plot(t_raw, raw_df["baro_alt"], label="Raw Baro Alt",
            color="#B0BEC5", linewidth=1.0, zorder=2)
    ax.plot(t_raw, raw_df["alt"],      label="Raw GPS Alt",
            color="#1565C0", alpha=0.45, linewidth=1.0, zorder=3)
    ax.plot(t_ekf, alt_ekf,            label="EKF Alt (−p_D)",
            color="#C62828", linewidth=1.8, zorder=4)
    ax.fill_between(t_ekf,
                    alt_ekf - 3.0 * sigma_pD,
                    alt_ekf + 3.0 * sigma_pD,
                    color="#C62828", alpha=0.15, label="±3σ Bound", zorder=1)

    ax.set_ylabel("Altitude (m)", fontsize=9)
    ax.set_title("Altitude Tracking  |  EKF ±3σ Confidence Ribbon",
                 fontsize=10, fontweight="bold")
    ax.legend(loc="upper left", fontsize=8)
    _grid(ax)


def plot_velocity_down(ax: plt.Axes, raw_df: pd.DataFrame,
                       ekf_df: pd.DataFrame) -> None:
    """Vertical velocity: raw GPS vs EKF with ±3σ envelope."""
    t_raw   = raw_df["time"]
    t_ekf   = ekf_df["time"]
    sigma_v = ekf_df["std_vD"].clip(lower=0.0)

    ax.plot(t_raw, raw_df["vel_d"],  label="Raw GPS Vel D",
            color="#1565C0", alpha=0.45, linewidth=1.0, zorder=3)
    ax.plot(t_ekf, ekf_df["v_D"],   label="EKF Vel D",
            color="#C62828", linewidth=1.8, zorder=4)
    ax.fill_between(t_ekf,
                    ekf_df["v_D"] - 3.0 * sigma_v,
                    ekf_df["v_D"] + 3.0 * sigma_v,
                    color="#C62828", alpha=0.12, label="±3σ Bound", zorder=1)
    ax.axhline(0.0, color="grey", linewidth=0.6, linestyle=":", zorder=2)

    ax.set_ylabel("Velocity D (m/s)", fontsize=9)
    ax.set_title("Vertical Velocity  |  ±3σ Confidence Bounds",
                 fontsize=10, fontweight="bold")
    ax.legend(loc="upper left", fontsize=8)
    _grid(ax)


def plot_attitude_and_bias(ax: plt.Axes, ekf_df: pd.DataFrame) -> None:
    """Attitude angles (deg) on primary axis; gyro biases on secondary."""
    t_ekf  = ekf_df["time"]
    ax_b   = ax.twinx()

    ax.plot(t_ekf, np.degrees(ekf_df["roll"]),  label="Roll",  linewidth=1.4, color="#1565C0")
    ax.plot(t_ekf, np.degrees(ekf_df["pitch"]), label="Pitch", linewidth=1.4, color="#2E7D32")
    ax.plot(t_ekf, np.degrees(ekf_df["yaw"]),   label="Yaw",   linewidth=1.4, color="#6A1B9A")
    ax.axhline(0.0, color="grey", linewidth=0.5, linestyle=":", zorder=1)

    ax_b.plot(t_ekf, ekf_df["bg_x"], label="Gyro Bias X",
              color="darkorange", linestyle="--", linewidth=1.0, alpha=0.8)
    ax_b.plot(t_ekf, ekf_df["bg_y"], label="Gyro Bias Y",
              color="forestgreen", linestyle="--", linewidth=1.0, alpha=0.8)
    ax_b.plot(t_ekf, ekf_df["bg_z"], label="Gyro Bias Z",
              color="saddlebrown", linestyle="--", linewidth=1.0, alpha=0.8)
    ax_b.set_ylabel("Gyro Bias (rad/s)", fontsize=8)

    ax.set_ylabel("Angle (deg)", fontsize=9)
    ax.set_title("Attitude  |  Gyro Bias Convergence (dashed, right axis)",
                 fontsize=10, fontweight="bold")

    lines_a, labels_a = ax.get_legend_handles_labels()
    lines_b, labels_b = ax_b.get_legend_handles_labels()
    ax_b.legend(lines_a + lines_b, labels_a + labels_b,
                loc="upper left", fontsize=8, ncol=2)
    _grid(ax)


def plot_innovations(ax: plt.Axes, ekf_df: pd.DataFrame) -> None:
    """Baro and GPS-Down innovation residuals (measured − predicted)."""
    t_ekf = ekf_df["time"]

    ax.plot(t_ekf, ekf_df["baro_innov"],   label="Baro Innovation (m)",
            color="#6A1B9A", linewidth=1.4, zorder=3)
    ax.plot(t_ekf, ekf_df["gps_innov_pD"], label="GPS Alt Innovation (m)",
            color="#2E7D32", linewidth=1.4, zorder=3)
    ax.axhline(0.0, color="black", linestyle="--", linewidth=0.7, zorder=2)

    ax.set_ylabel("Innovation (m)", fontsize=9)
    ax.set_title("Sensor Innovations  (Measured − Predicted)",
                 fontsize=10, fontweight="bold")
    ax.legend(loc="upper left", fontsize=8)
    _grid(ax)


def plot_chi2(ax: plt.Axes, ekf_df: pd.DataFrame) -> None:
    """χ² test ratios on a log scale; gate line; rejection events marked."""
    t_ekf = ekf_df["time"]

    # Only plot non-zero samples so log scale doesn't explode on all-zero runs
    baro_nz = ekf_df[ekf_df["baro_ratio"] > 0]
    gps_nz  = ekf_df[ekf_df["gps_ratio_pD"] > 0]

    if not baro_nz.empty:
        ax.plot(baro_nz["time"], baro_nz["baro_ratio"],
                color="#6A1B9A", linewidth=0.9, alpha=0.8,
                label="Baro χ²", zorder=3)
    if not gps_nz.empty:
        ax.plot(gps_nz["time"], gps_nz["gps_ratio_pD"],
                color="#2E7D32", linewidth=1.0, alpha=0.85,
                marker=".", markersize=2.5, linestyle="none",
                label="GPS-Down χ²", zorder=3)

    ax.axhline(CHI2_GATE, color="#C62828", linewidth=1.5,
               linestyle="--", label=f"Rejection gate  (χ² = {CHI2_GATE:.1f})", zorder=4)

    # Scatter exact rejection events above the gate line
    baro_rejs = ekf_df[ekf_df["baro_rej"] == 1]
    gps_rejs  = ekf_df[ekf_df["gps_rej"]  == 1]
    if not baro_rejs.empty:
        ax.scatter(baro_rejs["time"], baro_rejs["baro_ratio"].clip(lower=CHI2_GATE * 1.05),
                   color="#6A1B9A", marker="x", s=35, zorder=5,
                   label=f"Baro rejected  ({len(baro_rejs)})")
    if not gps_rejs.empty:
        ax.scatter(gps_rejs["time"], gps_rejs["gps_ratio_pD"].clip(lower=CHI2_GATE * 1.05),
                   color="#2E7D32", marker="x", s=35, zorder=5,
                   label=f"GPS rejected  ({len(gps_rejs)})")

    ax.set_yscale("log")
    non_zero_vals = pd.concat([baro_nz["baro_ratio"], gps_nz["gps_ratio_pD"]])
    if not non_zero_vals.empty:
        ax.set_ylim(bottom=max(1e-4, non_zero_vals.quantile(0.01) * 0.5),
                    top=non_zero_vals.quantile(0.999) * 3.0)
    ax.yaxis.set_major_formatter(ticker.FuncFormatter(
        lambda v, _: f"{v:.3g}"
    ))

    ax.set_xlabel("Time (s)", fontsize=9)
    ax.set_ylabel("χ² Test Ratio", fontsize=9)
    ax.set_title("Filter Health — Innovation χ²  (log scale  |  gate = 1.0)",
                 fontsize=10, fontweight="bold")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, which="both", linestyle="--", linewidth=0.3, alpha=0.5)


# ---------------------------------------------------------------------------
# Console statistics
# ---------------------------------------------------------------------------

def print_statistics(raw_df: pd.DataFrame, ekf_df: pd.DataFrame) -> None:
    t_end    = ekf_df["time"].iloc[-1]
    n_rows   = len(ekf_df)
    max_mode = int(ekf_df["flight_mode"].max())

    flight_df = ekf_df[ekf_df["flight_mode"] == 4]

    baro_total = (ekf_df["baro_ratio"] > 0).sum()
    baro_rejs  = (ekf_df["baro_rej"] == 1).sum()
    gps_total  = (ekf_df["gps_ratio_pD"] > 0).sum()
    gps_rejs   = (ekf_df["gps_rej"] == 1).sum()

    final_bg = ekf_df[["bg_x", "bg_y", "bg_z"]].iloc[-1]
    final_std_pD = ekf_df["std_pD"].iloc[-1]
    final_std_vD = ekf_df["std_vD"].iloc[-1]

    print()
    print("┌─ EKF SIL CSV Runner — Statistics ──────────────────────────────────┐")
    print(f"│  Duration:           {t_end:.1f} s  ({n_rows:,} EKF steps)")
    print(f"│  Final flight mode:  {FLIGHT_MODE_LABELS.get(max_mode, str(max_mode))}")
    print(f"│  Flight-mode rows:   {len(flight_df):,}")
    print(f"│")
    print(f"│  Baro fused:         {baro_total:,}  →  {baro_rejs} rejected"
          f"  ({100*baro_rejs/max(baro_total,1):.1f}%)")
    print(f"│  GPS-D fused:        {gps_total:,}  →  {gps_rejs} rejected"
          f"  ({100*gps_rejs/max(gps_total,1):.1f}%)")
    if baro_total > 0:
        peak_baro = ekf_df["baro_ratio"].max()
        print(f"│  Baro χ² peak:       {peak_baro:.4f}")
    if gps_total > 0:
        peak_gps = ekf_df["gps_ratio_pD"].max()
        print(f"│  GPS-D χ² peak:      {peak_gps:.4f}")
    print(f"│")
    print(f"│  Final σ(pos D):     {final_std_pD:.4f} m")
    print(f"│  Final σ(vel D):     {final_std_vD:.4f} m/s")
    print(f"│  Final gyro bias:    X={final_bg['bg_x']:.5f}  "
          f"Y={final_bg['bg_y']:.5f}  Z={final_bg['bg_z']:.5f}  rad/s")
    print("└────────────────────────────────────────────────────────────────────┘")
    print()


# ---------------------------------------------------------------------------
# Master plot
# ---------------------------------------------------------------------------

def build_title(ekf_df: pd.DataFrame) -> str:
    max_mode = ekf_df["flight_mode"].max()
    mag_tag  = "MAG" if max_mode >= 2 else "no-MAG"
    has_baro = (ekf_df["baro_ratio"] > 0).any()
    baro_tag = "BARO" if has_baro else "no-BARO"
    has_gps  = (ekf_df["gps_ratio_pD"] > 0).any()
    gps_tag  = "GPS" if has_gps else "no-GPS"
    t_end    = ekf_df["time"].iloc[-1]
    return (
        f"Phoenix Avionics EKF — SIL Diagnostic Dashboard\n"
        f"{t_end:.0f} s  |  250 Hz IMU  |  [{gps_tag} | {baro_tag} | {mag_tag}]"
    )


def plot(raw_df: pd.DataFrame, ekf_df: pd.DataFrame, save: bool) -> None:
    print_statistics(raw_df, ekf_df)

    fig, axs = plt.subplots(5, 1, figsize=(14, 20), sharex=True)
    fig.suptitle(build_title(ekf_df), fontsize=13, fontweight="bold", y=0.995)

    plot_altitude        (axs[0], raw_df, ekf_df)
    plot_velocity_down   (axs[1], raw_df, ekf_df)
    plot_attitude_and_bias(axs[2], ekf_df)
    plot_innovations     (axs[3], ekf_df)
    plot_chi2            (axs[4], ekf_df)

    # Phase shading on every panel (must run after data is plotted so ylim is set)
    for ax in axs:
        ax.autoscale(axis="y")
        ax.relim()
        ax.autoscale_view()
    _shade_all(list(axs), ekf_df)

    # Mode-change vertical dotted lines across all panels
    _mode_change_lines(list(axs), ekf_df)

    plt.tight_layout(rect=[0, 0, 1, 0.993])

    if save:
        PLOTS_DIR.mkdir(parents=True, exist_ok=True)
        outfile = PLOTS_DIR / "csv_runner_results.png"
        fig.savefig(outfile, dpi=150, bbox_inches="tight")
        print(f"[plot] Saved → {outfile}")
    else:
        try:
            fig.canvas.manager.set_window_title("Phoenix Avionics EKF SIL Diagnostics")
        except Exception:
            pass
        plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Plot EKF SIL diagnostic dashboard.")
    parser.add_argument("--save", action="store_true",
                        help="Save figure to tests/plots/ instead of showing.")
    args = parser.parse_args()

    raw_df, ekf_df = load_data()
    plot(raw_df, ekf_df, save=args.save)


if __name__ == "__main__":
    main()
