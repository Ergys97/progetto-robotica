#!/usr/bin/env python3
"""Generate static figures for the final robotics project report."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
FIG_DIR = ROOT / "docs" / "report" / "figures"
FLAT_SUMMARY = ROOT / "docs" / "results" / "flat" / "bag_20260615_160043_metrics.json"
OBSTACLE_SUMMARY = ROOT / "docs" / "results" / "obstacle" / "bag_20260615_164534_metrics.json"
FLAT_TELEMETRY = Path("/home/ergys/progetto_robotica_bags/bag_20260615_160043.csv")
OBSTACLE_TELEMETRY = Path("/home/ergys/progetto_robotica_bags/bag_20260615_164534.csv")

TOKENS = {
    "surface": "#FCFCFD",
    "panel": "#FFFFFF",
    "ink": "#1F2430",
    "muted": "#6F768A",
    "grid": "#E6E8F0",
    "axis": "#D7DBE7",
    "blue_base": "#A3BEFA",
    "blue_mid": "#5477C4",
    "blue_dark": "#2E4780",
    "gold_base": "#FFE15B",
    "gold_mid": "#B8A037",
    "orange_base": "#F0986E",
    "orange_mid": "#CC6F47",
    "olive_base": "#A3D576",
    "olive_mid": "#71B436",
    "pink_base": "#F390CA",
    "pink_mid": "#BD569B",
}


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def read_telemetry(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append({key: float(value) for key, value in row.items()})
    return rows


def configure_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": TOKENS["surface"],
            "axes.facecolor": TOKENS["panel"],
            "axes.edgecolor": TOKENS["axis"],
            "axes.labelcolor": TOKENS["ink"],
            "xtick.color": TOKENS["muted"],
            "ytick.color": TOKENS["muted"],
            "text.color": TOKENS["ink"],
            "font.family": ["DejaVu Sans", "sans-serif"],
            "axes.titleweight": "bold",
            "axes.titlelocation": "left",
            "savefig.facecolor": TOKENS["surface"],
            "savefig.bbox": "tight",
            "savefig.dpi": 180,
        }
    )


def add_header(fig: plt.Figure, title: str, subtitle: str) -> None:
    fig.text(0.08, 0.955, title, fontsize=14, weight="bold", ha="left", va="top")
    fig.text(0.08, 0.905, subtitle, fontsize=9.5, color=TOKENS["muted"], ha="left", va="top")


def save(fig: plt.Figure, name: str) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG_DIR / f"{name}.png")
    fig.savefig(FIG_DIR / f"{name}.svg")
    plt.close(fig)


def clean_axes(ax: plt.Axes, *, x_grid: bool = False, y_grid: bool = True) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(TOKENS["axis"])
    ax.spines["bottom"].set_color(TOKENS["axis"])
    if y_grid:
        ax.grid(axis="y", color=TOKENS["grid"], linewidth=0.8)
    if x_grid:
        ax.grid(axis="x", color=TOKENS["grid"], linewidth=0.8)
    ax.set_axisbelow(True)


def draw_box(ax: plt.Axes, xy: tuple[float, float], width: float, height: float, title: str, body: str, color: str) -> None:
    patch = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle="round,pad=0.018,rounding_size=0.025",
        facecolor=color,
        edgecolor=TOKENS["blue_dark"],
        linewidth=1.1,
    )
    ax.add_patch(patch)
    ax.text(xy[0] + width / 2, xy[1] + height * 0.64, title, ha="center", va="center", fontsize=10.5, weight="bold")
    ax.text(xy[0] + width / 2, xy[1] + height * 0.34, body, ha="center", va="center", fontsize=8.3, color=TOKENS["muted"])


def draw_arrow(ax: plt.Axes, start: tuple[float, float], end: tuple[float, float], label: str) -> None:
    arrow = FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=12, linewidth=1.2, color=TOKENS["blue_dark"])
    ax.add_patch(arrow)
    mid_x = (start[0] + end[0]) / 2
    mid_y = (start[1] + end[1]) / 2
    ax.text(mid_x, mid_y + 0.035, label, ha="center", va="center", fontsize=8.2, color=TOKENS["blue_dark"])


def figure_architecture() -> None:
    fig, ax = plt.subplots(figsize=(9.8, 5.1))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    add_header(
        fig,
        "Architettura ROS 2 del prototipo",
        "Flusso tra input utente, dashboard, simulazione MuJoCo, rosbag2 e replay_eval.",
    )

    draw_box(ax, (0.05, 0.56), 0.18, 0.16, "Input utente", "Tastiera / gamepad", TOKENS["gold_base"])
    draw_box(ax, (0.32, 0.56), 0.22, 0.16, "web_teleop", "Flask-SocketIO\nDashboard realtime", TOKENS["blue_base"])
    draw_box(ax, (0.68, 0.56), 0.24, 0.16, "mujoco_sim", "Unitree G1\nMuJoCo + policy", TOKENS["olive_base"])
    draw_box(ax, (0.68, 0.24), 0.24, 0.14, "rosbag2", "Topic registrati\nper metriche", TOKENS["orange_base"])
    draw_box(ax, (0.32, 0.24), 0.22, 0.14, "replay_eval", "Replay e MSE\ntraiettoria", TOKENS["pink_base"])

    draw_arrow(ax, (0.23, 0.64), (0.32, 0.64), "eventi input")
    draw_arrow(ax, (0.54, 0.64), (0.68, 0.64), "/cmd_vel")
    draw_arrow(ax, (0.68, 0.59), (0.54, 0.59), "telemetria")
    draw_arrow(ax, (0.80, 0.56), (0.80, 0.38), "topic")
    draw_arrow(ax, (0.68, 0.31), (0.54, 0.31), "bag")

    ax.text(
        0.50,
        0.48,
        "/imu  /joint_states  /odom  /contacts/*  /fall_detected  /metrics/cmd_latency_ms",
        ha="center",
        va="center",
        fontsize=8.5,
        color=TOKENS["muted"],
    )
    save(fig, "architettura_ros2")


def figure_latency(flat: dict, obstacle: dict) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    add_header(
        fig,
        "Latenza comando sotto target salvo picco obstacle",
        "Media e p95 restano sotto 50 ms; il massimo nello scenario obstacle e un outlier isolato.",
    )
    fig.subplots_adjust(top=0.78, left=0.10, right=0.97, bottom=0.16)

    labels = ["Media", "P95", "Massimo"]
    x = range(len(labels))
    width = 0.34
    flat_values = [flat["latency_ms_avg"], flat["latency_ms_p95"], flat["latency_ms_max"]]
    obs_values = [obstacle["latency_ms_avg"], obstacle["latency_ms_p95"], obstacle["latency_ms_max"]]

    ax.bar([i - width / 2 for i in x], flat_values, width, label="flat", color=TOKENS["blue_base"], edgecolor=TOKENS["blue_dark"])
    ax.bar([i + width / 2 for i in x], obs_values, width, label="obstacle_course", color=TOKENS["orange_base"], edgecolor=TOKENS["orange_mid"])
    ax.axhline(50, color=TOKENS["ink"], linestyle="--", linewidth=1.1)
    ax.text(2.43, 50, "target 50 ms", va="bottom", ha="right", fontsize=8.5, color=TOKENS["ink"])

    for i, value in enumerate(flat_values):
        ax.text(i - width / 2, value + 8, f"{value:.1f}", ha="center", va="bottom", fontsize=8.2)
    for i, value in enumerate(obs_values):
        ax.text(i + width / 2, value + 8, f"{value:.1f}", ha="center", va="bottom", fontsize=8.2)

    ax.set_ylabel("Latenza comando (ms)")
    ax.set_xticks(list(x), labels)
    ax.set_ylim(0, max(obs_values) * 1.18)
    ax.legend(loc="upper left", frameon=False, ncols=2)
    clean_axes(ax)
    save(fig, "latenza_comandi")


def figure_events(obstacle: dict) -> None:
    fig, ax = plt.subplots(figsize=(8.6, 4.6))
    add_header(
        fig,
        "Lo scenario obstacle attiva gli alert attesi",
        "Conteggi aggregati della sessione bag_20260615_164534: caduta, flight phase e perdite di contatto.",
    )
    fig.subplots_adjust(top=0.77, left=0.15, right=0.96, bottom=0.17)

    labels = ["Fall detection", "Flight phase", "Perdita contatto L", "Perdita contatto R"]
    values = [
        obstacle["fall_detected_count"],
        obstacle["flight_phase_count"],
        obstacle["left_contact_loss_count"],
        obstacle["right_contact_loss_count"],
    ]
    colors = [TOKENS["pink_base"], TOKENS["gold_base"], TOKENS["blue_base"], TOKENS["olive_base"]]
    edge_colors = [TOKENS["pink_mid"], TOKENS["gold_mid"], TOKENS["blue_dark"], TOKENS["olive_mid"]]
    bars = ax.barh(labels, values, color=colors, edgecolor=edge_colors)

    for bar, value in zip(bars, values):
        ax.text(value + 3, bar.get_y() + bar.get_height() / 2, f"{value}", va="center", fontsize=9)

    ax.set_xlabel("Numero eventi")
    ax.set_xlim(0, max(values) * 1.20)
    clean_axes(ax, x_grid=True, y_grid=False)
    save(fig, "eventi_obstacle")


def roll_pitch_from_quaternion(w: float, x: float, y: float, z: float) -> tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    return math.degrees(roll), math.degrees(pitch)


def downsample(rows: list[dict[str, float]], max_points: int = 850) -> Iterable[dict[str, float]]:
    step = max(1, len(rows) // max_points)
    return rows[::step]


def split_segments(rows: list[dict[str, float]], *, max_gap_s: float = 0.05, max_z_jump_m: float = 0.25) -> list[list[dict[str, float]]]:
    segments: list[list[dict[str, float]]] = []
    current: list[dict[str, float]] = []
    previous: dict[str, float] | None = None

    for row in rows:
        is_break = False
        if previous is not None:
            is_break = (
                row["sim_time"] < previous["sim_time"]
                or row["sim_time"] - previous["sim_time"] > max_gap_s
                or abs(row["pos_z"] - previous["pos_z"]) > max_z_jump_m
            )
        if is_break and current:
            segments.append(current)
            current = []
        current.append(row)
        previous = row

    if current:
        segments.append(current)
    return segments


def figure_obstacle_timeseries(rows: list[dict[str, float]]) -> None:
    fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(9.2, 5.8), sharex=True)
    add_header(
        fig,
        "Assetto e quota evidenziano gli eventi critici",
        "Scenario obstacle: segmenti registrati in ordine, separati nei punti di reset/discontinuita.",
    )
    fig.subplots_adjust(top=0.80, left=0.10, right=0.97, bottom=0.12, hspace=0.30)

    label_roll = True
    label_pitch = True
    offset = 0.0
    segment_gap_s = 1.0
    for segment in split_segments(rows):
        sampled = list(downsample(segment, max_points=180))
        segment_start = segment[0]["sim_time"]
        times = [row["sim_time"] - segment_start + offset for row in sampled]
        z_values = [row["pos_z"] for row in sampled]
        rolls: list[float] = []
        pitches: list[float] = []
        for row in sampled:
            roll, pitch = roll_pitch_from_quaternion(row["qpos_3"], row["qpos_4"], row["qpos_5"], row["qpos_6"])
            rolls.append(roll)
            pitches.append(pitch)

        ax_top.plot(times, rolls, color=TOKENS["blue_mid"], linewidth=1.2, label="roll" if label_roll else None)
        ax_top.plot(times, pitches, color=TOKENS["orange_mid"], linewidth=1.2, label="pitch" if label_pitch else None)
        ax_bottom.plot(times, z_values, color=TOKENS["olive_mid"], linewidth=1.3)
        label_roll = False
        label_pitch = False
        offset += segment[-1]["sim_time"] - segment_start + segment_gap_s

    ax_top.axhline(35, color=TOKENS["ink"], linestyle="--", linewidth=0.9)
    ax_top.axhline(-35, color=TOKENS["ink"], linestyle="--", linewidth=0.9)
    ax_top.set_ylabel("Angolo (deg)")
    ax_top.legend(loc="upper left", frameon=False, ncols=2)
    ax_top.text(max(0.0, offset - segment_gap_s), 35, " soglia 35 deg", va="bottom", ha="right", fontsize=8.2, color=TOKENS["ink"])
    clean_axes(ax_top)

    ax_bottom.set_ylabel("Quota base (m)")
    ax_bottom.set_xlabel("Tempo relativo dei segmenti registrati (s)")
    clean_axes(ax_bottom)
    save(fig, "assetto_quota_obstacle")


def figure_flat_trajectory(rows: list[dict[str, float]]) -> None:
    fig, ax = plt.subplots(figsize=(7.4, 5.4))
    add_header(
        fig,
        "Traiettoria sul piano usata per replay deterministico",
        "Scenario flat: posizione XY registrata durante la teleoperazione live.",
    )
    fig.subplots_adjust(top=0.78, left=0.13, right=0.97, bottom=0.13)

    sampled = list(downsample(rows))
    ax.plot(
        [row["pos_x"] for row in sampled],
        [row["pos_y"] for row in sampled],
        color=TOKENS["blue_mid"],
        linewidth=1.4,
    )
    ax.scatter([sampled[0]["pos_x"]], [sampled[0]["pos_y"]], color=TOKENS["olive_mid"], s=35, label="start")
    ax.scatter([sampled[-1]["pos_x"]], [sampled[-1]["pos_y"]], color=TOKENS["orange_mid"], s=35, label="fine")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="upper left", frameon=False)
    clean_axes(ax)
    ax.set_aspect("equal", adjustable="datalim")
    save(fig, "traiettoria_flat")


def main() -> None:
    configure_style()
    flat = load_json(FLAT_SUMMARY)
    obstacle = load_json(OBSTACLE_SUMMARY)
    flat_rows = read_telemetry(FLAT_TELEMETRY)
    obstacle_rows = read_telemetry(OBSTACLE_TELEMETRY)

    figure_architecture()
    figure_latency(flat, obstacle)
    figure_events(obstacle)
    figure_obstacle_timeseries(obstacle_rows)
    figure_flat_trajectory(flat_rows)


if __name__ == "__main__":
    main()
