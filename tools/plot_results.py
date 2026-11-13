#!/usr/bin/env python3
"""Draw the figures of docs/ from the CSV files written by the benchmark and train tools.
Usage: python3 tools/plot_results.py   (from the repository root, needs matplotlib)"""
import csv
import statistics
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS = "docs/results/"
CONTROLLERS = ["random", "fixed", "heuristic", "maxpressure", "qlearning", "dqn"]
LABELS = {"random": "Random", "fixed": "Fixed cycle", "heuristic": "Heuristic",
          "maxpressure": "Max pressure", "qlearning": "Q-learning", "dqn": "DQN"}
# One fixed color per controller, in every figure
COLORS = dict(zip(CONTROLLERS, ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]))
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"

plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": MUTED,
                     "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK,
                     "axes.spines.top": False, "axes.spines.right": False,
                     "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
                     "savefig.facecolor": SURFACE})


def load(path):
    """rows[scenario or demand][controller] = list of per-network values of each metric"""
    data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    for row in csv.DictReader(open(path)):
        key = row["scenario"] if row["scenario"] != "custom" else float(row["demand"])
        for metric in ("time_spent", "delay", "flow"):
            data[key][row["controller"]][metric].append(float(row[metric]))
    return data


def style(ax, ylabel):
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(length=0)


def benchmark_figure():
    data = load(RESULTS + "benchmark.csv")
    scenarios = ["light", "medium", "heavy", "saturated"]
    fig, axes = plt.subplots(1, 4, figsize=(15, 3.8))
    for ax, scenario in zip(axes, scenarios):
        metric = "flow" if scenario == "saturated" else "delay"
        means = [statistics.mean(data[scenario][c][metric]) for c in CONTROLLERS]
        errors = [statistics.stdev(data[scenario][c][metric]) for c in CONTROLLERS]
        ax.bar(range(6), means, yerr=errors, width=0.62, color=[COLORS[c] for c in CONTROLLERS],
               error_kw={"ecolor": MUTED, "elinewidth": 1, "capsize": 2})
        for x, m, e in zip(range(6), means, errors):
            ax.text(x, m + e + max(means) * 0.02, f"{m:.0f}" if m >= 20 else f"{m:.1f}",
                    ha="center", va="bottom", fontsize=9)
        ax.set_xticks(range(6), [LABELS[c].replace(" ", "\n") for c in CONTROLLERS], fontsize=8)
        ax.set_title(f"{scenario.capitalize()} traffic", fontsize=11, loc="left")
        style(ax, "Trips per minute (higher is better)" if metric == "flow"
              else "Delay per trip, s (lower is better)")
    fig.suptitle("30 random networks never seen in training: mean and standard deviation",
                 x=0.01, ha="left", fontsize=10, color=MUTED)
    fig.tight_layout()
    fig.savefig("docs/benchmark.png", dpi=150)


def demand_figure():
    data = load(RESULTS + "demand_sweep.csv")
    demands = sorted(data)
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    for c in CONTROLLERS:
        medians = [statistics.median(data[d][c]["time_spent"]) for d in demands]
        ax.plot(demands, medians, color=COLORS[c], linewidth=2, marker="o", markersize=5, label=LABELS[c])
    ax.set_yscale("log")
    ax.set_yticks([10, 20, 50, 100], ["10", "20", "50", "100"])
    ax.set_xticks(demands)
    ax.set_xlabel("Demand (vehicles generated per minute)")
    style(ax, "Time spent per vehicle, s (log scale, lower is better)")
    ax.legend(frameon=False, ncol=2, loc="upper left")
    ax.set_title("Median over 20 random networks", fontsize=10, loc="left", color=MUTED)
    fig.tight_layout()
    fig.savefig("docs/demand_sweep.png", dpi=150)


def training_figure():
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    for name, controller in (("training_qlearning.csv", "qlearning"), ("training_dqn.csv", "dqn")):
        rows = list(csv.DictReader(open(RESULTS + name)))
        ax.plot([int(r["episode"]) for r in rows], [float(r["validation_score"]) for r in rows],
                color=COLORS[controller], linewidth=2, label=LABELS[controller])
    ax.axhline(1.0, color=COLORS["heuristic"], linewidth=1.5, linestyle="--", label="Heuristic")
    ax.set_xlabel("Training episode (one episode: a random network for 10 simulated minutes)")
    style(ax, "Time spent relative to the heuristic\n(validation networks, lower is better)")
    ax.legend(frameon=False, ncol=3, loc="upper right")
    fig.tight_layout()
    fig.savefig("docs/training.png", dpi=150)


if __name__ == "__main__":
    benchmark_figure()
    demand_figure()
    training_figure()
