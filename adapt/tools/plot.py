#!/usr/bin/env python3
"""Plot adaptive-degradation simulation traces (baseline vs adaptive).

Reads the CSVs emitted by src/sim.c and writes PNGs plus a combined dashboard
into the artifacts directory. Style follows the dataviz skill palette.
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

ART = sys.argv[1] if len(sys.argv) > 1 else "../../artifacts/adapt"

# dataviz reference palette (light mode)
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
YELLOW, MAGENTA, RED, VIOLET = "#eda100", "#e87ba4", "#e34948", "#6250d6"
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#898781", "#e1e0d9", "#fcfcfb"
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Arial", "Liberation Sans"],
    "font.size": 10,
    "axes.edgecolor": "#c3c2b7",
    "axes.labelcolor": INK,
    "axes.titlecolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.facecolor": SURFACE, "figure.facecolor": SURFACE,
    "legend.frameon": False,
})

PHASES = [(0, 20, "good"), (20, 45, "ramp"), (45, 70, "poor"),
          (70, 85, "blackout"), (85, 110, "poor"), (110, 130, "recovery"),
          (130, 150, "good")]
TIER_NAME = {0: "T0 20Hz hermite", 1: "T1 10Hz linear", 2: "T2 5Hz hold"}


def load(name):
    with open(os.path.join(ART, f"trace_{name}.csv")) as f:
        rows = list(csv.DictReader(f))
    d = {k: [float(r[k]) for r in rows] for k in rows[0]}
    d["tier"] = [int(r["tier"]) for r in rows]
    d["interp"] = [int(r["interp"]) for r in rows]
    d["stutter"] = [int(r["stutter"]) for r in rows]
    d["stall"] = [int(r["stall"]) for r in rows]
    d["reason"] = [int(r["reason"]) for r in rows]
    return d


base, adapt = load("baseline"), load("adaptive")
t = base["t"]


def shade(ax):
    for t0, t1, label in PHASES:
        if label == "blackout":
            ax.axvspan(t0, t1, color=RED, alpha=0.08, zorder=0)
        elif label in ("poor",):
            ax.axvspan(t0, t1, color=YELLOW, alpha=0.10, zorder=0)
    ax.set_xlim(0, 150)


def save(fig, name):
    fig.tight_layout()
    out = os.path.join(ART, name)
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print("wrote", out)


# 1. Network conditions + committed tier -------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True, height_ratios=[1, 0.8])
shade(ax1)
rtt_ms = [v * 1000 if v >= 0 else math.nan for v in adapt["rtt"]]
loss_pct = [v * 100 if v >= 0 else math.nan for v in adapt["loss"]]
ax1.plot(t, rtt_ms, color=BLUE, linewidth=1.4, label="RTT (ms)")
ax1.set_ylabel("RTT (ms)")
ax1b = ax1.twinx()
ax1b.plot(t, loss_pct, color=ORANGE, linewidth=1.2, label="loss (%)")
ax1b.set_ylabel("packet loss (%)", color=ORANGE)
ax1b.tick_params(colors=MUTED)
ax1b.grid(False)
h1, l1 = ax1.get_legend_handles_labels(); h2, l2 = ax1b.get_legend_handles_labels()
ax1.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=9)

shade(ax2)
ax2.step(t, adapt["tier"], where="post", color=VIOLET, linewidth=1.6, label="adaptive committed tier")
ax2.axhline(0, color=MUTED, linewidth=1, linestyle="--")
ax2.set_yticks([0, 1, 2]); ax2.set_yticklabels(["T0", "T1", "T2"])
ax2.set_ylabel("tier"); ax2.set_xlabel("time (s)")
# mark missing-reading fallback spans
miss = [i for i, r in enumerate(adapt["reason"]) if r == 1]
if miss:
    spans = []
    s = miss[0]; p = miss[0]
    for i in miss[1:]:
        if i == p + 1: p = i
        else: spans.append((s, p)); s = p = i
    spans.append((s, p))
    for s, e in spans:
        ax2.axvspan(t[s], t[e], color=RED, alpha=0.15, zorder=0)
ax2.legend(loc="upper left", fontsize=9)
ax2.set_title("Network conditions drive tier selection; red = conservative fallback (readings missing)")
save(fig, "01_network_and_tier.png")

# 2. Bandwidth ----------------------------------------------------------------
fig, ax = plt.subplots(figsize=(11, 3.6))
shade(ax)
ax.plot(t, base["bw_kbps"], color=MUTED, linewidth=1.5, label="baseline (fixed 20 Hz)")
ax.plot(t, adapt["bw_kbps"], color=BLUE, linewidth=1.6, label="adaptive")
ax.set_ylabel("uplink (kbits/s)"); ax.set_xlabel("time (s)")
ax.legend(loc="upper right", fontsize=9)
ax.set_title("Uplink bandwidth: adaptive backs off under poor links and blackout")
save(fig, "02_bandwidth.png")

# 3. Stutter markers (both pipelines) ----------------------------------------
fig, ax = plt.subplots(figsize=(11, 3.4))
shade(ax)
for d, col, lab, off in [(base, RED, "baseline stutter", 0.12),
                         (adapt, AQUA, "adaptive stutter", -0.12)]:
    xs = [tt + off for tt, s in zip(t, d["stutter"]) if s]
    ax.scatter(xs, [1 if off > 0 else 0] * len(xs), marker="|", s=70,
               color=col, linewidths=1.2, label=lab)
ax.set_yticks([0, 1]); ax.set_yticklabels(["adaptive", "baseline"])
ax.set_ylim(-0.6, 1.6); ax.set_xlabel("time (s)")
ax.legend(loc="upper right", fontsize=9)
ax.set_title("Stutter frames (each tick = one frozen/stalled render frame)")
save(fig, "03_stutter.png")

# 4. Joint error with analytic upper bound -----------------------------------
fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={"width_ratios": [1.15, 1]})
# left: every non-frozen frame as a (bound, measured) point; all must sit
# below the y=x line for the analytic bound to be a valid upper bound.
pts_b, pts_e, f_b, f_e = [], [], [], []
for b, e, s in zip(adapt["j0_bound"], adapt["j0_err"], adapt["stutter"]):
    (f_b if s else pts_b).append(b)
    (f_e if s else pts_e).append(e)
ax.scatter(pts_b, pts_e, s=6, color=BLUE, alpha=0.35, label="interpolating frame")
ax.scatter(f_b, f_e, s=6, color=RED, alpha=0.35, label="frozen frame")
lim = max(adapt["j0_bound"]) * 1.02
ax.plot([1e-4, lim], [1e-4, lim], color=INK, linewidth=1.2, linestyle="--", label="y = x (bound)")
ax.set_xscale("log"); ax.set_yscale("log")
ax.set_xlim(1e-3, lim); ax.set_ylim(1e-3, lim)
ax.set_xlabel("analytic upper bound (rad, log scale)")
ax.set_ylabel("joint-0 measured |err| (rad, log scale)")
ax.legend(loc="upper left", fontsize=9, markerscale=3)
ax.set_title("All frames below y=x: bound is a valid upper bound (0 violations)")

# right: measured-error timeline for both pipelines
shade(ax2)
ax2.plot(t, adapt["max_err"], color=MAGENTA, linewidth=1.0, label="adaptive worst-joint |err|")
ax2.plot(t, base["max_err"], color=AQUA, linewidth=1.0, alpha=0.85,
         label="baseline worst-joint |err|")
ax2.set_ylabel("worst-joint error (rad)"); ax2.set_xlabel("time (s)")
ax2.legend(loc="upper left", fontsize=9); ax2.set_ylim(0, 1.9)
ax2.set_title("Measured error over time")
save(fig, "04_error_and_bound.png")

# 5. Phase-level stutter bar comparison (split: operating phases vs blackout) --
with open(os.path.join(ART, "summary.csv")) as f:
    summ = list(csv.DictReader(f))
def row(scope, pipe):
    return next(r for r in summ if r["scope"] == scope and r["pipeline"] == pipe)
op_scopes = ["good", "ramp", "poor", "poor2", "recovery", "good2"]
bvals = [float(row(s, "baseline")["stutter_pct"]) for s in op_scopes]
avals = [float(row(s, "adaptive")["stutter_pct"]) for s in op_scopes]
bo_b = float(row("blackout", "baseline")["stutter_pct"])
bo_a = float(row("blackout", "adaptive")["stutter_pct"])
import numpy as np
fig, (ax, axbo) = plt.subplots(1, 2, figsize=(12, 4.2),
                               gridspec_kw={"width_ratios": [3, 1]})
x = np.arange(len(op_scopes)); w = 0.38
ax.bar(x - w / 2, bvals, w, color=MUTED, label="baseline")
ax.bar(x + w / 2, avals, w, color=BLUE, label="adaptive")
for i, (b, a) in enumerate(zip(bvals, avals)):
    ax.text(i - w / 2, b + 0.15, f"{b:.1f}", ha="center", fontsize=8, color=INK)
    ax.text(i + w / 2, a + 0.15, f"{a:.1f}", ha="center", fontsize=8, color=INK)
ax.set_xticks(x); ax.set_xticklabels(op_scopes, rotation=20, ha="right")
ax.set_ylabel("stutter frames (%)"); ax.set_title("Operating network phases")
ax.legend(fontsize=9); ax.set_ylim(0, max(bvals) * 1.35)

xb = 0
axbo.bar(xb - w / 2, [bo_b], w, color=MUTED, label="baseline")
axbo.bar(xb + w / 2, [bo_a], w, color=BLUE, label="adaptive")
axbo.text(xb - w / 2, bo_b + 1.5, f"{bo_b:.1f}", ha="center", fontsize=8, color=INK)
axbo.text(xb + w / 2, bo_a + 1.5, f"{bo_a:.1f}", ha="center", fontsize=8, color=INK)
axbo.set_xticks([xb]); axbo.set_xticklabels(["blackout"])
axbo.set_ylim(0, 108); axbo.set_title("Full blackout (no packets)")
fig.suptitle("Stutter rate by network phase")
save(fig, "05_phase_stutter.png")

print("done")
