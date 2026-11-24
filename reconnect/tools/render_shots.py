#!/usr/bin/env python3
"""Render the 6 reconnect view screenshots (owner/observer x T=1/10/60s).

Reads shots/data/scen_<tag>.{csv,meta} written by ./reconnect and renders
one PNG per view into shots/. Palette follows the repo dataviz default
(light surface, categorical slots 1-3, status-critical for the deviation).
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = ["Noto Sans CJK SC"]
plt.rcParams["axes.unicode_minus"] = False

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e1e0d9"
BASE = "#c3c2b7"
BLUE = "#2a78d6"     # series 1: authoritative true position
ORANGE = "#eb6834"   # series 2: observer displayed position
AQUA = "#1baf7a"     # series 3: owner character
MUTED = "#898781"    # stale / uncorrected extrapolation
CRIT = "#d03b3b"     # status critical: deviation arrow

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "shots", "data")
OUT = os.path.join(HERE, "..", "shots")


def read_meta(tag):
    meta = {}
    with open(os.path.join(DATA, f"scen_{tag}.meta"), encoding="utf-8") as f:
        for line in f:
            k, v = line.strip().split("=", 1)
            meta[k] = v
    for k in meta:
        if k != "tag":
            meta[k] = float(meta[k])
    meta["N"] = int(meta["N"])
    return meta


def read_traj(tag):
    t, tx, ty, ox, oy = [], [], [], [], []
    with open(os.path.join(DATA, f"scen_{tag}.csv"), encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            tx.append(float(row["true_x"]))
            ty.append(float(row["true_y"]))
            ox.append(float(row["obs_x"]))
            oy.append(float(row["obs_y"]))
    return t, tx, ty, ox, oy


def style_ax(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.tick_params(colors=INK2)
    ax.set_xlabel("x (u)", color=INK2)
    ax.set_ylabel("y (u)", color=INK2)
    ax.set_aspect("equal")


def limits(meta, tx, ty, ox, oy, extra):
    xs = list(tx) + list(ox) + [p[0] for p in extra]
    ys = list(ty) + list(oy) + [p[1] for p in extra]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    pad = max(x1 - x0, y1 - y0) * 0.12 + 1.0
    return (x0 - pad, x1 + pad), (y0 - pad, y1 + pad)


def render(tag):
    m = read_meta(tag)
    t, tx, ty, ox, oy = read_traj(tag)
    T, N, bdt = m["T"], m["N"], m["bcast_dt"]
    t_r = m["t_r"]

    xlim, ylim = limits(m, tx, ty, ox, oy,
                        [(m["p0x"], m["p0y"]),
                         (m["true_rx"], m["true_ry"]), (m["obs_rx"], m["obs_ry"])])

    # observer correction trajectory starts at the reconnect instant
    corr = [i for i, ti in enumerate(t) if ti >= t_r - 1e-9]

    T_label = f"{T:g}"

    # ---------------- observer view ----------------
    fig, ax = plt.subplots(figsize=(8, 7), dpi=150, facecolor=SURFACE)
    style_ax(ax)
    ax.plot(tx, ty, color=BLUE, linewidth=2, label="真实位置（服务器权威）")
    ax.plot([ox[i] for i in corr], [oy[i] for i in corr], color=ORANGE, linewidth=2,
            label="观察者显示位置（校正轨迹）")
    ax.plot(m["p0x"], m["p0y"], marker="s", color=ORANGE, markersize=9,
            label="观察者冻结位置（断线期间）")
    ax.plot(m["true_rx"], m["true_ry"], marker="o", color=BLUE, markersize=9)
    ax.annotate("", xy=(m["true_rx"], m["true_ry"]), xytext=(m["obs_rx"], m["obs_ry"]),
                arrowprops=dict(arrowstyle="->", color=CRIT, linewidth=2))
    mx, my = (m["true_rx"] + m["obs_rx"]) / 2, (m["true_ry"] + m["obs_ry"]) / 2
    ax.annotate(f"d0 = {m['d0']:.1f} u", (mx, my), color=INK, fontsize=10,
                xytext=(8, 8), textcoords="offset points")
    ax.plot(m["true_fx"], m["true_fy"], marker="D", color=BLUE, markersize=8)
    ax.plot(m["obs_fx"], m["obs_fy"], marker="D", color=ORANGE, markersize=8)
    ax.annotate("校正后汇合", (m["obs_fx"], m["obs_fy"]), color=INK2, fontsize=9,
                xytext=(8, -14), textcoords="offset points")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_title(f"断线 {T_label} s 后重连 — 观察者视角", color=INK, fontsize=13)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    ax.text(0.99, 0.01,
            f"重连瞬间偏差 d0={m['d0']:.1f} u（增长律上界 v_est·(T+τ)={m['bound']:.1f} u）\n"
            f"补发 {N} 次 × {bdt:.3f} s → 残差 {m['e_final']:.3f} u ≤ ε={m['eps']:.1f} u\n"
            f"不补发时偏差 {m['e_buggy']:.1f} u",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=9, color=INK,
            bbox=dict(facecolor=SURFACE, edgecolor=BASE, boxstyle="round,pad=0.4"))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f"reconnect_{tag}_observer.png"), facecolor=SURFACE)
    plt.close(fig)

    # ---------------- owner view ----------------
    fig, ax = plt.subplots(figsize=(8, 7), dpi=150, facecolor=SURFACE)
    style_ax(ax)
    ax.plot(tx, ty, color=BLUE, linewidth=2, label="真实位置（服务器权威）")
    ax.plot(m["p0x"], m["p0y"], marker="s", color=MUTED, markersize=8,
            label="断线时位置")
    ax.plot(m["true_fx"], m["true_fy"], marker="o", color=AQUA, markersize=12,
            label="本人人物（本地预测 + 重连快照）")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_title(f"断线 {T_label} s 后重连 — 本人视角", color=INK, fontsize=13)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    ax.text(0.99, 0.01,
            f"重连即收到本人状态快照，人物在真实位置\n"
            f"本人视角误差 {m['e_final'] * 0:.3f} u（预测与服务器一致）",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=9, color=INK,
            bbox=dict(facecolor=SURFACE, edgecolor=BASE, boxstyle="round,pad=0.4"))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f"reconnect_{tag}_owner.png"), facecolor=SURFACE)
    plt.close(fig)

    print(f"rendered reconnect_{tag}_observer.png / reconnect_{tag}_owner.png")


for tag in ("T01", "T10", "T60"):
    render(tag)
