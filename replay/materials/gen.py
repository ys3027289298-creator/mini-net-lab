#!/usr/bin/env python3
"""Single source of truth for the replay fixture.

Emits the three CSV materials consumed by the repro steps and embedded in
src/replay.c:
  traj_samples.csv  - material 1, trajectory sampling (remote clock)
  timing_records.csv - material 2, send/recv/remote timing (both clocks)
  packet_summary.csv - material 3, packet body summaries with FNV-1a CRC

Run from anywhere; files are written next to this script. Run with -I so no
imports resolve from the working directory.
"""
import math
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))

TICK_DT = 0.05          # 20 Hz remote ticks
EPOCH = 100.0           # remote time of tick 0
TRUE_OFFSET = 2.0       # remote clock leads local clock by 2 s
SCALE = 64.0            # position quantization, matches snapshot/
JITTER_SEQ = 4          # packet 4 arrives 30 ms late on the wire
JITTER = 0.03


def quant(v):
    return int(round(v * SCALE))


def zigzag(q):
    return ((q << 1) ^ (q >> 31)) & 0xFFFFFFFF


def fnv1a(data):
    h = 0x811C9DC5
    for b in data:
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


def pos(tick):
    return tick * 0.25, 0.1 * math.sin(tick * 0.2)


def main():
    traj_rows, pkt_rows, timing_rows = [], [], []

    # material 1: one trajectory sample every 4 ticks, ticks 0..40
    for k in range(11):
        tick = k * 4
        t = EPOCH + tick * TICK_DT
        x, y = pos(tick)
        traj_rows.append((k, tick, f"{t:.4f}", f"{x:.6f}", f"{y:.6f}"))

    # materials 2+3: eight packets, five ticks each, ticks 0..39
    for s in range(1, 9):
        lo, hi = (s - 1) * 5, s * 5 - 1
        rt = EPOCH + hi * TICK_DT
        half = 0.02 + 0.005 * (s % 4)          # RTT 40..55 ms, all benign
        mid = rt - TRUE_OFFSET                 # symmetric route -> midpoint offset
        t_send = mid - half
        t_recv = mid + half + (JITTER if s == JITTER_SEQ else 0.0)
        x, y = pos(hi)
        qx, qy = quant(x), quant(y)
        body_len = 24 + s * 4
        canonical = struct.pack("<6I", s, lo, hi, body_len,
                                zigzag(qx), zigzag(qy))
        crc = fnv1a(canonical)
        timing_rows.append((s, f"{t_send:.6f}", f"{t_recv:.6f}", f"{rt:.4f}"))
        pkt_rows.append((s, lo, hi, body_len, qx, qy, f"{crc:08x}"))

    def dump(name, header, rows):
        path = os.path.join(HERE, name)
        with open(path, "w") as f:
            f.write(",".join(header) + "\n")
            for r in rows:
                f.write(",".join(str(c) for c in r) + "\n")

    dump("traj_samples.csv",
         ["sample", "tick", "remote_t", "x", "y"], traj_rows)
    dump("timing_records.csv",
         ["seq", "local_t_send", "local_t_recv", "remote_t"], timing_rows)
    dump("packet_summary.csv",
         ["seq", "tick_lo", "tick_hi", "body_len", "qx", "qy", "crc32_fnv1a"],
         pkt_rows)
    print("wrote 3 CSV materials into", HERE)


if __name__ == "__main__":
    main()
