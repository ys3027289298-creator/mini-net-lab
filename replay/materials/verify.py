#!/usr/bin/env python3
"""Cross-check the CSV materials against the fixture embedded in
src/replay.c by parsing the C arrays structurally (no literal matching).
Exits non-zero on the first drift. Run with -I.
"""
import csv
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "src", "replay.c")


def zigzag(q):
    return ((q << 1) ^ (q >> 31)) & 0xFFFFFFFF


def fnv1a(seq, lo, hi, ln, qx, qy):
    h = 0x811C9DC5
    for b in struct.pack("<6I", seq, lo, hi, ln, zigzag(qx), zigzag(qy)):
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


def read_csv(name):
    with open(os.path.join(HERE, name), newline="") as f:
        return list(csv.DictReader(f))


def block(src, name):
    m = re.search(r"%s\[[^\]]*\]\s*=\s*\{(.*?)\};" % name, src, re.S)
    if not m:
        raise AssertionError("array %s not found in src/replay.c" % name)
    return re.findall(r"\{([^{}]*)\}", m.group(1))


def main():
    src = open(SRC).read()
    failures = []

    c_pkts = [tuple(int(x.strip().rstrip("u"), 16)
                    if x.strip().lower().startswith("0x") else int(x.strip())
                    for x in row.split(",")[:7])
              for row in block(src, "pkts")]
    c_recs = [tuple(float(x) for x in row.split(","))
              for row in block(src, "recs")]
    c_traj = [tuple(float(x.rstrip("f")) for x in row.split(","))
              for row in block(src, "traj_in")]

    pkts = read_csv("packet_summary.csv")
    if len(pkts) != len(c_pkts):
        failures.append("packet count drift csv=%d c=%d" % (len(pkts), len(c_pkts)))
    for r, c in zip(pkts, c_pkts):
        seq, lo, hi, ln, qx, qy = (int(r[k]) for k in
                                   ("seq", "tick_lo", "tick_hi", "body_len", "qx", "qy"))
        want_crc = fnv1a(seq, lo, hi, ln, qx, qy)
        if f"{want_crc:08x}" != r["crc32_fnv1a"]:
            failures.append("crc mismatch in csv seq %d" % seq)
        if c != (seq, lo, hi, ln, qx, qy, want_crc):
            failures.append("C packet fixture drift seq %d: %s" % (seq, c))

    timing = read_csv("timing_records.csv")
    if len(timing) != len(c_recs):
        failures.append("timing count drift")
    for r, c in zip(timing, c_recs):
        vals = (float(r["local_t_send"]), float(r["local_t_recv"]),
                float(r["remote_t"]))
        if any(abs(a - b) > 1e-9 for a, b in zip(vals, c)):
            failures.append("C timing fixture drift seq %s: %s vs %s"
                            % (r["seq"], vals, c))

    traj = read_csv("traj_samples.csv")
    if len(traj) != len(c_traj):
        failures.append("trajectory count drift")
    for r, c in zip(traj, c_traj):
        vals = (float(r["remote_t"]), float(r["x"]), float(r["y"]))
        if any(abs(a - b) > 1e-6 for a, b in zip(vals, c)):
            failures.append("C trajectory fixture drift tick %s: %s vs %s"
                            % (r["tick"], vals, c))

    if failures:
        for m in failures:
            print("FAIL:", m)
        return 1
    print(f"verify_ok: {len(pkts)} packets, {len(timing)} timing rows, "
          f"{len(traj)} trajectory samples consistent with src/replay.c")
    return 0


if __name__ == "__main__":
    sys.exit(main())
