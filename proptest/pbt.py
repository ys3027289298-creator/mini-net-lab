#!/usr/bin/env python3
"""Property-based test driver for mini-net-lab.

Subcommands:
  run        generate random scenarios (delay/loss/reorder), shrink any
             failing one to a minimal sequence, save it under failures/
  replay     re-run a saved failure file, verify it still fails the same way
  manual     run the hand-crafted case set (manual/*.txt); report misses
  mutations  inject mutants; verify every mutant the manual suite kills is
             also killed by the property tests (zero missed reports)
  all        manual + run + mutations

See proptest/README.md for the model and parameter ranges.
"""
import argparse
import glob
import json
import os
import random
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PBT = os.path.abspath(__file__)
HARNESS = os.path.join(HERE, "harness")
HARNESS_SRC = os.path.join(HERE, "harness.c")
MANUAL_DIR = os.path.join(HERE, "manual")
FAIL_DIR = os.path.join(HERE, "failures")
MUTANTS = os.path.join(HERE, "mutants.json")
MODULES = ("snapshot", "clock", "aoi", "reconcile")

# ---- parameter ranges of the random generator (reported in every summary) ----
PARAMS = {
    "ops_per_scenario": 80,
    "focus_weights": {"recon": 35, "clock": 25, "aoi": 20, "mixed": 20},
    "x_range": [-50.0, 50.0],
    "y_range": [-50.0, 50.0],
    "scale_choices": [4, 16, 64, 256],
    "delay_range": [0.0, 0.5],       # per-packet delay, seconds -> reordering
    "loss_prob": 0.25,               # per-packet drop
    "clock_oneway_delay": [0.01, 0.4],
    "true_offset": [-5.0, 5.0],
    "ack_error_prob": 0.5,           # authoritative != predicted
    "ack_error_range": [0.002, 2.0],
    "aoi_range": [-100.0, 100.0],
    "aoi_cell": 10.0,
    "aoi_radius": 15.0,
    "aoi_boundary_prob": 0.3,        # place entities exactly on the radius
}

FAIL_RE = re.compile(r"^FAIL op=\d+ invariant=([a-z0-9_]+)", re.M)


# ---------------------------------------------------------------- harness ----
def module_sources(root):
    return [os.path.join(root, m, "src", m + ".c") for m in MODULES]


def build(harness_src, out):
    subprocess.run(
        ["gcc", "-O2", "-std=c11", "-w", harness_src, "-o", out, "-lm"],
        check=True)


def ensure_built():
    srcs = [HARNESS_SRC] + module_sources(ROOT)
    if (not os.path.exists(HARNESS)
            or any(os.path.getmtime(s) > os.path.getmtime(HARNESS) for s in srcs)):
        build(HARNESS_SRC, HARNESS)


def run_scenario(binpath, lines):
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        f.write("\n".join(lines) + "\n")
        tmp = f.name
    try:
        p = subprocess.run([binpath, tmp], capture_output=True, text=True, timeout=60)
        return p.returncode, p.stdout, p.stderr
    finally:
        os.unlink(tmp)


def first_failure(rc, out):
    m = FAIL_RE.search(out)
    if m:
        return m.group(1)
    return "crash" if rc != 0 else None


# ------------------------------------------------------------- generator ----
class Gen:
    def __init__(self, rng):
        self.rng = rng
        self.header = ["param true_offset 2.000000",
                       "param dmin 0.010000",
                       "param dmax 0.400000"]
        self.body = []
        self.history = []      # all pushed ticks, in order
        self.pushed = {}       # tick -> predicted x
        self.pending = []      # snapped ticks still in flight
        self.acked = 0
        self.next_tick = 1
        self.clock_ts = 0.0
        self.aoi_next = 1
        self.aoi_ids = []

    def emit(self, s):
        self.body.append(s)

    def push(self, tick, x):
        self.emit(f"push {tick} {x:.6g}")
        self.history.append(tick)
        self.pushed[tick] = x
        if tick >= self.next_tick:
            self.next_tick = tick + 1

    def snap(self, tick, x, y=None, scale=None, delay=None, drop=None):
        r = self.rng
        y = r.uniform(*PARAMS["y_range"]) if y is None else y
        scale = r.choice(PARAMS["scale_choices"]) if scale is None else scale
        delay = r.uniform(*PARAMS["delay_range"]) if delay is None else delay
        drop = (1 if r.random() < PARAMS["loss_prob"] else 0) if drop is None else drop
        self.emit(f"snap {tick} {x:.6g} {y:.6g} {scale} {delay:.6f} {drop}")
        if not drop:
            self.pending.append(tick)

    def deliver(self):
        self.emit("deliver")
        if self.pending:
            self.acked = max(self.acked, max(self.pending))
        self.pending = []

    def clock(self, off):
        r = self.rng
        lo, hi = PARAMS["clock_oneway_delay"]
        ds, dr = r.uniform(lo, hi), r.uniform(lo, hi)
        ts = self.clock_ts
        self.clock_ts += 1.0
        self.emit(f"clock {ts:.6f} {ts + ds + dr:.6f} {ts + ds + off:.6f}")


def gen_recon(g):
    r = g.rng
    # guaranteed prefix: deterministically exercises stale/dup/rollback paths
    for t in (1, 2, 3):
        g.push(t, float(t))
    g.snap(3, 3.0, y=-1.25, delay=0.01, drop=0)
    g.deliver()
    g.snap(50, 0.0, y=-1.25, delay=0.01, drop=0)   # acked jumps past evicted ticks
    g.deliver()
    g.emit("pushn 51 70")                             # evicts ticks 1..3 from the ring
    for t in range(51, 121):
        g.history.append(t)
        g.pushed[t] = float(t)
    g.push(3, 9.0)                                    # stale and evicted from ring
    g.push(2, 8.0)
    g.push(60, 5.0)                                   # non-adjacent duplicate
    g.push(121, 3.0)
    g.snap(121, 3.5, y=0.5, delay=0.02, drop=0)       # 0.5 authoritative error
    g.deliver()
    # random filler
    for _ in range(PARAMS["ops_per_scenario"]):
        x = r.random()
        if x < 0.40:
            k = r.random()
            if k < 0.15:
                cand = [t for t in set(g.history) if t <= g.acked]
                if cand:
                    g.push(r.choice(cand), r.uniform(*PARAMS["x_range"]))
                    continue
            if k < 0.35 and g.history:
                g.push(r.choice(g.history[-30:]), r.uniform(*PARAMS["x_range"]))
                continue
            g.push(g.next_tick, r.uniform(*PARAMS["x_range"]))
        elif x < 0.75:
            if not g.history:
                continue
            t = r.choice(g.history[-20:])
            base = g.pushed.get(t, 0.0)
            err = (0.0 if r.random() < 1 - PARAMS["ack_error_prob"]
                   else r.uniform(*PARAMS["ack_error_range"]) * r.choice([-1, 1]))
            g.snap(t, base + err)
        else:
            g.deliver()
    g.deliver()


def gen_clock(g):
    r = g.rng
    off = r.uniform(*PARAMS["true_offset"])
    g.header = [f"param true_offset {off:.6f}",
                "param dmin 0.010000",
                "param dmax 0.400000"]
    # guaranteed: symmetric sample whose one-way delay (0.2) exceeds the offset
    # bound (0.4-0.01)/2 = 0.195 if the midpoint formula is wrong
    g.emit(f"clock 0.000000 0.400000 {0.2 + off:.6f}")
    g.clock_ts = 1.0
    for _ in range(60):
        g.clock(off)


def gen_aoi(g):
    r = g.rng
    g.emit("aoi_init 10 15")
    g.emit("aoi_addn 1 16")          # id 16 lands exactly on the radius boundary
    g.aoi_ids = list(range(1, 17))
    g.aoi_next = 17
    g.emit("aoi_query 100 0 0")
    g.emit("aoi_remove 1")
    g.emit("aoi_remove 2")
    g.emit("aoi_add 100 1 1")        # 14 alive, all 16 slots used
    for _ in range(PARAMS["ops_per_scenario"]):
        x = r.random()
        if x < 0.35:
            idv = g.aoi_next
            g.aoi_next += 1
            if r.random() < PARAMS["aoi_boundary_prob"]:
                px, py = r.choice([(15, 0), (0, 15), (-15, 0), (0, -15)])
            else:
                px, py = r.uniform(*PARAMS["aoi_range"]), r.uniform(*PARAMS["aoi_range"])
            g.emit(f"aoi_add {idv} {px:.6g} {py:.6g}")
            g.aoi_ids.append(idv)
        elif x < 0.55 and g.aoi_ids:
            g.emit(f"aoi_move {r.choice(g.aoi_ids)} "
                   f"{r.uniform(*PARAMS['aoi_range']):.6g} {r.uniform(*PARAMS['aoi_range']):.6g}")
        elif x < 0.70 and g.aoi_ids:
            g.emit(f"aoi_remove {r.choice(g.aoi_ids)}")
        elif g.aoi_ids:
            if r.random() < 0.5:
                g.emit(f"aoi_query {r.choice(g.aoi_ids)} 0 0")
            else:
                g.emit(f"aoi_query {r.choice(g.aoi_ids)} "
                       f"{r.uniform(*PARAMS['aoi_range']):.6g} {r.uniform(*PARAMS['aoi_range']):.6g}")


def gen_mixed(g):
    r = g.rng
    g.emit("aoi_init 10 15")
    for _ in range(PARAMS["ops_per_scenario"]):
        x = r.random()
        if x < 0.22:
            g.push(g.next_tick, r.uniform(*PARAMS["x_range"]))
        elif x < 0.40 and g.history:
            t = r.choice(g.history[-20:])
            err = (0.0 if r.random() < 1 - PARAMS["ack_error_prob"]
                   else r.uniform(*PARAMS["ack_error_range"]))
            g.snap(t, g.pushed.get(t, 0.0) + err)
        elif x < 0.48:
            g.deliver()
        elif x < 0.63:
            g.clock(off=2.0)
        elif x < 0.78:
            idv = g.aoi_next
            g.aoi_next += 1
            g.emit(f"aoi_add {idv} {r.uniform(*PARAMS['aoi_range']):.6g} "
                   f"{r.uniform(*PARAMS['aoi_range']):.6g}")
            g.aoi_ids.append(idv)
        elif x < 0.88 and g.aoi_ids:
            g.emit(f"aoi_remove {r.choice(g.aoi_ids)}")
        elif g.aoi_ids:
            g.emit(f"aoi_query {r.choice(g.aoi_ids)} "
                   f"{r.uniform(*PARAMS['aoi_range']):.6g} {r.uniform(*PARAMS['aoi_range']):.6g}")
    g.deliver()


def gen_scenario(seed):
    rng = random.Random(seed)
    kinds = list(PARAMS["focus_weights"])
    focus = rng.choices(kinds, weights=[PARAMS["focus_weights"][k] for k in kinds])[0]
    g = Gen(rng)
    {"recon": gen_recon, "clock": gen_clock,
     "aoi": gen_aoi, "mixed": gen_mixed}[focus](g)
    return g.header, g.body, focus


# -------------------------------------------------------------- shrinker ----
def shrink(binpath, header, body, inv):
    def fails(b):
        rc, out, _ = run_scenario(binpath, header + b)
        return first_failure(rc, out) == inv

    assert fails(body), "scenario does not reproduce before shrinking"
    # ddmin: remove chunks of decreasing granularity while the failure stays
    n = 2
    while len(body) >= 2:
        chunk = (len(body) + n - 1) // n
        reduced = False
        for i in range(0, len(body), chunk):
            cand = body[:i] + body[i + chunk:]
            if cand and fails(cand):
                body, n, reduced = cand, 2, True
                break
        if not reduced:
            if n >= len(body):
                break
            n = min(n * 2, len(body))
    # numeric minimization: zero out float tokens while the failure stays
    improved = True
    while improved:
        improved = False
        for i, line in enumerate(body):
            toks = line.split()
            for j, t in enumerate(toks):
                try:
                    v = float(t)
                except ValueError:
                    continue
                if v == 0.0:
                    continue
                cand = list(body)
                cand[i] = " ".join(toks[:j] + ["0"] + toks[j + 1:])
                if fails(cand):
                    body = cand
                    improved = True
                    break
    return body


def save_failure(inv, seed, header, body, focus):
    path = os.path.join(FAIL_DIR, f"{inv}-seed{seed}.txt")
    with open(path, "w") as f:
        f.write("# mini-net-lab property-test failure (minimal, shrunk)\n")
        f.write(f"# invariant: {inv}\n# seed: {seed}\n# focus: {focus}\n")
        f.write(f"# reproduce: python3 {os.path.relpath(PBT, ROOT)} replay "
                f"{os.path.relpath(path, ROOT)}\n")
        f.write(f"# params: {json.dumps(PARAMS, sort_keys=True)}\n")
        for l in header + body:
            f.write(l + "\n")
    return path


def load_scenario(path):
    inv = None
    lines = []
    with open(path) as f:
        for raw in f:
            s = raw.strip()
            if s.startswith("#"):
                m = re.match(r"#\s*invariant:\s*(\S+)", s)
                if m:
                    inv = m.group(1)
                continue
            if s:
                lines.append(s)
    return lines, inv


# -------------------------------------------------------------- commands ----
def cmd_run(args):
    ensure_built()
    os.makedirs(FAIL_DIR, exist_ok=True)
    found, counts = {}, {}
    for i in range(args.cases):
        seed = args.seed + i
        header, body, focus = gen_scenario(seed)
        rc, out, err = run_scenario(HARNESS, header + body)
        inv = first_failure(rc, out)
        if inv is None:
            continue
        counts[inv] = counts.get(inv, 0) + 1
        if inv in found:
            continue
        minimal = shrink(HARNESS, header, body, inv)
        path = save_failure(inv, seed, header, minimal, focus)
        found[inv] = (seed, path, len(minimal))
        print(f"[fail] seed={seed} focus={focus} invariant={inv} "
              f"-> {os.path.relpath(path, ROOT)} ({len(minimal)} ops after shrinking)")
    print("\n== PBT run summary ==")
    print(f"seeds: {args.seed}..{args.seed + args.cases - 1} ({args.cases} scenarios)")
    print(f"parameter ranges: {json.dumps(PARAMS, sort_keys=True)}")
    if not found:
        print("failures: none")
    else:
        print(f"failures: {len(found)} invariant classes")
        for inv, (seed, path, n) in sorted(found.items()):
            print(f"  {inv}: {counts[inv]} scenario(s), minimal sequence "
                  f"({n} ops): {os.path.relpath(path, ROOT)}")
        print(f"replay: python3 {os.path.relpath(PBT, ROOT)} replay <file>")
    return 0


def cmd_replay(args):
    ensure_built()
    lines, inv = load_scenario(args.file)
    rc, out, err = run_scenario(HARNESS, lines)
    actual = first_failure(rc, out)
    sys.stdout.write(out)
    if inv is None:
        print("no '# invariant:' header; ran scenario only")
        return 0 if actual is None else 1
    if actual == inv:
        print(f"reproduced: {inv}")
        return 0
    print(f"NOT reproduced: expected {inv}, got {actual}")
    return 1


def manual_cases():
    return sorted(glob.glob(os.path.join(MANUAL_DIR, "*.txt")))


def parse_expect(path):
    with open(path) as f:
        for line in f:
            m = re.match(r"#\s*expect:\s*(\S+)(?:\s+(\S+))?", line)
            if m:
                return m.group(1), m.group(2)
    return "pass", None


def failing_manual_cases(binpath):
    bad = set()
    for path in manual_cases():
        lines, _ = load_scenario(path)
        rc, out, _ = run_scenario(binpath, lines)
        if first_failure(rc, out) is not None:
            bad.add(os.path.basename(path))
    return bad


def cmd_manual(args):
    ensure_built()
    misses = 0
    for path in manual_cases():
        name = os.path.basename(path)
        kind, inv = parse_expect(path)
        expect = "pass" if kind == "pass" else f"fail {inv}"
        lines, _ = load_scenario(path)
        rc, out, _ = run_scenario(HARNESS, lines)
        actual = first_failure(rc, out)
        ok = (kind == "pass" and actual is None) or (kind == "fail" and actual == inv)
        if not ok:
            misses += 1
        print(f"{'ok  ' if ok else 'MISS'} {name}: expect={expect} actual={actual or 'pass'}")
    print(f"manual cases: {len(manual_cases())}, misses: {misses}")
    return 1 if misses else 0


def pbt_failure_pairs(binpath, seeds):
    pairs = set()
    for seed in seeds:
        header, body, _ = gen_scenario(seed)
        rc, out, _ = run_scenario(binpath, header + body)
        inv = first_failure(rc, out)
        if inv is not None:
            pairs.add((seed, inv))
    return pairs


def build_mutant(mut, tmp):
    for m in MODULES:
        shutil.copytree(os.path.join(ROOT, m), os.path.join(tmp, m))
    os.makedirs(os.path.join(tmp, "proptest"))
    shutil.copy(HARNESS_SRC, os.path.join(tmp, "proptest", "harness.c"))
    target = os.path.join(tmp, mut["file"])
    with open(target) as f:
        src = f.read()
    if mut["old"] not in src:
        raise SystemExit(f"mutant {mut['name']}: pattern not found in {mut['file']}")
    with open(target, "w") as f:
        f.write(src.replace(mut["old"], mut["new"], 1))
    binpath = os.path.join(tmp, "harness")
    build(os.path.join(tmp, "proptest", "harness.c"), binpath)
    return binpath


def cmd_mutations(args):
    ensure_built()
    with open(MUTANTS) as f:
        mutants = json.load(f)
    seeds = list(range(args.seed, args.seed + args.seeds))
    base_manual = failing_manual_cases(HARNESS)
    base_pbt = pbt_failure_pairs(HARNESS, seeds)
    print(f"baseline: manual failures={sorted(base_manual)}")
    print(f"baseline: pbt failures={sorted(base_pbt)}")
    missed = []
    for m in mutants:
        with tempfile.TemporaryDirectory() as tmp:
            binpath = build_mutant(m, tmp)
            km = failing_manual_cases(binpath) - base_manual
            kp = pbt_failure_pairs(binpath, seeds) - base_pbt
        if km and kp:
            status = "killed by both"
        elif km:
            status = "MISSED by pbt"
        elif kp:
            status = "killed by pbt only"
        else:
            status = "survived both"
        print(f"{m['name']:<28} manual={'killed' if km else '     -'} "
              f"pbt={'killed' if kp else '     -'}  {status}")
        if km and not kp:
            missed.append(m["name"])
    print(f"\nmissed reports (manual kills, property tests miss): "
          f"{len(missed)} {missed}")
    return 1 if missed else 0


def cmd_all(args):
    rc = cmd_manual(args)
    rc |= cmd_run(args)
    rc |= cmd_mutations(args)
    return rc


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run", help="generate + shrink random scenarios")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--cases", type=int, default=50)
    p.set_defaults(fn=cmd_run)
    p = sub.add_parser("replay", help="re-run a saved failure file")
    p.add_argument("file")
    p.set_defaults(fn=cmd_replay)
    p = sub.add_parser("manual", help="run the hand-crafted case set")
    p.set_defaults(fn=cmd_manual)
    p = sub.add_parser("mutations", help="mutation check: zero missed reports vs manual set")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--seeds", type=int, default=12)
    p.set_defaults(fn=cmd_mutations)
    p = sub.add_parser("all", help="manual + run + mutations")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--cases", type=int, default=50)
    p.add_argument("--seeds", type=int, default=12)
    p.set_defaults(fn=cmd_all)
    args = ap.parse_args()
    sys.exit(args.fn(args))


if __name__ == "__main__":
    main()
