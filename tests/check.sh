#!/usr/bin/env bash
# Unified acceptance for mini-net-lab.
#   1. per-module self-tests stay byte-identical to the pre-change golden
#   2. failure-matrix / isolation / localization suite passes
#   3. zero-copy: static scan + LD_PRELOAD interception under real batches
set -u
cd "$(dirname "$0")/.."
fail=0
say() { printf '\n=== %s ===\n' "$1"; }

say "1/4 build everything"
for d in snapshot clock aoi reconcile; do make -C "$d" clean >/dev/null && make -C "$d" || fail=1; done
make -C tests clean >/dev/null && make -C tests || fail=1

say "2/4 golden byte-identical check (legacy success paths)"
for d in snapshot clock aoi reconcile; do
  ./"$d/$d" > /tmp/mnl_golden_new.out
  if diff -u tests/golden/"$d".out /tmp/mnl_golden_new.out; then
    echo "$d: byte-identical"
  else
    echo "$d: OUTPUT CHANGED"; fail=1
  fi
done

say "3/4 static zero-copy scan (no copy of payload carriers in runners)"
# Any memcpy/memmove/mempcpy call lexically inside an mnl_*_run function is a
# contract violation. awk tracks the function body by brace depth instead of
# grepping the whole file (clock legitimately memmoves its own ring inside
# sync_add, which is a primitive, not a runner).
violations=$(awk '
  /^int mnl_[a-z]+_run\(/ { in_run = 1; depth = 0 }
  in_run {
    for (i = 1; i <= length($0); i++) {
      c = substr($0, i, 1)
      if (c == "{") depth++
      if (c == "}") { depth--; if (depth == 0) { in_run = 0; break } }
    }
    if (in_run && match($0, /(memcpy|memmove|mempcpy)[[:space:]]*\(/))
      print FILENAME ":" FNR ": " $0
  }
' snapshot/src/*.c clock/src/*.c aoi/src/*.c reconcile/src/*.c)
if [ -n "$violations" ]; then
  echo "$violations"; echo "runner contains a copy call - zero-copy contract broken"; fail=1
else
  echo "no memcpy/memmove/mempcpy call inside any mnl_*_run function"
fi
# The only copy routine in module sources may be clock's ring eviction.
other=$(grep -nE '\b(memcpy|memmove|mempcpy)\b' \
        snapshot/src/*.c aoi/src/*.c reconcile/src/*.c || true)
if [ -n "$other" ]; then
  echo "$other"; echo "unexpected copy routine outside clock ring"; fail=1
else
  echo "non-clock modules contain zero copy routines"
fi

say "4/4 runtime suites"
make -C tests test || fail=1
echo
if [ "$fail" -eq 0 ]; then echo "ALL CHECKS PASSED"; else echo "CHECKS FAILED"; fi
exit "$fail"
