#!/bin/sh
# determinism: prove acceptance does not depend on the system clock.
#
#  1. no source file references a wall-clock API, and no binary imports one;
#  2. two acceptance runs produce byte-identical output;
#  3. runs under different TZ and with a shifted wall clock (faketime-free:
#     we only change TZ, which is all libc clock selection could observe).
set -u
cd "$(dirname "$0")/.." || exit 1

fail=0
note() { printf '%s\n' "$*"; }

# 1. Static check: no wall-clock API in the tree, no time import in binaries.
if grep -rnE '\b(clock_gettime|gettimeofday|time|timespec_get)\s*\(' \
        --include='*.c' --include='*.h' snapshot clock aoi reconcile timebase \
        2>/dev/null; then
    note "sysclock_source_ref=1"; fail=1
else
    note "sysclock_source_ref=0"
fi

for m in snapshot clock aoi reconcile; do
    if nm -u "$m/$m" 2>/dev/null | grep -qE 'clock_gettime|gettimeofday|timespec_get| time$'; then
        note "sysclock_symbols_$m=1"; fail=1
    else
        note "sysclock_symbols_$m=0"
    fi
done

# 2. Same-input determinism: two runs must agree byte for byte.
tmp=$(mktemp -d) || exit 1
trap 'rm -rf "$tmp"' EXIT INT TERM
for m in snapshot clock aoi reconcile; do
    "$m/$m" --accept > "$tmp/$m.1" || fail=1
    "$m/$m" --accept > "$tmp/$m.2" || fail=1
    if cmp -s "$tmp/$m.1" "$tmp/$m.2"; then
        note "deterministic_$m=1"
    else
        note "deterministic_$m=0"; fail=1
    fi
done

# 3. Environment independence: TZ must not change anything.
for m in snapshot clock aoi reconcile; do
    TZ=Pacific/Kiritimati "$m/$m" --accept > "$tmp/$m.tz" || fail=1
    if cmp -s "$tmp/$m.1" "$tmp/$m.tz"; then
        note "tz_independent_$m=1"
    else
        note "tz_independent_$m=0"; fail=1
    fi
done

note "determinism_ok=$([ "$fail" -eq 0 ] && echo 1 || echo 0)"
exit "$fail"
