#!/bin/sh
# run: build every module, run its self-tests, then run acceptance for all
# four modules on the shared time base.  Exit 0 only when everything passes.
#
# No system clock is involved: acceptance sources are injected sequences, so
# this is safe to run at any time and on any machine.
set -u

cd "$(dirname "$0")/.." || exit 1

fail=0
run() {
    printf '\n== %s\n' "$*"
    if ! "$@"; then fail=1; fi
}

run make -C timebase test
for m in snapshot clock aoi reconcile; do
    run make -C "$m" test
done

# Success-path output must be byte-for-byte identical to the committed golden
# files (captured before the shared time base was introduced).
for m in snapshot clock aoi reconcile; do
    printf '\n== golden %s\n' "$m"
    if ! "$m/$m" | cmp -s - "tests/golden/$m.out"; then
        echo "golden mismatch: $m"
        fail=1
    fi
done

for m in snapshot clock aoi reconcile; do
    printf '\n== accept %s\n' "$m"
    if ! "$m/$m" --accept; then fail=1; fi
done

printf '\n== drift report\n'
if ! ./tests/drift-report.sh; then fail=1; fi

printf '\n== determinism\n'
if ! ./tests/determinism.sh; then fail=1; fi

printf '\nresult=%s\n' "$([ "$fail" -eq 0 ] && echo pass || echo fail)"
exit "$fail"
