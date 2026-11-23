#!/bin/sh
# drift-report: combines the four acceptance runs into one per-module summary
# plus a cross-module consistency check.
#
# The long phase of each module uses identical source parameters (step, ppm,
# seed), so every module must report the same long-run actual_ms and src_drift_ms.
# A divergence means the modules are not on a common time base.
set -u
cd "$(dirname "$0")/.." || exit 1

fail=0
printf 'module,phase,events,step_ms,nominal_ms,actual_ms,src_drift_ms,jitter_ms,drift_ms,drift_ppm,monotone,rejects,step_errors\n'
for m in snapshot clock aoi reconcile; do
    out=$("$m/$m" --accept) || fail=1
    printf '%s\n' "$out" | awk -v m="$m" '
        /^module=/ {
            delete f; n = split($0, a, " ")
            for (i = 1; i <= n; i++) { split(a[i], kv, "="); f[kv[1]] = kv[2] }
            if (f["phase"] == "normal" || f["phase"] == "long")
                printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
                    m, f["phase"], f["events"], f["step_ms"], f["nominal_ms"],
                    f["actual_ms"], f["src_drift_ms"], f["jitter_ms"],
                    f["drift_ms"], f["drift_ppm"], f["monotone"],
                    f["rejects"], f["step_errors"]
        }'
done

# Cross-module: the shared phase uses one identical source block for every
# module, so the reported base tick and source drift must agree exactly.
ref=$(for m in snapshot clock aoi reconcile; do
        "$m/$m" --accept | awk '/phase=shared/{print $4, $5}'
      done | sort -u)
count=$(printf '%s\n' "$ref" | grep -c .)
if [ "$count" -eq 1 ]; then
    echo "cross_module_shared_consistent=1 ($ref)"
else
    echo "cross_module_shared_consistent=0"
    printf '%s\n' "$ref" | sed 's/^/  /'
    fail=1
fi

echo "drift_report_ok=$([ "$fail" -eq 0 ] && echo 1 || echo 0)"
exit "$fail"
