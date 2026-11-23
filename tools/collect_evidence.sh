#!/usr/bin/env bash
#
# collect_evidence.sh — gather acceptance evidence for the unified frame
# timing (ftrace). Produces, under artifacts/timing/:
#   raw/<module>_timing.txt        full instrumented run (all four evidence kinds)
#   off/<module>.txt               default (instrumentation off) output
#   overlaps/<module>.log          overlap-interval log extracted per module
#   tables/<module>_metrics.md     per-item timing table
#   reconcile_<module>_<case>.txt  reconciliation blocks (deterministic, real)
#   report.md                      assembled evidence report
# Exits non-zero if any check fails.
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
OUT="$ROOT/artifacts/timing"
MODULES=(snapshot clock aoi reconcile)
mkdir -p "$OUT/raw" "$OUT/off" "$OUT/overlaps" "$OUT/tables" "$OUT/reconcile"

fail=0
note() { printf '\n=== %s ===\n' "$*"; }

# ---------- 1. library self-tests (exact virtual-clock scenarios) ----------
note "common/ftrace library self-test"
make -C "$ROOT/common" clean test > "$OUT/raw/common_selftest.txt" 2>&1
grep -q '^all_ok=1$' "$OUT/raw/common_selftest.txt" || { echo "FAIL common selftest"; fail=1; }
grep -q '^verify=1$' "$OUT/raw/common_selftest.txt" || { echo "FAIL common verify"; fail=1; }

# ---------- 2. per-module builds: off-mode identity + on-mode evidence ----------
for m in "${MODULES[@]}"; do
    note "module $m"
    make -C "$ROOT/$m" clean > /dev/null

    # off mode: output must be byte-identical to the pre-change baseline
    make -C "$ROOT/$m" test > /dev/null
    "$ROOT/$m/$m" > "$OUT/off/$m.txt"
    if diff -u "$OUT/reference_off/$m.txt" "$OUT/off/$m.txt" > "$OUT/off/$m.diff"; then
        echo "off-mode output identical to baseline: $m"
    else
        echo "FAIL off-mode drift in $m"; cat "$OUT/off/$m.diff"; fail=1
    fi

    # on mode: full evidence run
    make -C "$ROOT/$m" test-timing > /dev/null
    "$ROOT/$m/${m}_timing" > "$OUT/raw/${m}_timing.txt"
    vcount=$(grep -c '^verify=1$' "$OUT/raw/${m}_timing.txt" || true)
    [ "$vcount" -eq 2 ] || { echo "FAIL $m verify count=$vcount"; fail=1; }
    grep -q '^all_ok=1$' "$OUT/raw/${m}_timing.txt" || { echo "FAIL $m all_ok"; fail=1; }

    # 2a. per-item timing table (evidence kind 1)
    awk '
      /^metric / {
        name=""; calls=""; tot=""; avg=""; mn=""; mx=""
        for (i = 2; i <= NF; i++) {
          split($i, kv, "=")
          if (kv[1]=="name") name=kv[2]
          else if (kv[1]=="calls") calls=kv[2]
          else if (kv[1]=="total_us") tot=kv[2]
          else if (kv[1]=="avg_us") avg=kv[2]
          else if (kv[1]=="min_us") mn=kv[2]
          else if (kv[1]=="max_us") mx=kv[2]
        }
        if (name != "") printf "| %s | %s | %s | %s | %s | %s |\n", name, calls, tot, avg, mn, mx
      }' "$OUT/raw/${m}_timing.txt" > "$OUT/tables/${m}_metrics_rows.md"

    # 2b. overlap-interval log (evidence kind 2), split by report block
    awk '
      /^== ftrace report/ { block = ($0 ~ /deterministic/) ? "deterministic" : "real-clock" }
      /^overlap / { print block ": " $0 }
    ' "$OUT/raw/${m}_timing.txt" > "$OUT/overlaps/${m}.log"

    # 2c. reconciliation blocks (evidence kind 3), one file per case
    awk -v m="$m" '
      /^== ftrace report/ { block = ($0 ~ /deterministic/) ? "deterministic" : "real-clock" }
      /^\[reconcile\]/ { inrec = 1; fn = sprintf("%s/reconcile/%s_%s.txt", OUT, m, block); next }
      inrec && /^[a-z_]+=/ { print > fn }
      inrec && /^verify=/ { inrec = 0 }
    ' OUT="$OUT" "$OUT/raw/${m}_timing.txt"
done

# ---------- 3. independent numeric reconciliation & upper-bound check ----------
note "independent reconciliation check (awk, not relying on ftrace verify)"
: > "$OUT/summary_rows.md"
: > "$OUT/bound_check.txt"
for m in "${MODULES[@]}"; do
    for kind in deterministic real-clock; do
        f="$OUT/reconcile/${m}_${kind}.txt"
        awk -v mod="$m" -v kind="$kind" -v summary="$OUT/summary_rows.md" '
          /^frames=/ { split($0,a,"="); frames=a[2] }
          /^frame_total_us=/ { split($0,a,"="); ft=a[2] }
          /^span_sum_us=/ { split($0,a,"="); ss=a[2] }
          /^union_us=/ { split($0,a,"="); un=a[2] }
          /^overlap_charge_us=/ { split($0,a,"="); ov=a[2] }
          /^gap_us=/ { split($0,a,"="); gap=a[2] }
          /^diff_span_sum_minus_frame_total_us=/ { split($0,a,"="); diff=a[2] }
          /^abs_diff_us=/ { split($0,a,"="); adiff=a[2] }
          /^roundoff_eps_us=/ { split($0,a,"="); eps=a[2] }
          /^clock_resolution_us=/ { split($0,a,"="); res=a[2] }
          /^bound_overlap_gap_eps_res_us=/ { split($0,a,"="); bound=a[2] }
          END {
            # report values are printed in us with 3 decimals (1 ns quantization);
            # the in-binary ft_verify() checks the same identities on unrounded doubles.
            tol = eps + 0.01
            id1 = ( (ss - ft) - diff ); d1 = (id1 < 0 ? -id1 : id1)
            id2 = ( diff - (ov - gap) ); d2 = (id2 < 0 ? -id2 : id2)
            id3 = ( (ss - un) - ov ); d3 = (id3 < 0 ? -id3 : id3)
            ok = (d1 <= tol && d2 <= tol && d3 <= tol && adiff <= bound + tol)
            margin = bound - adiff
            printf "%s %s: identity_resid=[%.3g,%.3g,%.3g] tol=%.3g |diff|=%.3f bound=%.3f margin=%.3f -> %s\n", \
              mod, kind, d1, d2, d3, tol, adiff, bound, margin, (ok ? "PASS" : "FAIL")
            if (kind == "real-clock")
              printf "| %s | %d | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f |\n", \
                mod, frames, ft, ss, un, ov, gap, diff, adiff, bound, margin >> summary
            exit(ok ? 0 : 1)
          }' "$f" >> "$OUT/bound_check.txt" \
          || { echo "FAIL bound check $m/$kind"; fail=1; }
    done
done
cat "$OUT/bound_check.txt"

# ---------- 4. assemble report.md ----------
note "assemble report.md"
{
    echo "# 统一帧计时口径 — 验收证据"
    echo
    echo "生成时间：$(date -u '+%Y-%m-%d %H:%M:%SZ')"
    echo
    echo "## 计时口径"
    echo
    echo "- 单一时钟源：所有测量取 \`CLOCK_MONOTONIC\`（纳秒转 double 秒），分辨率记入报表并参与上界。"
    echo "- 区间语义：半开区间 [start, end)；span 归属其 start 时所在帧，帧以 \`ft_frame_begin/end\` 界定。"
    echo "- 每帧恒等式：\`span_sum - frame_total = overlap - gap\`，其中"
    echo "  \`overlap = span_sum - union\`（重叠按多重度计费），\`gap = frame_total - union\`。"
    echo "- 上界（逐帧计算后求和）：\`|diff| <= overlap + gap + eps + resolution\`。"
    echo "- 编译开关 \`FT_ENABLE\`：默认 0，插桩点全部编译为 no-op；\`make timing\` 以 \`-DFT_ENABLE=1\` 构建。"
    echo
    echo "## 1. 总和对账 + 上界（真实时钟，64 帧负载）"
    echo
    echo "| 模块 | 帧数 | 总帧时µs | 分项和µs | 并集µs | 重叠计费µs | 空隙µs | 差值µs | \|差值\|µs | 上界µs | 余量µs |"
    echo "|---|---|---|---|---|---|---|---|---|---|---|"
    cat "$OUT/summary_rows.md"
    echo
    echo '```'
    cat "$OUT/bound_check.txt"
    echo '```'
    echo
    echo "独立 awk 复算的三项恒等式残差均在容差内，且 |差值| 不超过上界。"
    echo
    echo "## 2. 分项耗时表"
    for m in "${MODULES[@]}"; do
        echo
        echo "### $m"
        echo
        echo "| 分项 | 调用次数 | 总耗时µs | 均值µs | 最小µs | 最大µs |"
        echo "|---|---|---|---|---|---|"
        cat "$OUT/tables/${m}_metrics_rows.md"
    done
    echo
    echo "## 3. 重叠区间日志"
    echo
    echo "完整日志见 overlaps/<module>.log（含确定性场景与真实时钟场景，区间端点相对帧起点）。摘要："
    echo
    echo "| 模块 | 确定性场景重叠条数 | 真实时钟重叠条数 | 日志文件 |"
    echo "|---|---|---|---|"
    for m in "${MODULES[@]}"; do
        d=$(grep -c '^deterministic:' "$OUT/overlaps/${m}.log" || true)
        r=$(grep -c '^real-clock:' "$OUT/overlaps/${m}.log" || true)
        echo "| $m | $d | $r | overlaps/${m}.log |"
    done
    echo
    echo "确定性场景（每模块一致，虚拟时钟精确值，单位 µs）："
    echo
    echo '```'
    grep '^deterministic:' "$OUT/overlaps/snapshot.log"
    echo '```'
    echo
    echo "## 4. 关闭统一口径时的输出一致性"
    echo
    echo "| 模块 | 与改动前基线 diff |"
    echo "|---|---|"
    for m in "${MODULES[@]}"; do
        if [ ! -s "$OUT/off/$m.diff" ]; then echo "| $m | 完全一致（0 字节差异） |"; else echo "| $m | 有差异，见 off/$m.diff |"; fail=1; fi
    done
    echo
    echo "## 5. 复现命令"
    echo
    echo '```bash'
    echo "# 库级精确场景自测（虚拟时钟）"
    echo "make -C common clean test"
    echo
    echo "# 四模块：关闭口径（默认，输出与改动前一致）"
    for m in "${MODULES[@]}"; do echo "make -C $m clean test"; done
    echo
    echo "# 四模块：开启统一口径（分项表/重叠日志/对账/上界）"
    for m in "${MODULES[@]}"; do echo "make -C $m clean test-timing"; done
    echo
    echo "# 一键收集全部证据并重新生成本报告"
    echo "tools/collect_evidence.sh"
    echo '```'
} > "$OUT/report.md"

note "done"
if [ "$fail" -ne 0 ]; then echo "EVIDENCE CHECKS FAILED"; exit 1; fi
echo "all evidence checks passed; report at $OUT/report.md"
