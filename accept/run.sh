#!/usr/bin/env bash
#
# run.sh - end-to-end cross-version compatibility acceptance.
#
# One command, from a clean tree:
#   1. build all four modules and run each module's self test
#   2. build the pinned v1 reference codec and the two independent clients
#   3. regenerate the committed replay recording from the current encoder and
#      assert every packet still starts with the exact pinned v1 bytes
#   4. build and run accept.c -> error table, compatibility matrix, pixel
#      identity, package-body hashes
#   5. run the old and new client binaries over the same recording and compare
#      their framebuffers (independent proof of pixel identity)
#   6. verify committed golden body hashes
#   7. hard gate: all required evidence files exist and are non-empty, every
#      matrix cell passed, golden bodies match; otherwise ACCEPTANCE=FAIL.
#
# Usage:
#   accept/run.sh                verify
#   accept/run.sh --regenerate   rebuild committed recording + golden bodies
#                                (only when the wire format is intentionally
#                                changed; review the golden diff afterwards)
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ACC="$ROOT/accept"
EVI="$ACC/evidence"
GOLD="$ACC/golden"
REC="$ACC/recordings"
WORK="$ACC/build"
CC="${CC:-gcc}"
CFLAGS="-O2 -std=c11 -Wall -Wextra"
MODULES=(snapshot clock aoi reconcile)

REGEN=0
[[ "${1:-}" == "--regenerate" ]] && REGEN=1

FAILED=0
log()  { printf '\n=== %s ===\n' "$*"; }
die()  { echo "FATAL: $*" >&2; exit 1; }
note_fail() { echo "FAIL: $*"; FAILED=1; }

rm -rf "$WORK"
# Start from an empty evidence directory: the completeness gate below must
# only ever see files produced by THIS run, never a stale prior one.
rm -rf "$EVI"
mkdir -p "$EVI" "$GOLD" "$WORK/obj" "$WORK/bin" "$REC"

# ---------------------------------------------------------------------------
# 0. provenance
# ---------------------------------------------------------------------------
HEAD_SHA="$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
BASE_SHA="0bdd277b7f181db9fd40c8653604070e55268020"   # v1 wire format baseline
PINNED_SHA="$(sha256sum "$ACC/legacy/snapshot_v1.c" | awk '{print $1}')"
cat > "$EVI/provenance.txt" <<EOF
repo=$ROOT
head=$HEAD_SHA
v1_baseline=$BASE_SHA
pinned_snapshot_source_sha256=$PINNED_SHA
cc=$($CC --version | head -1)
date_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
cat "$EVI/provenance.txt"

# Re-export the pinned source from the baseline commit and prove it is
# byte-identical to what is committed, so the "old client" is really old.
# A recorded hash in accept/legacy/SHA256 lets the check also run without .git.
PINNED_EXPECT="$(awk -v f="$ACC/legacy/snapshot_v1.c" '$2==f || $2=="snapshot_v1.c"{print $1}' "$ACC/legacy/SHA256" 2>/dev/null | head -1)"
if [[ -n "${PINNED_EXPECT:-}" ]]; then
  if [[ "$PINNED_SHA" == "$PINNED_EXPECT" ]]; then
    echo "pinned v1 source matches recorded hash $PINNED_EXPECT"
  else
    note_fail "pinned v1 source hash $PINNED_SHA != recorded $PINNED_EXPECT"
  fi
fi
if git -C "$ROOT" rev-parse --git-dir >/dev/null 2>&1; then
  if git -C "$ROOT" cat-file -e "$BASE_SHA" 2>/dev/null; then
    git -C "$ROOT" show "$BASE_SHA:snapshot/src/snapshot.c" > "$WORK/snapshot_v1.fromgit"
    if cmp -s "$WORK/snapshot_v1.fromgit" "$ACC/legacy/snapshot_v1.c"; then
      echo "pinned v1 source matches $BASE_SHA exactly"
    else
      note_fail "accept/legacy/snapshot_v1.c differs from $BASE_SHA:snapshot/src/snapshot.c"
    fi
  else
    note_fail "baseline commit $BASE_SHA missing from git; cannot verify pinned source"
  fi
else
  echo "not a git checkout: pinned-source check used recorded SHA256 only"
  [[ -n "${PINNED_EXPECT:-}" ]] || note_fail "no .git and no recorded pinned hash"
fi

# ---------------------------------------------------------------------------
# 1. module self tests
# ---------------------------------------------------------------------------
log "1. per-module self tests"
: > "$EVI/selftest.log"
for m in "${MODULES[@]}"; do
  make -C "$ROOT/$m" clean >/dev/null 2>&1
  if make -C "$ROOT/$m" test > "$WORK/$m.selftest" 2>&1; then
    res=pass
  else
    res=FAIL; note_fail "self test $m"
  fi
  tail -1 "$WORK/$m.selftest" | sed "s/^/$m: /" >> "$EVI/selftest.log"
  printf '%-10s %s\n' "$m" "$res"
done

# ---------------------------------------------------------------------------
# 2. build objects, reference codec, driver and clients
# ---------------------------------------------------------------------------
log "2. build acceptance harness"
for m in "${MODULES[@]}"; do
  "$CC" $CFLAGS -DMINILAB_NO_MAIN -I"$ROOT/$m/src" -c "$ROOT/$m/src/$m.c" \
    -o "$WORK/obj/$m.o" || die "compile $m"
done

INC="-I$ACC -I$ACC/legacy -I$ROOT/snapshot/src -I$ROOT/clock/src -I$ROOT/aoi/src -I$ROOT/reconcile/src"

"$CC" $CFLAGS -I"$ACC" -I"$ACC/legacy" -c "$ACC/render.c"    -o "$WORK/obj/render.o"    || die "compile render"
"$CC" $CFLAGS -I"$ACC" -I"$ACC/legacy" -c "$ACC/recording.c" -o "$WORK/obj/recording.o" || die "compile recording"
"$CC" $CFLAGS -Wno-unused-function -I"$ACC/legacy" -c "$ACC/legacy/snapshot_pinned.c" -o "$WORK/obj/snapshot_pinned.o" || die "compile pinned"
"$CC" $CFLAGS -I"$ACC/legacy" -c "$ACC/legacy/legacy_v1.c" -o "$WORK/obj/legacy_v1.o" || die "compile legacy"

"$CC" $CFLAGS $INC "$ACC/accept.c" "$WORK/obj/render.o" "$WORK/obj/recording.o" \
  "$WORK/obj/legacy_v1.o" "$WORK/obj/snapshot_pinned.o" \
  "$WORK/obj"/{snapshot,clock,aoi,reconcile}.o -lm -o "$WORK/bin/accept" || die "build accept"

"$CC" $CFLAGS -I"$ACC" -I"$ACC/legacy" -I"$ROOT/snapshot/src" "$ACC/tools/make_recording.c" \
  "$WORK/obj/recording.o" "$WORK/obj/legacy_v1.o" "$WORK/obj/snapshot_pinned.o" \
  "$WORK/obj/snapshot.o" -lm -o "$WORK/bin/make_recording" || die "build make_recording"

"$CC" $CFLAGS -I"$ACC" -I"$ACC/legacy" "$ACC/tools/client_old.c" \
  "$WORK/obj/recording.o" "$WORK/obj/render.o" "$WORK/obj/legacy_v1.o" "$WORK/obj/snapshot_pinned.o" \
  -lm -o "$WORK/bin/client_old" || die "build client_old"

"$CC" $CFLAGS -I"$ACC" -I"$ACC/legacy" -I"$ROOT/snapshot/src" "$ACC/tools/client_new.c" \
  "$WORK/obj/recording.o" "$WORK/obj/render.o" "$WORK/obj/snapshot.o" \
  -lm -o "$WORK/bin/client_new" || die "build client_new"

# ---------------------------------------------------------------------------
# 3. recording: generate and pin, or verify the committed one
# ---------------------------------------------------------------------------
log "3. replay recording"
if (( REGEN )); then
  "$WORK/bin/make_recording" "$REC" | tee "$EVI/recording.log" || note_fail "recording generation"
else
  # Regenerate into a work dir and diff against the committed recording; the
  # committed recording stays authoritative but must be reproducible.
  mkdir -p "$WORK/rec"
  "$WORK/bin/make_recording" "$WORK/rec" > "$EVI/recording.log" || note_fail "recording generation"
  for f in replay.bin replay_v1.bin replay_naive.bin; do
    if cmp -s "$REC/$f" "$WORK/rec/$f"; then
      echo "recording $f: reproducible (matches committed)"
    else
      echo "recording $f: differs from committed (run --regenerate if intentional)"
      note_fail "recording $f not reproducible"
    fi
  done
fi
cat "$EVI/recording.log"
( cd "$REC" && sha256sum *.bin > SHA256SUMS )
cat "$REC/SHA256SUMS" | tee -a "$EVI/recording.log"

# ---------------------------------------------------------------------------
# 4. acceptance driver
# ---------------------------------------------------------------------------
log "4. compatibility matrix + error table + package hashes"
"$WORK/bin/accept" "$EVI" "$REC" | tee "$EVI/accept_stdout.log"
ACCEPT_RC=${PIPESTATUS[0]}
(( ACCEPT_RC == 0 )) || note_fail "accept driver returned $ACCEPT_RC"

# ---------------------------------------------------------------------------
# 5. two independent client binaries on the committed recording
# ---------------------------------------------------------------------------
log "5. old vs new client binaries"
OLD_OUT="$("$WORK/bin/client_old" "$REC/replay.bin" "$WORK/old.ppm")"
NEW_OUT="$("$WORK/bin/client_new" "$REC/replay.bin" "$WORK/new.ppm")"
OLD_HASH="$(printf '%s\n' "$OLD_OUT" | sed -n 's/.*sha256=//p')"
NEW_HASH="$(printf '%s\n' "$NEW_OUT" | sed -n 's/.*sha256=//p')"
echo "$OLD_OUT"
echo "$NEW_OUT"
if cmp -s "$WORK/old.ppm" "$WORK/new.ppm" && [[ "$OLD_HASH" == "$NEW_HASH" && -n "$OLD_HASH" ]]; then
  PPM_SAME=1; echo "framebuffer bytes: identical"
else
  PPM_SAME=0; echo "framebuffer bytes: DIFFER"; note_fail "two-client framebuffer mismatch"
fi
# old client on the pure-v1 recording is the "old version" reference
V1_OUT="$("$WORK/bin/client_old" "$REC/replay_v1.bin")"
V1_HASH="$(printf '%s\n' "$V1_OUT" | sed -n 's/.*sha256=//p')"
echo "old client on pure-v1 recording: $V1_OUT"
if [[ "$V1_HASH" == "$OLD_HASH" ]]; then
  echo "old-version vs new-recording: identical (pixel-identical replay)"
else
  echo "old-version vs new-recording: DIFFER"; note_fail "old client differs between v1-only and new recording"
fi
NAIVE_OUT="$("$WORK/bin/client_old" "$REC/replay_naive.bin")"
NAIVE_HASH="$(printf '%s\n' "$NAIVE_OUT" | sed -n 's/.*sha256=//p')"
echo "negative control (old client on naive recording): $NAIVE_OUT"
if [[ "$NAIVE_HASH" != "$OLD_HASH" ]]; then
  echo "negative control: differs from good replay (as required)"
else
  echo "negative control: DEAD (naive layout renders identically)"
  note_fail "negative control did not diverge"
fi
{
  echo "# Two-client replay evidence"
  echo "recording=$REC/replay.bin sha256=$(cut -d' ' -f1 "$REC/SHA256SUMS" | head -1)"
  echo "old_client_sha256=$OLD_HASH"
  echo "new_client_sha256=$NEW_HASH"
  echo "framebuffers_identical=$PPM_SAME"
  echo "naive_control_sha256=$NAIVE_HASH"
  echo "naive_control_differs=$([[ "$NAIVE_HASH" != "$OLD_HASH" ]] && echo 1 || echo 0)"
  echo "old_vs_pure_v1_identical=$([[ "$V1_HASH" == "$OLD_HASH" ]] && echo 1 || echo 0)"
} | tee "$EVI/two_client.log"

# ---------------------------------------------------------------------------
# 6. committed golden package-body hashes
# ---------------------------------------------------------------------------
log "6. golden package-body hashes"
# body_hashes.csv columns: module,producer,body,bytes,sha256
python3 - "$EVI/body_hashes.csv" "$GOLD/bodies.sha256" "$REGEN" <<'PY'
import sys
rows = [l.strip().split(',') for l in open(sys.argv[1]) if l.strip() and not l.startswith('module')]
gold = sys.argv[2]
regen = sys.argv[3] == '1'
cur = {",".join(r[:3]): (r[3], r[4]) for r in rows}
if regen:
    with open(gold, 'w') as f:
        for k in sorted(cur):
            f.write(f"{cur[k][1]}  {k}\n")
    print(f"golden regenerated: {len(cur)} body hashes -> {gold}")
    sys.exit(0)
exp = {}
for line in open(gold):
    line = line.rstrip('\n')
    if not line or line.startswith('#'):
        continue
    digest, key = line.split('  ', 1)
    exp[key] = digest
bad = 0
for k in sorted(exp):
    got = cur.get(k)
    if got is None:
        print(f"MISSING body {k}"); bad += 1
    elif got[1] != exp[k]:
        print(f"HASH DIFF {k}\n  expected {exp[k]}\n  got      {got[1]}"); bad += 1
    else:
        print(f"ok {k} {exp[k][:16]}...")
for k in sorted(cur):
    if k not in exp:
        print(f"UNPINNED new body {k} (run --regenerate if intentional)"); bad += 1
print("golden_body_result=FAIL" if bad else "golden_body_result=PASS")
sys.exit(1 if bad else 0)
PY
(( $? == 0 )) || note_fail "golden body hash mismatch"

# ---------------------------------------------------------------------------
# 7. reproduction commands
# ---------------------------------------------------------------------------
log "7. reproduction commands"
cat > "$EVI/repro.log" <<EOF
# Reproduction (run from repo root: $ROOT)
# Generated against head=$HEAD_SHA, v1 baseline=$BASE_SHA
# at $(date -u +%Y-%m-%dT%H:%M:%SZ)

# 1. one command, clean tree:
accept/run.sh

# 2. per-module self tests:
make -C snapshot test
make -C clock test
make -C aoi test
make -C reconcile test

# 3. the compatibility matrix + pixel identity + package hashes:
accept/build/bin/accept accept/evidence accept/recordings

# 4. two independent clients over the committed recording:
accept/build/bin/client_old accept/recordings/replay.bin
accept/build/bin/client_new accept/recordings/replay.bin
#    both print "sha256=<hash>"; the hashes must be equal.
#    negative control: accept/build/bin/client_old accept/recordings/replay_naive.bin
#    must print a DIFFERENT hash.

# 5. confirm the pinned "old client" really is the v1 baseline:
git show $BASE_SHA:snapshot/src/snapshot.c | cmp - accept/legacy/snapshot_v1.c

# Regenerate the committed recording and golden body hashes only when the wire
# format is intentionally changed (then review accept/golden/ and
# accept/recordings/ diffs):
accept/run.sh --regenerate
EOF
cat "$EVI/repro.log"

# ---------------------------------------------------------------------------
# 8. hard gate
# ---------------------------------------------------------------------------
log "8. evidence completeness gate"
REQUIRED=(
  "$EVI/error_codes.md:error-code table"
  "$EVI/compat_matrix.csv:compatibility matrix"
  "$EVI/pixel_identity.csv:pixel identity"
  "$EVI/body_hashes.csv:package-body hashes"
  "$EVI/two_client.log:two-client replay"
  "$EVI/repro.log:reproduction commands"
)
MISSING=0
for spec in "${REQUIRED[@]}"; do
  file="${spec%%:*}"; label="${spec##*:}"
  if [[ -s "$file" ]]; then
    printf 'PRESENT  %-24s %s (%s bytes)\n' "$label" "${file#$ROOT/}" "$(wc -c < "$file")"
  else
    printf 'MISSING  %-24s %s\n' "$label" "${file#$ROOT/}"; MISSING=1
  fi
done

# Matrix must be complete and all-pass: any row with FAIL is fatal.
MTX_FAIL="$(grep -c ',FAIL,' "$EVI/compat_matrix.csv" || true)"
MTX_ROWS="$(($(wc -l < "$EVI/compat_matrix.csv") - 1))"
echo "matrix_rows=$MTX_ROWS matrix_fail_rows=$MTX_FAIL"

echo
echo "summary: failed=$FAILED missing_evidence=$MISSING matrix_fail=$MTX_FAIL ppm_identical=$PPM_SAME"
if (( FAILED || MISSING || MTX_FAIL )); then
  echo "ACCEPTANCE=FAIL"
  exit 1
fi
echo "ACCEPTANCE=PASS"
