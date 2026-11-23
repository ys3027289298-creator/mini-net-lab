#!/usr/bin/env bash
#
# run.sh - end-to-end failure-isolation acceptance.
#
# Pipeline:
#   1. export PRISTINE module sources from git HEAD (the pre-refactor tree)
#      and build capture_* tools against them -> golden packet bodies
#   2. build the same capture tools against the REFACTORED working-tree
#      module sources -> current packet bodies
#   3. sha256 both sets: success path must be byte-identical
#   4. build accept.c against refactored objects -> error table, injection
#      log + 4x4 isolation matrix, dynamic no-copy log
#   5. static no-copy audit
#   6. hard gate: the four required evidence kinds must all exist; any
#      missing piece makes the run fail.
#
# Reproduction:  accept/run.sh            (verify against committed goldens)
# Golden regen:  accept/run.sh --regenerate
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ACC="$ROOT/accept"
EVI="$ACC/evidence"
GOLD="$ACC/golden"
WORK="$ACC/build"
CC="${CC:-gcc}"
CFLAGS="-O2 -std=c11 -Wall -Wextra"

REGEN=0
[[ "${1:-}" == "--regenerate" ]] && REGEN=1

log()  { printf '\n=== %s ===\n' "$*"; }
die()  { echo "FATAL: $*" >&2; exit 1; }

rm -rf "$WORK"
mkdir -p "$EVI" "$GOLD" "$WORK/pristine" "$WORK/current" "$WORK/obj"

# ---------------------------------------------------------------------------
# 1. pristine sources straight from git HEAD (read-only, isolated directory)
# ---------------------------------------------------------------------------
log "1. export pristine module sources from git HEAD"
HEAD_SHA="$(git -C "$ROOT" rev-parse HEAD)"
echo "HEAD=$HEAD_SHA"
for m in snapshot clock aoi reconcile; do
  git -C "$ROOT" show "HEAD:$m/src/$m.c" > "$WORK/pristine/$m.c" \
    || die "cannot read HEAD:$m/src/$m.c"
done
echo "$HEAD_SHA" > "$WORK/pristine/HEAD.sha"
ls -l "$WORK/pristine"

# Capture tools are dual-mode. Pristine mode textually includes the git-HEAD
# .c (its functions are static); the include renames that .c's main so only
# the capture tool keeps a real main. Current mode includes the module header
# and links the refactored object's public (prefixed) API.
build_capture() { # $1=source dir tag, $2=path to module .c dir, $3=output dir
  local tag="$1" srcdir="$2" outdir="$3"
  for m in snapshot clock aoi reconcile; do
    if [[ "$tag" == pristine ]]; then
      "$CC" $CFLAGS -Wno-unused-function -I"$ACC" \
        -DML_PRISTINE \
        -DML_PRISTINE_SRC="\"$srcdir/$m.c\"" \
        "$ACC/tools/capture_$m.c" -lm \
        -o "$outdir/capture_$m" || die "capture build failed: $tag/$m"
    else
      "$CC" $CFLAGS -I"$ACC" -I"$ROOT/$m/src" \
        "$ACC/tools/capture_$m.c" "$WORK/obj/$m.o" -lm \
        -o "$outdir/capture_$m" || die "capture build failed: $tag/$m"
    fi
  done
}

# ---------------------------------------------------------------------------
# 2a. golden bodies from pristine sources
# ---------------------------------------------------------------------------
log "2a. capture golden bodies from pristine sources"
build_capture pristine "$WORK/pristine" "$WORK/pristine"
for m in snapshot clock aoi reconcile; do
  "$WORK/pristine/capture_$m" "$WORK/pristine/$m.body" || die "golden capture $m"
done
( cd "$WORK/pristine" && sha256sum *.body > SHA256SUMS )
cat "$WORK/pristine/SHA256SUMS"

# ---------------------------------------------------------------------------
# 2b. current bodies from refactored working-tree sources
# ---------------------------------------------------------------------------
log "2b. capture bodies from refactored working-tree sources"
for m in snapshot clock aoi reconcile; do
  "$CC" $CFLAGS -DMINILAB_NO_MAIN -I"$ROOT/$m/src" \
    -c "$ROOT/$m/src/$m.c" -o "$WORK/obj/$m.o" || die "compile $m"
done
build_capture current "$ROOT" "$WORK/current"
for m in snapshot clock aoi reconcile; do
  "$WORK/current/capture_$m" "$WORK/current/$m.body" || die "current capture $m"
done
( cd "$WORK/current" && sha256sum *.body > SHA256SUMS )
cat "$WORK/current/SHA256SUMS"

# ---------------------------------------------------------------------------
# 3. success-path byte identity: golden vs current
# ---------------------------------------------------------------------------
log "3. sha256 golden vs current (success path must be byte-identical)"
HASH_OK=1
: > "$EVI/body_hashes.txt"
{
  echo "# Packet-body sha256: pristine git-HEAD golden vs refactored current"
  echo "# HEAD=$HEAD_SHA"
  printf "%-9s | %-64s | %-64s | %s\n" MODULE GOLDEN_SHA256 CURRENT_SHA256 IDENTICAL
} >> "$EVI/body_hashes.txt"
for m in snapshot clock aoi reconcile; do
  gh="$(sha256sum "$WORK/pristine/$m.body" | awk '{print $1}')"
  ch="$(sha256sum "$WORK/current/$m.body"  | awk '{print $1}')"
  if [[ "$gh" == "$ch" ]]; then state=yes; else state=NO; HASH_OK=0; fi
  printf "%-9s | %s | %s | %s\n" "$m" "$gh" "$ch" "$state" >> "$EVI/body_hashes.txt"
done
cat "$EVI/body_hashes.txt"

# manage committed golden set
if (( REGEN )); then
  cp "$WORK/pristine"/*.body "$GOLD"/
  cp "$WORK/pristine/SHA256SUMS" "$GOLD/SHA256SUMS.pristine"
  log "golden bodies regenerated into accept/golden/"
fi
if ls "$GOLD"/*.body >/dev/null 2>&1; then
  log "3b. verify current bodies against committed accept/golden"
  ( cd "$GOLD" && sha256sum -c SHA256SUMS.pristine ) || HASH_OK=0
  # and confirm current matches the committed golden bytes
  for m in snapshot clock aoi reconcile; do
    cmp -s "$GOLD/$m.body" "$WORK/current/$m.body" \
      || { echo "BODY DIFF: $m"; HASH_OK=0; }
  done
  echo "committed-golden comparison finished"
fi

# ---------------------------------------------------------------------------
# 4. acceptance driver: error table, injection log/matrix, dynamic no-copy
# ---------------------------------------------------------------------------
log "4. build and run acceptance driver"
"$CC" $CFLAGS -I"$ACC" \
  -I"$ROOT/snapshot/src" -I"$ROOT/clock/src" -I"$ROOT/aoi/src" -I"$ROOT/reconcile/src" \
  "$ACC/accept.c" "$WORK/obj"/snapshot.o "$WORK/obj"/clock.o "$WORK/obj"/aoi.o "$WORK/obj"/reconcile.o \
  -lm -o "$WORK/accept" || die "accept build"
"$WORK/accept" "$EVI" | tee "$EVI/accept_stdout.log"
ACCEPT_RC=${PIPESTATUS[0]}

# ---------------------------------------------------------------------------
# 5. static no-copy audit
# ---------------------------------------------------------------------------
log "5. static no-second-copy audit"
bash "$ACC/audit_no_copy.sh" | tee "$EVI/no_copy_audit.log"
AUDIT_RC=${PIPESTATUS[0]}

# ---------------------------------------------------------------------------
# 6. reproduction commands
# ---------------------------------------------------------------------------
log "6. write reproduction commands"
cat > "$EVI/repro.log" <<EOF
# Reproduction (run from repo root: $ROOT)
# Generated against HEAD=$HEAD_SHA at $(date -u +%Y-%m-%dT%H:%M:%SZ)

# Per-module self tests (each includes its own injection check):
make -C snapshot test
make -C clock test
make -C aoi test
make -C reconcile test

# Full failure-isolation acceptance (golden hash + injection matrix + no-copy):
accept/run.sh

# Regenerate golden packet bodies only when the success-path wire format is
# intentionally changed (then re-review the diff in accept/golden/):
accept/run.sh --regenerate

# Direct fault injection per module (self-test binaries already exercise it):
#   snapshot -> SNAP_FAULT_ENCODE / SNAP_FAULT_DECODE
#   clock    -> CLK_FAULT_ADD
#   aoi      -> AOI_FAULT_ADD
#   reconcile-> RECON_FAULT_PUSH
EOF
cat "$EVI/repro.log"

# ---------------------------------------------------------------------------
# 7. hard gate: the four required evidence kinds must all be present
# ---------------------------------------------------------------------------
log "7. evidence completeness gate"
REQUIRED=(
  "$EVI/error_codes.md:error-code table"
  "$EVI/injection.log:failure-injection log"
  "$EVI/body_hashes.txt:packet-body hashes"
  "$EVI/repro.log:reproduction commands"
)
MISSING=0
for spec in "${REQUIRED[@]}"; do
  file="${spec%%:*}"; label="${spec##*:}"
  if [[ -s "$file" ]]; then
    echo "PRESENT  $label -> ${file#$ROOT/} ($(wc -c < "$file") bytes)"
  else
    echo "MISSING  $label -> ${file#$ROOT/}"; MISSING=1
  fi
done

echo
echo "summary: hash_identity=$HASH_OK accept_rc=$ACCEPT_RC audit_rc=$AUDIT_RC missing_evidence=$MISSING"
if (( HASH_OK == 0 || ACCEPT_RC != 0 || AUDIT_RC != 0 || MISSING != 0 )); then
  echo "ACCEPTANCE=FAIL"
  exit 1
fi
echo "ACCEPTANCE=PASS"
