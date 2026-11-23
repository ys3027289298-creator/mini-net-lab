#!/usr/bin/env bash
#
# audit_no_copy.sh - static evidence that no module performs a second payload
# copy. The modules must operate in place on the caller's storage: no heap
# allocation of payloads, no strdup, and no memcpy/memmove of a payload-sized
# object except an explicit, line-annotated allowlist (internal ring eviction
# / zero-init, never a copy handed back to the caller).
#
# Exit 0 = clean, 1 = forbidden construct found.
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC_DIRS=(snapshot clock aoi reconcile)

# Functions that would introduce an extra payload allocation or copy.
FORBIDDEN='\b(malloc|calloc|realloc|free|strdup|asprintf|alloca)\b'

# memcpy/memmove call sites that are legitimate internal maintenance rather
# than a second payload copy. Each entry is a literal substring that must
# appear on the matched line; any unlisted site fails the audit.
ALLOW=(
  # clock ring eviction: moves samples within the SAME caller-owned ring
  'memmove(&sy->s[0], &sy->s[1]'
  # init helpers zero the caller's struct in place (no second object)
  'memset(sy, 0, sizeof(*sy))'
  'memset(w, 0, sizeof(*w))'
  'memset(r, 0, sizeof(*r))'
  # selftest scaffolding: initializes local test packets, not module payloads
  'memset(&p, 0, sizeof(p))'
  'memset(&p2, 0, sizeof(p2))'
)

fail=0
echo "# Static no-second-payload-copy audit"
echo "# root: $ROOT"
echo

for d in "${SRC_DIRS[@]}"; do
  for src in "$ROOT/$d"/src/*.c; do
    echo "## $src"
    hits="$(grep -nE "$FORBIDDEN" "$src" || true)"
    if [[ -n "$hits" ]]; then
      echo "FORBIDDEN allocator/copy:"
      echo "$hits" | sed 's/^/  /'
      fail=1
    else
      echo "  no heap/payload allocator: OK"
    fi

    while IFS= read -r line; do
      [[ -z "$line" ]] && continue
      ok=0
      for a in "${ALLOW[@]}"; do
        if [[ "$line" == *"$a"* ]]; then ok=1; break; fi
      done
      if (( ok )); then
        echo "  allowed mem-site: $line"
      else
        echo "  UNALLOWED memcpy/memmove: $line"
        fail=1
      fi
    done < <(grep -nE '\b(memcpy|memmove)\b' "$src" || true)
    echo
  done
done

# The shared serializer header copies into the single caller-provided MLBody;
# document and verify its only mem operations explicitly.
HDR="$ROOT/accept/minilab.h"
echo "## $HDR (shared canonical serializer)"
while IFS= read -r line; do
  case "$line" in
    *"memcpy(&u, &f"*|*"memcpy(&u, &d"*|*"memcpy(b->data + b->len"*)
      echo "  allowed in-place serialization: $line" ;;
    *)
      echo "  UNALLOWED mem op: $line"; fail=1 ;;
  esac
done < <(grep -nE '\b(memcpy|memmove)\b' "$HDR" || true)

echo
if (( fail )); then
  echo "audit_result=FAIL"
  exit 1
fi
echo "audit_result=PASS"
