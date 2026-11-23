/*
 * capture_snapshot.c - canonical success-path body for snapshot.
 *
 * Two build modes (selected by the acceptance harness):
 *   ML_PRISTINE defined: textually include the pristine git-HEAD .c, whose
 *                        functions are static and whose main is renamed so it
 *                        does not collide with this file's main.
 *   otherwise:           include the refactored header and link the module
 *                        object, calling the public snap_* API.
 *
 * Both modes build the identical MLBody below; the harness sha256-compares
 * the outputs, so any success-path drift fails acceptance.
 * Body = exact wire bytes emitted by encode(): varint tick + zigzag x/y.
 */
#include <stdio.h>
#include <string.h>

#include "../minilab.h"

#if defined(ML_PRISTINE)
#  define main minilab_pristine_main
#  include ML_PRISTINE_SRC
#  undef main
#  define CALL_ENCODE(s, p, sc) encode((s), (p), (sc))
#else
#  include "snapshot.h"
#  define CALL_ENCODE(s, p, sc) snap_encode((s), (p), (sc))
#endif

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    Packet p;
    State in = { 1000u, 12.5f, -3.25f, 0, 0, 1u };
    memset(&p, 0, sizeof(p));
    int n = CALL_ENCODE(&in, &p, 64.0f);
    if (n <= 0) { fprintf(stderr, "capture_snapshot: encode failed\n"); return 1; }

    MLBody b;
    ml_reset(&b);
    ml_bytes(&b, p.buf, p.len);           /* the packet payload, nothing more */

    FILE* f = fopen(argv[1], "wb");
    if (!f) { perror("fopen"); return 2; }
    fwrite(b.data, 1, (size_t)b.len, f);
    fclose(f);
    printf("snapshot len=%d\n", b.len);
    return 0;
}
