/*
 * capture_reconcile.c - canonical success-path body for reconcile.
 * Two build modes; see capture_snapshot.c for the contract.
 * Body = prediction buffer entries plus counters.
 */
#include <stdio.h>

#include "../minilab.h"

#if defined(ML_PRISTINE)
#  define main minilab_pristine_main
#  include ML_PRISTINE_SRC
#  undef main
#  define CALL_INIT(r)       recon_init((r))
#  define CALL_PUSH(r,t,x)   recon_push((r), (t), (x))
#else
#  include "reconcile.h"
#  define CALL_INIT(r)       recon_init((r))
#  define CALL_PUSH(r,t,x)   recon_push((r), (t), (x))
#endif

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    Recon r;
    CALL_INIT(&r);
    if (CALL_PUSH(&r, 1, 1.0f) != 0) { fprintf(stderr, "capture_reconcile: push1\n"); return 1; }
    if (CALL_PUSH(&r, 2, 2.0f) != 0) { fprintf(stderr, "capture_reconcile: push2\n"); return 1; }
    if (CALL_PUSH(&r, 3, 3.0f) != 0) { fprintf(stderr, "capture_reconcile: push3\n"); return 1; }

    MLBody b;
    ml_reset(&b);
    ml_i32(&b, r.count);
    ml_u32(&b, r.acked);
    ml_i32(&b, r.rollbacks);
    for (int i = 0; i < r.count; i++) {
        int idx = (r.head - 1 - i + BUF * 2) % BUF;
        ml_u32(&b, r.p[idx].tick);
        ml_f32(&b, r.p[idx].x);
    }

    FILE* f = fopen(argv[1], "wb");
    if (!f) { perror("fopen"); return 2; }
    fwrite(b.data, 1, (size_t)b.len, f);
    fclose(f);
    printf("reconcile len=%d\n", b.len);
    return 0;
}
