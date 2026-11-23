/*
 * capture_clock.c - canonical success-path body for clock.
 * Two build modes; see capture_snapshot.c for the contract.
 * Body = sample ring contents plus the filter output the client relies on.
 */
#include <stdio.h>

#include "../minilab.h"

#if defined(ML_PRISTINE)
#  define main minilab_pristine_main
#  include ML_PRISTINE_SRC
#  undef main
#  define CALL_INIT(sy)      sync_init((sy))
#  define CALL_ADD(sy,a,b,c) sync_add((sy), (a), (b), (c))
#else
#  include "clock.h"
#  define CALL_INIT(sy)      clock_sync_init((sy))
#  define CALL_ADD(sy,a,b,c) clock_sync_add((sy), (a), (b), (c))
#endif

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    Sync sy;
    CALL_INIT(&sy);
    double true_off = 2.0;
    if (CALL_ADD(&sy, 0.0, 0.10, 0.05 + true_off) <= 0) {
        fprintf(stderr, "capture_clock: sync_add failed\n");
        return 1;
    }

    MLBody b;
    ml_reset(&b);
    ml_i32(&b, sy.n);
    for (int i = 0; i < sy.n; i++) {
        ml_f64(&b, sy.s[i].rtt);
        ml_f64(&b, sy.s[i].offset);
        ml_f64(&b, sy.s[i].at);
    }
    ml_f64(&b, sy.best_rtt);
    ml_f64(&b, sy.offset);

    FILE* f = fopen(argv[1], "wb");
    if (!f) { perror("fopen"); return 2; }
    fwrite(b.data, 1, (size_t)b.len, f);
    fclose(f);
    printf("clock len=%d\n", b.len);
    return 0;
}
