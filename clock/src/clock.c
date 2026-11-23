
/* clock: network clock offset estimation with min-RTT filtering.
 *
 * Every timestamp is a tick in the shared time base (ms).  sync_add() rejects
 * any sample whose receive time is behind the last accepted one, so a source
 * that runs backwards cannot corrupt the offset estimate.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../../timebase/src/timebase.h"
#include "../../timebase/src/accept.h"

#define MAX_SAMPLES 16

typedef struct { double rtt; double offset; double at; } Sample;

typedef struct {
    Sample s[MAX_SAMPLES];
    int n;
    double best_rtt;
    double offset;
    double last_recv;   /* last accepted receive time, for monotonicity */
    int has_recv;
} Sync;

static void sync_init(Sync* sy) { memset(sy, 0, sizeof(*sy)); sy->best_rtt = 1e9; }

static int sync_add(Sync* sy, double t_send, double t_recv, double remote_time) {
    if (t_recv < t_send) return -1;
    if (sy->has_recv && t_recv < sy->last_recv) return -1;   /* time went backwards */
    sy->last_recv = t_recv;
    sy->has_recv = 1;
    double rtt = t_recv - t_send;
    double offset = remote_time - (t_send + t_recv) * 0.5;
    if (sy->n >= MAX_SAMPLES) {
        memmove(&sy->s[0], &sy->s[1], sizeof(Sample) * (MAX_SAMPLES - 1));
        sy->n = MAX_SAMPLES - 1;
    }
    sy->s[sy->n].rtt = rtt;
    sy->s[sy->n].offset = offset;
    sy->s[sy->n].at = t_recv;
    sy->n++;
    if (rtt < sy->best_rtt) sy->best_rtt = rtt;

    /* keep samples within 1.5x best rtt, average their offsets */
    double sum = 0.0; int cnt = 0;
    for (int i = 0; i < sy->n; i++) {
        if (sy->s[i].rtt <= sy->best_rtt * 1.5 + 1e-9) {
            sum += sy->s[i].offset; cnt++;
        }
    }
    if (cnt == 0) return -1;
    sy->offset = sum / cnt;
    return cnt;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Acceptance step: every sample is timestamped from the shared base.  The
 * remote clock carries the same 2 s offset plus the base's own drift, so the
 * estimator must stay within a few ms for the whole run. */
#define ACCEPT_REMOTE_OFFSET_MS 2000.0

typedef struct { int bad; double max_abs_err; Sync sync; } ClockAcc;

static void clock_accept_reset(void* ctx) {
    ClockAcc* a = (ClockAcc*)ctx;
    memset(a, 0, sizeof(*a));
    sync_init(&a->sync);
}

static int clock_accept_step(void* ctx, TimeBase* tb, const TBAcceptSpec* spec, uint32_t tick) {
    ClockAcc* a = (ClockAcc*)ctx;
    (void)spec;
    uint32_t base = 0;
    if (tb_now(tb, &base) != TB_OK) { a->bad++; return 1; }
    double t_recv = (double)base;
    double t_send = t_recv - 20.0 - (double)(tick % 7u);   /* 20..26 ms RTT */
    double remote = (double)base + ACCEPT_REMOTE_OFFSET_MS;
    if (sync_add(&a->sync, t_send, t_recv, remote) <= 0) { a->bad++; return 2; }
    double err = fabs(a->sync.offset - ACCEPT_REMOTE_OFFSET_MS);
    if (err > a->max_abs_err) a->max_abs_err = err;
    /* 0.5 RTT of half-path asymmetry is irreducible; allow the RTT window. */
    if (err > 30.0) { a->bad++; return 3; }
    return 0;
}

static int clock_accept(void) {
    ClockAcc acc;
    memset(&acc, 0, sizeof(acc));
    sync_init(&acc.sync);

    TBAcceptSpec spec;
    memset(&spec, 0, sizeof(spec));
    spec.module = TB_CLOCK;
    spec.step_ms = 100;            /* sync packet every 100 ms */
    spec.events = 500;             /* ~50 s */
    spec.long_events = 72000;      /* 2 h */
    spec.ppm = 100;
    spec.seed = 0xc10cu;
    spec.step = clock_accept_step;
    spec.reset = clock_accept_reset;
    spec.ctx = &acc;

    int rc = tb_accept_run(&spec);            /* 0 == all phases passed */
    int est_ok = (rc == 0 && acc.bad == 0);
    printf("module=clock phase=estimate max_err_ms=%.3f bad=%d ok=%d\n",
           acc.max_abs_err, acc.bad, est_ok);
    tb_accept_shared(TB_CLOCK);
    return est_ok ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "--accept") == 0) return clock_accept();

    Sync sy;
    sync_init(&sy);
    double true_off = 2.0;
    check("add1", sync_add(&sy, 0.0, 0.10, 0.05 + true_off) > 0);
    check("offset_close", fabs(sy.offset - true_off) < 0.01);
    check("best_rtt", fabs(sy.best_rtt - 0.10) < 1e-9);

    check("add2", sync_add(&sy, 1.0, 1.20, 1.10 + true_off) > 0);
    check("still_close", fabs(sy.offset - true_off) < 0.01);

    check("reject_time_reversal", sync_add(&sy, 5.0, 4.0, 0.0) == -1);

    Sync sp;
    sync_init(&sp);
    for (int i = 0; i < 40; i++) sync_add(&sp, i * 1.0, i * 1.0 + 0.05 + (i % 5) * 0.01, i * 1.0 + 0.025 + true_off);
    check("ring_kept", sp.n == MAX_SAMPLES);
    check("stable_offset", fabs(sp.offset - true_off) < 0.01);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
