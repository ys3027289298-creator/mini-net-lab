
/* reconcile: prediction buffer with rollback on authoritative state. */
#include "../../common/ftrace.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define BUF 64

typedef struct { unsigned tick; float x; } Pred;

typedef struct {
    Pred p[BUF];
    int head;      /* next write */
    int count;
    unsigned acked;
    int rollbacks;
    float last_err;
} Recon;

static void recon_init(Recon* r) { memset(r, 0, sizeof(*r)); }

static int recon_push(Recon* r, unsigned tick, float x) {
    if (tick <= r->acked) return -1;                 /* stale tick rejected */
    for (int i = 0; i < r->count; i++) {             /* duplicate tick anywhere */
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->p[idx].tick == tick) return -1;
    }
    r->p[r->head].tick = tick;
    r->p[r->head].x = x;
    r->head = (r->head + 1) % BUF;
    if (r->count < BUF) r->count++;
    return 0;
}

/* authoritative state arrives for tick A: drop acked, count rollback if mismatch */
static int recon_ack(Recon* r, unsigned tick, float x) {
    float pred = 0.0f; int found = 0;
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->p[idx].tick == tick) { pred = r->p[idx].x; found = 1; break; }
    }
    r->last_err = found ? fabsf(pred - x) : -1.0f;
    if (found && fabsf(pred - x) > 1e-3f) r->rollbacks++;
    if (tick > r->acked) r->acked = tick;
    return found ? 1 : 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

#if FT_ENABLE
#include "../../common/ft_accept.h"

/* Unified-timing acceptance: deterministic exact scenarios plus a
 * real CLOCK_MONOTONIC frame loop exercising this module's workload. */
static int ft_run_reconcile(void) {
    ft_deterministic_accept("reconcile", "rec");

    enum { FT_FRAMES = 64, FT_ITERS = 500 };
    ft_t t;
    ft_init(&t);
    volatile int sink = 0;

    for (int f = 0; f < FT_FRAMES; f++) {
        Recon r;
        recon_init(&r);
        ft_frame_begin(&t);
        int outer = ft_span_start(&t, "rec.frame_outer");   /* wraps push + ack */
            int psh = ft_span_start(&t, "rec.push");
            for (int k = 1; k <= FT_ITERS; k++) sink += recon_push(&r, (unsigned)k, (float)k) == 0;
            ft_span_end(&t, psh);
            int ack = ft_span_start(&t, "rec.ack");
            for (int k = 1; k <= FT_ITERS; k++) sink += recon_ack(&r, (unsigned)k, (float)k + 0.25f);
            ft_span_end(&t, ack);
        ft_span_end(&t, outer);
        int scan = ft_span_start(&t, "rec.scan");          /* disjoint, after the outer */
        int live = 0;
        for (int k = 0; k < r.count; k++) if (r.p[k].tick > 0) live++;
        sink += live + r.rollbacks;
        ft_span_end(&t, scan);
        ft_frame_end(&t);
    }

    const FtReport* r = ft_report(&t);
    check("ft_rec_real_frames", r->frames == FT_FRAMES);
    check("ft_rec_real_overlap_positive", r->overlap_sum > 0.0);
    check("ft_rec_real_gap_nonneg", r->gap_sum >= 0.0);
    check("ft_rec_real_identity",
          fabs(r->diff_sum - (r->overlap_sum - r->gap_sum)) <= r->eps_sum);
    check("ft_rec_real_bound", r->abs_diff_sum <= r->bound_sum + r->eps_sum);
    check("ft_rec_real_verify", ft_verify(&t) == 0);
    ft_print_report(&t, "reconcile real-clock workload", stdout);
    (void)sink;
    return ft_verify(&t) == 0 ? 0 : 1;
}
#endif

int main(void) {
#if FT_ENABLE
    if (ft_run_reconcile() != 0) return 1;
#endif
    Recon r;
    recon_init(&r);
    check("push1", recon_push(&r, 1, 1.0f) == 0);
    check("push2", recon_push(&r, 2, 2.0f) == 0);
    check("dup_rejected", recon_push(&r, 2, 5.0f) == -1);

    check("ack_match", recon_ack(&r, 2, 2.0f) == 1);
    check("no_rollback", r.rollbacks == 0);

    check("stale_rejected", recon_push(&r, 2, 3.0f) == -1);

    recon_push(&r, 3, 3.0f);
    check("ack_mismatch", recon_ack(&r, 3, 3.5f) == 1);
    check("rollback_counted", r.rollbacks == 1);
    check("err_recorded", fabsf(r.last_err - 0.5f) < 1e-5f);

    check("ack_missing", recon_ack(&r, 99, 0.0f) == 0);

    Recon big;
    recon_init(&big);
    for (int i = 1; i <= 70; i++) recon_push(&big, (unsigned)i, (float)i);
    check("buffer_capped", big.count == BUF);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
