
/* clock: network clock offset estimation with min-RTT filtering. */
#include "../../common/ftrace.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_SAMPLES 16

typedef struct { double rtt; double offset; double at; } Sample;

typedef struct {
    Sample s[MAX_SAMPLES];
    int n;
    double best_rtt;
    double offset;
} Sync;

static void sync_init(Sync* sy) { memset(sy, 0, sizeof(*sy)); sy->best_rtt = 1e9; }

static int sync_add(Sync* sy, double t_send, double t_recv, double remote_time) {
    if (t_recv < t_send) return -1;
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

#if FT_ENABLE
#include "../../common/ft_accept.h"

/* Unified-timing acceptance: deterministic exact scenarios plus a
 * real CLOCK_MONOTONIC frame loop exercising this module's workload. */
static int ft_run_clock(void) {
    ft_deterministic_accept("clock", "clk");

    enum { FT_FRAMES = 64, FT_ITERS = 3000 };
    ft_t t;
    ft_init(&t);
    volatile double sink = 0.0;

    for (int f = 0; f < FT_FRAMES; f++) {
        Sync sy;
        sync_init(&sy);
        ft_frame_begin(&t);
        int sp = ft_span_start(&t, "clk.sample");
        for (int k = 0; k < FT_ITERS; k++) {
            double ts = (double)k * 0.01;
            sync_add(&sy, ts, ts + 0.05 + (k % 5) * 0.01, ts + 0.025 + 2.0);
        }
        sink += sy.offset;
        ft_span_end(&t, sp);
        int fl = ft_span_start(&t, "clk.filter");   /* disjoint span, same frame */
        double acc = 0.0;
        for (int k = 0; k < sy.n; k++) acc += sy.s[k].rtt;
        sink += acc;
        ft_span_end(&t, fl);
        ft_frame_end(&t);
    }

    const FtReport* r = ft_report(&t);
    check("ft_clk_real_frames", r->frames == FT_FRAMES);
    check("ft_clk_real_gap_positive", r->gap_sum >= 0.0);
    check("ft_clk_real_identity",
          fabs(r->diff_sum - (r->overlap_sum - r->gap_sum)) <= r->eps_sum);
    check("ft_clk_real_bound", r->abs_diff_sum <= r->bound_sum + r->eps_sum);
    check("ft_clk_real_verify", ft_verify(&t) == 0);
    ft_print_report(&t, "clock real-clock workload", stdout);
    (void)sink;
    return ft_verify(&t) == 0 ? 0 : 1;
}
#endif

int main(void) {
#if FT_ENABLE
    if (ft_run_clock() != 0) return 1;
#endif
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
