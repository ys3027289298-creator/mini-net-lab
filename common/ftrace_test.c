/*
 * ftrace_test.c — library-level acceptance for the unified timing API.
 * Build with -DFT_ENABLE=1; uses the injected virtual clock so the
 * reconciliation numbers are exact (resolution 0, no scheduling noise).
 */
#include "ftrace.h"

#include <stdio.h>

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Drive a frame by stepping one virtual clock variable. */
static double vnow;
static void at(ft_t* t, double sec) { vnow = sec; (void)t; }

int main(void) {
    ft_t t;
    ft_init(&t);
    ft_set_clock(&t, ft_virtual_clock, &vnow);

    /* frame 0, frame interval [0,10ms): partial sibling overlaps
     * A[1,4) B[3,6) C[5,8): sum 9, union 7, overlap 2, gap 3, diff -1 */
    ft_frame_begin(&t);
    at(&t, 0.001); int a = ft_span_start(&t, "alpha");   at(&t, 0.004); ft_span_end(&t, a);
    at(&t, 0.003); int b = ft_span_start(&t, "bravo");   at(&t, 0.006); ft_span_end(&t, b);
    at(&t, 0.005); int c = ft_span_start(&t, "charlie"); at(&t, 0.008); ft_span_end(&t, c);
    at(&t, 0.010); ft_frame_end(&t);

    /* frame 1: nested outer[0,8) inner[2,5): sum 11, union 8,
     * overlap 3, gap 2, diff +1 */
    ft_frame_begin(&t);
    at(&t, 0.020); int o = ft_span_start(&t, "outer");
    at(&t, 0.022); int i = ft_span_start(&t, "inner");   at(&t, 0.025); ft_span_end(&t, i);
    at(&t, 0.028); ft_span_end(&t, o);
    at(&t, 0.030); ft_frame_end(&t);

    /* frame 2: disjoint [1,2) [3,4): sum 2, union 2, overlap 0, gap 8, diff -2 */
    ft_frame_begin(&t);
    at(&t, 0.031); int d1 = ft_span_start(&t, "d1");     at(&t, 0.032); ft_span_end(&t, d1);
    at(&t, 0.033); int d2 = ft_span_start(&t, "d2");     at(&t, 0.034); ft_span_end(&t, d2);
    at(&t, 0.040); ft_frame_end(&t);

    const FtReport* r = ft_report(&t);
    /* virtual frame spans: [0,10ms) [10,30ms) [30,40ms) -> 40ms total */
    check("frames_3", r->frames == 3);
    check("frame_total_40ms", fabs(r->frame_total_sum - 0.040) < r->eps_sum);
    check("span_sum_22ms", fabs(r->span_sum_sum - 0.022) < r->eps_sum);
    check("union_17ms", fabs(r->union_sum - 0.017) < r->eps_sum);
    check("overlap_5ms", fabs(r->overlap_sum - 0.005) < r->eps_sum);
    check("gap_23ms", fabs(r->gap_sum - 0.023) < r->eps_sum);
    check("diff_minus_18ms", fabs(r->diff_sum - (-0.018)) < r->eps_sum);
    check("abs_diff_18ms", fabs(r->abs_diff_sum - 0.018) < r->eps_sum);
    check("identity_diff_equals_overlap_minus_gap",
          fabs(r->diff_sum - (r->overlap_sum - r->gap_sum)) <= r->eps_sum);
    check("bound_covers_abs_diff", r->abs_diff_sum <= r->bound_sum + r->eps_sum);
    check("bound_equals_28ms", fabs(r->bound_sum - 0.028) < r->eps_sum * 8.0);
    check("overlap_pairs_logged_3", r->overlap_count == 3);
    check("no_invariant_violation", r->invariant_violations == 0);
    check("verify_ok", ft_verify(&t) == 0);
    check("metrics_7", r->metric_count == 7);

    ft_print_report(&t, "ftrace virtual-clock selftest", stdout);

    /* negative control: a tampered report must be rejected */
    ft_t bad = t;
    ((FtReport*)ft_report(&bad))->bound_sum = 0.0;
    check("verifier_rejects_tampered_report", ft_verify(&bad) != 0);

    /* span slot exhaustion / no-frame calls must not crash */
    ft_t edge; ft_init(&edge); ft_set_clock(&edge, ft_virtual_clock, &vnow);
    check("span_without_frame_rejected", ft_span_start(&edge, "x") == -1);
    ft_frame_begin(&edge);
    int handles[FT_MAX_SPANS_PER_FRAME + 2];
    for (int k = 0; k < FT_MAX_SPANS_PER_FRAME + 2; k++) { handles[k] = ft_span_start(&edge, "edge"); vnow += 0.001; }
    check("span_slots_capped", handles[FT_MAX_SPANS_PER_FRAME] == -1);
    for (int k = 0; k < FT_MAX_SPANS_PER_FRAME; k++) ft_span_end(&edge, handles[k]);
    ft_span_end(&edge, -1);
    vnow += 0.001; ft_frame_end(&edge);
    check("edge_verify_ok", ft_verify(&edge) == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
