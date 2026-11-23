/*
 * ft_accept.h — per-module acceptance harness for unified timing.
 * Included by each module's source with FT_ENABLE=1. The deterministic
 * part drives the same injected virtual clock in every module, so the
 * reconciliation numbers are identical and exact (resolution 0).
 *
 * Requires the including file to provide check(name, ok) and <stdio.h>.
 */
#ifndef FT_ACCEPT_H
#define FT_ACCEPT_H

#if FT_ENABLE

#include "ftrace.h"

static double ft_acc_vnow;
static inline void ft_acc_at(ft_t* t, double sec) { ft_acc_vnow = sec; (void)t; }

/*
 * Three frames with known geometry (virtual seconds relative to each
 * frame's begin):
 *   f0 sibling overlaps: a[1,4) b[3,6) c[5,8), frame [0,10)
 *   f1 nesting:          outer[0,8) inner[2,5),   frame [10,30)
 *   f2 disjoint:         d1[1,2) d2[3,4),         frame [30,40)
 * Totals: frame 40ms, span_sum 22ms, union 17ms, overlap 5ms,
 *         gap 23ms, diff -18ms, bound 28ms.
 */
static inline void ft_deterministic_accept(const char* module, const char* tag) {
    ft_t t;
    ft_init(&t);
    ft_set_clock(&t, ft_virtual_clock, &ft_acc_vnow);

    ft_frame_begin(&t);
    ft_acc_at(&t, 0.001); int a = ft_span_start(&t, "alpha");   ft_acc_at(&t, 0.004); ft_span_end(&t, a);
    ft_acc_at(&t, 0.003); int b = ft_span_start(&t, "bravo");   ft_acc_at(&t, 0.006); ft_span_end(&t, b);
    ft_acc_at(&t, 0.005); int c = ft_span_start(&t, "charlie"); ft_acc_at(&t, 0.008); ft_span_end(&t, c);
    ft_acc_at(&t, 0.010); ft_frame_end(&t);

    ft_frame_begin(&t);
    ft_acc_at(&t, 0.020); int o = ft_span_start(&t, "outer");
    ft_acc_at(&t, 0.022); int i = ft_span_start(&t, "inner");   ft_acc_at(&t, 0.025); ft_span_end(&t, i);
    ft_acc_at(&t, 0.028); ft_span_end(&t, o);
    ft_acc_at(&t, 0.030); ft_frame_end(&t);

    ft_frame_begin(&t);
    ft_acc_at(&t, 0.031); int d1 = ft_span_start(&t, "d1");    ft_acc_at(&t, 0.032); ft_span_end(&t, d1);
    ft_acc_at(&t, 0.033); int d2 = ft_span_start(&t, "d2");    ft_acc_at(&t, 0.034); ft_span_end(&t, d2);
    ft_acc_at(&t, 0.040); ft_frame_end(&t);

    const FtReport* r = ft_report(&t);
    char name[96];
    snprintf(name, sizeof(name), "ft_%s_det_frames", tag);        check(name, r->frames == 3);
    snprintf(name, sizeof(name), "ft_%s_det_frame_total", tag);   check(name, fabs(r->frame_total_sum - 0.040) <= r->eps_sum);
    snprintf(name, sizeof(name), "ft_%s_det_span_sum", tag);      check(name, fabs(r->span_sum_sum - 0.022) <= r->eps_sum);
    snprintf(name, sizeof(name), "ft_%s_det_union", tag);         check(name, fabs(r->union_sum - 0.017) <= r->eps_sum);
    snprintf(name, sizeof(name), "ft_%s_det_overlap", tag);       check(name, fabs(r->overlap_sum - 0.005) <= r->eps_sum);
    snprintf(name, sizeof(name), "ft_%s_det_gap", tag);           check(name, fabs(r->gap_sum - 0.023) <= r->eps_sum);
    snprintf(name, sizeof(name), "ft_%s_det_diff", tag);          check(name, fabs(r->diff_sum - (-0.018)) <= r->eps_sum);
    snprintf(name, sizeof(name), "ft_%s_det_identity", tag);
    check(name, fabs(r->diff_sum - (r->overlap_sum - r->gap_sum)) <= r->eps_sum);
    snprintf(name, sizeof(name), "ft_%s_det_bound", tag);         check(name, r->abs_diff_sum <= r->bound_sum + r->eps_sum);
    snprintf(name, sizeof(name), "ft_%s_det_overlap_pairs", tag); check(name, r->overlap_count == 3);
    snprintf(name, sizeof(name), "ft_%s_det_verify", tag);        check(name, ft_verify(&t) == 0);

    char title[128];
    snprintf(title, sizeof(title), "%s deterministic (%s)", module, tag);
    ft_print_report(&t, title, stdout);
}

#endif /* FT_ENABLE */
#endif /* FT_ACCEPT_H */
