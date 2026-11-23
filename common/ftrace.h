/*
 * ftrace.h — unified frame timing for mini-net-lab modules.
 *
 * The instrumentation is compiled in only when FT_ENABLE is defined to 1
 * (build with -DFT_ENABLE=1). Otherwise every API is a static inline
 * no-op and no timing code reaches the object file, so un-instrumented
 * builds keep their original output and behavior.
 *
 * Accounting model (active mode):
 *   - one clock source per ft_t: CLOCK_MONOTONIC by default; tests may
 *     inject a virtual clock with ft_set_clock() (resolution becomes 0).
 *   - spans are half-open intervals [start, end) attached to the frame
 *     that was open when ft_span_start() was called.
 *   - per frame, with frame interval [fb, fe), T = fe - fb:
 *       span_sum = sum of (end - start) over the frame's spans
 *       union    = length of the union of span intervals
 *       overlap  = span_sum - union           (multiplicity counted)
 *       gap      = T - union
 *       diff     = span_sum - T = overlap - gap
 *     Identity: span_sum - frame_total == overlap - gap (exact up to
 *     floating-point roundoff), so overlap and gap fully explain why
 *     the per-module sums do not add up to frame time.
 *   - per-frame upper bound, summed in the report:
 *       |diff| <= overlap + gap + eps + resolution
 *     eps = floating-point roundoff allowance, resolution = clock
 *     measurement resolution (clock_getres, 0 for injected clocks).
 */
#ifndef FTRACE_H
#define FTRACE_H

#ifndef FT_ENABLE
#define FT_ENABLE 0
#endif

#if FT_ENABLE

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L   /* clock_gettime / clock_getres under -std=c11 */
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define FT_MAX_SPANS_PER_FRAME 16
#define FT_MAX_METRICS 16
#define FT_MAX_OVERLAP_LOG 256
#define FT_ROUNDOFF 4e-16

typedef struct {
    double start;
    double end;
    const char* name;
    int active;
} FtSpan;

typedef struct {
    const char* name;
    long calls;
    double total;
    double min;
    double max;
} FtMetric;

typedef struct {
    int frame;
    const char* a;
    const char* b;
    double start;      /* intersection start, relative to frame begin */
    double end;        /* intersection end, relative to frame begin   */
    double duration;
} FtOverlap;

typedef struct {
    long frames;
    double frame_total_sum;
    double span_sum_sum;
    double union_sum;
    double overlap_sum;
    double gap_sum;
    double diff_sum;        /* sum of per-frame (span_sum - frame_total) */
    double abs_diff_sum;
    double eps_sum;
    double bound_sum;       /* sum of per-frame upper bounds */
    double resolution;
    long metric_count;
    FtMetric metrics[FT_MAX_METRICS];
    long overlap_count;
    long dropped_overlaps;
    FtOverlap overlaps[FT_MAX_OVERLAP_LOG];
    int invariant_violations;
} FtReport;

typedef struct {
    double (*clock_fn)(void*);
    void* clock_arg;
    double resolution;
    int frame_open;
    long frame_index;
    double frame_begin;
    double frame_end;
    int nspans;
    FtSpan spans[FT_MAX_SPANS_PER_FRAME];
    FtReport report;
} ft_t;

/* Virtual clock for deterministic tests: returns the double pointed to. */
static inline double ft_virtual_clock(void* arg) { return *(const double*)arg; }

static inline double ft_default_clock(void* arg) {
    (void)arg;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static inline void ft_init(ft_t* t) {
    memset(t, 0, sizeof(*t));
    t->clock_fn = ft_default_clock;
    struct timespec res;
    if (clock_getres(CLOCK_MONOTONIC, &res) == 0)
        t->resolution = (double)res.tv_sec + (double)res.tv_nsec * 1e-9;
    for (int i = 0; i < FT_MAX_METRICS; i++) t->report.metrics[i].min = 1e300;
}

static inline void ft_set_clock(ft_t* t, double (*fn)(void*), void* arg) {
    t->clock_fn = fn;
    t->clock_arg = arg;
    t->resolution = 0.0;                 /* injected clocks are exact */
    t->report.resolution = 0.0;
}

static inline double ft_now(ft_t* t) { return t->clock_fn(t->clock_arg); }

static inline void ft_frame_begin(ft_t* t) {
    t->frame_open = 1;
    t->nspans = 0;
    t->frame_begin = ft_now(t);
}

/* Returns a span handle, or -1 when no frame is open / no slot left. */
static inline int ft_span_start(ft_t* t, const char* name) {
    if (!t->frame_open || t->nspans >= FT_MAX_SPANS_PER_FRAME) return -1;
    int h = t->nspans++;
    t->spans[h].start = ft_now(t);
    t->spans[h].end = 0.0;
    t->spans[h].name = name;
    t->spans[h].active = 1;
    return h;
}

static inline void ft_span_end(ft_t* t, int handle) {
    if (handle < 0 || handle >= t->nspans || !t->spans[handle].active) return;
    t->spans[handle].end = ft_now(t);
    t->spans[handle].active = 0;
}

static inline FtMetric* ft_metric_slot(FtReport* r, const char* name) {
    for (int i = 0; i < FT_MAX_METRICS; i++) {
        if (r->metrics[i].name && strcmp(r->metrics[i].name, name) == 0)
            return &r->metrics[i];
    }
    for (int i = 0; i < FT_MAX_METRICS; i++) {
        if (!r->metrics[i].name) { r->metrics[i].name = name; r->metric_count = i + 1; return &r->metrics[i]; }
    }
    return NULL;
}

static inline int ft_interval_cmp(const void* pa, const void* pb) {
    const FtSpan* a = (const FtSpan*)pa;
    const FtSpan* b = (const FtSpan*)pb;
    if (a->start < b->start) return -1;
    if (a->start > b->start) return 1;
    if (a->end < b->end) return -1;
    if (a->end > b->end) return 1;
    return 0;
}

static inline void ft_frame_end(ft_t* t) {
    if (!t->frame_open) return;
    t->frame_open = 0;
    t->frame_end = ft_now(t);

    /* auto-close spans left open, so nothing leaks across frames */
    for (int i = 0; i < t->nspans; i++)
        if (t->spans[i].active) { t->spans[i].end = t->frame_end; t->spans[i].active = 0; }

    const double fb = t->frame_begin;
    const double T = t->frame_end - fb;
    double span_sum = 0.0;
    for (int i = 0; i < t->nspans; i++) {
        double d = t->spans[i].end - t->spans[i].start;
        span_sum += d;
        FtMetric* m = ft_metric_slot(&t->report, t->spans[i].name);
        if (m) {
            m->calls++;
            m->total += d;
            if (d < m->min) m->min = d;
            if (d > m->max) m->max = d;
        }
    }

    FtSpan sorted[FT_MAX_SPANS_PER_FRAME];
    memcpy(sorted, t->spans, sizeof(FtSpan) * (size_t)t->nspans);
    qsort(sorted, (size_t)t->nspans, sizeof(FtSpan), ft_interval_cmp);

    double union_len = 0.0;
    double cur_s = 0.0, cur_e = 0.0;
    int have = 0;
    for (int i = 0; i < t->nspans; i++) {
        double s = sorted[i].start - fb, e = sorted[i].end - fb;
        if (s < 0.0) s = 0.0;
        if (e > T) e = T;
        if (!have) { cur_s = s; cur_e = e; have = 1; }
        else if (s <= cur_e) { if (e > cur_e) cur_e = e; }
        else { union_len += cur_e - cur_s; cur_s = s; cur_e = e; }
    }
    if (have) union_len += cur_e - cur_s;

    /* pairwise overlap log: every pair whose half-open intervals intersect */
    double eps = (double)(t->nspans + 4) * fmax(1.0, T) * FT_ROUNDOFF;
    for (int i = 0; i < t->nspans; i++) {
        for (int j = i + 1; j < t->nspans; j++) {
            double lo = fmax(t->spans[i].start, t->spans[j].start) - fb;
            double hi = fmin(t->spans[i].end, t->spans[j].end) - fb;
            if (lo < hi && hi - lo > eps) {
                if (t->report.overlap_count < FT_MAX_OVERLAP_LOG) {
                    FtOverlap* o = &t->report.overlaps[t->report.overlap_count++];
                    o->frame = (int)t->frame_index;
                    o->a = t->spans[i].name;
                    o->b = t->spans[j].name;
                    o->start = lo;
                    o->end = hi;
                    o->duration = hi - lo;
                } else {
                    t->report.dropped_overlaps++;
                }
            }
        }
    }

    double overlap = span_sum - union_len;   /* multiplicity included */
    double gap = T - union_len;
    double diff = span_sum - T;
    double bound = overlap + gap + eps + t->resolution;

    /* invariant checks for this frame */
    if (fabs(diff - (overlap - gap)) > eps) t->report.invariant_violations++;
    if (overlap < -eps || gap < -eps) t->report.invariant_violations++;
    if (union_len > T + eps) t->report.invariant_violations++;
    if (fabs(diff) > bound + eps) t->report.invariant_violations++;

    t->report.frames++;
    t->report.frame_total_sum += T;
    t->report.span_sum_sum += span_sum;
    t->report.union_sum += union_len;
    t->report.overlap_sum += overlap;
    t->report.gap_sum += gap;
    t->report.diff_sum += diff;
    t->report.abs_diff_sum += fabs(diff);
    t->report.eps_sum += eps;
    t->report.bound_sum += bound;
    t->report.resolution = t->resolution;
    t->frame_index++;
}

/* Checks aggregate invariants; returns 0 when all hold, failure count otherwise. */
static inline int ft_verify(const ft_t* t) {
    const FtReport* r = &t->report;
    int bad = r->invariant_violations;
    if (r->frames == 0) return bad + 1;
    if (fabs(r->diff_sum - (r->overlap_sum - r->gap_sum)) > r->eps_sum) bad++;
    if (fabs((r->span_sum_sum - r->frame_total_sum) - r->diff_sum) > r->eps_sum) bad++;
    if (r->abs_diff_sum > r->bound_sum + r->eps_sum) bad++;
    if (r->overlap_sum < -r->eps_sum || r->gap_sum < -r->eps_sum) bad++;
    return bad;
}

static inline const FtReport* ft_report(const ft_t* t) { return &t->report; }

static inline void ft_print_report(const ft_t* t, const char* title, FILE* out) {
    const FtReport* r = &t->report;
    fprintf(out, "== ftrace report: %s ==\n", title);
    fprintf(out, "[metrics]\n");
    for (int i = 0; i < r->metric_count; i++) {
        const FtMetric* m = &r->metrics[i];
        double avg = m->calls ? m->total / (double)m->calls : 0.0;
        fprintf(out,
                "metric name=%s calls=%ld total_us=%.3f avg_us=%.3f min_us=%.3f max_us=%.3f\n",
                m->name, m->calls, m->total * 1e6, avg * 1e6, m->min * 1e6, m->max * 1e6);
    }
    fprintf(out, "[overlaps]\n");
    for (long i = 0; i < r->overlap_count; i++) {
        const FtOverlap* o = &r->overlaps[i];
        fprintf(out,
                "overlap frame=%d a=%s b=%s start_us=%.3f end_us=%.3f dur_us=%.3f\n",
                o->frame, o->a, o->b, o->start * 1e6, o->end * 1e6, o->duration * 1e6);
    }
    if (r->dropped_overlaps)
        fprintf(out, "overlap_dropped=%ld (log cap %d)\n", r->dropped_overlaps, FT_MAX_OVERLAP_LOG);
    fprintf(out, "[reconcile]\n");
    fprintf(out, "frames=%ld\n", r->frames);
    fprintf(out, "frame_total_us=%.3f\n", r->frame_total_sum * 1e6);
    fprintf(out, "span_sum_us=%.3f\n", r->span_sum_sum * 1e6);
    fprintf(out, "union_us=%.3f\n", r->union_sum * 1e6);
    fprintf(out, "overlap_charge_us=%.3f\n", r->overlap_sum * 1e6);
    fprintf(out, "gap_us=%.3f\n", r->gap_sum * 1e6);
    fprintf(out, "diff_span_sum_minus_frame_total_us=%.3f\n", r->diff_sum * 1e6);
    fprintf(out, "abs_diff_us=%.3f\n", r->abs_diff_sum * 1e6);
    fprintf(out, "roundoff_eps_us=%.6f\n", r->eps_sum * 1e6);
    fprintf(out, "clock_resolution_us=%.6f\n", r->resolution * 1e6);
    fprintf(out, "bound_overlap_gap_eps_res_us=%.3f\n", r->bound_sum * 1e6);
    fprintf(out, "verify=%d\n", ft_verify(t) == 0 ? 1 : 0);
}

#else  /* !FT_ENABLE: no-op surface, zero codegen at call sites */

typedef struct { char _ft_unused; } ft_t;

static inline void ft_init(ft_t* t) { (void)t; }
static inline void ft_set_clock(ft_t* t, void* fn, void* arg) { (void)t; (void)fn; (void)arg; }
static inline double ft_now(ft_t* t) { (void)t; return 0.0; }
static inline void ft_frame_begin(ft_t* t) { (void)t; }
static inline int ft_span_start(ft_t* t, const char* name) { (void)t; (void)name; return -1; }
static inline void ft_span_end(ft_t* t, int handle) { (void)t; (void)handle; }
static inline void ft_frame_end(ft_t* t) { (void)t; }

#endif /* FT_ENABLE */
#endif /* FTRACE_H */
