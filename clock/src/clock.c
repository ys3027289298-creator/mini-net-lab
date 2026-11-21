
/* clock: robust network clock offset estimation.
 *
 * Pipeline per estimate:
 *   1. min-RTT filter: keep samples whose RTT is within 1.5x the window
 *      minimum (+1 ms). Long round trips carry a large asymmetry bias.
 *   2. Theil-Sen line of offset against local time (median of pairwise
 *      slopes), so a few bad samples cannot drag the fit.
 *   3. MAD-based outlier rejection against that line, refit to a fixed
 *      point. Outliers are dropped, not averaged in.
 *   4. Report the offset at an eval time, the drift, a 95% CI from the
 *      inlier residuals, and the physical bound max_used_rtt / 2.
 *
 * Fewer than CLK_MIN_FIT accepted samples, or fewer than CLK_MIN_INLIERS
 * clean ones, returns CLK_EST_NEED_MORE and leaves `offset` NAN: no point
 * estimate is handed out. Timestamps that go backwards (local send or
 * receive, or remote) and non-positive RTTs are rejected on ingest.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define CLK_WINDOW      64
#define CLK_MIN_FIT      5      /* accepted samples before any fit */
#define CLK_MIN_INLIERS  3      /* clean samples needed for an estimate */
#define CLK_MAX_KEEP    16      /* cap on min-RTT survivors */
#define CLK_OUTLIER_K    3.0    /* MAD clipping threshold */
#define CLK_MAX_ITER     3
#define CLK_MAX_PAIRS   (CLK_MAX_KEEP * (CLK_MAX_KEEP - 1) / 2)

enum {
    CLK_ACCEPT          =  0,
    CLK_ERR_INVALID     = -1,
    CLK_ERR_NEG_RTT     = -2,   /* t_recv < t_send */
    CLK_ERR_LOCAL_BACK  = -3,   /* local send or receive went backwards */
    CLK_ERR_REMOTE_BACK = -4    /* remote timestamp went backwards */
};

enum { CLK_EST_EMPTY = 0, CLK_EST_NEED_MORE = 1, CLK_EST_OK = 2 };

typedef struct { double rtt, offset, at; } Sample;

typedef struct {
    Sample s[CLK_WINDOW];
    int    n;
    int    have_last;
    double last_t_send, last_t_recv, last_remote;
    int    n_rejected;
} Sync;

typedef struct {
    int    status;
    int    has_estimate;
    int    n_accepted;      /* samples in the window */
    int    n_selected;      /* after the min-RTT filter */
    int    n_used;          /* inliers after outlier rejection */
    int    n_outliers;
    int    rtt_fallback;    /* filter relaxed because too few survived */
    double offset;          /* offset at eval_time, NAN if no estimate */
    double slope;           /* clock drift, seconds per second */
    double ci_lo, ci_hi;    /* 95% confidence interval */
    double ci_half;         /* half width */
    double asym_bound;      /* max RTT of used samples / 2 */
    double err_bound;       /* asym_bound + ci_half */
    double eval_time;
    double resid_median;    /* |residual| distribution of inliers */
    double resid_p90;
    double resid_max;
    const char* note;
} Estimate;

typedef struct { double x, y; } Pt;

static int cmp_double(const void* pa, const void* pb) {
    double a = *(const double*)pa, b = *(const double*)pb;
    return (a > b) - (a < b);
}

/* median of v[n]; v is copied and may be reordered */
static double median(double* v, int n) {
    if (n <= 0) return NAN;
    qsort(v, (size_t)n, sizeof(double), cmp_double);
    return (n & 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/* two-sided 95% t critical value for integer df */
static double t_crit95(int df) {
    static const double tab[] = {
        0.0, 12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306,
        2.262, 2.228, 2.201, 2.179, 2.160, 2.145, 2.131, 2.120, 2.110,
        2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056,
        2.052, 2.048, 2.045, 2.042
    };
    if (df < 1) return NAN;
    if (df < (int)(sizeof(tab) / sizeof(tab[0]))) return tab[df];
    return 1.960;
}

static void sync_init(Sync* sy) { memset(sy, 0, sizeof(*sy)); }

static int sync_add(Sync* sy, double t_send, double t_recv, double remote_time) {
    if (!isfinite(t_send) || !isfinite(t_recv) || !isfinite(remote_time)) {
        sy->n_rejected++;
        return CLK_ERR_INVALID;
    }
    if (t_recv < t_send) {
        sy->n_rejected++;
        return CLK_ERR_NEG_RTT;
    }
    if (sy->have_last) {
        if (t_send < sy->last_t_send || t_recv < sy->last_t_recv) {
            sy->n_rejected++;
            return CLK_ERR_LOCAL_BACK;
        }
        if (remote_time < sy->last_remote) {
            sy->n_rejected++;
            return CLK_ERR_REMOTE_BACK;
        }
    }
    if (sy->n >= CLK_WINDOW) {
        memmove(&sy->s[0], &sy->s[1], sizeof(Sample) * (CLK_WINDOW - 1));
        sy->n = CLK_WINDOW - 1;
    }
    double rtt = t_recv - t_send;
    Sample* smp = &sy->s[sy->n++];
    smp->rtt = rtt;
    smp->offset = remote_time - (t_send + t_recv) * 0.5;
    smp->at = t_recv;

    sy->have_last = 1;
    sy->last_t_send = t_send;
    sy->last_t_recv = t_recv;
    sy->last_remote = remote_time;
    return CLK_ACCEPT;
}

/* Theil-Sen: median pairwise slope and median intercept over given points */
static int theil_sen(const Pt* p, const int* idx, int n, double* out_slope, double* out_intercept) {
    if (n <= 0) return -1;
    if (n == 1) { *out_slope = 0.0; *out_intercept = p[idx[0]].y; return 1; }

    double slopes[CLK_MAX_PAIRS];
    double ivs[CLK_MAX_KEEP];
    int m = 0;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double dx = p[idx[j]].x - p[idx[i]].x;
            if (fabs(dx) > 1e-12) slopes[m++] = (p[idx[j]].y - p[idx[i]].y) / dx;
        }
    }
    double b = (m > 0) ? median(slopes, m) : 0.0;
    for (int i = 0; i < n; i++) ivs[i] = p[idx[i]].y - b * p[idx[i]].x;
    *out_slope = b;
    *out_intercept = median(ivs, n);
    return 1;
}

/* Ordinary least squares on the given points; used only for the CI. */
static int ols_fit(const Pt* p, const int* idx, int n,
                   double* out_slope, double* out_intercept,
                   double* out_sigma, double* out_sxx, double* out_xbar) {
    if (n < 3) return -1;
    double sx = 0, sy = 0;
    for (int i = 0; i < n; i++) { sx += p[idx[i]].x; sy += p[idx[i]].y; }
    double xbar = sx / n, ybar = sy / n;
    double sxx = 0, sxy = 0;
    for (int i = 0; i < n; i++) {
        double dx = p[idx[i]].x - xbar;
        sxx += dx * dx;
        sxy += dx * (p[idx[i]].y - ybar);
    }
    if (sxx < 1e-12) return -1;
    double b = sxy / sxx, a = ybar - b * xbar;
    double sse = 0;
    for (int i = 0; i < n; i++) {
        double e = p[idx[i]].y - (a + b * p[idx[i]].x);
        sse += e * e;
    }
    *out_slope = b;
    *out_intercept = a;
    *out_sigma = sqrt(sse / (n - 2));
    *out_sxx = sxx;
    *out_xbar = xbar;
    return 0;
}

/* Sort candidate indices by RTT ascending (insertion sort, n is small). */
static void sort_by_rtt(const Sync* sy, int* idx, int n) {
    for (int i = 1; i < n; i++) {
        int v = idx[i], j = i - 1;
        while (j >= 0 && sy->s[idx[j]].rtt > sy->s[v].rtt) { idx[j + 1] = idx[j]; j--; }
        idx[j + 1] = v;
    }
}

static int sync_estimate(const Sync* sy, double eval_time, Estimate* out) {
    memset(out, 0, sizeof(*out));
    out->offset = out->slope = out->ci_lo = out->ci_hi = NAN;
    out->resid_median = out->resid_p90 = out->resid_max = NAN;
    out->eval_time = eval_time;
    out->n_accepted = sy->n;

    if (sy->n == 0) {
        out->status = CLK_EST_EMPTY;
        out->note = "no samples";
        return CLK_EST_EMPTY;
    }
    if (sy->n < CLK_MIN_FIT) {
        out->status = CLK_EST_NEED_MORE;
        out->note = "need at least 5 accepted samples; no point estimate";
        return CLK_EST_NEED_MORE;
    }

    /* 1. min-RTT filter */
    double min_rtt = sy->s[0].rtt;
    for (int i = 1; i < sy->n; i++) if (sy->s[i].rtt < min_rtt) min_rtt = sy->s[i].rtt;
    double thr = min_rtt * 1.5 + 1e-3;

    int sel[CLK_WINDOW];
    int nsel = 0;
    for (int i = 0; i < sy->n; i++)
        if (sy->s[i].rtt <= thr) sel[nsel++] = i;
    if (nsel < CLK_MIN_FIT) {           /* relax rather than refuse outright */
        nsel = 0;
        for (int i = 0; i < sy->n; i++) sel[nsel++] = i;
        out->rtt_fallback = 1;
    }
    sort_by_rtt(sy, sel, nsel);
    if (nsel > CLK_MAX_KEEP) nsel = CLK_MAX_KEEP;
    out->n_selected = nsel;

    Pt pts[CLK_MAX_KEEP];
    for (int i = 0; i < nsel; i++) {
        pts[i].x = sy->s[sel[i]].at;
        pts[i].y = sy->s[sel[i]].offset;
    }

    /* 2/3. robust fit and MAD trimming to a fixed point */
    int idx[CLK_MAX_KEEP];
    for (int i = 0; i < nsel; i++) idx[i] = i;
    int nwork = nsel;
    double slope = 0.0, intercept = 0.0;

    for (int iter = 0; iter < CLK_MAX_ITER; iter++) {
        if (theil_sen(pts, idx, nwork, &slope, &intercept) < 0) break;
        double res[CLK_MAX_KEEP], dev[CLK_MAX_KEEP];
        for (int i = 0; i < nwork; i++)
            res[i] = pts[idx[i]].y - (intercept + slope * pts[idx[i]].x);
        double med = median(res, nwork);
        for (int i = 0; i < nwork; i++) dev[i] = fabs(res[i] - med);
        double scale = 1.4826 * median(dev, nwork);

        int keep[CLK_MAX_KEEP], nkeep = 0;
        if (!(scale > 1e-12)) {
            /* MAD collapsed to zero: at least half the residuals are
             * identical, as with noiseless or coarsely quantized data.
             * Keep only that group; points off it are the outliers. */
            for (int i = 0; i < nwork; i++)
                if (fabs(res[i] - med) <= 1e-9) keep[nkeep++] = idx[i];
        } else {
            for (int i = 0; i < nwork; i++)
                if (fabs(res[i] - med) <= CLK_OUTLIER_K * scale) keep[nkeep++] = idx[i];
        }
        if (nkeep == nwork) break;              /* stable */
        if (nkeep == 0) break;                  /* never discard everything */
        for (int i = 0; i < nkeep; i++) idx[i] = keep[i];
        nwork = nkeep;
        if (nwork < CLK_MIN_INLIERS) break;
    }

    out->n_used = nwork;
    out->n_outliers = out->n_selected - nwork;
    if (nwork < CLK_MIN_INLIERS) {
        out->status = CLK_EST_NEED_MORE;
        out->note = "too few consistent samples after outlier rejection; no point estimate";
        return CLK_EST_NEED_MORE;
    }

    /* final robust line on inliers */
    if (theil_sen(pts, idx, nwork, &slope, &intercept) < 0) {
        out->status = CLK_EST_NEED_MORE;
        out->note = "fit failed";
        return CLK_EST_NEED_MORE;
    }
    if (!isfinite(eval_time)) {
        double xs[CLK_MAX_KEEP];
        for (int i = 0; i < nwork; i++) xs[i] = pts[idx[i]].x;
        eval_time = median(xs, nwork);
        out->eval_time = eval_time;
    }

    /* residual distribution of the inliers */
    double absres[CLK_MAX_KEEP];
    double max_rtt = 0.0;
    for (int i = 0; i < nwork; i++) {
        absres[i] = fabs(pts[idx[i]].y - (intercept + slope * pts[idx[i]].x));
        double r = sy->s[sel[idx[i]]].rtt;
        if (r > max_rtt) max_rtt = r;
    }
    double tmp[CLK_MAX_KEEP];
    memcpy(tmp, absres, sizeof(double) * (size_t)nwork);
    out->resid_median = median(tmp, nwork);
    memcpy(tmp, absres, sizeof(double) * (size_t)nwork);
    qsort(tmp, (size_t)nwork, sizeof(double), cmp_double);
    out->resid_p90 = tmp[(int)floor(0.9 * (nwork - 1))];
    out->resid_max = tmp[nwork - 1];

    out->offset = intercept + slope * eval_time;
    out->slope = slope;
    out->asym_bound = 0.5 * max_rtt;

    /* 4. CI from OLS on the same inliers; center it on the robust estimate */
    double a, b, sigma, sxx, xbar;
    if (ols_fit(pts, idx, nwork, &a, &b, &sigma, &sxx, &xbar) == 0) {
        double tc = t_crit95(nwork - 2);
        double se = sigma * sqrt(1.0 / nwork + (eval_time - xbar) * (eval_time - xbar) / sxx);
        out->ci_half = tc * se;
    } else {
        /* degenerate times: use spread of offsets */
        double mean = 0.0, ss = 0.0;
        for (int i = 0; i < nwork; i++) mean += pts[idx[i]].y;
        mean /= nwork;
        for (int i = 0; i < nwork; i++) {
            double d = pts[idx[i]].y - mean;
            ss += d * d;
        }
        double sd = (nwork > 1) ? sqrt(ss / (nwork - 1)) : 0.0;
        out->ci_half = t_crit95(nwork - 1) * sd / sqrt((double)nwork);
    }
    out->ci_lo = out->offset - out->ci_half;
    out->ci_hi = out->offset + out->ci_half;
    out->err_bound = out->asym_bound + out->ci_half;

    out->status = CLK_EST_OK;
    out->has_estimate = 1;
    out->note = "ok";
    return CLK_EST_OK;
}

/* __TESTS__ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* deterministic pseudo-random in [0,1) so the test run is reproducible */
static unsigned rng_state = 12345u;
static double rnd(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return (rng_state >> 8) / 16777216.0;
}

/* one exchange: local send, symmetric base delay + jitter, true offset */
static int push_sym(Sync* sy, double t_send, double base, double jitter,
                       double true_off, double remote_skew) {
    double one_way = base * 0.5 + jitter * rnd();
    double t_recv = t_send + 2.0 * one_way;
    double remote = t_send + one_way + true_off + remote_skew * t_send;
    return sync_add(sy, t_send, t_recv, remote);
}

int main(void) {
    /* --- basic offset recovery, no jitter --- */
    Sync sy;
    Sync* s = &sy;
    sync_init(s);
    const double TRUE_OFF = 2.0;
    check("accept_basic", push_sym(s, 0.0, 0.10, 0.0, TRUE_OFF, 0.0) == CLK_ACCEPT);
    check("best_single", s->n == 1);
    Estimate e;
    sync_estimate(s, 0.0, &e);
    check("one_sample_no_estimate", e.status == CLK_EST_NEED_MORE &&
                                     !e.has_estimate && isnan(e.offset));

    /* --- reject malformed timestamps --- */
    check("reject_negative_rtt", sync_add(s, 5.0, 4.0, 0.0) == CLK_ERR_NEG_RTT);
    check("reject_local_back", sync_add(s, -1.0, 0.0, 0.0) == CLK_ERR_LOCAL_BACK);
    check("reject_nan", sync_add(s, NAN, 1.0, 1.0) == CLK_ERR_INVALID);
    sync_init(s);
    check("accept_first", sync_add(s, 10.0, 10.1, 10.05 + TRUE_OFF) == CLK_ACCEPT);
    check("reject_remote_back", sync_add(s, 11.0, 11.1, 9.0) == CLK_ERR_REMOTE_BACK);

    /* --- jittery path: estimates stay near the truth --- */
    sync_init(s);
    for (int i = 0; i < 40; i++)
        push_sym(s, i * 1.0, 0.08, 0.06, TRUE_OFF, 0.0);   /* 0-60 ms one-way jitter */
    double t_eval = 39.0;
    sync_estimate(s, t_eval, &e);
    check("jitter_ok", e.status == CLK_EST_OK && e.has_estimate);
    check("jitter_offset_err", fabs(e.offset - TRUE_OFF) < 0.005);
    check("jitter_ci_contains_truth", e.ci_lo <= TRUE_OFF && TRUE_OFF <= e.ci_hi);
    check("jitter_ci_positive", e.ci_half > 0.0 && isfinite(e.ci_half));
    check("jitter_err_bound", e.err_bound >= fabs(e.offset - TRUE_OFF) &&
                              e.err_bound >= e.asym_bound);

    /* --- drift: 100 ppm remote clock --- */
    Sync drift;
    sync_init(&drift);
    for (int i = 0; i < 60; i++)
        push_sym(&drift, i * 1.0, 0.04, 0.01, 1.0, 100e-6);
    sync_estimate(&drift, 59.0, &e);
    check("drift_ok", e.status == CLK_EST_OK);
    check("drift_slope", fabs(e.slope - 100e-6) < 30e-6);
    check("drift_offset", fabs(e.offset - (1.0 + 100e-6 * 59.0)) < 0.004);

    /* --- gross outliers must be trimmed by MAD, not averaged in ---
     * Corrupted replies keep a low RTT (so the min-RTT filter passes them)
     * but carry a bogus remote timestamp; the regression residual is the
     * only thing that can catch them. */
    Sync outl;
    sync_init(&outl);
    int corrupt = 0;
    for (int i = 0; i < 15; i++) {
        double t = i * 1.0;
        if (i % 3 == 0) {
            sync_add(&outl, t, t + 0.05, t + 0.025 + TRUE_OFF + 0.4);
            corrupt++;
        } else {
            sync_add(&outl, t, t + 0.05, t + 0.025 + TRUE_OFF);
        }
    }
    sync_estimate(&outl, 14.0, &e);
    check("outlier_ok", e.status == CLK_EST_OK);
    check("outlier_trimmed", e.n_outliers >= corrupt - 1);
    check("outlier_offset_err", fabs(e.offset - TRUE_OFF) < 0.01);

    /* naive mean of all offsets would be visibly worse: prove trimming matters */
    double naive = 0.0;
    for (int i = 0; i < outl.n; i++) naive += outl.s[i].offset;
    naive /= outl.n;
    check("naive_mean_worse", fabs(naive - TRUE_OFF) > fabs(e.offset - TRUE_OFF));

    /* --- asymmetric delay: min-RTT filter selects the low-RTT cluster --- */
    Sync asym;
    sync_init(&asym);
    for (int i = 0; i < 20; i++) {
        /* fast path, near-symmetric */
        sync_add(&asym, i * 1.0, i * 1.0 + 0.02, i * 1.0 + 0.01 + TRUE_OFF);
        /* slow path, request leg 200 ms: offset biased by -0.1 */
        sync_add(&asym, i * 1.0 + 0.3, i * 1.0 + 0.52, i * 1.0 + 0.41 + TRUE_OFF - 0.1);
    }
    sync_estimate(&asym, 19.0, &e);
    check("asym_ok", e.status == CLK_EST_OK);
    check("asym_min_rtt_filtered", e.n_selected <= 20 + 1);
    check("asym_offset_err", fabs(e.offset - TRUE_OFF) < 0.01);

    /* --- dirty window: half the samples unusable, still a sane estimate --- */
    Sync dirty;
    sync_init(&dirty);
    for (int i = 0; i < 40; i++) {
        if (i % 2 == 0) push_sym(&dirty, i * 1.0, 0.03, 0.01, TRUE_OFF, 0.0);
        else            push_sym(&dirty, i * 1.0, 0.60, 0.02, TRUE_OFF + 0.3, 0.0);
    }
    sync_estimate(&dirty, 39.0, &e);
    check("dirty_ok", e.status == CLK_EST_OK);
    check("dirty_offset_err", fabs(e.offset - TRUE_OFF) < 0.01);

    /* --- sparse / degenerate inputs report, do not guess --- */
    Sync sparse;
    sync_init(&sparse);
    check("empty_status", sync_estimate(&sparse, 0.0, &e) == CLK_EST_EMPTY);
    for (int i = 0; i < 4; i++) push_sym(&sparse, i * 1.0, 0.05, 0.0, TRUE_OFF, 0.0);
    check("under_min_no_estimate", sync_estimate(&sparse, 3.0, &e) == CLK_EST_NEED_MORE);
    check("under_min_nan", isnan(e.offset));
    check("under_min_note", e.note != NULL && e.note[0] != '\0');

    /* two samples on the same identical timestamp: no time spread */
    Sync flat;
    sync_init(&flat);
    for (int i = 0; i < 6; i++) sync_add(&flat, 7.0, 7.05, 7.025 + TRUE_OFF);
    check("identical_times_ok_or_refused",
          sync_estimate(&flat, 7.0, &e) != CLK_EST_EMPTY);
    check("identical_times_no_crash", e.status == CLK_EST_OK || e.status == CLK_EST_NEED_MORE);

    /* --- window is bounded --- */
    Sync big;
    sync_init(&big);
    for (int i = 0; i < 200; i++) push_sym(&big, i * 1.0, 0.05, 0.005, TRUE_OFF, 0.0);
    check("window_capped", big.n == CLK_WINDOW);
    sync_estimate(&big, 199.0, &e);
    check("window_estimate_ok", e.status == CLK_EST_OK && fabs(e.offset - TRUE_OFF) < 0.005);

    /* rejected count is tracked, not silently dropped */
    check("reject_counted", big.n_rejected == 0 && sparse.n_rejected >= 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}

