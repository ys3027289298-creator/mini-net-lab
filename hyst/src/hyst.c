
/* hyst: hysteresis band selection from movement speed and AOI radius.
 *
 * Problem: an entity hovering on an AOI boundary makes the visible set flip
 * on every crossing, so enter/leave events (and the associated updates) churn.
 * A hysteresis band h fixes this: enter when the true distance is h/2 inside
 * the radius, leave only when it is h/2 outside.  A wider band costs less
 * churn and more visible latency; the recommended band is the knee of that
 * trade-off curve.
 *
 * Movement model (documented assumption + evidence in README):
 *   While hovering, the boundary-relative displacement is treated as
 *   diffusive, with diffusion coefficient D = kappa * v^2 * dt / 2, where
 *   v is the RMS movement speed and dt the visibility sample period.  That
 *   scaling reproduces a 1-D Brownian hover Monte Carlo within ~1.3x over
 *   h in [0.1R, R] for the tested speeds; the knee location is insensitive
 *   to the absolute calibration because it is set by curve *shape*.
 *
 * Curve used for the knee:
 *   churn(h)   = kappa * D / (R * h)          visibility flips per second
 *   latency(h) = h / (2 v) + dt / 2           extra visibility delay
 *   u(h) = (latency - latency(hmin)) / (latency(hmax) - latency(hmin))
 *   w(h) = (churn   - churn(hmax))   / (churn(hmin)   - churn(hmax))
 *   g(h) = 1 - u(h) - w(h)                    distance below the chord
 *                                                  between both ends
 * Admissible bands are h in [hmin, hmax] with hmin = v*dt (the sampler
 * cannot resolve less) and hmax = 2R (the band must fit inside the AOI).
 *
 * Because latency is affine in h, u is affine in h, and churn is
 * proportional to 1/h, so g is concave on the whole domain and its maximum
 * is the geometric mean h* = sqrt(hmin*hmax) = sqrt(2 R v dt).
 * Concavity also gives an a-posteriori bound on any grid (brute-force)
 * maximiser, see hyst_bruteforce().
 *
 * Explicit rejections: zero band (churn is unbounded -> no finite trade-off),
 * non-positive / NaN inputs, missing speed estimate, and hmin >= hmax.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define SPEED_WIN 8   /* velocity samples kept */
#define SPEED_MIN 4   /* samples needed before a speed is reported */

typedef enum {
    HYST_OK = 0,
    HYST_ERR_MISSING = -1,    /* no usable speed estimate */
    HYST_ERR_ZERO_BAND = -2,  /* band of zero width has no finite churn */
    HYST_ERR_BAD_ARG = -3,
    HYST_ERR_RANGE = -4       /* hmin >= hmax: band range is degenerate */
} HystStatus;

/* ---- movement speed estimate ------------------------------------------- */

typedef struct {
    int have;              /* anchor set */
    long long t_ms;        /* anchor timestamp */
    double x, y;           /* anchor position */
    double v2[SPEED_WIN];  /* recent squared speeds */
    int n, head;
} SpeedEst;

static void speed_est_reset(SpeedEst* s) { memset(s, 0, sizeof(*s)); }

/* Add a position sample.  Returns 1 if a velocity sample was produced,
 * 0 if only the anchor moved (first sample, or non-increasing timestamp),
 * -1 on bad argument.  Samples with dt <= 0 are dropped, not clamped. */
static int speed_est_add(SpeedEst* s, long long t_ms, double x, double y) {
    if (s == NULL) return -1;
    if (!s->have) { s->have = 1; s->t_ms = t_ms; s->x = x; s->y = y; return 0; }
    long long dms = t_ms - s->t_ms;
    if (dms <= 0) return 0;
    double dts = (double)dms / 1000.0;
    double dx = x - s->x, dy = y - s->y;
    double sp = sqrt(dx * dx + dy * dy) / dts;
    s->v2[s->head] = sp * sp;
    s->head = (s->head + 1) % SPEED_WIN;
    if (s->n < SPEED_WIN) s->n++;
    s->t_ms = t_ms; s->x = x; s->y = y;
    return 1;
}

/* RMS speed over the window.  Returns -1 (missing) until SPEED_MIN samples
 * exist.  Standard error treats the window as iid samples of v^2:
 *   var(v^2) ~ sample variance, se(v^2) = sqrt(var/n), se(v) = se(v^2)/(2v).
 * se may be NULL. */
static int speed_est_v(const SpeedEst* s, double* v, double* se) {
    if (s == NULL || v == NULL) return -1;
    if (s->n < SPEED_MIN) return -1;
    double m = 0.0;
    for (int i = 0; i < s->n; i++) m += s->v2[i];
    m /= (double)s->n;
    double var = 0.0;
    if (s->n > 1) {
        double ss = 0.0;
        for (int i = 0; i < s->n; i++) { double d = s->v2[i] - m; ss += d * d; }
        var = ss / (double)(s->n - 1);
    }
    double vv = sqrt(m);
    *v = vv;
    if (se) *se = (vv > 0.0) ? sqrt(var / (double)s->n) / (2.0 * vv) : 0.0;
    return 0;
}

static const char* speed_est_why_missing(const SpeedEst* s) {
    if (s == NULL) return "speed estimate unavailable: no estimator state";
    return "speed estimate unavailable: need >= 4 accepted samples with "
           "strictly increasing timestamps in the 8-sample window";
}

/* ---- trade-off model --------------------------------------------------- */

typedef struct {
    double v;        /* RMS movement speed, units/s */
    double v_se;     /* its standard error (0 when caller supplied v) */
    double radius;   /* AOI radius */
    double dt;       /* visibility sample period, s */
    double hmin;     /* v*dt */
    double hmax;     /* 2*radius */
    double kappa;    /* churn calibration, default 1.0 */
} HystModel;

typedef struct {
    double h, churn, latency, u, w, g;
} HystPoint;

typedef struct {
    double h_bf;       /* grid maximiser (raw) */
    double h_ref;      /* parabolic refinement of h_bf */
    double g_bf;       /* g at grid maximiser */
    double step;       /* grid spacing */
    double hard_bound; /* guaranteed: step (concavity + grid) */
    double curv_bound; /* a-posteriori curvature bound */
    double bound;      /* min(hard_bound, curv_bound) */
    int n;
} HystBF;

static int hyst_init(HystModel* m, double v, double radius, double dt) {
    if (m == NULL) return HYST_ERR_BAD_ARG;
    memset(m, 0, sizeof(*m));
    /* !(x > 0) also rejects NaN */
    if (!(v > 0.0) || !(radius > 0.0) || !(dt > 0.0)) return HYST_ERR_BAD_ARG;
    m->v = v; m->radius = radius; m->dt = dt; m->kappa = 1.0;
    m->hmin = v * dt;
    m->hmax = 2.0 * radius;
    if (!(m->hmin < m->hmax)) return HYST_ERR_RANGE;
    return HYST_OK;
}

static int hyst_init_from_speed(HystModel* m, const SpeedEst* s,
                                double radius, double dt) {
    double v = 0.0, se = 0.0;
    if (m == NULL) return HYST_ERR_BAD_ARG;
    if (speed_est_v(s, &v, &se) != 0) return HYST_ERR_MISSING;
    int r = hyst_init(m, v, radius, dt);
    if (r == HYST_OK) m->v_se = se;
    return r;
}

static double hyst_churn(const HystModel* m, double h) {
    double D = 0.5 * m->v * m->v * m->dt;
    return m->kappa * D / (m->radius * h);
}

static double hyst_latency(const HystModel* m, double h) {
    return h / (2.0 * m->v) + m->dt / 2.0;
}

/* Evaluate the model at band h.  A zero (or negative/NaN) band is refused
 * instead of returning churn=inf.  Normalised coordinates are only defined
 * inside [hmin, hmax] and are left as NaN outside. */
static int hyst_point(const HystModel* m, double h, HystPoint* p) {
    if (m == NULL || p == NULL) return HYST_ERR_BAD_ARG;
    memset(p, 0, sizeof(*p));
    p->h = h;
    p->u = p->w = p->g = NAN;
    if (h == 0.0) return HYST_ERR_ZERO_BAND;
    if (!(h > 0.0)) return HYST_ERR_BAD_ARG;
    p->churn = hyst_churn(m, h);
    p->latency = hyst_latency(m, h);

    double L0 = hyst_latency(m, m->hmin), L1 = hyst_latency(m, m->hmax);
    double C0 = hyst_churn(m, m->hmin), C1 = hyst_churn(m, m->hmax);
    if (h >= m->hmin && h <= m->hmax && L1 > L0 && C0 > C1) {
        p->u = (p->latency - L0) / (L1 - L0);
        p->w = (p->churn - C1) / (C0 - C1);
        p->g = 1.0 - p->u - p->w;
    }
    return HYST_OK;
}

/* Recommended band: g is concave and maximised at the geometric mean of the
 * admissible range.  Exact for this model; the self-test cross-checks it
 * against a dense grid scan. */
static int hyst_knee(const HystModel* m, HystPoint* k) {
    if (m == NULL || k == NULL) return HYST_ERR_BAD_ARG;
    double h = sqrt(m->hmin * m->hmax);
    return hyst_point(m, h, k);
}

/* Exhaustive band scan: the "brute force" reference.  Returns the maximiser
 * on an n-interval grid plus an upper bound on its distance to the true
 * knee.  For concave g with |g''| >= m on the domain,
 *     |h_bf - h*| <= sqrt(2 * (g* - g_bf) / m).
 * Here g'' = -2K/h^3 with K = hmin*hmax/(hmax-hmin), so the smallest
 * curvature is m = 2K/hmax^3 and the bound is a-posteriori and tight. */
static int hyst_bruteforce(const HystModel* m, int n, HystBF* out) {
    if (m == NULL || out == NULL || n < 2) return HYST_ERR_BAD_ARG;
    memset(out, 0, sizeof(*out));
    double step = (m->hmax - m->hmin) / (double)n;
    if (!(step > 0.0)) return HYST_ERR_RANGE;

    double best_g = -INFINITY, best_h = m->hmin;
    int bi = 0;
    for (int i = 0; i <= n; i++) {
        double h = m->hmin + step * (double)i;
        HystPoint p;
        if (hyst_point(m, h, &p) != HYST_OK) continue;
        if (p.g > best_g) { best_g = p.g; best_h = h; bi = i; }
    }
    out->h_bf = best_h;
    out->g_bf = best_g;
    out->step = step;
    out->n = n;

    /* parabolic refinement using the grid neighbours, clamped to the cell */
    double h_ref = best_h;
    if (bi > 0 && bi < n) {
        HystPoint pm, pp;
        hyst_point(m, best_h - step, &pm);
        hyst_point(m, best_h + step, &pp);
        double den = pm.g - 2.0 * best_g + pp.g;
        if (den < 0.0) {
            double d = 0.5 * (pm.g - pp.g) / den;
            if (d > 1.0) d = 1.0;
            if (d < -1.0) d = -1.0;
            h_ref = best_h + d * step;
        }
    }
    out->h_ref = h_ref;

    HystPoint k;
    double g_star = 0.0;
    if (hyst_knee(m, &k) == HYST_OK) g_star = k.g;
    double dg = g_star - best_g;
    if (!(dg > 0.0)) dg = 0.0;

    double K = m->hmin * m->hmax / (m->hmax - m->hmin);
    double mcurv = 2.0 * K / (m->hmax * m->hmax * m->hmax);
    double curv = (mcurv > 0.0) ? sqrt(2.0 * dg / mcurv) : step;
    if (!(curv >= 0.0) || !isfinite(curv)) curv = step;

    out->hard_bound = step;
    out->curv_bound = curv;
    out->bound = (curv < step ? curv : step) + 1e-12;
    return HYST_OK;
}

static const char* hyst_status_str(int st) {
    switch (st) {
    case HYST_OK: return "ok";
    case HYST_ERR_MISSING: return "missing-speed-estimate";
    case HYST_ERR_ZERO_BAND: return "zero-band-rejected";
    case HYST_ERR_BAD_ARG: return "bad-argument";
    case HYST_ERR_RANGE: return "degenerate-band-range";
    default: return "unknown";
    }
}

/* ---- self-test --------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int close_to(double a, double b, double tol) { return fabs(a - b) <= tol; }

typedef struct { double v, R, dt; } Case;

int main(void) {
    Case cases[] = {
        {   5.0, 20.0, 1.0 / 30.0 },
        {  50.0, 20.0, 1.0 / 30.0 },
        {   2.0, 30.0, 1.0 / 60.0 },
        {  20.0, 10.0, 1.0 / 20.0 },
        {   0.5, 20.0, 1.0 / 30.0 },
        {   4.0,  2.0, 1.0 / 30.0 },
    };
    const int NC = (int)(sizeof(cases) / sizeof(cases[0]));

    HystModel m;
    check("init_ok", hyst_init(&m, 5.0, 20.0, 1.0 / 30.0) == HYST_OK);
    check("hmin", close_to(m.hmin, 5.0 / 30.0, 1e-12));
    check("hmax", close_to(m.hmax, 40.0, 1e-12));
    check("kappa_default", close_to(m.kappa, 1.0, 0.0));

    /* closed-form knee vs dense brute force, and the deviation bound */
    for (int c = 0; c < NC; c++) {
        HystModel cm;
        int st = hyst_init(&cm, cases[c].v, cases[c].R, cases[c].dt);
        check("case_init", st == HYST_OK);
        HystPoint k;
        check("case_knee", hyst_knee(&cm, &k) == HYST_OK);
        double expect = sqrt(2.0 * cases[c].R * cases[c].v * cases[c].dt);
        check("knee_formula", close_to(k.h, expect, 1e-9));
        check("knee_inside", k.h > cm.hmin && k.h < cm.hmax);
        check("knee_g_top", close_to(k.g, 1.0 - k.u - k.w, 1e-12));

        /* brute force at two resolutions: deviation must stay under bound */
        HystBF bf_coarse, bf_fine;
        check("bf64", hyst_bruteforce(&cm, 64, &bf_coarse) == HYST_OK);
        check("bf4096", hyst_bruteforce(&cm, 4096, &bf_fine) == HYST_OK);
        double dev_c = fabs(bf_coarse.h_bf - k.h);
        double dev_f = fabs(bf_fine.h_bf - k.h);
        printf("case%d_v=%.3f_hstar=%.6f_bf64_dev=%.3e_bound=%.3e_"
               "bf4096_dev=%.3e_bound=%.3e\n",
               c, cases[c].v, k.h, dev_c, bf_coarse.bound,
               dev_f, bf_fine.bound);
        check("bf_bound_holds_coarse", dev_c <= bf_coarse.bound + 1e-12);
        check("bf_bound_holds_fine", dev_f <= bf_fine.bound + 1e-12);
        check("bound_within_step", bf_fine.bound <= bf_fine.step + 1e-12);
        check("refine_closer", fabs(bf_coarse.h_ref - k.h) <= dev_c + 1e-12);
        check("fine_beats_coarse", dev_f <= dev_c + 1e-12);
    }

    /* knee is invariant to the churn calibration: kappa cancels in the
     * normalised w coordinate, so only the curve shape matters */
    HystPoint k1, k5;
    hyst_knee(&m, &k1);
    m.kappa = 5.0;
    hyst_knee(&m, &k5);
    check("knee_kappa_invariant", close_to(k1.h, k5.h, 1e-12));
    m.kappa = 1.0;

    /* shape: churn falls with band, latency rises, g is concave/unimodal */
    HystPoint lo, mid, hi;
    hyst_point(&m, m.hmin, &lo);
    hyst_point(&m, sqrt(m.hmin * m.hmax), &mid);
    hyst_point(&m, m.hmax, &hi);
    check("churn_monotone_down", lo.churn > mid.churn && mid.churn > hi.churn);
    check("latency_monotone_up", lo.latency < mid.latency && mid.latency < hi.latency);
    check("knee_between_endpoints", mid.g > lo.g && mid.g > hi.g);
    check("endpoint_u_zero", close_to(lo.u, 0.0, 1e-12));
    check("endpoint_w_one", close_to(lo.w, 1.0, 1e-12));
    check("endpoint_u_one", close_to(hi.u, 1.0, 1e-12));
    check("endpoint_w_zero", close_to(hi.w, 0.0, 1e-12));

    /* concavity check away from the knee, both sides */
    int concave = 1;
    for (int i = 1; i < 64; i++) {
        double h = m.hmin + (m.hmax - m.hmin) * (double)i / 64.0;
        double e = (m.hmax - m.hmin) * 1e-4;
        HystPoint a, b, c2;
        hyst_point(&m, h - e, &a);
        hyst_point(&m, h, &b);
        hyst_point(&m, h + e, &c2);
        double d2 = (a.g - 2.0 * b.g + c2.g) / (e * e);
        if (!(d2 < 0.0)) concave = 0;
    }
    check("g_concave", concave);

    /* zero / invalid bands refused explicitly */
    HystPoint z;
    check("zero_band_rejected", hyst_point(&m, 0.0, &z) == HYST_ERR_ZERO_BAND);
    check("zero_band_no_churn", z.churn == 0.0);
    check("neg_band_rejected", hyst_point(&m, -1.0, &z) == HYST_ERR_BAD_ARG);
    check("nan_band_rejected", hyst_point(&m, NAN, &z) == HYST_ERR_BAD_ARG);
    check("zero_status_str", strcmp(hyst_status_str(HYST_ERR_ZERO_BAND),
                                    "zero-band-rejected") == 0);

    /* degenerate range: sampling step already exceeds the AOI diameter */
    HystModel bad;
    check("range_rejected", hyst_init(&bad, 100.0, 1.0, 0.1) == HYST_ERR_RANGE);
    check("bad_radius", hyst_init(&bad, 5.0, 0.0, 0.1) == HYST_ERR_BAD_ARG);
    check("bad_dt", hyst_init(&bad, 5.0, 10.0, 0.0) == HYST_ERR_BAD_ARG);
    check("nan_speed", hyst_init(&bad, NAN, 10.0, 0.1) == HYST_ERR_BAD_ARG);

    /* recommended band grows with speed and with radius */
    HystModel slow, fast, wide;
    hyst_init(&slow, 1.0, 20.0, 1.0 / 30.0);
    hyst_init(&fast, 50.0, 20.0, 1.0 / 30.0);
    hyst_init(&wide, 5.0, 80.0, 1.0 / 30.0);
    HystPoint ks, kf, kw;
    hyst_knee(&slow, &ks); hyst_knee(&fast, &kf); hyst_knee(&wide, &kw);
    check("band_grows_with_speed", kf.h > ks.h);
    check("band_grows_with_radius", kw.h > ks.h);

    /* speed estimator: a constant 5 u/s track sampled at 30 Hz */
    SpeedEst se;
    speed_est_reset(&se);
    double v = -1.0, serr = -1.0;
    check("speed_missing_early", speed_est_v(&se, &v, &serr) == -1);
    check("speed_why_nonnull", speed_est_why_missing(&se) != NULL);
    for (int i = 0; i < 6; i++) speed_est_add(&se, 1000 + 33 * i, 5.0 * 0.033 * i, 0.0);
    check("speed_ready", speed_est_v(&se, &v, &serr) == 0);
    check("speed_value", close_to(v, 5.0, 1e-6));
    check("speed_se_small", serr < 1e-9);

    /* duplicate timestamp is ignored, not a divide-by-zero sample */
    SpeedEst dup;
    speed_est_reset(&dup);
    speed_est_add(&dup, 1000, 0.0, 0.0);
    check("dup_ts_ignored", speed_est_add(&dup, 1000, 3.0, 0.0) == 0);
    check("dup_not_sample", dup.n == 0);
    check("backward_ts_ignored", speed_est_add(&dup, 900, 9.0, 0.0) == 0);

    /* build from a short track: missing -> refused; enough -> matches */
    HystModel mse;
    SpeedEst shorty;
    speed_est_reset(&shorty);
    speed_est_add(&shorty, 0, 0.0, 0.0);
    speed_est_add(&shorty, 33, 0.16, 0.0);
    check("build_missing", hyst_init_from_speed(&mse, &shorty, 20.0, 1.0 / 30.0)
                           == HYST_ERR_MISSING);
    check("build_from_speed_ok",
          hyst_init_from_speed(&mse, &se, 20.0, 1.0 / 30.0) == HYST_OK);
    HystPoint kse;
    hyst_knee(&mse, &kse);
    check("build_from_speed_knee", close_to(kse.h, k1.h, 1e-6));

    /* trade-off rows for the default case, knee marked */
    printf("curve_h=%.6f_churn=%.6f_latency=%.6f_g=%.6f\n",
           mid.h, mid.churn, mid.latency, mid.g);
    for (int i = 0; i <= 8; i++) {
        double h = m.hmin + (m.hmax - m.hmin) * (double)i / 8.0;
        HystPoint p;
        hyst_point(&m, h, &p);
        printf("row%d_h=%.4f_churn=%.5f_latency=%.4f_g=%.4f\n",
               i, p.h, p.churn, p.latency, p.g);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
