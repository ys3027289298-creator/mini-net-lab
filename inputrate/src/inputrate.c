
/* inputrate: windowed input-rate anomaly detection.
 *
 * Motivation: a client can modify its packet send rate to gain a status
 * advantage.  A wholesale rate increase is a location shift in the
 * inter-arrival distribution; it can only be judged against that client's
 * learned cadence, not inside a single window, so we fit a robust (trimmed
 * normal) baseline from a warm-up period and then test every later window.
 *
 * Per window we run two complementary, one-sided tests for "sending faster":
 *   A. location: is the window mean interval below the baseline mean?
 *      (a normal score test, with the standard error inflated by the
 *       uncertainty of the fitted mean)
 *   B. short mass: are there more sub-threshold intervals than the fitted
 *      baseline predicts?  (exact binomial upper tail; this catches a
 *      cheater that injects extra packets on top of the base cadence)
 * The two p-values are combined with a Bonferroni factor of 2, so the
 * per-window false-alarm probability stays at most `alpha`.
 *
 * Policy:
 *   - ANOMALOUS requires at least `min_samples` intervals and p < alpha.
 *   - Too few samples -> INSUFFICIENT, together with the evidence p-value
 *     and a confidence (1 - p) instead of a ban.
 *   - Illegal configuration / data -> IR_EINVAL, state untouched.
 * The evaluator compares decisions against a labelled set and reports a
 * one-sided Hoeffding upper bound on the false-positive rate, which is valid
 * for any number of clean windows and needs no distributional assumption.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define IR_PI 3.14159265358979323846
#define IR_P_MIN 1e-300
#define IR_MAX_WINDOW 256
#define IR_MAX_WARM  4096

typedef enum {
    IR_OK = 0,
    IR_EINVAL = -1,       /* illegal threshold or malformed data */
    IR_ENOBASELINE = -2,  /* baseline not fitted yet */
    IR_ENOSAMPLES = -3    /* window too small to decide */
} IrStatus;

typedef enum {
    IR_PASS = 0,          /* judged, no anomaly */
    IR_ANOMALOUS = 1,     /* judged anomaly (never a ban by itself) */
    IR_INSUFFICIENT = 2   /* not judged: report confidence only */
} IrVerdict;

typedef struct {
    double window_sec;     /* (0, 3600], finite */
    int    min_samples;    /* [5, IR_MAX_WINDOW] */
    double alpha;          /* (0, 0.5], per-window false-alarm budget */
    double short_z;        /* [1, 6] threshold = mu - short_z*sigma */
    int    warmup_windows; /* [1, 256] baseline windows before judging */
} IrConfig;

typedef struct {
    int    fitted;
    int    n_fit;          /* intervals kept by the trimmed fit */
    double mu;             /* baseline mean interval, seconds */
    double sigma;          /* baseline standard deviation, seconds */
} IrModel;

typedef struct {
    IrVerdict verdict;
    IrStatus  status;
    int    n;              /* intervals in the window */
    double mean;           /* window mean interval */
    double z_loc;          /* location test statistic (>0 == faster) */
    double p_loc;
    double p_short;        /* short-mass binomial upper tail */
    double p_value;        /* Bonferroni-combined */
    double confidence;     /* 1 - p_value, in [0,1] */
    double speedup_lcb;    /* 95% lower bound on mu_baseline / mean */
    int    needed_samples; /* >0 when INSUFFICIENT for lack of samples */
} IrResult;

typedef struct {
    const double* iv;
    int n;
    int is_anomaly;        /* human label */
} IrLabeledWindow;

typedef struct {
    int tp, fp, tn, fn;
    int undecided_anom;    /* anomalies with no decision (not counted as fn) */
    int undecided_clean;
    int judged_clean;
    int judged_anom;
    double fp_rate;
    double fp_rate_ucb;    /* one-sided upper bound at confidence 1-delta */
    double fn_rate_ucb;
} IrEval;

/* ------------------------------------------------------------------ */

static int ir_config_valid(const IrConfig* c) {
    if (!c) return 0;
    if (!isfinite(c->window_sec) || c->window_sec <= 0.0 ||
        c->window_sec > 3600.0) return 0;
    if (c->min_samples < 5 || c->min_samples > IR_MAX_WINDOW) return 0;
    if (!isfinite(c->alpha) || c->alpha <= 0.0 || c->alpha > 0.5) return 0;
    if (!isfinite(c->short_z) || c->short_z < 1.0 || c->short_z > 6.0) return 0;
    if (c->warmup_windows < 1 || c->warmup_windows > 256) return 0;
    return 1;
}

/* One-sided normal tail Q(z) = P(Z > z) for z >= 0. */
static double norm_q(double z) {
    if (!(z > 0.0)) return 0.5;
    return 0.5 * erfcf(z / sqrt(2.0));
}

/* P(S >= s) for S ~ Binomial(n, p), accumulated forward from the smallest
 * term of the upper tail.  Working upwards from s (instead of downwards from
 * n) keeps the log-domain terms finite when p is tiny. */
static double binom_sf(int s, int n, double p) {
    if (s <= 0) return 1.0;
    if (s > n) return 0.0;
    if (p <= 0.0) return 0.0;
    if (p >= 1.0) return 1.0;

    /* log pmf(s) via a stable log-binomial coefficient */
    double lc = lgamma((double)(n + 1)) - lgamma((double)(s + 1))
                - lgamma((double)(n - s + 1));
    double term = exp(lc + (double)s * log(p) + (double)(n - s) * log1p(-p));
    double sum = term;
    for (int k = s; k < n; k++) {
        term *= ((double)(n - k) / (double)(k + 1)) * (p / (1.0 - p));
        if (!isfinite(term) || term == 0.0) break;
        sum += term;
    }
    return sum > 1.0 ? 1.0 : sum;
}

/* Hoeffding one-sided upper bound on a [0,1] mean from k/n observations. */
static double hoeffding_ucb(int k, int n, double delta) {
    if (n <= 0 || !(delta > 0.0) || !(delta < 1.0)) return 1.0;
    double rate = (double)k / (double)n;
    double slack = sqrt(log(1.0 / delta) / (2.0 * (double)n));
    double u = rate + slack;
    return u > 1.0 ? 1.0 : u;
}

/* ------------------------------------------------------------------ */
/* baseline fit: trimmed normal (iteratively drop values beyond 3 sigma) */

static int ir_model_fit(IrModel* m, const double* iv, int n) {
    if (!m || !iv || n < 3 || n > IR_MAX_WARM) return IR_EINVAL;
    for (int i = 0; i < n; i++)
        if (!isfinite(iv[i]) || iv[i] <= 0.0) return IR_EINVAL;

    double work[IR_MAX_WARM];
    memcpy(work, iv, sizeof(double) * (size_t)n);
    int count = n;

    for (int iter = 0; iter < 4; iter++) {
        double s = 0.0;
        for (int i = 0; i < count; i++) s += work[i];
        double mu = s / (double)count;
        double ss = 0.0;
        for (int i = 0; i < count; i++) { double d = work[i] - mu; ss += d * d; }
        double sd = count > 1 ? sqrt(ss / (double)(count - 1)) : 0.0;
        if (!(sd > 0.0)) return IR_EINVAL;

        double lo = mu - 3.0 * sd, hi = mu + 3.0 * sd;
        int j = 0;
        for (int i = 0; i < count; i++)
            if (work[i] >= lo && work[i] <= hi) work[j++] = work[i];
        if (j < 3 || j == count) break;  /* converged, or too few left */
        count = j;
    }

    /* final moments over exactly the `count` values that were kept */
    double s = 0.0;
    for (int i = 0; i < count; i++) s += work[i];
    double mu = s / (double)count;
    double ss = 0.0;
    for (int i = 0; i < count; i++) { double d = work[i] - mu; ss += d * d; }
    double sd = count > 1 ? sqrt(ss / (double)(count - 1)) : 0.0;
    if (count < 3 || !(sd > 0.0) || !isfinite(mu) || !isfinite(sd)) return IR_EINVAL;
    m->fitted = 1;
    m->n_fit = count;
    m->mu = mu;
    m->sigma = sd;
    return IR_OK;
}

/* ------------------------------------------------------------------ */
/* per-window classification                                          */

int ir_classify(const IrModel* model, const IrConfig* cfg,
                const double* iv, int n, IrResult* out) {
    if (!model || !cfg || !iv || !out || !ir_config_valid(cfg)) return IR_EINVAL;
    if (n < 1 || n > IR_MAX_WINDOW) return IR_EINVAL;
    for (int i = 0; i < n; i++)
        if (!isfinite(iv[i]) || iv[i] <= 0.0) return IR_EINVAL;

    memset(out, 0, sizeof(*out));
    out->verdict = IR_INSUFFICIENT;
    out->n = n;
    out->status = IR_ENOSAMPLES;

    if (!model->fitted || !(model->sigma > 0.0) || model->n_fit < 3) {
        out->status = IR_ENOBASELINE;
        return IR_ENOBASELINE;
    }

    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += iv[i];
    out->mean = sum / (double)n;

    /* location test: standard error includes the fitted-mean uncertainty */
    double se = model->sigma *
                sqrt(1.0 / (double)n + 1.0 / (double)model->n_fit);
    out->z_loc = (model->mu - out->mean) / se;
    out->p_loc = out->z_loc > 0.0 ? norm_q(out->z_loc) : 1.0;

    /* short-mass test */
    double tau = model->mu - cfg->short_z * model->sigma;
    double p0 = norm_q(cfg->short_z);
    int short_count = 0;
    for (int i = 0; i < n; i++) if (iv[i] < tau) short_count++;
    out->p_short = binom_sf(short_count, n, p0);

    /* Bonferroni-combined p-value */
    double p = out->p_loc < out->p_short ? out->p_loc : out->p_short;
    p *= 2.0;
    if (p > 1.0) p = 1.0;
    out->p_value = p;
    out->confidence = 1.0 - p;

    /* 95% one-sided lower bound on speedup = mu_baseline / mean */
    double mean_hi = out->mean + 1.6448536269514722 * se;
    out->speedup_lcb = mean_hi > 0.0 ? model->mu / mean_hi : 0.0;

    if (n < cfg->min_samples) {
        out->verdict = IR_INSUFFICIENT;
        out->status = IR_ENOSAMPLES;
        out->needed_samples = cfg->min_samples - n;
        return IR_ENOSAMPLES;
    }

    out->status = IR_OK;
    out->verdict = p < cfg->alpha ? IR_ANOMALOUS : IR_PASS;
    return IR_OK;
}

/* ------------------------------------------------------------------ */
/* evaluation against a labelled set                                  */

int ir_eval(const IrModel* model, const IrConfig* cfg,
            const IrLabeledWindow* set, int count,
            double delta, IrEval* out) {
    if (!model || !cfg || !set || !out || count < 0) return IR_EINVAL;
    if (!ir_config_valid(cfg)) return IR_EINVAL;
    if (!(delta > 0.0) || !(delta < 1.0)) return IR_EINVAL;

    memset(out, 0, sizeof(*out));
    for (int i = 0; i < count; i++) {
        IrResult r;
        int st = ir_classify(model, cfg, set[i].iv, set[i].n, &r);
        if (st == IR_EINVAL) return IR_EINVAL;
        if (st == IR_ENOBASELINE) return IR_ENOBASELINE;

        if (r.verdict == IR_INSUFFICIENT) {
            if (set[i].is_anomaly) out->undecided_anom++;
            else out->undecided_clean++;
            continue;
        }
        if (set[i].is_anomaly) {
            out->judged_anom++;
            if (r.verdict == IR_ANOMALOUS) out->tp++; else out->fn++;
        } else {
            out->judged_clean++;
            if (r.verdict == IR_ANOMALOUS) out->fp++; else out->tn++;
        }
    }
    out->fp_rate = out->judged_clean > 0
                       ? (double)out->fp / (double)out->judged_clean : 0.0;
    out->fp_rate_ucb = hoeffding_ucb(out->fp, out->judged_clean, delta);
    out->fn_rate_ucb = hoeffding_ucb(out->fn, out->judged_anom, delta);
    return IR_OK;
}

/* ------------------------------------------------------------------ */
/* calibration against a human-labelled set                           */

typedef struct {
    int    feasible;      /* zero misses AND false-alarm bound met */
    double alpha;         /* threshold chosen (smallest that misses nothing) */
    double ceiling;       /* requested false-positive ceiling */
    int    fn;            /* missed anomalies at `alpha` */
    int    fp;            /* false positives at `alpha` */
    int    tp, tn;
    int    judged_clean;
    int    judged_anom;
    int    undecided_anom;/* anomalies the sample floor refused to judge */
    double fp_rate;
    double fp_rate_ucb;   /* one-sided upper bound at confidence 1-delta */
} IrCalibration;

/* Pick the smallest alpha that flags every labelled anomaly, then report the
 * false-positive rate and its upper bound.  The decision rule p < alpha is
 * monotone in alpha, so "smallest alpha with zero misses" is exactly the
 * threshold with the fewest false positives among all zero-miss choices.
 * Returning feasible=0 is a real answer: with this window configuration the
 * two constraints cannot both be met on this data. */
int ir_calibrate(const IrModel* model, const IrConfig* cfg,
                 const IrLabeledWindow* set, int count,
                 double fp_ceiling, double delta, IrCalibration* out) {
    if (!model || !cfg || !set || !out || count < 0) return IR_EINVAL;
    if (!ir_config_valid(cfg)) return IR_EINVAL;
    if (!(fp_ceiling > 0.0) || !(fp_ceiling < 1.0)) return IR_EINVAL;
    if (!(delta > 0.0) || !(delta < 1.0)) return IR_EINVAL;

    memset(out, 0, sizeof(*out));
    out->ceiling = fp_ceiling;
    out->alpha = -1.0;

    double max_anom_p = -1.0;
    for (int i = 0; i < count; i++) {
        IrResult r;
        int st = ir_classify(model, cfg, set[i].iv, set[i].n, &r);
        if (st == IR_EINVAL) return IR_EINVAL;
        if (st == IR_ENOBASELINE) return IR_ENOBASELINE;
        if (r.verdict == IR_INSUFFICIENT) {
            if (set[i].is_anomaly) out->undecided_anom++;
            continue;
        }
        if (set[i].is_anomaly) {
            out->judged_anom++;
            if (r.p_value > max_anom_p) max_anom_p = r.p_value;
        } else {
            out->judged_clean++;
        }
    }

    /* every anomaly must be decidable, and there must be enough headroom
     * below alpha_max to place a threshold above the worst anomaly */
    if (out->judged_anom == 0 || out->undecided_anom > 0 ||
        !(max_anom_p >= 0.0) || max_anom_p >= 0.5) {
        out->feasible = 0;
        return IR_OK;         /* a valid, well-formed negative result */
    }
    double alpha = nextafter(max_anom_p, 1.0);
    if (alpha > 0.5) alpha = 0.5;
    if (!(alpha > 0.0)) alpha = IR_P_MIN;
    out->alpha = alpha;

    for (int i = 0; i < count; i++) {
        IrResult r;
        if (ir_classify(model, cfg, set[i].iv, set[i].n, &r) == IR_EINVAL) return IR_EINVAL;
        if (r.verdict == IR_INSUFFICIENT) continue;
        int flagged = (r.p_value < alpha) ? 1 : 0;
        if (set[i].is_anomaly) {
            if (flagged) out->tp++; else out->fn++;
        } else {
            if (flagged) out->fp++; else out->tn++;
        }
    }

    out->fp_rate = out->judged_clean > 0
                       ? (double)out->fp / (double)out->judged_clean : 0.0;
    out->fp_rate_ucb = hoeffding_ucb(out->fp, out->judged_clean, delta);
    out->feasible = (out->fn == 0 && out->fp_rate_ucb <= fp_ceiling) ? 1 : 0;
    return IR_OK;
}

/* ------------------------------------------------------------------ */
/* streaming session: warming up, then one decision per elapsed window */

typedef struct {
    IrConfig cfg;
    int      have_cfg;
    IrModel  model;
    double   prev;
    int      have_prev;
    double   win_start;
    double   buf[IR_MAX_WINDOW];
    int      n;
    int      warm_windows;
    double   warm[IR_MAX_WARM];
    int      warm_n;
} IrSession;

int ir_session_init(IrSession* s, const IrConfig* cfg) {
    if (!s) return IR_EINVAL;
    if (!ir_config_valid(cfg)) return IR_EINVAL;
    memset(s, 0, sizeof(*s));
    s->cfg = *cfg;
    s->have_cfg = 1;
    return IR_OK;
}

int ir_session_observe(IrSession* s, double t, IrResult* out) {
    if (!s || !out || !s->have_cfg) return IR_EINVAL;
    if (!isfinite(t)) return IR_EINVAL;

    memset(out, 0, sizeof(*out));
    out->verdict = IR_INSUFFICIENT;
    out->status = IR_ENOSAMPLES;

    if (!s->have_prev) {
        s->prev = t;
        s->have_prev = 1;
        s->win_start = t;
        return IR_OK;
    }

    double dt = t - s->prev;
    s->prev = t;
    if (dt < 0.0) {              /* clock stepped backwards: resync, no data */
        s->win_start = t;
        return IR_OK;
    }
    if (s->n < IR_MAX_WINDOW) s->buf[s->n++] = dt;

    int closed = (t - s->win_start) >= s->cfg.window_sec;

    if (!s->model.fitted) {
        if (closed) {
            for (int i = 0; i < s->n && s->warm_n < IR_MAX_WARM; i++)
                s->warm[s->warm_n++] = s->buf[i];
            if (s->warm_windows < s->cfg.warmup_windows) s->warm_windows++;
            if (s->warm_windows >= s->cfg.warmup_windows)
                (void)ir_model_fit(&s->model, s->warm, s->warm_n);
            s->n = 0;
            s->win_start = t;
        }
        out->status = s->model.fitted ? IR_OK : IR_ENOBASELINE;
        return IR_OK;
    }

    if (!closed) {
        IrResult r;
        if (s->n > 0 &&
            ir_classify(&s->model, &s->cfg, s->buf, s->n, &r) != IR_EINVAL) {
            out->p_value = r.p_value;
            out->confidence = r.confidence;
            out->mean = r.mean;
            out->n = r.n;
            out->z_loc = r.z_loc;
            out->speedup_lcb = r.speedup_lcb;
        }
        out->verdict = IR_INSUFFICIENT;
        out->status = IR_ENOSAMPLES;
        out->needed_samples = s->cfg.min_samples > s->n
                                  ? s->cfg.min_samples - s->n : 0;
        return IR_OK;
    }

    if (s->n == 0) {             /* empty window */
        s->win_start = t;
        return IR_OK;
    }

    IrResult r;
    int st = ir_classify(&s->model, &s->cfg, s->buf, s->n, &r);
    s->n = 0;
    s->win_start = t;
    if (st == IR_EINVAL) return IR_EINVAL;
    *out = r;
    return st;
}

/* ================================================================== */
/* self-test                                                          */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned long long rng_state = 0x9E3779B97F4A7C15ULL;
static double urand(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (double)(rng_state >> 11) * (1.0 / 9007199254740992.0);
}
static double nrand(void) {
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * IR_PI * u2);
}

/* legitimate stream: normal cadence plus the occasional real pause */
static int gen_human(double* out, int n, double mu, double sd) {
    int k = 0;
    for (int i = 0; i < n; i++) {
        double x;
        if (urand() < 0.01) x = mu + 0.2 + 0.2 * urand(); /* pause */
        else x = mu + sd * nrand();
        if (x > 0.0 && k < IR_MAX_WINDOW) out[k++] = x;
    }
    return k;
}

/* cheater: whole-window speed-up */
static int gen_faster(double* out, int n, double mu, double sd) {
    int k = 0;
    for (int i = 0; i < n; i++) {
        double x = mu + sd * nrand();
        if (x > 0.0 && k < IR_MAX_WINDOW) out[k++] = x;
    }
    return k;
}

/* cheater: keep the base cadence but inject extra short packets */
static int gen_inject(double* out, int n, double base, double sd) {
    int k = 0;
    for (int i = 0; i < n; i++) {
        double x;
        if (urand() < 0.35) x = 0.004 + 0.006 * urand();
        else x = base + sd * nrand();
        if (x > 0.0 && k < IR_MAX_WINDOW) out[k++] = x;
    }
    return k;
}

#define CLEAN_N 200
#define ANOM_PER_KIND 30
#define WIN_N 40

static double clean_iv[CLEAN_N][WIN_N];
static double fast2_iv[ANOM_PER_KIND][WIN_N];
static double fast15_iv[ANOM_PER_KIND][WIN_N];
static double inject_iv[ANOM_PER_KIND][WIN_N];
static double marginal_iv[ANOM_PER_KIND][WIN_N];

int main(void) {
    const double base_mu = 0.050, base_sd = 0.006;
    const double alpha = 1e-4;

    /* --- baseline fit ------------------------------------------------ */
    double pool[400];
    int pn = gen_human(pool, 400, base_mu, base_sd);
    IrModel model;
    memset(&model, 0, sizeof(model));
    check("fit_ok", ir_model_fit(&model, pool, pn) == IR_OK);
    check("fit_fitted", model.fitted == 1);
    check("fit_mu_close", fabs(model.mu - base_mu) < 0.0015);
    check("fit_sigma_trimmed", model.sigma > 0.003 && model.sigma < 0.010);

    IrConfig cfg = { 0.25, 20, alpha, 2.0, 8 };
    check("config_ok", ir_config_valid(&cfg) == 1);

    /* --- clean windows: false positives and their bound --------------- */
    IrLabeledWindow set[CLEAN_N + 3 * ANOM_PER_KIND];
    int cnt = 0;
    for (int i = 0; i < CLEAN_N; i++) {
        int n = gen_human(clean_iv[i], WIN_N, base_mu, base_sd);
        set[cnt].iv = clean_iv[i]; set[cnt].n = n; set[cnt].is_anomaly = 0; cnt++;
    }
    /* --- labelled anomalies ------------------------------------------ */
    for (int i = 0; i < ANOM_PER_KIND; i++) {
        int n = gen_faster(fast2_iv[i], WIN_N, 0.025, 0.004);
        set[cnt].iv = fast2_iv[i]; set[cnt].n = n; set[cnt].is_anomaly = 1; cnt++;
    }
    for (int i = 0; i < ANOM_PER_KIND; i++) {
        int n = gen_faster(fast15_iv[i], WIN_N, 0.0333, 0.005);
        set[cnt].iv = fast15_iv[i]; set[cnt].n = n; set[cnt].is_anomaly = 1; cnt++;
    }
    for (int i = 0; i < ANOM_PER_KIND; i++) {
        int n = gen_inject(inject_iv[i], WIN_N, base_mu, base_sd);
        set[cnt].iv = inject_iv[i]; set[cnt].n = n; set[cnt].is_anomaly = 1; cnt++;
    }

    IrEval ev;
    check("eval_ok", ir_eval(&model, &cfg, set, cnt, 0.05, &ev) == IR_OK);
    printf("clean_judged=%d fp=%d fp_rate_ucb=%.4f\n",
           ev.judged_clean, ev.fp, ev.fp_rate_ucb);
    printf("anom_judged=%d tp=%d fn=%d undecided_anom=%d\n",
           ev.judged_anom, ev.tp, ev.fn, ev.undecided_anom);
    check("no_false_negatives", ev.fn == 0);
    check("no_undecided_anomalies", ev.undecided_anom == 0);
    check("all_anomalies_caught", ev.tp == 3 * ANOM_PER_KIND);
    check("fp_rate_bounded", ev.fp_rate_ucb <= 0.10);
    check("fp_small", ev.fp <= 2);

    /* per-pattern detection, to show the tests are complementary */
    int det2 = 0, det15 = 0, detinj = 0;
    for (int i = 0; i < ANOM_PER_KIND; i++) {
        IrResult r;
        ir_classify(&model, &cfg, fast2_iv[i], WIN_N, &r);
        det2 += r.verdict == IR_ANOMALOUS;
        ir_classify(&model, &cfg, fast15_iv[i], WIN_N, &r);
        det15 += r.verdict == IR_ANOMALOUS;
        ir_classify(&model, &cfg, inject_iv[i], WIN_N, &r);
        detinj += r.verdict == IR_ANOMALOUS;
    }
    printf("detect_fast2=%d/%d detect_fast1.5=%d/%d detect_inject=%d/%d\n",
           det2, ANOM_PER_KIND, det15, ANOM_PER_KIND, detinj, ANOM_PER_KIND);
    check("detect_2x_all", det2 == ANOM_PER_KIND);
    check("detect_1.5x_all", det15 == ANOM_PER_KIND);
    check("detect_inject_all", detinj == ANOM_PER_KIND);

    /* --- calibration on the labelled set ------------------------------ */
    IrCalibration cal;
    check("calibrate_ok",
          ir_calibrate(&model, &cfg, set, cnt, 0.10, 0.05, &cal) == IR_OK);
    printf("cal_alpha=%.3g cal_fn=%d cal_fp=%d cal_fp_ucb=%.4f feasible=%d\n",
           cal.alpha, cal.fn, cal.fp, cal.fp_rate_ucb, cal.feasible);
    check("calibrate_zero_misses", cal.fn == 0);
    check("calibrate_catches_all", cal.tp == 3 * ANOM_PER_KIND);
    check("calibrate_feasible_at_10pct", cal.feasible == 1);
    check("calibrate_alpha_sane", cal.alpha > 0.0 && cal.alpha < 0.01);

    IrCalibration tight;
    check("calibrate_tight_ok",
          ir_calibrate(&model, &cfg, set, cnt, 0.01, 0.05, &tight) == IR_OK);
    check("calibrate_tight_infeasible", tight.feasible == 0);
    check("calibrate_tight_still_zero_misses", tight.fn == 0);
    check("calibrate_bad_ceiling",
          ir_calibrate(&model, &cfg, set, cnt, 0.0, 0.05, &cal) == IR_EINVAL);
    check("calibrate_bad_delta",
          ir_calibrate(&model, &cfg, set, cnt, 0.1, 1.0, &cal) == IR_EINVAL);

    /* --- marginal cheater (1.2x) plus a sample-starved anomaly --------
     * A 1.2x shift is only ~2 sigma per window: some windows land above the
     * naive threshold.  Calibration must raise alpha to keep zero misses,
     * and the false-alarm bound must still be honest. */
    for (int i = 0; i < ANOM_PER_KIND; i++)
        (void)gen_faster(marginal_iv[i], WIN_N, base_mu / 1.2, base_sd);
    IrLabeledWindow marg[ANOM_PER_KIND];
    for (int i = 0; i < ANOM_PER_KIND; i++) {
        marg[i].iv = marginal_iv[i];
        marg[i].n = WIN_N;
        marg[i].is_anomaly = 1;
    }

    IrCalibration mcal;
    check("calibrate_marginal_ok",
          ir_calibrate(&model, &cfg, marg, ANOM_PER_KIND, 0.10, 0.05, &mcal) == IR_OK);
    printf("marginal_alpha=%.3g fn=%d tp=%d\n", mcal.alpha, mcal.fn, mcal.tp);
    check("marginal_zero_misses", mcal.fn == 0);
    check("marginal_detects_all", mcal.tp == ANOM_PER_KIND);

    /* the same threshold applied to the clean set: the price of zero misses
     * on a 1.2x cheater, measured rather than assumed. */
    IrConfig mcfg = cfg;
    mcfg.alpha = mcal.alpha;
    IrEval mcost;
    check("marginal_cost_ok",
          ir_eval(&model, &mcfg, set, CLEAN_N, 0.05, &mcost) == IR_OK);
    printf("marginal_cost_fp=%d fp_ucb=%.4f judged_clean=%d\n",
           mcost.fp, mcost.fp_rate_ucb, mcost.judged_clean);
    check("marginal_cost_still_bounded", mcost.fp_rate_ucb <= 0.10);

    /* a labelled anomaly that the sample floor refuses to judge must make
     * the whole calibration infeasible: a silent miss is still a miss. */
    double short_anom[4] = { 0.010, 0.011, 0.010, 0.012 };
    IrLabeledWindow with_short[ANOM_PER_KIND + 1];
    for (int i = 0; i < ANOM_PER_KIND; i++) with_short[i] = marg[i];
    with_short[ANOM_PER_KIND].iv = short_anom;
    with_short[ANOM_PER_KIND].n = 4;
    with_short[ANOM_PER_KIND].is_anomaly = 1;
    IrCalibration scal;
    check("calibrate_starved_ok",
          ir_calibrate(&model, &cfg, with_short, ANOM_PER_KIND + 1, 0.10, 0.05,
                       &scal) == IR_OK);
    check("calibrate_starved_infeasible", scal.feasible == 0);
    check("calibrate_starved_counts_undecided", scal.undecided_anom == 1);

    /* --- insufficient samples: confidence, not a ban ------------------- */
    double tiny[3] = { 0.011, 0.013, 0.010 };   /* obviously too fast */
    IrResult tr;
    check("tiny_status", ir_classify(&model, &cfg, tiny, 3, &tr) == IR_ENOSAMPLES);
    check("tiny_not_anomalous", tr.verdict == IR_INSUFFICIENT);
    check("tiny_needed_samples", tr.needed_samples == cfg.min_samples - 3);
    check("tiny_confidence_present", tr.confidence >= 0.0 && tr.confidence <= 1.0);
    check("tiny_evidence_is_strong", tr.confidence > 0.9);
    printf("tiny_n=%d tiny_p=%.3g tiny_confidence=%.3f needed=%d\n",
           tr.n, tr.p_value, tr.confidence, tr.needed_samples);

    /* --- illegal thresholds are rejected ------------------------------ */
    IrConfig bad = cfg;
    bad.window_sec = 0.0;
    check("reject_window_zero", ir_config_valid(&bad) == 0);
    bad = cfg; bad.window_sec = NAN;
    check("reject_window_nan", ir_config_valid(&bad) == 0);
    bad = cfg; bad.window_sec = INFINITY;
    check("reject_window_inf", ir_config_valid(&bad) == 0);
    bad = cfg; bad.alpha = 0.0;
    check("reject_alpha_zero", ir_config_valid(&bad) == 0);
    bad = cfg; bad.alpha = 0.75;
    check("reject_alpha_high", ir_config_valid(&bad) == 0);
    bad = cfg; bad.min_samples = 0;
    check("reject_min_zero", ir_config_valid(&bad) == 0);
    bad = cfg; bad.min_samples = IR_MAX_WINDOW + 1;
    check("reject_min_high", ir_config_valid(&bad) == 0);
    bad = cfg; bad.short_z = 0.1;
    check("reject_short_z_low", ir_config_valid(&bad) == 0);
    bad = cfg; bad.short_z = 9.0;
    check("reject_short_z_high", ir_config_valid(&bad) == 0);
    bad = cfg; bad.warmup_windows = 0;
    check("reject_warmup_zero", ir_config_valid(&bad) == 0);

    IrSession ss;
    memset(&ss, 0xAB, sizeof(ss));
    IrSession ss_before = ss;
    check("session_init_rejects", ir_session_init(&ss, &bad) == IR_EINVAL);
    check("session_init_untouched", memcmp(&ss, &ss_before, sizeof(ss)) == 0);

    check("classify_null", ir_classify(NULL, &cfg, tiny, 3, &tr) == IR_EINVAL);
    double neg[20]; for (int i = 0; i < 20; i++) neg[i] = 0.02;
    neg[5] = -1.0;
    check("classify_neg_interval", ir_classify(&model, &cfg, neg, 20, &tr) == IR_EINVAL);
    double nanv[20]; for (int i = 0; i < 20; i++) nanv[i] = 0.02;
    nanv[3] = NAN;
    check("classify_nan_interval", ir_classify(&model, &cfg, nanv, 20, &tr) == IR_EINVAL);
    check("classify_zero_n", ir_classify(&model, &cfg, neg, 0, &tr) == IR_EINVAL);
    IrModel frozen; memset(&frozen, 0, sizeof(frozen));
    check("classify_no_baseline", ir_classify(&frozen, &cfg, tiny, 3, &tr) == IR_ENOBASELINE);
    check("fit_too_few", ir_model_fit(&frozen, tiny, 2) == IR_EINVAL);
    double same[5] = { 0.02, 0.02, 0.02, 0.02, 0.02 };
    check("fit_zero_variance", ir_model_fit(&frozen, same, 5) == IR_EINVAL);
    check("eval_bad_delta", ir_eval(&model, &cfg, set, 1, 0.0, &ev) == IR_EINVAL);

    /* --- streaming: warm up on clean data, then a cheater arrives ----- */
    IrConfig scfg = { 0.5, 5, 1e-4, 2.0, 8 };
    IrSession sess;
    check("session_init_ok", ir_session_init(&sess, &scfg) == IR_OK);

    double t = 0.0;
    IrResult sr;
    int early_flags = 0, late_flags = 0, undecided_clean = 0;

    for (int w = 0; w < 30; w++) {
        double end = (double)(w + 1) * scfg.window_sec;
        while (t < end - 1e-9) {
            ir_session_observe(&sess, t, &sr);
            if (sr.verdict == IR_ANOMALOUS) early_flags++;
            if (sr.verdict == IR_INSUFFICIENT && sess.model.fitted) undecided_clean++;
            t += base_mu + base_sd * nrand();
        }
    }
    check("session_baseline_fitted", sess.model.fitted == 1);
    check("session_no_early_flags", early_flags == 0);

    for (int w = 0; w < 10; w++) {
        double end = t + scfg.window_sec;
        while (t < end - 1e-9) {
            ir_session_observe(&sess, t, &sr);
            if (sr.verdict == IR_ANOMALOUS) late_flags++;
            t += 0.0125 + 0.002 * nrand(); /* cheater: ~4x faster */
        }
    }
    printf("session_early_flags=%d late_flags=%d\n", early_flags, late_flags);
    check("session_detects_cheater", late_flags > 0);

    /* --- streaming: a short window yields confidence, not a verdict --- */
    IrConfig tcfg = { 0.02, 20, 1e-4, 2.0, 2 }; /* too few packets per window */
    IrSession tsess;
    check("tiny_session_init", ir_session_init(&tsess, &tcfg) == IR_OK);
    double tt = 0.0;
    IrResult trr;
    int tiny_flags = 0, tiny_insufficient = 0;
    for (int w = 0; w < 200; w++) {
        double end = (double)(w + 1) * tcfg.window_sec;
        while (tt < end - 1e-9) {
            ir_session_observe(&tsess, tt, &trr);
            if (trr.verdict == IR_ANOMALOUS) tiny_flags++;
            if (trr.verdict == IR_INSUFFICIENT && tsess.model.fitted &&
                trr.n > 0) tiny_insufficient++;
            tt += 0.004 + 0.001 * nrand(); /* fast, but windows are too small */
        }
    }
    printf("tiny_session_flags=%d insufficient_reports=%d\n",
           tiny_flags, tiny_insufficient);
    check("tiny_session_no_ban", tiny_flags == 0);
    check("tiny_session_reports_insufficient", tiny_insufficient > 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
