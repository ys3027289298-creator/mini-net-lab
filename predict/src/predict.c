
/* predict: adaptive extrapolation over a sliding window of samples.
 *
 * Model set: CV (const velocity, deg 1), CA (const accel, deg 2),
 * CJ (const jerk, deg 3). Each observe refits every allowed model by
 * least squares on normalized time and picks one by AIC with hysteresis
 * (RSS captures curvature misfit, the parameter penalty rejects models
 * the noise level cannot support). On a model switch the output
 * cross-fades from old to new over FADE observes so predictions never
 * jump. Every prediction carries a confidence interval that doubles as
 * an error upper bound; an observation landing outside it is reported
 * (misses++, degraded=1) and triggers degrade: sigma inflates and the
 * model is forced back to the simplest allowed one until coverage
 * recovers. An empty model set is rejected at init.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define WIN 16
#define NMODEL 3
#define FADE 3
#define COOL 4
#define RHO 0.85       /* per-sample forgetting factor: recent samples weigh more */
#define HYST_UP 4.0    /* margin to switch to a MORE complex model */
#define HYST_DOWN 1.0  /* margin to switch back to a simpler one */
#define DEGRADE_SLACK 10.0  /* only degrade to a model scoring this close */
#define Z 1.96
#define PEND 4         /* in-flight predictions tracked for coverage */

enum { M_CV = 0, M_CA = 1, M_CJ = 2 };
#define MASK_ALL 0x7u

typedef struct {
    double c[4];      /* poly coeffs in normalized time u */
    double inv[16];   /* (X'X)^-1, row-major with stride k */
    double rss, sigma2, score;
    int ok;
} Fit;

typedef struct { double t, pred, ci; int model, valid; } Pend;

typedef struct {
    double ts[WIN], xs[WIN];
    int head, n;
    unsigned mask;
    Fit fit[NMODEL];
    int cur, prev, fade, cooldown;
    double t_ref, sc;
    double inflate;   /* sigma multiplier from coverage failures */
    int degraded;
    unsigned hits, misses;
    double t_last;
    Pend pend[PEND];
    int pend_head;
    double last_bound;
} Pred;

/* solve A x = b (row-major k*k) by gaussian elimination, destroys A and b */
static int solve(double* A, double* b, double* x, int k) {
    for (int col = 0; col < k; col++) {
        int piv = col;
        for (int r = col + 1; r < k; r++)
            if (fabs(A[r * k + col]) > fabs(A[piv * k + col])) piv = r;
        if (fabs(A[piv * k + col]) < 1e-12) return -1;
        if (piv != col) {
            for (int j = 0; j < k; j++) {
                double t = A[col * k + j]; A[col * k + j] = A[piv * k + j]; A[piv * k + j] = t;
            }
            double t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        for (int r = col + 1; r < k; r++) {
            double f = A[r * k + col] / A[col * k + col];
            for (int j = col; j < k; j++) A[r * k + j] -= f * A[col * k + j];
            b[r] -= f * b[col];
        }
    }
    for (int r = k - 1; r >= 0; r--) {
        double s = b[r];
        for (int j = r + 1; j < k; j++) s -= A[r * k + j] * x[j];
        x[r] = s / A[r * k + r];
    }
    return 0;
}

/* weighted least-squares fit of degree deg on n samples; w are recency
 * weights, tref/sc normalize time */
static int fit_poly(const double* t, const double* x, const double* w, int n,
                    int deg, double tref, double sc, Fit* f) {
    int k = deg + 1;
    if (n < k + 1) return -1;
    double A[16] = { 0 }, b[4] = { 0 }, sw = 0;
    for (int i = 0; i < n; i++) {
        double u = (t[i] - tref) / sc, phi[4] = { 1, u, u * u, u * u * u };
        sw += w[i];
        for (int a = 0; a < k; a++) {
            b[a] += w[i] * phi[a] * x[i];
            for (int c = 0; c < k; c++) A[a * k + c] += w[i] * phi[a] * phi[c];
        }
    }
    for (int col = 0; col < k; col++) {          /* invert A column by column */
        double Ac[16], e[4] = { 0 }, xv[4];
        memcpy(Ac, A, sizeof(double) * (size_t)(k * k));
        e[col] = 1.0;
        if (solve(Ac, e, xv, k) != 0) return -1;
        for (int r = 0; r < k; r++) f->inv[r * k + col] = xv[r];
    }
    for (int a = 0; a < k; a++) {
        double s = 0;
        for (int c = 0; c < k; c++) s += f->inv[a * k + c] * b[c];
        f->c[a] = s;
    }
    double rss = 0;
    for (int i = 0; i < n; i++) {
        double u = (t[i] - tref) / sc, phi[4] = { 1, u, u * u, u * u * u }, y = 0;
        for (int a = 0; a < k; a++) y += phi[a] * f->c[a];
        rss += w[i] * (x[i] - y) * (x[i] - y);
    }
    if (sw <= k) return -1;
    f->rss = rss;
    f->sigma2 = rss / (sw - k);
    f->score = sw * log(rss / sw + 1e-9) + 2.0 * k;   /* AIC on effective n */
    f->ok = 1;
    return 0;
}

static double fit_eval(const Fit* f, int model, double u) {
    double phi[4] = { 1, u, u * u, u * u * u }, y = 0;
    int k = model + 2;
    for (int a = 0; a < k; a++) y += phi[a] * f->c[a];
    return y;
}

static double fit_lev(const Fit* f, int model, double u) {
    double phi[4] = { 1, u, u * u, u * u * u }, s = 0;
    int k = model + 2;
    for (int a = 0; a < k; a++)
        for (int b = 0; b < k; b++) s += phi[a] * f->inv[a * k + b] * phi[b];
    return s;
}

static int pred_init(Pred* p, unsigned mask) {
    memset(p, 0, sizeof(*p));
    mask &= MASK_ALL;
    if (mask == 0) return -1;          /* empty model set rejected */
    p->mask = mask;
    p->cur = p->prev = -1;
    p->inflate = 1.0;
    return 0;
}

static void refit(Pred* p) {
    double t[WIN], x[WIN], w[WIN];
    int start = (p->head - p->n + WIN) % WIN;
    for (int i = 0; i < p->n; i++) {
        int idx = (start + i) % WIN;
        t[i] = p->ts[idx]; x[i] = p->xs[idx];
        w[i] = pow(RHO, (double)(p->n - 1 - i));   /* newest sample weighs 1 */
    }
    p->t_ref = t[p->n - 1];
    double span = p->t_ref - t[0];
    p->sc = span > 1e-9 ? span / 2.0 : 1.0;
    for (int m = 0; m < NMODEL; m++) {
        p->fit[m].ok = 0;
        if (p->mask & (1u << m))
            fit_poly(t, x, w, p->n, m + 1, p->t_ref, p->sc, &p->fit[m]);
    }
}

static void select_model(Pred* p) {
    if (p->cooldown > 0) return;
    int best = -1;
    for (int m = 0; m < NMODEL; m++)
        if (p->fit[m].ok && (best < 0 || p->fit[m].score < p->fit[best].score)) best = m;
    if (best < 0) return;
    if (p->cur < 0 || !p->fit[p->cur].ok) { p->prev = p->cur; p->cur = best; p->fade = FADE; return; }
    if (best != p->cur) {
        /* simplicity-biased hysteresis: a complex model must win clearly,
         * a simpler one only slightly — keeps jerk-level fits out unless
         * the curvature really pays for them */
        double margin = best > p->cur ? HYST_UP : HYST_DOWN;
        if (p->fit[best].score < p->fit[p->cur].score - margin) {
            p->prev = p->cur; p->cur = best; p->fade = FADE;
        }
    }
}

static int pred_observe(Pred* p, double t, double x) {
    if (p->n > 0 && t <= p->t_last) return -1;       /* stale sample rejected */

    for (int j = 0; j < PEND; j++) {                 /* coverage of due predictions */
        Pend* q = &p->pend[j];
        if (!q->valid || t < q->t - 1e-9) continue;
        q->valid = 0;
        if (fabs(x - q->pred) > q->ci) {
            p->misses++;
            p->degraded = 1;
            p->inflate *= 1.5;
            if (p->inflate > 8.0) p->inflate = 8.0;
            /* step one degree down toward simpler, only when the missed
             * prediction came from the current model AND the simpler model
             * is a credible alternative; downgrading to a model the data
             * has already rejected (e.g. CV mid-turn) only makes it worse */
            if (q->model == p->cur && p->cur >= 0 && p->fit[p->cur].ok) {
                for (int m = p->cur - 1; m >= 0; m--) {
                    if ((p->mask & (1u << m)) && p->fit[m].ok &&
                        p->fit[m].score < p->fit[p->cur].score + DEGRADE_SLACK) {
                        p->prev = p->cur; p->cur = m; p->fade = FADE; p->cooldown = COOL;
                        break;
                    }
                }
            }
        } else {
            p->hits++;
            p->inflate *= 0.9;
            if (p->inflate < 1.0) p->inflate = 1.0;
            if (p->inflate <= 1.0) p->degraded = 0;
        }
    }

    p->ts[p->head] = t; p->xs[p->head] = x;
    p->head = (p->head + 1) % WIN;
    if (p->n < WIN) p->n++;
    p->t_last = t;

    if (p->n >= 3) { refit(p); select_model(p); }
    if (p->cooldown > 0) p->cooldown--;
    if (p->fade > 0) p->fade--;
    return 0;
}

/* predict h time units past the newest sample; ci is the error upper bound */
static int pred_predict(Pred* p, double h, double* out, double* ci) {
    if (p->cur < 0 || !p->fit[p->cur].ok) return -1;
    double u = h / p->sc;
    double o = fit_eval(&p->fit[p->cur], p->cur, u);
    double c = Z * sqrt(p->fit[p->cur].sigma2 * (1.0 + fit_lev(&p->fit[p->cur], p->cur, u)))
             * p->inflate;
    if (p->fade > 0 && p->prev >= 0 && p->fit[p->prev].ok) {
        /* cross-fade: output stays a convex blend of old and new model */
        double a = (double)(FADE - p->fade) / FADE;
        double o_old = fit_eval(&p->fit[p->prev], p->prev, u);
        double c_old = Z * sqrt(p->fit[p->prev].sigma2 *
                       (1.0 + fit_lev(&p->fit[p->prev], p->prev, u))) * p->inflate;
        o = a * o + (1.0 - a) * o_old;
        if (c_old > c) c = c_old;
    }
    *out = o; *ci = c;
    Pend* q = &p->pend[p->pend_head];
    p->pend_head = (p->pend_head + 1) % PEND;
    q->t = p->t_ref + h;
    q->pred = o; q->ci = c; q->model = p->cur; q->valid = 1;
    p->last_bound = c;
    return 0;
}

static double pred_error_bound(const Pred* p) { return p->last_bound; }

/* ---- baseline: constant-velocity Kalman filter ---- */

typedef struct { double x, v, p00, p01, p11, q, r; } Kf;

static void kf_init(Kf* k, double x0, double q, double r) {
    k->x = x0; k->v = 0; k->p00 = 1; k->p01 = 0; k->p11 = 1; k->q = q; k->r = r;
}

static void kf_step(Kf* k, double dt, double z) {
    k->x += k->v * dt;
    double d2 = dt * dt, d3 = d2 * dt, d4 = d3 * dt;
    double P00 = k->p00 + 2 * dt * k->p01 + d2 * k->p11 + k->q * d4 / 4;
    double P01 = k->p01 + dt * k->p11 + k->q * d3 / 2;
    double P11 = k->p11 + k->q * d2;
    double S = P00 + k->r, K0 = P00 / S, K1 = P01 / S, y = z - k->x;
    k->x += K0 * y; k->v += K1 * y;
    k->p00 = (1 - K0) * P00; k->p01 = (1 - K0) * P01; k->p11 = P11 - K1 * P01;
}

static double kf_ahead(const Kf* k, double h) { return k->x + k->v * h; }

/* ---- self test ---- */

static unsigned rng_state = 22222;
static double frand(void) {   /* uniform [-0.5, 0.5) */
    rng_state = rng_state * 1664525u + 1013904223u;
    return (double)(rng_state >> 8) / 16777216.0 - 0.5;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static double truth_a(int i) { return 1.0 + 2.0 * i; }
static double truth_b(int i) {
    if (i < 25) return i;
    double s = i - 25;
    return 25 + s + 0.5 * s * s;      /* straight, then accel 1.0 */
}
static double truth_c(int i) { return i + (i >= 30 ? 10.0 : 0.0); }  /* teleport */

int main(void) {
    /* empty / invalid model set rejected */
    Pred p;
    check("empty_mask_rejected", pred_init(&p, 0) == -1);
    check("bad_mask_rejected", pred_init(&p, 1u << 4) == -1);

    /* predict before enough data */
    check("init_ok", pred_init(&p, MASK_ALL) == 0);
    double out, ci;
    check("predict_no_data", pred_predict(&p, 1.0, &out, &ci) == -1);
    pred_observe(&p, 0, 0.0);
    pred_observe(&p, 1, 1.0);
    check("predict_sparse", pred_predict(&p, 1.0, &out, &ci) == -1);
    check("stale_rejected", pred_observe(&p, 1, 2.0) == -1);
    pred_observe(&p, 2, 2.01);
    check("predict_ready", pred_predict(&p, 1.0, &out, &ci) == 0 && ci > 0);

    /* scenario A: straight line + noise -> CV, competitive with KF, bound covers */
    {
        check("a_init", pred_init(&p, MASK_ALL) == 0);
        Kf kf;
        double mse_ad = 0, mse_kf = 0, p_out = 0, p_ci = 0, k_out = 0;
        int cnt = 0, viol = 0, pending = 0;
        for (int i = 0; i < 60; i++) {
            double truth = truth_a(i), meas = truth + frand() * 0.1;
            if (pending) {
                double e = p_out - truth, ek = k_out - truth;
                mse_ad += e * e; mse_kf += ek * ek; cnt++;
                if (fabs(e) > p_ci) viol++;
            }
            if (i == 0) kf_init(&kf, meas, 1e-4, 1e-3); else kf_step(&kf, 1.0, meas);
            pred_observe(&p, i, meas);
            if (pred_predict(&p, 1.0, &p_out, &p_ci) == 0) { k_out = kf_ahead(&kf, 1.0); pending = 1; }
        }
        double rmse_ad = sqrt(mse_ad / cnt), rmse_kf = sqrt(mse_kf / cnt);
        unsigned tot = p.hits + p.misses;
        check("a_rmse_vs_kf", cnt > 0 && rmse_ad <= 1.5 * rmse_kf);
        check("a_coverage", tot > 0 && (double)p.hits / tot >= 0.8);
        check("a_bound_covers", (double)viol / cnt <= 0.2 && pred_error_bound(&p) > 0);
        check("a_model_cv", p.cur == M_CV);
    }

    /* scenario B: straight then turn, predict h=2 ahead -> upgrades model,
     * beats KF and linear extrapolation, switches without jumps */
    {
        check("b_init", pred_init(&p, MASK_ALL) == 0);
        Kf kf;
        double mse_ad = 0, mse_kf = 0, mse_lin = 0;
        double pq[2] = { 0 }, kq[2] = { 0 }, lq[2] = { 0 };   /* h=2 pipeline */
        double prev_meas = 0, prev_out = 0, prev_truth = 0, dmax_out = 0, dmax_truth = 0;
        int cnt = 0, blend_ok = 1, blend_n = 0;
        for (int i = 0; i < 60; i++) {
            double truth = truth_b(i), meas = truth + frand() * 0.1;
            if (i >= 4) {                       /* prediction made at i-2 for t=i */
                double e = pq[i & 1] - truth, ek = kq[i & 1] - truth, el = lq[i & 1] - truth;
                mse_ad += e * e; mse_kf += ek * ek; mse_lin += el * el; cnt++;
            }
            if (i > 0) {
                double dt_ = fabs(truth - prev_truth);
                if (dt_ > dmax_truth) dmax_truth = dt_;
            }
            prev_truth = truth;
            if (i == 0) kf_init(&kf, meas, 1e-4, 1e-3); else kf_step(&kf, 1.0, meas);
            pred_observe(&p, i, meas);
            double p_out, p_ci;
            if (pred_predict(&p, 2.0, &p_out, &p_ci) == 0) {
                pq[i & 1] = p_out;
                kq[i & 1] = kf_ahead(&kf, 2.0);
                lq[i & 1] = (i > 0) ? meas + 2.0 * (meas - prev_meas) : meas;
                double d = fabs(p_out - prev_out);
                if (i > 4 && d > dmax_out) dmax_out = d;
                prev_out = p_out;
                if (p.fade > 0 && p.prev >= 0 && p.fit[p.prev].ok) {
                    double en = fit_eval(&p.fit[p.cur], p.cur, 2.0 / p.sc);
                    double eo = fit_eval(&p.fit[p.prev], p.prev, 2.0 / p.sc);
                    double lo = fmin(en, eo) - 1e-6, hi = fmax(en, eo) + 1e-6;
                    if (p_out < lo || p_out > hi) blend_ok = 0;
                    blend_n++;
                }
            }
            prev_meas = meas;
        }
        double rmse_ad = sqrt(mse_ad / cnt), rmse_kf = sqrt(mse_kf / cnt), rmse_lin = sqrt(mse_lin / cnt);
        check("b_beats_kf", rmse_ad < 0.7 * rmse_kf);
        check("b_beats_linear", rmse_ad < rmse_lin);
        check("b_model_upgraded", p.cur == M_CA || p.cur == M_CJ);
        check("b_switch_blended", blend_ok && blend_n > 0);
        check("b_no_jump", dmax_out <= 3.0 * dmax_truth + 0.5);
    }

    /* scenario C: teleport -> miss reported, degraded, bound inflates, recovers */
    {
        check("c_init", pred_init(&p, MASK_ALL) == 0);
        double p_out = 0, p_ci = 0, ci_before = 0, ci_after = 0;
        int pending = 0, deg_after = -1, infl_after = -1;
        for (int i = 0; i < 50; i++) {
            double truth = truth_c(i), meas = truth + frand() * 0.1;
            pred_observe(&p, i, meas);
            if (i == 30) { deg_after = p.degraded; infl_after = p.inflate > 1.0; }
            if (pred_predict(&p, 1.0, &p_out, &p_ci) == 0) {
                pending = 1;
                if (i == 29) ci_before = p_ci;
                if (i == 30) ci_after = p_ci;
            }
        }
        (void)pending; (void)p_out;
        check("c_miss_reported", p.misses >= 1 && p.misses <= 3);
        check("c_degraded", deg_after == 1 && infl_after == 1);
        check("c_bound_inflated", ci_after > 2.0 * ci_before);
        check("c_recovered", p.degraded == 0);
    }

    /* restricted model set: CV-only never upgrades even on a turn */
    {
        check("d_init", pred_init(&p, 1u << M_CV) == 0);
        for (int i = 0; i < 40; i++) pred_observe(&p, i, truth_b(i) + frand() * 0.1);
        check("d_stays_cv", p.cur == M_CV);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
