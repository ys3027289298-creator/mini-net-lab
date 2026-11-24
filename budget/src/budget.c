/* budget: distance/vision weighted entity update-rate allocation.
 *
 * Two functions share the x axis f (updates per second, one entity):
 *   bw(f)   = PKT * f          bytes/s, rising with f
 *   err(f)  = RMS position error, metres, falling with f
 *
 * err(f) has two branches because the receiver's behaviour differs by
 * visibility: in view it dead-reckons from the snapshot, out of view it
 * freezes the last snapshot (nothing renders the entity, so nothing predicts
 * it).  The two branches fall at different rates, which matters for any
 * allocation that hands out-of-view entities a small keepalive rate.
 *
 * Allocation: f_i = beta * B/PKT * w_i / sum(n_j * w_j),
 *   w(d) = 1/(1+d/D0) in view, W_OUT outside, f capped at the server tick.
 *
 *   budget            self-test
 *   budget curves     write both curves as CSV
 *   budget cal        constant-rate calibration CSV
 *   budget exp        three candidate betas x three population tiers
 *   budget equal      equal-split baseline at the selected beta
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

/* ---- world and link constants ---- */
#define V_SPEED   6.0           /* m/s                                      */
#define YAW_SIG   1.0           /* yaw-rate std, rad/s                      */
#define LAT       0.040         /* one-way latency, s                       */
#define TICK      (1.0/60.0)    /* server tick: rate cap                    */
#define RENDER    (1.0/60.0)    /* client render interval                   */
#define QSTEP     (1.0/64.0)    /* position quantization, m                 */
#define PKT       40.0          /* bytes per entity update (28 IP/UDP + 12) */
#define BUDGET    (10.0*1024.0) /* bytes/s per client                       */
#define R_VIS     25.0          /* vision radius, m                         */
#define D0        15.0          /* weight falloff distance, m               */
#define W_OUT     0.15          /* weight outside vision                    */
#define SIM_TIME  40.0
#define WARM      5.0
#define TRACKS    24            /* simulated tracks per class               */

#define NCLS 3
enum { CLS_NEAR = 0, CLS_MID, CLS_OUT };
static const double CLS_DIST[NCLS] = { 4.0, 18.0, 32.0 };

static const int TIERS[3][NCLS] = {
    {  3,  4,  2 },
    {  6, 10,  8 },
    { 10, 16, 22 },
};
static const char* TIER_NAME[3] = { "sparse", "medium", "dense" };
static const double BETA[3] = { 0.55, 0.70, 0.85 };
#define SEL_BETA 1              /* index of the selected candidate (0.70) */

/* ---- deterministic RNG: SplitMix64 + Box-Muller ---- */
typedef struct { uint64_t s; } Rng;
static uint64_t rng_u64(Rng* r) {
    r->s += 0x9E3779B97F4A7C15ULL;
    uint64_t z = r->s;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}
static double rng_uni(Rng* r) { return (double)(rng_u64(r) >> 11) * (1.0 / 9007199254740992.0); }
static double rng_gauss(Rng* r) {
    double u1 = rng_uni(r); if (u1 < 1e-12) u1 = 1e-12;
    double u2 = rng_uni(r);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
}

/* ---- the two functions ---- */
static double bw_of(double f) { return PKT * f; }

/* Integrated-OU yaw: dy(t) = -(y/T)dt + sigma dW.  The two error branches
 * differ in what the receiver does with an old snapshot, not in their input
 * moments, so they are two ways of averaging its mean-square lag. */
#define YAW_TC_D 0.5
static double yaw_var(double t) {
    double T = YAW_TC_D;
    return YAW_SIG * YAW_SIG * (T * t - T * T * (1.0 - exp(-t / T)));
}

/* For a snapshot of age tau, E[|position error|^2] / V^2, with the heading a
 * driftless integrated OU process.  The velocity correlation at lag u is
 * exp(-phi(u)), phi(u) = sigma^2 * (T*u - T^2*(1 - exp(-u/T))).
 *
 *   drag:  the receiver extrapolates, so the error is the velocity change:
 *          tau^2 - 2 * int_0^tau u * exp(-phi(u)) du
 *   hold:  the receiver freezes, so the error is the displacement:
 *          2 * int_0^tau (tau - u) * exp(-phi(u)) du
 */
static double lag_ms(double tau, int freeze) {
    if (tau <= 0.0) return 0.0;
    int n = 48;
    double h = tau / n, s = 0.0;
    for (int k = 0; k < n; k++) {
        double u = (k + 0.5) * h;
        s += (freeze ? tau - u : u) * exp(-yaw_var(u));
    }
    s *= h;
    return freeze ? 2.0 * s : tau * tau - 2.0 * s;
}

/* Average over age ~ U[LAT, LAT + 1/f]. */
static double age_avg(double f, int freeze, int steps) {
    double I = 1.0 / f, lo = LAT, hi = LAT + I, h = (hi - lo) / steps, s = 0.0;
    for (int k = 0; k < steps; k++) s += lag_ms(lo + (k + 0.5) * h, freeze);
    return s / steps;
}

/* In view: the receiver extrapolates the snapshot velocity. */
static double err_drag(double f) {
    if (f <= 0.0) return INFINITY;
    return sqrt(V_SPEED * V_SPEED * age_avg(f, 0, 12) + QSTEP * QSTEP / 6.0);
}

/* Out of view: the receiver freezes the last snapshot. */
static double err_hold(double f) {
    if (f <= 0.0) return INFINITY;
    return sqrt(V_SPEED * V_SPEED * age_avg(f, 1, 12) + QSTEP * QSTEP / 6.0);
}

/* ---- allocation ---- */
static double view_weight(double d) {
    if (d > R_VIS) return W_OUT;
    return 1.0 / (1.0 + d / D0);
}

/* Weighted split of beta * BUDGET, capped at the server tick. */
static void alloc_freq(double beta, const int counts[NCLS], double f[NCLS]) {
    double w[NCLS], F = beta * BUDGET / PKT;
    int fixed[NCLS] = { 0, 0, 0 };
    for (int c = 0; c < NCLS; c++) { w[c] = view_weight(CLS_DIST[c]); f[c] = 0.0; }
    for (;;) {
        double sw = 0.0, rem = F;
        for (int c = 0; c < NCLS; c++) {
            if (counts[c] <= 0) continue;
            if (fixed[c]) rem -= counts[c] * f[c];
            else sw += counts[c] * w[c];
        }
        if (sw <= 0.0) break;
        int hit = 0;
        for (int c = 0; c < NCLS; c++) {
            if (fixed[c] || counts[c] <= 0) continue;
            f[c] = rem * w[c] / sw;
            if (f[c] > 1.0 / TICK) { f[c] = 1.0 / TICK; fixed[c] = 1; hit = 1; }
        }
        if (!hit) break;
    }
    for (int c = 0; c < NCLS; c++) if (f[c] < 0.0) f[c] = 0.0;
}

/* Equal split: every entity in view gets the same rate, the old policy. */
static void equal_freq(double beta, const int counts[NCLS], double f[NCLS]) {
    int n = counts[0] + counts[1] + counts[2];
    double r = n ? beta * BUDGET / PKT / n : 0.0;
    for (int c = 0; c < NCLS; c++) f[c] = r;
}

/* ---- simulation ---- */
typedef struct { double at, ts, x, y, vx, vy; } Pkt;
typedef struct {
    double f, x, y, th, om, next_send;
    Pkt delivered;
    int has;
} Track;

#define MAXP 8192

/* Runs one allocation.  err[] is render RMS per class, f_meas[] the rate
 * observed at send time.  Tracks move at constant speed with an OU yaw rate;
 * snapshots are quantized and delayed by LAT; the receiver extrapolates
 * in-view snapshots and freezes out-of-view ones. */
static void sim_run(const double f_by_cls[NCLS], uint64_t seed,
                    double err[NCLS], double f_meas[NCLS], int ntr_per_cls) {
    Rng rng = { seed };
    int ntr = NCLS * ntr_per_cls;
    Track* tr = (Track*)calloc((size_t)ntr, sizeof(Track));
    Pkt* pend = (Pkt*)calloc(MAXP, sizeof(Pkt));
    int* pidx = (int*)calloc(MAXP, sizeof(int));
    double* last_t = (double*)calloc((size_t)ntr, sizeof(double));
    if (!tr || !pend || !pidx || !last_t) {
        free(tr); free(pend); free(pidx); free(last_t);
        for (int c = 0; c < NCLS; c++) { err[c] = NAN; f_meas[c] = 0.0; }
        return;
    }

    for (int c = 0; c < NCLS; c++) {
        for (int k = 0; k < ntr_per_cls; k++) {
            Track* e = &tr[c * ntr_per_cls + k];
            e->f = f_by_cls[c];
            e->th = rng_uni(&rng) * 2.0 * PI;
            e->om = rng_gauss(&rng) * YAW_SIG;
            e->next_send = rng_uni(&rng) / e->f;
            e->has = 0;
        }
    }
    for (int i = 0; i < ntr; i++) last_t[i] = -1.0;

    double ss[NCLS] = { 0 }, isum[NCLS] = { 0 };
    int nerr[NCLS] = { 0 }, nint[NCLS] = { 0 };
    int np = 0;

    const double dt = 1.0 / 240.0;
    double t = 0.0, next_tick = TICK, next_render = RENDER + 0.011;
    double decay = exp(-dt / YAW_TC_D);
    double sstep = sqrt(1.0 - decay * decay);

    while (t <= SIM_TIME) {
        for (int i = 0; i < ntr; i++) {
            Track* e = &tr[i];
            e->om = e->om * decay + rng_gauss(&rng) * sstep;
            e->th += e->om * dt;
            e->x += V_SPEED * cos(e->th) * dt;
            e->y += V_SPEED * sin(e->th) * dt;
        }
        if (t + 1e-12 >= next_tick) {
            for (int i = 0; i < ntr; i++) {
                Track* e = &tr[i];
                while (e->next_send <= t + 1e-12) {
                    double ts = next_tick;
                    if (np < MAXP) {
                        Pkt* p = &pend[np];
                        p->at = ts + LAT;
                        p->ts = ts;
                        p->x = round(e->x / QSTEP) * QSTEP;
                        p->y = round(e->y / QSTEP) * QSTEP;
                        p->vx = V_SPEED * cos(e->th);
                        p->vy = V_SPEED * sin(e->th);
                        pidx[np] = i;
                        np++;
                    }
                    int c = i / ntr_per_cls;
                    if (last_t[i] >= 0.0 && ts > WARM) {
                        isum[c] += ts - last_t[i];
                        nint[c]++;
                    }
                    last_t[i] = ts;
                    e->next_send += 1.0 / e->f;
                }
            }
            next_tick += TICK;
        }
        int w = 0;
        for (int k = 0; k < np; k++) {
            if (pend[k].at <= t) {
                Track* e = &tr[pidx[k]];
                e->delivered = pend[k];
                e->has = 1;
            } else {
                pend[w] = pend[k]; pidx[w] = pidx[k]; w++;
            }
        }
        np = w;
        if (t >= next_render) {
            if (t >= WARM) {
                for (int i = 0; i < ntr; i++) {
                    Track* e = &tr[i];
                    if (!e->has) continue;
                    int c = i / ntr_per_cls;
                    double age = t - e->delivered.ts;
                    double px = e->delivered.x, py = e->delivered.y;
                    if (c != CLS_OUT) {
                        px += e->delivered.vx * age;
                        py += e->delivered.vy * age;
                    }
                    ss[c] += (px - e->x) * (px - e->x) + (py - e->y) * (py - e->y);
                    nerr[c]++;
                }
            }
            next_render += RENDER;
        }
        t += dt;
    }
    for (int c = 0; c < NCLS; c++) {
        err[c] = nerr[c] ? sqrt(ss[c] / nerr[c]) : NAN;
        f_meas[c] = nint[c] ? (double)nint[c] / isum[c] : 0.0;
    }
    free(tr); free(pend); free(pidx); free(last_t);
}

/* ---- outputs ---- */
static void emit_curves(void) {
    printf("f,bw_bytes_per_s,err_drag_m,err_hold_m\n");
    for (double f = 0.5; f <= 40.0001; f += 0.5)
        printf("%.2f,%.3f,%.5f,%.5f\n", f, bw_of(f), err_drag(f), err_hold(f));
}

static void emit_cal(void) {
    static const double grid[] = { 0.5, 0.8, 1.2, 1.6, 2, 3, 4, 6, 8, 12, 16, 24, 30, 40 };
    printf("f,err_near,err_mid,err_drag_pred\n");
    for (size_t j = 0; j < sizeof(grid) / sizeof(grid[0]); j++) {
        double f = grid[j], e[NCLS], m[NCLS];
        double fixed[NCLS] = { f, f, f };
        sim_run(fixed, 0xC0DE + (uint64_t)j * 13u, e, m, 48);
        printf("%.2f,%.5f,%.5f,%.5f\n", f, e[0], e[1], err_drag(f));
    }
}

static void emit_exp(void) {
    printf("beta,tier,near_n,mid_n,out_n,"
           "f_near,f_mid,f_out,err_near,err_mid,err_out,"
           "pred_drag_near,pred_drag_mid,pred_drag_out,"
           "pred_hold_near,pred_hold_mid,pred_hold_out\n");
    for (int b = 0; b < 3; b++) {
        for (int ti = 0; ti < 3; ti++) {
            double f[NCLS], e[NCLS], m[NCLS];
            alloc_freq(BETA[b], TIERS[ti], f);
            uint64_t seed = 0xB000 + (uint64_t)(BETA[b] * 1000) + 17u * (uint64_t)ti;
            sim_run(f, seed, e, m, TRACKS);
            printf("%.2f,%s,%d,%d,%d,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,"
                   "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                   BETA[b], TIER_NAME[ti],
                   TIERS[ti][0], TIERS[ti][1], TIERS[ti][2],
                   m[0], m[1], m[2], e[0], e[1], e[2],
                   err_drag(m[0]), err_drag(m[1]), err_drag(m[2]),
                   err_hold(m[0]), err_hold(m[1]), err_hold(m[2]));
        }
    }
}

static void emit_equal(void) {
    printf("tier,near_n,mid_n,out_n,f_near,f_mid,f_out,err_near,err_mid,err_out\n");
    for (int ti = 0; ti < 3; ti++) {
        double f[NCLS], e[NCLS], m[NCLS];
        equal_freq(BETA[SEL_BETA], TIERS[ti], f);
        uint64_t seed = 0xE000 + 17u * (uint64_t)ti;
        sim_run(f, seed, e, m, TRACKS);
        printf("%s,%d,%d,%d,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f\n",
               TIER_NAME[ti], TIERS[ti][0], TIERS[ti][1], TIERS[ti][2],
               m[0], m[1], m[2], e[0], e[1], e[2]);
    }
}

/* ---- self-test ---- */
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(int argc, char** argv) {
    if (argc > 1) {
        if (!strcmp(argv[1], "curves")) { emit_curves(); return 0; }
        if (!strcmp(argv[1], "cal"))    { emit_cal(); return 0; }
        if (!strcmp(argv[1], "exp"))    { emit_exp(); return 0; }
        if (!strcmp(argv[1], "equal"))  { emit_equal(); return 0; }
    }

    check("weight_near_gt_mid", view_weight(4.0) > view_weight(18.0));
    check("weight_out_small", view_weight(32.0) == W_OUT);
    check("bw_monotone", bw_of(10.0) > bw_of(5.0));
    check("err_monotone", err_drag(10.0) < err_drag(5.0));
    check("err_hold_monotone", err_hold(10.0) < err_hold(5.0));
    check("hold_worse_than_drag", err_hold(4.0) > err_drag(4.0));
    check("err_quant_floor", err_drag(1e6) < QSTEP / 1.5);

    double f[NCLS];
    int c0[NCLS] = { 3, 4, 2 }, c1[NCLS] = { 10, 16, 22 };
    alloc_freq(0.7, c0, f);
    check("alloc_order", f[0] > f[1] && f[1] > f[2]);
    check("alloc_band",
          fabs(3 * f[0] + 4 * f[1] + 2 * f[2] - 0.7 * BUDGET / PKT) < 1e-6);
    alloc_freq(0.7, c1, f);
    check("alloc_band_dense",
          fabs(10 * f[0] + 16 * f[1] + 22 * f[2] - 0.7 * BUDGET / PKT) < 1e-6);
    check("alloc_dense_slower", f[0] < 8.0);

    /* a huge budget saturates every class at the tick rate and leaves the
     * remainder unspent instead of overrunning the link */
    alloc_freq(100.0, c1, f);
    check("alloc_capped", f[0] == 1.0 / TICK && f[1] == 1.0 / TICK && f[2] == 1.0 / TICK);

    double e[NCLS], m[NCLS];
    double fixed[NCLS] = { 8.0, 8.0, 8.0 };
    sim_run(fixed, 12345, e, m, TRACKS);
    check("sim_rate_kept", fabs(m[0] - 8.0) < 0.1 && fabs(m[2] - 8.0) < 0.1);

    /* the model section, not just the model itself */
    check("sim_near_matches_drag",
          e[0] > 0.85 * err_drag(8.0) && e[0] < 1.15 * err_drag(8.0));
    check("sim_out_matches_hold",
          e[2] > 0.85 * err_hold(8.0) && e[2] < 1.15 * err_hold(8.0));
    check("sim_out_misses_drag", e[2] > 2.0 * err_drag(8.0));

    double e2[NCLS], m2[NCLS];
    sim_run(fixed, 12345, e2, m2, TRACKS);
    check("sim_deterministic", fabs(e[0] - e2[0]) < 1e-12 && fabs(e[2] - e2[2]) < 1e-12);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
