
/* spectator: smoothing window over sparse snapshots for the spectator view.
 *
 * The local machine renders a smooth motion signal; the spectator only gets
 * snapshots every Ts seconds and must smooth them with a window of length W.
 * Window sizing has to satisfy both:
 *   1) W >= Ts          window covers at least one snapshot gap (anti-stutter)
 *   2) W/2 <= Lmax      symmetric window buffers W/2 of delay (lag budget)
 * usable interval: Ts <= W <= 2*Lmax.
 *
 * Per clip the dominant frequency is estimated from snapshot zero crossings,
 * then W = max(Ts, kappa / fest), i.e. a quarter of the estimated period but
 * never shorter than one snapshot gap. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define DT      0.005          /* local render grid: 200 Hz */
#define CLIP    8.0            /* clip length in seconds */
#define T_LO    1.0            /* metric interior: skip window fill edges */
#define T_HI    7.0
#define TS      0.10           /* snapshot interval: 10 Hz */
#define LMAX    0.10           /* noticeable-lag budget: 100 ms */
#define KAPPA   0.25           /* window = quarter of estimated period */
#define NGROUP  3
#define PI      3.14159265358979323846

static double wave(double f, double t) { return sin(2.0 * PI * f * t); }

/* dominant frequency from upward zero crossings of the sampled signal */
static double fest_zero_cross(const double* y, int n, double ts) {
    int cross = 0;
    for (int k = 1; k < n; k++)
        if (y[k - 1] < 0.0 && y[k] >= 0.0) cross++;
    return (double)cross / ((double)(n - 1) * ts);
}

/* boxcar reconstruction at local time t: mean of snapshots inside the
 * symmetric window; hold the nearest snapshot if the window is empty */
static double reconstruct(const double* snap, int n, double ts, double w, double t) {
    double sum = 0.0; int cnt = 0;
    for (int k = 0; k < n; k++) {
        double tk = k * ts;
        if (fabs(tk - t) <= w * 0.5 + 1e-12) { sum += snap[k]; cnt++; }
    }
    if (cnt > 0) return sum / (double)cnt;
    int near = 0;
    for (int k = 1; k < n; k++)
        if (fabs(k * ts - t) < fabs(near * ts - t)) near = k;
    return snap[near];
}

/* live-framed delay: at wall time T the spectator can only center its window
 * at c = T - W/2. Feed a continuous unit step at t=0 through the boxcar:
 * the output is the fraction of [c-W/2, c+W/2] lying past the step. It
 * crosses 0.5 exactly when c reaches 0, i.e. at wall time T = W/2. */
static double measure_lag(double w) {
    for (double T = 0.0; T <= 1.0; T += DT) {
        double c = T - w * 0.5;
        double lo = c - w * 0.5, hi = c + w * 0.5;
        double v;
        if (hi <= 0.0) v = 0.0;
        else if (lo >= 0.0) v = 1.0;
        else v = hi / w;                    /* clamped by window bounds */
        if (v >= 0.5) return T;
    }
    return -1.0;
}

typedef struct {
    const char* name;
    double f;                 /* true dominant frequency */
    double fest;              /* estimated from snapshots */
    double w;                 /* chosen window length */
    double jitter;            /* lag-compensated track error RMS (fraction of amp) */
    double lag;               /* measured lag vs local */
    int inside;               /* inside usable interval */
} Group;

static Group run_group(const char* name, double f, double ts) {
    Group g;
    g.name = name; g.f = f;
    int n = (int)round(CLIP / ts) + 1;
    double snap[256];
    for (int k = 0; k < n; k++) snap[k] = wave(f, k * ts);
    g.fest = fest_zero_cross(snap, n, ts);

    double w_sig = KAPPA / g.fest;
    g.w = w_sig > ts ? w_sig : ts;           /* lower clamp = gap coverage */

    double acc = 0.0; int cnt = 0;
    double lag0 = g.w * 0.5;               /* known constant lag of the boxcar */
    for (double t = T_LO; t <= T_HI; t += DT) {
        double out = reconstruct(snap, n, ts, g.w, t);
        double ref = wave(f, t - lag0);   /* local truth shifted by the lag */
        double e = out - ref;
        acc += e * e; cnt++;
    }
    g.jitter = sqrt(acc / (double)cnt);   /* RMS track error, fraction of amp */
    g.lag = measure_lag(g.w);
    /* constraint 1 on W directly, constraint 2 on the measured lag */
    g.inside = (g.w >= ts - 1e-12) && (g.lag <= LMAX + 1e-9);
    return g;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    double w_min = TS;
    double w_max = 2.0 * LMAX;
    printf("constraint_cover=W>=Ts\n");
    printf("constraint_lag=W/2<=Lmax_equiv_W<=2*Lmax\n");
    printf("usable_interval=%.3f<=W<=%.3f\n", w_min, w_max);
    printf("Ts=%.3f Lmax=%.3f f_nyquist=%.1f\n", TS, LMAX, 1.0 / (2.0 * TS));
    printf("window_rule=W=max(Ts,kappa/fest) kappa=%.2f\n", KAPPA);

    const double f[NGROUP] = { 1.5, 4.0, 9.0 };   /* turn / swing / combo */
    const char* nm[NGROUP] = { "turn", "swing", "combo" };
    Group g[NGROUP];
    for (int i = 0; i < NGROUP; i++) {
        g[i] = run_group(nm[i], f[i], TS);
        printf("%s_fest=%.3f %s_W=%.3f %s_jitter=%.3f %s_lag=%.3f %s_inside=%d\n",
               g[i].name, g[i].fest, g[i].name, g[i].w,
               g[i].name, g[i].jitter, g[i].name, g[i].lag,
               g[i].name, g[i].inside);
    }

    /* the nine recorded numbers, grouped three per clip */
    printf("nine_numbers=W[%.3f %.3f %.3f] jitter[%.3f %.3f %.3f] lag[%.3f %.3f %.3f]\n",
           g[0].w, g[1].w, g[2].w,
           g[0].jitter, g[1].jitter, g[2].jitter,
           g[0].lag, g[1].lag, g[2].lag);

    check("interval_exists", TS <= 2.0 * LMAX);
    check("turn_in_interval", g[0].inside);
    check("swing_in_interval", g[1].inside);
    check("fest_turn_ok", fabs(g[0].fest - 1.5) < 0.3);
    check("fest_swing_ok", fabs(g[1].fest - 4.0) < 0.3);

    /* conditional diagnosis: combo lag over the upper bound means its
     * dominant frequency was under-read and the window came out long */
    int combo_violates = g[2].lag > LMAX + 1e-9;
    check("combo_lag_violates", combo_violates);
    if (combo_violates) {
        printf("diagnosis=combo_fest_underread_window_too_long\n");
        printf("%-8s %8s %8s %8s %10s\n",
               "clip", "fest_hz", "Ts_s", "fnyq_hz", "fest/fnyq");
        for (int i = 0; i < NGROUP; i++)
            printf("%-8s %8.3f %8.3f %10.2f %10.2f\n",
                   g[i].name, g[i].fest, TS, 1.0 / (2.0 * TS),
                   g[i].fest / (1.0 / (2.0 * TS)));
        check("combo_fest_aliased_low", fabs(g[2].fest - 1.0) < 0.3);
    }

    /* remediation: snapshot faster than twice the combo frequency,
     * the alias disappears, fest recovers and the window fits again */
    Group fix = run_group("combo_fix", 9.0, 0.05);
    printf("combo_fix_fest=%.3f combo_fix_W=%.3f combo_fix_lag=%.3f\n",
           fix.fest, fix.w, fix.lag);
    check("fix_fest_recovers", fabs(fix.fest - 9.0) < 0.5);
    check("fix_lag_in_budget", fix.lag <= LMAX + 1e-9);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
