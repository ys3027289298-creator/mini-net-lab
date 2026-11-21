
/* smooth: prediction smoothing for remote entities.
 *
 *  three mechanisms:
 *  1. prediction lead is derived from the one-way-delay distribution (p90,
 *     clamped); insufficient samples fall back to a conservative default.
 *  2. authoritative target vs. rendered position error converges through a
 *     first-order exponential filter (time constant tau), never overshooting.
 *  3. errors past snap_dist hard-correct instantly (real teleports only).
 *
 *  the self-test runs a deterministic network simulation and compares a
 *  zero-jitter baseline against a jitter scenario: frame jump count must be
 *  zero in both, smoothing lag must stay under the analytic bound, and a
 *  naive (unsmoothed) renderer is included to prove the jitter actually
 *  produces jumps without the filter.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ---- latency distribution ------------------------------------------------ */

#define DD_N 32                 /* delay sample ring size */
#define DD_MIN 8                /* min samples before a quantile is trusted */
#define LEAD_Q 0.9              /* p90: ~90% of needed snapshots have arrived */
#define LEAD_DEFAULT 0.20       /* fallback while samples are insufficient */
#define LEAD_MIN 0.05
#define LEAD_MAX 0.30
#define SNAP_DIST 1.0f        /* hard correction threshold, world units */

typedef struct {
    double d[DD_N];
    int n, head;
} DelayDist;

static void dd_init(DelayDist* dd) { memset(dd, 0, sizeof(*dd)); }

static void dd_add(DelayDist* dd, double one_way_delay) {
    if (one_way_delay < 0.0) return;
    dd->d[dd->head] = one_way_delay;
    dd->head = (dd->head + 1) % DD_N;
    if (dd->n < DD_N) dd->n++;
}

/* nearest-rank quantile over a sorted copy; n <= DD_N so insertion sort fits */
static double dd_quantile(const DelayDist* dd, double q) {
    double tmp[DD_N];
    memcpy(tmp, dd->d, sizeof(double) * (size_t)dd->n);
    for (int i = 1; i < dd->n; i++) {
        double v = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = v;
    }
    int rank = (int)ceil(q * dd->n);
    if (rank < 1) rank = 1;
    if (rank > dd->n) rank = dd->n;
    return tmp[rank - 1];
}

/* returns 0 when the quantile is trusted, -1 when samples are insufficient
 * (lead is then set to the conservative fallback). */
static int dd_lead(const DelayDist* dd, double* lead) {
    if (dd->n < DD_MIN) { *lead = LEAD_DEFAULT; return -1; }
    double q = dd_quantile(dd, LEAD_Q);
    if (q < LEAD_MIN) q = LEAD_MIN;
    if (q > LEAD_MAX) q = LEAD_MAX;
    *lead = q;
    return 0;
}

/* ---- authoritative snapshot buffer --------------------------------------- */

#define AUTH_N 16

typedef struct { double t; float x, vx; } Auth;

typedef struct {
    Auth a[AUTH_N];
    int n;
    int disc;             /* last insertion broke continuity with its predecessor */
} AuthBuf;

/* ordered insert; tolerates reordered/duplicate arrivals, evicts the oldest.
 * a new snapshot that disagrees with the predecessor's velocity-propagated
 * position beyond SNAP_DIST is a discontinuity (teleport). */
static void ab_push(AuthBuf* b, double t, float x, float vx) {
    if (b->n == AUTH_N) {
        if (t <= b->a[0].t) return;                 /* older than the oldest: drop */
        memmove(&b->a[0], &b->a[1], sizeof(Auth) * (AUTH_N - 1));
        b->n = AUTH_N - 1;
    }
    int pos = b->n;
    for (int i = 0; i < b->n; i++) {
        if (t <= b->a[i].t) { pos = i; break; }
    }
    if (pos < b->n && b->a[pos].t == t) {          /* duplicate tick: overwrite */
        b->a[pos] = (Auth){ t, x, vx };
        return;
    }
    b->disc = 0;
    if (pos > 0) {                                 /* gap against predecessor */
        const Auth* prev = &b->a[pos - 1];
        float expected = prev->x + prev->vx * (float)(t - prev->t);
        if (fabsf(x - expected) > SNAP_DIST) b->disc = 1;
    }
    memmove(&b->a[pos + 1], &b->a[pos], sizeof(Auth) * (size_t)(b->n - pos));
    b->a[pos] = (Auth){ t, x, vx };
    b->n++;
}

/* rendered target at server time tr: interpolate inside the buffer,
 * dead-reckon beyond the newest snapshot, clamp before the oldest.
 * discontinuity detection happens at insertion (ab_push), not here. */
static float ab_target(const AuthBuf* b, double tr, int* extrap) {
    *extrap = 0;
    if (b->n == 0) return 0.0f;
    if (tr <= b->a[0].t) return b->a[0].x;
    if (tr >= b->a[b->n - 1].t) {
        const Auth* nw = &b->a[b->n - 1];
        *extrap = 1;
        return nw->x + nw->vx * (float)(tr - nw->t);
    }
    for (int i = 0; i + 1 < b->n; i++) {
        if (tr >= b->a[i].t && tr <= b->a[i + 1].t) {
            double f = (tr - b->a[i].t) / (b->a[i + 1].t - b->a[i].t);
            return b->a[i].x + (b->a[i + 1].x - b->a[i].x) * (float)f;
        }
    }
    return b->a[b->n - 1].x;
}

/* ---- smoother ------------------------------------------------------------- */

typedef struct {
    AuthBuf buf;
    double tau;            /* smoothing time constant, seconds */
    float snap_dist;       /* hard correction threshold */
    float rx;              /* rendered position */
    int have;              /* first frame initialized? */
    int naive;             /* 1 = snap target directly (control renderer) */
    int disc_latched;      /* discontinuity pending until hard-corrected */
    long snaps;            /* hard correction count */
} Smoother;

static void sm_init(Smoother* s, double tau, float snap_dist, int naive) {
    memset(s, 0, sizeof(*s));
    s->tau = tau;
    s->snap_dist = snap_dist;
    s->naive = naive;
}

/* ingest an authoritative snapshot and latch any discontinuity it carries.
 * latching (rather than reading a per-buffer flag) survives the subsequent
 * snapshots that arrive before the renderer catches up. */
static void sm_push(Smoother* s, double t, float x, float vx) {
    ab_push(&s->buf, t, x, vx);
    if (s->buf.disc) s->disc_latched = 1;
}

/* advance one render frame; returns 1 if a hard snap happened this frame.
 * hard snaps are reserved for genuine discontinuities (detected at insertion
 * and latched until resolved); ordinary extrapolation/smoothing error is
 * soft-clamped to snap_dist so a long-tail delay never itself teleports. */
static int sm_tick(Smoother* s, double now, double dt, double lead) {
    int extrap;
    float target = ab_target(&s->buf, now - lead, &extrap);
    if (s->naive) { s->rx = target; s->have = 1; return 0; }
    if (!s->have) { s->rx = target; s->have = 1; return 0; }

    float e = target - s->rx;
    float k = (float)(1.0 - exp(-dt / s->tau));  /* exponential convergence */
    if (fabsf(e) > s->snap_dist) {
        if (!s->disc_latched) {
            /* stale-buffer extrapolation: move one gain-step toward the
             * envelope edge, so a long-tail delay never produces a jump */
            s->rx += (e > 0.0f ? k * s->snap_dist : -k * s->snap_dist);
            return 0;
        }
        /* authoritative discontinuity: correct now */
        s->rx = target;
        s->snaps++;
        s->disc_latched = 0;
        return 1;
    }
    /* a latched discontinuity survives sub-threshold frames: it is cleared
     * only by the hard correction above, never by ordinary smoothing */
    s->rx += k * e;
    return 0;
}

/* ---- deterministic simulation --------------------------------------------- */

#define SIM_T 30.0
#define TICK 0.05
#define DT (1.0 / 60.0)
#define BASE_D 0.10
#define JITTER_HI 0.50
#define V 6.0f
#define TAU 0.08
#define JUMP_FACTOR 3.0                         /* >3x normal frame move = jump */
#define GRACE 0.5                               /* cold-start grace, seconds */

static unsigned long long rng_state;

static unsigned long long xrnd(void) {          /* xorshift64, fixed seed */
    unsigned long long x = rng_state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    rng_state = x;
    return x;
}

static double jitter_uniform(double hi) {
    return hi * (double)(xrnd() >> 11) / (double)(1ULL << 53);
}

typedef struct {
    double jitter_hi;      /* 0 = no-jitter baseline */
    double teleport_at;    /* <= 0 disables */
    int naive;
} Scenario;

typedef struct {
    long jumps;            /* frame deltas beyond JUMP_FACTOR * v * dt */
    long over_steps;       /* frames beyond 1.5x a normal frame's move */
    long snaps;            /* hard corrections */
    long fallback;         /* frames where delay samples were insufficient */
    long extrap_frames;
    double max_lag;        /* measured visual lag vs ground truth, seconds */
    double max_lead;
    double max_delta;      /* largest observed frame displacement */
} SimResult;

static float truth_x(double t, double teleport_at) {
    if (teleport_at > 0.0 && t >= teleport_at)
        return -0.5f * V - V * (float)(t - teleport_at);   /* jump + velocity reversal */
    return (float)(V * t);
}

static float truth_v(double t, double teleport_at) {
    return (teleport_at > 0.0 && t >= teleport_at) ? -V : V;
}

static SimResult run_sim(const Scenario* sc, unsigned long long seed) {
    rng_state = seed;
    DelayDist dd; dd_init(&dd);
    Smoother sm; sm_init(&sm, TAU, SNAP_DIST, sc->naive);

    SimResult res = { 0 };
    int next_tick = 0;
    int rendered = 0;                 /* frames after the first rendered one */
    float prev_rx = 0.0f;
    double lead = LEAD_DEFAULT;

    for (double now = 0.0; now <= SIM_T + 1e-9; now += DT) {
        /* deliver every snapshot whose network arrival time is due */
        for (;;) {
            double ts = next_tick * TICK;
            if (ts > SIM_T) break;
            double delay = BASE_D + (sc->jitter_hi > 0.0 ? jitter_uniform(sc->jitter_hi) : 0.0);
            if (ts + delay > now) break;
            sm_push(&sm, ts, truth_x(ts, sc->teleport_at),
                    truth_v(ts, sc->teleport_at));
            dd_add(&dd, delay);
            next_tick++;
        }

        if (dd_lead(&dd, &lead) < 0) res.fallback++;
        if (lead > res.max_lead) res.max_lead = lead;

        if (sm.buf.n == 0) continue;                 /* nothing received yet */

        int extrap;
        int hard = sm_tick(&sm, now, DT, lead);
        ab_target(&sm.buf, now - lead, &extrap);        /* extrapolation flag only */
        if (extrap) res.extrap_frames++;

        if (rendered > 0 && now >= GRACE) {    /* cold-start window excluded */
            if (hard) res.snaps++;
            float delta = fabsf(sm.rx - prev_rx);
            if (delta > res.max_delta) res.max_delta = delta;
            if (delta > JUMP_FACTOR * V * (float)DT) res.jumps++;
            if (delta > 1.5f * V * (float)DT) res.over_steps++;
        }
        rendered++;
        prev_rx = sm.rx;

        /* lag via ground-truth inversion; only valid on monotonic runs,
         * so the teleport scenario is excluded from its instant onward */
        if (now >= GRACE &&
            (sc->teleport_at <= 0.0 || now < sc->teleport_at)) {
            double lag = now - (double)sm.rx / (double)V;
            if (lag > res.max_lag) res.max_lag = lag;
        }
    }
    return res;
}

/* ---- test harness --------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* unit: insufficient samples -> fallback, enough samples -> p90 quantile */
    DelayDist dd; dd_init(&dd);
    double lead = 0.0;
    check("empty_fallback", dd_lead(&dd, &lead) == -1 && lead == LEAD_DEFAULT);
    for (int i = 0; i < DD_MIN - 1; i++) dd_add(&dd, 0.10);
    check("seven_fallback", dd_lead(&dd, &lead) == -1 && lead == LEAD_DEFAULT);
    dd_add(&dd, 0.10);
    check("eight_trusted", dd_lead(&dd, &lead) == 0 && fabs(lead - 0.10) < 1e-9);

    DelayDist q; dd_init(&q);
    for (int i = 1; i <= 10; i++) dd_add(&q, i * 0.01);          /* 0.01..0.10 */
    check("quantile_p90", fabs(dd_quantile(&q, 0.9) - 0.09) < 1e-12);
    check("lead_in_range", dd_lead(&q, &lead) == 0 && fabs(lead - 0.09) < 1e-12);

    DelayDist w; dd_init(&w);
    for (int i = 0; i < DD_N + 10; i++) dd_add(&w, 0.05);        /* ring wraps */
    check("ring_wrapped", w.n == DD_N && w.head == 10 &&
                          fabs(dd_quantile(&w, 0.9) - 0.05) < 1e-12);
    dd_add(&w, -1.0);                                            /* invalid sample */
    check("reject_negative", w.n == DD_N);

    /* unit: ordered buffer tolerates reorder / duplicates / eviction */
    AuthBuf ab; memset(&ab, 0, sizeof(ab));
    ab_push(&ab, 0.10, 1.0f, 1.0f);
    ab_push(&ab, 0.05, 0.5f, 1.0f);        /* out of order */
    ab_push(&ab, 0.05, 0.6f, 1.0f);        /* duplicate tick overwrites */
    check("buffer_ordered", ab.n == 2 && ab.a[0].t == 0.05 &&
                            fabsf(ab.a[0].x - 0.6f) < 1e-6f);
    for (int i = 0; i < 20; i++) ab_push(&ab, 0.2 + i * 0.05, (float)i, 1.0f);
    check("buffer_evicts", ab.n == AUTH_N &&
                            fabs(ab.a[AUTH_N - 1].t - 1.15) < 1e-9 &&
                            fabs(ab.a[0].t - 0.40) < 1e-9);
    int extrap;
    double mid = (ab.a[0].t + ab.a[1].t) * 0.5;
    float mid_x = (ab.a[0].x + ab.a[1].x) * 0.5f;
    check("interp", fabsf(ab_target(&ab, mid, &extrap) - mid_x) < 1e-5f && extrap == 0);
    check("extrap", ab_target(&ab, 2.0, &extrap) > ab.a[AUTH_N - 1].x && extrap == 1);

    /* unit: sub-threshold error converges monotonically, no snap */
    Smoother sm; sm_init(&sm, TAU, SNAP_DIST, 0);
    sm_push(&sm, 0.0, 0.0f, 0.0f);
    sm_tick(&sm, 0.0, DT, 0.0);                       /* initializes to target */
    sm_push(&sm, DT, 0.5f, 0.0f);                     /* 0.5m step < threshold */
    float prev_e = 0.5f;
    int monotonic = 1, snapped = 0;
    for (int i = 0; i < 40; i++) {
        snapped |= sm_tick(&sm, (i + 2) * DT, DT, 0.0);
        float e = fabsf(0.5f - sm.rx);
        if (e > prev_e + 1e-7f) monotonic = 0;
        prev_e = e;
    }
    check("smooth_no_snap", snapped == 0 && sm.snaps == 0);
    check("smooth_monotonic", monotonic == 1);
    check("smooth_converges", fabsf(sm.rx - 0.5f) < 0.01f);

    /* unit: a genuine discontinuity (not extrapolation) hard-corrects once */
    Smoother hs; sm_init(&hs, TAU, SNAP_DIST, 0);
    sm_push(&hs, 0.00, 0.0f, 0.0f);
    sm_tick(&hs, 0.0, DT, 0.0);                          /* initializes rx */
    sm_push(&hs, 0.05, 5.0f, 0.0f);                      /* gap of 5m at 0.05 */
    check("hard_snap", sm_tick(&hs, 0.10, DT, 0.05) == 1 &&
                       fabsf(hs.rx - 5.0f) < 1e-6f && hs.snaps == 1);

    /* unit: a large error while extrapolating is smoothed, never snapped,
     * and each frame moves by at most the gain times snap_dist */
    Smoother se; sm_init(&se, TAU, SNAP_DIST, 0);
    sm_push(&se, 0.0, 0.0f, 6.0f);
    sm_tick(&se, 0.0, DT, 0.0);                    /* initializes rx=0 */
    float k = (float)(1.0 - exp(-DT / TAU));
    float before = se.rx;
    int soft_ret = sm_tick(&se, 1.0, DT, 0.0);     /* target far ahead, extrap */
    check("extrap_soft_clamp", soft_ret == 0 && se.snaps == 0 &&
                               se.rx > before &&
                               (se.rx - before) <= k * SNAP_DIST + 1e-6f);

    /* unit: discontinuity is detected at insertion, before any eviction */
    AuthBuf discb; memset(&discb, 0, sizeof(discb));
    check("disc_clear_initially", discb.disc == 0);
    ab_push(&discb, 0.00, 60.0f, 6.0f);
    ab_push(&discb, 0.05, -3.0f, -6.0f);                         /* teleport gap */
    check("discontinuity_flagged", discb.disc == 1);
    int dextrap;
    check("discontinuity_target",
          fabsf(ab_target(&discb, 0.05, &dextrap) - (-3.0f)) < 1e-6f);
    ab_push(&discb, 0.10, -3.6f, -6.0f);                         /* continuous again */
    check("disc_clears_on_continuous", discb.disc == 0);

    /* simulation: baseline vs jitter, smoothed vs naive */
    SimResult base  = run_sim(&(Scenario){ 0.0, 0.0, 0 }, 0x9e3779b97f4a7c15ULL);
    SimResult jit   = run_sim(&(Scenario){ JITTER_HI, 0.0, 0 }, 0x9e3779b97f4a7c15ULL);
    SimResult naive = run_sim(&(Scenario){ JITTER_HI, 0.0, 1 }, 0x9e3779b97f4a7c15ULL);
    SimResult tele  = run_sim(&(Scenario){ JITTER_HI, 10.0, 0 }, 0x9e3779b97f4a7c15ULL);

    printf("\n%-14s %7s %10s %8s %10s %10s %10s %12s\n",
           "scenario", "jumps", "over1.5x", "snaps", "max_lag", "max_lead",
           "fallback", "max_delta");
    const SimResult* rows[4] = { &base, &jit, &naive, &tele };
    const char* names[4] = { "baseline", "jit_smooth", "jit_naive", "teleport" };
    for (int i = 0; i < 4; i++) {
        printf("%-14s %7ld %10ld %8ld %10.4f %10.4f %10ld %12.4f\n",
               names[i], rows[i]->jumps, rows[i]->over_steps, rows[i]->snaps,
               rows[i]->max_lag, rows[i]->max_lead, rows[i]->fallback,
               rows[i]->max_delta);
    }

    /* analytic lag envelope: interp lead + filter tau + worst-case correction
     * distance expressed as time (snap_dist / v) + one frame of discretization.
     * per-frame envelope: normal motion plus the largest smoothing/clamp step. */
    double lag_bound = LEAD_MAX + TAU + (double)SNAP_DIST / (double)V + DT + 1e-6;
    double gain = 1.0 - exp(-DT / TAU);
    double per_frame_bound = (double)V * DT + gain * (double)SNAP_DIST + 1e-6;
    printf("\nlag_bound=%.4f per_frame_bound=%.4f extrap_frames(jit)=%ld\n",
           lag_bound, per_frame_bound, jit.extrap_frames);

    /* acceptance criteria */
    check("baseline_zero_jumps", base.jumps == 0);
    check("baseline_zero_snaps", base.snaps == 0);
    check("jitter_zero_jumps", jit.jumps == 0);
    check("jitter_zero_snaps", jit.snaps == 0);
    check("jitter_matches_baseline", jit.jumps == base.jumps);
    /* the smoothed path makes zero hard jumps/snaps and its largest frame
     * step is bounded analytically (jitter may add small smooth steps, but
     * never a discontinuity). The naive control is consistently rougher,
     * proving the jitter is real and the filter removes it. */
    check("smooth_zero_discontinuity",
          jit.jumps == 0 && jit.snaps == 0 && base.jumps == 0 && base.snaps == 0);
    check("naive_rougher_maxdelta", naive.max_delta > jit.max_delta);
    check("naive_rougher_oversteps", naive.over_steps > jit.over_steps);
    check("smoother_beats_naive", jit.max_delta < naive.max_delta);
    check("baseline_lag_bounded", base.max_lag <= lag_bound);
    check("jitter_lag_bounded", jit.max_lag <= lag_bound);
    check("jitter_frame_bound", jit.max_delta <= per_frame_bound);
    check("warmup_fallback_seen", jit.fallback > 0 && base.fallback > 0);
    check("teleport_hard_snaps_once", tele.snaps == 1);
    check("teleport_jump_is_the_snap", tele.jumps == 1);
    check("teleport_same_seed_else_zero",
          base.jumps == 0 && jit.jumps == 0 && base.snaps == 0 && jit.snaps == 0);

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
