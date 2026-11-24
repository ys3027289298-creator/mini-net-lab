/* dualtime: two parallel timelines for client prediction.
 *
 * The local timeline integrates the player's input immediately. The server
 * timeline integrates the same input, but only after it has crossed the
 * uplink, so at server tick T it lags the local timeline by the number of
 * ticks currently in flight. Every ACK_EVERY ticks the server publishes its
 * state; the client keeps it as the confirmed timeline.
 *
 * When a confirmation for tick T lands, the client measures
 *     e = |local[T] - confirmed[T]|
 * and converges: it moves local[T] toward confirmed[T] by a gain chosen from
 * e (soft / mid / hard), then replays the local inputs from T to the present
 * so the correction reaches the state the player is about to see. Small
 * model lag is absorbed gradually; a genuine jump is corrected at once.
 * Velocity is deliberately not corrected: input response stays local and only
 * the position gap is reconciled.
 *
 * The self-test runs three one-way delay profiles for 60 s each and reports
 * the deviation distribution, the number of convergences and the actual
 * convergence intervals. A per-frame cap of one convergence is the throttle
 * under test; the same run is repeated with the cap raised to 8 so the count
 * lost to the cap can be separated from the count lost to the network
 * bunching confirmations into the same frame.
 *
 * Deterministic: no wall clock and no libc rand(); all randomness comes from a
 * seeded xoshiro256** generator.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#define HZ 60
#define DT (1.0 / HZ)
#define ACK_EVERY 2                 /* server publishes every 2 ticks = 30 Hz */
#define WARM (5 * HZ)               /* discarded warm-up */
#define MEAS (60 * HZ)              /* one minute of measurement */
#define DRAIN (12 * HZ)             /* let acks in flight land after the window */
#define FRAMES (WARM + MEAS + DRAIN)
#define SMAX 4096                   /* per-run sample capacity */
#define PMAX 512                    /* acks in flight */
#define VMAX 8.0                    /* speed clamp, world units/s */

/* Deviation thresholds, expressed in ticks of maximum displacement
 * (VMAX * DT = 0.1333 world units per tick): below a quarter tick the gap is
 * model lag and is absorbed slowly; below one tick it is corrected decisively;
 * above one tick it is a genuine jump and is corrected at once. */
#define E_SOFT (VMAX * DT * 0.25)
#define E_HARD (VMAX * DT * 1.00)
#define G_SOFT 0.35
#define G_MID 0.80
#define G_HARD 1.00

#define CAP_THROTTLED 1             /* at most one convergence per frame */
#define CAP_CONTROL 8               /* unthrottled control */

typedef struct { const char* name; double base, jit, burst_p, burst; } Profile;

/* One-way delay = base + |N(0, jit)|, plus burst with probability burst_p.
 * RTT is roughly twice the base. Bursts are what bunch acks into one frame. */
static const Profile PROFILES[3] = {
    { "low",  0.020, 0.002, 0.000, 0.000 },
    { "mid",  0.060, 0.010, 0.015, 0.050 },
    { "high", 0.140, 0.030, 0.040, 0.080 },
};

/* ------------------------------------------------------------------ rng */

typedef struct { uint64_t s[4]; } Rng;

static uint64_t rotl64(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

static uint64_t splitmix64(uint64_t* x) {
    uint64_t z = (*x += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

static void rng_seed(Rng* r, uint64_t seed) {
    uint64_t x = seed;
    for (int i = 0; i < 4; i++) r->s[i] = splitmix64(&x);
}

static uint64_t rng_next(Rng* r) {
    uint64_t result = rotl64(r->s[1] * 5, 7) * 9;
    uint64_t t = r->s[1] << 17;
    r->s[2] ^= r->s[0];
    r->s[3] ^= r->s[1];
    r->s[1] ^= r->s[2];
    r->s[0] ^= r->s[3];
    r->s[2] ^= t;
    r->s[3] = rotl64(r->s[3], 45);
    return result;
}

static double rng_u(Rng* r) {
    return (double)(rng_next(r) >> 11) * 0x1.0p-53;
}

/* Irwin-Hall sum of 4 uniforms, rescaled to unit variance. */
static double rng_gauss(Rng* r) {
    double s = 0.0;
    for (int i = 0; i < 4; i++) s += rng_u(r);
    return (s - 2.0) * 1.7320508075688772;
}

static double sample_delay(Rng* r, const Profile* p) {
    double d = p->base + fabs(rng_gauss(r)) * p->jit;
    if (rng_u(r) < p->burst_p) d += p->burst;
    return d;
}

/* --------------------------------------------------------------- motion */

/* Eight 2 s acceleration segments; deterministic, so both timelines see the
 * same input stream and any difference is caused by timing alone. */
static const double SCRIPT[8][2] = {
    {  6.0,  1.0 }, {  1.0,  7.0 }, { -5.0,  3.0 }, {  4.0, -6.0 },
    { -7.0, -1.0 }, { -2.0,  6.0 }, {  7.0, -3.0 }, { -4.0, -4.0 },
};

static void script(double t, double* ax, double* ay) {
    int i = (int)(t / 2.0) % 8;
    if (i < 0) i += 8;
    *ax = SCRIPT[i][0];
    *ay = SCRIPT[i][1];
}

typedef struct { double x, y, vx, vy; } Kin;

static void clamp_speed(double* vx, double* vy) {
    double s = hypot(*vx, *vy);
    if (s > VMAX) { *vx *= VMAX / s; *vy *= VMAX / s; }
}

static void advance(Kin* k, double ax, double ay) {
    k->vx += ax * DT;
    k->vy += ay * DT;
    clamp_speed(&k->vx, &k->vy);
    k->x += k->vx * DT;
    k->y += k->vy * DT;
}

static double dist(const Kin* a, const Kin* b) {
    return hypot(a->x - b->x, a->y - b->y);
}

/* ------------------------------------------------------------ statistics */

typedef struct {
    double dev[SMAX];   int ndev;     /* pre-convergence deviation, world units */
    double pre[SMAX];   int npair;    /* deviation of a convergence ... */
    double post[SMAX];                /* ... and the residual right after it */
    int pair_mode[SMAX];
    double interval[SMAX]; int nintv; /* ms between consecutive convergences */
    /* Every statistic below is keyed on the confirmed tick T, so the columns
     * of one row always describe the same population of confirmations. */
    long acks;          /* authoritative ticks published inside the window */
    long conv;          /* ... of which converged (each at most once) */
    long merged;        /* fresh, but displaced by the per-frame cap */
    long superseded;    /* arrived after a newer tick was already consumed */
    long mode[3];
    long lead_sum, lead_n, max_fresh;
    uint64_t hash;      /* FNV-1a over every measured deviation */
} Report;

static void report_init(Report* r) {
    memset(r, 0, sizeof(*r));
    r->hash = 0xcbf29ce484222325ULL;
}

static void hash_double(uint64_t* h, double v) {
    uint64_t u;
    memcpy(&u, &v, sizeof(u));
    for (int i = 0; i < 8; i++) {
        *h ^= (u >> (8 * i)) & 0xffu;
        *h *= 0x100000001b3ULL;
    }
}

static int classify(double e) {
    return e < E_SOFT ? 0 : (e < E_HARD ? 1 : 2);
}

static double gain_of(int m) {
    return m == 0 ? G_SOFT : (m == 1 ? G_MID : G_HARD);
}

static int cmp_double(const void* a, const void* b) {
    double x = *(const double*)a, y = *(const double*)b;
    return (x > y) - (x < y);
}

/* Nearest-rank on a sorted copy: index = round(q * (n - 1)). */
static double quantile(const double* v, int n, double q) {
    if (n <= 0) return 0.0;
    double* c = malloc(sizeof(double) * (size_t)n);
    if (!c) return 0.0;
    memcpy(c, v, sizeof(double) * (size_t)n);
    qsort(c, (size_t)n, sizeof(double), cmp_double);
    int i = (int)(q * (n - 1) + 0.5);
    if (i < 0) i = 0;
    if (i >= n) i = n - 1;
    double out = c[i];
    free(c);
    return out;
}

static double max_of(const double* v, int n) {
    double m = 0.0;
    for (int i = 0; i < n; i++) if (v[i] > m) m = v[i];
    return m;
}

/* ------------------------------------------------------------ simulation */

/* Ack in flight: arrival time and the server tick it confirms. */
typedef struct { double at; int T; } Pend;

static void simulate(const Profile* p, uint64_t seed, int cap, Report* R) {
    static double up_arr[FRAMES + 2];
    static double ack_arr[FRAMES + 2];
    static Kin local[FRAMES + 2];
    static Kin auth[FRAMES + 2];
    static unsigned char has_auth[FRAMES + 2];
    static Pend pend[PMAX];
    static int freshT[PMAX];

    Rng rng;
    rng_seed(&rng, seed);

    /* Draw the whole network schedule up front, in a fixed order, so the
     * throttled and control runs see exactly the same packets. */
    for (int t = 0; t < FRAMES + 2; t++)
        up_arr[t] = (t + 1) * DT + sample_delay(&rng, p);
    for (int T = 0; T < FRAMES + 2; T++)
        ack_arr[T] = (T + 1) * DT + (T % ACK_EVERY == 0 ? sample_delay(&rng, p) : -1.0);

    memset(local, 0, sizeof(local));
    memset(auth, 0, sizeof(auth));
    memset(has_auth, 0, sizeof(has_auth));

    Kin c = { 0, 0, 0, 0 };
    Kin s = { 0, 0, 0, 0 };
    int srv_in = -1;          /* newest input the server has received */
    int front = -1;           /* newest authoritative tick seen so far */
    int lastconv = -1;        /* frame of the previous convergence */
    int npend = 0;

    for (int t = 0; t < FRAMES; t++) {
        double now = (t + 1) * DT;
        double ax, ay, bx, by;
        int lo = WARM, hi = WARM + MEAS;    /* measurement window in ticks */

        /* Local timeline advances first, on the player's own input. */
        script(t * DT, &ax, &ay);
        advance(&c, ax, ay);
        local[t] = c;

        /* Server applies the newest input that has crossed the uplink. */
        for (int k = srv_in + 1; k <= t; k++)
            if (up_arr[k] <= now) srv_in = k;
        if (srv_in >= 0) {
            script(srv_in * DT, &bx, &by);
            advance(&s, bx, by);
        }

        /* Server publishes its confirmed timeline. */
        if (t % ACK_EVERY == 0) {
            auth[t] = s;
            has_auth[t] = 1;
            if (t >= lo && t < hi) R->acks++;
            if (npend < PMAX) { pend[npend].at = ack_arr[t]; pend[npend].T = t; npend++; }
        }

        /* Confirmations that land on this frame. */
        int nf = 0, w = 0;
        for (int i = 0; i < npend; i++) {
            if (pend[i].at <= now) {
                /* A tick already passed by, or the same tick twice: the
                 * second copy cannot converge anything. */
                if (pend[i].T > front && has_auth[pend[i].T]) freshT[nf++] = pend[i].T;
                else if (pend[i].T >= lo && pend[i].T < hi) R->superseded++;
            } else {
                pend[w++] = pend[i];
            }
        }
        npend = w;

        /* Server ticks can arrive out of order; process them oldest first so
         * the newest correction is the one that survives. */
        for (int i = 1; i < nf; i++) {
            int v = freshT[i], j = i - 1;
            while (j >= 0 && freshT[j] > v) { freshT[j + 1] = freshT[j]; j--; }
            freshT[j + 1] = v;
        }

        int inwin = (t >= lo && t < hi);

        /* Measure every fresh confirmation before anything is corrected: this
         * is the gap the player sees between the two timelines. */
        for (int i = 0; i < nf; i++) {
            int T = freshT[i];
            if (T < lo || T >= hi) continue;
            double e = dist(&local[T], &auth[T]);
            if (R->ndev < SMAX) R->dev[R->ndev++] = e;
            hash_double(&R->hash, e);
        }
        if (inwin && nf > R->max_fresh) R->max_fresh = nf;

        /* Per-frame cap: only the newest cap confirmations are acted on. The
         * older ones are superseded by newer authoritative state, not lost
         * state, so they are counted as merged rather than as convergences. */
        int start = nf > cap ? nf - cap : 0;
        for (int i = 0; i < start; i++)
            if (freshT[i] >= lo && freshT[i] < hi) R->merged++;
        for (int i = start; i < nf; i++) {
            int T = freshT[i];
            int counted = (T >= lo && T < hi);

            double e = dist(&local[T], &auth[T]);
            int m = classify(e);
            double g = gain_of(m);

            local[T].x += (auth[T].x - local[T].x) * g;
            local[T].y += (auth[T].y - local[T].y) * g;

            /* Replay the local inputs from T to the present so the correction
             * reaches the state the player is about to see. */
            Kin q = local[T];
            for (int k = T + 1; k <= t; k++) {
                script(k * DT, &ax, &ay);
                advance(&q, ax, ay);
                local[k] = q;
            }
            c = local[t];

            if (!counted) continue;      /* corrections run, statistics do not */
            R->conv++;
            R->mode[m]++;
            hash_double(&R->hash, e);
            if (R->npair < SMAX) {
                R->pre[R->npair] = e;
                R->post[R->npair] = dist(&local[T], &auth[T]);
                R->pair_mode[R->npair] = m;
                R->npair++;
            }
            if (lastconv >= 0 && R->nintv < SMAX)
                R->interval[R->nintv++] = (double)(t - lastconv) * DT * 1000.0;
            lastconv = t;
        }

        if (inwin && front >= 0 && nf > 0) {
            R->lead_sum += t - front;
            R->lead_n++;
        }
        if (nf > 0) front = freshT[nf - 1];
    }
}

/* --------------------------------------------------------------- checks */

static int failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Convergence gain relation: post should be (1 - g) * pre exactly. */
static int gain_relation_ok(const Report* r, int mode) {
    int n = 0;
    for (int i = 0; i < r->npair; i++) {
        if (r->pair_mode[i] != mode) continue;
        n++;
        double want = (1.0 - gain_of(mode)) * r->pre[i];
        if (r->pre[i] > 1e-9 && fabs(r->post[i] - want) > 1e-9 * (1.0 + r->pre[i])) return 0;
    }
    return n > 0;
}

static int all_intervals_ge_tick(const Report* r) {
    for (int i = 0; i < r->nintv; i++)
        if (r->interval[i] < 1000.0 / HZ - 1e-6) return 0;
    return 1;
}

int main(void) {
    Report run[3], ctl[3];
    uint64_t seed = 20261124u;

    for (int i = 0; i < 3; i++) {
        report_init(&run[i]);
        report_init(&ctl[i]);
        simulate(&PROFILES[i], seed, CAP_THROTTLED, &run[i]);
        simulate(&PROFILES[i], seed, CAP_CONTROL, &ctl[i]);
    }

    printf("seed=%llu dt=1/%d ack_every=%d frames=%d window=%.1fs\n",
           (unsigned long long)seed, HZ, ACK_EVERY, FRAMES, (double)MEAS / HZ);

    /* ---- headline: three groups of six numbers ---------------------- */
    printf("\n=== 60 s window: three groups of six numbers ===\n");
    printf("columns: convergences, then deviation (world units) p50 p90 p95 max mean\n");
    printf("%-5s %8s %8s %8s %8s %8s %8s\n",
           "delay", "conv", "dev p50", "dev p90", "dev p95", "dev max", "dev mean");
    for (int i = 0; i < 3; i++) {
        const double* d = run[i].dev;
        int n = run[i].ndev;
        double sum = 0.0;
        for (int k = 0; k < n; k++) sum += d[k];
        printf("%-5s %8ld %8.4f %8.4f %8.4f %8.4f %8.4f\n",
               PROFILES[i].name, run[i].conv,
               quantile(d, n, 0.50), quantile(d, n, 0.90),
               quantile(d, n, 0.95), max_of(d, n), n ? sum / n : 0.0);
    }

    /* ---- convergence intervals, side by side ------------------------- */
    printf("\n=== actual convergence interval (ms) ===\n");
    printf("%-5s %5s %7s %7s %8s %7s | %s\n",
           "delay", "n", "p50", "p95", "mean", "max", "control cap=8 (n p50 p95 conv)");
    for (int i = 0; i < 3; i++) {
        int n = run[i].nintv;
        double sum = 0.0;
        for (int k = 0; k < n; k++) sum += run[i].interval[k];
        printf("%-5s %5d %7.1f %7.1f %8.1f %7.1f | n=%d p50=%.1f p95=%.1f conv=%ld\n",
               PROFILES[i].name, n,
               quantile(run[i].interval, n, 0.50), quantile(run[i].interval, n, 0.95),
               n ? sum / n : 0.0, max_of(run[i].interval, n),
               ctl[i].nintv, quantile(ctl[i].interval, ctl[i].nintv, 0.50),
               quantile(ctl[i].interval, ctl[i].nintv, 0.95), ctl[i].conv);
    }

    /* ---- interval histogram: the throttle leaves a visible grid ------ */
    printf("\n=== interval histogram (share of intervals) ===\n");
    {
        const double edges[] = { 25.0, 41.7, 58.3, 75.0, 100.0 };
        const char* labels[] = { "16.7", "33.3", "50.0", "66.7", "83.3", ">=100" };
        printf("%-5s", "delay");
        for (int b = 0; b < 6; b++) printf(" %7s", labels[b]);
        printf("\n");
        for (int i = 0; i < 3; i++) {
            long bin[6] = { 0, 0, 0, 0, 0, 0 };
            int n = run[i].nintv;
            for (int k = 0; k < n; k++) {
                int b = 0;
                while (b < 5 && run[i].interval[k] >= edges[b] - 1e-9) b++;
                bin[b]++;
            }
            printf("%-5s", PROFILES[i].name);
            for (int b = 0; b < 6; b++)
                printf(" %6.1f%%", n ? 100.0 * (double)bin[b] / n : 0.0);
            printf("\n");
        }
    }

    /* ---- bookkeeping: where the confirmations went ------------------- */
    printf("\n=== where each confirmation went (throttled, cap=1) ===\n");
    printf("every number below is keyed on the confirmed server tick\n");
    printf("%-5s %8s %8s %8s %8s %10s %8s %s\n",
           "delay", "acks", "conv", "merged", "superseded", "lead(tick)", "max/frm", "modes soft/mid/hard");
    for (int i = 0; i < 3; i++) {
        printf("%-5s %8ld %8ld %8ld %8ld %10.1f %8ld %ld/%ld/%ld\n",
               PROFILES[i].name, run[i].acks, run[i].conv,
               run[i].merged, run[i].superseded,
               run[i].lead_n ? (double)run[i].lead_sum / run[i].lead_n : 0.0,
               run[i].max_fresh, run[i].mode[0], run[i].mode[1], run[i].mode[2]);
    }

    /* ---- throttle verdict -------------------------------------------- */
    printf("\n=== throttle check ===\n");
    printf("high_convs_fewer_than_low=%d\n", run[2].conv < run[0].conv);
    for (int i = 0; i < 3; i++) {
        long lost_to_cap = ctl[i].conv - run[i].conv;
        printf("%-5s acks=%ld conv=%ld control=%ld lost_to_cap=%ld (%.1f%% of control) "
               "merged=%ld superseded=%ld\n",
               PROFILES[i].name, run[i].acks, run[i].conv, ctl[i].conv, lost_to_cap,
               ctl[i].conv ? 100.0 * (double)lost_to_cap / ctl[i].conv : 0.0,
               run[i].merged, run[i].superseded);
    }

    /* ---- self-checks ------------------------------------------------- */
    printf("\n=== checks ===\n");

    Report again;
    report_init(&again);
    simulate(&PROFILES[2], seed, CAP_THROTTLED, &again);

    check("determinism", again.hash == run[2].hash && again.conv == run[2].conv);
    check("window_one_minute", MEAS == 60 * HZ);
    check("counts_positive", run[0].conv > 0 && run[1].conv > 0 && run[2].conv > 0);
    check("conv_ordering_high_lt_low", run[2].conv < run[0].conv);
    check("conv_ordering_monotone", run[0].conv >= run[1].conv && run[1].conv >= run[2].conv);
    check("control_ge_throttled",
          ctl[0].conv >= run[0].conv && ctl[1].conv >= run[1].conv && ctl[2].conv >= run[2].conv);
    check("low_no_merge",
          run[0].merged == 0 && run[0].superseded == 0 && run[0].conv == run[0].acks);
    check("ack_accounting",
          run[0].conv + run[0].merged + run[0].superseded == run[0].acks &&
          run[1].conv + run[1].merged + run[1].superseded == run[1].acks &&
          run[2].conv + run[2].merged + run[2].superseded == run[2].acks);
    check("interval_ge_tick",
          all_intervals_ge_tick(&run[0]) && all_intervals_ge_tick(&run[1]) &&
          all_intervals_ge_tick(&run[2]));

    {
        double a = quantile(run[0].dev, run[0].ndev, 0.50);
        double b = quantile(run[1].dev, run[1].ndev, 0.50);
        double c = quantile(run[2].dev, run[2].ndev, 0.50);
        check("dev_median_grows_with_delay", a < b && b < c);
    }
    {
        double a = quantile(run[0].dev, run[0].ndev, 0.95);
        double b = quantile(run[1].dev, run[1].ndev, 0.95);
        double c = quantile(run[2].dev, run[2].ndev, 0.95);
        check("dev_p95_grows_with_delay", a < b && b < c);
    }
    {
        double a = (double)run[0].lead_sum / run[0].lead_n;
        double b = (double)run[1].lead_sum / run[1].lead_n;
        double c = (double)run[2].lead_sum / run[2].lead_n;
        check("lead_grows_with_delay", a < b && b < c);
    }
    check("soft_gain_relation", gain_relation_ok(&run[0], 0) || gain_relation_ok(&run[1], 0));
    check("mid_gain_relation", gain_relation_ok(&run[1], 1) || gain_relation_ok(&run[2], 1));
    check("hard_gain_exact",
          gain_of(2) == 1.0 &&
          (gain_relation_ok(&run[2], 2) || gain_relation_ok(&run[1], 2)));
    check("gain_table", gain_of(0) == G_SOFT && gain_of(1) == G_MID && gain_of(2) == G_HARD);
    check("classify_table",
          classify(0.0) == 0 && classify(E_SOFT - 1e-9) == 0 &&
          classify(E_SOFT) == 1 && classify(E_HARD - 1e-9) == 1 &&
          classify(E_HARD) == 2 && classify(1e9) == 2);
    {
        double v[5] = { 4.0, 1.0, 3.0, 2.0, 5.0 };
        check("quantile_edges",
              quantile(v, 5, 0.0) == 1.0 && quantile(v, 5, 1.0) == 5.0 &&
              quantile(v, 5, 0.5) == 3.0 && quantile(v, 5, 0.95) == 5.0);
    }
    check("superseded_only_when_reordered", run[0].superseded == 0 && run[2].superseded > 0);

    /* Cross-seed robustness: the count ordering must not be an artefact of
       one seed. Five extra seeds, throttled run only. */
    {
        int ordering = 1, grow = 1;
        double worst_ratio = 1e9;
        for (uint64_t s = 1; s <= 5; s++) {
            Report a, b, c;
            report_init(&a); report_init(&b); report_init(&c);
            simulate(&PROFILES[0], seed + s * 7919, CAP_THROTTLED, &a);
            simulate(&PROFILES[1], seed + s * 7919, CAP_THROTTLED, &b);
            simulate(&PROFILES[2], seed + s * 7919, CAP_THROTTLED, &c);
            if (!(a.conv >= b.conv && b.conv >= c.conv)) ordering = 0;
            double r = a.conv ? (double)c.conv / a.conv : 0.0;
            if (r < worst_ratio) worst_ratio = r;
            if (!(quantile(a.dev, a.ndev, 0.5) < quantile(c.dev, c.ndev, 0.5))) grow = 0;
        }
        check("seed_robust_conv_ordering", ordering);
        check("seed_robust_dev_growth", grow);
        /* Confirm the cap, not the network, still leaves a clear gap on every
           seed: high/low convergence ratio stays well below one. */
        check("seed_robust_throttle_gap", worst_ratio < 0.9);
        printf("seed_robust_worst_high_over_low_conv_ratio=%.3f\n", worst_ratio);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
