
/* anchor: keyframe (anchor) tick selection from change magnitude and loss rate.
 *
 * Two senders share one deterministic loss/change scenario:
 *   fixed  - keyframe every fixed interval (interval 0 rejected)
 *   adapt  - urgency = change/chg_th + loss_ewma/loss_th, send when >= 1,
 *            bounded by min_gap / max_gap
 * Receiver counts recovery ticks from a desync to the next delivered keyframe.
 * Sweeps both policies, emits anchor.csv + an ASCII tradeoff curve, and matches
 * adaptive points against fixed points at equal bandwidth.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define TICKS       4000
#define KEY_BYTES   120
#define DELTA_BYTES 12
#define EWMA_ALPHA  (1.0 / 16.0)
#define MAX_REC     2048

/* scenario: recurring loss episodes that also carry large scene changes */
#define EP_EVERY    180
#define EP_LEN      30
#define EP_LOSS     0.35
#define BASE_LOSS   0.02
#ifndef SCEN_SEED
#define SCEN_SEED   0x12345678u
#endif
#ifndef SIM_SEED
#define SIM_SEED    0x9e3779b9u
#endif

typedef struct { double change; double loss; } Env;

/* ---------- deterministic rng (xorshift32), so runs are reproducible ---------- */
static unsigned rng_state;
static unsigned rndu(void) {
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}
static double rnd01(void) { return (rndu() >> 8) / 16777216.0; }

static int in_episode(int t) { return (t % EP_EVERY) < EP_LEN; }

static void build_scenario(Env* env, int n, unsigned seed) {
    rng_state = seed;
    for (int t = 0; t < n; t++) {
        int ep = in_episode(t);
        env[t].loss   = ep ? EP_LOSS : BASE_LOSS;
        env[t].change = ep ? 0.9 + 0.5 * rnd01() : 0.1 + 0.1 * rnd01();
    }
}

/* ---------- policies ---------- */
typedef struct { int interval; } Fixed;

static int fixed_init(Fixed* f, int interval) {
    if (interval <= 0) return -1;              /* zero / negative interval rejected */
    f->interval = interval;
    return 0;
}
static int fixed_should(const Fixed* f, int gap) { return gap >= f->interval; }

typedef struct {
    double chg_th, loss_th, loss_ewma;
    int min_gap, max_gap;
    int retry_gap;          /* gap after a loss-observed, possibly-healing keyframe */
    int retries_left;       /* remaining fast-retry budget */
    int max_retries;
} Adapt;

static int adapt_init(Adapt* a, double chg_th, double loss_th, int min_gap, int max_gap) {
    if (chg_th <= 0.0 || loss_th <= 0.0) return -1;
    if (min_gap <= 0 || max_gap < min_gap) return -1;
    a->chg_th = chg_th; a->loss_th = loss_th;
    a->min_gap = min_gap; a->max_gap = max_gap;
    a->retry_gap = min_gap; a->max_retries = 3; a->retries_left = 0;
    a->loss_ewma = 0.0;
    return 0;
}

/* feed 0/1 delivery feedback, decide whether tick t carries a keyframe.
 * a lost packet means the receiver may be desync: the next keyframe goes out
 * quickly and, if it too gets lost, a small retry budget keeps re-anchoring on
 * a short gap instead of waiting a full max_gap cycle. */
static int adapt_should(Adapt* a, double change, int lost, int gap) {
    a->loss_ewma += EWMA_ALPHA * ((double)lost - a->loss_ewma);
    if (gap >= a->max_gap) { a->retries_left = 0; return 1; }
    if (gap < a->min_gap) return 0;
    if (lost) {
        if (gap >= a->min_gap) a->retries_left = a->max_retries;
    }
    if (a->retries_left > 0 && gap >= a->retry_gap) {
        a->retries_left--;
        return 1;
    }
    double urgency = change / a->chg_th + a->loss_ewma / a->loss_th;
    return urgency >= 1.0;
}

/* ---------- simulation ---------- */
typedef struct {
    long keys, bytes, desyncs, keys_in_ep;
    int rec[MAX_REC]; int nrec;
    double join_sum; int join_n;
    double rate, join_mean;
    int rec_mean, rec_p50, rec_p90, rec_max;
} Stats;

static int cmp_int(const void* pa, const void* pb) {
    int a = *(const int*)pa, b = *(const int*)pb;
    return (a > b) - (a < b);
}

/* adaptive != 0 runs the adaptive policy (ap_in), otherwise the fixed policy.
 * the loss draw happens before the policy decision, so both senders see the
 * exact same packet-loss trajectory. */
static void simulate(const Env* env, int n, int adaptive,
                     const Fixed* fp, const Adapt* ap_in, Stats* st, unsigned seed) {
    memset(st, 0, sizeof(*st));
    Adapt a = {0};
    if (adaptive) a = *ap_in;
    rng_state = seed;
    int last_key = -1000000;                   /* first tick is always eligible */
    int synced = 1, desync_at = 0;
    unsigned char* key_del = calloc((size_t)n, 1);

    for (int t = 0; t < n; t++) {
        int lost = rnd01() < env[t].loss;     /* shared draw: fair comparison */
        int gap = t - last_key;
        int iskey = adaptive ? adapt_should(&a, env[t].change, lost, gap)
                             : fixed_should(fp, gap);
        st->bytes += iskey ? KEY_BYTES : DELTA_BYTES;
        if (iskey) {
            st->keys++;
            last_key = t;
            if (in_episode(t)) st->keys_in_ep++;
            if (!lost) key_del[t] = 1;
        }
        if (synced) {
            if (lost) { synced = 0; desync_at = t; st->desyncs++; }
        } else if (iskey && !lost) {
            synced = 1;
            if (st->nrec < MAX_REC) st->rec[st->nrec++] = t - desync_at;
        }
    }

    /* late joins: fresh receiver waits for the first delivered keyframe */
    for (int j = 200; j < n; j += 400) {
        for (int t = j; t < n; t++) {
            if (key_del[t]) { st->join_sum += (double)(t - j); st->join_n++; break; }
        }
    }
    free(key_del);

    st->rate = (double)st->bytes / n;
    st->join_mean = st->join_n ? st->join_sum / st->join_n : 0.0;
    if (st->nrec) {
        qsort(st->rec, st->nrec, sizeof(int), cmp_int);
        long sum = 0;
        for (int i = 0; i < st->nrec; i++) sum += st->rec[i];
        st->rec_mean = (int)(sum / st->nrec + 0.5);
        st->rec_p50  = st->rec[st->nrec / 2];
        st->rec_p90  = st->rec[(st->nrec * 9) / 10 < st->nrec ? (st->nrec * 9) / 10 : st->nrec - 1];
        st->rec_max  = st->rec[st->nrec - 1];
    }
}

/* ---------- test harness ---------- */
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

typedef struct { char tag; double param; Stats st; } Point;

static int cmp_point_rate(const void* pa, const void* pb) {
    double d = ((const Point*)pa)->st.rate - ((const Point*)pb)->st.rate;
    return (d > 0) - (d < 0);
}

#define GW 64
#define GH 18
static void tradeoff_curve(const Point* pts, int np) {
    char grid[GH][GW + 1];
    double xmin = pts[0].st.rate, xmax = pts[0].st.rate, ymax = 1;
    for (int i = 0; i < np; i++) {
        if (pts[i].st.rate < xmin) xmin = pts[i].st.rate;
        if (pts[i].st.rate > xmax) xmax = pts[i].st.rate;
        if (pts[i].st.rec_mean > ymax) ymax = pts[i].st.rec_mean;
    }
    for (int r = 0; r < GH; r++) memset(grid[r], ' ', GW), grid[r][GW] = 0;
    for (int i = 0; i < np; i++) {
        int gx = (int)((pts[i].st.rate - xmin) / (xmax - xmin) * (GW - 1) + 0.5);
        int gy = GH - 1 - (int)(pts[i].st.rec_mean / ymax * (GH - 1) + 0.5);
        if (gx < 0) gx = 0;
        if (gx >= GW) gx = GW - 1;
        if (gy < 0) gy = 0;
        if (gy >= GH) gy = GH - 1;
        grid[gy][gx] = pts[i].tag;          /* F = fixed, A = adaptive */
    }
    printf("\nrecovery (ticks) vs bandwidth (bytes/tick), F=fixed A=adaptive\n");
    for (int r = 0; r < GH; r++) {
        if (r % 4 == 0) printf("%4.0f |%s\n", ymax * (GH - 1 - r) / (GH - 1), grid[r]);
        else            printf("     |%s\n", grid[r]);
    }
    printf("     +");
    for (int i = 0; i < GW; i++) putchar('-');
    printf("\n      %-*g%*g\n", GW / 2, xmin, GW / 2, xmax);
}

int main(void) {
    /* ---- unit tests on policy state machines ---- */
    Fixed f;
    check("fixed_zero_rejected", fixed_init(&f, 0) == -1);
    check("fixed_negative_rejected", fixed_init(&f, -3) == -1);
    check("fixed_init_ok", fixed_init(&f, 10) == 0 && f.interval == 10);

    Adapt a;
    check("adapt_zero_th_rejected", adapt_init(&a, 0.0, 0.15, 4, 128) == -1);
    check("adapt_zero_gap_rejected", adapt_init(&a, 1.0, 0.15, 0, 128) == -1);
    check("adapt_bad_range_rejected", adapt_init(&a, 1.0, 0.15, 64, 8) == -1);
    check("adapt_init_ok", adapt_init(&a, 1.0, 0.15, 4, 128) == 0);

    fixed_init(&f, 10);
    check("fixed_periodic", !fixed_should(&f, 9) && fixed_should(&f, 10) && fixed_should(&f, 11));

    Adapt t;
    adapt_init(&t, 1.0, 0.15, 1, 128);
    check("adapt_change_trigger", adapt_should(&t, 1.0, 0, 100) == 1);
    /* EWMA path: calm changes so urgency only crosses via loss_ewma; spend the
     * retry budget first (all-loss stream, gaps >= min_gap keeps it armed) */
    adapt_init(&t, 1.0, 0.15, 1, 128);
    int fire_at = -1;
    for (int i = 0; i < 10; i++) {
        if (adapt_should(&t, 0.0, 1, 100)) { fire_at = i; break; }
    }
    check("adapt_loss_trigger", fire_at == 0);   /* first loss arms immediate retry */
    Adapt r;
    adapt_init(&r, 1.0, 0.15, 2, 128);
    check("retry_budget_caps",
          adapt_should(&r, 0.0, 1, 2) == 1 &&    /* loss arms: immediate keyframe */
          adapt_should(&r, 0.0, 0, 2) == 1 &&    /* then 2 budgeted fast retries */
          adapt_should(&r, 0.0, 0, 2) == 1 &&
          adapt_should(&r, 0.0, 0, 2) == 0 &&    /* budget spent; urgency below 1 */
          adapt_should(&r, 0.0, 0, 2) == 0);
    check("loss_rearms_retry",
          adapt_should(&r, 0.0, 1, 2) == 1 &&    /* a new loss re-arms the budget */
          adapt_should(&r, 0.0, 0, 2) == 1);
    adapt_init(&t, 1.0, 0.15, 1, 5);
    check("adapt_max_gap_cap", adapt_should(&t, 0.0, 0, 5) == 1);
    adapt_init(&t, 1.0, 0.15, 4, 128);
    check("adapt_min_gap_floor", adapt_should(&t, 100.0, 1, 3) == 0 &&
                                  adapt_should(&t, 100.0, 1, 4) == 1);

    /* ---- end-to-end on hand-built loss patterns (loss 1.0/0.0 is rng-independent) ---- */
    Env small[7];
    for (int i = 0; i < 7; i++) small[i] = (Env){ 0.0, 0.0 };
    Stats s;
    Fixed f3; fixed_init(&f3, 3);
    small[2].loss = 1.0;                       /* delta lost: desync at 2, key at 3 heals */
    simulate(small, 7, 0, &f3, NULL, &s, 1);
    check("recovery_basic", s.desyncs == 1 && s.nrec == 1 && s.rec[0] == 1);

    small[3].loss = 1.0;                       /* healing key lost too: next key at 6 */
    simulate(small, 7, 0, &f3, NULL, &s, 1);
    check("recovery_lost_key", s.desyncs == 1 && s.rec[0] == 4);

    Env calm[12];
    for (int i = 0; i < 12; i++) calm[i] = (Env){ 0.0, 0.0 };
    Adapt ac; adapt_init(&ac, 1.0, 0.15, 1, 5);
    simulate(calm, 12, 1, NULL, &ac, &s, 1);  /* only max_gap forces keys: 0,5,10 */
    check("adapt_calm_sparse", s.keys == 3 && s.desyncs == 0);

    /* ---- full scenario + sweeps ---- */
    Env* env = malloc(sizeof(Env) * TICKS);
    build_scenario(env, TICKS, SCEN_SEED);

    static const int intervals[] = {
        8,9,10,11,12,13,14,15,16,17,18,19,20,22,24,26,28,30,32,36,40,
        48,56,64,80,96,112,128,160
    };
    static const double thresholds[] = {
        0.25,0.35,0.5,0.7,1.0,1.4,2.0,2.8,4.0,6.0
    };
    int nf = (int)(sizeof(intervals) / sizeof(intervals[0]));
    int na = (int)(sizeof(thresholds) / sizeof(thresholds[0]));
    int np = nf + na;
    Point* pts = malloc(sizeof(Point) * np);
    int k = 0;
    for (int i = 0; i < nf; i++) {
        Fixed fp; fixed_init(&fp, intervals[i]);
        pts[k].tag = 'F'; pts[k].param = intervals[i];
        simulate(env, TICKS, 0, &fp, NULL, &pts[k].st, SIM_SEED);
        k++;
    }
    for (int i = 0; i < na; i++) {
        Adapt ap; adapt_init(&ap, thresholds[i], 0.15, 4, 128);
        pts[k].tag = 'A'; pts[k].param = thresholds[i];
        simulate(env, TICKS, 1, NULL, &ap, &pts[k].st, SIM_SEED);
        k++;
    }

    FILE* csv = fopen("anchor.csv", "w");
    if (csv) {
        fprintf(csv, "policy,param,keys,bytes_per_tick,desyncs,rec_mean,"
                     "rec_p50,rec_p90,rec_max,join_mean,keys_in_ep_pct\n");
        for (int i = 0; i < np; i++) {
            Stats* z = &pts[i].st;
            fprintf(csv, "%c,%g,%ld,%.3f,%ld,%d,%d,%d,%d,%.1f,%.1f\n",
                    pts[i].tag, pts[i].param, z->keys, z->rate, z->desyncs,
                    z->rec_mean, z->rec_p50, z->rec_p90, z->rec_max,
                    z->join_mean, 100.0 * z->keys_in_ep / z->keys);
        }
        fclose(csv);
        printf("\ncsv=anchor.csv\n");
    }

    printf("\npolicy param  keys  B/tick  desync  rec_mean  p50  p90  max  join  in_ep%%\n");
    for (int i = 0; i < np; i++) {
        Stats* z = &pts[i].st;
        printf("%-6c %5g %5ld %8.3f %6ld %9d %4d %4d %4d %5.1f %6.1f\n",
               pts[i].tag, pts[i].param, z->keys, z->rate, z->desyncs,
               z->rec_mean, z->rec_p50, z->rec_p90, z->rec_max,
               z->join_mean, 100.0 * z->keys_in_ep / z->keys);
    }

    Point* sorted = malloc(sizeof(Point) * np);
    memcpy(sorted, pts, sizeof(Point) * np);
    qsort(sorted, np, sizeof(Point), cmp_point_rate);
    tradeoff_curve(sorted, np);

    /* ---- equal-bandwidth matching: nearest fixed point within 3% ---- */
    printf("\nequal-bandwidth pairs (fixed vs adaptive, same B/tick):\n");
    printf(" B/tick  fixed(I->rec)  adapt(th->rec)  improve%%\n");
    int matched = 0, wins = 0;
    double imp_sum = 0.0;
    for (int i = 0; i < np; i++) {
        if (pts[i].tag != 'A') continue;
        int best = -1; double best_diff = 1e9;
        for (int j = 0; j < np; j++) {
            if (pts[j].tag != 'F') continue;
            double d = fabs(pts[j].st.rate - pts[i].st.rate) / pts[i].st.rate;
            if (d < best_diff) { best_diff = d; best = j; }
        }
        if (best < 0 || best_diff > 0.03) continue;
        matched++;
        Stats* af = &pts[best].st, * aa = &pts[i].st;
        double imp = 100.0 * (af->rec_mean - aa->rec_mean) / af->rec_mean;
        imp_sum += imp;
        if (aa->rec_mean < af->rec_mean) wins++;
        printf("%7.3f  %4g -> %-5d  %4g -> %-5d  %+6.1f\n",
               aa->rate, pts[best].param, af->rec_mean,
               pts[i].param, aa->rec_mean, imp);
    }
    double win_pct = matched ? 100.0 * wins / matched : 0.0;
    double imp_mean = matched ? imp_sum / matched : 0.0;
    printf("matched=%d adaptive_wins=%d win_pct=%.1f%% mean_improve=%.1f%%\n",
           matched, wins, win_pct, imp_mean);
    check("equal_bw_shorter_recovery", matched >= 8 && win_pct >= 80.0 && imp_mean >= 15.0);

    /* ---- determinism: same seed must reproduce every number ---- */
    Stats r1, r2;
    Adapt ap; adapt_init(&ap, 1.0, 0.15, 4, 128);
    simulate(env, TICKS, 1, NULL, &ap, &r1, 777);
    simulate(env, TICKS, 1, NULL, &ap, &r2, 777);
    int same = r1.bytes == r2.bytes && r1.keys == r2.keys && r1.nrec == r2.nrec &&
               r1.join_sum == r2.join_sum &&
               memcmp(r1.rec, r2.rec, sizeof(int) * r1.nrec) == 0;
    check("deterministic", same);

    free(env); free(pts); free(sorted);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
