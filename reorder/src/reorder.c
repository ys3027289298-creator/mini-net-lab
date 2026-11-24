
/* reorder: length-configurable resequencing buffer for snapshot streams.
 *
 * Snapshots are emitted at a fixed tick interval dt.  The network may deliver
 * them out of order; applying an arrival immediately makes the client rewind
 * to an older tick, which shows up as a backward teleport.  The buffer holds
 * frame for `hold` ticks (the resequencing window) and releases strictly in
 * increasing tick order, so the applied stream can never move backwards.
 *
 * Relation between the reorder window and the tolerable rollback time T:
 *   a frame that overtakes a later frame by w ticks rewinds w*dt seconds,
 *   so w*dt <= T  <=>  w <= T/dt.  With hold = D ticks the buffer absorbs
 *   every out-of-order span <= D and drops anything older; D = floor(T/dt)
 *   is the largest window that cannot produce a rollback beyond T.
 *   Capacity: ticks inside (newest-D, newest] are at most D+1 distinct
 *   frames, so cap >= D+1 keeps the window intact; if cap is smaller the
 *   oldest frame is evicted and counted.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define REORDER_MAX 64
#define OUT_MAX 4096

typedef struct { unsigned tick; float x; } Frame;

typedef struct {
    Frame f[REORDER_MAX];
    int cap;                 /* configured length in frames */
    int count;
    int hold;                /* playout delay D in ticks */
    long long newest;        /* highest tick seen, -1 if none */
    long long last_out;      /* tick of last delivered frame, -1 if none */
    long late_dropped;       /* tick <= last_out: cannot be placed any more */
    long evicted;            /* oldest frame discarded because buffer was full */
    long duplicated;
    long delivered;
    Frame out[OUT_MAX];      /* delivered sequence, for inspection */
    int out_n;
    int out_trunc;
} Reorder;

static void reorder_init(Reorder* r, int cap, int hold) {
    memset(r, 0, sizeof(*r));
    r->cap = cap;
    r->hold = hold;
    r->newest = -1;
    r->last_out = -1;
}

static void reorder_deliver(Reorder* r, const Frame* fr) {
    r->last_out = (long long)fr->tick;
    r->delivered++;
    if (r->out_n < OUT_MAX) r->out[r->out_n++] = *fr;
    else r->out_trunc = 1;
}

/* Drop the delivered-queue head after it has been emitted. */
static void reorder_pop(Reorder* r) {
    if (r->count <= 0) return;
    memmove(&r->f[0], &r->f[1], sizeof(Frame) * (size_t)(r->count - 1));
    r->count--;
}

/* Accept one frame.  Returns 0 when accepted, -1 when rejected. */
static int reorder_insert(Reorder* r, unsigned tick, float x) {
    if (r->cap < 1 || r->hold < 0) return -1;
    if (r->last_out >= 0 && (long long)tick <= r->last_out) {
        r->late_dropped++;                 /* would rewind the client */
        return -1;
    }
    for (int i = 0; i < r->count; i++) {
        if (r->f[i].tick == tick) { r->duplicated++; return -1; }
    }
    if (r->count == r->cap) {              /* full: discard the oldest frame */
        reorder_pop(r);
        r->evicted++;
    }
    int i = r->count;
    while (i > 0 && r->f[i - 1].tick > tick) { r->f[i] = r->f[i - 1]; i--; }
    r->f[i].tick = tick;
    r->f[i].x = x;
    r->count++;
    if ((long long)tick > r->newest) r->newest = (long long)tick;

    /* release everything older than the playout delay, in tick order */
    while (r->count > 0 && (long long)r->f[0].tick <= r->newest - r->hold) {
        reorder_deliver(r, &r->f[0]);
        reorder_pop(r);
    }
    return 0;
}

/* End of stream: emit whatever is still held, still in tick order. */
static void reorder_flush(Reorder* r) {
    while (r->count > 0) {
        reorder_deliver(r, &r->f[0]);
        reorder_pop(r);
    }
}

/* Strictly increasing delivered ticks mean no rollback can come from reorder. */
static int reorder_monotonic(const Reorder* r) {
    for (int i = 1; i < r->out_n; i++) {
        if (r->out[i - 1].tick >= r->out[i].tick) return 0;
    }
    return 1;
}

/* Max backward tick jump of the delivered sequence (0 when monotonic). */
static long reorder_max_rewind(const Reorder* r) {
    long worst = 0;
    long long prev = -1;
    for (int i = 0; i < r->out_n; i++) {
        if (prev >= 0 && (long long)r->out[i].tick < prev) {
            long d = (long)(prev - (long long)r->out[i].tick);
            if (d > worst) worst = d;
        }
        if ((long long)r->out[i].tick > prev) prev = (long long)r->out[i].tick;
    }
    return worst;
}

/* ---------------- deterministic channel + experiment ---------------- */

#define TICKS 200
#define DT 0.05          /* seconds per tick, 20 Hz */
#define SPEED 10.0       /* units per second */
#define STEP (SPEED * DT) /* units per tick, 0.5 */
#define JITTER 5         /* max network delay jitter, ticks (250 ms) */
#define MAX_ARR 256

typedef struct { unsigned tick; float x; double at; } Arrival;

static unsigned rng_state;
static void rng_seed(unsigned s) { rng_state = s ? s : 1u; }
static unsigned rng_next(void) {
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}
static double rng_unit(void) { return (double)(rng_next() >> 8) / 16777216.0; }

static int cmp_arrival(const void* a, const void* b) {
    const Arrival* x = (const Arrival*)a;
    const Arrival* y = (const Arrival*)b;
    if (x->at < y->at) return -1;
    if (x->at > y->at) return 1;
    return x->tick < y->tick ? -1 : (x->tick > y->tick ? 1 : 0);
}

/* Emit TICKS snapshots, lose each with probability `loss`, give each
 * survivor a uniform delay in [0, JITTER) ticks and sort by arrival time.
 * Returns surviving count and the largest out-of-order span seen. */
static int build_stream(double loss, unsigned seed, Arrival* arr, int* max_span) {
    rng_seed(seed);
    int n = 0;
    for (unsigned t = 0; t < TICKS; t++) {
        if (rng_unit() < loss) continue;
        arr[n].tick = t;
        arr[n].x = (float)((double)t * STEP);
        arr[n].at = (double)t + JITTER * rng_unit();
        n++;
    }
    qsort(arr, (size_t)n, sizeof(Arrival), cmp_arrival);
    long long newest = -1;
    int span = 0;
    for (int i = 0; i < n; i++) {
        if (newest >= 0 && newest > (long long)arr[i].tick) {
            long d = (long)(newest - (long long)arr[i].tick);
            if (d > span) span = d;
        }
        if ((long long)arr[i].tick > newest) newest = (long long)arr[i].tick;
    }
    *max_span = span;
    return n;
}

typedef struct {
    int sent, net_lost, arrived, delivered;
    int max_span;
    long late, evicted, duplicated, dropped;
    long naive_rewind_ticks;   /* max backward jump when applied on arrival */
    long buffered_rewind_ticks;
    float final_x;
} RunResult;

/* Feed a stream through the reorder buffer and measure both the naive
 * apply-on-arrival rewind and the buffered result. */
static void measure(const Arrival* arr, int n, int cap, int hold,
                    Reorder* r, RunResult* out) {
    long long last = -1;
    out->naive_rewind_ticks = 0;
    for (int i = 0; i < n; i++) {
        long long t = (long long)arr[i].tick;
        if (last >= 0 && t < last) {
            long d = (long)(last - t);
            if (d > out->naive_rewind_ticks) out->naive_rewind_ticks = d;
        }
        if (t > last) last = t;
    }
    reorder_init(r, cap, hold);
    for (int i = 0; i < n; i++) reorder_insert(r, arr[i].tick, arr[i].x);
    reorder_flush(r);
    out->arrived = n;
    out->late = r->late_dropped;
    out->evicted = r->evicted;
    out->duplicated = r->duplicated;
    out->dropped = r->late_dropped + r->evicted + r->duplicated;
    out->delivered = r->delivered;
    out->buffered_rewind_ticks = reorder_max_rewind(r);
    out->final_x = r->out_n > 0 ? r->out[r->out_n - 1].x : 0.0f;
}

static void run_once(double loss, unsigned seed, int cap, int hold, RunResult* out) {
    Arrival arr[MAX_ARR];
    memset(out, 0, sizeof(*out));
    int n = build_stream(loss, seed, arr, &out->max_span);
    out->sent = TICKS;
    out->net_lost = TICKS - n;
    Reorder r;
    measure(arr, n, cap, hold, &r, out);
}

static void print_sequence(const Reorder* r) {
    for (int i = 0; i < r->out_n; i++) {
        printf("%s%.2f", i ? "," : "", (double)r->out[i].x);
    }
}

static void report_run(double loss, int run, int cap, int hold, unsigned seed) {
    Arrival arr[MAX_ARR];
    RunResult out;
    memset(&out, 0, sizeof(out));
    int n = build_stream(loss, seed, arr, &out.max_span);
    out.sent = TICKS;
    out.net_lost = TICKS - n;
    Reorder r;
    measure(arr, n, cap, hold, &r, &out);
    printf("run loss=%.0f%% run=%d seed=%u cap=%d hold=%d sent=%d net_lost=%d "
           "arrived=%d late=%ld evict=%ld dup=%ld dropped=%ld delivered=%d "
           "max_span=%d naive_rewind_ticks=%ld naive_rewind_ms=%.0f "
           "buffered_rewind_ticks=%ld final_x=%.2f\n",
           loss * 100.0, run, seed, cap, hold, out.sent, out.net_lost, out.arrived,
           out.late, out.evicted, out.duplicated, out.dropped, out.delivered,
           out.max_span, out.naive_rewind_ticks,
           (double)out.naive_rewind_ticks * DT * 1000.0,
           out.buffered_rewind_ticks, (double)out.final_x);
    printf("positions=");
    print_sequence(&r);
    printf("\n");
}

/* ---------------- self-tests ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void test_basic(void) {
    Reorder r;
    reorder_init(&r, 8, 2);
    check("bad_cap", reorder_insert(&(Reorder){ .cap = 0, .last_out = -1, .newest = -1 }, 1, 0.0f) == -1);
    check("bad_hold", reorder_insert(&(Reorder){ .cap = 4, .hold = -1, .last_out = -1, .newest = -1 }, 1, 0.0f) == -1);

    check("in_order_1", reorder_insert(&r, 1, 0.5f) == 0);
    check("in_order_2", reorder_insert(&r, 2, 1.0f) == 0);
    check("held_until_window", r.delivered == 0 && r.count == 2);
    check("in_order_3", reorder_insert(&r, 3, 1.5f) == 0);
    check("release_head", r.delivered == 1 && r.last_out == 1);
    reorder_flush(&r);
    check("flush_all", r.delivered == 3);
    check("monotonic", reorder_monotonic(&r));
    check("no_rewind", reorder_max_rewind(&r) == 0);
    check("final_pos", fabsf(r.out[r.out_n - 1].x - 1.5f) < 1e-6f);

    Reorder q;
    reorder_init(&q, 8, 4);
    reorder_insert(&q, 10, 10.0f);
    reorder_insert(&q, 8, 8.0f);          /* out of order, inside window */
    reorder_insert(&q, 9, 9.0f);
    reorder_flush(&q);
    check("reordered_sorted", q.out_n == 3 && q.out[0].tick == 8 &&
                              q.out[1].tick == 9 && q.out[2].tick == 10);
    check("reordered_no_drop", q.late_dropped == 0 && q.evicted == 0);
    check("reordered_monotonic", reorder_monotonic(&q) && reorder_max_rewind(&q) == 0);

    Reorder s;
    reorder_init(&s, 8, 2);
    reorder_insert(&s, 1, 1.0f);
    reorder_insert(&s, 5, 5.0f);
    reorder_insert(&s, 4, 4.0f);          /* newer than last_out, fine */
    check("late_drop", reorder_insert(&s, 1, 9.0f) == -1 && s.late_dropped == 1);
    reorder_flush(&s);
    check("late_no_rewind", reorder_monotonic(&s) && reorder_max_rewind(&s) == 0);

    Reorder u;
    reorder_init(&u, 8, 2);
    reorder_insert(&u, 4, 4.0f);
    check("dup_drop", reorder_insert(&u, 4, 9.0f) == -1 && u.duplicated == 1 &&
                      u.count == 1 && fabsf(u.f[0].x - 4.0f) < 1e-6f);
}

static void test_capacity(void) {
    Reorder r;
    reorder_init(&r, 2, 10);              /* undersized vs hold+1 = 11 */
    check("cap_insert1", reorder_insert(&r, 1, 1.0f) == 0);
    check("cap_insert2", reorder_insert(&r, 2, 2.0f) == 0);
    check("cap_full", r.count == 2 && r.delivered == 0);
    check("cap_insert3", reorder_insert(&r, 3, 3.0f) == 0);
    check("cap_evicted", r.evicted == 1 && r.count == 2);
    check("cap_oldest_gone", r.f[0].tick == 2 && r.f[1].tick == 3);
    check("cap_at_bound", r.count <= r.cap);
    reorder_flush(&r);
    check("cap_flush", r.delivered == 2 && r.last_out == 3);
    check("cap_monotonic", reorder_monotonic(&r) && reorder_max_rewind(&r) == 0);

    /* full buffer whose next frame belongs before a held one: drop the
       oldest, not the newcomer, and never rewind. */
    Reorder e;
    reorder_init(&e, 2, 10);
    reorder_insert(&e, 2, 2.0f);
    reorder_insert(&e, 5, 5.0f);
    check("evict_insert", reorder_insert(&e, 3, 3.0f) == 0 && e.evicted == 1);
    check("evict_kept_newer", e.count == 2 && e.f[0].tick == 3 && e.f[1].tick == 5);
    reorder_flush(&e);
    check("evict_monotonic", reorder_monotonic(&e) && reorder_max_rewind(&e) == 0);
}

static void test_stress(void) {
    Reorder r;
    reorder_init(&r, 8, 4);               /* hold+1 = 5, cap is enough */
    rng_seed(12345u);
    int sent = 0, rejected = 0;
    for (unsigned t = 0; t < 1000; t++) {
        if (rng_unit() < 0.2) continue;   /* 20% loss */
        if (reorder_insert(&r, t, (float)t * STEP) != 0) rejected++;
        sent++;
    }
    reorder_flush(&r);
    check("stress_sent", sent > 700);
    check("stress_accounted", r.delivered + r.late_dropped + r.evicted +
                              r.duplicated == (long)sent);
    check("stress_rejected", rejected == r.late_dropped + r.duplicated);
    check("stress_monotonic", reorder_monotonic(&r));
    check("stress_no_rewind", reorder_max_rewind(&r) == 0);
    check("stress_cap", r.evicted == 0);  /* cap >= hold+1 and unique ticks */
}

static void test_relation(void) {
    /* T = 100 ms, dt = 50 ms => D = floor(T/dt) = 2 ticks, cap = D+1 = 3. */
    int D = (int)floorf(0.100f / (float)DT + 1e-6f);
    check("relation_window", D == 2);
    check("relation_rewind_time", (double)D * DT <= 0.100 + 1e-9);
    check("relation_capacity", D + 1 == 3);

    /* span == D: absorbed, order repaired, no rewind */
    Reorder ok;
    reorder_init(&ok, D + 1, D);
    reorder_insert(&ok, 10, 5.0f);
    reorder_insert(&ok, 8, 4.0f);
    reorder_flush(&ok);
    check("relation_hold_absorbs", ok.late_dropped == 0 && ok.out[0].tick == 8 &&
                                   reorder_max_rewind(&ok) == 0);

    /* span > D: the frontier has already passed (10 released), so the
       frame would rewind more than T and is dropped instead. */
    Reorder bad;
    reorder_init(&bad, D + 1, D);
    reorder_insert(&bad, 12, 6.0f);
    reorder_insert(&bad, 10, 5.0f);       /* release tick <= newest-D */
    check("relation_frontier_released", bad.last_out == 10);
    check("relation_span_too_large_drops", reorder_insert(&bad, 9, 4.5f) == -1 &&
                                           bad.late_dropped == 1);
    reorder_flush(&bad);
    check("relation_no_rewind_after_drop", reorder_monotonic(&bad) &&
                                           reorder_max_rewind(&bad) == 0);

    /* gaps are fine: hold is measured against the watermark, not arrivals */
    Reorder gap;
    reorder_init(&gap, 3, 2);
    reorder_insert(&gap, 1, 1.0f);
    reorder_insert(&gap, 3, 3.0f);        /* releases 1 */
    reorder_insert(&gap, 2, 2.0f);        /* later arrival, still inside window */
    reorder_flush(&gap);
    check("gap_absorbs", gap.late_dropped == 0 && gap.out_n == 3 &&
                         gap.out[0].tick == 1 && gap.out[1].tick == 2 &&
                         gap.out[2].tick == 3);
    check("gap_monotonic", reorder_monotonic(&gap) && reorder_max_rewind(&gap) == 0);
}

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "--report") == 0) {
        /* Two configurations, both from D = floor(T/dt):
         *   undersized: T = 100 ms -> D = 2, cap = 3
         *   sized:      T = 250 ms -> D = 5, cap = 6 (covers jitter = 5 ticks)
         */
        struct { int hold, cap; double T; } cfg[2] = {
            { 2, 3, 0.100 }, { 5, 6, 0.250 }
        };
        double losses[3] = { 0.0, 0.10, 0.30 };
        printf("# reorder report: ticks=%d dt=%.2fs speed=%.1fu/s step=%.2fu "
               "jitter=%d ticks\n", TICKS, DT, SPEED, STEP, JITTER);
        for (int ci = 0; ci < 2; ci++) {
            printf("# config T=%.0fms hold=%d ticks (%.0fms) cap=%d\n",
                   cfg[ci].T * 1000.0, cfg[ci].hold,
                   cfg[ci].hold * DT * 1000.0, cfg[ci].cap);
            for (int li = 0; li < 3; li++) {
                for (int run = 1; run <= 3; run++) {
                    unsigned seed = 7000u + (unsigned)li * 101u + (unsigned)run;
                    report_run(losses[li], run, cfg[ci].cap, cfg[ci].hold, seed);
                }
            }
        }
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "--verify") == 0) {
        /* Property sweep over losses x seeds x hold, checking the three
           claims the design rests on:
           (a) delivered ticks are strictly increasing for every hold and
               every cap, including undersized ones (no rewind, ever);
           (b) hold >= max observed out-of-order span => zero drops;
           (c) every late drop happens when tick <= last_out, and since
               last_out <= newest - hold that frame is at least hold ticks
               behind the newest seen -- i.e. tolerance is never exceeded
               silently, it is either repaired or dropped and counted. */
        int bad = 0, cases = 0, zero_drop_ok = 0, zero_drop_cases = 0;
        int late_ok = 0, late_cases = 0;
        double losses[5] = { 0.0, 0.05, 0.10, 0.20, 0.30 };
        for (int li = 0; li < 5; li++) {
            for (unsigned seed = 1; seed <= 50; seed++) {
                Arrival arr[MAX_ARR];
                int span = 0;
                int n = build_stream(losses[li], seed, arr, &span);
                for (int hold = 0; hold <= span + 2; hold++) {
                    for (int cap = 1; cap <= hold + 2; cap++) {
                        Reorder r;
                        reorder_init(&r, cap, hold);
                        for (int i = 0; i < n; i++) {
                            long before = r.late_dropped;
                            long long newest = r.newest, last_out = r.last_out;
                            int rc = reorder_insert(&r, arr[i].tick, arr[i].x);
                            cases++;
                            if (!reorder_monotonic(&r) || reorder_max_rewind(&r) != 0) bad++;
                            if (rc == -1 && r.late_dropped != before) {
                                late_cases++;
                                int cond = last_out >= 0 &&
                                           (long long)arr[i].tick <= last_out &&
                                           newest - (long long)arr[i].tick >= hold;
                                if (cond) late_ok++;
                                else bad++;
                            }
                        }
                        reorder_flush(&r);
                        if (!reorder_monotonic(&r) || reorder_max_rewind(&r) != 0) bad++;
                        /* zero drops only promised when the window is big
                           enough AND the capacity can hold it: cap >= hold+1 */
                        if (hold >= span && cap >= hold + 1) {
                            zero_drop_cases++;
                            if (r.late_dropped + r.evicted + r.duplicated == 0) zero_drop_ok++;
                            else bad++;
                        }
                    }
                }
            }
        }
        printf("verify insert_cases=%d late_drops=%d late_ok=%d "
               "zero_drop_cases=%d zero_drop_ok=%d bad=%d\n",
               cases, late_cases, late_ok, zero_drop_cases, zero_drop_ok, bad);
        printf("verify_all_ok=%d\n", bad == 0);
        return bad == 0 ? 0 : 1;
    }
    if (argc > 1 && strcmp(argv[1], "--sweep") == 0) {
        printf("# hold sweep at loss=10%%, seed=7102, cap=hold+1\n");
        for (int hold = 0; hold <= 7; hold++) {
            RunResult rr;
            run_once(0.10, 7102u, hold + 1, hold, &rr);
            printf("hold=%d cap=%d late=%ld evict=%ld dropped=%ld delivered=%d "
                   "max_span=%d naive_rewind=%ld buffered_rewind=%ld final_x=%.2f\n",
                   hold, hold + 1, rr.late, rr.evicted, rr.dropped, rr.delivered,
                   rr.max_span, rr.naive_rewind_ticks, rr.buffered_rewind_ticks,
                   (double)rr.final_x);
        }
        printf("# capacity stress at hold=4 (D from T=200ms), loss=0%%, seed=7101\n");
        int caps[4] = { 1, 2, 3, 5 };      /* D+1 = 5 is the no-evict bound */
        for (int i = 0; i < 4; i++) {
            RunResult rr;
            run_once(0.0, 7101u, caps[i], 4, &rr);
            printf("hold=4 cap=%d late=%ld evict=%ld dropped=%ld delivered=%d "
                   "naive_rewind=%ld buffered_rewind=%ld final_x=%.2f\n",
                   caps[i], rr.late, rr.evicted, rr.dropped, rr.delivered,
                   rr.naive_rewind_ticks, rr.buffered_rewind_ticks, (double)rr.final_x);
        }
        printf("# no-loss latency/reorder trade-off, seed=7101\n");
        for (int hold = 0; hold <= JITTER; hold++) {
            RunResult rr;
            run_once(0.0, 7101u, hold + 1, hold, &rr);
            printf("hold=%d cap=%d dropped=%ld delivered=%d max_span=%d "
                   "naive_rewind=%ld buffered_rewind=%ld\n",
                   hold, hold + 1, rr.dropped, rr.delivered, rr.max_span,
                   rr.naive_rewind_ticks, rr.buffered_rewind_ticks);
        }
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "--naive") == 0) {
        /* same nine runs but positions as applied on arrival, unsorted */
        double losses[3] = { 0.0, 0.10, 0.30 };
        for (int li = 0; li < 3; li++) {
            for (int run = 1; run <= 3; run++) {
                unsigned seed = 7000u + (unsigned)li * 101u + (unsigned)run;
                Arrival arr[MAX_ARR];
                int span = 0;
                int n = build_stream(losses[li], seed, arr, &span);
                printf("naive loss=%.0f%% run=%d positions=", losses[li] * 100.0, run);
                for (int i = 0; i < n; i++)
                    printf("%s%.2f", i ? "," : "", (double)arr[i].x);
                printf("\n");
            }
        }
        return 0;
    }

    test_basic();
    test_capacity();
    test_stress();
    test_relation();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
