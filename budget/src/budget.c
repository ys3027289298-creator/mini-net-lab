
/* budget: rollback budget — cost model from sync rate and prediction error,
 * per-frame depth cap, degrade to interpolation when over budget, reject
 * when no budget is left. A budgeted engine and an unlimited engine replay
 * the same pre-generated event stream; an analytic state-error upper bound
 * is asserted every tick, and rollback-depth histograms are kept for
 * requested and rejected/degraded corrections. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define BUF 64
#define HIST_BINS 7
#define HIST_EDGES {0, 1, 2, 4, 8, 16, 32}
#define DRIFT 2.0f             /* worst-case per-tick gap growth: |v| <= 1 */

typedef struct { unsigned tick; float x; } Cell;

/* ---------- budget decision core ---------- */

typedef struct {
    float tokens;
    float cap;
    float refill;          /* tokens added per frame */
    int   max_depth;       /* hard per-frame depth cap */
    float err_scale;       /* prediction-error weight in the cost model */
    float sync_ema;
    float err_ema;
    int   requests;
    int   rollbacks;
    int   interpolations;
    int   rejects;
    int   hist_req[HIST_BINS];
    int   hist_rej[HIST_BINS];
} Budget;

#define DEC_OK      0      /* full rollback performed */
#define DEC_INTERP  1      /* over budget/deep: degraded to interpolation */
#define DEC_REJECT  2      /* no tokens: correction refused entirely */

static int budget_init(Budget* b, float refill, float cap, int max_depth,
                       float err_scale) {
    if (refill < 0.0f || cap < 0.0f || refill > cap ||
        max_depth < 1 || err_scale < 0.0f)
        return -1;
    memset(b, 0, sizeof(*b));
    b->refill = refill;
    b->cap = cap;
    b->max_depth = max_depth;
    b->err_scale = err_scale;
    b->tokens = cap;
    b->sync_ema = 1.0f;    /* optimistic before the first sample */
    return 0;
}

static void budget_refill(Budget* b) {
    b->tokens += b->refill;
    if (b->tokens > b->cap) b->tokens = b->cap;
}

/* estimator inputs: sync_rate in [0,1], e >= 0 prediction error */
static void budget_observe(Budget* b, float sync_rate, float e) {
    if (sync_rate < 0.0f || sync_rate > 1.0f || e < 0.0f) return;
    b->sync_ema = 0.9f * b->sync_ema + 0.1f * sync_rate;
    b->err_ema  = 0.9f * b->err_ema  + 0.1f * e;
}

/* expected cost of a rollback of `depth` ticks given current observations:
 * deeper, less-synced, higher-error states cost more */
static float budget_cost(const Budget* b, int depth) {
    float s = b->sync_ema > 0.05f ? b->sync_ema : 0.05f;
    return (float)depth * (1.0f + b->err_scale * b->err_ema) / s;
}

/* expected per-ack-slot cost from a prediction model: mismatch probability
 * times the cost of correcting at the expected depth */
static float budget_expected(const Budget* b, float mismatch_prob,
                             int expect_depth) {
    if (mismatch_prob < 0.0f || mismatch_prob > 1.0f || expect_depth < 0)
        return -1.0f;
    return mismatch_prob * budget_cost(b, expect_depth < 1 ? 1
                                                           : expect_depth);
}

static int hist_bin(int depth) {
    static const int edges[HIST_BINS] = HIST_EDGES;
    int bin = 0;
    for (int i = 1; i < HIST_BINS; i++)
        if (depth >= edges[i]) bin = i; else break;
    return bin;
}

/* decide what happens to one correction request and update the ledger.
 * returns DEC_OK / DEC_INTERP / DEC_REJECT. */
#define INTERP_FRACTION 0.4f   /* degraded correction pays this share of cost */

static int budget_decide(Budget* b, int depth) {
    if (depth < 1) return -1;
    b->requests++;
    b->hist_req[hist_bin(depth)]++;

    float cost = budget_cost(b, depth);
    if (b->tokens <= 0.0f) {                 /* budget zero: refuse */
        b->rejects++;
        b->hist_rej[hist_bin(depth)]++;
        return DEC_REJECT;
    }
    if (depth <= b->max_depth && cost <= b->tokens) {
        b->tokens -= cost;                  /* full price, exact correction */
        b->rollbacks++;
        return DEC_OK;
    }
    /* over budget or deeper than the per-frame cap: degrade to a partial
     * interpolation that pays a fixed fraction of the full cost; if even
     * that fraction is unaffordable the request is refused outright */
    float spend = cost * INTERP_FRACTION;
    if (spend > b->tokens) {
        b->rejects++;
        b->hist_rej[hist_bin(depth)]++;
        return DEC_REJECT;
    }
    b->tokens -= spend;
    b->interpolations++;
    b->hist_rej[hist_bin(depth)]++;
    return DEC_INTERP;
}

/* ---------- 1-D prediction buffer shared by both engines ---------- */

typedef struct {
    Cell c[BUF];
    int head;
    int count;
} Ring;

static void ring_push(Ring* r, unsigned tick, float x) {
    r->c[r->head].tick = tick;
    r->c[r->head].x = x;
    r->head = (r->head + 1) % BUF;
    if (r->count < BUF) r->count++;
}

static Cell* ring_find(Ring* r, unsigned tick) {
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->c[idx].tick == tick) return &r->c[idx];
    }
    return NULL;
}

/* full correction: overwrite the acked tick with authoritative truth and
 * replay later ticks with each tick's predicted velocity */
static void engine_rollback(Ring* r, unsigned tick, float auth,
                            const float* vel, int now) {
    Cell* cell = ring_find(r, tick);
    if (!cell) return;
    cell->x = auth;
    int depth = now - (int)tick;
    for (int k = 1; k <= depth; k++) {
        Cell* nxt = ring_find(r, tick + (unsigned)k);
        Cell* prv = ring_find(r, tick + (unsigned)(k - 1));
        if (!nxt || !prv) break;
        nxt->x = prv->x + vel[tick + k];
    }
}

/* degraded correction: ease the current state toward where a full replay
 * would have landed, keeping the direction of the correction */
static void engine_interp(Ring* r, unsigned tick, float auth,
                          const float* vel, int now, float beta) {
    Cell* cell = ring_find(r, tick);
    if (!cell) return;
    float target = auth;
    for (int tt = (int)tick; tt < now; tt++) target += vel[tt + 1];
    Cell* cur = ring_find(r, (unsigned)now);
    if (cur) cur->x += beta * (target - cur->x);
}

/* ---------- simulation over one pre-generated event stream ---------- */

#define N 400
#define VEL_STEP 1.0f          /* |velocity| per tick <= 1 */
#define MIS_DRIFT 0.4f         /* max |client-truth| velocity on mismatch */
#define MISMATCH_P 0.25f       /* probability a tick's input is mispredicted */

typedef struct { unsigned seed; } Rng;

static float rng_next(Rng* g) {
    g->seed = g->seed * 1103515245u + 12345u;
    return (float)((g->seed >> 8) & 0xffffu) / 65536.0f;
}

typedef struct {
    Ring ring;
    Budget* b;               /* NULL = unlimited engine */
    float vel[N + 2];        /* client-predicted velocity per tick */
    float truth[N + 2];
    float bound;             /* online analytic error bound */
    int   max_err_milli;     /* max observed |state - truth|, x1000 */
    int   max_bound_milli;
    int   bound_violations;
} Engine;

static void engine_init(Engine* e, Budget* b) {
    memset(e, 0, sizeof(*e));
    e->b = b;
}

/* truth_v[t]  authoritative velocity at tick t
 * bad[t]      1 when the client mispredicted tick t's input
 * lag[t]      ticks before the ack for tick t is delivered */
static void run(Engine* e, const float* truth_v, const int* bad,
                const int* lag, int frames, float interp_beta) {
    for (int t = 0; t < frames; t++) {
        if (e->b) budget_refill(e->b);

        /* predict tick t: on a mismatch the client adds a small bounded
         * perturbation, so |dv| <= MIS_DRIFT <= DRIFT per tick */
        float cv = bad[t] ? truth_v[t]
                                + (rng_next(&(Rng){ 0x9e37u + (unsigned)t })
                                   - 0.5f) * 2.0f * MIS_DRIFT
                          : truth_v[t];
        e->vel[t] = cv;
        e->truth[t] = (t == 0 ? 0.0f : e->truth[t - 1]) + truth_v[t];
        Cell* prev = ring_find(&e->ring, (unsigned)(t - 1));
        ring_push(&e->ring, (unsigned)t,
                  (prev ? prev->x : 0.0f) + cv);

        /* worst-case bound step 1: one tick of prediction drift */
        if (e->b) e->bound += DRIFT;

        /* the ack for tick at lands this frame exactly when at+lag[at]=t */
        for (int at = 0; at <= t; at++) {
            if (at + lag[at] != t || !bad[at]) continue;
            int depth = t - at;
            if (depth < 1) continue;
            float auth = e->truth[at];
            Cell* acell = ring_find(&e->ring, (unsigned)at);
            if (!acell) continue;
            float err = fabsf(acell->x - auth);

            int dec;
            if (e->b) {
                /* time-varying network: alternating good/bad windows shift
                 * the cost model through the sync-rate term */
                float sync_now = ((t / 60) % 2 == 0) ? 0.90f : 0.55f;
                budget_observe(e->b, sync_now, err);
                dec = budget_decide(e->b, depth);
            } else {
                dec = DEC_OK;
            }

            if (dec == DEC_OK) {
                engine_rollback(&e->ring, (unsigned)at, auth, e->vel, t);
                /* replay residual: each replayed tick contributes <= DRIFT */
                if (e->b) e->bound = DRIFT * (float)depth;
            } else if (dec == DEC_INTERP) {
                engine_interp(&e->ring, (unsigned)at, auth, e->vel, t,
                              interp_beta);
                /* g' = (1-beta)g + beta r, |r| <= DRIFT*depth */
                if (e->b)
                    e->bound = (1.0f - interp_beta) * e->bound
                             + interp_beta * DRIFT * (float)depth;
            }
            /* DEC_REJECT: state untouched, bound already covers the gap */
        }

        /* verify the bound against the actual state at tick t */
        Cell* cur = ring_find(&e->ring, (unsigned)t);
        float gap = cur ? fabsf(cur->x - e->truth[t]) : 0.0f;
        int gi = (int)lrintf(gap * 1000.0f);
        if (gi > e->max_err_milli) e->max_err_milli = gi;
        if (e->b) {
            if (gap > e->bound + 1e-3f) e->bound_violations++;
            int bi = (int)lrintf(e->bound * 1000.0f);
            if (bi > e->max_bound_milli) e->max_bound_milli = bi;
        }
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* ---- unit tests on the decision core ---- */
    Budget b0;
    check("bad_init", budget_init(&b0, 5.0f, 1.0f, 2, 1.0f) == -1 &&
                     budget_init(&b0, -1.0f, 1.0f, 2, 1.0f) == -1 &&
                     budget_init(&b0, 0.0f, 0.0f, 0, 1.0f) == -1);

    Budget bc;
    budget_init(&bc, 2.0f, 8.0f, 4, 4.0f);
    budget_observe(&bc, 1.0f, 0.0f);
    float c1 = budget_cost(&bc, 2);
    budget_observe(&bc, 0.5f, 0.5f);          /* lower sync, higher error */
    float c2 = budget_cost(&bc, 2);
    check("cost_monotone", c2 > c1 && c1 > 0.0f);
    check("cost_depth", budget_cost(&bc, 4) > budget_cost(&bc, 1));
    check("expected_bad", budget_expected(&bc, -1.0f, 2) < 0.0f);
    check("expected_scales",
          budget_expected(&bc, 0.8f, 3) > budget_expected(&bc, 0.2f, 3));

    /* zero budget: everything refused, refilling never creates tokens */
    Budget bz;
    budget_init(&bz, 0.0f, 0.0f, 4, 1.0f);
    check("zero_reject1", budget_decide(&bz, 1) == DEC_REJECT);
    budget_refill(&bz);
    check("zero_reject2", budget_decide(&bz, 5) == DEC_REJECT);
    check("zero_ledger", bz.rejects == 2 && bz.rollbacks == 0 &&
                         bz.interpolations == 0 && bz.tokens == 0.0f);

    /* depth cap forces interpolation even with a full bucket */
    Budget bd;
    budget_init(&bd, 100.0f, 100.0f, 2, 0.0f);
    check("cap_interp", budget_decide(&bd, 5) == DEC_INTERP);
    check("cap_hist", bd.hist_rej[hist_bin(5)] == 1);

    /* affordable request spends tokens and rolls back; empty bucket refuses */
    Budget ba;
    budget_init(&ba, 0.0f, 1.0f, 8, 0.0f);
    check("afford_ok", budget_decide(&ba, 1) == DEC_OK);
    check("spent", ba.tokens == 0.0f && ba.rollbacks == 1);
    check("empty_refuse", budget_decide(&ba, 1) == DEC_REJECT);

    /* ---- shared event stream (generated once, replayed by both) ---- */
    static float truth_v[N + 2];
    static int bad[N + 2], lag[N + 2];
    Rng gen = { 777u };
    int max_lag_seen = 0;
    for (int t = 0; t < N; t++) {
        truth_v[t] = (rng_next(&gen) - 0.5f) * 2.0f * VEL_STEP;
        bad[t] = rng_next(&gen) < MISMATCH_P;
        /* jittered ack delay: mostly 4-8 ticks, occasionally 12-20 */
        float roll = rng_next(&gen);
        lag[t] = roll < 0.5f ? 4 : roll < 0.8f ? 6
               : roll < 0.95f ? 8 : roll < 0.99f ? 12
               : 16 + (int)(rng_next(&gen) * 5);
        if (lag[t] > max_lag_seen) max_lag_seen = lag[t];
    }

    Engine unlim, budg;
    Budget bb;
    engine_init(&unlim, NULL);
    engine_init(&budg, &bb);
    budget_init(&bb, 0.8f, 12.0f, 8, 1.0f);
    run(&unlim, truth_v, bad, lag, N, 0.25f);
    run(&budg, truth_v, bad, lag, N, 0.25f);

    check("requests_present", bb.requests > 0);
    check("budget_mix", bb.rollbacks > 0 && bb.interpolations > 0 &&
                        bb.rejects > 0);
    check("ledger_sums",
          bb.rollbacks + bb.interpolations + bb.rejects == bb.requests);

    int req_sum = 0, rej_sum = 0;
    for (int i = 0; i < HIST_BINS; i++) {
        req_sum += bb.hist_req[i];
        rej_sum += bb.hist_rej[i];
    }
    check("hist_req_total", req_sum == bb.requests);
    check("hist_rej_total", rej_sum == bb.interpolations + bb.rejects);
    check("budget_bound_held", budg.bound_violations == 0);

    /* unlimited engine: error can only live inside the lag window, and a
     * full replay leaves at most DRIFT per replayed tick */
    int unlim_struct_bound =
        (int)lrintf(DRIFT * (float)max_lag_seen * 1000.0f);
    check("unlim_structural_bound",
          unlim.max_err_milli <= unlim_struct_bound);

    /* zero-budget engine over the same stream: every correction refused */
    Engine ez;
    Budget bzz;
    engine_init(&ez, &bzz);
    budget_init(&bzz, 0.0f, 0.0f, 6, 1.0f);
    run(&ez, truth_v, bad, lag, N, 0.25f);
    check("zero_all_rejected", bzz.rejects == bzz.requests &&
                               bzz.rollbacks == 0 &&
                               bzz.interpolations == 0);
    check("zero_bound_held", ez.bound_violations == 0);

    /* ---- report ---- */
    static const char* labels[HIST_BINS] = {
        "0", "1", "2-3", "4-7", "8-15", "16-31", "32+"
    };
    printf("\n-- depth histogram (requested / rejected-or-degraded) --\n");
    for (int i = 0; i < HIST_BINS; i++)
        printf("depth_>=%s req=%d rej=%d\n",
               labels[i], bb.hist_req[i], bb.hist_rej[i]);

    printf("\n-- state error vs authoritative truth (x1000 units) --\n");
    printf("unlimited_max_err=%d structural_bound=%d\n",
           unlim.max_err_milli, unlim_struct_bound);
    printf("budget_max_err=%d analytic_bound_peak=%d\n",
           budg.max_err_milli, budg.max_bound_milli);
    printf("zero_budget_max_err=%d analytic_bound_peak=%d\n",
           ez.max_err_milli, ez.max_bound_milli);

    printf("\n-- budget ledger --\n");
    printf("requests=%d rollbacks=%d interpolations=%d rejects=%d\n",
           bb.requests, bb.rollbacks, bb.interpolations, bb.rejects);
    printf("zero_requests=%d zero_rejects=%d\n",
           bzz.requests, bzz.rejects);
    printf("sync_ema=%.4f err_ema=%.4f tokens_left=%.3f\n",
           bb.sync_ema, bb.err_ema, bb.tokens);
    check("estimator_alive", bb.sync_ema > 0.0f && bb.err_ema > 0.0f);

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
