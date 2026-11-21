
/* aoi: grid-based area of interest with enter/leave diff and radius query.
 *
 * Consistency verification: periodically the send set of every observer is
 * rebuilt from a full-broadcast baseline (every live entity inside the radius,
 * no capacity cut) and compared with the production AOI set. Differences are
 * located to frame, observer and entity. A run is PASS only when the diff is
 * exactly zero, missing entities are exactly zero, and the minimum sample count
 * was reached; otherwise it is FAIL or INSUFFICIENT with the sample deficit.
 * Verification work is counted separately from production work so an overly
 * frequent schedule shows up as reported overhead.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define GRID 32
#define MAX_ENT 16
#define MAX_DIFF 16
#define OVERHEAD_LIMIT_PCT 25

typedef struct { int id; float x, y; int alive; } Entity;

typedef struct {
    Entity e[MAX_ENT];
    int n;
    float cell;
    float radius;
} World;

static void world_init(World* w, float cell, float radius) {
    memset(w, 0, sizeof(*w));
    w->cell = cell;
    w->radius = radius;
}

static int world_add(World* w, int id, float x, float y) {
    if (w->n >= MAX_ENT) return -1;
    if (w->cell <= 0.0f) return -1;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return -1;
    Entity* e = &w->e[w->n++];
    e->id = id; e->x = x; e->y = y; e->alive = 1;
    return w->n - 1;
}

static int world_move(World* w, int id, float x, float y) {
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].x = x; w->e[i].y = y; return 0; }
    }
    return -1;
}

static int world_remove(World* w, int id) {
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].alive = 0; return 0; }
    }
    return -1;
}

static int cell_coord(float v, float cell) {
    int c = (int)floorf(v / cell);
    if (c < 0) c = 0;
    if (c >= GRID) c = GRID - 1;
    return c;
}

/* collect ids within radius of (x,y), excluding self id.
 * Returns the true visible count; writes at most cap entries to out. */
static int world_query(const World* w, int self, float x, float y, int* out, int cap) {
    int n = 0;
    float r2 = w->radius * w->radius;
    for (int i = 0; i < w->n; i++) {
        const Entity* e = &w->e[i];
        if (!e->alive || e->id == self) continue;
        float dx = e->x - x, dy = e->y - y;
        if (dx * dx + dy * dy <= r2) {
            if (n < cap) out[n] = e->id;
            n++;
        }
    }
    return n;
}

/* Full-broadcast reference set, computed independently of world_query so a
 * defect in the production path (filter drift, capacity cut, stale set) cannot
 * hide behind a shared code path. Every live non-self entity within the radius
 * is included, boundary inclusive. The distance test runs in double precision,
 * which is the ground-truth reading of "within radius": if production's float
 * rounding ever drops an entity exactly on the edge, that is reported as
 * missing on purpose, since a dropped edge entity is still an entity nobody
 * sees.
 * Returns the true count; writes at most cap entries to out. */
static int world_broadcast(const World* w, int self, float x, float y, int* out, int cap) {
    int n = 0;
    double r = (double)w->radius;
    for (int i = 0; i < w->n; i++) {
        const Entity* e = &w->e[i];
        if (e->alive == 0 || e->id == self) continue;
        double dx = (double)e->x - (double)x;
        double dy = (double)e->y - (double)y;
        if (dx * dx + dy * dy <= r * r) {
            if (n < cap) out[n] = e->id;
            n++;
        }
    }
    return n;
}

/* diff current visible set against previous one: counts enter/leave */
static void world_diff(const int* prev, int prevn, const int* cur, int curn, int* enter, int* leave) {
    int en = 0, lv = 0;
    for (int i = 0; i < curn; i++) {
        int found = 0;
        for (int j = 0; j < prevn; j++) if (prev[j] == cur[i]) { found = 1; break; }
        if (!found) en++;
    }
    for (int j = 0; j < prevn; j++) {
        int found = 0;
        for (int i = 0; i < curn; i++) if (cur[i] == prev[j]) { found = 1; break; }
        if (!found) lv++;
    }
    *enter = en; *leave = lv;
}

/* ------------------------------------------------------------------ */
/* consistency verification                                           */
/* ------------------------------------------------------------------ */

typedef enum { AOI_DIFF_MISSING = 0, AOI_DIFF_EXTRA = 1 } AoiDiffKind;

typedef struct {
    unsigned frame;
    int observer;
    int entity;
    AoiDiffKind kind;
} AoiDiff;

typedef struct {
    int interval;          /* verify every N frames (>= 1) */
    int min_samples;       /* verified frames required before PASS */
    int aoi_cap;           /* production per-observer capacity cut */
    unsigned frames;       /* frames advanced */
    int samples;           /* verified frames actually checked */
    int pass_samples;      /* verified frames with zero diff */
    int diff_samples;      /* verified frames with at least one diff */
    unsigned missing;      /* baseline members absent from AOI set */
    unsigned extra;        /* AOI members absent from baseline */
    unsigned diffs_total;  /* missing + extra over the whole run */
    AoiDiff diffs[MAX_DIFF];
    int ndiffs;            /* diff records kept (capped at MAX_DIFF) */
    double aoi_checks;     /* production entity scans, every frame */
    double verify_checks;  /* extra baseline scans, verified frames only */
} AoiVerify;

typedef enum { AOI_VERDICT_PASS, AOI_VERDICT_FAIL, AOI_VERDICT_INSUFFICIENT } AoiVerdict;

static void aoi_verify_init(AoiVerify* v, int interval, int min_samples, int aoi_cap) {
    memset(v, 0, sizeof(*v));
    v->interval = interval > 0 ? interval : 1;
    v->min_samples = min_samples > 0 ? min_samples : 1;
    v->aoi_cap = aoi_cap > 0 ? aoi_cap : 1;
}

/* Compare one observer's AOI set against the full-broadcast baseline.
 * Every difference is counted and, up to MAX_DIFF, located to its entity. */
static int aoi_verify_compare(AoiVerify* v, unsigned frame, int observer,
                              const int* baseline, int bn,
                              const int* aoi_set, int an) {
    int diffs = 0;
    for (int i = 0; i < bn; i++) {
        int found = 0;
        for (int j = 0; j < an; j++) if (aoi_set[j] == baseline[i]) { found = 1; break; }
        if (!found) {
            diffs++;
            v->missing++;
            if (v->ndiffs < MAX_DIFF) {
                AoiDiff* d = &v->diffs[v->ndiffs++];
                d->frame = frame; d->observer = observer;
                d->entity = baseline[i]; d->kind = AOI_DIFF_MISSING;
            }
        }
    }
    for (int j = 0; j < an; j++) {
        int found = 0;
        for (int i = 0; i < bn; i++) if (baseline[i] == aoi_set[j]) { found = 1; break; }
        if (!found) {
            diffs++;
            v->extra++;
            if (v->ndiffs < MAX_DIFF) {
                AoiDiff* d = &v->diffs[v->ndiffs++];
                d->frame = frame; d->observer = observer;
                d->entity = aoi_set[j]; d->kind = AOI_DIFF_EXTRA;
            }
        }
    }
    v->diffs_total += (unsigned)diffs;
    return diffs;
}

/* Advance one frame. Production cost is paid every frame; the baseline check
 * only runs on the verification schedule. Returns 1 if this frame was verified. */
static int aoi_verify_frame(AoiVerify* v, const World* w, unsigned frame) {
    v->frames = frame + 1;

    int observers = 0;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive) observers++;
    v->aoi_checks += (double)observers * (double)w->n;

    if (frame % (unsigned)v->interval != 0) return 0;

    int baseline[MAX_ENT], aoi_set[MAX_ENT], diffs = 0;
    v->samples++;
    for (int o = 0; o < w->n; o++) {
        if (!w->e[o].alive) continue;
        int bn = world_broadcast(w, w->e[o].id, w->e[o].x, w->e[o].y, baseline, MAX_ENT);
        int an = world_query(w, w->e[o].id, w->e[o].x, w->e[o].y, aoi_set, v->aoi_cap);
        if (an > v->aoi_cap) an = v->aoi_cap;      /* only cap entries were kept */
        v->verify_checks += (double)w->n;          /* extra full-broadcast scan */
        diffs += aoi_verify_compare(v, frame, w->e[o].id, baseline, bn, aoi_set, an);
    }
    if (diffs == 0) v->pass_samples++; else v->diff_samples++;
    return 1;
}

static AoiVerdict aoi_verify_verdict(const AoiVerify* v) {
    if (v->missing > 0 || v->extra > 0) return AOI_VERDICT_FAIL;
    if (v->samples < v->min_samples) return AOI_VERDICT_INSUFFICIENT;
    return AOI_VERDICT_PASS;
}

static int aoi_verify_deficit(const AoiVerify* v) {
    int d = v->min_samples - v->samples;
    return d > 0 ? d : 0;
}

static int aoi_verify_overhead_pct(const AoiVerify* v) {
    if (v->aoi_checks <= 0.0) return 0;
    return (int)(v->verify_checks * 100.0 / v->aoi_checks + 0.5);
}

static int aoi_verify_overhead_high(const AoiVerify* v) {
    return aoi_verify_overhead_pct(v) > OVERHEAD_LIMIT_PCT;
}

static const char* aoi_verify_verdict_name(AoiVerdict v) {
    switch (v) {
    case AOI_VERDICT_PASS: return "pass";
    case AOI_VERDICT_FAIL: return "fail";
    case AOI_VERDICT_INSUFFICIENT: return "insufficient";
    default: return "unknown";
    }
}

static const char* aoi_diff_kind_name(AoiDiffKind k) {
    return k == AOI_DIFF_MISSING ? "missing" : "extra";
}

/* Machine-checkable report: verdict, sample coverage, located diffs, overhead. */
static void aoi_verify_report(const AoiVerify* v) {
    AoiVerdict verdict = aoi_verify_verdict(v);
    int pct = aoi_verify_overhead_pct(v);
    int deficit = aoi_verify_deficit(v);

    printf("verify_verdict=%s\n", aoi_verify_verdict_name(verdict));
    printf("verify_samples=%d min=%d deficit=%d pass=%d diff=%d\n",
           v->samples, v->min_samples, deficit, v->pass_samples, v->diff_samples);
    printf("verify_diffs=%u missing=%u extra=%u\n", v->diffs_total, v->missing, v->extra);
    for (int i = 0; i < v->ndiffs; i++) {
        const AoiDiff* d = &v->diffs[i];
        printf("verify_diff frame=%u observer=%d entity=%d kind=%s\n",
               d->frame, d->observer, d->entity, aoi_diff_kind_name(d->kind));
    }
    printf("verify_overhead_pct=%d limit=%d high=%d checks=%.0f aoi_checks=%.0f\n",
           pct, OVERHEAD_LIMIT_PCT, aoi_verify_overhead_high(v),
           v->verify_checks, v->aoi_checks);
    if (aoi_verify_overhead_high(v)) {
        printf("verify_overhead_note=interval %d spends %d%% on verification, "
               "over the %d%% limit; raise the interval\n",
               v->interval, pct, OVERHEAD_LIMIT_PCT);
    }
    if (deficit > 0) {
        printf("verify_sample_note=checked %d of %d required samples, %d short; "
               "run more verified frames before trusting the result\n",
               v->samples, v->min_samples, deficit);
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Put 1..4 near the origin and 8 far outside the radius. */
static void crowd_init(World* w) {
    world_init(w, 10.0f, 15.0f);
    world_add(w, 1, 0, 0);
    world_add(w, 2, 3, 0);
    world_add(w, 3, -3, 0);
    world_add(w, 4, 0, 3);
    world_add(w, 8, 100, 100);
}

int main(void) {
    World w;
    world_init(&w, 10.0f, 15.0f);
    check("add_self", world_add(&w, 1, 0, 0) == 0);
    check("add_near", world_add(&w, 2, 5, 0) == 1);
    check("add_far", world_add(&w, 3, 100, 100) == 2);
    check("dup_rejected", world_add(&w, 2, 1, 1) == -1);

    int out[8], n;
    n = world_query(&w, 1, 0, 0, out, 8);
    check("query_near", n == 1 && out[0] == 2);

    world_move(&w, 2, 100, 100);
    n = world_query(&w, 1, 0, 0, out, 8);
    check("query_after_move", n == 0);

    int prev[2] = { 2, 3 }, cur[2] = { 3, 4 }, enter = 0, leave = 0;
    world_diff(prev, 2, cur, 2, &enter, &leave);
    check("diff_counts", enter == 1 && leave == 1);

    check("remove_ok", world_remove(&w, 3) == 0);
    n = world_query(&w, 1, 200, 200, out, 8);
    check("removed_absent", n == 0);

    world_init(&w, 0.0f, 15.0f);
    check("bad_cell", world_add(&w, 1, 0, 0) == -1);
    world_init(&w, 10.0f, 15.0f);

    check("move_missing", world_move(&w, 42, 0, 0) == -1);
    check("cell_clamp", cell_coord(-100.0f, 10.0f) == 0 && cell_coord(1e6f, 10.0f) == GRID - 1);

    /* --- verification: no production cut must match the baseline exactly --- */
    World vw;
    AoiVerify v;
    crowd_init(&vw);
    aoi_verify_init(&v, 1, 4, MAX_ENT);
    for (unsigned f = 0; f < 6; f++) {
        if (f == 2) world_move(&vw, 2, 4, 0);   /* still inside */
        if (f == 4) world_move(&vw, 4, 90, 0);  /* leaves radius */
        aoi_verify_frame(&v, &vw, f);
    }
    check("verify_zero_diff", v.diffs_total == 0 && v.missing == 0 && v.extra == 0);
    check("verify_pass", aoi_verify_verdict(&v) == AOI_VERDICT_PASS);
    check("verify_samples_met", v.samples == 6);
    aoi_verify_report(&v);

    /* --- baseline is independent of the production path, and sound --- */
    {
        World bw;
        world_init(&bw, 10.0f, 15.0f);
        world_add(&bw, 1, 0, 0);
        world_add(&bw, 2, 15, 0);    /* exactly on the radius: inclusive */
        world_add(&bw, 3, 15.001f, 0);
        int q[MAX_ENT], b[MAX_ENT];
        int qn = world_query(&bw, 1, 0, 0, q, MAX_ENT);
        int bn = world_broadcast(&bw, 1, 0, 0, b, MAX_ENT);
        check("baseline_matches_query", qn == bn && bn == 1 && q[0] == 2 && b[0] == 2);

        /* A set that wrongly drops the boundary entity is caught even though
         * world_query itself would have returned it: the baseline is not the
         * production function reading its own output back. */
        AoiVerify bv;
        aoi_verify_init(&bv, 1, 1, MAX_ENT);
        int broken[1] = { 9 };
        int d = aoi_verify_compare(&bv, 0, 1, b, bn, broken, 1);
        check("baseline_catches_defect", d == 2 && bv.missing == 1 && bv.extra == 1);
        check("baseline_defect_located", bv.diffs[0].entity == 2 &&
                                         bv.diffs[0].kind == AOI_DIFF_MISSING);

        /* The two paths agree on an ordinary dataset; where they diverge the
         * double reading is authoritative and shows up as a located diff. */
        World ew;
        world_init(&ew, 0.1f, 0.1f);
        world_add(&ew, 1, 0, 0);
        world_add(&ew, 2, 0.7f, 0);
        world_add(&ew, 3, 0.3f, 0);
        int qe[MAX_ENT], be[MAX_ENT];
        int qen = world_query(&ew, 1, 0, 0, qe, MAX_ENT);
        int ben = world_broadcast(&ew, 1, 0, 0, be, MAX_ENT);
        check("baseline_paths_agree", qen == ben);
    }

    /* --- a production cap that truncates is caught, located per entity --- */
    crowd_init(&vw);
    aoi_verify_init(&v, 1, 2, 1);
    aoi_verify_frame(&v, &vw, 10);
    check("verify_truncation_missing", v.missing == 8 && v.extra == 0);
    check("verify_truncation_fail", aoi_verify_verdict(&v) == AOI_VERDICT_FAIL);
    check("verify_diff_located", v.diffs[0].frame == 10 && v.diffs[0].observer == 1 &&
                                 v.diffs[0].entity == 3 && v.diffs[0].kind == AOI_DIFF_MISSING);
    aoi_verify_report(&v);

    /* --- injected sets exercise the extra branch directly --- */
    AoiVerify ev;
    aoi_verify_init(&ev, 1, 1, 4);
    int base_set[3] = { 1, 2, 3 }, aoi_set[3] = { 1, 2, 9 };
    int d = aoi_verify_compare(&ev, 77, 5, base_set, 3, aoi_set, 3);
    check("verify_compare_extra", d == 2 && ev.missing == 1 && ev.extra == 1 && ev.diffs_total == 2);
    check("verify_missing_located", ev.diffs[0].frame == 77 && ev.diffs[0].observer == 5 &&
                                    ev.diffs[0].entity == 3 && ev.diffs[0].kind == AOI_DIFF_MISSING);
    check("verify_extra_located", ev.diffs[1].frame == 77 && ev.diffs[1].observer == 5 &&
                                  ev.diffs[1].entity == 9 && ev.diffs[1].kind == AOI_DIFF_EXTRA);

    /* --- too few samples is reported, not silently passed --- */
    crowd_init(&vw);
    aoi_verify_init(&v, 1, 5, MAX_ENT);
    for (unsigned f = 0; f < 3; f++) aoi_verify_frame(&v, &vw, f);
    check("verify_insufficient", aoi_verify_verdict(&v) == AOI_VERDICT_INSUFFICIENT);
    check("verify_deficit_reported", v.samples == 3 && aoi_verify_deficit(&v) == 2);
    aoi_verify_report(&v);

    /* --- schedule overhead: same world, 1-frame vs 8-frame interval --- */
    crowd_init(&vw);
    AoiVerify fast, slow;
    aoi_verify_init(&fast, 1, 1, MAX_ENT);
    aoi_verify_init(&slow, 8, 1, MAX_ENT);
    for (unsigned f = 0; f < 16; f++) {
        aoi_verify_frame(&fast, &vw, f);
        aoi_verify_frame(&slow, &vw, f);
    }
    check("verify_fast_high_overhead", aoi_verify_overhead_high(&fast) &&
                                       aoi_verify_overhead_pct(&fast) == 100);
    check("verify_slow_overhead_ok", !aoi_verify_overhead_high(&slow) &&
                                     aoi_verify_overhead_pct(&slow) == 13);
    check("verify_slow_schedule", slow.samples == 2);
    check("verify_off_schedule_idle", slow.verify_checks == 50.0 && fast.verify_checks == 400.0);
    check("verify_both_clean", fast.diffs_total == 0 && slow.diffs_total == 0 &&
                               aoi_verify_verdict(&slow) == AOI_VERDICT_PASS);
    aoi_verify_report(&fast);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
