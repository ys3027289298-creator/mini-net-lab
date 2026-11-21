
/* priority: distance+facing tiered update rates with hysteresis and dwell;
   degenerate facing falls back to distance-only. Sim compares against
   uniform-rate updates: extrapolation error stays under an analytic bound,
   bandwidth and stutter (visible pop on apply) are reported. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define TIERS 3
#define MAX_ENT 16
#define PI 3.14159265f

static const float TH[TIERS - 1] = { 20.0f, 50.0f }; /* distance tier edges */
static const float MARGIN = 4.0f;                    /* hysteresis band around edges */
static const unsigned DWELL = 10;                    /* min ticks between tier changes */
static const int RATE[TIERS] = { 1, 2, 4 };          /* send one update every N ticks */
static const float COS_OFF = -0.1f, COS_ON = 0.1f;   /* facing demote hysteresis */
static const float VMAX = 2.0f;                      /* max entity speed, units/tick */
static const float STUTTER_TH = 3.0f;                /* correction pop counted as stutter */

typedef struct {
    int id;
    float x, y;          /* authoritative position */
    int tier;
    int demoted;         /* facing demote state, hysteretic */
    unsigned last_chg;   /* tick of last tier change */
    float cx, cy;        /* client: last received position */
    float cvx, cvy;      /* client: last received velocity */
    unsigned last_rx;    /* client: tick of last received update */
} Ent;

typedef struct {
    Ent e[MAX_ENT];
    int n;
    float vx, vy;        /* viewer position */
    float fx, fy;        /* viewer facing */
} World;

static int tier_distance(float d) {
    if (d < TH[0]) return 0;
    if (d < TH[1]) return 1;
    return 2;
}

/* facing demote with sign hysteresis; degenerate facing or zero offset
   falls back to distance-only (no demote) */
static int facing_demote(Ent* e, float fx, float fy, float dx, float dy) {
    float fl = sqrtf(fx * fx + fy * fy);
    float dl = sqrtf(dx * dx + dy * dy);
    if (fl < 1e-3f || dl < 1e-3f) { e->demoted = 0; return 0; }
    float c = (fx * dx + fy * dy) / (fl * dl);
    if (e->demoted) { if (c > COS_ON) e->demoted = 0; }
    else            { if (c < COS_OFF) e->demoted = 1; }
    return e->demoted;
}

/* recompute tier: distance edges are hysteretic relative to the current tier,
   facing may demote one extra tier, changes commit only after DWELL ticks */
static void tier_update(World* w, Ent* e, unsigned tick) {
    float dx = e->x - w->vx, dy = e->y - w->vy;
    float d = sqrtf(dx * dx + dy * dy);
    int t = e->tier;
    if (t > 0 && d < TH[t - 1] - MARGIN) t--;
    else if (t < TIERS - 1 && d > TH[t] + MARGIN) t++;
    t += facing_demote(e, w->fx, w->fy, dx, dy);
    if (t > TIERS - 1) t = TIERS - 1;
    if (t != e->tier && tick - e->last_chg >= DWELL) {
        e->tier = t;
        e->last_chg = tick;
    }
}

/* linear extrapolation, worst case is a full velocity reversal right after
   the last update: |v_old - v_new| * age <= 2 * VMAX * interval */
static float err_bound(int tier) {
    return 2.0f * VMAX * (float)RATE[tier];
}

typedef struct {
    unsigned long updates;   /* bandwidth proxy */
    unsigned long samples;   /* entity-ticks measured */
    double err_sum;
    float err_max;
    unsigned long stutters;  /* applies whose correction pop exceeded STUTTER_TH */
} Stats;

/* deterministic orbit: center on a ray at distance 10+9i, radius 5,
   angular speed 0.3 -> speed 1.5 <= VMAX */
static void traj(int i, unsigned t, float* x, float* y, float* vx, float* vy) {
    float ca = (float)i * (PI / 4.0f);
    float cd = 10.0f + 9.0f * (float)i;
    float r = 5.0f, w = 0.3f;
    float ph = w * (float)t + (float)i;
    *x = cd * cosf(ca) + r * cosf(ph);
    *y = cd * sinf(ca) + r * sinf(ph);
    *vx = -r * w * sinf(ph);
    *vy =  r * w * cosf(ph);
}

static Stats sim_run(int prioritized, unsigned ticks) {
    Stats s;
    World w;
    memset(&s, 0, sizeof(s));
    memset(&w, 0, sizeof(w));
    for (int i = 0; i < 8; i++) {
        Ent* e = &w.e[w.n++];
        float x, y, vx, vy;
        e->id = i + 1;
        traj(i, 0, &x, &y, &vx, &vy);
        e->x = x; e->y = y;
        e->tier = prioritized ? tier_distance(sqrtf(x * x + y * y)) : 0;
        e->cx = x; e->cy = y; e->cvx = vx; e->cvy = vy;
        e->last_rx = 0;
    }
    for (unsigned t = 1; t <= ticks; t++) {
        w.fx = cosf(0.01f * (float)t);   /* slowly rotating view */
        w.fy = sinf(0.01f * (float)t);
        for (int i = 0; i < w.n; i++) {
            Ent* e = &w.e[i];
            float x, y, vx, vy;
            traj(i, t, &x, &y, &vx, &vy);
            e->x = x; e->y = y;
            if (prioritized) tier_update(&w, e, t);
            int rate = prioritized ? RATE[e->tier] : 1;
            /* client render position = extrapolation from last received state */
            float age = (float)(t - e->last_rx);
            float ex = e->cx + e->cvx * age - x;
            float ey = e->cy + e->cvy * age - y;
            float err = sqrtf(ex * ex + ey * ey);
            s.err_sum += err;
            if (err > s.err_max) s.err_max = err;
            s.samples++;
            if ((t + (unsigned)e->id) % (unsigned)rate == 0) {
                if (err > STUTTER_TH) s.stutters++;
                e->cx = x; e->cy = y; e->cvx = vx; e->cvy = vy;
                e->last_rx = t;
                s.updates++;
            }
        }
    }
    return s;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    World w;
    memset(&w, 0, sizeof(w));
    w.fx = 1.0f; w.fy = 0.0f;

    Ent near_front = { .id = 1, .x = 10, .y = 0, .tier = 0 };
    tier_update(&w, &near_front, 100);
    check("near_front_tier0", near_front.tier == 0);

    Ent near_back = { .id = 2, .x = -10, .y = 0, .tier = 0 };
    tier_update(&w, &near_back, 100);
    check("near_back_demoted", near_back.tier == 1 && near_back.demoted == 1);

    Ent far = { .id = 3, .x = 80, .y = 0, .tier = 0 };
    for (unsigned t = 100; t < 130; t++) tier_update(&w, &far, t);
    check("far_tier2", far.tier == 2);

    /* degenerate facing -> distance only, even directly behind */
    w.fx = 0.0f; w.fy = 0.0f;
    Ent deg = { .id = 4, .x = -10, .y = 0, .tier = 0 };
    tier_update(&w, &deg, 200);
    check("degenerate_facing_distance_only", deg.tier == 0 && deg.demoted == 0);

    /* entity on top of viewer -> no demote */
    w.fx = 1.0f;
    Ent ontop = { .id = 5, .x = 0, .y = 0, .tier = 0 };
    tier_update(&w, &ontop, 200);
    check("zero_offset_no_demote", ontop.tier == 0);

    /* distance oscillating inside the margin band never flips tier */
    w.fx = 1.0f; w.fy = 0.0f;
    Ent osc = { .id = 6, .tier = 1 };
    int changes = 0;
    for (unsigned t = 0; t < 200; t++) {
        int prev = osc.tier;
        osc.x = TH[0] + 2.0f * sinf(0.5f * (float)t);
        osc.y = 0.0f;
        tier_update(&w, &osc, t + 300);
        if (osc.tier != prev) changes++;
    }
    check("hysteresis_no_flap", changes == 0);

    /* dwell: a clean crossing is not applied before DWELL ticks */
    Ent dw = { .id = 7, .x = 10, .y = 0, .tier = 0, .last_chg = 0 };
    dw.x = 30;
    tier_update(&w, &dw, 5);
    check("dwell_blocks_early", dw.tier == 0);
    tier_update(&w, &dw, 5 + DWELL);
    check("dwell_allows_after", dw.tier == 1);

    check("bound_order", err_bound(0) < err_bound(1) && err_bound(1) < err_bound(2));

    Stats un = sim_run(0, 600);
    Stats pr = sim_run(1, 600);
    printf("mode=uniform  updates=%lu mean_err=%.3f max_err=%.3f bound=%.1f stutters=%lu\n",
           un.updates, un.err_sum / un.samples, un.err_max, err_bound(0), un.stutters);
    printf("mode=priority updates=%lu mean_err=%.3f max_err=%.3f bound=%.1f stutters=%lu\n",
           pr.updates, pr.err_sum / pr.samples, pr.err_max, err_bound(TIERS - 1), pr.stutters);
    printf("bandwidth_ratio=%.3f\n", (double)pr.updates / (double)un.updates);

    check("priority_saves_bandwidth", pr.updates * 2 <= un.updates);
    check("uniform_err_bounded", un.err_max <= err_bound(0) + 1e-3f);
    check("priority_err_bounded", pr.err_max <= err_bound(TIERS - 1) + 1e-3f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
