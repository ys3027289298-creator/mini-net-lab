
/* aoi_tier: speed-based player promotion with a relaxed AOI culling tier.
 *
 * Players moving faster than SPEED_THRESHOLD (units/tick) are promoted to a
 * high-speed tier. Normal players are culled against R_BASE; high-speed
 * players get one relaxed step (R_FAST = R_BASE + RELAX_STEP), so snapshots
 * on the AOI boundary are not dropped between ticks. The relaxed radius only
 * applies while the fast state lasts; every other player is unaffected.
 *
 * Self-test runs three trajectory groups once each (walk / run / vehicle),
 * records per-player snapshot coverage and culled-frame counts, aggregates
 * per group, and fails if any low-speed frame is mis-promoted. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define TICKS              80
#define MAP_BOUND          100.0f
#define R_BASE             30.0f   /* normal AOI culling radius */
#define RELAX_STEP         14.0f   /* one relaxed tier for fast players */
#define R_FAST             (R_BASE + RELAX_STEP)
#define SPEED_THRESHOLD    8.0f    /* strictly greater than this => fast */

#define GROUPS             3
#define PER_GROUP          2
#define MAX_PLAYERS        (GROUPS * PER_GROUP)
#define MAX_FALSE_FRAMES   64

typedef enum { TIER_NORMAL = 0, TIER_FAST = 1 } Tier;

typedef struct {
    int id;
    int group;
    float x;
    int dir;
    float speed_base;
    float speed_amp;
    unsigned seed;

    float last_speed;
    int last_tier;
    float speed_min;
    float speed_max;

    int included;      /* snapshots kept under the actual policy */
    int culled;        /* snapshots dropped under the actual policy */
    int culled_strict; /* counterfactual: dropped if R_BASE were always used */
    int promoted;      /* frames classified as high speed */
    int n_false;       /* mis-promoted frames (low-speed groups only) */
    int false_frames[MAX_FALSE_FRAMES];
} Player;

typedef struct {
    const char* name;
    float base;
    float amp;
} GroupDef;

static const GroupDef GROUPS_DEF[GROUPS] = {
    { "walk",     2.0f,  0.2f },   /* 1.8 .. 2.2 units/tick */
    { "run",      5.0f,  0.5f },   /* 4.5 .. 5.5 units/tick */
    { "vehicle", 12.0f,  1.2f },   /* 10.8 .. 13.2 units/tick */
};

static unsigned rng_next(unsigned* s) {
    *s = *s * 1103515245u + 12345u;
    return *s;
}

static float rng_uniform(unsigned* s) {
    return (float)(rng_next(s) >> 8) / (float)(1u << 24); /* [0, 1) */
}

/* promotion rule: strictly above the threshold; equal speed stays normal */
static Tier classify(float speed) {
    return speed > SPEED_THRESHOLD ? TIER_FAST : TIER_NORMAL;
}

/* snapshot culling: distance from the observer at the origin */
static int snapshot_included(float dist, Tier tier) {
    float radius = (tier == TIER_FAST) ? R_FAST : R_BASE;
    return dist <= radius;
}

static void player_init(Player* p, int id, int group, float x, unsigned seed) {
    memset(p, 0, sizeof(*p));
    p->id = id;
    p->group = group;
    p->x = x;
    p->dir = 1;
    p->speed_base = GROUPS_DEF[group].base;
    p->speed_amp = GROUPS_DEF[group].amp;
    p->seed = seed;
    p->speed_min = 1e30f;
}

/* advance one tick; returns the measured speed for the tick */
static float player_step(Player* p) {
    float speed = p->speed_base + (rng_uniform(&p->seed) * 2.0f - 1.0f) * p->speed_amp;
    float nx = p->x + speed * (float)p->dir;
    if (nx > MAP_BOUND)      { nx = 2.0f * MAP_BOUND - nx; p->dir = -1; }
    else if (nx < -MAP_BOUND) { nx = -2.0f * MAP_BOUND - nx; p->dir = 1; }
    p->x = nx;
    return speed;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Player ps[MAX_PLAYERS];
    int np = 0;
    /* separated starting lanes so every player crosses both AOI bands */
    const float starts[PER_GROUP] = { -90.0f, -70.0f };
    for (int g = 0; g < GROUPS; g++)
        for (int k = 0; k < PER_GROUP; k++) {
            int id = np + 1;
            player_init(&ps[np], id, g, starts[k],
                        0x9e3779b9u + (unsigned)(g * 7 + k) * 2654435761u);
            np++;
        }

    int g_included[GROUPS] = {0}, g_culled[GROUPS] = {0};
    int g_strict[GROUPS] = {0}, g_promoted[GROUPS] = {0};

    for (int tick = 1; tick <= TICKS; tick++) {
        for (int i = 0; i < np; i++) {
            Player* p = &ps[i];
            float speed = player_step(p);
            Tier tier = classify(speed);
            float dist = fabsf(p->x);

            p->last_speed = speed;
            p->last_tier = tier;
            if (speed < p->speed_min) p->speed_min = speed;
            if (speed > p->speed_max) p->speed_max = speed;

            int keep_strict = snapshot_included(dist, TIER_NORMAL);
            int keep_policy = snapshot_included(dist, tier);

            if (!keep_strict) { p->culled_strict++; g_strict[p->group]++; }
            if (keep_policy)  { p->included++;      g_included[p->group]++; }
            else              { p->culled++;        g_culled[p->group]++; }

            if (tier == TIER_FAST) {
                p->promoted++;
                g_promoted[p->group]++;
                /* walk/run are low-speed groups: any promotion is a misjudgement */
                if (p->group != 2 && p->n_false < MAX_FALSE_FRAMES)
                    p->false_frames[p->n_false++] = tick;
            }
        }
    }

    /* ---- per-player report ---- */
    for (int i = 0; i < np; i++) {
        Player* p = &ps[i];
        printf("player=%d group=%s speed_min=%.4f speed_max=%.4f covered=%d/%d "
               "coverage=%.4f culled=%d culled_strict=%d rescued=%d promoted=%d\n",
               p->id, GROUPS_DEF[p->group].name, p->speed_min, p->speed_max,
               p->included, TICKS, (double)p->included / TICKS,
               p->culled, p->culled_strict, p->culled_strict - p->culled, p->promoted);
    }

    /* ---- per-group aggregates, one run per group ---- */
    for (int g = 0; g < GROUPS; g++) {
        int frames = PER_GROUP * TICKS;
        printf("group=%s players=%d covered=%d/%d coverage=%.4f culled=%d "
               "culled_strict=%d rescued=%d promoted=%d\n",
               GROUPS_DEF[g].name, PER_GROUP, g_included[g], frames,
               (double)g_included[g] / frames, g_culled[g], g_strict[g],
               g_strict[g] - g_culled[g], g_promoted[g]);
    }

    /* ---- low-speed mis-promotion list: must be empty, frame by frame ---- */
    int false_total = 0;
    for (int i = 0; i < np; i++)
        if (ps[i].group != 2) false_total += ps[i].n_false;
    printf("lowspeed_false_promotions=%d\n", false_total);
    for (int i = 0; i < np; i++)
        for (int j = 0; j < ps[i].n_false; j++)
            printf("false_promotion player=%d tick=%d\n", ps[i].id, ps[i].false_frames[j]);
    check("lowspeed_no_false_promotion", false_total == 0);

    /* ---- behavioural assertions ---- */
    for (int i = 0; i < np; i++) {
        Player* p = &ps[i];
        check("coverage_accounting", p->included + p->culled == TICKS);
        if (p->group == 2) {
            /* vehicle: jittered minimum 10.8 stays above the 8.0 threshold */
            check("vehicle_promoted_every_frame", p->promoted == TICKS);
            /* relaxed tier must actually keep boundary frames the strict rule drops */
            check("vehicle_relaxed_rescues_frames", p->culled_strict > p->culled);
        } else {
            check("lowspeed_never_promoted", p->promoted == 0);
            /* never promoted => policy radius is always R_BASE, nothing changes */
            check("lowspeed_policy_equals_strict", p->culled == p->culled_strict);
        }
    }

    /* ---- culling predicate boundary semantics ---- */
    check("normal_radius_inclusive",  snapshot_included(R_BASE, TIER_NORMAL) == 1);
    check("normal_just_outside",      snapshot_included(R_BASE + 0.01f, TIER_NORMAL) == 0);
    check("fast_radius_inclusive",    snapshot_included(R_FAST, TIER_FAST) == 1);
    check("fast_just_outside",        snapshot_included(R_FAST + 0.01f, TIER_FAST) == 0);
    check("band_kept_for_fast_only",  snapshot_included(35.0f, TIER_FAST) == 1 &&
                                      snapshot_included(35.0f, TIER_NORMAL) == 0);

    /* ---- threshold semantics: equal speed is not promoted ---- */
    check("threshold_equal_is_normal", classify(SPEED_THRESHOLD) == TIER_NORMAL);
    check("threshold_above_is_fast",   classify(SPEED_THRESHOLD + 0.0001f) == TIER_FAST);
    check("threshold_below_is_normal", classify(SPEED_THRESHOLD - 0.0001f) == TIER_NORMAL);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
