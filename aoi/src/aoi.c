
/* aoi: grid-based area of interest with enter/leave diff and radius query.
 *
 * Query time is a tick in the shared time base (ms).  A caller passes the
 * tick it observed; a tick that goes backwards is refused, so enter/leave
 * diffs are always computed between two ordered observations.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../../timebase/src/timebase.h"
#include "../../timebase/src/accept.h"

#define GRID 32
#define MAX_ENT 16

typedef struct { int id; float x, y; int alive; } Entity;

typedef struct {
    Entity e[MAX_ENT];
    int n;
    float cell;
    float radius;
    uint32_t tick;       /* last query tick accepted */
    int has_tick;
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
 * `tick` is the shared-base observation time; a tick older than the previous
 * query is refused (returns -1) so visibility diffs stay time-ordered. */
static int world_query_at(World* w, uint32_t tick, int self, float x, float y,
                          int* out, int cap) {
    if (w->has_tick && tick < w->tick) return -1;
    w->tick = tick;
    w->has_tick = 1;

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

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Acceptance: the world moves with the shared base.  Entity 3 sweeps in and
 * out of the interest radius on a schedule driven by the base tick; every
 * frame queries visibility at that tick and diffs against the previous frame,
 * so enter/leave must stay consistent over a long run. */
typedef struct {
    World w;
    int prev[8];
    int prevn;
    int queried;
    int bad;
    long enters, leaves;
} AoiAcc;

static void aoi_accept_reset(void* ctx) {
    AoiAcc* a = (AoiAcc*)ctx;
    memset(a, 0, sizeof(*a));
    world_init(&a->w, 10.0f, 15.0f);
    world_add(&a->w, 1, 0.0f, 0.0f);
    world_add(&a->w, 2, 5.0f, 0.0f);
    world_add(&a->w, 3, 100.0f, 100.0f);
}

static int aoi_accept_step(void* ctx, TimeBase* tb, const TBAcceptSpec* spec, uint32_t tick) {
    AoiAcc* a = (AoiAcc*)ctx;
    (void)spec; (void)tick;
    uint32_t base = 0;
    if (tb_now(tb, &base) != TB_OK) { a->bad++; return 1; }

    /* entity 3 sweeps through the interest radius once per second, driven by
     * the base tick: 5 (inside r=15) for 1 s, then 35 (outside) for 1 s. */
    float x = ((base / 1000u) % 2u == 0u) ? 5.0f : 35.0f;
    if (world_move(&a->w, 3, x, 0.0f) != 0) { a->bad++; return 2; }

    int out[8];
    int n = world_query_at(&a->w, base, 1, 0.0f, 0.0f, out, 8);
    if (n < 0) { a->bad++; return 3; }   /* time must not go backwards */
    if (n > 8) n = 8;

    if (a->queried) {
        int en = 0, lv = 0;
        world_diff(a->prev, a->prevn, out, n, &en, &lv);
        a->enters += en; a->leaves += lv;
        /* Only entity 3 moves, so at most one enter and one leave per frame. */
        if (en > 1 || lv > 1) { a->bad++; return 4; }
    }
    memcpy(a->prev, out, sizeof(int) * (size_t)n);
    a->prevn = n;
    a->queried = 1;
    return 0;
}

static int aoi_accept(void) {
    AoiAcc acc;
    memset(&acc, 0, sizeof(acc));

    TBAcceptSpec spec;
    memset(&spec, 0, sizeof(spec));
    spec.module = TB_AOI;
    spec.step_ms = 50;             /* 20 Hz AOI refresh */
    spec.events = 2000;            /* 100 s */
    spec.long_events = 144000;     /* 2 h */
    spec.ppm = 100;
    spec.seed = 0xa01u;
    spec.step = aoi_accept_step;
    spec.reset = aoi_accept_reset;
    spec.ctx = &acc;

    int rc = tb_accept_run(&spec);
    printf("module=aoi phase=visibility enters=%ld leaves=%ld bad=%d ok=%d\n",
           acc.enters, acc.leaves, acc.bad, rc == 0 && acc.bad == 0);
    tb_accept_shared(TB_AOI);
    return (rc == 0 && acc.bad == 0) ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "--accept") == 0) return aoi_accept();

    World w;
    world_init(&w, 10.0f, 15.0f);
    check("add_self", world_add(&w, 1, 0, 0) == 0);
    check("add_near", world_add(&w, 2, 5, 0) == 1);
    check("add_far", world_add(&w, 3, 100, 100) == 2);
    check("dup_rejected", world_add(&w, 2, 1, 1) == -1);

    int out[8], n;
    n = world_query_at(&w, 0, 1, 0, 0, out, 8);
    check("query_near", n == 1 && out[0] == 2);

    world_move(&w, 2, 100, 100);
    n = world_query_at(&w, 1, 1, 0, 0, out, 8);
    check("query_after_move", n == 0);

    int prev[2] = { 2, 3 }, cur[2] = { 3, 4 }, enter = 0, leave = 0;
    world_diff(prev, 2, cur, 2, &enter, &leave);
    check("diff_counts", enter == 1 && leave == 1);

    check("remove_ok", world_remove(&w, 3) == 0);
    n = world_query_at(&w, 2, 1, 200, 200, out, 8);
    check("removed_absent", n == 0);

    world_init(&w, 0.0f, 15.0f);
    check("bad_cell", world_add(&w, 1, 0, 0) == -1);
    world_init(&w, 10.0f, 15.0f);

    check("move_missing", world_move(&w, 42, 0, 0) == -1);
    check("cell_clamp", cell_coord(-100.0f, 10.0f) == 0 && cell_coord(1e6f, 10.0f) == GRID - 1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
