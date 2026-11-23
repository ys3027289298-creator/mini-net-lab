
/* aoi: grid-based area of interest with enter/leave diff and radius query. */
#include "../../common/ftrace.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define GRID 32
#define MAX_ENT 16

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

/* collect ids within radius of (x,y), excluding self id */
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

#if FT_ENABLE
#include "../../common/ft_accept.h"

/* Unified-timing acceptance: deterministic exact scenarios plus a
 * real CLOCK_MONOTONIC frame loop exercising this module's workload. */
static int ft_run_aoi(void) {
    ft_deterministic_accept("aoi", "aoi");

    enum { FT_FRAMES = 64, FT_ITERS = 400 };
    ft_t t;
    ft_init(&t);
    volatile int sink = 0;

    for (int f = 0; f < FT_FRAMES; f++) {
        World w;
        world_init(&w, 10.0f, 15.0f);
        for (int id = 1; id <= MAX_ENT; id++) world_add(&w, id, (float)id, 0.0f);

        ft_frame_begin(&t);
        int outer = ft_span_start(&t, "aoi.scan_outer");   /* wraps the query */
            int mv = ft_span_start(&t, "aoi.move");
            for (int k = 0; k < FT_ITERS; k++) world_move(&w, 2, (float)(k % 7), 0.0f);
            ft_span_end(&t, mv);
            int q = ft_span_start(&t, "aoi.query");
            int out[16];
            for (int k = 0; k < FT_ITERS; k++) sink += world_query(&w, 1, 0, 0, out, 16);
            ft_span_end(&t, q);
        ft_span_end(&t, outer);
        int df = ft_span_start(&t, "aoi.diff");            /* disjoint from the scan */
        int enter = 0, leave = 0;
        int prev[4] = { 2, 3, 4, 5 }, cur[4] = { 3, 4, 5, 6 };
        for (int k = 0; k < FT_ITERS; k++) world_diff(prev, 4, cur, 4, &enter, &leave);
        sink += enter + leave;
        ft_span_end(&t, df);
        ft_frame_end(&t);
    }

    const FtReport* r = ft_report(&t);
    check("ft_aoi_real_frames", r->frames == FT_FRAMES);
    check("ft_aoi_real_overlap_positive", r->overlap_sum > 0.0);
    check("ft_aoi_real_gap_nonneg", r->gap_sum >= 0.0);
    check("ft_aoi_real_identity",
          fabs(r->diff_sum - (r->overlap_sum - r->gap_sum)) <= r->eps_sum);
    check("ft_aoi_real_bound", r->abs_diff_sum <= r->bound_sum + r->eps_sum);
    check("ft_aoi_real_verify", ft_verify(&t) == 0);
    ft_print_report(&t, "aoi real-clock workload", stdout);
    (void)sink;
    return ft_verify(&t) == 0 ? 0 : 1;
}
#endif

int main(void) {
#if FT_ENABLE
    if (ft_run_aoi() != 0) return 1;
#endif
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

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
