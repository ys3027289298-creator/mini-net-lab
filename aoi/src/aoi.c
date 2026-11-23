
/* aoi: grid-based area of interest with enter/leave diff and radius query.
 *
 * Rejected edits and bad queries are reported through the shared mnl sink
 * (module=AOI) with the exact offending field. A rejected entry never mutates
 * the world, so a batch with mixed good/bad entries still applies the good
 * ones. Legacy self-test output is byte-identical.
 */
#include "mnl.h"

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

_Static_assert(sizeof(World) <= sizeof(mnl_state_t), "World exceeds mnl_state_t");

static void world_init(World* w, float cell, float radius) {
    memset(w, 0, sizeof(*w));
    w->cell = cell;
    w->radius = radius;
}

static int world_add(World* w, int id, float x, float y, mnl_sink_t* sk) {
    if (w == NULL) return mnl_report(sk, MNL_MOD_AOI, MNL_F_WORLD, MNL_R_NULL);
    if (w->cell <= 0.0f) return mnl_report(sk, MNL_MOD_AOI,
                                           MNL_F_CELL, MNL_R_RANGE);
    if (!isfinite(w->cell)) return mnl_report(sk, MNL_MOD_AOI,
                                              MNL_F_CELL, MNL_R_NONFINITE);
    if (!isfinite(w->radius)) return mnl_report(sk, MNL_MOD_AOI,
                                                MNL_F_RADIUS, MNL_R_NONFINITE);
    if (w->radius < 0.0f) return mnl_report(sk, MNL_MOD_AOI,
                                            MNL_F_RADIUS, MNL_R_RANGE);
    if (!isfinite(x)) return mnl_report(sk, MNL_MOD_AOI, MNL_F_X, MNL_R_NONFINITE);
    if (!isfinite(y)) return mnl_report(sk, MNL_MOD_AOI, MNL_F_Y, MNL_R_NONFINITE);
    if (w->n >= MAX_ENT) return mnl_report(sk, MNL_MOD_AOI,
                                           MNL_F_WORLD, MNL_R_FULL);
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id)
        return mnl_report(sk, MNL_MOD_AOI, MNL_F_ID, MNL_R_DUPLICATE);
    Entity* e = &w->e[w->n++];
    e->id = id; e->x = x; e->y = y; e->alive = 1;
    return w->n - 1;
}

static int world_move(World* w, int id, float x, float y, mnl_sink_t* sk) {
    if (w == NULL) return mnl_report(sk, MNL_MOD_AOI, MNL_F_WORLD, MNL_R_NULL);
    if (!isfinite(x)) return mnl_report(sk, MNL_MOD_AOI, MNL_F_X, MNL_R_NONFINITE);
    if (!isfinite(y)) return mnl_report(sk, MNL_MOD_AOI, MNL_F_Y, MNL_R_NONFINITE);
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].x = x; w->e[i].y = y; return 0; }
    }
    return mnl_report(sk, MNL_MOD_AOI, MNL_F_ID, MNL_R_NOT_FOUND);
}

static int world_remove(World* w, int id, mnl_sink_t* sk) {
    if (w == NULL) return mnl_report(sk, MNL_MOD_AOI, MNL_F_WORLD, MNL_R_NULL);
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].alive = 0; return 0; }
    }
    return mnl_report(sk, MNL_MOD_AOI, MNL_F_ID, MNL_R_NOT_FOUND);
}

static MNL_MAYBE_UNUSED int cell_coord(float v, float cell) {
    int c = (int)floorf(v / cell);
    if (c < 0) c = 0;
    if (c >= GRID) c = GRID - 1;
    return c;
}

/* collect ids within radius of (x,y), excluding self id */
static int world_query(const World* w, int self, float x, float y,
                       int* out, int cap, mnl_sink_t* sk) {
    if (w == NULL) return mnl_report(sk, MNL_MOD_AOI, MNL_F_WORLD, MNL_R_NULL);
    if (out == NULL) return mnl_report(sk, MNL_MOD_AOI, MNL_F_QUERY, MNL_R_NULL);
    if (cap <= 0) return mnl_report(sk, MNL_MOD_AOI, MNL_F_CAP, MNL_R_RANGE);
    if (!isfinite(x)) return mnl_report(sk, MNL_MOD_AOI, MNL_F_X, MNL_R_NONFINITE);
    if (!isfinite(y)) return mnl_report(sk, MNL_MOD_AOI, MNL_F_Y, MNL_R_NONFINITE);
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
static int world_diff(const mnl_idset_t* prev, const mnl_idset_t* cur,
                      int* enter, int* leave, mnl_sink_t* sk) {
    if (prev == NULL) return mnl_report(sk, MNL_MOD_AOI, MNL_F_PREV, MNL_R_NULL);
    if (cur == NULL) return mnl_report(sk, MNL_MOD_AOI, MNL_F_CUR, MNL_R_NULL);
    if (enter == NULL || leave == NULL)
        return mnl_report(sk, MNL_MOD_AOI, MNL_F_DIFF, MNL_R_NULL);
    if (prev->n < 0) return mnl_report(sk, MNL_MOD_AOI, MNL_F_PREV, MNL_R_RANGE);
    if (cur->n < 0) return mnl_report(sk, MNL_MOD_AOI, MNL_F_CUR, MNL_R_RANGE);
    int en = 0, lv = 0;
    for (int i = 0; i < cur->n; i++) {
        int found = 0;
        for (int j = 0; j < prev->n; j++) if (prev->ids[j] == cur->ids[i]) { found = 1; break; }
        if (!found) en++;
    }
    for (int j = 0; j < prev->n; j++) {
        int found = 0;
        for (int i = 0; i < cur->n; i++) if (cur->ids[i] == prev->ids[j]) { found = 1; break; }
        if (!found) lv++;
    }
    *enter = en; *leave = lv;
    return 0;
}

/* ---- zero-copy batch runner ------------------------------------------- */

int mnl_aoi_run(mnl_state_t* st, mnl_sink_t* sk,
                const mnl_item_t* items, int n, mnl_result_t* res) {
    if (sk == NULL) return -1;
    if (st == NULL) return mnl_report(sk, MNL_MOD_AOI, MNL_F_WORLD, MNL_R_NULL);
    if (items == NULL || n < 0 || res == NULL)
        return mnl_report(sk, MNL_MOD_AOI, MNL_F_NONE, MNL_R_NULL);

    World* w = (World*)st;
    int failures = 0;
    for (int i = 0; i < n; i++) {
        mnl_sink_begin(sk, i);
        mnl_result_t* r = &res[i];
        memset(r, 0, sizeof(*r));
        const mnl_item_t* it = &items[i];

        int rc = -1;
        switch (it->op) {
        case MNL_AOI_INIT: {
            const mnl_aoi_config_t* cfg = (const mnl_aoi_config_t*)it->payload;
            if (cfg == NULL) {
                rc = mnl_report(sk, MNL_MOD_AOI, MNL_F_CONFIG, MNL_R_NULL);
            } else if (!isfinite(cfg->cell) || cfg->cell <= 0.0f) {
                rc = mnl_report(sk, MNL_MOD_AOI, MNL_F_CELL,
                                isfinite(cfg->cell) ? MNL_R_RANGE : MNL_R_NONFINITE);
            } else if (!isfinite(cfg->radius)) {
                rc = mnl_report(sk, MNL_MOD_AOI, MNL_F_RADIUS, MNL_R_NONFINITE);
            } else if (cfg->radius < 0.0f) {
                rc = mnl_report(sk, MNL_MOD_AOI, MNL_F_RADIUS, MNL_R_RANGE);
            } else {
                world_init(w, cfg->cell, cfg->radius);
                rc = 0;
            }
            break;
        }
        case MNL_AOI_ADD:
        case MNL_AOI_MOVE: {
            const mnl_point_t* pt = (const mnl_point_t*)it->payload;
            if (pt == NULL) {
                rc = mnl_report(sk, MNL_MOD_AOI, MNL_F_POINT, MNL_R_NULL);
            } else if (it->op == MNL_AOI_ADD) {
                /* point carrier is read in place, never copied */
                rc = world_add(w, pt->id, pt->x, pt->y, sk);
            } else {
                rc = world_move(w, pt->id, pt->x, pt->y, sk);
            }
            break;
        }
        case MNL_AOI_REMOVE:
            rc = world_remove(w, it->id, sk);
            break;
        case MNL_AOI_QUERY: {
            const mnl_point_t* ctr = (const mnl_point_t*)it->payload;
            if (ctr == NULL) {
                rc = mnl_report(sk, MNL_MOD_AOI, MNL_F_POINT, MNL_R_NULL);
            } else {
                rc = world_query(w, it->id, ctr->x, ctr->y,
                                 (int*)it->out, it->cap, sk);
            }
            break;
        }
        case MNL_AOI_DIFF: {
            /* payload=prev set, out=cur set (both caller-owned arrays) */
            const mnl_idset_t* prev = (const mnl_idset_t*)it->payload;
            mnl_idset_t* cur = (mnl_idset_t*)it->out;
            int enter = 0, leave = 0;
            rc = world_diff(prev, cur, &enter, &leave, sk);
            if (rc == 0) { r->ival = enter; r->ival2 = leave; }
            break;
        }
        default:
            rc = mnl_report(sk, MNL_MOD_AOI, MNL_F_NONE, MNL_R_BAD_OP);
            break;
        }

        r->rc = rc;
        if (rc < 0) {
            failures++;
            r->err = sk->errs[sk->count - 1];
        } else if (it->op != MNL_AOI_DIFF) {
            r->ival = rc;
        }
    }
    return failures;
}

#ifndef MNL_NO_MAIN
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    World w;
    world_init(&w, 10.0f, 15.0f);
    check("add_self", world_add(&w, 1, 0, 0, NULL) == 0);
    check("add_near", world_add(&w, 2, 5, 0, NULL) == 1);
    check("add_far", world_add(&w, 3, 100, 100, NULL) == 2);
    check("dup_rejected", world_add(&w, 2, 1, 1, NULL) == -1);

    int out[8], n;
    n = world_query(&w, 1, 0, 0, out, 8, NULL);
    check("query_near", n == 1 && out[0] == 2);

    world_move(&w, 2, 100, 100, NULL);
    n = world_query(&w, 1, 0, 0, out, 8, NULL);
    check("query_after_move", n == 0);

    int prev[2] = { 2, 3 }, cur[2] = { 3, 4 }, enter = 0, leave = 0;
    mnl_idset_t ps = { prev, 2 }, cs = { cur, 2 };
    world_diff(&ps, &cs, &enter, &leave, NULL);
    check("diff_counts", enter == 1 && leave == 1);

    check("remove_ok", world_remove(&w, 3, NULL) == 0);
    n = world_query(&w, 1, 200, 200, out, 8, NULL);
    check("removed_absent", n == 0);

    world_init(&w, 0.0f, 15.0f);
    check("bad_cell", world_add(&w, 1, 0, 0, NULL) == -1);
    world_init(&w, 10.0f, 15.0f);

    check("move_missing", world_move(&w, 42, 0, 0, NULL) == -1);
    check("cell_clamp", cell_coord(-100.0f, 10.0f) == 0 && cell_coord(1e6f, 10.0f) == GRID - 1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
#endif
