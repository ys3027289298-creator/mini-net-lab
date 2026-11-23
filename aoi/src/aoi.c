
/* aoi: grid-based area of interest with enter/leave diff and radius query.
 *
 * Failure-isolation version: named aoi_err_t codes plus deterministic fault
 * hooks. World layout, query math and diff logic are unchanged.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "aoi.h"

void aoi_world_init(World* w, float cell, float radius) {
    if (w == NULL) return;
    memset(w, 0, sizeof(*w));
    w->cell = cell;
    w->radius = radius;
}

/* --- deterministic, process-local fault injection ----------------------- */
static aoi_fault_t g_aoi_fault = AOI_FAULT_NONE;

void aoi_inject(aoi_fault_t f) { g_aoi_fault = f; }
void aoi_reset_faults(void) { g_aoi_fault = AOI_FAULT_NONE; }
aoi_fault_t aoi_current_fault(void) { return g_aoi_fault; }

const char* aoi_err_name(int code) {
    switch (code) {
    case AOI_OK:             return "AOI_OK";
    case AOI_ERR_FULL:       return "AOI_ERR_FULL";
    case AOI_ERR_BAD_CELL:   return "AOI_ERR_BAD_CELL";
    case AOI_ERR_DUP_ID:     return "AOI_ERR_DUP_ID";
    case AOI_ERR_NOT_FOUND:  return "AOI_ERR_NOT_FOUND";
    case AOI_ERR_NULL:       return "AOI_ERR_NULL";
    case AOI_ERR_INJECT_ADD: return "AOI_ERR_INJECT_ADD";
    default:                 return "AOI_ERR_UNKNOWN";
    }
}

const char* aoi_fault_name(aoi_fault_t f) {
    switch (f) {
    case AOI_FAULT_NONE: return "AOI_FAULT_NONE";
    case AOI_FAULT_ADD:  return "AOI_FAULT_ADD";
    default:             return "AOI_FAULT_UNKNOWN";
    }
}

int aoi_world_add(World* w, int id, float x, float y) {
    if (w == NULL) return AOI_ERR_NULL;
    if (w->n >= MAX_ENT) return AOI_ERR_FULL;
    if (w->cell <= 0.0f) return AOI_ERR_BAD_CELL;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return AOI_ERR_DUP_ID;
    /* hook sits before any mutation: a fired fault cannot change the world */
    if (g_aoi_fault == AOI_FAULT_ADD) return AOI_ERR_INJECT_ADD;
    Entity* e = &w->e[w->n++];
    e->id = id; e->x = x; e->y = y; e->alive = 1;
    return w->n - 1;
}

int aoi_world_move(World* w, int id, float x, float y) {
    if (w == NULL) return AOI_ERR_NULL;
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].x = x; w->e[i].y = y; return 0; }
    }
    return AOI_ERR_NOT_FOUND;
}

int aoi_world_remove(World* w, int id) {
    if (w == NULL) return AOI_ERR_NULL;
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].alive = 0; return 0; }
    }
    return AOI_ERR_NOT_FOUND;
}

int aoi_cell_coord(float v, float cell) {
    int c = (int)floorf(v / cell);
    if (c < 0) c = 0;
    if (c >= GRID) c = GRID - 1;
    return c;
}

/* collect ids within radius of (x,y), excluding self id */
int aoi_world_query(const World* w, int self, float x, float y, int* out, int cap) {
    if (w == NULL || out == NULL || cap <= 0) return AOI_ERR_NULL;
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
void aoi_world_diff(const int* prev, int prevn, const int* cur, int curn, int* enter, int* leave) {
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

#ifdef MINILAB_SELFTEST
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int MINILAB_SELFTEST(void) {
    World w;
    aoi_world_init(&w, 10.0f, 15.0f);
    check("add_self", aoi_world_add(&w, 1, 0, 0) == 0);
    check("add_near", aoi_world_add(&w, 2, 5, 0) == 1);
    check("add_far", aoi_world_add(&w, 3, 100, 100) == 2);
    check("dup_rejected", aoi_world_add(&w, 2, 1, 1) == AOI_ERR_DUP_ID);

    int out[8], n;
    n = aoi_world_query(&w, 1, 0, 0, out, 8);
    check("query_near", n == 1 && out[0] == 2);

    aoi_world_move(&w, 2, 100, 100);
    n = aoi_world_query(&w, 1, 0, 0, out, 8);
    check("query_after_move", n == 0);

    int prev[2] = { 2, 3 }, cur[2] = { 3, 4 }, enter = 0, leave = 0;
    aoi_world_diff(prev, 2, cur, 2, &enter, &leave);
    check("diff_counts", enter == 1 && leave == 1);

    check("remove_ok", aoi_world_remove(&w, 3) == 0);
    n = aoi_world_query(&w, 1, 200, 200, out, 8);
    check("removed_absent", n == 0);

    aoi_world_init(&w, 0.0f, 15.0f);
    check("bad_cell", aoi_world_add(&w, 1, 0, 0) == AOI_ERR_BAD_CELL);
    aoi_world_init(&w, 10.0f, 15.0f);

    check("move_missing", aoi_world_move(&w, 42, 0, 0) == AOI_ERR_NOT_FOUND);
    check("cell_clamp", aoi_cell_coord(-100.0f, 10.0f) == 0 && aoi_cell_coord(1e6f, 10.0f) == GRID - 1);

    /* failure injection: fault fires, entity count and contents stay put */
    int n_before = w.n;
    aoi_inject(AOI_FAULT_ADD);
    check("inject_add", aoi_world_add(&w, 99, 1, 1) == AOI_ERR_INJECT_ADD);
    aoi_reset_faults();
    check("inject_isolated", w.n == n_before);
    check("fault_cleared", aoi_world_add(&w, 99, 1, 1) >= 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
#endif

#ifndef MINILAB_NO_MAIN
int main(void) {
#ifdef MINILAB_SELFTEST
    return MINILAB_SELFTEST();
#else
    return 0;
#endif
}
#endif
