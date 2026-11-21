/* aoi: area of interest with a density-adaptive quadtree.
 *
 * The fixed grid uses one cell size for the whole map: a crowded city gate
 * overloads a handful of cells while the empty wilderness keeps the same
 * resolution.  This version keeps the fixed grid as the reference index and
 * additionally maintains a density-adaptive quadtree:
 *
 *   - a leaf splits when it holds more than SPLIT_ENT entities, so dense
 *     regions get small cells and sparse regions keep large ones;
 *   - add/move/remove mark the index dirty; the next query (or an explicit
 *     world_rebuild) rebuilds it and re-packs every live entity into its
 *     leaf.  Ids, positions and pending enter/leave events are untouched;
 *   - queries always apply the exact radius test, so the adaptive visible
 *     set is identical to the fixed-grid one (checked by a deterministic
 *     randomized parity test);
 *   - with fewer than MIN_SAMPLES live entities there is not enough density
 *     information to resize: the original cell size is kept as the leaf
 *     size (population-based splitting is disabled), and RebuildInfo
 *     reports sample_ok=0, cell_kept=1;
 *   - every query is timed and its single-query latency is bucketed, split
 *     by dense and sparse query zones.
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define GRID 32
#define MAX_ENT 2048
#define MAX_NODES 4096
#define MAX_OBS 4
#define MAX_EVENTS 1024
#define MAX_DEPTH 8
#define SPLIT_ENT 16           /* split a leaf holding more than this */
#define MIN_SAMPLES 8        /* fewer live entities -> keep original size */
#define MIN_CELL_DIV 2.0f    /* never split below base_cell / MIN_CELL_DIV */

#define EV_ENTER 1
#define EV_LEAVE (-1)

#define LAT_NB 7

typedef struct { int id; float x, y; int alive; } Entity;

/* A quadtree node.  Leaves refer to a contiguous run of LeafEnt packed at
 * build time, so a query walks dense arrays and touches no scattered
 * entity slots.  Inner nodes own up to four children and no entities. */
typedef struct {
    float x0, y0, x1, y1;
    int first;               /* offset into World::leaf_ent */
    int count;               /* number of packed entities, 0 for inner nodes */
    int child[4];
    int nchild;
} Node;

typedef struct {
    float x, y;
    int id;
} LeafEnt;

typedef struct {
    int observer;
    int id;
    int kind;                /* EV_ENTER / EV_LEAVE */
    unsigned tick;
} AoiEvent;

typedef struct {
    int id;
    float x, y;
    int prev[MAX_ENT];       /* visible set at the previous step */
    int prevn;
} Observer;

typedef struct {
    int ok;                  /* tree built */
    int sample_ok;           /* enough live entities for density stats */
    int cell_kept;           /* original cell size kept (thin samples) */
    int leaves;
    int max_depth;
    int migrated;            /* entity slots re-linked by this rebuild */
    float base_cell;
    float leaf_min, leaf_max, leaf_avg;
} RebuildInfo;

typedef struct {
    Entity e[MAX_ENT];
    int n;
    int alive_n;
    float cell;              /* fixed-grid base cell size */
    float radius;

    /* fixed grid: bucket heads plus overflow for out-of-grid coordinates */
    int ghead[GRID * GRID];
    int goverflow;
    int gnext[MAX_ENT];

    /* adaptive quadtree over the live extent */
    Node nodes[MAX_NODES];
    int node_n;
    int root;
    int scratch[MAX_ENT];    /* build-time quadrant scatter buffer */
    LeafEnt leaf_ent[MAX_ENT]; /* entities packed leaf by leaf */
    int leaf_ent_n;
    int tree_ok;
    int split_cap;
    int density_mode;        /* 1 = adapt by population, 0 = keep base cell */
    int leaf_count;
    float leaf_min, leaf_max;
    double leaf_sum;
    int build_depth;

    int dirty;
    int rebuilds;

    /* last-query work counters (reset by each query entry point) */
    int stat_nodes;         /* tree nodes visited (adaptive) */
    int stat_cells;         /* grid cells visited (fixed) */
    int stat_tested;        /* exact radius tests performed */

    unsigned tick;

    Observer obs[MAX_OBS];
    int nobs;

    AoiEvent evlog[MAX_EVENTS];
    int evn;
    int ev_dropped;
} World;

static void world_init(World* w, float cell, float radius) {
    memset(w, 0, sizeof(*w));
    w->cell = cell;
    w->radius = radius;
    w->root = -1;
    w->goverflow = -1;
    w->tree_ok = 1;
    w->split_cap = SPLIT_ENT;
    for (int i = 0; i < GRID * GRID; i++) w->ghead[i] = -1;
    w->dirty = 1;
}

static int cell_coord(float v, float cell) {
    int c = (int)floorf(v / cell);
    if (c < 0) c = 0;
    if (c >= GRID) c = GRID - 1;
    return c;
}

static int world_add(World* w, int id, float x, float y) {
    if (w->n >= MAX_ENT) return -1;
    if (w->cell <= 0.0f) return -1;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return -1;
    Entity* e = &w->e[w->n++];
    e->id = id; e->x = x; e->y = y; e->alive = 1;
    w->alive_n++;
    w->dirty = 1;
    return w->n - 1;
}

static int world_move(World* w, int id, float x, float y) {
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) {
            w->e[i].x = x; w->e[i].y = y;
            w->dirty = 1;
            return 0;
        }
    }
    return -1;
}

static int world_remove(World* w, int id) {
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) {
            w->e[i].alive = 0;
            w->alive_n--;
            w->dirty = 1;
            return 0;
        }
    }
    return -1;
}

static int world_find(const World* w, int id) {
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return i;
    return -1;
}

/* ---- fixed-grid reference index ------------------------------------- */

static void grid_build(World* w) {
    for (int i = 0; i < GRID * GRID; i++) w->ghead[i] = -1;
    w->goverflow = -1;
    float span = w->cell * (float)GRID;
    for (int i = 0; i < w->n; i++) {
        const Entity* e = &w->e[i];
        if (!e->alive) continue;
        if (e->x < 0.0f || e->y < 0.0f || e->x >= span || e->y >= span) {
            w->gnext[i] = w->goverflow;
            w->goverflow = i;
        } else {
            int cx = cell_coord(e->x, w->cell);
            int cy = cell_coord(e->y, w->cell);
            int k = cy * GRID + cx;
            w->gnext[i] = w->ghead[k];
            w->ghead[k] = i;
        }
    }
}

/* ---- density-adaptive quadtree -------------------------------------- */

static int build_rec(World* w, int* idx, int cnt, float x0, float y0,
                     float x1, float y1, int depth) {
    if (w->node_n >= MAX_NODES) return -1;
    int ni = w->node_n++;
    Node* nd = &w->nodes[ni];
    nd->x0 = x0; nd->y0 = y0; nd->x1 = x1; nd->y1 = y1;
    nd->first = w->leaf_ent_n;
    nd->count = 0;
    nd->nchild = 0;
    for (int c = 0; c < 4; c++) nd->child[c] = -1;
    if (depth > w->build_depth) w->build_depth = depth;

    float spanx = x1 - x0, spany = y1 - y0;
    int split_now;
    if (w->density_mode) {
        /* adaptive: split by population */
        float min_span = w->cell / MIN_CELL_DIV;
        int can_split = depth < MAX_DEPTH && spanx > min_span && spany > min_span;
        split_now = cnt > w->split_cap && can_split;
    } else {
        /* thin samples: keep the original grid cell size as the leaf size */
        split_now = cnt > 0 && depth < MAX_DEPTH &&
                    (spanx > w->cell * 1.0001f || spany > w->cell * 1.0001f);
    }

    if (!split_now) {
        for (int k = 0; k < cnt; k++) {
            const Entity* e = &w->e[idx[k]];
            LeafEnt* le = &w->leaf_ent[w->leaf_ent_n++];
            le->x = e->x;
            le->y = e->y;
            le->id = e->id;
        }
        nd->count = cnt;
        w->leaf_count++;
        float side = fmaxf(spanx, spany);
        if (w->leaf_count == 1 || side < w->leaf_min) w->leaf_min = side;
        if (side > w->leaf_max) w->leaf_max = side;
        w->leaf_sum += side;
        return ni;
    }

    float xm = x0 + spanx * 0.5f;
    float ym = y0 + spany * 0.5f;

    /* Stable in-place quadrant partition of idx[0..cnt): count, prefix,
     * scatter through one shared scratch buffer, copy back.  Done fully
     * before recursing so a single scratch buffer serves every level and
     * the stack stays small no matter how deep the tree goes. */
    int qn[4] = { 0, 0, 0, 0 };
    for (int k = 0; k < cnt; k++) {
        const Entity* e = &w->e[idx[k]];
        int qx = (e->x < xm) ? 0 : 1;
        int qy = (e->y < ym) ? 0 : 1;
        qn[qy * 2 + qx]++;
    }
    int start[4], cur[4];
    start[0] = 0; cur[0] = 0;
    for (int q = 1; q < 4; q++) { start[q] = start[q - 1] + qn[q - 1]; cur[q] = start[q]; }
    for (int k = 0; k < cnt; k++) {
        const Entity* e = &w->e[idx[k]];
        int qx = (e->x < xm) ? 0 : 1;
        int qy = (e->y < ym) ? 0 : 1;
        w->scratch[cur[qy * 2 + qx]++] = idx[k];
    }
    memcpy(idx, w->scratch, sizeof(int) * (size_t)cnt);

    for (int q = 0; q < 4; q++) {
        if (qn[q] == 0) continue;
        float cx0 = (q & 1) ? xm : x0, cx1 = (q & 1) ? x1 : xm;
        float cy0 = (q & 2) ? ym : y0, cy1 = (q & 2) ? y1 : ym;
        int child = build_rec(w, idx + start[q], qn[q], cx0, cy0, cx1, cy1, depth + 1);
        if (child < 0) return -1;
        nd->child[nd->nchild++] = child;
    }
    return ni;
}

static int world_rebuild(World* w, RebuildInfo* info) {
    RebuildInfo local;
    if (info == NULL) info = &local;
    memset(info, 0, sizeof(*info));
    info->base_cell = w->cell;

    grid_build(w);
    w->node_n = 0;
    w->root = -1;
    w->leaf_count = 0;
    w->leaf_min = 0.0f;
    w->leaf_max = 0.0f;
    w->leaf_sum = 0.0;
    w->build_depth = 0;
    w->tree_ok = 1;
    w->leaf_ent_n = 0;

    int idx[MAX_ENT];
    int cnt = 0;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive) idx[cnt++] = i;
    info->migrated = cnt;

    info->sample_ok = (cnt >= MIN_SAMPLES);
    info->cell_kept = !info->sample_ok;
    w->density_mode = info->sample_ok;
    w->split_cap = info->sample_ok ? SPLIT_ENT : MAX_ENT;

    float x0, y0, x1, y1;
    if (cnt == 0) {
        x0 = 0.0f; y0 = 0.0f;
        x1 = w->cell * (float)GRID;
        y1 = x1;
    } else {
        float minx = w->e[idx[0]].x, maxx = minx;
        float miny = w->e[idx[0]].y, maxy = miny;
        for (int k = 1; k < cnt; k++) {
            float ex = w->e[idx[k]].x, ey = w->e[idx[k]].y;
            if (ex < minx) minx = ex;
            if (ex > maxx) maxx = ex;
            if (ey < miny) miny = ey;
            if (ey > maxy) maxy = ey;
        }
        float pad = w->cell * 0.5f;
        x0 = minx - pad; y0 = miny - pad;
        x1 = maxx + pad; y1 = maxy + pad;
    }

    w->root = build_rec(w, idx, cnt, x0, y0, x1, y1, 0);
    if (w->root < 0) {
        w->tree_ok = 0;
        info->ok = 0;
    } else {
        info->ok = 1;
    }
    info->leaves = w->leaf_count;
    info->max_depth = w->build_depth;
    info->leaf_min = w->leaf_min;
    info->leaf_max = w->leaf_max;
    info->leaf_avg = w->leaf_count ? (float)(w->leaf_sum / w->leaf_count) : 0.0f;

    w->rebuilds++;
    w->dirty = 0;
    return info->ok;
}

/* ---- queries --------------------------------------------------------- */

static int in_radius(const Entity* e, int self, float x, float y, float r2) {
    if (!e->alive || e->id == self) return 0;
    float dx = e->x - x, dy = e->y - y;
    return dx * dx + dy * dy <= r2;
}

/* Reference: fixed grid, exact radius on candidates. */
static int world_query_fixed(World* w, int self, float x, float y, int* out, int cap) {
    if (w->dirty) world_rebuild(w, NULL);
    int n = 0;
    float r2 = w->radius * w->radius;
    w->stat_cells = 0;
    w->stat_tested = 0;

    for (int i = w->goverflow; i != -1; i = w->gnext[i]) {
        w->stat_tested++;
        if (in_radius(&w->e[i], self, x, y, r2)) {
            if (n < cap) out[n] = w->e[i].id;
            n++;
        }
    }

    float span = w->cell * (float)GRID;
    if (x + w->radius >= 0.0f && x - w->radius <= span &&
        y + w->radius >= 0.0f && y - w->radius <= span) {
        int cx0 = cell_coord(x - w->radius, w->cell);
        int cx1 = cell_coord(x + w->radius, w->cell);
        int cy0 = cell_coord(y - w->radius, w->cell);
        int cy1 = cell_coord(y + w->radius, w->cell);
        for (int cy = cy0; cy <= cy1; cy++) {
            for (int cx = cx0; cx <= cx1; cx++) {
                w->stat_cells++;
                for (int i = w->ghead[cy * GRID + cx]; i != -1; i = w->gnext[i]) {
                    w->stat_tested++;
                    if (in_radius(&w->e[i], self, x, y, r2)) {
                        if (n < cap) out[n] = w->e[i].id;
                        n++;
                    }
                }
            }
        }
    }
    return n;
}

/* Brute-force reference used only to validate the fixed grid. */
static int world_query_scan(const World* w, int self, float x, float y, int* out, int cap) {
    int n = 0;
    float r2 = w->radius * w->radius;
    for (int i = 0; i < w->n; i++) {
        if (in_radius(&w->e[i], self, x, y, r2)) {
            if (n < cap) out[n] = w->e[i].id;
            n++;
        }
    }
    return n;
}

static int circle_hits_rect(const Node* nd, float qx, float qy, float r2) {
    float dx = qx < nd->x0 ? nd->x0 - qx : (qx > nd->x1 ? qx - nd->x1 : 0.0f);
    float dy = qy < nd->y0 ? nd->y0 - qy : (qy > nd->y1 ? qy - nd->y1 : 0.0f);
    return dx * dx + dy * dy <= r2;
}

static int circle_covers_rect(const Node* nd, float qx, float qy, float r2) {
    float dx = fmaxf(fabsf(qx - nd->x0), fabsf(qx - nd->x1));
    float dy = fmaxf(fabsf(qy - nd->y0), fabsf(qy - nd->y1));
    return dx * dx + dy * dy <= r2;
}

static void collect_node(World* w, int ni, int self, float qx, float qy,
                         float r2, int* out, int cap, int* count) {
    const Node* nd = &w->nodes[ni];
    w->stat_nodes++;
    if (!circle_hits_rect(nd, qx, qy, r2)) return;
    int covered = circle_covers_rect(nd, qx, qy, r2);
    if (nd->count > 0) {
        const LeafEnt* le = &w->leaf_ent[nd->first];
        for (int i = 0; i < nd->count; i++) {
            const LeafEnt* e = &le[i];
            if (e->id == self) continue;
            if (!covered) {
                w->stat_tested++;
                float dx = e->x - qx, dy = e->y - qy;
                if (dx * dx + dy * dy > r2) continue;
            }
            if (*count < cap) out[*count] = e->id;
            (*count)++;
        }
        return;
    }
    for (int c = 0; c < nd->nchild; c++) {
        collect_node(w, nd->child[c], self, qx, qy, r2, out, cap, count);
    }
}

static int world_query_adaptive(World* w, int self, float x, float y, int* out, int cap) {
    if (w->dirty) world_rebuild(w, NULL);
    if (!w->tree_ok) return world_query_fixed(w, self, x, y, out, cap);
    int n = 0;
    w->stat_nodes = 0;
    w->stat_tested = 0;
    collect_node(w, w->root, self, x, y, w->radius * w->radius, out, cap, &n);
    return n;
}

/* Leaf node currently holding an entity id, or -1. */
static int leaf_of_rec(const World* w, int ni, int id) {
    const Node* nd = &w->nodes[ni];
    if (nd->count > 0) {
        const LeafEnt* le = &w->leaf_ent[nd->first];
        for (int i = 0; i < nd->count; i++) if (le[i].id == id) return ni;
        return -1;
    }
    for (int c = 0; c < nd->nchild; c++) {
        int r = leaf_of_rec(w, nd->child[c], id);
        if (r >= 0) return r;
    }
    return -1;
}

static int world_leaf_of(World* w, int id) {
    if (w->dirty) world_rebuild(w, NULL);
    if (!w->tree_ok || w->root < 0) return -1;
    return leaf_of_rec(w, w->root, id);
}

/* ---- enter/leave events ---------------------------------------------- */

static int world_set_observer(World* w, int id, float x, float y) {
    if (w->nobs >= MAX_OBS) return -1;
    Observer* o = &w->obs[w->nobs++];
    memset(o, 0, sizeof(*o));
    o->id = id;
    o->x = x;
    o->y = y;
    return w->nobs - 1;
}

static int world_move_observer(World* w, int id, float x, float y) {
    for (int i = 0; i < w->nobs; i++) {
        if (w->obs[i].id == id) {
            w->obs[i].x = x;
            w->obs[i].y = y;
            return 0;
        }
    }
    return -1;
}

static int event_push(World* w, int observer, int id, int kind) {
    if (w->evn >= MAX_EVENTS) { w->ev_dropped++; return -1; }
    AoiEvent* ev = &w->evlog[w->evn++];
    ev->observer = observer;
    ev->id = id;
    ev->kind = kind;
    ev->tick = w->tick;
    return 0;
}

static int seen(const int* ids, int n, int id) {
    for (int i = 0; i < n; i++) if (ids[i] == id) return 1;
    return 0;
}

/* Recompute every observer's visible set, emit enter/leave deltas.  A
 * rebuild happens first when the index is dirty; pending events are kept. */
static int world_step(World* w, unsigned tick) {
    if (w->dirty) world_rebuild(w, NULL);
    w->tick = tick;
    int emitted = 0;
    for (int oi = 0; oi < w->nobs; oi++) {
        Observer* o = &w->obs[oi];
        int cur[MAX_ENT];
        int cn = world_query_adaptive(w, o->id, o->x, o->y, cur, MAX_ENT);
        for (int i = 0; i < o->prevn; i++) {
            if (!seen(cur, cn, o->prev[i])) {
                event_push(w, o->id, o->prev[i], EV_LEAVE);
                emitted++;
            }
        }
        for (int i = 0; i < cn; i++) {
            if (!seen(o->prev, o->prevn, cur[i])) {
                event_push(w, o->id, cur[i], EV_ENTER);
                emitted++;
            }
        }
        memcpy(o->prev, cur, sizeof(int) * (size_t)cn);
        o->prevn = cn;
    }
    return emitted;
}

static int world_events_pending(const World* w) { return w->evn; }

static int world_drain_events(World* w) {
    int n = w->evn;
    w->evn = 0;
    return n;
}

/* Diff two explicit visible sets: counts enter/leave (legacy helper). */
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

/* ---- tests ----------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned rng_state = 0x12345678u;
static unsigned rnd(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}
static float rndf(float lo, float hi) {
    return lo + (hi - lo) * (float)(rnd() >> 8) / 16777216.0f;
}

static int cmp_int(const void* a, const void* b) {
    int x = *(const int*)a, y = *(const int*)b;
    return (x > y) - (x < y);
}

/* Compare adaptive vs fixed-grid vs brute-force visible sets over random
 * query points.  Every few queries an observer id is passed so the
 * exclude-self path is exercised too. */
static int parity_check(World* w, int queries, float span) {
    int bad = 0;
    for (int q = 0; q < queries; q++) {
        float qx = rndf(0.0f, span), qy = rndf(0.0f, span);
        int self = -1;
        if ((q % 3) == 0 && w->n > 0) {
            /* pick a live id near the query point so it would be visible */
            int best = -1;
            float bd = 1e9f;
            for (int i = 0; i < w->n; i++) {
                if (!w->e[i].alive) continue;
                float dx = w->e[i].x - qx, dy = w->e[i].y - qy;
                float d = dx * dx + dy * dy;
                if (d < bd) { bd = d; best = i; }
            }
            if (best >= 0) self = w->e[best].id;
        }
        int a[MAX_ENT], b[MAX_ENT], c[MAX_ENT];
        int an = world_query_adaptive(w, self, qx, qy, a, MAX_ENT);
        int bn = world_query_fixed(w, self, qx, qy, b, MAX_ENT);
        int cn = world_query_scan(w, self, qx, qy, c, MAX_ENT);
        qsort(a, (size_t)an, sizeof(int), cmp_int);
        qsort(b, (size_t)bn, sizeof(int), cmp_int);
        qsort(c, (size_t)cn, sizeof(int), cmp_int);
        if (an != bn || memcmp(a, b, sizeof(int) * (size_t)an) != 0) bad++;
        if (bn != cn || memcmp(b, c, sizeof(int) * (size_t)bn) != 0) bad++;
        if (self >= 0) {
            for (int i = 0; i < an; i++) if (a[i] == self) { bad++; break; }
        }
    }
    return bad;
}

static void fill_dense(World* w, int near_n, int spread_n) {
    for (int i = 0; i < near_n; i++) {
        world_add(w, 10000 + i, 100.0f + rndf(-12.0f, 12.0f), 100.0f + rndf(-12.0f, 12.0f));
    }
    for (int i = 0; i < spread_n; i++) {
        world_add(w, 20000 + i, rndf(0.0f, 300.0f), rndf(0.0f, 300.0f));
    }
}

static double now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

/* 7 buckets: <50ns <100ns <200ns <400ns <800ns <1600ns >=1600ns */
static int lat_bucket(double ns) {
    static const double edge[LAT_NB - 1] = { 50.0, 100.0, 200.0, 400.0, 800.0, 1600.0 };
    for (int i = 0; i < LAT_NB - 1; i++) if (ns < edge[i]) return i;
    return LAT_NB - 1;
}

/* Query latencies, split by dense (city gate) and sparse (wilderness)
 * query zones.  Reports nanoseconds plus index work per query. */
static void benchmark_zone(World* w, const char* zone, int iters,
                           float x0, float y0, float x1, float y1) {
    int ab[LAT_NB] = { 0 }, fb[LAT_NB] = { 0 };
    double asum = 0.0, fsum = 0.0;
    long anode = 0, fcell = 0, atest = 0, ftest = 0;
    long mismatch = 0;
    int out[MAX_ENT];
    int fix[MAX_ENT];
    world_rebuild(w, NULL);
    for (int i = 0; i < iters; i++) {
        float qx = rndf(x0, x1), qy = rndf(y0, y1);
        double t0 = now_ns();
        int n = world_query_adaptive(w, -1, qx, qy, out, MAX_ENT);
        double dt = now_ns() - t0;
        ab[lat_bucket(dt)]++;
        asum += dt;
        anode += w->stat_nodes;
        atest += w->stat_tested;

        t0 = now_ns();
        int m = world_query_fixed(w, -1, qx, qy, fix, MAX_ENT);
        dt = now_ns() - t0;
        fb[lat_bucket(dt)]++;
        fsum += dt;
        fcell += w->stat_cells;
        ftest += w->stat_tested;

        qsort(out, (size_t)n, sizeof(int), cmp_int);
        qsort(fix, (size_t)m, sizeof(int), cmp_int);
        if (n != m || memcmp(out, fix, sizeof(int) * (size_t)n) != 0) mismatch++;
    }
    int as = 0, fs = 0;
    for (int i = 0; i < LAT_NB; i++) { as += ab[i]; fs += fb[i]; }
    check("lat_buckets_sum", as == iters && fs == iters && asum > 0.0 && fsum > 0.0);
    printf("zone=%s iters=%d\n", zone, iters);
    printf("  query_adaptive_ns: <50=%d <100=%d <200=%d <400=%d <800=%d <1600=%d >=1600=%d avg=%.1f\n",
           ab[0], ab[1], ab[2], ab[3], ab[4], ab[5], ab[6], asum / iters);
    printf("  query_fixed_ns:    <50=%d <100=%d <200=%d <400=%d <800=%d <1600=%d >=1600=%d avg=%.1f\n",
           fb[0], fb[1], fb[2], fb[3], fb[4], fb[5], fb[6], fsum / iters);
    printf("  work_per_query: adaptive_nodes=%.1f adaptive_tests=%.1f "
           "fixed_cells=%.1f fixed_tests=%.1f\n",
           (double)anode / iters, (double)atest / iters,
           (double)fcell / iters, (double)ftest / iters);
    printf("  identical=%d mismatch=%ld\n", mismatch == 0, mismatch);
    check("bench_identical", mismatch == 0);
}

static void benchmark(World* w, int iters, float span) {
    benchmark_zone(w, "dense", iters, 80.0f, 80.0f, 120.0f, 120.0f);
    benchmark_zone(w, "sparse", iters, 0.0f, 180.0f, span, span);
}

int main(void) {
    /* ---- legacy fixed-grid behaviour ---- */
    static World w;
    world_init(&w, 10.0f, 15.0f);
    check("add_self", world_add(&w, 1, 0, 0) == 0);
    check("add_near", world_add(&w, 2, 5, 0) == 1);
    check("add_far", world_add(&w, 3, 100, 100) == 2);
    check("dup_rejected", world_add(&w, 2, 1, 1) == -1);

    int out[8], n;
    n = world_query_fixed(&w, 1, 0, 0, out, 8);
    check("query_near", n == 1 && out[0] == 2);
    n = world_query_adaptive(&w, 1, 0, 0, out, 8);
    check("query_near_adaptive", n == 1 && out[0] == 2);

    world_move(&w, 2, 100, 100);
    n = world_query_fixed(&w, 1, 0, 0, out, 8);
    check("query_after_move", n == 0);
    n = world_query_adaptive(&w, 1, 0, 0, out, 8);
    check("query_after_move_adaptive", n == 0);

    int prev[2] = { 2, 3 }, cur[2] = { 3, 4 }, enter = 0, leave = 0;
    world_diff(prev, 2, cur, 2, &enter, &leave);
    check("diff_counts", enter == 1 && leave == 1);

    check("remove_ok", world_remove(&w, 3) == 0);
    n = world_query_fixed(&w, 1, 200, 200, out, 8);
    check("removed_absent", n == 0);

    world_init(&w, 0.0f, 15.0f);
    check("bad_cell", world_add(&w, 1, 0, 0) == -1);
    world_init(&w, 10.0f, 15.0f);

    check("move_missing", world_move(&w, 42, 0, 0) == -1);
    check("cell_clamp", cell_coord(-100.0f, 10.0f) == 0 && cell_coord(1e6f, 10.0f) == GRID - 1);

    /* ---- density sampling fallback ---- */
    static World thin;
    world_init(&thin, 10.0f, 15.0f);
    world_add(&thin, 1, 10, 10);
    world_add(&thin, 2, 30, 30);
    world_add(&thin, 3, 70, 60);
    RebuildInfo ti;
    world_rebuild(&thin, &ti);
    check("thin_sample_ok", ti.sample_ok == 0);
    check("thin_cell_kept", ti.cell_kept == 1);
    check("thin_base_cell", ti.base_cell == 10.0f);
    /* thin mode does not resize by population: leaves stay at the original
     * base cell size instead of collapsing into one giant leaf */
    check("thin_cell_sized", ti.leaves > 1 && ti.leaf_max <= 10.0f + 1e-3f);
    check("thin_parity", parity_check(&thin, 200, 100.0f) == 0);
    printf("density_thin: sample_ok=%d cell_kept=%d base_cell=%.2f leaves=%d leaf_min=%.2f leaf_max=%.2f\n",
           ti.sample_ok, ti.cell_kept, ti.base_cell, ti.leaves, ti.leaf_min, ti.leaf_max);

    /* ---- dense/sparse adaptation ---- */
    static World dense;
    world_init(&dense, 10.0f, 15.0f);
    fill_dense(&dense, 48, 32);
    RebuildInfo di;
    world_rebuild(&dense, &di);
    check("dense_sample_ok", di.sample_ok == 1);
    check("dense_cell_kept", di.cell_kept == 0);
    check("dense_split", di.leaves > 8);
    check("dense_varied_cell", di.leaf_max > di.leaf_min * 2.0f);
    check("dense_migrated", di.migrated == 80);
    check("dense_depth", di.max_depth > 0);
    check("dense_parity", parity_check(&dense, 500, 300.0f) == 0);
    printf("density_dense: sample_ok=%d cell_kept=%d base_cell=%.2f leaves=%d "
           "leaf_min=%.2f leaf_max=%.2f leaf_avg=%.2f depth=%d migrated=%d\n",
           di.sample_ok, di.cell_kept, di.base_cell, di.leaves,
           di.leaf_min, di.leaf_max, di.leaf_avg, di.max_depth, di.migrated);

    /* ---- entity migration across cells keeps ids, positions, parity ---- */
    int mover = world_find(&dense, 10000);
    int before_leaf = world_leaf_of(&dense, 10000);
    world_move(&dense, 10000, 290.0f, 290.0f);
    RebuildInfo mi;
    world_rebuild(&dense, &mi);
    int after_leaf = world_leaf_of(&dense, 10000);
    check("migrate_leaf_changed", before_leaf >= 0 && after_leaf >= 0 && before_leaf != after_leaf);
    check("migrate_alive", mover >= 0 && dense.e[mover].alive && dense.e[mover].id == 10000);
    check("migrate_pos_kept", fabsf(dense.e[mover].x - 290.0f) < 1e-4f &&
                              fabsf(dense.e[mover].y - 290.0f) < 1e-4f);
    check("migrate_parity", parity_check(&dense, 500, 300.0f) == 0);

    /* random moves + rebuilds must keep parity with the fixed grid */
    for (int r = 0; r < 20; r++) {
        int id = 20000 + (int)(rnd() % 32u);
        world_move(&dense, id, rndf(0.0f, 300.0f), rndf(0.0f, 300.0f));
    }
    check("churn_parity", parity_check(&dense, 500, 300.0f) == 0);

    /* ---- events: emitted once, survive a rebuild, drained cleanly ---- */
    static World ew;
    world_init(&ew, 10.0f, 12.0f);
    check("obs_add", world_set_observer(&ew, 1, 0.0f, 0.0f) == 0);
    world_add(&ew, 2, 5, 0);
    world_add(&ew, 3, 100, 100);
    check("enter_once", world_step(&ew, 1) == 1 && world_events_pending(&ew) == 1);
    world_rebuild(&ew, NULL);
    check("events_survive_rebuild", world_events_pending(&ew) == 1);
    check("no_dup_after_rebuild", world_step(&ew, 2) == 0 && world_events_pending(&ew) == 1);
    world_move(&ew, 2, 100, 0);
    check("leave_once", world_step(&ew, 3) == 1);
    check("events_total", world_events_pending(&ew) == 2);
    check("drain_count", world_drain_events(&ew) == 2 && world_events_pending(&ew) == 0);
    world_move(&ew, 2, 0, 5);
    check("reenter", world_step(&ew, 4) == 1 && world_events_pending(&ew) == 1);
    check("observer_move", world_move_observer(&ew, 1, 100.0f, 100.0f) == 0);
    check("observer_leave", world_step(&ew, 5) == 2); /* 2 leaves at 0,5; 3 enters at 100,100 */

    /* migration between rebuilds must not fabricate events */
    static World mew;
    world_init(&mew, 10.0f, 25.0f);
    world_set_observer(&mew, 1, 100.0f, 100.0f);
    fill_dense(&mew, 20, 0);
    check("migrate_first_step", world_step(&mew, 10) == 20);
    world_drain_events(&mew);
    world_move(&mew, 10000, 105.0f, 100.0f);
    world_rebuild(&mew, NULL);
    check("migrate_no_event", world_step(&mew, 11) == 0);
    check("migrate_parity_events", parity_check(&mew, 100, 140.0f) == 0);

    /* ---- single-query latency buckets ---- */
    static World bench;
    world_init(&bench, 10.0f, 4.0f);
    fill_dense(&bench, 1500, 200);
    RebuildInfo bi;
    world_rebuild(&bench, &bi);
    check("bench_dense_split", bi.leaves > 16);
    printf("density_bench: leaves=%d leaf_min=%.2f leaf_max=%.2f leaf_avg=%.2f depth=%d\n",
           bi.leaves, bi.leaf_min, bi.leaf_max, bi.leaf_avg, bi.max_depth);
    benchmark(&bench, 20000, 300.0f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
