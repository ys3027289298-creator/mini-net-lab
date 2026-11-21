
/* aoi: grid-based area of interest with spatial hash, enter/leave diff and radius query. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define GRID 32
#define GRID_CELLS (GRID * GRID)
#define MAX_ENT 16
#define MIN_BUCKETS 8
#define MAX_BUCKETS 2048

typedef struct { int id; float x, y; int alive; int bucket; } Entity;

typedef struct {
    Entity e[MAX_ENT];
    int n;
    int live;
    float cell;
    float radius;

    /* spatial hash: chained buckets, capacity is a power of two */
    float max_load;
    int cap;
    int heads[MAX_BUCKETS];
    int next[MAX_ENT];

    /* per-query bucket de-duplication and scan counters */
    unsigned stamp;
    unsigned visit[MAX_BUCKETS];
    int last_buckets;
    int last_candidates;
    int last_matches;
} World;

typedef struct {
    int cap;
    int live;
    int used;
    int colliding;
    int max_chain;
    int max_cell_occ;
    int theoretical_bound;
    int hist[MAX_ENT + 1];
    float load;
    float avg_chain;
} HashStats;

static void world_init_links(World* w) {
    for (int i = 0; i < MAX_BUCKETS; i++) w->heads[i] = -1;
    for (int i = 0; i < MAX_ENT; i++) w->next[i] = -1;
}

/* max_load must be > 0: a zero load factor would make resize/grow undefined. */
static int world_init(World* w, float cell, float radius, float max_load) {
    memset(w, 0, sizeof(*w));
    world_init_links(w);
    if (!(max_load > 0.0f)) return -1;
    if (!(cell > 0.0f)) return -1;
    if (!(radius >= 0.0f)) return -1;
    w->cell = cell;
    w->radius = radius;
    w->max_load = max_load;
    w->cap = MIN_BUCKETS;
    return 0;
}

static int cell_coord(float v, float cell) {
    int c = (int)floorf(v / cell);
    if (c < 0) c = 0;
    if (c >= GRID) c = GRID - 1;
    return c;
}

static uint32_t cell_index(int cx, int cy) {
    return (uint32_t)(cy * GRID + cx);
}

/* power-of-two masking. the odd multiplier is a bijection on k-bit residues;
 * because cells are exactly the 1024 values 0..GRID_CELLS-1, a bucket holds
 * at most ceil(GRID_CELLS / cap) distinct cells. this is the basis of the
 * chain-length bound reported by world_stats(). */
static int bucket_of(int cx, int cy, int cap) {
    uint32_t cell = cell_index(cx, cy);
    return (int)((cell * 2654435761u) & (uint32_t)(cap - 1));
}

static void hash_insert(World* w, int ei) {
    int cx = cell_coord(w->e[ei].x, w->cell);
    int cy = cell_coord(w->e[ei].y, w->cell);
    int b = bucket_of(cx, cy, w->cap);
    w->e[ei].bucket = b;
    w->next[ei] = w->heads[b];
    w->heads[b] = ei;
}

static void hash_unlink(World* w, int ei) {
    int b = w->e[ei].bucket;
    int* p = &w->heads[b];
    while (*p != -1) {
        if (*p == ei) {
            *p = w->next[ei];
            w->next[ei] = -1;
            return;
        }
        p = &w->next[*p];
    }
}

static int world_resize(World* w, int newcap) {
    if (!(w->max_load > 0.0f)) return -1;
    if (newcap < MIN_BUCKETS) newcap = MIN_BUCKETS;
    if (newcap > MAX_BUCKETS) newcap = MAX_BUCKETS;
    int cap = MIN_BUCKETS;
    while (cap < newcap) cap <<= 1;
    newcap = cap;

    world_init_links(w);
    w->cap = newcap;
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive) hash_insert(w, i);
    }
    return 0;
}

/* grow before an insert would push live/cap past max_load */
static int world_grow_for(World* w, int add) {
    if (!(w->max_load > 0.0f)) return -1;
    while (w->cap < MAX_BUCKETS &&
           (float)(w->live + add) > w->max_load * (float)w->cap) {
        if (world_resize(w, w->cap * 2) != 0) return -1;
    }
    return 0;
}

static int world_add(World* w, int id, float x, float y) {
    if (w->n >= MAX_ENT) return -1;
    if (w->cell <= 0.0f || w->cap < MIN_BUCKETS) return -1;
    if (w->cap > MAX_BUCKETS) return -1;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return -1;
    if (world_grow_for(w, 1) != 0) return -1;

    int i = w->n++;
    Entity* e = &w->e[i];
    e->id = id; e->x = x; e->y = y; e->alive = 1; e->bucket = -1;
    w->live++;
    hash_insert(w, i);
    return i;
}

static int world_move(World* w, int id, float x, float y) {
    for (int i = 0; i < w->n; i++) {
        if (!w->e[i].alive || w->e[i].id != id) continue;
        int old_bucket = w->e[i].bucket;
        w->e[i].x = x; w->e[i].y = y;
        int cx = cell_coord(x, w->cell);
        int cy = cell_coord(y, w->cell);
        if (bucket_of(cx, cy, w->cap) != old_bucket) {
            hash_unlink(w, i);
            hash_insert(w, i);
        }
        return 0;
    }
    return -1;
}

static int world_remove(World* w, int id) {
    for (int i = 0; i < w->n; i++) {
        if (!w->e[i].alive || w->e[i].id != id) continue;
        hash_unlink(w, i);
        w->e[i].alive = 0;
        w->live--;
        return 0;
    }
    return -1;
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

/* full-field reference implementation used to validate the hash path */
static int world_query_brute(const World* w, int self, float x, float y, int* out, int cap) {
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

/* collect ids within radius of (x,y), excluding self id.
 * only cells touched by the radius are visited; a bucket shared by several
 * of those cells is still traversed just once (stamp check). */
static int world_query(World* w, int self, float x, float y, int* out, int cap) {
    int n = 0;
    if (w->cap < MIN_BUCKETS) return 0;
    float r2 = w->radius * w->radius;
    int cx0 = cell_coord(x - w->radius, w->cell);
    int cx1 = cell_coord(x + w->radius, w->cell);
    int cy0 = cell_coord(y - w->radius, w->cell);
    int cy1 = cell_coord(y + w->radius, w->cell);

    w->stamp++;
    if (w->stamp == 0) {
        memset(w->visit, 0, sizeof(w->visit));
        w->stamp = 1;
    }

    int buckets = 0, candidates = 0;
    for (int cy = cy0; cy <= cy1; cy++) {
        for (int cx = cx0; cx <= cx1; cx++) {
            int b = bucket_of(cx, cy, w->cap);
            if (w->visit[b] == w->stamp) continue;
            w->visit[b] = w->stamp;
            buckets++;
            for (int i = w->heads[b]; i != -1; i = w->next[i]) {
                const Entity* e = &w->e[i];
                candidates++;
                if (!e->alive || e->id == self) continue;
                float dx = e->x - x, dy = e->y - y;
                if (dx * dx + dy * dy <= r2) {
                    if (n < cap) out[n] = e->id;
                    n++;
                }
            }
        }
    }
    w->last_buckets = buckets;
    w->last_candidates = candidates;
    w->last_matches = n;
    return n;
}

static void world_stats(const World* w, HashStats* s) {
    memset(s, 0, sizeof(*s));
    s->cap = w->cap;
    s->live = w->live;
    if (w->cap < MIN_BUCKETS) return;

    for (int b = 0; b < w->cap; b++) {
        int len = 0;
        for (int i = w->heads[b]; i != -1; i = w->next[i]) len++;
        if (len <= MAX_ENT) s->hist[len]++;
        if (len > 0) s->used++;
        if (len > 1) s->colliding++;
        if (len > s->max_chain) s->max_chain = len;
    }

    /* largest number of live entities sharing one grid cell */
    int cells[MAX_ENT], occ[MAX_ENT], ncells = 0;
    for (int i = 0; i < w->n; i++) {
        if (!w->e[i].alive) continue;
        int cx = cell_coord(w->e[i].x, w->cell);
        int cy = cell_coord(w->e[i].y, w->cell);
        int idx = (int)cell_index(cx, cy);
        int k = 0;
        while (k < ncells && cells[k] != idx) k++;
        if (k == ncells) { cells[ncells] = idx; occ[ncells] = 0; ncells++; }
        occ[k]++;
        if (occ[k] > s->max_cell_occ) s->max_cell_occ = occ[k];
    }

    /* each bucket owns at most ceil(GRID_CELLS/cap) cells, each cell holds at
     * most max_cell_occ live entities, and a chain cannot exceed live. */
    int cells_per_bucket = (GRID_CELLS + w->cap - 1) / w->cap;
    int bound = cells_per_bucket * s->max_cell_occ;
    if (bound > w->live) bound = w->live;
    s->theoretical_bound = bound;
    s->load = (float)w->live / (float)w->cap;
    s->avg_chain = s->load;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int cmp_int(const void* a, const void* b) {
    int x = *(const int*)a, y = *(const int*)b;
    return (x > y) - (x < y);
}

static int query_sets_equal(World* w, int self, float x, float y) {
    int a[MAX_ENT], b[MAX_ENT];
    int na = world_query(w, self, x, y, a, MAX_ENT);
    int nb = world_query_brute(w, self, x, y, b, MAX_ENT);
    if (na != nb) return 0;
    qsort(a, (size_t)na, sizeof(int), cmp_int);
    qsort(b, (size_t)nb, sizeof(int), cmp_int);
    return na == 0 || memcmp(a, b, (size_t)na * sizeof(int)) == 0;
}

static int capture(World* w, int self, float x, float y, int* out) {
    int n = world_query(w, self, x, y, out, MAX_ENT);
    qsort(out, (size_t)n, sizeof(int), cmp_int);
    return n;
}

static void print_stats(const char* tag, const HashStats* s) {
    printf("%s cap=%d live=%d load=%.3f used=%d colliding=%d "
           "max_chain=%d bound=%d max_cell_occ=%d avg_chain=%.3f\n",
           tag, s->cap, s->live, s->load, s->used, s->colliding,
           s->max_chain, s->theoretical_bound, s->max_cell_occ, s->avg_chain);
    printf("%s_hist", tag);
    for (int k = 0; k <= MAX_ENT; k++) if (s->hist[k]) printf(" %d:%d", k, s->hist[k]);
    printf("\n");
}

int main(void) {
    World w;
    check("init_ok", world_init(&w, 10.0f, 15.0f, 0.75f) == 0);
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

    world_init(&w, 0.0f, 15.0f, 0.75f);
    check("bad_cell", world_add(&w, 1, 0, 0) == -1);
    world_init(&w, 10.0f, 15.0f, 0.75f);

    check("move_missing", world_move(&w, 42, 0, 0) == -1);
    check("cell_clamp", cell_coord(-100.0f, 10.0f) == 0 && cell_coord(1e6f, 10.0f) == GRID - 1);

    /* --- spatial hash: consistency with full-field traversal --- */
    check("hash_init", world_init(&w, 10.0f, 15.0f, 0.5f) == 0);

    struct { int id; float x, y; } pts[16] = {
        { 100,   0.0f,    0.0f }, { 101,   3.0f,    4.0f },
        { 102,   9.0f,    0.0f }, { 103,  14.0f,    0.0f },
        { 104,  16.0f,    0.0f }, { 105,   0.0f,   16.0f },
        { 106, -10.0f,  -10.0f }, { 107, -20.0f,    0.0f },
        { 108, 100.0f,  100.0f }, { 109, 300.0f,  300.0f },
        { 110, 320.0f,  320.0f }, { 111, 330.0f,  330.0f },
        { 112, 1e6f,   -1e6f   }, { 113,   0.0f,    0.0f },
        { 114,   5.0f,    5.0f }, { 115, -15.0f,    0.0f },
    };
    int added = 0;
    for (int i = 0; i < 16; i++) {
        if (world_add(&w, pts[i].id, pts[i].x, pts[i].y) == i) added++;
    }
    check("hash_add_all", added == 16);
    check("load_factor_grow", w.cap == 32);

    const float qx[8] = { 0.0f, 14.0f, 300.0f, -10.0f, -1e6f, 1000.0f, 16.0f, 5.0f };
    const float qy[8] = { 0.0f,  0.0f, 300.0f, -10.0f,  1e6f, 1000.0f,  0.0f, 5.0f };
    const float radii[5] = { 0.0f, 5.0f, 15.0f, 33.0f, 1000.0f };
    int match = 1;
    for (int r = 0; r < 5; r++) {
        w.radius = radii[r];
        for (int k = 0; k < 8; k++) if (!query_sets_equal(&w, 999, qx[k], qy[k])) match = 0;
        if (!query_sets_equal(&w, 100, qx[0], qy[0])) match = 0;
    }
    w.radius = 15.0f;
    check("hash_matches_brute", match);

    /* --- resize by load factor: query sets must not change --- */
    int pre[8][MAX_ENT], pren[8];
    for (int k = 0; k < 8; k++) pren[k] = capture(&w, 999, qx[k], qy[k], pre[k]);
    HashStats before;
    world_stats(&w, &before);
    print_stats("stats_before_resize", &before);
    check("chain_bound_holds_before", before.max_chain <= before.theoretical_bound);
    check("query_scan_bounded_before", w.last_buckets <= w.cap);

    check("resize_ok", world_resize(&w, 1024) == 0);
    check("resize_cap", w.cap == 1024);
    check("resize_respects_load", (float)w.live / (float)w.cap <= w.max_load);

    int same = 1;
    for (int k = 0; k < 8; k++) {
        int post[MAX_ENT];
        int postn = capture(&w, 999, qx[k], qy[k], post);
        if (postn != pren[k]) { same = 0; continue; }
        for (int j = 0; j < postn; j++) if (post[j] != pre[k][j]) same = 0;
    }
    check("resize_query_same", same);

    HashStats after;
    world_stats(&w, &after);
    print_stats("stats_after_resize", &after);
    check("chain_bound_holds_after", after.max_chain <= after.theoretical_bound);
    check("chain_bound_tight_after",
          after.max_chain == after.max_cell_occ && after.theoretical_bound == after.max_cell_occ);

    /* a small-radius query touches only the cells around it, not the field */
    int tmp[MAX_ENT];
    w.radius = 15.0f;
    (void)world_query(&w, 999, 0.0f, 0.0f, tmp, MAX_ENT);
    printf("query_scan buckets=%d candidates=%d live=%d cap=%d\n",
           w.last_buckets, w.last_candidates, w.live, w.cap);
    check("neighbor_buckets_only", w.last_buckets <= 9 && w.last_buckets < w.cap);
    check("candidates_below_field", w.last_candidates < w.live);

    /* --- zero / invalid load factor is rejected --- */
    check("load_zero_rejected", world_init(&w, 10.0f, 15.0f, 0.0f) == -1);
    check("load_negative_rejected", world_init(&w, 10.0f, 15.0f, -1.0f) == -1);
    check("bad_init_add_rejected", world_add(&w, 1, 0, 0) == -1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
