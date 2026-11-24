
/* scenetree: incremental scene-tree rebuild driven by dirty-mark propagation.
 *
 * Propagation rules (must not miss):
 *  1. every state change goes through set_pos/set_hp, never bare writes;
 *  2. mark_dirty(n) marks n and walks up to root (early-out on an already
 *     dirty ancestor is safe: its ancestors were marked when it was);
 *  3. dependents are marked too: children read the parent's world transform
 *     when building, so moving a node dirties its whole subtree.
 * rebuild() only descends into dirty subtrees; clean subtrees reuse the
 * cached checksum (a "mark hit"). full_sum() recomputes everything from
 * state and is used to verify the incremental result matches, every frame.
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define COMPS 7 /* component nodes per player */

enum { COMP_MESH, COMP_NAME, COMP_HPBAR, COMP_BUFF, COMP_SHADOW, COMP_WEAPON, COMP_AURA };

typedef struct {
    int parent;
    int first_child;
    int next_sib;
    int dirty;
    float lx, ly;   /* local state */
    float hp;
    float wx, wy;   /* cached built world transform */
    unsigned built; /* cached subtree checksum */
} Node;

typedef struct {
    Node* nodes;
    int n;        /* total nodes */
    int players;
    long rebuilt; /* per-frame stats */
    long hits;
} Scene;

static int player_node(int i) { return 1 + i * (1 + COMPS); }

static void scene_init(Scene* s, int players) {
    s->players = players;
    s->n = 1 + players * (1 + COMPS);
    s->nodes = malloc(sizeof(Node) * (size_t)s->n);
    for (int i = 0; i < s->n; i++) {
        Node* nd = &s->nodes[i];
        nd->parent = nd->first_child = nd->next_sib = -1;
        nd->dirty = 1;
        nd->lx = nd->ly = 0.0f;
        nd->hp = 100.0f;
        nd->wx = nd->wy = 0.0f;
        nd->built = 0;
    }
    for (int p = 0; p < players; p++) {
        int pi = player_node(p);
        s->nodes[pi].parent = 0;
        s->nodes[pi].next_sib = (p + 1 < players) ? player_node(p + 1) : -1;
        s->nodes[pi].lx = p * 1.5f;
        s->nodes[pi].ly = 0.1f * p;
        if (p == 0) s->nodes[0].first_child = pi;
        s->nodes[pi].first_child = pi + 1;
        for (int c = 0; c < COMPS; c++) {
            int ci = pi + 1 + c;
            s->nodes[ci].parent = pi;
            s->nodes[ci].next_sib = (c + 1 < COMPS) ? ci + 1 : -1;
            s->nodes[ci].lx = 0.1f * c;
        }
    }
}

static void mark_dirty(Scene* s, int i) {
    while (i >= 0 && !s->nodes[i].dirty) {
        s->nodes[i].dirty = 1;
        i = s->nodes[i].parent;
    }
}

static void set_pos(Scene* s, int p, float x, float y) {
    int pi = player_node(p);
    s->nodes[pi].lx = x;
    s->nodes[pi].ly = y;
    mark_dirty(s, pi); /* self + ancestors */
    for (int c = s->nodes[pi].first_child; c >= 0; c = s->nodes[c].next_sib)
        mark_dirty(s, c); /* dependents: children consume parent transform */
}

static void set_hp(Scene* s, int p, float hp) {
    int bar = player_node(p) + 1 + COMP_HPBAR;
    s->nodes[bar].hp = hp;
    mark_dirty(s, bar); /* leaf state: only self + ancestors */
}

static unsigned mix(unsigned h, unsigned v) { return (h ^ v) * 16777619u; }
static unsigned fbits(float f) { unsigned u; memcpy(&u, &f, 4); return u; }

static void compose(const Node* n, float px, float py,
                    float* wx, float* wy, unsigned* h) {
    float c = cosf(n->ly), sn = sinf(n->ly);
    *wx = px + n->lx * c - n->hp * sn * 0.01f;
    *wy = py + n->lx * sn + n->hp * c * 0.01f;
    unsigned hh = 2166136261u;
    hh = mix(hh, fbits(*wx));
    hh = mix(hh, fbits(*wy));
    hh = mix(hh, fbits(n->hp));
    *h = hh;
}

static unsigned rebuild(Scene* s, int i) {
    Node* n = &s->nodes[i];
    if (!n->dirty) { s->hits++; return n->built; }
    float px = 0.0f, py = 0.0f;
    if (n->parent >= 0) { px = s->nodes[n->parent].wx; py = s->nodes[n->parent].wy; }
    unsigned h;
    compose(n, px, py, &n->wx, &n->wy, &h);
    for (int c = n->first_child; c >= 0; c = s->nodes[c].next_sib)
        h = mix(h, rebuild(s, c));
    n->built = h;
    n->dirty = 0;
    s->rebuilt++;
    return h;
}

/* full recompute from state, ignoring marks; verification + baseline cost */
static unsigned full_sum(const Scene* s, int i, float px, float py) {
    const Node* n = &s->nodes[i];
    float wx, wy;
    unsigned h;
    compose(n, px, py, &wx, &wy, &h);
    for (int c = n->first_child; c >= 0; c = s->nodes[c].next_sib)
        h = mix(h, full_sum(s, c, wx, wy));
    return h;
}

static unsigned long long rng_s;
static unsigned long long rng_next(void) {
    unsigned long long x = rng_s;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    rng_s = x;
    return x;
}
static float rngf(void) { return (float)((rng_next() >> 11) * (1.0 / 9007199254740992.0)); }

static long long now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000ll + ts.tv_nsec;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int selftest(void) {
    Scene s;
    scene_init(&s, 17);

    s.rebuilt = s.hits = 0;
    rebuild(&s, 0);
    check("initial_full_build", s.rebuilt == s.n);

    s.rebuilt = s.hits = 0;
    rebuild(&s, 0);
    check("idle_frame_rebuilds_nothing", s.rebuilt == 0);

    set_hp(&s, 3, 55.0f);
    s.rebuilt = s.hits = 0;
    rebuild(&s, 0);
    check("hp_rebuilds_path_only", s.rebuilt == 3); /* hpbar + player + root */
    check("hp_checksum_match", s.nodes[0].built == full_sum(&s, 0, 0.0f, 0.0f));

    set_pos(&s, 5, 10.0f, 1.0f);
    s.rebuilt = s.hits = 0;
    rebuild(&s, 0);
    check("move_rebuilds_subtree", s.rebuilt == 1 + 1 + COMPS);
    check("move_checksum_match", s.nodes[0].built == full_sum(&s, 0, 0.0f, 0.0f));

    int leaf = player_node(2) + 1 + COMP_AURA;
    mark_dirty(&s, leaf);
    int reach = 1;
    for (int i = leaf; i >= 0; i = s.nodes[i].parent)
        if (!s.nodes[i].dirty) reach = 0;
    check("mark_reaches_root", reach);
    rebuild(&s, 0);

    rng_s = 12345;
    int all = 1;
    for (int f = 0; f < 300; f++) {
        for (int p = 0; p < 17; p++) {
            if (rngf() < 0.2f) set_pos(&s, p, rngf() * 50.0f, rngf() * 6.0f);
            if (rngf() < 0.1f) set_hp(&s, p, rngf() * 100.0f);
        }
        rebuild(&s, 0);
        if (s.nodes[0].built != full_sum(&s, 0, 0.0f, 0.0f)) all = 0;
    }
    check("random_300f_consistent", all);

    printf("all_ok=%d\n", failures == 0);
    free(s.nodes);
    return failures ? 1 : 0;
}

static int run_scale(int players) {
    Scene s;
    scene_init(&s, players);
    rebuild(&s, 0); /* initial full build, not measured */

    rng_s = 0x9e3779b97f4a7c15ull ^ (unsigned long long)players;
    const int frames = 3600; /* 60s @ 60fps */
    long long sum_rebuilt = 0, sum_hits = 0, sum_ns = 0, sum_full_ns = 0, max_ns = 0;
    int mismatches = 0;

    for (int f = 0; f < frames; f++) {
        for (int p = 0; p < players; p++) {
            if (rngf() < 0.05f) set_pos(&s, p, rngf() * 100.0f, rngf() * 6.28f);
            if (rngf() < 0.01f) set_hp(&s, p, rngf() * 100.0f);
        }
        s.rebuilt = 0;
        s.hits = 0;
        long long t0 = now_ns();
        rebuild(&s, 0);
        long long t1 = now_ns();
        unsigned inc = s.nodes[0].built;
        long long t2 = now_ns();
        unsigned full = full_sum(&s, 0, 0.0f, 0.0f);
        long long t3 = now_ns();
        if (inc != full) mismatches++;
        sum_rebuilt += s.rebuilt;
        sum_hits += s.hits;
        sum_ns += t1 - t0;
        sum_full_ns += t3 - t2;
        if (t1 - t0 > max_ns) max_ns = t1 - t0;
    }

    double hit = (double)sum_hits / (double)(sum_hits + sum_rebuilt);
    printf("scale=%d total_nodes=%d frames=%d rebuilt_per_frame=%.1f main_us_per_frame=%.2f "
           "hit_ratio=%.4f full_rebuild_us=%.2f max_us=%.2f checksum_match=%d\n",
           players, s.n, frames,
           (double)sum_rebuilt / frames, (double)sum_ns / frames / 1000.0,
           hit, (double)sum_full_ns / frames / 1000.0, (double)max_ns / 1000.0,
           mismatches == 0);
    free(s.nodes);
    return mismatches ? 1 : 0;
}

static int bench(void) {
    int rc = 0;
    rc |= run_scale(10);
    rc |= run_scale(50);
    rc |= run_scale(200);
    return rc;
}

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "bench") == 0) return bench();
    return selftest();
}
