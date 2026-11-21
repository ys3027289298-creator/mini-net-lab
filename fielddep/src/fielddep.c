/* fielddep: field dependency graph — derive redundant fields on the receiver
 * instead of transmitting them. Reports derivation order, per-field error
 * against full transmission, bandwidth saved, and fields on dependency
 * cycles or with missing dependencies. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define FD_MAX_FIELDS 16
#define FD_MAX_DEPS 4

typedef float (*DeriveFn)(const float* v);

typedef struct {
    const char* name;
    int bytes;                       /* wire size if transmitted */
    int derived;
    DeriveFn fn;
    int deps[FD_MAX_DEPS];
    int ndeps;
    float max_err;                   /* declared derivation error bound */
} Field;

typedef struct {
    Field f[FD_MAX_FIELDS];
    int n;
    int order[FD_MAX_FIELDS];        /* derived fields in recompute order */
    int norder;
    int cycle[FD_MAX_FIELDS];        /* fields found on a dependency cycle */
    int ncycle;
    int missing[FD_MAX_FIELDS];      /* fields referencing unknown deps */
    int nmissing;
} Graph;

static void fd_init(Graph* g) { memset(g, 0, sizeof(*g)); }

static int fd_add_base(Graph* g, const char* name, int bytes) {
    if (g->n >= FD_MAX_FIELDS) return -1;
    Field* f = &g->f[g->n];
    f->name = name; f->bytes = bytes; f->derived = 0;
    f->fn = NULL; f->ndeps = 0; f->max_err = 0.0f;
    return g->n++;
}

static int fd_add_derived(Graph* g, const char* name, int bytes,
                          DeriveFn fn, const int* deps, int ndeps, float max_err) {
    if (g->n >= FD_MAX_FIELDS) return -1;
    if (fn == NULL || ndeps < 1 || ndeps > FD_MAX_DEPS) return -1;
    Field* f = &g->f[g->n];
    f->name = name; f->bytes = bytes; f->derived = 1;
    f->fn = fn; f->ndeps = ndeps; f->max_err = max_err;
    for (int i = 0; i < ndeps; i++) f->deps[i] = deps[i];
    return g->n++;
}

/* DFS colors */
enum { WHITE = 0, GRAY = 1, BLACK = 2 };

static void fd_visit(Graph* g, int u, int* color, int* stack, int* sp, int* ok) {
    color[u] = GRAY;
    stack[(*sp)++] = u;
    for (int i = 0; i < g->f[u].ndeps; i++) {
        int d = g->f[u].deps[i];
        if (d < 0 || d >= g->n) {                    /* missing dependency */
            int seen = 0;
            for (int j = 0; j < g->nmissing; j++) if (g->missing[j] == u) seen = 1;
            if (!seen) g->missing[g->nmissing++] = u;
            *ok = 0;
            continue;
        }
        if (!g->f[d].derived) continue;              /* base field: always ready */
        if (color[d] == GRAY) {                      /* back edge: record cycle */
            for (int j = 0; j < *sp; j++) {
                if (stack[j] == d) {
                    for (int k = j; k < *sp; k++) {
                        int seen = 0;
                        for (int m = 0; m < g->ncycle; m++) if (g->cycle[m] == stack[k]) seen = 1;
                        if (!seen) g->cycle[g->ncycle++] = stack[k];
                    }
                    break;
                }
            }
            *ok = 0;
        } else if (color[d] == WHITE) {
            fd_visit(g, d, color, stack, sp, ok);
        }
    }
    (*sp)--;
    color[u] = BLACK;
    g->order[g->norder++] = u;                       /* postorder = deps first */
}

/* build recompute order; 0 ok, -1 on cycle or missing dep (see g->cycle/g->missing) */
static int fd_build(Graph* g) {
    int color[FD_MAX_FIELDS] = { 0 };
    int stack[FD_MAX_FIELDS];
    int sp = 0, ok = 1;
    g->norder = 0; g->ncycle = 0; g->nmissing = 0;
    for (int u = 0; u < g->n; u++) {
        if (g->f[u].derived && color[u] == WHITE)
            fd_visit(g, u, color, stack, &sp, &ok);
    }
    return ok ? 0 : -1;
}

/* recompute derived fields in dependency order; call after base fields are set */
static void fd_derive_all(const Graph* g, float* v) {
    for (int i = 0; i < g->norder; i++) {
        int u = g->order[i];
        v[u] = g->f[u].fn(v);
    }
}

static int fd_full_bytes(const Graph* g) {
    int b = 0;
    for (int i = 0; i < g->n; i++) b += g->f[i].bytes;
    return b;
}

static int fd_wire_bytes(const Graph* g) {
    int b = 0;
    for (int i = 0; i < g->n; i++) if (!g->f[i].derived) b += g->f[i].bytes;
    return b;
}

/* per-field |got - full| for derived fields; returns worst offender or -1 */
static int fd_check(const Graph* g, const float* full, const float* got, float* errs) {
    int worst = -1;
    float wmax = -1.0f;
    for (int i = 0; i < g->norder; i++) {
        int u = g->order[i];
        errs[u] = fabsf(got[u] - full[u]);
        if (errs[u] > wmax) { wmax = errs[u]; worst = u; }
    }
    return worst;
}

/* ---- test rules ---- */

static float f_speed(const float* v) { return hypotf(v[2], v[3]); }        /* vx, vy */
static float f_dist(const float* v) { return hypotf(v[0], v[1]); }         /* x, y */
static float f_energy(const float* v) { return 0.5f * v[4] * v[4]; }       /* speed */
static float f_cyc_a(const float* v) { return v[1] + 1.0f; }
static float f_cyc_b(const float* v) { return v[2] + 1.0f; }
static float f_cyc_c(const float* v) { return v[0] + 1.0f; }

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int find_field(const Graph* g, const int* list, int n, int id) {
    (void)g;
    for (int i = 0; i < n; i++) if (list[i] == id) return 1;
    return 0;
}

int main(void) {
    Graph g;
    fd_init(&g);
    /* base: x y vx vy (4B each); derived: speed, dist, energy (4B each) */
    check("add_x", fd_add_base(&g, "x", 4) == 0);
    check("add_y", fd_add_base(&g, "y", 4) == 1);
    check("add_vx", fd_add_base(&g, "vx", 4) == 2);
    check("add_vy", fd_add_base(&g, "vy", 4) == 3);
    int d_vxvy[2] = { 2, 3 }, d_xy[2] = { 0, 1 }, d_sp[1] = { 4 };
    check("add_speed", fd_add_derived(&g, "speed", 4, f_speed, d_vxvy, 2, 0.05f) == 4);
    check("add_dist", fd_add_derived(&g, "dist", 4, f_dist, d_xy, 2, 0.05f) == 5);
    check("add_energy", fd_add_derived(&g, "energy", 4, f_energy, d_sp, 1, 0.5f) == 6);
    check("no_rule", fd_add_derived(&g, "bad", 4, NULL, d_sp, 1, 0.0f) == -1);

    check("build_ok", fd_build(&g) == 0);
    check("all_derived_ordered", g.norder == 3);
    /* energy depends on speed: speed must come first */
    int pos_speed = -1, pos_energy = -1;
    for (int i = 0; i < g.norder; i++) {
        if (g.order[i] == 4) pos_speed = i;
        if (g.order[i] == 6) pos_energy = i;
    }
    check("dep_order", pos_speed >= 0 && pos_energy > pos_speed);

    /* bandwidth: 28B full -> 16B wire */
    check("full_bytes", fd_full_bytes(&g) == 28);
    check("wire_bytes", fd_wire_bytes(&g) == 16);
    float saved = 1.0f - (float)fd_wire_bytes(&g) / (float)fd_full_bytes(&g);
    check("saved_43pct", fabsf(saved - (12.0f / 28.0f)) < 1e-6f);
    printf("bandwidth_full=%d wire=%d saved=%.1f%%\n",
           fd_full_bytes(&g), fd_wire_bytes(&g), saved * 100.0f);

    /* exact base -> derived matches full transmission field by field */
    float full[7] = { 3.0f, 4.0f, 1.5f, 2.0f, 0, 0, 0 };
    fd_derive_all(&g, full);                    /* reference: full transmit values */
    float got[7] = { 3.0f, 4.0f, 1.5f, 2.0f, 0, 0, 0 };
    fd_derive_all(&g, got);
    float errs[7] = { 0 };
    fd_check(&g, full, got, errs);
    check("exact_consistent", errs[4] == 0.0f && errs[5] == 0.0f && errs[6] == 0.0f);
    check("speed_value", fabsf(got[4] - 2.5f) < 1e-6f);
    check("dist_value", fabsf(got[5] - 5.0f) < 1e-6f);
    check("energy_value", fabsf(got[6] - 3.125f) < 1e-6f);

    /* quantized base (as on the wire) -> derived error within declared bound */
    float q[7] = { 3.002f, 3.998f, 1.51f, 1.99f, 0, 0, 0 };
    fd_derive_all(&g, q);
    fd_check(&g, full, q, errs);
    int within = errs[4] <= g.f[4].max_err && errs[5] <= g.f[5].max_err && errs[6] <= g.f[6].max_err;
    check("err_within_bound", within);
    printf("err_speed=%.5f err_dist=%.5f err_energy=%.5f\n", errs[4], errs[5], errs[6]);

    /* cycle: a<-b<-c<-a, all three reported */
    Graph g2;
    fd_init(&g2);
    int d1[1] = { 1 }, d2[1] = { 2 }, d0[1] = { 0 };
    fd_add_derived(&g2, "a", 4, f_cyc_a, d1, 1, 0.0f);
    fd_add_derived(&g2, "b", 4, f_cyc_b, d2, 1, 0.0f);
    fd_add_derived(&g2, "c", 4, f_cyc_c, d0, 1, 0.0f);
    check("cycle_detected", fd_build(&g2) == -1);
    check("cycle_fields", g2.ncycle == 3 &&
          find_field(&g2, g2.cycle, g2.ncycle, 0) &&
          find_field(&g2, g2.cycle, g2.ncycle, 1) &&
          find_field(&g2, g2.cycle, g2.ncycle, 2));

    /* missing dep: references field id 99 */
    Graph g3;
    fd_init(&g3);
    fd_add_base(&g3, "x", 4);
    int d99[1] = { 99 };
    fd_add_derived(&g3, "orphan", 4, f_dist, d99, 1, 0.0f);
    check("missing_detected", fd_build(&g3) == -1);
    check("missing_field", g3.nmissing == 1 && g3.missing[0] == 1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
