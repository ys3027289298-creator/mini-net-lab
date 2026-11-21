/* aoi_sync: AOI-driven snapshot sync. The visible set decides packet content:
   enters get a full record, updates a masked delta, leavers exactly one
   closing record, re-enters a full record again. An empty visible set is
   reported as an anomaly, never silently upgraded to a full sync. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_ENT 16
#define MAX_REC (2 * MAX_ENT)   /* up to MAX_ENT enters + MAX_ENT leaves per tick */

#define F_X     1u
#define F_Y     2u
#define F_VX    4u
#define F_VY    8u
#define F_FLAGS 16u
#define F_ALL   (F_X | F_Y | F_VX | F_VY | F_FLAGS)

enum { REC_FULL = 0, REC_DELTA = 1, REC_LEAVE = 2 };

typedef struct {
    int id;
    float x, y, vx, vy;
    unsigned flags;
    int alive;
} Ent;

typedef struct {
    Ent e[MAX_ENT];
    int n;
    float radius;
} World;

typedef struct {
    int id;
    int qx, qy, qvx, qvy;   /* quantized wire values */
    unsigned flags;
} QState;

typedef struct {
    int id;
    int kind;
    unsigned mask;          /* REC_DELTA: which fields are present */
    QState q;
} Rec;

typedef struct {
    unsigned tick;
    Rec r[MAX_REC];
    int n;
    int anomaly;            /* visible set was empty this tick */
} Packet;

typedef struct {
    QState known[MAX_ENT];  /* entities the client currently holds, last-sent state */
    int known_n;
    int anomalies;          /* empty-visible-set events seen */
    int closings;           /* closing records sent */
} Session;

typedef struct {
    Ent e[MAX_ENT];
    int n;
    int errors;             /* protocol violations (delta/leave without baseline) */
} Client;

/* ---------------- world ---------------- */

static void world_init(World* w, float radius) {
    memset(w, 0, sizeof(*w));
    w->radius = radius;
}

static int world_add(World* w, int id, float x, float y) {
    if (w->n >= MAX_ENT) return -1;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return -1;
    Ent* e = &w->e[w->n++];
    memset(e, 0, sizeof(*e));
    e->id = id; e->x = x; e->y = y; e->alive = 1;
    return w->n - 1;
}

static Ent* world_find(World* w, int id) {
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return &w->e[i];
    return NULL;
}

static const Ent* world_cfind(const World* w, int id) {
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return &w->e[i];
    return NULL;
}

static int world_move(World* w, int id, float x, float y) {
    Ent* e = world_find(w, id);
    if (!e) return -1;
    e->x = x; e->y = y;
    return 0;
}

static int world_set_vel(World* w, int id, float vx, float vy) {
    Ent* e = world_find(w, id);
    if (!e) return -1;
    e->vx = vx; e->vy = vy;
    return 0;
}

static int world_set_flags(World* w, int id, unsigned flags) {
    Ent* e = world_find(w, id);
    if (!e) return -1;
    e->flags = flags;
    return 0;
}

static int world_remove(World* w, int id) {
    Ent* e = world_find(w, id);
    if (!e) return -1;
    e->alive = 0;
    return 0;
}

/* visible set: alive entities within radius of (px,py), excluding self */
static int world_visible(const World* w, int self, float px, float py, int* out, int cap) {
    int n = 0;
    float r2 = w->radius * w->radius;
    for (int i = 0; i < w->n; i++) {
        const Ent* e = &w->e[i];
        if (!e->alive || e->id == self) continue;
        float dx = e->x - px, dy = e->y - py;
        if (dx * dx + dy * dy <= r2) {
            if (n < cap) out[n] = e->id;
            n++;
        }
    }
    return n;
}

/* ---------------- server ---------------- */

static int quant(float v, float scale) { return (int)lrintf(v * scale); }

static QState quantize(const Ent* e, float scale) {
    QState q;
    q.id = e->id;
    q.qx = quant(e->x, scale);
    q.qy = quant(e->y, scale);
    q.qvx = quant(e->vx, scale);
    q.qvy = quant(e->vy, scale);
    q.flags = e->flags;
    return q;
}

static unsigned diff_mask(const QState* a, const QState* b) {
    unsigned m = 0;
    if (a->qx != b->qx) m |= F_X;
    if (a->qy != b->qy) m |= F_Y;
    if (a->qvx != b->qvx) m |= F_VX;
    if (a->qvy != b->qvy) m |= F_VY;
    if (a->flags != b->flags) m |= F_FLAGS;
    return m;
}

static int known_idx(const Session* s, int id) {
    for (int i = 0; i < s->known_n; i++) if (s->known[i].id == id) return i;
    return -1;
}

static int push_rec(Packet* p, int kind, int id, unsigned mask, const QState* q) {
    if (p->n >= MAX_REC) return -1;
    Rec* r = &p->r[p->n++];
    r->kind = kind; r->id = id; r->mask = mask;
    if (q) r->q = *q;
    else memset(&r->q, 0, sizeof(r->q));
    return 0;
}

/* Build this tick's packet for one client. Returns record count, -1 on bad args. */
static int server_tick(const World* w, Session* s, int self, float px, float py,
                       unsigned tick, float scale, Packet* out) {
    if (out == NULL || s == NULL) return -1;
    if (scale <= 0.0f) return -1;
    out->tick = tick;
    out->n = 0;
    out->anomaly = 0;

    int vis[MAX_ENT];
    int vn = world_visible(w, self, px, py, vis, MAX_ENT);
    if (vn == 0) {                 /* empty visible set: report, do not fall back */
        s->anomalies++;
        out->anomaly = 1;
    }

    /* enters -> full record; known -> masked delta on quantized change */
    for (int i = 0; i < vn; i++) {
        const Ent* e = world_cfind(w, vis[i]);
        QState q = quantize(e, scale);
        int k = known_idx(s, e->id);
        if (k < 0) {
            if (push_rec(out, REC_FULL, e->id, F_ALL, &q) < 0) return -1;
            if (s->known_n < MAX_ENT) s->known[s->known_n++] = q;
        } else {
            unsigned mask = diff_mask(&s->known[k], &q);
            if (mask) {
                if (push_rec(out, REC_DELTA, e->id, mask, &q) < 0) return -1;
                s->known[k] = q;
            }
        }
    }

    /* leavers (incl. despawned) -> exactly one closing record, then forgotten */
    for (int k = 0; k < s->known_n; ) {
        int id = s->known[k].id;
        int still = 0;
        for (int i = 0; i < vn; i++) if (vis[i] == id) { still = 1; break; }
        if (still) { k++; continue; }
        if (push_rec(out, REC_LEAVE, id, 0, NULL) < 0) return -1;
        s->closings++;
        s->known[k] = s->known[--s->known_n];
    }
    return out->n;
}

/* reference full sync: every alive entity as a full record, every tick */
static int full_sync_tick(const World* w, int self, unsigned tick, float scale, Packet* out) {
    out->tick = tick;
    out->n = 0;
    out->anomaly = 0;
    for (int i = 0; i < w->n; i++) {
        const Ent* e = &w->e[i];
        if (!e->alive || e->id == self) continue;
        QState q = quantize(e, scale);
        if (push_rec(out, REC_FULL, e->id, F_ALL, &q) < 0) return -1;
    }
    return out->n;
}

/* ---------------- client ---------------- */

static int client_find(const Client* c, int id) {
    for (int i = 0; i < c->n; i++) if (c->e[i].id == id) return i;
    return -1;
}

static void decode_fields(Ent* e, const QState* q, unsigned mask, float scale) {
    if (mask & F_X) e->x = (float)q->qx / scale;
    if (mask & F_Y) e->y = (float)q->qy / scale;
    if (mask & F_VX) e->vx = (float)q->qvx / scale;
    if (mask & F_VY) e->vy = (float)q->qvy / scale;
    if (mask & F_FLAGS) e->flags = q->flags;
}

static void client_apply(Client* c, const Packet* p, float scale) {
    for (int i = 0; i < p->n; i++) {
        const Rec* r = &p->r[i];
        int idx = client_find(c, r->id);
        switch (r->kind) {
        case REC_FULL: {
            Ent e;
            memset(&e, 0, sizeof(e));
            e.id = r->id;
            e.alive = 1;
            decode_fields(&e, &r->q, F_ALL, scale);
            if (idx < 0) {
                if (c->n < MAX_ENT) c->e[c->n++] = e;
                else c->errors++;
            } else {
                c->e[idx] = e;
            }
            break;
        }
        case REC_DELTA:
            if (idx < 0) { c->errors++; break; }   /* delta without baseline */
            decode_fields(&c->e[idx], &r->q, r->mask, scale);
            break;
        case REC_LEAVE:
            if (idx < 0) { c->errors++; break; }   /* leave for unknown entity */
            c->e[idx] = c->e[--c->n];
            break;
        default:
            c->errors++;
        }
    }
}

/* ---------------- consistency check ---------------- */

/* AOI client must hold exactly the visible set, each entity field-by-field
   identical to the full-sync reference client. Zero residual on leave. */
static int consistent(const World* w, const Client* c, const Client* ref,
                      int self, float px, float py) {
    int vis[MAX_ENT];
    int vn = world_visible(w, self, px, py, vis, MAX_ENT);
    if (c->n != vn) return 0;                      /* residual or missing entities */
    for (int i = 0; i < vn; i++) {
        int a = client_find(c, vis[i]);
        int b = client_find(ref, vis[i]);
        if (a < 0 || b < 0) return 0;
        const Ent* x = &c->e[a];
        const Ent* y = &ref->e[b];
        if (x->x != y->x || x->y != y->y || x->vx != y->vx || x->vy != y->vy ||
            x->flags != y->flags) return 0;        /* field-by-field equality */
    }
    return 1;
}

/* ---------------- self test ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    const float scale = 64.0f;
    const int self = 1;
    World w;
    Session s;
    Client c, ref;
    Packet p, fp;
    unsigned tick = 0;

    world_init(&w, 15.0f);
    memset(&s, 0, sizeof(s));
    memset(&c, 0, sizeof(c));
    world_add(&w, self, 0, 0);
    world_add(&w, 2, 5, 0);
    world_add(&w, 3, 100, 0);

    /* tick 1: entity 2 enters -> full record */
    check("tick_ok", server_tick(&w, &s, self, 0, 0, ++tick, scale, &p) == 1);
    check("enter_full", p.n == 1 && p.r[0].kind == REC_FULL && p.r[0].id == 2 &&
                        p.r[0].mask == F_ALL);
    client_apply(&c, &p, scale);
    memset(&ref, 0, sizeof(ref));
    full_sync_tick(&w, self, tick, scale, &fp);
    client_apply(&ref, &fp, scale);
    check("consistent_enter", consistent(&w, &c, &ref, self, 0, 0));

    /* tick 2: small move -> delta with x only */
    world_move(&w, 2, 6.5f, 0);
    server_tick(&w, &s, self, 0, 0, ++tick, scale, &p);
    check("delta_x_only", p.n == 1 && p.r[0].kind == REC_DELTA && p.r[0].mask == F_X);
    client_apply(&c, &p, scale);
    memset(&ref, 0, sizeof(ref));
    full_sync_tick(&w, self, tick, scale, &fp);
    client_apply(&ref, &fp, scale);
    check("consistent_delta", consistent(&w, &c, &ref, self, 0, 0));

    /* tick 3: velocity change -> delta vx|vy */
    world_set_vel(&w, 2, 1.0f, -2.0f);
    server_tick(&w, &s, self, 0, 0, ++tick, scale, &p);
    check("delta_vel", p.n == 1 && p.r[0].kind == REC_DELTA &&
                       p.r[0].mask == (F_VX | F_VY));
    client_apply(&c, &p, scale);

    /* tick 4: flags change -> delta flags */
    world_set_flags(&w, 2, 7u);
    server_tick(&w, &s, self, 0, 0, ++tick, scale, &p);
    check("delta_flags", p.n == 1 && p.r[0].kind == REC_DELTA && p.r[0].mask == F_FLAGS);
    client_apply(&c, &p, scale);
    memset(&ref, 0, sizeof(ref));
    full_sync_tick(&w, self, tick, scale, &fp);
    client_apply(&ref, &fp, scale);
    check("consistent_fields", consistent(&w, &c, &ref, self, 0, 0));

    /* tick 5: nothing changed -> empty packet, no anomaly */
    server_tick(&w, &s, self, 0, 0, ++tick, scale, &p);
    check("idle_empty", p.n == 0 && p.anomaly == 0);

    /* tick 6: entity 2 leaves radius -> one closing record, client drops to zero,
       visible set now empty -> anomaly reported, no full-sync fallback */
    world_move(&w, 2, 100, 0);
    server_tick(&w, &s, self, 0, 0, ++tick, scale, &p);
    check("leave_closing", p.n == 1 && p.r[0].kind == REC_LEAVE && p.r[0].id == 2);
    check("empty_anomaly", p.anomaly == 1 && s.anomalies == 1);
    check("no_full_fallback", p.r[0].kind != REC_FULL && p.r[0].kind != REC_DELTA);
    client_apply(&c, &p, scale);
    check("zero_residual", c.n == 0);
    memset(&ref, 0, sizeof(ref));
    full_sync_tick(&w, self, tick, scale, &fp);
    client_apply(&ref, &fp, scale);
    check("consistent_empty", consistent(&w, &c, &ref, self, 0, 0));

    /* tick 7: still empty -> anomaly again, but no duplicate closing */
    server_tick(&w, &s, self, 0, 0, ++tick, scale, &p);
    check("closing_once", p.n == 0 && s.closings == 1 && s.anomalies == 2);

    /* tick 8: entity 2 re-enters -> full record again (client has no baseline) */
    world_move(&w, 2, 3, 0);
    server_tick(&w, &s, self, 0, 0, ++tick, scale, &p);
    check("reenter_full", p.n == 1 && p.r[0].kind == REC_FULL && p.r[0].mask == F_ALL);
    client_apply(&c, &p, scale);
    memset(&ref, 0, sizeof(ref));
    full_sync_tick(&w, self, tick, scale, &fp);
    client_apply(&ref, &fp, scale);
    check("consistent_reenter", consistent(&w, &c, &ref, self, 0, 0));

    /* tick 9: entity despawns while visible -> closing record, zero residual */
    world_remove(&w, 2);
    server_tick(&w, &s, self, 0, 0, ++tick, scale, &p);
    check("despawn_closing", p.n == 1 && p.r[0].kind == REC_LEAVE && p.r[0].id == 2);
    client_apply(&c, &p, scale);
    check("despawn_zero_residual", c.n == 0);

    /* protocol violations are counted, not silently absorbed */
    Packet bad;
    memset(&bad, 0, sizeof(bad));
    QState q = { 42, 0, 0, 0, 0, 0 };
    push_rec(&bad, REC_DELTA, 42, F_X, &q);
    client_apply(&c, &bad, scale);
    check("orphan_delta_error", c.errors == 1);

    check("bad_scale", server_tick(&w, &s, self, 0, 0, ++tick, 0.0f, &p) == -1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
