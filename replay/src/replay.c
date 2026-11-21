/* replay: bit-exact deterministic replay with per-frame input and state
   hashes, field-level divergence localization, and a bounded replay window.

   Why two replays of the same input can disagree
   ----------------------------------------------
   The simulation sums one interaction term per peer entity. Floating-point
   addition is not associative, so summing those terms in storage order makes
   the result depend on allocation/spawn history. The canonical mode below
   always sums in ascending entity-id order; the storage-order mode exists
   only as a control that reproduces the bug (and is rejected on record).

   Determinism contract
   --------------------
   - canonical summation order for every reduction over entities;
   - forces are computed for every entity before any entity is integrated;
   - the RNG advances exactly once per step, before any input branch;
   - inputs are stored as raw IEEE-754 bit patterns, never as decimal text;
   - states are compared bit for bit, never with ==, so +0.0/-0.0 and NaN
     payload differences are visible;
   - the build uses -ffp-contract=off and must not use fast-math.

   Recording layout (little-endian, fixed records, strictly linear in frames):
     header 48B : magic, version, flags, frame count, spawn count, seed,
                  serialized byte count, reserved, initial state hash (128b)
     spawns     : 16B each { id, x bits, y bits, hp }
     per frame  : 276B { frame, tick, input(buttons/ax/ay/pad), state hash
                        (128b), full canonical state (236B) }
   total = 48 + 16 * spawn_count + 276 * frame_count

   Hash collision notes are printed by the self-test and documented in
   README.md. The hash is a fast frame filter only: every frame that survives
   the filter is also compared bitwise, so a collision cannot turn a differing
   state into a passing replay.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#if defined(__FAST_MATH__)
#error "replay: -ffast-math may reassociate sums and breaks determinism"
#endif
#if defined(__FLT_EVAL_METHOD__) && __FLT_EVAL_METHOD__ != 0
#error "replay: excess float evaluation precision breaks bit-exact replay"
#endif

#define MAX_ENT        8
#define MAX_FRAMES     192
#define WIN_CAP        8
#define DT             (1.0f / 60.0f)

#define FNV_OFFSET     1469598103934665603ULL
#define FNV_PRIME      1099511628211ULL

#define BTN_HIT        1u
#define BTN_JUMP       4u

#define ORDER_CANONICAL 0
#define ORDER_STORAGE   1

#define REPLAY_OK           0
#define REPLAY_DIVERGED     1
#define REPLAY_BAD_REC      2
#define REPLAY_WINDOW_SHORT (-1)
#define REPLAY_WINDOW_FUTURE (-2)

#define REC_MAGIC       0x4d4e5250u   /* "MNRP" */
#define REC_VERSION     1u
#define REC_FLAG_CANON  1u
#define REC_HEADER_BYTES 48u
#define REC_SPAWN_BYTES  16u
#define REC_FRAME_BYTES  276u
#define REC_MAX_BYTES    (REC_HEADER_BYTES + REC_SPAWN_BYTES * MAX_ENT + \
                          REC_FRAME_BYTES * MAX_FRAMES)

/* ------------------------------------------------------------------ state */

typedef struct { uint32_t buttons; float ax, ay; } Input;

typedef struct {
    int32_t id;
    float x, y, vx, vy;
    int32_t hp;
    int alive;
} Ent;

typedef struct {
    Ent e[MAX_ENT];
    int n;
    uint32_t tick, rng;
    int order;          /* ORDER_CANONICAL / ORDER_STORAGE (control only) */
} World;

typedef struct {
    uint32_t id, alive;
    uint32_t x, y, vx, vy;   /* raw IEEE-754 bits */
    int32_t hp;
} EntBits;

typedef struct {
    uint32_t n, tick, rng;
    EntBits e[MAX_ENT];      /* canonical order: ascending id */
} Snapshot;

typedef struct { uint32_t id; float x, y; int32_t hp; } Spawn;

typedef struct { uint64_t lo, hi; } Hash128;

typedef struct {
    uint32_t frame, tick;
    Input in;
    uint32_t pad;
    Hash128 hash;
    Snapshot snap;
} RecFrame;

typedef struct {
    uint32_t version, flags, frame_count, spawn_count, seed, bytes;
    Spawn spawn[MAX_ENT];
    Hash128 initial;
    RecFrame frame[MAX_FRAMES];
} Recording;

typedef struct {
    uint32_t frames_checked;
    uint32_t hash_diffs;        /* replay hash != recorded hash */
    uint32_t bit_diffs;         /* replay bytes != recorded bytes */
    uint32_t integrity_diffs;   /* recorded bytes != recorded hash */
    uint32_t collision_guard;   /* hash equal, bytes differ: bitwise wins */
    uint32_t first_frame;
    char first_field[40];
    uint32_t rec_bits, live_bits;
    int status;
} VerifyResult;

typedef struct {
    Snapshot snap[WIN_CAP];
    uint32_t frame[WIN_CAP];
    uint32_t head, n;
    uint32_t window_short;
} History;

/* ----------------------------------------------------------- bit helpers */

static uint32_t fbits(float f) {
    uint32_t u;
    memcpy(&u, &f, sizeof u);
    return u;
}

static float bitsf(uint32_t u) {
    float f;
    memcpy(&f, &u, sizeof f);
    return f;
}

static uint64_t hash_u32(uint64_t h, uint32_t v) {
    for (int i = 0; i < 4; i++) {
        h ^= (uint64_t)((v >> (8 * i)) & 0xffu);
        h *= FNV_PRIME;
    }
    return h;
}

/* Two independent 64-bit accumulators over the same canonical fields. The
   second one walks the fields in reverse and starts from a different basis,
   so a single FNV weakness is not shared. */
static Hash128 snapshot_hash(const Snapshot* s) {
    uint64_t a = FNV_OFFSET;
    uint64_t b = 0x9e3779b97f4a7c15ULL;

    a = hash_u32(a, s->n);
    a = hash_u32(a, s->tick);
    a = hash_u32(a, s->rng);
    for (int k = 0; k < MAX_ENT; k++) {
        const EntBits* e = &s->e[k];
        a = hash_u32(a, e->id);
        a = hash_u32(a, e->alive);
        a = hash_u32(a, e->x);
        a = hash_u32(a, e->y);
        a = hash_u32(a, e->vx);
        a = hash_u32(a, e->vy);
        a = hash_u32(a, (uint32_t)e->hp);
    }

    static const uint64_t P2 = 0x100000001b3ULL ^ 0x5bd1e995ULL;
    for (int k = MAX_ENT - 1; k >= 0; k--) {
        const EntBits* e = &s->e[k];
        uint32_t f[7] = { (uint32_t)e->hp, e->vy, e->vx, e->y, e->x, e->alive, e->id };
        for (int q = 0; q < 7; q++) {
            uint32_t v = f[q];
            for (int i = 3; i >= 0; i--) {
                b ^= (uint64_t)((v >> (8 * i)) & 0xffu);
                b *= P2;
            }
        }
    }
    b = hash_u32(b, s->rng);
    b = hash_u32(b, s->tick);
    b = hash_u32(b, s->n);

    Hash128 h;
    h.lo = a;
    h.hi = b;
    return h;
}

/* ------------------------------------------------------------ simulation */

static uint32_t rng_next(uint32_t* s) {
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

/* Mass by id. The wide magnitude spread makes summation order observable in
   ordinary float32 arithmetic. */
static float mass_of(int32_t id) {
    static const float m[8] = { 1.0f, 1.0f, 1e6f, 1e-4f, 30.0f, 1e4f, 1.0f, 1.0f };
    return m[(uint32_t)id & 7u];
}

static void sort_ids(const World* w, int* ord) {
    for (int i = 0; i < w->n; i++) ord[i] = i;
    for (int i = 1; i < w->n; i++) {
        int k = ord[i], j = i - 1;
        while (j >= 0 && w->e[ord[j]].id > w->e[k].id) {
            ord[j + 1] = ord[j];
            j--;
        }
        ord[j + 1] = k;
    }
}

static int world_init(World* w, uint32_t seed, int order) {
    if (w == NULL) return -1;
    if (order != ORDER_CANONICAL && order != ORDER_STORAGE) return -1;
    memset(w, 0, sizeof *w);
    w->rng = seed ? seed : 0x1234567u;
    w->order = order;
    return 0;
}

static int world_spawn(World* w, int32_t id, float x, float y, int32_t hp) {
    if (w == NULL || w->n >= MAX_ENT) return -1;
    if (!isfinite(x) || !isfinite(y)) return -1;
    for (int i = 0; i < w->n; i++)
        if (w->e[i].id == id) return -1;
    Ent* e = &w->e[w->n++];
    e->id = id;
    e->x = x;
    e->y = y;
    e->vx = 0.0f;
    e->vy = 0.0f;
    e->hp = hp;
    e->alive = 1;
    return 0;
}

static void world_step(World* w, Input in) {
    uint32_t r = rng_next(&w->rng);   /* exactly once, before any branch */
    float gust = ((float)((r >> 8) & 0xffffu) / 65535.0f - 0.5f) * 0.05f;

    int ord[MAX_ENT];
    sort_ids(w, ord);

    /* Phase 1: forces, read-only over the whole state. */
    float ax[MAX_ENT], ay[MAX_ENT];
    for (int i = 0; i < w->n; i++) {
        float sx = 0.0f, sy = 0.0f;
        for (int k = 0; k < w->n; k++) {
            int j = (w->order == ORDER_CANONICAL) ? ord[k] : k;
            if (j == i) continue;
            const Ent* e = &w->e[i];
            const Ent* o = &w->e[j];
            float dx = o->x - e->x;
            float dy = o->y - e->y;
            float d2 = dx * dx + dy * dy + 1.0f;
            float inv = 1.0f / d2;
            float c = mass_of(o->id) * inv * inv;
            sx += dx * c;
            sy += dy * c;
        }
        ax[i] = sx;
        ay[i] = sy;
    }

    /* Phase 2: integrate in canonical order (per-entity, order free). */
    for (int k = 0; k < w->n; k++) {
        int i = ord[k];
        Ent* e = &w->e[i];
        e->vx += ax[i] * DT;
        e->vy += ay[i] * DT;
        e->vx *= 0.99f;
        e->vy *= 0.99f;
        if (e->id == 1) {
            e->vx += gust + in.ax * 0.5f;
            e->vy += in.ay * 0.5f;
            if ((in.buttons & BTN_HIT) && e->hp > 0) e->hp -= 1;
            if (in.buttons & BTN_JUMP) e->vy += 1.0f;
        }
        e->x += e->vx * DT;
        e->y += e->vy * DT;
    }
    w->tick++;
}

static void world_snapshot(const World* w, Snapshot* s) {
    memset(s, 0, sizeof *s);
    s->n = (uint32_t)w->n;
    s->tick = w->tick;
    s->rng = w->rng;
    int ord[MAX_ENT];
    sort_ids(w, ord);
    for (int k = 0; k < w->n; k++) {
        const Ent* e = &w->e[ord[k]];
        EntBits* b = &s->e[k];
        b->id = (uint32_t)e->id;
        b->alive = (uint32_t)e->alive;
        b->x = fbits(e->x);
        b->y = fbits(e->y);
        b->vx = fbits(e->vx);
        b->vy = fbits(e->vy);
        b->hp = e->hp;
    }
}

static Input input_at(uint32_t f) {
    Input in;
    in.buttons = ((f % 7u) == 3u ? BTN_HIT : 0u) |
                 ((f % 23u) == 0u ? BTN_JUMP : 0u);
    in.ax = (float)((int)((f * 13u) % 17u) - 8) / 8.0f;
    in.ay = (float)((int)((f * 5u) % 9u) - 4) / 4.0f;
    return in;
}

/* ------------------------------------------------------------- recording */

static void put_u32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; i++) p[i] = (uint8_t)((v >> (8 * i)) & 0xffu);
}

static void put_u64(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)((v >> (8 * i)) & 0xffu);
}

static uint32_t get_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t get_u64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)p[i] << (8 * i);
    return v;
}

static size_t rec_bytes(uint32_t spawn_count, uint32_t frame_count) {
    return REC_HEADER_BYTES + (size_t)REC_SPAWN_BYTES * spawn_count +
           (size_t)REC_FRAME_BYTES * frame_count;
}

static void put_snapshot(uint8_t* p, const Snapshot* s) {
    put_u32(p + 0, s->n);
    put_u32(p + 4, s->tick);
    put_u32(p + 8, s->rng);
    for (int k = 0; k < MAX_ENT; k++) {
        uint8_t* q = p + 12 + k * 28;
        put_u32(q + 0, s->e[k].id);
        put_u32(q + 4, s->e[k].alive);
        put_u32(q + 8, s->e[k].x);
        put_u32(q + 12, s->e[k].y);
        put_u32(q + 16, s->e[k].vx);
        put_u32(q + 20, s->e[k].vy);
        put_u32(q + 24, (uint32_t)s->e[k].hp);
    }
}

static void get_snapshot(const uint8_t* p, Snapshot* s) {
    s->n = get_u32(p + 0);
    s->tick = get_u32(p + 4);
    s->rng = get_u32(p + 8);
    for (int k = 0; k < MAX_ENT; k++) {
        const uint8_t* q = p + 12 + k * 28;
        s->e[k].id = get_u32(q + 0);
        s->e[k].alive = get_u32(q + 4);
        s->e[k].x = get_u32(q + 8);
        s->e[k].y = get_u32(q + 12);
        s->e[k].vx = get_u32(q + 16);
        s->e[k].vy = get_u32(q + 20);
        s->e[k].hp = (int32_t)get_u32(q + 24);
    }
}

static int rec_serialize(const Recording* r, uint8_t* out, size_t cap,
                         size_t* out_len) {
    if (r == NULL || out == NULL || out_len == NULL) return -1;
    if (r->frame_count > MAX_FRAMES || r->spawn_count > MAX_ENT) return -1;
    size_t need = rec_bytes(r->spawn_count, r->frame_count);
    if (cap < need) return -1;

    memset(out, 0, need);
    put_u32(out + 0, REC_MAGIC);
    put_u32(out + 4, REC_VERSION);
    put_u32(out + 8, r->flags);
    put_u32(out + 12, r->frame_count);
    put_u32(out + 16, r->spawn_count);
    put_u32(out + 20, r->seed);
    put_u32(out + 24, (uint32_t)need);
    put_u32(out + 28, 0);
    put_u64(out + 32, r->initial.lo);
    put_u64(out + 40, r->initial.hi);

    uint8_t* p = out + REC_HEADER_BYTES;
    for (uint32_t i = 0; i < r->spawn_count; i++) {
        put_u32(p + 0, r->spawn[i].id);
        put_u32(p + 4, fbits(r->spawn[i].x));
        put_u32(p + 8, fbits(r->spawn[i].y));
        put_u32(p + 12, (uint32_t)r->spawn[i].hp);
        p += REC_SPAWN_BYTES;
    }
    for (uint32_t f = 0; f < r->frame_count; f++) {
        const RecFrame* rf = &r->frame[f];
        put_u32(p + 0, rf->frame);
        put_u32(p + 4, rf->tick);
        put_u32(p + 8, rf->in.buttons);
        put_u32(p + 12, fbits(rf->in.ax));
        put_u32(p + 16, fbits(rf->in.ay));
        put_u32(p + 20, 0);
        put_u64(p + 24, rf->hash.lo);
        put_u64(p + 32, rf->hash.hi);
        put_snapshot(p + 40, &rf->snap);
        p += REC_FRAME_BYTES;
    }
    *out_len = need;
    return 0;
}

static int rec_parse(const uint8_t* in, size_t len, Recording* r,
                     char* err, size_t errsz) {
    if (err && errsz) err[0] = '\0';
    if (in == NULL || r == NULL) return -1;
    if (len < REC_HEADER_BYTES) {
        if (err) snprintf(err, errsz, "truncated header: %zu of %u bytes",
                          len, REC_HEADER_BYTES);
        return -1;
    }
    if (get_u32(in + 0) != REC_MAGIC) {
        if (err) snprintf(err, errsz, "bad magic 0x%08x", get_u32(in + 0));
        return -1;
    }
    if (get_u32(in + 4) != REC_VERSION) {
        if (err) snprintf(err, errsz, "bad version %u", get_u32(in + 4));
        return -1;
    }
    uint32_t flags = get_u32(in + 8);
    if ((flags & REC_FLAG_CANON) == 0) {
        if (err) snprintf(err, errsz, "recording not canonical (flags 0x%x)", flags);
        return -1;
    }
    uint32_t frames = get_u32(in + 12);
    uint32_t spawns = get_u32(in + 16);
    if (frames > MAX_FRAMES) {
        if (err) snprintf(err, errsz, "frame count %u > %u", frames, MAX_FRAMES);
        return -1;
    }
    if (spawns > MAX_ENT) {
        if (err) snprintf(err, errsz, "spawn count %u > %u", spawns, MAX_ENT);
        return -1;
    }
    size_t need = rec_bytes(spawns, frames);
    uint32_t declared = get_u32(in + 24);
    if (declared != (uint32_t)need || len != need) {
        if (err) snprintf(err, errsz,
                          "size mismatch: declared %u, header says %zu, got %zu",
                          declared, need, len);
        return -1;
    }

    memset(r, 0, sizeof *r);
    r->version = get_u32(in + 4);
    r->flags = flags;
    r->frame_count = frames;
    r->spawn_count = spawns;
    r->seed = get_u32(in + 20);
    r->bytes = declared;
    r->initial.lo = get_u64(in + 32);
    r->initial.hi = get_u64(in + 40);

    const uint8_t* p = in + REC_HEADER_BYTES;
    for (uint32_t i = 0; i < spawns; i++) {
        r->spawn[i].id = get_u32(p + 0);
        r->spawn[i].x = bitsf(get_u32(p + 4));
        r->spawn[i].y = bitsf(get_u32(p + 8));
        r->spawn[i].hp = (int32_t)get_u32(p + 12);
        p += REC_SPAWN_BYTES;
    }
    for (uint32_t f = 0; f < frames; f++) {
        RecFrame* rf = &r->frame[f];
        rf->frame = get_u32(p + 0);
        rf->tick = get_u32(p + 4);
        rf->in.buttons = get_u32(p + 8);
        rf->in.ax = bitsf(get_u32(p + 12));
        rf->in.ay = bitsf(get_u32(p + 16));
        rf->hash.lo = get_u64(p + 24);
        rf->hash.hi = get_u64(p + 32);
        get_snapshot(p + 40, &rf->snap);
        if (rf->frame != f) {
            if (err) snprintf(err, errsz, "frame index %u at slot %u",
                              rf->frame, f);
            return -1;
        }
        p += REC_FRAME_BYTES;
    }
    return 0;
}

static int world_from_recording(const Recording* r, World* w) {
    if (world_init(w, r->seed, ORDER_CANONICAL) != 0) return -1;
    for (uint32_t i = 0; i < r->spawn_count; i++) {
        if (world_spawn(w, (int32_t)r->spawn[i].id, r->spawn[i].x,
                        r->spawn[i].y, r->spawn[i].hp) != 0) return -1;
    }
    return 0;
}

/* Run the canonical simulation and capture one snapshot per frame. */
static int record_run(const Spawn* spawn, uint32_t spawn_count, uint32_t seed,
                      uint32_t frame_count, Recording* r) {
    if (r == NULL || spawn_count > MAX_ENT || frame_count > MAX_FRAMES)
        return -1;
    memset(r, 0, sizeof *r);
    r->version = REC_VERSION;
    r->flags = REC_FLAG_CANON;
    r->frame_count = frame_count;
    r->spawn_count = spawn_count;
    r->seed = seed;
    r->bytes = (uint32_t)rec_bytes(spawn_count, frame_count);
    for (uint32_t i = 0; i < spawn_count; i++) r->spawn[i] = spawn[i];

    World w;
    if (world_init(&w, seed, ORDER_CANONICAL) != 0) return -1;
    for (uint32_t i = 0; i < spawn_count; i++) {
        if (world_spawn(&w, (int32_t)spawn[i].id, spawn[i].x, spawn[i].y,
                        spawn[i].hp) != 0) return -1;
    }
    Snapshot s;
    world_snapshot(&w, &s);
    r->initial = snapshot_hash(&s);

    for (uint32_t f = 0; f < frame_count; f++) {
        RecFrame* rf = &r->frame[f];
        rf->frame = f;
        rf->in = input_at(f);
        world_step(&w, rf->in);
        rf->tick = w.tick;
        world_snapshot(&w, &rf->snap);
        rf->hash = snapshot_hash(&rf->snap);
    }
    return 0;
}

/* ------------------------------------------------------- divergence diff */

/* Compare recorded (a) with replayed (b), field by field in canonical order.
   Returns 1 and fills d on the first difference, 0 if bit-identical. */
static int snapshot_diff(const Snapshot* a, const Snapshot* b,
                         char* field, size_t fsz,
                         uint32_t* rec_bits, uint32_t* live_bits) {
    if (a->n != b->n) {
        snprintf(field, fsz, "n");
        *rec_bits = a->n; *live_bits = b->n;
        return 1;
    }
    if (a->tick != b->tick) {
        snprintf(field, fsz, "tick");
        *rec_bits = a->tick; *live_bits = b->tick;
        return 1;
    }
    if (a->rng != b->rng) {
        snprintf(field, fsz, "rng");
        *rec_bits = a->rng; *live_bits = b->rng;
        return 1;
    }
    for (int k = 0; k < MAX_ENT; k++) {
        const EntBits* x = &a->e[k];
        const EntBits* y = &b->e[k];
        uint32_t xb[7] = { x->id, x->alive, x->x, x->y, x->vx, x->vy, (uint32_t)x->hp };
        uint32_t yb[7] = { y->id, y->alive, y->x, y->y, y->vx, y->vy, (uint32_t)y->hp };
        static const char* names[7] = { "id", "alive", "x", "y", "vx", "vy", "hp" };
        for (int q = 0; q < 7; q++) {
            if (xb[q] != yb[q]) {
                snprintf(field, fsz, "ent[%u].%s", x->id, names[q]);
                *rec_bits = xb[q]; *live_bits = yb[q];
                return 1;
            }
        }
    }
    return 0;
}

/* --------------------------------------------------------------- verify */

static void verify_result_init(VerifyResult* v) {
    memset(v, 0, sizeof *v);
    v->status = REPLAY_OK;
}

static void verify(const Recording* r, VerifyResult* v) {
    verify_result_init(v);
    World w;
    if (world_from_recording(r, &w) != 0) {
        v->status = REPLAY_BAD_REC;
        return;
    }
    Snapshot s;
    world_snapshot(&w, &s);
    Hash128 ih = snapshot_hash(&s);
    if (ih.lo != r->initial.lo || ih.hi != r->initial.hi) {
        v->status = REPLAY_DIVERGED;
        v->first_frame = 0;
        snprintf(v->first_field, sizeof v->first_field, "initial");
        return;
    }

    for (uint32_t f = 0; f < r->frame_count; f++) {
        const RecFrame* rf = &r->frame[f];
        world_step(&w, rf->in);
        world_snapshot(&w, &s);
        Hash128 h = snapshot_hash(&s);
        Hash128 rec_h = snapshot_hash(&rf->snap);

        int hash_diff = (h.lo != rf->hash.lo || h.hi != rf->hash.hi);
        int bit_diff = (memcmp(&s, &rf->snap, sizeof s) != 0);
        /* The recorded bytes must hash to the recorded hash. If not, the
           recording was edited in place; detect it even when replay agrees
           with the stored hash. */
        int integrity = (rec_h.lo != rf->hash.lo || rec_h.hi != rf->hash.hi);

        if (hash_diff) v->hash_diffs++;
        if (bit_diff) v->bit_diffs++;
        if (integrity) v->integrity_diffs++;
        if (!hash_diff && bit_diff) v->collision_guard++;
        v->frames_checked++;

        if ((hash_diff || bit_diff || integrity) && v->status == REPLAY_OK) {
            v->status = REPLAY_DIVERGED;
            v->first_frame = f;
            if (bit_diff) {
                snapshot_diff(&rf->snap, &s, v->first_field,
                              sizeof v->first_field,
                              &v->rec_bits, &v->live_bits);
            } else if (integrity) {
                snprintf(v->first_field, sizeof v->first_field, "record_hash");
                v->rec_bits = (uint32_t)rf->hash.lo;
                v->live_bits = (uint32_t)rec_h.lo;
            } else {
                snprintf(v->first_field, sizeof v->first_field, "hash_only");
                v->rec_bits = (uint32_t)rf->hash.lo;
                v->live_bits = (uint32_t)h.lo;
            }
        }
    }
}

/* -------------------------------------------------------- replay window */

static void history_init(History* h) {
    memset(h, 0, sizeof *h);
}

static void history_store(History* h, uint32_t frame, const Snapshot* s) {
    uint32_t slot;
    if (h->n < WIN_CAP) {
        slot = (h->head + h->n) % WIN_CAP;
        h->n++;
    } else {
        slot = h->head;
        h->head = (h->head + 1) % WIN_CAP;
    }
    h->frame[slot] = frame;
    h->snap[slot] = *s;
}

static int history_get(History* h, uint32_t frame, Snapshot* out) {
    if (h->n == 0) { h->window_short++; return REPLAY_WINDOW_SHORT; }
    uint32_t oldest = h->frame[h->head];
    uint32_t newest = h->frame[(h->head + h->n - 1) % WIN_CAP];
    if (frame > newest) return REPLAY_WINDOW_FUTURE;
    if (frame < oldest) { h->window_short++; return REPLAY_WINDOW_SHORT; }
    for (uint32_t i = 0; i < h->n; i++) {
        uint32_t slot = (h->head + i) % WIN_CAP;
        if (h->frame[slot] == frame) {
            *out = h->snap[slot];
            return REPLAY_OK;
        }
    }
    h->window_short++;
    return REPLAY_WINDOW_SHORT;
}

static uint32_t history_oldest(const History* h) { return h->frame[h->head]; }
static uint32_t history_newest(const History* h) {
    return h->frame[(h->head + h->n - 1) % WIN_CAP];
}

/* Rewind: restore the world to `frame` and re-run the recorded inputs up to
   the newest retained frame. Returns REPLAY_OK when every recomputed snapshot
   matches the retained one bit for bit, REPLAY_WINDOW_SHORT when `frame` has
   fallen out of the window, and REPLAY_WINDOW_FUTURE when it has not been
   recorded yet. A short window is reported and the live world is untouched. */
static int history_rewind(History* h, uint32_t frame, Recording* rec,
                          World* live, uint32_t* replayed_frames) {
    *replayed_frames = 0;
    Snapshot base;
    int rc = history_get(h, frame, &base);
    if (rc != REPLAY_OK) return rc;

    uint32_t newest = history_newest(h);
    World w;
    if (world_init(&w, rec->seed, ORDER_CANONICAL) != 0)
        return REPLAY_WINDOW_SHORT;
    for (uint32_t i = 0; i < rec->spawn_count; i++) {
        if (world_spawn(&w, (int32_t)rec->spawn[i].id, rec->spawn[i].x,
                        rec->spawn[i].y, rec->spawn[i].hp) != 0)
            return REPLAY_WINDOW_SHORT;
    }
    for (uint32_t f = 0; f <= frame; f++) world_step(&w, rec->frame[f].in);

    Snapshot s;
    world_snapshot(&w, &s);
    if (memcmp(&s, &base, sizeof s) != 0) return REPLAY_DIVERGED;

    for (uint32_t f = frame + 1; f <= newest; f++) {
        world_step(&w, rec->frame[f].in);
        world_snapshot(&w, &s);
        (*replayed_frames)++;
        Snapshot want;
        if (history_get(h, f, &want) != REPLAY_OK) return REPLAY_DIVERGED;
        if (memcmp(&s, &want, sizeof s) != 0) return REPLAY_DIVERGED;
    }
    *live = w;
    return REPLAY_OK;
}

/* ------------------------------------------------------------ self-test */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static const Spawn BASE[6] = {
    { 1,   0.0f,  0.0f, 100 },
    { 2,   3.0f,  1.0f, 100 },
    { 3,  -5.0f,  2.0f, 100 },
    { 4,  11.0f, -3.0f, 100 },
    { 5, -13.0f,  7.0f, 100 },
    { 6,  17.0f,  5.0f, 100 },
};

static void permute(const Spawn* in, uint32_t n, const int* perm, Spawn* out) {
    for (uint32_t i = 0; i < n; i++) out[i] = in[perm[i]];
}

/* Next lexicographic permutation of 0..n-1; returns 0 when exhausted. */
static int next_perm(int* p, int n) {
    int i = n - 2;
    while (i >= 0 && p[i] >= p[i + 1]) i--;
    if (i < 0) return 0;
    int j = n - 1;
    while (p[j] <= p[i]) j--;
    int t = p[i]; p[i] = p[j]; p[j] = t;
    for (int a = i + 1, b = n - 1; a < b; a++, b--) {
        t = p[a]; p[a] = p[b]; p[b] = t;
    }
    return 1;
}

static void test_determinism(void) {
    /* The recording is produced by one canonical run. */
    Recording rec;
    check("record_ok", record_run(BASE, 6, 0x1234567u, 128, &rec) == 0);

    static uint8_t buf[REC_MAX_BYTES];
    size_t len = 0;
    check("serialize_ok", rec_serialize(&rec, buf, sizeof buf, &len) == 0);
    check("record_size_linear",
          len == rec_bytes(6, 128) && len == REC_HEADER_BYTES +
                REC_SPAWN_BYTES * 6 + REC_FRAME_BYTES * 128);

    Recording parsed;
    char err[160];
    check("parse_ok", rec_parse(buf, len, &parsed, err, sizeof err) == 0);

    VerifyResult v;
    verify(&parsed, &v);
    check("verify_match", v.status == REPLAY_OK);
    check("frames_verified", v.frames_checked == 128);
    check("hash_diffs_zero", v.hash_diffs == 0);
    check("bit_diffs_zero", v.bit_diffs == 0);
    check("integrity_diffs_zero", v.integrity_diffs == 0);
    check("hash_bitwise_agree", v.hash_diffs == v.bit_diffs);
    check("bitwise_diffs_zero", v.bit_diffs == 0);

    /* Serializing a second run must yield the same bytes, and a second parse
       + verify must reach the same verdict over the same hash chain. */
    Recording rec2;
    check("rerun_ok", record_run(BASE, 6, 0x1234567u, 128, &rec2) == 0);
    static uint8_t buf2[REC_MAX_BYTES];
    size_t len2 = 0;
    check("reserialize_ok", rec_serialize(&rec2, buf2, sizeof buf2, &len2) == 0);
    check("rerun_bytes_identical", len == len2 && memcmp(buf, buf2, len) == 0);

    VerifyResult v2;
    verify(&parsed, &v2);
    check("verify_twice_same", v2.status == v.status &&
                               v2.hash_diffs == v.hash_diffs &&
                               v2.bit_diffs == v.bit_diffs);
}

static void test_layout_independence(void) {
    /* Same logical world in every one of the 6! = 720 storage orders.
       Canonical mode must reach a bit-identical state after every frame;
       storage-order mode must diverge, reproducing the original report. */
    enum { FRAMES = 128 };
    static Snapshot ref[FRAMES];

    /* Reference: canonical run in identity order. */
    {
        World w;
        world_init(&w, 0x1234567u, ORDER_CANONICAL);
        for (int i = 0; i < 6; i++)
            world_spawn(&w, (int32_t)BASE[i].id, BASE[i].x, BASE[i].y,
                        BASE[i].hp);
        for (uint32_t f = 0; f < FRAMES; f++) {
            world_step(&w, input_at(f));
            world_snapshot(&w, &ref[f]);
        }
    }

    int perm[6] = { 0, 1, 2, 3, 4, 5 };
    int canon_diffs = 0;
    int naive_orders_diverging = 0;
    int naive_frame_first = -1;
    char naive_field[40] = { 0 };
    uint32_t rb = 0, lb = 0;

    do {
        Spawn order[6];
        permute(BASE, 6, perm, order);

        World cw, nw;
        world_init(&cw, 0x1234567u, ORDER_CANONICAL);
        world_init(&nw, 0x1234567u, ORDER_STORAGE);
        for (int i = 0; i < 6; i++) {
            world_spawn(&cw, (int32_t)order[i].id, order[i].x, order[i].y,
                        order[i].hp);
            world_spawn(&nw, (int32_t)order[i].id, order[i].x, order[i].y,
                        order[i].hp);
        }

        int naive_frame = -1;
        for (uint32_t f = 0; f < FRAMES; f++) {
            Input in = input_at(f);
            world_step(&cw, in);
            world_step(&nw, in);
            Snapshot cs, ns;
            world_snapshot(&cw, &cs);
            world_snapshot(&nw, &ns);
            if (memcmp(&ref[f], &cs, sizeof cs) != 0) canon_diffs++;
            if (naive_frame < 0 && memcmp(&ref[f], &ns, sizeof ns) != 0) {
                naive_frame = (int)f;
                if (naive_frame_first < 0) {
                    snapshot_diff(&ns, &ref[f], naive_field,
                                  sizeof naive_field, &rb, &lb);
                    naive_frame_first = naive_frame;
                }
            }
        }
        if (naive_frame >= 0) naive_orders_diverging++;
    } while (next_perm(perm, 6));

    printf("layout_permutations_total=720\n");
    printf("layout_canonical_diffs=%d\n", canon_diffs);
    printf("layout_naive_orders_diverging=%d\n", naive_orders_diverging);
    printf("naive_first_diff_frame=%d\n", naive_frame_first);
    printf("naive_first_diff_field=%s\n", naive_field);
    printf("naive_rec=0x%08x\n", rb);
    printf("naive_live=0x%08x\n", lb);
    check("layout_all_orders_identical", canon_diffs == 0);
    check("naive_order_diverges", naive_orders_diverging > 0);
    check("naive_field_named", naive_field[0] != '\0');
    check("naive_majority_diverge", naive_orders_diverging > 720 / 2);

    /* The canonical recording carries the canonical flag, so a storage-order
       run cannot be mistaken for a recording. */
    Recording r;
    record_run(BASE, 6, 0x1234567u, 16, &r);
    check("record_flagged_canonical", (r.flags & REC_FLAG_CANON) != 0);
}

static void test_tamper_localization(void) {
    Recording rec;
    record_run(BASE, 6, 0x1234567u, 64, &rec);
    static uint8_t buf[REC_MAX_BYTES];
    size_t len = 0;
    rec_serialize(&rec, buf, sizeof buf, &len);

    /* 1. Input tamper at frame 17: flip the HIT bit. */
    static uint8_t t1[REC_MAX_BYTES];
    memcpy(t1, buf, len);
    size_t f17 = REC_HEADER_BYTES + REC_SPAWN_BYTES * 6 +
                 (size_t)REC_FRAME_BYTES * 17;
    put_u32(t1 + f17 + 8, get_u32(t1 + f17 + 8) ^ BTN_HIT);
    Recording p1;
    char err[160];
    int ok1 = rec_parse(t1, len, &p1, err, sizeof err) == 0;
    VerifyResult v1;
    verify(&p1, &v1);
    printf("tamper_input_first_frame=%u\n", v1.first_frame);
    printf("tamper_input_first_field=%s\n", v1.first_field);
    printf("tamper_input_rec=0x%08x\n", v1.rec_bits);
    printf("tamper_input_live=0x%08x\n", v1.live_bits);
    check("tamper_input_parsed", ok1);
    check("tamper_input_localized", v1.status == REPLAY_DIVERGED &&
                                    v1.first_frame == 17);
    check("tamper_input_field_named", v1.first_field[0] != '\0');
    check("tamper_input_bit_diffs", v1.bit_diffs > 0);

    /* 2. State tamper at frame 29: flip the low bit of the first canonical
          entity's x. The recorded hash still describes the untampered state,
          so replay's hash equals the recorded hash while the bytes differ:
          this is exactly the case a hash-only verifier would pass. */
    static uint8_t t2[REC_MAX_BYTES];
    memcpy(t2, buf, len);
    size_t f29 = REC_HEADER_BYTES + REC_SPAWN_BYTES * 6 +
                 (size_t)REC_FRAME_BYTES * 29;
    size_t x0 = f29 + 40 + 12 + 8;          /* snapshot + header + slot0.x */
    put_u32(t2 + x0, get_u32(t2 + x0) ^ 1u);
    Recording p2;
    int ok2 = rec_parse(t2, len, &p2, err, sizeof err) == 0;
    VerifyResult v2;
    verify(&p2, &v2);
    printf("tamper_state_first_frame=%u\n", v2.first_frame);
    printf("tamper_state_first_field=%s\n", v2.first_field);
    printf("tamper_state_rec=0x%08x\n", v2.rec_bits);
    printf("tamper_state_live=0x%08x\n", v2.live_bits);
    printf("tamper_state_hash_diffs=%u\n", v2.hash_diffs);
    printf("tamper_state_integrity_diffs=%u\n", v2.integrity_diffs);
    check("tamper_state_parsed", ok2);
    check("tamper_state_localized", v2.status == REPLAY_DIVERGED &&
                                    v2.first_frame == 29);
    check("tamper_state_field_named", strstr(v2.first_field, ".x") != NULL);
    check("tamper_state_hash_filter_blind", v2.hash_diffs == 0);
    check("tamper_state_bitwise_catches", v2.bit_diffs == 1 &&
                                          v2.collision_guard == 1);

    /* 3. Two tampers: only the earliest is reported. */
    static uint8_t t3[REC_MAX_BYTES];
    memcpy(t3, buf, len);
    size_t f11 = REC_HEADER_BYTES + REC_SPAWN_BYTES * 6 +
                 (size_t)REC_FRAME_BYTES * 11;
    put_u32(t3 + f11 + 8, get_u32(t3 + f11 + 8) ^ BTN_JUMP);
    put_u32(t3 + x0, get_u32(t3 + x0) ^ 1u);
    Recording p3;
    rec_parse(t3, len, &p3, err, sizeof err);
    VerifyResult v3;
    verify(&p3, &v3);
    printf("tamper_multi_first_frame=%u\n", v3.first_frame);
    check("tamper_multi_earliest", v3.first_frame == 11);

    /* 4. Hash-record tamper: rewrite the stored hash only, leaving bytes
          intact. Replay bytes match but the hash no longer does, so the frame
          is still flagged rather than trusted. */
    Recording p4;
    rec_parse(buf, len, &p4, err, sizeof err);
    p4.frame[33].hash.lo ^= 1u;
    VerifyResult v4;
    verify(&p4, &v4);
    printf("hash_record_tamper_frame=%u\n", v4.first_frame);
    printf("hash_record_tamper_field=%s\n", v4.first_field);
    printf("hash_record_tamper_bit_diffs=%u\n", v4.bit_diffs);
    check("hash_record_localized", v4.status == REPLAY_DIVERGED &&
                                   v4.first_frame == 33);
    check("hash_record_bitwise_clean", v4.bit_diffs == 0 &&
                                       v4.hash_diffs == 1);

    /* 5. Forced-collision guard. A real FNV-128 collision cannot be built,
          so we simulate its observable effect: corrupt the recorded bytes at
          frame 29 and set the stored hash to the hash replay will compute for
          the true state. Hash comparison is then blind (hash_diffs == 0).
          The bitwise pass alone must still localize the frame and field. */
    Recording p5;
    rec_parse(buf, len, &p5, err, sizeof err);
    World w5;
    world_from_recording(&rec, &w5);
    Snapshot true29;
    for (uint32_t f = 0; f <= 29; f++) {
        world_step(&w5, rec.frame[f].in);
        world_snapshot(&w5, &true29);
    }
    p5.frame[29].snap.e[0].x ^= 1u;           /* bytes no longer true */
    p5.frame[29].hash = snapshot_hash(&true29); /* hash agrees with replay */
    VerifyResult v5;
    verify(&p5, &v5);
    printf("collision_guard_frame=%u\n", v5.first_frame);
    printf("collision_guard_field=%s\n", v5.first_field);
    printf("collision_guard_hash_diffs=%u\n", v5.hash_diffs);
    printf("collision_guard_bit_diffs=%u\n", v5.bit_diffs);
    printf("collision_guard_hash_only_verdict=%s\n",
           v5.hash_diffs == 0 ? "pass (would hide the bug)" : "fail");
    check("collision_guard_hash_blind", v5.hash_diffs == 0);
    check("collision_guard_caught", v5.status == REPLAY_DIVERGED &&
                                    v5.first_frame == 29 &&
                                    v5.collision_guard == 1);
    check("collision_guard_not_masked", v5.bit_diffs == 1);

    /* 6. Corrupt containers must be rejected, not silently replayed. */
    Recording bad;
    check("truncated_rejected",
          rec_parse(buf, len - 1, &bad, err, sizeof err) == -1);
    printf("truncated_error=%s\n", err);
    check("truncated_error_names_size", strstr(err, "size") != NULL);

    static uint8_t t5[64];
    memcpy(t5, buf, sizeof t5);
    put_u32(t5, 0xdeadbeefu);
    check("bad_magic_rejected",
          rec_parse(t5, sizeof t5, &bad, err, sizeof err) == -1);
    check("bad_magic_error", strstr(err, "magic") != NULL);

    static uint8_t t6[REC_MAX_BYTES];
    memcpy(t6, buf, len);
    put_u32(t6 + 4, 99u);
    check("bad_version_rejected",
          rec_parse(t6, len, &bad, err, sizeof err) == -1);
    check("bad_version_error", strstr(err, "version") != NULL);

    static uint8_t t7[REC_MAX_BYTES];
    memcpy(t7, buf, len);
    put_u32(t7 + 8, 0u);                     /* clear canonical flag */
    check("noncanonical_rejected",
          rec_parse(t7, len, &bad, err, sizeof err) == -1);

    static uint8_t t8[REC_MAX_BYTES];
    memcpy(t8, buf, len);
    put_u32(t8 + 12, MAX_FRAMES + 1u);
    check("too_many_frames_rejected",
          rec_parse(t8, len, &bad, err, sizeof err) == -1);
}

static void test_window(void) {
    Recording rec;
    record_run(BASE, 6, 0x1234567u, 20, &rec);
    History h;
    history_init(&h);
    for (uint32_t f = 0; f < 20; f++) history_store(&h, f, &rec.frame[f].snap);

    printf("window_cap=%d\n", WIN_CAP);
    printf("window_kept=%u\n", h.n);
    printf("window_oldest=%u\n", history_oldest(&h));
    printf("window_newest=%u\n", history_newest(&h));

    Snapshot got;
    int newest_ok = history_get(&h, history_newest(&h), &got) == REPLAY_OK;
    int oldest_ok = history_get(&h, history_oldest(&h), &got) == REPLAY_OK;
    uint32_t before_short = h.window_short;
    int short_rc = history_get(&h, 11, &got);
    printf("window_requested=11\n");
    printf("window_oldest_available=%u\n", history_oldest(&h));
    printf("window_short_reported=%u\n", h.window_short);
    printf("window_short_rc=%d\n", short_rc);

    check("window_newest_available", newest_ok);
    check("window_oldest_available_ok", oldest_ok);
    check("window_short_returned", short_rc == REPLAY_WINDOW_SHORT);
    check("window_short_counted", h.window_short == before_short + 1);

    /* A too-new request is a different error and is not counted as short. */
    uint32_t before_future = h.window_short;
    check("window_future_distinct",
          history_get(&h, 99, &got) == REPLAY_WINDOW_FUTURE &&
          h.window_short == before_future);

    /* Asking past the window must not evict or alter retained frames. */
    Snapshot snap_after;
    int intact = history_get(&h, history_newest(&h), &snap_after) == REPLAY_OK &&
                 memcmp(&snap_after, &rec.frame[19].snap, sizeof snap_after) == 0;
    check("window_state_untouched", intact);

    /* Window is bounded and strictly the last WIN_CAP frames. */
    check("window_bounded", h.n == WIN_CAP &&
                            history_oldest(&h) == 12 &&
                            history_newest(&h) == 19);

    /* Rewind to the oldest retained frame re-runs every retained input and
       must reproduce each snapshot bit for bit. */
    World live;
    world_init(&live, 0xdeadbeefu, ORDER_CANONICAL);    /* stale live state */
    uint32_t replayed = 0;
    int rewind_ok = history_rewind(&h, history_oldest(&h), &rec, &live,
                                   &replayed);
    printf("rewind_from_frame=%u\n", history_oldest(&h));
    printf("rewind_replayed_frames=%u\n", replayed);
    printf("rewind_rc=%d\n", rewind_ok);
    check("window_rewind_ok", rewind_ok == REPLAY_OK);
    check("window_rewind_covered", replayed == WIN_CAP - 1);
    Snapshot live_snap;
    world_snapshot(&live, &live_snap);
    check("window_rewind_bitwise",
          memcmp(&live_snap, &rec.frame[19].snap, sizeof live_snap) == 0);

    /* Rewind to a frame outside the window reports short, changes nothing. */
    World untouched = live;
    uint32_t r2 = 0;
    int short_rewind = history_rewind(&h, 11, &rec, &untouched, &r2);
    printf("window_rewind_short_rc=%d\n", short_rewind);
    check("window_rewind_short", short_rewind == REPLAY_WINDOW_SHORT &&
                                 r2 == 0 && h.window_short > before_short + 1);
    check("window_rewind_short_leaves_live",
          memcmp(&untouched, &live, sizeof live) == 0);
}

static void test_hash_budget(void) {
    /* 128-bit state hash. For a run of N frames, the chance that any recorded
       frame pair collides (uniform-hash model) is bounded by N / 2^128; the
       birthday bound over all N states is N(N-1)/2 / 2^128. */
    const uint32_t n = MAX_FRAMES;
    double two128 = ldexp(1.0, 128);
    double pair = 1.0 / two128;
    double run_bound = (double)n / two128;
    double birthday = (double)n * (double)(n - 1) / 2.0 / two128;
    printf("hash_bits=128\n");
    printf("hash_frames=%u\n", n);
    printf("collision_p_single_pair=%.3e\n", pair);
    printf("collision_p_run_bound=%.3e\n", run_bound);
    printf("collision_p_birthday=%.3e\n", birthday);
    printf("field_names_localized_by=bitwise_diff\n");
    check("collision_bound_negligible", run_bound < 1e-30 && birthday < 1e-30);
    check("hash_is_128_bit", sizeof(Hash128) == 16);
}

static void test_cross_build_digest(void) {
    /* One stable line that any build (compiler, -O level) must reproduce
       byte for byte. Capture it in CI and diff across toolchains. */
    Recording rec;
    record_run(BASE, 6, 0x1234567u, 192, &rec);
    printf("cross_build_final_hash_lo=%016llx\n",
           (unsigned long long)rec.frame[191].hash.lo);
    printf("cross_build_final_hash_hi=%016llx\n",
           (unsigned long long)rec.frame[191].hash.hi);
    check("cross_build_digest_ready", rec.frame_count == 192);
}

int main(void) {
    test_determinism();
    test_layout_independence();
    test_tamper_localization();
    test_window();
    test_hash_budget();
    test_cross_build_digest();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
