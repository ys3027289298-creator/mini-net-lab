
/* reconcile: prediction buffer with rollback on authoritative state. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define BUF 64

typedef struct { unsigned tick; float x; } Pred;

typedef struct {
    Pred p[BUF];
    int head;      /* next write */
    int count;
    unsigned acked;
    int rollbacks;
    float last_err;
} Recon;

static void recon_init(Recon* r) { memset(r, 0, sizeof(*r)); }

static int recon_push(Recon* r, unsigned tick, float x) {
    if (tick <= r->acked) return -1;                 /* stale tick rejected */
    for (int i = 0; i < r->count; i++) {             /* duplicate tick anywhere */
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->p[idx].tick == tick) return -1;
    }
    r->p[r->head].tick = tick;
    r->p[r->head].x = x;
    r->head = (r->head + 1) % BUF;
    if (r->count < BUF) r->count++;
    return 0;
}

/* authoritative state arrives for tick A: drop acked, count rollback if mismatch */
static int recon_ack(Recon* r, unsigned tick, float x) {
    float pred = 0.0f; int found = 0;
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->p[idx].tick == tick) { pred = r->p[idx].x; found = 1; break; }
    }
    r->last_err = found ? fabsf(pred - x) : -1.0f;
    if (found && fabsf(pred - x) > 1e-3f) r->rollbacks++;
    if (tick > r->acked) r->acked = tick;
    return found ? 1 : 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ---- replay consistency: rollback replay must match one-shot advance ----
 *
 * Rules enforced here:
 *  - iteration order is fixed: inputs are applied in strictly ascending tick
 *    order; out-of-order arrival is an error, never a reorder;
 *  - accumulation order is fixed: sim_step always integrates flags, then
 *    velocity, then position, so replay sums floats in the same sequence
 *    as the one-shot pass (float addition is not associative);
 *  - undefined fields are flagged, never skipped: every field carries a
 *    defined-bit and sim_step refuses to read a field whose bit is clear;
 *  - results are compared field by field in a fixed order and divergence
 *    is reported at the first differing field; replay vs one-shot must
 *    diverge on zero fields.
 */

enum { F_TICK, F_X, F_Y, F_VX, F_VY, F_FLAGS, NFIELDS };
static const char* const field_names[NFIELDS] = {
    "tick", "x", "y", "vx", "vy", "flags"
};

#define DEF_TICK  (1u << F_TICK)
#define DEF_X     (1u << F_X)
#define DEF_Y     (1u << F_Y)
#define DEF_VX    (1u << F_VX)
#define DEF_VY    (1u << F_VY)
#define DEF_FLAGS (1u << F_FLAGS)
#define DEF_ALL   (DEF_TICK | DEF_X | DEF_Y | DEF_VX | DEF_VY | DEF_FLAGS)

typedef struct {
    unsigned tick;
    float x, y, vx, vy;
    unsigned flags;
    unsigned def;            /* DEF_* bits: which fields are defined */
} SState;

typedef struct {
    unsigned tick;
    float ax, ay;            /* impulse applied this tick */
    unsigned flags;          /* authoritative flag set for this tick */
    unsigned def;            /* DEF_* bits for tick/ax/ay/flags (reuses F_* order) */
} SInput;

#define REPLAY_MAX 128

typedef struct {
    SInput in[REPLAY_MAX];
    int n;
    unsigned last_tick;
    int has_tick;
} InputLog;

/* append-only, strictly ascending ticks; disorder is an error */
static int log_push(InputLog* l, const SInput* in) {
    if (l->n >= REPLAY_MAX) return -1;
    if (l->has_tick && in->tick <= l->last_tick) return -1;  /* out of order */
    l->in[l->n++] = *in;
    l->last_tick = in->tick;
    l->has_tick = 1;
    return 0;
}

static int first_missing(unsigned def, unsigned need) {
    for (int f = 0; f < NFIELDS; f++)
        if ((need & (1u << f)) && !(def & (1u << f))) return f;
    return -1;
}

/* one deterministic step; fixed field order: flags, velocity, position, tick.
 * returns 0 on success, -1 with *bad_field set if an undefined field is read */
static int sim_step(SState* s, const SInput* in, int* bad_field) {
    static const unsigned need_state = DEF_X | DEF_Y | DEF_VX | DEF_VY | DEF_FLAGS;
    static const unsigned need_input = DEF_TICK | DEF_X | DEF_Y | DEF_FLAGS; /* ax,ay,flags */
    int miss = first_missing(s->def, need_state);
    if (miss >= 0) { *bad_field = miss; return -1; }
    miss = first_missing(in->def, need_input);
    if (miss >= 0) { *bad_field = miss; return -1; }
    s->flags = in->flags;
    s->vx += in->ax; s->vy += in->ay;
    s->x += s->vx; s->y += s->vy;
    s->tick = in->tick;
    s->def = DEF_ALL;
    return 0;
}

/* first differing field, checked in causal integration order
 * (tick, flags, velocity, position) so the reported field is the
 * root-cause field, not a downstream one; bitwise for floats.
 * returns -1 if identical */
static int state_diff(const SState* a, const SState* b) {
    if (a->tick != b->tick) return F_TICK;
    if (a->flags != b->flags) return F_FLAGS;
    if (memcmp(&a->vx, &b->vx, sizeof(float))) return F_VX;
    if (memcmp(&a->vy, &b->vy, sizeof(float))) return F_VY;
    if (memcmp(&a->x, &b->x, sizeof(float))) return F_X;
    if (memcmp(&a->y, &b->y, sizeof(float))) return F_Y;
    return -1;
}

/* run inputs [from, log->n) over *s in stored (ascending-tick) order */
static int sim_run(SState* s, const InputLog* l, int from, int* bad_field) {
    for (int i = from; i < l->n; i++)
        if (sim_step(s, &l->in[i], bad_field) != 0) return -1;
    return 0;
}

static SState full_state(unsigned tick, float x, float y, float vx, float vy, unsigned flags) {
    SState s = { tick, x, y, vx, vy, flags, DEF_ALL };
    return s;
}

static SInput full_input(unsigned tick, float ax, float ay, unsigned flags) {
    SInput in = { tick, ax, ay, flags, DEF_TICK | DEF_X | DEF_Y | DEF_FLAGS };
    return in;
}

static void replay_tests(void) {
    /* deterministic input stream, fixed seed */
    unsigned rng = 12345u;
    InputLog log; memset(&log, 0, sizeof(log));
    int ordered = 1;
    for (int i = 1; i <= 40; i++) {
        rng = rng * 1664525u + 1013904223u;
        float ax = (float)(int)(rng & 0xff) / 128.0f - 1.0f;
        rng = rng * 1664525u + 1013904223u;
        float ay = (float)(int)(rng & 0xff) / 128.0f - 1.0f;
        SInput in = full_input((unsigned)i, ax, ay, (unsigned)i & 3u);
        if (log_push(&log, &in) != 0) ordered = 0;
    }
    check("push_ordered", ordered && log.n == 40);

    /* one-shot advance, snapshotting every tick */
    SState snaps[REPLAY_MAX];
    SState cur = full_state(0, 0.0f, 0.0f, 0.0f, 0.0f, 0u);
    int bad = -1;
    check("oneshot_ok", sim_run(&cur, &log, 0, &bad) == 0);
    {
        SState s = full_state(0, 0.0f, 0.0f, 0.0f, 0.0f, 0u);
        for (int i = 0; i < log.n; i++) { sim_step(&s, &log.in[i], &bad); snaps[i] = s; }
    }
    SState oneshot = cur;

    /* rollback + replay from every snapshot: divergence must be zero */
    int divergent = 0, first_field = -1;
    for (int k = 0; k < log.n; k++) {
        SState s = k == 0 ? full_state(0, 0.0f, 0.0f, 0.0f, 0.0f, 0u) : snaps[k - 1];
        if (sim_run(&s, &log, k, &bad) != 0) { divergent++; continue; }
        int d = state_diff(&s, &oneshot);
        if (d >= 0) { divergent++; if (first_field < 0) first_field = d; }
    }
    check("replay_zero_divergence", divergent == 0 && first_field < 0);
    if (first_field >= 0)
        printf("first_divergent_field=%s\n", field_names[first_field]);

    /* out-of-order input is an error, not a silent reorder */
    {
        InputLog l2; memset(&l2, 0, sizeof(l2));
        SInput i4 = full_input(4, 0, 0, 0), i5 = full_input(5, 0, 0, 0),
               i6 = full_input(6, 0, 0, 0);
        check("oo_first_ok", log_push(&l2, &i5) == 0);
        check("oo_rejected", log_push(&l2, &i4) == -1);
        check("oo_same_tick_rejected", log_push(&l2, &i5) == -1);
        check("oo_still_appends", log_push(&l2, &i6) == 0);
    }

    /* undefined field on the replay path is flagged, not skipped */
    {
        SState s = full_state(0, 0, 0, 0, 0, 0);
        s.def &= ~DEF_VX;                       /* vx never initialized */
        SInput in = full_input(1, 0.5f, 0.0f, 0u);
        int bf = -1;
        check("undef_flagged", sim_step(&s, &in, &bf) == -1);
        check("undef_named", bf == F_VX);
        SInput in2 = full_input(1, 0.5f, 0.0f, 0u);
        in2.def &= ~DEF_FLAGS;                  /* input missing flags */
        SState s2 = full_state(0, 0, 0, 0, 0, 0);
        check("undef_input_flagged", sim_step(&s2, &in2, &bf) == -1 && bf == F_FLAGS);
    }

    /* divergence is localized to the first differing field */
    {
        InputLog l3 = log;
        l3.in[10].ax += 0.25f;                  /* tamper one replayed input */
        SState s = snaps[9];
        check("tamper_runs", sim_run(&s, &l3, 10, &bad) == 0);
        int d = state_diff(&s, &oneshot);
        check("divergence_localized", d == F_VX);  /* ax hits vx first */
        if (d >= 0) printf("first_divergent_field=%s\n", field_names[d]);
    }
}

int main(void) {
    Recon r;
    recon_init(&r);
    check("push1", recon_push(&r, 1, 1.0f) == 0);
    check("push2", recon_push(&r, 2, 2.0f) == 0);
    check("dup_rejected", recon_push(&r, 2, 5.0f) == -1);

    check("ack_match", recon_ack(&r, 2, 2.0f) == 1);
    check("no_rollback", r.rollbacks == 0);

    check("stale_rejected", recon_push(&r, 2, 3.0f) == -1);

    recon_push(&r, 3, 3.0f);
    check("ack_mismatch", recon_ack(&r, 3, 3.5f) == 1);
    check("rollback_counted", r.rollbacks == 1);
    check("err_recorded", fabsf(r.last_err - 0.5f) < 1e-5f);

    check("ack_missing", recon_ack(&r, 99, 0.0f) == 0);

    Recon big;
    recon_init(&big);
    for (int i = 1; i <= 70; i++) recon_push(&big, (unsigned)i, (float)i);
    check("buffer_capped", big.count == BUF);

    replay_tests();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
