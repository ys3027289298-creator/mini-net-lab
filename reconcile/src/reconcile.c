
/* reconcile: prediction buffer with rollback on authoritative state.
 *
 * Prediction and ack ticks both live in the shared time base (ms).  Push and
 * ack are rejected when their tick goes backwards, so a rollback is never
 * counted from a stale or reordered observation.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../../timebase/src/timebase.h"
#include "../../timebase/src/accept.h"

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

/* Acceptance: predict each base tick, then acknowledge it one interval later
 * with a small deliberately-wrong authoritative value, so every ack rolls back.
 * Ticks come from the shared base, and the buffer must stay bounded over a
 * 2 h run while rollback accounting stays exact. */
typedef struct {
    Recon r;
    uint32_t pending_tick;
    int has_pending;
    uint32_t last_base;
    int has_base;
    int bad;
    long pushes, acks, rollbacks;
} ReconAcc;

static void recon_accept_reset(void* ctx) {
    ReconAcc* a = (ReconAcc*)ctx;
    memset(a, 0, sizeof(*a));
    recon_init(&a->r);
}

static int recon_accept_step(void* ctx, TimeBase* tb, const TBAcceptSpec* spec, uint32_t tick) {
    ReconAcc* a = (ReconAcc*)ctx;
    (void)spec; (void)tick;
    uint32_t base = 0;
    if (tb_now(tb, &base) != TB_OK) { a->bad++; return 1; }

    /* Tick 0 is the epoch: Recon treats 0 as the initial ack watermark, so
     * the first observation is not predicted.  Every later tick is. */
    if (!a->has_base) {
        a->has_base = 1;
        a->last_base = base;
        return 0;
    }

    /* Ack the previous prediction first, then record this tick's prediction. */
    if (a->has_pending) {
        float auth = (float)(a->pending_tick % 100u) + 0.25f;  /* differs */
        if (recon_ack(&a->r, a->pending_tick, auth) != 1) { a->bad++; return 2; }
        if (a->r.last_err <= 1e-3f) { a->bad++; return 3; }    /* must mismatch */
        a->acks++;
    }
    if (base <= a->last_base) { a->bad++; return 4; }          /* base must advance */
    if (recon_push(&a->r, base, (float)(base % 100u)) != 0) { a->bad++; return 5; }
    a->last_base = base;
    a->pending_tick = base;
    a->has_pending = 1;
    a->pushes++;
    return 0;
}

static int reconcile_accept(void) {
    ReconAcc acc;
    memset(&acc, 0, sizeof(acc));

    TBAcceptSpec spec;
    memset(&spec, 0, sizeof(spec));
    spec.module = TB_RECONCILE;
    spec.step_ms = 33;             /* ~30 Hz prediction */
    spec.events = 3000;            /* ~100 s */
    spec.long_events = 218182;     /* ~2 h */
    spec.ppm = 100;
    spec.seed = 0x7ec0u;
    spec.step = recon_accept_step;
    spec.reset = recon_accept_reset;
    spec.ctx = &acc;

    int rc = tb_accept_run(&spec);
    acc.rollbacks = acc.r.rollbacks;
    int buf_ok = (acc.r.count <= BUF);
    printf("module=reconcile phase=rollback pushes=%ld acks=%ld rollbacks=%d "
           "buf=%d capped=%d bad=%d ok=%d\n",
           acc.pushes, acc.acks, acc.r.rollbacks, acc.r.count, buf_ok,
           acc.bad, rc == 0 && acc.bad == 0 && buf_ok);
    tb_accept_shared(TB_RECONCILE);
    return (rc == 0 && acc.bad == 0 && buf_ok) ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "--accept") == 0) return reconcile_accept();

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

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
