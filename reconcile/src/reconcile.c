
/* reconcile: prediction buffer with rollback on authoritative state.
 *
 * Failure-isolation version: named recon_err_t codes plus deterministic
 * fault hooks. Ring indexing, ack matching and rollback math are unchanged.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "reconcile.h"

void recon_init(Recon* r) {
    if (r == NULL) return;
    memset(r, 0, sizeof(*r));
}

/* --- deterministic, process-local fault injection ----------------------- */
static recon_fault_t g_recon_fault = RECON_FAULT_NONE;

void recon_inject(recon_fault_t f) { g_recon_fault = f; }
void recon_reset_faults(void) { g_recon_fault = RECON_FAULT_NONE; }
recon_fault_t recon_current_fault(void) { return g_recon_fault; }

const char* recon_err_name(int code) {
    switch (code) {
    case RECON_OK:              return "RECON_OK";
    case RECON_ERR_NULL:        return "RECON_ERR_NULL";
    case RECON_ERR_STALE:       return "RECON_ERR_STALE";
    case RECON_ERR_DUP:         return "RECON_ERR_DUP";
    case RECON_ERR_INJECT_PUSH: return "RECON_ERR_INJECT_PUSH";
    default:                    return "RECON_ERR_UNKNOWN";
    }
}

const char* recon_fault_name(recon_fault_t f) {
    switch (f) {
    case RECON_FAULT_NONE: return "RECON_FAULT_NONE";
    case RECON_FAULT_PUSH: return "RECON_FAULT_PUSH";
    default:               return "RECON_FAULT_UNKNOWN";
    }
}

int recon_push(Recon* r, unsigned tick, float x) {
    if (r == NULL) return RECON_ERR_NULL;
    if (tick <= r->acked) return RECON_ERR_STALE;          /* stale tick rejected */
    for (int i = 0; i < r->count; i++) {                   /* duplicate tick anywhere */
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->p[idx].tick == tick) return RECON_ERR_DUP;
    }
    /* hook sits after validation, before any write: a fired fault leaves
       head/count and every ring slot exactly as they were */
    if (g_recon_fault == RECON_FAULT_PUSH) return RECON_ERR_INJECT_PUSH;
    r->p[r->head].tick = tick;
    r->p[r->head].x = x;
    r->head = (r->head + 1) % BUF;
    if (r->count < BUF) r->count++;
    return 0;
}

/* authoritative state arrives for tick A: drop acked, count rollback if mismatch */
int recon_ack(Recon* r, unsigned tick, float x) {
    if (r == NULL) return RECON_ERR_NULL;
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

#ifdef MINILAB_SELFTEST
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int MINILAB_SELFTEST(void) {
    Recon r;
    recon_init(&r);
    check("push1", recon_push(&r, 1, 1.0f) == 0);
    check("push2", recon_push(&r, 2, 2.0f) == 0);
    check("dup_rejected", recon_push(&r, 2, 5.0f) == RECON_ERR_DUP);

    check("ack_match", recon_ack(&r, 2, 2.0f) == 1);
    check("no_rollback", r.rollbacks == 0);

    check("stale_rejected", recon_push(&r, 2, 3.0f) == RECON_ERR_STALE);

    recon_push(&r, 3, 3.0f);
    check("ack_mismatch", recon_ack(&r, 3, 3.5f) == 1);
    check("rollback_counted", r.rollbacks == 1);
    check("err_recorded", fabsf(r.last_err - 0.5f) < 1e-5f);

    check("ack_missing", recon_ack(&r, 99, 0.0f) == 0);

    Recon big;
    recon_init(&big);
    for (int i = 1; i <= 70; i++) recon_push(&big, (unsigned)i, (float)i);
    check("buffer_capped", big.count == BUF);

    /* failure injection on a fresh buffer: the hook must fire before any
       ring write, so count/head stay put; done before any high-tick ack. */
    Recon inj;
    recon_init(&inj);
    recon_push(&inj, 1, 1.0f);
    recon_push(&inj, 2, 2.0f);
    int count_before = inj.count, head_before = inj.head;
    recon_inject(RECON_FAULT_PUSH);
    check("inject_push", recon_push(&inj, 3, 3.0f) == RECON_ERR_INJECT_PUSH);
    recon_reset_faults();
    check("inject_isolated", inj.count == count_before && inj.head == head_before);
    check("fault_cleared", recon_push(&inj, 3, 3.0f) == 0);

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
