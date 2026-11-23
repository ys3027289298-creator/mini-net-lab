
/* reconcile: prediction buffer with rollback on authoritative state.
 *
 * Stale/duplicate pushes and malformed acks are reported through the shared
 * mnl sink (module=RECONCILE) with the offending field named. A rejected
 * push never touches the ring buffer. Legacy self-test output is
 * byte-identical.
 */
#include "mnl.h"

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

_Static_assert(sizeof(Recon) <= sizeof(mnl_state_t), "Recon exceeds mnl_state_t");

static MNL_MAYBE_UNUSED void recon_init(Recon* r) { memset(r, 0, sizeof(*r)); }

static int recon_push(Recon* r, unsigned tick, float x, mnl_sink_t* sk) {
    if (r == NULL) return mnl_report(sk, MNL_MOD_RECONCILE,
                                     MNL_F_RECON, MNL_R_NULL);
    if (!isfinite(x)) return mnl_report(sk, MNL_MOD_RECONCILE,
                                        MNL_F_X, MNL_R_NONFINITE);
    if (tick <= r->acked)                       /* stale tick rejected */
        return mnl_report(sk, MNL_MOD_RECONCILE, MNL_F_TICK, MNL_R_RANGE);
    for (int i = 0; i < r->count; i++) {        /* duplicate tick anywhere */
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->p[idx].tick == tick)
            return mnl_report(sk, MNL_MOD_RECONCILE, MNL_F_TICK, MNL_R_DUPLICATE);
    }
    r->p[r->head].tick = tick;
    r->p[r->head].x = x;
    r->head = (r->head + 1) % BUF;
    if (r->count < BUF) r->count++;
    return 0;
}

/* authoritative state arrives for tick A: drop acked, count rollback if mismatch */
static int recon_ack(Recon* r, unsigned tick, float x, mnl_sink_t* sk) {
    if (r == NULL) return mnl_report(sk, MNL_MOD_RECONCILE,
                                     MNL_F_RECON, MNL_R_NULL);
    if (!isfinite(x)) return mnl_report(sk, MNL_MOD_RECONCILE,
                                        MNL_F_X, MNL_R_NONFINITE);
    float pred = 0.0f; int found = 0;
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->p[idx].tick == tick) { pred = r->p[idx].x; found = 1; break; }
    }
    r->last_err = found ? fabsf(pred - x) : -1.0f;
    if (found && fabsf(pred - x) > 1e-3f) r->rollbacks++;
    if (tick > r->acked) r->acked = tick;
    if (!found) mnl_report(sk, MNL_MOD_RECONCILE, MNL_F_TICK, MNL_R_NOT_FOUND);
    return found ? 1 : 0;
}

/* ---- zero-copy batch runner ------------------------------------------- */

int mnl_recon_run(mnl_state_t* st, mnl_sink_t* sk,
                  const mnl_item_t* items, int n, mnl_result_t* res) {
    if (sk == NULL) return -1;
    if (st == NULL) return mnl_report(sk, MNL_MOD_RECONCILE,
                                      MNL_F_RECON, MNL_R_NULL);
    if (items == NULL || n < 0 || res == NULL)
        return mnl_report(sk, MNL_MOD_RECONCILE, MNL_F_NONE, MNL_R_NULL);

    Recon* r = (Recon*)st;
    int failures = 0;
    for (int i = 0; i < n; i++) {
        mnl_sink_begin(sk, i);
        mnl_result_t* rr = &res[i];
        memset(rr, 0, sizeof(*rr));
        const mnl_item_t* it = &items[i];

        int rc;
        switch (it->op) {
        case MNL_RECON_PUSH: {
            const mnl_recon_point_t* pt = (const mnl_recon_point_t*)it->payload;
            if (pt == NULL) {
                rc = mnl_report(sk, MNL_MOD_RECONCILE,
                                MNL_F_RECON_POINT, MNL_R_NULL);
            } else {
                /* point carrier is read in place; it is never copied */
                rc = recon_push(r, pt->tick, pt->x, sk);
            }
            break;
        }
        case MNL_RECON_ACK: {
            const mnl_recon_point_t* pt = (const mnl_recon_point_t*)it->payload;
            if (pt == NULL) {
                rc = mnl_report(sk, MNL_MOD_RECONCILE,
                                MNL_F_RECON_POINT, MNL_R_NULL);
            } else {
                rc = recon_ack(r, pt->tick, pt->x, sk);
            }
            break;
        }
        case MNL_RECON_STATS:
            rr->ival = r->rollbacks;
            rr->ival2 = r->count;
            rr->fval = r->last_err;
            rc = 0;
            break;
        default:
            rc = mnl_report(sk, MNL_MOD_RECONCILE, MNL_F_NONE, MNL_R_BAD_OP);
            break;
        }

        rr->rc = rc;
        if (rc < 0) {
            failures++;
            rr->err = sk->errs[sk->count - 1];
        } else if (sk->count > 0 &&
                   sk->errs[sk->count - 1].entry_index == i) {
            /* soft failure: rc >= 0 (e.g. ack miss) but a record exists */
            rr->err = sk->errs[sk->count - 1];
        } else if (it->op != MNL_RECON_STATS) {
            rr->ival = rc;
        }
    }
    return failures;
}

#ifndef MNL_NO_MAIN
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Recon r;
    recon_init(&r);
    check("push1", recon_push(&r, 1, 1.0f, NULL) == 0);
    check("push2", recon_push(&r, 2, 2.0f, NULL) == 0);
    check("dup_rejected", recon_push(&r, 2, 5.0f, NULL) == -1);

    check("ack_match", recon_ack(&r, 2, 2.0f, NULL) == 1);
    check("no_rollback", r.rollbacks == 0);

    check("stale_rejected", recon_push(&r, 2, 3.0f, NULL) == -1);

    recon_push(&r, 3, 3.0f, NULL);
    check("ack_mismatch", recon_ack(&r, 3, 3.5f, NULL) == 1);
    check("rollback_counted", r.rollbacks == 1);
    check("err_recorded", fabsf(r.last_err - 0.5f) < 1e-5f);

    check("ack_missing", recon_ack(&r, 99, 0.0f, NULL) == 0);

    Recon big;
    recon_init(&big);
    for (int i = 1; i <= 70; i++) recon_push(&big, (unsigned)i, (float)i, NULL);
    check("buffer_capped", big.count == BUF);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
#endif
