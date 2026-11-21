
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

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
