/*
 * reconcile.h - prediction buffer with rollback on authoritative state.
 *
 * Failure isolation contract:
 *   - stale/duplicate ticks are rejected with named recon_err_t;
 *   - recon_inject() arms exactly one fault and leaves the buffer untouched.
 */
#ifndef RECONCILE_H
#define RECONCILE_H

#define BUF 64

typedef enum {
    RECON_OK           = 0,
    RECON_ERR_NULL     = -40,
    RECON_ERR_STALE    = -41,   /* tick <= highest acknowledged */
    RECON_ERR_DUP      = -42,   /* tick already present in buffer */
    RECON_ERR_INJECT_PUSH = -43 /* injected failure inside recon_push */
} recon_err_t;

typedef enum {
    RECON_FAULT_NONE = 0,
    RECON_FAULT_PUSH = 1
} recon_fault_t;

typedef struct { unsigned tick; float x; } Pred;

typedef struct {
    Pred p[BUF];
    int head;      /* next write */
    int count;
    unsigned acked;
    int rollbacks;
    float last_err;
} Recon;

void recon_init(Recon* r);
/* Returns 0 on insert or a negative recon_err_t. */
int  recon_push(Recon* r, unsigned tick, float x);
/* Returns 1 if a prediction matched the tick, 0 if the tick was unknown. */
int  recon_ack(Recon* r, unsigned tick, float x);

void recon_inject(recon_fault_t f);
void recon_reset_faults(void);
recon_fault_t recon_current_fault(void);

const char* recon_err_name(int code);
const char* recon_fault_name(recon_fault_t f);

#endif /* RECONCILE_H */
