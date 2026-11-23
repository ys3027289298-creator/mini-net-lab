/*
 * clock.h - network clock offset estimation with min-RTT filtering.
 *
 * Failure isolation contract:
 *   - bad samples are rejected with a named clock_err_t, never abort;
 *   - clock_inject() arms exactly one fault; faults stay inside this module.
 */
#ifndef CLOCK_H
#define CLOCK_H

#define MAX_SAMPLES 16

typedef enum {
    CLK_OK             = 0,
    CLK_ERR_NULL       = -20,
    CLK_ERR_TIME_ORDER = -21,   /* t_recv before t_send */
    CLK_ERR_NO_SAMPLES = -22,   /* filter window empty */
    CLK_ERR_INJECT_ADD = -23    /* injected failure inside sync_add */
} clock_err_t;

typedef enum {
    CLK_FAULT_NONE = 0,
    CLK_FAULT_ADD  = 1
} clock_fault_t;

typedef struct { double rtt; double offset; double at; } Sample;

typedef struct {
    Sample s[MAX_SAMPLES];
    int n;
    double best_rtt;
    double offset;
} Sync;

void clock_sync_init(Sync* sy);
/* Returns number of samples averaged (>0) or a negative clock_err_t. */
int  clock_sync_add(Sync* sy, double t_send, double t_recv, double remote_time);

void clock_inject(clock_fault_t f);
void clock_reset_faults(void);
clock_fault_t clock_current_fault(void);

const char* clock_err_name(int code);
const char* clock_fault_name(clock_fault_t f);

#endif /* CLOCK_H */
