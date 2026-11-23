
/* clock: network clock offset estimation with min-RTT filtering.
 *
 * Failure-isolation version: named clock_err_t codes plus deterministic
 * fault hooks. Estimation math, ring eviction and filtering are unchanged.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "clock.h"

void clock_sync_init(Sync* sy) {
    if (sy == NULL) return;
    memset(sy, 0, sizeof(*sy));
    sy->best_rtt = 1e9;
}

/* --- deterministic, process-local fault injection ----------------------- */
static clock_fault_t g_clk_fault = CLK_FAULT_NONE;

void clock_inject(clock_fault_t f) { g_clk_fault = f; }
void clock_reset_faults(void) { g_clk_fault = CLK_FAULT_NONE; }
clock_fault_t clock_current_fault(void) { return g_clk_fault; }

const char* clock_err_name(int code) {
    switch (code) {
    case CLK_OK:             return "CLK_OK";
    case CLK_ERR_NULL:       return "CLK_ERR_NULL";
    case CLK_ERR_TIME_ORDER: return "CLK_ERR_TIME_ORDER";
    case CLK_ERR_NO_SAMPLES: return "CLK_ERR_NO_SAMPLES";
    case CLK_ERR_INJECT_ADD: return "CLK_ERR_INJECT_ADD";
    default:                 return "CLK_ERR_UNKNOWN";
    }
}

const char* clock_fault_name(clock_fault_t f) {
    switch (f) {
    case CLK_FAULT_NONE: return "CLK_FAULT_NONE";
    case CLK_FAULT_ADD:  return "CLK_FAULT_ADD";
    default:             return "CLK_FAULT_UNKNOWN";
    }
}

int clock_sync_add(Sync* sy, double t_send, double t_recv, double remote_time) {
    if (sy == NULL) return CLK_ERR_NULL;
    if (t_recv < t_send) return CLK_ERR_TIME_ORDER;
    if (g_clk_fault == CLK_FAULT_ADD) return CLK_ERR_INJECT_ADD;
    double rtt = t_recv - t_send;
    double offset = remote_time - (t_send + t_recv) * 0.5;
    if (sy->n >= MAX_SAMPLES) {
        memmove(&sy->s[0], &sy->s[1], sizeof(Sample) * (MAX_SAMPLES - 1));
        sy->n = MAX_SAMPLES - 1;
    }
    sy->s[sy->n].rtt = rtt;
    sy->s[sy->n].offset = offset;
    sy->s[sy->n].at = t_recv;
    sy->n++;
    if (rtt < sy->best_rtt) sy->best_rtt = rtt;

    /* keep samples within 1.5x best rtt, average their offsets */
    double sum = 0.0; int cnt = 0;
    for (int i = 0; i < sy->n; i++) {
        if (sy->s[i].rtt <= sy->best_rtt * 1.5 + 1e-9) {
            sum += sy->s[i].offset; cnt++;
        }
    }
    if (cnt == 0) return CLK_ERR_NO_SAMPLES;
    sy->offset = sum / cnt;
    return cnt;
}

#ifdef MINILAB_SELFTEST
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int MINILAB_SELFTEST(void) {
    Sync sy;
    clock_sync_init(&sy);
    double true_off = 2.0;
    check("add1", clock_sync_add(&sy, 0.0, 0.10, 0.05 + true_off) > 0);
    check("offset_close", fabs(sy.offset - true_off) < 0.01);
    check("best_rtt", fabs(sy.best_rtt - 0.10) < 1e-9);

    check("add2", clock_sync_add(&sy, 1.0, 1.20, 1.10 + true_off) > 0);
    check("still_close", fabs(sy.offset - true_off) < 0.01);

    check("reject_time_reversal", clock_sync_add(&sy, 5.0, 4.0, 0.0) == CLK_ERR_TIME_ORDER);

    Sync sp;
    clock_sync_init(&sp);
    for (int i = 0; i < 40; i++)
        clock_sync_add(&sp, i * 1.0, i * 1.0 + 0.05 + (i % 5) * 0.01, i * 1.0 + 0.025 + true_off);
    check("ring_kept", sp.n == MAX_SAMPLES);
    check("stable_offset", fabs(sp.offset - true_off) < 0.01);

    /* failure injection: one fault, named code, ring untouched */
    int n_before = sp.n;
    clock_inject(CLK_FAULT_ADD);
    check("inject_add", clock_sync_add(&sp, 99.0, 99.1, 101.0) == CLK_ERR_INJECT_ADD);
    clock_reset_faults();
    check("inject_isolated", sp.n == n_before);
    check("fault_cleared", clock_sync_add(&sp, 99.0, 99.1, 101.0) > 0);

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
