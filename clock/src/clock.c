
/* clock: network clock offset estimation with min-RTT filtering.
 *
 * Bad samples are reported through the shared mnl sink (module=CLOCK) with
 * the offending field named explicitly; a rejected sample never enters the
 * ring or moves the estimate. Legacy self-test output is byte-identical.
 */
#include "mnl.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_SAMPLES 16

typedef struct { double rtt; double offset; double at; } Sample;

typedef struct {
    Sample s[MAX_SAMPLES];
    int n;
    double best_rtt;
    double offset;
} Sync;

/* Compile-time guarantee that caller-owned mnl_state_t storage fits. */
_Static_assert(sizeof(Sync) <= sizeof(mnl_state_t), "Sync exceeds mnl_state_t");

static MNL_MAYBE_UNUSED void sync_init(Sync* sy) { memset(sy, 0, sizeof(*sy)); sy->best_rtt = 1e9; }

static int sync_add(Sync* sy, double t_send, double t_recv, double remote_time,
                    mnl_sink_t* sk) {
    if (sy == NULL) return mnl_report(sk, MNL_MOD_CLOCK,
                                      MNL_F_SYNC, MNL_R_NULL);
    if (!isfinite(t_send)) return mnl_report(sk, MNL_MOD_CLOCK,
                                             MNL_F_T_SEND, MNL_R_NONFINITE);
    if (!isfinite(t_recv)) return mnl_report(sk, MNL_MOD_CLOCK,
                                             MNL_F_T_RECV, MNL_R_NONFINITE);
    if (!isfinite(remote_time)) return mnl_report(sk, MNL_MOD_CLOCK,
                                                  MNL_F_REMOTE_TIME, MNL_R_NONFINITE);
    if (t_recv < t_send) return mnl_report(sk, MNL_MOD_CLOCK,
                                           MNL_F_T_RECV, MNL_R_RANGE);
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
    if (cnt == 0) return -1;
    sy->offset = sum / cnt;
    return cnt;
}

/* ---- zero-copy batch runner ------------------------------------------- */

int mnl_clock_run(mnl_state_t* st, mnl_sink_t* sk,
                  const mnl_item_t* items, int n, mnl_result_t* res) {
    if (sk == NULL) return -1;
    if (st == NULL) return mnl_report(sk, MNL_MOD_CLOCK, MNL_F_SYNC, MNL_R_NULL);
    if (items == NULL || n < 0 || res == NULL)
        return mnl_report(sk, MNL_MOD_CLOCK, MNL_F_NONE, MNL_R_NULL);

    Sync* sy = (Sync*)st;
    /* Lazy init: a freshly zeroed caller-owned state needs the sentinel
     * best_rtt before the first sample can be filtered. */
    if (sy->n == 0 && sy->best_rtt == 0.0) sy->best_rtt = 1e9;
    int failures = 0;
    for (int i = 0; i < n; i++) {
        mnl_sink_begin(sk, i);
        mnl_result_t* r = &res[i];
        memset(r, 0, sizeof(*r));
        const mnl_item_t* it = &items[i];

        int rc;
        if (it->op != MNL_CLOCK_ADD) {
            rc = mnl_report(sk, MNL_MOD_CLOCK, MNL_F_NONE, MNL_R_BAD_OP);
        } else if (it->payload == NULL) {
            rc = mnl_report(sk, MNL_MOD_CLOCK, MNL_F_SAMPLE, MNL_R_NULL);
        } else {
            /* The sample carrier is read in place by pointer; it is never
             * copied into local storage or a scratch buffer. */
            const mnl_clock_sample_t* sm = (const mnl_clock_sample_t*)it->payload;
            rc = sync_add(sy, sm->t_send, sm->t_recv, sm->remote_time, sk);
        }

        r->rc = rc;
        if (rc < 0) {
            failures++;
            r->err = sk->errs[sk->count - 1];
        } else {
            r->fval = (float)sy->offset;
            r->ival = rc;
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
    Sync sy;
    sync_init(&sy);
    double true_off = 2.0;
    check("add1", sync_add(&sy, 0.0, 0.10, 0.05 + true_off, NULL) > 0);
    check("offset_close", fabs(sy.offset - true_off) < 0.01);
    check("best_rtt", fabs(sy.best_rtt - 0.10) < 1e-9);

    check("add2", sync_add(&sy, 1.0, 1.20, 1.10 + true_off, NULL) > 0);
    check("still_close", fabs(sy.offset - true_off) < 0.01);

    check("reject_time_reversal", sync_add(&sy, 5.0, 4.0, 0.0, NULL) == -1);

    Sync sp;
    sync_init(&sp);
    for (int i = 0; i < 40; i++) sync_add(&sp, i * 1.0, i * 1.0 + 0.05 + (i % 5) * 0.01, i * 1.0 + 0.025 + true_off, NULL);
    check("ring_kept", sp.n == MAX_SAMPLES);
    check("stable_offset", fabs(sp.offset - true_off) < 0.01);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
#endif
