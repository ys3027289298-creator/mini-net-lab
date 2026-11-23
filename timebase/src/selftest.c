
/* timebase self-test: monotonicity, per-module attribution, injected sources. */
#include "timebase.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Source that walks a fixed schedule, so no system clock is involved. */
typedef struct {
    const uint32_t* v;
    int n;
    int i;
} Seq;

static uint32_t seq_read(void* ctx) {
    Seq* s = (Seq*)ctx;
    uint32_t v = s->v[s->i];
    if (s->i < s->n - 1) s->i++;
    return v;
}

/* Source that never decreases but repeats and jitters: emulates sampling a
 * monotonic clock at irregular intervals. */
typedef struct { uint32_t v; unsigned rng; int n; int i; } Jitter;

static uint32_t jitter_read(void* ctx) {
    Jitter* j = (Jitter*)ctx;
    j->rng = j->rng * 1103515245u + 12345u;
    j->v += 1u + (j->rng >> 29);   /* 1..8 ms steps */
    j->i++;
    return j->v;
}

int main(void) {
    TimeBase tb;
    uint32_t now = 0;

    tb_init(&tb);
    check("no_base_yet", tb_now(&tb, &now) == TB_ERR_NULL_SOURCE);

    check("feed_ok", tb_feed(&tb, TB_SNAPSHOT, 100) == TB_OK);
    check("now_is_fed", tb_now(&tb, &now) == TB_OK && now == 100);
    check("module_last", tb_stat(&tb, TB_SNAPSHOT)->last == 100);

    /* Roommate source: 90 is behind the shared base but this module has not
     * seen it before, so it is a lag, not a reversal. */
    check("other_module_lag", tb_feed(&tb, TB_CLOCK, 90) == TB_OK_LAGGING);
    check("base_not_pulled_back", tb_now(&tb, &now) == TB_OK && now == 100);
    check("lag_counted", tb_stat(&tb, TB_CLOCK)->lags == 1);

    check("same_module_reversal", tb_feed(&tb, TB_SNAPSHOT, 99) == TB_ERR_BACKWARD);
    check("reversal_counted", tb_stat(&tb, TB_SNAPSHOT)->rejects == 1);
    check("reversal_keeps_base", tb_now(&tb, &now) == TB_OK && now == 100);

    check("out_of_range", tb_feed(&tb, TB_AOI, TIME_MAX_TICK + 1u) == TB_ERR_RANGE);
    check("range_counted", tb_stat(&tb, TB_AOI)->rejects == 1);
    check("range_last_err", tb_stat(&tb, TB_AOI)->last_err == TB_ERR_RANGE);

    check("forward_wins", tb_feed(&tb, TB_RECONCILE, 400) == TB_OK);
    check("now_forward", tb_now(&tb, &now) == TB_OK && now == 400);

    /* No source configured: the error belongs to the reading module. */
    check("null_source", tb_read(&tb, TB_AOI, NULL) == TB_ERR_NULL_SOURCE);
    check("null_source_attr", tb_stat(&tb, TB_AOI)->rejects == 2);
    check("null_source_other_clean", tb_stat(&tb, TB_RECONCILE)->rejects == 0);

    /* Injected sequence, including a deliberate reversal, attributed to the
     * first reader while the second reader sees it as a lag. */
    TimeBase ti;
    tb_init(&ti);
    static const uint32_t seq[] = { 500, 500, 499, 500 };
    Seq sq = { seq, 4, 0 };
    tb_set_source(&ti, seq_read, &sq);
    check("src_500", tb_read(&ti, TB_SNAPSHOT, &now) == TB_OK && now == 500);
    check("src_repeat", tb_read(&ti, TB_SNAPSHOT, &now) == TB_OK && now == 500);
    check("src_back", tb_read(&ti, TB_SNAPSHOT, &now) == TB_ERR_BACKWARD);
    check("src_back_attr", tb_stat(&ti, TB_SNAPSHOT)->rejects == 1 &&
                            tb_stat(&ti, TB_CLOCK)->rejects == 0);
    check("src_recovers", tb_read(&ti, TB_SNAPSHOT, &now) == TB_OK && now == 500);

    /* Long run: 2 simulated hours, 10 ms nominal, monotonic + jitter.  The
     * base must never go backwards and drift must stay bounded. */
    TimeBase tl;
    tb_init(&tl);
    Jitter j = { 0, 1u, 0, 0 };
    tb_set_source(&tl, jitter_read, &j);
    uint32_t prev = 0, cur = 0;
    int monotone = 1, errors = 0;
    const int samples = 720000;   /* 2 h / 10 ms */
    for (int i = 0; i < samples; i++) {
        tb_status st = tb_read(&tl, TB_CLOCK, &cur);
        if (st < 0) errors++;
        if (i > 0 && cur < prev) monotone = 0;
        prev = cur;
    }
    tb_now(&tl, &cur);
    check("long_monotone", monotone);
    check("long_no_errors", errors == 0);
    check("long_accepts", tb_stat(&tl, TB_CLOCK)->accepts == (uint64_t)samples);
    check("long_base_matches", cur == prev);

    /* Drift helpers are integer-only. */
    check("drift_zero", tb_drift_units(30000, 30000) == 0);
    check("drift_units", tb_drift_units(30003, 30000) == 3);
    check("drift_ppm", tb_drift_ppm(3, 30000) == 100);
    check("drift_ppm_zero_span", tb_drift_ppm(5, 0) == 0);

    check("status_names", strcmp(tb_status_name(TB_ERR_NULL_SOURCE), "null_source") == 0 &&
                          strcmp(tb_status_name(TB_ERR_BACKWARD), "backward") == 0);
    check("module_names", strcmp(tb_module_name(TB_SNAPSHOT), "snapshot") == 0 &&
                          strcmp(tb_module_name(TB_RECONCILE), "reconcile") == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
