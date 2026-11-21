
/* tickdrift: bounded tick-interval correction against a reference clock.
 *
 * A client ticks at a nominal interval `base`.  When the server hitches or
 * runs fast/slow the client's timeline drifts away from the reference clock.
 * Snapping the interval to "catch up" is what makes a stalled server turn
 * into a client avalanche (a huge frame delta, then rollback).  Instead we
 * nudge the interval a little each tick:
 *
 *     err  = client_time - ref_time            (+ : client is ahead)
 *     want = base - gain * err                 (ahead -> shorter interval)
 *     iv   = clamp(want, base*(1-max_adj), base*(1+max_adj))
 *
 * Properties the tests pin down:
 *   - client time strictly increases; no interval is ever <= 0,
 *   - per-tick correction never exceeds max_adj * base,
 *   - the cumulative deviation from a non-correcting baseline is bounded
 *     by ticks * max_adj * base (and by sum(|iv - base|), which is tighter),
 *   - a correction request beyond the limit is reported (counted and logged
 *     with the requested value) and applied as the limit, never as a snap,
 *   - a non-finite reference sample, or one that moves backwards, is counted
 *     and held at the base interval; it never enters the timeline or the
 *     corrector.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_LOG 8

/* One correction the corrector wanted to make but could not fully apply. */
typedef struct {
    int tick;
    double err;         /* client - ref at the step */
    double requested;   /* interval asked for, pre-clamp */
    double applied;     /* interval actually used */
    int clamped;
} LogEntry;

typedef struct {
    double base;        /* nominal tick interval (s) */
    double max_adj;     /* max |iv - base| per tick, fraction of base, < 1 */
    double deadband;    /* |err| at or below this is left uncorrected */
    double gain;        /* fraction of err corrected per tick, 0..1 */
    double min_iv;      /* hard floor for any emitted interval */

    double client;      /* corrected client timeline, strictly increasing */
    double nominal;     /* same ticks run at `base`, the no-drift baseline */
    double ref;         /* previous reference-clock reading */
    double cum;         /* client - nominal: cumulative drift vs baseline */
    double abs_adj;     /* sum |iv - base|: the correction budget spent */
    double max_cum;     /* max |cum| observed */
    double last_err;    /* client - ref as of the last step */
    double last_iv;     /* interval used by the last step */
    double last_request;/* interval the corrector asked for, pre-clamp */

    int ticks;
    int clamped;        /* steps whose request hit the per-tick limit */
    int skipped;        /* steps with no usable reference (non-finite or rewound) */
    int backward;       /* of those, reference readings that went backwards */
    int violations;     /* non-finite/illegal values refused */

    LogEntry log[MAX_LOG];  /* over-limit steps, for reporting */
    int logn;
} Drift;

static void drift_init(Drift* d, double base, double max_adj, double deadband,
                       double gain) {
    memset(d, 0, sizeof(*d));
    d->base = base > 0.0 ? base : 1.0 / 60.0;
    if (max_adj < 0.0) max_adj = 0.0;
    if (max_adj > 0.95) max_adj = 0.95;      /* keep the floor strictly positive */
    d->max_adj = max_adj;
    d->deadband = deadband < 0.0 ? 0.0 : deadband;
    d->gain = gain < 0.0 ? 0.0 : (gain > 1.0 ? 1.0 : gain);
    d->min_iv = d->base * (1.0 - d->max_adj);
    d->last_iv = d->base;
    d->last_request = d->base;
}

/* Advance one tick against the reference reading `ref_now`.  Returns the
 * interval the client should use.  Call once per tick, in order. */
static double drift_step(Drift* d, double ref_now) {
    d->ticks++;
    int usable = isfinite(ref_now);

    if (usable && d->ticks > 1 && ref_now < d->ref - 1e-12) {
        /* An out-of-order / rewound reference sample.  Report it and hold:
         * feeding it in would ask the corrector to slow down for a clock
         * glitch it did not cause. */
        d->backward++;
        usable = 0;
    }
    if (!usable) d->skipped++;

    double err = usable ? d->client - ref_now : d->last_err;
    if (!isfinite(err)) { err = 0.0; d->violations++; }
    d->last_err = err;

    double iv = d->base;
    if (usable && fabs(err) > d->deadband) {
        double want = d->base - d->gain * err;
        double hi = d->base * (1.0 + d->max_adj);
        double lo = d->base * (1.0 - d->max_adj);
        d->last_request = want;
        if (want > hi)      { iv = hi; d->clamped++; }
        else if (want < lo) { iv = lo; d->clamped++; }
        else                iv = want;
    } else {
        d->last_request = d->base;
    }

    /* Last line of defence: nothing illegal may enter the timeline. */
    if (!isfinite(iv) || iv <= 0.0) { iv = d->base; d->violations++; }
    if (iv < d->min_iv) iv = d->min_iv;

    d->client += iv;
    d->nominal += d->base;
    d->cum = d->client - d->nominal;
    d->abs_adj += fabs(iv - d->base);
    if (fabs(d->cum) > d->max_cum) d->max_cum = fabs(d->cum);
    d->last_iv = iv;
    if (usable) d->ref = ref_now;

    if (d->logn < MAX_LOG && d->last_request != iv) {
        LogEntry* e = &d->log[d->logn++];
        e->tick = d->ticks - 1;
        e->err = err;
        e->requested = d->last_request;
        e->applied = iv;
        e->clamped = 1;
    }
    return iv;
}

/* Structural upper bound on |cum| after `ticks` steps. */
static double drift_bound(const Drift* d) {
    return d->ticks * d->base * d->max_adj + 1e-12;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    const double BASE = 1.0 / 60.0;

    /* ---- 1. baseline: reference advances exactly one base interval ---- */
    Drift b;
    drift_init(&b, BASE, 0.05, 0.0, 0.5);
    double r = 0.0, max_iv_err = 0.0;
    for (int i = 0; i < 600; i++) {
        double iv = drift_step(&b, r);
        if (fabs(iv - BASE) > max_iv_err) max_iv_err = fabs(iv - BASE);
        r += BASE;
    }
    check("baseline_iv_is_base", max_iv_err < 1e-15);
    check("baseline_cum_zero", fabs(b.cum) < 1e-9);
    check("baseline_no_clamp", b.clamped == 0 && b.skipped == 0);
    check("baseline_no_violation", b.violations == 0 && b.backward == 0);
    check("baseline_elapsed", fabs(b.client - 600 * BASE) < 1e-9);

    /* ---- 2. 12-tick server hitch (200 ms): no snap, gradual recovery ---- */
    Drift d;
    drift_init(&d, BASE, 0.05, 0.0, 0.5);
    double rr = 0.0, prev = 0.0, worst_err = 0.0;
    int monotonic = 1, iv_legal = 1, clamp_seen = 0;
    for (int i = 0; i < 400; i++) {
        if (i < 100)      rr += BASE;             /* healthy */
        else if (i < 112) { /* stall: reference does not advance */ }
        else              rr += BASE;             /* resume */
        double iv = drift_step(&d, rr);
        if (d.client <= prev) monotonic = 0;
        prev = d.client;
        if (iv < BASE * 0.95 - 1e-12 || iv > BASE * 1.05 + 1e-12) iv_legal = 0;
        if (d.clamped) clamp_seen = 1;
        if (fabs(d.last_err) > worst_err) worst_err = fabs(d.last_err);
    }
    double nominal_ticks = 400 * BASE;
    check("hitch_strictly_forward", monotonic);
    check("hitch_iv_legal", iv_legal);
    check("hitch_was_clamped", clamp_seen);
    check("hitch_no_violation", d.violations == 0 && d.backward == 0);
    check("hitch_cum_within_bound", fabs(d.cum) <= drift_bound(&d));
    check("hitch_cum_within_budget", fabs(d.max_cum) <= d.abs_adj + 1e-12);
    check("hitch_recovered", fabs(d.last_err) < 0.01);
    check("hitch_error_was_real", worst_err > 0.15);
    check("hitch_correction_spent", d.abs_adj > 0.0);
    check("hitch_baseline_differs", fabs(d.client - nominal_ticks) > BASE * 0.5);

    /* ---- 3. request beyond the per-tick limit: report, do not snap ---- */
    Drift j;
    drift_init(&j, BASE, 0.05, 0.0, 1.0);
    double rj = 0.0;
    drift_step(&j, rj); rj += BASE;              /* seed */
    double c_before = j.client;
    rj += BASE + 2.0;                            /* reference leaps 2 s */
    double ivj = drift_step(&j, rj);
    check("jump_clamped", j.clamped >= 1);
    check("jump_iv_at_limit", fabs(ivj - BASE * 1.05) < 1e-12);
    check("jump_request_reported", fabs(j.last_request - BASE) > BASE * 0.05);
    check("jump_forward_only", j.client > c_before);
    check("jump_error_visible", j.last_err < -1.0);
    check("jump_cum_bounded", fabs(j.cum) <= drift_bound(&j) + 1e-12);

    /* ---- 4. backward reference: reported, timeline still monotonic ---- */
    Drift bk;
    drift_init(&bk, BASE, 0.05, 0.0, 0.5);
    double rb = 0.0;
    for (int i = 0; i < 10; i++) { drift_step(&bk, rb); rb += BASE; }
    double cb = bk.client;
    double ivb = drift_step(&bk, rb - 1.0);     /* reference jumps back */
    check("backward_counted", bk.backward == 1);
    check("backward_held_at_base", fabs(ivb - BASE) < 1e-15);
    check("backward_forward_only", bk.client > cb && ivb > 0.0);

    /* ---- 5. non-finite reference: skipped, held at base, no NaN ---- */
    Drift n;
    drift_init(&n, BASE, 0.05, 0.0, 0.5);
    drift_step(&n, 0.0);
    double ivn = drift_step(&n, NAN);
    check("nan_skipped", n.skipped == 1);
    check("nan_holds_base", fabs(ivn - BASE) < 1e-15);
    check("nan_no_violation", n.violations == 0);
    check("nan_client_finite", isfinite(n.client));
    drift_step(&n, INFINITY);
    check("inf_skipped", n.skipped == 2 && isfinite(n.client));

    /* ---- 6. deadband: sub-threshold jitter is not chased ---- */
    Drift db;
    drift_init(&db, BASE, 0.05, 0.002, 1.0);
    double rdb = 0.0, max_dev = 0.0;
    for (int i = 0; i < 200; i++) {
        double iv = drift_step(&db, rdb);
        if (fabs(iv - BASE) > max_dev) max_dev = fabs(iv - BASE);
        rdb += BASE + (i % 2 ? 0.0005 : -0.0005);
    }
    check("deadband_ignores_jitter", max_dev < 1e-15);
    check("deadband_no_clamp", db.clamped == 0);

    /* ---- 7. max_adj == 0 locks the interval at base ---- */
    Drift z;
    drift_init(&z, BASE, 0.0, 0.0, 1.0);
    double rz = 0.0;
    int z_moved = 0;
    for (int i = 0; i < 50; i++) {
        double iv = drift_step(&z, rz);
        if (fabs(iv - BASE) > 1e-15) z_moved = 1;
        rz += BASE + 0.01;
    }
    check("zero_adj_locked", z_moved == 0);
    check("zero_adj_cum_zero", fabs(z.cum) < 1e-12);
    check("zero_adj_clamped", z.clamped >= z.ticks - 1);

    /* ---- 8. configuration guards ---- */
    Drift g;
    drift_init(&g, -1.0, 5.0, -1.0, 2.0);
    check("bad_base_defaulted", g.base > 0.0);
    check("adj_capped_below_one", g.max_adj < 1.0);
    check("gain_capped", g.gain <= 1.0);
    check("min_iv_positive", g.min_iv > 0.0);

    /* ---- 9. the over-limit report log stays legal and records the jump ---- */
    int log_ok = j.logn > 0;
    for (int i = 0; i < j.logn; i++) {
        if (!(j.log[i].applied > 0.0) || !isfinite(j.log[i].applied)) log_ok = 0;
        if (!j.log[i].clamped) log_ok = 0;
    }
    check("log_iv_positive", log_ok);
    check("log_records_request", fabs(j.log[0].requested - j.log[0].applied) > BASE * 0.05);
    check("log_captures_jump", j.log[0].err < -1.0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
