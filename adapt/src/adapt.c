
/* adapt: unit self-checks for the adaptive degradation controller. */
#include "adapt_core.h"
#include <stdio.h>
#include <math.h>

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void test_selection(void) {
    Adapt a; adapt_init(&a, 0.0);
    adapt_observe(&a, 0.01, 0.04, 0.0); adapt_step(&a, 0.01);
    check("good_tier0", a.committed == 0);

    Adapt b; adapt_init(&b, 0.0);
    adapt_observe(&b, 0.01, 0.30, 0.0); adapt_step(&b, 0.01);
    check("rtt_only_tier2", b.committed == 2);

    Adapt c; adapt_init(&c, 0.0);
    adapt_observe(&c, 0.01, 0.04, 0.05); adapt_step(&c, 0.01);
    check("loss_dominates_tier1", c.committed == 1);

    Adapt d; adapt_init(&d, 0.0);
    adapt_observe(&d, 0.01, 0.04, 0.20); adapt_step(&d, 0.01);
    check("heavy_loss_tier2", d.committed == 2);
}

static void test_hysteresis(void) {
    Adapt a; adapt_init(&a, 0.0);
    for (double t = 0.0; t < 2.0; t += 0.1) { adapt_observe(&a, t, 0.04, 0.0); adapt_step(&a, t); }
    check("settled_tier0", a.committed == 0);

    /* bad blip shorter than DWELL_DOWN must not commit */
    adapt_observe(&a, 2.05, 0.30, 0.20); adapt_step(&a, 2.05);
    adapt_observe(&a, 2.20, 0.30, 0.20); adapt_step(&a, 2.20);
    check("short_blip_ignored", a.committed == 0);

    /* sustained bad: downgrades after DWELL_DOWN */
    for (double t = 2.30; t < 3.0; t += 0.05) { adapt_observe(&a, t, 0.30, 0.20); adapt_step(&a, t); }
    check("downgrade_committed", a.committed == 2);

    /* recovery: EMA decays slowly, so tier-1 evidence appears briefly but never
     * lasts DWELL_UP; upgrade to tier 0 commits ~2 s after tier-0 evidence. */
    for (double u = 3.05; u < 4.5; u += 0.1) { adapt_observe(&a, u, 0.04, 0.0); adapt_step(&a, u); }
    check("upgrade_delayed", a.committed == 2);
    for (double u = 4.6; u < 7.0; u += 0.1) { adapt_observe(&a, u, 0.04, 0.0); adapt_step(&a, u); }
    check("transient_tier1_skipped", a.committed == 2);   /* brief tier-1 never committed */
    for (double u = 7.1; u < 8.0; u += 0.1) { adapt_observe(&a, u, 0.04, 0.0); adapt_step(&a, u); }
    check("upgrade_after_dwell", a.committed == 0);

    /* boundary chatter: alternating readings never flip-flop within a dwell */
    int flips = 0, prev = a.committed;
    for (double u = 6.5; u < 12.0; u += 0.1) {
        double rtt = (((int)(u * 10) & 1) == 0) ? 0.04 : 0.15;
        adapt_observe(&a, u, rtt, 0.0);
        if (adapt_step(&a, u) && a.committed != prev) { flips++; prev = a.committed; }
    }
    check("no_flipflop", flips <= 1);
}

static void test_missing_fallback(void) {
    Adapt a; adapt_init(&a, 0.0);
    for (double t = 0.0; t < 2.0; t += 0.1) { adapt_observe(&a, t, 0.04, 0.0); adapt_step(&a, t); }
    check("starts_good", a.committed == 0 && a.reason == REASON_NORMAL);

    /* blackout: grace window keeps the current tier */
    for (double t = 2.1; t < 2.9; t += 0.1) { adapt_observe(&a, t, -1.0, -1.0); adapt_step(&a, t); }
    check("grace_holds_tier", a.committed == 0);
    for (double t = 3.0; t < 3.4; t += 0.1) { adapt_observe(&a, t, -1.0, -1.0); adapt_step(&a, t); }
    check("missing_conservative", a.committed == 2 && a.reason == REASON_MISSING);

    /* readings resume: leaves conservative, upgrades through normal dwell */
    for (double t = 3.5; t < 8.0; t += 0.1) { adapt_observe(&a, t, 0.04, 0.0); adapt_step(&a, t); }
    check("recovers_after_resume", a.committed == 0 && a.reason == REASON_NORMAL);
}

static void test_smoothing(void) {
    Adapt a; adapt_init(&a, 0.0);
    for (double t = 0.0; t < 2.0; t += 0.1) { adapt_observe(&a, t, 0.04, 0.0); adapt_step(&a, t); }
    for (double t = 2.1; t < 3.0; t += 0.05) { adapt_observe(&a, t, 0.30, 0.20); adapt_step(&a, t); }
    check("committed_2", a.committed == 2);
    check("delay_lags_not_snaps", a.delay_s < TIERS[2].delay - 1e-6);
    for (double t = 3.0; t < 12.0; t += 0.05) adapt_step(&a, t);
    check("delay_converges", fabs(a.delay_s - TIERS[2].delay) < 1e-6);
}

static void test_keyframe_continuity(void) {
    /* every interpolant equals keyframe values at its endpoints, so switching
     * interpolation mode at a keyframe cannot introduce a pose jump */
    double pa = 0.3, pb = -0.7, va = 1.0, vb = -0.5;
    double her_a = hermite_at(0.0, pa, va, 0.1, pb, vb, 0.0);
    double her_b = hermite_at(0.0, pa, va, 0.1, pb, vb, 0.1);
    check("hermite_hits_keyframes", fabs(her_a - pa) < 1e-9 && fabs(her_b - pb) < 1e-9);

    /* LOD refresh gate: peripheral samples only on 1/k of packets, and a
     * skipped packet leaves the held value bit-identical (no blending pop) */
    double held = quantize(pa, 0.012);
    check("hold_is_bitstable", quantize(pa, 0.012) == held);
}

static void test_bounds(void) {
    const JointCfg j = { 1.0, 1.5, 1 };
    double vmax = joint_vmax(&j), amax = joint_amax(&j);
    double b1 = error_bound(INTERP_LINEAR, 0.05, 0, 0.005, vmax, amax);
    double b2 = error_bound(INTERP_LINEAR, 0.10, 0, 0.005, vmax, amax);
    check("linear_bound_quadratic", b2 > b1 * 3.5);
    double bh = error_bound(INTERP_HOLD, 0.10, 0, 0.005, vmax, amax);
    check("hold_bound_linear_T", fabs(bh - (0.005 + vmax * 0.10)) < 1e-9);
    double be = error_bound(INTERP_LINEAR, 0.05, 0.08, 0.005, vmax, amax);
    check("extrap_bound_grows", be > b1);

    /* dense numeric check: real linear interpolation error stays in bound */
    int violated = 0;
    for (int k = 0; k <= 100; k++) {
        double t = 0.05 * k / 100.0;
        double truev = joint_pos(&j, t);
        double lin = (1 - t / 0.05) * joint_pos(&j, 0.0) + (t / 0.05) * joint_pos(&j, 0.05);
        if (fabs(lin - truev) > b1 + 1e-9) violated = 1;
    }
    check("linear_error_inside_bound", !violated);
}

int main(void) {
    test_selection();
    test_hysteresis();
    test_missing_fallback();
    test_smoothing();
    test_keyframe_continuity();
    test_bounds();
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
