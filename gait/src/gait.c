/* gait: locomotion phase tracking for remote characters.
 *
 * The remote animation plays at freq strides/s with footfall spacing `stride`
 * metres. Left foot plants at phase 0, right at 0.5. A planted foot only stays
 * put when stride length matches the distance covered per stride, so the
 * cadence implied by speed is
 *
 *     f_exp = |v| / stride          (strides/s)
 *
 * Backpedalling (negative x velocity) is a contract violation and rejected;
 * the transverse sign is irrelevant, so speed is |v|.
 *
 * The local phase advances at f_exp plus a correction that pulls it toward the
 * remote phase. The correction is the exact solution of de/dt = -kp * e over
 * dt, in revolutions,
 *
 *     corr = (e / 2pi) * (1 - exp(-kp * dt)),
 *
 * clamped to pull_max strides/s, so the loop cannot overshoot for any dt.
 *
 * Observability - each state-changing call refreshes:
 *   phase_err   signed remote - local error, wrapped to |e| <= pi radians
 *   slip_bound  |phase_err| mapped to metres of foot-placement error
 *   slip_total  accumulated |correction| in metres: the ground slip spent
 *               pulling into sync (feed-forward alone adds nothing)
 *   speed_resid |v| - freq * stride, instantaneous cadence mismatch
 *   trackable   last update had a valid, non-zero cadence
 *   rejects     number of rejected updates
 *
 * Rejected input never mutates tracked state: negative or non-finite velocity,
 * NaN error or dt, dt < 0, zero cadence, and manual pulls outside the budget
 * (a phase skip must be re-aligned, not pulled).
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define GAIT_PI 3.14159265358979323846f
#define GAIT_TWO_PI (2.0f * GAIT_PI)

typedef struct {
    float phase;        /* local phase in [0,1) */
    float freq;         /* cadence applied on the last update, strides/s */
    float stride;       /* footfall spacing, m */
    float kp;           /* phase pull gain, 1/s */
    float pull_max;     /* cap on corrective cadence, strides/s */
    float min_freq;     /* cadences at or below this are treated as zero */
    float max_corr;     /* largest manual pull accepted, rev */
    float speed_resid;  /* |v| - freq * stride, m/s */
    float phase_err;    /* last signed remote - local error, rad */
    float slip_bound;   /* phase_err mapped to metres */
    float slip_total;   /* accumulated correction slip, m */
    int trackable;      /* last update had a usable cadence */
    int rejects;
} Gait;

typedef struct {
    float f_exp;        /* cadence implied by speed, strides/s */
    float speed;        /* |v|, m/s */
    float phase;        /* expected phase after dt, [0,1) */
} GaitExpected;

static float wrap_pi(float e) {
    e = fmodf(e, GAIT_TWO_PI);
    if (e > GAIT_PI) e -= GAIT_TWO_PI;
    if (e < -GAIT_PI) e += GAIT_TWO_PI;
    return e;
}

static float phase01(float p) {
    p = fmodf(p, 1.0f);
    if (p < 0.0f) p += 1.0f;
    return p;
}

/* one stride of ground travel per full phase turn */
static float slip_m(const Gait* g, float err_rad) {
    return (fabsf(err_rad) / GAIT_TWO_PI) * g->stride;
}

static int gait_init(Gait* g, float stride, float kp) {
    if (g == NULL) return -1;
    memset(g, 0, sizeof(*g));
    if (!isfinite(stride) || stride <= 0.0f) return -1;
    if (!isfinite(kp) || kp <= 0.0f) return -1;
    g->stride = stride;
    g->kp = kp;
    g->pull_max = 4.0f;
    g->min_freq = 1e-3f;
    g->max_corr = 0.25f;
    return 0;
}

/* expected cadence and phase from horizontal velocity over dt */
static int gait_expect(const Gait* g, float vx, float vy, float dt, GaitExpected* out) {
    if (g == NULL || out == NULL) return -1;
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(dt) || dt < 0.0f) return -1;
    if (vx < 0.0f) return -1;
    float s = sqrtf(vx * vx + vy * vy);
    if (!isfinite(s)) return -1;
    float f = s / g->stride;
    if (!isfinite(f)) return -1;
    out->speed = s;
    out->f_exp = f;
    out->phase = phase01(g->phase + f * dt);
    return 0;
}

/* advance local phase at the speed-implied cadence plus a pull toward err_rad */
static int gait_track(Gait* g, float vx, float vy, float err_rad, float dt) {
    if (g == NULL) return -1;
    GaitExpected ex;
    if (!isfinite(err_rad) || gait_expect(g, vx, vy, dt, &ex) < 0) {
        g->rejects++;
        return -1;
    }
    if (ex.f_exp <= g->min_freq) {      /* zero cadence: no gait to synchronise */
        g->rejects++;
        return -1;
    }

    float e = wrap_pi(err_rad);
    float corr = (e / GAIT_TWO_PI) * (1.0f - expf(-g->kp * dt));  /* rev */
    float cap = g->pull_max * dt;
    if (corr > cap) corr = cap;
    if (corr < -cap) corr = -cap;

    g->phase = phase01(g->phase + ex.f_exp * dt + corr);
    g->freq = ex.f_exp + (dt > 0.0f ? corr / dt : 0.0f);
    g->speed_resid = ex.speed - g->freq * g->stride;
    g->phase_err = e;
    g->slip_bound = slip_m(g, e);
    g->slip_total += fabsf(corr) * g->stride;
    g->trackable = 1;
    return 0;
}

/* pull local phase to a remote phase sample over the shortest arc. A pull
 * beyond max_corr is refused: an ambiguous (>= half turn) or large offset must
 * be re-aligned, not eased in, or the feet swap sides. */
static int gait_sync(Gait* g, float remote_phase) {
    if (g == NULL) return -1;
    if (!isfinite(remote_phase)) { g->rejects++; return -1; }
    float d = wrap_pi((remote_phase - g->phase) * GAIT_TWO_PI) / GAIT_TWO_PI;
    if (fabsf(d) > g->max_corr) { g->rejects++; return -1; }
    g->phase = phase01(g->phase + d);
    g->phase_err = d * GAIT_TWO_PI;
    g->slip_bound = slip_m(g, g->phase_err);
    g->slip_total += g->slip_bound;
    return 0;
}

/* read the tracker against a fresh remote phase sample (revolutions) */
static void gait_observe(const Gait* g, float remote_phase, float* err_rad, float* slip_m_out) {
    float e = wrap_pi((remote_phase - g->phase) * GAIT_TWO_PI);
    if (err_rad) *err_rad = e;
    if (slip_m_out) *slip_m_out = slip_m(g, e);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* -- 1. init and expected phase from speed/stride -- */
    {
        Gait g;
        check("init_ok", gait_init(&g, 1.0f, 10.0f) == 0);
        check("init_bad_stride", gait_init(&g, 0.0f, 10.0f) == -1);
        check("init_bad_gain", gait_init(&g, 1.0f, 0.0f) == -1);

        GaitExpected ex;
        gait_init(&g, 0.5f, 10.0f);
        check("expect_ok", gait_expect(&g, 3.0f, 4.0f, 0.1f, &ex) == 0);
        check("expect_speed", fabsf(ex.speed - 5.0f) < 1e-5f);
        check("expect_freq", fabsf(ex.f_exp - 10.0f) < 1e-5f);
        check("expect_phase_wrapped", ex.phase >= 0.0f && ex.phase < 1.0f);
        check("expect_neg_velocity", gait_expect(&g, -5.0f, 0.0f, 0.0f, &ex) == -1);
        check("expect_nan", gait_expect(&g, NAN, 0.0f, 0.1f, &ex) == -1);
        check("expect_neg_dt", gait_expect(&g, 1.0f, 0.0f, -0.1f, &ex) == -1);
    }

    /* -- 2. rejects leave state untouched -- */
    {
        Gait g;
        gait_init(&g, 1.0f, 10.0f);
        check("neg_velocity_rejected", gait_track(&g, -2.0f, 0.0f, 0.0f, 0.02f) == -1);
        check("neg_reject_kept_state", g.rejects == 1 && g.phase == 0.0f &&
                                       g.freq == 0.0f && g.trackable == 0 &&
                                       g.slip_total == 0.0f);
        check("zero_cadence_rejected", gait_track(&g, 0.0f, 0.0f, 0.0f, 0.02f) == -1 &&
                                       g.rejects == 2 && g.phase == 0.0f);
        check("nan_err_rejected", gait_track(&g, 2.0f, 0.0f, NAN, 0.02f) == -1);
        check("nan_speed_rejected", gait_track(&g, NAN, 0.0f, 0.0f, 0.02f) == -1);
        check("neg_dt_rejected", gait_track(&g, 2.0f, 0.0f, 0.0f, -0.02f) == -1);
        check("valid_after_rejects", gait_track(&g, 2.0f, 0.0f, 0.0f, 0.02f) == 0 &&
                                      g.trackable == 1 && g.freq > 0.0f);
    }

    /* -- 3. constant speed: misaligned start is pulled in, error never grows -- */
    {
        Gait g;
        gait_init(&g, 1.0f, 10.0f);
        const float vx = 2.5f, dt = 0.001f, T = 10.0f;
        g.phase = 0.13f;
        float p_true = 0.0f, max_e = 0.0f, init_e = 0.0f, late_max = 0.0f;
        gait_observe(&g, p_true, &init_e, NULL);
        int steps = (int)(T / dt);
        for (int i = 0; i < steps; i++) {
            float e = wrap_pi((p_true - g.phase) * GAIT_TWO_PI);
            if (gait_track(&g, vx, 0.0f, e, dt) < 0) break;
            p_true = phase01(p_true + (vx / g.stride) * dt);
            float ae = fabsf(wrap_pi((p_true - g.phase) * GAIT_TWO_PI));
            if (ae > max_e) max_e = ae;
            if ((float)i * dt > 1.0f && ae > late_max) late_max = ae;
        }
        printf("const_max_err_deg=%.4f late_max_deg=%.5f slip_bound_m=%.6f slip_total_m=%.6f\n",
               max_e * 180.0f / GAIT_PI, late_max * 180.0f / GAIT_PI,
               slip_m(&g, max_e), g.slip_total);
        check("const_no_divergence", max_e <= fabsf(init_e) + 1e-6f);
        check("const_pulled_in", fabsf(g.phase_err) < 1e-4f);
        check("const_late_tight", late_max < 0.0005f * GAIT_TWO_PI);
        check("const_slip_total", g.slip_total <= slip_m(&g, init_e) + 1e-5f);
        check("const_freq", fabsf(g.freq - vx / g.stride) < 0.05f);
        check("const_resid", fabsf(g.speed_resid) < 0.05f);
        check("const_trackable", g.trackable == 1);
    }

    /* -- 4. velocity step with a lagged estimate: bounded, then re-settles -- */
    {
        Gait g;
        gait_init(&g, 1.0f, 10.0f);
        const float dt = 0.001f, v_old = 1.0f, v_new = 4.0f, tau = 0.05f;
        float p_true = 0.0f, v_meas = v_old, peak = 0.0f, settle = -1.0f;
        float slip_at_step = -1.0f, slip_max = 0.0f;
        for (float t = 0.0f; t < 6.0f; t += dt) {
            float v = (t < 1.0f) ? v_old : v_new;
            float e = wrap_pi((p_true - g.phase) * GAIT_TWO_PI);
            if (gait_track(&g, v_meas, 0.0f, e, dt) < 0) break;
            if (slip_at_step < 0.0f && t >= 1.0f) slip_at_step = g.slip_total;
            v_meas += (v - v_meas) * (dt / tau);
            p_true = phase01(p_true + (v / g.stride) * dt);
            if (t > 1.0f + 5.0f * tau) {                /* past the lag window */
                float ae = fabsf(wrap_pi((p_true - g.phase) * GAIT_TWO_PI));
                if (ae > peak) peak = ae;
                if (ae > 0.01f * GAIT_TWO_PI) settle = -1.0f;
                else if (settle < 0.0f) settle = t - 1.0f;
                if (g.slip_total - slip_at_step > slip_max)
                    slip_max = g.slip_total - slip_at_step;
            }
        }
        /* one lag constant of mismatch is unavoidable: (v_new - v_old) * tau */
        float lag_slip = (v_new - v_old) * tau * g.stride;
        printf("step_peak_deg=%.4f settle_s=%.4f slip_after_step_m=%.6f lag_slip_m=%.6f\n",
               peak * 180.0f / GAIT_PI, settle, slip_max, lag_slip);
        check("step_transient_bound", peak < 0.03f * GAIT_TWO_PI);
        check("step_settles", settle >= 0.0f && settle < 1.0f);
        check("step_slip_bounded", slip_max <= lag_slip + 0.02f * g.stride);
        check("step_freq", fabsf(g.freq - v_new / g.stride) < 0.2f);
    }

    /* -- 5. manual alignment matches setting the phase by hand -- */
    {
        Gait g;
        gait_init(&g, 1.0f, 10.0f);
        gait_track(&g, 1.0f, 0.0f, 0.0f, 0.02f);    /* give the tracker a cadence */
        float remote = 0.20f, e0 = 0.0f;
        gait_observe(&g, remote, &e0, NULL);
        printf("sync_pre_err_deg=%.3f\n", e0 * 180.0f / GAIT_PI);
        check("sync_pull", gait_sync(&g, remote) == 0);

        Gait manual;
        gait_init(&manual, 1.0f, 10.0f);
        manual.phase = remote;                       /* hand-aligned reference */
        float pe = 9.0f, slip = 9.0f;
        gait_observe(&g, remote, &pe, &slip);
        printf("sync_err_deg=%.5f slip_after_m=%.7f slip_total_m=%.7f\n",
               pe * 180.0f / GAIT_PI, slip, g.slip_total);
        check("sync_matches_manual", fabsf(g.phase - manual.phase) < 1e-6f);
        check("sync_aligns", fabsf(pe) < 1e-5f && slip < 1e-6f);
        check("sync_slip_accounted", g.slip_total <= slip_m(&g, e0) + 1e-5f);

        g.phase = 0.20f;
        check("sync_skip_rejected", gait_sync(&g, 0.70f) == -1 && g.phase == 0.20f);
        check("sync_too_far_rejected", gait_sync(&g, 0.90f) == -1);
        check("sync_nan_rejected", gait_sync(&g, NAN) == -1);
        check("sync_short_ok", gait_sync(&g, 0.22f) == 0 &&
                               fabsf(g.phase - 0.22f) < 1e-5f);
    }

    /* -- 6. footfall landmarks from displacement; compares against manual
     *       alignment. (a) time-aligned remote sample, 3% speed bias --
     *       residual bounded by bias/kp. (b) stale remote sample: the loop
     *       can only match the sample it is given, so the bound widens to
     *       v * sample_age metres of foot slip. -- */
    {
        Gait g;
        gait_init(&g, 1.0f, 10.0f);
        const float vx = 1.6f, dt = 0.001f, T = 12.0f;
        const float period = g.stride / vx;          /* one footfall per stride */
        const float v_meas = vx * 1.03f;             /* 3% fast estimate */
        float next_hit = period, max_hit = 0.0f, max_slip = 0.0f;
        int hits = 0;
        for (float t = 0.0f; t < T; t += dt) {
            float remote = phase01(t * vx / g.stride);          /* aligned sample */
            float e = wrap_pi((remote - g.phase) * GAIT_TWO_PI);
            if (gait_track(&g, v_meas, 0.0f, e, dt) < 0) break;
            while (t >= next_hit) {
                if (next_hit >= 1.0f) {                        /* after lock-in */
                    float ae = fabsf(wrap_pi(g.phase * GAIT_TWO_PI)); /* footfall = phase 0 */
                    if (ae > max_hit) max_hit = ae;
                    if (slip_m(&g, ae) > max_slip) max_slip = slip_m(&g, ae);
                }
                next_hit += period;
                hits++;
            }
        }
        printf("aligned_footfalls=%d max_foot_err_deg=%.4f foot_slip_m=%.6f\n",
               hits, max_hit * 180.0f / GAIT_PI, max_slip);
        check("aligned_footfalls_seen", hits >= 15);
        check("aligned_foot_phase_bound", max_hit < 0.01f * GAIT_TWO_PI);
        check("aligned_foot_slip_bound", max_slip < 0.01f);
    }
    {
        Gait g;
        gait_init(&g, 1.0f, 10.0f);
        const float vx = 1.6f, dt = 0.001f, T = 12.0f, age = 0.05f;
        const float period = g.stride / vx;
        float next_hit = period, max_hit = 0.0f;
        for (float t = 0.0f; t < T; t += dt) {
            float remote = phase01((t - age) * vx / g.stride);  /* stale sample */
            float e = wrap_pi((remote - g.phase) * GAIT_TWO_PI);
            if (gait_track(&g, vx, 0.0f, e, dt) < 0) break;
            while (t >= next_hit) {
                if (next_hit >= 1.0f) {
                    float ae = fabsf(wrap_pi(g.phase * GAIT_TWO_PI));
                    if (ae > max_hit) max_hit = ae;
                }
                next_hit += period;
            }
        }
        float latency_slip = vx * age;                /* m of ground travel per sample age */
        float got = max_hit * g.stride / GAIT_TWO_PI;
        printf("stale_foot_err_deg=%.4f latency_slip_m=%.6f measured_slip_m=%.6f\n",
               max_hit * 180.0f / GAIT_PI, latency_slip, got);
        check("stale_matches_latency", fabsf(got - latency_slip) < 0.01f);
        check("stale_slip_bound", got < latency_slip + 0.01f);
    }

    /* -- 7. start half a stride off: saturating pull shrinks the error -- */
    {
        Gait g;
        gait_init(&g, 1.0f, 10.0f);
        g.phase = 0.5f;
        const float vx = 0.4f;
        float p_true = 0.0f, late = 0.0f;
        for (float t = 0.0f; t < 5.0f; t += 0.01f) {
            float e = wrap_pi((p_true - g.phase) * GAIT_TWO_PI);
            if (gait_track(&g, vx, 0.0f, e, 0.01f) < 0) break;
            p_true = phase01(p_true + (vx / g.stride) * 0.01f);
            if (t > 2.0f) late = fabsf(wrap_pi((p_true - g.phase) * GAIT_TWO_PI));
        }
        printf("slow_late_err_deg=%.5f slip_total_m=%.6f freq=%.4f\n",
               late * 180.0f / GAIT_PI, g.slip_total, g.freq);
        check("slow_trackable", g.trackable == 1 && fabsf(g.freq - vx) < 0.05f);
        check("slow_converged", late < 0.02f * GAIT_TWO_PI);
        check("slow_slip_total_bound", g.slip_total <= 0.5f * g.stride + 1e-4f);
        check("slow_zero_rejected", gait_track(&g, 0.0f, 0.0f, 0.0f, 0.01f) == -1);
    }

    /* -- 8. any dt: correction never increases |error| (no overshoot) -- */
    {
        int ok = 1;
        for (float dt = 0.001f; dt <= 0.5f; dt *= 2.0f) {
            Gait g;
            gait_init(&g, 1.0f, 10.0f);
            g.phase = 0.30f;
            float e = wrap_pi((0.0f - g.phase) * GAIT_TWO_PI);
            float before = fabsf(wrap_pi((0.0f - g.phase) * GAIT_TWO_PI));
            gait_track(&g, 1.0f, 0.0f, e, dt);
            /* remove the feed-forward advance to isolate the correction */
            float after = fabsf(wrap_pi((0.0f - (g.phase - 1.0f * dt)) * GAIT_TWO_PI));
            if (after > before + 1e-5f) ok = 0;
        }
        check("dt_no_overshoot", ok);
    }

    /* -- 9. near-zero speed (below min_freq) rejected as zero cadence -- */
    {
        Gait g;
        gait_init(&g, 1.0f, 10.0f);
        check("near_zero_rejected", gait_track(&g, 1e-4f, 0.0f, 0.0f, 0.02f) == -1);
        check("near_zero_counted", g.rejects == 1 && g.phase == 0.0f);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
