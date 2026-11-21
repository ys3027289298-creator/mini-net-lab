
/* extrap: input extrapolation compensation with error-bounded stop. */
#include <stdio.h>
#include <string.h>
#include <math.h>

typedef struct {
    float x, v;       /* state at extrapolation start */
    float max_win;    /* hard cap on extension length (s), must be > 0 */
    float a_max;      /* accel bound of the real input (m/s^2), must be > 0 */
    float err_thresh; /* stop extending once the error bound exceeds this */
    float stop_win;   /* effective window: min(max_win, sqrt(2*thresh/a_max)) */
    float t;          /* elapsed extrapolation time */
    int active;       /* 0 once the window is exhausted: position held */
} Extrap;

static int extrap_init(Extrap* e, float x, float v,
                       float max_win, float a_max, float err_thresh) {
    if (max_win <= 0.0f) return -1;   /* zero/negative window rejected */
    if (a_max <= 0.0f) return -1;
    if (err_thresh < 0.0f) return -1;
    e->x = x; e->v = v;
    e->max_win = max_win;
    e->a_max = a_max;
    e->err_thresh = err_thresh;
    float w_thresh = sqrtf(2.0f * err_thresh / a_max);
    e->stop_win = max_win < w_thresh ? max_win : w_thresh;
    e->t = 0.0f;
    e->active = 1;
    return 0;
}

/* position error upper bound after extending w seconds: the real input may
   accelerate at up to a_max against the extrapolated constant velocity */
static float extrap_err_bound(const Extrap* e, float w) {
    return 0.5f * e->a_max * w * w;
}

static float extrap_pos(const Extrap* e) {
    float w = e->t < e->stop_win ? e->t : e->stop_win;
    return e->x + e->v * w;
}

/* advance extrapolation by dt; past stop_win the position is clamped at the
   stop point, so output is continuous (no jump) when extension ends */
static float extrap_step(Extrap* e, float dt) {
    if (dt > 0.0f) e->t += dt;
    if (e->t >= e->stop_win) e->active = 0;
    return extrap_pos(e);
}

/* compare against the real input: integrate a real trajectory driven by
   acceleration profile a_fn(t) (|a_fn| <= a_max) next to the extrapolated
   one, and sample both the measured error and its upper bound at n window
   lengths in (0, max_win]. returns the worst measured/bound ratio. */
static float extrap_curve(const Extrap* e, float (*a_fn)(float),
                          float* w_out, float* measured, float* bound, int n) {
    const float dt = 1e-4f;
    float rx = e->x, rv = e->v;   /* real trajectory */
    float worst = 0.0f;
    int sample = 1;
    int steps = (int)(e->max_win / dt + 0.5f);
    for (int k = 0; k <= steps; k++) {
        float t = k * dt;
        if (sample <= n && t >= e->max_win * sample / n) {
            float w = e->max_win * sample / n;
            float ex = e->x + e->v * (w < e->stop_win ? w : e->stop_win);
            w_out[sample - 1] = w;
            measured[sample - 1] = fabsf(ex - rx);
            bound[sample - 1] = extrap_err_bound(e, w);
            float ratio = bound[sample - 1] > 0.0f
                        ? measured[sample - 1] / bound[sample - 1] : 0.0f;
            if (ratio > worst) worst = ratio;
            sample++;
        }
        float a = a_fn(t);
        rv += a * dt;
        rx += rv * dt;
    }
    return worst;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static float a_const(float t) { (void)t; return 4.0f; }              /* worst case */
static float a_sine(float t) { return 4.0f * sinf(t * 6.2831853f); } /* |a| <= 4 */

int main(void) {
    Extrap e;
    check("init_ok", extrap_init(&e, 0.0f, 2.0f, 2.0f, 4.0f, 0.5f) == 0);
    check("zero_window_rejected", extrap_init(&e, 0.0f, 2.0f, 0.0f, 4.0f, 0.5f) == -1);
    check("neg_window_rejected", extrap_init(&e, 0.0f, 2.0f, -0.1f, 4.0f, 0.5f) == -1);
    check("zero_amax_rejected", extrap_init(&e, 0.0f, 2.0f, 1.0f, 0.0f, 0.5f) == -1);

    /* threshold stop: sqrt(2*0.5/4) = 0.5s, below the 2.0s hard cap */
    check("init_thresh", extrap_init(&e, 0.0f, 2.0f, 2.0f, 4.0f, 0.5f) == 0);
    check("stop_at_threshold", fabsf(e.stop_win - 0.5f) < 1e-6f);

    /* hard cap wins when it is tighter than the threshold window */
    Extrap cap;
    extrap_init(&cap, 0.0f, 2.0f, 0.1f, 4.0f, 0.5f);
    check("stop_at_cap", fabsf(cap.stop_win - 0.1f) < 1e-6f);

    /* no jump at the stop point: position is clamped, then held */
    float before = extrap_step(&e, 0.49f);
    float at_stop = extrap_step(&e, 0.02f);          /* crosses 0.5s */
    float after = extrap_step(&e, 1.0f);             /* well past stop */
    check("continuous_at_stop", fabsf(at_stop - (0.0f + 2.0f * 0.5f)) < 1e-5f);
    check("no_jump", fabsf(at_stop - before) < 2.0f * 0.02f + 1e-5f);
    check("held_after_stop", fabsf(after - at_stop) < 1e-6f);
    check("inactive_after_stop", e.active == 0);

    /* extension length vs error curve, compared against real input */
    float w[8], meas[8], bnd[8];
    Extrap c;
    extrap_init(&c, 0.0f, 2.0f, 0.5f, 4.0f, 1e9f);   /* no threshold: full window */
    float worst = extrap_curve(&c, a_const, w, meas, bnd, 8);
    printf("curve_worstcase:\n");
    for (int i = 0; i < 8; i++)
        printf("  w=%.4f measured=%.4f bound=%.4f\n", w[i], meas[i], bnd[i]);
    check("curve_measured_within_bound", worst <= 1.02f);  /* 2% integration slack */
    check("curve_bound_monotonic", bnd[7] > bnd[0]);

    float w2[8], meas2[8], bnd2[8];
    extrap_init(&c, 0.0f, 2.0f, 0.5f, 4.0f, 1e9f);
    float worst2 = extrap_curve(&c, a_sine, w2, meas2, bnd2, 8);
    printf("curve_sine_worst_ratio=%.3f\n", worst2);
    check("sine_within_bound", worst2 <= 1.02f);

    /* real input identical to extrapolation: zero error */
    Extrap z;
    extrap_init(&z, 1.0f, 3.0f, 1.0f, 4.0f, 0.5f);
    float p = extrap_step(&z, 0.3f);
    check("exact_when_input_matches", fabsf(p - (1.0f + 3.0f * 0.3f)) < 1e-6f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
