
/* reconcile: client prediction, input-history replay, and authoritative
 * correction with a snap-or-smooth policy.
 *
 * The client advances one tick per input.  Every tick's input and predicted
 * body are retained in a replay window.  When an uncompressed authoritative
 * state arrives for tick A, the client
 *
 *   1. reports RECON_WINDOW instead of silently dropping the frame if A is
 *      older than the replay window (state at A can no longer be recovered),
 *   2. compares its prediction at A with the authority,
 *   3. rolls back to the authoritative body at A and replays inputs A+1..now,
 *   4. snaps immediately when the resulting correction is larger than the
 *      snap threshold, otherwise spreads the same correction over several
 *      ticks by interpolating a render offset back to zero.
 *
 * The self-test also runs the observer against a full-rate, uncompressed
 * authoritative sequence and reports the maximum position error and the
 * number of rollbacks.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define RECON_HIST 32
#define RECON_EPS 1e-4f

typedef struct { float ax, ay; } Input;      /* acceleration input */
typedef struct { float x, y, vx, vy; } Body;

typedef struct {
    unsigned tick;
    Input in;        /* input that produced pred at this tick */
    Body pred;       /* best prediction for this tick */
} Frame;

typedef enum {
    RECON_MATCH  =  0,   /* authority agreed, no rollback needed */
    RECON_SMOOTH =  1,   /* small correction spread over several ticks */
    RECON_SNAP   =  2,   /* large correction applied immediately */
    RECON_STALE  = -1,   /* old/duplicate authoritative tick */
    RECON_FUTURE = -2,   /* authority is ahead of the local simulation */
    RECON_WINDOW = -3    /* replay window does not reach the authoritative tick */
} ReconStatus;

typedef struct {
    Frame hist[RECON_HIST];
    int head;                 /* next write slot */
    int count;
    unsigned cur;             /* newest simulated tick */
    unsigned last_auth;
    int have_auth;

    Body body;                /* simulated state at cur */
    float dt;
    float snap_threshold;
    int smooth_ticks;
    float off_x, off_y;       /* visual offset currently applied */
    float smooth_x, smooth_y; /* offset at the start of the interpolation */
    int smooth_left, smooth_total;

    /* observable counters */
    int auth_seen;
    int replays;              /* authoritative frames that rolled back + replayed */
    int rollbacks;            /* replays whose result differed from the prediction */
    int snaps;
    int smooths;
    int matches;
    int stale;
    int future;
    int window_short;

    float max_auth_err;       /* max |pred(A) - auth(A)| */
    float max_correction;     /* max |pred(now) - replayed(now)| */
    float max_pos_err;        /* rendered position error vs uncompressed truth */
    unsigned max_err_tick;
    float max_sim_err;        /* simulated position error vs truth */
    float max_step;           /* largest per-tick rendered movement (incl. snaps) */
    float max_smooth_jump;    /* visible jump at a SMOOTH correction (must be ~0) */
    float max_snap_jump;      /* visible jump at a SNAP correction */
    int have_last_render;
    float last_rx, last_ry;
} Recon;

static void recon_init(Recon* r, float dt, float snap_threshold,
                       int smooth_ticks, Body initial) {
    memset(r, 0, sizeof(*r));
    r->dt = dt;
    r->snap_threshold = snap_threshold;
    r->smooth_ticks = smooth_ticks < 0 ? 0 : smooth_ticks;
    r->body = initial;
    r->cur = 0;
    /* tick 0 is the initial state; its replay input is zero */
    r->hist[0].tick = 0;
    r->hist[0].in = (Input){ 0.0f, 0.0f };
    r->hist[0].pred = initial;
    r->head = 1 % RECON_HIST;
    r->count = 1;
}

static Frame* recon_frame(Recon* r, unsigned tick) {
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - 1 - i + RECON_HIST * 2) % RECON_HIST;
        if (r->hist[idx].tick == tick) return &r->hist[idx];
    }
    return NULL;
}

static void recon_store(Recon* r, unsigned tick, Input in, Body pred) {
    r->hist[r->head].tick = tick;
    r->hist[r->head].in = in;
    r->hist[r->head].pred = pred;
    r->head = (r->head + 1) % RECON_HIST;
    if (r->count < RECON_HIST) r->count++;
}

static Body recon_step(Body b, Input in, float dt) {
    b.vx += in.ax * dt;
    b.vy += in.ay * dt;
    b.x  += b.vx * dt;
    b.y  += b.vy * dt;
    return b;
}

/* oldest tick still available for replay */
static unsigned recon_oldest(const Recon* r) {
    if (r->count == 0) return r->cur;
    return r->cur - (unsigned)(r->count - 1);
}

static void recon_render(const Recon* r, float* x, float* y);

/* advance one tick with a locally sampled input */
static void recon_tick(Recon* r, Input in) {
    /* decay the smoothing offset once per tick */
    if (r->smooth_left > 0) {
        r->smooth_left--;
        if (r->smooth_left == 0) {
            r->off_x = r->off_y = 0.0f;
        } else {
            float f = (float)r->smooth_left / (float)r->smooth_total;
            r->off_x = r->smooth_x * f;
            r->off_y = r->smooth_y * f;
        }
    }
    r->cur++;
    r->body = recon_step(r->body, in, r->dt);
    recon_store(r, r->cur, in, r->body);
}

/* Apply an uncompressed authoritative body for tick `tick`. */
static int recon_auth(Recon* r, unsigned tick, Body auth) {
    r->auth_seen++;

    if (tick > r->cur) { r->future++; return RECON_FUTURE; }
    if (r->have_auth && tick <= r->last_auth) { r->stale++; return RECON_STALE; }

    Frame* f = recon_frame(r, tick);
    if (f == NULL) {
        /* The authoritative tick fell out of the replay window.  Report it;
         * do not silently discard the frame or advance the local tick. */
        r->window_short++;
        return RECON_WINDOW;
    }

    float ex = f->pred.x - auth.x;
    float ey = f->pred.y - auth.y;
    float err_auth = sqrtf(ex * ex + ey * ey);

    /* Rendered position before the correction, for measuring visible jumps. */
    float rx0, ry0;
    recon_render(r, &rx0, &ry0);

    /* Roll back to the authoritative state and replay every input after it. */
    Body replayed = auth;
    f->pred = auth;
    for (unsigned t = tick + 1; t <= r->cur; t++) {
        Frame* g = recon_frame(r, t);
        if (g == NULL) {          /* contiguous history, should not happen */
            r->window_short++;
            return RECON_WINDOW;
        }
        replayed = recon_step(replayed, g->in, r->dt);
        g->pred = replayed;
    }
    r->replays++;

    float cx = r->body.x - replayed.x;
    float cy = r->body.y - replayed.y;
    float correction = sqrtf(cx * cx + cy * cy);

    if (err_auth > r->max_auth_err) r->max_auth_err = err_auth;
    if (correction > r->max_correction) r->max_correction = correction;

    r->have_auth = 1;
    r->last_auth = tick;

    if (correction <= RECON_EPS) {
        /* Prediction already agrees; keep the replay result for velocity. */
        r->matches++;
        r->body = replayed;
        return RECON_MATCH;
    }

    r->rollbacks++;

    float jump, rx1, ry1;
    if (correction > r->snap_threshold || r->smooth_ticks <= 0) {
        r->snaps++;
        r->body = replayed;
        r->off_x = r->off_y = 0.0f;
        r->smooth_x = r->smooth_y = 0.0f;
        r->smooth_left = r->smooth_total = 0;
        recon_render(r, &rx1, &ry1);
        jump = sqrtf((rx1 - rx0) * (rx1 - rx0) + (ry1 - ry0) * (ry1 - ry0));
        if (jump > r->max_snap_jump) r->max_snap_jump = jump;
        return RECON_SNAP;
    }

    /* Keep the currently rendered position, then interpolate the offset that
     * separates it from the corrected body back to zero. */
    r->smooths++;
    r->body = replayed;
    r->off_x = rx0 - replayed.x;
    r->off_y = ry0 - replayed.y;
    r->smooth_x = r->off_x;
    r->smooth_y = r->off_y;
    r->smooth_total = r->smooth_ticks;
    r->smooth_left = r->smooth_ticks;
    recon_render(r, &rx1, &ry1);
    jump = sqrtf((rx1 - rx0) * (rx1 - rx0) + (ry1 - ry0) * (ry1 - ry0));
    if (jump > r->max_smooth_jump) r->max_smooth_jump = jump;
    return RECON_SMOOTH;
}

static void recon_render(const Recon* r, float* x, float* y) {
    *x = r->body.x + r->off_x;
    *y = r->body.y + r->off_y;
}

/* Compare the client against an uncompressed, full-rate authoritative state
 * for the current tick.  Returns the rendered position error. */
static float recon_compare_truth(Recon* r, unsigned tick, Body truth) {
    float rx, ry;
    recon_render(r, &rx, &ry);
    float dx = rx - truth.x, dy = ry - truth.y;
    float err = sqrtf(dx * dx + dy * dy);

    float sx = r->body.x - truth.x, sy = r->body.y - truth.y;
    float sim_err = sqrtf(sx * sx + sy * sy);

    if (err > r->max_pos_err) {
        r->max_pos_err = err;
        r->max_err_tick = tick;
    }
    if (sim_err > r->max_sim_err) r->max_sim_err = sim_err;

    if (r->have_last_render) {
        float jx = rx - r->last_rx, jy = ry - r->last_ry;
        float step = sqrtf(jx * jx + jy * jy);
        if (step > r->max_step) r->max_step = step;
    }
    r->last_rx = rx;
    r->last_ry = ry;
    r->have_last_render = 1;
    return err;
}

static void recon_report(const Recon* r) {
    printf("auth_seen=%d\n", r->auth_seen);
    printf("replays=%d\n", r->replays);
    printf("rollbacks=%d\n", r->rollbacks);
    printf("snaps=%d\n", r->snaps);
    printf("smooths=%d\n", r->smooths);
    printf("matches=%d\n", r->matches);
    printf("window_short=%d\n", r->window_short);
    printf("stale=%d\n", r->stale);
    printf("future=%d\n", r->future);
    printf("max_auth_err=%.6f\n", (double)r->max_auth_err);
    printf("max_correction=%.6f\n", (double)r->max_correction);
    printf("max_pos_err=%.6f\n", (double)r->max_pos_err);
    printf("max_err_tick=%u\n", r->max_err_tick);
    printf("max_sim_err=%.6f\n", (double)r->max_sim_err);
    printf("max_step=%.6f\n", (double)r->max_step);
    printf("max_smooth_jump=%.6f\n", (double)r->max_smooth_jump);
    printf("max_snap_jump=%.6f\n", (double)r->max_snap_jump);
}

/* Every authoritative frame must be classified exactly once. */
static int recon_accounted(const Recon* r) {
    return r->auth_seen == r->matches + r->rollbacks + r->window_short +
                           r->stale + r->future;
}

/* ------------------------------------------------------------------ */

#define DT      (1.0f / 60.0f)
#define TICKS   120
#define SAMPLE  4
#define DELAY   5

static Input test_input(unsigned t) {
    Input in;
    in.ax = 0.60f * sinf((float)t * 0.11f);
    in.ay = 0.50f * cosf((float)t * 0.07f);
    return in;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Full-rate authoritative sequence.  The server injects two events that the
 * client cannot derive from its inputs: a small velocity kick and a large
 * position teleport. */
static void build_truth(Body* truth) {
    truth[0] = (Body){ 0.0f, 0.0f, 0.0f, 0.0f };
    for (unsigned t = 1; t <= TICKS; t++) {
        truth[t] = recon_step(truth[t - 1], test_input(t), DT);
        if (t == 32) truth[t].vx += 1.5f;
        if (t == 64) truth[t].x += 2.0f;
    }
}

/* Client follows the same inputs and receives sampled authority with delay. */
static void test_prediction_and_correction(void) {
    Body truth[TICKS + 1];
    build_truth(truth);

    Recon r;
    recon_init(&r, DT, 0.50f, 8, truth[0]);

    for (unsigned t = 1; t <= TICKS; t++) {
        recon_tick(&r, test_input(t));

        /* authoritative packet for tick A arrives DELAY ticks later */
        if (t >= DELAY) {
            unsigned a = t - DELAY;
            if (a % SAMPLE == 0) recon_auth(&r, a, truth[a]);
        }
        recon_compare_truth(&r, t, truth[t]);
    }

    recon_report(&r);

    check("auth_frames_accounted", recon_accounted(&r));
    check("replays_classified", r.replays == r.matches + r.rollbacks);
    check("rollbacks_seen", r.rollbacks >= 2);
    check("snap_used", r.snaps >= 1);
    check("smooth_used", r.smooths >= 1);
    check("smooth_no_visible_jump", r.max_smooth_jump < 1e-4f);
    check("snap_absorbs_jump", r.max_snap_jump > 1.0f);
    check("no_window_loss", r.window_short == 0);
    check("error_bounded", r.max_pos_err < 3.0f);
    check("correction_matches_error", r.max_auth_err > 1.0f);

    printf("position_error_bound=%.6f\n", (double)r.max_pos_err);
    printf("rollback_count=%d\n", r.rollbacks);
}

/* With no server-only divergence, replay must reproduce the reference
 * exactly and no rollback must be reported. */
static void test_replay_is_exact(void) {
    Body ref[TICKS + 1];
    ref[0] = (Body){ 0.0f, 0.0f, 0.0f, 0.0f };

    Recon r;
    recon_init(&r, DT, 0.50f, 8, ref[0]);

    int saw_window = 0, saw_snap = 0;
    for (unsigned t = 1; t <= TICKS; t++) {
        Input in = test_input(t);
        recon_tick(&r, in);
        ref[t] = recon_step(ref[t - 1], in, DT);

        if (t >= DELAY) {
            unsigned a = t - DELAY;
            if (a % SAMPLE == 0) {
                int st = recon_auth(&r, a, ref[a]);
                if (st == RECON_WINDOW) saw_window = 1;
                if (st == RECON_SNAP) saw_snap = 1;
            }
        }
        recon_compare_truth(&r, t, ref[t]);
    }

    check("exact_no_rollback", r.rollbacks == 0);
    check("exact_no_snap", !saw_snap);
    check("exact_no_window", !saw_window);
    check("exact_zero_error", r.max_pos_err < 1e-5f);
    check("exact_accounted", recon_accounted(&r));
}

/* A frame older than the replay window must be reported and must not damage
 * or truncate the running simulation. */
static void test_window_short_reported(void) {
    Body ref[128];
    ref[0] = (Body){ 0.0f, 0.0f, 0.0f, 0.0f };

    Recon w;
    recon_init(&w, DT, 0.50f, 4, ref[0]);
    for (unsigned t = 1; t <= 60; t++) {
        Input in = test_input(t);
        recon_tick(&w, in);
        ref[t] = recon_step(ref[t - 1], in, DT);
    }

    unsigned oldest = recon_oldest(&w);
    check("window_is_bounded", oldest > 5);
    unsigned cur_before = w.cur;
    Body body_before = w.body;

    int st = recon_auth(&w, 5, ref[5]);   /* far older than the window */
    check("window_reported", st == RECON_WINDOW);
    check("window_counted", w.window_short == 1);
    check("window_no_rollback", w.rollbacks == 0);
    check("window_no_replay", w.replays == 0);
    check("window_no_tick_drop", w.cur == cur_before);
    check("window_state_untouched",
          w.body.x == body_before.x && w.body.y == body_before.y);

    /* The rejected frame must leave the simulation fully usable: keep running
     * and the client must stay exact. */
    int continued_exact = 1;
    for (unsigned t = 61; t <= 70; t++) {
        Input in = test_input(t);
        recon_tick(&w, in);
        ref[t] = recon_step(ref[t - 1], in, DT);
        if (recon_compare_truth(&w, t, ref[t]) > 1e-5f) continued_exact = 0;
    }
    check("window_continues_exact", continued_exact);

    /* A tick still inside the window is applied normally. */
    check("window_valid_tick", recon_auth(&w, 69, ref[69]) == RECON_MATCH);

    /* Duplicate and future frames are classified, not lost. */
    check("duplicate_stale", recon_auth(&w, 69, ref[69]) == RECON_STALE);
    check("future_reported", recon_auth(&w, w.cur + 1, ref[70]) == RECON_FUTURE);
    check("window_accounted", recon_accounted(&w));
}

/* A single bounded, position-only divergence (velocity is unchanged) is the
 * case with a clean analytic bound: replay reproduces the authority's future
 * exactly, so the rendered error never exceeds the divergence itself and the
 * smooth offset decays monotonically to zero without a visible jump. */
static void test_bounded_position_error(void) {
    enum { T = 90, A_TICK = 40, A_DELAY = 5 };
    const float amp = 0.05f;

    Body truth[T + 1];
    truth[0] = (Body){ 0.0f, 0.0f, 0.0f, 0.0f };
    for (unsigned t = 1; t <= T; t++) {
        truth[t] = recon_step(truth[t - 1], test_input(t), DT);
        if (t == A_TICK) truth[t].x += amp;   /* server-only nudge, no velocity */
    }

    Recon r;
    recon_init(&r, DT, 0.50f, 8, truth[0]);

    for (unsigned t = 1; t <= T; t++) {
        recon_tick(&r, test_input(t));
        if (t == A_TICK + A_DELAY) recon_auth(&r, A_TICK, truth[A_TICK]);
        recon_compare_truth(&r, t, truth[t]);
    }

    printf("bounded_position_error_bound=%.6f\n", (double)r.max_pos_err);
    printf("bounded_rollback_count=%d\n", r.rollbacks);

    check("bounded_one_rollback", r.rollbacks == 1);
    check("bounded_smooth_not_snap", r.snaps == 0 && r.smooths == 1);
    check("bounded_error_bound", r.max_pos_err <= amp + 1e-5f);
    check("bounded_error_above_amp", r.max_pos_err >= amp - 1e-5f);
    check("bounded_no_jump", r.max_smooth_jump < 1e-4f);
    check("bounded_settles", recon_compare_truth(&r, T, truth[T]) < 1e-5f);
    check("bounded_accounted", recon_accounted(&r));
}

int main(void) {
    test_prediction_and_correction();
    test_replay_is_exact();
    test_window_short_reported();
    test_bounded_position_error();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
