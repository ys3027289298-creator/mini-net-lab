/* resample: variable-rate render-time resampling of ticked snapshots.
 *
 * Snapshots carry position and optional velocity. A render time is mapped onto
 * the snapshot timeline and the bracketing interval is evaluated with a cubic
 * Hermite basis; a missing endpoint velocity is replaced by the secant (linear
 * fallback, flagged). Times outside the timeline extrapolate linearly along the
 * boundary snapshot's velocity (or hold, if it has none), and only survive
 * within a bounded fraction of the edge interval.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define RESAMPLE_MAX_SNAPSHOTS 64
#define RESAMPLE_MAX_STEP 1.0      /* seconds; larger snapshot gaps are rejected */
#define RESAMPLE_EXTRAP_LIMIT 0.25 /* fraction of the edge interval */

enum {
    RESAMPLE_OK = 1,
    RESAMPLE_EXTRAPOLATED = 2,

    RESAMPLE_ERR_NULL = -1,
    RESAMPLE_ERR_RATE = -2,   /* tick rate unset/not positive, or gap too large */
    RESAMPLE_ERR_TIME = -3,   /* non-finite time or value */
    RESAMPLE_ERR_ORDER = -4,  /* tick/time not strictly increasing */
    RESAMPLE_ERR_EMPTY = -5,
    RESAMPLE_ERR_FAR = -6,    /* render time too far outside the timeline */
};

enum {
    RESAMPLE_F_LINEAR = 1u, /* an interval endpoint lacked velocity */
    RESAMPLE_F_EXTRAP = 2u, /* time was outside the snapshot timeline */
    RESAMPLE_F_HOLD = 4u,   /* no boundary velocity to extrapolate with */
};

typedef struct {
    unsigned tick;
    double t;    /* tick / tick_rate */
    float x, y;
    int has_vel;
    float vx, vy;
} ResampleSnap;

typedef struct {
    ResampleSnap s[RESAMPLE_MAX_SNAPSHOTS];
    int n;
    double tick_rate;
    int rate_set;
    unsigned last_tick;
    /* running maxima over every interval ever stored, so the error bounds stay
     * valid after eviction/pruning (they may only grow) */
    double dt_max;
    double dev_max;
    float vmax;
} ResampleTrack;

typedef struct {
    double t;          /* render-time coordinate of the result */
    double t0, t1;     /* bracketing snapshot times (edge interval when extrapolating) */
    double u;          /* (t - t0) / (t1 - t0); outside [0,1] means extrapolated */
    double dt;         /* t1 - t0 */
    float x, y;
    unsigned flags;
} Resampled;

static void drop_front(ResampleTrack* tr, int k) {
    if (k <= 0) return;
    if (k >= tr->n) { tr->n = 0; return; }
    memmove(&tr->s[0], &tr->s[k], sizeof(ResampleSnap) * (size_t)(tr->n - k));
    tr->n -= k;
}

static int tick_to_time(const ResampleTrack* tr, unsigned tick, double* out) {
    if (!tr->rate_set || !isfinite(tr->tick_rate) || tr->tick_rate <= 0.0)
        return RESAMPLE_ERR_RATE;
    double t = (double)tick / tr->tick_rate;
    if (!isfinite(t)) return RESAMPLE_ERR_TIME;
    *out = t;
    return RESAMPLE_OK;
}

void resample_init(ResampleTrack* tr, double tick_rate) {
    if (tr == NULL) return;
    memset(tr, 0, sizeof(*tr));
    tr->tick_rate = tick_rate;
    tr->rate_set = isfinite(tick_rate) && tick_rate > 0.0;
}

/* Set a new snapshot rate. Existing snapshots keep their absolute times, so
 * the next push must still land strictly after the last stored time. */
int resample_set_rate(ResampleTrack* tr, double tick_rate) {
    if (tr == NULL) return RESAMPLE_ERR_NULL;
    if (!isfinite(tick_rate) || tick_rate <= 0.0) return RESAMPLE_ERR_RATE;
    tr->tick_rate = tick_rate;
    tr->rate_set = 1;
    return RESAMPLE_OK;
}

int resample_push(ResampleTrack* tr, unsigned tick, float x, float y,
                  int has_vel, float vx, float vy) {
    if (tr == NULL) return RESAMPLE_ERR_NULL;
    double t = 0.0;
    int rc = tick_to_time(tr, tick, &t);
    if (rc != RESAMPLE_OK) return rc;
    if (!isfinite(x) || !isfinite(y)) return RESAMPLE_ERR_TIME;
    if (has_vel && (!isfinite(vx) || !isfinite(vy))) return RESAMPLE_ERR_TIME;

    if (tr->n > 0) {
        const ResampleSnap* last = &tr->s[tr->n - 1];
        if (tick <= last->tick) return RESAMPLE_ERR_ORDER;
        if (!(t > last->t)) return RESAMPLE_ERR_ORDER; /* rate change moved time backwards */
        if (t - last->t > RESAMPLE_MAX_STEP) return RESAMPLE_ERR_RATE;
    }

    if (tr->n > 0) {
        const ResampleSnap* prev = &tr->s[tr->n - 1];
        double dt = t - prev->t;
        float v = sqrtf(vx * vx + vy * vy);
        double bx = (double)x - prev->x, by = (double)y - prev->y;
        double r0x = (prev->has_vel ? dt * prev->vx : 0.0) - bx;
        double r0y = (prev->has_vel ? dt * prev->vy : 0.0) - by;
        double r1x = (has_vel ? dt * vx : 0.0) - bx;
        double r1y = (has_vel ? dt * vy : 0.0) - by;
        if (dt > tr->dt_max) tr->dt_max = dt;
        if (v > tr->vmax) tr->vmax = v;
        double dev = hypot(r0x, r0y) + hypot(r1x, r1y);
        if (dev > tr->dev_max) tr->dev_max = dev;
    }

    if (tr->n == RESAMPLE_MAX_SNAPSHOTS) drop_front(tr, 1);
    ResampleSnap* sn = &tr->s[tr->n++];
    sn->tick = tick;
    sn->t = t;
    sn->x = x;
    sn->y = y;
    sn->has_vel = has_vel ? 1 : 0;
    sn->vx = has_vel ? vx : 0.0f;
    sn->vy = has_vel ? vy : 0.0f;
    tr->last_tick = tick;
    return RESAMPLE_OK;
}

/* Drop old snapshots while keeping a bracket that contains `now`. */
int resample_prune(ResampleTrack* tr, double now) {
    if (tr == NULL) return RESAMPLE_ERR_NULL;
    if (!isfinite(now)) return RESAMPLE_ERR_TIME;
    int dropped = 0;
    while (tr->n >= 3 && tr->s[1].t < now) {
        drop_front(tr, 1);
        dropped++;
    }
    return dropped;
}

/* Cubic Hermite. Tangent terms are dp/du, so a missing endpoint velocity is
 * replaced by the secant (which makes that side exactly linear). */
static void hermite_eval(const ResampleSnap* a, const ResampleSnap* b, double u,
                         Resampled* out) {
    double dt = b->t - a->t;
    double x0 = a->x, y0 = a->y, x1 = b->x, y1 = b->y;
    double bx = x1 - x0, by = y1 - y0;
    double q0x = a->has_vel ? dt * a->vx : bx;
    double q0y = a->has_vel ? dt * a->vy : by;
    double q1x = b->has_vel ? dt * b->vx : bx;
    double q1y = b->has_vel ? dt * b->vy : by;
    double u2 = u * u, u3 = u2 * u;
    double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    double h10 = u3 - 2.0 * u2 + u;
    double h01 = -2.0 * u3 + 3.0 * u2;
    double h11 = u3 - u2;

    out->t0 = a->t;
    out->t1 = b->t;
    out->dt = dt;
    out->u = u;
    out->t = a->t + u * dt;
    out->x = (float)(h00 * x0 + h10 * q0x + h01 * x1 + h11 * q1x);
    out->y = (float)(h00 * y0 + h10 * q0y + h01 * y1 + h11 * q1y);
    out->flags = (a->has_vel && b->has_vel) ? 0u : RESAMPLE_F_LINEAR;
    if (u < 0.0 || u > 1.0) out->flags |= RESAMPLE_F_EXTRAP;
}

static void set_exact(const ResampleSnap* sn, Resampled* out) {
    memset(out, 0, sizeof(*out));
    out->t = out->t0 = out->t1 = sn->t;
    out->x = sn->x;
    out->y = sn->y;
}

/* Outside the timeline: linear extension along the edge snapshot's velocity. */
static int eval_edge(const ResampleTrack* tr, double now, int at_end, Resampled* out) {
    int n = tr->n;
    const ResampleSnap* edge = at_end ? &tr->s[n - 1] : &tr->s[0];
    const ResampleSnap* nbr = at_end ? &tr->s[n - 2] : &tr->s[1];
    if (n == 1) {
        /* no neighbour, so no interval to scale rejection by */
        if (now != edge->t) return RESAMPLE_ERR_FAR;
        set_exact(edge, out);
        return RESAMPLE_OK;
    }
    double dt = fabs(nbr->t - edge->t);
    if (fabs(now - edge->t) > RESAMPLE_EXTRAP_LIMIT * dt) return RESAMPLE_ERR_FAR;

    /* report the edge interval itself (forwards) and its extrapolated u, so a
     * right-edge sample still reads as u > 1 on the last stored interval */
    double a = at_end ? edge->t - dt : edge->t;
    memset(out, 0, sizeof(*out));
    out->t = now;
    out->t0 = a;
    out->t1 = a + dt;
    out->dt = dt;
    out->u = (now - a) / dt;
    out->flags = RESAMPLE_F_EXTRAP;
    if (edge->has_vel) {
        double lead = now - edge->t;
        out->x = (float)(edge->x + lead * edge->vx);
        out->y = (float)(edge->y + lead * edge->vy);
    } else {
        out->x = edge->x;
        out->y = edge->y;
        out->flags |= RESAMPLE_F_HOLD;
    }
    return RESAMPLE_EXTRAPOLATED;
}

int resample_sample(const ResampleTrack* tr, double now, Resampled* out) {
    if (tr == NULL || out == NULL) return RESAMPLE_ERR_NULL;
    if (!isfinite(now)) return RESAMPLE_ERR_TIME;
    if (!tr->rate_set || !isfinite(tr->tick_rate) || tr->tick_rate <= 0.0)
        return RESAMPLE_ERR_RATE;
    if (tr->n == 0) return RESAMPLE_ERR_EMPTY;

    int first = 0, last = tr->n - 1;
    if (now < tr->s[first].t) return eval_edge(tr, now, 0, out);
    if (now > tr->s[last].t) return eval_edge(tr, now, 1, out);
    if (now == tr->s[first].t) { set_exact(&tr->s[first], out); return RESAMPLE_OK; }
    if (now == tr->s[last].t) { set_exact(&tr->s[last], out); return RESAMPLE_OK; }

    for (int i = 0; i < last; i++) {
        if (tr->s[i].t <= now && now < tr->s[i + 1].t) {
            double dt = tr->s[i + 1].t - tr->s[i].t;
            hermite_eval(&tr->s[i], &tr->s[i + 1], (now - tr->s[i].t) / dt, out);
            return RESAMPLE_OK;
        }
    }
    /* sorted times guarantee a bracket; reaching here means the invariant broke */
    return RESAMPLE_ERR_ORDER;
}

/* Conservative bounds, from running maxima so eviction never shrinks them:
 *   pos/ang interpolation error  <= dt_max * vmax / 2
 *     (already covers the linear fallback at the sampled speeds; exact for
 *      constant velocity, curvature enters as H^2*accel/8)
 *   position overshoot past the interval chord  <= (4/27) * (|r0| + |r1|),
 *     r = dt*v - chord, maximized over stored intervals. The Hermite-vs-chord
 *     difference is u*(1-u)*((1-u)*r0 - u*r1); each basis weight is bounded by
 *     4/27, so this is a safe (if not tight) constant. A missing velocity makes
 *     that side's r zero, i.e. the secant tangent, so the same form covers the
 *     linear fallback.
 * `ang_bound` uses the same vmax: push angles with their angular velocity and
 * read it there. Pass NULL for outputs you do not need. */
int resample_error_bounds(const ResampleTrack* tr, double* max_dt,
                          double* pos_bound, double* over_bound,
                          double* ang_bound) {
    if (tr == NULL) return RESAMPLE_ERR_NULL;
    double dt_max = tr->dt_max, vmax = tr->vmax, dev_max = tr->dev_max;
    double b = 0.5 * dt_max * vmax;
    if (!isfinite(b) || !isfinite(dev_max)) return RESAMPLE_ERR_RATE;
    if (max_dt) *max_dt = dt_max;
    if (pos_bound) *pos_bound = b;
    if (ang_bound) *ang_bound = b;
    if (over_bound) *over_bound = (4.0 / 27.0) * dev_max;
    return tr->n == 0 ? RESAMPLE_ERR_EMPTY : RESAMPLE_OK;
}

/* ------------------------------------------------------------------ tests */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static double traj_x(double t) { return 3.0 * cos(0.5 * t); }
static double traj_y(double t) { return 1.5 * sin(0.8 * t); }
static double traj_vx(double t) { return -1.5 * sin(0.5 * t); }
static double traj_vy(double t) { return 1.2 * cos(0.8 * t); }
static double traj_ang(double t) { return 0.7 * t + 0.3 * sin(t); }
static double traj_omg(double t) { return 0.7 + 0.3 * cos(t); }

#define RENDER_MAX 1024
static double g_times[RENDER_MAX];
static double g_bounds[32];
static int g_ntimes = 0, g_nbounds = 0;

/* strictly increasing render times; abrupt rate changes land exactly on g_bounds */
static void build_render_times(const double* rates, const double* durations, int nrates,
                               double t0) {
    double t = t0;
    g_ntimes = 0;
    g_nbounds = 0;
    g_times[g_ntimes++] = t;
    for (int r = 0; r < nrates; r++) {
        int steps = (int)lround(durations[r] * rates[r]);
        if (steps < 1) steps = 1;
        for (int k = 1; k <= steps; k++) {
            t += 1.0 / rates[r];
            if (g_ntimes < RENDER_MAX) g_times[g_ntimes++] = t;
        }
        if (g_nbounds < 32) g_bounds[g_nbounds++] = t;
    }
}

static void fill_track(ResampleTrack* tr, double rate, double t_end, int vel_every) {
    resample_init(tr, rate);
    unsigned n = (unsigned)lround(t_end * rate);
    for (unsigned i = 0; i <= n; i++) {
        double t = (double)i / rate;
        int has = vel_every <= 0 || (i % (unsigned)vel_every) != 0;
        resample_push(tr, i, (float)traj_x(t), (float)traj_y(t),
                      has, (float)traj_vx(t), (float)traj_vy(t));
    }
}

/* distance from p to segment [a,b]; 0 when inside */
static double seg_dist(double px, double py, double ax, double ay, double bx, double by) {
    double dx = bx - ax, dy = by - ay;
    double len2 = dx * dx + dy * dy;
    double u = len2 > 0.0 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0;
    if (u < 0.0) u = 0.0;
    if (u > 1.0) u = 1.0;
    double qx = ax + u * dx, qy = ay + u * dy;
    return hypot(px - qx, py - qy);
}

int main(void) {
    const double rates[4] = { 40.0, 52.0, 24.0, 75.0 };
    const double durations[4] = { 0.3, 0.3, 0.3, 0.3 };
    const double vmax_analytic = sqrt(1.5 * 1.5 + 1.2 * 1.2);
    const double omax_analytic = 1.0;

    /* ---- interpolation error against analytic trajectories, variable rate ---- */
    ResampleTrack tr;
    fill_track(&tr, 40.0, 1.4, 0);
    check("track_len", tr.n == 57);

    build_render_times(rates, durations, 4, 0.0);
    check("render_times_ordered", g_ntimes > 50 && g_times[g_ntimes - 1] <= 1.3);

    double max_dt = 0.0, pos_bound = 0.0, over_bound = 0.0, ang_bound = 0.0;
    check("bounds_ok", resample_error_bounds(&tr, &max_dt, &pos_bound, &over_bound,
                                             &ang_bound) == RESAMPLE_OK);
    check("bound_dt", fabs(max_dt - 0.025) < 1e-12);
    check("bounds_positive", pos_bound > 0.0 && over_bound > 0.0 && ang_bound > 0.0);

    double max_pos_err = 0.0, max_over = 0.0;
    int all_ok = 1, no_linear = 1, in_bracket = 1;
    for (int k = 0; k < g_ntimes; k++) {
        Resampled r;
        int rc = resample_sample(&tr, g_times[k], &r);
        if (rc != RESAMPLE_OK) { all_ok = 0; break; }
        if (r.flags & RESAMPLE_F_LINEAR) no_linear = 0;
        if (!(g_times[k] >= r.t0 && g_times[k] <= r.t1)) in_bracket = 0;
        if (fabs(r.t - g_times[k]) > 1e-12) all_ok = 0;
        double ex = fabs((double)r.x - traj_x(g_times[k]));
        double ey = fabs((double)r.y - traj_y(g_times[k]));
        if (hypot(ex, ey) > max_pos_err) max_pos_err = hypot(ex, ey);

        /* overshoot: how far the sample strays from its bracketing chord */
        for (int i = 0; i + 1 < tr.n; i++) {
            if (tr.s[i].t <= g_times[k] && g_times[k] <= tr.s[i + 1].t) {
                double d = seg_dist(r.x, r.y, tr.s[i].x, tr.s[i].y,
                                     tr.s[i + 1].x, tr.s[i + 1].y);
                if (d > max_over) max_over = d;
                break;
            }
        }
    }
    check("sample_all_ok", all_ok);
    check("no_linear_flag_with_full_velocity", no_linear);
    check("render_time_in_bracket", in_bracket);
    check("pos_error_within_bound", max_pos_err <= pos_bound);
    check("pos_error_tight", max_pos_err < 1e-3);
    check("overshoot_within_bound", max_over <= over_bound + 1e-9);

    /* ---- continuity across abrupt frame-rate changes (no jumps) ---- */
    int steps_ok = 1, seam_ok = 1;
    double max_step = 0.0, max_ang_step = 0.0;
    for (int k = 1; k < g_ntimes; k++) {
        Resampled a, b;
        if (resample_sample(&tr, g_times[k - 1], &a) != RESAMPLE_OK) { steps_ok = 0; break; }
        if (resample_sample(&tr, g_times[k], &b) != RESAMPLE_OK) { steps_ok = 0; break; }
        double step = hypot((double)b.x - a.x, (double)b.y - a.y);
        if (step > max_step) max_step = step;
        if (step > vmax_analytic * (g_times[k] - g_times[k - 1]) + 1e-3) steps_ok = 0;
    }
    check("pos_step_matches_motion", steps_ok);
    check("pos_no_jump_on_rate_change",
          max_step <= vmax_analytic * (1.0 / 24.0) + 1e-3);
    for (int q = 1; q < g_nbounds; q++) { /* duplicate crossings excluded */
        Resampled lo, hi;
        if (resample_sample(&tr, g_bounds[q] - 1e-7, &lo) != RESAMPLE_OK ||
            resample_sample(&tr, g_bounds[q] + 1e-7, &hi) != RESAMPLE_OK) { seam_ok = 0; break; }
        if (hypot((double)hi.x - lo.x, (double)hi.y - lo.y) > 1e-3) seam_ok = 0;
    }
    check("pos_continuous_at_seam", seam_ok);

    ResampleTrack tr_ang;
    resample_init(&tr_ang, 40.0);
    for (unsigned i = 0; i <= 56; i++) {
        double t = (double)i / 40.0;
        resample_push(&tr_ang, i, (float)traj_ang(t), 0.0f,
                      1, (float)traj_omg(t), 0.0f);
    }
    check("angle_bounds_ok", resample_error_bounds(&tr_ang, NULL, NULL, NULL,
                                                   &ang_bound) == RESAMPLE_OK);
    int ang_ok = 1, ang_steps_ok = 1;
    double max_ang_err = 0.0;
    Resampled prev;
    int have_prev = 0;
    for (int k = 0; k < g_ntimes; k++) {
        Resampled cur;
        if (resample_sample(&tr_ang, g_times[k], &cur) != RESAMPLE_OK) { ang_ok = 0; break; }
        double e = fabs((double)cur.x - traj_ang(g_times[k]));
        if (e > max_ang_err) max_ang_err = e;
        if (have_prev) {
            double step = fabs((double)cur.x - (double)prev.x);
            if (step > max_ang_step) { max_ang_step = step; max_ang_step = step; }
            if (step > omax_analytic * (g_times[k] - g_times[k - 1]) + 1e-3) ang_steps_ok = 0;
        }
        prev = cur;
        have_prev = 1;
    }
    check("angle_samples_ok", ang_ok);
    check("angle_error_within_bound", max_ang_err <= ang_bound);
    check("angle_error_tight", max_ang_err < 1e-3);
    check("angle_step_matches_motion", ang_steps_ok);
    check("angle_no_jump_on_rate_change",
          max_ang_step <= omax_analytic * (1.0 / 24.0) + 1e-3);

    /* ---- missing interval velocity: linear fallback, flagged ---- */
    ResampleTrack tr_miss;
    fill_track(&tr_miss, 40.0, 1.4, 10); /* every 10th snapshot has no velocity */
    Resampled r;
    check("missing_vel_linear_flag",
          resample_sample(&tr_miss, 30.0 / 40.0 + 0.012, &r) == RESAMPLE_OK &&
          (r.flags & RESAMPLE_F_LINEAR) != 0); /* tick 30 has no velocity */
    check("missing_vel_value_bounded",
          fabs((double)r.x - traj_x(r.t)) <= 1e-3 && fabs((double)r.y - traj_y(r.t)) <= 1e-3);
    check("full_vel_not_linear",
          resample_sample(&tr, 30.0 / 40.0 + 0.012, &r) == RESAMPLE_OK &&
          (r.flags & RESAMPLE_F_LINEAR) == 0);

    double seam_err = 0.0;
    for (int s = 0; s < 2; s++) {
        double tt = (s == 0) ? 10.0 / 40.0 : 20.0 / 40.0; /* missing-velocity seams */
        Resampled lo, hi;
        resample_sample(&tr_miss, tt - 1e-6, &lo);
        resample_sample(&tr_miss, tt + 1e-6, &hi);
        double d = hypot((double)hi.x - lo.x, (double)hi.y - lo.y);
        if (d > seam_err) seam_err = d;
    }
    check("linear_fallback_continuous", seam_err < 1e-3);

    /* ---- bounded extrapolation at the edges ---- */
    ResampleTrack te; /* 10 Hz, linear motion with velocity: extrapolation is exact */
    resample_init(&te, 10.0);
    for (unsigned i = 0; i <= 2; i++) {
        double t = (double)i / 10.0;
        resample_push(&te, i, (float)(2.0 + 3.0 * t), (float)(1.0 - 2.0 * t),
                      1, 3.0f, -2.0f);
    }
    double last_t = 0.2;
    check("extrap_within_limit",
          resample_sample(&te, last_t + 0.02, &r) == RESAMPLE_EXTRAPOLATED &&
          (r.flags & RESAMPLE_F_EXTRAP) != 0);
    check("extrap_uses_boundary_segment",
          r.u > 1.0 && fabs(r.t0 - (last_t - 0.1)) < 1e-12 && fabs(r.t1 - last_t) < 1e-12);
    check("extrap_reports_render_time", fabs(r.t - (last_t + 0.02)) < 1e-12);
    check("extrap_value_exact",
          fabsf(r.x - (float)(2.0 + 3.0 * r.t)) < 1e-5f &&
          fabsf(r.y - (float)(1.0 - 2.0 * r.t)) < 1e-5f);
    check("extrap_left_within_limit",
          resample_sample(&te, -0.02, &r) == RESAMPLE_EXTRAPOLATED &&
          (r.flags & RESAMPLE_F_EXTRAP) && r.u < 0.0);
    check("extrap_left_value_exact", fabsf(r.x - (float)(2.0 + 3.0 * r.t)) < 1e-5f);
    check("extrap_far_right_rejected",
          resample_sample(&te, last_t + 0.3, &r) == RESAMPLE_ERR_FAR);
    check("extrap_far_left_rejected", resample_sample(&te, -0.3, &r) == RESAMPLE_ERR_FAR);
    check("exact_edge_not_extrapolated",
          resample_sample(&te, last_t, &r) == RESAMPLE_OK &&
          (r.flags & RESAMPLE_F_EXTRAP) == 0);

    ResampleTrack t_hold; /* edge velocity missing -> hold, still flagged */
    resample_init(&t_hold, 10.0);
    resample_push(&t_hold, 0, 1.0f, 2.0f, 0, 0, 0);
    resample_push(&t_hold, 1, 3.0f, 2.0f, 0, 0, 0);
    check("extrap_hold_flag",
          resample_sample(&t_hold, 0.12, &r) == RESAMPLE_EXTRAPOLATED &&
          (r.flags & RESAMPLE_F_HOLD) != 0);
    check("extrap_hold_value", fabsf(r.x - 3.0f) < 1e-6f && fabsf(r.y - 2.0f) < 1e-6f);

    /* ---- rejection paths ---- */
    ResampleTrack t1;
    resample_init(&t1, 10.0);
    resample_push(&t1, 5, 1.0f, 1.0f, 0, 0, 0);
    check("single_exact_ok", resample_sample(&t1, 0.5, &r) == RESAMPLE_OK);
    check("single_missing_out_of_range_rejected", resample_sample(&t1, 0.51, &r) == RESAMPLE_ERR_FAR);
    check("reject_inf_time", resample_sample(&t1, INFINITY, &r) == RESAMPLE_ERR_TIME);
    check("reject_nan_time", resample_sample(&t1, NAN, &r) == RESAMPLE_ERR_TIME);
    check("reject_null_out", resample_sample(&te, 0.0, NULL) == RESAMPLE_ERR_NULL);
    check("reject_null_track", resample_sample(NULL, 0.0, &r) == RESAMPLE_ERR_NULL);
    check("reject_bad_rate", resample_set_rate(&t1, 0.0) == RESAMPLE_ERR_RATE);

    ResampleTrack tempty;
    resample_init(&tempty, 30.0);
    check("empty_rejected", resample_sample(&tempty, 0.0, &r) == RESAMPLE_ERR_EMPTY);

    ResampleTrack torder;
    resample_init(&torder, 10.0);
    check("push_ok", resample_push(&torder, 10, 0, 0, 0, 0, 0) == RESAMPLE_OK);
    check("dup_tick_rejected", resample_push(&torder, 10, 0, 0, 0, 0, 0) == RESAMPLE_ERR_ORDER);
    check("backwards_tick_rejected", resample_push(&torder, 9, 0, 0, 0, 0, 0) == RESAMPLE_ERR_ORDER);
    check("gap_too_large_rejected", resample_push(&torder, 110, 0, 0, 0, 0, 0) == RESAMPLE_ERR_RATE);
    check("rate_change_time_reversal_rejected",
          resample_set_rate(&torder, 100.0) == RESAMPLE_OK &&
          resample_push(&torder, 11, 0, 0, 0, 0, 0) == RESAMPLE_ERR_ORDER);
    check("reject_nonfinite_value", resample_push(&tempty, 1, NAN, 0, 0, 0, 0) == RESAMPLE_ERR_TIME);
    check("reject_nonfinite_velocity", resample_push(&tempty, 1, 0, 0, 1, INFINITY, 0) == RESAMPLE_ERR_TIME);

    /* ---- capacity and pruning keep the newest bracket ---- */
    ResampleTrack tcap;
    resample_init(&tcap, 100.0);
    for (unsigned i = 0; i < 100; i++)
        resample_push(&tcap, i, (float)i, 0.0f, 1, 100.0f, 0.0f);
    check("capacity_capped", tcap.n == RESAMPLE_MAX_SNAPSHOTS);
    check("capacity_keeps_newest", tcap.s[tcap.n - 1].tick == 99 && tcap.s[0].tick == 36);
    double cap_dt = 0.0, cap_pos = 0.0;
    check("bounds_survive_eviction",
          resample_error_bounds(&tcap, &cap_dt, &cap_pos, NULL, NULL) == RESAMPLE_OK &&
          fabs(cap_dt - 0.01) < 1e-12 && cap_pos > 0.0);
    check("old_time_too_far", resample_sample(&tcap, 0.1, &r) == RESAMPLE_ERR_FAR);
    check("newest_time_ok", resample_sample(&tcap, 0.95, &r) == RESAMPLE_OK);

    int dropped = resample_prune(&tcap, 0.99);
    check("prune_drops", dropped > 0 && tcap.n == 2);
    check("prune_keeps_bracket", tcap.s[0].t <= 0.99 && tcap.s[1].t >= 0.99);
    check("prune_push_ok", resample_push(&tcap, 100, 100.0f, 0.0f, 1, 100.0f, 0.0f) == RESAMPLE_OK);

    printf("max_pos_err=%.9f pos_bound=%.9f\n", max_pos_err, pos_bound);
    printf("max_ang_err=%.9f ang_bound=%.9f\n", max_ang_err, ang_bound);
    printf("max_overshoot=%.9f over_bound=%.9f\n", max_over, over_bound);
    printf("max_step=%.9f max_ang_step=%.9f\n", max_step, max_ang_step);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
