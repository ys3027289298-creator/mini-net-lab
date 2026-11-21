
/* interp: render-timeline alignment for remote entities.
 *
 * render_time = now - off, where off is a smoothed offset estimated from
 * snapshot arrival delay. Sampling interpolates the snapshot buffer at
 * render_time. Offset slews at a bounded rate (smooth), large deviations
 * are counted as jumps, and timestamp rollbacks are recovered (drop or
 * rebase) instead of crashing.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define SNAP_BUF 32

typedef struct { double ts; float x; } Snap;

typedef struct {
    double off;         /* smoothed render offset (s) */
    double rate;        /* max offset slew, s per s */
    double margin;      /* extra delay margin on top of measured delay */
    double jump_thresh; /* |delay+margin - off| above this counts as a jump */
    double rebase_gap;  /* timestamp rollback >= this => timeline rebase */
    double max_dev;     /* running max of |target - off|: sampling-time error bound */
    double last_ts;
    int has_ts;
    int jumps, rebases, stale;
    Snap buf[SNAP_BUF];
    int head, count;
} Interp;

static void interp_init(Interp* ip) {
    memset(ip, 0, sizeof(*ip));
    ip->rate = 1.0;
    ip->margin = 0.1;
    ip->jump_thresh = 0.25;
    ip->rebase_gap = 2.0;
}

/* returns 0 accepted, 1 stale/duplicate dropped, 2 timeline rebased */
static int interp_push(Interp* ip, double now, double ts, float x) {
    int rc = 0;
    if (ip->has_ts && ts <= ip->last_ts) {
        if (ip->last_ts - ts >= ip->rebase_gap) {
            /* server timeline restarted: drop everything, re-seed offset */
            ip->count = 0;
            ip->head = 0;
            ip->off = (now - ts) + ip->margin;
            if (ip->off < 0) ip->off = 0;
            ip->max_dev = 0;
            ip->rebases++;
            rc = 2;
        } else {
            ip->stale++;            /* late/duplicate packet: keep timeline */
            return 1;
        }
    }
    if (!ip->has_ts) {              /* first packet seeds the offset */
        ip->off = (now - ts) + ip->margin;
        if (ip->off < 0) ip->off = 0;
    }
    ip->last_ts = ts;
    ip->has_ts = 1;
    ip->buf[ip->head].ts = ts;
    ip->buf[ip->head].x = x;
    ip->head = (ip->head + 1) % SNAP_BUF;
    if (ip->count < SNAP_BUF) ip->count++;
    if (rc == 0) {
        double delay = now - ts;
        if (delay < 0) delay = 0;   /* clock weirdness: clamp, don't crash */
        if (fabs(delay + ip->margin - ip->off) > ip->jump_thresh) ip->jumps++;
    }
    return rc;
}

/* advance smoothed offset toward target, slew-limited for smoothness */
static void interp_update(Interp* ip, double now, double dt) {
    if (!ip->has_ts || ip->count == 0 || dt <= 0) return;
    double newest = ip->buf[(ip->head - 1 + SNAP_BUF) % SNAP_BUF].ts;
    double target = (now - newest) + ip->margin;
    if (target < 0) target = 0;
    double delta = target - ip->off;
    double step = ip->rate * dt;
    if (delta > step) delta = step;
    else if (delta < -step) delta = -step;
    ip->off += delta;
    double dev = fabs(target - ip->off);
    if (dev > ip->max_dev) ip->max_dev = dev;
}

/* upper bound on |sample_time - intended_time| observed so far */
static double interp_error_bound(const Interp* ip) { return ip->max_dev; }

/* sample buffer at render_time = now - off.
 * returns 0 interpolated, 1 clamped to an endpoint, -1 no data */
static int interp_sample(const Interp* ip, double now, float* out) {
    if (ip->count == 0 || out == NULL) return -1;
    double rt = now - ip->off;
    int newest = (ip->head - 1 + SNAP_BUF) % SNAP_BUF;
    int oldest = (ip->head - ip->count + SNAP_BUF) % SNAP_BUF;
    if (rt >= ip->buf[newest].ts) { *out = ip->buf[newest].x; return 1; }
    if (rt <= ip->buf[oldest].ts) { *out = ip->buf[oldest].x; return 1; }
    for (int i = 0; i < ip->count - 1; i++) {
        int hi = (newest - i + SNAP_BUF) % SNAP_BUF;
        int lo = (newest - i - 1 + SNAP_BUF) % SNAP_BUF;
        if (ip->buf[lo].ts <= rt && rt <= ip->buf[hi].ts) {
            double span = ip->buf[hi].ts - ip->buf[lo].ts;
            double a = span > 0 ? (rt - ip->buf[lo].ts) / span : 0.0;
            *out = (float)(ip->buf[lo].x + a * (ip->buf[hi].x - ip->buf[lo].x));
            return 0;
        }
    }
    *out = ip->buf[oldest].x;       /* unreachable if ts monotonic */
    return 1;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* scenario: 10Hz snapshots, x = v*t analytically, net delay 0.15s + jitter */
    Interp ip;
    interp_init(&ip);
    const double v = 2.0, delay = 0.15, dt = 1.0 / 60.0;
    double now = 0.0, next_ts = 0.0;
    double max_slew = 0.0, prev_off = 0.0, max_pos_err = 0.0, max_time_err = 0.0;
    int interp_frames = 0, bound_ok = 1, empty_sample;

    check("empty_sample", interp_sample(&ip, 0.0, &(float){0}) == -1);
    empty_sample = 1;

    for (int frame = 0; frame < 600; frame++) {
        now = frame * dt;
        /* deliver snapshots whose (delayed) arrival time has passed */
        int i = 0;
        while (next_ts + delay + 0.02 * ((i++ * 7 + frame) % 3 - 1) <= now) {
            interp_push(&ip, now, next_ts, (float)(v * next_ts));
            next_ts += 0.1;
        }
        prev_off = ip.off;
        interp_update(&ip, now, dt);
        double slew = fabs(ip.off - prev_off);
        if (slew > max_slew) max_slew = slew;

        /* sampling-time error vs the same target the estimator chases */
        if (ip.count > 0) {
            double newest = ip.buf[(ip.head - 1 + SNAP_BUF) % SNAP_BUF].ts;
            double target = (now - newest) + ip.margin;
            double terr = fabs(ip.off - target);
            if (terr > max_time_err) max_time_err = terr;
            if (terr > interp_error_bound(&ip) + 1e-12) bound_ok = 0;
        }
        /* interpolated position must match the analytic timeline at now-off */
        float xs = 0.0f;
        if (interp_sample(&ip, now, &xs) == 0) {
            double perr = fabs(xs - v * (now - ip.off));
            if (perr > max_pos_err) max_pos_err = perr;
            interp_frames++;
        }
    }
    check("empty_sample_flag", empty_sample);
    check("converged", fabs(ip.off - (delay + ip.margin)) < 0.05);
    check("smooth_slew", max_slew <= ip.rate * dt + 1e-12);
    check("bound_covers_err", bound_ok && max_time_err <= interp_error_bound(&ip) + 1e-12);
    check("interp_accurate", interp_frames > 100 && max_pos_err < 1e-4);

    /* delay spike: offset jump must be detected, then re-converge smoothly */
    int jumps_before = ip.jumps;
    double last_ts = ip.buf[(ip.head - 1 + SNAP_BUF) % SNAP_BUF].ts;
    for (int k = 0; k < 5; k++)
        interp_push(&ip, now + 0.8 + k * 0.1, last_ts + 0.1 * (k + 1), 0.0f);
    check("jump_detected", ip.jumps > jumps_before);

    /* small timestamp rollback: stale packet dropped, timeline intact */
    Interp r1;
    interp_init(&r1);
    interp_push(&r1, 1.0, 1.0, 1.0f);
    interp_push(&r1, 1.1, 1.1, 2.0f);
    check("stale_dropped", interp_push(&r1, 1.15, 1.05, 9.0f) == 1);
    check("stale_counted", r1.stale == 1 && r1.count == 2 && r1.rebases == 0);

    /* large rollback (server restart): rebase, recover, keep sampling */
    interp_push(&r1, 3.2, 3.0, 3.0f);
    check("rebase_rc", interp_push(&r1, 5.0, 0.5, 0.5f) == 2);
    check("rebase_counted", r1.rebases == 1 && r1.count == 1);
    interp_push(&r1, 5.1, 0.7, 1.5f);
    interp_update(&r1, 5.2, 0.1);
    float xr = 0.0f;
    check("rebase_recovered", interp_sample(&r1, 5.2, &xr) == 0);
    check("rebase_sane", fabsf(xr - 1.0f) < 1e-3f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
