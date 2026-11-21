/* interp: adaptive-depth interpolation buffer for jittery sample streams.
 *
 * A hard-coded buffer depth is wrong in both directions: shallow, it hitches
 * whenever a burst of packets arrives late; deep, it adds latency even on a
 * clean link.  This module measures the delay spread it must absorb and sizes
 * the playout depth from it:
 *
 *   delay_i  = arrival_i - t_i                 (one-way delay, seconds)
 *   base     = min(delay_i)                    (fastest path, min-filtered)
 *   spread   = decayed peak of (delay_i - base)  <- burst depth to absorb
 *   period   = sender sample interval            <- lookahead to interpolate
 *   target   = clamp(gain * spread + period, depth_min, depth_max)
 *   depth    = target reached at a bounded slew rate (no visible steps)
 *
 * The `period` term is not slack: to render sender time P by interpolation we
 * need the sample after P to have arrived too, so the budget is spread + one
 * interval.  Leaving it out makes a nominally sufficient depth underrun by
 * exactly one frame every time a packet is late.
 *
 * Why peak-of-spread and not the usual RTP mean-|D|: an isolated 30 ms burst
 * is exactly what an underrun is made of, and a mean washes it out.  The peak
 * is held for ~30 s (a decay time constant) so the buffer stays deep enough
 * across a run of periodic bursts, then comes back down when the link is quiet.
 *
 * Sender time is mapped to local time with the min-filtered base delay.  The
 * playhead is `local_time - base - depth` in sender time; it advances at real
 * time with a <=5 % rate trim so that changing the depth does not step the
 * picture.  A burst that outruns the buffer holds the last sample and counts an
 * underrun.  Depth zero is refused: it is "no buffer", the exact hitch this
 * module exists to absorb.
 *
 * build/test: make -C interp test
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define RING 128
#define PEAK_TAU 30.0               /* s; hold a learned burst for many cycles */
#define OFFSET_LEAK (1.0 / 2048.0)  /* slow upward re-estimate after a path change */
#define TRIM_MAX 0.05               /* playhead rate correction, fraction */
#define TRIM_KP 4.0                 /* 1 s of playhead error -> 4x correction */

typedef struct {
    double depth_min;   /* > 0; a zero floor is rejected */
    double depth_max;
    double slew;        /* max depth change per second of wall time */
    double gain;        /* depth = gain * spread (>= 1 to cover the burst) */
} Config;

typedef struct {
    double t;           /* sender timestamp, seconds */
    float  x;
} Sample;

typedef struct {
    Config cfg;
    Sample s[RING];
    int head, count;

    double base;                /* min-filtered one-way delay */
    int    have_base;
    double period;              /* EMA of sender sample interval */
    double prev_t;              /* last sender time (spread decay, period) */
    double spread;              /* decayed peak of delay - base */

    double depth;               /* current depth, seconds */
    double playhead;            /* sender-time position being rendered */
    double clock;               /* last local time seen by interp_update */
    int    started;
    double max_depth_seen;
    unsigned underruns;         /* frames where the playhead ran past newest */
    int    starved;             /* currently held at newest; count once */
} Interp;

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* returns 0 on success, -1 on a configuration that cannot buffer anything */
static int interp_init(Interp* b, const Config* cfg) {
    if (cfg == NULL || b == NULL) return -1;
    if (!(cfg->depth_min > 0.0)) return -1;          /* zero depth refused */
    if (!(cfg->depth_max >= cfg->depth_min)) return -1;
    if (!(cfg->slew > 0.0)) return -1;
    if (!(cfg->gain > 0.0)) return -1;
    memset(b, 0, sizeof(*b));
    b->cfg = *cfg;
    b->depth = cfg->depth_min;
    b->max_depth_seen = b->depth;
    return 0;
}

static int interp_push(Interp* b, double t, double arrival, float x) {
    if (!(t >= 0.0) || !(arrival >= 0.0)) return -1;

    double delay = arrival - t;                   /* one-way delay sample */
    if (!b->have_base) { b->base = delay; b->have_base = 1; }
    else if (delay < b->base) b->base = delay;    /* snap to a faster path */
    else b->base += (delay - b->base) * OFFSET_LEAK;

    double over = delay - b->base;
    if (over < 0.0) over = 0.0;

    if (b->spread > 0.0)                          /* spread decays with time */
        b->spread *= exp(-(t - b->prev_t) / PEAK_TAU);
    if (over > b->spread) b->spread = over;       /* ...and jumps to a new peak */

    if (b->count > 0) {
        double gap = t - b->prev_t;
        if (gap > 0.0) b->period += (gap - b->period) * 0.1;  /* sender cadence */
    }

    b->prev_t = t;
    b->s[b->head].t = t;
    b->s[b->head].x = x;
    b->head = (b->head + 1) % RING;
    if (b->count < RING) b->count++;

    if (!b->started) {
        /* Anchor `depth` behind mapped-now, not behind the first sample:
         * the base delay has already been spent by the time it arrives. */
        b->playhead = arrival - b->base - b->depth;
        b->started = 1;
    }
    return 0;
}

/* advance to local time `now`: retarget depth (slew-limited), then move the
 * playhead toward (now - base - depth), bounded to TRIM_MAX of real time. */
static int interp_update(Interp* b, double now) {
    if (!(now >= 0.0)) return -1;
    double dt = now - b->clock;
    if (dt < 0.0) return -1;
    b->clock = now;
    if (dt == 0.0) return 0;

    double target = clampd(b->cfg.gain * b->spread + b->period,
                           b->cfg.depth_min, b->cfg.depth_max);
    double step = b->cfg.slew * dt;
    b->depth = clampd(b->depth + clampd(target - b->depth, -step, step),
                      b->cfg.depth_min, b->cfg.depth_max);
    if (b->depth > b->max_depth_seen) b->max_depth_seen = b->depth;

    if (b->started) {
        double goal = now - b->base - b->depth;
        b->playhead += dt;                              /* real time */
        /* close only part of the gap, and no faster than 5 % of real time,
         * so a depth change never appears as a step in the output. */
        double corr = (goal - b->playhead) * TRIM_KP * dt;
        double lim = dt * TRIM_MAX;
        b->playhead += clampd(corr, -lim, lim);
    }
    return 0;
}

/* render at the playhead.  returns 1 and writes *x when a value is available;
 * 0 when there is no buffer (empty, not started, or depth == 0). */
static int interp_sample(Interp* b, float* x) {
    if (!b->started || b->count == 0) return 0;
    if (!(b->depth > 0.0)) return 0;                 /* zero depth refused */
    int oldest = b->head - b->count;
    while (oldest < 0) oldest += RING;
    int newest = (b->head - 1 + RING) % RING;
    double lo = b->s[oldest].t, hi = b->s[newest].t;

    if (b->playhead > hi) {                           /* burst outran the buffer */
        b->playhead = hi;
        if (!b->starved) { b->underruns++; b->starved = 1; }  /* count episode */
    } else if (b->playhead < hi - 1e-6) {
        b->starved = 0;                               /* data is ahead again */
    }
    if (b->playhead < lo) {
        if (b->count < RING) return 0;   /* still filling the buffer; nothing */
        /* Ring is full and the playhead has fallen off the back: the data it
         * wanted was overwritten, i.e. the ring is too shallow for this depth. */
        b->playhead = lo;
        if (!b->starved) { b->underruns++; b->starved = 1; }
    }

    int a = oldest, c = newest;
    for (int i = 0; i < b->count; i++) {
        int idx = (oldest + i) % RING;
        if (b->s[idx].t <= b->playhead) a = idx;
        if (b->s[idx].t >= b->playhead) { c = idx; break; }
    }
    double span = b->s[c].t - b->s[a].t;
    double u = span > 1e-12 ? (b->playhead - b->s[a].t) / span : 0.0;
    if (x) *x = (float)(b->s[a].x + (b->s[c].x - b->s[a].x) * u);
    return 1;
}

/* how far the newest arrived sample sits ahead of the playhead; this is the
 * latency the buffer is actually paying right now. */
static double interp_lead(const Interp* b) {
    if (!b->started || b->count == 0) return 0.0;
    int newest = (b->head - 1 + RING) % RING;
    return b->s[newest].t - b->playhead;
}

/* ---- self test ---------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static double noise(unsigned* seed) {
    *seed = *seed * 1664525u + 1013904223u;
    return ((*seed >> 8) & 0xffff) / 65535.0 - 0.5;   /* [-0.5, 0.5] */
}

typedef struct {
    double depth_avg, depth_max, lead_avg_ms, out_jitter_ms;
    unsigned underruns, ur_at_warmup, steady_underruns;
} Result;

#define FRAME_DT (1.0 / 60.0)
#define BASE_LAT 0.030
#define WARMUP 240
#define FRAMES 1800

/* 60 Hz sender.  Packets arrive with +/- jitter/2 s of noise; a 0.5 s burst
 * every 3 s adds 0.75*jitter of delay to a run of packets.  adaptive=1 sizes
 * depth from jitter; adaptive=0 pins it to fixed_depth. */
static Result simulate(int adaptive, double fixed_depth, double jitter) {
    Config cfg = { .depth_min = adaptive ? 0.015 : fixed_depth,
                   .depth_max = adaptive ? 0.250 : fixed_depth,
                   .slew = 0.80, .gain = 1.2 };
    Interp b;
    Result r = { 0 };
    if (interp_init(&b, &cfg) != 0) { r.out_jitter_ms = -1.0; return r; }

    unsigned seed = 0x9e3779b9u;
    long next_k = 0;
    double last_x = 0.0, sq = 0.0, lat = 0.0, depth = 0.0;
    int have_last = 0, n = 0, nl = 0;

    for (int f = 0; f < FRAMES; f++) {
        double now = f * FRAME_DT;
        int burst = (f % 180) >= 150;
        for (;;) {
            double t = next_k * FRAME_DT;
            double arrival = t + BASE_LAT + noise(&seed) * jitter * 0.5;
            if (burst) arrival += 0.75 * jitter;
            if (arrival > now) break;
            interp_push(&b, t, arrival, (float)t);   /* x == sender time: v = 1 */
            next_k++;
        }
        interp_update(&b, now);
        float x = 0.0f;
        if (f == WARMUP) r.ur_at_warmup = b.underruns;
        if (interp_sample(&b, &x)) {
            if (have_last && f >= WARMUP) {
                double e = (double)x - last_x - FRAME_DT;
                sq += e * e;
                n++;
            }
            last_x = x;
            have_last = 1;
        }
        if (f >= WARMUP) { lat += interp_lead(&b); depth += b.depth; nl++; }
    }
    r.depth_avg = nl ? depth / nl : 0.0;
    r.depth_max = b.max_depth_seen;
    r.lead_avg_ms = nl ? lat / nl * 1000.0 : 0.0;
    r.out_jitter_ms = n ? sqrt(sq / n) * 1000.0 : -1.0;
    r.underruns = b.underruns;
    r.steady_underruns = b.underruns - r.ur_at_warmup;
    return r;
}

int main(void) {
    /* --- zero depth is refused, at init and at render --- */
    {
        Interp b;
        Config zero = { .depth_min = 0.0, .depth_max = 0.2, .slew = 1.0, .gain = 2.0 };
        check("init_rejects_zero_depth", interp_init(&b, &zero) == -1);

        Config ok = { .depth_min = 0.01, .depth_max = 0.2, .slew = 1.0, .gain = 2.0 };
        check("init_accepts_positive_depth", interp_init(&b, &ok) == 0);
        interp_push(&b, 0.0, 0.04, 1.0f);
        interp_update(&b, FRAME_DT);
        b.depth = 0.0;                       /* force it; render must refuse */
        check("sample_rejects_zero_depth", interp_sample(&b, NULL) == 0);

        check("init_rejects_inverted_range",
              interp_init(&b, &(Config){ 0.2, 0.1, 1.0, 2.0 }) == -1);
        check("init_rejects_zero_slew",
              interp_init(&b, &(Config){ 0.01, 0.2, 0.0, 2.0 }) == -1);
    }

    /* --- input validation --- */
    {
        Interp b;
        Config ok = { 0.01, 0.2, 1.0, 2.0 };
        interp_init(&b, &ok);
        check("reject_negative_time", interp_push(&b, -1.0, 0.0, 0.0f) == -1);
        check("reject_negative_arrival", interp_push(&b, 0.0, -1.0, 0.0f) == -1);
        check("empty_render_misses", interp_sample(&b, NULL) == 0);
        check("clock_reversal_rejected", interp_update(&b, 1.0) == 0 &&
                                         interp_update(&b, 0.5) == -1);
    }

    /* --- comparison table: fixed shallow / fixed deep / adaptive --- */
    const double LOW_J = 0.004;    /* +/-2 ms inter-arrival noise  */
    const double HIGH_J = 0.040;   /* +/-20 ms inter-arrival noise */

    Result sh_low  = simulate(0, 0.015, LOW_J);
    Result ad_low  = simulate(1, 0.0,   LOW_J);
    Result df_low  = simulate(0, 0.160, LOW_J);
    Result sh_high = simulate(0, 0.015, HIGH_J);
    Result ad_high = simulate(1, 0.0,   HIGH_J);
    Result df_high = simulate(0, 0.160, HIGH_J);

    printf("cmp scenario depth_ms lead_ms out_jitter_ms underruns steady_ur\n");
#define ROW(tag, r) printf("cmp %-18s %7.1f %7.1f %10.3f %u %u\n", tag, \
                           (r).depth_avg * 1000.0, (r).lead_avg_ms, \
                           (r).out_jitter_ms, (r).underruns, (r).steady_underruns)
    ROW("low_fixed_shallow", sh_low);
    ROW("low_fixed_deep", df_low);
    ROW("low_adaptive", ad_low);
    ROW("high_fixed_shallow", sh_high);
    ROW("high_fixed_deep", df_high);
    ROW("high_adaptive", ad_high);
#undef ROW

    /* A burst the buffer has never seen cannot be anticipated: the first one
     * is expected to underrun.  After one learning cycle the estimate holds. */
    check("low_jitter_adaptive_no_underruns", ad_low.steady_underruns == 0);
    check("low_jitter_adaptive_near_floor", ad_low.depth_avg < 0.040);
    check("low_jitter_adaptive_cheaper_than_deep",
          ad_low.depth_avg + 0.020 < df_low.depth_avg);
    check("high_jitter_shallow_underruns", sh_high.steady_underruns > 0);
    check("high_jitter_adaptive_fixes_underruns",
          ad_high.steady_underruns == 0 && sh_high.steady_underruns > 0);
    check("high_jitter_adaptive_beats_shallow_jitter",
          ad_high.out_jitter_ms * 2.0 < sh_high.out_jitter_ms);
    check("high_jitter_adaptive_cheaper_than_deep",
          ad_high.depth_avg + 0.020 < df_high.depth_avg);
    /* the hard upper bound on extra latency: adaptive never buffers more than
     * depth_max, so its added delay over the bare link is <= depth_max. */
    check("adaptive_extra_latency_bounded",
          ad_high.depth_max <= 0.250 + 1e-9 &&
          ad_high.lead_avg_ms <= 0.250 * 1000.0);
    /* low-jitter link must not pay for high-jitter safety: depth stays near
     * the floor rather than the ceiling. */
    check("adaptive_does_not_inflate_low_jitter",
          ad_low.depth_avg < ad_high.depth_avg * 0.5);

    /* --- latency / smoothness curve for fixed depths --- */
    printf("curve jitter_ms depth_ms out_jitter_ms underruns\n");
    for (double j = 0.005; j < 0.041; j += 0.010) {
        for (double d = 0.015; d < 0.181; d += 0.020) {
            Result r = simulate(0, d, j);
            printf("curve %.1f %.1f %.3f %u\n", j * 1000.0, d * 1000.0,
                   r.out_jitter_ms, r.underruns);
        }
    }

    /* --- depth tracks jitter monotonically (more jitter -> more buffer) --- */
    {
        double prev = -1.0;
        int mono = 1;
        for (double j = 0.002; j < 0.051; j += 0.008) {
            Result r = simulate(1, 0.0, j);
            printf("track jitter_ms=%.1f depth_ms=%.1f lead_ms=%.1f underruns=%u\n",
                   j * 1000.0, r.depth_avg * 1000.0, r.lead_avg_ms, r.underruns);
            if (prev >= 0.0 && r.depth_avg + 1e-6 < prev) mono = 0;
            prev = r.depth_avg;
        }
        check("depth_monotone_in_jitter", mono);
    }

    /* --- slew bound: depth moves at most slew*dt per tick --- */
    {
        Config cfg = { 0.010, 0.500, 0.50, 50.0 };   /* huge gain, 0.5 s/s slew */
        Interp b;
        interp_init(&b, &cfg);
        double max_jump = 0.0, prev = b.depth;
        for (int i = 0; i < 600; i++) {
            double now = i * FRAME_DT;
            double arrival = now + 0.04 + (i & 1 ? 0.03 : -0.03);  /* violent */
            interp_push(&b, now, arrival, (float)now);
            interp_update(&b, now);
            double jump = fabs(b.depth - prev);
            if (jump > max_jump) max_jump = jump;
            prev = b.depth;
        }
        check("slew_bounded", max_jump <= 0.50 * FRAME_DT + 1e-9);
        check("depth_stays_in_range", b.depth >= cfg.depth_min - 1e-9 &&
                                      b.depth <= cfg.depth_max + 1e-9);
    }

    /* --- depth changes do not step the picture: rate stays within +/-5 % --- */
    {
        Config cfg = { 0.010, 0.250, 5.0, 4.0 };     /* deliberately fast slew */
        Interp b;
        interp_init(&b, &cfg);
        unsigned seed = 3u;
        double last_x = 0.0, max_err = 0.0;
        int have = 0;
        for (int f = 0; f < 600; f++) {
            double now = f * FRAME_DT;
            int burst = (f % 120) >= 100;
            /* simple sender: one sample per frame, jitter switches on mid-run */
            double t = f * FRAME_DT;
            double amp = f < 300 ? 0.002 : 0.020;
            double arrival = t + BASE_LAT + noise(&seed) * amp * 0.5 + (burst ? 0.015 : 0.0);
            interp_push(&b, t, arrival, (float)t);
            interp_update(&b, now);
            float x = 0.0f;
            int got = interp_sample(&b, &x);
            if (got && have && f >= 60) {
                double err = fabs((double)x - last_x - FRAME_DT);
                if (err > max_err) max_err = err;
            }
            if (got) { have = 1; last_x = x; }
        }
        check("depth_change_does_not_step", max_err <= FRAME_DT * 0.06 + 1e-3);
    }

    /* --- a burst is absorbed with enough depth, not with 15 ms.  Adaptive
     * needs one learning cycle: after that it absorbs every repeat burst. --- */
    {
        check("burst_needs_depth",
              sh_high.steady_underruns > df_high.steady_underruns);
        check("deep_absorbs_burst", df_high.underruns == 0);
        check("adaptive_absorbs_repeat_bursts", ad_high.steady_underruns == 0);
        check("adaptive_cold_start_bounded", ad_high.underruns <= 8);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
