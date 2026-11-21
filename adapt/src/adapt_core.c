
/* adapt_core: adaptive degradation controller + motion/error math. */
#include "adapt_core.h"
#include <math.h>
#include <string.h>

const Tier TIERS[N_TIER] = {
    { 20.0, INTERP_HERMITE, 1.00, 0.10, 0.005 },
    { 10.0, INTERP_LINEAR,  0.50, 0.15, 0.012 },
    {  5.0, INTERP_HOLD,    0.25, 0.25, 0.020 },
};

/* RTT (s) and loss-fraction ceilings per tier; worst input wins. */
const double RTT_CEIL[N_TIER]  = { 0.10, 0.25, 1e9 };
const double LOSS_CEIL[N_TIER] = { 0.02, 0.08, 1e9 };

/* sinusoidal joint rig: 4 core joints (full rate), 8 peripheral (LOD-gated) */
const JointCfg JOINTS[N_JOINT] = {
    { 0.90, 1.20, 1 }, { 0.80, 1.50, 1 }, { 0.70, 1.00, 1 }, { 0.60, 1.80, 1 },
    { 0.55, 0.80, 0 }, { 0.50, 1.10, 0 }, { 0.45, 1.40, 0 }, { 0.40, 0.90, 0 },
    { 0.35, 1.60, 0 }, { 0.30, 1.00, 0 }, { 0.25, 1.30, 0 }, { 0.20, 0.70, 0 },
};

void adapt_init(Adapt* a, double now) {
    memset(a, 0, sizeof(*a));
    a->target = a->committed = a->cand = 0;
    a->cand_since = now;
    a->last_valid_at = now;
    a->last_t = now;
    a->delay_s = TIERS[0].delay;
}

/* rtt<0 or loss<0: no fresh reading this step (probe lost / blackout). */
void adapt_observe(Adapt* a, double now, double rtt, double loss) {
    int any = 0;
    if (rtt >= 0.0) {
        a->rtt_ema = a->have_rtt ? ALPHA_EWMA * rtt + (1 - ALPHA_EWMA) * a->rtt_ema : rtt;
        a->have_rtt = 1; any = 1;
    }
    if (loss >= 0.0) {
        a->loss_ema = a->have_loss ? ALPHA_EWMA * loss + (1 - ALPHA_EWMA) * a->loss_ema : loss;
        a->have_loss = 1; any = 1;
    }
    if (any) a->last_valid_at = now;
}

static int tier_from_readings(double rtt, double loss) {
    int t = 0;
    while (t < N_TIER - 1 && (rtt > RTT_CEIL[t] || loss > LOSS_CEIL[t])) t++;
    return t;
}

/* Hysteresis: once primed, a candidate different from the committed tier must
 * persist for the dwell time (short to downgrade, long to upgrade). The first
 * evidence after init commits immediately; a missing-reading conservative
 * fallback commits immediately too, because the MISS_GRACE window itself is
 * the required evidence. Returns 1 on change. */
int adapt_step(Adapt* a, double now) {
    int prev = a->committed;

    if (now - a->last_valid_at > MISS_GRACE) {
        a->target = N_TIER - 1;              /* conservative while readings missing */
        a->reason = REASON_MISSING;
    } else {
        a->target = tier_from_readings(a->have_rtt ? a->rtt_ema : 1e9,
                                       a->have_loss ? a->loss_ema : 1e9);
        a->reason = REASON_NORMAL;
    }

    if (a->target != a->cand) {              /* new candidate: restart dwell clock */
        a->cand = a->target;
        a->cand_since = now;
    }

    int immediate = !a->primed || a->reason == REASON_MISSING;
    if (a->cand != a->committed) {
        double dwell = a->cand > a->committed ? DWELL_DOWN : DWELL_UP;
        if (immediate || now - a->cand_since >= dwell) {
            a->committed = a->cand;
        }
    }
    a->primed = 1;

    /* continuous knob lags toward the committed tier instead of snapping */
    double dt = now - a->last_t;
    a->last_t = now;
    if (dt > 0.0) {
        double k = 1.0 - exp(-dt / SMOOTH_TAU);
        a->delay_s += (TIERS[a->committed].delay - a->delay_s) * k;
    }
    return a->committed != prev;
}

double adapt_play_delay(const Adapt* a) { return a->delay_s; }

double joint_pos(const JointCfg* j, double t) { return j->amp * sin(2 * PI * j->freq * t); }
double joint_vel(const JointCfg* j, double t) { return j->amp * 2 * PI * j->freq * cos(2 * PI * j->freq * t); }
double joint_vmax(const JointCfg* j)          { return j->amp * 2 * PI * j->freq; }
double joint_amax(const JointCfg* j)          { return j->amp * pow(2 * PI * j->freq, 2.0); }

float quantize(double v, double q) { return (float)(lrint(v / q) * q); }

double hermite_at(double ta, double pa, double va, double tb, double pb, double vb, double t) {
    double T = tb - ta;
    if (T <= STALL_EPS) return pb;
    double u = (t - ta) / T, u2 = u * u, u3 = u2 * u;
    double h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u,
           h01 = -2 * u3 + 3 * u2,    h11 = u3 - u2;
    return h00 * pa + h10 * T * va + h01 * pb + h11 * T * vb;
}

/*
 * Analytic upper bound on |rendered - true| for one joint at one frame.
 *  T : keyframe interval bracketing play time (s)
 *  L : extrapolation look-ahead when play time passes the newest sample
 *      (0 when interpolating)
 * Hermite carries a 10% tangent-slack term: packet velocities are quantized
 * estimates, not exact tangents.
 */
double error_bound(int interp, double T, double L, double q, double vmax, double amax) {
    double e = q;
    if (L > 0.0) {
        /* extrapolation beyond the newest keyframe: motion envelope over L,
         * plus the tangent/velocity quantization carried into the slope. */
        e += vmax * L + 0.5 * amax * L * L;
        if (interp == INTERP_HERMITE || interp == INTERP_LINEAR)
            e += q * L;                          /* quantized slope error over look-ahead */
        if (interp == INTERP_HERMITE)
            e += 0.1 * vmax * T;                /* tangent-slack term */
    } else if (interp == INTERP_HOLD) {
        e += vmax * T;
    } else if (interp == INTERP_LINEAR) {
        e += 0.5 * amax * T * T;
    } else {
        e += 0.5 * amax * T * T + 0.1 * vmax * T;
    }
    return e;
}
