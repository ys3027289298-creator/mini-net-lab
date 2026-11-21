
/* interp: curvature-adaptive pose interpolation.
 *
 * Blends linear and Catmull-Rom interpolation with a smooth weight
 * driven by local Menger curvature: straight stretches stay exactly
 * linear (kappa = 0 -> pure lerp), curved stretches use the spline.
 * Both curves pass through the segment endpoints, so the blended pose
 * is continuous across segments even when the blend weight changes
 * there -- no jump at the switch.
 *
 * Per-axis error upper bound on one segment of length h (seconds),
 * blend weight a, Mi = max |d^i f/dt^i| over the segment:
 *   E(a) = (1-a)*M2*h^2/8 + a*(M4*h^4/384 + CR_C*M3*h^3)
 * The first term is the linear-interpolation remainder, the second the
 * exact-tangent Hermite remainder plus Catmull-Rom tangent truncation
 * (central difference of f' with error M3*h^2/6 per tangent, weighted
 * by max|h10| = 0.0962: CR_C = 2*0.0962/6 = 0.0321). Position error
 * is bounded by sqrt(2)*E with per-axis maxima; heading error uses the
 * same scalar form with theta's derivative maxima.
 */
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846
#define CR_C 0.0321 /* 2 * max|h10| / 6, Catmull-Rom tangent truncation */

typedef struct { float x, y, th; } Pose;

typedef struct {
    float k_lo, k_hi; /* curvature band: <= k_lo pure linear, >= k_hi pure spline */
} InterpCfg;

static float lerpf(float a, float b, float t) { return a + (b - a) * t; }

/* signed Menger curvature of triangle (a,b,c); 0 when collinear */
float interp_curvature(Pose a, Pose b, Pose c) {
    float abx = b.x - a.x, aby = b.y - a.y;
    float acx = c.x - a.x, acy = c.y - a.y;
    float d1 = hypotf(abx, aby);
    float d2 = hypotf(c.x - b.x, c.y - b.y);
    float d3 = hypotf(acx, acy);
    float den = d1 * d2 * d3;
    if (den < 1e-20f) return 0.0f;
    return 2.0f * (abx * acy - aby * acx) / den;
}

/* smoothstep blend weight from curvature: 0 = linear, 1 = spline */
float interp_blend(float kappa, const InterpCfg* c) {
    float k = fabsf(kappa);
    if (c->k_hi <= c->k_lo) return k > c->k_lo ? 1.0f : 0.0f;
    if (k <= c->k_lo) return 0.0f;
    if (k >= c->k_hi) return 1.0f;
    float t = (k - c->k_lo) / (c->k_hi - c->k_lo);
    return t * t * (3.0f - 2.0f * t);
}

/* blend weight of segment p[1]->p[2] from the larger endpoint curvature */
float interp_segment_alpha(const Pose p[4], const InterpCfg* c) {
    float k1 = interp_curvature(p[0], p[1], p[2]);
    float k2 = interp_curvature(p[1], p[2], p[3]);
    return interp_blend(fabsf(k1) > fabsf(k2) ? k1 : k2, c);
}

static float unwrap_near(float th, float ref) {
    while (th - ref > (float)PI) th -= 2.0f * (float)PI;
    while (th - ref < -(float)PI) th += 2.0f * (float)PI;
    return th;
}

/* blended pose on segment p[1]->p[2], u in [0,1]; p[0],p[3] set the tangents */
Pose interp_eval(const Pose p[4], float u, const InterpCfg* c) {
    float a = interp_segment_alpha(p, c);
    float u2 = u * u, u3 = u2 * u;
    float h00 = 2.0f*u3 - 3.0f*u2 + 1.0f;
    float h10 = u3 - 2.0f*u2 + u;
    float h01 = -2.0f*u3 + 3.0f*u2;
    float h11 = u3 - u2;

    float t0 = unwrap_near(p[0].th, p[1].th);
    float t2 = unwrap_near(p[2].th, p[1].th);
    float t3 = unwrap_near(p[3].th, t2);

    float lx = lerpf(p[1].x, p[2].x, u);
    float ly = lerpf(p[1].y, p[2].y, u);
    float lt = lerpf(p[1].th, t2, u);

    float m1x = 0.5f*(p[2].x - p[0].x), m2x = 0.5f*(p[3].x - p[1].x);
    float m1y = 0.5f*(p[2].y - p[0].y), m2y = 0.5f*(p[3].y - p[1].y);
    float m1t = 0.5f*(t2 - t0),        m2t = 0.5f*(t3 - p[1].th);

    Pose o;
    o.x = lerpf(lx, h00*p[1].x + h10*m1x + h01*p[2].x + h11*m2x, a);
    o.y = lerpf(ly, h00*p[1].y + h10*m1y + h01*p[2].y + h11*m2y, a);
    o.th = lerpf(lt, h00*p[1].th + h10*m1t + h01*t2 + h11*m2t, a);
    o.th = unwrap_near(o.th, 0.0f); /* wrap back to [-pi, pi] */
    return o;
}

/* per-axis error upper bound for one segment, see header comment */
double interp_err_bound(float alpha, double m2, double m3, double m4, double h) {
    double lin = m2*h*h/8.0;
    double spl = m4*h*h*h*h/384.0 + CR_C*m3*h*h*h;
    return (1.0 - (double)alpha)*lin + (double)alpha*spl;
}

/* ---- self test ---- */

typedef struct { double x, y, th; } DPose;
typedef DPose (*GtFn)(double t);

typedef struct {
    const char* name;
    GtFn gt;
    double t0, t1, dt;
    double m2, m3, m4; /* per-axis maxima of position derivatives */
    double a2, a3, a4; /* maxima of heading derivatives */
} Scenario;

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static DPose line_gt(double t) { /* straight: heading constant, kappa = 0 */
    DPose p = { 2.0*t, 0.5*t, 0.0 };
    p.th = atan2(0.5, 2.0);
    return p;
}

static DPose circle_gt(double t) { /* R=10, omega=0.5: kappa = 0.1 */
    DPose p = { 10.0*cos(0.5*t), 10.0*sin(0.5*t), 0.5*t + PI/2 };
    return p;
}

static DPose sine_gt(double t) { /* x=3t, y=2 sin t: kappa crosses the band */
    DPose p = { 3.0*t, 2.0*sin(t), atan2(2.0*cos(t), 3.0) };
    return p;
}

/* d2/ds2 of atan(k cos s), k = 2/3: heading acceleration of sine_gt */
static double sine_h2(double s) {
    const double k = 2.0/3.0;
    double c = cos(s), sn = sin(s);
    double q = 1.0 + k*k*c*c;
    return -k*c*(q + 2.0*k*k*sn*sn)/(q*q);
}

/* heading derivative maxima for sine_gt: analytic 2nd, finite-difference
 * 3rd/4th on a fine grid, inflated by a margin so they stay upper bounds */
static void sine_heading_maxima(double* m2, double* m3, double* m4) {
    const int n = 4096;
    const double h = 1e-4, margin = 1.25;
    double a2 = 0.0, a3 = 0.0, a4 = 0.0;
    for (int i = 0; i < n; i++) {
        double s = 2.0*PI*i/n;
        double v2 = fabs(sine_h2(s));
        double v3 = fabs((sine_h2(s+h) - sine_h2(s-h))/(2.0*h));
        double v4 = fabs((sine_h2(s+h) - 2.0*sine_h2(s) + sine_h2(s-h))/(h*h));
        if (v2 > a2) a2 = v2;
        if (v3 > a3) a3 = v3;
        if (v4 > a4) a4 = v4;
    }
    *m2 = a2*margin; *m3 = a3*margin; *m4 = a4*margin;
}

static double ang_dist(double a, double b) {
    double d = fmod(a - b, 2.0*PI);
    if (d > PI) d -= 2.0*PI;
    if (d < -PI) d += 2.0*PI;
    return fabs(d);
}

#define MAX_POSES 160
#define SAMPLES 32

typedef struct {
    double pos_err, pos_bound, ang_err, ang_bound;
    double pos_gap, ang_gap; /* discontinuity at segment joins */
    float amin, amax;
} RunStats;

/* sample the blended curve against the dense ground-truth trajectory */
static RunStats run_scenario(const Scenario* sc, const InterpCfg* cfg) {
    RunStats st = { 0,0,0,0, 0,0, 1.0f, 0.0f };
    int nseg = (int)((sc->t1 - sc->t0)/sc->dt + 0.5);
    int nposes = nseg + 3;
    Pose snaps[MAX_POSES];
    double base = sc->t0 - sc->dt;
    for (int j = 0; j < nposes; j++) {
        DPose g = sc->gt(base + j*sc->dt);
        snaps[j].x = (float)g.x; snaps[j].y = (float)g.y; snaps[j].th = (float)g.th;
    }
    Pose prev_end = snaps[1];
    int have_prev = 0;
    for (int j = 1; j <= nseg; j++) {
        Pose w[4] = { snaps[j-1], snaps[j], snaps[j+1], snaps[j+2] };
        float a = interp_segment_alpha(w, cfg);
        if (a < st.amin) st.amin = a;
        if (a > st.amax) st.amax = a;
        double pb = sqrt(2.0)*interp_err_bound(a, sc->m2, sc->m3, sc->m4, sc->dt);
        double ab = interp_err_bound(a, sc->a2, sc->a3, sc->a4, sc->dt);
        if (pb > st.pos_bound) st.pos_bound = pb;
        if (ab > st.ang_bound) st.ang_bound = ab;

        Pose start = interp_eval(w, 0.0f, cfg);
        if (have_prev) {
            double gp = hypot((double)start.x - prev_end.x, (double)start.y - prev_end.y);
            double ga = ang_dist(start.th, prev_end.th);
            if (gp > st.pos_gap) st.pos_gap = gp;
            if (ga > st.ang_gap) st.ang_gap = ga;
        }
        for (int s = 0; s <= SAMPLES; s++) {
            float u = (float)s/SAMPLES;
            Pose o = interp_eval(w, u, cfg);
            DPose g = sc->gt(sc->t0 + (j - 1 + (double)u)*sc->dt);
            double pe = hypot((double)o.x - g.x, (double)o.y - g.y);
            double ae = ang_dist(o.th, g.th);
            if (pe > st.pos_err) st.pos_err = pe;
            if (ae > st.ang_err) st.ang_err = ae;
        }
        prev_end = interp_eval(w, 1.0f, cfg);
        have_prev = 1;
    }
    return st;
}

static void check_scenario(const Scenario* sc, const RunStats* st) {
    char nm[64];
    double vis_px = st->pos_err * 32.0; /* visual error: 32 px per world unit */
    printf("metric %s pos_err=%.4g pos_bound=%.4g ang_err_deg=%.4g ang_bound_deg=%.4g vis_px=%.4g alpha=[%.3g,%.3g]\n",
           sc->name, st->pos_err, st->pos_bound,
           st->ang_err*180.0/PI, st->ang_bound*180.0/PI, vis_px, st->amin, st->amax);
    snprintf(nm, sizeof nm, "%s_pos_within_bound", sc->name);
    check(nm, st->pos_err <= st->pos_bound + 1e-5); /* 1e-5: float snapshot slack */
    snprintf(nm, sizeof nm, "%s_ang_within_bound", sc->name);
    check(nm, st->ang_err <= st->ang_bound + 1e-5);
    snprintf(nm, sizeof nm, "%s_no_jump_at_switch", sc->name);
    check(nm, st->pos_gap < 1e-5 && st->ang_gap < 1e-5);
    snprintf(nm, sizeof nm, "%s_visual_within_budget", sc->name);
    check(nm, vis_px < 0.5);
}

int main(void) {
    InterpCfg cfg = { 0.02f, 0.08f };

    Pose c0 = {0,0,0}, c1 = {1,0,0}, c2 = {2,0,0};
    check("collinear_zero_curvature", interp_curvature(c0, c1, c2) == 0.0f);
    check("blend_below_band", interp_blend(0.5f*cfg.k_lo, &cfg) == 0.0f);
    check("blend_above_band", interp_blend(2.0f*cfg.k_hi, &cfg) == 1.0f);
    float bm = interp_blend(0.5f*(cfg.k_lo + cfg.k_hi), &cfg);
    check("blend_mid_smooth", bm > 0.4f && bm < 0.6f);

    /* heading interpolation across the +-pi seam must not swing through 0 */
    Pose wq[4] = { {0,0,3.13f}, {1,0,3.14f}, {2,0,-3.14f}, {3,0,-3.13f} };
    Pose mid = interp_eval(wq, 0.5f, &cfg);
    check("angle_unwrap_seam", fabsf(fabsf(mid.th) - (float)PI) < 0.03f);

    double sa2, sa3, sa4;
    sine_heading_maxima(&sa2, &sa3, &sa4);

    Scenario line = { "line", line_gt, 0.0, 4.0, 0.25, 0,0,0, 0,0,0 };
    Scenario circ = { "circle", circle_gt, 0.0, 2.0, 0.2, 2.5, 1.25, 0.625, 0,0,0 };
    Scenario sine = { "sine", sine_gt, 0.0, 6.3, 0.15, 2.0, 2.0, 2.0, sa2, sa3, sa4 };

    RunStats sl = run_scenario(&line, &cfg);
    RunStats sc = run_scenario(&circ, &cfg);
    RunStats ss = run_scenario(&sine, &cfg);
    check_scenario(&line, &sl);
    check_scenario(&circ, &sc);
    check_scenario(&sine, &ss);

    check("line_stays_linear", sl.amax == 0.0f && sl.pos_err < 1e-5);
    check("circle_full_spline", sc.amin == 1.0f && sc.amax == 1.0f);
    check("sine_switch_exercised", ss.amin < 0.01f && ss.amax > 0.99f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
