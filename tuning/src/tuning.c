
/* tuning: update-frequency sweep, bandwidth/error Pareto frontier, knee pick.
 *
 * Model, grounded in the lab modules:
 *   bandwidth(f) = 8 * f * (snapshot packet bytes + UDP/IP header)   [bit/s]
 *   error(f)     = quant floor (snapshot) + dead-reckoning drift (reconcile)
 *                = 1/(2*POS_SCALE) + 0.5*ACC / f^2                  [m]
 * bandwidth grows linearly in f, error falls as 1/f^2 -> convex frontier,
 * so a knee exists and is found by max distance to the chord.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define POS_SCALE 64.0   /* snapshot quant scale */
#define UDP_HDR 28.0     /* IP+UDP header bytes */
#define ACC 20.0         /* entity accel m/s^2, dead-reckoning error source */

#define FMIN 1.0         /* Hz; f <= 0 is rejected, see sweep() */
#define FMAX 120.0
#define COARSE_N 13      /* swept samples */
#define DENSE_N 1201     /* exhaustive reference grid */

typedef struct { double f, bw, err; } Pt;

/* snapshot packet size: varint(tick) + zigzag-varint(qx) + zigzag-varint(qy) */
static int varint_len(unsigned v) { int n = 1; while (v >= 0x80) { v >>= 7; n++; } return n; }

static double pkt_bytes(void) {
    unsigned tick = 1000;
    int qx = (int)lrint(12.5 * POS_SCALE), qy = (int)lrint(-3.25 * POS_SCALE);
    unsigned zx = (unsigned)((qx << 1) ^ (qx >> 31));
    unsigned zy = (unsigned)((qy << 1) ^ (qy >> 31));
    return varint_len(tick) + varint_len(zx) + varint_len(zy);
}

static double bandwidth(double f) { return 8.0 * f * (pkt_bytes() + UDP_HDR); }

/* f <= 0 is meaningless (no updates, and 1/f^2 blows up): rejected */
static double err_model(double f) {
    if (f <= 0.0) return -1.0;
    return 1.0 / (2.0 * POS_SCALE) + 0.5 * ACC / (f * f);
}

/* log-spaced sweep of n samples in [fmin, fmax]; -1 on bad config */
static int sweep(double fmin, double fmax, int n, Pt* out) {
    if (out == NULL) return -1;
    if (fmin <= 0.0 || fmax <= fmin || n < 1) return -1;   /* zero/neg freq rejected */
    double step = n > 1 ? log(fmax / fmin) / (n - 1) : 0.0;
    for (int i = 0; i < n; i++) {
        double f = fmin * exp(step * i);
        out[i].f = f;
        out[i].bw = bandwidth(f);
        out[i].err = err_model(f);
    }
    return 0;
}

/* Pareto filter: input sorted by bw ascending, keep strictly decreasing err */
static int pareto(const Pt* in, int n, Pt* out) {
    int m = 0;
    double best = HUGE_VAL;
    for (int i = 0; i < n; i++) {
        if (in[i].err < best) { out[m++] = in[i]; best = in[i].err; }
    }
    return m;
}

/* knee = max distance to chord in min-max normalized space.
 * -1 if n < 3 (too few samples to define a bend) or frontier is degenerate. */
static int knee(const Pt* p, int n) {
    if (n < 3) return -1;
    double xmin = HUGE_VAL, xmax = -HUGE_VAL, ymin = HUGE_VAL, ymax = -HUGE_VAL;
    for (int i = 0; i < n; i++) {
        if (p[i].bw < xmin) xmin = p[i].bw;
        if (p[i].bw > xmax) xmax = p[i].bw;
        if (p[i].err < ymin) ymin = p[i].err;
        if (p[i].err > ymax) ymax = p[i].err;
    }
    double xr = xmax - xmin, yr = ymax - ymin;
    if (xr <= 0.0 || yr <= 0.0) return -1;                 /* degenerate frontier */
    double ax = (p[0].bw - xmin) / xr, ay = (p[0].err - ymin) / yr;
    double bx = (p[n-1].bw - xmin) / xr, by = (p[n-1].err - ymin) / yr;
    double dx = bx - ax, dy = by - ay, len = hypot(dx, dy);
    if (len <= 0.0) return -1;
    int best = -1;
    double bd = -1.0;
    for (int i = 0; i < n; i++) {
        double px = (p[i].bw - xmin) / xr, py = (p[i].err - ymin) / yr;
        double d = fabs(dx * (py - ay) - dy * (px - ax)) / len;
        if (d > bd) { bd = d; best = i; }
    }
    return best;
}

static int write_csv(const char* path, const Pt* p, int n) {
    FILE* fp = fopen(path, "w");
    if (!fp) return -1;
    fprintf(fp, "freq_hz,bandwidth_bits,error_m\n");
    for (int i = 0; i < n; i++) fprintf(fp, "%.4f,%.1f,%.6f\n", p[i].f, p[i].bw, p[i].err);
    fclose(fp);
    return 0;
}

static int write_svg(const char* path, const Pt* dense, int dn, const Pt* coarse, int cn, int k) {
    FILE* fp = fopen(path, "w");
    if (!fp) return -1;
    double W = 720, H = 480, ML = 80, MR = 30, MT = 30, MB = 60;
    double xmax = dense[dn-1].bw, ymax = dense[0].err;
    fprintf(fp, "<svg xmlns='http://www.w3.org/2000/svg' width='%.0f' height='%.0f'>\n", W, H);
    fprintf(fp, "<rect width='100%%' height='100%%' fill='white'/>\n");
    fprintf(fp, "<line x1='%.0f' y1='%.0f' x2='%.0f' y2='%.0f' stroke='black'/>\n", ML, H-MB, W-MR, H-MB);
    fprintf(fp, "<line x1='%.0f' y1='%.0f' x2='%.0f' y2='%.0f' stroke='black'/>\n", ML, MT, ML, H-MB);
    fprintf(fp, "<text x='%.0f' y='%.0f' font-size='13' text-anchor='middle'>bandwidth (bit/s)</text>\n", (ML+W-MR)/2, H-15);
    fprintf(fp, "<text x='15' y='%.0f' font-size='13' transform='rotate(-90 15 %.0f)' text-anchor='middle'>error (m)</text>\n", (MT+H-MB)/2, (MT+H-MB)/2);
    fprintf(fp, "<text x='%.0f' y='%.0f' font-size='11' text-anchor='middle'>0</text>\n", ML, H-MB+16);
    fprintf(fp, "<text x='%.0f' y='%.0f' font-size='11' text-anchor='middle'>%.0f</text>\n", W-MR, H-MB+16, xmax);
    fprintf(fp, "<text x='%.0f' y='%.0f' font-size='11' text-anchor='end'>%.3f</text>\n", ML-6, MT+10, ymax);
    /* dense (exhaustive) frontier in light gray */
    fprintf(fp, "<polyline fill='none' stroke='#bbb' stroke-width='1.5' points='");
    for (int i = 0; i < dn; i++)
        fprintf(fp, "%.1f,%.1f ", ML + dense[i].bw/xmax*(W-MR-ML), H-MB - dense[i].err/ymax*(H-MB-MT));
    fprintf(fp, "'/>\n");
    /* coarse swept frontier in blue */
    fprintf(fp, "<polyline fill='none' stroke='#2266cc' stroke-width='2' points='");
    for (int i = 0; i < cn; i++)
        fprintf(fp, "%.1f,%.1f ", ML + coarse[i].bw/xmax*(W-MR-ML), H-MB - coarse[i].err/ymax*(H-MB-MT));
    fprintf(fp, "'/>\n");
    for (int i = 0; i < cn; i++)
        fprintf(fp, "<circle cx='%.1f' cy='%.1f' r='3' fill='#2266cc'/>\n",
                ML + coarse[i].bw/xmax*(W-MR-ML), H-MB - coarse[i].err/ymax*(H-MB-MT));
    /* knee in red */
    double kx = ML + coarse[k].bw/xmax*(W-MR-ML), ky = H-MB - coarse[k].err/ymax*(H-MB-MT);
    fprintf(fp, "<circle cx='%.1f' cy='%.1f' r='6' fill='none' stroke='red' stroke-width='2'/>\n", kx, ky);
    fprintf(fp, "<text x='%.1f' y='%.1f' font-size='12' fill='red'>knee f=%.1f Hz</text>\n", kx+10, ky-10, coarse[k].f);
    fclose(fp);
    return 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(int argc, char** argv) {
    static Pt coarse[COARSE_N], dense[DENSE_N], front[DENSE_N];

    check("zero_freq_rejected", sweep(0.0, FMAX, COARSE_N, coarse) == -1);
    check("neg_freq_rejected", sweep(-5.0, FMAX, COARSE_N, coarse) == -1);
    check("bad_range_rejected", sweep(100.0, 100.0, COARSE_N, coarse) == -1);
    check("err_rejects_zero", err_model(0.0) < 0.0);

    check("sweep_ok", sweep(FMIN, FMAX, COARSE_N, coarse) == 0);
    int mono = 1;
    for (int i = 1; i < COARSE_N; i++)
        if (coarse[i].bw <= coarse[i-1].bw || coarse[i].err >= coarse[i-1].err) mono = 0;
    check("frontier_monotone", mono);

    int k = knee(coarse, COARSE_N);
    check("knee_found", k >= 0);
    check("too_few_points", knee(coarse, 2) == -1);        /* caller must warn, see below */

    Pt flat[4] = { {1,1,1}, {2,2,1}, {3,3,1}, {4,4,1} };
    check("degenerate_rejected", knee(flat, 4) == -1);

    Pt syn[3] = { {1,1,1.0}, {2,2,2.0}, {3,3,0.5} };       /* middle point dominated */
    Pt pf[3];
    check("pareto_drops_dominated", pareto(syn, 3, pf) == 2);

    /* exhaustive reference: dense grid knee; coarse knee deviation bounded by
     * one log step of the coarse grid (true knee lies within neighbors) */
    check("dense_ok", sweep(FMIN, FMAX, DENSE_N, dense) == 0);
    int fn = pareto(dense, DENSE_N, front);
    int kd = knee(front, fn);
    check("dense_knee_found", kd >= 0);
    double step = log(FMAX / FMIN) / (COARSE_N - 1);
    double dev = fabs(log(front[kd].f / coarse[k].f));
    check("knee_dev_within_bound", dev <= step + 1e-9);

    if (argc > 1 && strcmp(argv[1], "sweep") == 0) {
        if (COARSE_N < 3) {
            printf("样本点不足 (n=%d < 3)，无法定位拐点；请增加采样点数\n", COARSE_N);
        } else {
            printf("拐点: f=%.2f Hz, 带宽=%.0f bit/s, 误差=%.4f m\n",
                   coarse[k].f, coarse[k].bw, coarse[k].err);
            printf("选择依据: 归一化平面上距端点弦最远的点，再增带宽误差几乎不降\n");
            printf("穷举最优 (%d 点): f=%.2f Hz; 偏差=%.3f (log域), 上界=±%.3f\n",
                   DENSE_N, front[kd].f, dev, step);
        }
        if (write_csv("tuning_sweep.csv", coarse, COARSE_N) == 0 &&
            write_svg("tuning_frontier.svg", front, fn, coarse, COARSE_N, k) == 0)
            printf("wrote tuning_sweep.csv, tuning_frontier.svg\n");
        else
            printf("output_error=1\n");
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
