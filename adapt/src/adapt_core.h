
/* adapt_core: shared types/declarations for the adaptive degradation module. */
#ifndef ADAPT_CORE_H
#define ADAPT_CORE_H

#define N_TIER 3
#define N_JOINT 12
#define ALPHA_EWMA 0.1            /* network reading EWMA */
#define SMOOTH_TAU 0.25           /* playout-delay lag time constant, s */
#define DWELL_UP 2.0              /* better-tier evidence required before upgrading */
#define DWELL_DOWN 0.5            /* downgrade fast, but filter sub-dwell blips */
#define MISS_GRACE 1.0            /* blackout tolerated before conservative fallback */
#define STALL_EPS 1e-3
#define PI 3.14159265358979323846

enum { INTERP_HERMITE = 0, INTERP_LINEAR = 1, INTERP_HOLD = 2 };
enum { REASON_NORMAL = 0, REASON_MISSING = 1 };

typedef struct {
    double hz;        /* send/update rate */
    int    interp;    /* INTERP_* */
    double lod;       /* peripheral refresh fraction (1/k), 1.0 = every packet */
    double delay;     /* playout delay, s */
    double q;         /* quantization half-step, rad */
} Tier;

extern const Tier TIERS[N_TIER];
extern const double RTT_CEIL[N_TIER];
extern const double LOSS_CEIL[N_TIER];

typedef struct {
    double rtt_ema, loss_ema;
    int    have_rtt, have_loss;
    double last_valid_at;
    int    target, committed;
    int    cand;             /* candidate tier waiting out its dwell */
    double cand_since;       /* when the candidate first appeared */
    int    primed;           /* first evidence commits immediately */
    int    reason;
    double last_t;            /* previous adapt_step time, for lag dt */
    double delay_s;          /* smoothed playout delay (continuous knob) */
} Adapt;

void   adapt_init(Adapt* a, double now);
void   adapt_observe(Adapt* a, double now, double rtt, double loss);
int    adapt_step(Adapt* a, double now);       /* returns 1 if committed tier changed */
double adapt_play_delay(const Adapt* a);

typedef struct { double amp, freq; int core; } JointCfg;
extern const JointCfg JOINTS[N_JOINT];

double joint_pos(const JointCfg* j, double t);
double joint_vel(const JointCfg* j, double t);
double joint_vmax(const JointCfg* j);
double joint_amax(const JointCfg* j);
float  quantize(double v, double q);
double hermite_at(double ta, double pa, double va, double tb, double pb, double vb, double t);

/* analytic per-joint |rendered-true| upper bound, rad */
double error_bound(int interp, double T, double L, double q, double vmax, double amax);

#endif
