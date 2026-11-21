
/* sim: weak-network simulator, fixed baseline vs adaptive pipeline.
 *
 * Scenario (150 s, 30 Hz render): good -> degrading -> poor -> full blackout
 * (no RTT/loss readings and no delivered packets) -> poor -> recovery -> good.
 * Both pipelines share the same ground-truth rig and the same packet stream
 * conditions; only the controller differs. Writes per-frame CSV traces.
 */
#include "adapt_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define DT (1.0 / 30.0)
#define SIM_T 150.0
#define WARMUP 2.0
#define RING 8
#define MAX_FLIGHT 256

typedef struct { double st, p, v; int valid; } JSample;
typedef struct { double arrive, st; double p[N_JOINT], v[N_JOINT]; int used[N_JOINT]; } Packet;

typedef struct {
    JSample j[N_JOINT][RING];
    double last_render[N_JOINT];
    int    have_render;
    double last_fresh_core;
} Receiver;

/* ---- deterministic rng ---- */
static unsigned long rng_state = 0x12345678abcdef01UL;
static double randu(void) {
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
    return (rng_state >> 11) * (1.0 / 9007199254740992.0);
}

/* ---- scenario phases: rtt base (s), loss fraction, blackout ---- */
typedef struct { double t0, t1, rtt, loss; int blackout; } Phase;
static const Phase PHASES[] = {
    {   0.0,  20.0, 0.040, 0.005, 0 },
    {  20.0,  45.0, 0.040, 0.010, 0 },   /* rtt/loss ramp applied inside */
    {  45.0,  70.0, 0.220, 0.100, 0 },
    {  70.0,  85.0, 0.220, 1.000, 1 },   /* full blackout: no packets, no readings */
    {  85.0, 110.0, 0.180, 0.080, 0 },
    { 110.0, 130.0, 0.100, 0.030, 0 },
    { 130.0, 150.0, 0.040, 0.005, 0 },
};
#define N_PHASE (int)(sizeof(PHASES) / sizeof(PHASES[0]))

static void net_at(double t, double* rtt, double* loss, int* blackout) {
    const Phase* ph = &PHASES[0];
    for (int i = 0; i < N_PHASE; i++) if (t >= PHASES[i].t0 && t < PHASES[i].t1) { ph = &PHASES[i]; break; }
    *blackout = ph->blackout;
    if (ph->t0 == 20.0) {                 /* linear ramp 40->220 ms, 1%->6% */
        double u = (t - 20.0) / 25.0;
        *rtt = 0.040 + u * (0.220 - 0.040);
        *loss = 0.010 + u * (0.060 - 0.010);
    } else {
        *rtt = ph->rtt; *loss = ph->loss;
    }
    *rtt += (randu() - 0.5) * 0.020;      /* +-10 ms jitter on RTT */
    if (*rtt < 0.005) *rtt = 0.005;
}

static int phase_index(double t) {
    for (int i = 0; i < N_PHASE; i++) if (t < PHASES[i].t1) return i;
    return N_PHASE - 1;
}

static void recv_init(Receiver* r) { memset(r, 0, sizeof(*r)); r->last_fresh_core = -1e9; }

static void recv_push(Receiver* r, int j, double st, double p, double v) {
    for (int k = 0; k < RING; k++) if (!r->j[j][k].valid) {
        r->j[j][k].st = st; r->j[j][k].p = p; r->j[j][k].v = v; r->j[j][k].valid = 1;
        return;
    }
    /* ring full: drop oldest */
    memmove(&r->j[j][0], &r->j[j][1], sizeof(JSample) * (RING - 1));
    r->j[j][RING - 1].st = st; r->j[j][RING - 1].p = p; r->j[j][RING - 1].v = v; r->j[j][RING - 1].valid = 1;
}

/* newest two valid samples with st <= pt; returns count (0/1/2) */
static int recv_bracket(Receiver* r, int j, double pt, JSample* a, JSample* b) {
    int bi = -1, ai = -1;
    for (int k = 0; k < RING; k++) {
        if (!r->j[j][k].valid || r->j[j][k].st > pt) continue;
        if (bi < 0 || r->j[j][k].st > r->j[j][bi].st) { ai = bi; bi = k; }
        else if (ai < 0 || r->j[j][k].st > r->j[j][ai].st) ai = k;
    }
    if (bi < 0) return 0;
    *b = r->j[j][bi];
    if (ai < 0) return 1;
    *a = r->j[j][ai];
    return 2;
}

typedef struct {
    double bw_phase;          /* bytes sent during this phase (uplink, incl. lost) */
    long   frames, stutter, stall, bound_viol;
    double err_sum, err_max, bound_max_observed;
} PhaseStat;

typedef struct {
    Adapt ad;
    Receiver rv;
    Packet flight[MAX_FLIGHT]; int nflight;
    double next_send;
    long   seq;
    int    adaptive;
    double frame_bytes;                 /* bytes enqueued this frame */
    double bw_ring[30]; int bw_n;       /* rolling 1 s bandwidth window */
    double total_bytes;
    PhaseStat ph[N_PHASE];
    PhaseStat tot;
    FILE*  trace;
} Pipe;

static double packet_bytes(double lod) {
    /* header 8 B; 4 B per included joint (quantized pos+vel floats) */
    return 8.0 + 4.0 * N_JOINT * lod;
}

static void pipe_init(Pipe* p, int adaptive, FILE* trace) {
    memset(p, 0, sizeof(*p));
    p->adaptive = adaptive;
    p->trace = trace;
    p->next_send = 0.0;
    adapt_init(&p->ad, 0.0);
    recv_init(&p->rv);
    fprintf(trace, "t,rtt,loss,tier,hz,interp,lod,play_delay,bw_kbps,stutter,stall,mean_err,max_err,bound_max,j0_err,j0_bound,reason\n");
}

static void send_packets(Pipe* p, double now, double loss, int blackout) {
    int tier = p->adaptive ? p->ad.committed : 0;
    const Tier* tp = &TIERS[tier];
    int gate = (int)lrint(1.0 / tp->lod);           /* peripheral every gate-th packet */
    while (p->next_send <= now) {
        double st = p->next_send;
        if (!blackout && randu() >= loss) {
            if (p->nflight < MAX_FLIGHT) {
                Packet* pk = &p->flight[p->nflight++];
                memset(pk, 0, sizeof(*pk));
                pk->st = st;
                for (int j = 0; j < N_JOINT; j++) {
                    int include = JOINTS[j].core || (p->seq % gate == 0);
                    if (include) {
                        pk->used[j] = 1;
                        pk->p[j] = quantize(joint_pos(&JOINTS[j], st), tp->q);
                        pk->v[j] = quantize(joint_vel(&JOINTS[j], st), tp->q);
                    }
                }
                /* one-way delay = half RTT + jitter; arrive later via flight queue */
                double rtt_used; { double r, l; int bo; net_at(st, &r, &l, &bo); rtt_used = r; }
                pk->arrive = now + 0.5 * rtt_used + randu() * 0.010;
            }
        }
        p->frame_bytes += packet_bytes(tp->lod);   /* lost bytes still hit the uplink */
        p->seq++;
        p->next_send += 1.0 / tp->hz;
    }
}

static void deliver(Pipe* p, double now) {
    for (int k = 0; k < p->nflight; ) {
        if (p->flight[k].arrive <= now) {
            Packet* pk = &p->flight[k];
            for (int j = 0; j < N_JOINT; j++)
                if (pk->used[j]) recv_push(&p->rv, j, pk->st, pk->p[j], pk->v[j]);
            p->flight[k] = p->flight[--p->nflight];
        } else k++;
    }
}

/* render one frame; returns per-frame metrics through pointers */
static void render(Pipe* p, double now, double rtt, double loss, int blackout) {
    int tier = p->adaptive ? p->ad.committed : 0;
    const Tier* tp = &TIERS[tier];
    double hz = tp->hz, q = tp->q;
    int    gate = (int)lrint(1.0 / tp->lod);       /* LOD: peripheral every gate-th packet */
    double play_delay = p->adaptive ? adapt_play_delay(&p->ad) : TIERS[0].delay;
    double pt = now - play_delay;
    double lmax = 2.0 / hz;
    double margin = 1e-6;                            /* bound slack against fp rounding */

    int stutter = 0, stall = 0;
    double err_max = 0.0, err_sum = 0.0, bound_max = 0.0;
    double j0_err = 0.0, j0_bound = 0.0;        /* representative core joint */

    for (int j = 0; j < N_JOINT; j++) {
        JSample a, b; int n = recv_bracket(&p->rv, j, pt, &a, &b);
        double rendered, bound, truev = joint_pos(&JOINTS[j], pt);
        double vmax = joint_vmax(&JOINTS[j]), amax = joint_amax(&JOINTS[j]);
        /* LOD-gated joints actually receive a keyframe every gate packets, so
         * their true sampling interval is gate times the nominal one. */
        int jgate = JOINTS[j].core ? 1 : gate;
        int frozen = 0;

        if (n == 0) {                               /* nothing buffered: freeze */
            stall = 1; frozen = 1;
            rendered = p->rv.have_render ? p->rv.last_render[j] : 0.0;
            bound = vmax * fabs(pt) + q;            /* conservative: motion since t0 */
        } else if (n == 1) {
            double age = pt - b.st;
            rendered = b.p;
            bound = error_bound(INTERP_HOLD, age, 0, q, vmax, amax);
            if (age > 1.5 / hz) frozen = 1;
        } else {
            double T = (b.st - a.st) * jgate;       /* real keyframe spacing for this joint */
            double L = pt - b.st;
            if (L <= STALL_EPS) {                   /* inside bracket: interpolate */
                if (tp->interp == INTERP_HERMITE)
                    rendered = hermite_at(a.st, a.p, a.v, b.st, b.p, b.v, pt);
                else if (tp->interp == INTERP_LINEAR)
                    rendered = a.p + (b.p - a.p) * (pt - a.st) / (b.st - a.st);
                else
                    rendered = b.p;
                double qq = (tp->interp == INTERP_HOLD) ? q : 2.0 * q;
                bound = error_bound(tp->interp, T, 0, qq, vmax, amax);
            } else if (L <= lmax) {                 /* beyond newest sample: extrapolate */
                if (tp->interp == INTERP_HERMITE)
                    rendered = hermite_at(a.st, a.p, a.v, b.st, b.p, b.v, pt);
                else if (tp->interp == INTERP_LINEAR)
                    rendered = b.p + b.v * L;
                else
                    rendered = b.p;
                bound = error_bound(tp->interp, T, L, q, vmax, amax);
            } else {                                /* too far ahead: freeze newest */
                rendered = b.p;
                frozen = 1;
                bound = error_bound(INTERP_HOLD, (pt - b.st) * jgate, 0, q, vmax, amax);
            }        }
        bound += margin;

        if (p->rv.have_render) p->rv.last_render[j] = rendered;
        else { p->rv.last_render[j] = rendered; p->rv.have_render = 1; }

        double err = fabs(rendered - truev);
        if (j == 0) { j0_err = err; j0_bound = bound; }
        if (err > err_max) err_max = err;
        err_sum += err;
        if (bound > bound_max) bound_max = bound;
        if (err > bound) p->tot.bound_viol++, p->ph[phase_index(now)].bound_viol++;
        /* core joint 0 defines the visible stutter/stall of the whole rig */
        if (JOINTS[j].core && j == 0) {
            if (frozen) stutter = 1;
            if (n == 0) stall = 1;
            if (n > 0) p->rv.last_fresh_core = b.st;
        }
    }
    if (stall) stutter = 1;

    double mean_err = err_sum / N_JOINT;
    int pi = phase_index(now);
    if (now >= WARMUP) {
        PhaseStat* s = &p->ph[pi];
        s->frames++; s->stutter += stutter; s->stall += stall;
        s->err_sum += mean_err; s->err_max = fmax(s->err_max, err_max);
        s->bound_max_observed = fmax(s->bound_max_observed, bound_max);
        p->tot.frames++; p->tot.stutter += stutter; p->tot.stall += stall;
        p->tot.err_sum += mean_err;
        p->tot.err_max = fmax(p->tot.err_max, err_max);
        p->tot.bound_max_observed = fmax(p->tot.bound_max_observed, bound_max);
    }

    /* bw: kilobits/s over the trailing 1 s window (updated in main loop) */
    double win = 0.0;
    for (int k = 0; k < p->bw_n; k++) win += p->bw_ring[k];
    double bw_kbps = win * 8.0 / 1000.0;
    fprintf(p->trace, "%.3f,%.4f,%.4f,%d,%.1f,%d,%.2f,%.4f,%.2f,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%d\n",
            now, rtt, loss, tier, hz, tp->interp, tp->lod, play_delay, bw_kbps,
            stutter, stall, mean_err, err_max, bound_max, j0_err, j0_bound,
            p->adaptive ? p->ad.reason : REASON_NORMAL);
    (void)blackout;
}

int main(int argc, char** argv) {
    const char* outdir = argc > 1 ? argv[1] : ".";
    char path[256];
    snprintf(path, sizeof(path), "%s/trace_baseline.csv", outdir);
    FILE* fb = fopen(path, "w");
    snprintf(path, sizeof(path), "%s/trace_adaptive.csv", outdir);
    FILE* fa = fopen(path, "w");
    if (!fb || !fa) { perror("open trace"); return 2; }

    Pipe base, adapt;
    pipe_init(&base, 0, fb);
    pipe_init(&adapt, 1, fa);

    FILE* fe;
    snprintf(path, sizeof(path), "%s/fallback_events.csv", outdir);
    fe = fopen(path, "w");
    fprintf(fe, "t_start,t_end,tier,reason\n");
    int miss_active = 0; double miss_start = 0.0;

    long nframes = (long)(SIM_T / DT);
    for (long f = 0; f < nframes; f++) {
        double now = f * DT;
        double rtt, loss; int blackout;
        net_at(now, &rtt, &loss, &blackout);

        /* controller readings: blackout => none; otherwise probes occasionally drop */
        int probe_lost = !blackout && randu() < 0.05;
        double r_in = (blackout || probe_lost) ? -1.0 : rtt;
        double l_in = blackout ? -1.0 : loss;
        adapt_observe(&adapt.ad, now, r_in, l_in);
        adapt_step(&adapt.ad, now);

        int was_miss = miss_active;
        miss_active = (adapt.ad.reason == REASON_MISSING);
        if (miss_active && !was_miss) miss_start = now;
        if (!miss_active && was_miss)
            fprintf(fe, "%.3f,%.3f,%d,missing_readings_conservative_tier2\n",
                    miss_start, now, N_TIER - 1);

        send_packets(&base, now, loss, blackout);
        send_packets(&adapt, now, loss, blackout);
        deliver(&base, now);
        deliver(&adapt, now);

        for (int adp = 0; adp < 2; adp++) {
            Pipe* p = adp ? &adapt : &base;
            if (p->bw_n < 30) p->bw_ring[p->bw_n++] = p->frame_bytes;
            else { memmove(&p->bw_ring[0], &p->bw_ring[1], sizeof(double) * 29);
                   p->bw_ring[29] = p->frame_bytes; }
            p->total_bytes += p->frame_bytes;
            if (now >= WARMUP) p->ph[phase_index(now)].bw_phase += p->frame_bytes;
            p->frame_bytes = 0.0;
        }

        render(&base, now, r_in < 0 ? -1 : rtt, loss, blackout);
        render(&adapt, now, r_in < 0 ? -1 : rtt, loss, blackout);
    }
    if (miss_active)
        fprintf(fe, "%.3f,%.3f,%d,missing_readings_conservative_tier2\n",
                miss_start, SIM_T, N_TIER - 1);
    fclose(fe); fclose(fb); fclose(fa);

    FILE* fs;
    snprintf(path, sizeof(path), "%s/summary.csv", outdir);
    fs = fopen(path, "w");
    fprintf(fs, "scope,pipeline,frames,stutter,stall,stutter_pct,mean_err,max_err,bound_max,bound_viol,bw_kbps_mean\n");
    static const char* PN[N_PHASE] = {"good", "ramp", "poor", "blackout", "poor2", "recovery", "good2"};

    for (int pi = -1; pi < N_PHASE; pi++) {
        for (int adp = 0; adp < 2; adp++) {
            Pipe* p = adp ? &adapt : &base;
            PhaseStat* s = pi < 0 ? &p->tot : &p->ph[pi];
            if (s->frames == 0) continue;
            double dur;
            double bytes;
            if (pi < 0) {
                dur = SIM_T - WARMUP;
                bytes = p->total_bytes;
            } else {
                double t0 = fmax(PHASES[pi].t0, WARMUP), t1 = PHASES[pi].t1;
                dur = t1 - t0;
                bytes = s->bw_phase;
            }
            fprintf(fs, "%s,%s,%ld,%ld,%ld,%.2f,%.5f,%.5f,%.5f,%ld,%.2f\n",
                    pi < 0 ? "total" : PN[pi],
                    adp ? "adaptive" : "baseline",
                    s->frames, s->stutter, s->stall,
                    100.0 * s->stutter / s->frames,
                    s->err_sum / s->frames, s->err_max, s->bound_max_observed,
                    s->bound_viol, bytes * 8.0 / 1000.0 / dur);
        }
    }
    fclose(fs);

    printf("sim_done frames=%ld\n", nframes);
    printf("baseline: stutter=%ld (%.2f%%) stall=%ld max_err=%.4f bound_viol=%ld\n",
           base.tot.stutter, 100.0 * base.tot.stutter / base.tot.frames,
           base.tot.stall, base.tot.err_max, base.tot.bound_viol);
    printf("adaptive: stutter=%ld (%.2f%%) stall=%ld max_err=%.4f bound_viol=%ld\n",
           adapt.tot.stutter, 100.0 * adapt.tot.stutter / adapt.tot.frames,
           adapt.tot.stall, adapt.tot.err_max, adapt.tot.bound_viol);
    return 0;
}
