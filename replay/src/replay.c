
/* replay: input replay of trimmed packet captures with clock alignment.
 *
 * Problem: input replay and trimming use different time bases. Trajectory
 * samples and packet summaries are timestamped on the remote clock, while
 * timing records are timestamped on the local (capture) clock. The legacy
 * replay applies every surviving packet at its local receive time, so after
 * trimming at a remote tick the remote position jumps from the last pre-cut
 * position to the first post-cut position, and receive jitter shifts every
 * subsequent update.
 *
 * Two replay paths share one renderer contract:
 *   replay_naive   - legacy semantics: trim by remote tick, apply at local
 *                    receive time, snap/hold positions (jumps allowed).
 *   replay_aligned - offset-align remote and local clocks (min-RTT filter,
 *                    same estimator as clock/), trim on the local timeline,
 *                    split the straddling packet at the cut via trajectory
 *                    interpolation, and chase every target with a speed cap
 *                    derived from the sampled trajectory (no jumps).
 *
 * replay_naive() is byte-for-byte stable: running it twice on the same input
 * yields identical frames, and it never touches the alignment code path.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define MAX_TRAJ   32
#define MAX_PKT    32
#define MAX_SYNC   16
#define MAX_FRAMES 128
#define TICK_DT    0.05      /* seconds per remote tick */
#define POS_SCALE  64.0f     /* position quantization, matches snapshot/ */
#define SPEED_GUARD 1.05     /* chase may run 5% faster than the sample max */
#define TRAIL_FRAMES 4       /* hold frames after the last target for convergence */

/* ---- material 1: trajectory samples (remote clock) --------------------- */

typedef struct { double t; float x, y; } TSample;

typedef struct {
    TSample s[MAX_TRAJ];
    int n;
    float vmax;             /* max segment speed observed in the samples */
} Traj;

/* ---- material 2: timing records (local clock) -------------------------- */

typedef struct { double t_send, t_recv, remote_time; } TRec;

typedef struct {
    TRec s[MAX_SYNC];
    int n;
    double best_rtt;
    double offset;          /* remote = local + offset */
} Sync;

/* ---- material 3: packet summaries -------------------------------------- */

typedef struct {
    unsigned seq, tick_lo, tick_hi, body_len;
    int qx, qy;             /* quantized endpoint position (zigzag on wire) */
    unsigned crc32;         /* FNV-1a over the 24-byte canonical record */
} Pkt;

typedef struct { double t; float x, y; int seq; } Frame;

/* ================= clock offset (same estimator as clock/) ============== */

static void sync_init(Sync* sy) { memset(sy, 0, sizeof(*sy)); sy->best_rtt = 1e9; }

static int sync_add(Sync* sy, double t_send, double t_recv, double remote_time) {
    if (t_recv < t_send) return -1;
    double rtt = t_recv - t_send;
    double off = remote_time - (t_send + t_recv) * 0.5;
    if (sy->n >= MAX_SYNC) {
        memmove(&sy->s[0], &sy->s[1], sizeof(TRec) * (MAX_SYNC - 1));
        sy->n = MAX_SYNC - 1;
    }
    sy->s[sy->n].t_send = t_send;
    sy->s[sy->n].t_recv = t_recv;
    sy->s[sy->n].remote_time = remote_time;
    sy->n++;
    if (rtt < sy->best_rtt) sy->best_rtt = rtt;

    double sum = 0.0; int cnt = 0;
    for (int i = 0; i < sy->n; i++) {
        double r = sy->s[i].t_recv - sy->s[i].t_send;
        if (r <= sy->best_rtt * 1.5 + 1e-9) { sum += off; cnt++; }
    }
    if (cnt == 0) return -1;
    sy->offset = sum / cnt;
    return cnt;
}

/* ================= trajectory =========================================== */

static int traj_build(Traj* tr, const TSample* in, int n) {
    if (n < 1 || n > MAX_TRAJ) return -1;
    tr->n = n;
    tr->vmax = 0.0f;
    for (int i = 0; i < n; i++) {
        if (i > 0 && in[i].t <= in[i - 1].t) return -1;   /* monotonic remote time */
        tr->s[i] = in[i];
        if (i > 0) {
            float dx = in[i].x - in[i - 1].x, dy = in[i].y - in[i - 1].y;
            float v = (float)(sqrtf(dx * dx + dy * dy) / (in[i].t - in[i - 1].t));
            if (v > tr->vmax) tr->vmax = v;
        }
    }
    return 0;
}

/* piecewise-linear position at remote time t; clamps outside the sample range */
static void traj_at(const Traj* tr, double t, float* x, float* y) {
    if (t <= tr->s[0].t) { *x = tr->s[0].x; *y = tr->s[0].y; return; }
    if (t >= tr->s[tr->n - 1].t) {
        *x = tr->s[tr->n - 1].x; *y = tr->s[tr->n - 1].y; return;
    }
    for (int i = 1; i < tr->n; i++) {
        if (t <= tr->s[i].t) {
            double a = (t - tr->s[i - 1].t) / (tr->s[i].t - tr->s[i - 1].t);
            *x = tr->s[i - 1].x + (float)a * (tr->s[i].x - tr->s[i - 1].x);
            *y = tr->s[i - 1].y + (float)a * (tr->s[i].y - tr->s[i - 1].y);
            return;
        }
    }
}

/* ================= packet body summary check ============================ */

/* unsigned zigzag of a signed quantized coordinate (well-defined on negatives) */
static uint32_t zigzag32(int v) {
    return ((uint32_t)v << 1) ^ (uint32_t)(v >> 31);
}

static unsigned fnv1a_canon(const Pkt* p) {
    uint32_t h = 0x811c9dc5u;
    uint32_t words[6];
    words[0] = p->seq;
    words[1] = p->tick_lo;
    words[2] = p->tick_hi;
    words[3] = p->body_len;
    words[4] = zigzag32(p->qx);
    words[5] = zigzag32(p->qy);
    const unsigned char* b = (const unsigned char*)words;
    for (int i = 0; i < 24; i++) { h ^= b[i]; h *= 0x01000193u; }
    return h;
}

/* 0 = intact, -1 = checksum mismatch (caller must drop the packet) */
static int pkt_verify(const Pkt* p) { return fnv1a_canon(p) == p->crc32 ? 0 : -1; }

static void pkt_pos(const Pkt* p, float* x, float* y) {
    *x = p->qx / POS_SCALE;
    *y = p->qy / POS_SCALE;
}

/* ================= legacy path: jumps allowed, no alignment ============= */

static int frame_index(double t_event, double t0) {
    return (int)lround((t_event - t0) / TICK_DT);
}

static int replay_naive(const Pkt* pkts, const double* recv, int n, unsigned cut,
                        Frame* out, int cap, int* bad) {
    *bad = 0;
    float hx = 0.0f, hy = 0.0f;
    int have_post = 0;

    /* hold the endpoint of the last fully pre-cut packet, as before */
    for (int i = 0; i < n; i++) {
        if (pkt_verify(&pkts[i]) != 0) { (*bad)++; continue; }
        if (pkts[i].tick_hi <= cut) pkt_pos(&pkts[i], &hx, &hy);
        else have_post = 1;
    }
    if (!have_post) return 0;

    /* earliest receive time among post-cut packets starts the grid */
    double t0 = 1e300;
    for (int i = 0; i < n; i++) {
        if (pkt_verify(&pkts[i]) != 0) continue;
        if (pkts[i].tick_hi > cut && recv[i] < t0) t0 = recv[i];
    }

    int max_idx = 0;
    for (int i = 0; i < n; i++) {
        if (pkt_verify(&pkts[i]) != 0) continue;
        if (pkts[i].tick_hi <= cut) continue;
        int idx = frame_index(recv[i], t0);
        if (idx > max_idx) max_idx = idx;
    }
    int nf = max_idx + 2;                  /* one held frame before the first snap */
    if (nf > cap) return -1;
    for (int f = 0; f < nf; f++) {
        out[f].t = t0 + (double)(f - 1) * TICK_DT;
        out[f].x = hx; out[f].y = hy; out[f].seq = 0;
    }
    for (int i = 0; i < n; i++) {
        if (pkt_verify(&pkts[i]) != 0) continue;
        if (pkts[i].tick_hi <= cut) continue;
        int idx = frame_index(recv[i], t0);
        float x, y; pkt_pos(&pkts[i], &x, &y);
        for (int f = idx + 1; f < nf; f++) { out[f].x = x; out[f].y = y; out[f].seq = pkts[i].seq; }
    }
    return nf;
}

/* ================= aligned path: no jumps =============================== */

typedef struct { double t; float x, y; int seq; } Target;

static int replay_aligned(const Pkt* pkts, int n, unsigned cut,
                          const Traj* tr, const Sync* sy,
                          double epoch, Frame* out, int cap, int* bad) {
    *bad = 0;
    Target tg[MAX_PKT]; int m = 0;
    for (int i = 0; i < n; i++) {
        if (pkt_verify(&pkts[i]) != 0) { (*bad)++; continue; }
        if (pkts[i].tick_hi <= cut) continue;          /* trimmed away, never a target */
        double rt = epoch + pkts[i].tick_hi * TICK_DT;
        tg[m].t = rt - sy->offset;                     /* remote event on local timeline */
        pkt_pos(&pkts[i], &tg[m].x, &tg[m].y);
        tg[m].seq = (int)pkts[i].seq;
        m++;
    }
    if (m == 0) return 0;

    double t_cut = epoch + (double)cut * TICK_DT - sy->offset;
    float cx = 0.0f, cy = 0.0f;
    traj_at(tr, epoch + (double)cut * TICK_DT, &cx, &cy);   /* split the straddling packet */

    double end = tg[0].t;
    for (int i = 1; i < m; i++) if (tg[i].t > end) end = tg[i].t;
    /* trailing hold frames let the speed-capped chase settle on the last target */
    int nf = frame_index(end, t_cut) + 1 + TRAIL_FRAMES;
    if (nf > cap) return -1;

    float vmax = tr->vmax * SPEED_GUARD;
    int ti = 0;
    for (int f = 0; f < nf; f++) {
        double t = t_cut + (double)f * TICK_DT;
        while (ti < m && tg[ti].t <= t + 1e-9) ti++;        /* packets due by now */
        out[f].t = t;
        out[f].seq = ti > 0 ? tg[ti - 1].seq : 0;
        if (f == 0) { out[f].x = cx; out[f].y = cy; continue; }
        float tx, ty;
        if (ti == 0) { tx = cx; ty = cy; }                  /* no target yet: hold cut entry */
        else { tx = tg[ti - 1].x; ty = tg[ti - 1].y; }
        float dx = tx - out[f - 1].x, dy = ty - out[f - 1].y;
        float d = sqrtf(dx * dx + dy * dy);
        float step = vmax > 0.0f ? vmax * (float)TICK_DT : d;
        if (d <= step || step == 0.0f) { out[f].x = tx; out[f].y = ty; }
        else { float a = step / d; out[f].x = out[f - 1].x + dx * a;
               out[f].y = out[f - 1].y + dy * a; }
    }
    return nf;
}

/* ================= fixture + self test ================================== */

static float max_step(const Frame* f, int n) {
    float m = 0.0f;
    for (int i = 1; i < n; i++) {
        float dx = f[i].x - f[i - 1].x, dy = f[i].y - f[i - 1].y;
        float d = sqrtf(dx * dx + dy * dy);
        if (d > m) m = d;
    }
    return m;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* fixture: 20 Hz ticks, true remote offset +2.0s, 8 packets of 5 ticks,
 * packet 4 carries +30 ms receive jitter. Generated by materials/gen.py. */
#define EPOCH 100.0
#define TRUE_OFFSET 2.0

static const TSample traj_in[11] = {
    {100.00, 0.0f,    0.000000f}, {100.20, 1.0f, 0.071736f},
    {100.40, 2.0f,    0.099957f}, {100.60, 3.0f, 0.067546f},
    {100.80, 4.0f,   -0.005837f}, {101.00, 5.0f,-0.075680f},
    {101.20, 6.0f,   -0.099616f}, {101.40, 7.0f,-0.063127f},
    {101.60, 8.0f,    0.011655f}, {101.80, 9.0f, 0.079367f},
    {102.00, 10.0f,   0.098936f}
};

/* parallel arrays: timing records and packet summaries share index i */
static const TRec recs[8] = {
    {98.175, 98.225, 100.20}, {98.420, 98.480, 100.45},
    {98.665, 98.735, 100.70}, {98.930, 99.000, 100.95},
    {99.175, 99.225, 101.20}, {99.420, 99.480, 101.45},
    {99.665, 99.735, 101.70}, {99.930, 99.970, 101.95}
};

static Pkt pkts[8] = {
    {1, 0,  4, 28,  64,  5, 0x578b7db6u}, {2, 5,  9, 32, 144,  6, 0x436ac32cu},
    {3, 10, 14, 36, 224,  2, 0x8e2a1f45u}, {4, 15, 19, 40, 304, -4, 0xc7e616f8u},
    {5, 20, 24, 44, 384, -6, 0xc3a7706au}, {6, 25, 29, 48, 464, -3, 0xffcb3c37u},
    {7, 30, 34, 52, 544,  3, 0xd25d5e98u}, {8, 35, 39, 56, 624,  6, 0x3b828ea1u}
};

static void print_csv(const char* tag, const Frame* f, int n) {
    printf("# %s t,x,y,seq\n", tag);
    for (int i = 0; i < n; i++)
        printf("%.4f,%.6f,%.6f,%d\n", f[i].t, f[i].x, f[i].y, f[i].seq);
}

int main(int argc, char** argv) {
    double recv[8];
    for (int i = 0; i < 8; i++) recv[i] = recs[i].t_recv;

    Traj tr;
    check("traj_build", traj_build(&tr, traj_in, 11) == 0);
    check("traj_vmax", fabsf(tr.vmax - 5.013961f) < 1e-3f);

    int crc_ok = 1;
    for (int i = 0; i < 8; i++) if (pkt_verify(&pkts[i]) != 0) crc_ok = 0;
    check("crc_all_intact", crc_ok);
    Pkt tampered = pkts[4];
    tampered.qx += 1;
    check("crc_tamper_detected", pkt_verify(&tampered) == -1);

    Sync sy; sync_init(&sy);
    int added = 0;
    for (int i = 0; i < 8; i++) added = sync_add(&sy, recs[i].t_send, recs[i].t_recv, recs[i].remote_time);
    check("sync_feed", added > 0);
    check("sync_offset", fabs(sy.offset - TRUE_OFFSET) < 1e-9);   /* jittered pkt filtered out */
    check("sync_min_rtt", fabs(sy.best_rtt - 0.04) < 1e-9);

    Frame fa[MAX_FRAMES], fb[MAX_FRAMES], fc[MAX_FRAMES];
    int bad_n = 0, bad_a = 0;
    int nn = replay_naive(pkts, recv, 8, 12, fa, MAX_FRAMES, &bad_n);
    int na = replay_aligned(pkts, 8, 12, &tr, &sy, EPOCH, fb, MAX_FRAMES, &bad_a);
    check("naive_frames", nn == 27);
    check("aligned_frames", na == 32);

    /* kill switch: alignment off is deterministic and self-identical */
    int nn2 = replay_naive(pkts, recv, 8, 12, fc, MAX_FRAMES, &bad_n);
    check("off_path_stable", nn2 == nn && memcmp(fa, fc, sizeof(Frame) * (size_t)nn) == 0);

    /* aligned timeline: first frame sits exactly on the cut, split point */
    check("cut_on_grid", fabs(fb[0].t - 98.60) < 1e-9);
    check("cut_entry_x", fabsf(fb[0].x - 3.0f) < 1e-5f);
    check("cut_entry_y", fabsf(fb[0].y - 0.067546f) < 1e-4f);

    /* no jump: aligned steps bounded by the speed cap; legacy jumps hard */
    float step_a = max_step(fa, nn), step_b = max_step(fb, na);
    check("legacy_jumps", step_a > 1.0f);                 /* 1.25 held->snap jump */
    float bound = tr.vmax * SPEED_GUARD * (float)TICK_DT + 1.0f / POS_SCALE;
    check("aligned_no_jump", step_b <= bound + 1e-6f);

    /* trimmed packets never drive a post-cut target */
    int seq2_seen = 0;
    for (int i = 0; i < na; i++) if (fb[i].seq == 2) seq2_seen = 1;
    check("trim_respected", !seq2_seen);

    /* jittered packet 4: aligned frame 7 (its true remote time); naive frame 6
     * (30 ms receive delay pushes it, and the naive grid has a held prefix) */
    int a4 = -1, n4 = -1;
    for (int i = 0; i < na; i++) if (fb[i].seq == 4) { a4 = i; break; }
    for (int i = 0; i < nn; i++) if (fa[i].seq == 4) { n4 = i; break; }
    check("jitter_realigned", a4 == 7 && n4 == 6);

    /* convergence: the trailing hold frames let the chase land on the target */
    float fx, fy; pkt_pos(&pkts[7], &fx, &fy);
    float tail = sqrtf((fb[na - 1].x - fx) * (fb[na - 1].x - fx) +
                       (fb[na - 1].y - fy) * (fb[na - 1].y - fy));
    check("converges", tail < 1e-5f);

    /* corrupted packet is skipped, replay stays continuous */
    Pkt broken[8]; memcpy(broken, pkts, sizeof(pkts));
    broken[4].crc32 ^= 0xdeadbeefu;
    int bad2 = 0;
    Frame fd[MAX_FRAMES];
    int nd = replay_aligned(broken, 8, 12, &tr, &sy, EPOCH, fd, MAX_FRAMES, &bad2);
    check("bad_pkt_counted", bad2 == 1 && nd > 0);
    check("bad_pkt_no_jump", max_step(fd, nd) <= bound + 1e-6f);

    if (argc > 1 && strcmp(argv[1], "csv") == 0) {
        print_csv("naive", fa, nn);
        print_csv("aligned", fb, na);
    }
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
