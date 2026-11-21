
/* interest: relevance culling vs full broadcast.
 *
 *  - AOI radius + line-of-sight visibility culling
 *  - frequency tiers: near every tick, mid every 2nd, out-of-bounds every 4th
 *  - enter/leave diff events, immediate sync on enter
 *  - byte-accurate wire model (varint tick, zigzag quantized position)
 *  - held-position culled client compared tick-by-tick with the
 *    full-broadcast client; analytic position-error upper bound.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_P   48
#define MAX_OBS 8
#define ROOM    100.0f      /* playable room: [0, ROOM]^2 */
#define ROAM_LO (-20.0f)    /* OOB walker may wander this far outside */
#define ROAM_HI 120.0f
#define CROWD_LO 22.0f      /* random-walk crowd stays in the middle box */
#define CROWD_HI 82.0f
#define AOI_R   30.0f
#define NEAR_R  15.0f
#define DT      0.05f       /* 20 Hz tick */
#define VMAX    6.0f        /* max entity speed, units/s */
#define SCALE   32.0f       /* position quantization scale */
#define TICKS   240
#define HDR     2           /* per-batch packet header bytes */
#define EV_BYTES 2          /* enter/leave event: id + op */

typedef enum { TIER_NEAR = 0, TIER_MID = 1, TIER_OOB = 2, TIER_N = 3 } Tier;
static const int tier_interval[TIER_N] = { 1, 2, 4 };
static const char* tier_name[TIER_N] = { "near", "mid", "oob" };

typedef struct { int id; float x, y, vx, vy; } Ent;
typedef struct { float x, y, r; } Obs;

typedef struct {
    int prev[MAX_P], prevn;          /* visible set last tick */
    int last[MAX_P];                 /* last sent tick, -1 = never */
    int known[MAX_P];                /* culled client holds an image */
    float cx[MAX_P], cy[MAX_P];      /* culled client image */
    float fx[MAX_P], fy[MAX_P];      /* full-broadcast client image */
    int empty_ticks, first_empty, last_empty;
} View;

typedef struct {
    Ent e[MAX_P];
    int n;
    Obs obs[MAX_OBS];
    int no;
    View v[MAX_P];
    long bytes_full, bytes_cull;
    long tier_bytes[TIER_N];
    int gap_seen[5];                 /* send-gap histogram, index 1..4 */
    int enters, leaves;
    float err_max;
    unsigned rng;
} Sim;

/* ---- wire model, same encoding both clients so the comparison is fair ---- */

static int varint_len(unsigned v) {
    int n = 1;
    while (v >= 0x80) { n++; v >>= 7; }
    return n;
}

static int quant_q(float v) { return (int)lrintf(v * SCALE); }
static float quant_deq(int q) { return (float)q / SCALE; }

static int zigzag_len(int q) {
    unsigned z = (unsigned)((q << 1) ^ (q >> 31));
    return varint_len(z);
}

/* id (1 byte) + varint tick + zigzag varint qx + zigzag varint qy */
static int update_bytes(unsigned tick, float x, float y) {
    return 1 + varint_len(tick) + zigzag_len(quant_q(x)) + zigzag_len(quant_q(y));
}

/* ---- geometry / classification ---- */

static int in_room(float x, float y) {
    return x >= 0.0f && x <= ROOM && y >= 0.0f && y <= ROOM;
}

static Tier tier_of(const Ent* e, float dist) {
    if (!in_room(e->x, e->y)) return TIER_OOB;
    return dist <= NEAR_R ? TIER_NEAR : TIER_MID;
}

static float point_seg_dist(float px, float py, float ax, float ay,
                            float bx, float by) {
    float dx = bx - ax, dy = by - ay;
    float l2 = dx * dx + dy * dy;
    float t = l2 > 0.0f ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float cx = ax + t * dx, cy = ay + t * dy;
    return hypotf(px - cx, py - cy);
}

static int los_blocked(const Sim* s, float ax, float ay, float bx, float by) {
    for (int i = 0; i < s->no; i++) {
        const Obs* o = &s->obs[i];
        if (point_seg_dist(o->x, o->y, ax, ay, bx, by) < o->r) return 1;
    }
    return 0;
}

static int visible(const Sim* s, int viewer, int target) {
    const Ent* a = &s->e[viewer];
    const Ent* b = &s->e[target];
    float d = hypotf(b->x - a->x, b->y - a->y);
    if (d > AOI_R) return 0;
    if (los_blocked(s, a->x, a->y, b->x, b->y)) return 0;
    return 1;
}

static int in_list(const int* a, int n, int id) {
    for (int i = 0; i < n; i++) if (a[i] == id) return 1;
    return 0;
}

/* ---- deterministic movement ---- */

static unsigned lcg(unsigned* st) {
    *st = *st * 1103515245u + 12345u;
    return *st;
}

static float lcgf(unsigned* st) { return (lcg(st) >> 8) / (float)(1u << 24); }

static void bounce(Ent* e, float lo, float hi) {
    if (e->x < lo) { e->x = lo; e->vx = fabsf(e->vx); }
    if (e->x > hi) { e->x = hi; e->vx = -fabsf(e->vx); }
    if (e->y < lo) { e->y = lo; e->vy = fabsf(e->vy); }
    if (e->y > hi) { e->y = hi; e->vy = -fabsf(e->vy); }
}

static void sim_move(Sim* s, int tick) {
    for (int i = 0; i < s->n; i++) {
        Ent* e = &s->e[i];
        if (i == 0) {                       /* scripted OOB walker */
            e->x += e->vx * DT;
            bounce(e, ROAM_LO, ROAM_HI);
        } else if (i >= 4) {                /* random-walk crowd */
            if (tick % 40 == 0) {
                float ang = 6.2831853f * lcgf(&s->rng);
                float sp = 2.0f + 4.0f * lcgf(&s->rng);   /* <= VMAX */
                e->vx = sp * cosf(ang);
                e->vy = sp * sinf(ang);
            }
            e->x += e->vx * DT;
            e->y += e->vy * DT;
            bounce(e, CROWD_LO, CROWD_HI);
        }
        /* ids 1..3 are frozen anchors / isolated viewers */
    }
}

/* ---- one simulation tick ---- */

static void sim_tick(Sim* s, int tick) {
    sim_move(s, tick);

    for (int v = 0; v < s->n; v++) {
        View* vw = &s->v[v];
        int vis[MAX_P], vn = 0;
        for (int j = 0; j < s->n; j++)
            if (j != v && visible(s, v, j)) vis[vn++] = j;

        if (vn == 0) {
            vw->empty_ticks++;
            if (vw->first_empty == 0) vw->first_empty = tick;
            vw->last_empty = tick;
        }

        long batch = 0;

        /* enter / leave diff against last tick's visible set */
        for (int k = 0; k < vn; k++) {
            int j = vis[k];
            if (!in_list(vw->prev, vw->prevn, j)) {
                s->enters++;
                batch += EV_BYTES;
                vw->last[j] = -1;                       /* force immediate sync */
            }
        }
        for (int k = 0; k < vw->prevn; k++) {
            int j = vw->prev[k];
            if (!in_list(vis, vn, j)) {
                s->leaves++;
                batch += EV_BYTES;
                vw->known[j] = 0;
                vw->last[j] = -1;
            }
        }

        /* tiered update scheduling */
        for (int k = 0; k < vn; k++) {
            int j = vis[k];
            Ent* b = &s->e[j];
            float d = hypotf(b->x - s->e[v].x, b->y - s->e[v].y);
            Tier tr = tier_of(b, d);
            int iv = tier_interval[tr];
            if (vw->last[j] < 0 || tick - vw->last[j] >= iv) {
                if (vw->last[j] > 0) {
                    int gap = tick - vw->last[j];
                    if (gap >= 1 && gap <= 4) s->gap_seen[gap]++;
                }
                int nb = update_bytes((unsigned)tick, b->x, b->y);
                batch += nb;
                s->tier_bytes[tr] += nb;
                vw->last[j] = tick;
                vw->known[j] = 1;
                vw->cx[j] = quant_deq(quant_q(b->x));
                vw->cy[j] = quant_deq(quant_q(b->y));
            }
        }
        if (batch > 0) s->bytes_cull += batch + HDR;

        /* baseline: full broadcast of every other entity every tick */
        long fb = 0;
        for (int j = 0; j < s->n; j++) {
            if (j == v) continue;
            Ent* b = &s->e[j];
            fb += update_bytes((unsigned)tick, b->x, b->y);
            vw->fx[j] = quant_deq(quant_q(b->x));
            vw->fy[j] = quant_deq(quant_q(b->y));
        }
        s->bytes_full += fb + HDR;

        /* client-state divergence: culled image vs full-broadcast image */
        for (int k = 0; k < vn; k++) {
            int j = vis[k];
            if (!vw->known[j]) continue;
            float err = hypotf(vw->cx[j] - vw->fx[j], vw->cy[j] - vw->fy[j]);
            if (err > s->err_max) s->err_max = err;
        }

        memcpy(vw->prev, vis, sizeof(int) * (size_t)vn);
        vw->prevn = vn;
    }
}

static void sim_setup(Sim* s) {
    memset(s, 0, sizeof(*s));
    s->rng = 42u;
    s->n = 36;
    for (int v = 0; v < s->n; v++)
        for (int j = 0; j < MAX_P; j++) s->v[v].last[j] = -1;

    /* id 0: walks straight out of the room while staying in id 1's AOI */
    s->e[0] = (Ent){ 0, 12.0f, 10.0f, -3.0f, 0.0f };
    /* id 1: anchored viewer next to the door */
    s->e[1] = (Ent){ 1, 10.0f, 10.0f, 0.0f, 0.0f };
    /* id 2/3: isolated outside the room, nobody inside AOI — every tick empty */
    s->e[2] = (Ent){ 2, -12.0f, 112.0f, 0.0f, 0.0f };
    s->e[3] = (Ent){ 3, 112.0f, -12.0f, 0.0f, 0.0f };

    /* ids 4..35: jittered 6x6 grid inside [24,76]^2 */
    for (int i = 4; i < s->n; i++) {
        int g = i - 4, gx = g % 6, gy = g / 6;
        float jx = (lcgf(&s->rng) - 0.5f) * 2.0f;
        float jy = (lcgf(&s->rng) - 0.5f) * 2.0f;
        s->e[i] = (Ent){ i, 25.0f + gx * 10.0f + jx,
                              25.0f + gy * 10.0f + jy, 0.0f, 0.0f };
    }

    s->no = 3;
    s->obs[0] = (Obs){ 50.0f, 50.0f, 6.0f };
    s->obs[1] = (Obs){ 35.0f, 62.0f, 5.0f };
    s->obs[2] = (Obs){ 65.0f, 38.0f, 5.0f };
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* focused unit checks */
    check("varint_len", varint_len(0) == 1 && varint_len(127) == 1 &&
                        varint_len(128) == 2 && varint_len(16384) == 3);
    check("in_room_edge", in_room(0.0f, ROOM) && !in_room(-0.01f, 50.0f));

    Ent near = { 9, 5.0f, 0.0f, 0, 0 }, far = { 9, 25.0f, 0, 0, 0 };
    Ent oob = { 9, -5.0f, 0.0f, 0, 0 };
    check("tier_near", tier_of(&near, 5.0f) == TIER_NEAR);
    check("tier_mid", tier_of(&far, 25.0f) == TIER_MID);
    check("tier_oob", tier_of(&oob, 5.0f) == TIER_OOB &&
                      tier_interval[TIER_OOB] == 4);

    Sim s;
    sim_setup(&s);
    check("los_clear", !los_blocked(&s, 10.0f, 10.0f, 12.0f, 10.0f));
    check("los_blocked", los_blocked(&s, 44.0f, 50.0f, 56.0f, 50.0f));
    check("aoi_sees_near", visible(&s, 1, 0));
    check("aoi_rejects_far", !visible(&s, 1, 2));

    for (int t = 1; t <= TICKS; t++) sim_tick(&s, t);

    /* analytic bound: held position drifts at most VMAX over the
     * coarsest interval; both clients quantize once (sqrt(2)/(2*S) each). */
    float q_each = sqrtf(2.0f) / (2.0f * SCALE);
    float drift = VMAX * DT * (float)tier_interval[TIER_OOB];
    float err_bound = drift + 2.0f * q_each;

    long total_tier = 0;
    for (int i = 0; i < TIER_N; i++) total_tier += s.tier_bytes[i];
    float reduction = 100.0f * (1.0f - (double)s.bytes_cull /
                                         (double)s.bytes_full);

    printf("--- metrics ---\n");
    printf("players=%d ticks=%d aoi_r=%.0f room=%.0fx%.0f\n",
           s.n, TICKS, AOI_R, ROOM, ROOM);
    printf("bytes_full=%ld\n", s.bytes_full);
    printf("bytes_cull=%ld\n", s.bytes_cull);
    for (int i = 0; i < TIER_N; i++)
        printf("tier_bytes_%s=%ld\n", tier_name[i], s.tier_bytes[i]);
    printf("tier_bytes_total=%ld\n", total_tier);
    printf("enter_events=%d leave_events=%d\n", s.enters, s.leaves);
    printf("send_gap_1=%d send_gap_2=%d send_gap_4=%d\n",
           s.gap_seen[1], s.gap_seen[2], s.gap_seen[4]);
    printf("err_max_measured=%.4f\n", s.err_max);
    printf("err_drift_bound=%.4f err_quant_2x=%.4f err_bound=%.4f\n",
           drift, 2.0f * q_each, err_bound);
    printf("bandwidth_reduction_pct=%.2f\n", reduction);

    printf("--- empty region report ---\n");
    int reported = 0;
    for (int v = 0; v < s.n; v++) {
        if (s.v[v].empty_ticks == 0) continue;
        reported++;
        printf("empty_region viewer=%d ticks=%d first=%d last=%d\n",
               v, s.v[v].empty_ticks, s.v[v].first_empty, s.v[v].last_empty);
    }
    if (reported == 0) printf("empty_region none\n");

    printf("--- assertions ---\n");
    check("enter_leave_seen", s.enters > 0 && s.leaves > 0);
    check("oob_tier_exercised", s.tier_bytes[TIER_OOB] > 0 &&
                                 s.gap_seen[4] > 0);
    check("near_tier_every_tick", s.gap_seen[1] > 0);
    check("mid_tier_half_rate", s.gap_seen[2] > 0);
    check("isolated_reported",
           s.v[2].empty_ticks == TICKS && s.v[3].empty_ticks == TICKS);
    check("error_within_bound", s.err_max <= err_bound + 1e-6f);
    check("cull_cheaper", s.bytes_cull < s.bytes_full);
    check("bandwidth_cut_70pct", reduction >= 70.0f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
