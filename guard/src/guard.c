/* guard: continuity validation via reachable set from speed/accel limits.
 *
 * Reachable set for the next report, given the last two accepted reports
 * (x0,t-2) and (x1,t-1) with tick spacing dt:
 *   speed: |x - x1|        <= vmax*dt         + k*sigma
 *   accel: |x - (2*x1-x0)| <= 0.5*amax*dt^2   + k*sigma
 * A report outside the intersection is anomalous.
 *
 * Noise model: report noise per axis has mean 0 and std sigma (nothing else
 * assumed). Markov on the squared 2D noise norm gives, per observation:
 *   P(speed false alarm) <= 4/k^2    E|n_i - n_{i-1}|^2            = 4 sigma^2
 *   P(accel false alarm) <= 12/k^2   E|n_i - 2 n_{i-1} + n_{i-2}|^2 = 12 sigma^2
 *   FPR                  <= 16/k^2   (union bound, distribution-free)
 * Over N observations of one player: FPR <= 16 N / k^2.
 *
 * Tuning: to keep zero misses on a labeled cheat set, k must stay below
 * min over cheat events of max(speed_z, accel_z); to meet a per-observation
 * FPR target a, k must be at least sqrt(16/a). The eval below prints that
 * window and reports whether the current k is too loose, too tight, or in
 * range.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_TRACK 32

typedef struct {
    float vmax;    /* max speed, units/s */
    float amax;    /* max accel, units/s^2 */
    float sigma;   /* report noise std per axis, units */
    float k;       /* noise margin multiplier: FPR per observation <= 16/k^2 */
    float tick_dt; /* seconds per tick */
} GuardCfg;

typedef struct {
    int id, alive;
    float x0, y0;  /* older report */
    float x1, y1;  /* newest report */
    unsigned tick; /* tick of newest report */
    int have;      /* 0,1,2: how many reports in history */
    unsigned checks;
    unsigned alarms;
} Track;

typedef struct { GuardCfg cfg; Track t[MAX_TRACK]; int n; } Guard;

typedef struct {
    int anomaly;     /* 1 if outside reachable set */
    float speed_z;   /* (speed residual - vmax*dt) / sigma; alarm iff > k */
    float accel_z;   /* (accel residual - 0.5*amax*dt^2) / sigma; alarm iff > k */
} GuardRes;

static void guard_init(Guard* g, GuardCfg cfg) {
    memset(g, 0, sizeof(*g));
    g->cfg = cfg;
}

static Track* guard_track(Guard* g, int id) {
    for (int i = 0; i < g->n; i++)
        if (g->t[i].alive && g->t[i].id == id) return &g->t[i];
    if (g->n >= MAX_TRACK) return 0;
    Track* t = &g->t[g->n++];
    memset(t, 0, sizeof(*t));
    t->id = id;
    t->alive = 1;
    return t;
}

static float dist2d(float ax, float ay, float bx, float by) {
    float dx = ax - bx, dy = ay - by;
    return sqrtf(dx * dx + dy * dy);
}

/* observe one report; res.anomaly==1 when outside the reachable set.
 * returns -1 on stale tick or unknown track capacity; 0 otherwise.
 * On a tick gap the track resyncs without alarming: lost reports must not
 * turn into false bans. */
static int guard_observe(Guard* g, int id, unsigned tick, float x, float y, GuardRes* res) {
    Track* t = guard_track(g, id);
    if (!t) return -1;
    res->anomaly = 0;
    res->speed_z = -1e30f;
    res->accel_z = -1e30f;
    if (t->have > 0 && tick <= t->tick) return -1;      /* stale tick rejected */
    if (t->have > 0 && tick != t->tick + 1) t->have = 0; /* gap: resync */

    const GuardCfg* c = &g->cfg;
    float dt = c->tick_dt;
    if (t->have >= 1) {
        float sr = dist2d(x, y, t->x1, t->y1);
        res->speed_z = (sr - c->vmax * dt) / c->sigma;
        t->checks++;
    }
    if (t->have >= 2) {
        float px = 2.0f * t->x1 - t->x0, py = 2.0f * t->y1 - t->y0;
        float ar = dist2d(x, y, px, py);
        res->accel_z = (ar - 0.5f * c->amax * dt * dt) / c->sigma;
    }
    if (res->speed_z > c->k || res->accel_z > c->k) {
        res->anomaly = 1;
        t->alarms++;
    }
    t->x0 = t->x1; t->y0 = t->y1;
    t->x1 = x;   t->y1 = y;
    t->tick = tick;
    /* on anomaly keep only the newest report: the pre-jump history would
     * otherwise make the accel check cascade alarms after one teleport */
    t->have = res->anomaly ? 1 : (t->have < 2 ? t->have + 1 : 2);
    return 0;
}

/* ---- labeled-set evaluation ------------------------------------------ */

static unsigned rng_s = 12345u;
static float frand(void) {
    rng_s = rng_s * 1103515245u + 12345u;
    return ((rng_s >> 16) & 0x7FFF) / 32768.0f;
}
/* Irwin-Hall: mean 0, std 1, bounded in [-6,6] */
static float gnoise(float sigma) {
    float s = 0.0f;
    for (int i = 0; i < 12; i++) s += frand();
    return (s - 6.0f) * sigma;
}

typedef struct { float x, y, vx, vy; } Sim;

/* legit motion: accel within 0.9*amax, speed within 0.9*vmax */
static void sim_step(Sim* s, const GuardCfg* c) {
    float dt = c->tick_dt;
    float am = 0.9f * c->amax;
    float ax = (frand() * 2.0f - 1.0f) * am;
    float ay = (frand() * 2.0f - 1.0f) * am;
    float an = sqrtf(ax * ax + ay * ay);
    if (an > am) { ax *= am / an; ay *= am / an; }
    s->vx += ax * dt;
    s->vy += ay * dt;
    float vn = sqrtf(s->vx * s->vx + s->vy * s->vy);
    float vm = 0.9f * c->vmax;
    if (vn > vm) { s->vx *= vm / vn; s->vy *= vm / vn; }
    s->x += s->vx * dt;
    s->y += s->vy * dt;
}

/* hand-labeled teleport events: {trace, at_tick, dx, dy} — the "cheat set" */
typedef struct { int trace; int at; float dx, dy; } Teleport;

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    GuardCfg cfg = { 10.0f, 40.0f, 0.02f, 20.0f, 0.05f };
    Guard g;
    GuardRes r;

    /* unit: reachable-set behaviour */
    guard_init(&g, cfg);
    check("first_ok", guard_observe(&g, 1, 1, 0.0f, 0.0f, &r) == 0 && !r.anomaly);
    check("second_ok", guard_observe(&g, 1, 2, 0.4f, 0.0f, &r) == 0 && !r.anomaly);
    check("third_ok", guard_observe(&g, 1, 3, 0.8f, 0.0f, &r) == 0 && !r.anomaly);
    check("stale_rejected", guard_observe(&g, 1, 3, 0.9f, 0.0f, &r) == -1);
    check("teleport_flagged", guard_observe(&g, 1, 4, 50.0f, 0.8f, &r) == 0 && r.anomaly);
    check("resync_no_cascade", guard_observe(&g, 1, 5, 50.3f, 0.8f, &r) == 0 && !r.anomaly);
    check("gap_resync", guard_observe(&g, 1, 9, 51.0f, 0.9f, &r) == 0 && !r.anomaly);

    /* accel-only violation: speed ok, direction reversal breaks accel bound */
    guard_init(&g, cfg);
    guard_observe(&g, 2, 1, 0.0f, 0.0f, &r);
    guard_observe(&g, 2, 2, 0.4f, 0.0f, &r);
    check("accel_only_alarm", guard_observe(&g, 2, 3, -0.45f, 0.0f, &r) == 0 && r.anomaly);

    /* eval against normal traces + hand-labeled cheat set */
    enum { N_NORMAL = 8, N_CHEAT = 6, T = 200, T_CHEAT = 60 };
    static const Teleport tp[] = {
        { 0,  8,  30.0f,   0.0f },
        { 1, 15, -25.0f,  40.0f },
        { 2, 20,   3.0f,   0.0f },   /* small jump, near decision edge */
        { 3, 25,   0.0f,   2.2f },   /* small jump, near decision edge */
        { 4, 30, -50.0f, -50.0f },
        { 4, 45,  20.0f, -10.0f },   /* same cheater teleports twice */
        { 5, 40,   5.0f,  -4.0f },
    };
    int tp_count = 0, fp_count = 0, fn_count = 0, tn_count = 0, obs = 0;
    float k_max_recall = 1e30f;

    guard_init(&g, cfg);
    for (int p = 0; p < N_NORMAL + N_CHEAT; p++) {
        int is_cheat = p >= N_NORMAL;
        int steps = is_cheat ? T_CHEAT : T;
        Sim s; memset(&s, 0, sizeof(s));
        s.vx = (frand() * 2.0f - 1.0f) * 0.5f * cfg.vmax;
        s.vy = (frand() * 2.0f - 1.0f) * 0.5f * cfg.vmax;
        int id = 100 + p;
        for (int i = 0; i < steps; i++) {
            sim_step(&s, &cfg);
            int label = 0;
            if (is_cheat) {
                for (unsigned j = 0; j < sizeof(tp) / sizeof(tp[0]); j++)
                    if (tp[j].trace == p - N_NORMAL && tp[j].at == i) {
                        s.x += tp[j].dx; s.y += tp[j].dy;  /* teleport persists */
                        label = 1;
                    }
            }
            float rx = s.x + gnoise(cfg.sigma);
            float ry = s.y + gnoise(cfg.sigma);
            if (guard_observe(&g, id, (unsigned)(i + 1), rx, ry, &r) != 0) continue;
            if (i < 2) continue;             /* both checks active from 3rd report */
            obs++;
            if (label) {
                if (r.anomaly) {
                    tp_count++;
                    float z = r.speed_z > r.accel_z ? r.speed_z : r.accel_z;
                    if (z < k_max_recall) k_max_recall = z;
                } else fn_count++;
            } else {
                if (r.anomaly) fp_count++; else tn_count++;
            }
        }
    }

    float recall = (tp_count + fn_count) ? (float)tp_count / (tp_count + fn_count) : 1.0f;
    float fpr = (fp_count + tn_count) ? (float)fp_count / (fp_count + tn_count) : 0.0f;
    float fpr_bound = 16.0f / (cfg.k * cfg.k);
    float fpr_target = 0.05f;
    float k_min_target = sqrtf(16.0f / fpr_target);

    printf("eval_tp=%d eval_fp=%d eval_fn=%d eval_tn=%d obs=%d\n",
           tp_count, fp_count, fn_count, tn_count, obs);
    printf("recall=%.3f\n", recall);
    printf("fpr=%.6f fpr_bound=%.6f fpr_trace_bound=%.3f\n",
           fpr, fpr_bound, 16.0f * T / (cfg.k * cfg.k));
    printf("k=%.1f k_min_for_target=%.1f k_max_for_recall=%.1f\n",
           cfg.k, k_min_target, k_max_recall);

    const char* verdict;
    const char* action;
    if (fn_count > 0) {
        verdict = "too_loose";  action = "decrease_k";   /* tighten: cheats pass */
    } else if (fpr > fpr_target) {
        verdict = "too_tight";  action = "increase_k";   /* loosen: legit players flagged */
    } else {
        verdict = "in_range";   action = "none";
    }
    printf("verdict=%s action=%s\n", verdict, action);

    check("zero_miss", fn_count == 0);
    check("recall_one", recall >= 1.0f);
    check("fpr_within_bound", fpr <= fpr_bound);
    check("k_window_valid", k_min_target <= cfg.k && cfg.k < k_max_recall);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
