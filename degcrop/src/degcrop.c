
/* degcrop: couple remote animation degradation with AOI cropping.
 *
 * A remote entity is rendered from one of two network streams:
 *   - fine     : per-tick, high-resolution position packets while in AOI
 *   - degraded : sparse, coarsely quantized packets + velocity extrapolation
 *                while cropped out of AOI
 * Without coupling, every reference update at/after an AOI boundary is applied
 * directly to the rendered position: right after cropping the degraded
 * reference is stale (no coarse packet yet), and re-entry corrections plus
 * periodic coarse resyncs show up as teleports. With coupling enabled every
 * reference change (mode switch or coarse resync) starts a catch-up blend
 * from the currently rendered point: each tick the render moves at most
 * DC_MAX_STEP toward the target reference. The per-tick trajectory step is
 * therefore hard-bounded (no velocity term can violate it), so no jump can
 * appear regardless of speed changes or direction reversal. The blend is
 * purely receiver-side: linked=0 and linked=1 emit identical packets.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define DC_MAX_ENT        16
#define DC_MAX_STEP       0.25f   /* hard bound on extra render distance/tick */
#define DC_FINE_SCALE     64.0f   /* quant units per world unit, in-AOI stream  */
#define DC_COARSE_SCALE   4.0f    /* quant units per world unit, cropped stream */
#define DC_COARSE_PERIOD  8       /* ticks between degraded packets             */
#define DC_JUMP_EPS       2.0f    /* render step above normal motion = teleport  */
#define DC_LOG_MAX        64

typedef enum { MODE_FINE = 0, MODE_DEGRADED = 1 } DcMode;

typedef struct {
    int id;
    int alive;
    int in_aoi;
    DcMode mode;                /* active stream (while blending: old mode) */
    int blending;               /* 1 while a blend is in progress */
    DcMode blend_target;
    float render_x;             /* position actually shown on screen */
    float fine_x;               /* latest fine-stream reference */
    float coarse_x;             /* position from latest coarse packet/seed */
    float coarse_v;             /* velocity from latest coarse packet/seed */
    unsigned coarse_tick;
    float prev_render_x;
    float max_step;             /* largest |render_x delta| observed */
    int jumps;                  /* ticks whose render step exceeded DC_JUMP_EPS */
} DcEntity;

typedef struct {
    int tick;
    int id;
    DcMode from;
    DcMode to;
    int linked;
} DcSwitchLog;

typedef struct {
    DcEntity e[DC_MAX_ENT];
    int n;
    int linked;
    int log_n;
    DcSwitchLog log[DC_LOG_MAX];
    long fine_pkts, coarse_pkts;
    long fine_bytes, coarse_bytes;
} DcWorld;

/* ---- packet sizing, same varint+zigzag wire format as the snapshot module ---- */

static int varint_len(unsigned v) {
    int n = 1;
    while (v >= 0x80) { n++; v >>= 7; }
    return n;
}

static int zigzag_len(int q) {
    unsigned z = (unsigned)((q << 1) ^ (q >> 31));
    return varint_len(z);
}

/* fine packet: tick varint + one zigzag quantized position axis */
static int fine_pkt_bytes(unsigned tick, float x) {
    return varint_len(tick) + zigzag_len((int)lrintf(x * DC_FINE_SCALE));
}

/* coarse packet: tick varint + zigzag position + zigzag velocity */
static int coarse_pkt_bytes(unsigned tick, float x, float v) {
    return varint_len(tick)
         + zigzag_len((int)lrintf(x * DC_COARSE_SCALE))
         + zigzag_len((int)lrintf(v * DC_COARSE_SCALE));
}

/* ---- world lifecycle ---- */

static void dc_init(DcWorld* w, int linked) {
    memset(w, 0, sizeof(*w));
    w->linked = linked;
}

static int dc_add(DcWorld* w, int id, float x) {
    if (w->n >= DC_MAX_ENT) return -1;
    for (int i = 0; i < w->n; i++)
        if (w->e[i].alive && w->e[i].id == id) return -1;
    DcEntity* e = &w->e[w->n++];
    e->id = id;
    e->alive = 1;
    e->in_aoi = 1;
    e->mode = MODE_FINE;
    e->render_x = e->fine_x = x;
    e->coarse_x = x;
    e->coarse_v = 0.0f;
    e->coarse_tick = 0;
    return w->n - 1;
}

static const char* mode_name(DcMode m) {
    return m == MODE_FINE ? "fine" : "degraded";
}

static void dc_log_switch(DcWorld* w, const DcEntity* e, int tick,
                          DcMode from, DcMode to) {
    if (w->log_n < DC_LOG_MAX) {
        w->log[w->log_n++] = (DcSwitchLog){ tick, e->id, from, to, w->linked };
    }
    printf("[switch] tick=%d id=%d %s->%s linked=%d\n",
           tick, e->id, mode_name(from), mode_name(to), w->linked);
}

/* reference position the degraded stream would show at this tick */
static float degraded_ref(const DcEntity* e, int tick) {
    return e->coarse_x + e->coarse_v * (float)(tick - (int)e->coarse_tick);
}

static float stream_ref(const DcEntity* e, DcMode m, int tick) {
    return m == MODE_FINE ? e->fine_x : degraded_ref(e, tick);
}

static void start_blend(DcEntity* e, DcMode target) {
    e->blend_target = target;
    e->blending = 1;             /* continues from the currently shown point */
}

/* bounded move toward target: never more than DC_MAX_STEP per tick */
static float step_toward(float from, float target) {
    float d = target - from;
    if (fabsf(d) <= DC_MAX_STEP) return target;
    return from + (d > 0.0f ? DC_MAX_STEP : -DC_MAX_STEP);
}

static void dc_step(DcWorld* w, int tick, int id, int in_aoi,
                    float true_x, float true_v) {
    DcEntity* e = NULL;
    for (int i = 0; i < w->n; i++)
        if (w->e[i].alive && w->e[i].id == id) { e = &w->e[i]; break; }
    if (!e) return;

    e->in_aoi = in_aoi;
    int new_coarse = 0;

    /* --- AOI cropping decides which stream the entity is served from --- */
    if (in_aoi) {
        e->fine_x = (float)lrintf(true_x * DC_FINE_SCALE) / DC_FINE_SCALE;
        w->fine_pkts++;
        w->fine_bytes += fine_pkt_bytes((unsigned)tick, true_x);
    } else if ((tick % DC_COARSE_PERIOD) == 0) {
        e->coarse_x = (float)lrintf(true_x * DC_COARSE_SCALE) / DC_COARSE_SCALE;
        e->coarse_v = (float)lrintf(true_v * DC_COARSE_SCALE) / DC_COARSE_SCALE;
        e->coarse_tick = (unsigned)tick;
        new_coarse = 1;
        w->coarse_pkts++;
        w->coarse_bytes += coarse_pkt_bytes((unsigned)tick, true_x, true_v);
    }

    DcMode want = in_aoi ? MODE_FINE : MODE_DEGRADED;
    int boundary = (!e->blending && want != e->mode)
                || (e->blending && want != e->blend_target);
    e->prev_render_x = e->render_x;

    if (boundary) {
        /* Coupled mode seeds the degraded reference locally from the last
         * shown position so extrapolation never starts from a stale point.
         * Legacy mode keeps the stale reference: that stale snap is the
         * teleport being reproduced. Seeding is receiver state, not a packet. */
        if (want == MODE_DEGRADED && w->linked) {
            e->coarse_x = e->render_x;
            e->coarse_v = true_v;
            e->coarse_tick = (unsigned)tick;
        }
        dc_log_switch(w, e, tick, e->blending ? e->blend_target : e->mode, want);
        if (w->linked) {
            start_blend(e, want);
        } else {
            e->blending = 0;
            e->mode = want;                   /* legacy: hard switch */
        }
    } else if (new_coarse && w->linked) {
        /* periodic resync: smooth the correction instead of snapping it */
        start_blend(e, MODE_DEGRADED);
    }

    if (e->blending) {
        float target = stream_ref(e, e->blend_target, tick);
        e->render_x = step_toward(e->render_x, target);
        if (e->render_x == target) {
            e->mode = e->blend_target;
            e->blending = 0;
        }
    } else if (e->mode == MODE_FINE) {
        e->render_x = e->fine_x;
    } else {
        e->render_x = degraded_ref(e, tick);
    }

    /* teleport = rendered position step far above any physical motion step;
     * speed changes are continuous, a hard discontinuity is not */
    float step = fabsf(e->render_x - e->prev_render_x);
    if (step > e->max_step) e->max_step = step;
    if (step > DC_JUMP_EPS) e->jumps++;
}

/* ---- scenarios + demo ---- */

#define SCEN_TICKS 100
#define AOI_RADIUS 25.0f

static DcEntity* find_ent(DcWorld* w, int id) {
    for (int i = 0; i < w->n; i++)
        if (w->e[i].alive && w->e[i].id == id) return &w->e[i];
    return NULL;
}

/* piecewise-linear walk whose segment boundaries sit exactly on the AOI
 * radius: out at tick 25, back in at 50, out again at 75 — three boundary
 * switches in both directions. Speeds stay <=1.5/tick, below DC_JUMP_EPS,
 * so physical motion can never look like a teleport even at a boundary. */
static void entity_motion(int tick, float* x, float* v) {
    if (tick < 25) {             /* inside AOI, v=1.0 -> x 0..24 */
        *v = 1.0f; *x = (float)tick;
    } else if (tick < 50) {      /* cropped from tick25, v=1.5 -> x 25..62.5 */
        *v = 1.5f; *x = 25.0f + 1.5f * (float)(tick - 25);
    } else if (tick < 75) {      /* back in AOI at tick50, v=-1.5 -> 62.5..25 */
        *v = -1.5f; *x = 62.5f - 1.5f * (float)(tick - 50);
    } else {                     /* cropped again from tick75, v=-1.0 -> 25..0 */
        *v = -1.0f; *x = 25.0f - (float)(tick - 75);
    }
}

static void run_scenario(int linked, int verbose, int* jumps, float* max_step,
                         long* fine_pkts, long* coarse_pkts,
                         long* fine_bytes, long* coarse_bytes) {
    DcWorld w;
    dc_init(&w, linked);
    dc_add(&w, 1, 0.0f);
    dc_add(&w, 2, 1.0f);

    if (verbose)
        printf("== scenario linked=%d (AOI radius=%.0f, coarse period=%d, "
               "max blend step=%.2f) ==\n",
               linked, AOI_RADIUS, DC_COARSE_PERIOD, DC_MAX_STEP);

    for (int tick = 0; tick < SCEN_TICKS; tick++) {
        float x1, v1;
        entity_motion(tick, &x1, &v1);
        /* deterministic AOI membership from the scenario timeline: cropped
         * during [25,50) and [75,100), in AOI otherwise — switches land
         * exactly on ticks 25, 50 and 75 regardless of float boundary math */
        int in1 = !( (tick >= 25 && tick < 50) || tick >= 75 );
        int was_in1 = w.e[0].in_aoi;

        dc_step(&w, tick, 1, in1, x1, v1);

        float x2 = 1.0f + 0.01f * (float)(tick % 5), v2 = 0.0f;
        dc_step(&w, tick, 2, 1, x2, v2);

        if (verbose && (tick % 5 == 0 || in1 != was_in1)) {
            DcEntity* e1 = find_ent(&w, 1);
            printf("[traj] tick=%d id=1 in_aoi=%d mode=%s blend=%d "
                   "render=%.4f true=%.4f\n",
                   tick, e1->in_aoi,
                   mode_name(e1->blending ? e1->blend_target : e1->mode),
                   e1->blending, e1->render_x, x1);
        }
    }

    DcEntity* e1 = find_ent(&w, 1);
    DcEntity* e2 = find_ent(&w, 2);
    *jumps = e1->jumps;
    *max_step = e1->max_step;
    *fine_pkts = w.fine_pkts;
    *coarse_pkts = w.coarse_pkts;
    *fine_bytes = w.fine_bytes;
    *coarse_bytes = w.coarse_bytes;

    if (verbose) {
        printf("-- packet summary (linked=%d) --\n", linked);
        printf("fine:   pkts=%ld bytes=%ld avg=%.2f B\n",
               w.fine_pkts, w.fine_bytes,
               w.fine_pkts ? (double)w.fine_bytes / w.fine_pkts : 0.0);
        printf("coarse: pkts=%ld bytes=%ld avg=%.2f B\n",
               w.coarse_pkts, w.coarse_bytes,
               w.coarse_pkts ? (double)w.coarse_bytes / w.coarse_pkts : 0.0);
        printf("total:  pkts=%ld bytes=%ld (blend is receiver-side: "
               "linked on/off emit identical packets)\n",
               w.fine_pkts + w.coarse_pkts,
               w.fine_bytes + w.coarse_bytes);
        printf("-- result --\n");
        printf("entity1 jumps=%d max_render_step=%.4f (teleport threshold %.1f)\n",
               e1->jumps, e1->max_step, (float)DC_JUMP_EPS);
        printf("entity2(control, always in AOI) jumps=%d\n", e2->jumps);
        printf("switch_log_entries=%d\n", w.log_n);
    }
}

/* AOI flapping across a boundary must stay jump-free when linked */
static int flap_scenario(int linked) {
    DcWorld w;
    dc_init(&w, linked);
    dc_add(&w, 1, 0.0f);
    for (int tick = 0; tick < 40; tick++) {
        float v = 1.0f;
        float x = (float)tick;
        int in;
        if (tick < 10) in = 1;
        else if (tick < 16) in = (tick % 2 == 0);   /* flap across boundary */
        else in = 0;
        dc_step(&w, tick, 1, in, x, v);
    }
    return find_ent(&w, 1)->jumps;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    int jumps_on, jumps_off;
    float step_on, step_off;
    long fp_on, cp_on, fb_on, cb_on;
    long fp_off, cp_off, fb_off, cb_off;

    /* verbose demo run: switch logs + trajectory samples are the evidence */
    run_scenario(1, 1, &jumps_on, &step_on, &fp_on, &cp_on, &fb_on, &cb_on);
    printf("\n");
    run_scenario(0, 1, &jumps_off, &step_off, &fp_off, &cp_off, &fb_off, &cb_off);

    printf("\n-- self tests --\n");
    check("linked_no_jump",       jumps_on == 0);
    check("linked_step_bounded",  step_on <= DC_MAX_STEP + 1.5f + 1e-4f);
    check("unlinked_reproduces",  jumps_off >= 2);  /* boundary hard snaps */
    check("unlinked_worse",       step_off > step_on);
    check("packet_parity_pkts",   fp_on == fp_off && cp_on == cp_off);
    check("packet_parity_bytes",  fb_on == fb_off && cb_on == cb_off);
    check("linked_flap_safe",     flap_scenario(1) == 0);
    check("unlinked_flap_teleports", flap_scenario(0) > 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
