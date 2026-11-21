/* memaudit: memory audit for sync buffers and queues.
 *
 * Per-structure accounting of live bytes against static capacity:
 * structural upper bounds, peak (high-water) tracking, bound-breach
 * alarms, and leak localization by structure name. The audit is
 * observation-only: it receives element counts and never touches the
 * audited state, so enabling it cannot change sync behavior.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MA_MAX_REGIONS 16

typedef struct {
    const char* name;   /* owning structure, used to localize leaks */
    size_t elem;        /* bytes per element */
    size_t cap;         /* capacity, elements */
    size_t used;        /* live elements */
    size_t peak;        /* high-water mark, elements */
    size_t bound;       /* alarm threshold in bytes; 0 = no alarm */
    unsigned alarms;    /* bound-breach transitions (edge-triggered) */
} MaRegion;

typedef struct {
    MaRegion r[MA_MAX_REGIONS];
    int n;
    size_t peak_total;  /* high-water of total live bytes */
    unsigned alarms;    /* total alarms across regions */
} MemAudit;

static void ma_init(MemAudit* a) { memset(a, 0, sizeof(*a)); }

static int ma_register(MemAudit* a, const char* name, size_t elem, size_t cap, size_t bound) {
    if (a->n >= MA_MAX_REGIONS) return -1;
    if (elem == 0 || cap == 0) return -1;
    MaRegion* r = &a->r[a->n];
    r->name = name; r->elem = elem; r->cap = cap;
    r->used = 0; r->peak = 0; r->bound = bound; r->alarms = 0;
    return a->n++;
}

static size_t ma_region_bytes(const MaRegion* r) { return r->used * r->elem; }

static size_t ma_total_bytes(const MemAudit* a) {
    size_t t = 0;
    for (int i = 0; i < a->n; i++) t += ma_region_bytes(&a->r[i]);
    return t;
}

/* structural upper bound of a region: capacity in bytes */
static size_t ma_bound_bytes(const MemAudit* a, int id) {
    return a->r[id].cap * a->r[id].elem;
}

static void ma_note_peak(MemAudit* a, MaRegion* r) {
    if (r->used > r->peak) r->peak = r->used;
    size_t t = ma_total_bytes(a);
    if (t > a->peak_total) a->peak_total = t;
}

static int ma_acquire(MemAudit* a, int id, size_t elems) {
    MaRegion* r = &a->r[id];
    if (elems > r->cap - r->used) return -1;   /* over capacity: reject */
    size_t before = ma_region_bytes(r);
    r->used += elems;
    if (r->bound && before <= r->bound && ma_region_bytes(r) > r->bound) {
        r->alarms++; a->alarms++;              /* crossing the bound: alarm once */
    }
    ma_note_peak(a, r);
    return 0;
}

static int ma_release(MemAudit* a, int id, size_t elems) {
    MaRegion* r = &a->r[id];
    if (elems > r->used) return -1;            /* underflow: double release */
    r->used -= elems;
    return 0;
}

/* leaked bytes = still-live bytes at teardown; names the leaking structures */
static size_t ma_leaks(const MemAudit* a, char* out, size_t outcap) {
    size_t leaked = 0, off = 0;
    if (out && outcap) out[0] = '\0';
    for (int i = 0; i < a->n; i++) {
        const MaRegion* r = &a->r[i];
        size_t b = ma_region_bytes(r);
        if (!b) continue;
        leaked += b;
        if (out && off < outcap) {
            int w = snprintf(out + off, outcap - off, "%s%s=%zu",
                             off ? " " : "", r->name, b);
            if (w > 0) off += (size_t)w;
        }
    }
    return leaked;
}

/* --- system under audit: replica of reconcile's prediction ring --- */

#define RBUF 64

typedef struct { unsigned tick; float x; } RPred;

typedef struct {
    RPred p[RBUF];
    int head;
    int count;
    unsigned acked;
    int rollbacks;
    float last_err;
} RRecon;

static void rr_init(RRecon* r) { memset(r, 0, sizeof(*r)); }

static int rr_push(RRecon* r, unsigned tick, float x) {
    if (tick <= r->acked) return -1;
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - 1 - i + RBUF * 2) % RBUF;
        if (r->p[idx].tick == tick) return -1;
    }
    r->p[r->head].tick = tick;
    r->p[r->head].x = x;
    r->head = (r->head + 1) % RBUF;
    if (r->count < RBUF) r->count++;
    return 0;
}

static int rr_ack(RRecon* r, unsigned tick, float x) {
    float pred = 0.0f; int found = 0;
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - 1 - i + RBUF * 2) % RBUF;
        if (r->p[idx].tick == tick) { pred = r->p[idx].x; found = 1; break; }
    }
    r->last_err = found ? fabsf(pred - x) : -1.0f;
    if (found && fabsf(pred - x) > 1e-3f) r->rollbacks++;
    if (tick > r->acked) r->acked = tick;
    return found ? 1 : 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* deterministic op mix over the ring; when audit != NULL, mirror element
 * counts into region rid. Returns a checksum of every observable output. */
static unsigned long recon_script(MemAudit* audit, int rid, int* final_count) {
    RRecon r;
    rr_init(&r);
    unsigned long sink = 0;
    for (unsigned i = 1; i <= 200; i++) {
        int prev = r.count;
        int rc = rr_push(&r, i, (float)i * 0.5f);
        if (audit && rc == 0 && r.count > prev)
            check("mirror_acquire", ma_acquire(audit, rid, 1) == 0);
        sink = sink * 31 + (unsigned long)(rc + 1) + (unsigned long)r.count;
        if (i % 3 == 0) {
            rc = rr_ack(&r, i - 1, (float)(i - 1) * 0.5f + (i % 2 ? 0.0f : 0.25f));
            sink = sink * 31 + (unsigned long)(rc + 1) + r.acked;
        }
        if (i % 7 == 0) {  /* duplicate + stale pushes, rejected */
            sink = sink * 31 + (unsigned long)(rr_push(&r, i, 1.0f) + 1);
            sink = sink * 31 + (unsigned long)(rr_push(&r, 1, 1.0f) + 1);
        }
    }
    sink = sink * 31 + (unsigned long)r.rollbacks + (unsigned long)r.head;
    unsigned eb;
    memcpy(&eb, &r.last_err, sizeof(eb));
    sink = sink * 31 + eb;
    *final_count = r.count;
    return sink;
}

/* --- layouts of the audited sync buffers, for manual accounting --- */

typedef struct { unsigned tick; float x; } Pred;                  /* reconcile */
typedef struct { double rtt; double offset; double at; } Sample;  /* clock */
typedef struct { int id; float x, y; int alive; } Entity;         /* aoi */
typedef struct { unsigned char buf[64]; int len; } Packet;        /* snapshot */

/* hand-computed bytes, independent of the audit's own arithmetic */
#define MANUAL_PRED    (64 * 8)   /* 512: unsigned + float        */
#define MANUAL_SAMPLE  (16 * 24)  /* 384: 3 doubles               */
#define MANUAL_ENTITY  (16 * 16)  /* 256: 2 ints + 2 floats       */
#define MANUAL_PACKET  (64 + 4)   /*  68: byte buffer + len       */
#define MANUAL_TOTAL   (MANUAL_PRED + MANUAL_SAMPLE + MANUAL_ENTITY + MANUAL_PACKET)

int main(void) {
    /* manual accounting must agree with the actual layouts */
    check("sizeof_pred", sizeof(Pred) == 8);
    check("sizeof_sample", sizeof(Sample) == 24);
    check("sizeof_entity", sizeof(Entity) == 16);
    check("sizeof_packet", sizeof(Packet) == 68);

    MemAudit a;
    ma_init(&a);
    int rp = ma_register(&a, "reconcile.pred", sizeof(Pred), 64, MANUAL_PRED);
    int rc = ma_register(&a, "clock.samples", sizeof(Sample), 16, MANUAL_SAMPLE);
    int ra = ma_register(&a, "aoi.entities", sizeof(Entity), 16, MANUAL_ENTITY);
    int rs = ma_register(&a, "snapshot.packet", sizeof(Packet), 1, MANUAL_PACKET);
    check("registered", rp == 0 && rc == 1 && ra == 2 && rs == 3);
    check("empty_total", ma_total_bytes(&a) == 0);

    /* structural upper bounds */
    check("bound_pred", ma_bound_bytes(&a, rp) == MANUAL_PRED);
    check("bound_clock", ma_bound_bytes(&a, rc) == MANUAL_SAMPLE);
    check("bound_aoi", ma_bound_bytes(&a, ra) == MANUAL_ENTITY);
    check("bound_snap", ma_bound_bytes(&a, rs) == MANUAL_PACKET);

    /* fill every buffer to capacity, in steps */
    check("fill_pred", ma_acquire(&a, rp, 40) == 0 && ma_acquire(&a, rp, 24) == 0);
    check("fill_clock", ma_acquire(&a, rc, 16) == 0);
    check("fill_aoi", ma_acquire(&a, ra, 10) == 0 && ma_acquire(&a, ra, 6) == 0);
    check("fill_snap", ma_acquire(&a, rs, 1) == 0);

    /* audit totals agree with manual accounting */
    check("total_matches_manual", ma_total_bytes(&a) == MANUAL_TOTAL);
    check("peak_total_matches", a.peak_total == MANUAL_TOTAL);
    check("peak_pred", a.r[rp].peak == 64);
    check("peak_clock", a.r[rc].peak == 16);
    check("no_alarm_at_bound", a.alarms == 0);

    /* capacity is a hard bound */
    check("over_cap_rejected", ma_acquire(&a, rp, 1) == -1);
    check("cap_unchanged", ma_region_bytes(&a.r[rp]) == MANUAL_PRED);

    /* ring overwrite at full capacity: release oldest, acquire new, net zero */
    check("ring_churn", ma_release(&a, rp, 1) == 0 && ma_acquire(&a, rp, 1) == 0);
    check("churn_total_stable", ma_total_bytes(&a) == MANUAL_TOTAL);

    /* bound-breach alarm: tight bound below capacity, edge-triggered */
    int rt = ma_register(&a, "test.tight", 16, 8, 64);
    check("tight_registered", rt == 4);
    check("tight_at_bound", ma_acquire(&a, rt, 4) == 0 && a.r[rt].alarms == 0);
    check("tight_breach", ma_acquire(&a, rt, 1) == 0 && a.r[rt].alarms == 1);
    check("tight_no_realarm", ma_acquire(&a, rt, 1) == 0 && a.r[rt].alarms == 1);
    check("tight_rearm", ma_release(&a, rt, 2) == 0 &&
                         ma_acquire(&a, rt, 2) == 0 && a.r[rt].alarms == 2);
    check("alarm_total", a.alarms == 2);
    check("tight_peak", a.r[rt].peak == 6);
    check("release_underflow", ma_release(&a, rt, 7) == -1);
    check("tight_drained", ma_release(&a, rt, 6) == 0);

    /* leak localization: drain all but 3 clock samples */
    check("drain_pred", ma_release(&a, rp, 64) == 0);
    check("drain_aoi", ma_release(&a, ra, 16) == 0);
    check("drain_snap", ma_release(&a, rs, 1) == 0);
    check("drain_clock_partial", ma_release(&a, rc, 13) == 0);
    char report[256];
    size_t leaked = ma_leaks(&a, report, sizeof(report));
    check("leak_bytes", leaked == 3 * sizeof(Sample));
    check("leak_located", strstr(report, "clock.samples=72") != NULL);
    check("leak_not_misattributed", strstr(report, "reconcile.pred") == NULL);

    /* leaks must be zero after full teardown */
    check("drain_clock_rest", ma_release(&a, rc, 3) == 0);
    check("leaks_zero", ma_leaks(&a, report, sizeof(report)) == 0);
    check("total_zero", ma_total_bytes(&a) == 0);

    /* audit must not change sync behavior: same script, off vs on */
    int count_off = 0, count_on = 0;
    unsigned long sink_off = recon_script(NULL, -1, &count_off);

    MemAudit live;
    ma_init(&live);
    int rl = ma_register(&live, "recon.pred", sizeof(RPred), RBUF, 0);
    unsigned long sink_on = recon_script(&live, rl, &count_on);

    check("audit_neutral", sink_off == sink_on && count_off == count_on);
    check("audit_tracks_live", (int)ma_region_bytes(&live.r[rl]) ==
                               count_on * (int)sizeof(RPred));
    check("live_within_bound", ma_region_bytes(&live.r[rl]) <= ma_bound_bytes(&live, rl));
    check("live_peak_capped", live.r[rl].peak == RBUF);
    check("live_drain", ma_release(&live, rl, (size_t)count_on) == 0);
    check("live_leaks_zero", ma_leaks(&live, report, sizeof(report)) == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
