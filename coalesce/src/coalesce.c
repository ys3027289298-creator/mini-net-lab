
/* coalesce: per-target enter/leave coalescing, frame batching, snapshot fallback. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define SLOT_MAX 16   /* distinct targets tracked per frame window */
#define PRES_MAX 64   /* authoritative presence set size */

#define EV_ENTER 0
#define EV_LEAVE 1

#define BATCH_EMPTY    0
#define BATCH_DELTA    1
#define BATCH_SNAPSHOT 2

typedef struct { int target; int kind; } Ev;

/* one slot per target touched in the window; baseline is captured before its
   first event, so flush can derive the net delta from authoritative presence */
typedef struct { int target; int was_present; int used; } Slot;

typedef struct {
    int mode;
    unsigned frame;
    Ev ev[SLOT_MAX];
    int n;              /* delta event count (BATCH_DELTA) */
    int snap[PRES_MAX];
    int nsnap;          /* full snapshot target count (BATCH_SNAPSHOT) */
    int raw;            /* raw events ingested in this window */
    int deltas;         /* net delta events emitted for this window */
    double reduction;   /* 1 - deltas/raw, 0.0 when raw == 0 */
} Batch;

typedef struct {
    Slot slot[SLOT_MAX];
    int present[PRES_MAX];
    int npresent;
    int capacity;       /* max deliverable deltas per batch, 1..SLOT_MAX */
    unsigned frame;
    int win_raw;
    int force_snap;     /* an event overflowed the tables this window */
    long raw_total;
    long delta_total;   /* net deltas that survived coalescing */
    long dropped;       /* events that could not be represented as deltas */
    long snapshots;     /* degradation count */
    long delta_events;  /* deltas actually delivered */
} Coal;

static int set_has(const int* s, int n, int id) {
    for (int i = 0; i < n; i++) if (s[i] == id) return 1;
    return 0;
}

static int set_add(int* s, int* n, int cap, int id) {
    if (set_has(s, *n, id)) return 0;
    if (*n >= cap) return -1;
    s[(*n)++] = id;
    return 1;
}

static int set_del(int* s, int* n, int id) {
    for (int i = 0; i < *n; i++) {
        if (s[i] == id) { s[i] = s[--*n]; return 1; }
    }
    return 0;
}

static int set_equal(const int* a, int an, const int* b, int bn) {
    if (an != bn) return 0;
    for (int i = 0; i < an; i++) if (!set_has(b, bn, a[i])) return 0;
    return 1;
}

static int coal_init(Coal* c, int capacity) {
    if (c == NULL) return -1;
    if (capacity <= 0 || capacity > SLOT_MAX) return -1;  /* zero capacity rejected */
    memset(c, 0, sizeof(*c));
    c->capacity = capacity;
    return 0;
}

/* ingest one raw enter/leave event; -1 if invalid or dropped from delta table */
static int coal_event(Coal* c, int target, int kind) {
    if (c == NULL || (kind != EV_ENTER && kind != EV_LEAVE)) return -1;
    c->win_raw++;
    c->raw_total++;

    /* capture baseline presence before applying this target's first event */
    Slot* s = NULL;
    for (int i = 0; i < SLOT_MAX; i++) {
        if (c->slot[i].used && c->slot[i].target == target) { s = &c->slot[i]; break; }
    }
    if (s == NULL) {
        for (int i = 0; i < SLOT_MAX; i++) {
            if (!c->slot[i].used) {
                s = &c->slot[i];
                s->used = 1;
                s->target = target;
                s->was_present = set_has(c->present, c->npresent, target);
                break;
            }
        }
    }

    /* authoritative presence stays correct even when delta tracking overflows */
    int r = kind == EV_ENTER ? set_add(c->present, &c->npresent, PRES_MAX, target)
                             : set_del(c->present, &c->npresent, target);
    if (r < 0 || s == NULL) {
        c->dropped++;        /* event lost from the delta channel */
        c->force_snap = 1;   /* snapshot at flush keeps the client consistent */
        return -1;
    }
    return 0;
}

/* close the frame window and emit a delta batch, a full snapshot, or empty */
static int coal_flush(Coal* c, Batch* b) {
    if (c == NULL || b == NULL) return -1;
    memset(b, 0, sizeof(*b));
    b->frame = ++c->frame;
    b->raw = c->win_raw;

    /* derive net deltas: final authoritative state vs per-target baseline */
    for (int i = 0; i < SLOT_MAX; i++) {
        Slot* s = &c->slot[i];
        if (!s->used) continue;
        int now = set_has(c->present, c->npresent, s->target);
        if (now != s->was_present) {
            b->ev[b->n].target = s->target;
            b->ev[b->n].kind = now ? EV_ENTER : EV_LEAVE;
            b->n++;
        }
        s->used = 0;
    }
    b->deltas = b->n;
    b->reduction = c->win_raw > 0 ? 1.0 - (double)b->n / c->win_raw : 0.0;
    c->delta_total += b->n;

    if (c->force_snap || b->n > c->capacity) {
        b->mode = BATCH_SNAPSHOT;
        memcpy(b->snap, c->present, sizeof(int) * c->npresent);
        b->nsnap = c->npresent;
        b->n = 0;
        c->snapshots++;
    } else if (b->n == 0) {
        b->mode = BATCH_EMPTY;
    } else {
        b->mode = BATCH_DELTA;
        c->delta_events += b->n;
    }

    /* pending deltas reset per window; authoritative presence persists */
    memset(c->slot, 0, sizeof(c->slot));
    c->win_raw = 0;
    c->force_snap = 0;
    return 0;
}

static double coal_reduction(const Coal* c) {
    return c->raw_total > 0 ? 1.0 - (double)c->delta_total / (double)c->raw_total : 0.0;
}

/* client side: apply one batch the way a delivered client would */
static void client_apply(int* s, int* n, const Batch* b) {
    if (b->mode == BATCH_SNAPSHOT) {
        memcpy(s, b->snap, sizeof(int) * b->nsnap);
        *n = b->nsnap;
    } else if (b->mode == BATCH_DELTA) {
        for (int i = 0; i < b->n; i++) {
            if (b->ev[i].kind == EV_ENTER) set_add(s, n, PRES_MAX, b->ev[i].target);
            else set_del(s, n, b->ev[i].target);
        }
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Coal c;
    check("zero_cap_rejected", coal_init(&c, 0) == -1);
    check("neg_cap_rejected", coal_init(&c, -3) == -1);
    check("oversize_rejected", coal_init(&c, SLOT_MAX + 1) == -1);
    check("init_ok", coal_init(&c, 4) == 0);

    int ref[PRES_MAX], rn = 0;   /* reference: every event applied one by one */
    int cli[PRES_MAX], cn = 0;   /* client: coalesced batches applied */
    Batch b;

    /* enter,leave,enter on one target collapses to a single enter; ratio 2/3 */
    coal_event(&c, 7, EV_ENTER); set_add(ref, &rn, PRES_MAX, 7);
    coal_event(&c, 7, EV_LEAVE); set_del(ref, &rn, 7);
    coal_event(&c, 7, EV_ENTER); set_add(ref, &rn, PRES_MAX, 7);
    coal_flush(&c, &b);
    check("frame_number", b.frame == 1);
    check("coalesce_delta", b.mode == BATCH_DELTA && b.n == 1 &&
                            b.ev[0].target == 7 && b.ev[0].kind == EV_ENTER);
    check("reduction_ratio", fabs(b.reduction - (1.0 - 1.0 / 3.0)) < 1e-9);
    client_apply(cli, &cn, &b);
    check("state_match_coalesce", set_equal(cli, cn, ref, rn));

    /* full cancellation: enter then leave -> empty batch, 100% reduction */
    coal_event(&c, 8, EV_ENTER); set_add(ref, &rn, PRES_MAX, 8);
    coal_event(&c, 8, EV_LEAVE); set_del(ref, &rn, 8);
    coal_flush(&c, &b);
    check("cancel_empty", b.mode == BATCH_EMPTY && b.n == 0 && b.reduction == 1.0);
    client_apply(cli, &cn, &b);
    check("state_match_cancel", set_equal(cli, cn, ref, rn));

    /* no-op leave on an absent target must not eat a following enter */
    coal_event(&c, 11, EV_LEAVE); set_del(ref, &rn, 11);
    coal_event(&c, 11, EV_ENTER); set_add(ref, &rn, PRES_MAX, 11);
    coal_flush(&c, &b);
    check("nooppair_enter", b.mode == BATCH_DELTA && b.n == 1 &&
                            b.ev[0].target == 11 && b.ev[0].kind == EV_ENTER);
    client_apply(cli, &cn, &b);
    check("state_match_nooppair", set_equal(cli, cn, ref, rn));

    /* idle window: no events, no divide-by-zero */
    coal_flush(&c, &b);
    check("idle_empty", b.mode == BATCH_EMPTY && b.raw == 0 && b.reduction == 0.0);

    /* 4 raw events on 2 targets -> 2 net deltas, reduction 0.5 */
    coal_event(&c, 1, EV_ENTER); set_add(ref, &rn, PRES_MAX, 1);
    coal_event(&c, 1, EV_LEAVE); set_del(ref, &rn, 1);
    coal_event(&c, 1, EV_ENTER); set_add(ref, &rn, PRES_MAX, 1);
    coal_event(&c, 9, EV_ENTER); set_add(ref, &rn, PRES_MAX, 9);
    coal_flush(&c, &b);
    check("half_reduction", b.mode == BATCH_DELTA && b.n == 2 &&
                            fabs(b.reduction - 0.5) < 1e-9);
    client_apply(cli, &cn, &b);
    check("state_match_half", set_equal(cli, cn, ref, rn));

    /* capacity 4, 5 distinct new targets: degrade to full snapshot */
    for (int t = 20; t <= 24; t++) { coal_event(&c, t, EV_ENTER); set_add(ref, &rn, PRES_MAX, t); }
    coal_flush(&c, &b);
    check("overcap_snapshot", b.mode == BATCH_SNAPSHOT && b.nsnap == rn);
    check("snapshot_counted", c.snapshots == 1);
    check("snap_is_truth", set_equal(b.snap, b.nsnap, ref, rn));
    client_apply(cli, &cn, &b);
    check("state_match_snapshot", set_equal(cli, cn, ref, rn));

    /* deltas after a snapshot must still land on the replaced client state */
    coal_event(&c, 21, EV_LEAVE); set_del(ref, &rn, 21);
    coal_event(&c, 10, EV_ENTER); set_add(ref, &rn, PRES_MAX, 10);
    coal_flush(&c, &b);
    check("delta_after_snap", b.mode == BATCH_DELTA && b.n == 2);
    client_apply(cli, &cn, &b);
    check("state_match_after_snap", set_equal(cli, cn, ref, rn));

    check("bad_kind_rejected", coal_event(&c, 1, 99) == -1);
    check("no_drops_here", c.dropped == 0);
    check("cum_reduction_bounded", coal_reduction(&c) >= 0.0 && coal_reduction(&c) <= 1.0);

    /* slot table overflow: events dropped and counted, snapshot keeps truth */
    Coal d;
    check("drop_init_ok", coal_init(&d, SLOT_MAX) == 0);
    int dref[PRES_MAX], drn = 0, dcli[PRES_MAX], dcn = 0;
    for (int t = 1; t <= SLOT_MAX + 1; t++) {
        coal_event(&d, t, EV_ENTER);
        set_add(dref, &drn, PRES_MAX, t);
    }
    check("drop_counted", d.dropped == 1);
    coal_flush(&d, &b);
    check("overflow_forces_snapshot", b.mode == BATCH_SNAPSHOT);
    check("snap_after_drop_truth", set_equal(b.snap, b.nsnap, dref, drn));
    client_apply(dcli, &dcn, &b);
    check("state_match_drop", set_equal(dcli, dcn, dref, drn));

    /* randomized multi-frame stream: batch delivery must match per-event delivery */
    Coal r;
    coal_init(&r, 3);
    int rr[PRES_MAX], rrn = 0, rc[PRES_MAX], rcn = 0;
    unsigned seed = 12345u;
    int frames_ok = 1;
    for (int f = 0; f < 50; f++) {
        seed = seed * 1103515245u + 12345u;
        int events = 1 + (seed >> 16) % 20;
        for (int k = 0; k < events; k++) {
            seed = seed * 1103515245u + 12345u;
            int target = 1 + (seed >> 16) % 12;
            int kind = (seed >> 8) & 1;
            coal_event(&r, target, kind);
            if (kind == EV_ENTER) set_add(rr, &rrn, PRES_MAX, target);
            else set_del(rr, &rrn, target);
        }
        coal_flush(&r, &b);
        client_apply(rc, &rcn, &b);
        if (!set_equal(rc, rcn, rr, rrn)) frames_ok = 0;
    }
    check("random_consistency", frames_ok);
    check("random_exercised_snapshot", r.snapshots > 0 && r.raw_total > 100);
    check("random_reduction_bounded", coal_reduction(&r) >= 0.0 && coal_reduction(&r) <= 1.0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
