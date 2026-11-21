
/* aoievent: per-frame radius visible-set diff streamed as ordered enter/leave events.
 *
 * Each tick recomputes the visible set by radius, diffs it against the previous
 * frame, and enqueues one event per changed entity. Events leave the queue ordered
 * by (frame asc, entity slot asc): ticks run in frame order and each tick scans
 * slots once, so enqueue order already is the delivery order and the FIFO only has
 * to preserve it. Overflow is tail-dropped and counted; drops delete events but
 * never permute them, so the delivered stream is always a subsequence of the ideal
 * ordered stream. See README for the full enter/leave alternation argument.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_ENT 32
#define EVQ_CAP 16

enum { EV_LEAVE = 0, EV_ENTER = 1 };

typedef struct {
    unsigned frame;
    int id;       /* entity id, carried for the consumer */
    int slot;     /* stable entity order (insertion index; tombstones keep it stable) */
    int type;     /* EV_ENTER / EV_LEAVE */
} Event;

typedef struct {
    Event buf[EVQ_CAP];
    int head;
    int count;
    unsigned long dropped;   /* enqueue attempts lost to overflow */
} EvQueue;

/* FIFO push. returns 0 on success, -1 on overflow and counts the drop. */
static int evq_push(EvQueue* q, Event e) {
    if (q->count >= EVQ_CAP) { q->dropped++; return -1; }
    q->buf[(q->head + q->count) % EVQ_CAP] = e;
    q->count++;
    return 0;
}

static int evq_pop(EvQueue* q, Event* out) {
    if (q->count == 0) return 0;
    *out = q->buf[q->head];
    q->head = (q->head + 1) % EVQ_CAP;
    q->count--;
    return 1;
}

typedef struct { int id; float x, y; int alive; } Entity;

typedef struct {
    Entity e[MAX_ENT];
    int n;
    float radius;
    float x, y;              /* watcher position */
    int seen[MAX_ENT];       /* previous frame visible set, slot order */
    int seen_n;
    unsigned frame;
    EvQueue q;
} Watcher;

static void watcher_init(Watcher* w, float radius) {
    memset(w, 0, sizeof(*w));
    w->radius = radius;
}

static int watcher_add(Watcher* w, int id, float x, float y) {
    if (w->radius <= 0.0f || w->n >= MAX_ENT) return -1;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return -1;
    Entity* e = &w->e[w->n++];
    e->id = id; e->x = x; e->y = y; e->alive = 1;
    return w->n - 1;
}

static int watcher_move(Watcher* w, int id, float x, float y) {
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].x = x; w->e[i].y = y; return 0; }
    }
    return -1;
}

static int watcher_remove(Watcher* w, int id) {
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].alive = 0; return 0; }
    }
    return -1;
}

static int visible_contains(const Watcher* w, int slot) {
    const Entity* e = &w->e[slot];
    if (!e->alive) return 0;
    float dx = e->x - w->x, dy = e->y - w->y;
    return dx * dx + dy * dy <= w->radius * w->radius;
}

/* advance one frame: diff the radius visible set against the previous frame and
 * enqueue events in (frame, entity slot) order. returns events actually queued. */
static int watcher_tick(Watcher* w, float x, float y) {
    w->frame++;
    w->x = x; w->y = y;

    int cur[MAX_ENT], curn = 0;
    for (int s = 0; s < w->n; s++) if (visible_contains(w, s)) cur[curn++] = s;

    int emitted = 0;
    /* one slot-order scan: this frame's events are born in entity order, mixing
     * enters and leaves by slot rather than batching one type ahead of the other */
    for (int s = 0; s < w->n; s++) {
        int in_prev = 0, in_cur = 0;
        for (int i = 0; i < w->seen_n; i++) if (w->seen[i] == s) { in_prev = 1; break; }
        for (int i = 0; i < curn; i++) if (cur[i] == s) { in_cur = 1; break; }
        if (in_cur == in_prev) continue;       /* both present or both absent */
        Event ev = { w->frame, w->e[s].id, s, in_cur ? EV_ENTER : EV_LEAVE };
        if (evq_push(&w->q, ev) == 0) emitted++;
    }

    /* truth tracking follows the world regardless of delivery drops */
    memcpy(w->seen, cur, sizeof(cur[0]) * (size_t)curn);
    w->seen_n = curn;
    return emitted;
}

/* consumer under test: loads on ENTER, unloads on LEAVE. any LEAVE without a
 * matching outstanding ENTER is the "leave before enter" ordering failure. */
typedef struct {
    int active[MAX_ENT];
    int leave_before_enter;
    int double_enter;
} Consumer;

static void consumer_handle(Consumer* c, const Event* ev) {
    if (ev->type == EV_ENTER) {
        if (c->active[ev->slot]) c->double_enter++;
        c->active[ev->slot] = 1;
    } else {
        if (!c->active[ev->slot]) c->leave_before_enter++;
        c->active[ev->slot] = 0;
    }
}

/* buggy dispatcher for the counterexample: LIFO instead of frame-order FIFO. */
static int stack_push(Event* st, int* n, Event e) { st[(*n)++] = e; return 0; }
static int stack_pop(Event* st, int* n, Event* out) {
    if (*n == 0) return 0;
    *out = st[--*n];
    return 1;
}

/* independent oracle: brute-force full visible set every frame, no reused state
 * beyond its own previous set, set-diff in slot order. the watcher must match it. */
typedef struct { int prev[MAX_ENT]; int prev_n; } Oracle;

static int oracle_tick(const Watcher* w, Oracle* o, float x, float y,
                       unsigned frame, Event* out, int cap) {
    int cur[MAX_ENT], curn = 0;
    float r2 = w->radius * w->radius;
    for (int s = 0; s < w->n; s++) {
        const Entity* e = &w->e[s];
        if (!e->alive) continue;
        float dx = e->x - x, dy = e->y - y;
        if (dx * dx + dy * dy <= r2) cur[curn++] = s;
    }
    int k = 0;
    for (int s = 0; s < w->n; s++) {
        int in_prev = 0, in_cur = 0;
        for (int i = 0; i < o->prev_n; i++) in_prev |= (o->prev[i] == s);
        for (int i = 0; i < curn; i++) in_cur |= (cur[i] == s);
        if (in_cur && !in_prev && k < cap)
            out[k++] = (Event){ frame, w->e[s].id, s, EV_ENTER };
        if (in_prev && !in_cur && k < cap)
            out[k++] = (Event){ frame, w->e[s].id, s, EV_LEAVE };
    }
    memcpy(o->prev, cur, sizeof(cur[0]) * (size_t)curn);
    o->prev_n = curn;
    return k;
}

static unsigned long rng_state = 0x12345;
static unsigned rng(void) {
    rng_state = rng_state * 1103515245u + 12345u;
    return (rng_state >> 16) & 0x7fffu;
}
static float rng_float(float lo, float hi) { return lo + (hi - lo) * (rng() / 32768.0f); }

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* basic radius semantics */
    Watcher w;
    watcher_init(&w, 10.0f);
    check("add_ok", watcher_add(&w, 1, 0, 0) == 0);
    check("add_near", watcher_add(&w, 2, 6, 0) == 1);
    check("add_far", watcher_add(&w, 3, 50, 0) == 2);
    check("dup_rejected", watcher_add(&w, 2, 1, 1) == -1);
    check("move_missing", watcher_move(&w, 42, 0, 0) == -1);
    watcher_init(&w, 0.0f);
    check("bad_radius", watcher_add(&w, 1, 0, 0) == -1);

    /* boundary: distance == radius counts as visible, first tick emits ENTER */
    Watcher b;
    watcher_init(&b, 10.0f);
    watcher_add(&b, 1, 10.0f, 0.0f);
    watcher_add(&b, 2, 10.001f, 0.0f);
    watcher_tick(&b, 0, 0);
    Event ev;
    check("boundary_enter", b.q.count == 1 && evq_pop(&b.q, &ev) &&
                            ev.type == EV_ENTER && ev.id == 1 && ev.frame == 1);

    /* removal of a visible entity must surface as LEAVE on the next tick */
    Watcher rw;
    watcher_init(&rw, 10.0f);
    watcher_add(&rw, 1, 0, 0);
    watcher_tick(&rw, 0, 0);
    check("remove_ok", watcher_remove(&rw, 1) == 0);
    watcher_tick(&rw, 0, 0);
    int leave_n = 0;
    while (evq_pop(&rw.q, &ev))
        if (ev.type == EV_LEAVE && ev.frame == 2) leave_n++;
    check("remove_emits_leave", leave_n == 1);

    /* counterexample: the same two-event stream through FIFO vs LIFO.
     * f1 ENTER(7) sits queued, f2 LEAVE(7) is appended behind it. LIFO pops the
     * LEAVE first and the consumer immediately sees leave-before-enter. */
    Watcher cw;
    watcher_init(&cw, 10.0f);
    watcher_add(&cw, 7, 1.0f, 0.0f);
    watcher_tick(&cw, 0, 0);
    watcher_move(&cw, 7, 50.0f, 0.0f);
    watcher_tick(&cw, 0, 0);
    check("counter_stream_depth", cw.q.count == 2);

    Consumer good = { 0 };
    EvQueue fifo_copy = cw.q;
    while (evq_pop(&fifo_copy, &ev)) consumer_handle(&good, &ev);
    check("fifo_no_violation", good.leave_before_enter == 0 && good.double_enter == 0);

    Event st[8]; int sn = 0;
    EvQueue lifo_src = cw.q;
    while (evq_pop(&lifo_src, &ev)) stack_push(st, &sn, ev);
    Consumer bad = { 0 };
    while (stack_pop(st, &sn, &ev)) consumer_handle(&bad, &ev);
    check("lifo_leave_before_enter", bad.leave_before_enter == 1);

    /* equivalence with per-frame full comparison, plus ordering and consumer
     * alternation invariants over a randomized 60-frame run (drained each frame,
     * so the queue never overflows here). */
    Watcher f;
    watcher_init(&f, 20.0f);
    for (int i = 0; i < 8; i++)
        watcher_add(&f, i + 1, rng_float(-30, 30), rng_float(-30, 30));
    Oracle orc; memset(&orc, 0, sizeof(orc));
    Consumer cons = { 0 };
    int eq = 1, order_ok = 1;
    unsigned last_frame = 0; int last_slot = -1;
    for (int t = 0; t < 60; t++) {
        for (int i = 0; i < f.n; i++)
            if ((rng() & 1) && f.e[i].alive)
                watcher_move(&f, f.e[i].id, rng_float(-30, 30), rng_float(-30, 30));
        float wx = rng_float(-25, 25), wy = rng_float(-25, 25);

        watcher_tick(&f, wx, wy);
        Event got[32], want[32];
        int gn = 0;
        while (evq_pop(&f.q, &ev)) got[gn++] = ev;
        int wn = oracle_tick(&f, &orc, wx, wy, f.frame, want, 32);
        if (gn != wn) { eq = 0; }
        for (int i = 0; i < gn && i < wn; i++) {
            if (got[i].frame != want[i].frame || got[i].slot != want[i].slot ||
                got[i].type != want[i].type || got[i].id != want[i].id) eq = 0;
            if (got[i].frame < last_frame ||
                (got[i].frame == last_frame && got[i].slot <= last_slot)) order_ok = 0;
            last_frame = got[i].frame; last_slot = got[i].slot;
            consumer_handle(&cons, &got[i]);
        }
    }
    check("matches_full_scan_oracle", eq);
    check("delivery_frame_slot_order", order_ok);
    check("consumer_never_leave_first", cons.leave_before_enter == 0 &&
                                       cons.double_enter == 0);
    int final_ok = 1;
    for (int s = 0; s < f.n; s++)
        if (cons.active[s] != visible_contains(&f, s)) final_ok = 0;
    check("consumer_set_matches_visible", final_ok);

    /* overflow: 20 enters into a 16-deep queue, tail dropped and counted,
     * surviving events keep their (frame, slot) order; counter is cumulative. */
    Watcher ow;
    watcher_init(&ow, 100.0f);
    for (int i = 0; i < 20; i++) watcher_add(&ow, 100 + i, 0, 0);
    int queued = watcher_tick(&ow, 0, 0);
    check("overflow_tail_dropped", ow.q.count == EVQ_CAP &&
                                   ow.q.dropped == (unsigned long)(20 - EVQ_CAP) &&
                                   queued == EVQ_CAP);
    int order_kept = 1;
    for (int i = 0; i < EVQ_CAP; i++) {
        if (!evq_pop(&ow.q, &ev) || ev.slot != i || ev.type != EV_ENTER ||
            ev.frame != 1) order_kept = 0;
    }
    check("overflow_preserves_order", order_kept && ow.q.count == 0);
    for (int i = 0; i < 20; i++) watcher_move(&ow, 100 + i, 999.0f, 999.0f);
    watcher_tick(&ow, 0, 0);
    check("dropped_counter_cumulative", ow.q.dropped ==
                                        (unsigned long)(2 * (20 - EVQ_CAP)));

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
