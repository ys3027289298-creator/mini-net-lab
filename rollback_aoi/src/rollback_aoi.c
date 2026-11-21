
/* rollback_aoi: keep the AOI visible set stable while the server rolls back.
 *
 * A naive client replaces its view with whatever the authoritative frame says,
 * so when an old tick arrives the view jumps into the past and entities that
 * are currently on screen blink off (and back on when the client catches up).
 *
 * This module keeps the currently drawn set across a rollback:
 *   - the frame from the past is diffed against the *current* set, not
 *     against the historical set;
 *   - only ids missing from the current set are sent (the difference);
 *   - an id that is/was already on screen is merged instead of respawned;
 *   - AOI leaves during the rollback window are deferred and applied only
 *     once a frame reaches the old high-water tick again.
 *
 * A rollback deeper than the configured window cannot be trusted against the
 * local history: it is reported and degraded to a full snapshot.
 */
#include <stdio.h>
#include <string.h>

#define MAX_ENT 32
#define MAX_HIST 64

typedef struct { int id; float x, y; int alive; } Entity;
typedef struct { Entity e[MAX_ENT]; int n; } World;
typedef struct { int ids[MAX_ENT]; int n; } IdSet;

typedef struct { int id; unsigned last; } HistEnt;

typedef struct {
    IdSet shown;                 /* what is drawn right now */
    IdSet deferred;              /* drawn, but a rolled-back frame dropped it */
    HistEnt hist[MAX_HIST];      /* id -> last tick it was drawn */
    int nhist;
    unsigned cur_tick;           /* high-water tick, never moves backwards */
    int has_tick;
    int max_depth;               /* rollback window the client can diff */

    int frames;
    int sends, reenters, leaves;
    int flash_frames;            /* frames that dropped something still visible */
    int full_resyncs;
    int depth_reports;
    unsigned last_depth_report;
} Client;

typedef struct {
    unsigned tick;
    int depth;                   /* cur_tick - tick, 0 when moving forward */
    int sends;
    int reenters;                /* merged instead of respawned */
    int leaves;
    int deferred;
    int full;                    /* degraded to a full snapshot */
} Frame;

/* ---- id set ------------------------------------------------------------ */

static int idset_has(const IdSet* s, int id) {
    for (int i = 0; i < s->n; i++) if (s->ids[i] == id) return 1;
    return 0;
}

static int idset_add(IdSet* s, int id) {
    if (idset_has(s, id)) return 0;
    if (s->n >= MAX_ENT) return -1;
    s->ids[s->n++] = id;
    return 1;
}

static int idset_del(IdSet* s, int id) {
    for (int i = 0; i < s->n; i++) {
        if (s->ids[i] == id) { s->ids[i] = s->ids[--s->n]; return 1; }
    }
    return 0;
}

/* ---- world / AOI query ------------------------------------------------- */

static void world_init(World* w) { memset(w, 0, sizeof(*w)); }

static int world_add(World* w, int id, float x, float y) {
    if (w->n >= MAX_ENT) return -1;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return -1;
    w->e[w->n].id = id; w->e[w->n].x = x; w->e[w->n].y = y; w->e[w->n].alive = 1;
    return w->n++;
}

static int world_move(World* w, int id, float x, float y) {
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].x = x; w->e[i].y = y; return 0; }
    }
    return -1;
}

static int world_remove(World* w, int id) {
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].alive = 0; return 0; }
    }
    return -1;
}

/* authoritative visible set around (cx,cy): live entities inside radius */
static int visible_at(const World* w, float cx, float cy, float radius, IdSet* out) {
    float r2 = radius * radius;
    out->n = 0;
    for (int i = 0; i < w->n; i++) {
        const Entity* e = &w->e[i];
        if (!e->alive) continue;
        float dx = e->x - cx, dy = e->y - cy;
        if (dx * dx + dy * dy <= r2) idset_add(out, e->id);
    }
    return out->n;
}

/* ---- client ------------------------------------------------------------ */

static void client_init(Client* c, int max_depth) {
    memset(c, 0, sizeof(*c));
    c->max_depth = max_depth;
}

static int hist_find(const Client* c, int id) {
    for (int i = 0; i < c->nhist; i++) if (c->hist[i].id == id) return i;
    return -1;
}

/* remember that id was drawn at `tick` (keep the newest tick) */
static void hist_touch(Client* c, int id, unsigned tick) {
    int i = hist_find(c, id);
    if (i < 0) {
        if (c->nhist < MAX_HIST) {
            c->hist[c->nhist].id = id;
            c->hist[c->nhist].last = tick;
            c->nhist++;
        }
        return;
    }
    if (tick > c->hist[i].last) c->hist[i].last = tick;
}

/* was id drawn at `tick` or at some tick after it? then it must be merged */
static int hist_since(const Client* c, int id, unsigned tick) {
    int i = hist_find(c, id);
    return i >= 0 && c->hist[i].last >= tick;
}

/* Apply one authoritative frame. The drawn set is only ever added to inside
 * the rollback window; removals are deferred until the window closes. */
static void client_frame(Client* c, const IdSet* auth, unsigned tick, Frame* out) {
    memset(out, 0, sizeof(*out));
    out->tick = tick;

    int depth = (c->has_tick && tick < c->cur_tick)
              ? (int)(c->cur_tick - tick) : 0;
    out->depth = depth;

    if (depth > c->max_depth) {
        /* Too far into the past to trust the local history: report and let
         * the caller push a full snapshot. This is the one place the view is
         * rebuilt, so it is accounted separately from a flash. */
        c->depth_reports++;
        c->full_resyncs++;
        c->last_depth_report = (unsigned)depth;
        out->full = 1;
        c->shown.n = 0;
        c->deferred.n = 0;
        c->nhist = 0;
        for (int i = 0; i < auth->n; i++) {
            idset_add(&c->shown, auth->ids[i]);
            hist_touch(c, auth->ids[i], tick);
            out->sends++;
        }
        c->sends += out->sends;
        c->frames++;
        return;
    }

    if (!c->has_tick || tick >= c->cur_tick) {
        /* Forward (or the last replayed tick): the rollback window is over,
         * so real AOI leaves are applied now. */
        c->deferred.n = 0;
        for (int i = 0; i < c->shown.n; ) {
            if (!idset_has(auth, c->shown.ids[i])) {
                idset_del(&c->shown, c->shown.ids[i]);
                out->leaves++;
                i = 0;                      /* del swaps the last element in */
            } else {
                i++;
            }
        }
        for (int i = 0; i < auth->n; i++) {
            int id = auth->ids[i];
            if (!idset_has(&c->shown, id)) {
                if (hist_since(c, id, tick)) out->reenters++;
                else out->sends++;
                idset_add(&c->shown, id);
            }
            hist_touch(c, id, tick);
        }
        c->cur_tick = tick;
        c->has_tick = 1;
    } else {
        /* Rollback inside the window: diff the past frame against the
         * currently drawn set and keep everything that is already visible. */
        for (int i = 0; i < auth->n; i++) {
            int id = auth->ids[i];
            if (idset_del(&c->deferred, id)) {
                out->reenters++;            /* it came back: merge, no send */
            } else if (!idset_has(&c->shown, id)) {
                if (hist_since(c, id, tick)) out->reenters++;
                else out->sends++;          /* genuinely new: send the diff */
                idset_add(&c->shown, id);
            }
            hist_touch(c, id, tick);
        }
        for (int i = 0; i < c->shown.n; i++) {
            int id = c->shown.ids[i];
            if (idset_has(auth, id)) continue;
            if (idset_add(&c->deferred, id) == 1) out->deferred++;
        }
    }

    c->sends += out->sends;
    c->reenters += out->reenters;
    c->leaves += out->leaves;
    c->frames++;
}

/* Compare the drawn set against the set a client that never rolls back would
 * show. Any id the reference still has but we dropped is a flash frame. */
static int client_observe(Client* c, const IdSet* ref) {
    int missing = 0;
    for (int i = 0; i < ref->n; i++)
        if (!idset_has(&c->shown, ref->ids[i])) missing++;
    if (missing > 0) c->flash_frames++;
    return missing;
}

/* ---- comparison clients ------------------------------------------------ */

/* No-rollback reference: follows authoritative frames in tick order and
 * ignores stale (rewound) frames, so its view never jumps into the past. */
typedef struct { IdSet shown; unsigned tick; int has; } Ref;

static void ref_frame(Ref* r, const IdSet* auth, unsigned tick) {
    if (!r->has || tick >= r->tick) {
        r->shown = *auth;
        r->tick = tick;
        r->has = 1;
    }
}

/* Control: the old behaviour, where the view is replaced by every frame. */
typedef struct { IdSet shown; int flash_frames; } Naive;

static void naive_frame(Naive* n, const IdSet* auth, const IdSet* ref) {
    n->shown = *auth;
    for (int i = 0; i < ref->n; i++)
        if (!idset_has(&n->shown, ref->ids[i])) { n->flash_frames++; break; }
}

/* ---- self-test --------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int set_eq(const IdSet* a, const IdSet* b) {
    if (a->n != b->n) return 0;
    for (int i = 0; i < a->n; i++) if (!idset_has(b, a->ids[i])) return 0;
    return 1;
}

int main(void) {
    World w;
    IdSet auth[8];
    Frame f;

    /* Timeline: 2 is near, 4 moves in at t2, 2 leaves and 3 enters at t3,
     * 2 comes back at t4, 3 despawns at t5. */
    world_init(&w);
    world_add(&w, 2, 5.0f, 0.0f);
    world_add(&w, 3, 100.0f, 100.0f);
    world_add(&w, 4, 100.0f, 100.0f);
    visible_at(&w, 0, 0, 15.0f, &auth[1]);
    world_move(&w, 4, 5.0f, 5.0f);
    visible_at(&w, 0, 0, 15.0f, &auth[2]);
    world_move(&w, 2, 100.0f, 0.0f);
    world_move(&w, 3, 6.0f, 6.0f);
    visible_at(&w, 0, 0, 15.0f, &auth[3]);
    world_move(&w, 2, 5.0f, 0.0f);
    visible_at(&w, 0, 0, 15.0f, &auth[4]);
    world_remove(&w, 3);
    visible_at(&w, 0, 0, 15.0f, &auth[5]);

    check("auth_sets", auth[1].n == 1 && auth[2].n == 2 &&
                       auth[3].n == 2 && auth[4].n == 3 && auth[5].n == 2);

    /* Deliver t1..t4, then rewind to t2 and replay up to t5. */
    const unsigned script[] = { 1, 2, 3, 4, 2, 1, 2, 3, 4, 5 };
    const int nscript = (int)(sizeof(script) / sizeof(script[0]));

    Client c;
    Ref ref;
    Naive naive;
    client_init(&c, 3);
    memset(&ref, 0, sizeof(ref));
    memset(&naive, 0, sizeof(naive));

    int rollback_frames = 0, sends_in_rollback = 0;
    for (int k = 0; k < nscript; k++) {
        unsigned t = script[k];
        unsigned before = c.cur_tick;
        int had_tick = c.has_tick;
        client_frame(&c, &auth[t], t, &f);
        ref_frame(&ref, &auth[t], t);
        client_observe(&c, &ref.shown);
        naive_frame(&naive, &auth[t], &ref.shown);

        if (had_tick && t < before) {
            rollback_frames++;
            sends_in_rollback += f.sends;
        }
        if (k >= 4) {                    /* replay of the rollback window */
            check("replay_no_send", f.sends == 0);
        }
    }

    check("rollback_frames_seen", rollback_frames == 4);
    check("diff_only_during_rollback", sends_in_rollback == 0);
    check("reenters_merged", c.reenters == 2);
    check("zero_flash_vs_no_rollback", c.flash_frames == 0);
    check("naive_does_flash", naive.flash_frames > 0);
    check("sends_only_new", c.sends == 4);
    check("leaves_after_window", c.leaves == 2);
    check("no_false_degrade", c.full_resyncs == 0 && c.depth_reports == 0);

    IdSet final_expect;
    final_expect.n = 0;
    idset_add(&final_expect, 2);
    idset_add(&final_expect, 4);
    check("converged_to_reference", set_eq(&c.shown, &final_expect) &&
                                    set_eq(&c.shown, &ref.shown));

    /* Re-entry through history while the id is not currently drawn: it is
     * merged (no spawn) rather than sent again. */
    Client m;
    client_init(&m, 3);
    IdSet five, none;
    five.n = 0; idset_add(&five, 5);
    none.n = 0;
    client_frame(&m, &five, 1, &f);       /* send 5 */
    client_frame(&m, &none, 2, &f);       /* 5 leaves normally */
    client_frame(&m, &five, 1, &f);       /* rollback: 5 re-enters */
    check("history_reenter_merged", f.reenters == 1 && f.sends == 0);
    check("history_no_resend", m.sends == 1);

    /* Depth at the limit is still diffed; the drawn set is untouched. */
    Client b;
    client_init(&b, 3);
    for (unsigned t = 1; t <= 4; t++) client_frame(&b, &auth[t], t, &f);
    client_frame(&b, &auth[1], 1, &f);
    check("depth_at_limit_ok", f.depth == 3 && f.sends == 0 &&
                               b.depth_reports == 0 && b.shown.n == 3);

    /* Deeper than the window: report the depth, degrade to a full snapshot
     * (every visible id is sent and the drawn set is rebuilt). */
    Client d;
    client_init(&d, 2);
    for (unsigned t = 1; t <= 4; t++) client_frame(&d, &auth[t], t, &f);
    client_frame(&d, &auth[1], 1, &f);
    check("deep_reported", f.depth == 3 && d.depth_reports == 1 &&
                           d.last_depth_report == 3);
    check("deep_full_snapshot", f.full == 1 && f.sends == auth[1].n &&
                                d.full_resyncs == 1);
    check("deep_set_rebuilt", set_eq(&d.shown, &auth[1]));
    check("high_water_kept", d.cur_tick == 4);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
