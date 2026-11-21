
/* observer: per-observer AOI subscriptions, cross-view consistency checks,
   full-broadcast baseline comparison and disconnect cleanup. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_OBS 8
#define MAX_ENT 16
#define MAX_VIEW 16

enum { DIFF_STATE = 0, DIFF_MISSING = 1, DIFF_EXTRA = 2 };

typedef struct { int id; float x, y; unsigned rev; int alive; } Ent;
typedef struct { int id; float x, y; unsigned rev; } Copy;
typedef struct { int observer; int entity; int kind; } Diff;

typedef struct {
    int id;
    int alive;
    float x, y;
    Copy view[MAX_VIEW];
    int nview;
} Obs;

typedef struct {
    Ent ent[MAX_ENT];
    int nent;
    Obs obs[MAX_OBS];
    int nobs;
    float radius;
} Net;

static void net_init(Net* n, float radius) {
    memset(n, 0, sizeof(*n));
    n->radius = radius;
}

static Ent* ent_find(Net* n, int id) {
    for (int i = 0; i < n->nent; i++)
        if (n->ent[i].alive && n->ent[i].id == id) return &n->ent[i];
    return 0;
}

static const Ent* ent_get(const Net* n, int id) {
    for (int i = 0; i < n->nent; i++)
        if (n->ent[i].alive && n->ent[i].id == id) return &n->ent[i];
    return 0;
}

static Obs* obs_find(Net* n, int id) {
    for (int i = 0; i < n->nobs; i++)
        if (n->obs[i].alive && n->obs[i].id == id) return &n->obs[i];
    return 0;
}

static const Obs* obs_get(const Net* n, int id) {
    for (int i = 0; i < n->nobs; i++)
        if (n->obs[i].alive && n->obs[i].id == id) return &n->obs[i];
    return 0;
}

static int view_index(const Obs* o, int id) {
    for (int i = 0; i < o->nview; i++)
        if (o->view[i].id == id) return i;
    return -1;
}

static int net_add_entity(Net* n, int id, float x, float y) {
    if (n->radius <= 0.0f) return -1;
    if (ent_find(n, id)) return -1;
    Ent* slot = 0;
    for (int i = 0; i < n->nent; i++)
        if (!n->ent[i].alive) { slot = &n->ent[i]; break; }
    if (!slot) {
        if (n->nent >= MAX_ENT) return -1;
        slot = &n->ent[n->nent++];
    }
    slot->id = id; slot->x = x; slot->y = y; slot->rev = 0; slot->alive = 1;
    return 0;
}

static int net_move_entity(Net* n, int id, float x, float y) {
    Ent* e = ent_find(n, id);
    if (!e) return -1;
    e->x = x; e->y = y; e->rev++;
    return 0;
}

static int net_remove_entity(Net* n, int id) {
    Ent* e = ent_find(n, id);
    if (!e) return -1;
    e->alive = 0;
    return 0;
}

static int net_add_observer(Net* n, int id, float x, float y) {
    if (n->radius <= 0.0f) return -1;
    if (obs_find(n, id)) return -1;
    Obs* slot = 0;
    for (int i = 0; i < n->nobs; i++)
        if (!n->obs[i].alive) { slot = &n->obs[i]; break; }
    if (!slot) {
        if (n->nobs >= MAX_OBS) return -1;
        slot = &n->obs[n->nobs++];
    }
    memset(slot, 0, sizeof(*slot));
    slot->id = id; slot->alive = 1; slot->x = x; slot->y = y;
    return 0;
}

static int net_observer_move(Net* n, int id, float x, float y) {
    Obs* o = obs_find(n, id);
    if (!o) return -1;
    o->x = x; o->y = y;
    return 0;
}

/* authoritative visible set for one observer; count may exceed cap */
static int net_visible(const Net* n, int oid, int* out, int cap) {
    const Obs* o = obs_get(n, oid);
    if (!o || n->radius <= 0.0f) return -1;
    int cnt = 0;
    float r2 = n->radius * n->radius;
    for (int i = 0; i < n->nent; i++) {
        const Ent* e = &n->ent[i];
        if (!e->alive) continue;
        float dx = e->x - o->x, dy = e->y - o->y;
        if (dx * dx + dy * dy <= r2) {
            if (cnt < cap) out[cnt] = e->id;
            cnt++;
        }
    }
    return cnt;
}

/* reconcile one observer's subscriptions to the authoritative visible set */
static int net_sync(Net* n, int oid) {
    Obs* o = obs_find(n, oid);
    if (!o) return -1;
    int vis[MAX_VIEW];
    int m = net_visible(n, oid, vis, MAX_VIEW);
    if (m < 0) return -1;
    /* leaves (and dead entities still held): drop subscriptions */
    for (int i = o->nview - 1; i >= 0; i--) {
        int keep = 0;
        for (int j = 0; j < m; j++)
            if (vis[j] == o->view[i].id) { keep = 1; break; }
        if (!keep) {
            o->view[i] = o->view[o->nview - 1];
            o->nview--;
        }
    }
    /* enters subscribe fresh, retained entries refresh to authority */
    for (int j = 0; j < m; j++) {
        const Ent* e = ent_get(n, vis[j]);
        int idx = view_index(o, vis[j]);
        Copy* c;
        if (idx < 0) {
            if (o->nview >= MAX_VIEW) continue;
            c = &o->view[o->nview++];
            c->id = e->id;
        } else {
            c = &o->view[idx];
        }
        c->x = e->x; c->y = e->y; c->rev = e->rev;
    }
    return o->nview;
}

/* full-broadcast reference: every alive entity with authoritative state */
static int net_full_broadcast(const Net* n, Copy* out, int cap) {
    int cnt = 0;
    for (int i = 0; i < n->nent; i++) {
        const Ent* e = &n->ent[i];
        if (!e->alive) continue;
        if (cnt < cap) out[cnt] = (Copy){ e->id, e->x, e->y, e->rev };
        cnt++;
    }
    return cnt;
}

static int diff_emit(Diff* out, int cap, int k, int obs, int ent, int kind) {
    if (k < cap) out[k] = (Diff){ obs, ent, kind };
    return k + 1;
}

static int state_mismatch(const Copy* c, const Ent* e) {
    return c->rev != e->rev || c->x != e->x || c->y != e->y;
}

/* diff one observer's view against the full-broadcast baseline:
   visible entities must be present with exact state, invisible ones absent */
static int net_compare_broadcast(const Net* n, int oid, Diff* out, int cap) {
    const Obs* o = obs_get(n, oid);
    if (!o) return -1;
    int vis[MAX_VIEW];
    int m = net_visible(n, oid, vis, MAX_VIEW);
    if (m < 0) return -1;
    Copy bc[MAX_ENT];
    int bm = net_full_broadcast(n, bc, MAX_ENT);
    int k = 0;
    for (int i = 0; i < bm; i++) {
        const Copy* b = &bc[i];
        int should = 0;
        for (int j = 0; j < m; j++)
            if (vis[j] == b->id) { should = 1; break; }
        int idx = view_index(o, b->id);
        if (should) {
            const Ent* e = ent_get(n, b->id);
            if (idx < 0) {
                k = diff_emit(out, cap, k, oid, b->id, DIFF_MISSING);
            } else if (state_mismatch(&o->view[idx], e)) {
                k = diff_emit(out, cap, k, oid, b->id, DIFF_STATE);
            }
        } else if (idx >= 0) {
            k = diff_emit(out, cap, k, oid, b->id, DIFF_EXTRA);
        }
    }
    return k;
}

/* subscriptions an observer still holds despite them leaving its visible set */
static int cross_extras(const Net* n, const Obs* o, const int* vis, int m,
                        Diff* out, int cap, int k) {
    (void)n;
    for (int i = 0; i < o->nview; i++) {
        int eid = o->view[i].id;
        int should = 0;
        for (int j = 0; j < m; j++)
            if (vis[j] == eid) { should = 1; break; }
        if (!should) k = diff_emit(out, cap, k, o->id, eid, DIFF_EXTRA);
    }
    return k;
}

/* cross-check two observers: for entities both should see, each side's copy
   is compared to authority, localizing every diff to observer + entity;
   stale subscriptions outside either visible set are reported as EXTRA */
static int net_cross_check(const Net* n, int a, int b, Diff* out, int cap) {
    const Obs *oa = obs_get(n, a), *ob = obs_get(n, b);
    if (!oa || !ob) return -1;
    int va[MAX_VIEW], vb[MAX_VIEW];
    int ma = net_visible(n, a, va, MAX_VIEW);
    int mb = net_visible(n, b, vb, MAX_VIEW);
    if (ma < 0 || mb < 0) return -1;
    int k = 0;
    for (int i = 0; i < ma; i++) {
        int shared = 0;
        for (int j = 0; j < mb; j++)
            if (vb[j] == va[i]) { shared = 1; break; }
        if (!shared) continue;
        int eid = va[i];
        const Ent* e = ent_get(n, eid);
        const Obs* sides[2] = { oa, ob };
        for (int s = 0; s < 2; s++) {
            int idx = view_index(sides[s], eid);
            if (idx < 0) {
                k = diff_emit(out, cap, k, sides[s]->id, eid, DIFF_MISSING);
            } else if (state_mismatch(&sides[s]->view[idx], e)) {
                k = diff_emit(out, cap, k, sides[s]->id, eid, DIFF_STATE);
            }
        }
    }
    k = cross_extras(n, oa, va, ma, out, cap, k);
    k = cross_extras(n, ob, vb, mb, out, cap, k);
    return k;
}

/* observer goes offline: its subscriptions are dropped, slot freed for reuse */
static int net_disconnect(Net* n, int id) {
    Obs* o = obs_find(n, id);
    if (!o) return -1;
    o->alive = 0;
    o->nview = 0;
    return 0;
}

static int net_subscription_count(const Net* n) {
    int total = 0;
    for (int i = 0; i < n->nobs; i++)
        if (n->obs[i].alive) total += n->obs[i].nview;
    return total;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int has_diff(const Diff* d, int k, int obs, int ent, int kind) {
    for (int i = 0; i < k; i++)
        if (d[i].observer == obs && d[i].entity == ent && d[i].kind == kind)
            return 1;
    return 0;
}

int main(void) {
    Net n;
    Diff d[16];
    int vis[8], k;

    /* two players standing close; entity 12 is far out of sight */
    net_init(&n, 15.0f);
    check("ent_add", net_add_entity(&n, 10, 5, 0) == 0);
    check("ent_add2", net_add_entity(&n, 11, 6, 1) == 0);
    check("ent_far", net_add_entity(&n, 12, 100, 100) == 0);
    check("ent_dup_rejected", net_add_entity(&n, 10, 0, 0) == -1);

    check("obs_add", net_add_observer(&n, 1, 0, 0) == 0);
    check("obs_add2", net_add_observer(&n, 2, 4, 0) == 0);
    check("obs_dup_rejected", net_add_observer(&n, 1, 9, 9) == -1);

    /* each observer computes its own visible set */
    k = net_visible(&n, 1, vis, 8);
    check("visible_set", k == 2 && vis[0] == 10 && vis[1] == 11);
    k = net_visible(&n, 1, vis, 1);
    check("visible_cap_truncates", k == 2 && vis[0] == 10);

    /* subscriptions built from enter/leave diffs */
    check("sync1", net_sync(&n, 1) == 2);
    check("sync2", net_sync(&n, 2) == 2);
    check("subs_tracked", net_subscription_count(&n) == 4);

    /* invariant: versus full-broadcast baseline both views match exactly */
    check("broadcast_clean_a", net_compare_broadcast(&n, 1, d, 16) == 0);
    check("broadcast_clean_b", net_compare_broadcast(&n, 2, d, 16) == 0);
    check("cross_clean", net_cross_check(&n, 1, 2, d, 16) == 0);

    /* the reported bug: player 1 keeps a stale copy of shared entity 10 */
    Obs* o1 = obs_find(&n, 1);
    o1->view[view_index(o1, 10)].x = 99.0f;
    k = net_cross_check(&n, 1, 2, d, 16);
    check("cross_localizes_state",
          k == 1 && has_diff(d, k, 1, 10, DIFF_STATE) &&
          !has_diff(d, k, 2, 10, DIFF_STATE));
    check("broadcast_flags_stale", net_compare_broadcast(&n, 1, d, 16) == 1);
    check("other_side_clean", net_compare_broadcast(&n, 2, d, 16) == 0);

    /* resync converges back to zero cross-check diffs */
    check("resync", net_sync(&n, 1) == 2);
    check("reconverged", net_cross_check(&n, 1, 2, d, 16) == 0);

    /* missed subscription: player 1 lost entity 11 */
    int vi = view_index(o1, 11);
    o1->view[vi] = o1->view[o1->nview - 1];
    o1->nview--;
    k = net_cross_check(&n, 1, 2, d, 16);
    check("cross_localizes_missing", k == 1 && has_diff(d, k, 1, 11, DIFF_MISSING));
    check("broadcast_localizes_missing",
          net_compare_broadcast(&n, 1, d, 16) == 1 &&
          has_diff(d, 1, 1, 11, DIFF_MISSING));

    /* stale extra subscription: player 1 still holds far entity 12 */
    o1->view[o1->nview++] = (Copy){ 12, 100.0f, 100.0f, 0 };
    k = net_compare_broadcast(&n, 1, d, 16);
    check("broadcast_missing_and_extra",
          k == 2 && has_diff(d, k, 1, 11, DIFF_MISSING) &&
          has_diff(d, k, 1, 12, DIFF_EXTRA));
    k = net_cross_check(&n, 1, 2, d, 16);
    check("cross_missing_and_extra",
          k == 2 && has_diff(d, k, 1, 11, DIFF_MISSING) &&
          has_diff(d, k, 1, 12, DIFF_EXTRA));
    check("sync_repairs", net_sync(&n, 1) == 2);
    check("zero_after_repair", net_cross_check(&n, 1, 2, d, 16) == 0);

    /* entity leaves the area: leave diff cleans the subscription on sync */
    check("move_bumps_rev", net_move_entity(&n, 11, 100, 100) == 0 &&
          ent_get(&n, 11)->rev == 1);
    k = net_compare_broadcast(&n, 1, d, 16);
    check("flagged_before_sync", k == 1 && has_diff(d, k, 1, 11, DIFF_EXTRA));
    check("sync_leave_a", net_sync(&n, 1) == 1);
    check("sync_leave_b", net_sync(&n, 2) == 1);
    check("zero_after_leave", net_cross_check(&n, 1, 2, d, 16) == 0);
    check("subs_after_leave", net_subscription_count(&n) == 2);

    /* observer walking away loses everything, walking back re-subscribes */
    check("obs_move", net_observer_move(&n, 2, 200, 200) == 0);
    check("sync_empty_view", net_sync(&n, 2) == 0);
    check("subs_when_away", net_subscription_count(&n) == 1);
    net_observer_move(&n, 2, 4, 0);
    check("sync_resubscribe", net_sync(&n, 2) == 1);

    /* disconnect must release subscriptions instead of accumulating them */
    check("third_observer", net_add_observer(&n, 3, 0, 0) == 0);
    check("sync3", net_sync(&n, 3) == 1);
    check("subs_three", net_subscription_count(&n) == 3);
    check("disconnect", net_disconnect(&n, 1) == 0);
    check("disconnect_again_rejected", net_disconnect(&n, 1) == -1);
    check("subs_released", net_subscription_count(&n) == 2);
    check("dead_no_sync", net_sync(&n, 1) == -1);
    check("dead_no_visible", net_visible(&n, 1, vis, 8) == -1);
    check("peer_view_untouched", obs_find(&n, 2)->nview == 1);

    /* same id reconnects with a fresh empty view: no leak, no accumulation */
    check("readd_same_id", net_add_observer(&n, 1, 0, 0) == 0);
    check("fresh_view", obs_find(&n, 1)->nview == 0);
    check("readd_sync", net_sync(&n, 1) == 1);
    check("no_accumulation", net_subscription_count(&n) == 3);

    /* entity removed from the world drops out of every view on sync */
    check("remove_entity", net_remove_entity(&n, 10) == 0);
    check("sync_all_after_remove",
          net_sync(&n, 1) == 0 && net_sync(&n, 2) == 0 && net_sync(&n, 3) == 0);
    check("subs_all_gone", net_subscription_count(&n) == 0);
    check("zero_with_no_entities", net_cross_check(&n, 1, 2, d, 16) == 0);
    Copy bc[16];
    check("broadcast_lists_alive", net_full_broadcast(&n, bc, 16) == 2);

    /* error paths and capacity limits on a fresh world */
    Net t;
    net_init(&t, 0.0f);
    check("bad_radius_entity", net_add_entity(&t, 1, 0, 0) == -1);
    check("bad_radius_observer", net_add_observer(&t, 1, 0, 0) == -1);

    net_init(&t, 15.0f);
    for (int i = 0; i < MAX_ENT; i++)
        if (net_add_entity(&t, 100 + i, (float)i, 0) != 0) check("ent_fill", 0);
    check("ent_full", net_add_entity(&t, 999, 0, 0) == -1);
    check("ent_tombstone_reuse",
          net_remove_entity(&t, 100) == 0 && net_add_entity(&t, 100, 0, 0) == 0);
    for (int i = 0; i < MAX_OBS; i++)
        if (net_add_observer(&t, 200 + i, 0, 0) != 0) check("obs_fill", 0);
    check("obs_full", net_add_observer(&t, 999, 0, 0) == -1);

    check("unknown_visible", net_visible(&t, 42, vis, 8) == -1);
    check("unknown_compare", net_compare_broadcast(&t, 42, d, 16) == -1);
    check("unknown_cross", net_cross_check(&t, 200, 42, d, 16) == -1);
    check("unknown_move", net_observer_move(&t, 42, 0, 0) == -1);
    check("move_missing", net_move_entity(&t, 4242, 0, 0) == -1);
    check("remove_missing", net_remove_entity(&t, 4242) == -1);

    k = net_full_broadcast(&t, bc, 1);
    check("broadcast_cap_truncates", k == MAX_ENT && bc[0].id == 100);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
