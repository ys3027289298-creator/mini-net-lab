/* hier: layered clock synchronization with per-layer drift compensation.
 *
 * Nodes are arranged in layers.  Layer 0 is the root authority; every other
 * node estimates its offset to a parent in the layer above.  The end-to-end
 * offset of a leaf is the composition of the per-layer offsets, each
 * extrapolated from its own measured drift and weighted with its own
 * uncertainty.  A "manual" composition that freezes the last measured offset
 * at every layer is kept alongside, so the drift compensation can be compared
 * against the naive layer-by-layer result and bounded.
 *
 * All times are seconds on the resolving node's clock.  Drift is expressed as
 * offset units per second.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define HIER_MAX_LAYERS 8
#define HIER_MAX_NODES 16
#define HIER_MAX_SAMPLES 8
#define HIER_DEGRADE_INFLATE 4.0
#define HIER_STALE_AFTER 30.0

typedef enum {
    HIER_OK = 0,
    HIER_LINK_DOWN,
    HIER_STALE
} HierReason;

typedef struct { double rtt; double offset; double mid; } HierSample;

typedef struct {
    int layer;
    int parent;
    int linked;                 /* link to parent is up */
    HierSample s[HIER_MAX_SAMPLES];
    int n;
    double best_rtt;
    double anchor_offset;       /* offset of the min-RTT sample */
    double anchor_at;           /* its midpoint timestamp */
    double anchor_rtt;
    double drift;               /* estimated offset drift, units/second */
    double drift_unc;           /* bound on drift estimation error */
    double last_at;
} HierNode;

typedef struct {
    HierNode node[HIER_MAX_NODES];
    int n;
    int layers;
    double stale_after;         /* 0 disables the staleness check */
} Hier;

typedef struct {
    double offset;              /* drift-compensated end-to-end offset */
    double manual;              /* naive sum of last per-layer offsets */
    double bound;               /* upper bound on |offset - true offset| */
    double deviation;           /* |offset - manual| */
    double deviation_bound;     /* upper bound on the deviation */
    int degraded;
    HierReason reason;          /* first degradation along the path */
    int degraded_layer;
    int degraded_node;
} HierResult;

static const char* hier_reason_str(HierReason r) {
    switch (r) {
        case HIER_OK: return "ok";
        case HIER_LINK_DOWN: return "link-down";
        case HIER_STALE: return "stale";
        default: return "unknown";
    }
}

/* Zero layers is meaningless: refuse to build a hierarchy. */
static int hier_init(Hier* h, int layers) {
    if (h == NULL) return -1;
    if (layers <= 0 || layers > HIER_MAX_LAYERS) return -1;
    memset(h, 0, sizeof(*h));
    h->layers = layers;
    h->stale_after = HIER_STALE_AFTER;
    for (int i = 0; i < HIER_MAX_NODES; i++) h->node[i].parent = -1;
    return 0;
}

static int hier_set_stale_after(Hier* h, double secs) {
    if (h == NULL || !isfinite(secs) || secs < 0.0) return -1;
    h->stale_after = secs;
    return 0;
}

/* Layer 0 has exactly one root (parent -1); layer k's parent must be in k-1. */
static int hier_add_node(Hier* h, int layer, int parent) {
    if (h == NULL || h->n >= HIER_MAX_NODES) return -1;
    if (layer < 0 || layer >= h->layers) return -1;
    if (layer == 0) {
        if (parent != -1) return -1;
        for (int i = 0; i < h->n; i++) if (h->node[i].layer == 0) return -1;
    } else {
        if (parent < 0 || parent >= h->n) return -1;
        if (h->node[parent].layer != layer - 1) return -1;
    }
    int id = h->n++;
    HierNode* nd = &h->node[id];
    memset(nd, 0, sizeof(*nd));
    nd->layer = layer;
    nd->parent = parent;
    nd->linked = 1;
    nd->best_rtt = 1e9;
    return id;
}

static int hier_set_link(Hier* h, int node, int up) {
    if (h == NULL || node < 0 || node >= h->n) return -1;
    if (h->node[node].parent < 0) return -1;   /* root has no upstream link */
    h->node[node].linked = up ? 1 : 0;
    return 0;
}

/* Recompute the best-RTT anchor and the endpoint drift estimate. */
static void node_recompute(HierNode* nd) {
    int bi = 0;
    double best = 1e9;
    for (int i = 0; i < nd->n; i++) {
        if (nd->s[i].rtt < best) { best = nd->s[i].rtt; bi = i; }
    }
    nd->best_rtt = best;
    nd->anchor_offset = nd->s[bi].offset;
    nd->anchor_at = nd->s[bi].mid;
    nd->anchor_rtt = nd->s[bi].rtt;

    nd->drift = 0.0;
    nd->drift_unc = 0.0;
    if (nd->n >= 2) {
        const HierSample* a = &nd->s[0];
        const HierSample* b = &nd->s[nd->n - 1];
        double span = b->mid - a->mid;
        if (span > 1e-12) {
            nd->drift = (b->offset - a->offset) / span;
            /* A measured offset is off by at most rtt/2 (plus the drift over
             * one RTT); the slope therefore inherits those errors over span. */
            double ea = a->rtt * 0.5 * (1.0 + fabs(nd->drift));
            double eb = b->rtt * 0.5 * (1.0 + fabs(nd->drift));
            nd->drift_unc = (ea + eb) / span;
        }
    }
}

/* Add one offset sample for node against its parent.
 * t_send/t_recv are local; remote_time is the parent's clock reading. */
static int hier_sample(Hier* h, int node, double t_send, double t_recv, double remote_time) {
    if (h == NULL || node < 0 || node >= h->n) return -1;
    if (!isfinite(t_send) || !isfinite(t_recv) || !isfinite(remote_time)) return -1;
    HierNode* nd = &h->node[node];
    if (nd->parent < 0) return -1;      /* root has no upstream to sample */
    if (!nd->linked) return -1;         /* no samples arrive over a dead link */
    if (t_recv < t_send) return -1;

    double mid = (t_send + t_recv) * 0.5;
    if (nd->n > 0 && mid <= nd->s[nd->n - 1].mid) return -1;   /* local time went backwards */

    if (nd->n >= HIER_MAX_SAMPLES) {
        memmove(&nd->s[0], &nd->s[1], sizeof(HierSample) * (HIER_MAX_SAMPLES - 1));
        nd->n = HIER_MAX_SAMPLES - 1;
    }
    HierSample* sp = &nd->s[nd->n++];
    sp->rtt = t_recv - t_send;
    sp->offset = remote_time - mid;
    sp->mid = mid;
    nd->last_at = t_recv;
    node_recompute(nd);
    return nd->n;
}

/* Resolve the end-to-end offset of leaf at local time now. */
static int hier_resolve(const Hier* h, int leaf, double now, HierResult* out) {
    if (h == NULL || out == NULL || !isfinite(now)) return -1;
    if (leaf < 0 || leaf >= h->n) return -1;

    memset(out, 0, sizeof(*out));
    out->reason = HIER_OK;
    out->degraded_layer = -1;
    out->degraded_node = -1;

    /* Root -> leaf path, so uncertainty can be accumulated top-down. */
    int path[HIER_MAX_LAYERS];
    int plen = 0;
    int cur = leaf;
    while (cur >= 0) {
        if (plen >= HIER_MAX_LAYERS) return -1;
        path[plen++] = cur;
        cur = h->node[cur].parent;
    }

    int degraded = 0;
    HierReason reason = HIER_OK;
    int degraded_layer = -1, degraded_node = -1;

    for (int i = plen - 1; i >= 0; i--) {
        const HierNode* nd = &h->node[path[i]];
        if (nd->layer == 0) continue;               /* the root is the reference */
        if (nd->n < 2) return -1;                   /* no drift estimate yet */

        double age = now - nd->anchor_at;
        if (age < 0.0) age = 0.0;

        HierReason lr = HIER_OK;
        if (!nd->linked) lr = HIER_LINK_DOWN;
        else if (h->stale_after > 0.0 && now - nd->last_at > h->stale_after) lr = HIER_STALE;

        double inflate = 1.0;
        if (lr != HIER_OK) {
            inflate = HIER_DEGRADE_INFLATE;
            if (!degraded) {
                degraded = 1;
                reason = lr;
                degraded_layer = nd->layer;
                degraded_node = path[i];
            }
        }

        double emax = nd->anchor_rtt * 0.5 * (1.0 + fabs(nd->drift));
        double layer_bound = emax + nd->drift_unc * age;

        out->offset += nd->anchor_offset + nd->drift * age;
        out->manual += nd->anchor_offset;
        out->bound += layer_bound * inflate;
        out->deviation_bound += (fabs(nd->drift) + nd->drift_unc) * age * inflate;
    }

    out->deviation = fabs(out->offset - out->manual);
    out->degraded = degraded;
    out->reason = reason;
    out->degraded_layer = degraded_layer;
    out->degraded_node = degraded_node;
    return 0;
}

/* Human-readable explanation of a resolve result. */
static int hier_explain(const HierResult* r, char* buf, int cap) {
    if (r == NULL || buf == NULL || cap <= 0) return -1;
    if (!r->degraded) {
        snprintf(buf, (size_t)cap, "ok");
    } else {
        snprintf(buf, (size_t)cap, "degraded: layer %d %s (node %d)",
                 r->degraded_layer, hier_reason_str(r->reason), r->degraded_node);
    }
    return 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Simulated one-way delays and the resulting offset measurement. */
static int feed(Hier* h, int node, double t, double up, double down,
                double base, double drift) {
    double remote_at = t + up;
    double remote_time = remote_at + base + drift * remote_at;
    return hier_sample(h, node, t, t + up + down, remote_time);
}

static int build_chain(Hier* h) {
    if (hier_init(h, 3) != 0) return -1;
    if (hier_add_node(h, 0, -1) != 0) return -1;
    if (hier_add_node(h, 1, 0) != 1) return -1;
    if (hier_add_node(h, 2, 1) != 2) return -1;
    return 0;
}

static const double up1[6] = { 0.0016, 0.0010, 0.0014, 0.0009, 0.0012, 0.0010 };
static const double dn1[6] = { 0.0014, 0.0010, 0.0011, 0.0008, 0.0013, 0.0011 };
static const double up2[6] = { 0.0020, 0.0016, 0.0014, 0.0012, 0.0010, 0.0013 };
static const double dn2[6] = { 0.0010, 0.0014, 0.0016, 0.0018, 0.0010, 0.0017 };

static void feed_chain(Hier* h) {
    for (int i = 0; i < 6; i++) {
        feed(h, 1, (double)i, up1[i], dn1[i], 5.0, 0.001);
        feed(h, 2, (double)i, up2[i], dn2[i], -2.0, 0.002);
    }
}

static void test_init_and_shape(void) {
    Hier h;
    check("init_zero_layers_rejected", hier_init(&h, 0) == -1);
    check("init_negative_layers_rejected", hier_init(&h, -1) == -1);
    check("init_too_many_layers_rejected", hier_init(&h, HIER_MAX_LAYERS + 1) == -1);
    check("init_three_layers", hier_init(&h, 3) == 0);

    check("root_requires_no_parent", hier_add_node(&h, 0, 0) == -1);
    check("layer1_before_root_rejected", hier_add_node(&h, 1, -1) == -1);
    check("root_added", hier_add_node(&h, 0, -1) == 0);
    check("second_root_rejected", hier_add_node(&h, 0, -1) == -1);
    check("layer_out_of_range_rejected", hier_add_node(&h, 3, 0) == -1);
    check("parent_layer_mismatch_rejected", hier_add_node(&h, 2, 0) == -1);
    check("layer1_added", hier_add_node(&h, 1, 0) == 1);
    check("layer2_added", hier_add_node(&h, 2, 1) == 2);

    check("sample_on_root_rejected", hier_sample(&h, 0, 0.0, 0.1, 0.0) == -1);
    check("root_link_rejected", hier_set_link(&h, 0, 0) == -1);
    check("time_reversal_rejected", hier_sample(&h, 1, 5.0, 4.0, 0.0) == -1);
}

static void test_end_to_end(void) {
    Hier h;
    check("chain_built", build_chain(&h) == 0);
    feed_chain(&h);

    HierResult r;
    check("resolve_ok", hier_resolve(&h, 2, 20.0, &r) == 0);
    check("resolve_healthy", !r.degraded && r.reason == HIER_OK);
    check("bound_positive", r.bound > 0.0);

    /* True client-to-root offset: (5 + 0.001 t) + (-2 + 0.002 t). */
    double truth = 3.0 + 0.003 * 20.0;
    check("hier_within_bound", fabs(r.offset - truth) <= r.bound + 1e-9);
    check("deviation_within_bound", r.deviation <= r.deviation_bound + 1e-9);
    check("compensation_nonzero", fabs(r.offset - r.manual) > 1e-3);
    check("manual_drifts_outside_bound", fabs(r.manual - truth) > r.bound);

    /* Both layers drifted; the compensated result must beat the frozen one. */
    check("hier_beats_manual", fabs(r.offset - truth) < fabs(r.manual - truth));

    double old_bound = r.bound;

    /* Re-sync at query time: the bound collapses but the drift stays. */
    check("resync_l1", feed(&h, 1, 20.0, 0.0008, 0.0008, 5.0, 0.001) > 0);
    check("resync_l2", feed(&h, 2, 20.0, 0.0008, 0.0008, -2.0, 0.002) > 0);
    check("resolve_after_resync", hier_resolve(&h, 2, 20.0, &r) == 0);
    check("fresh_bound_smaller", r.bound < old_bound);
    check("fresh_offset_close", fabs(r.offset - truth) < 0.005);
}

static void test_degrade(void) {
    Hier h;
    check("degrade_chain", build_chain(&h) == 0);
    feed_chain(&h);

    HierResult ok;
    check("baseline_resolve", hier_resolve(&h, 2, 20.0, &ok) == 0);

    check("link_down_server", hier_set_link(&h, 1, 0) == 0);
    check("sample_blocked_when_down",
          feed(&h, 1, 30.0, 0.001, 0.001, 5.0, 0.001) == -1);

    HierResult r;
    check("degraded_resolve_ok", hier_resolve(&h, 2, 20.0, &r) == 0);
    check("degraded_flag", r.degraded == 1);
    check("degraded_reason_link", r.reason == HIER_LINK_DOWN);
    check("degraded_layer_reported", r.degraded_layer == 1 && r.degraded_node == 1);
    check("degraded_bound_inflated", r.bound > ok.bound);
    check("degraded_still_estimates", fabs(r.offset - ok.offset) < 1e-9);

    char why[96];
    check("explain_ok", hier_explain(&r, why, (int)sizeof(why)) == 0);
    check("explain_link", strstr(why, "layer 1") != NULL && strstr(why, "link-down") != NULL);

    check("link_up", hier_set_link(&h, 1, 1) == 0);
    check("resolve_after_link_up", hier_resolve(&h, 2, 20.0, &r) == 0);
    check("recovered", !r.degraded);

    check("link_down_client", hier_set_link(&h, 2, 0) == 0);
    check("client_degraded", hier_resolve(&h, 2, 20.0, &r) == 0 &&
                             r.degraded && r.degraded_layer == 2 &&
                             r.reason == HIER_LINK_DOWN);

    check("stale_rejected_value", hier_set_stale_after(&h, -1.0) == -1);
    check("stale_config", hier_set_stale_after(&h, 5.0) == 0);
    check("link_up_client", hier_set_link(&h, 2, 1) == 0);
    check("stale_flagged", hier_resolve(&h, 2, 20.0, &r) == 0 &&
                           r.degraded && r.reason == HIER_STALE &&
                           r.degraded_layer == 1);
}

static void test_incomplete(void) {
    Hier h;
    check("incomplete_chain", build_chain(&h) == 0);
    for (int i = 0; i < 6; i++) feed(&h, 1, (double)i, up1[i], dn1[i], 5.0, 0.001);

    HierResult r;
    check("unsynced_layer_rejected", hier_resolve(&h, 2, 10.0, &r) == -1);
    check("one_sample_rejected",
          feed(&h, 2, 0.0, 0.001, 0.001, -2.0, 0.002) > 0 &&
          hier_resolve(&h, 2, 10.0, &r) == -1);
    check("two_samples_ok",
          feed(&h, 2, 1.0, 0.001, 0.001, -2.0, 0.002) > 0 &&
          hier_resolve(&h, 2, 10.0, &r) == 0);

    check("nonmonotonic_rejected", feed(&h, 2, 0.5, 0.001, 0.001, -2.0, 0.002) == -1);

    HierResult root;
    check("root_resolves_zero", hier_resolve(&h, 0, 10.0, &root) == 0 &&
                                root.offset == 0.0 && root.bound == 0.0 &&
                                !root.degraded);
}

int main(void) {
    test_init_and_shape();
    test_end_to_end();
    test_degrade();
    test_incomplete();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
