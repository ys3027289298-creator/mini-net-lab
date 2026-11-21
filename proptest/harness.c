/* proptest/harness.c — deterministic scenario runner over the real module code.
 *
 * The four module sources are #included directly (their main()/check()/
 * failures symbols renamed aside, `static` neutralized) so scenarios exercise
 * the shipped implementation, not a reimplementation. After every op the
 * module state is checked against an independent shadow model; any divergence
 * prints
 *   FAIL op=<line> invariant=<name> <detail>
 * and the process exits 1. Exit 0 means every invariant held.
 *
 * Scenario format: one op per line, '#' comments, blank lines ignored.
 *   param true_offset|dmin|dmax <f>
 *   push <tick> <x>                                prediction push (reconcile)
 *   pushn <tick0> <n>                              n pushes, x = tick
 *   snap <tick> <x> <y> <scale> <delay> <drop>     encode snapshot -> network
 *   deliver                                        flush in-flight packets in
 *                                                  delivery-time order, ack each
 *   clock <t_send> <t_recv> <remote>               one clock-sync sample
 *   clockn <n> <rtt0> <step> <off>                 n samples, rtt = rtt0+(i%5)*step,
 *                                                  symmetric delays, true offset off
 *   aoi_init <cell> <radius>
 *   aoi_add <id> <x> <y> / aoi_addn <id0> <n> / aoi_move <id> <x> <y>
 *   aoi_remove <id> / aoi_query <self> <x> <y>
 *   reset                                          re-init modules + shadow models
 *
 * Network model: each snap gets a random per-packet delay (reordering emerges
 * from delay variance when several packets are in flight) and a drop flag
 * (loss). deliver flushes all pending packets sorted by delivery time.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define static
#define main mod_main_snapshot
#define check mod_check_snapshot
#define failures mod_fail_snapshot
#include "../snapshot/src/snapshot.c"
#undef failures
#undef check
#undef main

#define main mod_main_clock
#define check mod_check_clock
#define failures mod_fail_clock
#include "../clock/src/clock.c"
#undef failures
#undef check
#undef main

#define main mod_main_aoi
#define check mod_check_aoi
#define failures mod_fail_aoi
#include "../aoi/src/aoi.c"
#undef failures
#undef check
#undef main

#define main mod_main_recon
#define check mod_check_recon
#define failures mod_fail_recon
#include "../reconcile/src/reconcile.c"
#undef failures
#undef check
#undef main
#undef static

/* ---------- failure reporting ---------- */
static int hfails = 0;
static int opline = 0;

static void fail(const char* inv, const char* detail) {
    printf("FAIL op=%d invariant=%s %s\n", opline, inv, detail);
    hfails++;
}

/* ---------- scenario params ---------- */
static double P_true_off = 0.0, P_dmin = 0.0, P_dmax = 0.0;

/* ---------- network ---------- */
#define MAXPEND 1024
typedef struct { double at; Packet pkt; State orig; float scale; } Pend;
static Pend pend[MAXPEND];
static int npend = 0;
static double now = 0.0;

/* ---------- reconcile shadow model ---------- */
static Recon R;
static unsigned m_tick[BUF];
static float m_x[BUF];
static int m_head, m_count;
static unsigned m_acked;
static int m_rollbacks;

static void model_reset(void) { m_head = m_count = 0; m_acked = 0; m_rollbacks = 0; }

static int model_find(unsigned tick, float* x) {
    for (int i = 0; i < m_count; i++) {
        int idx = (m_head - 1 - i + BUF * 2) % BUF;
        if (m_tick[idx] == tick) { if (x) *x = m_x[idx]; return 1; }
    }
    return 0;
}

static void op_push(unsigned tick, float x) {
    int exp = (tick <= m_acked || model_find(tick, NULL)) ? -1 : 0;
    int ret = recon_push(&R, tick, x);
    char d[160];
    if ((ret == 0) != (exp == 0)) {
        snprintf(d, sizeof d, "tick=%u x=%g ret=%d expected=%d", tick, x, ret, exp);
        fail("recon_push_ret", d);
    }
    if (exp == 0) {
        m_tick[m_head] = tick; m_x[m_head] = x;
        m_head = (m_head + 1) % BUF;
        if (m_count < BUF) m_count++;
    }
    if (R.acked != m_acked) { snprintf(d, sizeof d, "acked=%u model=%u", R.acked, m_acked); fail("recon_acked", d); }
    if (R.count != m_count) { snprintf(d, sizeof d, "count=%d model=%d", R.count, m_count); fail("recon_count", d); }
    if (R.rollbacks != m_rollbacks) { snprintf(d, sizeof d, "rollbacks=%d model=%d", R.rollbacks, m_rollbacks); fail("recon_rollbacks", d); }
}

static void do_ack(unsigned tick, float x) {
    float pred = 0.0f;
    int found = model_find(tick, &pred);
    float exp_err = found ? fabsf(pred - x) : -1.0f;
    int exp_rb = m_rollbacks + ((found && fabsf(pred - x) > 1e-3f) ? 1 : 0);
    unsigned exp_acked = tick > m_acked ? tick : m_acked;
    int ret = recon_ack(&R, tick, x);
    char d[192];
    if (ret != (found ? 1 : 0)) {
        snprintf(d, sizeof d, "tick=%u ret=%d found=%d", tick, ret, found);
        fail("recon_ack_ret", d);
    }
    if (fabsf(R.last_err - exp_err) > 1e-5f) {
        snprintf(d, sizeof d, "tick=%u last_err=%g expected=%g", tick, R.last_err, exp_err);
        fail("recon_ack_err", d);
    }
    if (R.rollbacks != exp_rb) {
        snprintf(d, sizeof d, "tick=%u rollbacks=%d expected=%d", tick, R.rollbacks, exp_rb);
        fail("recon_ack_rollback", d);
    }
    if (R.acked != exp_acked) {
        snprintf(d, sizeof d, "tick=%u acked=%u expected=%u", tick, R.acked, exp_acked);
        fail("recon_ack_acked", d);
    }
    m_acked = exp_acked; m_rollbacks = exp_rb;
}

/* ---------- snapshot + network ---------- */
static void check_roundtrip(const State* st, const Packet* p, float scale, const char* via) {
    State out; memset(&out, 0, sizeof out);
    char d[192];
    if (decode(p, &out, scale) <= 0) { fail(via, "decode of untampered packet failed"); return; }
    if (out.tick != st->tick) {
        snprintf(d, sizeof d, "tick=%u decoded=%u", st->tick, out.tick);
        fail("snap_roundtrip_tick", d);
    }
    float tol = 0.5f / scale + 1e-3f;
    if (fabsf(out.x - st->x) > tol || fabsf(out.y - st->y) > tol) {
        snprintf(d, sizeof d, "tick=%u in=(%g,%g) out=(%g,%g) scale=%g",
                 st->tick, st->x, st->y, out.x, out.y, scale);
        fail("snap_roundtrip_pos", d);
    }
}

static void op_snap(unsigned tick, float x, float y, float scale, double delay, int drop) {
    State st; memset(&st, 0, sizeof st);
    st.tick = tick; st.x = x; st.y = y;
    Packet p; memset(&p, 0, sizeof p);
    if (encode(&st, &p, scale) <= 0) { fail("snap_encode", "encode rejected valid state"); return; }
    check_roundtrip(&st, &p, scale, "snap_decode");
    /* any strict prefix must not be read past its end (and must not crash) */
    for (int L = 0; L < p.len; L++) {
        Packet q = p; q.len = L;
        State tmp; memset(&tmp, 0, sizeof tmp);
        if (decode(&q, &tmp, scale) > L) fail("snap_prefix_overread", "decode consumed past packet end");
    }
    if (!drop) {
        if (npend >= MAXPEND) { fail("net_overflow", "too many in-flight packets"); return; }
        pend[npend].at = now + delay;
        pend[npend].pkt = p;
        pend[npend].orig = st;
        pend[npend].scale = scale;
        npend++;
    }
}

static void op_deliver(void) {
    for (;;) {
        int best = -1;
        for (int i = 0; i < npend; i++)
            if (best < 0 || pend[i].at < pend[best].at) best = i;
        if (best < 0) break;
        Pend pd = pend[best];
        pend[best] = pend[--npend];
        if (pd.at > now) now = pd.at;
        State out; memset(&out, 0, sizeof out);
        if (decode(&pd.pkt, &out, pd.scale) <= 0) { fail("net_decode", "in-flight packet corrupted"); continue; }
        check_roundtrip(&pd.orig, &pd.pkt, pd.scale, "net_roundtrip");
        do_ack(out.tick, out.x);
    }
}

/* ---------- clock ---------- */
static Sync SY;
static double cts = 0.0;

static void op_clock(double ts, double tr, double remote) {
    int ret = sync_add(&SY, ts, tr, remote);
    char d[192];
    if (tr < ts) {
        if (ret != -1) { snprintf(d, sizeof d, "ret=%d for t_recv<t_send", ret); fail("clock_reject", d); }
        return;
    }
    if (ret <= 0) {
        snprintf(d, sizeof d, "valid sample rejected: ts=%g tr=%g best_rtt=%g n=%d",
                 ts, tr, SY.best_rtt, SY.n);
        fail("clock_accept", d);
        return;
    }
    /* every sample's offset estimate is true_off + (d_send - d_recv)/2 with
       one-way delays in [dmin,dmax], so any filtered average stays in bound */
    double bound = (P_dmax - P_dmin) * 0.5 + 1e-6;
    if (fabs(SY.offset - P_true_off) > bound) {
        snprintf(d, sizeof d, "offset=%g true=%g bound=%g", SY.offset, P_true_off, bound);
        fail("clock_offset_bound", d);
    }
}

static void op_clockn(int n, double rtt0, double step, double off) {
    for (int i = 0; i < n; i++) {
        double rtt = rtt0 + (i % 5) * step;
        double ts = cts; cts += 1.0;
        op_clock(ts, ts + rtt, ts + rtt * 0.5 + off);
    }
}

/* ---------- aoi shadow model ---------- */
static World W;
static float W_cell = 10.0f, W_radius = 15.0f;
typedef struct { int id; float x, y; int alive; } AEnt;
static AEnt ae[256];
static int an = 0;

static void op_aoi_init(float cell, float radius) {
    world_init(&W, cell, radius);
    W_cell = cell; W_radius = radius; an = 0;
}

static int aoi_alive_count(void) {
    int c = 0;
    for (int i = 0; i < an; i++) c += ae[i].alive;
    return c;
}

static int aoi_alive_id(int id) {
    for (int i = 0; i < an; i++) if (ae[i].alive && ae[i].id == id) return 1;
    return 0;
}

static void op_aoi_add(int id, float x, float y) {
    int ret = world_add(&W, id, x, y);
    char d[192];
    if (W_cell <= 0.0f) {
        if (ret != -1) fail("aoi_add_badcell", "add accepted with non-positive cell");
        return;
    }
    if (aoi_alive_id(id)) {
        if (ret != -1) { snprintf(d, sizeof d, "id=%d ret=%d", id, ret); fail("aoi_add_dup", d); }
        return;
    }
    if (aoi_alive_count() >= MAX_ENT) {
        if (ret != -1) { snprintf(d, sizeof d, "id=%d ret=%d", id, ret); fail("aoi_add_overflow", d); }
        return;
    }
    if (ret == -1) {
        snprintf(d, sizeof d, "id=%d alive=%d MAX_ENT=%d: rejected though live capacity free",
                 id, aoi_alive_count(), MAX_ENT);
        fail("aoi_add_capacity", d);
        return;
    }
    ae[an].id = id; ae[an].x = x; ae[an].y = y; ae[an].alive = 1; an++;
}

static void op_aoi_move(int id, float x, float y) {
    int ret = world_move(&W, id, x, y);
    int alive = aoi_alive_id(id);
    char d[160];
    if ((ret == 0) != alive) {
        snprintf(d, sizeof d, "id=%d ret=%d alive=%d", id, ret, alive);
        fail("aoi_move_ret", d);
    }
    if (alive) {
        for (int i = 0; i < an; i++)
            if (ae[i].alive && ae[i].id == id) { ae[i].x = x; ae[i].y = y; }
    }
}

static void op_aoi_remove(int id) {
    int ret = world_remove(&W, id);
    int alive = aoi_alive_id(id);
    char d[160];
    if ((ret == 0) != alive) {
        snprintf(d, sizeof d, "id=%d ret=%d alive=%d", id, ret, alive);
        fail("aoi_remove_ret", d);
    }
    if (alive) {
        for (int i = 0; i < an; i++)
            if (ae[i].alive && ae[i].id == id) ae[i].alive = 0;
    }
}

static void isort(int* a, int n) {
    for (int i = 1; i < n; i++) {
        int v = a[i], j = i - 1;
        while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
        a[j + 1] = v;
    }
}

static void op_aoi_query(int self, float x, float y) {
    int out[MAX_ENT];
    int n = world_query(&W, self, x, y, out, MAX_ENT);
    int exp[MAX_ENT], en = 0;
    float r2 = W_radius * W_radius;
    for (int i = 0; i < an; i++) {
        if (!ae[i].alive || ae[i].id == self) continue;
        float dx = ae[i].x - x, dy = ae[i].y - y;
        if (dx * dx + dy * dy <= r2) exp[en++] = ae[i].id;
    }
    char d[192];
    if (n != en) {
        snprintf(d, sizeof d, "self=%d at=(%g,%g) got=%d expected=%d", self, x, y, n, en);
        fail("aoi_query_count", d);
        return;
    }
    isort(out, n); isort(exp, en);
    for (int i = 0; i < n; i++) {
        if (out[i] != exp[i]) {
            snprintf(d, sizeof d, "self=%d at=(%g,%g) sets differ", self, x, y);
            fail("aoi_query_set", d);
            return;
        }
    }
}

/* ---------- reset ---------- */
static void do_reset(void) {
    recon_init(&R); model_reset();
    sync_init(&SY); cts = 0.0;
    npend = 0; now = 0.0;
    an = 0; world_init(&W, W_cell, W_radius);
}

/* ---------- driver ---------- */
int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: harness <scenario>\n"); return 2; }
    FILE* f = fopen(argv[1], "r");
    if (!f) { perror(argv[1]); return 2; }
    do_reset();
    char line[512];
    int ln = 0;
    while (fgets(line, sizeof line, f)) {
        ln++; opline = ln;
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\n' || *p == '\0') continue;
        char op[32];
        if (sscanf(p, "%31s", op) != 1) continue;
        if (!strcmp(op, "param")) {
            char key[32]; double v;
            if (sscanf(p, "%*s %31s %lf", key, &v) != 2) goto bad;
            if (!strcmp(key, "true_offset")) P_true_off = v;
            else if (!strcmp(key, "dmin")) P_dmin = v;
            else if (!strcmp(key, "dmax")) P_dmax = v;
            else { fprintf(stderr, "line %d: unknown param %s\n", ln, key); return 2; }
        } else if (!strcmp(op, "push")) {
            unsigned t; double x;
            if (sscanf(p, "%*s %u %lf", &t, &x) != 2) goto bad;
            op_push(t, (float)x);
        } else if (!strcmp(op, "pushn")) {
            unsigned t; int n;
            if (sscanf(p, "%*s %u %d", &t, &n) != 2) goto bad;
            for (int k = 0; k < n; k++) op_push(t + (unsigned)k, (float)(t + (unsigned)k));
        } else if (!strcmp(op, "snap")) {
            unsigned t; double x, y, sc, delay; int drop;
            if (sscanf(p, "%*s %u %lf %lf %lf %lf %d", &t, &x, &y, &sc, &delay, &drop) != 6) goto bad;
            op_snap(t, (float)x, (float)y, (float)sc, delay, drop);
        } else if (!strcmp(op, "deliver")) {
            op_deliver();
        } else if (!strcmp(op, "clock")) {
            double a, b, c;
            if (sscanf(p, "%*s %lf %lf %lf", &a, &b, &c) != 3) goto bad;
            op_clock(a, b, c);
        } else if (!strcmp(op, "clockn")) {
            int n; double r0, step, off;
            if (sscanf(p, "%*s %d %lf %lf %lf", &n, &r0, &step, &off) != 4) goto bad;
            op_clockn(n, r0, step, off);
        } else if (!strcmp(op, "aoi_init")) {
            double c, r;
            if (sscanf(p, "%*s %lf %lf", &c, &r) != 2) goto bad;
            op_aoi_init((float)c, (float)r);
        } else if (!strcmp(op, "aoi_add")) {
            int id; double x, y;
            if (sscanf(p, "%*s %d %lf %lf", &id, &x, &y) != 3) goto bad;
            op_aoi_add(id, (float)x, (float)y);
        } else if (!strcmp(op, "aoi_addn")) {
            int id0, n;
            if (sscanf(p, "%*s %d %d", &id0, &n) != 2) goto bad;
            for (int k = 0; k < n; k++) op_aoi_add(id0 + k, (float)k, 0.0f);
        } else if (!strcmp(op, "aoi_move")) {
            int id; double x, y;
            if (sscanf(p, "%*s %d %lf %lf", &id, &x, &y) != 3) goto bad;
            op_aoi_move(id, (float)x, (float)y);
        } else if (!strcmp(op, "aoi_remove")) {
            int id;
            if (sscanf(p, "%*s %d", &id) != 1) goto bad;
            op_aoi_remove(id);
        } else if (!strcmp(op, "aoi_query")) {
            int self; double x, y;
            if (sscanf(p, "%*s %d %lf %lf", &self, &x, &y) != 3) goto bad;
            op_aoi_query(self, (float)x, (float)y);
        } else if (!strcmp(op, "reset")) {
            do_reset();
        } else {
            fprintf(stderr, "line %d: unknown op %s\n", ln, op);
            return 2;
        }
        continue;
    bad:
        fprintf(stderr, "line %d: malformed op\n", ln);
        return 2;
    }
    fclose(f);
    printf("scenario_ok=%d\n", hfails == 0);
    return hfails ? 1 : 0;
}
