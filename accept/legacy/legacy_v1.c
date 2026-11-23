/*
 * legacy_v1.c - independent v1 reference codec for the compatibility matrix.
 *
 * Snapshot is the pinned 0bdd277 source (accept/legacy/snapshot_v1.c) compiled
 * with -Dmain=legacy_snapshot_pinned_main.  The other three are frozen copies
 * of the baseline struct layout; baseline had no byte codec for them, so these
 * functions define the v1 state-body contract they must stay readable by.
 */
#include <string.h>
#include <math.h>

#include "legacy_v1.h"

/* ---- snapshot: call the pinned 0bdd277 implementation ------------------ */
int legacy_snapshot_pinned_encode(const PState* s, PPacket* p, float scale);
int legacy_snapshot_pinned_decode(const PPacket* p, PState* s, float scale);

int legacy_snapshot_encode(unsigned tick, float x, float y, float pos_scale,
                           unsigned char* out, int cap) {
    PState s = { tick, x, y, 0, 0, 0 };
    PPacket p; memset(&p, 0, sizeof(p));
    int n = legacy_snapshot_pinned_encode(&s, &p, pos_scale);
    if (n <= 0 || n > cap) return -1;
    memcpy(out, p.buf, (size_t)n);
    return n;
}

int legacy_snapshot_decode(const unsigned char* in, int len, float pos_scale,
                           unsigned* tick, float* x, float* y) {
    PPacket p; memset(&p, 0, sizeof(p));
    if (len < 0 || len > (int)sizeof(p.buf)) return -1;
    memcpy(p.buf, in, (size_t)len);
    p.len = len;
    PState s;
    int n = legacy_snapshot_pinned_decode(&p, &s, pos_scale);
    if (n <= 0) return -1;
    *tick = s.tick; *x = s.x; *y = s.y;
    return n;   /* consumed exactly the v1 prefix, ignored anything past it */
}

/* ---- shared varint helpers ------------------------------------------- */
static int vput(unsigned char* b, int cap, unsigned v) {
    int n = 0;
    while (v >= 0x80) { if (n >= cap) return -1; b[n++] = (unsigned char)((v & 0x7f) | 0x80); v >>= 7; }
    if (n >= cap) return -1;
    b[n++] = (unsigned char)(v & 0x7f);
    return n;
}
static int vget(const unsigned char* b, int len, unsigned* out) {
    unsigned v = 0; int shift = 0, n = 0;
    while (n < len && shift <= 28) {
        unsigned char c = b[n++];
        v |= (unsigned)(c & 0x7f) << shift;
        if (!(c & 0x80)) { *out = v; return n; }
        shift += 7;
    }
    return -1;
}
static unsigned fb(float f) { unsigned u; memcpy(&u, &f, sizeof(u)); return u; }
static float bf(unsigned u) { float f; memcpy(&f, &u, sizeof(f)); return f; }
static unsigned zig(int v) { return ((unsigned)v << 1) ^ (unsigned)(v >> 31); }
static int unzig(unsigned v) { return (int)((v >> 1) ^ (unsigned)-(int)(v & 1)); }

static int dput(unsigned char* b, int cap, double d) {
    unsigned long long u; memcpy(&u, &d, sizeof(u));
    int n = vput(b, cap, (unsigned)(u >> 32));
    if (n < 0) return -1;
    int m = vput(b + n, cap - n, (unsigned)(u & 0xffffffffu));
    if (m < 0) return -1;
    return n + m;
}
static int dget(const unsigned char* b, int len, double* d) {
    unsigned hi = 0, lo = 0;
    int n = vget(b, len, &hi); if (n < 0) return -1;
    int m = vget(b + n, len - n, &lo); if (m < 0) return -1;
    unsigned long long u = ((unsigned long long)hi << 32) | lo;
    memcpy(d, &u, sizeof(*d));
    return n + m;
}

/* ---- clock v1 -------------------------------------------------------- */
int legacy_clock_encode_v1(const LClockV1* c, unsigned char* out, int cap) {
    int o = vput(out, cap, (unsigned)c->n); if (o < 0) return -1;
    for (int i = 0; i < c->n; i++) {
        int n = dput(out + o, cap - o, c->s[i].rtt); if (n < 0) return -1; o += n;
        n = dput(out + o, cap - o, c->s[i].offset); if (n < 0) return -1; o += n;
        n = dput(out + o, cap - o, c->s[i].at); if (n < 0) return -1; o += n;
    }
    int n = dput(out + o, cap - o, c->best_rtt); if (n < 0) return -1; o += n;
    n = dput(out + o, cap - o, c->offset); if (n < 0) return -1; o += n;
    return o;
}
int legacy_clock_decode_v1(const unsigned char* in, int len, LClockV1* c) {
    int o = 0, n; unsigned cnt = 0;
    n = vget(in, len, &cnt); if (n < 0 || cnt > L_MAX_SAMPLES) return -1; o += n;
    memset(c, 0, sizeof(*c)); c->best_rtt = 1e9;
    for (unsigned i = 0; i < cnt; i++) {
        n = dget(in + o, len - o, &c->s[i].rtt); if (n < 0) return -1; o += n;
        n = dget(in + o, len - o, &c->s[i].offset); if (n < 0) return -1; o += n;
        n = dget(in + o, len - o, &c->s[i].at); if (n < 0) return -1; o += n;
    }
    n = dget(in + o, len - o, &c->best_rtt); if (n < 0) return -1; o += n;
    n = dget(in + o, len - o, &c->offset); if (n < 0) return -1; o += n;
    c->n = (int)cnt;
    return o;
}

/* ---- aoi v1 ---------------------------------------------------------- */
int legacy_aoi_encode_v1(const LAoiV1* a, unsigned char* out, int cap) {
    int o = vput(out, cap, (unsigned)a->n); if (o < 0) return -1;
    for (int i = 0; i < a->n; i++) {
        int n = vput(out + o, cap - o, zig(a->e[i].id)); if (n < 0) return -1; o += n;
        n = vput(out + o, cap - o, fb(a->e[i].x)); if (n < 0) return -1; o += n;
        n = vput(out + o, cap - o, fb(a->e[i].y)); if (n < 0) return -1; o += n;
        if (o >= cap) return -1;
        out[o++] = (unsigned char)(a->e[i].alive ? 1 : 0);
    }
    int n = vput(out + o, cap - o, fb(a->cell)); if (n < 0) return -1; o += n;
    n = vput(out + o, cap - o, fb(a->radius)); if (n < 0) return -1; o += n;
    return o;
}
int legacy_aoi_decode_v1(const unsigned char* in, int len, LAoiV1* a) {
    int o = 0, n; unsigned cnt = 0;
    n = vget(in, len, &cnt); if (n < 0 || cnt > L_MAX_ENT) return -1; o += n;
    memset(a, 0, sizeof(*a)); a->n = (int)cnt;
    for (unsigned i = 0; i < cnt; i++) {
        unsigned zid = 0, fx = 0, fy = 0;
        n = vget(in + o, len - o, &zid); if (n < 0) return -1; o += n;
        n = vget(in + o, len - o, &fx); if (n < 0) return -1; o += n;
        n = vget(in + o, len - o, &fy); if (n < 0) return -1; o += n;
        if (o >= len) return -1;
        a->e[i].id = unzig(zid); a->e[i].x = bf(fx); a->e[i].y = bf(fy);
        a->e[i].alive = in[o++] ? 1 : 0;
    }
    unsigned fc = 0, fr = 0;
    n = vget(in + o, len - o, &fc); if (n < 0) return -1; o += n;
    n = vget(in + o, len - o, &fr); if (n < 0) return -1; o += n;
    a->cell = bf(fc); a->radius = bf(fr);
    return o;
}

/* ---- reconcile v1 ---------------------------------------------------- */
int legacy_recon_encode_v1(const LReconV1* r, unsigned char* out, int cap) {
    int o = vput(out, cap, (unsigned)r->count); if (o < 0) return -1;
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - r->count + i + L_BUF * 2) % L_BUF;
        int n = vput(out + o, cap - o, r->p[idx].tick); if (n < 0) return -1; o += n;
        n = vput(out + o, cap - o, fb(r->p[idx].x)); if (n < 0) return -1; o += n;
    }
    int n = vput(out + o, cap - o, r->acked); if (n < 0) return -1; o += n;
    n = vput(out + o, cap - o, (unsigned)r->rollbacks); if (n < 0) return -1; o += n;
    n = vput(out + o, cap - o, fb(r->last_err)); if (n < 0) return -1; o += n;
    return o;
}
int legacy_recon_decode_v1(const unsigned char* in, int len, LReconV1* r) {
    int o = 0, n; unsigned cnt = 0;
    n = vget(in, len, &cnt); if (n < 0 || cnt > L_BUF) return -1; o += n;
    memset(r, 0, sizeof(*r));
    for (unsigned i = 0; i < cnt; i++) {
        unsigned tick = 0, fx = 0;
        n = vget(in + o, len - o, &tick); if (n < 0) return -1; o += n;
        n = vget(in + o, len - o, &fx); if (n < 0) return -1; o += n;
        r->p[i].tick = tick; r->p[i].x = bf(fx);
    }
    unsigned acked = 0, rb = 0, fe = 0;
    n = vget(in + o, len - o, &acked); if (n < 0) return -1; o += n;
    n = vget(in + o, len - o, &rb); if (n < 0) return -1; o += n;
    n = vget(in + o, len - o, &fe); if (n < 0) return -1; o += n;
    r->count = (int)cnt; r->head = (int)cnt % L_BUF;
    r->acked = acked; r->rollbacks = (int)rb; r->last_err = bf(fe);
    return o;
}
