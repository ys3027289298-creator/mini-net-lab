
/* snapshot: quantized state snapshot encode/decode with varint tick. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../../timebase/src/timebase.h"
#include "../../timebase/src/accept.h"

#define SNAP_MAX 64

typedef struct {
    unsigned tick;
    float x, y, vx, vy;
    unsigned flags;
} State;

typedef struct {
    unsigned char buf[SNAP_MAX];
    int len;
} Packet;

static int varint_put(unsigned char* b, unsigned v) {
    int n = 0;
    while (v >= 0x80) { b[n++] = (unsigned char)((v & 0x7f) | 0x80); v >>= 7; }
    b[n++] = (unsigned char)(v & 0x7f);
    return n;
}

static int varint_get(const unsigned char* b, int len, unsigned* out) {
    unsigned v = 0; int shift = 0, n = 0;
    while (n < len && shift <= 28) {
        unsigned char c = b[n++];
        v |= (unsigned)(c & 0x7f) << shift;
        if (!(c & 0x80)) { *out = v; return n; }
        shift += 7;
    }
    return -1;
}

static int quant(float v, float scale) { return (int)lrintf(v * scale); }

/* Zigzag in unsigned arithmetic: (q << 1) ^ (q >> 31) on a signed int shifts
 * a negative value left, which is undefined in C11.  These produce the same
 * bits without signed overflow. */
static unsigned zigzag(int q) {
    unsigned u = (unsigned)q;
    return (u << 1) ^ (unsigned)-(q < 0);
}

static unsigned unzigzag(unsigned z) {   /* inverse of zigzag, as unsigned */
    return (z >> 1) ^ (unsigned)-(int)(z & 1u);
}

static int encode(const State* s, Packet* p, float pos_scale) {
    if (p == NULL) return -1;
    if (pos_scale <= 0.0f) return -1;
    int n = varint_put(p->buf, s->tick);
    int qx = quant(s->x, pos_scale), qy = quant(s->y, pos_scale);
    unsigned zx = zigzag(qx);
    unsigned zy = zigzag(qy);
    n += varint_put(p->buf + n, zx);
    n += varint_put(p->buf + n, zy);
    p->len = n;
    return n;
}

static int decode(const Packet* p, State* s, float pos_scale) {
    if (p == NULL || s == NULL) return -1;
    if (pos_scale <= 0.0f) return -1;
    unsigned tick = 0, zx = 0, zy = 0;
    int n = varint_get(p->buf, p->len, &tick);
    if (n < 0) return -1;
    int o = n;
    n = varint_get(p->buf + o, p->len - o, &zx); if (n < 0) return -1; o += n;
    n = varint_get(p->buf + o, p->len - o, &zy); if (n < 0) return -1; o += n;
    s->tick = tick;
    s->x = (float)(int)unzigzag(zx) / pos_scale;
    s->y = (float)(int)unzigzag(zy) / pos_scale;
    s->vx = 0; s->vy = 0; s->flags = 0;
    return o;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Wire-format regression vectors captured from the original encoder, before
 * the time base existed.  The success path must stay byte-for-byte identical. */
typedef struct { unsigned tick; float x, y; float scale; const char* hex; } WireVec;

static const WireVec wire[] = {
    { 0u,     0.0f,      0.0f,     1.0f,  "000000" },
    { 1u,     1.0f,     -1.0f,     1.0f,  "010201" },
    { 127u,   12.5f,    -3.25f,    64.0f, "7fc00c9f03" },
    { 128u,   -1000.0f, 1000.0f,   4.0f,  "8001bf3ec03e" },
    { 1000u,  12.5f,    -3.25f,    4.0f,  "e8076419" },
    { 70000u, 1000.0f,  -1000.0f,  64.0f, "f0a20480e807ffe707" },
};

static int wire_matches(const WireVec* v) {
    State s = { v->tick, v->x, v->y, 0, 0, 0 };
    Packet p; memset(&p, 0, sizeof(p));
    int n = encode(&s, &p, v->scale);
    if (n <= 0 || (size_t)(n * 2) != strlen(v->hex)) return 0;
    for (int i = 0; i < n; i++) {
        char byte[3];
        snprintf(byte, sizeof(byte), "%02x", p.buf[i]);
        if (byte[0] != v->hex[i * 2] || byte[1] != v->hex[i * 2 + 1]) return 0;
    }
    return 1;
}

/* Acceptance step: the snapshot's tick comes from the shared time base, the
 * wire bytes must be unchanged, and decode must round-trip that tick. */
static int snap_accept_step(void* ctx, TimeBase* tb, const TBAcceptSpec* spec, uint32_t tick) {
    (void)ctx; (void)spec;
    uint32_t base = 0;
    if (tb_now(tb, &base) != TB_OK || base < tick) return 6;   /* base must lead */
    State in, out;
    in.tick = tick;
    in.x = (float)(tick % 1000u) * 0.25f;
    in.y = -(float)(tick % 700u) * 0.5f;
    in.vx = in.vy = 0.0f; in.flags = 0u;
    Packet p; memset(&p, 0, sizeof(p));
    if (encode(&in, &p, 64.0f) <= 0) return 1;
    if (decode(&p, &out, 64.0f) <= 0) return 2;
    if (out.tick != tick) return 3;
    if (fabsf(out.x - in.x) > 1.0f / 64.0f + 1e-6f) return 4;
    if (fabsf(out.y - in.y) > 1.0f / 64.0f + 1e-6f) return 5;
    return 0;
}

static int snapshot_accept(void) {
    int wire_ok = 1;
    for (unsigned i = 0; i < sizeof(wire) / sizeof(wire[0]); i++) {
        if (!wire_matches(&wire[i])) { wire_ok = 0; break; }
    }
    printf("module=snapshot phase=wire vectors=%u ok=%d\n",
           (unsigned)(sizeof(wire) / sizeof(wire[0])), wire_ok);

    TBAcceptSpec spec;
    memset(&spec, 0, sizeof(spec));
    spec.module = TB_SNAPSHOT;
    spec.step_ms = 16;          /* 60 Hz snapshot cadence */
    spec.events = 1000;
    spec.long_events = 450000;  /* 2 h of frames at 16 ms */
    spec.ppm = 100;             /* source runs 100 ppm off nominal */
    spec.seed = 0x51a9u;
    spec.step = snap_accept_step;
    int rc = wire_ok ? tb_accept_run(&spec) : 1;
    tb_accept_shared(TB_SNAPSHOT);   /* shared base, compared across modules */
    return wire_ok ? rc : 1;
}

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "--accept") == 0) return snapshot_accept();

    Packet p;
    State in = { 1000u, 12.5f, -3.25f, 0, 0, 1u }, out;
    memset(&p, 0, sizeof(p));
    check("encode_ok", encode(&in, &p, 64.0f) > 0);
    check("size_small", p.len <= 12);
    check("decode_ok", decode(&p, &out, 64.0f) > 0);
    check("tick_roundtrip", out.tick == in.tick);
    check("pos_quantized", fabsf(out.x - in.x) <= 1.0f / 64.0f + 1e-6f &&
                           fabsf(out.y - in.y) <= 1.0f / 64.0f + 1e-6f);

    check("bad_scale", encode(&in, &p, 0.0f) == -1);
    check("bad_packet", decode(&(Packet){ .len = 2 }, &out, 64.0f) == -1);

    State big = { 70000u, 1000.0f, -1000.0f, 0, 0, 0 };
    Packet p2; memset(&p2, 0, sizeof(p2));
    encode(&big, &p2, 4.0f);
    State out2;
    check("big_roundtrip", decode(&p2, &out2, 4.0f) > 0 && out2.tick == big.tick &&
                            fabsf(out2.x - big.x) <= 0.25f + 1e-4f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
