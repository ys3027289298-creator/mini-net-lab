
/* snapshot: quantized state snapshot encode/decode with varint tick. */
#include <stdio.h>
#include <string.h>
#include <math.h>

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

static int encode(const State* s, Packet* p, float pos_scale) {
    if (p == NULL) return -1;
    if (pos_scale <= 0.0f) return -1;
    int n = varint_put(p->buf, s->tick);
    int qx = quant(s->x, pos_scale), qy = quant(s->y, pos_scale);
    unsigned zx = (unsigned)((qx << 1) ^ (qx >> 31));   /* zigzag */
    unsigned zy = (unsigned)((qy << 1) ^ (qy >> 31));
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
    s->x = (float)(int)((zx >> 1) ^ (unsigned)-(int)(zx & 1)) / pos_scale;
    s->y = (float)(int)((zy >> 1) ^ (unsigned)-(int)(zy & 1)) / pos_scale;
    s->vx = 0; s->vy = 0; s->flags = 0;
    return o;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
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
