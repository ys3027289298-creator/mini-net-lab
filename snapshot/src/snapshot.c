
/* snapshot: quantized state snapshot encode/decode with varint tick.
 *
 * Failure-isolation version: every error path returns a named snap_err_t
 * instead of a bare -1, and deterministic fault hooks (snap_inject) let the
 * acceptance suite exercise failure without touching the success path.
 * The encode/decode math and the wire layout are unchanged.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "snapshot.h"

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
    return SNAP_ERR_TRUNCATED;
}

static int quant(float v, float scale) { return (int)lrintf(v * scale); }

/* --- deterministic, process-local fault injection ----------------------- */
static snap_fault_t g_snap_fault = SNAP_FAULT_NONE;

void snap_inject(snap_fault_t f) { g_snap_fault = f; }
void snap_reset_faults(void) { g_snap_fault = SNAP_FAULT_NONE; }
snap_fault_t snap_current_fault(void) { return g_snap_fault; }

const char* snap_err_name(int code) {
    switch (code) {
    case SNAP_OK:                return "SNAP_OK";
    case SNAP_ERR_PACKET_NULL:   return "SNAP_ERR_PACKET_NULL";
    case SNAP_ERR_BAD_SCALE:     return "SNAP_ERR_BAD_SCALE";
    case SNAP_ERR_STATE_NULL:    return "SNAP_ERR_STATE_NULL";
    case SNAP_ERR_TRUNCATED:     return "SNAP_ERR_TRUNCATED";
    case SNAP_ERR_INJECT_ENC:    return "SNAP_ERR_INJECT_ENC";
    case SNAP_ERR_INJECT_DEC:    return "SNAP_ERR_INJECT_DEC";
    default:                     return "SNAP_ERR_UNKNOWN";
    }
}

const char* snap_fault_name(snap_fault_t f) {
    switch (f) {
    case SNAP_FAULT_NONE:   return "SNAP_FAULT_NONE";
    case SNAP_FAULT_ENCODE: return "SNAP_FAULT_ENCODE";
    case SNAP_FAULT_DECODE: return "SNAP_FAULT_DECODE";
    default:                return "SNAP_FAULT_UNKNOWN";
    }
}

int snap_encode(const State* s, Packet* p, float pos_scale) {
    if (p == NULL) return SNAP_ERR_PACKET_NULL;
    if (s == NULL) return SNAP_ERR_STATE_NULL;
    if (pos_scale <= 0.0f) return SNAP_ERR_BAD_SCALE;
    if (g_snap_fault == SNAP_FAULT_ENCODE) return SNAP_ERR_INJECT_ENC;
    int n = varint_put(p->buf, s->tick);
    int qx = quant(s->x, pos_scale), qy = quant(s->y, pos_scale);
    unsigned zx = (unsigned)((qx << 1) ^ (qx >> 31));   /* zigzag */
    unsigned zy = (unsigned)((qy << 1) ^ (qy >> 31));
    n += varint_put(p->buf + n, zx);
    n += varint_put(p->buf + n, zy);
    p->len = n;
    return n;
}

int snap_decode(const Packet* p, State* s, float pos_scale) {
    if (p == NULL || s == NULL) return SNAP_ERR_STATE_NULL;
    if (pos_scale <= 0.0f) return SNAP_ERR_BAD_SCALE;
    if (g_snap_fault == SNAP_FAULT_DECODE) return SNAP_ERR_INJECT_DEC;
    unsigned tick = 0, zx = 0, zy = 0;
    int n = varint_get(p->buf, p->len, &tick);
    if (n < 0) return n;                       /* already a snap_err_t */
    int o = n;
    n = varint_get(p->buf + o, p->len - o, &zx); if (n < 0) return n; o += n;
    n = varint_get(p->buf + o, p->len - o, &zy); if (n < 0) return n; o += n;
    s->tick = tick;
    s->x = (float)(int)((zx >> 1) ^ (unsigned)-(int)(zx & 1)) / pos_scale;
    s->y = (float)(int)((zy >> 1) ^ (unsigned)-(int)(zy & 1)) / pos_scale;
    s->vx = 0; s->vy = 0; s->flags = 0;
    return o;
}

#ifdef MINILAB_SELFTEST
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int MINILAB_SELFTEST(void) {
    Packet p;
    State in = { 1000u, 12.5f, -3.25f, 0, 0, 1u }, out;
    memset(&p, 0, sizeof(p));
    check("encode_ok", snap_encode(&in, &p, 64.0f) > 0);
    check("size_small", p.len <= 12);
    check("decode_ok", snap_decode(&p, &out, 64.0f) > 0);
    check("tick_roundtrip", out.tick == in.tick);
    check("pos_quantized", fabsf(out.x - in.x) <= 1.0f / 64.0f + 1e-6f &&
                           fabsf(out.y - in.y) <= 1.0f / 64.0f + 1e-6f);

    check("bad_scale", snap_encode(&in, &p, 0.0f) == SNAP_ERR_BAD_SCALE);
    check("bad_packet", snap_decode(&(Packet){ .len = 2 }, &out, 64.0f) == SNAP_ERR_TRUNCATED);

    State big = { 70000u, 1000.0f, -1000.0f, 0, 0, 0 };
    Packet p2; memset(&p2, 0, sizeof(p2));
    snap_encode(&big, &p2, 4.0f);
    State out2;
    check("big_roundtrip", snap_decode(&p2, &out2, 4.0f) > 0 && out2.tick == big.tick &&
                            fabsf(out2.x - big.x) <= 0.25f + 1e-4f);

    /* failure injection: fault is reported as a named code, caller survives */
    snap_inject(SNAP_FAULT_ENCODE);
    check("inject_encode", snap_encode(&in, &p, 64.0f) == SNAP_ERR_INJECT_ENC);
    snap_reset_faults();
    check("fault_cleared", snap_encode(&in, &p, 64.0f) > 0);
    snap_inject(SNAP_FAULT_DECODE);
    check("inject_decode", snap_decode(&p, &out, 64.0f) == SNAP_ERR_INJECT_DEC);
    snap_reset_faults();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
#endif

#ifndef MINILAB_NO_MAIN
int main(void) {
#ifdef MINILAB_SELFTEST
    return MINILAB_SELFTEST();
#else
    return 0;
#endif
}
#endif
