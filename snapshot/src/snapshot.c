
/* snapshot: quantized state snapshot encode/decode with varint tick.
 *
 * Failures go through the shared mnl sink (module=SNAPSHOT) so a bad entry
 * is recorded with its exact field and reason and never disturbs the rest of
 * a batch. Legacy self-test output is byte-identical to the original module.
 */
#include "mnl.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* Internal names are aliases of the public payload types: no conversion and
 * no copy is ever needed between a batch payload and the primitives. */
typedef mnl_snap_state_t State;
typedef mnl_packet_t     Packet;

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

static int encode(const State* s, Packet* p, float pos_scale, mnl_sink_t* sk) {
    if (p == NULL) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                     MNL_F_SNAP_PACKET, MNL_R_NULL);
    if (pos_scale <= 0.0f) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                             MNL_F_POS_SCALE, MNL_R_RANGE);
    if (!isfinite(pos_scale)) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                                MNL_F_POS_SCALE, MNL_R_NONFINITE);
    if (s == NULL) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                     MNL_F_SNAP_STATE, MNL_R_NULL);
    if (!isfinite(s->x)) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                           MNL_F_X, MNL_R_NONFINITE);
    if (!isfinite(s->y)) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                           MNL_F_Y, MNL_R_NONFINITE);
    int n = varint_put(p->bytes, s->tick);
    int qx = quant(s->x, pos_scale), qy = quant(s->y, pos_scale);
    unsigned zx = (unsigned)((qx << 1) ^ (qx >> 31));   /* zigzag */
    unsigned zy = (unsigned)((qy << 1) ^ (qy >> 31));
    n += varint_put(p->bytes + n, zx);
    n += varint_put(p->bytes + n, zy);
    p->len = n;
    return n;
}

static int decode(const Packet* p, State* s, float pos_scale, mnl_sink_t* sk) {
    if (p == NULL) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                     MNL_F_SNAP_PACKET, MNL_R_NULL);
    if (s == NULL) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                     MNL_F_SNAP_OUT, MNL_R_NULL);
    if (pos_scale <= 0.0f) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                             MNL_F_POS_SCALE, MNL_R_RANGE);
    if (!isfinite(pos_scale)) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                                MNL_F_POS_SCALE, MNL_R_NONFINITE);
    unsigned tick = 0, zx = 0, zy = 0;
    int n = varint_get(p->bytes, p->len, &tick);
    if (n < 0) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                 MNL_F_PACKET_BYTES, MNL_R_PARSE);
    int o = n;
    n = varint_get(p->bytes + o, p->len - o, &zx);
    if (n < 0) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                 MNL_F_PACKET_BYTES, MNL_R_PARSE);
    o += n;
    n = varint_get(p->bytes + o, p->len - o, &zy);
    if (n < 0) return mnl_report(sk, MNL_MOD_SNAPSHOT,
                                 MNL_F_PACKET_BYTES, MNL_R_PARSE);
    o += n;
    s->tick = tick;
    s->x = (float)(int)((zx >> 1) ^ (unsigned)-(int)(zx & 1)) / pos_scale;
    s->y = (float)(int)((zy >> 1) ^ (unsigned)-(int)(zy & 1)) / pos_scale;
    s->vx = 0; s->vy = 0; s->flags = 0;
    return o;
}

/* ---- zero-copy batch runner ------------------------------------------- */

int mnl_snapshot_run(mnl_state_t* st, mnl_sink_t* sk,
                     const mnl_item_t* items, int n, mnl_result_t* res) {
    (void)st; /* encode/decode are stateless across entries */
    if (sk == NULL) return -1;
    if (items == NULL || n < 0 || res == NULL)
        return mnl_report(sk, MNL_MOD_SNAPSHOT, MNL_F_NONE, MNL_R_NULL);

    int failures = 0;
    for (int i = 0; i < n; i++) {
        mnl_sink_begin(sk, i);
        mnl_result_t* r = &res[i];
        memset(r, 0, sizeof(*r));
        const mnl_item_t* it = &items[i];

        int rc;
        switch (it->op) {
        case MNL_SNAP_ENCODE: {
            if (it->payload == NULL) {
                rc = mnl_report(sk, MNL_MOD_SNAPSHOT,
                                MNL_F_SNAP_STATE, MNL_R_NULL);
            } else if (it->out == NULL) {
                rc = mnl_report(sk, MNL_MOD_SNAPSHOT,
                                MNL_F_SNAP_PACKET, MNL_R_NULL);
            } else {
                /* payload/out are used directly: pointers pass through,
                 * the payload bytes are never copied. */
                rc = encode((const State*)it->payload,
                            (Packet*)it->out, it->scale, sk);
            }
            break;
        }
        case MNL_SNAP_DECODE: {
            if (it->payload == NULL) {
                rc = mnl_report(sk, MNL_MOD_SNAPSHOT,
                                MNL_F_SNAP_PACKET, MNL_R_NULL);
            } else if (it->out == NULL) {
                rc = mnl_report(sk, MNL_MOD_SNAPSHOT,
                                MNL_F_SNAP_OUT, MNL_R_NULL);
            } else {
                rc = decode((const Packet*)it->payload,
                            (State*)it->out, it->scale, sk);
            }
            break;
        }
        default:
            rc = mnl_report(sk, MNL_MOD_SNAPSHOT, MNL_F_NONE, MNL_R_BAD_OP);
            break;
        }

        r->rc = rc;
        if (rc < 0) {
            failures++;
            r->err = sk->errs[sk->count - 1];
        }
    }
    return failures;
}

#ifndef MNL_NO_MAIN
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Packet p;
    State in = { 1000u, 12.5f, -3.25f, 0, 0, 1u }, out;
    memset(&p, 0, sizeof(p));
    check("encode_ok", encode(&in, &p, 64.0f, NULL) > 0);
    check("size_small", p.len <= 12);
    check("decode_ok", decode(&p, &out, 64.0f, NULL) > 0);
    check("tick_roundtrip", out.tick == in.tick);
    check("pos_quantized", fabsf(out.x - in.x) <= 1.0f / 64.0f + 1e-6f &&
                           fabsf(out.y - in.y) <= 1.0f / 64.0f + 1e-6f);

    check("bad_scale", encode(&in, &p, 0.0f, NULL) == -1);
    check("bad_packet", decode(&(Packet){ .len = 2 }, &out, 64.0f, NULL) == -1);

    State big = { 70000u, 1000.0f, -1000.0f, 0, 0, 0 };
    Packet p2; memset(&p2, 0, sizeof(p2));
    encode(&big, &p2, 4.0f, NULL);
    State out2;
    check("big_roundtrip", decode(&p2, &out2, 4.0f, NULL) > 0 && out2.tick == big.tick &&
                            fabsf(out2.x - big.x) <= 0.25f + 1e-4f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
#endif
