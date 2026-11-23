
/* snapshot: quantized state snapshot encode/decode with a v1-compatible
 * tagged extension.
 *
 * The first three varints (tick, zigzag x, zigzag y) are byte-for-byte the
 * 0bdd277 wire format.  Anything new is appended as {tag, len, payload}
 * records, so a legacy decoder that stops after three varints observes the
 * identical prefix.  See snapshot.h for the compatibility contract.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "snapshot.h"

static int varint_put(unsigned char* b, int cap, unsigned v) {
    int n = 0;
    while (v >= 0x80) {
        if (n >= cap) return -1;
        b[n++] = (unsigned char)((v & 0x7f) | 0x80);
        v >>= 7;
    }
    if (n >= cap) return -1;
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

const char* snap_err_name(int code) {
    switch (code) {
#define X(name, value, desc) case name: return #name;
    SNAP_ERR_LIST(X)
#undef X
    default: return "SNAP_ERR_UNKNOWN";
    }
}

/* Emit one tagged record {tag, len, payload} at offset `at`.
 * Returns bytes written, or SNAP_ERR_BUFFER_FULL. */
static int ext_put(Packet* p, int at, unsigned tag, unsigned value) {
    unsigned char payload[5];
    int plen = varint_put(payload, (int)sizeof(payload), value);
    if (plen < 0) return SNAP_ERR_BUFFER_FULL;
    int n = varint_put(p->buf + at, SNAP_MAX - at, tag);
    if (n < 0) return SNAP_ERR_BUFFER_FULL;
    int cap = SNAP_MAX - at - n;
    int m = varint_put(p->buf + at + n, cap, (unsigned)plen);
    if (m < 0) return SNAP_ERR_BUFFER_FULL;
    if (plen > cap - m) return SNAP_ERR_BUFFER_FULL;
    memcpy(p->buf + at + n + m, payload, (size_t)plen);
    return n + m + plen;
}

int snap_encode(const State* s, Packet* p, float pos_scale) {
    if (p == NULL || s == NULL) return SNAP_ERR_ARG;
    if (pos_scale <= 0.0f) return SNAP_ERR_BAD_SCALE;

    int n = varint_put(p->buf, SNAP_MAX, s->tick);
    if (n < 0) return SNAP_ERR_BUFFER_FULL;
    int qx = quant(s->x, pos_scale), qy = quant(s->y, pos_scale);
    /* zigzag with unsigned arithmetic: shifting a negative int is UB */
    unsigned zx = ((unsigned)qx << 1) ^ (unsigned)(qx >> 31);
    unsigned zy = ((unsigned)qy << 1) ^ (unsigned)(qy >> 31);
    int m = varint_put(p->buf + n, SNAP_MAX - n, zx);
    if (m < 0) return SNAP_ERR_BUFFER_FULL;
    n += m;
    m = varint_put(p->buf + n, SNAP_MAX - n, zy);
    if (m < 0) return SNAP_ERR_BUFFER_FULL;
    n += m;

    /* flags == 0 is the common case and must stay byte-identical to v1. */
    if (s->flags != 0u) {
        m = ext_put(p, n, SNAP_TAG_VERSION, SNAP_VERSION_MAX);
        if (m < 0) { p->len = 0; return m; }
        n += m;
        m = ext_put(p, n, SNAP_TAG_FLAGS, s->flags);
        if (m < 0) { p->len = 0; return m; }
        n += m;
    }
    p->len = n;
    return n;
}

/* Parse the tagged extension following a clean v1 prefix.
 *
 * `s->flags` is staged locally and committed only on success, so a malformed
 * extension can never leave a half-applied value on a state that a renderer
 * is already using.  Returns the number of bytes consumed, or an error. */
static int ext_parse(const unsigned char* b, int len, int at, State* s, SnapInfo* info) {
    int o = at;
    int saw_version = 0, rec = 0;
    unsigned staged_flags = s->flags;

    while (o < len) {
        unsigned tag = 0, plen = 0, value = 0;
        int n = varint_get(b + o, len - o, &tag);
        if (n < 0) return SNAP_ERR_EXT_TRUNCATED;
        o += n;
        n = varint_get(b + o, len - o, &plen);
        if (n < 0) return SNAP_ERR_EXT_TRUNCATED;
        o += n;
        if (plen > (unsigned)(len - o)) return SNAP_ERR_EXT_TRUNCATED;

        if (tag == SNAP_TAG_VERSION) {
            /* the version record must be the first record when present */
            if (saw_version || rec != 0) return SNAP_ERR_EXT_LEN;
            int vn = varint_get(b + o, (int)plen, &value);
            if (vn < 0 || (unsigned)vn != plen) return SNAP_ERR_EXT_LEN;
            if (value < SNAP_VERSION_MIN || value > SNAP_VERSION_MAX)
                return SNAP_ERR_VERSION_RANGE;
            saw_version = 1;
            info->version = value;
            info->fields_read++;
        } else if (tag == SNAP_TAG_FLAGS) {
            int vn = varint_get(b + o, (int)plen, &value);
            if (vn < 0 || (unsigned)vn != plen) return SNAP_ERR_EXT_LEN;
            staged_flags = value;
            info->flags_present = 1;
            info->fields_read++;
        } else {
            info->fields_skipped++;
            info->bytes_skipped += (int)plen + 2;   /* tag + len + payload */
        }
        o += (int)plen;
        rec++;
    }

    if (saw_version) s->flags = staged_flags;
    else if (info->fields_read > 3 || info->fields_skipped > 0) {
        /* v1 prefix followed by extension records but no version: the writer
         * must declare what it appended.  Refuse rather than assume. */
        return SNAP_ERR_EXT_LEN;
    }
    return o;
}

int snap_decode_info(const Packet* p, State* s, float pos_scale, SnapInfo* info) {
    if (info != NULL) memset(info, 0, sizeof(*info));
    if (p == NULL || s == NULL) {
        if (info != NULL) info->err = SNAP_ERR_ARG;
        return SNAP_ERR_ARG;
    }
    if (p->len < 0) {
        if (info != NULL) info->err = SNAP_ERR_TRUNCATED;
        return SNAP_ERR_TRUNCATED;
    }
    if (pos_scale <= 0.0f) {
        if (info != NULL) info->err = SNAP_ERR_BAD_SCALE;
        return SNAP_ERR_BAD_SCALE;
    }

    unsigned tick = 0, zx = 0, zy = 0;
    int n = varint_get(p->buf, p->len, &tick);
    if (n < 0) { if (info) info->err = SNAP_ERR_TRUNCATED; return SNAP_ERR_TRUNCATED; }
    int o = n;
    n = varint_get(p->buf + o, p->len - o, &zx);
    if (n < 0) { if (info) info->err = SNAP_ERR_TRUNCATED; return SNAP_ERR_TRUNCATED; }
    o += n;
    n = varint_get(p->buf + o, p->len - o, &zy);
    if (n < 0) { if (info) info->err = SNAP_ERR_TRUNCATED; return SNAP_ERR_TRUNCATED; }
    o += n;

    /* The prefix is authoritative: publish it before touching the extension
     * so a rejected extension can never blank a position. */
    s->tick = tick;
    s->x = (float)(int)((zx >> 1) ^ (unsigned)-(int)(zx & 1)) / pos_scale;
    s->y = (float)(int)((zy >> 1) ^ (unsigned)-(int)(zy & 1)) / pos_scale;
    s->vx = 0; s->vy = 0; s->flags = 0;

    if (info != NULL) {
        info->err = SNAP_OK;
        info->version = SNAP_VERSION_MIN;
        info->prefix_ok = 1;
        info->fields_read = 3;
    }
    if (o == p->len) return o;               /* bare v1 packet */

    SnapInfo scratch;
    SnapInfo* inf = info;
    if (inf == NULL) {
        memset(&scratch, 0, sizeof(scratch));
        scratch.version = SNAP_VERSION_MIN;
        scratch.prefix_ok = 1;
        scratch.fields_read = 3;
        inf = &scratch;
    }

    int r = ext_parse(p->buf, p->len, o, s, inf);
    if (r < 0) {
        if (info != NULL) info->err = (snap_err_t)r;
        return r;
    }
    if (info != NULL) info->err = SNAP_OK;
    return r;
}

int snap_decode(const Packet* p, State* s, float pos_scale) {
    return snap_decode_info(p, s, pos_scale, NULL);
}

#ifdef MINILAB_SELFTEST
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int minilab_selftest(void) {
    /* Exact v1 wire vector, captured from the 0bdd277 encoder:
       tick=1000 -> E8 07, x=12.5 @64 -> qx=800 -> zigzag 1600 -> C0 0C,
       y=-3.25 @64 -> qy=-208 -> zigzag 415 -> 9F 03. */
    const unsigned char V1_WIRE[] = { 0xE8, 0x07, 0xC0, 0x0C, 0x9F, 0x03 };

    Packet p;
    State in = { 1000u, 12.5f, -3.25f, 0, 0, 0u }, out;
    memset(&p, 0, sizeof(p));
    check("encode_ok", snap_encode(&in, &p, 64.0f) == 6);
    check("v1_wire_identical", p.len == 6 && memcmp(p.buf, V1_WIRE, 6) == 0);
    check("decode_ok", snap_decode(&p, &out, 64.0f) == 6);
    check("tick_roundtrip", out.tick == in.tick);
    check("pos_quantized", fabsf(out.x - in.x) <= 1.0f / 64.0f + 1e-6f &&
                           fabsf(out.y - in.y) <= 1.0f / 64.0f + 1e-6f);

    /* a v2 packet decodes flags and keeps the prefix values untouched */
    State v2 = { 1000u, 12.5f, -3.25f, 0, 0, 7u }, o2;
    SnapInfo si;
    check("v2_encode_ok", snap_encode(&v2, &p, 64.0f) > 6);
    check("v2_decode_ok", snap_decode_info(&p, &o2, 64.0f, &si) > 0);
    check("v2_flags", o2.flags == 7u && si.version == 2u && si.flags_present);
    check("v2_prefix_same", o2.tick == out.tick && o2.x == out.x && o2.y == out.y);
    check("v2_starts_with_v1", memcmp(p.buf, V1_WIRE, 6) == 0);

    /* a truncated extension keeps the prefix usable */
    Packet cut = p; cut.len = 6 + 1;
    State o3;
    SnapInfo s3;
    check("ext_truncated", snap_decode_info(&cut, &o3, 64.0f, &s3) == SNAP_ERR_EXT_TRUNCATED);
    check("ext_prefix_kept", s3.prefix_ok == 1 && o3.tick == in.tick && o3.x == out.x);

    /* an unknown version is refused, but the prefix still renders */
    unsigned char bad[] = { 0xE8, 0x07, 0xC0, 0x0C, 0x9F, 0x03, 0x00, 0x01, 0x7F };
    Packet bp = { .len = (int)sizeof(bad) };
    memcpy(bp.buf, bad, sizeof(bad));
    State o4; SnapInfo s4;
    check("version_range", snap_decode_info(&bp, &o4, 64.0f, &s4) == SNAP_ERR_VERSION_RANGE);
    check("version_prefix_kept", s4.prefix_ok == 1 && o4.x == out.x && o4.flags == 0u);

    check("bad_scale", snap_encode(&in, &p, 0.0f) == SNAP_ERR_BAD_SCALE);
    check("bad_arg", snap_encode(NULL, &p, 64.0f) == SNAP_ERR_ARG);
    check("truncated", snap_decode(&(Packet){ .len = 2 }, &out, 64.0f) == SNAP_ERR_TRUNCATED);

    State big = { 70000u, 1000.0f, -1000.0f, 0, 0, 0 };
    Packet p2; memset(&p2, 0, sizeof(p2));
    snap_encode(&big, &p2, 4.0f);
    State out2;
    check("big_roundtrip", snap_decode(&p2, &out2, 4.0f) > 0 && out2.tick == big.tick &&
                            fabsf(out2.x - big.x) <= 0.25f + 1e-4f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
#endif

#ifndef MINILAB_NO_MAIN
int main(void) {
#ifdef MINILAB_SELFTEST
    return minilab_selftest();
#else
    return 0;
#endif
}
#endif
