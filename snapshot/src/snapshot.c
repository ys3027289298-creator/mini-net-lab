
/* snapshot: versioned state snapshot encode/decode.
 *
 * Wire format: varint version header, then the fields of that version's
 * schema, each a varint (zigzag for quantized floats). Decode rules:
 *  - version missing or outside [SNAP_VERSION_MIN, SNAP_VERSION_MAX]:
 *    reject outright, never guess one
 *  - known field cut off mid-varint: fail, reporting field and version
 *  - fields absent at a clean boundary: fill schema defaults, count them
 *  - trailing fields past the schema: skip, count fields and bytes
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define SNAP_MAX 64
#define SNAP_VERSION_MIN 1
#define SNAP_VERSION_MAX 2

typedef struct {
    unsigned tick;
    float x, y, vx, vy;
    unsigned flags;
} State;

typedef struct {
    unsigned char buf[SNAP_MAX];
    int len;
} Packet;

typedef enum {
    SNAP_OK = 0,
    SNAP_ERR_ARG,             /* null pointer / corrupt packet header */
    SNAP_ERR_VERSION_MISSING, /* no version byte at all */
    SNAP_ERR_VERSION_RANGE,   /* version unknown: refuse, do not guess */
    SNAP_ERR_TRUNCATED,       /* varint cut off inside a field */
} SnapErr;

typedef struct {
    SnapErr err;
    unsigned version;    /* version being parsed when err happened */
    const char* field;   /* field that failed, NULL when not applicable */
    /* accounting for a successful decode */
    int fields_read;
    int fields_defaulted;
    int fields_skipped;
    int bytes_skipped;
} SnapInfo;

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

static unsigned zig(int v) { return (unsigned)((v << 1) ^ (v >> 31)); }
static int unzig(unsigned v) { return (int)((v >> 1) ^ (unsigned)-(int)(v & 1)); }
static int quant(float v, float scale) { return (int)lrintf(v * scale); }

/* ---- per-version schemas ---- */

enum { F_TICK, F_X, F_Y, F_FLAGS };

typedef struct {
    const char* name;
    int id;
    float scale;      /* quantization scale for float fields, 0 otherwise */
    unsigned def;     /* raw default when the field is absent */
} FieldSpec;

static const FieldSpec V1_FIELDS[] = {
    { "tick", F_TICK, 0, 0 },
    { "x", F_X, 64.0f, 0 },
    { "y", F_Y, 64.0f, 0 },
};

/* v2 adds the status field that crashed unversioned old clients. */
static const FieldSpec V2_FIELDS[] = {
    { "tick", F_TICK, 0, 0 },
    { "x", F_X, 64.0f, 0 },
    { "y", F_Y, 64.0f, 0 },
    { "flags", F_FLAGS, 0, 0 },
};

typedef struct {
    const FieldSpec* fields;
    int n;
} Schema;

static const Schema SCHEMAS[] = {
    { NULL, 0 },            /* version 0: invalid */
    { V1_FIELDS, 3 },
    { V2_FIELDS, 4 },
};

static unsigned field_raw(const State* s, const FieldSpec* f) {
    switch (f->id) {
    case F_TICK: return s->tick;
    case F_X: return zig(quant(s->x, f->scale));
    case F_Y: return zig(quant(s->y, f->scale));
    case F_FLAGS: return s->flags;
    }
    return 0;
}

static void field_apply(State* s, const FieldSpec* f, unsigned raw) {
    switch (f->id) {
    case F_TICK: s->tick = raw; break;
    case F_X: s->x = (float)unzig(raw) / f->scale; break;
    case F_Y: s->y = (float)unzig(raw) / f->scale; break;
    case F_FLAGS: s->flags = raw; break;
    }
}

static int encode(const State* s, Packet* p, unsigned version) {
    if (s == NULL || p == NULL) return -1;
    if (version < SNAP_VERSION_MIN || version > SNAP_VERSION_MAX) return -1;
    const Schema* sch = &SCHEMAS[version];
    int n = varint_put(p->buf, version);
    for (int i = 0; i < sch->n; i++) {
        if (n + 5 > SNAP_MAX) return -1;    /* worst-case varint */
        n += varint_put(p->buf + n, field_raw(s, &sch->fields[i]));
    }
    p->len = n;
    return n;
}

static SnapInfo decode(const Packet* p, State* s) {
    SnapInfo info;
    memset(&info, 0, sizeof(info));
    if (p == NULL || s == NULL || p->len < 0 || p->len > SNAP_MAX) {
        info.err = SNAP_ERR_ARG;
        return info;
    }
    unsigned ver = 0;
    int n = varint_get(p->buf, p->len, &ver);
    if (n < 0) {                            /* empty or dangling version */
        info.err = SNAP_ERR_VERSION_MISSING;
        return info;
    }
    info.version = ver;
    if (ver < SNAP_VERSION_MIN || ver > SNAP_VERSION_MAX) {
        info.err = SNAP_ERR_VERSION_RANGE;  /* reject, never guess */
        return info;
    }
    const Schema* sch = &SCHEMAS[ver];
    *s = (State){ 0, 0, 0, 0, 0, 0 };
    int o = n;
    for (int i = 0; i < sch->n; i++) {
        const FieldSpec* f = &sch->fields[i];
        if (o >= p->len) {                  /* clean end: default the rest */
            field_apply(s, f, f->def);
            info.fields_defaulted++;
            continue;
        }
        unsigned raw = 0;
        n = varint_get(p->buf + o, p->len - o, &raw);
        if (n < 0) {
            info.err = SNAP_ERR_TRUNCATED;
            info.field = f->name;
            return info;
        }
        o += n;
        info.fields_read++;
        field_apply(s, f, raw);
    }
    while (o < p->len) {                    /* unknown trailing fields */
        unsigned raw = 0;
        n = varint_get(p->buf + o, p->len - o, &raw);
        if (n < 0) {
            info.err = SNAP_ERR_TRUNCATED;
            info.field = "<unknown>";
            return info;
        }
        o += n;
        info.fields_skipped++;
        info.bytes_skipped += n;
    }
    return info;
}

static int snap_describe(const SnapInfo* i, char* buf, int cap) {
    switch (i->err) {
    case SNAP_OK:
        return snprintf(buf, cap, "ok (version %u)", i->version);
    case SNAP_ERR_ARG:
        return snprintf(buf, cap, "bad argument");
    case SNAP_ERR_VERSION_MISSING:
        return snprintf(buf, cap, "version missing");
    case SNAP_ERR_VERSION_RANGE:
        return snprintf(buf, cap, "version %u out of range [%d,%d]",
                        i->version, SNAP_VERSION_MIN, SNAP_VERSION_MAX);
    case SNAP_ERR_TRUNCATED:
        return snprintf(buf, cap, "field '%s' truncated (version %u)",
                        i->field ? i->field : "?", i->version);
    }
    return snprintf(buf, cap, "unknown error");
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Packet p;
    State in = { 1000u, 12.5f, -3.25f, 0, 0, 1u }, out;
    char msg[96];

    check("schema_table", (int)(sizeof(SCHEMAS) / sizeof(SCHEMAS[0])) == SNAP_VERSION_MAX + 1);

    /* same-version interop: v1 roundtrip, field by field */
    memset(&p, 0, sizeof(p));
    check("enc_v1", encode(&in, &p, 1) > 0);
    SnapInfo i1 = decode(&p, &out);
    check("dec_v1", i1.err == SNAP_OK && i1.version == 1);
    check("v1_accounting", i1.fields_read == 3 && i1.fields_defaulted == 0 &&
                           i1.fields_skipped == 0);
    check("v1_tick", out.tick == in.tick);
    check("v1_pos", fabsf(out.x - in.x) <= 1.0f / 64.0f + 1e-6f &&
                    fabsf(out.y - in.y) <= 1.0f / 64.0f + 1e-6f);
    check("v1_flags_zero", out.flags == 0);   /* not on the wire in v1 */

    /* same-version interop: v2 roundtrip including the status field */
    State in2 = { 1000u, 12.5f, -3.25f, 0, 0, 0x2Au };
    check("enc_v2", encode(&in2, &p, 2) > 0 && p.len <= 12);
    SnapInfo i2 = decode(&p, &out);
    check("dec_v2", i2.err == SNAP_OK && i2.version == 2 && i2.fields_read == 4);
    check("v2_same", out.tick == in2.tick && out.flags == in2.flags &&
                     fabsf(out.x - in2.x) <= 1.0f / 64.0f + 1e-6f &&
                     fabsf(out.y - in2.y) <= 1.0f / 64.0f + 1e-6f);

    /* v2 packet with the status field absent: default, and count it */
    Packet pm = p;
    pm.len -= 1;                              /* drop the 1-byte flags field */
    SnapInfo im = decode(&pm, &out);
    check("v2_missing_default", im.err == SNAP_OK && im.fields_defaulted == 1 &&
                                out.flags == 0 && out.tick == in2.tick);

    /* unknown trailing fields: skipped and accounted, known fields intact */
    Packet pu = p;
    pu.buf[pu.len++] = 0x2A;                  /* one unknown field, 1 byte */
    pu.buf[pu.len++] = 0x81;                  /* another, 2 bytes */
    pu.buf[pu.len++] = 0x01;
    SnapInfo iu = decode(&pu, &out);
    check("v2_skip_unknown", iu.err == SNAP_OK && iu.fields_skipped == 2 &&
                             iu.bytes_skipped == 3 && out.flags == in2.flags);

    /* version missing or out of range: reject, never guess */
    SnapInfo ie = decode(&(Packet){ .len = 0 }, &out);
    check("ver_missing", ie.err == SNAP_ERR_VERSION_MISSING);
    Packet pv; memset(&pv, 0, sizeof(pv));
    pv.len = 1; pv.buf[0] = 3;
    SnapInfo iv = decode(&pv, &out);
    check("ver_range", iv.err == SNAP_ERR_VERSION_RANGE && iv.version == 3);
    pv.buf[0] = 0;
    check("ver_zero", decode(&pv, &out).err == SNAP_ERR_VERSION_RANGE);
    check("enc_bad_ver", encode(&in, &p, 0) == -1 && encode(&in, &p, 3) == -1);
    snap_describe(&iv, msg, sizeof(msg));
    check("ver_msg", strstr(msg, "version 3") != NULL);

    /* cross-version failure names the field and the version */
    State it = { 7u, 1.0f, 1.0f, 0, 0, 0x2000u };   /* flags: 2-byte varint */
    encode(&it, &p, 2);
    p.len -= 1;                               /* cut flags mid-varint */
    SnapInfo itc = decode(&p, &out);
    check("trunc_v2", itc.err == SNAP_ERR_TRUNCATED && itc.version == 2 &&
                      itc.field != NULL && strcmp(itc.field, "flags") == 0);
    snap_describe(&itc, msg, sizeof(msg));
    check("trunc_v2_msg", strstr(msg, "flags") != NULL && strstr(msg, "2") != NULL);

    encode(&in, &p, 1);                       /* tick: 2 bytes, x: 2 bytes */
    p.len = 4;                                /* version + tick + first byte of x */
    SnapInfo ix = decode(&p, &out);
    check("trunc_v1", ix.err == SNAP_ERR_TRUNCATED && ix.version == 1 &&
                      ix.field != NULL && strcmp(ix.field, "x") == 0);

    /* bad arguments */
    check("bad_args", encode(NULL, &p, 1) == -1 && encode(&in, NULL, 1) == -1 &&
                      decode(NULL, &out).err == SNAP_ERR_ARG &&
                      decode(&p, NULL).err == SNAP_ERR_ARG);

    /* large values still roundtrip on v2 */
    State big = { 70000u, 1000.0f, -1000.0f, 0, 0, 0xFFFFFFFFu };
    check("enc_big", encode(&big, &p, 2) > 0);
    SnapInfo ib = decode(&p, &out);
    check("big_roundtrip", ib.err == SNAP_OK && out.tick == big.tick &&
                           out.flags == big.flags &&
                           fabsf(out.x - big.x) <= 1.0f / 64.0f + 1e-4f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
