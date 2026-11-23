
/* aoi: grid-based area of interest with enter/leave diff and radius query. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "aoi.h"

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

static unsigned zig(int v) { return ((unsigned)v << 1) ^ (unsigned)(v >> 31); }
static int unzig(unsigned v) { return (int)((v >> 1) ^ (unsigned)-(int)(v & 1)); }
static unsigned fbits(float f) { unsigned u; memcpy(&u, &f, sizeof(u)); return u; }
static float frombits(unsigned u) { float f; memcpy(&f, &u, sizeof(f)); return f; }

static int ext_put(unsigned char* b, int cap, unsigned tag, unsigned value) {
    unsigned char payload[5];
    int plen = varint_put(payload, (int)sizeof(payload), value);
    if (plen < 0) return -1;
    int n = varint_put(b, cap, tag);
    if (n < 0) return -1;
    int m = varint_put(b + n, cap - n, (unsigned)plen);
    if (m < 0) return -1;
    if (plen > cap - n - m) return -1;
    memcpy(b + n + m, payload, (size_t)plen);
    return n + m + plen;
}

const char* aoi_err_name(int code) {
    switch (code) {
#define X(name, value, desc) case name: return #name;
    AOI_ERR_LIST(X)
#undef X
    default: return "AOI_ERR_UNKNOWN";
    }
}

void aoi_world_init(World* w, float cell, float radius) {
    if (w == NULL) return;
    memset(w, 0, sizeof(*w));
    w->cell = cell;
    w->radius = radius;
}

int aoi_world_add(World* w, int id, float x, float y) {
    if (w == NULL) return AOI_ERR_ARG;
    if (w->n >= MAX_ENT) return AOI_ERR_FULL;
    if (w->cell <= 0.0f) return AOI_ERR_BAD_CELL;
    for (int i = 0; i < w->n; i++) if (w->e[i].alive && w->e[i].id == id) return AOI_ERR_DUP_ID;
    Entity* e = &w->e[w->n++];
    e->id = id; e->x = x; e->y = y; e->alive = 1;
    w->rev++;
    return w->n - 1;
}

int aoi_world_move(World* w, int id, float x, float y) {
    if (w == NULL) return AOI_ERR_ARG;
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) {
            w->e[i].x = x; w->e[i].y = y; w->rev++;
            return 0;
        }
    }
    return AOI_ERR_NOT_FOUND;
}

int aoi_world_remove(World* w, int id) {
    if (w == NULL) return AOI_ERR_ARG;
    for (int i = 0; i < w->n; i++) {
        if (w->e[i].alive && w->e[i].id == id) { w->e[i].alive = 0; w->rev++; return 0; }
    }
    return AOI_ERR_NOT_FOUND;
}

int aoi_cell_coord(float v, float cell) {
    if (cell <= 0.0f) return 0;
    int c = (int)floorf(v / cell);
    if (c < 0) c = 0;
    if (c >= GRID) c = GRID - 1;
    return c;
}

/* collect ids within radius of (x,y), excluding self id */
int aoi_world_query(const World* w, int self, float x, float y, int* out, int cap) {
    if (w == NULL) return AOI_ERR_ARG;
    int n = 0;
    float r2 = w->radius * w->radius;
    for (int i = 0; i < w->n; i++) {
        const Entity* e = &w->e[i];
        if (!e->alive || e->id == self) continue;
        float dx = e->x - x, dy = e->y - y;
        if (dx * dx + dy * dy <= r2) {
            if (n < cap) out[n] = e->id;
            n++;
        }
    }
    return n;
}

/* diff current visible set against previous one: counts enter/leave */
void aoi_world_diff(const int* prev, int prevn, const int* cur, int curn, int* enter, int* leave) {
    int en = 0, lv = 0;
    for (int i = 0; i < curn; i++) {
        int found = 0;
        for (int j = 0; j < prevn; j++) if (prev[j] == cur[i]) { found = 1; break; }
        if (!found) en++;
    }
    for (int j = 0; j < prevn; j++) {
        int found = 0;
        for (int i = 0; i < curn; i++) if (cur[i] == prev[j]) { found = 1; break; }
        if (!found) lv++;
    }
    *enter = en; *leave = lv;
}

/* ---- canonical state body --------------------------------------------- */

int aoi_state_encode(const World* w, unsigned char* buf, int cap) {
    if (w == NULL || buf == NULL || cap < 0) return AOI_ERR_ARG;
    if (w->n < 0 || w->n > MAX_ENT) return AOI_ERR_ARG;

    int o = 0, n;
    n = varint_put(buf, cap, (unsigned)w->n);
    if (n < 0) return AOI_ERR_BUFFER;
    o += n;
    for (int i = 0; i < w->n; i++) {
        n = varint_put(buf + o, cap - o, zig(w->e[i].id));
        if (n < 0) return AOI_ERR_BUFFER;
        o += n;
        n = varint_put(buf + o, cap - o, fbits(w->e[i].x));
        if (n < 0) return AOI_ERR_BUFFER;
        o += n;
        n = varint_put(buf + o, cap - o, fbits(w->e[i].y));
        if (n < 0) return AOI_ERR_BUFFER;
        o += n;
        if (o >= cap) return AOI_ERR_BUFFER;
        buf[o++] = (unsigned char)(w->e[i].alive ? 1 : 0);
    }
    n = varint_put(buf + o, cap - o, fbits(w->cell));
    if (n < 0) return AOI_ERR_BUFFER;
    o += n;
    n = varint_put(buf + o, cap - o, fbits(w->radius));
    if (n < 0) return AOI_ERR_BUFFER;
    o += n;

    if (w->rev != 0u) {
        n = ext_put(buf + o, cap - o, AOI_TAG_VERSION, AOI_VERSION_MAX);
        if (n < 0) return AOI_ERR_BUFFER;
        o += n;
        n = ext_put(buf + o, cap - o, AOI_TAG_REV, w->rev);
        if (n < 0) return AOI_ERR_BUFFER;
        o += n;
    }
    return o;
}

static int ext_parse(const unsigned char* b, int len, int at, World* w, AoiInfo* info) {
    int o = at, saw_version = 0, rec = 0;
    unsigned staged_rev = w->rev;

    while (o < len) {
        unsigned tag = 0, plen = 0, value = 0;
        int n = varint_get(b + o, len - o, &tag);
        if (n < 0) return AOI_ERR_EXT;
        o += n;
        n = varint_get(b + o, len - o, &plen);
        if (n < 0) return AOI_ERR_EXT;
        o += n;
        if (plen > (unsigned)(len - o)) return AOI_ERR_EXT;

        if (tag == AOI_TAG_VERSION) {
            if (saw_version || rec != 0) return AOI_ERR_EXT;
            int vn = varint_get(b + o, (int)plen, &value);
            if (vn < 0 || (unsigned)vn != plen) return AOI_ERR_EXT;
            if (value < AOI_VERSION_MIN || value > AOI_VERSION_MAX)
                return AOI_ERR_VERSION;
            saw_version = 1;
            info->version = value;
        } else if (tag == AOI_TAG_REV) {
            int vn = varint_get(b + o, (int)plen, &value);
            if (vn < 0 || (unsigned)vn != plen) return AOI_ERR_EXT;
            staged_rev = value;
        } else {
            info->fields_skipped++;
        }
        o += (int)plen;
        rec++;
    }
    if (!saw_version) return AOI_ERR_EXT;
    w->rev = staged_rev;
    return o;
}

int aoi_state_decode_info(const unsigned char* b, int len, World* w, AoiInfo* info) {
    if (info != NULL) memset(info, 0, sizeof(*info));
    if (b == NULL || w == NULL) {
        if (info) info->err = AOI_ERR_ARG;
        return AOI_ERR_ARG;
    }
    if (len < 0) { if (info) info->err = AOI_ERR_TRUNCATED; return AOI_ERR_TRUNCATED; }

    int o = 0, n;
    unsigned cnt = 0;
    n = varint_get(b, len, &cnt);
    if (n < 0) { if (info) info->err = AOI_ERR_TRUNCATED; return AOI_ERR_TRUNCATED; }
    o += n;
    if (cnt > MAX_ENT) { if (info) info->err = AOI_ERR_TRUNCATED; return AOI_ERR_TRUNCATED; }

    memset(w, 0, sizeof(*w));
    w->n = (int)cnt;
    for (unsigned i = 0; i < cnt; i++) {
        unsigned zid = 0, fx = 0, fy = 0;
        n = varint_get(b + o, len - o, &zid);
        if (n < 0) { if (info) info->err = AOI_ERR_TRUNCATED; return AOI_ERR_TRUNCATED; }
        o += n;
        n = varint_get(b + o, len - o, &fx);
        if (n < 0) { if (info) info->err = AOI_ERR_TRUNCATED; return AOI_ERR_TRUNCATED; }
        o += n;
        n = varint_get(b + o, len - o, &fy);
        if (n < 0) { if (info) info->err = AOI_ERR_TRUNCATED; return AOI_ERR_TRUNCATED; }
        o += n;
        if (o >= len) { if (info) info->err = AOI_ERR_TRUNCATED; return AOI_ERR_TRUNCATED; }
        w->e[i].id = unzig(zid);
        w->e[i].x = frombits(fx);
        w->e[i].y = frombits(fy);
        w->e[i].alive = b[o++] ? 1 : 0;
    }
    unsigned fc = 0, fr = 0;
    n = varint_get(b + o, len - o, &fc);
    if (n < 0) { if (info) info->err = AOI_ERR_TRUNCATED; return AOI_ERR_TRUNCATED; }
    o += n;
    n = varint_get(b + o, len - o, &fr);
    if (n < 0) { if (info) info->err = AOI_ERR_TRUNCATED; return AOI_ERR_TRUNCATED; }
    o += n;
    w->cell = frombits(fc);
    w->radius = frombits(fr);

    if (info) {
        info->err = AOI_OK;
        info->version = AOI_VERSION_MIN;
        info->prefix_ok = 1;
        info->entities = (int)cnt;
    }
    if (o == len) return o;

    AoiInfo scratch;
    AoiInfo* inf = info;
    if (inf == NULL) {
        memset(&scratch, 0, sizeof(scratch));
        scratch.version = AOI_VERSION_MIN;
        scratch.prefix_ok = 1;
        scratch.entities = (int)cnt;
        inf = &scratch;
    }
    int r = ext_parse(b, len, o, w, inf);
    if (r < 0) { if (info) info->err = (aoi_err_t)r; return r; }
    if (info) info->err = AOI_OK;
    return r;
}

int aoi_state_decode(const unsigned char* b, int len, World* w) {
    return aoi_state_decode_info(b, len, w, NULL);
}

#ifdef MINILAB_SELFTEST
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int minilab_selftest(void) {
    World w;
    aoi_world_init(&w, 10.0f, 15.0f);
    check("add_self", aoi_world_add(&w, 1, 0, 0) == 0);
    check("add_near", aoi_world_add(&w, 2, 5, 0) == 1);
    check("add_far", aoi_world_add(&w, 3, 100, 100) == 2);
    check("dup_rejected", aoi_world_add(&w, 2, 1, 1) == AOI_ERR_DUP_ID);

    int out[8], n;
    n = aoi_world_query(&w, 1, 0, 0, out, 8);
    check("query_near", n == 1 && out[0] == 2);

    aoi_world_move(&w, 2, 100, 100);
    n = aoi_world_query(&w, 1, 0, 0, out, 8);
    check("query_after_move", n == 0);

    int prev[2] = { 2, 3 }, cur[2] = { 3, 4 }, enter = 0, leave = 0;
    aoi_world_diff(prev, 2, cur, 2, &enter, &leave);
    check("diff_counts", enter == 1 && leave == 1);

    check("remove_ok", aoi_world_remove(&w, 3) == 0);
    n = aoi_world_query(&w, 1, 200, 200, out, 8);
    check("removed_absent", n == 0);

    aoi_world_init(&w, 0.0f, 15.0f);
    check("bad_cell", aoi_world_add(&w, 1, 0, 0) == AOI_ERR_BAD_CELL);
    aoi_world_init(&w, 10.0f, 15.0f);

    check("move_missing", aoi_world_move(&w, 42, 0, 0) == AOI_ERR_NOT_FOUND);
    check("cell_clamp", aoi_cell_coord(-100.0f, 10.0f) == 0 && aoi_cell_coord(1e6f, 10.0f) == GRID - 1);

    World full; aoi_world_init(&full, 10.0f, 15.0f);
    for (int i = 0; i < MAX_ENT; i++) aoi_world_add(&full, i, 0, 0);
    check("capacity", aoi_world_add(&full, 999, 0, 0) == AOI_ERR_FULL);

    /* v1 body (rev == 0) roundtrips exactly and carries no extension. */
    World v1; aoi_world_init(&v1, 10.0f, 15.0f);
    aoi_world_add(&v1, 1, 0, 0);
    aoi_world_add(&v1, 2, 5, 0);
    aoi_world_add(&v1, -3, 100, -100);
    unsigned rev_saved = v1.rev;
    v1.rev = 0;
    unsigned char body[AOI_STATE_MAX];
    int blen = aoi_state_encode(&v1, body, (int)sizeof(body));
    check("v1_encode", blen > 0);
    World back; AoiInfo ai;
    check("v1_decode", aoi_state_decode_info(body, blen, &back, &ai) == blen);
    check("v1_version", ai.version == 1u && ai.prefix_ok == 1);
    check("v1_entities", back.n == 3 && back.e[2].id == -3 && back.e[2].x == 100.0f &&
                         back.e[2].y == -100.0f && back.cell == 10.0f && back.radius == 15.0f);

    /* v2 appends rev; the v1 byte range is identical. */
    v1.rev = rev_saved;
    unsigned char body2[AOI_STATE_MAX];
    int l2 = aoi_state_encode(&v1, body2, (int)sizeof(body2));
    check("v2_longer", l2 > blen);
    check("v2_prefix_same", memcmp(body, body2, (size_t)blen) == 0);
    World b2; AoiInfo ai2;
    check("v2_decode", aoi_state_decode_info(body2, l2, &b2, &ai2) == l2);
    check("v2_fields", ai2.version == 2u && b2.rev == rev_saved && b2.n == 3);

    /* A v1 decoder reads the prefix and ignores the tail: emulate it by
       passing only the v1 byte range and check nothing depends on rev. */
    World legacy; AoiInfo lai;
    check("v1_reads_prefix_only", aoi_state_decode_info(body2, blen, &legacy, &lai) == blen);
    check("v1_prefix_values", legacy.n == 3 && legacy.e[1].id == 2 && legacy.e[1].x == 5.0f);
    /* decoding a pure-v1 body must not invent the v2-only field */
    check("v1_no_v2_field", back.rev == 0u);

    check("truncated", aoi_state_decode(body, 1, &back) != AOI_OK);
    check("arg_null", aoi_state_encode(NULL, body, (int)sizeof(body)) == AOI_ERR_ARG);

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
