
/* reconcile: prediction buffer with rollback on authoritative state. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "reconcile.h"

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

const char* recon_err_name(int code) {
    switch (code) {
#define X(name, value, desc) case name: return #name;
    RECON_ERR_LIST(X)
#undef X
    default: return "RECON_ERR_UNKNOWN";
    }
}

void recon_init(Recon* r) { if (r) memset(r, 0, sizeof(*r)); }

int recon_push(Recon* r, unsigned tick, float x) {
    if (r == NULL) return RECON_ERR_ARG;
    if (tick <= r->acked) return RECON_ERR_STALE;    /* stale tick rejected */
    for (int i = 0; i < r->count; i++) {             /* duplicate tick anywhere */
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->p[idx].tick == tick) return RECON_ERR_DUP;
    }
    r->p[r->head].tick = tick;
    r->p[r->head].x = x;
    r->head = (r->head + 1) % BUF;
    if (r->count < BUF) r->count++;
    return 0;
}

/* authoritative state arrives for tick A: drop acked, count rollback if mismatch */
int recon_ack(Recon* r, unsigned tick, float x) {
    if (r == NULL) return RECON_ERR_ARG;
    float pred = 0.0f; int found = 0;
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        if (r->p[idx].tick == tick) { pred = r->p[idx].x; found = 1; break; }
    }
    r->last_err = found ? fabsf(pred - x) : -1.0f;
    if (found) r->ack_hits++;
    if (found && fabsf(pred - x) > 1e-3f) r->rollbacks++;
    if (tick > r->acked) r->acked = tick;
    return found ? 1 : 0;
}

/* ---- canonical state body --------------------------------------------- */

int recon_state_encode(const Recon* r, unsigned char* buf, int cap) {
    if (r == NULL || buf == NULL || cap < 0) return RECON_ERR_ARG;
    if (r->count < 0 || r->count > BUF) return RECON_ERR_ARG;

    int o = 0, n;
    n = varint_put(buf, cap, (unsigned)r->count);
    if (n < 0) return RECON_ERR_BUFFER;
    o += n;
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - r->count + i + BUF * 2) % BUF;   /* oldest first */
        n = varint_put(buf + o, cap - o, r->p[idx].tick);
        if (n < 0) return RECON_ERR_BUFFER;
        o += n;
        n = varint_put(buf + o, cap - o, fbits(r->p[idx].x));
        if (n < 0) return RECON_ERR_BUFFER;
        o += n;
    }

    /* baseline fields: acked, rollbacks, last_err */
    n = varint_put(buf + o, cap - o, r->acked);
    if (n < 0) return RECON_ERR_BUFFER;
    o += n;
    n = varint_put(buf + o, cap - o, (unsigned)r->rollbacks);
    if (n < 0) return RECON_ERR_BUFFER;
    o += n;
    n = varint_put(buf + o, cap - o, fbits(r->last_err));
    if (n < 0) return RECON_ERR_BUFFER;
    o += n;

    /* v2-only field; zero reproduces the v1 body exactly. */
    if (r->ack_hits != 0u) {
        n = ext_put(buf + o, cap - o, RECON_TAG_VERSION, RECON_VERSION_MAX);
        if (n < 0) return RECON_ERR_BUFFER;
        o += n;
        n = ext_put(buf + o, cap - o, RECON_TAG_ACK_HITS, r->ack_hits);
        if (n < 0) return RECON_ERR_BUFFER;
        o += n;
    }
    return o;
}

static int ext_parse(const unsigned char* b, int len, int at, Recon* r, ReconInfo* info) {
    int o = at, saw_version = 0, rec = 0;
    unsigned staged_hits = r->ack_hits;

    while (o < len) {
        unsigned tag = 0, plen = 0, value = 0;
        int n = varint_get(b + o, len - o, &tag);
        if (n < 0) return RECON_ERR_EXT;
        o += n;
        n = varint_get(b + o, len - o, &plen);
        if (n < 0) return RECON_ERR_EXT;
        o += n;
        if (plen > (unsigned)(len - o)) return RECON_ERR_EXT;

        if (tag == RECON_TAG_VERSION) {
            if (saw_version || rec != 0) return RECON_ERR_EXT;
            int vn = varint_get(b + o, (int)plen, &value);
            if (vn < 0 || (unsigned)vn != plen) return RECON_ERR_EXT;
            if (value < RECON_VERSION_MIN || value > RECON_VERSION_MAX)
                return RECON_ERR_VERSION;
            saw_version = 1;
            info->version = value;
        } else if (tag == RECON_TAG_ACK_HITS) {
            int vn = varint_get(b + o, (int)plen, &value);
            if (vn < 0 || (unsigned)vn != plen) return RECON_ERR_EXT;
            staged_hits = value;
        } else {
            info->fields_skipped++;
        }
        o += (int)plen;
        rec++;
    }
    if (!saw_version) return RECON_ERR_EXT;
    r->ack_hits = staged_hits;
    return o;
}

int recon_state_decode_info(const unsigned char* b, int len, Recon* r, ReconInfo* info) {
    if (info != NULL) memset(info, 0, sizeof(*info));
    if (b == NULL || r == NULL) {
        if (info) info->err = RECON_ERR_ARG;
        return RECON_ERR_ARG;
    }
    if (len < 0) { if (info) info->err = RECON_ERR_TRUNCATED; return RECON_ERR_TRUNCATED; }

    int o = 0, n;
    unsigned cnt = 0;
    n = varint_get(b, len, &cnt);
    if (n < 0) { if (info) info->err = RECON_ERR_TRUNCATED; return RECON_ERR_TRUNCATED; }
    o += n;
    if (cnt > BUF) { if (info) info->err = RECON_ERR_TRUNCATED; return RECON_ERR_TRUNCATED; }

    memset(r, 0, sizeof(*r));
    for (unsigned i = 0; i < cnt; i++) {
        unsigned tick = 0, fx = 0;
        n = varint_get(b + o, len - o, &tick);
        if (n < 0) { if (info) info->err = RECON_ERR_TRUNCATED; return RECON_ERR_TRUNCATED; }
        o += n;
        n = varint_get(b + o, len - o, &fx);
        if (n < 0) { if (info) info->err = RECON_ERR_TRUNCATED; return RECON_ERR_TRUNCATED; }
        o += n;
        r->p[i].tick = tick;
        r->p[i].x = frombits(fx);
    }
    unsigned acked = 0, rb = 0, fe = 0;
    n = varint_get(b + o, len - o, &acked);
    if (n < 0) { if (info) info->err = RECON_ERR_TRUNCATED; return RECON_ERR_TRUNCATED; }
    o += n;
    n = varint_get(b + o, len - o, &rb);
    if (n < 0) { if (info) info->err = RECON_ERR_TRUNCATED; return RECON_ERR_TRUNCATED; }
    o += n;
    n = varint_get(b + o, len - o, &fe);
    if (n < 0) { if (info) info->err = RECON_ERR_TRUNCATED; return RECON_ERR_TRUNCATED; }
    o += n;
    r->count = (int)cnt;
    r->head = (int)cnt % BUF;
    r->acked = acked;
    r->rollbacks = (int)rb;
    r->last_err = frombits(fe);

    if (info) {
        info->err = RECON_OK;
        info->version = RECON_VERSION_MIN;
        info->prefix_ok = 1;
        info->predictions = (int)cnt;
    }
    if (o == len) return o;

    ReconInfo scratch;
    ReconInfo* inf = info;
    if (inf == NULL) {
        memset(&scratch, 0, sizeof(scratch));
        scratch.version = RECON_VERSION_MIN;
        scratch.prefix_ok = 1;
        scratch.predictions = (int)cnt;
        inf = &scratch;
    }
    int res = ext_parse(b, len, o, r, inf);
    if (res < 0) { if (info) info->err = (recon_err_t)res; return res; }
    if (info) info->err = RECON_OK;
    return res;
}

int recon_state_decode(const unsigned char* b, int len, Recon* r) {
    return recon_state_decode_info(b, len, r, NULL);
}

#ifdef MINILAB_SELFTEST
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int minilab_selftest(void) {
    Recon r;
    recon_init(&r);
    check("push1", recon_push(&r, 1, 1.0f) == 0);
    check("push2", recon_push(&r, 2, 2.0f) == 0);
    check("dup_rejected", recon_push(&r, 2, 5.0f) == RECON_ERR_DUP);

    check("ack_match", recon_ack(&r, 2, 2.0f) == 1);
    check("no_rollback", r.rollbacks == 0);

    check("stale_rejected", recon_push(&r, 2, 3.0f) == RECON_ERR_STALE);

    recon_push(&r, 3, 3.0f);
    check("ack_mismatch", recon_ack(&r, 3, 3.5f) == 1);
    check("rollback_counted", r.rollbacks == 1);
    check("err_recorded", fabsf(r.last_err - 0.5f) < 1e-5f);

    check("ack_missing", recon_ack(&r, 99, 0.0f) == 0);

    Recon big;
    recon_init(&big);
    for (int i = 1; i <= 70; i++) recon_push(&big, (unsigned)i, (float)i);
    check("buffer_capped", big.count == BUF);

    /* v1 body: ack_hits == 0 -> the baseline byte layout. */
    Recon v1; recon_init(&v1);
    recon_push(&v1, 1, 1.0f);
    recon_push(&v1, 2, 2.0f);
    recon_push(&v1, 3, 3.25f);
    recon_ack(&v1, 3, 3.5f);              /* acked=3, rollbacks=1, last_err=0.25 */
    v1.ack_hits = 0u;                     /* baseline client has no such field */
    unsigned char body[RECON_STATE_MAX];
    int blen = recon_state_encode(&v1, body, (int)sizeof(body));
    check("v1_encode", blen > 0);
    Recon back; ReconInfo ri;
    check("v1_decode", recon_state_decode_info(body, blen, &back, &ri) == blen);
    check("v1_version", ri.version == 1u && ri.prefix_ok == 1);
    check("v1_roundtrip", back.count == 3 && back.p[0].tick == 1 && back.p[2].tick == 3 &&
                          back.p[2].x == 3.25f && back.acked == 3u &&
                          back.rollbacks == 1 && fabsf(back.last_err - 0.25f) < 1e-6f);

    /* Same baseline state with the v2 field set: v1 byte range is identical. */
    v1.ack_hits = 2u;
    unsigned char body2[RECON_STATE_MAX];
    int l2 = recon_state_encode(&v1, body2, (int)sizeof(body2));
    check("v2_longer", l2 > blen);
    check("v2_prefix_same", memcmp(body, body2, (size_t)blen) == 0);
    Recon b2; ReconInfo ri2;
    check("v2_decode", recon_state_decode_info(body2, l2, &b2, &ri2) == l2);
    check("v2_fields", ri2.version == 2u && b2.acked == 3u && b2.rollbacks == 1 &&
                       b2.ack_hits == 2u && b2.count == 3);

    /* A v1 decoder consumes only the baseline byte range and sees the same
       predictions, acked, rollbacks and last_err; the tail is invisible. */
    Recon legacy; ReconInfo lri;
    check("v1_reads_prefix", recon_state_decode_info(body2, blen, &legacy, &lri) == blen);
    check("v1_prefix_values", legacy.count == 3 && legacy.p[1].tick == 2 &&
                              legacy.p[1].x == 2.0f && legacy.acked == 3u &&
                              legacy.rollbacks == 1 && legacy.ack_hits == 0u);
    /* decoding a pure-v1 body must not invent the v2-only field */
    check("v1_no_v2_field", back.ack_hits == 0u);

    check("truncated", recon_state_decode(body, 1, &back) == RECON_ERR_TRUNCATED);
    check("arg_null", recon_push(NULL, 1, 0) == RECON_ERR_ARG);

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
