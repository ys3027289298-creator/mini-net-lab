
/* clock: network clock offset estimation with min-RTT filtering.
 *
 * State codec keeps the 0bdd277 v1 body as an immutable prefix and appends
 * v2 filter fields as tagged records; see clock.h for the layout contract.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "clock.h"

/* ---- varint + exact-double helpers ------------------------------------- */

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

/* A double travels as two varints: high 32 bits then low 32 bits of its
 * IEEE-754 bits.  Two halves keep every value exact while staying inside the
 * 32-bit varint shift guard. */
static int put_double(unsigned char* b, int cap, double d) {
    unsigned long long u;
    memcpy(&u, &d, sizeof(u));
    int n = varint_put(b, cap, (unsigned)(u >> 32));
    if (n < 0) return -1;
    int m = varint_put(b + n, cap - n, (unsigned)(u & 0xffffffffu));
    if (m < 0) return -1;
    return n + m;
}

static int get_double(const unsigned char* b, int len, double* out) {
    unsigned hi = 0, lo = 0;
    int n = varint_get(b, len, &hi);
    if (n < 0) return -1;
    int m = varint_get(b + n, len - n, &lo);
    if (m < 0) return -1;
    unsigned long long u = ((unsigned long long)hi << 32) | lo;
    memcpy(out, &u, sizeof(*out));
    return n + m;
}

/* Emit {tag, len, payload} where payload is one varint. */
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

const char* clock_err_name(int code) {
    switch (code) {
#define X(name, value, desc) case name: return #name;
    CLOCK_ERR_LIST(X)
#undef X
    default: return "CLK_ERR_UNKNOWN";
    }
}

void clock_sync_init(Sync* sy) { memset(sy, 0, sizeof(*sy)); sy->best_rtt = 1e9; }

int clock_sync_add(Sync* sy, double t_send, double t_recv, double remote_time) {
    if (sy == NULL) return CLK_ERR_ARG;
    if (t_recv < t_send) return CLK_ERR_TIME_ORDER;
    double rtt = t_recv - t_send;
    double offset = remote_time - (t_send + t_recv) * 0.5;
    if (sy->n >= MAX_SAMPLES) {
        memmove(&sy->s[0], &sy->s[1], sizeof(Sample) * (MAX_SAMPLES - 1));
        sy->n = MAX_SAMPLES - 1;
    }
    sy->s[sy->n].rtt = rtt;
    sy->s[sy->n].offset = offset;
    sy->s[sy->n].at = t_recv;
    sy->n++;
    if (rtt < sy->best_rtt) sy->best_rtt = rtt;

    /* keep samples within 1.5x best rtt, average their offsets */
    double sum = 0.0; int cnt = 0;
    for (int i = 0; i < sy->n; i++) {
        if (sy->s[i].rtt <= sy->best_rtt * 1.5 + 1e-9) {
            sum += sy->s[i].offset; cnt++;
        }
    }
    if (cnt == 0) return CLK_ERR_NO_SAMPLES;
    sy->offset = sum / cnt;
    sy->filtered = (unsigned)cnt;
    return cnt;
}

/* ---- state body codec -------------------------------------------------- */

int clock_state_encode(const Sync* sy, unsigned char* buf, int cap) {
    if (sy == NULL || buf == NULL || cap < 0) return CLK_ERR_ARG;
    if (sy->n < 0 || sy->n > MAX_SAMPLES) return CLK_ERR_ARG;

    int o = 0, n;
    n = varint_put(buf, cap - o, (unsigned)sy->n);         if (n < 0) return CLK_ERR_FULL;
         o += n;
    for (int i = 0; i < sy->n; i++) {
        n = put_double(buf + o, cap - o, sy->s[i].rtt);    if (n < 0) return CLK_ERR_FULL;
    o += n;
        n = put_double(buf + o, cap - o, sy->s[i].offset); if (n < 0) return CLK_ERR_FULL;
 o += n;
        n = put_double(buf + o, cap - o, sy->s[i].at);     if (n < 0) return CLK_ERR_FULL;
     o += n;
    }
    n = put_double(buf + o, cap - o, sy->best_rtt);        if (n < 0) return CLK_ERR_FULL;
        o += n;
    n = put_double(buf + o, cap - o, sy->offset);          if (n < 0) return CLK_ERR_FULL;
          o += n;

    /* v2-only field; zero reproduces the v1 body exactly. */
    if (sy->filtered != 0u) {
        n = ext_put(buf + o, cap - o, CLOCK_TAG_VERSION, CLOCK_VERSION_MAX);
        if (n < 0) return CLK_ERR_FULL;
        o += n;
        n = ext_put(buf + o, cap - o, CLOCK_TAG_FILTERED, sy->filtered);
        if (n < 0) return CLK_ERR_FULL;
        o += n;
    }
    return o;
}

static int ext_parse(const unsigned char* b, int len, int at, Sync* sy, ClockInfo* info) {
    int o = at, saw_version = 0, rec = 0;
    unsigned staged_filtered = sy->filtered;

    while (o < len) {
        unsigned tag = 0, plen = 0, value = 0;
        int n = varint_get(b + o, len - o, &tag);
        if (n < 0) return CLK_ERR_EXT;
        o += n;
        n = varint_get(b + o, len - o, &plen);
        if (n < 0) return CLK_ERR_EXT;
        o += n;
        if (plen > (unsigned)(len - o)) return CLK_ERR_EXT;

        if (tag == CLOCK_TAG_VERSION) {
            if (saw_version || rec != 0) return CLK_ERR_EXT;
            int vn = varint_get(b + o, (int)plen, &value);
            if (vn < 0 || (unsigned)vn != plen) return CLK_ERR_EXT;
            if (value < CLOCK_VERSION_MIN || value > CLOCK_VERSION_MAX)
                return CLK_ERR_VERSION;
            saw_version = 1;
            info->version = value;
        } else if (tag == CLOCK_TAG_FILTERED) {
            int vn = varint_get(b + o, (int)plen, &value);
            if (vn < 0 || (unsigned)vn != plen) return CLK_ERR_EXT;
            staged_filtered = value;
            info->ext_fields++;
        } else {
            info->fields_skipped++;
        }
        o += (int)plen;
        rec++;
    }
    if (!saw_version) return CLK_ERR_EXT;   /* extension records need a version */
    sy->filtered = staged_filtered;
    return o;
}

int clock_state_decode_info(const unsigned char* b, int len, Sync* sy, ClockInfo* info) {
    if (info != NULL) memset(info, 0, sizeof(*info));
    if (b == NULL || sy == NULL) {
        if (info) info->err = CLK_ERR_ARG;
        return CLK_ERR_ARG;
    }
    if (len < 0) { if (info) info->err = CLK_ERR_TRUNCATED; return CLK_ERR_TRUNCATED; }

    int o = 0, n;
    unsigned cnt = 0;
    n = varint_get(b, len, &cnt);
    if (n < 0) { if (info) info->err = CLK_ERR_TRUNCATED; return CLK_ERR_TRUNCATED; }
    o += n;
    if (cnt > MAX_SAMPLES) { if (info) info->err = CLK_ERR_TRUNCATED; return CLK_ERR_TRUNCATED; }

    memset(sy, 0, sizeof(*sy));
    sy->best_rtt = 1e9;
    for (unsigned i = 0; i < cnt; i++) {
        n = get_double(b + o, len - o, &sy->s[i].rtt);
        if (n < 0) { if (info) info->err = CLK_ERR_TRUNCATED; return CLK_ERR_TRUNCATED; }
        o += n;
        n = get_double(b + o, len - o, &sy->s[i].offset);
        if (n < 0) { if (info) info->err = CLK_ERR_TRUNCATED; return CLK_ERR_TRUNCATED; }
        o += n;
        n = get_double(b + o, len - o, &sy->s[i].at);
        if (n < 0) { if (info) info->err = CLK_ERR_TRUNCATED; return CLK_ERR_TRUNCATED; }
        o += n;
    }
    n = get_double(b + o, len - o, &sy->best_rtt);
    if (n < 0) { if (info) info->err = CLK_ERR_TRUNCATED; return CLK_ERR_TRUNCATED; }
    o += n;
    n = get_double(b + o, len - o, &sy->offset);
    if (n < 0) { if (info) info->err = CLK_ERR_TRUNCATED; return CLK_ERR_TRUNCATED; }
    o += n;
    sy->n = (int)cnt;

    if (info) {
        info->err = CLK_OK;
        info->version = CLOCK_VERSION_MIN;
        info->prefix_ok = 1;
        info->fields_read = 1 + 3 * (int)cnt + 2;
    }
    if (o == len) return o;

    ClockInfo scratch;
    ClockInfo* inf = info;
    if (inf == NULL) {
        memset(&scratch, 0, sizeof(scratch));
        scratch.version = CLOCK_VERSION_MIN;
        scratch.prefix_ok = 1;
        scratch.fields_read = 1 + 3 * (int)cnt + 2;
        inf = &scratch;
    }
    int r = ext_parse(b, len, o, sy, inf);
    if (r < 0) { if (info) info->err = (clock_err_t)r; return r; }
    if (info) info->err = CLK_OK;
    return r;
}

int clock_state_decode(const unsigned char* b, int len, Sync* sy) {
    return clock_state_decode_info(b, len, sy, NULL);
}

#ifdef MINILAB_SELFTEST
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int minilab_selftest(void) {
    Sync sy;
    clock_sync_init(&sy);
    double true_off = 2.0;
    check("add1", clock_sync_add(&sy, 0.0, 0.10, 0.05 + true_off) > 0);
    check("offset_close", fabs(sy.offset - true_off) < 0.01);
    check("best_rtt", fabs(sy.best_rtt - 0.10) < 1e-9);

    check("add2", clock_sync_add(&sy, 1.0, 1.20, 1.10 + true_off) > 0);
    check("still_close", fabs(sy.offset - true_off) < 0.01);

    check("reject_time_reversal", clock_sync_add(&sy, 5.0, 4.0, 0.0) == CLK_ERR_TIME_ORDER);
    check("reject_null", clock_sync_add(NULL, 0.0, 1.0, 0.0) == CLK_ERR_ARG);

    Sync sp;
    clock_sync_init(&sp);
    for (int i = 0; i < 40; i++) clock_sync_add(&sp, i * 1.0, i * 1.0 + 0.05 + (i % 5) * 0.01, i * 1.0 + 0.025 + true_off);
    check("ring_kept", sp.n == MAX_SAMPLES);
    check("stable_offset", fabs(sp.offset - true_off) < 0.01);

    /* Build a v1-shaped Sync by clearing the v2-only field, then prove the
       encoder emits the bare v1 body and the decoder restores it bit-exactly. */
    Sync v1 = sp;
    v1.filtered = 0u;
    unsigned char body[CLOCK_STATE_MAX];
    int blen = clock_state_encode(&v1, body, (int)sizeof(body));
    check("v1_encode", blen > 0);

    Sync back; ClockInfo ci;
    check("v1_decode", clock_state_decode_info(body, blen, &back, &ci) == blen);
    check("v1_version", ci.version == 1u && ci.prefix_ok == 1);
    check("v1_roundtrip", back.n == v1.n && back.s[0].rtt == v1.s[0].rtt &&
                          back.s[0].at == v1.s[0].at &&
                          back.best_rtt == v1.best_rtt && back.offset == v1.offset);
    /* decoding a pure-v1 body must not invent the v2-only field */
    check("v1_no_v2_field", back.filtered == 0u);

    /* v2 records are appended; the v1 byte range is untouched. */
    Sync v2 = sp; v2.filtered = 3u;
    unsigned char body2[CLOCK_STATE_MAX];
    int l2 = clock_state_encode(&v2, body2, (int)sizeof(body2));
    check("v2_longer", l2 > blen);
    check("v2_prefix_same", memcmp(body, body2, (size_t)blen) == 0);
    Sync b2; ClockInfo ci2;
    check("v2_decode", clock_state_decode_info(body2, l2, &b2, &ci2) == l2);
    check("v2_fields", ci2.version == 2u && b2.filtered == 3u &&
                       b2.best_rtt == v1.best_rtt && b2.offset == v1.offset);

    /* Truncation inside the v1 prefix is a hard error, never a partial state. */
    check("v1_truncated", clock_state_decode(body, 2, &back) == CLK_ERR_TRUNCATED);
    check("arg_null", clock_state_encode(NULL, body, (int)sizeof(body)) == CLK_ERR_ARG);

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
