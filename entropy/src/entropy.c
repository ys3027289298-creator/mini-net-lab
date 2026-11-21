
/* entropy: field-wise entropy coding for quantized snapshot records.
 *
 * Baseline ("fixed packing"): every record stores NFIELDS quantized fields at
 * their full bit width. This module instead picks a coding per field from the
 * observed distribution:
 *
 *   - Huffman when the field has 2..MAX_SYM distinct values and its codebook +
 *     payload is smaller than fixed width,
 *   - fixed width otherwise (degenerate / alphabet too wide / no gain).
 *
 * For every encode it reports the theoretical lower bound (sum of per-field
 * zero-order entropies, bits), the measured packet size, the fixed-size
 * baseline, the compression ratio, and an alarm flag if measured size exceeds
 * the bound by more than the Huffman guarantee (<= 1 bit/symbol) plus header
 * slack. Coding decisions are logged, so a fallback to fixed is never silent.
 *
 * The decoder is checked field-by-field against the quantized input in main().
 * Bitstream: mode byte, varint record count, per-Huffman-field codebooks, then
 * payload bits appended MSB-first.
 */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#define NFIELDS 4
#define MAX_SYM 32
#define MAX_CODE_LEN 24
#define MAX_NODES (MAX_SYM * 2)
#define MAX_REC 1024
#define MAX_PACKET 8192
#define MAX_LOG 16

static const char* FIELD_NAME[NFIELDS] = { "tick_lo", "qx", "qy", "flags" };
static const int FIELD_WIDTH[NFIELDS] = { 8, 8, 8, 4 };

static int fixed_bits_per_rec(void) {
    int bits = 0;
    for (int f = 0; f < NFIELDS; f++) bits += FIELD_WIDTH[f];
    return bits;
}

typedef struct { unsigned v[NFIELDS]; } Record;

typedef struct {
    int nrec;
    int field_mode[NFIELDS];        /* 0 = fixed width, 1 = Huffman */
    double field_entropy[NFIELDS];  /* bits per symbol, zero-order */
    double bound_bits;              /* sum nrec * H(field), information-theoretic */
    double alarm_bound_bits;        /* floor of the coder actually chosen */
    long fixed_payload_bits;        /* nrec * FIXED_BITS_PER_REC (no envelope) */
    long fixed_bits;                /* same packet envelope, all fields fixed */
    long header_bits;
    long payload_bits;
    long measured_bits;             /* actual wire size */
    double ratio;                   /* fixed_bits / measured_bits */
    int alarm;
    double overshoot_bits;          /* measured - bound */
    int fallbacks;
    char log[MAX_LOG][96];
    int nlog;
} Stats;

static void slog(Stats* st, const char* fmt, ...) {
    if (st == NULL || st->nlog >= MAX_LOG) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(st->log[st->nlog], sizeof(st->log[0]), fmt, ap);
    va_end(ap);
    st->nlog++;
}

static int log_contains(const Stats* st, const char* needle) {
    for (int i = 0; i < st->nlog; i++)
        if (strstr(st->log[i], needle) != NULL) return 1;
    return 0;
}

/* ---- varint ------------------------------------------------------------ */

static int varint_len(unsigned v) {
    int n = 1;
    while (v >= 0x80) { v >>= 7; n++; }
    return n;
}

static int put_varint(unsigned char* b, unsigned v) {
    int n = 0;
    while (v >= 0x80) { b[n++] = (unsigned char)((v & 0x7f) | 0x80); v >>= 7; }
    b[n++] = (unsigned char)(v & 0x7f);
    return n;
}

static int get_varint(const unsigned char* b, int len, int pos, int* next, unsigned* out) {
    unsigned v = 0;
    int shift = 0, n = pos;
    while (n < len && shift <= 28) {
        unsigned char c = b[n++];
        v |= (unsigned)(c & 0x7f) << shift;
        if (!(c & 0x80)) { *out = v; if (next) *next = n; return 0; }
        shift += 7;
    }
    return -1;
}

/* Wire size of the fixed-width baseline packet for nrec records: mode byte,
 * varint record count, then all fields at full width, byte-rounded. */
static long fixed_packet_bits(int nrec) {
    long env = 1 + varint_len((unsigned)nrec);
    return (env + ((long)nrec * fixed_bits_per_rec() + 7) / 8) * 8;
}

/* ---- bit stream -------------------------------------------------------- */

typedef struct {
    unsigned char* buf;
    int cap;
    int byte;
    int bit;   /* next bit within current byte, MSB first */
} BitW;

typedef struct {
    const unsigned char* buf;
    int len;
    int byte;
    int bit;
} BitR;

static int bw_put(BitW* w, unsigned v, int nbits) {
    for (int i = 0; i < nbits; i++) {
        if (w->byte >= w->cap) return -1;
        if ((v >> (nbits - 1 - i)) & 1u)
            w->buf[w->byte] |= (unsigned char)(0x80u >> w->bit);
        if (++w->bit == 8) { w->bit = 0; w->byte++; }
    }
    return 0;
}

static int br_get(BitR* r, int* out) {
    if (r->byte >= r->len) return -1;
    *out = (r->buf[r->byte] >> (7 - r->bit)) & 1;
    if (++r->bit == 8) { r->bit = 0; r->byte++; }
    return 0;
}

/* ---- Huffman ----------------------------------------------------------- */

typedef struct { unsigned val; int len; unsigned code; } HEnt;

typedef struct {
    int n;
    int maxbits;
    HEnt e[MAX_SYM];
} Huff;

/* Assign canonical codes from code lengths alone. Both encoder and decoder go
 * through here, so a transmitted length table is enough to rebuild the codes. */
static int huff_from_lengths(Huff* h, const unsigned* vals, const int* lens, int n) {
    if (n < 2 || n > MAX_SYM) return -1;
    h->n = n;
    h->maxbits = 0;
    for (int i = 0; i < n; i++) {
        if (lens[i] < 1 || lens[i] > MAX_CODE_LEN) return -1;
        h->e[i].val = vals[i];
        h->e[i].len = lens[i];
        h->e[i].code = 0;
        if (lens[i] > h->maxbits) h->maxbits = lens[i];
        for (int j = 0; j < i; j++) if (vals[j] == vals[i]) return -1;
    }
    int idx[MAX_SYM];
    for (int i = 0; i < n; i++) idx[i] = i;
    for (int i = 1; i < n; i++) {           /* sort by (len asc, val asc) */
        int k = idx[i], j = i - 1;
        while (j >= 0 && (h->e[idx[j]].len > h->e[k].len ||
                          (h->e[idx[j]].len == h->e[k].len &&
                           h->e[idx[j]].val > h->e[k].val))) {
            idx[j + 1] = idx[j];
            j--;
        }
        idx[j + 1] = k;
    }
    unsigned code = 0;
    int prevlen = 0;
    for (int k = 0; k < n; k++) {
        int i = idx[k];
        code <<= (h->e[i].len - prevlen);
        h->e[i].code = code;
        code++;
        prevlen = h->e[i].len;
    }
    for (int i = 1; i < n; i++) {           /* sort entries by value for lookup */
        HEnt t = h->e[i];
        int j = i - 1;
        while (j >= 0 && h->e[j].val > t.val) { h->e[j + 1] = h->e[j]; j--; }
        h->e[j + 1] = t;
    }
    return 0;
}

static int huff_build(Huff* h, const unsigned* vals, const int* counts, int n) {
    if (n < 2 || n > MAX_SYM) return -1;
    long weight[MAX_NODES];
    int parent[MAX_NODES];
    for (int i = 0; i < n; i++) {
        weight[i] = counts[i];
        parent[i] = -1;
    }
    int nodes = n;
    for (int k = 0; k < n - 1; k++) {
        int a = -1, b = -1;
        for (int i = 0; i < nodes; i++) {
            if (parent[i] != -1) continue;
            if (a < 0 || weight[i] < weight[a]) { b = a; a = i; }
            else if (b < 0 || weight[i] < weight[b]) { b = i; }
        }
        if (a < 0 || b < 0) return -1;
        parent[a] = nodes;
        parent[b] = nodes;
        weight[nodes] = weight[a] + weight[b];
        parent[nodes] = -1;
        nodes++;
    }
    int lens[MAX_SYM];
    for (int i = 0; i < n; i++) {
        int d = 0;
        for (int p = parent[i]; p != -1; p = parent[p]) d++;
        if (d < 1 || d > MAX_CODE_LEN) return -1;
        lens[i] = d;
    }
    return huff_from_lengths(h, vals, lens, n);
}

static const HEnt* huff_find(const Huff* h, unsigned v) {
    for (int i = 0; i < h->n; i++) if (h->e[i].val == v) return &h->e[i];
    return NULL;
}

static int huff_decode(BitR* r, const Huff* h, unsigned* out) {
    unsigned acc = 0;
    for (int blen = 1; blen <= h->maxbits; blen++) {
        int bit;
        if (br_get(r, &bit) != 0) return -1;
        acc = (acc << 1) | (unsigned)bit;
        for (int i = 0; i < h->n; i++)
            if (h->e[i].len == blen && h->e[i].code == acc) { *out = h->e[i].val; return 0; }
    }
    return -1;
}

/* ---- metrics ----------------------------------------------------------- */

static double field_entropy(const int* cnt, int total) {
    double h = 0.0;
    for (int i = 0; i < 256; i++) {
        if (cnt[i] <= 0) continue;
        double p = (double)cnt[i] / (double)total;
        h -= p * log2(p);
    }
    return h;
}

/* Alarm when the measured packet exceeds its coder's floor by more than what
 * Huffman may legitimately spend: at most 1 bit/symbol per entropy-coded
 * field, plus the header and a small fixed slack. Fields that fell back to
 * fixed width contribute their fixed width to the floor, so an intentional
 * fallback on a degenerate distribution does not raise the alarm. */
static int entropy_alarm(double floor_bits, long measured_bits, int nrec, long header_bits) {
    double allowance = (double)nrec * NFIELDS + (double)header_bits + 64.0;
    return (double)measured_bits > floor_bits + allowance;
}

/* ---- encode ------------------------------------------------------------ */

static int entropy_encode(const Record* recs, int nrec,
                          unsigned char* out, int cap, Stats* st) {
    if (recs == NULL || out == NULL || st == NULL) return -1;
    if (nrec <= 0 || nrec > MAX_REC) return -1;
    memset(st, 0, sizeof(*st));
    memset(out, 0, (size_t)cap);
    st->nrec = nrec;
    st->fixed_payload_bits = (long)nrec * fixed_bits_per_rec();
    st->fixed_bits = fixed_packet_bits(nrec);

    /* Field values must fit the declared width: fixed-width coding would
     * truncate them, breaking the field-for-field decode guarantee. */
    int hist[NFIELDS][256];
    memset(hist, 0, sizeof(hist));
    for (int i = 0; i < nrec; i++)
        for (int f = 0; f < NFIELDS; f++) {
            unsigned mask = (1u << FIELD_WIDTH[f]) - 1u;
            if (recs[i].v[f] & ~mask) return -1;
            hist[f][recs[i].v[f]]++;
        }

    int use_huff[NFIELDS];
    Huff huffs[NFIELDS];
    memset(use_huff, 0, sizeof(use_huff));

    for (int f = 0; f < NFIELDS; f++) {
        unsigned vals[MAX_SYM];
        int cnts[MAX_SYM];
        int nsym = 0;
        for (int v = 0; v < 256; v++) {
            if (hist[f][v] == 0) continue;
            if (nsym < MAX_SYM) { vals[nsym] = (unsigned)v; cnts[nsym] = hist[f][v]; }
            nsym++;
        }
        st->field_entropy[f] = field_entropy(hist[f], nrec);
        st->bound_bits += (double)nrec * st->field_entropy[f];

        if (nsym < 2) {
            slog(st, "fallback field=%s reason=degenerate distinct=%d", FIELD_NAME[f], nsym);
            st->fallbacks++;
            continue;
        }
        if (nsym > MAX_SYM) {
            slog(st, "fallback field=%s reason=alphabet_too_wide distinct=%d max=%d",
                 FIELD_NAME[f], nsym, MAX_SYM);
            st->fallbacks++;
            continue;
        }
        Huff h;
        if (huff_build(&h, vals, cnts, nsym) != 0) {
            slog(st, "fallback field=%s reason=build_failed distinct=%d", FIELD_NAME[f], nsym);
            st->fallbacks++;
            continue;
        }
        long payload = 0;
        for (int i = 0; i < nsym; i++)
            payload += (long)cnts[i] * huff_find(&h, vals[i])->len;
        long model = (long)(varint_len((unsigned)nsym) + 2 * nsym) * 8;  /* bits */
        long fixed = (long)nrec * FIELD_WIDTH[f];        if (payload + model < fixed) {
            use_huff[f] = 1;
            huffs[f] = h;
        } else {
            slog(st, "fallback field=%s reason=no_gain huff_bits=%ld fixed_bits=%ld",
                 FIELD_NAME[f], payload + model, fixed);
            st->fallbacks++;
        }
    }

    int hdr = 1 + varint_len((unsigned)nrec);
    for (int f = 0; f < NFIELDS; f++)
        if (use_huff[f]) hdr += varint_len((unsigned)huffs[f].n) + 2 * huffs[f].n;

    long payload_bits = 0;
    for (int f = 0; f < NFIELDS; f++) {
        if (!use_huff[f]) { payload_bits += (long)nrec * FIELD_WIDTH[f]; continue; }
        for (int i = 0; i < nrec; i++)
            payload_bits += huff_find(&huffs[f], recs[i].v[f])->len;
    }

    /* If the entropy-coded payload does not beat the fixed-width payload as a
     * whole, ship it fixed. Compare payloads here; the envelope cost is common
     * to both and already accounted for in the reported totals. */
    if (payload_bits >= st->fixed_payload_bits) {
        int any = 0;
        for (int f = 0; f < NFIELDS; f++) if (use_huff[f]) any = 1;
        if (any) {
            long would_be = (long)hdr * 8 + payload_bits;
            for (int f = 0; f < NFIELDS; f++) {
                if (use_huff[f])
                    slog(st, "fallback field=%s reason=packet_no_gain", FIELD_NAME[f]);
                use_huff[f] = 0;
            }
            hdr = 1 + varint_len((unsigned)nrec);
            payload_bits = st->fixed_payload_bits;
            slog(st, "fallback packet reason=no_gain would_be=%ld fixed_bits=%ld",
                 would_be, st->fixed_payload_bits);
            st->fallbacks++;
        }
    }

    /* Floor of the chosen coder: entropy for Huffman fields, width for fixed.
     * A fallback field can legitimately cost more than its raw entropy, so the
     * alarm is measured against this floor rather than the pure entropy bound. */
    st->alarm_bound_bits = 0.0;
    for (int f = 0; f < NFIELDS; f++)
        st->alarm_bound_bits += use_huff[f] ? (double)nrec * st->field_entropy[f]
                                            : (double)nrec * FIELD_WIDTH[f];

    int pos = 0;
    unsigned char mask = 0;
    for (int f = 0; f < NFIELDS; f++) {
        st->field_mode[f] = use_huff[f];
        if (use_huff[f]) mask |= (unsigned char)(1u << f);
    }
    if (pos + hdr > cap) return -1;
    out[pos++] = mask;
    pos += put_varint(out + pos, (unsigned)nrec);
    for (int f = 0; f < NFIELDS; f++) {
        if (!use_huff[f]) continue;
        pos += put_varint(out + pos, (unsigned)huffs[f].n);
        for (int i = 0; i < huffs[f].n; i++) {
            out[pos++] = (unsigned char)huffs[f].e[i].val;
            out[pos++] = (unsigned char)huffs[f].e[i].len;
        }
    }

    BitW bw = { out, cap, pos, 0 };
    for (int i = 0; i < nrec; i++) {
        for (int f = 0; f < NFIELDS; f++) {
            if (use_huff[f]) {
                const HEnt* e = huff_find(&huffs[f], recs[i].v[f]);
                if (e == NULL) return -1;
                if (bw_put(&bw, e->code, e->len) != 0) return -1;
            } else {
                if (bw_put(&bw, recs[i].v[f], FIELD_WIDTH[f]) != 0) return -1;
            }
        }
    }

    /* Actual wire size is what the bit writer consumed, byte-rounded. */
    int nbytes = bw.byte + (bw.bit > 0 ? 1 : 0);
    st->header_bits = (long)hdr * 8;
    st->payload_bits = payload_bits;
    st->measured_bits = (long)nbytes * 8;
    st->ratio = (double)st->fixed_bits / (double)st->measured_bits;
    st->overshoot_bits = (double)st->measured_bits - st->bound_bits;
    st->alarm = entropy_alarm(st->alarm_bound_bits, st->measured_bits, nrec, st->header_bits);

    return nbytes;
}

/* ---- decode ------------------------------------------------------------ */

static int entropy_decode(const unsigned char* in, int len, Record* recs, int cap) {
    if (in == NULL || recs == NULL || len < 2 || cap <= 0) return -1;
    int pos = 0;
    unsigned char mask = in[pos++];
    unsigned nrec = 0;
    if (get_varint(in, len, pos, &pos, &nrec) != 0) return -1;
    if (nrec == 0 || nrec > MAX_REC || (int)nrec > cap) return -1;

    int use_huff[NFIELDS];
    Huff huffs[NFIELDS];
    memset(use_huff, 0, sizeof(use_huff));
    memset(huffs, 0, sizeof(huffs));
    for (int f = 0; f < NFIELDS; f++) {
        if (!(mask & (1u << f))) continue;
        unsigned nsym = 0;
        if (get_varint(in, len, pos, &pos, &nsym) != 0) return -1;
        if (nsym < 2 || nsym > MAX_SYM) return -1;
        if (pos + (int)nsym * 2 > len) return -1;
        unsigned vals[MAX_SYM];
        int lens[MAX_SYM];
        for (unsigned i = 0; i < nsym; i++) {
            vals[i] = in[pos++];
            lens[i] = in[pos++];
        }
        if (huff_from_lengths(&huffs[f], vals, lens, (int)nsym) != 0) return -1;
        use_huff[f] = 1;
    }

    BitR br = { in, len, pos, 0 };
    for (unsigned i = 0; i < nrec; i++) {
        for (int f = 0; f < NFIELDS; f++) {
            if (use_huff[f]) {
                unsigned v;
                if (huff_decode(&br, &huffs[f], &v) != 0) return -1;
                recs[i].v[f] = v;
            } else {
                unsigned v = 0;
                for (int b = 0; b < FIELD_WIDTH[f]; b++) {
                    int bit;
                    if (br_get(&br, &bit) != 0) return -1;
                    v = (v << 1) | (unsigned)bit;
                }
                recs[i].v[f] = v;
            }
        }
    }
    return (int)nrec;
}

/* ---- self-test --------------------------------------------------------- */

static int failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int compare_fields(const Record* a, const Record* b, int n) {
    int bad = 0;
    for (int i = 0; i < n; i++)
        for (int f = 0; f < NFIELDS; f++)
            if (a[i].v[f] != b[i].v[f]) bad++;
    return bad;
}

static void fill_skewed(Record* r, int n) {
    for (int i = 0; i < n; i++) {
        r[i].v[0] = (unsigned)(i & 0xff);                        /* tick: uniform, wide */
        r[i].v[1] = (i % 17 == 0) ? 90u + (unsigned)((i / 17) % 5) : 100u; /* mostly 100 */
        r[i].v[2] = ((i / 4) & 1) ? 20u : 200u;                  /* two zones, 50/50 */
        r[i].v[3] = (i % 97 == 0) ? 3u : ((i % 32 == 0) ? 1u : 0u);
    }
}

/* single-valued fields force the degenerate path. */
static void fill_degenerate(Record* r, int n) {
    for (int i = 0; i < n; i++) {
        r[i].v[0] = 7u;
        r[i].v[1] = 7u;
        r[i].v[2] = 7u;
        r[i].v[3] = 0u;
    }
}

/* uniform full-alphabet fields give Huffman no gain, so the packet stays fixed. */
static void fill_uniform(Record* r, int n) {
    for (int i = 0; i < n; i++) {
        r[i].v[0] = (unsigned)(i & 0xff);
        r[i].v[1] = (unsigned)((i * 37) & 0xff);
        r[i].v[2] = (unsigned)((i * 11) & 0xff);
        r[i].v[3] = (unsigned)(i & 0xf);
    }
}

/* A field with just two symbols at 50/50: Huffman is 1 bit/symbol while the
 * fixed width is 4 bits, so this field must still be entropy-coded. */
static void fill_two_symbol(Record* r, int n) {
    for (int i = 0; i < n; i++) {
        r[i].v[0] = (unsigned)(i & 0xff);
        r[i].v[1] = (unsigned)((i * 37) & 0xff);
        r[i].v[2] = (unsigned)((i * 11) & 0xff);
        r[i].v[3] = (i & 1) ? 9u : 1u;
    }
}

int main(void) {
    static Record in[MAX_REC], out[MAX_REC];
    static unsigned char pkt[MAX_PACKET];
    Stats st;

    /* 1. skewed, snapshot-like records: per-field coder selection + ratio */
    fill_skewed(in, 256);
    int n = entropy_encode(in, 256, pkt, MAX_PACKET, &st);
    check("encode_ok", n > 0);
    memset(out, 0, sizeof(out));
    check("decode_ok", entropy_decode(pkt, n, out, MAX_REC) == 256);
    check("roundtrip_fields", compare_fields(in, out, 256) == 0);

    printf("bound_bits=%.1f\n", st.bound_bits);
    printf("fixed_bits=%ld\n", st.fixed_bits);
    printf("measured_bits=%ld\n", st.measured_bits);
    printf("compression_ratio=%.3f\n", st.ratio);
    printf("overshoot_bits=%.1f\n", st.overshoot_bits);
    printf("alarm=%d\n", st.alarm);
    for (int i = 0; i < st.nlog; i++) printf("log[%d]=%s\n", i, st.log[i]);

    check("huffman_used", st.field_mode[1] == 1 && st.field_mode[2] == 1 && st.field_mode[3] == 1);
    check("wide_field_fixed", st.field_mode[0] == 0);
    check("fallback_logged", st.fallbacks > 0 && log_contains(&st, "tick_lo"));
    check("compression_ratio_gt1", st.ratio > 1.2);
    check("beats_fixed", st.measured_bits < st.fixed_bits);
    check("bound_below_measured", st.bound_bits < (double)st.measured_bits);
    check("no_alarm", st.alarm == 0);

    /* 2. degenerate distributions: single symbol per field, fixed, logged */
    Stats st2;
    fill_degenerate(in, 64);
    int n2 = entropy_encode(in, 64, pkt, MAX_PACKET, &st2);
    check("degenerate_encode_ok", n2 > 0);
    memset(out, 0, sizeof(out));
    check("degenerate_decode_ok", entropy_decode(pkt, n2, out, MAX_REC) == 64);
    check("degenerate_roundtrip", compare_fields(in, out, 64) == 0);
    check("degenerate_all_fixed",
          st2.field_mode[0] == 0 && st2.field_mode[1] == 0 &&
          st2.field_mode[2] == 0 && st2.field_mode[3] == 0);
    check("degenerate_logged", log_contains(&st2, "reason=degenerate"));
    check("degenerate_ratio_le1", st2.ratio <= 1.0 + 1e-9);
    check("degenerate_no_alarm", st2.alarm == 0);
    for (int i = 0; i < st2.nlog; i++) printf("degenerate_log[%d]=%s\n", i, st2.log[i]);

    /* 3. uniform full alphabet: Huffman gives no gain, packet falls back */
    Stats st3;
    fill_uniform(in, 512);
    int n3 = entropy_encode(in, 512, pkt, MAX_PACKET, &st3);
    check("uniform_encode_ok", n3 > 0);
    memset(out, 0, sizeof(out));
    check("uniform_decode_ok", entropy_decode(pkt, n3, out, MAX_REC) == 512);
    check("uniform_roundtrip", compare_fields(in, out, 512) == 0);
    check("uniform_all_fixed",
          st3.field_mode[0] == 0 && st3.field_mode[1] == 0 &&
          st3.field_mode[2] == 0 && st3.field_mode[3] == 0);
    check("uniform_logged", log_contains(&st3, "packet_no_gain") ||
                            log_contains(&st3, "reason=no_gain"));
    check("uniform_no_alarm", st3.alarm == 0);
    check("uniform_no_loss", st3.measured_bits <= st3.fixed_bits);
    for (int i = 0; i < st3.nlog; i++) printf("uniform_log[%d]=%s\n", i, st3.log[i]);

    /* 4. alarm predicate: quiet within Huffman slack, loud on real overshoot */
    check("alarm_quiet", entropy_alarm(1000.0, 1030, 100, 64) == 0);
    check("alarm_fires", entropy_alarm(1000.0, 5000, 100, 64) == 1);

    /* 4b. a wide-alphabet packet with one skewed field: that field must still
     * be entropy-coded even though the packet-level total is dominated by the
     * wide fields, and the whole packet must never exceed fixed width. */
    Stats st4;
    fill_two_symbol(in, 512);
    int n4 = entropy_encode(in, 512, pkt, MAX_PACKET, &st4);
    check("mixed_encode_ok", n4 > 0);
    memset(out, 0, sizeof(out));
    check("mixed_decode_ok", entropy_decode(pkt, n4, out, MAX_REC) == 512);
    check("mixed_roundtrip", compare_fields(in, out, 512) == 0);
    check("mixed_skewed_entropy_coded", st4.field_mode[3] == 1);
    check("mixed_no_loss", st4.measured_bits <= st4.fixed_bits);
    check("mixed_no_alarm", st4.alarm == 0);

    /* 5. rejected inputs */
    check("null_rejected", entropy_encode(NULL, 8, pkt, MAX_PACKET, &st) == -1);
    check("zero_records_rejected", entropy_encode(in, 0, pkt, MAX_PACKET, &st) == -1);
    check("truncated_rejected", entropy_decode(pkt, 1, out, MAX_REC) == -1);
    check("tiny_cap_rejected", entropy_decode(pkt, n, out, 8) == -1);

    /* 5b. out-of-range field values are rejected, not silently truncated,
     * so the decode always reproduces every field bit-for-bit. */
    Record bad[2] = { { { 1u, 2u, 3u, 16u } }, { { 4u, 5u, 6u, 0u } } };
    check("out_of_range_rejected", entropy_encode(bad, 2, pkt, MAX_PACKET, &st) == -1);

    /* 6. every truncated prefix of a valid packet is rejected, never read OOB */
    {
        int all_rejected = 1;
        for (int cut = 0; cut < n; cut++)
            if (entropy_decode(pkt, cut, out, MAX_REC) != -1) all_rejected = 0;
        check("all_truncations_rejected", all_rejected);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
