/* resync: item-state incremental resync after reconnect, with full-snapshot fallback. */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define ITEM_COUNT        1024
#define JOURNAL_CAP       40000            /* update records kept for replay */
#define UPDATE_REC_PER_S  128              /* 1024 items x 0.125 field updates/s */
#define RETENTION_S       300              /* journal spans about this long */
#define FULL_HEADER_BYTES 16
#define FULL_ITEM_BYTES   64               /* static + dynamic fields per item */
#define LOBBY_LINK_BPS    65536.0          /* downlink budget, bytes/s */
#define PATCH_CAP         (JOURNAL_CAP * 12 + 64)

typedef struct { unsigned seq; unsigned item; unsigned mask; unsigned state; } Rec;

typedef struct {
    Rec r[JOURNAL_CAP];
    int head;          /* next write */
    int count;
    unsigned next_seq;
} Journal;

/* full-snapshot record: what one item costs in a complete lobby dump */
typedef struct {
    unsigned short id;
    unsigned short icon;
    unsigned owner;
    unsigned price;
    short x, y;
    unsigned short rot;
    unsigned short count;
    unsigned state;
    unsigned char flags;
    unsigned char reserved;
    unsigned char pad[38];
} FullItem;

_Static_assert(sizeof(FullItem) == FULL_ITEM_BYTES, "full item record must stay 64 bytes");

typedef struct {
    Journal j;
    unsigned items[ITEM_COUNT];
    unsigned client[ITEM_COUNT];
    unsigned client_seq;
    unsigned t;
} World;

typedef struct { double max_seconds; int max_bytes; } Policy;

typedef struct {
    double outage_s;
    int delta_bytes;
    int full_bytes;
    int sent_bytes;
    double span_s;
    int stale_before;
    int stale_after;
    int applied;
    double apply_ms;
    double transfer_ms;
    double recovery_ms;
    int is_full;
    const char* reason;
} Tier;

static World g_world;

static void journal_init(Journal* j) { memset(j, 0, sizeof(*j)); j->next_seq = 1; }

static unsigned journal_oldest(const Journal* j) {
    return j->count == 0 ? j->next_seq : j->next_seq - (unsigned)j->count;
}

static unsigned journal_append(Journal* j, unsigned item, unsigned mask, unsigned state) {
    Rec* r = &j->r[j->head];
    r->seq = j->next_seq++;
    r->item = item;
    r->mask = mask;
    r->state = state;
    j->head = (j->head + 1) % JOURNAL_CAP;
    if (j->count < JOURNAL_CAP) j->count++;
    return r->seq;
}

static const Rec* journal_at(const Journal* j, int i) {
    int idx;
    if (i < 0 || i >= j->count) return NULL;
    idx = (j->head - j->count + i + 2 * JOURNAL_CAP) % JOURNAL_CAP;
    return &j->r[idx];
}

static int put_varint(unsigned char* buf, int cap, int* n, unsigned v) {
    unsigned char tmp[5];
    int k = 0;
    while (v >= 0x80) { tmp[k++] = (unsigned char)((v & 0x7f) | 0x80); v >>= 7; }
    tmp[k++] = (unsigned char)(v & 0x7f);
    if (*n + k > cap) return -1;
    memcpy(buf + *n, tmp, (size_t)k);
    *n += k;
    return 0;
}

static int get_varint(const unsigned char* buf, int len, int* off, unsigned* out) {
    unsigned v = 0;
    int shift = 0;
    while (*off < len && shift <= 28) {
        unsigned char c = buf[(*off)++];
        v |= (unsigned)(c & 0x7f) << shift;
        if (!(c & 0x80)) { *out = v; return 0; }
        shift += 7;
    }
    return -1;
}

/* Replay every journal record after last_seq. Returns encoded size, 0 if nothing
   is pending, -1 if the journal no longer reaches back far enough. */
static int build_delta(const Journal* j, unsigned last_seq, unsigned char* buf, int cap) {
    unsigned newest = j->next_seq - 1;
    if (newest < last_seq) return -1;
    if (newest == last_seq) return 0;
    if (j->count == 0) return -1;
    {
        unsigned oldest = journal_oldest(j);
        int first, nrec, n = 0;
        if (last_seq + 1 < oldest) return -1;
        first = (int)(last_seq + 1 - oldest);
        nrec = (int)(newest - last_seq);
        if (put_varint(buf, cap, &n, 0) < 0) return -1;             /* missing prefix */
        if (put_varint(buf, cap, &n, (unsigned)nrec) < 0) return -1;
        {
            unsigned prev = last_seq;
            for (int i = 0; i < nrec; i++) {
                const Rec* r = journal_at(j, first + i);
                if (r == NULL) return -1;
                if (put_varint(buf, cap, &n, r->seq - prev) < 0) return -1;
                if (put_varint(buf, cap, &n, r->item) < 0) return -1;
                if (n >= cap) return -1;
                buf[n++] = (unsigned char)r->mask;
                if (put_varint(buf, cap, &n, r->state) < 0) return -1;
                prev = r->seq;
            }
        }
        return n;
    }
}

static int apply_delta(const unsigned char* buf, int len, unsigned* items, unsigned* last_seq) {
    int off = 0;
    unsigned missing = 0, nrec = 0, d, item, state;
    unsigned prev;
    if (get_varint(buf, len, &off, &missing) < 0) return -1;
    if (get_varint(buf, len, &off, &nrec) < 0) return -1;
    if (missing != 0) return -2;
    prev = *last_seq;
    for (unsigned i = 0; i < nrec; i++) {
        if (get_varint(buf, len, &off, &d) < 0) return -1;
        if (get_varint(buf, len, &off, &item) < 0) return -1;
        if (off >= len) return -1;
        off++;                                              /* field mask */
        if (get_varint(buf, len, &off, &state) < 0) return -1;
        if (item >= ITEM_COUNT) return -1;
        prev += d;
        items[item] = state;
    }
    *last_seq = prev;
    return (int)nrec;
}

static int full_snapshot_bytes(void) {
    return FULL_HEADER_BYTES + ITEM_COUNT * (int)sizeof(FullItem);
}

static int build_full(unsigned seq, const unsigned* items, unsigned char* buf, int cap) {
    int need = full_snapshot_bytes();
    int n = 0;
    if (cap < need) return -1;
    if (put_varint(buf, cap, &n, seq) < 0) return -1;
    if (put_varint(buf, cap, &n, ITEM_COUNT) < 0) return -1;
    memset(buf + n, 0, (size_t)(FULL_HEADER_BYTES - n));
    n = FULL_HEADER_BYTES;
    for (int i = 0; i < ITEM_COUNT; i++) {
        FullItem fi;
        memset(&fi, 0, sizeof(fi));
        fi.id = (unsigned short)i;
        fi.icon = (unsigned short)(i * 3u);
        fi.owner = (unsigned)i * 2654435761u;
        fi.price = 100u + (unsigned)i;
        fi.x = (short)(i % 256);
        fi.y = (short)(i / 256);
        fi.rot = (unsigned short)(i * 11u);
        fi.count = (unsigned short)(i % 97u);
        fi.state = items[i];
        fi.flags = (unsigned char)(i & 1);
        memcpy(buf + n, &fi, sizeof(fi));
        n += (int)sizeof(fi);
    }
    return n;
}

static int apply_full(const unsigned char* buf, int len, unsigned* items, unsigned* last_seq) {
    int off = 0, p;
    unsigned seq = 0, n = 0;
    if (len < FULL_HEADER_BYTES) return -1;
    if (get_varint(buf, len, &off, &seq) < 0) return -1;
    if (get_varint(buf, len, &off, &n) < 0) return -1;
    if (n != ITEM_COUNT) return -1;
    if (len < full_snapshot_bytes()) return -1;
    p = FULL_HEADER_BYTES;
    for (unsigned i = 0; i < n; i++) {
        FullItem fi;
        memcpy(&fi, buf + p, sizeof(fi));
        p += (int)sizeof(fi);
        if (fi.id >= ITEM_COUNT) return -1;
        items[fi.id] = fi.state;
    }
    *last_seq = seq;
    return (int)n;
}

static void world_setup(World* w) {
    memset(w, 0, sizeof(*w));
    journal_init(&w->j);
    for (int i = 0; i < ITEM_COUNT; i++)
        w->items[i] = (unsigned)i * 2654435761u ^ 0x5a5a5a5au;
}

/* deterministic churn: UPDATE_REC_PER_S item updates per simulated second */
static void simulate(World* w, int seconds) {
    for (int s = 0; s < seconds; s++) {
        w->t++;
        for (int k = 0; k < UPDATE_REC_PER_S; k++) {
            unsigned item = (w->t * 97u + (unsigned)k * 37u) % (unsigned)ITEM_COUNT;
            unsigned state = (w->t * 2654435761u) ^ ((unsigned)k * 40503u) ^ (item * 2246822519u);
            journal_append(&w->j, item, 1u << (k & 7), state);
            w->items[item] = state;
        }
    }
}

static int count_diff(const unsigned* a, const unsigned* b) {
    int n = 0;
    for (int i = 0; i < ITEM_COUNT; i++) if (a[i] != b[i]) n++;
    return n;
}

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

/* measured cost of one second of journal replay, from the encoding itself */
static double measure_delta_bps(void) {
    static unsigned char buf[PATCH_CAP];
    int len;
    world_setup(&g_world);
    simulate(&g_world, 10);
    len = build_delta(&g_world.j, 0, buf, PATCH_CAP);
    return len > 0 ? (double)len / 10.0 : 0.0;
}

static void run_tier(int outage_s, const Policy* pol, Tier* r) {
    static unsigned char delta[PATCH_CAP];
    static unsigned char full[FULL_HEADER_BYTES + ITEM_COUNT * FULL_ITEM_BYTES];
    World* w = &g_world;
    unsigned server_seq, seq;
    int full_path = 0;
    const char* why = "incremental";
    double t0, t1;

    memset(r, 0, sizeof(*r));
    world_setup(w);
    simulate(w, 2);                                  /* state both sides agree on */
    memcpy(w->client, w->items, sizeof(w->client));
    w->client_seq = w->j.next_seq - 1;

    simulate(w, outage_s);
    server_seq = w->j.next_seq - 1;

    r->outage_s = (double)outage_s;
    r->full_bytes = build_full(server_seq, w->items, full, (int)sizeof(full));
    r->delta_bytes = build_delta(&w->j, w->client_seq, delta, PATCH_CAP);
    r->span_s = (double)(server_seq - w->client_seq) / UPDATE_REC_PER_S;

    if (r->delta_bytes < 0) { full_path = 1; why = "gap"; }
    else if (r->span_s > pol->max_seconds) { full_path = 1; why = "cap_seconds"; }
    else if (r->delta_bytes > pol->max_bytes) { full_path = 1; why = "cap_bytes"; }
    r->is_full = full_path;
    r->reason = why;
    r->sent_bytes = full_path ? r->full_bytes : r->delta_bytes;

    r->stale_before = count_diff(w->client, w->items);
    seq = w->client_seq;
    t0 = now_s();
    if (full_path) r->applied = apply_full(full, r->full_bytes, w->client, &seq);
    else r->applied = apply_delta(delta, r->delta_bytes, w->client, &seq);
    t1 = now_s();
    r->apply_ms = (t1 - t0) * 1000.0;
    r->stale_after = count_diff(w->client, w->items);
    r->transfer_ms = (double)r->sent_bytes / LOBBY_LINK_BPS * 1000.0;
    r->recovery_ms = r->transfer_ms + r->apply_ms;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* The switch point: how long a delta may get before a full dump is cheaper.
   Inputs are the item update rate and the measured encoded size of one update
   record; the budget is one full lobby snapshot. */
static Policy policy_derive(double delta_bps) {
    double rec_bytes = delta_bps / UPDATE_REC_PER_S;
    double full = (double)full_snapshot_bytes();
    Policy p;
    p.max_bytes = full_snapshot_bytes();
    p.max_seconds = floor(full / (UPDATE_REC_PER_S * rec_bytes));
    return p;
}

int main(void) {
    const int full_bytes = full_snapshot_bytes();
    double delta_bps = measure_delta_bps();
    double avg_rec = delta_bps / UPDATE_REC_PER_S;
    Policy loose = { RETENTION_S, 4 * 1024 * 1024 };  /* what shipped: retention only */
    Policy derived = policy_derive(delta_bps);        /* switch where delta == full */
    int cap_s = (int)derived.max_seconds;
    Tier t1s, t30s, t180s;
    Tier d1s, d30s, d180s;
    int i;

    run_tier(1, &loose, &t1s);
    run_tier(30, &loose, &t30s);
    run_tier(180, &loose, &t180s);
    run_tier(1, &derived, &d1s);
    run_tier(30, &derived, &d30s);
    run_tier(180, &derived, &d180s);

    printf("--- baseline: cap_seconds=%.0f cap_bytes=%d ---\n", loose.max_seconds, loose.max_bytes);
    printf("outage_s=1 delta_bytes=%d recovery_ms=%.1f stale_before=%d stale_after=%d\n",
           t1s.delta_bytes, t1s.recovery_ms, t1s.stale_before, t1s.stale_after);
    printf("outage_s=30 delta_bytes=%d recovery_ms=%.1f stale_before=%d stale_after=%d\n",
           t30s.delta_bytes, t30s.recovery_ms, t30s.stale_before, t30s.stale_after);
    printf("outage_s=180 delta_bytes=%d recovery_ms=%.1f stale_before=%d stale_after=%d\n",
           t180s.delta_bytes, t180s.recovery_ms, t180s.stale_before, t180s.stale_after);
    printf("full_bytes=%d full_recovery_ms=%.1f\n", full_bytes, t180s.full_bytes / LOBBY_LINK_BPS * 1000.0);

    printf("--- cap derivation ---\n");
    printf("update_rec_per_s=%d avg_delta_record_bytes=%.2f delta_bps=%.1f full_bytes=%d cap_derived_s=%d\n",
           UPDATE_REC_PER_S, avg_rec, delta_bps, full_bytes, cap_s);

    printf("--- delta vs full ---\n");
    {
        Tier* tt[3]; int idx[3] = { 1, 30, 180 };
        tt[0] = &t1s; tt[1] = &t30s; tt[2] = &t180s;
        for (i = 0; i < 3; i++)
            printf("tier=%ds delta_bytes=%d full_bytes=%d ratio=%.3f over_cap=%d\n",
                   idx[i], tt[i]->delta_bytes, full_bytes,
                   (double)tt[i]->delta_bytes / (double)full_bytes,
                   tt[i]->span_s > (double)cap_s);
    }
    printf("diagnosis=%s\n", t180s.delta_bytes > full_bytes ? "cap_too_high" : "cap_ok");
    printf("should_use_full=180s\n");

    printf("--- with derived cap ---\n");
    printf("outage_s=1 path=%s sent_bytes=%d recovery_ms=%.1f\n",
           d1s.reason, d1s.sent_bytes, d1s.recovery_ms);
    printf("outage_s=30 path=%s sent_bytes=%d recovery_ms=%.1f\n",
           d30s.reason, d30s.sent_bytes, d30s.recovery_ms);
    printf("outage_s=180 path=%s sent_bytes=%d recovery_ms=%.1f\n",
           d180s.reason, d180s.sent_bytes, d180s.recovery_ms);

    check("full_snapshot_size", full_bytes == FULL_HEADER_BYTES + ITEM_COUNT * FULL_ITEM_BYTES);
    check("delta_grows_with_outage",
          t1s.delta_bytes < t30s.delta_bytes && t30s.delta_bytes < t180s.delta_bytes);
    check("items_stale_without_item_resync", t180s.stale_before > 0);
    check("resync_converges_1s", t1s.stale_after == 0 && t1s.applied > 0);
    check("resync_converges_30s", t30s.stale_after == 0 && t30s.applied > 0);
    check("resync_converges_180s", t180s.stale_after == 0 && t180s.applied > 0);
    check("three_min_delta_exceeds_full", t180s.delta_bytes > full_bytes);
    check("cap_between_30s_and_180s", cap_s > 30 && cap_s < 180);
    check("derived_cap_keeps_30s_incremental",
          d30s.is_full == 0 && d30s.sent_bytes == d30s.delta_bytes);
    check("derived_cap_moves_180s_to_full",
          d180s.is_full == 1 && d180s.sent_bytes == full_bytes && d180s.stale_after == 0);
    check("derived_cap_faster_at_180s", d180s.recovery_ms < t180s.recovery_ms);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
