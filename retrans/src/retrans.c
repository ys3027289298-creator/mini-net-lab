
/* retrans: tick-based NACK retransmission.
 *
 * sender caches recently sent packets in a ring window and re-serves the exact
 * cached bytes on request; receiver detects tick gaps, asks for retransmission,
 * suppresses duplicates and reassembles in order. fill latency is recorded per
 * gap; a cache miss is reported explicitly instead of fabricating state. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define PKT_MAX 48
#define SEND_CACHE_MAX 64
#define REORDER_CAP 32
#define LOG_CAP 64
#define MISS_CAP 16
#define FILL_CAP 16
#define UNREC_CAP 16

/* recv_on_packet return codes below zero */
#define RECV_DUP  -1   /* tick already delivered or pending */
#define RECV_BAD  -2   /* packet failed to decode */
#define RECV_FULL -3   /* reorder buffer exhausted */

typedef struct {
    unsigned tick;
    float x, y, vx, vy;
    unsigned flags;
} State;

typedef struct {
    unsigned char buf[PKT_MAX];
    int len;
} Packet;

/* ---- codec: varint tick/flags, zigzag varint quantized floats ---- */

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

static unsigned zig(int v) {            /* zigzag, unsigned ops only (no signed shift UB) */
    unsigned u = (unsigned)v;
    return (u << 1) ^ (0u - (u >> 31));
}
static int zag(unsigned u) { return (int)((u >> 1) ^ (0u - (u & 1u))); }

static int encode(const State* s, Packet* p, float pos_scale, float vel_scale) {
    if (p == NULL || pos_scale <= 0.0f || vel_scale <= 0.0f) return -1;
    int n = 0;
    n += varint_put(p->buf + n, s->tick);
    n += varint_put(p->buf + n, zig((int)lrintf(s->x * pos_scale)));
    n += varint_put(p->buf + n, zig((int)lrintf(s->y * pos_scale)));
    n += varint_put(p->buf + n, zig((int)lrintf(s->vx * vel_scale)));
    n += varint_put(p->buf + n, zig((int)lrintf(s->vy * vel_scale)));
    n += varint_put(p->buf + n, s->flags);
    p->len = n;
    return n;
}

static int decode(const Packet* p, State* s, float pos_scale, float vel_scale) {
    if (p == NULL || s == NULL || pos_scale <= 0.0f || vel_scale <= 0.0f) return -1;
    unsigned u[6];
    int o = 0;
    for (int i = 0; i < 6; i++) {
        int g = varint_get(p->buf + o, p->len - o, &u[i]);
        if (g < 0) return -1;
        o += g;
    }
    s->tick = u[0];
    s->x = (float)zag(u[1]) / pos_scale;
    s->y = (float)zag(u[2]) / pos_scale;
    s->vx = (float)zag(u[3]) / vel_scale;
    s->vy = (float)zag(u[4]) / vel_scale;
    s->flags = u[5];
    return o;
}

/* ---- sender: ring cache window, exact-byte retransmit ---- */

typedef struct {
    struct { Packet pkt; unsigned tick; int used; } c[SEND_CACHE_MAX];
    int win;       /* configured window, <= SEND_CACHE_MAX */
    int head;      /* next ring slot to overwrite */
    float pos_scale, vel_scale;
    unsigned retrans_sent;   /* successful cache hits served */
    unsigned cache_miss;     /* requests the window could not serve */
    unsigned last_miss;      /* tick of the most recent miss */
} Sender;

static void sender_init(Sender* s, int win, float pos_scale, float vel_scale) {
    memset(s, 0, sizeof(*s));
    s->win = win > SEND_CACHE_MAX ? SEND_CACHE_MAX : win;
    s->pos_scale = pos_scale;
    s->vel_scale = vel_scale;
}

/* encode state for the wire and keep an identical copy in the cache window */
static int sender_send(Sender* s, const State* st, Packet* wire) {
    if (s == NULL || st == NULL || wire == NULL || s->win <= 0) return -1;
    if (encode(st, wire, s->pos_scale, s->vel_scale) < 0) return -1;
    int i = s->head;
    s->c[i].pkt = *wire;
    s->c[i].tick = st->tick;
    s->c[i].used = 1;
    s->head = (s->head + 1) % s->win;
    return 0;
}

/* NACK handling: 0 = retransmitted (out holds cached bytes), -1 = window miss */
static int sender_retrans(Sender* s, unsigned tick, Packet* out) {
    if (s == NULL || out == NULL) return -1;
    for (int i = 0; i < s->win; i++) {
        if (s->c[i].used && s->c[i].tick == tick) {
            *out = s->c[i].pkt;
            s->retrans_sent++;
            return 0;
        }
    }
    s->cache_miss++;
    s->last_miss = tick;
    return -1;
}

/* ---- receiver: gap detection, dedup, in-order reassembly ---- */

typedef struct { State st; unsigned tick; int used; } PSlot;
typedef struct { unsigned tick; unsigned detect_at; } Miss;
typedef struct { unsigned tick; unsigned delay; } Fill;

typedef struct {
    PSlot pend[REORDER_CAP];          /* arrived but blocked by a gap */
    Miss miss[MISS_CAP];              /* outstanding gaps, NACK source */
    int miss_n;
    Fill fills[FILL_CAP];             /* fill latency records (the data) */
    int fill_n;
    unsigned unrec[UNREC_CAP];        /* gaps reported unrecoverable */
    int unrec_n;
    State log[LOG_CAP];               /* states delivered in tick order */
    int log_n;
    unsigned next;                    /* next tick expected */
    unsigned dups;                    /* duplicate packets dropped */
    unsigned bad;
    float pos_scale, vel_scale;
} Receiver;

static void recv_init(Receiver* r, unsigned base_tick, float pos_scale, float vel_scale) {
    memset(r, 0, sizeof(*r));
    r->next = base_tick;
    r->pos_scale = pos_scale;
    r->vel_scale = vel_scale;
}

static int pend_find(const Receiver* r, unsigned tick) {
    for (int i = 0; i < REORDER_CAP; i++)
        if (r->pend[i].used && r->pend[i].tick == tick) return i;
    return -1;
}

static int miss_find(const Receiver* r, unsigned tick) {
    for (int i = 0; i < r->miss_n; i++)
        if (r->miss[i].tick == tick) return i;
    return -1;
}

/* deliver every consecutive tick starting at next straight from the reorder
 * buffer; each drained gap with an outstanding miss is a retrans fill */
static int recv_drain(Receiver* r, unsigned now) {
    int delivered = 0;
    for (;;) {
        int idx = pend_find(r, r->next);
        if (idx < 0) break;
        if (r->log_n < LOG_CAP) r->log[r->log_n++] = r->pend[idx].st;
        r->pend[idx].used = 0;
        int mi = miss_find(r, r->next);
        if (mi >= 0) {                                          /* a retrans fill */
            unsigned delay = now - r->miss[mi].detect_at;
            if (r->fill_n < FILL_CAP)
                r->fills[r->fill_n++] = (Fill){ r->next, delay };
            r->miss[mi] = r->miss[--r->miss_n];
        }
        r->next++;
        delivered++;
    }
    return delivered;
}

/* feed an original or retransmitted packet at simulated time `now` (ticks).
 * returns number of states newly delivered (0 means accepted behind a gap),
 * or RECV_DUP / RECV_BAD / RECV_FULL. */
static int recv_on_packet(Receiver* r, const Packet* p, unsigned now) {
    State st;
    if (decode(p, &st, r->pos_scale, r->vel_scale) < 0) { r->bad++; return RECV_BAD; }
    unsigned t = st.tick;
    if (t < r->next) { r->dups++; return RECV_DUP; }             /* already delivered */
    if (pend_find(r, t) >= 0) { r->dups++; return RECV_DUP; }    /* already pending */

    if (t == r->next) {
        /* in-order packet: deliver directly, no reorder slot required (this is
         * also how a fill gets admitted when the reorder buffer is full) */
        if (r->log_n < LOG_CAP) r->log[r->log_n++] = st;
        int mi = miss_find(r, t);
        if (mi >= 0) {
            unsigned delay = now - r->miss[mi].detect_at;
            if (r->fill_n < FILL_CAP)
                r->fills[r->fill_n++] = (Fill){ t, delay };
            r->miss[mi] = r->miss[--r->miss_n];
        }
        r->next++;
        return 1 + recv_drain(r, now);
    }

    int slot = -1;
    for (int i = 0; i < REORDER_CAP; i++)
        if (!r->pend[i].used) { slot = i; break; }
    if (slot < 0) return RECV_FULL;
    r->pend[slot].used = 1;
    r->pend[slot].tick = t;
    r->pend[slot].st = st;

    /* every tick between next and t is a gap; pending ticks already arrived */
    for (unsigned g = r->next; g < t; g++) {
        if (miss_find(r, g) < 0 && pend_find(r, g) < 0 && r->miss_n < MISS_CAP)
            r->miss[r->miss_n++] = (Miss){ g, now };
    }
    return recv_drain(r, now);
}

/* sender heartbeat announces how far its tick has advanced; gaps with no
 * arriving packet at all (total-loss bursts, stream tail) become detectable
 * without waiting for the next data packet. returns newly detected gaps. */
static int recv_on_heartbeat(Receiver* r, unsigned sender_tick, unsigned now) {
    if (sender_tick <= r->next) return 0;
    int added = 0;
    for (unsigned g = r->next; g < sender_tick; g++) {
        if (miss_find(r, g) < 0 && pend_find(r, g) < 0 && r->miss_n < MISS_CAP) {
            r->miss[r->miss_n++] = (Miss){ g, now };
            added++;
        }
    }
    return added;
}

/* outstanding missing ticks, ready to NACK; filled or already-pending ticks
 * are skipped (miss entries of pending ticks keep detect_at for fill latency) */
static int recv_missing(const Receiver* r, unsigned* out, int cap) {
    int n = 0;
    for (int i = 0; i < r->miss_n && n < cap; i++)
        if (pend_find(r, r->miss[i].tick) < 0) out[n++] = r->miss[i].tick;
    return n;
}

/* sender could not serve the NACK: report the hole, never fabricate the tick.
 * next tick is NOT advanced, so later states stay buffered behind the hole. */
static int recv_nack_failed(Receiver* r, unsigned tick) {
    int mi = miss_find(r, tick);
    if (mi < 0 || r->unrec_n >= UNREC_CAP) return -1;
    r->unrec[r->unrec_n++] = tick;
    r->miss[mi] = r->miss[--r->miss_n];
    return 0;
}

static int recv_has(const Receiver* r, unsigned tick) {
    if (tick < r->next) return 1;                 /* already in the delivered log */
    return pend_find(r, tick) >= 0;
}

static int fill_delay(const Receiver* r, unsigned tick, unsigned* out) {
    for (int i = 0; i < r->fill_n; i++)
        if (r->fills[i].tick == tick) { *out = r->fills[i].delay; return 0; }
    return -1;
}

static int state_equal(const State* a, const State* b) {
    return a->tick == b->tick && a->x == b->x && a->y == b->y &&
           a->vx == b->vx && a->vy == b->vy && a->flags == b->flags;
}

/* ---- test harness ---- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static State make_state(unsigned t) {
    State s;
    s.tick = t;
    s.x = 1.5f * (float)t - 3.25f;
    s.y = -0.75f * (float)t + 10.0f;
    s.vx = 0.5f * (float)(t % 7u) - 1.0f;
    s.vy = -0.25f * (float)(t % 5u);
    s.flags = t & 3u;
    return s;
}

static unsigned rng_state;
static unsigned rndu(void) {                       /* deterministic xorshift32 */
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}

typedef struct { Packet pkt; unsigned at; int used; } Job; /* scheduled retrans */

#define N 12

int main(void) {
    const float ps = 64.0f, vs = 16.0f;

    /* scenario A: no-loss baseline, same codec path */
    Sender base_snd; Receiver base;
    sender_init(&base_snd, 16, ps, vs);
    recv_init(&base, 1, ps, vs);
    for (unsigned t = 1; t <= N; t++) {
        Packet w;
        State st = make_state(t);
        check("base_send", sender_send(&base_snd, &st, &w) == 0);
        check("base_recv", recv_on_packet(&base, &w, t) == 1);
    }
    check("base_count", base.log_n == N);

    /* scenario B: loss + delayed retrans (2-tick RTT) + duplicate retrans +
     * one late original; receiver NACKs every tick while a gap stays open. */
    Sender snd; Receiver rcv;
    sender_init(&snd, 16, ps, vs);
    recv_init(&rcv, 1, ps, vs);
    Job jobs[24]; memset(jobs, 0, sizeof(jobs));
    Packet delayed; int has_delayed = 0;
    unsigned miss[MISS_CAP];

    for (unsigned now = 1; now <= N + 1; now++) {
        /* 1. retransmissions due this tick */
        for (int j = 0; j < (int)(sizeof(jobs) / sizeof(jobs[0])); j++) {
            if (jobs[j].used && jobs[j].at == now) {
                recv_on_packet(&rcv, &jobs[j].pkt, now);
                jobs[j].used = 0;
            }
        }
        /* 2. original stream: ticks 3,7,8 lost; tick 10 delayed to now 12 */
        if (now <= N) {
            State st = make_state(now);
            Packet wire;
            sender_send(&snd, &st, &wire);
            if (now != 3 && now != 7 && now != 8) {
                if (now == 10) { delayed = wire; has_delayed = 1; }
                else recv_on_packet(&rcv, &wire, now);
            }
        }
        if (now == 12 && has_delayed) recv_on_packet(&rcv, &delayed, now);

        /* 3. NACK every outstanding gap; sender serves from cache, RTT = 2 */
        int mn = recv_missing(&rcv, miss, MISS_CAP);
        for (int k = 0; k < mn; k++) {
            Packet rp;
            if (sender_retrans(&snd, miss[k], &rp) == 0) {
                for (int j = 0; j < (int)(sizeof(jobs) / sizeof(jobs[0])); j++) {
                    if (!jobs[j].used) {
                        jobs[j] = (Job){ rp, now + 2, 1 };
                        break;
                    }
                }
            }
        }
    }

    check("lossy_count", rcv.log_n == N);
    int field_eq = 1;
    for (int i = 0; i < N; i++)
        if (!state_equal(&rcv.log[i], &base.log[i])) field_eq = 0;
    check("lossy_field_equal", field_eq);
    int pend_empty = 1;
    for (int i = 0; i < REORDER_CAP; i++) if (rcv.pend[i].used) { pend_empty = 0; break; }
    check("no_pending_left", pend_empty);
    check("missing_drained", recv_missing(&rcv, miss, MISS_CAP) == 0);

    /* gap 3 NACKed at 4,5 -> fills at 6; gaps 7,8 NACKed at 9,10 -> fill 11;
     * gap 10 detected at 11, late original fills it at 12. */
    unsigned d = 0;
    check("fills_recorded", rcv.fill_n == 4);
    check("fill_delay_t3", fill_delay(&rcv, 3, &d) == 0 && d == 2);
    check("fill_delay_t7", fill_delay(&rcv, 7, &d) == 0 && d == 2);
    check("fill_delay_t8", fill_delay(&rcv, 8, &d) == 0 && d == 2);
    check("fill_delay_t10", fill_delay(&rcv, 10, &d) == 0 && d == 1);
    unsigned fill_sum = 0, fill_max = 0;
    for (int i = 0; i < rcv.fill_n; i++) {
        fill_sum += rcv.fills[i].delay;
        if (rcv.fills[i].delay > fill_max) fill_max = rcv.fills[i].delay;
    }
    check("fill_sum_max", fill_sum == 7 && fill_max == 2);
    /* duplicate retrans copies (3@7, 7@12, 8@12) and retrans 10 after the
     * late original (10@13): all suppressed, delivered exactly once */
    check("dup_suppressed", rcv.dups == 4);
    check("sender_retrans_count", snd.retrans_sent == 7);
    check("sender_no_miss", snd.cache_miss == 0);
    check("no_unrecovered", rcv.unrec_n == 0);

    /* scenario C: cache window too small -> report, do not pretend */
    Sender s2; Receiver r2;
    sender_init(&s2, 4, ps, vs);              /* window keeps only ticks 7..10 */
    recv_init(&r2, 1, ps, vs);
    for (unsigned t = 1; t <= 10; t++) {
        Packet w;
        State st = make_state(t);
        sender_send(&s2, &st, &w);
        if (t != 5) recv_on_packet(&r2, &w, t);   /* tick 5 lost */
    }
    Packet rp;
    check("window_miss", sender_retrans(&s2, 5, &rp) == -1);
    check("miss_counted", s2.cache_miss == 1 && s2.last_miss == 5);
    check("nack_failed_reported", recv_nack_failed(&r2, 5) == 0);
    check("unrec_listed", r2.unrec_n == 1 && r2.unrec[0] == 5);
    check("hole_not_fabricated", !recv_has(&r2, 5) && r2.next == 5);
    check("delivered_stops_at_hole", r2.log_n == 4 && r2.log[3].tick == 4);
    State want10 = make_state(10);
    int later_buffered = pend_find(&r2, 10) >= 0 &&
                         state_equal(&r2.pend[pend_find(&r2, 10)].st, &want10);
    check("later_states_buffered", later_buffered);
    check("in_window_retrans_ok", sender_retrans(&s2, 7, &rp) == 0);
    State got;
    State want7 = make_state(7);
    check("in_window_bytes_exact", decode(&rp, &got, ps, vs) >= 0 &&
                                   state_equal(&got, &want7));
    check("never_sent_miss", sender_retrans(&s2, 99, &rp) == -1 &&
                             s2.cache_miss == 2);
    check("double_report_rejected", recv_nack_failed(&r2, 5) == -1);

    /* scenario D: 200 random loss patterns. every retransmitted packet must be
     * byte-identical to the original; reassembled streams must match the
     * no-loss baseline tick-for-tick and field-for-field. */
    rng_state = 0x12345678u;
    int stress_ok = 1, byte_ok = 1, latency_ok = 1;
    unsigned total_fills = 0, max_delay_seen = 0;
    for (int run = 0; run < 200 && stress_ok; run++) {
        Sender ss; Receiver sr;
        sender_init(&ss, 64, ps, vs);
        recv_init(&sr, 1, ps, vs);
        Job sj[256]; memset(sj, 0, sizeof(sj));
        unsigned lost[40]; int nlost = 0;
        for (unsigned t = 1; t <= 30; t++)
            if ((rndu() % 100u) < 20u && nlost < 40) lost[nlost++] = t;
        unsigned smiss[MISS_CAP];
        for (unsigned now = 1; now <= 60; now++) {
            /* heartbeat carries the sender's current tick even when every data
             * packet of the window was lost (sender tick stops at 31 = end+1) */
            recv_on_heartbeat(&sr, now <= 31 ? now : 31u, now);
            for (int j = 0; j < (int)(sizeof(sj) / sizeof(sj[0])); j++)
                if (sj[j].used && sj[j].at == now) {
                    recv_on_packet(&sr, &sj[j].pkt, now);
                    sj[j].used = 0;
                }
            if (now <= 30) {
                Packet w;
                State st = make_state(now);
                sender_send(&ss, &st, &w);
                int is_lost = 0;
                for (int k = 0; k < nlost; k++) if (lost[k] == now) is_lost = 1;
                if (!is_lost) recv_on_packet(&sr, &w, now);
            }
            int mn = recv_missing(&sr, smiss, MISS_CAP);
            for (int k = 0; k < mn; k++) {
                Packet rp, orig;
                if (sender_retrans(&ss, smiss[k], &rp) != 0) { stress_ok = 0; break; }
                State orig_st = make_state(smiss[k]);
                encode(&orig_st, &orig, ps, vs);
                if (rp.len != orig.len || memcmp(rp.buf, orig.buf, rp.len) != 0)
                    byte_ok = 0;                       /* cached bytes == fresh encode */
                for (int j = 0; j < (int)(sizeof(sj) / sizeof(sj[0])); j++)
                    if (!sj[j].used) { sj[j] = (Job){ rp, now + 1 + rndu() % 3, 1 }; break; }
            }
        }
        if (sr.log_n != 30) { stress_ok = 0; break; }
        for (unsigned t = 1; t <= 30; t++) {
            State want = make_state(t);
            if (!state_equal(&sr.log[t - 1], &want)) stress_ok = 0;
        }
        for (int i = 0; i < sr.fill_n; i++) {
            if (sr.fills[i].delay == 0u) latency_ok = 0;      /* fill is never instantaneous */
            total_fills++;
            if (sr.fills[i].delay > max_delay_seen) max_delay_seen = sr.fills[i].delay;
        }
        if (recv_missing(&sr, smiss, MISS_CAP) != 0) stress_ok = 0;
    }
    check("stress_reassembled", stress_ok);
    check("stress_bytes_identical", byte_ok);
    check("stress_latency_data", latency_ok && total_fills > 0 && max_delay_seen >= 2);

    /* unit: immediate duplicate, bad packet, full reorder buffer, bad args */
    Receiver u;
    recv_init(&u, 1, ps, vs);
    Packet one;
    encode(&(State){ 1, 0, 0, 0, 0, 0 }, &one, ps, vs);
    check("first_accepted", recv_on_packet(&u, &one, 1) == 1);
    check("dup_rejected", recv_on_packet(&u, &one, 2) == RECV_DUP && u.dups == 1);
    Packet junk = { .len = 3 };
    junk.buf[0] = 0xff; junk.buf[1] = 0xff; junk.buf[2] = 0x00;
    check("bad_packet", recv_on_packet(&u, &junk, 2) == RECV_BAD && u.bad == 1);

    Receiver full;
    recv_init(&full, 1, ps, vs);
    int full_seen = 0, full_tick = 0;
    for (unsigned t = 2; ; t++) {
        Packet w;
        State st = { t, 0, 0, 0, 0, 0 };
        encode(&st, &w, ps, vs);
        int rc = recv_on_packet(&full, &w, t);
        if (rc == RECV_FULL) { full_seen = 1; full_tick = (int)t; break; }
        if (t > REORDER_CAP + 10) break;
    }
    check("reorder_full_reports", full_seen);
    /* full buffer still dedups an already-pending tick instead of hiding it */
    Packet again;
    State st2 = { 2, 0, 0, 0, 0, 0 };
    encode(&st2, &again, ps, vs);
    check("full_still_dedups", recv_on_packet(&full, &again, 99) == RECV_DUP &&
                               full.dups == 1);
    /* a genuinely new tick is rejected rather than evicting buffered state */
    Packet novel;
    State st3 = { (unsigned)(full_tick + 1), 0, 0, 0, 0, 0 };
    encode(&st3, &novel, ps, vs);
    check("full_rejects_new", recv_on_packet(&full, &novel, 99) == RECV_FULL);
    Packet hole;
    encode(&(State){ 1, 0, 0, 0, 0, 0 }, &hole, ps, vs);
    check("full_recovers_after_fill", recv_on_packet(&full, &hole, 100) == REORDER_CAP + 1 &&
                                      full.log_n == REORDER_CAP + 1);

    Sender bad;
    sender_init(&bad, 0, ps, vs);
    check("zero_window_rejects", sender_send(&bad, &(State){0}, &one) == -1);

    /* heartbeat gap detection: tail loss with no later data packet */
    Receiver hb;
    recv_init(&hb, 1, ps, vs);
    Packet hp;
    State h1 = { 1, 0, 0, 0, 0, 0 }, h2 = { 2, 0, 0, 0, 0, 0 };
    encode(&h1, &hp, ps, vs); recv_on_packet(&hb, &hp, 1);
    check("hb_no_gap_yet", recv_on_heartbeat(&hb, 2, 2) == 0);
    check("hb_detects_tail", recv_on_heartbeat(&hb, 4, 4) == 2);  /* ticks 2,3 */
    check("hb_idempotent", recv_on_heartbeat(&hb, 4, 5) == 0 &&
                           recv_missing(&hb, miss, MISS_CAP) == 2);
    check("hb_old_announce_noop", recv_on_heartbeat(&hb, 1, 6) == 0);
    Packet r3;
    Sender hs;
    sender_init(&hs, 8, ps, vs);
    sender_send(&hs, &h2, &r3);
    State h3 = { 3, 0, 0, 0, 0, 0 }; Packet r3b;
    sender_send(&hs, &h3, &r3b);
    check("hb_tail_fill_2", recv_on_packet(&hb, &r3, 5) == 1);
    check("hb_tail_fill_3", recv_on_packet(&hb, &r3b, 6) == 1 && hb.log_n == 3);

    /* total-loss burst: no data packets at all, heartbeat alone opens the gaps */
    Receiver hb2;
    recv_init(&hb2, 10, ps, vs);
    check("hb_burst_gaps", recv_on_heartbeat(&hb2, 14, 20) == 4);
    check("hb_burst_list", recv_missing(&hb2, miss, MISS_CAP) == 4 &&
                           miss[0] == 10 && miss[3] == 13);

    /* heartbeat must not NACK a tick whose packet is already pending */
    Receiver hb3;
    recv_init(&hb3, 1, ps, vs);
    Packet p5; State s5 = { 5, 1, 2, 3, 4, 5 };
    encode(&s5, &p5, ps, vs);
    recv_on_packet(&hb3, &p5, 5);                    /* ticks 1..4 gaps, 5 pending */
    check("hb_skips_pending", recv_on_heartbeat(&hb3, 6, 5) == 0 &&
                              recv_missing(&hb3, miss, MISS_CAP) == 4);
    int lists_gaps_only = 1;
    for (int k = 0; k < 4; k++) if (miss[k] < 1 || miss[k] > 4) lists_gaps_only = 0;
    check("hb_missing_excludes_pending", lists_gaps_only);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
