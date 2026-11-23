/* verify: weak-network action-consistency harness.
 *
 * An authoritative actor runs a deterministic action script; a client-side
 * remote avatar tracks it through a lossy link at several packet-loss tiers.
 * Optional degradation: when the client's sliding-window loss estimate
 * exceeds ENGAGE_PCT, the client freezes prediction (holds last pose) and
 * the server drops to 1/DEGRADE_DIV send rate; it releases below
 * RELEASE_PCT (hysteresis). Every toggle is logged.
 *
 * Four independent pieces of evidence are written to the output dir:
 *   phase_table.csv            action-phase diff per loss tier, pre/post degrade
 *   degrade_log.txt            degradation on/off toggle events
 *   packet_hashes.txt          fnv1a-64 of every packet body sent
 *   action_trace_off_tier*.txt per-frame action trace, degradation off
 * plus repro.sh (in this directory) which rebuilds and regenerates all of it.
 *
 * With degradation off the action trace must hash-match golden.txt, i.e.
 * frame-identical to current behavior.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#define TICKS        600
#define WIN          16   /* loss-estimate window, in send opportunities */
#define ENGAGE_PCT   15   /* engage degradation above this window loss % */
#define RELEASE_PCT  5    /* release below this % (hysteresis) */
#define DEGRADE_DIV  4    /* degraded send rate: 1 packet per N ticks */
#define NTIERS       5

#define FNV_OFF 1469598103934665603ULL

typedef struct { unsigned long long s; } Rng;

static unsigned rng_u32(Rng* r) {       /* xorshift64, deterministic */
    r->s ^= r->s << 13; r->s ^= r->s >> 7; r->s ^= r->s << 17;
    return (unsigned)(r->s >> 32);
}

static unsigned long long fnv_update(unsigned long long h, const void* p, int n) {
    const unsigned char* b = (const unsigned char*)p;
    for (int i = 0; i < n; i++) { h ^= b[i]; h *= 1099511628211ULL; }
    return h;
}

static int varint_put(unsigned char* b, unsigned v) {
    int n = 0;
    while (v >= 0x80) { b[n++] = (unsigned char)((v & 0x7f) | 0x80); v >>= 7; }
    b[n++] = (unsigned char)(v & 0x7f);
    return n;
}

/* authoritative action script: run 60, jump 10, turn 40, idle 10 ticks */
static int auth_action(unsigned tick) {
    unsigned c = tick % 120u;
    if (c < 60u) return 0;
    if (c < 70u) return 1;
    if (c < 110u) return 2;
    return 3;
}

static unsigned long long tier_seed(unsigned tier) {
    return 0x9e3779b97f4a7c15ULL ^ (unsigned long long)(tier + 1) * 0x100000001b3ULL;
}

typedef struct {
    double sum;
    unsigned long max;
    unsigned long n;
    unsigned long rollbacks;
} Stats;

typedef struct {
    Stats pre, post;            /* post = after first degradation engage */
    int ever_engaged;
    unsigned long packets;
    unsigned long long trace_hash;   /* fnv1a over per-frame trace lines */
    unsigned long long stream_hash;  /* fnv1a over all packet bodies */
} Run;

static void run_sim(unsigned tier, int degrade_on,
                    FILE* pkt, FILE* dlog, FILE* trace, Run* r) {
    memset(r, 0, sizeof(*r));
    r->trace_hash = FNV_OFF;
    r->stream_hash = FNV_OFF;
    Rng rng = { tier_seed(tier) };      /* same seed per tier: off/on see identical loss */
    int auth_prev = auth_action(0);
    long auth_phase = 0;
    int have = 0, cli_act = -1, engaged = 0;
    long cli_phase = 0;
    int win[WIN]; int win_n = 0, win_pos = 0, win_sum = 0;
    memset(win, 0, sizeof(win));

    for (unsigned tick = 0; tick < TICKS; tick++) {
        int act = auth_action(tick);
        if (act != auth_prev) { auth_phase = 0; auth_prev = act; }
        else if (tick > 0) auth_phase++;

        int send_tick = engaged ? (tick % DEGRADE_DIV == 0) : 1;
        int received = 0;
        if (send_tick) {
            unsigned char body[16];
            int blen = varint_put(body, tick);
            blen += varint_put(body + blen, (unsigned)act);
            blen += varint_put(body + blen, (unsigned)auth_phase);
            received = (int)(rng_u32(&rng) % 100u) >= (int)tier;
            r->stream_hash = fnv_update(r->stream_hash, body, blen);
            if (pkt) fprintf(pkt, "tier=%u mode=%s seq=%lu tick=%u len=%d hash=%016llx\n",
                             tier, degrade_on ? "on" : "off", r->packets, tick, blen,
                             fnv_update(FNV_OFF, body, blen));
            r->packets++;
            win_sum -= win[win_pos];
            win[win_pos] = received;
            win_sum += received;
            win_pos = (win_pos + 1) % WIN;
            if (win_n < WIN) win_n++;
        }

        /* predict first: normal mode extrapolates every tick, degraded freezes */
        if (have && !engaged) cli_phase++;

        if (received) {
            Stats* s = (degrade_on && r->ever_engaged) ? &r->post : &r->pre;
            if (have && (cli_act != act || cli_phase != auth_phase)) s->rollbacks++;
            cli_act = act; cli_phase = auth_phase; have = 1;
        }

        if (degrade_on && send_tick && win_n == WIN) {
            int loss_pct = (WIN - win_sum) * 100 / WIN;
            if (!engaged && loss_pct > ENGAGE_PCT) {
                engaged = 1; r->ever_engaged = 1;
                if (dlog) fprintf(dlog, "tier=%u tick=%u window_loss_pct=%d degrade=on\n",
                                  tier, tick, loss_pct);
            } else if (engaged && loss_pct < RELEASE_PCT) {
                engaged = 0;
                if (dlog) fprintf(dlog, "tier=%u tick=%u window_loss_pct=%d degrade=off\n",
                                  tier, tick, loss_pct);
            }
        }

        if (have) {
            long diff = cli_phase - auth_phase;
            if (diff < 0) diff = -diff;
            Stats* s = (degrade_on && r->ever_engaged) ? &r->post : &r->pre;
            s->sum += (double)diff; s->n++;
            if (s->max < (unsigned long)diff) s->max = (unsigned long)diff;
        }

        char line[64];
        int ln = snprintf(line, sizeof(line), "%u:%d:%ld:%d:%ld\n",
                          tick, act, auth_phase, have ? cli_act : -1, have ? cli_phase : -1L);
        r->trace_hash = fnv_update(r->trace_hash, line, ln);
        if (trace) fwrite(line, 1, (size_t)ln, trace);
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void phase_row(FILE* f, unsigned tier, const char* mode, const char* seg,
                      const Stats* s, unsigned long packets) {
    fprintf(f, "%u,%s,%s,%.3f,%lu,%lu,%lu,%lu\n", tier, mode, seg,
            s->n ? s->sum / (double)s->n : -1.0, s->max, s->n, s->rollbacks, packets);
}

int main(int argc, char** argv) {
    const char* outdir = argc > 1 ? argv[1] : "out";
    const char* golden = argc > 2 ? argv[2] : NULL;
    mkdir(outdir, 0755);

    char path[256];
    snprintf(path, sizeof path, "%s/phase_table.csv", outdir);
    FILE* phase = fopen(path, "w");
    snprintf(path, sizeof path, "%s/degrade_log.txt", outdir);
    FILE* dlog = fopen(path, "w");
    snprintf(path, sizeof path, "%s/packet_hashes.txt", outdir);
    FILE* pkt = fopen(path, "w");
    if (!phase || !dlog || !pkt) { fprintf(stderr, "cannot open output files\n"); return 2; }
    fprintf(phase, "loss_pct,mode,segment,mean_phase_diff_ticks,max_phase_diff_ticks,samples,rollbacks,packets_sent\n");

    static const unsigned tiers[NTIERS] = { 0, 5, 10, 20, 30 };
    unsigned long long trace_off[NTIERS], trace_on_tier0 = 0;

    for (int i = 0; i < NTIERS; i++) {
        for (int mode = 0; mode <= 1; mode++) {
            FILE* trace = NULL;
            if (mode == 0) {
                snprintf(path, sizeof path, "%s/action_trace_off_tier%02u.txt", outdir, tiers[i]);
                trace = fopen(path, "w");
            }
            Run r;
            run_sim(tiers[i], mode, pkt, dlog, trace, &r);
            if (trace) fclose(trace);
            if (mode == 0) {
                trace_off[i] = r.trace_hash;
                phase_row(phase, tiers[i], "off", "all", &r.pre, r.packets);
            } else {
                if (tiers[i] == 0) trace_on_tier0 = r.trace_hash;
                phase_row(phase, tiers[i], "on", "pre", &r.pre, r.packets);
                phase_row(phase, tiers[i], "on", "post", &r.post, r.packets);
            }
        }
    }
    fclose(phase); fclose(dlog); fclose(pkt);

    /* degradation machinery must not alter behavior when never engaged */
    check("frame_identical_tier0", trace_on_tier0 == trace_off[0]);

    Run a, b;
    run_sim(10, 1, NULL, NULL, NULL, &a);
    run_sim(10, 1, NULL, NULL, NULL, &b);
    check("deterministic", a.trace_hash == b.trace_hash && a.stream_hash == b.stream_hash);

    /* degrade-off traces must be frame-identical to recorded current behavior */
    if (golden) {
        FILE* g = fopen(golden, "r");
        if (!g) {
            g = fopen(golden, "w");
            if (!g) { fprintf(stderr, "cannot write golden file\n"); return 2; }
            for (int i = 0; i < NTIERS; i++)
                fprintf(g, "tier=%u fnv1a=%016llx\n", tiers[i], trace_off[i]);
            fclose(g);
            printf("golden_written=1\n");
        } else {
            int ok = 1, seen = 0;
            unsigned t; unsigned long long h;
            while (fscanf(g, " tier=%u fnv1a=%llx", &t, &h) == 2) {
                seen++;
                int found = 0;
                for (int i = 0; i < NTIERS; i++)
                    if (tiers[i] == t) { found = 1; if (trace_off[i] != h) ok = 0; }
                if (!found) ok = 0;
            }
            fclose(g);
            check("golden_match", ok && seen == NTIERS);
        }
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
