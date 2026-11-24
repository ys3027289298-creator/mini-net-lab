
/* streamload: chunked streaming map commit with rate-driven placeholder LOD.
 *
 * A map is split into CHUNKS tiles. At most MAX_INFLIGHT transfer streams are
 * open at once (connection cap) and each frame the network pump shares the
 * link bandwidth across them; in stream mode a chunk is committed the moment
 * its bytes finish, in atomic mode nothing is committed until the whole map
 * has arrived (the old whole-map behaviour).
 *
 * Pending chunks are shown as placeholders. The placeholder level is derived
 * from the measured delivery rate (EWMA), not from the nominal link speed.
 * Bandwidth is shared between active chunks, so per-chunk ETA is:
 *   eta = remaining_bytes * active_chunks / measured_rate + remaining_latency
 *   eta > 3.0s -> LOD2 (coarsest), eta > 0.5s -> LOD1, else LOD0.
 */
#include <stdio.h>
#include <string.h>

#define CHUNKS 64
#define MAX_INFLIGHT 16         /* concurrent transfer streams (connection cap) */
#define DT (1.0 / 60.0)          /* fixed 60 fps simulation step */
#define MAX_FRAMES 200000
#define EWMA_A 0.2               /* new-sample weight for rate estimate */
#define ETA_HI 3.0               /* seconds */
#define ETA_LO 0.5
#define UNIFORM_RATIO 0.05       /* diagnostic: spread < 5% of mean */

#define LOD_FULL 3
#define LOD0 0                   /* finest placeholder, arrival imminent */
#define LOD1 1
#define LOD2 2                   /* coarsest placeholder */

typedef struct {
    const char* name;
    double rate;                /* nominal bytes per second */
    double latency;             /* nominal chunk start delay, seconds */
    double jitter;              /* +/- fraction applied per frame/chunk */
} Net;

typedef struct {
    double size;                /* bytes */
    double start_delay;         /* seconds before first byte */
    double sent;                /* bytes received */
    double commit_time;         /* seconds, -1 until committed */
    long commit_seq;            /* monotonic commit order, -1 until committed */
    int full;                   /* all bytes downloaded */
    int committed;              /* visible at full precision */
    int inflight;               /* a transfer stream is open for this chunk */
} Chunk;

typedef struct {
    int frames_to_full;         /* frame count until the last chunk committed */
    long placeholder_frames;    /* frames where >=1 tile was not committed */
    long chunk_placeholder_frames; /* sum of per-chunk placeholder frames */
    long lod_frames[3];         /* per-LOD per-chunk placeholder frames */
    int distinct_commits;       /* distinct commit_seq values */
    int uniform;                /* frame-count heuristic verdict */
    int whole_map_suspect;      /* final diagnostic verdict */
} RunResult;

static const Net nets[3] = {
    { "slow",    4096.0,  0.40, 0.15 },
    { "medium", 16384.0,  0.15, 0.15 },
    { "fast",   65536.0,  0.05, 0.15 },
};

/* deterministic LCG so all three runs load the identical map */
static unsigned long rng_state;
static double rnd_unit(void) {
    rng_state = rng_state * 1103515245UL + 12345UL;
    return (double)((rng_state >> 16) & 0x7fff) / 32768.0; /* [0,1) */
}

static void build_map(double sizes[CHUNKS]) {
    rng_state = 7;
    for (int i = 0; i < CHUNKS; i++)
        sizes[i] = 1500.0 + (double)((rng_state >> 16) % 2000); /* 1500..3499 B */
}

static void run_load(const Net* net, const double sizes[CHUNKS], int stream,
                     Chunk out[CHUNKS], RunResult* res) {
    Chunk c[CHUNKS];
    rng_state = 99;
    for (int i = 0; i < CHUNKS; i++) {
        double j = rnd_unit() * 2.0 - 1.0; /* [-1,1) */
        c[i].size = sizes[i];
        c[i].start_delay = net->latency * (1.0 + net->jitter * j);
        c[i].sent = 0.0;
        c[i].commit_time = -1.0;
        c[i].commit_seq = -1;
        c[i].full = 0;
        c[i].committed = 0;
        c[i].inflight = 0;
    }

    memset(res, 0, sizeof(*res));
    double rate_ema = net->rate;   /* cold-start prior, then measured only */
    long next_seq = 0;
    int cursor = 0;
    int frame;
    for (frame = 0; frame < MAX_FRAMES; frame++) {
        double now = frame * DT;

        /* --- network pump: bounded concurrency, bandwidth shared round-robin --- */
        int inflight = 0;
        for (int i = 0; i < CHUNKS; i++) if (c[i].inflight) inflight++;
        /* admit waiting chunks whose start delay has elapsed, up to the cap */
        for (int i = 0; i < CHUNKS && inflight < MAX_INFLIGHT; i++) {
            if (!c[i].inflight && !c[i].full && now >= c[i].start_delay) {
                c[i].inflight = 1;
                inflight++;
            }
        }
        double j = rnd_unit() * 2.0 - 1.0;
        double budget = net->rate * (1.0 + net->jitter * j) * DT;
        double delivered = 0.0;
        int progressed = 1;
        while (budget > 1e-12 && progressed) {
            progressed = 0;
            for (int k = 0; k < CHUNKS && budget > 1e-12; k++) {
                Chunk* p = &c[(cursor + k) % CHUNKS];
                if (!p->inflight || p->full) continue;
                double rem = p->size - p->sent;
                if (rem <= 0.0) continue;
                double give = rem < budget ? rem : budget;
                p->sent += give;
                budget -= give;
                delivered += give;
                progressed = 1;
                if (p->sent >= p->size - 1e-9) {
                    p->sent = p->size;
                    p->full = 1;
                    p->inflight = 0;
                    if (stream) {
                        /* commit at frame end; seq disambiguates same-frame commits */
                        p->commit_time = now + DT;
                        p->commit_seq = next_seq++;
                        p->committed = 1;
                    }
                }
            }
            cursor = (cursor + 1) % CHUNKS;
        }
        if (delivered > 0.0) {
            double sample = delivered / DT;
            rate_ema = (1.0 - EWMA_A) * rate_ema + EWMA_A * sample;
        }

        /* --- render pass: placeholder/LOD exposure this frame --- */
        int active = 0;
        for (int i = 0; i < CHUNKS; i++) if (c[i].inflight) active++;
        int any_placeholder = 0;
        for (int i = 0; i < CHUNKS; i++) {
            Chunk* p = &c[i];
            if (p->committed) continue;
            any_placeholder = 1;
            res->chunk_placeholder_frames++;
            if (stream) {
                double rem = p->size - p->sent;
                double rem_lat = (!p->inflight && now < p->start_delay)
                                 ? p->start_delay - now : 0.0;
                /* the open transfer streams split the measured rate between them */
                double share = active > 0 ? (double)active : 1.0;
                double eta = rem * share / rate_ema + rem_lat;
                int lod = eta > ETA_HI ? LOD2 : eta > ETA_LO ? LOD1 : LOD0;
                res->lod_frames[lod]++;
            }
        }
        if (any_placeholder) res->placeholder_frames++;

        int done = 1;
        for (int i = 0; i < CHUNKS; i++) if (!c[i].full) done = 0;
        if (done) {
            frame++;
            break;
        }
    }
    res->frames_to_full = frame;

    if (!stream) {
        /* single whole-map commit: every tile gets the same moment and seq */
        double t = frame * DT;
        for (int i = 0; i < CHUNKS; i++) {
            c[i].commit_time = t;
            c[i].commit_seq = 0;
            c[i].committed = 1;
        }
    }

    int distinct = 0;
    for (int i = 0; i < CHUNKS; i++) {
        int seen = 0;
        for (int k = 0; k < i; k++)
            if (c[i].commit_seq == c[k].commit_seq) { seen = 1; break; }
        if (!seen) distinct++;
    }
    res->distinct_commits = distinct;
    memcpy(out, c, sizeof(c));
}

static int frames_uniform(const RunResult r[3]) {
    long lo = r[0].placeholder_frames, hi = lo, sum = 0;
    for (int i = 0; i < 3; i++) {
        long v = r[i].placeholder_frames;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        sum += v;
    }
    double mean = (double)sum / 3.0;
    return mean > 0.0 && (double)(hi - lo) < UNIFORM_RATIO * mean;
}

static void print_commit_grid(const Chunk c[CHUNKS]) {
    for (int row = 0; row < 8; row++) {
        printf("    ");
        for (int col = 0; col < 8; col++)
            printf("%6.2f ", c[row * 8 + col].commit_time);
        printf("\n");
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("check %s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    double sizes[CHUNKS];
    build_map(sizes);
    double total = 0.0;
    for (int i = 0; i < CHUNKS; i++) total += sizes[i];
    printf("map: chunks=%d total_bytes=%.0f\n", CHUNKS, total);
    printf("lod rule: eta=remaining*active/measured_rate+remaining_latency; "
           "eta>%.1fs->LOD2, eta>%.2fs->LOD1, else LOD0; full=LOD3\n",
           ETA_HI, ETA_LO);

    /* ---------------- streaming commits (the new behaviour) ---------------- */
    RunResult sr[3];
    Chunk sc[3][CHUNKS];
    printf("\n[stream] per-chunk commit on arrival\n");
    printf("%-8s %14s %18s %22s\n",
           "network", "full_display_ms", "placeholder_frames",
           "chunk_placeholder_frames");
    for (int i = 0; i < 3; i++) {
        run_load(&nets[i], sizes, 1, sc[i], &sr[i]);
        sr[i].uniform = frames_uniform(sr);
        sr[i].whole_map_suspect = sr[i].uniform || sr[i].distinct_commits == 1;
        printf("%-8s %14.0f %18ld %22ld\n",
               nets[i].name, sr[i].frames_to_full * DT * 1000.0,
               sr[i].placeholder_frames, sr[i].chunk_placeholder_frames);
        printf("         lod2=%ld lod1=%ld lod0=%ld distinct_commit_times=%d\n",
               sr[i].lod_frames[2], sr[i].lod_frames[1], sr[i].lod_frames[0],
               sr[i].distinct_commits);
    }
    printf("diagnostic frames_almost_equal=%d whole_map_suspect=%d\n",
           sr[0].uniform, sr[0].whole_map_suspect);
    printf("stream per-chunk actual commit times (s), slow run, 8x8 grid:\n");
    print_commit_grid(sc[0]);
    double first = 1e9, last = 0.0;
    for (int i = 0; i < CHUNKS; i++) {
        if (sc[0][i].commit_time < first) first = sc[0][i].commit_time;
        if (sc[0][i].commit_time > last) last = sc[0][i].commit_time;
    }
    printf("slow run: first_chunk_commit=%.2fs last_chunk_commit=%.2fs span=%.2fs\n",
           first, last, last - first);

    /* ---------------- atomic whole-map commit (old behaviour) --------------- */
    RunResult ar[3];
    Chunk ac[3][CHUNKS];
    printf("\n[atomic] single commit after every chunk arrives (blank ground before)\n");
    printf("%-8s %14s %18s %22s\n",
           "network", "full_display_ms", "placeholder_frames",
           "chunk_placeholder_frames");
    for (int i = 0; i < 3; i++) {
        run_load(&nets[i], sizes, 0, ac[i], &ar[i]);
        ar[i].uniform = frames_uniform(ar);
        ar[i].whole_map_suspect = ar[i].uniform || ar[i].distinct_commits == 1;
        printf("%-8s %14.0f %18ld %22ld\n",
               nets[i].name, ar[i].frames_to_full * DT * 1000.0,
               ar[i].placeholder_frames, ar[i].chunk_placeholder_frames);
    }
    printf("diagnostic frames_almost_equal=%d whole_map_suspect=%d\n",
           ar[0].uniform, ar[0].whole_map_suspect);
    if (ar[0].whole_map_suspect) {
        printf("-> commits look whole-map; per-chunk actual commit times (s), "
               "slow run, 8x8 grid:\n");
        print_commit_grid(ac[0]);
        printf("distinct_commit_times per network: %d/%d/%d\n",
               ar[0].distinct_commits, ar[1].distinct_commits,
               ar[2].distinct_commits);
    }

    /* ---------------- self-test assertions ---------------- */
    printf("\n");
    check("stream_full_time_ordered",
          sr[0].frames_to_full > sr[1].frames_to_full &&
          sr[1].frames_to_full > sr[2].frames_to_full);
    check("stream_placeholder_frames_ordered",
          sr[0].placeholder_frames > sr[1].placeholder_frames &&
          sr[1].placeholder_frames > sr[2].placeholder_frames);
    check("stream_diagnostic_not_fired", sr[0].whole_map_suspect == 0);
    check("stream_every_chunk_own_commit",
          sr[0].distinct_commits == CHUNKS &&
          sr[1].distinct_commits == CHUNKS &&
          sr[2].distinct_commits == CHUNKS);
    check("atomic_diagnostic_fired", ar[0].whole_map_suspect == 1);
    check("atomic_single_commit",
          ar[0].distinct_commits == 1 &&
          ar[1].distinct_commits == 1 &&
          ar[2].distinct_commits == 1);
    check("rate_drives_lod_slow_uses_coarse", sr[0].lod_frames[LOD2] > 0);
    check("rate_drives_lod_fast_skips_coarse", sr[2].lod_frames[LOD2] == 0);
    check("lod0_reserved_for_imminent",
          sr[0].lod_frames[LOD0] > 0 && sr[2].lod_frames[LOD0] > 0);
    /* streaming pays off: first tiles are visible long before the whole map */
    double first_stream = 1e9, first_atomic = 0.0;
    for (int i = 0; i < CHUNKS; i++) {
        if (sc[0][i].commit_time < first_stream) first_stream = sc[0][i].commit_time;
        if (ac[0][i].commit_time > first_atomic) first_atomic = ac[0][i].commit_time;
    }
    check("stream_first_commit_well_before_full",
          first_stream > 0.0 && first_stream < first_atomic * 0.5);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
