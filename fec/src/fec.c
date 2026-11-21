/* fec: block erasure coding (systematic RS over GF(256)) and redundancy planning.
 *
 * A block carries k data packets and m parity packets. Any k of the k+m
 * packets reconstruct the block, so up to m losses per block are repaired
 * without a retransmission round trip.
 *
 * The planner picks the smallest m that reaches a target recovery
 * probability for a measured loss rate, reports the bandwidth overhead and
 * the recovery-probability upper bound, and flags when the required
 * redundancy exceeds the caller's cap. Zero-redundancy configurations are
 * rejected.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define FEC_MAX_K 16
#define FEC_MAX_M 8
#define FEC_MAX_N (FEC_MAX_K + FEC_MAX_M)
#define FEC_MAX_PKT 64

enum {
    FEC_OK = 0,
    FEC_ERR_ARG = -1,
    FEC_ERR_ZERO_REDUNDANCY = -2,
    FEC_ERR_LOSS_RANGE = -3,
    FEC_ERR_UNDECODABLE = -4
};

typedef struct {
    int k;
    int m;
    int pkt_len;
} FecCode;

typedef struct {
    int k;
    int m;             /* applied parity count */
    int required_m;    /* m needed to reach target, -1 if unreachable within FEC_MAX_M */
    double loss;       /* measured loss rate in (0,1) */
    double target;     /* requested recovery probability */
    double prob_ub;    /* recovery-probability upper bound */
    double overhead;   /* m / k */
    int overhead_pct;  /* rounded percentage */
    int reached_target;
    int over_cap;      /* required redundancy exceeds max_overhead */
} FecPlan;

typedef struct {
    double retx_ms;      /* expected retransmission recovery delay */
    double block_ms;     /* average wait for the enclosing block */
    double fallback_ms;  /* expected retransmission delay when a block fails */
    double fec_ms;       /* expected FEC recovery delay incl. fallback */
    int fec_faster;
} FecLatency;

/* ---- GF(256), polynomial 0x11d, generator 2 ---- */

static uint8_t gf_exp[512];
static uint8_t gf_log[256];
static int gf_ready = 0;

static void gf_init(void) {
    if (gf_ready) return;
    int x = 1;
    for (int i = 0; i < 255; i++) {
        gf_exp[i] = (uint8_t)x;
        gf_log[x] = (uint8_t)i;
        x <<= 1;
        if (x & 0x100) x ^= 0x11d;
    }
    for (int i = 255; i < 512; i++) gf_exp[i] = gf_exp[i - 255];
    gf_log[0] = 0; /* log(0) unused; callers guard */
    gf_ready = 1;
}

static uint8_t gf_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0) return 0;
    return gf_exp[(int)gf_log[a] + (int)gf_log[b]];
}

static uint8_t gf_inv(uint8_t a) {
    return gf_exp[(255 - (int)gf_log[a]) % 255];
}

/* Cauchy coefficients: every square submatrix is invertible, so any k
 * surviving rows reconstruct the block. Parity row j uses x = j; data
 * column d uses y = m + d, keeping the two sets disjoint. */
static uint8_t cauchy_coef(int j, int d, int m) {
    return gf_inv((uint8_t)(j ^ (m + d)));
}

static int fec_code_init(FecCode* c, int k, int m, int pkt_len) {
    if (!c || k < 1 || k > FEC_MAX_K || m < 0 || m > FEC_MAX_M) return FEC_ERR_ARG;
    if (pkt_len < 1 || pkt_len > FEC_MAX_PKT) return FEC_ERR_ARG;
    if (m == 0) return FEC_ERR_ZERO_REDUNDANCY;
    gf_init();
    c->k = k;
    c->m = m;
    c->pkt_len = pkt_len;
    return FEC_OK;
}

static int fec_encode(const FecCode* c,
                      const unsigned char data[FEC_MAX_K][FEC_MAX_PKT],
                      unsigned char parity[FEC_MAX_M][FEC_MAX_PKT]) {
    if (!c || !data || !parity) return FEC_ERR_ARG;
    if (c->m == 0) return FEC_ERR_ZERO_REDUNDANCY;
    gf_init();
    for (int j = 0; j < c->m; j++) {
        memset(parity[j], 0, (size_t)c->pkt_len);
        for (int d = 0; d < c->k; d++) {
            uint8_t coef = cauchy_coef(j, d, c->m);
            if (coef == 0) continue;
            for (int x = 0; x < c->pkt_len; x++) {
                parity[j][x] ^= gf_mul(coef, data[d][x]);
            }
        }
    }
    return FEC_OK;
}

/* Reconstruct missing data shards in place. present[] has k+m entries.
 * Returns FEC_OK and sets *recovered to the number rebuilt, or
 * FEC_ERR_UNDECODABLE if too few shards survived. */
static int fec_decode(const FecCode* c,
                      unsigned char shard[FEC_MAX_N][FEC_MAX_PKT],
                      const int present[FEC_MAX_N],
                      int* recovered) {
    if (!c || !shard || !present) return FEC_ERR_ARG;
    if (recovered) *recovered = 0;
    if (c->m == 0) return FEC_ERR_ZERO_REDUNDANCY;
    gf_init();

    int k = c->k, m = c->m, len = c->pkt_len;
    int miss[FEC_MAX_K], nmiss = 0;
    for (int d = 0; d < k; d++) if (!present[d]) miss[nmiss++] = d;
    if (nmiss == 0) return FEC_OK;
    if (nmiss > m) return FEC_ERR_UNDECODABLE;

    int prow[FEC_MAX_M], nprow = 0;
    for (int j = 0; j < m; j++) if (present[k + j]) prow[nprow++] = j;
    if (nprow < nmiss) return FEC_ERR_UNDECODABLE;

    int t = nmiss;
    uint8_t A[FEC_MAX_M][FEC_MAX_M];
    uint8_t B[FEC_MAX_M][FEC_MAX_PKT];
    memset(A, 0, sizeof(A));
    memset(B, 0, sizeof(B));

    /* A * missing = parity XOR contributions of the data we already have */
    for (int r = 0; r < t; r++) {
        int j = prow[r];
        for (int col = 0; col < t; col++) A[r][col] = cauchy_coef(j, miss[col], m);
        memcpy(B[r], shard[k + j], (size_t)len);
        for (int d = 0; d < k; d++) {
            if (!present[d]) continue;
            uint8_t coef = cauchy_coef(j, d, m);
            if (coef == 0) continue;
            for (int x = 0; x < len; x++) B[r][x] ^= gf_mul(coef, shard[d][x]);
        }
    }

    /* Gauss-Jordan elimination over GF(256) */
    for (int col = 0; col < t; col++) {
        int piv = -1;
        for (int r = col; r < t; r++) if (A[r][col]) { piv = r; break; }
        if (piv < 0) return FEC_ERR_UNDECODABLE;
        if (piv != col) {
            for (int cc = 0; cc < t; cc++) {
                uint8_t tmp = A[col][cc]; A[col][cc] = A[piv][cc]; A[piv][cc] = tmp;
            }
            for (int x = 0; x < len; x++) {
                uint8_t tmp = B[col][x]; B[col][x] = B[piv][x]; B[piv][x] = tmp;
            }
        }
        uint8_t inv = gf_inv(A[col][col]);
        if (inv != 1) {
            for (int cc = col; cc < t; cc++) A[col][cc] = gf_mul(inv, A[col][cc]);
            for (int x = 0; x < len; x++) B[col][x] = gf_mul(inv, B[col][x]);
        }
        for (int r = 0; r < t; r++) {
            if (r == col) continue;
            uint8_t f = A[r][col];
            if (f == 0) continue;
            for (int cc = col + 1; cc < t; cc++) A[r][cc] ^= gf_mul(f, A[col][cc]);
            A[r][col] = 0;
            for (int x = 0; x < len; x++) B[r][x] ^= gf_mul(f, B[col][x]);
        }
    }

    for (int r = 0; r < t; r++) memcpy(shard[miss[r]], B[r], (size_t)len);
    if (recovered) *recovered = t;
    return FEC_OK;
}

/* Block recovery probability, exact for independent losses and an MDS code.
 * Correlated (burst) losses make the real value lower, hence "upper bound". */
static double fec_recover_prob_ub(int k, int m, double loss) {
    if (k < 1 || m < 0 || !(loss > 0.0 && loss < 1.0)) return 0.0;
    int n = k + m;
    double q = 1.0 - loss;
    double fail = 0.0;
    for (int r = 0; r < k; r++) {
        double lc = lgamma(n + 1.0) - lgamma(r + 1.0) - lgamma(n - r + 1.0);
        fail += exp(lc + r * log(q) + (n - r) * log(loss));
    }
    if (fail < 0.0) fail = 0.0;
    if (fail > 1.0) fail = 1.0;
    return 1.0 - fail;
}

/* Pick the smallest parity count reaching target within the overhead cap.
 * loss == 0 requires zero redundancy, which is rejected rather than silently
 * disabling protection. A cap that permits no parity is rejected too. */
static int fec_plan(int k, double loss, double target, double max_overhead, FecPlan* out) {
    if (!out || k < 1 || k > FEC_MAX_K || !(target > 0.0 && target <= 1.0)) return FEC_ERR_ARG;
    if (!(max_overhead > 0.0)) return FEC_ERR_ARG;
    if (!(loss > 0.0 && loss < 1.0)) {
        if (loss == 0.0) return FEC_ERR_ZERO_REDUNDANCY;
        return FEC_ERR_LOSS_RANGE;
    }

    int required = -1;
    for (int m = 1; m <= FEC_MAX_M; m++) {
        if (fec_recover_prob_ub(k, m, loss) + 1e-12 >= target) { required = m; break; }
    }

    int cap_m = (int)floor(max_overhead * k + 1e-9);
    if (cap_m > FEC_MAX_M) cap_m = FEC_MAX_M;
    if (cap_m < 1) return FEC_ERR_ZERO_REDUNDANCY;

    int applied = (required > 0) ? required : FEC_MAX_M;
    if (applied > cap_m) applied = cap_m;
    if (applied < 1) return FEC_ERR_ZERO_REDUNDANCY;

    memset(out, 0, sizeof(*out));
    out->k = k;
    out->m = applied;
    out->required_m = required;
    out->loss = loss;
    out->target = target;
    out->prob_ub = fec_recover_prob_ub(k, applied, loss);
    out->overhead = (double)applied / (double)k;
    out->overhead_pct = (int)lround(out->overhead * 100.0);
    out->reached_target = out->prob_ub + 1e-12 >= target;
    out->over_cap = (required > 0) ? (required > cap_m) : 0;
    return FEC_OK;
}

/* Retransmission: a lost packet is retried after one RTT; each retry is
 * independently lost with probability loss, so E[delay] = rtt / (1-loss).
 * FEC: a lost packet waits on average half the block span plus decode time;
 * a block that cannot be repaired falls back to retransmission. */
static int fec_compare_latency(const FecPlan* p, double rtt_ms, double pkt_interval_ms,
                               double decode_ms, FecLatency* out) {
    if (!p || !out || p->k < 1 || p->m < 1) return FEC_ERR_ARG;
    if (!(rtt_ms > 0.0) || !(pkt_interval_ms >= 0.0) || !(decode_ms >= 0.0)) return FEC_ERR_ARG;
    if (!(p->loss > 0.0 && p->loss < 1.0)) return FEC_ERR_LOSS_RANGE;

    memset(out, 0, sizeof(*out));
    out->retx_ms = rtt_ms / (1.0 - p->loss);
    out->block_ms = (p->k + p->m - 1) * 0.5 * pkt_interval_ms + decode_ms;
    out->fallback_ms = out->retx_ms;
    out->fec_ms = out->block_ms + (1.0 - p->prob_ub) * out->fallback_ms;
    out->fec_faster = (out->fec_ms < out->retx_ms) ? 1 : 0;
    return FEC_OK;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void fill_data(unsigned char data[FEC_MAX_K][FEC_MAX_PKT], int k, int len) {
    for (int d = 0; d < k; d++)
        for (int x = 0; x < len; x++) data[d][x] = (unsigned char)(d * 31 + x * 7 + 3);
}

static void load_shards(const FecCode* c,
                        const unsigned char data[FEC_MAX_K][FEC_MAX_PKT],
                        const unsigned char parity[FEC_MAX_M][FEC_MAX_PKT],
                        unsigned char shard[FEC_MAX_N][FEC_MAX_PKT],
                        int present[FEC_MAX_N]) {
    memset(shard, 0, sizeof(unsigned char) * FEC_MAX_N * FEC_MAX_PKT);
    for (int d = 0; d < c->k; d++) memcpy(shard[d], data[d], (size_t)c->pkt_len);
    for (int j = 0; j < c->m; j++) memcpy(shard[c->k + j], parity[j], (size_t)c->pkt_len);
    for (int i = 0; i < c->k + c->m; i++) present[i] = 1;
}

int main(void) {
    FecCode c;
    check("reject_zero_m", fec_code_init(&c, 8, 0, 16) == FEC_ERR_ZERO_REDUNDANCY);
    check("reject_bad_k", fec_code_init(&c, 0, 2, 16) == FEC_ERR_ARG);
    check("reject_big_pkt", fec_code_init(&c, 8, 2, FEC_MAX_PKT + 1) == FEC_ERR_ARG);
    check("init_ok", fec_code_init(&c, 6, 3, 24) == FEC_OK);

    unsigned char data[FEC_MAX_K][FEC_MAX_PKT];
    unsigned char parity[FEC_MAX_M][FEC_MAX_PKT];
    unsigned char shard[FEC_MAX_N][FEC_MAX_PKT];
    int present[FEC_MAX_N];
    fill_data(data, 16, 64);
    check("encode", fec_encode(&c, data, parity) == FEC_OK);

    int rec = -1;
    load_shards(&c, data, parity, shard, present);
    check("decode_none", fec_decode(&c, shard, present, &rec) == FEC_OK && rec == 0);

    /* one data + one data + one parity lost: 2 missing data, 2 parity left */
    load_shards(&c, data, parity, shard, present);
    present[1] = present[4] = present[6 + 2] = 0;
    rec = -1;
    check("decode_mixed", fec_decode(&c, shard, present, &rec) == FEC_OK && rec == 2);
    check("decode_mixed_data",
          memcmp(shard[1], data[1], 24) == 0 && memcmp(shard[4], data[4], 24) == 0);

    /* m data losses: the parity ceiling */
    load_shards(&c, data, parity, shard, present);
    present[0] = present[2] = present[5] = 0;
    rec = -1;
    check("decode_three", fec_decode(&c, shard, present, &rec) == FEC_OK && rec == 3);
    check("decode_three_data",
          memcmp(shard[0], data[0], 24) == 0 &&
          memcmp(shard[2], data[2], 24) == 0 &&
          memcmp(shard[5], data[5], 24) == 0);

    /* m+1 data losses cannot be repaired */
    load_shards(&c, data, parity, shard, present);
    present[0] = present[1] = present[2] = present[3] = 0;
    check("decode_too_many", fec_decode(&c, shard, present, &rec) == FEC_ERR_UNDECODABLE);

    /* 3 missing data but only 2 parity survive */
    load_shards(&c, data, parity, shard, present);
    present[0] = present[1] = present[2] = present[6] = 0;
    check("decode_short_parity", fec_decode(&c, shard, present, &rec) == FEC_ERR_UNDECODABLE);

    /* largest supported block: 8 of 16 data lost */
    FecCode big;
    check("init_big", fec_code_init(&big, 16, 8, 32) == FEC_OK);
    unsigned char bdata[FEC_MAX_K][FEC_MAX_PKT];
    unsigned char bparity[FEC_MAX_M][FEC_MAX_PKT];
    fill_data(bdata, 16, 32);
    check("encode_big", fec_encode(&big, bdata, bparity) == FEC_OK);
    load_shards(&big, bdata, bparity, shard, present);
    for (int d = 0; d < 8; d++) present[d] = 0;
    rec = -1;
    check("decode_big", fec_decode(&big, shard, present, &rec) == FEC_OK && rec == 8);
    int big_ok = 1;
    for (int d = 0; d < 8; d++) if (memcmp(shard[d], bdata[d], 32) != 0) big_ok = 0;
    check("decode_big_data", big_ok);

    /* exhaustive: every loss pattern for k=4,m=2 decodes exactly when
     * at most 2 shards are missing (Cauchy MDS property) */
    FecCode sm;
    fec_code_init(&sm, 4, 2, 8);
    unsigned char sdata[FEC_MAX_K][FEC_MAX_PKT];
    unsigned char spar[FEC_MAX_M][FEC_MAX_PKT];
    fill_data(sdata, 4, 8);
    fec_encode(&sm, sdata, spar);
    int all_patterns_ok = 1, pattern_n = 0;
    for (int mask = 0; mask < (1 << 6); mask++) {
        load_shards(&sm, sdata, spar, shard, present);
        int lost = 0;
        for (int b = 0; b < 6; b++) {
            if (mask & (1 << b)) { present[b] = 0; lost++; }
        }
        int expect_ok = lost <= sm.m;
        int r = fec_decode(&sm, shard, present, &rec);
        if (expect_ok != (r == FEC_OK)) { all_patterns_ok = 0; continue; }
        if (r == FEC_OK) {
            for (int d = 0; d < 4; d++) {
                if (memcmp(shard[d], sdata[d], 8) != 0) all_patterns_ok = 0;
            }
        }
        pattern_n++;
    }
    check("exhaustive_patterns", all_patterns_ok && pattern_n == 64);

    /* probability formula agrees with direct binomial enumeration */
    int prob_ok = 1;
    for (int kk = 2; kk <= 10; kk += 2) {
        for (int mm = 1; mm <= 4; mm++) {
            for (double l = 0.01; l < 0.5; l += 0.07) {
                int n = kk + mm;
                /* fail when fewer than k shards arrive: sum over received r < k */
                double fail = 0.0;
                for (int r = 0; r < kk; r++) {
                    double c = 1.0;
                    for (int i = 0; i < r; i++) c *= (double)(n - i) / (double)(i + 1);
                    fail += c * pow(1.0 - l, r) * pow(l, n - r);
                }
                double ref = 1.0 - fail;
                if (fabs(fec_recover_prob_ub(kk, mm, l) - ref) > 1e-9) prob_ok = 0;
            }
        }
    }
    check("prob_matches_binomial", prob_ok);

    /* planning */
    FecPlan plan;
    check("plan_zero_loss_rejected",
          fec_plan(8, 0.0, 0.99, 1.0, &plan) == FEC_ERR_ZERO_REDUNDANCY);
    check("plan_full_loss_rejected",
          fec_plan(8, 1.0, 0.99, 1.0, &plan) == FEC_ERR_LOSS_RANGE);
    check("plan_bad_target", fec_plan(8, 0.05, 0.0, 1.0, &plan) == FEC_ERR_ARG);
    check("plan_ok", fec_plan(8, 0.02, 0.99, 1.0, &plan) == FEC_OK);
    check("plan_has_parity", plan.m >= 1);
    check("plan_hits_target", plan.reached_target == 1 && plan.prob_ub + 1e-12 >= 0.99);
    check("plan_overhead_math",
          fabs(plan.overhead - (double)plan.m / 8.0) < 1e-12 &&
          plan.overhead_pct == (int)lround(plan.overhead * 100.0));
    check("plan_within_cap", plan.over_cap == 0);

    check("prob_rises_with_m",
          fec_recover_prob_ub(8, 2, 0.05) > fec_recover_prob_ub(8, 1, 0.05));
    check("prob_falls_with_loss",
          fec_recover_prob_ub(8, 2, 0.10) < fec_recover_prob_ub(8, 2, 0.05));
    check("prob_bounded",
          fec_recover_prob_ub(8, 2, 0.05) >= 0.0 &&
          fec_recover_prob_ub(8, 2, 0.05) <= 1.0);

    /* overhead cap: 3 parity needed, cap allows only 1 */
    check("cap_plan", fec_plan(10, 0.05, 0.99, 0.15, &plan) == FEC_OK);
    check("cap_reported", plan.over_cap == 1 && plan.required_m == 3);
    check("cap_applied", plan.m == 1 && plan.overhead_pct == 10);
    check("cap_misses_target", plan.reached_target == 0 && plan.prob_ub < 0.99);

    /* cap so tight that no parity packet fits -> zero redundancy rejected */
    check("cap_zero_rejected",
          fec_plan(10, 0.05, 0.99, 0.05, &plan) == FEC_ERR_ZERO_REDUNDANCY);

    /* target unreachable even at max parity */
    check("unreachable_plan", fec_plan(8, 0.9, 0.99, 10.0, &plan) == FEC_OK);
    check("unreachable_reported",
          plan.required_m == -1 && plan.reached_target == 0 && plan.m == FEC_MAX_M);

    /* latency comparison */
    FecPlan p;
    FecLatency lat;
    fec_plan(10, 0.05, 0.99, 1.0, &p);
    check("lat_ok", fec_compare_latency(&p, 60.0, 6.0, 1.0, &lat) == FEC_OK);
    check("lat_retx_formula", fabs(lat.retx_ms - 60.0 / 0.95) < 1e-9);
    check("lat_fec_formula",
          fabs(lat.fec_ms - (lat.block_ms + (1.0 - p.prob_ub) * lat.retx_ms)) < 1e-9);
    check("lat_fec_faster", lat.fec_faster == 1);

    /* same code, but a slow block and short RTT: retransmission wins */
    check("lat_slow_ok", fec_compare_latency(&p, 10.0, 20.0, 1.0, &lat) == FEC_OK);
    check("lat_retx_faster", lat.fec_faster == 0 && lat.fec_ms > lat.retx_ms);
    check("lat_bad_args", fec_compare_latency(&p, 0.0, 5.0, 0.0, &lat) == FEC_ERR_ARG);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
