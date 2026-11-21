
/* adaptive: per-field adaptive quantization driven by visibility/distance,
   bit allocation under per-field error budgets, and graceful degradation.
   Wire format is bit-packed; fixed-step and adaptive plans share one
   encode/decode path so the bandwidth comparison is apples-to-apples. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define FIELD_N 4
#define FIELD_X  0
#define FIELD_Y  1
#define FIELD_VX 2
#define FIELD_VY 3
#define MAX_WIDTH 31          /* largest supported field width, in bits */
#define VIS_MIN   0.1f        /* visibility floor so steps cannot explode */
#define PKT_CAP   64          /* bytes */

#define RET_OK       0
#define RET_DEGRADED 1        /* sent, optional fields dropped */
#define RET_BAD      -1       /* invalid input */
#define RET_SKIP     -2       /* budget infeasible: entity skipped this tick */
#define RET_RANGE    -3       /* value outside declared field range */

typedef struct {
    float lo, hi;
    float err_budget;         /* max allowed absolute quant error, world units */
} FieldSpec;

typedef struct {
    float visibility;         /* (0, 1] */
    float distance;           /* >= 0, world units */
} AdaptCtx;

typedef struct {
    float step[FIELD_N];      /* quantization step, world units */
    int   bits[FIELD_N];      /* allocated width, bits */
    unsigned mask;            /* bit i set => field i present on the wire */
    int   degraded;
} EntPlan;

typedef struct { unsigned id; float x, y, vx, vy; } EntState;

static int finite_pos(float x) { return isfinite(x) && x > 0.0f; }

/* Adapt one field's base step to visibility/distance.
   factor = (1 + d/d_ref) / max(vis, VIS_MIN): far or poorly visible
   entities get coarser steps; near + fully visible keeps the base. */
static int adapt_step(float base, const AdaptCtx* ctx, float d_ref, float* out) {
    if (out == NULL) return RET_BAD;
    if (!finite_pos(base) || !finite_pos(d_ref)) return RET_BAD;
    if (ctx == NULL || !isfinite(ctx->visibility) ||
        ctx->visibility <= 0.0f || ctx->visibility > 1.0f) return RET_BAD;
    if (!isfinite(ctx->distance) || ctx->distance < 0.0f) return RET_BAD;
    float vis = ctx->visibility < VIS_MIN ? VIS_MIN : ctx->visibility;
    *out = base * (1.0f + ctx->distance / d_ref) / vis;
    return RET_OK;
}

/* Bit width needed to cover span at the given step.
   Rejects zero/negative steps and ranges too wide for MAX_WIDTH. */
static int field_width(float span, float step, int* out_bits) {
    if (out_bits == NULL || !isfinite(span) || span < 0.0f) return RET_BAD;
    if (!finite_pos(step)) return RET_BAD;                /* zero step rejected */
    if (span == 0.0f) { *out_bits = 0; return RET_OK; }
    float ratio = span / step;
    if (!isfinite(ratio) || ratio > 2.0e9f) return RET_BAD;
    long long levels = (long long)floorf(ratio) + 1;     /* indices 0..levels-1 */
    int n = 0;
    for (long long v = levels - 1; v > 0; v >>= 1) n++;
    if (n > MAX_WIDTH) return RET_BAD;
    *out_bits = n;
    return RET_OK;
}

/* Round-to-nearest error upper bounds. */
static int field_error_bound(float step, float* out) {
    if (!finite_pos(step) || out == NULL) return RET_BAD;
    *out = step * 0.5f;
    return RET_OK;
}

static int position_error_bound(float sx, float sy, float* out) {
    if (!finite_pos(sx) || !finite_pos(sy) || out == NULL) return RET_BAD;
    *out = 0.5f * hypotf(sx, sy);
    return RET_OK;
}

/* Build an adaptive plan. The error budget may only tighten the adapted
   step (step <= 2*budget, since |error| <= step/2). If the required widths
   do not fit the bit budget, degrade: drop optional velocity fields first;
   if even essential x/y do not fit, skip the entity for this tick.
   Bit widths are never shrunk below what the error bound requires. */
static int plan_entity(EntPlan* pl, const FieldSpec spec[FIELD_N],
                       const AdaptCtx* ctx, const float base_step[FIELD_N],
                       int bit_budget, float d_ref) {
    if (pl == NULL || spec == NULL || ctx == NULL || base_step == NULL) return RET_BAD;
    if (bit_budget < 0) return RET_BAD;                  /* negative budget rejected */
    if (!finite_pos(d_ref)) return RET_BAD;
    memset(pl, 0, sizeof(*pl));

    for (int i = 0; i < FIELD_N; i++) {
        if (!isfinite(spec[i].lo) || !isfinite(spec[i].hi) ||
            spec[i].hi < spec[i].lo) return RET_BAD;
        if (!finite_pos(spec[i].err_budget)) return RET_BAD;
        if (!finite_pos(base_step[i])) return RET_BAD;
        float s;
        if (adapt_step(base_step[i], ctx, d_ref, &s) != RET_OK) return RET_BAD;
        float cap = 2.0f * spec[i].err_budget;           /* bound: step/2 <= budget */
        if (s > cap) s = cap;
        int b;
        if (field_width(spec[i].hi - spec[i].lo, s, &b) != RET_OK) return RET_BAD;
        pl->step[i] = s;
        pl->bits[i] = b;
    }

    int total = pl->bits[0] + pl->bits[1] + pl->bits[2] + pl->bits[3];
    pl->mask = 0xFu;
    if (total <= bit_budget) { pl->degraded = 0; return RET_OK; }

    pl->mask = 0x3u;                                     /* keep x/y, drop vx/vy */
    pl->degraded = 1;
    int essential = pl->bits[0] + pl->bits[1];
    if (essential <= bit_budget) return RET_DEGRADED;
    return RET_SKIP;
}

/* Fixed-step reference plan: one fine step per field for every entity. */
static int fixed_plan(EntPlan* pl, const FieldSpec spec[FIELD_N],
                      const float fixed_step[FIELD_N]) {
    if (pl == NULL || spec == NULL || fixed_step == NULL) return RET_BAD;
    memset(pl, 0, sizeof(*pl));
    for (int i = 0; i < FIELD_N; i++) {
        if (!finite_pos(fixed_step[i])) return RET_BAD;
        int b;
        if (field_width(spec[i].hi - spec[i].lo, fixed_step[i], &b) != RET_OK)
            return RET_BAD;
        pl->step[i] = fixed_step[i];
        pl->bits[i] = b;
    }
    pl->mask = 0xFu;
    return RET_OK;
}

/* Bit writer/reader: both schemes pay the identical 36-bit header
   (tick 24, id 8, field mask 4), only payload widths differ. */
typedef struct { unsigned char buf[PKT_CAP]; size_t bit; } BitWriter;

static int bw_put(BitWriter* w, unsigned v, int n) {
    if (n < 0 || n > MAX_WIDTH + 1) return RET_BAD;
    if (n < 32 && v > ((1ULL << n) - 1)) return RET_BAD;
    if ((w->bit + (size_t)n + 7) / 8 > PKT_CAP) return RET_BAD;
    for (int i = 0; i < n; i++) {
        w->buf[w->bit >> 3] |= (unsigned char)(((v >> (n - 1 - i)) & 1u)
                                               << (7 - (w->bit & 7)));
        w->bit++;
    }
    return RET_OK;
}

typedef struct { const unsigned char* buf; size_t bit, nbits; } BitReader;

static int br_get(BitReader* r, unsigned* out, int n) {
    if (n < 0 || n > MAX_WIDTH + 1 || r->bit + (size_t)n > r->nbits) return RET_BAD;
    unsigned v = 0;
    for (int i = 0; i < n; i++) {
        v = (v << 1) | ((r->buf[r->bit >> 3] >> (7 - (r->bit & 7))) & 1u);
        r->bit++;
    }
    *out = v;
    return RET_OK;
}

static const float* field_value(const EntState* e, int i) {
    const float* addr[FIELD_N] = { &e->x, &e->y, &e->vx, &e->vy };
    return addr[i];
}
static float* field_value_mut(EntState* e, int i) {
    float* addr[FIELD_N] = { &e->x, &e->y, &e->vx, &e->vy };
    return addr[i];
}

static int encode_entity(BitWriter* w, const EntState* e, const EntPlan* pl,
                         const FieldSpec spec[FIELD_N], unsigned tick) {
    if (w == NULL || e == NULL || pl == NULL || spec == NULL) return RET_BAD;
    if (bw_put(w, tick & 0xFFFFFFu, 24) != RET_OK) return RET_BAD;
    if (bw_put(w, e->id & 0xFFu, 8) != RET_OK) return RET_BAD;
    if (bw_put(w, pl->mask & 0xFu, 4) != RET_OK) return RET_BAD;
    for (int i = 0; i < FIELD_N; i++) {
        if (!(pl->mask & (1u << i))) continue;
        float v = *field_value(e, i);
        if (!isfinite(v)) return RET_BAD;
        long long idx = llrintf((v - spec[i].lo) / pl->step[i]);
        long long max_idx = (long long)floorf((spec[i].hi - spec[i].lo)
                                              / pl->step[i]);
        if (idx < 0 || idx > max_idx) return RET_RANGE;
        if (bw_put(w, (unsigned)idx, pl->bits[i]) != RET_OK) return RET_BAD;
    }
    return RET_OK;
}

/* Absent fields read back as 0; the caller is expected to hold the last
   received value for them. */
static int decode_entity(const BitWriter* pkt, EntState* e, const EntPlan* pl,
                         const FieldSpec spec[FIELD_N], unsigned* tick) {
    if (pkt == NULL || e == NULL || pl == NULL || spec == NULL) return RET_BAD;
    BitReader r = { pkt->buf, 0, pkt->bit };
    unsigned t, id, mask;
    if (br_get(&r, &t, 24) != RET_OK || br_get(&r, &id, 8) != RET_OK ||
        br_get(&r, &mask, 4) != RET_OK) return RET_BAD;
    if (mask & ~0xFu) return RET_BAD;
    memset(e, 0, sizeof(*e));
    e->id = id;
    if (tick != NULL) *tick = t;
    for (int i = 0; i < FIELD_N; i++) {
        if (!(mask & (1u << i))) continue;
        unsigned idx;
        if (br_get(&r, &idx, pl->bits[i]) != RET_OK) return RET_BAD;
        *field_value_mut(e, i) = spec[i].lo + (float)idx * pl->step[i];
    }
    return RET_OK;
}

/* ---- test harness ---- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned rng_state = 0x12345678u;
static float rng_float(float lo, float hi) {
    rng_state = rng_state * 1103515245u + 12345u;
    float u = (float)(rng_state >> 8) / (float)(1u << 24);
    return lo + u * (hi - lo);
}

int main(void) {
    /* x/y span 400 with 0.3 unit budget; vx/vy span 100 with 0.5. */
    FieldSpec spec[FIELD_N] = {
        { -200.0f, 200.0f, 0.3f },
        { -200.0f, 200.0f, 0.3f },
        {  -50.0f,  50.0f, 0.5f },
        {  -50.0f,  50.0f, 0.5f },
    };
    const float base_step[FIELD_N] = { 0.05f, 0.05f, 0.1f, 0.1f };
    const float d_ref = 30.0f;

    /* step adaptation */
    float s;
    AdaptCtx near_full = { 1.0f, 0.0f };
    AdaptCtx far_one = { 1.0f, d_ref };
    AdaptCtx dim = { 0.5f, 0.0f };
    check("adapt_near", adapt_step(0.05f, &near_full, d_ref, &s) == RET_OK &&
                        fabsf(s - 0.05f) < 1e-7f);
    check("adapt_distance", adapt_step(0.05f, &far_one, d_ref, &s) == RET_OK &&
                            fabsf(s - 0.10f) < 1e-7f);
    check("adapt_visibility", adapt_step(0.05f, &dim, d_ref, &s) == RET_OK &&
                              fabsf(s - 0.10f) < 1e-7f);
    check("adapt_vis_floor", ({ AdaptCtx c = { 0.01f, 0.0f };
                                adapt_step(0.05f, &c, d_ref, &s) == RET_OK &&
                                fabsf(s - 0.5f) < 1e-6f; }));
    check("zero_step_rejected", adapt_step(0.0f, &near_full, d_ref, &s) == RET_BAD);
    check("bad_visibility_rejected",
          ({ AdaptCtx c = { 0.0f, 1.0f }; adapt_step(0.05f, &c, d_ref, &s); })
          == RET_BAD);
    check("bad_distance_rejected",
          ({ AdaptCtx c = { 1.0f, -1.0f }; adapt_step(0.05f, &c, d_ref, &s); })
          == RET_BAD);
    check("nan_rejected", adapt_step(NAN, &near_full, d_ref, &s) == RET_BAD);

    /* widths and bounds */
    int bits;
    check("width_basic", field_width(100.0f, 0.5f, &bits) == RET_OK && bits == 8);
    check("width_zero_span", field_width(0.0f, 0.5f, &bits) == RET_OK && bits == 0);
    check("width_zero_step_rejected", field_width(100.0f, 0.0f, &bits) == RET_BAD);
    check("width_neg_step_rejected", field_width(100.0f, -1.0f, &bits) == RET_BAD);
    float bound;
    check("field_bound", field_error_bound(0.25f, &bound) == RET_OK &&
                         fabsf(bound - 0.125f) < 1e-7f);
    check("pos_bound", position_error_bound(0.2f, 0.4f, &bound) == RET_OK &&
                       fabsf(bound - 0.5f * hypotf(0.2f, 0.4f)) < 1e-7f);

    /* round-to-nearest bound holds over thousands of samples; the extra
       fabsf(q)*1e-6 term covers float reconstruction ulp error. */
    int bound_holds = 1;
    const float test_steps[] = { 0.01f, 0.25f, 1.0f, 10.0f };
    for (size_t k = 0; k < sizeof(test_steps) / sizeof(test_steps[0]); k++) {
        float st = test_steps[k];
        for (int j = 0; j < 2000; j++) {
            float v = rng_float(-173.0f, 191.0f);
            float q = (float)llrintf(v / st) * st;
            if (fabsf(v - q) > st * 0.5f + fabsf(q) * 1e-6f + 1e-7f) {
                bound_holds = 0;
            }
        }
    }
    check("error_bound_holds", bound_holds);

    /* planning: ok / degraded / skip / rejection */
    EntPlan pl;
    AdaptCtx ctx_near = { 1.0f, 5.0f };
    check("plan_ok", plan_entity(&pl, spec, &ctx_near, base_step, 46, d_ref)
                    == RET_OK && pl.mask == 0xFu);
    float pb;
    position_error_bound(pl.step[FIELD_X], pl.step[FIELD_Y], &pb);
    check("plan_within_position_budget",
          pb <= hypotf(spec[0].err_budget, spec[1].err_budget) + 1e-7f);

    check("plan_degraded", plan_entity(&pl, spec, &ctx_near, base_step, 40, d_ref)
                           == RET_DEGRADED && pl.mask == 0x3u && pl.degraded);
    position_error_bound(pl.step[FIELD_X], pl.step[FIELD_Y], &pb);
    check("degraded_still_within_budget",
          pb <= hypotf(spec[0].err_budget, spec[1].err_budget) + 1e-7f);

    check("plan_skip", plan_entity(&pl, spec, &ctx_near, base_step, 8, d_ref)
                       == RET_SKIP);
    check("neg_budget_rejected",
          plan_entity(&pl, spec, &ctx_near, base_step, -1, d_ref) == RET_BAD);
    check("zero_err_budget_rejected", ({
              FieldSpec bad[FIELD_N];
              memcpy(bad, spec, sizeof(bad));
              bad[0].err_budget = 0.0f;
              plan_entity(&pl, bad, &ctx_near, base_step, 46, d_ref);
          }) == RET_BAD);
    check("zero_base_step_rejected", ({
              float bad[FIELD_N] = { 0.0f, 0.05f, 0.1f, 0.1f };
              plan_entity(&pl, spec, &ctx_near, bad, 46, d_ref);
          }) == RET_BAD);

    /* roundtrip through the bit-packed wire format */
    EntState in = { 7u, 12.345f, -67.89f, 3.21f, -4.56f };
    check("plan_roundtrip_setup",
          plan_entity(&pl, spec, &ctx_near, base_step, 46, d_ref) == RET_OK);
    BitWriter w;
    memset(&w, 0, sizeof(w));
    check("encode_ok", encode_entity(&w, &in, &pl, spec, 1000u) == RET_OK);
    EntState out;
    unsigned tick;
    check("decode_ok", decode_entity(&w, &out, &pl, spec, &tick) == RET_OK);
    check("roundtrip_tick_id", tick == 1000u && out.id == in.id);
    check("roundtrip_pos_error",
          hypotf(out.x - in.x, out.y - in.y) <= pb + 1e-6f);
    check("out_of_range_rejected", ({
              EntState wild = { 8u, 9999.0f, 0.0f, 0.0f, 0.0f };
              BitWriter w2;
              memset(&w2, 0, sizeof(w2));
              encode_entity(&w2, &wild, &pl, spec, 1u);
          }) == RET_RANGE);

    /* ---- batch comparison: adaptive vs fixed step ---- */
    enum { NENT = 8 };
    const float dist[NENT] = { 5, 12, 20, 30, 45, 60, 85, 110 };
    const float vis[NENT]  = { 1.0f, 0.9f, 0.8f, 0.7f, 0.6f, 0.5f, 0.45f, 0.4f };
    EntState ent[NENT];
    EntPlan ap[NENT], fp;
    check("fixed_plan_ok", fixed_plan(&fp, spec, base_step) == RET_OK);

    int adapt_budget = 40;                 /* all entities sent, some degrade */
    size_t fixed_bits = 0, adapt_bits = 0;
    size_t fixed_bytes = 0, adapt_bytes = 0;
    double adapt_err_sum = 0.0, fixed_err_sum = 0.0;
    float adapt_err_max = 0.0f, fixed_err_max = 0.0f;
    int n_ok = 0, n_deg = 0, n_skip = 0, budget_holds = 1, finer_holds = 1;

    printf("\nid  dist   vis   step_x   step_y  payload  mask  status\n");
    for (int i = 0; i < NENT; i++) {
        ent[i].id = (unsigned)(i + 1);
        ent[i].x = rng_float(-180.0f, 180.0f);
        ent[i].y = rng_float(-180.0f, 180.0f);
        ent[i].vx = rng_float(-40.0f, 40.0f);
        ent[i].vy = rng_float(-40.0f, 40.0f);
        AdaptCtx c = { vis[i], dist[i] };

        int rc = plan_entity(&ap[i], spec, &c, base_step, adapt_budget, d_ref);
        if (rc == RET_SKIP) { n_skip++; continue; }
        if (rc == RET_DEGRADED) n_deg++; else n_ok++;

        /* structural invariant: the fixed reference never uses a coarser
           step than the adaptive plan, so its bound is always tighter. */
        for (int k = 0; k < FIELD_N; k++)
            if (ap[i].mask & (1u << k))
                if (fp.step[k] > ap[i].step[k] + 1e-9f) finer_holds = 0;

        BitWriter wa, wf;
        memset(&wa, 0, sizeof(wa));
        memset(&wf, 0, sizeof(wf));
        if (encode_entity(&wa, &ent[i], &ap[i], spec, 500u) != RET_OK ||
            encode_entity(&wf, &ent[i], &fp, spec, 500u) != RET_OK) {
            check("batch_encode", 0); return 1;
        }
        adapt_bits += wa.bit; fixed_bits += wf.bit;
        adapt_bytes += (wa.bit + 7) / 8;
        fixed_bytes += (wf.bit + 7) / 8;

        EntState da, df;
        if (decode_entity(&wa, &da, &ap[i], spec, NULL) != RET_OK ||
            decode_entity(&wf, &df, &fp, spec, NULL) != RET_OK) {
            check("batch_decode", 0); return 1;
        }
        float ea = hypotf(da.x - ent[i].x, da.y - ent[i].y);
        float ef = hypotf(df.x - ent[i].x, df.y - ent[i].y);
        adapt_err_sum += ea; fixed_err_sum += ef;
        if (ea > adapt_err_max) adapt_err_max = ea;
        if (ef > fixed_err_max) fixed_err_max = ef;

        float eb = 0.0f;
        position_error_bound(ap[i].step[0], ap[i].step[1], &eb);
        if (ea > eb + fmaxf(fabsf(da.x), fabsf(da.y)) * 1e-6f + 1e-7f)
            budget_holds = 0;

        int payload = 0;
        for (int k = 0; k < FIELD_N; k++)
            if (ap[i].mask & (1u << k)) payload += ap[i].bits[k];
        printf("%-3u %-6.1f %-5.2f %-8.4f %-8.4f %-8d 0x%x   %s\n",
               ent[i].id, dist[i], vis[i], ap[i].step[0], ap[i].step[1],
               payload, ap[i].mask, rc == RET_DEGRADED ? "degraded" : "ok");
    }

    double reduction = 100.0 * (double)(fixed_bytes - adapt_bytes)
                       / (double)fixed_bytes;
    float pos_budget = hypotf(spec[0].err_budget, spec[1].err_budget);
    int sent = n_ok + n_deg;
    printf("\nfixed:   %zu bytes (%zu bits) over %d entities\n",
           fixed_bytes, fixed_bits, sent);
    printf("adaptive: %zu bytes (%zu bits), ok=%d degraded=%d skipped=%d\n",
           adapt_bytes, adapt_bits, n_ok, n_deg, n_skip);
    printf("bw_reduction_pct=%.2f\n", reduction);
    printf("pos_err fixed:    max=%.5f mean=%.5f\n",
           fixed_err_max, fixed_err_sum / sent);
    printf("pos_err adaptive: max=%.5f mean=%.5f (budget=%.5f)\n",
           adapt_err_max, adapt_err_sum / sent, pos_budget);

    check("all_accounted", n_ok + n_deg + n_skip == NENT);
    check("no_skips_in_soft_budget", n_skip == 0);
    check("some_degraded", n_deg > 0);
    check("bandwidth_reduced", reduction > 0.0);
    check("adaptive_budget_holds", budget_holds && adapt_err_max <= pos_budget + 1e-6f);
    check("fixed_steps_never_coarser", finer_holds);

    /* tight budget: far/cheap entities still send, near/expensive ones skip */
    int skip_n = 0, sent_n = 0, sent_within_budget = 1;
    for (int i = 0; i < NENT; i++) {
        AdaptCtx c = { vis[i], dist[i] };
        EntPlan q;
        int rc = plan_entity(&q, spec, &c, base_step, 24, d_ref);
        if (rc == RET_SKIP) { skip_n++; continue; }
        sent_n++;
        float eb = 0.0f;
        position_error_bound(q.step[0], q.step[1], &eb);
        if (eb > pos_budget + 1e-6f) sent_within_budget = 0;
    }
    check("tight_budget_mixed", skip_n > 0 && sent_n > 0);
    check("tight_budget_no_violation", sent_within_budget);

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
