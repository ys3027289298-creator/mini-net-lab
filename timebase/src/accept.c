
/* accept: shared acceptance harness for the four modules.  See accept.h. */
#include "accept.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

/* Deterministic source: nominal `step_ms` per event, a configured drift in
 * ppm, and small non-negative sampling jitter.  No system clock is read. */
typedef struct {
    uint32_t raw;
    uint32_t step_ms;
    int32_t ppm;
    int64_t drift_acc;     /* micro-ticks not yet emitted */
    int64_t drift_units;   /* pure drift emitted so far */
    int64_t jitter_units;  /* sampling jitter emitted so far */
    uint32_t rng;
    int started;
} DriftSource;

static uint32_t lcg(uint32_t* s) {
    *s = (*s * 1103515245u) + 12345u;
    return *s >> 16;
}

static uint32_t drift_read(void* ctx) {
    DriftSource* d = (DriftSource*)ctx;
    if (!d->started) { d->started = 1; return d->raw; }

    d->raw += d->step_ms;
    uint32_t j = lcg(&d->rng) & 1u;   /* 0 or 1 ms sampling jitter, monotone */
    d->raw += j;
    d->jitter_units += j;

    d->drift_acc += (int64_t)d->step_ms * (int64_t)d->ppm;
    while (d->drift_acc >= 1000000) { d->drift_acc -= 1000000; d->raw += 1; d->drift_units += 1; }
    while (d->drift_acc <= -1000000) { d->drift_acc += 1000000; d->raw -= 1; d->drift_units -= 1; }
    return d->raw;
}

static void emit_row(const TBAcceptSpec* spec, const char* phase, uint32_t events,
                     uint32_t actual, uint32_t nominal, const DriftSource* src,
                     int monotone, uint64_t rejects, int step_errors) {
    int64_t total = (int64_t)actual - (int64_t)nominal;
    int ppm = tb_drift_ppm(src->drift_units, (int64_t)nominal);
    printf("module=%s phase=%s events=%" PRIu32 " step_ms=%" PRIu32
           " nominal_ms=%" PRIu32 " actual_ms=%" PRIu32
           " src_drift_ms=%" PRId64 " jitter_ms=%" PRId64 " drift_ms=%" PRId64
           " drift_ppm=%d monotone=%d rejects=%" PRIu64 " step_errors=%d\n",
           tb_module_name(spec->module), phase, events, spec->step_ms,
           nominal, actual, (int64_t)src->drift_units, (int64_t)src->jitter_units,
           total, ppm, monotone, rejects, step_errors);
}

static int run_phase(const TBAcceptSpec* spec, const char* phase, uint32_t events) {    TimeBase tb;
    tb_init(&tb);
    if (spec->reset != NULL) spec->reset(spec->ctx);
    DriftSource src;
    memset(&src, 0, sizeof(src));
    src.step_ms = spec->step_ms;
    src.ppm = spec->ppm;
    src.rng = spec->seed ? spec->seed : 1u;
    tb_set_source(&tb, drift_read, &src);

    uint32_t prev = 0, cur = 0;
    int monotone = 1;
    uint64_t step_errors = 0;
    for (uint32_t i = 0; i < events; i++) {
        tb_status st = tb_read(&tb, spec->module, &cur);
        if (st == TB_ERR_BACKWARD) monotone = 0;
        if (i > 0 && cur < prev) monotone = 0;
        prev = cur;
        if (spec->step != NULL && spec->step(spec->ctx, &tb, spec, cur) != 0) step_errors++;
    }
    tb_now(&tb, &cur);
    emit_row(spec, phase, events, cur, events * spec->step_ms, &src,
             monotone, tb_stat(&tb, spec->module)->rejects, (int)step_errors);
    return monotone && tb_stat(&tb, spec->module)->rejects == 0 && step_errors == 0;
}

/* Each illegal time source must fail and be charged to this module only. */
static int run_bad(const TBAcceptSpec* spec) {
    TimeBase tb;
    tb_init(&tb);

    int null_ok = tb_read(&tb, spec->module, NULL) == TB_ERR_NULL_SOURCE;
    int backward_ok = tb_feed(&tb, spec->module, 1000) == TB_OK &&
                      tb_feed(&tb, spec->module, 999) == TB_ERR_BACKWARD;
    int range_ok = tb_feed(&tb, spec->module, TIME_MAX_TICK + 1u) == TB_ERR_RANGE;

    const TBModuleStat* m = tb_stat(&tb, spec->module);
    int attr_ok = (m->rejects == 3) && (m->last_err == TB_ERR_RANGE);

    int peers_ok = 1;
    for (int i = 0; i < TB_MODULE_MAX; i++) {
        if (i == (int)spec->module) continue;
        if (tb_stat(&tb, (TBModule)i)->rejects != 0) peers_ok = 0;
    }

    printf("module=%s phase=bad null=%d backward=%d range=%d attr=%d peers=%d\n",
           tb_module_name(spec->module), null_ok, backward_ok, range_ok, attr_ok, peers_ok);
    return null_ok && backward_ok && range_ok && attr_ok && peers_ok;
}

int tb_accept_run(const TBAcceptSpec* spec) {
    if (spec == NULL || (unsigned)spec->module >= TB_MODULE_MAX) return 1;
    if (spec->ppm < 0) return 1;   /* backwards drift is not a valid source */

    int ok = 1;
    ok &= run_phase(spec, "normal", spec->events);
    ok &= run_phase(spec, "long", spec->long_events);
    ok &= run_bad(spec);

    printf("module=%s accept=%d\n", tb_module_name(spec->module), ok ? 1 : 0);
    return ok ? 0 : 1;
}

uint32_t tb_accept_shared(TBModule module) {
    TimeBase tb;
    tb_init(&tb);

    DriftSource src;
    memset(&src, 0, sizeof(src));
    src.step_ms = TB_SHARED_STEP_MS;
    src.ppm = TB_SHARED_PPM;
    src.rng = TB_SHARED_SEED;
    tb_set_source(&tb, drift_read, &src);

    int monotone = 1, errors = 0;
    uint32_t prev = 0, cur = 0;
    for (uint32_t i = 0; i < TB_SHARED_EVENTS; i++) {
        if (tb_read(&tb, module, &cur) < 0) errors++;
        if (i > 0 && cur < prev) monotone = 0;
        prev = cur;
    }
    /* Every module then reports the same tick it just observed.  They must all
     * agree; if one module were on its own clock, these would diverge. */
    uint32_t shared = 0;
    for (int i = 0; i < TB_MODULE_MAX; i++) {
        tb_status st = tb_feed(&tb, (TBModule)i, cur);
        if (st < 0) errors++;
    }
    tb_now(&tb, &shared);

    printf("module=%s phase=shared events=%" PRIu32 " base=%" PRIu32
           " src_drift_ms=%" PRId64 " jitter_ms=%" PRId64
           " nominal_ms=%" PRIu32 " monotone=%d errors=%d\n",
           tb_module_name(module), TB_SHARED_EVENTS, shared,
           (int64_t)src.drift_units, (int64_t)src.jitter_units,
           TB_SHARED_EVENTS * TB_SHARED_STEP_MS, monotone, errors);
    return shared;
}
