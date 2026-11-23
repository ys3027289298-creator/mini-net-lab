/* timebase: implementation.  Fixed point only; reads no wall clock. */
#include "timebase.h"

#include <stddef.h>

static const char* const k_mod_name[TB_MODULE_MAX] = {
    "snapshot", "clock", "aoi", "reconcile"
};

static const char* const k_mod_source[TB_MODULE_MAX] = {
    "state.tick", "send_clock", "query.tick", "ack.tick"
};

void tb_init(TimeBase* tb) {
    if (tb == NULL) return;
    tb->base = 0;
    tb->has_base = 0;
    tb->read = NULL;
    tb->ctx = NULL;
    tb->src_set = 0;
    tb->reads = 0;
    tb->read_errors = 0;
    for (int i = 0; i < TB_MODULE_MAX; i++) {
        TBModuleStat* m = &tb->mod[i];
        m->name = k_mod_name[i];
        m->source = k_mod_source[i];
        m->last = 0;
        m->has_last = 0;
        m->accepts = 0;
        m->lags = 0;
        m->rejects = 0;
        m->last_err = TB_OK;
    }
}

void tb_set_source(TimeBase* tb, tb_read_fn read, void* ctx) {
    if (tb == NULL) return;
    tb->read = read;
    tb->ctx = ctx;
    tb->src_set = (read != NULL);
}

const char* tb_module_name(TBModule who) {
    if ((unsigned)who >= TB_MODULE_MAX) return "?";
    return k_mod_name[who];
}

const char* tb_module_source(TBModule who) {
    if ((unsigned)who >= TB_MODULE_MAX) return "?";
    return k_mod_source[who];
}

const char* tb_status_name(tb_status st) {
    switch (st) {
    case TB_OK: return "ok";
    case TB_OK_LAGGING: return "lagging";
    case TB_ERR_NULL_SOURCE: return "null_source";
    case TB_ERR_BACKWARD: return "backward";
    case TB_ERR_RANGE: return "range";
    case TB_ERR_ARG: return "arg";
    }
    return "?";
}

const TBModuleStat* tb_stat(const TimeBase* tb, TBModule who) {
    if (tb == NULL || (unsigned)who >= TB_MODULE_MAX) return NULL;
    return &tb->mod[who];
}

static tb_status reject(TimeBase* tb, TBModule who, tb_status st) {
    TBModuleStat* m = &tb->mod[who];
    m->rejects++;
    m->last_err = st;
    return st;
}

/* Feed one tick observed by module `who`.  The shared base is a high-water
 * mark; a module reading below the base but not below its own last accepted
 * tick is a lag, not an error. */
tb_status tb_feed(TimeBase* tb, TBModule who, uint32_t raw) {
    if (tb == NULL) return TB_ERR_ARG;
    if ((unsigned)who >= TB_MODULE_MAX) return TB_ERR_ARG;
    if (raw > TIME_MAX_TICK) return reject(tb, who, TB_ERR_RANGE);

    TBModuleStat* m = &tb->mod[who];
    if (m->has_last && raw < m->last) return reject(tb, who, TB_ERR_BACKWARD);

    tb_status st = TB_OK;
    if (!tb->has_base || raw > tb->base) {
        tb->base = raw;
        tb->has_base = 1;
    } else if (raw < tb->base) {
        st = TB_OK_LAGGING;   /* behind the base, not a reversal */
        m->lags++;
    }                     /* equal to base: a repeat, normal and not a lag */

    m->last = raw;
    m->has_last = 1;
    m->accepts++;
    m->last_err = TB_OK;
    return st;
}

/* Pull one reading from the injected source and feed it as `who`. */
tb_status tb_read(TimeBase* tb, TBModule who, uint32_t* out) {
    if (tb == NULL) return TB_ERR_ARG;
    if ((unsigned)who >= TB_MODULE_MAX) return TB_ERR_ARG;
    if (!tb->src_set || tb->read == NULL) {
        tb->read_errors++;
        return reject(tb, who, TB_ERR_NULL_SOURCE);
    }
    uint32_t raw = tb->read(tb->ctx);
    tb->reads++;
    tb_status st = tb_feed(tb, who, raw);
    if (st < 0) tb->read_errors++;
    if (out != NULL) *out = raw;
    return st;
}

tb_status tb_now(const TimeBase* tb, uint32_t* out) {
    if (tb == NULL || out == NULL) return TB_ERR_ARG;
    if (!tb->has_base) return TB_ERR_NULL_SOURCE;
    *out = tb->base;
    return TB_OK;
}

int64_t tb_drift_units(uint32_t actual, uint32_t nominal) {
    return (int64_t)actual - (int64_t)nominal;
}

/* Drift in parts per million, rounded toward zero; span 0 yields 0. */
int tb_drift_ppm(int64_t drift_units, int64_t span_units) {
    if (span_units <= 0) return 0;
    return (int)((drift_units * 1000000) / span_units);
}
