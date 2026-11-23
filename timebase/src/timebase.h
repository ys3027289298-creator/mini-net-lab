
/* timebase: one monotonic time base shared by snapshot / clock / aoi / reconcile.
 *
 * Units are whole milliseconds (TIME_UNITS_PER_SEC == 1000), stored as
 * uint32_t.  The base keeps a high-water tick: it only ever moves forward.
 * Each module feeds ticks originating from its own source; the base records
 * per-module accept / lag / reject counters so a non-conforming source can be
 * attributed to the module that observed it.
 *
 * Rules, all enforced by tb_feed() / tb_read():
 *   - a tick below the module's own last accepted tick is a reversal
 *     (TB_ERR_BACKWARD) and is rejected;
 *   - a tick below the shared base but at or above the module's own last is a
 *     lagging observation: accepted, base unchanged, counted as a lag;
 *   - a tick above TIME_MAX_TICK is out of range (TB_ERR_RANGE);
 *   - a NULL source, or a source whose readings decrease, is TB_ERR_NULL_SOURCE
 *     / TB_ERR_BACKWARD on read, attributed to the reading module.
 *
 * No system-clock API is used: everything is driven by injected samples, so
 * acceptance runs are deterministic and independent of wall-clock time.
 */
#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

#define TIME_UNITS_PER_SEC 1000u

/* Kept below 2^31 so ticks stay exact as doubles and below the signed range
 * used by the modules; 2^31-1 ms is ~24.8 days of simulated time. */
#define TIME_MAX_TICK 0x7fffffffu

typedef enum {
    TB_SNAPSHOT = 0,
    TB_CLOCK,
    TB_AOI,
    TB_RECONCILE,
    TB_MODULE_MAX
} TBModule;

typedef enum {
    TB_OK = 0,
    TB_OK_LAGGING = 1,
    TB_ERR_NULL_SOURCE = -1,
    TB_ERR_BACKWARD = -2,
    TB_ERR_RANGE = -3,
    TB_ERR_ARG = -4
} tb_status;

typedef struct {
    const char* name;
    const char* source;
    uint32_t last;      /* last tick accepted from this module */
    int has_last;
    uint64_t accepts;
    uint64_t lags;
    uint64_t rejects;
    tb_status last_err;
} TBModuleStat;

/* Injected time source.  Readings are raw ticks; they may repeat but must
 * never decrease. */
typedef uint32_t (*tb_read_fn)(void* ctx);

typedef struct {
    uint32_t base;      /* shared high-water tick; never decreases */
    int has_base;
    TBModuleStat mod[TB_MODULE_MAX];
    tb_read_fn read;
    void* ctx;
    int src_set;
    uint64_t reads;
    uint64_t read_errors;
} TimeBase;

void tb_init(TimeBase* tb);
void tb_set_source(TimeBase* tb, tb_read_fn read, void* ctx);

tb_status tb_feed(TimeBase* tb, TBModule who, uint32_t raw);
tb_status tb_read(TimeBase* tb, TBModule who, uint32_t* out);
tb_status tb_now(const TimeBase* tb, uint32_t* out);

const TBModuleStat* tb_stat(const TimeBase* tb, TBModule who);
const char* tb_module_name(TBModule who);
const char* tb_module_source(TBModule who);
const char* tb_status_name(tb_status st);

/* Fixed-point drift helpers: drift = actual - nominal, both in ticks. */
int64_t tb_drift_units(uint32_t actual, uint32_t nominal);
int tb_drift_ppm(int64_t drift_units, int64_t span_units);

#endif
