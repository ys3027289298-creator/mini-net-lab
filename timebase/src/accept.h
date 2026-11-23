
/* accept: shared acceptance harness for the four modules.
 *
 * Each module runs the same three phases against its own TimeBase, with all
 * time injected, so results are deterministic and independent of the system
 * clock:
 *   normal   - nominal run; assert monotone, no rejections, report drift
 *   long     - long simulated run; report drift in ms and ppm
 *   bad      - NULL / backward / out-of-range sources; assert each is rejected
 *              and attributed to this module, with peers untouched
 */
#ifndef TIMEBASE_ACCEPT_H
#define TIMEBASE_ACCEPT_H

#include "timebase.h"

typedef struct TBAcceptSpec TBAcceptSpec;

/* Optional per-event hook: lets a module exercise its own API on each tick,
 * driving the module through the same TimeBase.  Return 0 for success; any
 * other value is counted and fails the phase. */
typedef int (*tb_step_fn)(void* ctx, TimeBase* tb, const TBAcceptSpec* spec, uint32_t tick);

/* Optional per-phase reset: called once before a phase, so module state such as
 * a sync ring or world does not leak from one phase into the next. */
typedef void (*tb_reset_fn)(void* ctx);

struct TBAcceptSpec {
    TBModule module;
    uint32_t step_ms;       /* nominal cost of one event */
    uint32_t events;        /* normal phase length */
    uint32_t long_events;   /* long phase length */
    int32_t ppm;            /* simulated source drift, >= 0 */
    uint32_t seed;
    tb_step_fn step;        /* optional; NULL means timebase only */
    tb_reset_fn reset;      /* optional; NULL keeps ctx as-is */
    void* ctx;
};

/* Runs all phases, prints one deterministic line per observation, returns 0
 * when every check passed. */
int tb_accept_run(const TBAcceptSpec* spec);

/* Shared-phase parameters.  Every module runs this identical block, so the
 * resulting base tick must match across modules: it proves the four modules
 * are driven by one time base and not four drifting ones. */
#define TB_SHARED_STEP_MS 10u
#define TB_SHARED_EVENTS  720000u   /* 2 h */
#define TB_SHARED_PPM     100
#define TB_SHARED_SEED    0x5eedu

/* Feeds the shared block and returns the final base tick.  Also prints a
 * phase=shared row so scripted cross-module comparison is possible. */
uint32_t tb_accept_shared(TBModule module);

#endif
