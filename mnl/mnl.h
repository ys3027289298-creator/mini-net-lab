/*
 * mnl.h - unified failure reporting and zero-copy batch runners for the
 * mini-net-lab modules (snapshot / clock / aoi / reconcile).
 *
 * Design contract
 * ---------------
 * 1. Every validation site reports through the same sink and produces the
 *    same record shape: (module id, field id, reason code, entry index).
 *    A failure is local to one entry: the runner keeps processing the rest.
 * 2. Batch runners never copy payloads. Caller-owned payload structs are
 *    passed by pointer and touched in place; the only writes are result
 *    metadata and the caller-provided output buffers (e.g. packet bytes).
 * 3. State storage is caller-owned, sized by MNL_STATE_WORDS; the modules
 *    never allocate.
 */
#ifndef MNL_H
#define MNL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Functions used only by the legacy self-test mains become unused when the
 * library is built with -DMNL_NO_MAIN. */
#if defined(__GNUC__) || defined(__clang__)
#define MNL_MAYBE_UNUSED __attribute__((unused))
#else
#define MNL_MAYBE_UNUSED
#endif

/* ---- modules ----------------------------------------------------------- */

typedef enum {
    MNL_MOD_SNAPSHOT  = 1,
    MNL_MOD_CLOCK     = 2,
    MNL_MOD_AOI       = 3,
    MNL_MOD_RECONCILE = 4
} mnl_module_t;

/* ---- fields (flat namespace; module id disambiguates) ------------------ */

typedef enum {
    MNL_F_NONE = 0,
    /* snapshot */
    MNL_F_SNAP_STATE,
    MNL_F_SNAP_PACKET,
    MNL_F_SNAP_OUT,
    MNL_F_POS_SCALE,
    MNL_F_TICK,
    MNL_F_X,
    MNL_F_Y,
    MNL_F_PACKET_BYTES,
    /* clock */
    MNL_F_SYNC,
    MNL_F_SAMPLE,
    MNL_F_T_SEND,
    MNL_F_T_RECV,
    MNL_F_REMOTE_TIME,
    /* aoi */
    MNL_F_WORLD,
    MNL_F_CONFIG,
    MNL_F_CELL,
    MNL_F_RADIUS,
    MNL_F_ID,
    MNL_F_POINT,
    MNL_F_QUERY,
    MNL_F_CAP,
    MNL_F_PREV,
    MNL_F_CUR,
    MNL_F_DIFF,
    /* reconcile */
    MNL_F_RECON,
    MNL_F_RECON_POINT
} mnl_field_t;

/* ---- failure reasons --------------------------------------------------- */

typedef enum {
    MNL_R_NONE = 0,
    MNL_R_NULL,        /* required pointer was NULL                 */
    MNL_R_RANGE,       /* value outside its allowed domain          */
    MNL_R_NONFINITE,   /* float/double was NaN or infinity          */
    MNL_R_DUPLICATE,   /* id/tick already present                    */
    MNL_R_NOT_FOUND,   /* id/tick not present                        */
    MNL_R_FULL,        /* capacity exhausted                         */
    MNL_R_PARSE,       /* wire bytes cannot be parsed                */
    MNL_R_BAD_OP       /* unknown opcode                             */
} mnl_reason_t;

/* ---- failure record and sink ------------------------------------------ */

#define MNL_ERR_CAP 64

typedef struct {
    mnl_module_t module;
    mnl_field_t  field;
    mnl_reason_t reason;
    int          entry_index;   /* position in the batch, -1 outside one */
} mnl_err_t;

typedef struct {
    mnl_err_t errs[MNL_ERR_CAP];
    int       count;            /* attempts to write past cap are dropped */
    int       cur_index;        /* stamped onto every record while != -1  */
} mnl_sink_t;

void mnl_sink_init(mnl_sink_t* sk);
void mnl_sink_begin(mnl_sink_t* sk, int entry_index);

/* Low-level report; sk == NULL is a no-op so primitives stay callable
 * without a reporter. Returns -1 so call sites can `return mnl_report(...)`. */
int mnl_report(mnl_sink_t* sk, mnl_module_t mod, mnl_field_t field,
               mnl_reason_t reason);

const char* mnl_module_name(mnl_module_t m);
const char* mnl_field_name(mnl_field_t f);
const char* mnl_reason_name(mnl_reason_t r);
/* Human-readable "module/field/reason" line into buf; never fails. */
void mnl_format(const mnl_err_t* e, char* buf, size_t n);

/* ---- caller-owned state ------------------------------------------------ */

typedef unsigned long mnl_state_align_t;
#define MNL_STATE_WORDS 96   /* 96 * 8 = 768 bytes, fits every module state */

typedef struct {
    mnl_state_align_t words[MNL_STATE_WORDS];
} mnl_state_t;

/* ---- payload carriers (caller-owned; passed by pointer, never copied) -- */

typedef struct {
    unsigned tick;
    float    x, y, vx, vy;
    unsigned flags;
} mnl_snap_state_t;

#define MNL_SNAP_MAX 64

typedef struct {
    unsigned char bytes[MNL_SNAP_MAX];
    int           len;
} mnl_packet_t;

typedef struct {
    double t_send;
    double t_recv;
    double remote_time;
} mnl_clock_sample_t;

typedef struct {
    float cell;
    float radius;
} mnl_aoi_config_t;

typedef struct {
    int   id;
    float x, y;
} mnl_point_t;

typedef struct {
    const int* ids;
    int        n;
} mnl_idset_t;

typedef struct {
    unsigned tick;
    float    x;
} mnl_recon_point_t;

/* ---- batch item: one entry, validated in isolation --------------------- */

typedef struct {
    int             op;        /* module-specific opcode, see enums below */
    const void*     payload;   /* carrier for this entry, never copied    */
    void*           out;       /* caller-provided output buffer, if any   */
    int             cap;       /* output capacity (elements), if relevant */
    float           scale;     /* snapshot pos_scale                      */
    int             id;        /* aoi self/explicit id                    */
} mnl_item_t;

typedef struct {
    int  rc;                  /* primitive return value, -1 on failure */
    int  ival;                /* integer result (counts, indices...)   */
    int  ival2;               /* second integer result (aoi diff leave)*/
    float fval;               /* float result, if any                  */
    mnl_err_t err;            /* filled when rc < 0                    */
} mnl_result_t;

/* ---- opcodes ----------------------------------------------------------- */

enum {
    MNL_SNAP_ENCODE = 1,
    MNL_SNAP_DECODE
};

enum {
    MNL_CLOCK_ADD = 1
};

enum {
    MNL_AOI_INIT = 1,
    MNL_AOI_ADD,
    MNL_AOI_MOVE,
    MNL_AOI_REMOVE,
    MNL_AOI_QUERY,
    MNL_AOI_DIFF
};

enum {
    MNL_RECON_PUSH = 1,
    MNL_RECON_ACK,
    MNL_RECON_STATS
};

/* ---- batch runners: one module each, shared record shape --------------- */

/* Each entry is validated independently. A failed entry records one error
 * (queryable in res->err and appended to sk) and never mutates module state;
 * subsequent entries are still processed. Returns the number of failures. */
int mnl_snapshot_run(mnl_state_t* st, mnl_sink_t* sk,
                     const mnl_item_t* items, int n, mnl_result_t* res);
int mnl_clock_run(mnl_state_t* st, mnl_sink_t* sk,
                  const mnl_item_t* items, int n, mnl_result_t* res);
int mnl_aoi_run(mnl_state_t* st, mnl_sink_t* sk,
                const mnl_item_t* items, int n, mnl_result_t* res);
int mnl_recon_run(mnl_state_t* st, mnl_sink_t* sk,
                  const mnl_item_t* items, int n, mnl_result_t* res);

#ifdef __cplusplus
}
#endif

#endif /* MNL_H */
