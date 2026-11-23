/*
 * clock.h - network clock offset estimation with min-RTT filtering.
 *
 * Wire compatibility contract
 * ---------------------------
 * The v1 state body is fixed:
 *
 *   varint n
 *   n x { varint rtt_bits, varint offset_bits, varint at_bits }   (IEEE-754)
 *   varint best_rtt_bits
 *   varint offset_bits
 *
 * A v2 body is that exact byte sequence followed by tagged records.  An old
 * decoder stops after the v1 fields and never looks at the tail, so it
 * observes the same samples, best_rtt and offset as before.
 *
 * The filter fields added by v2 are appended:
 *   tag 0 = version (must be first), tag 1 = filtered (sample count used)
 *
 * Error contract: every failure returns a distinct clock_err_t.  When only
 * the extension is bad, the v1 prefix is still published to *sy and
 * ClockInfo.prefix_ok is 1, so an old-style reader keeps a usable estimate.
 */
#ifndef CLOCK_H
#define CLOCK_H

#define MAX_SAMPLES 16
#define CLOCK_STATE_MAX 1024
#define CLOCK_VERSION_MIN 1u
#define CLOCK_VERSION_MAX 2u

#define CLOCK_TAG_VERSION  0u
#define CLOCK_TAG_FILTERED 1u

/* Single source of truth: the enum, clock_err_name() in the .c file and
 * accept/error_codes all derive from this list. */
#define CLOCK_ERR_LIST(X) \
    X(CLK_OK, 0, "success") \
    X(CLK_ERR_ARG, -20, "null state or buffer") \
    X(CLK_ERR_TIME_ORDER, -21, "t_recv before t_send") \
    X(CLK_ERR_NO_SAMPLES, -22, "filter window empty") \
    X(CLK_ERR_TRUNCATED, -23, "v1 prefix cut inside a varint") \
    X(CLK_ERR_EXT, -24, "malformed v2 extension") \
    X(CLK_ERR_VERSION, -25, "v2 body declares unsupported version") \
    X(CLK_ERR_FULL, -26, "encoded body exceeds caller buffer")

typedef enum {
    CLK_OK = 0,
    CLK_ERR_ARG = -20,
    CLK_ERR_TIME_ORDER = -21,
    CLK_ERR_NO_SAMPLES = -22,
    CLK_ERR_TRUNCATED = -23,
    CLK_ERR_EXT = -24,
    CLK_ERR_VERSION = -25,
    CLK_ERR_FULL = -26
} clock_err_t;


typedef struct { double rtt; double offset; double at; } Sample;

typedef struct {
    Sample s[MAX_SAMPLES];
    int n;
    double best_rtt;
    double offset;
    /* v2-only field; zero keeps the encoding byte-identical to v1 */
    unsigned filtered;
} Sync;

typedef struct {
    clock_err_t err;
    unsigned version;
    int prefix_ok;
    int fields_read;      /* v1 fields applied */
    int ext_fields;       /* v2 fields applied */
    int fields_skipped;   /* unknown tagged fields skipped */
} ClockInfo;

void clock_sync_init(Sync* sy);
/* Returns number of samples averaged (>0) or a negative clock_err_t. */
int  clock_sync_add(Sync* sy, double t_send, double t_recv, double remote_time);

/* State body codec.  Encode emits the bare v1 prefix when last_recv and
 * filtered are zero, otherwise the v1 prefix plus tagged v2 records.
 * Returns bytes written (>=0) or a negative clock_err_t. */
int clock_state_encode(const Sync* sy, unsigned char* buf, int cap);
/* Returns bytes consumed (>=0) or a negative clock_err_t. */
int clock_state_decode(const unsigned char* buf, int len, Sync* sy);
int clock_state_decode_info(const unsigned char* buf, int len, Sync* sy, ClockInfo* info);

/* Single source of truth for the error-code table. */
const char* clock_err_name(int code);

#endif /* CLOCK_H */
