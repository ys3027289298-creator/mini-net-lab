/*
 * snapshot.h - quantized state snapshot codec, public interface.
 *
 * Failure isolation contract:
 *   - every failure returns a distinct negative snap_err_t (never crashes);
 *   - snap_inject() arms exactly one fault, snap_reset_faults() clears it;
 *   - an armed fault affects this module's call only, never its input buffers.
 */
#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#define SNAP_MAX 64

typedef enum {
    SNAP_OK              = 0,
    SNAP_ERR_PACKET_NULL = -10,
    SNAP_ERR_BAD_SCALE   = -11,
    SNAP_ERR_STATE_NULL  = -12,
    SNAP_ERR_TRUNCATED   = -13,   /* varint runs past packet end */
    SNAP_ERR_INJECT_ENC  = -14,   /* injected failure inside encode */
    SNAP_ERR_INJECT_DEC  = -15    /* injected failure inside decode */
} snap_err_t;

typedef enum {
    SNAP_FAULT_NONE = 0,
    SNAP_FAULT_ENCODE = 1,
    SNAP_FAULT_DECODE = 2
} snap_fault_t;

typedef struct {
    unsigned tick;
    float x, y, vx, vy;
    unsigned flags;
} State;

typedef struct {
    unsigned char buf[SNAP_MAX];
    int len;
} Packet;

/* Returns bytes written (>=1) or a negative snap_err_t. */
int  snap_encode(const State* s, Packet* p, float pos_scale);
/* Returns bytes consumed (>=1) or a negative snap_err_t. */
int  snap_decode(const Packet* p, State* s, float pos_scale);

/* Fault injection, deterministic, process-local. */
void snap_inject(snap_fault_t f);
void snap_reset_faults(void);
snap_fault_t snap_current_fault(void);

/* Single source of truth for the error-code table. */
const char* snap_err_name(int code);
const char* snap_fault_name(snap_fault_t f);

#endif /* SNAPSHOT_H */
