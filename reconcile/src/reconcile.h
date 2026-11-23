/*
 * reconcile.h - prediction buffer with rollback on authoritative state.
 *
 * State-body compatibility contract
 * ---------------------------------
 * The v1 canonical body carries exactly the fields the 0bdd277 struct has:
 *
 *   varint count
 *   count x { varint tick, varint x_bits }   (oldest to newest, x raw IEEE)
 *   varint acked
 *   varint rollbacks
 *   varint last_err_bits                     (raw IEEE, exact incl. -1.0f)
 *
 * A v2 body is those exact bytes followed by tagged records.  An old decoder
 * stops after last_err and sees every baseline field unchanged.  v2 appends:
 *   tag 0 = version (must be first), tag 1 = ack_hits (new lifetime counter)
 *
 * Error contract: every failure returns a distinct recon_err_t and leaves the
 * buffer untouched.  When only the extension is bad, the v1 prefix is still
 * published and ReconInfo.prefix_ok is 1.
 */
#ifndef RECONCILE_H
#define RECONCILE_H

#define BUF 64
#define RECON_STATE_MAX 1024
#define RECON_VERSION_MIN 1u
#define RECON_VERSION_MAX 2u

#define RECON_TAG_VERSION 0u
#define RECON_TAG_ACK_HITS 1u

/* Single source of truth: the enum, recon_err_name() in the .c file and
 * accept/error_codes all derive from this list. */
#define RECON_ERR_LIST(X) \
    X(RECON_OK, 0, "success") \
    X(RECON_ERR_ARG, -40, "null buffer or state") \
    X(RECON_ERR_STALE, -41, "tick <= highest acknowledged") \
    X(RECON_ERR_DUP, -42, "tick already present in buffer") \
    X(RECON_ERR_TRUNCATED, -43, "v1 body cut short") \
    X(RECON_ERR_EXT, -44, "malformed v2 extension") \
    X(RECON_ERR_VERSION, -45, "unsupported v2 version") \
    X(RECON_ERR_BUFFER, -46, "encoded body exceeds caller buffer")

typedef enum {
    RECON_OK = 0,
    RECON_ERR_ARG = -40,
    RECON_ERR_STALE = -41,
    RECON_ERR_DUP = -42,
    RECON_ERR_TRUNCATED = -43,
    RECON_ERR_EXT = -44,
    RECON_ERR_VERSION = -45,
    RECON_ERR_BUFFER = -46
} recon_err_t;


typedef struct { unsigned tick; float x; } Pred;

typedef struct {
    Pred p[BUF];
    int head;      /* next write */
    int count;
    unsigned acked;
    int rollbacks;
    float last_err;
    /* v2-only; zero keeps the v1 body byte-identical */
    unsigned ack_hits;
} Recon;

typedef struct {
    recon_err_t err;
    unsigned version;
    int prefix_ok;
    int predictions;
    int fields_skipped;
} ReconInfo;

void recon_init(Recon* r);
/* Returns 0 on insert or a negative recon_err_t. */
int  recon_push(Recon* r, unsigned tick, float x);
/* Returns 1 if a prediction matched the tick, 0 if the tick was unknown. */
int  recon_ack(Recon* r, unsigned tick, float x);

/* Canonical state body, oldest prediction first. */
int  recon_state_encode(const Recon* r, unsigned char* buf, int cap);
int  recon_state_decode(const unsigned char* buf, int len, Recon* r);
int  recon_state_decode_info(const unsigned char* buf, int len, Recon* r, ReconInfo* info);

/* Single source of truth for the error-code table. */
const char* recon_err_name(int code);

#endif /* RECONCILE_H */
