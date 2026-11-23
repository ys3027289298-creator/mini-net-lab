/*
 * snapshot.h - quantized state snapshot codec, public interface.
 *
 * Wire compatibility contract
 * ---------------------------
 * A packet is the immutable v1 prefix followed by an optional extension:
 *
 *   v1 prefix : varint tick, zigzag-varint qx, zigzag-varint qy
 *   extension : { varint tag, varint payload_len, payload } ...
 *
 * The prefix bytes are exactly the 0bdd277 ("v1") layout.  New fields are
 * appended as tagged records only; they are never inserted before or between
 * the prefix fields.  A v1 client reads the three prefix varints and ignores
 * everything after them, so it always decodes the same tick/x/y as before.
 * That is what stops old clients from rendering jumping positions.
 *
 * A v2 packet with flags == 0 emits no extension and is byte-identical to a
 * v1 packet.  A v1 packet with no trailing bytes decodes as version 1.
 *
 * Extension tags:
 *   0 = format version  (payload: varint, must be first when present)
 *   1 = flags           (payload: varint)
 * Unknown tags are skipped by length and counted in SnapInfo, so a future
 * writer can add fields without breaking this decoder.
 *
 * Error contract: no failure aborts.  Each failure returns a distinct
 * snap_err_t; whenever the v1 prefix itself decoded, SnapInfo.prefix_ok is 1
 * and *s holds the prefix state even if the extension was rejected.  Callers
 * that only care about position (renderers, legacy recordings) can therefore
 * keep drawing an authoritative prefix instead of jumping to a default pose.
 */
#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#define SNAP_MAX 64
#define SNAP_VERSION_MIN 1u
#define SNAP_VERSION_MAX 2u

/* Extension tags; appended after the v1 prefix, never inside it. */
#define SNAP_TAG_VERSION 0u
#define SNAP_TAG_FLAGS   1u


/* Single source of truth: the enum, the enum->name table in the .c file
 * and accept/error_codes all derive from this list. */
#define SNAP_ERR_LIST(X) \
    X(SNAP_OK, 0, "success") \
    X(SNAP_ERR_ARG, -10, "null packet or state") \
    X(SNAP_ERR_BAD_SCALE, -11, "pos_scale <= 0") \
    X(SNAP_ERR_TRUNCATED, -12, "v1 prefix cut inside a varint") \
    X(SNAP_ERR_EXT_TRUNCATED, -13, "extension record cut short") \
    X(SNAP_ERR_EXT_LEN, -14, "extension record length invalid") \
    X(SNAP_ERR_VERSION_RANGE, -15, "extension version outside supported range") \
    X(SNAP_ERR_BUFFER_FULL, -16, "encoded packet exceeds SNAP_MAX")

typedef enum {
    SNAP_OK = 0,
    SNAP_ERR_ARG = -10,
    SNAP_ERR_BAD_SCALE = -11,
    SNAP_ERR_TRUNCATED = -12,
    SNAP_ERR_EXT_TRUNCATED = -13,
    SNAP_ERR_EXT_LEN = -14,
    SNAP_ERR_VERSION_RANGE = -15,
    SNAP_ERR_BUFFER_FULL = -16
} snap_err_t;



typedef struct {
    unsigned tick;
    float x, y, vx, vy;
    unsigned flags;
} State;

typedef struct {
    unsigned char buf[SNAP_MAX];
    int len;
} Packet;

typedef struct {
    snap_err_t err;
    unsigned version;       /* 1 = bare prefix, 2 = extension understood */
    int prefix_ok;          /* v1 tick/x/y decoded; *s is usable */
    int fields_read;        /* prefix fields + extension fields applied */
    int fields_skipped;     /* unknown tagged fields skipped */
    int bytes_skipped;
    int flags_present;      /* flags record was seen and applied */
} SnapInfo;

/* Returns bytes written (>=1) or a negative snap_err_t.
 * Emits the bare v1 prefix when s->flags == 0. */
int snap_encode(const State* s, Packet* p, float pos_scale);

/* Returns bytes consumed (>=1) or a negative snap_err_t. Fills s even when
 * only the prefix decoded (see SnapInfo for the richer view). */
int snap_decode(const Packet* p, State* s, float pos_scale);

/* As snap_decode, and always fills info (except when p or s is NULL). */
int snap_decode_info(const Packet* p, State* s, float pos_scale, SnapInfo* info);

/* Single source of truth for the error-code table. */
const char* snap_err_name(int code);

#endif /* SNAPSHOT_H */
