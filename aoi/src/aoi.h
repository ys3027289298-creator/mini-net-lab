/*
 * aoi.h - grid-based area of interest with enter/leave diff and radius query.
 *
 * State-body compatibility contract
 * ---------------------------------
 * The v1 canonical body is:
 *
 *   varint n
 *   n x { zigzag-varint id, varint x_bits, varint y_bits, u8 alive }
 *   varint cell_bits
 *   varint radius_bits
 *
 * where *_bits are raw IEEE-754 float bits.  A v2 body is exactly those bytes
 * followed by tagged records; old decoders stop after the v1 fields and see
 * identical entities and configuration.  v2 appends:
 *   tag 0 = version (must be first), tag 1 = rev (world revision)
 *
 * Error contract: every failure returns a distinct aoi_err_t and never
 * mutates the world.  When only the extension is bad, the v1 prefix is still
 * published and AoiInfo.prefix_ok is 1.
 */
#ifndef AOI_H
#define AOI_H

#define GRID 32
#define MAX_ENT 16
#define AOI_STATE_MAX 1024
#define AOI_VERSION_MIN 1u
#define AOI_VERSION_MAX 2u

#define AOI_TAG_VERSION 0u
#define AOI_TAG_REV     1u

/* Single source of truth: the enum, aoi_err_name() in the .c file and
 * accept/error_codes all derive from this list. */
#define AOI_ERR_LIST(X) \
    X(AOI_OK, 0, "success") \
    X(AOI_ERR_FULL, -30, "entity table at capacity") \
    X(AOI_ERR_BAD_CELL, -31, "cell size <= 0") \
    X(AOI_ERR_DUP_ID, -32, "duplicate entity id") \
    X(AOI_ERR_NOT_FOUND, -33, "entity id not present") \
    X(AOI_ERR_ARG, -34, "null world or buffer") \
    X(AOI_ERR_TRUNCATED, -35, "v1 body cut short") \
    X(AOI_ERR_EXT, -36, "malformed v2 extension") \
    X(AOI_ERR_VERSION, -37, "unsupported v2 version") \
    X(AOI_ERR_BUFFER, -38, "encoded body exceeds caller buffer")

typedef enum {
    AOI_OK = 0,
    AOI_ERR_FULL = -30,
    AOI_ERR_BAD_CELL = -31,
    AOI_ERR_DUP_ID = -32,
    AOI_ERR_NOT_FOUND = -33,
    AOI_ERR_ARG = -34,
    AOI_ERR_TRUNCATED = -35,
    AOI_ERR_EXT = -36,
    AOI_ERR_VERSION = -37,
    AOI_ERR_BUFFER = -38
} aoi_err_t;


typedef struct { int id; float x, y; int alive; } Entity;

typedef struct {
    Entity e[MAX_ENT];
    int n;
    float cell;
    float radius;
    unsigned rev;          /* v2-only; zero keeps the v1 body byte-identical */
} World;

typedef struct {
    aoi_err_t err;
    unsigned version;
    int prefix_ok;
    int entities;
    int fields_skipped;
} AoiInfo;

void aoi_world_init(World* w, float cell, float radius);
/* Returns insertion index (>=0) or a negative aoi_err_t. */
int  aoi_world_add(World* w, int id, float x, float y);
int  aoi_world_move(World* w, int id, float x, float y);
int  aoi_world_remove(World* w, int id);
int  aoi_cell_coord(float v, float cell);
/* Number of matching entities (may exceed cap); ids written only up to cap. */
int  aoi_world_query(const World* w, int self, float x, float y, int* out, int cap);
void aoi_world_diff(const int* prev, int prevn, const int* cur, int curn, int* enter, int* leave);

/* Canonical state body.  Returns bytes written (>=0) or a negative aoi_err_t. */
int  aoi_state_encode(const World* w, unsigned char* buf, int cap);
int  aoi_state_decode(const unsigned char* buf, int len, World* w);
int  aoi_state_decode_info(const unsigned char* buf, int len, World* w, AoiInfo* info);

/* Single source of truth for the error-code table. */
const char* aoi_err_name(int code);

#endif /* AOI_H */
