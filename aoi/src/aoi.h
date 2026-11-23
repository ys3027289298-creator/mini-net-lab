/*
 * aoi.h - grid-based area of interest with enter/leave diff and radius query.
 *
 * Failure isolation contract:
 *   - capacity, configuration and lookup failures return named aoi_err_t;
 *   - aoi_inject() arms exactly one fault and it cannot corrupt the world.
 */
#ifndef AOI_H
#define AOI_H

#define GRID 32
#define MAX_ENT 16

typedef enum {
    AOI_OK            = 0,
    AOI_ERR_FULL      = -30,
    AOI_ERR_BAD_CELL  = -31,
    AOI_ERR_DUP_ID    = -32,
    AOI_ERR_NOT_FOUND = -33,
    AOI_ERR_NULL      = -34,
    AOI_ERR_INJECT_ADD = -35
} aoi_err_t;

typedef enum {
    AOI_FAULT_NONE = 0,
    AOI_FAULT_ADD  = 1
} aoi_fault_t;

typedef struct { int id; float x, y; int alive; } Entity;

typedef struct {
    Entity e[MAX_ENT];
    int n;
    float cell;
    float radius;
} World;

void aoi_world_init(World* w, float cell, float radius);
/* Returns insertion index (>=0) or a negative aoi_err_t. */
int  aoi_world_add(World* w, int id, float x, float y);
int  aoi_world_move(World* w, int id, float x, float y);
int  aoi_world_remove(World* w, int id);
int  aoi_cell_coord(float v, float cell);
/* Number of matching entities (may exceed cap); ids written only up to cap. */
int  aoi_world_query(const World* w, int self, float x, float y, int* out, int cap);
void aoi_world_diff(const int* prev, int prevn, const int* cur, int curn, int* enter, int* leave);

void aoi_inject(aoi_fault_t f);
void aoi_reset_faults(void);
aoi_fault_t aoi_current_fault(void);

const char* aoi_err_name(int code);
const char* aoi_fault_name(aoi_fault_t f);

#endif /* AOI_H */
