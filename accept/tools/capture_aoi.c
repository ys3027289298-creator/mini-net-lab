/*
 * capture_aoi.c - canonical success-path body for aoi.
 * Two build modes; see capture_snapshot.c for the contract.
 * Body = serialized world state (entities) plus one radius-query result.
 */
#include <stdio.h>

#include "../minilab.h"

#if defined(ML_PRISTINE)
#  define main minilab_pristine_main
#  include ML_PRISTINE_SRC
#  undef main
#  define CALL_INIT(w,c,r)     world_init((w), (c), (r))
#  define CALL_ADD(w,id,x,y)   world_add((w), (id), (x), (y))
#  define CALL_QUERY(w,s,x,y,o,c) world_query((w), (s), (x), (y), (o), (c))
#else
#  include "aoi.h"
#  define CALL_INIT(w,c,r)     aoi_world_init((w), (c), (r))
#  define CALL_ADD(w,id,x,y)   aoi_world_add((w), (id), (x), (y))
#  define CALL_QUERY(w,s,x,y,o,c) aoi_world_query((w), (s), (x), (y), (o), (c))
#endif

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    World w;
    CALL_INIT(&w, 10.0f, 15.0f);
    if (CALL_ADD(&w, 1, 0, 0) != 0) { fprintf(stderr, "capture_aoi: add1\n"); return 1; }
    if (CALL_ADD(&w, 2, 5, 0) != 1) { fprintf(stderr, "capture_aoi: add2\n"); return 1; }
    if (CALL_ADD(&w, 3, 100, 100) != 2) { fprintf(stderr, "capture_aoi: add3\n"); return 1; }

    int out[8];
    int n = CALL_QUERY(&w, 1, 0, 0, out, 8);

    MLBody b;
    ml_reset(&b);
    ml_i32(&b, w.n);
    for (int i = 0; i < w.n; i++) {
        ml_i32(&b, w.e[i].id);
        ml_f32(&b, w.e[i].x);
        ml_f32(&b, w.e[i].y);
        ml_u8(&b, (uint8_t)w.e[i].alive);
    }
    ml_i32(&b, n);
    for (int i = 0; i < n; i++) ml_i32(&b, out[i]);

    FILE* f = fopen(argv[1], "wb");
    if (!f) { perror("fopen"); return 2; }
    fwrite(b.data, 1, (size_t)b.len, f);
    fclose(f);
    printf("aoi len=%d\n", b.len);
    return 0;
}
