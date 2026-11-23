/*
 * minilab.h - canonical packet-body serialization shared by:
 *   1. accept/tools/capture_*.c  (built against PRISTINE git-HEAD sources)
 *   2. accept/accept.c           (built against refactored module objects)
 *
 * Both phases must serialize identical inputs through these exact helpers,
 * so any success-path byte drift shows up as a golden hash mismatch.
 *
 * Fixed-width, little-endian, no padding: every multi-byte field is emitted
 * byte by byte; floats/doubles are copied from their in-memory IEEE-754
 * representation. Same compiler/ABI on both phases => identical bytes.
 */
#ifndef MINILAB_H
#define MINILAB_H

#include <stdint.h>
#include <string.h>

typedef struct {
    unsigned char data[256];
    int len;
} MLBody;

static inline void ml_reset(MLBody* b) { b->len = 0; }

static inline void ml_u8(MLBody* b, uint8_t v) {
    b->data[b->len++] = v;
}

static inline void ml_u32(MLBody* b, uint32_t v) {
    b->data[b->len++] = (unsigned char)(v & 0xff);
    b->data[b->len++] = (unsigned char)((v >> 8) & 0xff);
    b->data[b->len++] = (unsigned char)((v >> 16) & 0xff);
    b->data[b->len++] = (unsigned char)((v >> 24) & 0xff);
}

static inline void ml_i32(MLBody* b, int32_t v) {
    ml_u32(b, (uint32_t)v);
}

static inline void ml_f32(MLBody* b, float f) {
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    ml_u32(b, u);
}

static inline void ml_f64(MLBody* b, double d) {
    uint64_t u;
    memcpy(&u, &d, sizeof(u));
    for (int i = 0; i < 8; i++)
        b->data[b->len++] = (unsigned char)((u >> (8 * i)) & 0xff);
}

static inline void ml_bytes(MLBody* b, const unsigned char* p, int n) {
    memcpy(b->data + b->len, p, (size_t)n);
    b->len += n;
}

#endif /* MINILAB_H */
