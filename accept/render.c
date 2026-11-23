/*
 * render.c - deterministic framebuffer + SHA-256 (FIPS 180-4), self-contained
 * so the pixel-identity evidence has no dependency beyond libc.
 */
#include <string.h>
#include "render.h"

typedef struct {
    unsigned int h[8];
    unsigned long long bits;
    unsigned char buf[64];
    size_t n;
} Sha256;

static const unsigned int K[64] = {
    0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,0x923f82a4u,0xab1c5ed5u,
    0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,
    0xe49b69c1u,0xefbe4786u,0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
    0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,0x06ca6351u,0x14292967u,
    0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,
    0xa2bfe8a1u,0xa81a664bu,0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
    0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,0x5b9cca4fu,0x682e6ff3u,
    0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u,
};

static unsigned int ror(unsigned int x, int n) { return (x >> n) | (x << (32 - n)); }

static void sha_block(Sha256* s, const unsigned char* p) {
    unsigned int w[64];
    for (int i = 0; i < 16; i++)
        w[i] = ((unsigned int)p[i*4] << 24) | ((unsigned int)p[i*4+1] << 16) |
               ((unsigned int)p[i*4+2] << 8) | (unsigned int)p[i*4+3];
    for (int i = 16; i < 64; i++) {
        unsigned int s0 = ror(w[i-15], 7) ^ ror(w[i-15], 18) ^ (w[i-15] >> 3);
        unsigned int s1 = ror(w[i-2], 17) ^ ror(w[i-2], 19) ^ (w[i-2] >> 10);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
    }
    unsigned int a=s->h[0],b=s->h[1],c=s->h[2],d=s->h[3],e=s->h[4],f=s->h[5],g=s->h[6],h=s->h[7];
    for (int i = 0; i < 64; i++) {
        unsigned int S1 = ror(e,6) ^ ror(e,11) ^ ror(e,25);
        unsigned int ch = (e & f) ^ (~e & g);
        unsigned int t1 = h + S1 + ch + K[i] + w[i];
        unsigned int S0 = ror(a,2) ^ ror(a,13) ^ ror(a,22);
        unsigned int mj = (a & b) ^ (a & c) ^ (b & c);
        unsigned int t2 = S0 + mj;
        h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
    }
    s->h[0]+=a; s->h[1]+=b; s->h[2]+=c; s->h[3]+=d;
    s->h[4]+=e; s->h[5]+=f; s->h[6]+=g; s->h[7]+=h;
}

static void sha_init(Sha256* s) {
    static const unsigned int iv[8] = {
        0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,
        0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u };
    memcpy(s->h, iv, sizeof(iv));
    s->bits = 0; s->n = 0;
}

static void sha_update(Sha256* s, const unsigned char* p, size_t len) {
    s->bits += (unsigned long long)len * 8u;
    while (len) {
        size_t take = 64 - s->n;
        if (take > len) take = len;
        memcpy(s->buf + s->n, p, take);
        s->n += take; p += take; len -= take;
        if (s->n == 64) { sha_block(s, s->buf); s->n = 0; }
    }
}

static void sha_final(Sha256* s, unsigned char out[32]) {
    unsigned long long bits = s->bits;
    unsigned char pad = 0x80;
    sha_update(s, &pad, 1);
    unsigned char z = 0;
    while (s->n != 56) sha_update(s, &z, 1);
    unsigned char lenb[8];
    for (int i = 0; i < 8; i++) lenb[i] = (unsigned char)(bits >> (56 - 8*i));
    sha_update(s, lenb, 8);
    for (int i = 0; i < 8; i++) {
        out[i*4]   = (unsigned char)(s->h[i] >> 24);
        out[i*4+1] = (unsigned char)(s->h[i] >> 16);
        out[i*4+2] = (unsigned char)(s->h[i] >> 8);
        out[i*4+3] = (unsigned char)(s->h[i]);
    }
}

static void sha_hex(const unsigned char* p, size_t n, char hex[65]) {
    Sha256 s; sha_init(&s);
    sha_update(&s, p, n);
    unsigned char d[32]; sha_final(&s, d);
    static const char* hx = "0123456789abcdef";
    for (int i = 0; i < 32; i++) { hex[i*2] = hx[d[i] >> 4]; hex[i*2+1] = hx[d[i] & 15]; }
    hex[64] = 0;
}

void frame_hash(const Frame* f, char hex[65]) {
    sha_hex((const unsigned char*)f->px, sizeof(f->px), hex);
}

void bytes_hash(const unsigned char* p, int n, char hex[65]) {
    sha_hex(p, (size_t)(n < 0 ? 0 : n), hex);
}

void fb_clear(Frame* f, unsigned char r, unsigned char g, unsigned char b) {
    for (int y = 0; y < FB_H; y++)
        for (int x = 0; x < FB_W; x++) {
            f->px[y][x][0] = r; f->px[y][x][1] = g; f->px[y][x][2] = b;
        }
}

void fb_rect(Frame* f, int x, int y, int w, int h,
             unsigned char r, unsigned char g, unsigned char b) {
    for (int j = y; j < y + h; j++) {
        if (j < 0 || j >= FB_H) continue;
        for (int i = x; i < x + w; i++) {
            if (i < 0 || i >= FB_W) continue;
            f->px[j][i][0] = r; f->px[j][i][1] = g; f->px[j][i][2] = b;
        }
    }
}

/* integer-only Bresenham, no floating point anywhere */
void fb_line(Frame* f, int x0, int y0, int x1, int y1,
             unsigned char r, unsigned char g, unsigned char b) {
    int dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int dy = y1 > y0 ? y1 - y0 : y0 - y1;
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    for (;;) {
        if (x0 >= 0 && x0 < FB_W && y0 >= 0 && y0 < FB_H) {
            f->px[y0][x0][0] = r; f->px[y0][x0][1] = g; f->px[y0][x0][2] = b;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

int frame_diff(const Frame* a, const Frame* b) {
    int d = 0;
    const unsigned char* pa = (const unsigned char*)a->px;
    const unsigned char* pb = (const unsigned char*)b->px;
    for (size_t i = 0; i < sizeof(a->px); i++) if (pa[i] != pb[i]) d++;
    return d;
}
