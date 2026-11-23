/*
 * recording.c - read/write the shared replay recording.
 */
#include <stdio.h>
#include <string.h>
#include "recording.h"

static void put_u32(unsigned char* b, uint32_t v) {
    b[0] = (unsigned char)(v & 0xff); b[1] = (unsigned char)((v >> 8) & 0xff);
    b[2] = (unsigned char)((v >> 16) & 0xff); b[3] = (unsigned char)((v >> 24) & 0xff);
}
static uint32_t get_u32(const unsigned char* b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

int recording_write(const char* path, const Recording* rec) {
    FILE* f = fopen(path, "wb");
    if (!f) return 0;
    unsigned char hdr[16];
    memcpy(hdr, REC_MAGIC, 8);
    put_u32(hdr + 8, rec->count);
    uint32_t sb; memcpy(&sb, &rec->scale, sizeof(sb));
    put_u32(hdr + 12, sb);
    int ok = fwrite(hdr, 1, sizeof(hdr), f) == sizeof(hdr);
    for (uint32_t i = 0; i < rec->count && ok; i++) {
        unsigned char lb[2] = { (unsigned char)(rec->len[i] & 0xff),
                                (unsigned char)(rec->len[i] >> 8) };
        ok = fwrite(lb, 1, 2, f) == 2 &&
             fwrite(rec->pkt[i], 1, rec->len[i], f) == rec->len[i];
    }
    fclose(f);
    return ok;
}

int recording_read(const char* path, Recording* rec) {
    FILE* f = fopen(path, "rb");
    if (!f) return 0;
    unsigned char hdr[16];
    if (fread(hdr, 1, sizeof(hdr), f) != sizeof(hdr)) { fclose(f); return 0; }
    if (memcmp(hdr, REC_MAGIC, 8) != 0) { fclose(f); return 0; }
    memset(rec, 0, sizeof(*rec));
    rec->count = get_u32(hdr + 8);
    uint32_t sb = get_u32(hdr + 12);
    memcpy(&rec->scale, &sb, sizeof(rec->scale));
    if (rec->count > 256) { fclose(f); return 0; }
    int ok = 1;
    for (uint32_t i = 0; i < rec->count && ok; i++) {
        unsigned char lb[2];
        if (fread(lb, 1, 2, f) != 2) { ok = 0; break; }
        rec->len[i] = (uint16_t)(lb[0] | (lb[1] << 8));
        if (rec->len[i] > 64) { ok = 0; break; }
        if (fread(rec->pkt[i], 1, rec->len[i], f) != rec->len[i]) { ok = 0; break; }
    }
    fclose(f);
    return ok;
}
