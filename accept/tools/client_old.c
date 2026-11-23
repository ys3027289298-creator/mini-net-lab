/*
 * client_old.c - the old client, built ONLY against the pinned 0bdd277 codec.
 *
 * It links no current module code.  It reads a recording, decodes each packet
 * with the v1 codec, renders the same fixed world->screen map as the new
 * client, and prints the framebuffer SHA-256.  If the encodings are
 * compatible, its hash equals the new client's.
 *
 * Usage: client_old <recording> [framebuffer.ppm]
 */
#include <stdio.h>
#include <string.h>

#include "recording.h"
#include "render.h"
#include "legacy/legacy_v1.h"

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <recording> [out.ppm]\n", argv[0]); return 2; }
    Recording rec;
    if (!recording_read(argv[1], &rec)) { fprintf(stderr, "cannot read %s\n", argv[1]); return 2; }

    Frame f;
    fb_clear(&f, 8, 12, 20);
    int decoded = 0;
    for (uint32_t i = 0; i < rec.count; i++) {
        unsigned tick; float x, y;
        /* The pinned v1 decoder stops after tick/x/y and ignores any tail. */
        if (legacy_snapshot_decode(rec.pkt[i], rec.len[i], rec.scale, &tick, &x, &y) <= 0) {
            /* A packet the old client cannot decode is skipped, never guessed. */
            continue;
        }
        decoded++;
        int cx = FB_W / 2 + (int)(x * 0.5f);
        int cy = FB_H / 2 + (int)(y * 0.5f);
        int t = (int)i * 255 / (rec.count > 1 ? (int)rec.count - 1 : 1);
        unsigned char r = (unsigned char)(40 + (t * 215) / 255);
        unsigned char g = (unsigned char)(80 + (t * 100) / 255);
        fb_rect(&f, cx - 2, cy - 2, 5, 5, r, g, 220);
    }

    char hex[65]; frame_hash(&f, hex);
    printf("client=old decoded=%d/%u sha256=%s\n", decoded, rec.count, hex);

    if (argc >= 3) {
        FILE* fp = fopen(argv[2], "wb");
        if (fp) {
            fprintf(fp, "P6\n%d %d\n255\n", FB_W, FB_H);
            for (int y = 0; y < FB_H; y++)
                fwrite(f.px[y], 1, FB_W * 3, fp);
            fclose(fp);
        }
    }
    return 0;
}
