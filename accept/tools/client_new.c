/*
 * client_new.c - the new client, built against the current module code.
 *
 * Reads the same recording, decodes with snap_decode (which understands the
 * v1 prefix and the v2 extension), renders the same fixed map, and prints the
 * framebuffer SHA-256.
 *
 * Usage: client_new <recording> [framebuffer.ppm]
 */
#include <stdio.h>
#include <string.h>

#include "recording.h"
#include "render.h"
#include "snapshot.h"

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <recording> [out.ppm]\n", argv[0]); return 2; }
    Recording rec;
    if (!recording_read(argv[1], &rec)) { fprintf(stderr, "cannot read %s\n", argv[1]); return 2; }

    Frame f;
    fb_clear(&f, 8, 12, 20);
    int decoded = 0;
    for (uint32_t i = 0; i < rec.count; i++) {
        Packet p; memset(&p, 0, sizeof(p));
        memcpy(p.buf, rec.pkt[i], rec.len[i]);
        p.len = rec.len[i];
        State s; SnapInfo si;
        int rc = snap_decode_info(&p, &s, rec.scale, &si);
        if (rc < 0 && !si.prefix_ok) continue;   /* unusable packet */
        decoded++;
        int cx = FB_W / 2 + (int)(s.x * 0.5f);
        int cy = FB_H / 2 + (int)(s.y * 0.5f);
        int t = (int)i * 255 / (rec.count > 1 ? (int)rec.count - 1 : 1);
        unsigned char r = (unsigned char)(40 + (t * 215) / 255);
        unsigned char g = (unsigned char)(80 + (t * 100) / 255);
        fb_rect(&f, cx - 2, cy - 2, 5, 5, r, g, 220);
    }

    char hex[65]; frame_hash(&f, hex);
    printf("client=new decoded=%d/%u sha256=%s\n", decoded, rec.count, hex);

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
