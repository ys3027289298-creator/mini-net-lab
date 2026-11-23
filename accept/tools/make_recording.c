/*
 * make_recording.c - build the shared replay recording.
 *
 * The recording is committed under accept/recordings/ and is produced here,
 * once, by the current encoder.  It deliberately mixes three packet shapes so
 * every compatibility case is on disk and replayed by both clients:
 *   1. v1 packets (current encoder with flags == 0, bytes == pinned v1)
 *   2. v2 packets with the new field set (prefix + extension)
 * A separate negative-control recording inserts the field BEFORE x/y, the
 * naive layout that caused the jumping, so the matrix can prove the check is
 * not vacuous.
 *
 * Usage: make_recording <out-dir>
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "recording.h"
#include "snapshot.h"
#include "legacy/legacy_v1.h"

#define N 24

static void fill_positions(double* xs, double* ys) {
    for (int i = 0; i < N; i++) {
        xs[i] = (double)((i * 37) % 200 - 100) * 0.25;
        ys[i] = (double)((i * 53) % 120 - 60) * 0.25;
    }
}

int main(int argc, char** argv) {
    const char* dir = argc > 1 ? argv[1] : "accept/recordings";
    char path[512];
    double xs[N], ys[N];
    fill_positions(xs, ys);

    /* --- good recording: v1 packets then v2 packets, alternating --- */
    Recording rec; memset(&rec, 0, sizeof(rec));
    rec.count = N;
    rec.scale = 64.0f;
    int all_v1 = 1, all_prefix_ok = 1;
    for (int i = 0; i < N; i++) {
        Packet p; memset(&p, 0, sizeof(p));
        /* every third frame carries the new field, the rest stay pure v1 */
        State s = { 1000u + (unsigned)i * 3u, (float)xs[i], (float)ys[i], 0, 0,
                    (i % 3 == 2) ? 7u : 0u };
        int n = snap_encode(&s, &p, rec.scale);
        if (n <= 0) { fprintf(stderr, "encode failed at %d\n", i); return 2; }
        rec.len[i] = (uint16_t)n;
        memcpy(rec.pkt[i], p.buf, (size_t)n);

        /* every packet must start with the exact pinned v1 bytes for its
           position: this is the property the old client relies on */
        unsigned char lb[64];
        int ln = legacy_snapshot_encode(s.tick, s.x, s.y, rec.scale, lb, sizeof(lb));
        if (ln > n || memcmp(lb, p.buf, (size_t)ln) != 0) all_prefix_ok = 0;
        if (n != ln) all_v1 = 0;
    }

    snprintf(path, sizeof(path), "%s/replay.bin", dir);
    if (!recording_write(path, &rec)) { fprintf(stderr, "write %s failed\n", path); return 2; }
    printf("wrote %s: %u packets, scale=%.1f, all_v1=%d, prefix_ok=%d\n",
           path, rec.count, rec.scale, all_v1, all_prefix_ok);

    /* --- the same session as an old client recorded it: every packet v1 ---
       old client + this file is the "old version rendering" reference; old
       client + replay.bin must render identically. */
    Recording v1rec = rec;
    for (int i = 0; i < N; i++) {
        Packet p; memset(&p, 0, sizeof(p));
        State s = { 1000u + (unsigned)i * 3u, (float)xs[i], (float)ys[i], 0, 0, 0u };
        int n = snap_encode(&s, &p, v1rec.scale);
        if (n <= 0) { fprintf(stderr, "v1 encode failed at %d\n", i); return 2; }
        v1rec.len[i] = (uint16_t)n;
        memcpy(v1rec.pkt[i], p.buf, (size_t)n);
    }
    snprintf(path, sizeof(path), "%s/replay_v1.bin", dir);
    if (!recording_write(path, &v1rec)) { fprintf(stderr, "write %s failed\n", path); return 2; }
    printf("wrote %s: %u packets (pure v1: the old-version reference)\n", path, v1rec.count);

    /* --- negative-control recording: field inserted before x/y --- */
    Recording bad; memset(&bad, 0, sizeof(bad));
    bad.count = N;
    bad.scale = 64.0f;
    for (int i = 0; i < N; i++) {
        unsigned char naive[64];
        int bn = legacy_snapshot_encode(1000u + (unsigned)i * 3u, (float)xs[i],
                                        (float)ys[i], bad.scale, naive, sizeof(naive));
        /* tick varint length */
        int tl = 1;
        while (tl < bn && (naive[tl - 1] & 0x80)) tl++;
        if (tl >= bn) tl = 1;
        int o = 0;
        memcpy(bad.pkt[i], naive, (size_t)tl); o += tl;
        bad.pkt[i][o++] = 7;                       /* naive new field */
        memcpy(bad.pkt[i] + o, naive + tl, (size_t)(bn - tl)); o += bn - tl;
        bad.len[i] = (uint16_t)o;
    }
    snprintf(path, sizeof(path), "%s/replay_naive.bin", dir);
    if (!recording_write(path, &bad)) { fprintf(stderr, "write %s failed\n", path); return 2; }
    printf("wrote %s: %u packets (negative control: field before x/y)\n", path, bad.count);

    return all_prefix_ok ? 0 : 1;
}
