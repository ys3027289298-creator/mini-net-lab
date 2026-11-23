/*
 * accept.c - cross-version wire-compatibility acceptance for the four modules.
 *
 * Scenario: a snapshot record/replay session was recorded by an old client
 * (v1, commit 0bdd277) and is replayed by new clients.  After a field was
 * added naively, old clients decoded it as part of the position and
 * characters jumped.  This driver proves the new encoding does not do that.
 *
 * Evidence written under <evidence-dir>/:
 *   error_codes.md     - error tables generated from the header X-macro lists
 *   compat_matrix.csv  - producer x consumer matrix, every cell measured
 *   pixel_identity.csv - per-module recording replay, old vs new framebuffer
 *
 * Exit 0 only if every matrix cell passes and every recording renders
 * pixel-identically.  A missing cell is a failure, never a silent skip.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "snapshot.h"
#include "clock.h"
#include "aoi.h"
#include "reconcile.h"
#include "render.h"
#include "legacy/legacy_v1.h"
#include "recording.h"

#define MODULES 4

static int failures = 0;
static int matrix_cells = 0, matrix_pass = 0;
static FILE* mtx = NULL;
static FILE* pix = NULL;
static FILE* bodylog = NULL;

/* log a package-body hash: module, producer, type, bytes, sha256 */
static void body_hash(const char* module, const char* producer, const char* type,
                      const unsigned char* b, int len) {
    char hex[65];
    bytes_hash(b, len, hex);
    fprintf(bodylog, "%s,%s,%s,%d,%s\n", module, producer, type, len, hex);
}

static void sep(const char* title) { printf("\n=== %s ===\n", title); }

/* ---------------------------------------------------------------------- */
/* 1. error-code table, generated from the header X-macro lists            */
/* ---------------------------------------------------------------------- */
#define EMIT_ROW(modname, code, val, desc, namefn, f, ok)                     \
    do {                                                                      \
        const char* nm_ = (namefn)(code);                                     \
        int same_ = strcmp(nm_, #code) == 0;                                  \
        if (!same_) { fprintf(stderr, "%s: %s != %s\n", modname, nm_, #code); \
                      (ok) = 0; }                                             \
        fprintf((f), "| `%s` | %d | %s | %s |\n", #code, code, desc,          \
                same_ ? "same" : nm_);                                        \
    } while (0)

static int write_error_codes(const char* dir) {
    char path[512];
    snprintf(path, sizeof(path), "%s/error_codes.md", dir);
    FILE* f = fopen(path, "w");
    if (!f) { fprintf(stderr, "cannot write %s\n", path); return 0; }
    int ok = 1;

    fprintf(f, "# Error-code tables\n\n");
    fprintf(f, "Generated from the `*_ERR_LIST(X)` X-macro in each module header:\n");
    fprintf(f, "the enum, the `*_err_name()` switch and this table all expand the same\n");
    fprintf(f, "list, so they cannot drift. The last column is the runtime result of\n");
    fprintf(f, "`*_err_name()` for that code and is asserted equal to the enum name.\n\n");

#define TABLE_BEGIN(mod, type)                                                \
    fprintf(f, "## %s (`%s`)\n\n| code | value | meaning | runtime name |\n"   \
               "|---|---|---|---|\n", (mod), (type))
#define TABLE_END() fprintf(f, "\n")

    TABLE_BEGIN("snapshot", "snap_err_t");
#define X(code, val, desc) EMIT_ROW("snapshot", code, val, desc, snap_err_name, f, ok);
    SNAP_ERR_LIST(X)
#undef X
    TABLE_END();

    TABLE_BEGIN("clock", "clock_err_t");
#define X(code, val, desc) EMIT_ROW("clock", code, val, desc, clock_err_name, f, ok);
    CLOCK_ERR_LIST(X)
#undef X
    TABLE_END();

    TABLE_BEGIN("aoi", "aoi_err_t");
#define X(code, val, desc) EMIT_ROW("aoi", code, val, desc, aoi_err_name, f, ok);
    AOI_ERR_LIST(X)
#undef X
    TABLE_END();

    TABLE_BEGIN("reconcile", "recon_err_t");
#define X(code, val, desc) EMIT_ROW("reconcile", code, val, desc, recon_err_name, f, ok);
    RECON_ERR_LIST(X)
#undef X
    TABLE_END();
#undef TABLE_BEGIN
#undef TABLE_END
#undef EMIT_ROW

    fclose(f);
    printf("error_codes.md: %s\n", ok ? "names match enums" : "NAME MISMATCH");
    return ok;
}

/* ---------------------------------------------------------------------- */
/* matrix plumbing                                                         */
/* ---------------------------------------------------------------------- */
static void cell(const char* module, const char* producer, const char* consumer,
                 int pass, const char* detail) {
    matrix_cells++;
    if (pass) matrix_pass++;
    else failures++;
    /* detail is quoted: it may contain commas */
    fprintf(mtx, "%s,%s,%s,%s,\"%s\"\n", module, producer, consumer,
            pass ? "PASS" : "FAIL", detail);
    printf("%-9s %-6s -> %-6s %-4s %s\n", module, producer, consumer,
           pass ? "PASS" : "FAIL", detail);
}

/* ---------------------------------------------------------------------- */
/* 2. snapshot: pixel identity + matrix                                    */
/* ---------------------------------------------------------------------- */

/* A deterministic "recording": ticks and positions quantized at 64 units. */
typedef struct { unsigned tick; float x, y; } PosFrame;
#define REC_FRAMES 24

static void make_recording(PosFrame* rec) {
    for (int i = 0; i < REC_FRAMES; i++) {
        rec[i].tick = 1000u + (unsigned)i * 3u;
        rec[i].x = (float)((i * 37) % 200 - 100) * 0.25f;
        rec[i].y = (float)((i * 53) % 120 - 60) * 0.25f;
    }
}

/* Fixed world->screen map, integer only: position drives every pixel. */
static void render_positions(const PosFrame* rec, int n, Frame* f) {
    fb_clear(f, 8, 12, 20);
    for (int i = 0; i < n; i++) {
        int cx = FB_W / 2 + (int)(rec[i].x * 0.5f);
        int cy = FB_H / 2 + (int)(rec[i].y * 0.5f);
        /* older frames fade toward the background */
        int t = i * 255 / (n > 1 ? n - 1 : 1);
        unsigned char r = (unsigned char)(40 + (t * 215) / 255);
        unsigned char g = (unsigned char)(80 + (t * 100) / 255);
        fb_rect(f, cx - 2, cy - 2, 5, 5, r, g, 220);
    }
}

static void snapshot_accept(void) {
    sep("snapshot: pixel identity under replay");
    const float scale = 64.0f;
    PosFrame rec[REC_FRAMES];
    make_recording(rec);

    /* Packets as a v1 client wrote them: only the three prefix varints. */
    unsigned char v1_pkt[REC_FRAMES][SNAP_MAX];
    int v1_len[REC_FRAMES];
    for (int i = 0; i < REC_FRAMES; i++) {
        Packet p; memset(&p, 0, sizeof(p));
        State s = { rec[i].tick, rec[i].x, rec[i].y, 0, 0, 0 };
        int n = snap_encode(&s, &p, scale);
        if (n <= 0) { printf("encode failed\n"); failures++; return; }
        v1_len[i] = n;
        memcpy(v1_pkt[i], p.buf, (size_t)n);
        /* the pinned 0bdd277 encoder must produce identical bytes */
        unsigned char lb[SNAP_MAX];
        int ln = legacy_snapshot_encode(rec[i].tick, rec[i].x, rec[i].y, scale, lb, SNAP_MAX);
        if (ln != n || memcmp(lb, p.buf, (size_t)n) != 0) {
            printf("packet %d: new v1 prefix differs from pinned 0bdd277 encoder\n", i);
            failures++;
        }
    }

    PosFrame r_v1_legacy[REC_FRAMES], r_v1_cur[REC_FRAMES];
    PosFrame r_v2_legacy[REC_FRAMES], r_v2_cur[REC_FRAMES];
    int ok_v1cur = 1, ok_v2legacy = 1, ok_v2cur = 1, ok_v1legacy = 1;

    for (int i = 0; i < REC_FRAMES; i++) {
        /* --- v1 producer: identical bytes from the current encoder --- */
        Packet p; memset(&p, 0, sizeof(p));
        State s = { rec[i].tick, rec[i].x, rec[i].y, 0, 0, 0 };
        int n = snap_encode(&s, &p, scale);
        if (n != v1_len[i]) ok_v1cur = 0;

        Packet pv = p; pv.len = n;
        State out;
        if (snap_decode(&pv, &out, scale) <= 0) ok_v1cur = 0;
        r_v1_cur[i] = (PosFrame){ out.tick, out.x, out.y };

        unsigned tk; float x, y;
        if (legacy_snapshot_decode(v1_pkt[i], v1_len[i], scale, &tk, &x, &y) <= 0) ok_v1legacy = 0;
        r_v1_legacy[i] = (PosFrame){ tk, x, y };

        /* --- v2 producer: the new field is present --- */
        Packet p2; memset(&p2, 0, sizeof(p2));
        State s2 = { rec[i].tick, rec[i].x, rec[i].y, 0, 0, 7u };
        int n2 = snap_encode(&s2, &p2, scale);
        if (n2 <= n || memcmp(p2.buf, p.buf, (size_t)n) != 0) ok_v2legacy = 0;

        /* v2 packet read by the pinned v1 decoder: extension must be ignored */
        if (legacy_snapshot_decode(p2.buf, n2, scale, &tk, &x, &y) <= 0) ok_v2legacy = 0;
        r_v2_legacy[i] = (PosFrame){ tk, x, y };

        Packet p2c = p2; p2c.len = n2;
        if (snap_decode(&p2c, &out, scale) <= 0) ok_v2cur = 0;
        r_v2_cur[i] = (PosFrame){ out.tick, out.x, out.y };
    }

    Frame base, f;
    char hb[65], h1[65], h2[65], h3[65], h4[65];
    render_positions(rec, REC_FRAMES, &base);        frame_hash(&base, hb);
    render_positions(r_v1_legacy, REC_FRAMES, &f);   frame_hash(&f, h1);
    int diff_v1legacy = frame_diff(&base, &f);
    render_positions(r_v1_cur, REC_FRAMES, &f);      frame_hash(&f, h4);
    int diff_v1cur = frame_diff(&base, &f);
    render_positions(r_v2_legacy, REC_FRAMES, &f);   frame_hash(&f, h2);
    int diff_v2legacy = frame_diff(&base, &f);
    render_positions(r_v2_cur, REC_FRAMES, &f);      frame_hash(&f, h3);
    int diff_v2cur = frame_diff(&base, &f);

    printf("baseline (original positions)   %s\n", hb);
    printf("v1 record -> pinned v1 client   %s diff=%d\n", h1, diff_v1legacy);
    printf("v1 record -> current client     %s diff=%d\n", h4, diff_v1cur);
    printf("v2 record -> pinned v1 client   %s diff=%d\n", h2, diff_v2legacy);
    printf("v2 record -> current client     %s diff=%d\n", h3, diff_v2cur);

    if (!(diff_v1legacy == 0 && diff_v1cur == 0 && diff_v2legacy == 0 && diff_v2cur == 0))
        failures++;

    fprintf(pix, "module,producer,consumer,frames,baseline_sha256,replay_sha256,"
                 "diff_bytes,pixel_identical\n");
    fprintf(pix, "snapshot,v1,pinned_v1,%d,%s,%s,%d,%d\n", REC_FRAMES, hb, h1,
            diff_v1legacy, diff_v1legacy == 0);
    fprintf(pix, "snapshot,v1,current,%d,%s,%s,%d,%d\n", REC_FRAMES, hb, h4,
            diff_v1cur, diff_v1cur == 0);
    fprintf(pix, "snapshot,v2,pinned_v1,%d,%s,%s,%d,%d\n", REC_FRAMES, hb, h2,
            diff_v2legacy, diff_v2legacy == 0);
    fprintf(pix, "snapshot,v2,current,%d,%s,%s,%d,%d\n", REC_FRAMES, hb, h3,
            diff_v2cur, diff_v2cur == 0);

    cell("snapshot", "v1", "pinned_v1", ok_v1legacy && diff_v1legacy == 0,
         "replay renders identically");
    cell("snapshot", "v1", "current", ok_v1cur && diff_v1cur == 0,
         "replay renders identically");
    cell("snapshot", "v2", "pinned_v1", ok_v2legacy && diff_v2legacy == 0,
         "extension ignored, position intact");
    cell("snapshot", "v2", "current", ok_v2cur && diff_v2cur == 0,
         "extension read, position intact");

    {
        unsigned char lb[SNAP_MAX];
        int ln = legacy_snapshot_encode(rec[0].tick, rec[0].x, rec[0].y, scale, lb, SNAP_MAX);
        Packet pv2; memset(&pv2, 0, sizeof(pv2));
        State sv2 = { rec[0].tick, rec[0].x, rec[0].y, 0, 0, 7u };
        int ln2 = snap_encode(&sv2, &pv2, scale);
        body_hash("snapshot", "v1_reference", "v1_packet", (unsigned char*)v1_pkt[0], v1_len[0]);
        body_hash("snapshot", "current", "v1_packet", lb, ln);
        body_hash("snapshot", "current", "v2_packet", pv2.buf, ln2);
    }

    /* Negative control: build the buggy encoding this change avoids - the new
       field inserted right after tick, before x/y - and let the pinned v1
       decoder read it with the old field order.  The renderer must produce
       visibly different pixels, or the pixel check proves nothing. */
    PosFrame broken[REC_FRAMES];
    for (int i = 0; i < REC_FRAMES; i++) {
        unsigned char naive[SNAP_MAX];
        int bn = legacy_snapshot_encode(rec[i].tick, rec[i].x, rec[i].y, scale,
                                        naive, SNAP_MAX);
        /* naive layout: tick, newflags, x, y.  Splice a flags varint after
           the tick varint, shifting x/y one field later. */
        unsigned char fields[SNAP_MAX];
        int fn = legacy_snapshot_encode(rec[i].tick, 0.0f, 0.0f, scale, fields, SNAP_MAX);
        int tick_len = 1;
        while (tick_len < fn && (fields[tick_len - 1] & 0x80)) tick_len++;
        if (tick_len >= fn) tick_len = 1;
        unsigned char bad_pkt[SNAP_MAX];
        int o = 0;
        memcpy(bad_pkt + o, naive, (size_t)tick_len); o += tick_len;  /* tick */
        bad_pkt[o++] = 7;                                            /* flags */
        memcpy(bad_pkt + o, naive + tick_len, (size_t)(bn - tick_len)); o += bn - tick_len;
        unsigned tk; float x, y;
        if (legacy_snapshot_decode(bad_pkt, o, scale, &tk, &x, &y) <= 0)
            { tk = rec[i].tick; x = 0; y = 0; }
        broken[i] = (PosFrame){ tk, x, y };
    }
    render_positions(broken, REC_FRAMES, &f);
    char hbroken[65]; frame_hash(&f, hbroken);
    int diff_broken = frame_diff(&base, &f);
    printf("negative control (shifted framing) %s diff=%d\n", hbroken, diff_broken);
    fprintf(pix, "snapshot,negative_control,current,%d,%s,%s,%d,%d\n", REC_FRAMES, hb,
            hbroken, diff_broken, diff_broken == 0);
    if (diff_broken == 0) {
        printf("NEGATIVE CONTROL DEAD: renderer cannot tell a shifted decode apart\n");
        failures++;
    }
}

/* ---------------------------------------------------------------------- */
/* 3. clock / aoi / reconcile: v1-prefix identity + matrix                 */
/* ---------------------------------------------------------------------- */

static void clock_accept(void) {
    sep("clock: v1 body is an exact prefix of v2");
    Sync sy; clock_sync_init(&sy);
    for (int i = 0; i < 5; i++)
        clock_sync_add(&sy, i * 1.0, i * 1.0 + 0.05 + (i % 5) * 0.01,
                       i * 1.0 + 0.025 + 2.0);

    LClockV1 lv; memset(&lv, 0, sizeof(lv));
    lv.n = sy.n;
    for (int i = 0; i < sy.n; i++) {
        lv.s[i].rtt = sy.s[i].rtt; lv.s[i].offset = sy.s[i].offset; lv.s[i].at = sy.s[i].at;
    }
    lv.best_rtt = sy.best_rtt; lv.offset = sy.offset;

    /* v1 producer */
    unsigned char v1[CLOCK_STATE_MAX]; int l1 = legacy_clock_encode_v1(&lv, v1, (int)sizeof(v1));
    Sync cur; ClockInfo ci;
    int rc = clock_state_decode_info(v1, l1, &cur, &ci);
    int cur_ok = (rc == l1) && cur.n == sy.n && cur.best_rtt == sy.best_rtt &&
                 cur.offset == sy.offset && ci.version == 1u;
    for (int i = 0; i < sy.n && cur_ok; i++)
        cur_ok = cur.s[i].rtt == sy.s[i].rtt && cur.s[i].at == sy.s[i].at;

    /* current emits v1 bytes for a state with no v2 field set */
    Sync plain = sy; plain.filtered = 0;
    unsigned char body[CLOCK_STATE_MAX];
    int bl = clock_state_encode(&plain, body, (int)sizeof(body));
    int prefix_ok = (bl == l1) && memcmp(body, v1, (size_t)l1) == 0;

    /* v2 producer: filtered set; v1 consumer must ignore the tail */
    unsigned char v2[CLOCK_STATE_MAX];
    int l2 = clock_state_encode(&sy, v2, (int)sizeof(v2));
    LClockV1 back; int lr = legacy_clock_decode_v1(v2, l2, &back);
    int leg_ok = (lr == l1) && back.n == lv.n && back.best_rtt == lv.best_rtt &&
                 back.offset == lv.offset &&
                 memcmp(v2, v1, (size_t)l1) == 0;
    for (int i = 0; i < lv.n && leg_ok; i++) leg_ok = back.s[i].rtt == lv.s[i].rtt;

    body_hash("clock", "v1_reference", "v1_body", v1, l1);
    body_hash("clock", "current", "v1_prefix", body, bl);
    body_hash("clock", "current", "v2_body", v2, l2);
    printf("v1 encode=%d current decode=%d prefix_identical=%d v2->legacy=%d\n",
           l1, rc, prefix_ok, lr);
    cell("clock", "v1", "current", cur_ok, "all samples/config exact");
    cell("clock", "v1(no v2 field)", "current", cur.filtered == 0u, "v2 field not invented");
    cell("clock", "v1", "pinned_v1", prefix_ok, "current emits identical v1 bytes");
    cell("clock", "v2", "pinned_v1", leg_ok, "legacy reads prefix, ignores tail");
}

static void aoi_accept(void) {
    sep("aoi: v1 body is an exact prefix of v2");
    World w; aoi_world_init(&w, 10.0f, 15.0f);
    aoi_world_add(&w, 1, 0, 0);
    aoi_world_add(&w, 2, 5, 0);
    aoi_world_add(&w, -3, 100, -100);
    aoi_world_remove(&w, 2);

    LAoiV1 la; memset(&la, 0, sizeof(la));
    la.n = w.n; la.cell = w.cell; la.radius = w.radius;
    for (int i = 0; i < w.n; i++) {
        la.e[i].id = w.e[i].id; la.e[i].x = w.e[i].x;
        la.e[i].y = w.e[i].y; la.e[i].alive = w.e[i].alive;
    }

    unsigned char v1[AOI_STATE_MAX]; int l1 = legacy_aoi_encode_v1(&la, v1, (int)sizeof(v1));
    World cur; AoiInfo ai;
    int rc = aoi_state_decode_info(v1, l1, &cur, &ai);
    int cur_ok = (rc == l1) && cur.n == la.n && cur.cell == la.cell &&
                 cur.radius == la.radius && ai.version == 1u;
    for (int i = 0; i < la.n && cur_ok; i++)
        cur_ok = cur.e[i].id == la.e[i].id && cur.e[i].x == la.e[i].x &&
                 cur.e[i].y == la.e[i].y && cur.e[i].alive == la.e[i].alive;

    unsigned char body[AOI_STATE_MAX];
    World plain = w; plain.rev = 0;
    int bl = aoi_state_encode(&plain, body, (int)sizeof(body));
    int prefix_ok = (bl == l1) && memcmp(body, v1, (size_t)l1) == 0;

    unsigned char v2[AOI_STATE_MAX];
    int l2 = aoi_state_encode(&w, v2, (int)sizeof(v2));
    LAoiV1 back; int lr = legacy_aoi_decode_v1(v2, l2, &back);
    int leg_ok = (lr == l1) && back.n == la.n && back.cell == la.cell &&
                 back.radius == la.radius && memcmp(v2, v1, (size_t)l1) == 0;
    for (int i = 0; i < la.n && leg_ok; i++)
        leg_ok = back.e[i].id == la.e[i].id && back.e[i].x == la.e[i].x &&
                 back.e[i].y == la.e[i].y && back.e[i].alive == la.e[i].alive;

    body_hash("aoi", "v1_reference", "v1_body", v1, l1);
    body_hash("aoi", "current", "v1_prefix", body, bl);
    body_hash("aoi", "current", "v2_body", v2, l2);
    printf("v1 encode=%d current decode=%d prefix_identical=%d v2->legacy=%d\n",
           l1, rc, prefix_ok, lr);
    cell("aoi", "v1", "current", cur_ok, "entities/config exact");
    cell("aoi", "v1(no v2 field)", "current", cur.rev == 0u, "v2 field not invented");
    cell("aoi", "v1", "pinned_v1", prefix_ok, "current emits identical v1 bytes");
    cell("aoi", "v2", "pinned_v1", leg_ok, "legacy reads prefix, ignores tail");
}

static void reconcile_accept(void) {
    sep("reconcile: v1 body is an exact prefix of v2");
    Recon r; recon_init(&r);
    recon_push(&r, 1, 1.0f);
    recon_push(&r, 2, 2.0f);
    recon_push(&r, 3, 3.25f);
    recon_ack(&r, 2, 2.0f);
    recon_ack(&r, 3, 3.5f);

    LReconV1 lr0; memset(&lr0, 0, sizeof(lr0));
    lr0.count = r.count; lr0.head = r.head; lr0.acked = r.acked;
    lr0.rollbacks = r.rollbacks; lr0.last_err = r.last_err;
    for (int i = 0; i < BUF; i++) { lr0.p[i].tick = r.p[i].tick; lr0.p[i].x = r.p[i].x; }

    unsigned char v1[RECON_STATE_MAX]; int l1 = legacy_recon_encode_v1(&lr0, v1, (int)sizeof(v1));
    Recon cur; ReconInfo ri;
    int rc = recon_state_decode_info(v1, l1, &cur, &ri);
    int cur_ok = (rc == l1) && cur.count == r.count && cur.acked == r.acked &&
                 cur.rollbacks == r.rollbacks && cur.last_err == r.last_err &&
                 ri.version == 1u;
    for (int i = 0; i < r.count && cur_ok; i++)
        cur_ok = cur.p[i].tick == r.p[i].tick && cur.p[i].x == r.p[i].x;

    Recon plain = r; plain.ack_hits = 0;
    unsigned char body[RECON_STATE_MAX];
    int bl = recon_state_encode(&plain, body, (int)sizeof(body));
    int prefix_ok = (bl == l1) && memcmp(body, v1, (size_t)l1) == 0;

    unsigned char v2[RECON_STATE_MAX];
    int l2 = recon_state_encode(&r, v2, (int)sizeof(v2));
    LReconV1 back; int lrr = legacy_recon_decode_v1(v2, l2, &back);
    int leg_ok = (lrr == l1) && back.count == lr0.count && back.acked == lr0.acked &&
                 back.rollbacks == lr0.rollbacks && back.last_err == lr0.last_err &&
                 memcmp(v2, v1, (size_t)l1) == 0;
    for (int i = 0; i < lr0.count && leg_ok; i++)
        leg_ok = back.p[i].tick == lr0.p[i].tick && back.p[i].x == lr0.p[i].x;

    body_hash("reconcile", "v1_reference", "v1_body", v1, l1);
    body_hash("reconcile", "current", "v1_prefix", body, bl);
    body_hash("reconcile", "current", "v2_body", v2, l2);
    printf("v1 encode=%d current decode=%d prefix_identical=%d v2->legacy=%d\n",
           l1, rc, prefix_ok, lrr);
    cell("reconcile", "v1", "current", cur_ok, "buffer/acked/err exact");
    cell("reconcile", "v1(no v2 field)", "current", cur.ack_hits == 0u, "v2 field not invented");
    cell("reconcile", "v1", "pinned_v1", prefix_ok, "current emits identical v1 bytes");
    cell("reconcile", "v2", "pinned_v1", leg_ok, "legacy reads prefix, ignores tail");
}

/* ---------------------------------------------------------------------- */
/* 4. error-path compatibility: a bad tail must not move the position      */
/* ---------------------------------------------------------------------- */
static void error_path_accept(void) {
    sep("snapshot: malformed extension keeps the prefix authoritative");
    Packet p; memset(&p, 0, sizeof(p));
    State in = { 1000u, 12.5f, -3.25f, 0, 0, 5u };
    int n = snap_encode(&in, &p, 64.0f);
    printf("v2 packet len=%d\n", n);

    /* cut the extension mid-record */
    Packet cut = p; cut.len = n - 1;
    State s; SnapInfo si;
    int rc = snap_decode_info(&cut, &s, 64.0f, &si);
    int prefix_ok = (rc < 0) && si.prefix_ok == 1 && s.tick == in.tick &&
                    s.x == (float)(int)lrintf(in.x * 64.0f) / 64.0f;
    cell("snapshot", "v2_truncated", "current", prefix_ok,
         rc < 0 ? snap_err_name(rc) : "expected error");

    /* unsupported version in the tail */
    unsigned char bad[SNAP_MAX];
    Packet bp; memset(&bp, 0, sizeof(bp));
    State bi = { 1000u, 12.5f, -3.25f, 0, 0, 0 };
    int bn = snap_encode(&bi, &bp, 64.0f);
    memcpy(bad, bp.buf, (size_t)bn);
    bad[bn + 0] = SNAP_TAG_VERSION;  /* tag */
    bad[bn + 1] = 1;                 /* len */
    bad[bn + 2] = 99;                /* version 99 */
    Packet pv = { .len = bn + 3 };
    memcpy(pv.buf, bad, (size_t)(bn + 3));
    int vrc = snap_decode_info(&pv, &s, 64.0f, &si);
    int ver_ok = (vrc == SNAP_ERR_VERSION_RANGE) && si.prefix_ok == 1 &&\
                 s.tick == in.tick && s.y == -3.25f;
    cell("snapshot", "v2_bad_version", "current", ver_ok, snap_err_name(vrc));
}

/* ---------------------------------------------------------------------- */
/* 5. two independent binaries: old client vs new client on one recording  */
/* ---------------------------------------------------------------------- */
static void two_client_accept(const char* recdir) {
    sep("two independent clients replay one committed recording");
    char recpath[512];
    snprintf(recpath, sizeof(recpath), "%s/replay.bin", recdir);

    Recording rec;
    if (!recording_read(recpath, &rec)) {
        printf("cannot read %s\n", recpath);
        cell("snapshot", "v1", "pinned_v1", 0, "recording missing");
        cell("snapshot", "v1", "current", 0, "recording missing");
        cell("snapshot", "v2", "pinned_v1", 0, "recording missing");
        cell("snapshot", "v2", "current", 0, "recording missing");
        return;
    }

    /* old client: pinned decoder only.  new client: snap_decode_info. */
    Frame fo, fn;
    fb_clear(&fo, 8, 12, 20);
    fb_clear(&fn, 8, 12, 20);
    int ok_old = 1, ok_new = 1;
    int v1_prefix_matches = 1;
    for (uint32_t i = 0; i < rec.count; i++) {
        unsigned tk; float x, y;
        if (legacy_snapshot_decode(rec.pkt[i], rec.len[i], rec.scale, &tk, &x, &y) <= 0) {
            ok_old = 0; continue;
        }
        /* each packet must start with exactly the pinned v1 prefix */
        unsigned char lb[64];
        int ln = legacy_snapshot_encode(tk, x, y, rec.scale, lb, sizeof(lb));
        if (ln > rec.len[i] || memcmp(lb, rec.pkt[i], (size_t)ln) != 0) v1_prefix_matches = 0;

        int cx = FB_W / 2 + (int)(x * 0.5f), cy = FB_H / 2 + (int)(y * 0.5f);
        int t = (int)i * 255 / (rec.count > 1 ? (int)rec.count - 1 : 1);
        unsigned char r = (unsigned char)(40 + (t * 215) / 255);
        unsigned char g = (unsigned char)(80 + (t * 100) / 255);
        fb_rect(&fo, cx - 2, cy - 2, 5, 5, r, g, 220);

        Packet p; memset(&p, 0, sizeof(p));
        memcpy(p.buf, rec.pkt[i], rec.len[i]); p.len = rec.len[i];
        State s; SnapInfo si;
        if (snap_decode_info(&p, &s, rec.scale, &si) < 0 && !si.prefix_ok) { ok_new = 0; continue; }
        cx = FB_W / 2 + (int)(s.x * 0.5f); cy = FB_H / 2 + (int)(s.y * 0.5f);
        fb_rect(&fn, cx - 2, cy - 2, 5, 5, r, g, 220);
    }

    char ho[65], hn[65];
    frame_hash(&fo, ho);
    frame_hash(&fn, hn);
    int diff = frame_diff(&fo, &fn);
    printf("recording=%s frames=%u\n", recpath, rec.count);
    printf("old client (pinned 0bdd277) %s\n", ho);
    printf("new client (current snap)   %s\n", hn);
    printf("framebuffers differ in %d bytes\n", diff);

    fprintf(pix, "snapshot,recording_old_client,recording_new_client,%u,%s,%s,%d,%d\n",
            rec.count, ho, hn, diff, diff == 0);
    /* separate committed plan: files produced by run.sh clients */
    cell("snapshot", "recording(v1+v2)", "pinned_v1", ok_old && v1_prefix_matches,
         "decodes every packet, prefix exact");
    cell("snapshot", "recording(v1+v2)", "current", ok_new, "decodes every packet");
    cell("snapshot", "recording(v1+v2)", "pixel_compare", diff == 0,
         "old and new framebuffers identical");

    /* The claim, measured directly: the old client renders the CURRENT
       recording (v1 + v2 packets) exactly as it renders the PURE-v1 recording
       of the same session.  This is "old client + same session == old
       version". */
    char v1path[512];
    snprintf(v1path, sizeof(v1path), "%s/replay_v1.bin", recdir);
    Recording vrec;
    if (recording_read(v1path, &vrec) && vrec.count == rec.count) {
        Frame fv; fb_clear(&fv, 8, 12, 20);
        for (uint32_t i = 0; i < vrec.count; i++) {
            unsigned tk; float x, y;
            if (legacy_snapshot_decode(vrec.pkt[i], vrec.len[i], vrec.scale, &tk, &x, &y) <= 0) continue;
            int cx = FB_W / 2 + (int)(x * 0.5f), cy = FB_H / 2 + (int)(y * 0.5f);
            int t = (int)i * 255 / (vrec.count > 1 ? (int)vrec.count - 1 : 1);
            unsigned char r = (unsigned char)(40 + (t * 215) / 255);
            unsigned char g = (unsigned char)(80 + (t * 100) / 255);
            fb_rect(&fv, cx - 2, cy - 2, 5, 5, r, g, 220);
        }
        char hv[65]; frame_hash(&fv, hv);
        int vdiff = frame_diff(&fo, &fv);
        printf("old client: v1-only recording %s\n", hv);
        printf("old client: new recording    %s\n", ho);
        printf("old-version vs new-recording differs in %d bytes\n", vdiff);
        fprintf(pix, "snapshot,replay_v1(old only),replay_new(old client),%u,%s,%s,%d,%d\n",
                rec.count, hv, ho, vdiff, vdiff == 0);
        cell("snapshot", "replay_v1(old only)", "replay_new(old client)", vdiff == 0,
             "old client renders both identically");
    } else {
        cell("snapshot", "replay_v1(old only)", "replay_new(old client)", 0,
             "reference recording missing");
    }

    /* Negative control from disk: the naive recording put the new field
       before x/y.  The same old client must render something different, else
       the pixel comparison cannot detect the regression. */
    char naivepath[512];
    snprintf(naivepath, sizeof(naivepath), "%s/replay_naive.bin", recdir);
    Recording nrec;
    if (!recording_read(naivepath, &nrec)) {
        cell("snapshot", "v2_naive", "pinned_v1", 0, "negative recording missing");
        return;
    }
    Frame fnv; fb_clear(&fnv, 8, 12, 20);
    for (uint32_t i = 0; i < nrec.count; i++) {
        unsigned tk; float x, y;
        if (legacy_snapshot_decode(nrec.pkt[i], nrec.len[i], nrec.scale, &tk, &x, &y) <= 0) continue;
        int cx = FB_W / 2 + (int)(x * 0.5f), cy = FB_H / 2 + (int)(y * 0.5f);
        int t = (int)i * 255 / (nrec.count > 1 ? (int)nrec.count - 1 : 1);
        unsigned char r = (unsigned char)(40 + (t * 215) / 255);
        unsigned char g = (unsigned char)(80 + (t * 100) / 255);
        fb_rect(&fnv, cx - 2, cy - 2, 5, 5, r, g, 220);
    }
    int ndiff = frame_diff(&fo, &fnv);
    char hnv[65]; frame_hash(&fnv, hnv);
    printf("negative recording (field before x/y) %s differs=%d bytes\n", hnv, ndiff);
    fprintf(pix, "snapshot,v2_naive,pinned_v1,%u,%s,%s,%d,%d\n", nrec.count, ho, hnv,
            ndiff, ndiff == 0);
    cell("snapshot", "v2_naive", "pinned_v1", ndiff != 0,
         "field before x/y visibly moves the position");
}

/* ---------------------------------------------------------------------- */

int main(int argc, char** argv) {
    const char* dir = argc > 1 ? argv[1] : "evidence";
    const char* recdir = argc > 2 ? argv[2] : "accept/recordings";
    char path[512];

    int ec_ok = write_error_codes(dir);
    if (!ec_ok) failures++;

    snprintf(path, sizeof(path), "%s/compat_matrix.csv", dir);
    mtx = fopen(path, "w");
    if (!mtx) { fprintf(stderr, "cannot write %s\n", path); return 2; }
    fprintf(mtx, "module,producer,consumer,result,detail\n");

    snprintf(path, sizeof(path), "%s/pixel_identity.csv", dir);
    pix = fopen(path, "w");
    if (!pix) { fprintf(stderr, "cannot write %s\n", path); return 2; }

    snprintf(path, sizeof(path), "%s/body_hashes.csv", dir);
    bodylog = fopen(path, "w");
    if (!bodylog) { fprintf(stderr, "cannot write %s\n", path); return 2; }
    fprintf(bodylog, "module,producer,body,bytes,sha256\n");

    snapshot_accept();
    clock_accept();
    aoi_accept();
    reconcile_accept();
    error_path_accept();
    two_client_accept(recdir);

    fclose(mtx);
    fclose(pix);
    fclose(bodylog);

    const int REQUIRED_CELLS = 4 * 3 + 2 + 3 + 2 + 1 + 3;
    printf("\ncompat matrix cells: %d/%d pass\n", matrix_pass, matrix_cells);
    printf("required cells: %d (4 modules x 3 pairings + 2 error paths + 3 two-client + 2 negative controls + 1 old-reference + 3 no-invent)\n",
           REQUIRED_CELLS);
    if (matrix_cells != REQUIRED_CELLS || matrix_pass != REQUIRED_CELLS) {
        printf("MATRIX INCOMPLETE: any missing or failed cell fails acceptance\n");
        failures++;
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
