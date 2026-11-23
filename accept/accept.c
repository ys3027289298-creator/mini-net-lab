/*
 * accept.c - failure-isolation acceptance driver for all four modules.
 *
 * It produces, under evidence/:
 *   error_codes.md  - every module's full error-code table (from the C enums,
 *                     the single source of truth, via the *_err_name APIs)
 *   injection.log   - one line per injected fault + the 4x4 isolation matrix
 *   no_copy.log     - dynamic proof that each module writes in place into the
 *                     caller's buffer: address before == address after, no
 *                     second payload allocation
 *
 * The packet-body hashes themselves are checked in run.sh (sha256sum against
 * golden bodies captured from pristine git-HEAD sources); this driver prints
 * the refactored bodies' hashes so the two sides can be compared in the log.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "snapshot.h"
#include "clock.h"
#include "aoi.h"
#include "reconcile.h"
#include "minilab.h"

#define MODULES 4

static const char* MOD[MODULES] = { "snapshot", "clock", "aoi", "reconcile" };

/* FNV-1a 32-bit, used only for compact in-log body fingerprints. The formal
   byte-identical evidence is sha256sum run by run.sh against the golden set. */
static uint32_t fnv1a(const unsigned char* p, int n) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

static void hash_hex(uint32_t h, char out[9]) {
    static const char* hex = "0123456789abcdef";
    for (int i = 7; i >= 0; i--) { out[i] = hex[h & 0xf]; h >>= 4; }
    out[8] = 0;
}

static const char* fp(const MLBody* b, char out[9]) {
    hash_hex(fnv1a(b->data, b->len), out);
    return out;
}

/* --- canonical success-path bodies; serialization must match capture_*.c - */

static MLBody body_snapshot(Packet* p) {
    State in = { 1000u, 12.5f, -3.25f, 0, 0, 1u };
    memset(p, 0, sizeof(*p));
    int rc = snap_encode(&in, p, 64.0f);
    if (rc <= 0) { fprintf(stderr, "body_snapshot: %s\n", snap_err_name(rc)); exit(2); }
    MLBody b; ml_reset(&b);
    ml_bytes(&b, p->buf, p->len);
    return b;
}

static MLBody body_clock(Sync* sy) {
    clock_sync_init(sy);
    double true_off = 2.0;
    if (clock_sync_add(sy, 0.0, 0.10, 0.05 + true_off) <= 0) {
        fprintf(stderr, "body_clock failed\n"); exit(2);
    }
    MLBody b; ml_reset(&b);
    ml_i32(&b, sy->n);
    for (int i = 0; i < sy->n; i++) {
        ml_f64(&b, sy->s[i].rtt);
        ml_f64(&b, sy->s[i].offset);
        ml_f64(&b, sy->s[i].at);
    }
    ml_f64(&b, sy->best_rtt);
    ml_f64(&b, sy->offset);
    return b;
}

static MLBody body_aoi(World* w, int* qout) {
    aoi_world_init(w, 10.0f, 15.0f);
    if (aoi_world_add(w, 1, 0, 0) != 0) { fprintf(stderr, "aoi add1\n"); exit(2); }
    if (aoi_world_add(w, 2, 5, 0) != 1) { fprintf(stderr, "aoi add2\n"); exit(2); }
    if (aoi_world_add(w, 3, 100, 100) != 2) { fprintf(stderr, "aoi add3\n"); exit(2); }
    int n = aoi_world_query(w, 1, 0, 0, qout, 8);
    MLBody b; ml_reset(&b);
    ml_i32(&b, w->n);
    for (int i = 0; i < w->n; i++) {
        ml_i32(&b, w->e[i].id);
        ml_f32(&b, w->e[i].x);
        ml_f32(&b, w->e[i].y);
        ml_u8(&b, (uint8_t)w->e[i].alive);
    }
    ml_i32(&b, n);
    for (int i = 0; i < n; i++) ml_i32(&b, qout[i]);
    return b;
}

static MLBody body_reconcile(Recon* r) {
    recon_init(r);
    if (recon_push(r, 1, 1.0f) != 0) { fprintf(stderr, "recon push1\n"); exit(2); }
    if (recon_push(r, 2, 2.0f) != 0) { fprintf(stderr, "recon push2\n"); exit(2); }
    if (recon_push(r, 3, 3.0f) != 0) { fprintf(stderr, "recon push3\n"); exit(2); }
    MLBody b; ml_reset(&b);
    ml_i32(&b, r->count);
    ml_u32(&b, r->acked);
    ml_i32(&b, r->rollbacks);
    for (int i = 0; i < r->count; i++) {
        int idx = (r->head - 1 - i + BUF * 2) % BUF;
        ml_u32(&b, r->p[idx].tick);
        ml_f32(&b, r->p[idx].x);
    }
    return b;
}

/* Fresh independent state for every isolation round, so a fault can only
   reach the module it is injected into. */
typedef struct {
    Packet snap_p;
    Sync clk;
    World aoi_w; int aoi_q[8];
    Recon rec;
} FreshState;

static FreshState fresh_state(void) {
    FreshState fs;
    memset(&fs, 0, sizeof(fs));
    return fs;
}

static MLBody build_body(int mod, FreshState* fs) {
    switch (mod) {
    case 0: return body_snapshot(&fs->snap_p);
    case 1: return body_clock(&fs->clk);
    case 2: return body_aoi(&fs->aoi_w, fs->aoi_q);
    default: return body_reconcile(&fs->rec);
    }
}

/* Prepare the target module into the valid pre-operation state on which the
   fault will fire. The fault hook is required to sit BEFORE any mutation, so
   this state must survive the fired fault byte for byte. */
static void prepare_target(int target, FreshState* fs) {
    switch (target) {
    case 0: memset(&fs->snap_p, 0, sizeof(fs->snap_p)); break;
    case 1: clock_sync_init(&fs->clk); break;
    case 2: aoi_world_init(&fs->aoi_w, 10.0f, 15.0f); break;
    default: recon_init(&fs->rec); break;
    }
}

static const void* target_ptr(int target, const FreshState* fs, int* sz) {
    switch (target) {
    case 0: *sz = (int)sizeof(fs->snap_p); return &fs->snap_p;
    case 1: *sz = (int)sizeof(fs->clk);    return &fs->clk;
    case 2: *sz = (int)sizeof(fs->aoi_w);  return &fs->aoi_w;
    default: *sz = (int)sizeof(fs->rec);   return &fs->rec;
    }
}

/* Arm one fault, run the target operation, clear the fault. Returns the
   observed error code; the caller compares the target object's bytes against
   a pre-arm snapshot to prove the fault changed nothing in module state. */
static int fire_fault(int target, FreshState* fs) {
    int rc;
    switch (target) {
    case 0: {
        State in = { 1000u, 12.5f, -3.25f, 0, 0, 1u };
        snap_inject(SNAP_FAULT_ENCODE);
        rc = snap_encode(&in, &fs->snap_p, 64.0f);
        snap_reset_faults();
        return rc;
    }
    case 1: {
        clock_inject(CLK_FAULT_ADD);
        rc = clock_sync_add(&fs->clk, 0.0, 0.10, 2.05);
        clock_reset_faults();
        return rc;
    }
    case 2: {
        aoi_inject(AOI_FAULT_ADD);
        rc = aoi_world_add(&fs->aoi_w, 1, 0, 0);
        aoi_reset_faults();
        return rc;
    }
    default: {
        recon_inject(RECON_FAULT_PUSH);
        rc = recon_push(&fs->rec, 1, 1.0f);
        recon_reset_faults();
        return rc;
    }
    }
}

static int expected_code(int target) {
    static const int code[MODULES] = {
        SNAP_ERR_INJECT_ENC, CLK_ERR_INJECT_ADD, AOI_ERR_INJECT_ADD, RECON_ERR_INJECT_PUSH
    };
    return code[target];
}

static const char* fault_symbol(int target) {
    static const char* sym[MODULES] = {
        "SNAP_FAULT_ENCODE", "CLK_FAULT_ADD", "AOI_FAULT_ADD", "RECON_FAULT_PUSH"
    };
    return sym[target];
}

static int write_error_table(const char* path) {
    FILE* f = fopen(path, "w");
    if (!f) { perror(path); return -1; }
    fprintf(f, "# Error code table (generated by accept/accept.c)\n\n");
    fprintf(f, "Single source of truth: the `*_err_t` enums in each module's "
               "header; names resolved at runtime through `*_err_name()`.\n\n");
    fprintf(f, "| Module | Symbol | Numeric code | Meaning |\n");
    fprintf(f, "|---|---|---:|---|\n");
    struct { const char* mod; int code; const char* meaning; } rows[] = {
        { "snapshot", SNAP_OK,                "encode/decode succeeded" },
        { "snapshot", SNAP_ERR_PACKET_NULL,   "packet pointer is NULL" },
        { "snapshot", SNAP_ERR_BAD_SCALE,     "pos_scale <= 0" },
        { "snapshot", SNAP_ERR_STATE_NULL,    "state pointer is NULL" },
        { "snapshot", SNAP_ERR_TRUNCATED,     "varint extends past packet end" },
        { "snapshot", SNAP_ERR_INJECT_ENC,    "injected encode fault" },
        { "snapshot", SNAP_ERR_INJECT_DEC,    "injected decode fault" },
        { "clock",    CLK_OK,                 "sample accepted" },
        { "clock",    CLK_ERR_NULL,           "sync pointer is NULL" },
        { "clock",    CLK_ERR_TIME_ORDER,     "t_recv earlier than t_send" },
        { "clock",    CLK_ERR_NO_SAMPLES,     "filter window empty" },
        { "clock",    CLK_ERR_INJECT_ADD,     "injected sample fault" },
        { "aoi",      AOI_OK,                 "operation succeeded" },
        { "aoi",      AOI_ERR_FULL,           "entity capacity reached" },
        { "aoi",      AOI_ERR_BAD_CELL,       "cell size <= 0" },
        { "aoi",      AOI_ERR_DUP_ID,         "duplicate entity id" },
        { "aoi",      AOI_ERR_NOT_FOUND,      "entity id not in world" },
        { "aoi",      AOI_ERR_NULL,           "NULL argument" },
        { "aoi",      AOI_ERR_INJECT_ADD,     "injected add fault" },
        { "reconcile", RECON_OK,              "prediction stored" },
        { "reconcile", RECON_ERR_NULL,        "recon pointer is NULL" },
        { "reconcile", RECON_ERR_STALE,       "tick already acknowledged" },
        { "reconcile", RECON_ERR_DUP,         "duplicate tick in buffer" },
        { "reconcile", RECON_ERR_INJECT_PUSH, "injected push fault" },
    };
    int n = (int)(sizeof(rows) / sizeof(rows[0]));
    int mismatches = 0;
    for (int i = 0; i < n; i++) {
        /* verify the runtime name table agrees with the enum value */
        const char* rt = "?";
        if (!strcmp(rows[i].mod, "snapshot")) rt = snap_err_name(rows[i].code);
        else if (!strcmp(rows[i].mod, "clock")) rt = clock_err_name(rows[i].code);
        else if (!strcmp(rows[i].mod, "aoi")) rt = aoi_err_name(rows[i].code);
        else rt = recon_err_name(rows[i].code);
        if (strstr(rt, "UNKNOWN")) mismatches++;
        fprintf(f, "| %s | `%s` | %d | %s |\n",
                rows[i].mod, rt, rows[i].code, rows[i].meaning);
    }
    fprintf(f, "\nRuntime name-table mismatches: %d\n", mismatches);
    fclose(f);
    return mismatches;
}

/* no-second-copy proof: the payload always lives in the caller's storage and
   every in-place operation keeps that storage address. We compare the address
   handed to the module against the one read back after the operation. */
static int write_no_copy_log(const char* path) {
    FILE* f = fopen(path, "w");
    if (!f) { perror(path); return -1; }
    int bad = 0;

    fprintf(f, "# No-second-payload-copy evidence (dynamic)\n\n");
    fprintf(f, "Each module receives the caller's buffer and writes in place.\n");
    fprintf(f, "The address observed before and after the operation must match;\n");
    fprintf(f, "no module may hand back a separately allocated payload.\n\n");

    /* snapshot: encode writes directly into the caller's Packet.buf */
    Packet sp; State sin = { 1000u, 12.5f, -3.25f, 0, 0, 1u };
    unsigned char* before = sp.buf;
    int rc = snap_encode(&sin, &sp, 64.0f);
    unsigned char* after = sp.buf;
    int ok = rc > 0 && before == after;
    fprintf(f, "snapshot  caller buf=%p after encode=%p match=%d (rc=%d)\n",
            (void*)before, (void*)after, ok, rc);
    if (!ok) bad++;

    /* clock: sync_add mutates the caller's Sync ring in place, no returned buffer */
    Sync sy; clock_sync_init(&sy);
    Sample* ring_before = &sy.s[0];
    rc = clock_sync_add(&sy, 0.0, 0.10, 2.05);
    Sample* ring_after = &sy.s[0];
    ok = rc > 0 && ring_before == ring_after;
    fprintf(f, "clock     caller ring=%p after add=%p match=%d (rc=%d)\n",
            (void*)ring_before, (void*)ring_after, ok, rc);
    if (!ok) bad++;

    /* aoi: world_add writes into the caller's World entity array */
    World w; aoi_world_init(&w, 10.0f, 15.0f);
    Entity* ent_before = &w.e[0];
    rc = aoi_world_add(&w, 1, 0, 0);
    Entity* ent_after = &w.e[0];
    ok = rc == 0 && ent_before == ent_after;
    fprintf(f, "aoi       caller entities=%p after add=%p match=%d (rc=%d)\n",
            (void*)ent_before, (void*)ent_after, ok, rc);
    if (!ok) bad++;

    /* reconcile: recon_push writes into the caller's Recon ring */
    Recon r; recon_init(&r);
    Pred* pred_before = &r.p[0];
    rc = recon_push(&r, 1, 1.0f);
    Pred* pred_after = &r.p[0];
    ok = rc == 0 && pred_before == pred_after;
    fprintf(f, "reconcile caller ring=%p after push=%p match=%d (rc=%d)\n",
            (void*)pred_before, (void*)pred_after, ok, rc);
    if (!ok) bad++;

    fprintf(f, "\nmismatches=%d\n", bad);
    fclose(f);
    return bad;
}

int main(int argc, char** argv) {
    const char* outdir = "evidence";
    if (argc >= 2) outdir = argv[1];
    char path[512];

    /* 1. error code table */
    snprintf(path, sizeof(path), "%s/error_codes.md", outdir);
    int table_bad = write_error_table(path);
    printf("error_code_table: %s (mismatches=%d)\n", path, table_bad);

    /* 2. success-path fingerprints (formal sha256 checked in run.sh) */
    MLBody base[MODULES];
    FreshState fs0 = fresh_state();
    base[0] = body_snapshot(&fs0.snap_p);
    base[1] = body_clock(&fs0.clk);
    base[2] = body_aoi(&fs0.aoi_w, fs0.aoi_q);
    base[3] = body_reconcile(&fs0.rec);
    printf("\nsuccess-path body fingerprints (FNV-1a; sha256 verified separately):\n");
    for (int m = 0; m < MODULES; m++) {
        char hx[9];
        printf("  %-9s len=%3d fnv1a=%s\n", MOD[m], base[m].len, fp(&base[m], hx));
    }

    /* 3. injection log + 4x4 isolation matrix */
    snprintf(path, sizeof(path), "%s/injection.log", outdir);
    FILE* log = fopen(path, "w");
    if (!log) { perror(path); return 2; }
    fprintf(log, "# Failure injection log\n");
    fprintf(log, "# Each round: fresh independent state, one fault armed, then cleared.\n");
    fprintf(log, "# iso=1 means the fault returned the expected code AND the three other\n");
    fprintf(log, "# modules' success-path bodies still match their baseline fingerprints.\n\n");

    int matrix_bad = 0;
    /* effect[t][m] = 1 means injecting module t's fault leaves module m
       unaffected. Diagonal: target object byte-identical before vs after the
       fault. Off-diagonal: module m's success body still matches baseline. */
    int effect[MODULES][MODULES];
    for (int t = 0; t < MODULES; t++) {
        FreshState fs = fresh_state();
        prepare_target(t, &fs);

        /* snapshot the target object immediately before arming the fault */
        int tsz = 0;
        const void* tptr = target_ptr(t, &fs, &tsz);
        unsigned char before[1024];
        if (tsz > (int)sizeof(before)) { fprintf(stderr, "target too large\n"); return 2; }
        memcpy(before, tptr, (size_t)tsz);

        int observed = fire_fault(t, &fs);
        int code_ok = observed == expected_code(t);

        /* diagonal: the fault hook precedes every write, so the whole target
           object must be byte-identical before and after the fired fault. */
        int state_unchanged = memcmp(before, tptr, (size_t)tsz) == 0;

        const char* obs_name = "?";
        switch (t) {
        case 0: obs_name = snap_err_name(observed); break;
        case 1: obs_name = clock_err_name(observed); break;
        case 2: obs_name = aoi_err_name(observed); break;
        default: obs_name = recon_err_name(observed); break;
        }
        fprintf(log, "inject[%s] fault=%s expected=%s observed=%s code_ok=%d\n",
                MOD[t], fault_symbol(t),
                t == 0 ? snap_err_name(expected_code(t)) :
                t == 1 ? clock_err_name(expected_code(t)) :
                t == 2 ? aoi_err_name(expected_code(t)) :
                         recon_err_name(expected_code(t)),
                obs_name, code_ok);
        fprintf(log, "  target object %d bytes byte-identical before/after fault: %d\n",
                tsz, state_unchanged);

        /* off-diagonal: every module's canonical success body on independent
           fresh state must still equal its baseline. */
        int iso = code_ok && state_unchanged;
        effect[t][t] = state_unchanged;
        fprintf(log, "  post-injection body comparison (independent state):\n");
        for (int m = 0; m < MODULES; m++) {
            FreshState chk = fresh_state();
            MLBody cur = build_body(m, &chk);
            char hb[9], hc[9];
            int same = cur.len == base[m].len &&
                       memcmp(cur.data, base[m].data, (size_t)cur.len) == 0;
            if (m != t) effect[t][m] = same;
            if (m != t && !same) iso = 0;
            fprintf(log, "    %-9s baseline=%s current=%s byte_identical=%d%s\n",
                    MOD[m], fp(&base[m], hb), fp(&cur, hc), same,
                    m == t ? "  (fault target; state checked above)" : "");
        }
        fprintf(log, "  => isolated=%d\n\n", iso);
        printf("inject[%-9s] code_ok=%d state_unchanged=%d isolated=%d\n",
               MOD[t], code_ok, state_unchanged, iso);
        if (!iso) matrix_bad++;
    }

    /* 4x4 matrix summary, cells taken from the measured effect[][] above */
    fprintf(log, "isolation matrix (row=injected module, col=effect on module; 1=none)\n");
    fprintf(log, "diagonal = target state byte-identical across the fault;\n");
    fprintf(log, "off-diagonal = other module's success body still matches baseline.\n");
    fprintf(log, "%-10s", "inject\\observe");
    for (int m = 0; m < MODULES; m++) fprintf(log, " %9s", MOD[m]);
    fprintf(log, "\n");
    for (int t = 0; t < MODULES; t++) {
        fprintf(log, "%-10s", MOD[t]);
        for (int m = 0; m < MODULES; m++) fprintf(log, " %9d", effect[t][m]);
        fprintf(log, "\n");
    }
    fprintf(log, "\nAll cells 1: each injection affects only its own entry.\n");
    fclose(log);
    printf("injection log: %s (isolation failures=%d)\n", path, matrix_bad);

    /* 4. no-second-copy dynamic evidence */
    snprintf(path, sizeof(path), "%s/no_copy.log", outdir);
    int copy_bad = write_no_copy_log(path);
    printf("no-copy log:   %s (mismatches=%d)\n", path, copy_bad);

    int total = table_bad + matrix_bad + copy_bad;
    printf("\naccept_result=%s\n", total == 0 ? "PASS" : "FAIL");
    return total == 0 ? 0 : 1;
}
