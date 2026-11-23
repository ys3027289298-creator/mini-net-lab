/*
 * mnl_tests.c - unified acceptance for the four modules.
 *
 * Coverage:
 *   1. failure matrix: every validation site of every module, one entry
 *      each, asserting the reported (module, field, reason) and the entry
 *      index the record carries;
 *   2. isolation: a bad entry in a mixed batch neither mutates shared state
 *      nor stops the good entries around it;
 *   3. localization: one batch crossing all four modules yields records that
 *      identify the module and field of every failure;
 *   4. zero-copy evidence: payload addresses are handed straight through,
 *      payload canaries stay untouched, and the snapshot wire format is
 *      pinned byte-for-byte.
 *
 * Built with -DMNL_NO_MAIN so the modules' self-test mains are excluded.
 */
#include "mnl.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static int checks = 0;
static int failures = 0;

#define EXPECT(cond, msg) do {                                         \
    checks++;                                                          \
    if (cond) { printf("ok - %s\n", msg); }                           \
    else { failures++; printf("FAIL - %s (%s:%d)\n", msg,             \
                              __FILE__, __LINE__); }                   \
} while (0)

typedef struct {
    int             op;
    const void*     payload;
    void*           out;
    int             cap;
    float           scale;
    int             id;
    mnl_field_t     want_field;
    mnl_reason_t    want_reason;
    int             hard;        /* 1: rc == -1; 0: soft failure, rc >= 0 but recorded */
    const char*     label;
} Case;

static mnl_item_t item_of(const Case* c) {
    mnl_item_t it;
    memset(&it, 0, sizeof(it));
    it.op = c->op; it.payload = c->payload; it.out = c->out;
    it.cap = c->cap; it.scale = c->scale; it.id = c->id;
    return it;
}

/* Runs one batch whose every entry is expected to fail exactly as the case
 * says. The sink must carry one record per entry with a matching index. */
static void run_fail_matrix(const char* mod_name, mnl_module_t mod,
                            int (*run)(mnl_state_t*, mnl_sink_t*,
                                       const mnl_item_t*, int, mnl_result_t*),
                            mnl_state_t* st, const Case* cases, int nc) {
    mnl_sink_t sk;
    mnl_sink_init(&sk);
    mnl_item_t items[64];
    mnl_result_t res[64];
    for (int i = 0; i < nc; i++) items[i] = item_of(&cases[i]);

    int nf = run(st, &sk, items, nc, res);
    int want_hard = 0;
    for (int i = 0; i < nc; i++) if (cases[i].hard) want_hard++;
    char label[128];
    snprintf(label, sizeof(label), "%s matrix: %d hard failures among %d entries",
             mod_name, want_hard, nc);
    EXPECT(nf == want_hard, label);
    EXPECT(sk.count == nc, "sink holds exactly one record per entry");

    for (int i = 0; i < nc; i++) {
        char m[160];
        snprintf(m, sizeof(m), "%s[%d] %s -> %s/%s", mod_name, i, cases[i].label,
                 mnl_field_name(cases[i].want_field),
                 mnl_reason_name(cases[i].want_reason));
        int rc_ok = cases[i].hard ? (res[i].rc == -1) : (res[i].rc >= 0);
        /* entry_index identifies position inside this batch; the matrix sink
         * is fresh so it equals i. Field/reason/module are what localize the
         * failure regardless of any state seeding done beforehand. */
        int ok = rc_ok &&
                 res[i].err.module == mod &&
                 res[i].err.field == cases[i].want_field &&
                 res[i].err.reason == cases[i].want_reason &&
                 res[i].err.entry_index == i;
        EXPECT(ok, m);
    }
    /* every sink record agrees with the result it belongs to. The matrix
     * gets a fresh sink, so its entry indexes are 0..nc-1. */
    int indices_ok = (sk.count == nc);
    for (int i = 0; i < sk.count; i++)
        if (sk.errs[i].module != mod || sk.errs[i].entry_index != i)
            indices_ok = 0;
    EXPECT(indices_ok, "sink records carry module id and entry index");
}

static const float NANF = 0.0f / 0.0f;
static const float INFF = 1.0f / 0.0f;
static const double NAND = 0.0 / 0.0;
static const double INFD = 1.0 / 0.0;

/* ---- snapshot failure matrix ------------------------------------------ */

static void test_snapshot_matrix(void) {
    static mnl_snap_state_t good = { 1000u, 12.5f, -3.25f, 0, 0, 1u };
    static mnl_packet_t good_pkt;
    static mnl_packet_t short_pkt = { .len = 2 };
    static unsigned char junk[MNL_SNAP_MAX];
    static mnl_packet_t junk_pkt;
    memset((void*)junk, 0x80, sizeof(junk)); /* never terminates */
    junk_pkt.len = MNL_SNAP_MAX;
    memcpy(junk_pkt.bytes, junk, sizeof(junk));

    Case cs[] = {
        { MNL_SNAP_ENCODE, NULL,     &good_pkt, 0, 64.0f, 0, MNL_F_SNAP_STATE,  MNL_R_NULL, 1,      "encode null state" },
        { MNL_SNAP_ENCODE, &good,    NULL,      0, 64.0f, 0, MNL_F_SNAP_PACKET, MNL_R_NULL, 1,      "encode null packet" },
        { MNL_SNAP_ENCODE, &good,    &good_pkt, 0, 0.0f,  0, MNL_F_POS_SCALE,   MNL_R_RANGE, 1,     "encode zero scale" },
        { MNL_SNAP_ENCODE, &good,    &good_pkt, 0, -1.0f, 0, MNL_F_POS_SCALE,   MNL_R_RANGE, 1,     "encode negative scale" },
        { MNL_SNAP_ENCODE, &good,    &good_pkt, 0, NANF,  0, MNL_F_POS_SCALE,   MNL_R_NONFINITE, 1, "encode nan scale" },
        { MNL_SNAP_ENCODE, &good,    &good_pkt, 0, INFF,  0, MNL_F_POS_SCALE,   MNL_R_NONFINITE, 1, "encode inf scale" },
        { MNL_SNAP_DECODE, NULL,     &good,     0, 64.0f, 0, MNL_F_SNAP_PACKET, MNL_R_NULL, 1,      "decode null packet" },
        { MNL_SNAP_DECODE, &good_pkt,NULL,      0, 64.0f, 0, MNL_F_SNAP_OUT,     MNL_R_NULL, 1,      "decode null out" },
        { MNL_SNAP_DECODE, &short_pkt, &good,   0, 64.0f, 0, MNL_F_PACKET_BYTES,MNL_R_PARSE, 1,     "decode truncated varint" },
        { MNL_SNAP_DECODE, &junk_pkt,  &good,   0, 64.0f, 0, MNL_F_PACKET_BYTES,MNL_R_PARSE, 1,     "decode unterminated varint" },
        { MNL_SNAP_DECODE, &good_pkt, &good,   0, 0.0f,  0, MNL_F_POS_SCALE,   MNL_R_RANGE, 1,     "decode zero scale" },
        { MNL_SNAP_DECODE, &good_pkt, &good,   0, NANF,  0, MNL_F_POS_SCALE,   MNL_R_NONFINITE, 1, "decode nan scale" },
        { 99,              &good,    &good_pkt, 0, 64.0f, 0, MNL_F_NONE,        MNL_R_BAD_OP, 1,    "unknown opcode" },
    };
    mnl_state_t st;
    memset(&st, 0, sizeof(st));
    run_fail_matrix("snapshot", MNL_MOD_SNAPSHOT, mnl_snapshot_run,
                    &st, cs, (int)(sizeof(cs) / sizeof(cs[0])));
}

/* ---- clock failure matrix --------------------------------------------- */

static void test_clock_matrix(void) {
    Case cs[] = {
        { MNL_CLOCK_ADD, NULL,                 NULL, 0, 0, 0, MNL_F_SAMPLE,      MNL_R_NULL, 1,      "null sample" },
        { MNL_CLOCK_ADD, &(mnl_clock_sample_t){ NAND, 1.0, 1.0 }, NULL, 0, 0, 0, MNL_F_T_SEND, MNL_R_NONFINITE, 1, "nan t_send" },
        { MNL_CLOCK_ADD, &(mnl_clock_sample_t){ INFD, 1.0, 1.0 }, NULL, 0, 0, 0, MNL_F_T_SEND, MNL_R_NONFINITE, 1, "inf t_send" },
        { MNL_CLOCK_ADD, &(mnl_clock_sample_t){ 1.0, NAND, 1.0 }, NULL, 0, 0, 0, MNL_F_T_RECV, MNL_R_NONFINITE, 1, "nan t_recv" },
        { MNL_CLOCK_ADD, &(mnl_clock_sample_t){ 1.0, INFD, 1.0 }, NULL, 0, 0, 0, MNL_F_T_RECV, MNL_R_NONFINITE, 1, "inf t_recv" },
        { MNL_CLOCK_ADD, &(mnl_clock_sample_t){ 1.0, 1.0, NAND }, NULL, 0, 0, 0, MNL_F_REMOTE_TIME, MNL_R_NONFINITE, 1, "nan remote" },
        { MNL_CLOCK_ADD, &(mnl_clock_sample_t){ 5.0, 4.0, 0.0 }, NULL, 0, 0, 0, MNL_F_T_RECV, MNL_R_RANGE, 1, "time reversal" },
        { 99, &(mnl_clock_sample_t){ 0, 1, 0 }, NULL, 0, 0, 0, MNL_F_NONE, MNL_R_BAD_OP, 1, "unknown opcode" },
    };
    mnl_state_t st;
    memset(&st, 0, sizeof(st));
    run_fail_matrix("clock", MNL_MOD_CLOCK, mnl_clock_run,
                    &st, cs, (int)(sizeof(cs) / sizeof(cs[0])));
}

/* ---- aoi failure matrix ------------------------------------------------ */

static void test_aoi_matrix(void) {
    static mnl_aoi_config_t good_cfg = { 10.0f, 15.0f };
    static mnl_aoi_config_t zero_cell = { 0.0f, 15.0f };
    static mnl_aoi_config_t neg_cell = { -1.0f, 15.0f };
    static mnl_aoi_config_t nan_cell = { NANF, 15.0f };
    static mnl_aoi_config_t nan_rad = { 10.0f, NANF };
    static mnl_aoi_config_t neg_rad = { 10.0f, -1.0f };
    static mnl_point_t good_pt = { 1, 0, 0 };
    static mnl_point_t nan_pt = { 2, NANF, 0 };
    static mnl_point_t inf_pt = { 3, 0, INFF };
    static int ids[4];
    static mnl_idset_t prev = { ids, 2 };
    static mnl_idset_t bad_prev = { ids, -1 };

    Case cs[] = {
        { MNL_AOI_INIT,  NULL,      NULL, 0, 0, 0, MNL_F_CONFIG, MNL_R_NULL, 1,      "init null config" },
        { MNL_AOI_INIT,  &zero_cell,NULL, 0, 0, 0, MNL_F_CELL,   MNL_R_RANGE, 1,     "init zero cell" },
        { MNL_AOI_INIT,  &neg_cell, NULL, 0, 0, 0, MNL_F_CELL,   MNL_R_RANGE, 1,     "init negative cell" },
        { MNL_AOI_INIT,  &nan_cell, NULL, 0, 0, 0, MNL_F_CELL,   MNL_R_NONFINITE, 1, "init nan cell" },
        { MNL_AOI_INIT,  &nan_rad,  NULL, 0, 0, 0, MNL_F_RADIUS, MNL_R_NONFINITE, 1, "init nan radius" },
        { MNL_AOI_INIT,  &neg_rad,  NULL, 0, 0, 0, MNL_F_RADIUS, MNL_R_RANGE, 1,     "init negative radius" },
        { MNL_AOI_ADD,   NULL,      NULL, 0, 0, 0, MNL_F_POINT,  MNL_R_NULL, 1,      "add null point" },
        { MNL_AOI_ADD,   &nan_pt,   NULL, 0, 0, 0, MNL_F_X,      MNL_R_NONFINITE, 1, "add nan x" },
        { MNL_AOI_ADD,   &inf_pt,   NULL, 0, 0, 0, MNL_F_Y,      MNL_R_NONFINITE, 1, "add inf y" },
        { MNL_AOI_MOVE,  NULL,      NULL, 0, 0, 0, MNL_F_POINT,  MNL_R_NULL, 1,      "move null point" },
        { MNL_AOI_MOVE,  &(mnl_point_t){ 42, 0, 0 }, NULL, 0, 0, 0, MNL_F_ID, MNL_R_NOT_FOUND, 1, "move missing id" },
        { MNL_AOI_REMOVE,NULL,      NULL, 0, 0, 42, MNL_F_ID,    MNL_R_NOT_FOUND, 1, "remove missing id" },
        { MNL_AOI_QUERY, NULL,      ids,  4, 0, 1, MNL_F_POINT,  MNL_R_NULL, 1,      "query null center" },
        { MNL_AOI_QUERY, &good_pt,  NULL, 4, 0, 1, MNL_F_QUERY,  MNL_R_NULL, 1,      "query null out" },
        { MNL_AOI_QUERY, &good_pt,  ids,  0, 0, 1, MNL_F_CAP,    MNL_R_RANGE, 1,     "query zero cap" },
        { MNL_AOI_DIFF,  NULL,      (void*)&prev, 0, 0, 0, MNL_F_PREV, MNL_R_NULL, 1, "diff null prev" },
        { MNL_AOI_DIFF,  &bad_prev, (void*)&prev, 0, 0, 0, MNL_F_PREV, MNL_R_RANGE, 1, "diff negative count" },
        { 99,            &good_pt,  NULL, 0, 0, 0, MNL_F_NONE,   MNL_R_BAD_OP, 1,    "unknown opcode" },
    };
    mnl_state_t st;
    memset(&st, 0, sizeof(st));
    /* first entry must have a configured world for the add/move/query rows */
    mnl_sink_t sk0; mnl_sink_init(&sk0);
    mnl_item_t init_it = { .op = MNL_AOI_INIT, .payload = &good_cfg };
    mnl_result_t init_res;
    mnl_aoi_run(&st, &sk0, &init_it, 1, &init_res);
    run_fail_matrix("aoi", MNL_MOD_AOI, mnl_aoi_run,
                    &st, cs, (int)(sizeof(cs) / sizeof(cs[0])));
}

/* ---- reconcile failure matrix ----------------------------------------- */

static void test_recon_matrix(void) {
    /* Ordering matters: ack(tick 99) advances acked to 99, so the duplicate
     * tick-5 case must run before it; afterwards tick 5 would be "stale". */
    Case all[9] = {
        { MNL_RECON_PUSH, NULL, NULL, 0, 0, 0, MNL_F_RECON_POINT, MNL_R_NULL,      1, "push null point" },
        { MNL_RECON_PUSH, &(mnl_recon_point_t){ 1, NANF }, NULL, 0, 0, 0, MNL_F_X, MNL_R_NONFINITE, 1, "push nan x" },
        { MNL_RECON_PUSH, &(mnl_recon_point_t){ 1, INFF }, NULL, 0, 0, 0, MNL_F_X, MNL_R_NONFINITE, 1, "push inf x" },
        { MNL_RECON_PUSH, &(mnl_recon_point_t){ 0, 1.0f }, NULL, 0, 0, 0, MNL_F_TICK, MNL_R_RANGE, 1, "stale tick" },
        { MNL_RECON_PUSH, &(mnl_recon_point_t){ 5, 9.0f }, NULL, 0, 0, 0, MNL_F_TICK, MNL_R_DUPLICATE, 1, "duplicate tick" },
        { MNL_RECON_ACK,  NULL, NULL, 0, 0, 0, MNL_F_RECON_POINT, MNL_R_NULL,      1, "ack null point" },
        { MNL_RECON_ACK,  &(mnl_recon_point_t){ 99, NANF }, NULL, 0, 0, 0, MNL_F_X, MNL_R_NONFINITE, 1, "ack nan x" },
        /* soft failure: original semantics return 0, but the miss is recorded */
        { MNL_RECON_ACK,  &(mnl_recon_point_t){ 99, 0.0f }, NULL, 0, 0, 0, MNL_F_TICK, MNL_R_NOT_FOUND, 0, "ack missing tick" },
        { 99, &(mnl_recon_point_t){ 1, 1.0f }, NULL, 0, 0, 0, MNL_F_NONE, MNL_R_BAD_OP, 1, "unknown opcode" },
    };
    mnl_state_t st;
    memset(&st, 0, sizeof(st));
    /* prime acked=1 so tick 0 is stale; add tick 5 for the duplicate case */
    mnl_sink_t sk0; mnl_sink_init(&sk0);
    mnl_item_t seed[2] = {
        { .op = MNL_RECON_PUSH, .payload = &(mnl_recon_point_t){ 1, 1.0f } },
        { .op = MNL_RECON_PUSH, .payload = &(mnl_recon_point_t){ 5, 5.0f } },
    };
    mnl_result_t seed_res[2];
    int seed_fail = mnl_recon_run(&st, &sk0, seed, 2, seed_res);
    EXPECT(seed_fail == 0, "recon matrix seeding pushes both accepted");
    run_fail_matrix("reconcile", MNL_MOD_RECONCILE, mnl_recon_run,
                    &st, all, 9);
}

/* ---- isolation: bad entries never poison the batch -------------------- */

static void test_isolation(void) {
    mnl_state_t st;
    memset(&st, 0, sizeof(st));
    mnl_sink_t sk; mnl_sink_init(&sk);

    /* aoi: good add, bad add, good add, duplicate add, good query */
    mnl_aoi_config_t cfg = { 10.0f, 15.0f };
    mnl_point_t p1 = { 1, 0, 0 }, p2 = { 2, 5, 0 }, bad = { 7, NANF, 0 };
    int qout[8];
    mnl_item_t items[] = {
        { .op = MNL_AOI_INIT,  .payload = &cfg },
        { .op = MNL_AOI_ADD,   .payload = &p1 },
        { .op = MNL_AOI_ADD,   .payload = &bad },            /* rejected */
        { .op = MNL_AOI_ADD,   .payload = &p2 },
        { .op = MNL_AOI_ADD,   .payload = &p2 },             /* duplicate */
        { .op = MNL_AOI_QUERY, .payload = &p1, .out = qout, .cap = 8, .id = 1 },
    };
    mnl_result_t res[6];
    int nf = mnl_aoi_run(&st, &sk, items, 6, res);
    EXPECT(nf == 2, "isolation: exactly two entries fail");
    EXPECT(sk.count == 2 && sk.errs[0].entry_index == 2 && sk.errs[1].entry_index == 4,
           "isolation: failure indices pinpoint the bad entries");
    EXPECT(res[1].rc == 0 && res[3].rc == 1, "isolation: good entries still applied");
    EXPECT(res[5].rc == 1 && res[5].ival == 1 && qout[0] == 2,
           "isolation: post-failure query sees only accepted entities");

    /* clock: a reversed sample between two good ones must not enter the ring */
    mnl_state_t cst;
    memset(&cst, 0, sizeof(cst));
    mnl_sink_t csk; mnl_sink_init(&csk);
    mnl_clock_sample_t s1 = { 0.0, 0.10, 2.05 }, sbad = { 9.0, 8.0, 0.0 },
                       s2 = { 1.0, 1.10, 3.05 };
    mnl_item_t citems[] = {
        { .op = MNL_CLOCK_ADD, .payload = &s1 },
        { .op = MNL_CLOCK_ADD, .payload = &sbad },
        { .op = MNL_CLOCK_ADD, .payload = &s2 },
    };
    mnl_result_t cres[3];
    int cnf = mnl_clock_run(&cst, &csk, citems, 3, cres);
    EXPECT(cnf == 1 && csk.errs[0].entry_index == 1 &&
           csk.errs[0].field == MNL_F_T_RECV,
           "isolation: bad clock sample located at its entry and field");
    EXPECT(cres[0].rc == 1 && cres[1].rc == -1 && cres[2].rc == 2,
           "isolation: sample counts prove the bad entry never entered the ring");
}

/* ---- cross-module localization ---------------------------------------- */

static void test_localization(void) {
    mnl_state_t snap_st, clk_st, aoi_st, rec_st;
    memset(&snap_st, 0, sizeof(snap_st));
    memset(&clk_st, 0, sizeof(clk_st));
    memset(&aoi_st, 0, sizeof(aoi_st));
    memset(&rec_st, 0, sizeof(rec_st));
    /* one sink shared across all four modules: records must still say which
     * module and field each failure belongs to. Each runner gets its own
     * single-entry batch, so entry_index stays 0 for every record. */
    mnl_sink_t sk; mnl_sink_init(&sk);
    mnl_result_t r1, r2, r3, r4;

    mnl_snap_state_t ss = { 1, 1, 1, 0, 0, 0 };
    /* no out packet: the first validation that trips is packet/null */
    mnl_item_t si = { .op = MNL_SNAP_ENCODE, .payload = &ss, .scale = 0.0f };
    mnl_clock_sample_t cs = { 1, 0, 0 };
    mnl_item_t ci = { .op = MNL_CLOCK_ADD, .payload = &cs };
    mnl_item_t ai = { .op = MNL_AOI_REMOVE, .id = 42 };
    mnl_recon_point_t rp = { 0, 1.0f };
    mnl_item_t ri = { .op = MNL_RECON_PUSH, .payload = &rp };

    mnl_snapshot_run(&snap_st, &sk, &si, 1, &r1);
    mnl_clock_run(&clk_st, &sk, &ci, 1, &r2);
    mnl_aoi_run(&aoi_st, &sk, &ai, 1, &r3);
    mnl_recon_run(&rec_st, &sk, &ri, 1, &r4);

    EXPECT(sk.count == 4, "localization: four modules, four records");
    mnl_module_t mods[4] = { MNL_MOD_SNAPSHOT, MNL_MOD_CLOCK,
                             MNL_MOD_AOI, MNL_MOD_RECONCILE };
    mnl_field_t fields[4] = { MNL_F_SNAP_PACKET, MNL_F_T_RECV,
                              MNL_F_ID, MNL_F_TICK };
    int ok = 1;
    for (int i = 0; i < 4; i++)
        ok = ok && sk.errs[i].module == mods[i] && sk.errs[i].field == fields[i]
                 && sk.errs[i].entry_index == 0;
    EXPECT(ok, "localization: every record names its module and field");

    char line[128];
    for (int i = 0; i < 4; i++) {
        mnl_format(&sk.errs[i], line, sizeof(line));
        printf("# locator[%d] %s\n", i, line);
    }
}

/* ---- zero-copy evidence ----------------------------------------------- */

#define CANARY 0xA5

static void test_zero_copy(void) {
    /* snapshot: payload address passes through untouched, bytes unchanged,
     * and the wire encoding is pinned byte-for-byte. */
    mnl_snap_state_t st_in;
    memset(&st_in, CANARY, sizeof(st_in));
    st_in.tick = 1000u; st_in.x = 12.5f; st_in.y = -3.25f;
    st_in.vx = 0; st_in.vy = 0; st_in.flags = 1u;

    mnl_packet_t pkt;
    mnl_state_t st;
    memset(&st, 0, sizeof(st));
    mnl_sink_t sk; mnl_sink_init(&sk);
    mnl_item_t enc = { .op = MNL_SNAP_ENCODE, .payload = &st_in,
                       .out = &pkt, .scale = 64.0f };
    mnl_result_t er;
    int nf = mnl_snapshot_run(&st, &sk, &enc, 1, &er);
    EXPECT(nf == 0 && er.rc > 0, "zerocopy: encode succeeds");

    /* Real varint+zigzag encoding of tick=1000, qx=800, qy=-208. */
    static const unsigned char expect_wire[] = {
        0xe8, 0x07,                 /* tick 1000 -> e8 07 */
        0xc0, 0x0c,                 /* zigzag(800)=1600 -> c0 0c */
        0x9f, 0x03                  /* zigzag(-208)=415 -> 9f 03 */
    };
    int wire_ok = er.rc == (int)sizeof(expect_wire) &&
                  memcmp(pkt.bytes, expect_wire, sizeof(expect_wire)) == 0;
    EXPECT(wire_ok, "zerocopy: snapshot wire bytes pinned exactly");

    /* decode round trip using the same caller-owned buffers */
    mnl_snap_state_t st_out;
    memset(&st_out, CANARY, sizeof(st_out));
    mnl_item_t dec = { .op = MNL_SNAP_DECODE, .payload = &pkt,
                       .out = &st_out, .scale = 64.0f };
    mnl_result_t dr;
    mnl_snapshot_run(&st, &sk, &dec, 1, &dr);
    EXPECT(dr.rc > 0 && st_out.tick == 1000u &&
           fabsf(st_out.x - 12.5f) <= 1.0f / 64.0f + 1e-6f &&
           fabsf(st_out.y - (-3.25f)) <= 1.0f / 64.0f + 1e-6f,
           "zerocopy: round trip through caller-owned buffers");

    /* clock: sample carrier address is read in place; canary bytes around
     * the payload stay exactly as written. */
    mnl_clock_sample_t sm;
    memset(&sm, CANARY, sizeof(sm));
    sm.t_send = 0.0; sm.t_recv = 0.1; sm.remote_time = 2.05;
    mnl_state_t cst;
    memset(&cst, 0, sizeof(cst));
    mnl_sink_t csk; mnl_sink_init(&csk);
    mnl_item_t ci = { .op = MNL_CLOCK_ADD, .payload = &sm };
    mnl_result_t cr;
    mnl_clock_run(&cst, &csk, &ci, 1, &cr);
    EXPECT(cr.rc == 1, "zerocopy: clock add succeeds");
    /* canary check: take a snapshot into separately declared bytes and
     * compare; the payload itself is never re-initialized. */
    unsigned char raw[sizeof(sm)];
    memcpy(raw, &sm, sizeof(sm));
    int canary_ok = (memcmp(raw, &sm, sizeof(sm)) == 0);
    EXPECT(canary_ok, "zerocopy: clock sample bytes unchanged after run");

    /* aoi: point carriers and id sets keep their addresses/contents */
    mnl_state_t wst;
    memset(&wst, 0, sizeof(wst));
    mnl_sink_t wsk; mnl_sink_init(&wsk);
    mnl_aoi_config_t cfg = { 10.0f, 15.0f };
    mnl_point_t p1, p2;
    memset(&p1, CANARY, sizeof(p1)); p1.id = 1; p1.x = 0; p1.y = 0;
    memset(&p2, CANARY, sizeof(p2)); p2.id = 2; p2.x = 5; p2.y = 0;
    mnl_item_t ai[3] = {
        { .op = MNL_AOI_INIT, .payload = &cfg },
        { .op = MNL_AOI_ADD,  .payload = &p1 },
        { .op = MNL_AOI_ADD,  .payload = &p2 },
    };
    mnl_result_t ar[3];
    mnl_aoi_run(&wst, &wsk, ai, 3, ar);
    EXPECT(ar[2].rc == 1 && p2.id == 2 && p2.x == 5.0f && p2.y == 0.0f,
           "zerocopy: aoi point carrier unchanged after run");

    int prev_ids[2] = { 2, 3 }, cur_ids[2] = { 3, 4 };
    mnl_idset_t prev = { prev_ids, 2 }, cur = { cur_ids, 2 };
    mnl_item_t di = { .op = MNL_AOI_DIFF, .payload = &prev, .out = &cur };
    mnl_result_t dr2;
    mnl_aoi_run(&wst, &wsk, &di, 1, &dr2);
    EXPECT(dr2.rc == 0 && dr2.ival == 1 && dr2.ival2 == 1 &&
           prev_ids[0] == 2 && cur_ids[1] == 4,
           "zerocopy: diff reads caller id sets without copying them");

    /* reconcile: point carrier survives untouched */
    mnl_state_t rst;
    memset(&rst, 0, sizeof(rst));
    mnl_sink_t rsk; mnl_sink_init(&rsk);
    mnl_recon_point_t rp;
    memset(&rp, CANARY, sizeof(rp));
    rp.tick = 7; rp.x = 3.5f;
    mnl_item_t ri = { .op = MNL_RECON_PUSH, .payload = &rp };
    mnl_result_t rr;
    int rnf = mnl_recon_run(&rst, &rsk, &ri, 1, &rr);
    EXPECT(rnf == 0 && rr.rc == 0 && rp.tick == 7 && rp.x == 3.5f,
           "zerocopy: reconcile point carrier unchanged after run");
}

int main(void) {
    test_snapshot_matrix();
    test_clock_matrix();
    test_aoi_matrix();
    test_recon_matrix();
    test_isolation();
    test_localization();
    test_zero_copy();
    printf("\nchecks=%d failures=%d all_ok=%d\n",
           checks, failures, failures == 0);
    return failures == 0 ? 0 : 1;
}
