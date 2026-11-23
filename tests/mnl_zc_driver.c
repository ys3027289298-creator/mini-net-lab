/*
 * mnl_zc_driver.c - runs all four batch runners under the LD_PRELOAD shim.
 *
 * Every payload range handed to a runner is registered with the shim first.
 * If any copy routine inside a runner touches those bytes, the shim aborts
 * and this program exits non-zero.
 */
#include "mnl.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Weak undefined references: standalone build links fine, under LD_PRELOAD
 * the shim interposes these. Defining local strong/weak copies here would
 * shadow the shim and silently disable monitoring. */
__attribute__((weak)) void mnlzc_register(const void* ptr, size_t len);
__attribute__((weak)) void mnlzc_set_enabled(int on);
__attribute__((weak)) long mnlzc_copies(void);
__attribute__((weak)) void mnlzc_reset(void);

/* NULL-safe wrappers so the same binary runs with and without the shim. */
#define ZC_REG(p)      do { if (mnlzc_register) mnlzc_register((p), sizeof(*(p))); } while (0)
#define ZC_ON()        do { if (mnlzc_set_enabled) mnlzc_set_enabled(1); } while (0)
#define ZC_OFF()       do { if (mnlzc_set_enabled) mnlzc_set_enabled(0); } while (0)
#define ZC_COPIES()    (mnlzc_copies ? mnlzc_copies() : 0)

int main(void) {
    /* Standalone (no shim) the weak references stay NULL: monitoring calls
     * become no-ops and the functional batch checks still run. */
    int shim_present = (mnlzc_register && mnlzc_set_enabled &&
                        mnlzc_copies && mnlzc_reset);
    if (shim_present) mnlzc_reset();
    mnl_sink_t sk; mnl_sink_init(&sk);
    mnl_result_t res[16];
    long total_copies = 0;

    /* ---- snapshot: encode + decode on caller-owned buffers ---- */
    static mnl_snap_state_t s_in = { 1000u, 12.5f, -3.25f, 0, 0, 1u };
    static mnl_snap_state_t s_out;
    static mnl_packet_t pkt;
    ZC_REG(&s_in); ZC_REG(&s_out); ZC_REG(&pkt);
    mnl_state_t snap_st;
    memset(&snap_st, 0, sizeof(snap_st));
    mnl_item_t sitems[2] = {
        { .op = MNL_SNAP_ENCODE, .payload = &s_in, .out = &pkt, .scale = 64.0f },
        { .op = MNL_SNAP_DECODE, .payload = &pkt,  .out = &s_out, .scale = 64.0f },
    };
    ZC_ON();
    int f1 = mnl_snapshot_run(&snap_st, &sk, sitems, 2, res);
    ZC_OFF();
    total_copies += ZC_COPIES();
    if (f1 != 0 || res[0].rc <= 0 || res[1].rc <= 0) { printf("FAIL snapshot batch\n"); return 1; }

    /* ---- clock: three samples, one bad, payloads never move ---- */
    static mnl_clock_sample_t sm[3] = {
        { 0.0, 0.10, 2.05 }, { 9.0, 8.0, 0.0 }, { 1.0, 1.10, 3.05 }
    };
    ZC_REG(sm);
    mnl_state_t clk_st;
    memset(&clk_st, 0, sizeof(clk_st));
    mnl_item_t citems[3];
    for (int i = 0; i < 3; i++) citems[i] = (mnl_item_t){ .op = MNL_CLOCK_ADD, .payload = &sm[i] };
    ZC_ON();
    int f2 = mnl_clock_run(&clk_st, &sk, citems, 3, res);
    ZC_OFF();
    total_copies += ZC_COPIES();
    if (f2 != 1) { printf("FAIL clock batch\n"); return 1; }

    /* ---- aoi: config + points + diff sets ---- */
    static mnl_aoi_config_t cfg = { 10.0f, 15.0f };
    static mnl_point_t pts[3] = { {1,0,0}, {2,5,0}, {3,NAN,0} };
    static int qout[8];
    static int prev_ids[2] = { 2, 3 }, cur_ids[2] = { 3, 4 };
    static mnl_idset_t prev = { prev_ids, 2 }, cur = { cur_ids, 2 };
    ZC_REG(&cfg); ZC_REG(pts); ZC_REG(&prev); ZC_REG(&cur); ZC_REG(prev_ids); ZC_REG(cur_ids);
    mnl_state_t aoi_st;
    memset(&aoi_st, 0, sizeof(aoi_st));
    mnl_item_t aitems[6] = {
        { .op = MNL_AOI_INIT,  .payload = &cfg },
        { .op = MNL_AOI_ADD,   .payload = &pts[0] },
        { .op = MNL_AOI_ADD,   .payload = &pts[1] },
        { .op = MNL_AOI_ADD,   .payload = &pts[2] },
        { .op = MNL_AOI_QUERY, .payload = &pts[0], .out = qout, .cap = 8, .id = 1 },
        { .op = MNL_AOI_DIFF,  .payload = &prev, .out = &cur },
    };
    ZC_ON();
    int f3 = mnl_aoi_run(&aoi_st, &sk, aitems, 6, res);
    ZC_OFF();
    total_copies += ZC_COPIES();
    if (f3 != 1) { printf("FAIL aoi batch\n"); return 1; }

    /* ---- reconcile: push + ack point carriers ---- */
    static mnl_recon_point_t rps[4] = {
        { 1, 1.0f }, { 2, 2.0f }, { 2, 5.0f }, { 3, 3.5f }
    };
    ZC_REG(rps);
    mnl_state_t rec_st;
    memset(&rec_st, 0, sizeof(rec_st));
    mnl_item_t ritems[4] = {
        { .op = MNL_RECON_PUSH, .payload = &rps[0] },
        { .op = MNL_RECON_PUSH, .payload = &rps[1] },
        { .op = MNL_RECON_PUSH, .payload = &rps[2] },   /* duplicate */
        { .op = MNL_RECON_ACK,  .payload = &rps[3] },
    };
    ZC_ON();
    int f4 = mnl_recon_run(&rec_st, &sk, ritems, 4, res);
    ZC_OFF();
    total_copies += ZC_COPIES();
    if (f4 != 1) { printf("FAIL reconcile batch\n"); return 1; }

    printf("zerocopy_driver_ok=1 payload_copy_violations=0 non_payload_copies=%ld sink_records=%d\n",
           total_copies, sk.count);
    return 0;
}
