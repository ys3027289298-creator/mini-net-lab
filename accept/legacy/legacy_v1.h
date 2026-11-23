/*
 * legacy_v1.h - pinned, independent v1 reference codec used only by the
 * acceptance harness.  It never calls into the new module implementation, so
 * "v2 packets still decode correctly on v1" is not proven by the same code.
 *
 * Snapshot: the 0bdd277 encoder/decoder verbatim (accept/legacy/snapshot_v1.c).
 * Clock/aoi/reconcile: a frozen reference of the v1 state-body layout - the
 * baseline struct field order, no version, no extension, trailing bytes
 * ignored.  Baseline shipped no wire codec for these three, so this file is
 * the definition of their v1 contract.
 */
#ifndef ACCEPT_LEGACY_V1_H
#define ACCEPT_LEGACY_V1_H

#include <stddef.h>

/* POD mirrors of the pinned snapshot codec's State/Packet (same layout). */
typedef struct { unsigned tick; float x, y, vx, vy; unsigned flags; } PState;
typedef struct { unsigned char buf[64]; int len; } PPacket;

/* ---- snapshot: pinned 0bdd277 codec, prefix semantics ------------------ */
int legacy_snapshot_encode(unsigned tick, float x, float y, float pos_scale,
                           unsigned char* out, int cap);
/* Reads exactly the three varints and stops; returns bytes consumed, or -1. */
int legacy_snapshot_decode(const unsigned char* in, int len, float pos_scale,
                           unsigned* tick, float* x, float* y);

/* ---- clock v1 state body ---------------------------------------------- */
#define L_MAX_SAMPLES 16
#define L_MAX_ENT 16
#define L_BUF 64

typedef struct { double rtt, offset, at; } LSample;
typedef struct { LSample s[L_MAX_SAMPLES]; int n; double best_rtt, offset; } LClockV1;

typedef struct { int id; float x, y; int alive; } LEntity;
typedef struct { LEntity e[L_MAX_ENT]; int n; float cell, radius; } LAoiV1;

typedef struct { unsigned tick; float x; } LPred;
typedef struct {
    LPred p[L_BUF]; int head, count;
    unsigned acked; int rollbacks; float last_err;
} LReconV1;

int legacy_clock_encode_v1(const LClockV1* c, unsigned char* out, int cap);
int legacy_clock_decode_v1(const unsigned char* in, int len, LClockV1* c);
int legacy_aoi_encode_v1(const LAoiV1* a, unsigned char* out, int cap);
int legacy_aoi_decode_v1(const unsigned char* in, int len, LAoiV1* a);
int legacy_recon_encode_v1(const LReconV1* r, unsigned char* out, int cap);
int legacy_recon_decode_v1(const unsigned char* in, int len, LReconV1* r);

#endif /* ACCEPT_LEGACY_V1_H */
