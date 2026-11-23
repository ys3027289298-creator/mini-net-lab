/*
 * snapshot_pinned.c - textually include the pinned 0bdd277 snapshot codec so
 * its static encode()/decode() are reachable, and expose them under names the
 * compatibility harness can call.  No edits to the pinned source.
 */
#define main legacy_snapshot_pinned_main
#include "snapshot_v1.c"
#undef main

#include "legacy_v1.h"

int legacy_snapshot_pinned_encode(const PState* s, PPacket* p, float scale) {
    return encode((const State*)s, (Packet*)p, scale);
}

int legacy_snapshot_pinned_decode(const PPacket* p, PState* s, float scale) {
    return decode((const Packet*)p, (State*)s, scale);
}
