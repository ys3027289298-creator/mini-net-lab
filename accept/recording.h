/*
 * recording.h - on-disk replay recording shared by the old and new clients.
 *
 * A recording is a bounded, self-describing list of snapshot packets exactly
 * as they went over the wire.  The old client (pinned v1 decoder) and the new
 * client both read it and render every packet; their framebuffers must match
 * byte for byte.
 *
 * Layout (little-endian, fixed width):
 *   char     magic[8]   = "MNLREC1\0"
 *   uint32   count
 *   uint32   scale_bits (IEEE-754 float)
 *   count x { uint16 len, len bytes }
 */
#ifndef ACCEPT_RECORDING_H
#define ACCEPT_RECORDING_H

#include <stdint.h>

#define REC_MAGIC "MNLREC1"

typedef struct {
    uint32_t count;
    float scale;
    uint16_t len[256];
    unsigned char pkt[256][64];
} Recording;

int  recording_write(const char* path, const Recording* rec);
int  recording_read(const char* path, Recording* rec);

#endif /* ACCEPT_RECORDING_H */
