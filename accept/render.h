/*
 * render.h - deterministic software framebuffer used as pixel-identity evidence.
 *
 * No floating point, no external libraries, no platform state: the same
 * decoded state always produces the same bytes.  Two decoded states that
 * render to the same framebuffer produce the same SHA-256, which is what
 * "old client renders the recording pixel-identically" means here.
 */
#ifndef ACCEPT_RENDER_H
#define ACCEPT_RENDER_H

#define FB_W 64
#define FB_H 48

typedef struct { unsigned char px[FB_H][FB_W][3]; } Frame;

void fb_clear(Frame* f, unsigned char r, unsigned char g, unsigned char b);
void fb_rect(Frame* f, int x, int y, int w, int h,
             unsigned char r, unsigned char g, unsigned char b);
void fb_line(Frame* f, int x0, int y0, int x1, int y1,
             unsigned char r, unsigned char g, unsigned char b);

/* SHA-256 of the raw framebuffer bytes, lowercase hex + NUL. */
void frame_hash(const Frame* f, char hex[65]);
/* SHA-256 of an arbitrary byte range, lowercase hex + NUL. */
void bytes_hash(const unsigned char* p, int n, char hex[65]);
/* Number of differing bytes between two framebuffers (0 == pixel-identical). */
int  frame_diff(const Frame* a, const Frame* b);

#endif /* ACCEPT_RENDER_H */
