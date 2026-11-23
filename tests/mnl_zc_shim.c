/*
 * mnl_zc_shim.c - LD_PRELOAD interceptor proving the batch runners never
 * copy payload bytes.
 *
 * The test driver registers every payload address range it hands to a
 * runner and enables monitoring around the run. Any memcpy/memmove/mempcpy
 * whose source or destination overlaps a registered range aborts the
 * process with a clear message - that would be a second copy of payload
 * data. Copies of result/state/scratch memory (outside registered ranges)
 * are counted but allowed.
 *
 * Driver <-> shim channel:
 *   mnlzc_register(ptr,len)  watch this payload range
 *   mnlzc_set_enabled(0/1)   arm/disarm the overlap check
 *   mnlzc_copies()           total intercepted copy calls while armed
 *   mnlzc_reset()            clear ranges and counters
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MNLZC_MAX_RANGES 128

static struct { const void* ptr; size_t len; } ranges[MNLZC_MAX_RANGES];
static int nranges = 0;
static int enabled = 0;
static long copies = 0;

void mnlzc_register(const void* ptr, size_t len) {
    if (nranges < MNLZC_MAX_RANGES) { ranges[nranges].ptr = ptr; ranges[nranges].len = len; nranges++; }
}
void mnlzc_set_enabled(int on) { enabled = on; }
long mnlzc_copies(void) { return copies; }
void mnlzc_reset(void) { nranges = 0; enabled = 0; copies = 0; }

static int overlaps_registered(const void* p, size_t len) {
    uintptr_t a = (uintptr_t)p, b = a + len;
    for (int i = 0; i < nranges; i++) {
        uintptr_t ra = (uintptr_t)ranges[i].ptr;
        uintptr_t rb = ra + ranges[i].len;
        if (a < rb && ra < b) return 1;
    }
    return 0;
}

static void check(const char* fn, const void* dst, const void* src, size_t n) {
    if (!enabled) return;
    copies++;
    if (overlaps_registered(src, n) || overlaps_registered(dst, n)) {
        fprintf(stderr,
            "ZERCOPY VIOLATION: %s(%p, %p, %zu) touches a registered payload range\n",
            fn, dst, src, n);
        abort();
    }
}

/* Each glibc version node needs its own definition; a single function can
 * only carry one version. The wrappers share check() so behavior is
 * identical whichever version a caller's PLT resolves to. */
__asm__(".symver zc_memcpy_225, memcpy@GLIBC_2.2.5");
void* zc_memcpy_225(void* dst, const void* src, size_t n) {
    static void* (*real)(void*, const void*, size_t);
    if (!real) real = dlsym(RTLD_NEXT, "memcpy");
    check("memcpy", dst, src, n);
    return real(dst, src, n);
}

#if defined(__x86_64__) || defined(__aarch64__) || defined(__powerpc64__)
__asm__(".symver zc_memcpy_214, memcpy@GLIBC_2.14");
void* zc_memcpy_214(void* dst, const void* src, size_t n) {
    static void* (*real)(void*, const void*, size_t);
    if (!real) real = dlsym(RTLD_NEXT, "memcpy");
    check("memcpy", dst, src, n);
    return real(dst, src, n);
}
#endif

__asm__(".symver zc_memmove, memmove@GLIBC_2.2.5");
void* zc_memmove(void* dst, const void* src, size_t n) {
    static void* (*real)(void*, const void*, size_t);
    if (!real) real = dlsym(RTLD_NEXT, "memmove");
    check("memmove", dst, src, n);
    return real(dst, src, n);
}

__asm__(".symver zc_mempcpy, mempcpy@GLIBC_2.2.5");
void* zc_mempcpy(void* dst, const void* src, size_t n) {
    static void* (*real)(void*, const void*, size_t);
    if (!real) real = dlsym(RTLD_NEXT, "mempcpy");
    check("mempcpy", dst, src, n);
    return real(dst, src, n);
}
