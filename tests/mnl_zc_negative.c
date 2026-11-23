/*
 * mnl_zc_negative.c - negative control for the LD_PRELOAD shim.
 *
 * Performs a deliberate memcpy of a registered payload range while the
 * monitor is armed. The shim must abort the process (SIGABRT, exit 134).
 * The check script treats any other outcome as a broken interceptor.
 *
 * Built at -O0 -fno-builtin-memcpy so the copy stays a real libc call;
 * at -O2 gcc lowers fixed-size memcpy to an inline rep movsb that no
 * dynamic interposer can see (the static scan covers that layer).
 *
 * The mnlzc_* references are weak: standalone they are NULL, under
 * LD_PRELOAD the shim supplies them.
 */
#include "mnl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__attribute__((weak)) void mnlzc_register(const void*, size_t);
__attribute__((weak)) void mnlzc_set_enabled(int);

int main(void) {
    mnl_clock_sample_t sm = { 0.0, 0.1, 2.05 };
    mnl_clock_sample_t* scratch = malloc(sizeof *scratch);
    volatile double force;
    if (!mnlzc_register || !mnlzc_set_enabled) {
        printf("ERROR: shim control channel missing\n");
        free(scratch);
        return 2;
    }
    mnlzc_register(&sm, sizeof sm);
    mnlzc_set_enabled(1);
    memcpy(scratch, &sm, sizeof sm);          /* forbidden second copy */
    force = scratch->t_recv;                   /* keep the copy live */
    (void)force;
    printf("ERROR: shim did not catch the payload copy\n");
    free(scratch);
    return 2;
}
