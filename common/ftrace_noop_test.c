/*
 * ftrace_noop_test.c — compiled WITHOUT -DFT_ENABLE: the no-op surface
 * must build warning-free and leave timing code out of the binary.
 */
#include "ftrace.h"

int main(void) {
    ft_t t;
    ft_init(&t);
    ft_frame_begin(&t);
    int h = ft_span_start(&t, "x");
    (void)ft_now(&t);
    ft_span_end(&t, h);
    ft_frame_end(&t);
    ft_set_clock(&t, 0, 0);
    return 0;
}
