
/* clock: network clock offset estimation with min-RTT filtering. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_SAMPLES 16

typedef struct { double rtt; double offset; double at; } Sample;

typedef struct {
    Sample s[MAX_SAMPLES];
    int n;
    double best_rtt;
    double offset;
} Sync;

static void sync_init(Sync* sy) { memset(sy, 0, sizeof(*sy)); sy->best_rtt = 1e9; }

static int sync_add(Sync* sy, double t_send, double t_recv, double remote_time) {
    if (t_recv < t_send) return -1;
    double rtt = t_recv - t_send;
    double offset = remote_time - (t_send + t_recv) * 0.5;
    if (sy->n >= MAX_SAMPLES) {
        memmove(&sy->s[0], &sy->s[1], sizeof(Sample) * (MAX_SAMPLES - 1));
        sy->n = MAX_SAMPLES - 1;
    }
    sy->s[sy->n].rtt = rtt;
    sy->s[sy->n].offset = offset;
    sy->s[sy->n].at = t_recv;
    sy->n++;
    if (rtt < sy->best_rtt) sy->best_rtt = rtt;

    /* keep samples within 1.5x best rtt, average their offsets */
    double sum = 0.0; int cnt = 0;
    for (int i = 0; i < sy->n; i++) {
        if (sy->s[i].rtt <= sy->best_rtt * 1.5 + 1e-9) {
            sum += sy->s[i].offset; cnt++;
        }
    }
    if (cnt == 0) return -1;
    sy->offset = sum / cnt;
    return cnt;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Sync sy;
    sync_init(&sy);
    double true_off = 2.0;
    check("add1", sync_add(&sy, 0.0, 0.10, 0.05 + true_off) > 0);
    check("offset_close", fabs(sy.offset - true_off) < 0.01);
    check("best_rtt", fabs(sy.best_rtt - 0.10) < 1e-9);

    check("add2", sync_add(&sy, 1.0, 1.20, 1.10 + true_off) > 0);
    check("still_close", fabs(sy.offset - true_off) < 0.01);

    check("reject_time_reversal", sync_add(&sy, 5.0, 4.0, 0.0) == -1);

    Sync sp;
    sync_init(&sp);
    for (int i = 0; i < 40; i++) sync_add(&sp, i * 1.0, i * 1.0 + 0.05 + (i % 5) * 0.01, i * 1.0 + 0.025 + true_off);
    check("ring_kept", sp.n == MAX_SAMPLES);
    check("stable_offset", fabs(sp.offset - true_off) < 0.01);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
