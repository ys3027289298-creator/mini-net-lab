
/* inqueue: sequenced input reorder queue for deterministic client replay. */
#include <stdio.h>
#include <string.h>

#define IQ_MAX 64

typedef struct {
    unsigned seq;   /* uint32 sequence, wraps at 2^32 */
    int keys;       /* pressed-key bitmask */
    float ax, ay;   /* analog stick */
} Input;

typedef struct { float x, y, vx, vy; } SimState;

typedef struct {
    Input slot[IQ_MAX];
    int used[IQ_MAX];
    unsigned next;     /* next seq expected for replay (wraps at 2^32) */
    int head;          /* slot holding next; advances with each pop */
    int window;        /* accepted forward distance: offsets [0, window) */
    int queued;        /* buffered inputs */
    unsigned dropped;  /* arrivals past the window */
    unsigned dupes;    /* stale or duplicate arrivals */
} InQ;

/* signed circular distance of a from b; correct across uint32 wrap */
static int seq_diff(unsigned a, unsigned b) { return (int)(a - b); }

static int iq_init(InQ* q, int window, unsigned start) {
    if (window <= 0 || window > IQ_MAX) return -1;
    memset(q, 0, sizeof(*q));
    q->window = window;
    q->next = start;
    return 0;
}

/* returns 0 buffered, -1 stale/duplicate, -2 outside window */
static int iq_push(InQ* q, const Input* in) {
    int d = seq_diff(in->seq, q->next);
    if (d < 0) { q->dupes++; return -1; }                 /* stale */
    if (d >= q->window) { q->dropped++; return -2; }      /* past window */
    int idx = (q->head + d) % q->window;                  /* offset-based: wrap-safe */
    if (q->used[idx]) { q->dupes++; return -1; }          /* duplicate */
    q->slot[idx] = *in;
    q->used[idx] = 1;
    q->queued++;
    return 0;
}

/* deliver the next in-order input; 0 while a gap is open */
static int iq_pop(InQ* q, Input* out) {
    if (!q->used[q->head]) return 0;
    *out = q->slot[q->head];
    q->used[q->head] = 0;
    q->head = (q->head + 1) % q->window;
    q->queued--;
    q->next++;                                            /* wraps naturally */
    return 1;
}

/* deterministic client-side prediction step */
static void sim_step(SimState* s, const Input* in) {
    s->vx += in->ax;
    s->vy += in->ay;
    if (in->keys & 1) s->vy += 8.0f;                     /* jump */
    s->x += s->vx;
    s->y += s->vy;
}

static int state_eq(const SimState* a, const SimState* b) {
    return a->x == b->x && a->y == b->y &&
           a->vx == b->vx && a->vy == b->vy;
}

/* deterministic payloads so the same seq always yields the same input */
static Input make_input(unsigned seq) {
    unsigned z = seq * 2654435761u + 1u;
    Input in;
    in.seq = seq;
    in.keys = (int)(z & 3u);
    in.ax = (float)((z >> 4) & 15u) * 0.25f;
    in.ay = (float)((z >> 8) & 15u) * -0.125f;
    return in;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    InQ q;
    Input out;

    /* window validation */
    check("zero_window", iq_init(&q, 0, 0) == -1);
    check("neg_window", iq_init(&q, -1, 0) == -1);
    check("oversize_window", iq_init(&q, IQ_MAX + 1, 0) == -1);
    check("init_ok", iq_init(&q, 4, 0) == 0 && q.window == 4 && q.next == 0);

    /* gap: 0 and 2 arrive, replay blocks until 1 fills the hole */
    iq_init(&q, 4, 0);
    check("buf2", iq_push(&q, &(Input){ 2, 0, 0, 0 }) == 0);
    check("buf0", iq_push(&q, &(Input){ 0, 0, 0, 0 }) == 0);
    check("pop0", iq_pop(&q, &out) == 1 && out.seq == 0);
    check("gap_blocks", iq_pop(&q, &out) == 0 && q.queued == 1);
    check("fill1", iq_push(&q, &(Input){ 1, 0, 0, 0 }) == 0);
    check("pop1", iq_pop(&q, &out) == 1 && out.seq == 1);
    check("pop2", iq_pop(&q, &out) == 1 && out.seq == 2);
    check("drained", iq_pop(&q, &out) == 0 && q.next == 3 && q.queued == 0);

    /* out-of-window arrivals are dropped and counted */
    iq_init(&q, 3, 0);
    check("w0", iq_push(&q, &(Input){ 0, 0, 0, 0 }) == 0);
    check("w1", iq_push(&q, &(Input){ 1, 0, 0, 0 }) == 0);
    check("w2", iq_push(&q, &(Input){ 2, 0, 0, 0 }) == 0);
    check("w3_dropped", iq_push(&q, &(Input){ 3, 0, 0, 0 }) == -2 && q.dropped == 1);
    check("w9_dropped", iq_push(&q, &(Input){ 9, 0, 0, 0 }) == -2 && q.dropped == 2);
    iq_pop(&q, &out); iq_pop(&q, &out); iq_pop(&q, &out);
    check("drop_kept_out", q.queued == 0 && q.next == 3);

    /* duplicate while buffered, stale after delivery */
    iq_init(&q, 4, 0);
    iq_push(&q, &(Input){ 1, 0, 0, 0 });
    check("dup_buffered", iq_push(&q, &(Input){ 1, 0, 0, 0 }) == -1 && q.dupes == 1);
    iq_push(&q, &(Input){ 0, 0, 0, 0 });
    iq_pop(&q, &out); iq_pop(&q, &out);
    check("stale", iq_push(&q, &(Input){ 0, 0, 0, 0 }) == -1 && q.dupes == 2);

    /* shuffled arrival (reversed blocks, displacement < window) must replay
       into the same field-by-field state as perfect in-order arrival */
    enum { N = 24, W = 5 };
    SimState ref = { 0 }, got = { 0 };
    iq_init(&q, W, 0);
    for (int base = 0; base < N; base += W) {
        int end = base + W; if (end > N) end = N;
        for (int i = end - 1; i >= base; i--) {
            Input in = make_input((unsigned)i);
            check("shuffle_accept", iq_push(&q, &in) == 0);
        }
        while (iq_pop(&q, &out)) sim_step(&got, &out);
    }
    for (int i = 0; i < N; i++) {
        Input in = make_input((unsigned)i);
        sim_step(&ref, &in);
    }
    check("replay_field_match", state_eq(&ref, &got));
    check("no_drop", q.dropped == 0);
    check("all_delivered", q.next == N && q.queued == 0);

    /* wraparound: scrambled seqs across the 2^32 boundary pop in order */
    unsigned ws = 0xFFFFFFFCu;
    iq_init(&q, 8, ws);
    unsigned ord[8] = { ws + 2, ws + 5, ws, ws + 7, ws + 3, ws + 1, ws + 6, ws + 4 };
    int ok = 1;
    for (int i = 0; i < 8; i++) {
        Input in = make_input(ord[i]);
        if (iq_push(&q, &in) != 0) ok = 0;
    }
    check("wrap_accept", ok);
    int cnt = 0; ok = 1;
    while (iq_pop(&q, &out)) {
        if (out.seq != ws + (unsigned)cnt) ok = 0;
        cnt++;
    }
    check("wrap_order", ok && cnt == 8);
    check("wrap_next", q.next == ws + 8u && q.next == 4u && q.queued == 0);

    /* wraparound window: offsets measured circularly, drops still counted */
    iq_init(&q, 2, 0xFFFFFFFEu);
    Input in0 = make_input(0xFFFFFFFEu), in1 = make_input(0xFFFFFFFFu);
    Input inw0 = make_input(0u), inw1 = make_input(1u), inold = make_input(0xFFFFFFFDu);
    check("wrap_w0", iq_push(&q, &in0) == 0);
    check("wrap_w1", iq_push(&q, &in1) == 0);
    check("wrap_drop0", iq_push(&q, &inw0) == -2 && q.dropped == 1);
    check("wrap_drop1", iq_push(&q, &inw1) == -2 && q.dropped == 2);
    iq_pop(&q, &out); iq_pop(&q, &out);
    check("wrap_drained", q.next == 0u);
    check("wrap_stale", iq_push(&q, &inold) == -1 && q.dupes == 1);

    /* replay across the boundary must also match the in-order reference.
       reverse order needs room for the largest offset M-1, hence window M+1 */
    enum { M = 10 };
    SimState rref = { 0 }, rgot = { 0 };
    iq_init(&q, M + 1, ws);
    for (int i = M - 1; i >= 0; i--) {
        Input in = make_input(ws + (unsigned)i);
        iq_push(&q, &in);
    }
    while (iq_pop(&q, &out)) sim_step(&rgot, &out);
    for (int i = 0; i < M; i++) {
        Input in = make_input(ws + (unsigned)i);
        sim_step(&rref, &in);
    }
    check("wrap_replay_match", state_eq(&rref, &rgot));
    check("wrap_replay_next", q.next == ws + (unsigned)M && q.next == 6u);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
