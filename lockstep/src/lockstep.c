
/* lockstep: deterministic lockstep with input-delay window, prediction
   marking, rollback replay on late inputs, and stall/backlog metrics. */
#include <stdio.h>
#include <string.h>

#define NP 2                  /* players in the room */
#define RING 128              /* frame history ring (inputs + saved states) */
#define FULL_MASK ((1u << NP) - 1u)
#define BTN_JUMP 1u

typedef struct { unsigned buttons; int steer; } Input;
typedef struct { int x, y, vx, vy; } PState;

typedef struct {
    unsigned frame;
    unsigned csum;
    PState ps[NP];
} World;

typedef struct {
    Input in[NP];
    unsigned tag;             /* frame this slot currently holds (0 = empty) */
    unsigned char real;       /* bitmask of players whose input is real */
    unsigned char pred;       /* bitmask of players filled by prediction */
} FrameInput;

enum { LS_REAL = 0, LS_PREDICT = 1, LS_STALL = 2 };

typedef struct {
    int status;               /* LS_REAL / LS_PREDICT / LS_STALL */
    unsigned frame;           /* frame the tick advanced to (unchanged on stall) */
    unsigned pred_mask;       /* players whose input was predicted */
    unsigned backlog;         /* frames running ahead of the confirmed frame */
} TickInfo;

typedef struct {
    int W;                    /* input delay window */
    int allow_predict;        /* 0 = strict lockstep (stall), 1 = predict+mark */
    unsigned frame;           /* last simulated frame */
    unsigned confirmed;       /* highest contiguous fully-real frame <= frame */
    World state;
    World saved[RING];        /* world state before frame f, for replay */
    unsigned saved_tag[RING];
    FrameInput fin[RING];     /* inputs indexed by frame */
    Input last[NP];           /* last known real input, used for prediction */
    /* reported metrics */
    unsigned stall_frames;    /* ticks that refused to advance in strict mode */
    unsigned pred_frames;     /* frames advanced on at least one predicted input */
    unsigned rollbacks;       /* late-input replays performed */
    unsigned max_backlog;     /* deepest re-sim depth a late input forced */
    unsigned overruns;        /* predicted ticks where backlog exceeded window W */
} Lockstep;

static void ls_init(Lockstep* ls, int window, int allow_predict) {
    memset(ls, 0, sizeof(*ls));
    ls->W = window;
    ls->allow_predict = allow_predict;
}

static unsigned fnv_bytes(unsigned h, const void* p, int n) {
    const unsigned char* b = (const unsigned char*)p;
    for (int i = 0; i < n; i++) { h ^= b[i]; h *= 16777619u; }
    return h;
}

/* deterministic integer-only simulation: every field stays exactly replayable */
static void step(World* w, const FrameInput* fi) {
    for (int p = 0; p < NP; p++) {
        PState* s = &w->ps[p];
        const Input* in = &fi->in[p];
        s->vx += in->steer;
        if (s->vx < -8) s->vx = -8;
        if (s->vx > 8) s->vx = 8;
        if (in->buttons & BTN_JUMP) s->vy += 2;
        s->vy -= 1;                            /* gravity */
        if (s->vy < -10) s->vy = -10;
        s->x += s->vx;
        s->y += s->vy;
        if (s->y < 0) { s->y = 0; s->vy = 0; } /* floor */
    }
    w->frame++;
    w->csum = fnv_bytes(2166136261u, w->ps, sizeof(w->ps));
    w->csum = fnv_bytes(w->csum, &w->frame, sizeof(w->frame));
}

static FrameInput* slot(Lockstep* ls, unsigned f) {
    FrameInput* fi = &ls->fin[f % RING];
    if (fi->tag != f) { memset(fi, 0, sizeof(*fi)); fi->tag = f; }
    return fi;
}

/* confirmed frame never runs ahead of the simulated frame */
static void confirm_catchup(Lockstep* ls) {
    while (ls->confirmed < ls->frame) {
        unsigned f = ls->confirmed + 1;
        FrameInput* fi = &ls->fin[f % RING];
        if (fi->tag == f && fi->real == FULL_MASK) ls->confirmed = f;
        else break;
    }
}

static void replay(Lockstep* ls, unsigned from, unsigned until) {
    ls->state = ls->saved[from % RING];
    ls->frame = ls->state.frame;
    for (unsigned g = from; g <= until; g++) {
        /* refresh pre-frame snapshots: earlier rollbacks change the trajectory */
        ls->saved[g % RING] = ls->state;
        ls->saved_tag[g % RING] = g;
        step(&ls->state, &ls->fin[g % RING]);
    }
    ls->frame = until;
}

/* store a real input targeting frame f.
   returns 0 ok, -1 duplicate/stale, -2 older than the replay history ring. */
static int ls_store(Lockstep* ls, int p, unsigned f, Input in) {
    if (p < 0 || p >= NP) return -1;
    if (f <= ls->confirmed) return -1;                      /* finalized frame */
    if (f + RING <= ls->frame + 1) return -2;               /* history gone */
    FrameInput* fi = slot(ls, f);
    unsigned bit = 1u << p;
    if (fi->real & bit) return -1;                          /* duplicate */
    fi->in[p] = in;
    fi->real |= (unsigned char)bit;
    fi->pred &= (unsigned char)~bit;                        /* real replaces mark */
    ls->last[p] = in;
    if (f <= ls->frame) {                                   /* late: rollback */
        if (ls->saved_tag[f % RING] != f) return -2;
        unsigned depth = ls->frame - f + 1;
        if (depth > ls->max_backlog) ls->max_backlog = depth;
        unsigned until = ls->frame;
        replay(ls, f, until);
        ls->rollbacks++;
    }
    confirm_catchup(ls);
    return 0;
}

/* local input sampled at sample_frame is scheduled sample_frame + W */
static int ls_local(Lockstep* ls, int p, unsigned sample_frame, Input in) {
    return ls_store(ls, p, sample_frame + (unsigned)ls->W, in);
}

static int ls_tick(Lockstep* ls, TickInfo* info) {
    unsigned f = ls->frame + 1;
    FrameInput* fi = slot(ls, f);

    if (fi->real != FULL_MASK && !ls->allow_predict) {
        ls->stall_frames++;                                 /* hard lockstep: wait */
        info->status = LS_STALL;
        info->frame = ls->frame;
        info->pred_mask = 0;
        info->backlog = ls->frame - ls->confirmed;
        return LS_STALL;
    }

    int status = LS_REAL;
    if (fi->real != FULL_MASK) {
        unsigned missing = FULL_MASK ^ fi->real;
        for (int p = 0; p < NP; p++)
            if (missing & (1u << p)) fi->in[p] = ls->last[p]; /* neutral if none */
        fi->pred = (unsigned char)(fi->pred | missing);
        ls->pred_frames++;
        status = LS_PREDICT;
    }

    ls->saved[f % RING] = ls->state;
    ls->saved_tag[f % RING] = f;
    step(&ls->state, fi);
    ls->frame = f;
    confirm_catchup(ls);

    unsigned backlog = ls->frame - ls->confirmed;
    if (status == LS_PREDICT && backlog > (unsigned)ls->W)
        ls->overruns++;                                     /* window exhausted:
                                                               report, don't block */
    info->status = status;
    info->frame = f;
    info->pred_mask = (status == LS_PREDICT) ? (FULL_MASK ^ fi->real) : 0;
    info->backlog = backlog;
    return status;
}

/* ---- test harness ---- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void expect_world_equal(const char* tag, const World* a, const World* b) {
    char name[64];
    snprintf(name, sizeof name, "%s_frame", tag);
    if (a->frame != b->frame) printf("  [%s] frame %u != %u\n", tag, a->frame, b->frame);
    check(name, a->frame == b->frame);
    for (int p = 0; p < NP; p++) {
        snprintf(name, sizeof name, "%s_x%d", tag, p);
        check(name, a->ps[p].x == b->ps[p].x);
        snprintf(name, sizeof name, "%s_y%d", tag, p);
        check(name, a->ps[p].y == b->ps[p].y);
        snprintf(name, sizeof name, "%s_vx%d", tag, p);
        check(name, a->ps[p].vx == b->ps[p].vx);
        snprintf(name, sizeof name, "%s_vy%d", tag, p);
        check(name, a->ps[p].vy == b->ps[p].vy);
    }
    snprintf(name, sizeof name, "%s_csum", tag);
    check(name, a->csum == b->csum);
}

/* ---- deterministic script + simulated lossy network ---- */

#define N 100
#define W 4

typedef struct { Input s[NP]; } ScriptRow;
static ScriptRow script[N + W];
static unsigned netdelay[N + W];
static unsigned rng_state;

static unsigned rng(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static void gen_script(void) {
    rng_state = 0x1234abcdu;
    for (int t = 0; t < N + W; t++)
        for (int p = 0; p < NP; p++) {
            script[t].s[p].steer = (int)(rng() % 3u) - 1;
            script[t].s[p].buttons = (rng() & 7u) == 0 ? BTN_JUMP : 0;
        }
}

static void gen_delays(void) {
    for (int t = 0; t < N + W; t++) {
        unsigned r = rng() % 10u;
        if (r < 7) netdelay[t] = rng() % 2u;                 /* 0..1, inside W */
        else if (r < 9) netdelay[t] = 5 + rng() % 4u;        /* 5..8, past W */
        else netdelay[t] = 10 + rng() % 5u;                  /* 10..14, deep */
    }
    netdelay[0] = 12;   /* guaranteed window overrun for metric assertions */
}

typedef struct { unsigned due; int p; unsigned frame; Input in; int used; } Pkt;
#define QMAX 512
static Pkt q[QMAX];
static int qn;

static void q_add(unsigned due, int p, unsigned frame, Input in) {
    q[qn++] = (Pkt){ due, p, frame, in, 0 };
}

static void q_deliver(Lockstep* ls, unsigned now) {
    for (int i = 0; i < qn; i++)
        if (!q[i].used && q[i].due <= now) {
            ls_store(ls, q[i].p, q[i].frame, q[i].in);
            q[i].used = 1;
        }
}

static void q_drain(Lockstep* ls, unsigned max_frame) {
    for (int i = 0; i < qn; i++)
        if (!q[i].used && q[i].frame <= max_frame) {
            ls_store(ls, q[i].p, q[i].frame, q[i].in);
            q[i].used = 1;
        }
}

/* neutral inputs for sample frames -W..-1, targeting frames 0..W-1 */
static void prime_inputs(Lockstep* ls, int lossy, unsigned now) {
    Input neutral = { 0, 0 };
    for (int k = 0; k < W; k++) {
        ls_store(ls, 0, (unsigned)k, neutral);
        if (lossy) q_add(now + netdelay[k], 1, (unsigned)k, neutral);
        else ls_store(ls, 1, (unsigned)k, neutral);
    }
}

int main(void) {
    /* ---- unit mechanics ---- */
    Lockstep ls;
    ls_init(&ls, W, 1);
    Input i0 = { 0, 1 };

    check("schedule_window", ls_local(&ls, 0, 0, i0) == 0 &&
                             ls.fin[W].tag == (unsigned)W &&
                             ls.fin[W].real == 1u &&
                             ls.fin[0].tag == 0);
    check("duplicate_rejected", ls_local(&ls, 0, 0, i0) == -1);

    Lockstep strict;
    ls_init(&strict, W, 0);
    TickInfo ti;
    check("strict_stalls", ls_tick(&strict, &ti) == LS_STALL &&
                           strict.frame == 0 && strict.stall_frames == 1);

    Lockstep pred;
    ls_init(&pred, W, 1);
    check("predict_marks", ls_tick(&pred, &ti) == LS_PREDICT &&
                           pred.frame == 1 && ti.pred_mask == FULL_MASK &&
                           pred.fin[1].real == 0 && pred.fin[1].pred == FULL_MASK);

    /* late input triggers replay and converges with the all-real world */
    Lockstep ref;
    ls_init(&ref, 2, 1);
    Lockstep late;
    ls_init(&late, 2, 1);
    Input a0 = { 0, 1 }, a1 = { BTN_JUMP, -1 };
    Input b0 = { 0, -1 }, b1 = { 0, 1 };
    ls_store(&ref, 0, 1, a0); ls_store(&ref, 1, 1, a1);
    ls_store(&late, 0, 1, a0); ls_store(&late, 1, 1, a1);
    ls_tick(&ref, &ti); ls_tick(&late, &ti);
    ls_store(&ref, 0, 2, b0); ls_store(&ref, 1, 2, b1);
    ls_store(&late, 0, 2, b0);                   /* player 1 missing */
    check("late_predicted", ls_tick(&late, &ti) == LS_PREDICT &&
                            late.pred_frames == 1);
    ls_tick(&ref, &ti);
    check("late_accepted", ls_store(&late, 1, 2, b1) == 0 &&
                           late.rollbacks == 1 && late.frame == 2);
    check("late_dup_rejected", ls_store(&late, 1, 2, b1) == -1);
    expect_world_equal("replay", &ref.state, &late.state);

    check("stale_rejected", ls_store(&late, 0, 0, a0) == -1);

    /* inputs older than the history ring cannot be replayed */
    Lockstep old;
    ls_init(&old, 1, 1);
    for (int t = 0; t < RING + 2; t++) ls_tick(&old, &ti);
    check("history_guard", ls_store(&old, 0, 1, a0) == -2);

    /* ---- ideal world: no loss, every input arrives before its frame ---- */
    gen_script();
    gen_delays();

    Lockstep ideal;
    ls_init(&ideal, W, 1);
    prime_inputs(&ideal, 0, 0);
    int nonreal = 0, bad_submit = 0;
    for (int t = 0; t < N; t++) {
        for (int p = 0; p < NP; p++)
            if (ls_local(&ideal, p, (unsigned)t, script[t].s[p]) != 0) bad_submit++;
        if (ls_tick(&ideal, &ti) != LS_REAL) nonreal++;
    }
    check("ideal_submit_all", bad_submit == 0);
    check("ideal_all_real", nonreal == 0 && ideal.frame == N && ideal.confirmed == N);

    /* ---- lossy world, prediction mode: never hard-waits ---- */
    qn = 0;
    Lockstep lossy;
    ls_init(&lossy, W, 1);
    prime_inputs(&lossy, 1, 0);
    int stalls = 0;
    for (int t = 0; t < N; t++) {
        q_deliver(&lossy, (unsigned)t);
        ls_local(&lossy, 0, (unsigned)t, script[t].s[0]);
        q_add((unsigned)t + netdelay[t], 1, (unsigned)(t + W), script[t].s[1]);
        if (ls_tick(&lossy, &ti) == LS_STALL) stalls++;
    }
    check("lossy_never_stalls", stalls == 0);
    q_drain(&lossy, N);                         /* late retransmits -> rollbacks */
    expect_world_equal("lossy_parity", &ideal.state, &lossy.state);
    check("lossy_predicted_some", lossy.pred_frames > 0);
    check("lossy_rollbacks_some", lossy.rollbacks > 0);
    check("lossy_backlog_some", lossy.max_backlog > 0);
    check("lossy_overruns_reported", lossy.overruns > 0);
    check("lossy_backlog_cleared", lossy.frame - lossy.confirmed == 0);
    printf("metrics_predict stalls=%u predicted=%u rollbacks=%u max_backlog=%u "
           "overruns=%u backlog_now=%u\n",
           lossy.stall_frames, lossy.pred_frames, lossy.rollbacks,
           lossy.max_backlog, lossy.overruns, lossy.frame - lossy.confirmed);

    /* ---- same lossy network, strict lockstep: stalls instead of predicting ---- */
    qn = 0;
    Lockstep strictrun;
    ls_init(&strictrun, W, 0);
    prime_inputs(&strictrun, 1, 0);
    int t = 0;
    while (strictrun.frame < N && t < N + 300) {
        q_deliver(&strictrun, (unsigned)t);
        if (t < N) {
            ls_local(&strictrun, 0, (unsigned)t, script[t].s[0]);
            q_add((unsigned)t + netdelay[t], 1, (unsigned)(t + W), script[t].s[1]);
        }
        ls_tick(&strictrun, &ti);
        t++;
    }
    q_drain(&strictrun, N);
    check("strict_caught_up", strictrun.frame == N);
    expect_world_equal("strict_parity", &ideal.state, &strictrun.state);
    check("strict_stalled_some", strictrun.stall_frames > 0);
    check("strict_never_predicted",
          strictrun.pred_frames == 0 && strictrun.rollbacks == 0);
    check("strict_backlog_cleared", strictrun.frame - strictrun.confirmed == 0);
    printf("metrics_strict stalls=%u predicted=%u rollbacks=%u max_backlog=%u "
           "overruns=%u backlog_now=%u\n",
           strictrun.stall_frames, strictrun.pred_frames, strictrun.rollbacks,
           strictrun.max_backlog, strictrun.overruns,
           strictrun.frame - strictrun.confirmed);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
