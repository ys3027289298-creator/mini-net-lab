
/* anim: remote action state sync by action start time and phase alignment.
 *
 * Both sides replay an action from the local time it started, so the remote
 * phase at any instant follows from its start stamp ("aligned by action
 * start time"). Phase is normalized to one cycle: elapsed/duration.
 *
 * On an action change the switch point is chosen by comparing the remote
 * phase rp with the local playback phase lp:
 *
 *     o = (rp - lp) mod 1     (phase correction the remote asks for)
 *
 * o <= 1/2: enter the new action at the current local phase. The switch
 *   itself is continuous (jump 0) and the residual phase error vs the
 *   remote is o, so it stays within half a cycle.
 * o > 1/2: aligning would leave a phase error beyond half a cycle, so the
 *   switch is a hard cut that snaps to rp. The phase error after the cut
 *   is 0; the pose discontinuity (the jump) is 1-o, also within half a
 *   cycle.
 *
 * Invariant: every switch keeps phase error <= 1/2 cycle and jump <= 1/2
 * cycle. Each switch records its error and jump, and the report bounds are
 * the maxima over all switches.
 *
 * A zero phase (a sample taken exactly at the action start, or a start
 * stamp in the future that has not begun) is counted and flagged. An
 * unusable remote action (id 0, no duration or NULL) is reported as missing
 * and leaves the local player untouched. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define ANIM_EPS 1e-9
#define ANIM_HALF 0.5

typedef struct {
    unsigned id;
    double start;      /* local time the action began */
    double duration;   /* seconds per cycle */
} AnimAction;

typedef struct {
    unsigned id;
    double start;
    double duration;
    int valid;
} AnimPlayer;

typedef struct {
    unsigned samples;        /* calls with a usable remote action */
    unsigned switches;       /* action changes applied */
    unsigned aligned;        /* switches entered at the local phase */
    unsigned hard_cuts;      /* switches that snapped to the remote phase */
    unsigned zero_phase;     /* samples whose remote phase was zero */
    unsigned missing;        /* samples with no usable remote action */
    double max_phase_err;    /* upper bound over switches, cycles, <= 0.5 */
    double max_jump;         /* upper bound over switches, cycles, <= 0.5 */
    double last_phase_err;   /* phase error of the last switch, cycles */
    double last_jump;        /* pose discontinuity of the last switch, cycles */
    double last_switch_phase;/* local phase after the last switch */
    double last_switch_at;   /* local time of the last switch */
    int last_zero_phase;     /* this sample saw phase zero */
    int status;              /* ANIM_* of the last call */
} AnimReport;

enum {
    ANIM_OK = 0,
    ANIM_ALIGNED = 1,
    ANIM_HARD_CUT = 2,
    ANIM_MISSING = 3
};

static void anim_player_init(AnimPlayer* p) { memset(p, 0, sizeof(*p)); }

static void anim_report_init(AnimReport* r) {
    memset(r, 0, sizeof(*r));
    r->last_phase_err = -1.0;
    r->last_jump = -1.0;
    r->last_switch_phase = -1.0;
    r->last_switch_at = -1.0;
    r->status = ANIM_OK;
}

/* Normalized phase in [0,1); a start stamp in the future reads as phase 0. */
static double phase_of(double now, double start, double duration) {
    if (duration <= 0.0) return 0.0;
    double elapsed = now - start;
    if (elapsed < 0.0) elapsed = 0.0;
    double p = fmod(elapsed, duration);
    if (p < 0.0) p += duration;
    return p / duration;
}

/* Shortest circular distance between two phases, in [0, 0.5] cycles. */
static double phase_dist(double a, double b) {
    double d = fabs(a - b);
    return d > ANIM_HALF ? 1.0 - d : d;
}

/* Forward phase correction from local to remote, in [0, 1) cycles. */
static double phase_forward(double remote, double local) {
    double o = fmod(remote - local, 1.0);
    if (o < 0.0) o += 1.0;
    return o;
}

static int action_usable(const AnimAction* a) {
    return a != NULL && a->id != 0u && a->duration > 0.0;
}

/* Feed one authoritative action sample at local time `now`.
 * Returns the ANIM_* status; counters and error bounds land in `rep`. */
static int anim_sync(AnimPlayer* p, AnimReport* rep, const AnimAction* remote, double now) {
    if (p == NULL || rep == NULL) return ANIM_OK;

    if (!action_usable(remote)) {
        rep->missing++;
        rep->last_zero_phase = 0;
        rep->last_phase_err = -1.0;
        rep->last_jump = -1.0;
        rep->status = ANIM_MISSING;
        return ANIM_MISSING;
    }
    rep->samples++;

    double rp = phase_of(now, remote->start, remote->duration);
    rep->last_zero_phase = rp <= ANIM_EPS;
    if (rep->last_zero_phase) rep->zero_phase++;

    if (!p->valid) {
        /* first action: adopt it at its start-time phase, not a switch */
        p->id = remote->id;
        p->start = remote->start;
        p->duration = remote->duration;
        p->valid = 1;
        rep->last_phase_err = 0.0;
        rep->last_jump = 0.0;
        rep->status = ANIM_OK;
        return ANIM_OK;
    }

    if (remote->id == p->id) {
        /* same action: re-anchor to the authoritative start time. The
         * phase correction this absorbs is reported as a jump, but it is
         * not an action switch. */
        double old_lp = phase_of(now, p->start, p->duration);
        double jump = phase_dist(old_lp, rp);
        p->start = remote->start;
        p->duration = remote->duration;
        rep->last_phase_err = 0.0;
        rep->last_jump = jump;
        if (jump > rep->max_jump) rep->max_jump = jump;
        rep->status = ANIM_OK;
        return ANIM_OK;
    }

    double lp = phase_of(now, p->start, p->duration);
    double o = phase_forward(rp, lp);
    double applied, err, jump;
    int status;

    if (o <= ANIM_HALF) {
        /* phase alignment: enter at the local phase, no pose discontinuity */
        applied = lp;
        err = o;
        jump = 0.0;
        rep->aligned++;
        status = ANIM_ALIGNED;
    } else {
        /* the aligned phase error would exceed half a cycle: hard cut */
        applied = rp;
        err = 0.0;
        jump = 1.0 - o;
        rep->hard_cuts++;
        status = ANIM_HARD_CUT;
    }

    rep->switches++;
    rep->last_phase_err = err;
    rep->last_jump = jump;
    if (err > rep->max_phase_err) rep->max_phase_err = err;
    if (jump > rep->max_jump) rep->max_jump = jump;
    rep->last_switch_phase = applied;
    rep->last_switch_at = now;

    p->id = remote->id;
    p->duration = remote->duration;
    p->start = now - applied * remote->duration;
    rep->status = status;
    return status;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}
static int near(double a, double b) { return fabs(a - b) <= 1e-9; }

int main(void) {
    AnimPlayer p;
    AnimReport r;
    anim_player_init(&p);
    anim_report_init(&r);

    /* first action is adopted, not counted as a switch */
    AnimAction walk = { 1u, 0.0, 2.0 };
    check("adopt_first", anim_sync(&p, &r, &walk, 0.5) == ANIM_OK);
    check("adopted_id", p.id == 1u && p.valid);
    check("adopted_phase", near(phase_of(0.5, p.start, p.duration), 0.25));
    check("no_switch_yet", r.switches == 0 && r.aligned == 0 && r.hard_cuts == 0);

    /* same action: no switch, start re-anchored (start-time alignment) */
    AnimAction walk2 = { 1u, 0.2, 2.0 };
    check("same_action", anim_sync(&p, &r, &walk2, 1.0) == ANIM_OK);
    check("reanchored", near(p.start, 0.2));
    check("phase_after_reanchor", near(phase_of(1.0, p.start, p.duration), 0.4));
    check("reanchor_jump_is_not_a_switch", r.switches == 0 && near(r.last_jump, 0.1));

    /* hard cut: local phase 0.75, remote phase 0.6, o = 0.85 */
    AnimPlayer pa; AnimReport ra;
    anim_player_init(&pa); anim_report_init(&ra);
    AnimAction a1 = { 10u, 0.0, 2.0 };
    anim_sync(&pa, &ra, &a1, 0.0);
    check("pre_phase", near(phase_of(1.5, pa.start, pa.duration), 0.75));
    AnimAction a2 = { 11u, 0.9, 1.0 };
    check("rp_check", near(phase_of(1.5, a2.start, a2.duration), 0.6));
    check("hard_status", anim_sync(&pa, &ra, &a2, 1.5) == ANIM_HARD_CUT);
    check("hard_counted", ra.hard_cuts == 1 && ra.switches == 1 && ra.aligned == 0);
    check("hard_applied_phase", near(ra.last_switch_phase, 0.6));
    check("hard_phase_err", near(ra.last_phase_err, 0.0));
    check("hard_jump", near(ra.last_jump, 0.15));
    check("hard_snapped", near(phase_of(1.5, pa.start, pa.duration), 0.6));

    /* phase-aligned switch: local 0.9, remote 0.3, o = 0.4 */
    AnimPlayer pb; AnimReport rb;
    anim_player_init(&pb); anim_report_init(&rb);
    AnimAction b1 = { 20u, 0.0, 2.0 };
    anim_sync(&pb, &rb, &b1, 0.0);
    AnimAction b2 = { 21u, 1.5, 1.0 };
    check("align_status", anim_sync(&pb, &rb, &b2, 1.8) == ANIM_ALIGNED);
    check("align_counted", rb.aligned == 1 && rb.hard_cuts == 0 && rb.switches == 1);
    check("align_applied_phase", near(rb.last_switch_phase, 0.9));
    check("align_phase_err", near(rb.last_phase_err, 0.4));
    check("align_jump", near(rb.last_jump, 0.0));
    check("align_no_pop", near(phase_of(1.8, pb.start, pb.duration), 0.9));

    /* exactly half a cycle still aligns (hard cut only when exceeding) */
    AnimPlayer pc; AnimReport rc;
    anim_player_init(&pc); anim_report_init(&rc);
    AnimAction c1 = { 30u, 0.0, 4.0 };
    anim_sync(&pc, &rc, &c1, 0.0);
    AnimAction c2 = { 31u, -0.5, 1.0 };   /* rp = 0.5 at now = 0 */
    check("half_aligns", anim_sync(&pc, &rc, &c2, 0.0) == ANIM_ALIGNED);
    check("half_bound", near(rc.last_phase_err, 0.5) && near(rc.last_jump, 0.0));

    /* just over half a cycle hard cuts */
    AnimPlayer pd; AnimReport rd;
    anim_player_init(&pd); anim_report_init(&rd);
    AnimAction d1 = { 40u, 0.0, 4.0 };
    anim_sync(&pd, &rd, &d1, 0.0);
    AnimAction d2 = { 41u, -0.5000001, 1.0 };
    check("over_half_hard", anim_sync(&pd, &rd, &d2, 0.0) == ANIM_HARD_CUT);
    check("over_half_bounds", rd.last_phase_err == 0.0 && rd.last_jump < 0.5 + 1e-6);

    /* zero phase is reported */
    AnimPlayer pe; AnimReport re;
    anim_player_init(&pe); anim_report_init(&re);
    AnimAction e1 = { 50u, 0.0, 2.0 };
    anim_sync(&pe, &re, &e1, 0.5);
    AnimAction e2 = { 51u, 3.0, 1.0 };   /* rp = 0 at now = 3 */
    anim_sync(&pe, &re, &e2, 3.0);
    check("zero_phase_counted", re.zero_phase == 1);
    check("zero_phase_flag", re.last_zero_phase == 1);

    /* a start stamp in the future also reads as phase zero */
    AnimPlayer pf; AnimReport rf;
    anim_player_init(&pf); anim_report_init(&rf);
    AnimAction f1 = { 60u, 5.0, 1.0 };
    anim_sync(&pf, &rf, &f1, 4.0);
    check("future_start_zero", rf.zero_phase == 1 && near(phase_of(4.0, 5.0, 1.0), 0.0));

    /* missing action is reported and leaves the player untouched */
    AnimPlayer pg; AnimReport rg;
    anim_player_init(&pg); anim_report_init(&rg);
    AnimAction g1 = { 70u, 0.0, 2.0 };
    anim_sync(&pg, &rg, &g1, 0.5);
    AnimAction zero_id = { 0u, 1.0, 1.0 };
    check("missing_id", anim_sync(&pg, &rg, &zero_id, 1.0) == ANIM_MISSING);
    AnimAction no_dur = { 71u, 1.0, 0.0 };
    check("missing_dur", anim_sync(&pg, &rg, &no_dur, 1.0) == ANIM_MISSING);
    check("missing_null", anim_sync(&pg, &rg, NULL, 1.0) == ANIM_MISSING);
    check("missing_counted", rg.missing == 3);
    check("missing_keeps_state", pg.id == 70u && rg.switches == 0);
    check("missing_err_unset", rg.last_phase_err == -1.0 && rg.last_jump == -1.0);

    /* phase helpers */
    check("dist_wrap", near(phase_dist(0.1, 0.9), 0.2));
    check("dist_zero", near(phase_dist(0.3, 0.3), 0.0));
    check("forward_wrap", near(phase_forward(0.1, 0.9), 0.2));
    check("forward_half", near(phase_forward(0.5, 0.0), 0.5));
    check("phase_wrap", near(phase_of(85.0, 0.0, 40.0), 0.125));
    check("phase_future", near(phase_of(4.0, 5.0, 1.0), 0.0));

    /* the real complaint: the remote action change arrives a little late.
     * Because the switch point comes from the start stamp, a late arrival
     * does not shift the phase: the receiver lands on the same phase as if
     * the change had arrived on time, and the switch stays aligned. */
    AnimPlayer pl; AnimReport rl;
    anim_player_init(&pl); anim_report_init(&rl);
    AnimAction l0 = { 90u, 0.0, 0.5 };
    anim_sync(&pl, &rl, &l0, 0.0);
    for (int i = 0; i < 10; i++) {
        double change_at = 1.0 + i * 0.5;       /* one change per cycle */
        double received_at = change_at + 0.05;  /* 50 ms late */
        AnimAction lagged = { (unsigned)(91 + i), change_at, 0.5 };
        int st = anim_sync(&pl, &rl, &lagged, received_at);
        check("lag_aligned", st == ANIM_ALIGNED);
        check("lag_absorbed", near(rl.last_phase_err, 0.0));
    }
    check("lag_no_hard_cuts", rl.hard_cuts == 0 && rl.aligned == 10);

    /* invariant sweep: every switch keeps both quantities <= half a cycle */
    AnimPlayer ph; AnimReport rh;
    anim_player_init(&ph); anim_report_init(&rh);
    AnimAction h1 = { 80u, 0.0, 1.0 };
    anim_sync(&ph, &rh, &h1, 0.0);
    unsigned last_id = 80u;
    for (int i = 0; i < 200; i++) {
        /* start always precedes the sample; the gap drifts so the phase
         * correction sweeps the whole cycle */
        AnimAction next = {
            (unsigned)(81 + i),
            -3.0 + (double)i * 0.05,
            0.5 + 0.1 * (i % 4)
        };
        double now = 0.5 + (double)i * 0.23;
        int st = anim_sync(&ph, &rh, &next, now);
        check("sweep_switch", st == ANIM_ALIGNED || st == ANIM_HARD_CUT);
        check("sweep_err_bound", rh.last_phase_err >= 0.0 && rh.last_phase_err <= 0.5 + 1e-9);
        check("sweep_jump_bound", rh.last_jump >= 0.0 && rh.last_jump <= 0.5 + 1e-9);
        check("sweep_phase", near(phase_of(now, ph.start, ph.duration), rh.last_switch_phase));
        last_id = next.id;
    }
    check("sweep_counts", rh.switches == 200 && rh.aligned + rh.hard_cuts == 200);
    check("sweep_max_err", rh.max_phase_err <= 0.5 + 1e-9);
    check("sweep_max_jump", rh.max_jump <= 0.5 + 1e-9);
    check("sweep_had_hard_cuts", rh.hard_cuts > 0 && rh.aligned > 0);

    /* identical remote sample: nothing changes */
    AnimAction same = { last_id, -3.0 + 199.0 * 0.05, 0.5 + 0.1 * (199 % 4) };
    unsigned before = rh.switches;
    anim_sync(&ph, &rh, &same, 5.0);
    check("same_no_switch", rh.switches == before);

    printf("switches=%u aligned=%u hard_cuts=%u\n", rh.switches, rh.aligned, rh.hard_cuts);
    printf("max_phase_err=%.6f max_jump=%.6f (bound=0.5)\n", rh.max_phase_err, rh.max_jump);
    printf("zero_phase=%u missing=%u\n", rh.zero_phase, rh.missing);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
