
/* replay: wrap-safe sequence ordering inside a reorder window.
 *
 * Sequence numbers are 32-bit and wrap at 2^32.  "Newer" is decided with a
 * half-open interval comparison: a is newer than b iff (a - b) mod 2^32 lies
 * in (0, 2^31).  Distances of exactly 2^31 are ambiguous and are unordered.
 *
 * The reorder window is the set of numbers at most WIN behind the newest
 * number seen.  Because 2*WIN < 2^31, every pair inside the window has a
 * true distance below 2^31, so the half-open comparison is total there and
 * agrees with the true (unwrapped) order.  Outside the window comparisons
 * are left unordered -- the relation on all numbers is a partial order.
 */
#include <stdio.h>
#include <string.h>

typedef unsigned seq_t;

#define SEQ_HALF 0x80000000u    /* 2^(32-1): the half-open split */
#define WIN 64u                 /* reorder window; 2*WIN < SEQ_HALF */
#define RING 128

/* Signed distance a-b modulo 2^32, read as a value in [-2^31, 2^31). */
static int seq_diff(seq_t a, seq_t b) {
    return (int)(a - b);
}

/* Half-open comparison: newer iff d is in (0, 2^31).  Returns false for
 * equal numbers and for the antipodal distance (ambiguous). */
static int seq_newer(seq_t a, seq_t b) {
    seq_t d = a - b;
    return d != 0u && d < SEQ_HALF;
}

/* 1 if s is inside the window ending at head (head included).  The unsigned
 * subtraction wraps, so this works across 0. */
static int window_contains(seq_t head, seq_t s) {
    return (seq_t)(head - s) < WIN;
}

/* Window partial order: a is newer than b only when both are in the window.
 * Inside the window this is total; outside it, nothing is comparable. */
static int window_newer(seq_t head, seq_t a, seq_t b) {
    return window_contains(head, a) && window_contains(head, b) && seq_newer(a, b);
}

/* --- replay buffer ---------------------------------------------------- */

typedef struct { seq_t seq; float x; } Rec;

typedef struct {
    Rec r[RING];
    int n;
    seq_t head;          /* newest sequence number accepted */
    seq_t last;          /* last sequence number folded */
    unsigned long long folded;  /* bit k set: last-k has been folded */
    int have_last;
    int conflicts;       /* broken partial orders reported */
    int ambiguous;       /* last conflict was an unordered (2^31) pair */
    seq_t conflict_older;
    seq_t conflict_newer;
} Replay;

static void replay_init(Replay* rp) {
    memset(rp, 0, sizeof(*rp));
}

/* A window sequence number plus its unwrapped height, for callers that hold
 * a session's true tick count instead of re-deriving it from the wire. */
typedef struct {
    seq_t wire;
    unsigned long long truth;
} Stamp;

/* Tie-break for wire numbers the half-open rule leaves tied: two epochs can
 * emit the same wire number, and higher truth (then higher wire) wins.  Note
 * the antipodal distance 2^31 is always further apart than the window, so
 * only exact repeats can tie inside it. */
static int stamp_newer(Stamp a, Stamp b) {
    if (a.truth != b.truth) return a.truth > b.truth;
    return a.wire > b.wire;
}

/* The window order of two stamps, which is the order the replay must use. */
static int window_newer_stamped(seq_t head, Stamp a, Stamp b) {
    if (!window_contains(head, a.wire) || !window_contains(head, b.wire)) return 0;
    if (seq_newer(a.wire, b.wire)) return 1;
    if (seq_newer(b.wire, a.wire)) return 0;
    return stamp_newer(a, b);
}

/* Report a pair whose window order was violated.  names/values are the two
 * conflicting sequence numbers, whichever side the caller has for them. */
static void report_conflict(Replay* rp, seq_t a, seq_t b, int ambiguous) {
    rp->conflicts++;
    rp->ambiguous = ambiguous;
    rp->conflict_older = a;
    rp->conflict_newer = b;
    if (ambiguous)
        printf("conflict=1 ambiguous=1 a=%u b=%u\n", a, b);
    else
        printf("conflict=1 older=%u newer=%u\n", a, b);
}

/* Accept one record and keep r[] in true (oldest first) order.
 *
 * Conflict cases (both numbers are reported):
 *   - a number exactly 2^31 from the head is unordered, so the partial
 *     order has no answer for where it belongs;
 *   - a number older than something already folded arrived too late: the
 *     reorder window should have contained it, and now it cannot be replayed
 *     in order without rewinding (a re-delivery of a folded number is
 *     idempotent, not a conflict).
 * A number that is merely further behind than the window is dropped. */
static int replay_submit(Replay* rp, seq_t seq, float x) {
    if (rp->n >= RING) return -1;
    if (rp->n == 0 || seq_newer(seq, rp->head)) rp->head = seq;

    seq_t behind = rp->head - seq;
    if (behind == SEQ_HALF) {
        report_conflict(rp, seq, rp->head, 1);
        return -1;
    }
    /* Merely late (further back than the window) is not a conflict: the
     * window is exactly the accepted amount of reordering, so drop it. */
    if (!window_contains(rp->head, seq)) return -1;
    if (rp->have_last && !seq_newer(seq, rp->last)) {
        seq_t back = rp->last - seq;
        if (seq == rp->last || (back < (seq_t)WIN && ((rp->folded >> back) & 1ull)))
            return 0;                    /* re-delivery of a folded number */
        report_conflict(rp, seq, rp->last, 0);
        return -1;
    }

    for (int i = 0; i < rp->n; i++)       /* idempotent re-submission */
        if (rp->r[i].seq == seq) return 0;

    int i = rp->n;
    while (i > 0 && seq_newer(rp->r[i - 1].seq, seq)) {
        rp->r[i] = rp->r[i - 1];
        i--;
    }
    rp->r[i].seq = seq;
    rp->r[i].x = x;
    rp->n++;
    return 0;
}

/* Mark seq as folded, aging the bit history by the step from last. */
static void folded_advance(Replay* rp, seq_t seq) {
    if (!rp->have_last) {
        rp->folded = 1ull;
    } else {
        seq_t step = seq - rp->last;
        rp->folded = step < 64u ? ((rp->folded << step) | 1ull) : 1ull;
    }
    rp->last = seq;
    rp->have_last = 1;
}

/* Fold buffered records in window order.  A record that is not newer than
 * its predecessor, or not newer than the last folded number, means the
 * window order was broken; report the offending pair and stop.  Exact
 * repeats of a number handled earlier in this call are idempotent.  Returns
 * the folded count or -1 on conflict.  Folded records are dropped; anything
 * after a conflict stays buffered for a later call. */
static int replay_fold(Replay* rp, float* out, int cap) {
    int folded = 0, start = 0;
    for (int i = 0; i < rp->n; i++) {
        seq_t cur = rp->r[i].seq;
        int seen = 0;
        for (int j = start; j < i && !seen; j++)
            if (rp->r[j].seq == cur) seen = 1;
        if (seen) continue;
        if (rp->have_last && cur != rp->last) {
            seq_t back = rp->last - cur;
            if (back < 64u && ((rp->folded >> back) & 1ull)) continue;
        }
        if (rp->have_last && !seq_newer(cur, rp->last)) {
            report_conflict(rp, cur, rp->last, 0);
            return -1;
        }
        if (folded < cap) out[folded] = rp->r[i].x;
        folded++;
        start = i + 1;
        folded_advance(rp, cur);
    }
    /* Drop everything that has been folded or skipped. */
    int keep = rp->n - start;
    if (keep > 0) memmove(&rp->r[0], &rp->r[start], sizeof(Rec) * (size_t)keep);
    rp->n = keep;
    return folded;
}

/* --- tests ------------------------------------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Verify the window order against a true (unwrapped) order for every pair.
 * base is a 64-bit unwrapped start; wire numbers are base+i mod 2^32. */
static int window_matches_truth(unsigned long long base, unsigned n) {
    seq_t head = (seq_t)(base + n - 1);
    int ok = 1;
    for (unsigned i = 0; i < n; i++) {
        seq_t a = (seq_t)(base + i);
        if (!window_contains(head, a)) ok = 0;
        for (unsigned j = i + 1; j < n; j++) {
            seq_t b = (seq_t)(base + j);
            int truth = (base + j) > (base + i);
            if (window_newer(head, b, a) != truth) ok = 0;
            if (window_newer(head, a, b) != !truth) ok = 0;
        }
    }
    return ok;
}

static int half_open_sweep(void) {
    const unsigned deltas[] = { 1u, 2u, 3u, WIN - 1u, SEQ_HALF - 1u,
                                SEQ_HALF, SEQ_HALF + 1u, 0xffffffffu };
    int ok = 1;
    for (unsigned i = 0; i < sizeof(deltas) / sizeof(deltas[0]); i++) {
        unsigned d = deltas[i];
        seq_t b = 0x12345678u;
        seq_t a = b + d;
        int ab = seq_newer(a, b), ba = seq_newer(b, a);
        int want_ab = (d > 0u && d < SEQ_HALF);
        int want_ba = (d > SEQ_HALF);
        if (ab != want_ab || ba != want_ba) ok = 0;
    }
    return ok;
}

/* The ambiguity (a distance of exactly 2^31) has to be reported, not
 * silently resolved to either side. */
static int replay_submit_probe(seq_t first, seq_t second) {
    Replay rp;
    replay_init(&rp);
    replay_submit(&rp, first, 0.0f);
    int r = replay_submit(&rp, second, 1.0f);
    if (r != -1 || rp.conflicts != 1 || !rp.ambiguous) return 0;
    return -1;
}

static int partial_order_axioms(unsigned long long base, unsigned n) {
    seq_t head = (seq_t)(base + n - 1);
    int irreflexive = 1, asymmetric = 1, transitive = 1, total = 1;
    for (unsigned i = 0; i < n; i++) {
        seq_t a = (seq_t)(base + i);
        if (window_newer(head, a, a)) irreflexive = 0;
        for (unsigned j = 0; j < n; j++) {
            seq_t b = (seq_t)(base + j);
            if (window_newer(head, a, b) && window_newer(head, b, a)) asymmetric = 0;
            if (i != j) {
                int one = window_newer(head, a, b);
                int two = window_newer(head, b, a);
                if (one == two) total = 0;
            }
            for (unsigned k = 0; k < n; k++) {
                seq_t c = (seq_t)(base + k);
                if (window_newer(head, a, b) && window_newer(head, b, c) &&
                    !window_newer(head, a, c)) transitive = 0;
            }
        }
    }
    return irreflexive && asymmetric && transitive && total;
}

/* The same axioms for the window order on stamps, including the equal wire
 * numbers that the raw half-open rule leaves tied. */
static int stamp_axioms(unsigned long long base, unsigned n) {
    seq_t head = (seq_t)(base + n - 1);
    int irreflexive = 1, asymmetric = 1, transitive = 1, total = 1;
    for (unsigned i = 0; i < 2u * n; i++) {
        Stamp a = { (seq_t)(base + i / 2u), i };   /* two heights per wire number */
        Stamp a2 = { (seq_t)(base + i / 2u), i + 2u * n };
        if (window_newer_stamped(head, a, a)) irreflexive = 0;
        for (unsigned j = 0; j < 2u * n; j++) {
            Stamp b = { (seq_t)(base + j / 2u), j };
            Stamp b2 = { (seq_t)(base + j / 2u), j + 2u * n };
            if (window_newer_stamped(head, a, b) && window_newer_stamped(head, b, a))
                asymmetric = 0;
            int one = window_newer_stamped(head, a2, b2);
            int two = window_newer_stamped(head, b2, a2);
            if (i != j && one == two) total = 0;
            for (unsigned k = 0; k < 2u * n; k++) {
                Stamp c2 = { (seq_t)(base + k / 2u), k + 2u * n };
                if (window_newer_stamped(head, a2, b2) && window_newer_stamped(head, b2, c2) &&
                    !window_newer_stamped(head, a2, c2)) transitive = 0;
            }
        }
    }
    return irreflexive && asymmetric && transitive && total;
}

/* The stamp order must reproduce the unwrapped order it is built from. */
static int stamp_matches_truth(unsigned long long base, unsigned n) {
    seq_t head = (seq_t)(base + n - 1);
    int ok = 1;
    for (unsigned i = 0; i < n; i++) {
        Stamp a = { (seq_t)(base + i), base + i };
        for (unsigned j = i + 1u; j < n; j++) {
            Stamp b = { (seq_t)(base + j), base + j };
            if (!window_newer_stamped(head, b, a)) ok = 0;
            if (window_newer_stamped(head, a, b)) ok = 0;
        }
    }
    return ok;
}

int main(void) {
    /* Half-open comparison: boundary points, equality, and the wrap. */
    printf("--- half-open compare\n");
    seq_t pivot = 0x40000000u;
    check("newer_forward", seq_newer(pivot + 1u, pivot));
    check("older_by_one", !seq_newer(pivot - 1u, pivot));
    check("older_reverse", !seq_newer(pivot, pivot + 1u));
    check("equal_unordered", !seq_newer(pivot, pivot));
    check("antipodal_unordered", !seq_newer(pivot + SEQ_HALF, pivot) &&
                                  !seq_newer(pivot, pivot + SEQ_HALF));
    check("half_open_sweep", half_open_sweep());
    check("wrap_newer", seq_newer(5u, 0xfffffffdu));
    check("wrap_dist_one", seq_diff(0u, 0xffffffffu) == 1);

    /* Ordering must not depend on where the wrap falls.  Shift one logical
     * timeline by several offsets, including ones that cross 0. */
    {
        unsigned long long bases[] = { 0ull, 5ull, 0xffffff00ull, 0xffffffffull };
        unsigned long long origin = 0x100000ull;
        int shift_ok = 1;
        for (unsigned i = 0; i < 4; i++) {
            seq_t head = (seq_t)(bases[i] + origin + 20u);
            for (unsigned a = 0; a < 20u; a++) {
                for (unsigned b = a + 1u; b < 20u; b++) {
                    seq_t sa = (seq_t)(bases[i] + origin + a);
                    seq_t sb = (seq_t)(bases[i] + origin + b);
                    if (!window_newer(head, sb, sa) || window_newer(head, sa, sb))
                        shift_ok = 0;
                }
                seq_t straight = (seq_t)(bases[i] + origin + a);
                seq_t none = (seq_t)(bases[0] + origin + a);
                if (seq_diff(straight, (seq_t)(bases[i] + origin)) !=
                    seq_diff(none, (seq_t)origin)) shift_ok = 0;
            }
        }
        check("wrap_shift_invariant", shift_ok);
    }

    /* Window order must equal the true order for every pair, including
     * sequences that straddle 2^32. 2*WIN < 2^31 keeps this total. */
    printf("--- reorder window\n");
    const unsigned long long starts[] = {
        0ull, 1ull, 1000ull, 0x7ffffff0ull, 0xffffffc0ull,
        0xffffffe0ull, 0xfffffffeull, 0xffffffffull
    };
    int order_ok = 1;
    for (unsigned i = 0; i < sizeof(starts) / sizeof(starts[0]); i++)
        if (!window_matches_truth(starts[i], WIN)) order_ok = 0;
    check("window_order_matches_true_order", order_ok);

    int axioms_ok = 1;
    for (unsigned i = 0; i < sizeof(starts) / sizeof(starts[0]); i++)
        if (!partial_order_axioms(starts[i], 24u)) axioms_ok = 0;
    check("window_partial_order", axioms_ok);

    int stamps_ok = 1;
    for (unsigned i = 0; i < sizeof(starts) / sizeof(starts[0]); i++) {
        if (!stamp_axioms(starts[i], 12u)) stamps_ok = 0;
        if (!stamp_matches_truth(starts[i], WIN)) stamps_ok = 0;
    }
    check("window_stamp_total_order", stamps_ok);
    /* The tie-break only fires where the raw rule is unordered: equal wire
     * numbers from different epochs.  (A distance of 2^31 is always further
     * apart than the window, so it cannot occur inside one.) */
    {
        Stamp a = { 100u, 5ull }, b = { 100u, 4ull };
        check("stamp_equal_wire_breaks_tie", window_newer_stamped(100u, a, b) &&
                                              !window_newer_stamped(100u, b, a));
        check("stamp_same_wire_no_newer", !stamp_newer(a, a) && !stamp_newer(b, b));
    }

    seq_t head = 0x00000003u;
    check("window_total_inside", window_newer(head, 3u, 0xffffffffu) &&
                                 window_newer(head, 2u, 0xfffffffdu));
    check("window_total_inside_below_wrap", window_newer(0xffffffe4u, 0xffffffe4u, 0xffffffe0u));
    check("window_boundary_included", window_contains(head, head - (WIN - 1u)));
    check("window_boundary_excluded", !window_contains(head, head - WIN));
    check("outside_window_unordered", !window_newer(head, head - WIN, head));

    /* The window bound is tight: the oldest accepted number is head-WIN+1;
     * one further back is dropped without being a conflict, because the
     * window is exactly how much reordering the protocol tolerates. */
    {
        Replay rp;
        replay_init(&rp);
        replay_submit(&rp, 1000u, 0.0f);
        check("oldest_in_window_accepted", replay_submit(&rp, 1000u - (WIN - 1u), 0.0f) == 0);
        check("just_outside_dropped", replay_submit(&rp, 1000u - WIN, 0.0f) == -1);
        check("drop_is_not_conflict", rp.conflicts == 0);
    }

    /* Wrap-point continuity: the step across 0 is one tick, and the window
     * that straddles it stays ordered with no gap. */
    printf("--- wrap continuity\n");
    check("wrap_step_is_one", seq_diff(0u, 0xffffffffu) == seq_diff(1u, 0u));
    int contiguous = 1;
    for (seq_t s = 0xfffffffcu; s != 4u; s++)
        if (!seq_newer(s + 1u, s)) contiguous = 0;
    check("wrap_contiguous", contiguous);
    check("wrap_window_ordered", window_matches_truth(0xffffffc0ull, WIN));
    check("wrap_window_plus", window_matches_truth(0xfffffffdu, 8u));

    /* Replay over submissions arriving out of order.  The same logical log
     * is run once far from the wrap and once across it; both must fold to
     * the identical true order. */
    printf("--- replay\n");
    {
        enum { N = 48 };
        float xs[N];
        seq_t plain[N], wrapped[N], receive[N];
        unsigned perm[N];
        for (int i = 0; i < N; i++) {
            xs[i] = (float)i;
            plain[i] = 1000u + (unsigned)i;
            wrapped[i] = 0xfffffff0u + (unsigned)i;   /* crosses 2^32 at i=16 */
            perm[i] = (unsigned)i;
        }
        for (int i = N - 1; i > 0; i--) {             /* deterministic shuffle */
            int j = (i * 37 + 11) % (i + 1);
            unsigned t = perm[i]; perm[i] = perm[j]; perm[j] = t;
        }
        for (int i = 0; i < N; i++) receive[i] = perm[i];

        float out_plain[N], out_wrap[N];
        Replay rp_plain, rp_wrap;
        int np = 0, nw = 0;
        replay_init(&rp_plain);
        replay_init(&rp_wrap);
        for (int i = 0; i < N; i++) {
            unsigned k = receive[i];
            if (replay_submit(&rp_plain, plain[k], xs[k]) != 0) np = -1;
            if (replay_submit(&rp_wrap, wrapped[k], xs[k]) != 0) nw = -1;
        }
        int fp = np < 0 ? -1 : replay_fold(&rp_plain, out_plain, N);
        int fw = nw < 0 ? -1 : replay_fold(&rp_wrap, out_wrap, N);
        check("replay_folds_all", fp == N && fw == N);
        int in_order = (fp == N);
        for (int i = 0; in_order && i < N; i++)
            if (out_plain[i] != (float)i) in_order = 0;
        check("plain_replay_true_order", in_order);
        int wrap_in_order = (fw == N);
        for (int i = 0; wrap_in_order && i < N; i++)
            if (out_wrap[i] != (float)i) wrap_in_order = 0;
        check("wrap_replay_true_order", wrap_in_order);
        int same = (fp == fw);
        for (int i = 0; same && i < N; i++)
            if (out_plain[i] != out_wrap[i]) same = 0;
        check("wrap_matches_no_wrap", same);
        check("no_conflicts", rp_plain.conflicts == 0 && rp_wrap.conflicts == 0);
    }

    /* Replay exactly around the wrap point. */
    {
        enum { M = 8 };
        float xs[M] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        seq_t seqs[M] = { 0xfffffffeu, 0xffffffffu, 0u, 1u, 2u, 3u, 4u, 5u };
        seq_t recv[M] = { 2u, 6u, 0u, 4u, 1u, 7u, 3u, 5u }; /* indices */
        Replay rp;
        replay_init(&rp);
        int bad = 0;
        for (int i = 0; i < M; i++)
            if (replay_submit(&rp, seqs[recv[i]], xs[recv[i]]) != 0) bad = 1;
        float out[M];
        int n = bad ? -1 : replay_fold(&rp, out, M);
        int cont = (n == M);
        for (int i = 0; cont && i < M; i++)
            if (out[i] != (float)i) cont = 0;
        check("wrap_point_replay_contiguous", cont);
    }

    /* Break the partial order and require both numbers in the report.
     * A straggler that is older than what was already folded. */
    printf("--- conflict\n");
    {
        Replay rp;
        replay_init(&rp);
        replay_submit(&rp, 500u, 0.0f);
        replay_submit(&rp, 501u, 1.0f);
        float out[4];
        check("fold_before_conflict", replay_fold(&rp, out, 4) == 2);
        replay_init(&rp);
        replay_submit(&rp, 500u, 0.0f);
        float out1[4];
        check("fold_first", replay_fold(&rp, out1, 4) == 1);
        check("late_straggler_reported", replay_submit(&rp, 490u, 0.0f) == -1);
        check("conflict_detected", rp.conflicts == 1);
        check("conflict_reports_older", rp.conflict_older == 490u);
        check("conflict_reports_newer", rp.conflict_newer == 500u);
    }

    /* 2^31 away: the partial order has no answer, so it must be reported. */
    check("antipodal_reported", replay_submit_probe(0x1000u, 0x1000u + SEQ_HALF) == -1);
    {
        Replay rp;
        replay_init(&rp);
        replay_submit(&rp, 0x1000u, 0.0f);
        int r = replay_submit(&rp, 0x1000u + SEQ_HALF, 1.0f);
        /* Not comparable, so there is no older/newer label: the report is
         * just the pair, in the order it was observed. */
        check("antipodal_numbers", r == -1 &&
              ((rp.conflict_older == 0x1000u && rp.conflict_newer == 0x1000u + SEQ_HALF) ||
               (rp.conflict_newer == 0x1000u && rp.conflict_older == 0x1000u + SEQ_HALF)));
    }

    /* Buffered order corrupted after the fact: fold must still catch it. */
    {
        Replay rp;
        replay_init(&rp);
        replay_submit(&rp, 600u, 6.0f);
        replay_submit(&rp, 601u, 7.0f);
        replay_submit(&rp, 602u, 8.0f);
        Rec t = rp.r[0]; rp.r[0] = rp.r[1]; rp.r[1] = t;   /* corrupt */
        float out[4];
        check("corrupt_order_detected", replay_fold(&rp, out, 4) == -1);
        check("corrupt_order_numbers", rp.conflict_older == 600u &&
                                        rp.conflict_newer == 601u);
    }

    /* Conflict at the wrap point reports the wrapped numbers. */
    {
        Replay rp;
        replay_init(&rp);
        replay_submit(&rp, 0xffffffffu, 0.0f);
        replay_submit(&rp, 1u, 1.0f);
        float out[4];
        check("wrap_fold_ok", replay_fold(&rp, out, 4) == 2);
        replay_init(&rp);
        replay_submit(&rp, 1u, 0.0f);
        check("wrap_fold_first", replay_fold(&rp, out, 4) == 1);
        /* 0xffffffff is older than 1, but arrived after 1 was folded. */
        check("wrap_conflict_detected", replay_submit(&rp, 0xffffffffu, 0.0f) == -1);
        check("wrap_conflict_numbers", rp.conflict_older == 0xffffffffu &&
                                        rp.conflict_newer == 1u);
    }

    /* Folding is incremental: a record that arrives after a fold is appended
     * in order, and the same number fed twice is idempotent rather than a
     * conflict. */
    {
        Replay rp;
        replay_init(&rp);
        float out[8];
        replay_submit(&rp, 700u, 0.0f);
        check("incremental_first", replay_fold(&rp, out, 8) == 1 && out[0] == 0.0f);
        replay_submit(&rp, 701u, 1.0f);
        replay_submit(&rp, 702u, 2.0f);
        check("incremental_next", replay_fold(&rp, out, 8) == 2 &&
                                  out[0] == 1.0f && out[1] == 2.0f);
        replay_submit(&rp, 702u, 2.0f);
        check("repeat_across_fold_ignored", replay_fold(&rp, out, 8) == 0);
        check("repeat_no_conflict", rp.conflicts == 0);
        replay_submit(&rp, 700u, 0.0f);   /* in window, but already folded */
        check("repeat_old_ignored", rp.conflicts == 0);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
