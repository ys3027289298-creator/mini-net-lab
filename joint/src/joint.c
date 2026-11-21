
/* joint: joint AOI-visibility x field-importance send-set planner under one shared
 * bandwidth budget.
 *
 * AOI culling and snapshot field compression each optimize locally against their own
 * budget split: AOI admits whole entities by visibility, snapshot then picks field
 * types by importance. Neither stage sees the other's weight, so slack in one stage
 * cannot fund picks in the other, and the combined spend is worse than one global
 * optimum. This module fuses both signals into a single utility per (entity, field)
 * candidate and runs one 0/1 knapsack over the total byte budget.
 *
 * Rules:
 *  - utility(ent, field) = visibility(ent) * importance(field)
 *  - zero-importance fields never enter the candidate set, so they never spend budget
 *  - zero-visibility (outside AOI) candidates carry no utility and are never sent
 *  - an empty plan is reported to the caller with a reason; the caller must not
 *    fall back to sending the full state
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_ENT 8
#define MAX_FIELDS 4
#define MAX_CAND (MAX_ENT * MAX_FIELDS)
#define MAX_BUDGET 256
#define EPS 1e-5f

typedef struct { int id; float x, y; float vis; } JEnt;
typedef struct { int id; int imp; int cost; } JField;
typedef struct { int ent; int field; int cost; float benefit; } Pick;

typedef struct { int ent; int field; int cost; float benefit; } Cand;

/* AOI-derived fractional visibility: 0 at/beyond the radius, linear falloff inside. */
static float vis_of(float dx, float dy, float radius) {
    if (radius <= 0.0f) return 0.0f;
    float d = sqrtf(dx * dx + dy * dy);
    if (d >= radius) return 0.0f;
    float v = 1.0f - d / radius;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

/*
 * Joint planner: maximize sum of visibility*importance over (entity, field) candidates
 * subject to total encoded cost <= budget (0/1 knapsack, exact DP over integer bytes).
 *
 * Returns:
 *    1 - non-empty set written to out
 *    0 - empty set; *reason explains why and the caller must report it, not full-send
 *   -1 - invalid arguments
 */
static int plan_joint(const JEnt* ents, int ne, const JField* fs, int nf,
                      int budget, Pick* out, int cap, int* nout, int* used,
                      const char** reason) {
    if (ents == NULL || fs == NULL || out == NULL || nout == NULL ||
        used == NULL || reason == NULL) return -1;
    if (ne < 0 || ne > MAX_ENT || nf < 0 || nf > MAX_FIELDS ||
        budget < 0 || budget > MAX_BUDGET || cap < 0) return -1;

    *nout = 0;
    *used = 0;
    *reason = "ok";

    Cand c[MAX_CAND];
    int nc = 0;
    for (int j = 0; j < nf; j++) {
        if (fs[j].cost <= 0 || fs[j].imp < 0) { *reason = "bad_field"; return -1; }
    }
    for (int i = 0; i < ne; i++) {
        if (ents[i].vis <= 0.0f) continue;                 /* outside AOI: no utility */
        for (int j = 0; j < nf; j++) {
            if (fs[j].imp == 0) continue;                  /* zero importance: no budget */
            if (nc >= MAX_CAND) { *reason = "too_many"; return -1; }
            c[nc].ent = ents[i].id;
            c[nc].field = fs[j].id;
            c[nc].cost = fs[j].cost;
            c[nc].benefit = ents[i].vis * (float)fs[j].imp;
            nc++;
        }
    }
    if (nc == 0) {
        *reason = "no_positive_benefit_candidate";
        return 0;
    }

    float dp[MAX_BUDGET + 1];
    unsigned char take[MAX_CAND][MAX_BUDGET + 1];
    memset(dp, 0, sizeof(dp));
    memset(take, 0, sizeof(take));
    for (int k = 0; k < nc; k++) {
        for (int b = budget; b >= c[k].cost; b--) {
            float v = dp[b - c[k].cost] + c[k].benefit;
            if (v > dp[b]) {                               /* strict >: ties stay deterministic */
                dp[b] = v;
                take[k][b] = 1;
            }
        }
    }

    int b = budget, n = 0;
    for (int k = nc - 1; k >= 0; k--) {
        if (take[k][b]) {
            if (n < cap) {
                out[n].ent = c[k].ent;
                out[n].field = c[k].field;
                out[n].cost = c[k].cost;
                out[n].benefit = c[k].benefit;
                n++;
            }
            b -= c[k].cost;
        }
    }
    if (n == 0) {
        *reason = "budget_below_cheapest_candidate";
        return 0;
    }
    *nout = n;
    *used = budget - b;
    return 1;
}

/*
 * Baseline: the two local optima composed, with a fixed 50/50 split of the total
 * budget. AOI stage ranks entities by visibility and admits whole entities (charged
 * the cost of every field, since field selection is not its job); snapshot stage
 * ranks field types by importance and enables a field for all admitted entities while
 * its share lasts. Zero-importance fields are skipped by the snapshot stage.
 *
 * The set it produces costs at most `budget`, so it is always a feasible solution of
 * the joint knapsack; the joint result is therefore guaranteed >= this one.
 */
static float plan_separate(const JEnt* ents, int ne, const JField* fs, int nf,
                           int budget, Pick* out, int cap, int* nout, int* used) {
    int ba = budget / 2;
    int bs = budget - ba;

    int full = 0;
    for (int j = 0; j < nf; j++) full += fs[j].cost;

    int order[MAX_ENT], na = 0;
    for (int i = 0; i < ne; i++) if (ents[i].vis > 0.0f) order[na++] = i;
    for (int i = 1; i < na; i++) {                          /* insertion sort, vis desc */
        int k = order[i], j = i - 1;
        while (j >= 0 && ents[order[j]].vis < ents[k].vis) { order[j + 1] = order[j]; j--; }
        order[j + 1] = k;
    }
    int admitted[MAX_ENT], nad = 0, rema = ba;
    for (int i = 0; i < na; i++) {
        if (full <= rema) { admitted[nad++] = order[i]; rema -= full; }
    }

    int fo[MAX_FIELDS], nfo = 0;
    for (int j = 0; j < nf; j++) if (fs[j].imp > 0) fo[nfo++] = j;
    for (int i = 1; i < nfo; i++) {                         /* insertion sort, imp desc */
        int k = fo[i], j = i - 1;
        while (j >= 0 && fs[fo[j]].imp < fs[k].imp) { fo[j + 1] = fo[j]; j--; }
        fo[j + 1] = k;
    }

    int n = 0, rems = bs;
    for (int q = 0; q < nfo; q++) {
        int j = fo[q];
        int add = nad * fs[j].cost;                        /* all-or-nothing per field type */
        if (add > 0 && add <= rems) {
            for (int k = 0; k < nad; k++) {
                int ei = admitted[k];
                if (n < cap) {
                    out[n].ent = ents[ei].id;
                    out[n].field = fs[j].id;
                    out[n].cost = fs[j].cost;
                    out[n].benefit = ents[ei].vis * (float)fs[j].imp;
                    n++;
                }
            }
            rems -= add;
        }
    }
    *nout = n;
    *used = bs - rems;
    float ben = 0.0f;
    for (int i = 0; i < n; i++) ben += out[i].benefit;
    return ben;
}

static float pick_benefit(const Pick* p, int n) {
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += p[i].benefit;
    return s;
}

static int pick_cost(const Pick* p, int n) {
    int s = 0;
    for (int i = 0; i < n; i++) s += p[i].cost;
    return s;
}

static int pick_has_field(const Pick* p, int n, int field) {
    for (int i = 0; i < n; i++) if (p[i].field == field) return 1;
    return 0;
}

static int pick_has_ent(const Pick* p, int n, int ent) {
    for (int i = 0; i < n; i++) if (p[i].ent == ent) return 1;
    return 0;
}

/* caller policy: an empty plan is reported; bytes never fall back to a full send */
static int transmit_bytes(int status, int used) {
    return status == 1 ? used : 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    const float radius = 15.0f;
    float pos[5][2] = { {0, 0}, {3, 0}, {6, 0}, {9, 0}, {100, 100} };
    JEnt ents[5];
    for (int i = 0; i < 5; i++) {
        ents[i].id = i + 1;
        ents[i].x = pos[i][0];
        ents[i].y = pos[i][1];
        ents[i].vis = vis_of(pos[i][0], pos[i][1], radius);
    }
    /* POS, VEL carry utility; FLAGS has zero importance and must never spend budget */
    JField fs[3] = { {1, 10, 4}, {2, 6, 4}, {3, 0, 2} };
    const int B = 20;

    check("vis_falloff",
          fabsf(ents[0].vis - 1.0f) < EPS && fabsf(ents[1].vis - 0.8f) < EPS &&
          fabsf(ents[2].vis - 0.6f) < EPS && fabsf(ents[3].vis - 0.4f) < EPS &&
          ents[4].vis == 0.0f);

    Pick jp[MAX_CAND], sp[MAX_CAND];
    int nj = 0, ns = 0, usedj = 0, useds = 0;
    const char* reason = "unset";

    int status = plan_joint(ents, 5, fs, 3, B, jp, MAX_CAND, &nj, &usedj, &reason);
    float bj = pick_benefit(jp, nj);
    float bsep = plan_separate(ents, 5, fs, 3, B, sp, MAX_CAND, &ns, &useds);

    /* baseline: AOI share 10 admits one whole entity (full cost 10); snapshot share
       10 then buys POS+VEL for it: benefit 16, 8 bytes, 12 bytes of pooled slack wasted */
    check("separate_expected", ns == 2 && useds == 8 && fabsf(bsep - 16.0f) < EPS);
    /* joint: 5 candidates at 4 bytes each under budget 20, top utilities
       10 + 8 + 6 + 6 + 4.8 = 34.8 */
    check("joint_expected", status == 1 && nj == 5 && usedj == 20 &&
                            fabsf(bj - 34.8f) < 1e-4f);
    check("joint_cost_consistent", pick_cost(jp, nj) == usedj && usedj <= B);
    check("joint_beats_separate", bj > bsep);

    float ratio = bj / bsep;
    float gain_pct = (bj - bsep) / bsep * 100.0f;
    printf("separate_benefit=%.2f separate_bytes=%d\n", bsep, useds);
    printf("joint_benefit=%.2f joint_bytes=%d\n", bj, usedj);
    printf("gain_ratio=%.3f gain_pct=%.1f%%\n", ratio, gain_pct);
    check("gain_ratio_expected", fabsf(ratio - 2.175f) < 1e-3f);

    check("zero_importance_not_picked",
          !pick_has_field(jp, nj, 3) && !pick_has_field(sp, ns, 3));
    check("invisible_not_picked",
          !pick_has_ent(jp, nj, 5) && !pick_has_ent(sp, ns, 5));

    /* tight budget: zero-importance FLAGS must not squeeze out POS */
    Pick tight[MAX_CAND];
    int nt = 0, ut = 0;
    const char* rt = "unset";
    int stt = plan_joint(ents, 5, fs, 3, 4, tight, MAX_CAND, &nt, &ut, &rt);
    check("tight_picks_pos_only", stt == 1 && nt == 1 && ut == 4 &&
                                  tight[0].field == 1 && !pick_has_field(tight, nt, 3));

    /* empty plan: budget 0 -> report, zero bytes, no full send */
    Pick empty[MAX_CAND];
    int ne2 = 0, ue = 0;
    const char* re = "unset";
    int se = plan_joint(ents, 5, fs, 3, 0, empty, MAX_CAND, &ne2, &ue, &re);
    printf("empty_reason=%s\n", re);
    check("empty_budget_reported", se == 0 && ne2 == 0 && ue == 0 &&
                                   transmit_bytes(se, ue) == 0);

    /* empty plan: cheapest candidate costs 4, budget 3 cannot fund anything */
    int se3 = plan_joint(ents, 5, fs, 3, 3, empty, MAX_CAND, &ne2, &ue, &re);
    check("empty_tight_reported", se3 == 0 && ne2 == 0 &&
                                  transmit_bytes(se3, ue) == 0 &&
                                  strcmp(re, "budget_below_cheapest_candidate") == 0);

    /* empty plan: every field zero importance -> nothing may be sent even with budget */
    JField zf[2] = { {1, 0, 4}, {2, 0, 2} };
    int sz = plan_joint(ents, 5, zf, 2, 100, empty, MAX_CAND, &ne2, &ue, &re);
    check("empty_zeroimp_reported", sz == 0 && ne2 == 0 && ue == 0 &&
                                    transmit_bytes(sz, ue) == 0 &&
                                    strcmp(re, "no_positive_benefit_candidate") == 0);

    /* empty plan: no visible entities */
    JEnt blind[1] = { {1, 100, 100, 0.0f} };
    int sb = plan_joint(blind, 1, fs, 3, 100, empty, MAX_CAND, &ne2, &ue, &re);
    check("empty_invisible_reported", sb == 0 && ne2 == 0 &&
                                      transmit_bytes(sb, ue) == 0);

    /* dominance sweep: the separately-optimal set is always feasible for the joint
       knapsack, so joint benefit must dominate it at every budget */
    int dom_ok = 1;
    for (int b = 0; b <= 40; b++) {
        Pick pj[MAX_CAND], ps[MAX_CAND];
        int n1 = 0, n2 = 0, u1 = 0, u2 = 0;
        const char* rr = "unset";
        int s = plan_joint(ents, 5, fs, 3, b, pj, MAX_CAND, &n1, &u1, &rr);
        float vj = s == 1 ? pick_benefit(pj, n1) : 0.0f;
        float vs = plan_separate(ents, 5, fs, 3, b, ps, MAX_CAND, &n2, &u2);
        if (vj + EPS < vs || u1 > b) { dom_ok = 0; break; }
    }
    check("joint_dominates_at_all_budgets", dom_ok);

    check("bad_args",
          plan_joint(NULL, 5, fs, 3, B, jp, MAX_CAND, &nj, &usedj, &reason) == -1 &&
          plan_joint(ents, 5, fs, 3, -1, jp, MAX_CAND, &nj, &usedj, &reason) == -1 &&
          plan_joint(ents, 5, fs, 3, B + 256, jp, MAX_CAND, &nj, &usedj, &reason) == -1);
    JField bad1[1] = { {1, 1, 0} };
    JField bad2[1] = { {1, -1, 4} };
    check("bad_field",
          plan_joint(ents, 5, bad1, 1, B, jp, MAX_CAND, &nj, &usedj, &reason) == -1 &&
          plan_joint(ents, 5, bad2, 1, B, jp, MAX_CAND, &nj, &usedj, &reason) == -1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
