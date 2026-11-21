
/* budget: priority bit allocation over a fixed bandwidth budget.
 *
 * Each state carries an impact weight. A greedy allocator spends every spare
 * bit on the field with the largest marginal weighted error reduction. When
 * the budget cannot cover every floor, decorative fields are degraded to
 * zero bits before critical floors are touched. The result is compared with
 * an even split. A zero (or negative) budget is rejected.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_FIELD 16

typedef struct {
    int id;
    int critical;     /* 1 = key state, 0 = decorative */
    float weight;     /* impact area, must be > 0 */
    int min_bits;     /* floor bits demanded */
    int max_bits;     /* ceiling bits */
    float range;      /* represented value span */
    int bits;         /* allocation result */
} Field;

/* uniform quantizer over `range` with 2^b levels: max abs error is step/2 */
static float quant_err(float range, int b) {
    return ldexpf(range, -(b + 1));
}

static float field_err(const Field* f, int b) {
    return f->weight * quant_err(f->range, b);
}

static int validate(const Field* f, int n, int budget) {
    if (f == NULL || n <= 0 || n > MAX_FIELD) return -1;
    if (budget <= 0) return -1;                       /* zero budget: no deal */
    for (int i = 0; i < n; i++) {
        if (f[i].weight <= 0.0f || f[i].range <= 0.0f) return -1;
        if (f[i].min_bits < 0 || f[i].max_bits < f[i].min_bits) return -1;
    }
    return 0;
}

/* priority allocation; returns bits actually spent, or -1 on bad input /
 * an infeasible critical floor */
static int budget_allocate(Field* f, int n, int budget) {
    if (validate(f, n, budget) != 0) return -1;

    int bits[MAX_FIELD];
    int used = 0;
    for (int i = 0; i < n; i++) { bits[i] = f[i].min_bits; used += bits[i]; }

    /* oversubscribed: decorative fields are cut to zero and stay cut for
     * this allocation; every spare bit then goes to critical states */
    int degraded = 0;
    if (used > budget) {
        for (int i = 0; i < n; i++) {
            if (!f[i].critical) { used -= bits[i]; bits[i] = 0; }
        }
        degraded = 1;
        if (used > budget) return -1;                /* critical floor infeasible */
    }

    /* greedy: each spare bit buys the largest marginal weighted gain */
    for (;;) {
        if (used >= budget) break;
        int best = -1;
        float best_gain = 0.0f;
        for (int i = 0; i < n; i++) {
            if (bits[i] >= f[i].max_bits) continue;
            if (degraded && !f[i].critical) continue;
            float gain = field_err(&f[i], bits[i]) - field_err(&f[i], bits[i] + 1);
            if (gain > best_gain ||
                (gain == best_gain && best >= 0 &&
                 f[i].critical && !f[best].critical)) {
                best = i;
                best_gain = gain;
            }
        }
        if (best < 0) break;                         /* all fields at ceiling */
        bits[best]++;
        used++;
    }

    for (int i = 0; i < n; i++) f[i].bits = bits[i];
    return used;
}

/* baseline: even split, remainder spread round-robin past ceilings */
static int budget_uniform(const Field* f, int n, int budget, int* out) {
    if (f == NULL || out == NULL || n <= 0 || n > MAX_FIELD || budget <= 0)
        return -1;
    int base = budget / n;
    int used = 0;
    for (int i = 0; i < n; i++) {
        out[i] = base < f[i].max_bits ? base : f[i].max_bits;
        used += out[i];
    }
    int progressed = 1;
    while (used < budget && progressed) {
        progressed = 0;
        for (int i = 0; i < n && used < budget; i++) {
            if (out[i] < f[i].max_bits) { out[i]++; used++; progressed = 1; }
        }
    }
    return used;
}

static float total_weighted_err(const Field* f, int n, const int* bits) {
    float e = 0.0f;
    for (int i = 0; i < n; i++) e += field_err(&f[i], bits[i]);
    return e;
}

static float critical_weighted_err(const Field* f, int n, const int* bits) {
    float e = 0.0f;
    for (int i = 0; i < n; i++)
        if (f[i].critical) e += field_err(&f[i], bits[i]);
    return e;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* id kind weight min max range */
static const Field TEMPLATE[6] = {
    { 1, 1, 10.0f, 4, 12, 100.0f, 0 },   /* position: key state */
    { 2, 1,  8.0f, 4, 10, 100.0f, 0 },   /* health:   key state */
    { 3, 1,  5.0f, 3,  8,  50.0f, 0 },   /* velocity: key state */
    { 4, 0,  1.0f, 2,  6,  10.0f, 0 },   /* hat color: decorative */
    { 5, 0,  1.0f, 2,  6,  10.0f, 0 },   /* particles: decorative */
    { 6, 0,  2.0f, 2,  6,  20.0f, 0 },   /* name tag:  decorative */
};

int main(void) {
    Field f[6];
    memcpy(f, TEMPLATE, sizeof(f));
    int n = 6, budget = 24;

    int spent = budget_allocate(f, n, budget);
    check("alloc_ok", spent > 0);
    check("within_budget", spent <= budget);
    int sum = 0;
    for (int i = 0; i < n; i++) sum += f[i].bits;
    check("bits_consistent", sum == spent);

    int ubits[MAX_FIELD];
    int uspent = budget_uniform(f, n, budget, ubits);
    check("uniform_ok", uspent > 0 && uspent <= budget);

    int pbits[MAX_FIELD];
    for (int i = 0; i < n; i++) pbits[i] = f[i].bits;

    printf("\n%-4s %-10s %6s %8s\n", "id", "kind", "prio", "uniform");
    for (int i = 0; i < n; i++)
        printf("%-4d %-10s %6d %8d\n", f[i].id,
               f[i].critical ? "critical" : "decor", pbits[i], ubits[i]);

    float pcrit = critical_weighted_err(f, n, pbits);
    float ucrit = critical_weighted_err(f, n, ubits);
    float ratio = ucrit / pcrit;                       /* uniform / priority */
    float reduction = (1.0f - pcrit / ucrit) * 100.0f;
    printf("\ncrit_err_priority=%.4f\n", pcrit);
    printf("crit_err_uniform=%.4f\n", ucrit);
    printf("crit_err_ratio=%.3f\n", ratio);
    printf("crit_reduction_pct=%.1f\n", reduction);

    check("crit_better", pcrit < ucrit);
    check("ratio_above_one", ratio > 1.0f);
    int every_crit = 1;
    for (int i = 0; i < n; i++)
        if (f[i].critical && pbits[i] < ubits[i]) every_crit = 0;
    check("every_crit_wins", every_crit);
    check("total_err_no_worse",
          total_weighted_err(f, n, pbits) <= total_weighted_err(f, n, ubits));

    /* tight budget: floors sum to 18, decorative must be degraded to 0 and
     * the remaining 3 bits absorbed by critical states */
    Field g[6];
    memcpy(g, TEMPLATE, sizeof(g));
    int tight = budget_allocate(g, n, 14);
    check("degrade_ok", tight > 0 && tight <= 14);
    int decor_zero = 1, crit_floor = 1;
    for (int i = 0; i < n; i++) {
        if (!g[i].critical && g[i].bits != 0) decor_zero = 0;
        if (g[i].critical && g[i].bits < g[i].min_bits) crit_floor = 0;
    }
    check("decorative_degraded", decor_zero);
    check("critical_floor_kept", crit_floor);
    check("degraded_budget_used", tight == 14);

    /* 11 bits of critical floor cannot fit in 10 */
    Field h[6];
    memcpy(h, TEMPLATE, sizeof(h));
    check("infeasible_rejected", budget_allocate(h, n, 10) == -1);
    check("zero_budget_rejected", budget_allocate(h, n, 0) == -1);
    check("neg_budget_rejected", budget_allocate(h, n, -4) == -1);
    check("null_rejected", budget_allocate(NULL, n, budget) == -1);

    Field bad = { 7, 1, 0.0f, 2, 6, 10.0f, 0 };
    check("bad_weight_rejected", budget_allocate(&bad, 1, budget) == -1);
    bad.weight = 1.0f; bad.min_bits = 8;
    check("bad_floor_rejected", budget_allocate(&bad, 1, budget) == -1);
    check("uniform_bad_input", budget_uniform(h, n, 0, ubits) == -1);

    /* generous budget: allocation stops at the ceilings */
    Field k[6];
    memcpy(k, TEMPLATE, sizeof(k));
    int capped = budget_allocate(k, n, 1000);
    int ceiling = 0;
    for (int i = 0; i < n; i++) ceiling += k[i].max_bits;
    check("ceiling_caps", capped == ceiling);

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
