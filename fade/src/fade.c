
/* fade: visibility fade driven by the cull decision, with debounce hold.
 *
 * The cull decision (visible yes/no) is the only input.  A toggle only takes
 * effect once the new state has held for `hold` seconds, so jitter at the
 * boundary cannot restart the fade every frame.  Alpha is carried across a
 * direction change (a fade-out interrupted by a re-enter continues from where
 * it is), and both ends are pinned to exactly 0 / 1, so the composite at an
 * endpoint is pixel-identical to a hard cut with no residual image.  While a
 * fade is in flight the fully-transparent end is not drawn at all.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_FADES 64

typedef struct {
    int id;
    int vis;            /* debounced (committed) visibility */
    float alpha;        /* 0 hidden .. 1 opaque */
    unsigned fade;      /* 0 = not fading, else 1 (in) / 2 (out) */
    float elapsed;      /* fade progress time this direction */
    int cand;           /* pending raw cull decision */
    float held;         /* time the pending candidate has held */
} Fade;

typedef struct {
    Fade f[MAX_FADES];
    int n;
    float dur;          /* in and out duration, 0 rejected */
    float hold;         /* time a raw toggle must hold before committing */
} Fades;

/* dur must be > 0: zero would be a hard cut, callers can skip the entity. */
static int fades_init(Fades* fs, float dur, float hold) {
    memset(fs, 0, sizeof(*fs));
    if (!(dur > 0.0f) || isnan(dur)) return -1;
    if (!(hold >= 0.0f) || isnan(hold)) return -1;
    fs->dur = dur;
    fs->hold = hold;
    return 0;
}

static int find(const Fades* fs, int id) {
    for (int i = 0; i < fs->n; i++) if (fs->f[i].id == id) return i;
    return -1;
}

/* first sighting: start hidden and idle, so the first frame draws nothing.
 * The first update whose cull decision is visible starts the fade in. */
static int fades_register(Fades* fs, int id) {
    if (fs->dur <= 0.0f) return -1;                 /* init failed or never ran */
    if (find(fs, id) >= 0) return -1;
    if (fs->n >= MAX_FADES) return -1;
    Fade* f = &fs->f[fs->n++];
    f->id = id;
    f->vis = 0;
    f->alpha = 0.0f;
    f->fade = 0;
    f->elapsed = 0.0f;
    f->cand = 0;
    f->held = 0.0f;
    return 0;
}

/* commit a debounced toggle; start from the alpha already on screen */
static void commit(Fades* fs, Fade* f, int vis) {
    f->vis = vis;
    if (vis) {
        f->fade = 1;
        f->elapsed = f->alpha * fs->dur;            /* continue from current alpha */
    } else {
        f->fade = 2;
        f->elapsed = (1.0f - f->alpha) * fs->dur;
    }
    f->held = 0.0f;
    f->cand = vis;
}

/* called once per frame with the entity's raw cull decision; returns committed
 * visibility (0/1), or -1 for an unknown id or a negative dt */
static int fades_update(Fades* fs, int id, int cull_visible, float dt) {
    int i = find(fs, id);
    if (i < 0) return -1;
    if (dt < 0.0f || isnan(dt)) return -1;
    Fade* f = &fs->f[i];

    if (cull_visible != f->vis) {
        if (cull_visible != f->cand) { f->cand = cull_visible; f->held = 0.0f; }
        f->held += dt;
        if (f->held >= fs->hold) commit(fs, f, cull_visible);
    } else {
        f->cand = cull_visible;                     /* agrees again: drop the pending */
        f->held = 0.0f;
    }

    if (f->fade == 1) {
        f->elapsed += dt;
        float t = f->elapsed / fs->dur;
        if (t >= 1.0f) { f->fade = 0; f->alpha = 1.0f; f->elapsed = 0.0f; }
        else if (t < 0.0f) { f->alpha = 0.0f; }
        else f->alpha = t;
    } else if (f->fade == 2) {
        f->elapsed += dt;
        float t = f->elapsed / fs->dur;
        if (t >= 1.0f) { f->fade = 0; f->alpha = 0.0f; f->elapsed = 0.0f; }
        else if (t < 0.0f) { f->alpha = 1.0f; }
        else f->alpha = 1.0f - t;
    }
    return f->vis;
}

/* alpha the renderer should draw at; a hidden entity returns 0 and is skipped */
static float fades_alpha(const Fades* fs, int id) {
    int i = find(fs, id);
    if (i < 0) return 0.0f;
    return fs->f[i].alpha;
}

/* would the hard cut draw this entity right now? (alpha is exactly the answer) */
static int fades_drawn(const Fades* fs, int id) {
    return fades_alpha(fs, id) > 0.0f;
}

static int fades_is_idle(const Fades* fs, int id) {
    int i = find(fs, id);
    if (i < 0) return 1;
    return fs->f[i].fade == 0;
}

/* entities that never committed visible and have fully faded out can be dropped */
static void fades_compact(Fades* fs) {
    int k = 0;
    for (int i = 0; i < fs->n; i++) {
        Fade* f = &fs->f[i];
        if (!f->vis && f->fade == 0 && f->alpha == 0.0f) continue;
        if (k != i) fs->f[k] = fs->f[i];
        k++;
    }
    fs->n = k;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Fades fs;
    check("init_ok", fades_init(&fs, 0.2f, 0.05f) == 0);
    check("zero_dur_rejected", fades_init(&fs, 0.0f, 0.05f) == -1);
    check("neg_dur_rejected", fades_init(&fs, -0.1f, 0.05f) == -1);
    check("neg_hold_rejected", fades_init(&fs, 0.2f, -0.01f) == -1);
    fades_init(&fs, 0.2f, 0.05f);

    check("register_ok", fades_register(&fs, 7) == 0);
    check("register_dup", fades_register(&fs, 7) == -1);
    check("first_frame_hidden", fades_alpha(&fs, 7) == 0.0f && !fades_drawn(&fs, 7));
    check("update_unknown", fades_update(&fs, 999, 1, 0.1f) == -1);
    check("negative_dt_rejected", fades_update(&fs, 7, 1, -0.1f) == -1);

    /* fade in: starts at alpha 0, mid ~0.5, end pinned to 1 */
    fades_update(&fs, 7, 1, 0.0f);
    check("fadein_start", fades_alpha(&fs, 7) == 0.0f);
    fades_update(&fs, 7, 1, 0.1f);
    check("fadein_mid", fabsf(fades_alpha(&fs, 7) - 0.5f) < 1e-5f);
    fades_update(&fs, 7, 1, 0.1f);
    check("fadein_done", fades_alpha(&fs, 7) == 1.0f && fades_is_idle(&fs, 7));

    /* hard-cut parity at the opaque end */
    fades_update(&fs, 7, 1, 1.0f);
    check("steady_opaque", fades_alpha(&fs, 7) == 1.0f && fades_drawn(&fs, 7));

    /* fade out: still opaque on the frame the cull decision flips */
    fades_update(&fs, 7, 0, 0.0f);
    check("fadeout_first_frame_opaque", fades_alpha(&fs, 7) == 1.0f);
    fades_update(&fs, 7, 0, 0.1f);
    check("fadeout_mid", fabsf(fades_alpha(&fs, 7) - 0.5f) < 1e-5f);
    fades_update(&fs, 7, 0, 0.1f);
    check("fadeout_done_hidden", fades_alpha(&fs, 7) == 0.0f && !fades_drawn(&fs, 7));
    check("hidden_is_idle", fades_is_idle(&fs, 7));
    /* residual check: once hidden, later frames without another toggle draw nothing */
    fades_update(&fs, 7, 0, 0.5f);
    fades_update(&fs, 7, 0, 0.5f);
    check("no_residual_after_fadeout", !fades_drawn(&fs, 7) && fades_alpha(&fs, 7) == 0.0f);

    /* re-entering after a completed fade-out starts from 0, never pops */
    Fades fe;
    fades_init(&fe, 0.4f, 0.0f);
    fades_register(&fe, 1);
    for (int i = 0; i < 5; i++) fades_update(&fe, 1, 1, 0.1f);
    check("reenter_full", fades_alpha(&fe, 1) == 1.0f);
    for (int i = 0; i < 5; i++) fades_update(&fe, 1, 0, 0.1f);
    check("reenter_hidden", fades_alpha(&fe, 1) == 0.0f);
    fades_update(&fe, 1, 1, 0.1f);
    check("reenter_ramps_from_zero", fabsf(fades_alpha(&fe, 1) - 0.25f) < 1e-5f);

    /* reversing mid-fade continues from the visible alpha: no pop, no ghost */
    Fades fr;
    fades_init(&fr, 0.4f, 0.0f);
    fades_register(&fr, 3);
    for (int i = 0; i < 2; i++) fades_update(&fr, 3, 1, 0.1f);      /* alpha 0.5 */
    float mid = fades_alpha(&fr, 3);
    fades_update(&fr, 3, 0, 0.1f);                                   /* reverse */
    check("reverse_continuous", fabsf(fades_alpha(&fr, 3) - (mid - 0.25f)) < 1e-5f);

    /* debounce: a flicker shorter than hold must not restart a fade */
    Fades fb;
    fades_init(&fb, 0.4f, 0.1f);
    fades_register(&fb, 2);
    for (int i = 0; i < 12; i++) fades_update(&fb, 2, 1, 0.05f);    /* settle fully in */
    float solid = fades_alpha(&fb, 2);
    check("settled_in", solid == 1.0f && fades_is_idle(&fb, 2));
    fades_update(&fb, 2, 0, 0.05f);                                  /* one-frame flicker */
    fades_update(&fb, 2, 1, 0.05f);
    check("flicker_ignored", fades_alpha(&fb, 2) == solid && fades_is_idle(&fb, 2));

    /* a toggle that holds for hold seconds does commit */
    for (int i = 0; i < 2; i++) fades_update(&fb, 2, 0, 0.05f);
    check("hold_commits", fades_alpha(&fb, 2) < solid);

    /* sustained flicker at 2 frames per toggle never commits: no fade restart */
    Fades fj;
    fades_init(&fj, 0.4f, 0.2f);
    fades_register(&fj, 5);
    for (int i = 0; i < 12; i++) fades_update(&fj, 5, 1, 0.05f);    /* settle visible */
    int flicker_idle = 1;
    for (int i = 0; i < 40; i++) {
        fades_update(&fj, 5, i & 1, 0.05f);
        if (fades_alpha(&fj, 5) != 1.0f || !fades_is_idle(&fj, 5)) flicker_idle = 0;
    }
    check("flicker_never_restarts", flicker_idle);

    /* capacity: MAX_FADES distinct entities, then rejected */
    Fades fcap;
    fades_init(&fcap, 0.2f, 0.0f);
    int all_registered = 1;
    for (int i = 0; i < MAX_FADES; i++) if (fades_register(&fcap, 100 + i) != 0) all_registered = 0;
    check("capacity_ok", all_registered && fades_register(&fcap, 999) == -1);

    /* repeated fades stay bounded and land exactly on the endpoints */
    for (int i = 0; i < 10; i++) fades_update(&fb, 2, 0, 0.05f);
    int bounded = 1;
    for (int i = 0; i < 20; i++) {
        fades_update(&fb, 2, 1, 0.05f);
        float a = fades_alpha(&fb, 2);
        if (a < 0.0f || a > 1.0f) bounded = 0;
    }
    check("alpha_bounded", bounded && fades_alpha(&fb, 2) == 1.0f);

    /* compact drops the fully-gone, keeps the live one */
    Fades fc;
    fades_init(&fc, 0.2f, 0.0f);
    fades_register(&fc, 10);
    fades_register(&fc, 11);
    fades_update(&fc, 10, 1, 0.2f);
    fades_update(&fc, 11, 0, 0.2f);
    check("compact_keeps_live", fc.n == 2);
    fades_compact(&fc);
    check("compact_keeps_active", fc.n == 1 && fc.f[0].id == 10);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
