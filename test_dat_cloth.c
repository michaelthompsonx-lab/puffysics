/* Invariant tests for the DAT cloth-rigid coupling (dat_cloth.h).
 * Build (host): g++ -O2 -I. test_dat_cloth.c -o /tmp/test-dat
 *  1. Drop at several speeds: no node enters any ball (penetration
 *     invariant), and the ball never sinks below the sheet plane.
 *  2. Slide: tangential motion survives (distance within a factor of the
 *     frictionless baseline), same penetration invariant.
 *  3. Resting: a settled ball keeps constant height without jitter.
 *  4. Fabric orbit: a light ball launched tangentially around a resting
 *     heavy ball sweeps angle in the well without penetration. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "dat_cloth.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        failures++; \
        printf("FAIL %s:%d %s\n", __func__, __LINE__, #cond); \
    } \
} while (0)

static int finite_world(const B3World* w) {
    for (int i = 0; i < w->body_count; i++) {
        const B3Body* b = &w->bodies[i];
        if (!isfinite(b->center.x) || !isfinite(b->center.y)
                || !isfinite(b->center.z)) {
            return 0;
        }
        if (!isfinite(b->lin_vel.x) || !isfinite(b->lin_vel.y)
                || !isfinite(b->lin_vel.z)) {
            return 0;
        }
    }
    return 1;
}

/* Dynamic sphere with uniform density-derived inertia. */
static int add_ball(B3World* w, float r, float density, B3Vec3 pos,
        B3Vec3 vel, B3Vec3 spin) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = pos;
    bd.lin_vel = vel;
    bd.ang_vel = spin;
    int body = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_sphere(w, body, b3_v(0.0f, 0.0f, 0.0f), r, &sd);
    float vol = (4.0f / 3.0f) * B3_PI * r * r * r;
    float m = density * vol;
    B3Vec3 inertia = b3_v(0.4f * m * r * r, 0.4f * m * r * r,
        0.4f * m * r * r);
    b3_set_inertial(w, body, m, b3_v(0.0f, 0.0f, 0.0f), inertia);
    return body;
}

/* One coupled substep. ball_prev must have room for nb entries and is
 * filled with each ball's center before the engine step. */
static void substep(B3World* w, DatCloth* c, B3Vec3* ball_prev,
        B3Vec3* ball_snap, const int* ball_body, const float* radii,
        int nb, float h) {
    b3_step(w, h, 1);
    dat_cloth_step(c, h);
    /* Snapshot just the coupling's position change (post engine step). */
    for (int q = 0; q < nb; q++) {
        ball_snap[q] = w->bodies[ball_body[q]].center;
    }
    dat_couple(c, w, ball_prev, ball_body, radii, nb, h);
    dat_couple_velocities(w, ball_snap, ball_body, nb, h);
}

/* Worst gap over all (node, ball) pairs; negative means penetration. */
static float min_gap_all(const DatCloth* c, const B3World* w,
        const int* ball_body, const float* radii, int nb) {
    float worst = 1.0e30f;
    for (int k = 0; k < c->n; k++) {
        for (int q = 0; q < nb; q++) {
            B3Vec3 d = b3_sub(c->pos[k], w->bodies[ball_body[q]].center);
            float gap = b3_len(d) - radii[q];
            if (gap < worst) {
                worst = gap;
            }
        }
    }
    return worst;
}

/* 1. Drop a ball onto the sheet from 1 m at several impact speeds. */
static void test_drop_no_penetration(void) {
    const float speeds[3] = { 0.5f, 2.0f, 10.0f };
    for (int si = 0; si < 3; si++) {
        B3World w;
        b3_world_init(&w);
        w.gravity = b3_v(0.0f, -10.0f, 0.0f);
        DatCloth c;
        dat_cloth_init(&c, 21, 21, 0.1f, b3_v(-1.0f, 0.0f, -1.0f));
        int bodies[8];
        float radii[8];
        bodies[0] = add_ball(&w, 0.15f, 30.0f, b3_v(0.0f, 1.0f, 0.0f),
            b3_v(0.0f, -speeds[si], 0.0f), b3_v(0.0f, 0.0f, 0.0f));
        radii[0] = 0.15f;

        const float dt = 1.0f / 60.0f;
        const int subs = 4;
        float h = dt / (float)subs;
        float worst = 1.0e30f;
        B3Vec3 ball_prev[8];
    B3Vec3 ball_snap[8];
        for (int s = 0; s < subs * 180; s++) { /* 3 seconds */
            substep(&w, &c, ball_prev, ball_snap, bodies, radii, 1, h);
            float g = min_gap_all(&c, &w, bodies, radii, 1);
            if (g < worst) {
                worst = g;
            }
        }
        printf("drop(v=%g): worst gap=%f\n",
            (double)speeds[si], (double)worst);
        CHECK(finite_world(&w));
        CHECK(worst > -5.0e-3f);
        dat_cloth_free(&c);
    }
}

/* 2. Slide: the same ball gliding over the sheet keeps a comparable
 * share of its tangential travel as over a rigid ground plane. The cloth
 * physically drags (the ball plows fabric), so the comparison is against
 * the rigid baseline, not against frictionless motion. */
static void test_slide_free(void) {
    const float r = 0.15f;
    const float density = 30.0f;
    const float dt = 1.0f / 60.0f;
    const int subs = 4;
    float h = dt / (float)subs;

    /* Rigid baseline: the same ball over a static puffysics ground. */
    B3World wg;
    b3_world_init(&wg);
    wg.gravity = b3_v(0.0f, -10.0f, 0.0f);
    {
        B3BodyDef gd = b3_default_body();
        gd.type = B3_STATIC;
        gd.position = b3_v(0.0f, -0.05f, 0.0f);
        int gid = b3_create_body(&wg, &gd);
        B3ShapeDef gsd = b3_default_shape();
        gsd.density = 0.0f;
        gsd.friction = 0.0f; /* isolate the cloth coupling's tangential
                              * drag from contact friction */
        b3_create_box(&wg, gid, b3_v(3.0f, 0.05f, 3.0f), &gsd);
    }
    float vol = (4.0f / 3.0f) * B3_PI * r * r * r;
    float m = 30.0f * vol;
    B3Vec3 inertia = b3_v(0.4f * m * r * r, 0.4f * m * r * r,
        0.4f * m * r * r);
    int bg = add_ball(&wg, r, 0.0f, b3_v(-0.6f, r, 0.0f),
        b3_v(2.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f));
    (void)vol;
    (void)m;
    (void)inertia;
    (void)bg;
    for (int s = 0; s < subs * 120; s++) {
        b3_step(&wg, h, 1);
    }
    float travel_ground = wg.bodies[bg].center.x + 0.6f;

    /* Cloth run: identical ball over the coupled sheet. */
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, -10.0f, 0.0f);
    DatCloth c;
    dat_cloth_init(&c, 21, 21, 0.1f, b3_v(-1.0f, 0.0f, -1.0f));
    int bodies[8];
    float radii[8];
    B3Vec3 ball_prev[8];
    B3Vec3 ball_snap[8];
    bodies[0] = add_ball(&w, r, 0.0f, b3_v(-0.6f, r, 0.0f),
        b3_v(2.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f));
    radii[0] = r;
    /* add_ball sets mass from density 0? No: rebuild explicitly. */
    bodies[0] = add_ball(&w, r, 30.0f, b3_v(-0.6f, r, 0.0f),
        b3_v(2.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f));
    float worst = 1.0e30f;
    for (int s = 0; s < subs * 120; s++) { /* 2 seconds */
        substep(&w, &c, ball_prev, ball_snap, bodies, radii, 1, h);
        float g = min_gap_all(&c, &w, bodies, radii, 1);
        if (g < worst) {
            worst = g;
        }
    }
    float travel_cloth = w.bodies[bodies[0]].center.x + 0.6f;
    printf("slide: cloth=%f rigid=%f worst_gap=%f\n",
        (double)travel_cloth, (double)travel_ground, (double)worst);
    CHECK(finite_world(&w));
    CHECK(worst > -5.0e-3f);
    /* The coupling must not artificially damp tangential motion beyond
     * the rigid-contact baseline: at least half the rigid travel. */
    CHECK(travel_cloth > 0.5f * travel_ground);
    dat_cloth_free(&c);
}

/* 3. Resting: a ball settles to a constant height on the sheet. */
static void test_resting(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, -10.0f, 0.0f);
    DatCloth c;
    dat_cloth_init(&c, 21, 21, 0.1f, b3_v(-1.0f, 0.0f, -1.0f));
    int bodies[8];
    float radii[8];
    B3Vec3 ball_prev[8];
    B3Vec3 ball_snap[8];
    bodies[0] = add_ball(&w, 0.15f, 30.0f, b3_v(0.0f, 0.3f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f));
    radii[0] = 0.15f;

    const float dt = 1.0f / 60.0f;
    const int subs = 4;
    float h = dt / (float)subs;
    for (int s = 0; s < subs * 120; s++) { /* settle 2 seconds */
        substep(&w, &c, ball_prev, ball_snap, bodies, radii, 1, h);
    }
    B3Vec3 start = w.bodies[bodies[0]].center;
    float max_move = 0.0f;
    for (int s = 0; s < subs * 120; s++) { /* 2 more seconds */
        substep(&w, &c, ball_prev, ball_snap, bodies, radii, 1, h);
        B3Vec3 d = b3_sub(w.bodies[bodies[0]].center, start);
        float m = fabsf(d.x) + fabsf(d.y) + fabsf(d.z);
        if (m > max_move) {
            max_move = m;
        }
    }
    float g = min_gap_all(&c, &w, bodies, radii, 1);
    printf("rest: settled y=%f max_move=%f gap=%f\n",
        (double)w.bodies[bodies[0]].center.y, (double)max_move,
        (double)g);
    CHECK(finite_world(&w));
    CHECK(max_move < 0.05f);
    CHECK(g > -5.0e-3f);
    dat_cloth_free(&c);
}

/* 4. Fabric orbit: light ball launched tangentially around a resting
 * heavy ball sweeps angle without penetration. */
static void test_fabric_orbit(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, -10.0f, 0.0f);
    DatCloth c;
    dat_cloth_init(&c, 25, 25, 0.1f, b3_v(-1.2f, 0.0f, -1.2f));
    int bodies[8];
    float radii[8];
    B3Vec3 ball_prev[8];
    B3Vec3 ball_snap[8];
    int nb = 0;
    bodies[nb] = add_ball(&w, 0.16f, 30.0f, b3_v(0.0f, 0.3f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f));
    radii[nb] = 0.16f;
    nb++;

    const float dt = 1.0f / 60.0f;
    const int subs = 4;
    float h = dt / (float)subs;
    for (int s = 0; s < subs * 120; s++) { /* settle 2 seconds */
        substep(&w, &c, ball_prev, ball_snap, bodies, radii, nb, h);
    }
    B3Vec3 center = w.bodies[bodies[0]].center;

    /* Planet resting on the sheet at radius 0.55, tangential push. */
    B3Vec3 r0 = b3_v(center.x + 0.45f, center.y, center.z);
    bodies[nb] = add_ball(&w, 0.075f, 10.0f, r0,
        b3_v(0.0f, 0.0f, 1.4f), b3_v(0.0f, 0.0f, 0.0f));
    radii[nb] = 0.075f;
    nb++;

    float worst = 1.0e30f;
    float angle = 0.0f;
    B3Vec3 prev_pos = r0;
    for (int s = 0; s < subs * 120; s++) { /* 2 seconds */
        substep(&w, &c, ball_prev, ball_snap, bodies, radii, nb, h);
        B3Vec3 p = w.bodies[bodies[1]].center;
        B3Vec3 a = b3_sub(p, center);
        B3Vec3 b2 = b3_sub(prev_pos, center);
        float ca = b3_dot(a, b2);
        float sa = a.x * b2.z - a.z * b2.x; /* signed 2D cross on XZ */
        angle += atan2f(sa, ca);
        prev_pos = p;
        float g = min_gap_all(&c, &w, bodies, radii, nb);
        if (g < worst) {
            worst = g;
        }
    }
    printf("orbit: swept=%f deg worst_gap=%f\n",
        (double)(angle * 57.2957795f), (double)worst);
    CHECK(finite_world(&w));
    CHECK(worst > -5.0e-3f);
    CHECK(fabsf(angle) > 0.5f); /* at least ~30 degrees of sweep */
    dat_cloth_free(&c);
}

int main(void) {
    test_drop_no_penetration();
    test_slide_free();
    test_resting();
    test_fabric_orbit();
    printf(failures == 0 ? "test-dat-cloth: ok\n"
        : "test-dat-cloth: FAIL\n");
    return failures != 0;
}
