#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "puffysics.cuh"
#include "b3_loose.cuh"

#define CHECK(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        exit(1); \
    } \
} while (0)

static void test_rest(void) {
    B3Loose s;
    CHECK(b3_loose_init(&s, 8, 8, 32));
    CHECK(b3_loose_add_ground(&s) == 0);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 0.04f, 0.0f);
    int id = b3_loose_add_body(&s, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 400.0f;
    sd.friction = 0.5f;
    sd.category = MJCF_COL_CUBE;
    sd.mask = MJCF_COL_GROUND | MJCF_COL_CUBE;
    b3_loose_add_shape(&s, id, B3_BOX, b3_v(0, 0, 0), b3_q_id(), 0.0f,
        b3_v(0.015f, 0.015f, 0.015f), &sd);
    b3_loose_finalize_mass(&s, id);
    for (int i = 0; i < 120; i++) {
        b3_loose_step(&s, 0.02f, 4);
    }
    printf("  rest y=%.4f vy=%.4f contacts=%d overflow=%d\n",
        s.bodies[id].position.y, s.bodies[id].lin_vel.y,
        s.n_contacts, s.overflow);
    CHECK(s.overflow == 0);
    CHECK(s.bodies[id].position.y > 0.012f && s.bodies[id].position.y < 0.025f);
    CHECK(fabsf(s.bodies[id].lin_vel.y) < 0.15f);
    b3_loose_free(&s);
}

static void test_spin(void) {
    B3Loose s;
    CHECK(b3_loose_init(&s, 4, 4, 16));
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 1.0f, 0.0f);
    int id = b3_loose_add_body(&s, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 400.0f;
    sd.category = MJCF_COL_CUBE;
    sd.mask = MJCF_COL_CUBE;
    b3_loose_add_shape(&s, id, B3_BOX, b3_v(0, 0, 0), b3_q_id(), 0.0f,
        b3_v(0.02f, 0.02f, 0.02f), &sd);
    b3_loose_finalize_mass(&s, id);
    s.gravity = b3_v(0, 0, 0);
    /* Off-center impulse: torque r × P with r = (0.02, 0, 0). */
    B3Vec3 r = b3_v(0.02f, 0.0f, 0.0f);
    B3Vec3 P = b3_v(0.0f, 0.0f, 0.02f);
    s.bodies[id].lin_vel = b3_mul(P, s.bodies[id].inv_mass);
    s.bodies[id].ang_vel = b3_mv(s.bodies[id].inv_i_world, b3_cross(r, P));
    float w0 = b3_len(s.bodies[id].ang_vel);
    CHECK(w0 > 0.1f);
    B3Quat q0 = s.bodies[id].rotation;
    for (int i = 0; i < 30; i++) {
        b3_loose_step(&s, 0.02f, 4);
    }
    float qdot = fabsf(b3_qdot(q0, s.bodies[id].rotation));
    printf("  spin |w0|=%.3f qdot=%.4f\n", w0, qdot);
    CHECK(qdot < 0.999f);
    CHECK(isfinite(s.bodies[id].rotation.s));
    b3_loose_free(&s);
}

static void test_wall(void) {
    B3Loose s;
    CHECK(b3_loose_init(&s, 64, 64, 512));
    CHECK(b3_loose_add_ground(&s) == 0);
    int n = b3_loose_spawn_grid(&s, 1, 5, 5, 0.03f, 0.0005f, 250.0f,
        b3_v(0.35f, 0.0f, 0.0f));
    CHECK(n == 25);
    CHECK(s.overflow == 0);
    float ymax0 = 0.0f;
    for (int b = 1; b < s.n_bodies; b++) {
        if (s.bodies[b].position.y > ymax0) ymax0 = s.bodies[b].position.y;
    }
    CHECK(ymax0 > 0.12f);
    for (int i = 0; i < 10; i++) {
        b3_loose_step(&s, 0.02f, 4);
        CHECK(s.overflow == 0);
    }
    float ymax = 0.0f;
    float ymin = 1.0f;
    float xmax = 0.0f;
    float ke = 0.0f;
    for (int b = 1; b < s.n_bodies; b++) {
        CHECK(isfinite(s.bodies[b].position.y));
        CHECK(s.bodies[b].position.y > -0.05f);
        CHECK(s.bodies[b].position.y < 0.5f);
        if (s.bodies[b].position.y > ymax) ymax = s.bodies[b].position.y;
        if (s.bodies[b].position.y < ymin) ymin = s.bodies[b].position.y;
        float dx = fabsf(s.bodies[b].position.x - 0.35f);
        if (dx > xmax) xmax = dx;
        B3Vec3 v = s.bodies[b].lin_vel;
        ke += v.x * v.x + v.y * v.y + v.z * v.z;
    }
    printf("  wall n=%d contacts=%d y0=%.3f y=[%.3f, %.3f] |dx|=%.3f ke=%.4f\n",
        n, s.n_contacts, ymax0, ymin, ymax, xmax, ke);
    CHECK(ymin > 0.005f);
    CHECK(ymax > 0.08f);
    b3_loose_free(&s);
}

static void test_wake_chain(void) {
    B3Loose s;
    CHECK(b3_loose_init(&s, 16, 16, 64));
    CHECK(b3_loose_add_ground(&s) == 0);
    int n = b3_loose_spawn_grid(&s, 1, 3, 1, 0.03f, 0.0f, 250.0f,
        b3_v(0.0f, 0.0f, 0.0f));
    CHECK(n == 3);
    b3_loose_bind_grid(&s, 1, 3, 1);
    b3_loose_freeze(&s);
    CHECK(s.bodies[1].type == B3_STATIC);
    CHECK(s.bodies[3].type == B3_STATIC);
    s.bodies[1].type = B3_DYNAMIC;
    s.bodies[1].flags |= B3_FLAG_DYNAMIC;
    b3_loose_wake_flood(&s);
    printf("  wake chain types=%d %d %d hops=%d\n",
        s.bodies[1].type, s.bodies[2].type, s.bodies[3].type, s.nb_hops);
    CHECK(s.bodies[2].type == B3_DYNAMIC);
    CHECK(s.bodies[3].type == B3_DYNAMIC);
    b3_loose_free(&s);
}


static void test_coupled_frozen_robot_ground(void) {
    B3World w;
    B3Loose cubes;
    B3Contact xc[8];
    int nx;
    b3_world_init(&w);
    B3BodyDef bd = b3_default_body();
    int ground = b3_create_body(&w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.friction = 0.5f;
    b3_create_box(&w, ground, b3_v(2.0f, 0.1f, 2.0f), &sd);
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 0.25f, 0.0f);
    int ball = b3_create_body(&w, &bd);
    sd.density = 400.0f;
    b3_create_sphere(&w, ball, b3_v(0.0f, 0.0f, 0.0f), 0.1f, &sd);
    b3_finalize_mass(&w, ball);
    CHECK(b3_loose_init(&cubes, 8, 8, 32));
    CHECK(b3_loose_spawn_grid(&cubes, 1, 1, 1, 0.03f, 0.0f, 250.0f,
        b3_v(10.0f, 0.0f, 0.0f)) == 1);
    b3_loose_freeze(&cubes);
    CHECK(!b3_loose_any_dynamic(&cubes));
    for (int i = 0; i < 80; i++) {
        nx = -1;
        b3_loose_coupled_step(&w, &cubes, xc, 8, &nx, 0.02f, 4);
        CHECK(nx == 0);
    }
    printf("  coupled frozen ball y=%.4f vy=%.4f\n",
        w.bodies[ball].position.y, w.bodies[ball].lin_vel.y);
    CHECK(w.bodies[ball].position.y > 0.12f && w.bodies[ball].position.y < 0.28f);
    CHECK(fabsf(w.bodies[ball].lin_vel.y) < 0.3f);
    b3_loose_free(&cubes);
}

int main(void) {
    printf("b3_loose tests\n");
    test_rest();
    test_spin();
    test_wall();
    test_wake_chain();
    test_coupled_frozen_robot_ground();
    printf("all passed\n");
    return 0;
}
