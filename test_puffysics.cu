#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "puffysics.cuh"

#define CHECK(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        exit(1); \
    } \
} while (0)

#define CHECK_NEAR(a, e, tol) do { \
    float _a = (float)(a); \
    float _e = (float)(e); \
    if (fabsf(_a - _e) > (tol)) { \
        fprintf(stderr, "FAIL %s:%d: %s=%g expected %g\n", \
            __FILE__, __LINE__, #a, _a, _e); \
        exit(1); \
    } \
} while (0)

static void make_hello(B3World* w, float y0) {
    b3_world_init(w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, -1.0f, 0.0f);
    int ground = b3_create_body(w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.3f;
    b3_create_box(w, ground, b3_v(50.0f, 1.0f, 50.0f), &sd);
    b3_finalize_mass(w, ground);

    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, y0, 0.0f);
    int box = b3_create_body(w, &bd);
    sd.density = 1.0f;
    sd.friction = 0.3f;
    b3_create_box(w, box, b3_v(1.0f, 1.0f, 1.0f), &sd);
    b3_finalize_mass(w, box);
}

static void test_free_fall(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 10.0f, 0.0f);
    int id = b3_create_body(&w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1.0f;
    b3_create_sphere(&w, id, b3_v(0.0f, 0.0f, 0.0f), 0.5f, &sd);
    b3_finalize_mass(&w, id);

    float dt = 1.0f / 60.0f;
    int steps = 30;
    int subs = 4;
    for (int i = 0; i < steps; i++) {
        b3_step(&w, dt, subs);
    }
    float h = dt / (float)subs;
    int n = steps * subs;
    float g = -10.0f;
    float expected = 10.0f
        + h * h * g * (float)n * (float)(n + 1) * 0.5f;
    CHECK_NEAR(w.bodies[id].position.y, expected, 1.0e-3f);
    CHECK_NEAR(w.bodies[id].lin_vel.y, n * h * g, 1.0e-3f);
    printf("  free_fall y=%.4f expected=%.4f\n",
        w.bodies[id].position.y, expected);
}

static void test_rest_on_ground(void) {
    B3World w;
    make_hello(&w, 4.0f);
    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 180; i++) {
        b3_step(&w, dt, 4);
    }
    float y = w.bodies[1].position.y;
    printf("  rest y=%.4f vy=%.4f contacts=%d\n",
        y, w.bodies[1].lin_vel.y, w.contact_count);
    CHECK(y > 0.90f && y < 1.20f);
    CHECK(fabsf(w.bodies[1].lin_vel.y) < 0.25f);
}

static void test_sphere_bounce(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, -1.0f, 0.0f);
    int ground = b3_create_body(&w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.restitution = 0.8f;
    b3_create_box(&w, ground, b3_v(20.0f, 1.0f, 20.0f), &sd);
    b3_finalize_mass(&w, ground);

    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 3.0f, 0.0f);
    int sph = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    sd.restitution = 0.8f;
    sd.friction = 0.0f;
    b3_create_sphere(&w, sph, b3_v(0.0f, 0.0f, 0.0f), 0.5f, &sd);
    b3_finalize_mass(&w, sph);

    float dt = 1.0f / 60.0f;
    float min_y = 3.0f;
    float max_y_after = -100.0f;
    int bounced = 0;
    for (int i = 0; i < 180; i++) {
        b3_step(&w, dt, 4);
        float y = w.bodies[sph].position.y;
        if (y < min_y) {
            min_y = y;
        }
        if (min_y < 1.0f && y > max_y_after) {
            max_y_after = y;
            bounced = 1;
        }
    }
    printf("  bounce min_y=%.4f peak=%.4f\n", min_y, max_y_after);
    CHECK(bounced);
    CHECK(max_y_after > 1.4f);
}

static void test_kinematic_lift(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef kd = b3_default_body();
    kd.type = B3_KINEMATIC;
    kd.position = b3_v(0.0f, 0.0f, 0.0f);
    kd.lin_vel = b3_v(0.0f, 1.0f, 0.0f);
    int plat = b3_create_body(&w, &kd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.8f;
    b3_create_box(&w, plat, b3_v(2.0f, 0.1f, 2.0f), &sd);
    b3_finalize_mass(&w, plat);

    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 0.7f, 0.0f);
    int box = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    b3_create_box(&w, box, b3_v(0.4f, 0.4f, 0.4f), &sd);
    b3_finalize_mass(&w, box);

    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; i++) {
        b3_step(&w, dt, 4);
    }
    printf("  kinematic plat_y=%.4f box_y=%.4f\n",
        w.bodies[plat].position.y, w.bodies[box].position.y);
    CHECK(w.bodies[plat].position.y > 0.9f);
    CHECK(w.bodies[box].position.y > 1.2f);
}

static void test_sphere_sphere(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, 0.0f, 0.0f);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1.0f;
    sd.restitution = 1.0f;
    sd.friction = 0.0f;

    B3BodyDef a = b3_default_body();
    a.type = B3_DYNAMIC;
    a.position = b3_v(-1.5f, 0.0f, 0.0f);
    a.lin_vel = b3_v(2.0f, 0.0f, 0.0f);
    int ia = b3_create_body(&w, &a);
    b3_create_sphere(&w, ia, b3_v(0.0f, 0.0f, 0.0f), 0.5f, &sd);
    b3_finalize_mass(&w, ia);

    B3BodyDef b = b3_default_body();
    b.type = B3_DYNAMIC;
    b.position = b3_v(1.5f, 0.0f, 0.0f);
    int ib = b3_create_body(&w, &b);
    b3_create_sphere(&w, ib, b3_v(0.0f, 0.0f, 0.0f), 0.5f, &sd);
    b3_finalize_mass(&w, ib);

    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 90; i++) {
        b3_step(&w, dt, 4);
    }
    printf("  sphere-sphere xa=%.3f xb=%.3f va=%.3f vb=%.3f\n",
        w.bodies[ia].position.x, w.bodies[ib].position.x,
        w.bodies[ia].lin_vel.x, w.bodies[ib].lin_vel.x);
    CHECK(w.bodies[ia].position.x < w.bodies[ib].position.x);
    CHECK(w.bodies[ib].lin_vel.x > 0.3f);
}


static void test_weld_rigid(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, 0.0f, 0.0f);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1.0f;
    sd.friction = 0.2f;
    B3BodyDef a = b3_default_body();
    a.type = B3_DYNAMIC;
    a.position = b3_v(-0.5f, 0.0f, 0.0f);
    int ia = b3_create_body(&w, &a);
    b3_create_box(&w, ia, b3_v(0.4f, 0.2f, 0.2f), &sd);
    b3_finalize_mass(&w, ia);
    B3BodyDef b = b3_default_body();
    b.type = B3_DYNAMIC;
    b.position = b3_v(0.5f, 0.0f, 0.0f);
    int ib = b3_create_body(&w, &b);
    b3_create_box(&w, ib, b3_v(0.4f, 0.2f, 0.2f), &sd);
    b3_finalize_mass(&w, ib);
    b3_create_weld(&w, ia, ib, b3_v(0.5f, 0.0f, 0.0f),
        b3_v(-0.5f, 0.0f, 0.0f));
    b3_apply_force(&w, ia, b3_v(0.0f, 8.0f, 0.0f));
    b3_apply_force(&w, ib, b3_v(0.0f, 8.0f, 0.0f));
    for (int i = 0; i < 120; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    B3Vec3 d = b3_sub(w.bodies[ib].position, w.bodies[ia].position);
    CHECK_NEAR(b3_len(d), 1.0f, 0.04f);
    CHECK_NEAR(d.x, 1.0f, 0.06f);
    CHECK_NEAR(d.y, 0.0f, 0.06f);
    CHECK_NEAR(d.z, 0.0f, 0.04f);
    CHECK(w.bodies[ia].position.y > 0.4f);
    float qdot = b3_qdot(w.bodies[ia].rotation, w.bodies[ib].rotation);
    CHECK(fabsf(qdot) > 0.98f);
    printf("  weld d=(%.3f %.3f %.3f) y=%.3f qdot=%.4f\n",
        d.x, d.y, d.z, w.bodies[ia].position.y, qdot);
}

static void test_revolute_pendulum(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, 2.0f, 0.0f);
    int pivot = b3_create_body(&w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_sphere(&w, pivot, b3_v(0.0f, 0.0f, 0.0f), 0.05f, &sd);
    b3_finalize_mass(&w, pivot);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(1.0f, 2.0f, 0.0f);
    int bob = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    b3_create_box(&w, bob, b3_v(0.15f, 0.15f, 0.15f), &sd);
    b3_finalize_mass(&w, bob);
    int j = b3_create_revolute(&w, pivot, bob,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-1.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 1.0f));
    (void)j;
    for (int i = 0; i < 90; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    B3Vec3 p = w.bodies[bob].position;
    float dist = b3_len(b3_sub(p, w.bodies[pivot].position));
    CHECK_NEAR(dist, 1.0f, 0.05f);
    CHECK(p.y < 1.85f);
    CHECK(fabsf(p.z) < 0.05f);
    printf("  pendulum y=%.4f dist=%.4f angle=%.4f\n",
        p.y, dist, b3_joint_angle(&w, j));
}

static void test_revolute_motor(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, 0.0f, 0.0f);
    B3BodyDef gd = b3_default_body();
    int pivot = b3_create_body(&w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_sphere(&w, pivot, b3_v(0.0f, 0.0f, 0.0f), 0.05f, &sd);
    b3_finalize_mass(&w, pivot);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.8f, 0.0f, 0.0f);
    int arm = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    b3_create_box(&w, arm, b3_v(0.35f, 0.08f, 0.08f), &sd);
    b3_finalize_mass(&w, arm);
    int j = b3_create_revolute(&w, pivot, arm,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-0.8f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 1.0f));
    b3_joint_enable_motor(&w, j, 1);
    b3_joint_set_motor(&w, j, 1.5f, 40.0f);
    for (int i = 0; i < 60; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    float ang = b3_joint_angle(&w, j);
    CHECK_NEAR(ang, 1.5f, 0.15f);
    printf("  motor angle=%.4f expected=1.5\n", ang);
}

static void test_revolute_limit(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, 0.0f, 0.0f);
    B3BodyDef gd = b3_default_body();
    int pivot = b3_create_body(&w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_sphere(&w, pivot, b3_v(0.0f, 0.0f, 0.0f), 0.05f, &sd);
    b3_finalize_mass(&w, pivot);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.8f, 0.0f, 0.0f);
    int arm = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    b3_create_box(&w, arm, b3_v(0.35f, 0.08f, 0.08f), &sd);
    b3_finalize_mass(&w, arm);
    int j = b3_create_revolute(&w, pivot, arm,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-0.8f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 1.0f));
    b3_joint_enable_limit(&w, j, 1);
    b3_joint_set_limits(&w, j, -0.25f, 0.25f);
    b3_joint_enable_motor(&w, j, 1);
    b3_joint_set_motor(&w, j, 4.0f, 50.0f);
    for (int i = 0; i < 120; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    float ang = b3_joint_angle(&w, j);
    CHECK(ang > 0.18f && ang < 0.32f);
    printf("  limit angle=%.4f (cap 0.25)\n", ang);
}

static void test_two_link_leg(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef hd = b3_default_body();
    hd.position = b3_v(0.0f, 1.4f, 0.0f);
    int hip = b3_create_body(&w, &hd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_sphere(&w, hip, b3_v(0.0f, 0.0f, 0.0f), 0.04f, &sd);
    b3_finalize_mass(&w, hip);
    sd.density = 1.0f;
    sd.friction = 0.6f;
    B3BodyDef td = b3_default_body();
    td.type = B3_DYNAMIC;
    td.position = b3_v(0.0f, 1.18f, 0.0f);
    int thigh = b3_create_body(&w, &td);
    b3_create_capsule(&w, thigh, 0.18f, 0.06f, &sd);
    b3_finalize_mass(&w, thigh);
    B3BodyDef sdn = b3_default_body();
    sdn.type = B3_DYNAMIC;
    sdn.position = b3_v(0.0f, 0.76f, 0.0f);
    int shin = b3_create_body(&w, &sdn);
    b3_create_capsule(&w, shin, 0.16f, 0.05f, &sd);
    b3_finalize_mass(&w, shin);
    int jh = b3_create_revolute(&w, hip, thigh,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.22f, 0.0f),
        b3_v(1.0f, 0.0f, 0.0f));
    int jk = b3_create_revolute(&w, thigh, shin,
        b3_v(0.0f, -0.22f, 0.0f), b3_v(0.0f, 0.20f, 0.0f),
        b3_v(1.0f, 0.0f, 0.0f));
    b3_joint_enable_limit(&w, jk, 1);
    b3_joint_set_limits(&w, jk, 0.0f, 2.0f);
    b3_joint_enable_spring(&w, jh, 1);
    b3_joint_set_spring(&w, jh, 0.4f, 8.0f, 0.7f);
    for (int i = 0; i < 180; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    float hip_d = b3_len(b3_sub(w.bodies[thigh].position,
        w.bodies[hip].position));
    float knee_d = b3_len(b3_sub(w.bodies[shin].position,
        w.bodies[thigh].position));
    CHECK_NEAR(hip_d, 0.22f, 0.03f);
    CHECK_NEAR(knee_d, 0.42f, 0.05f);
    CHECK(w.bodies[shin].position.y < w.bodies[thigh].position.y + 0.05f);
    float kang = b3_joint_angle(&w, jk);
    CHECK(kang > -0.15f && kang < 2.1f);
    printf("  leg hip_d=%.3f knee_d=%.3f shin_y=%.3f knee=%.3f\n",
        hip_d, knee_d, w.bodies[shin].position.y, kang);
}

static void test_rolling_spin(void) {
    B3World off, on;
    b3_world_init(&off);
    b3_world_init(&on);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, -0.05f, 0.0f);
    int g0 = b3_create_body(&off, &gd);
    int g1 = b3_create_body(&on, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.0f;
    sd.rolling = 0.0f;
    b3_create_box(&off, g0, b3_v(4.0f, 0.05f, 4.0f), &sd);
    b3_finalize_mass(&off, g0);
    sd.rolling = 0.4f;
    b3_create_box(&on, g1, b3_v(4.0f, 0.05f, 4.0f), &sd);
    b3_finalize_mass(&on, g1);

    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 0.2f, 0.0f);
    int s0 = b3_create_body(&off, &bd);
    int s1 = b3_create_body(&on, &bd);
    sd.density = 1000.0f;
    sd.friction = 0.0f;
    sd.rolling = 0.0f;
    b3_create_sphere(&off, s0, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(&off, s0);
    sd.rolling = 0.4f;
    b3_create_sphere(&on, s1, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(&on, s1);

    for (int i = 0; i < 40; i++) {
        b3_step(&off, 1.0f / 60.0f, 4);
        b3_step(&on, 1.0f / 60.0f, 4);
    }
    off.bodies[s0].ang_vel = b3_v(4.0f, 0.0f, 0.0f);
    on.bodies[s1].ang_vel = b3_v(4.0f, 0.0f, 0.0f);
    off.bodies[s0].lin_vel = b3_v(0.0f, 0.0f, 0.0f);
    on.bodies[s1].lin_vel = b3_v(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 60; i++) {
        b3_step(&off, 1.0f / 60.0f, 4);
        b3_step(&on, 1.0f / 60.0f, 4);
        CHECK(isfinite(off.bodies[s0].ang_vel.x));
        CHECK(isfinite(on.bodies[s1].ang_vel.x));
    }
    float wx0 = off.bodies[s0].ang_vel.x;
    float wx1 = on.bodies[s1].ang_vel.x;
    printf("  rolling grain wx off=%g on=%g x=%g\n",
        wx0, wx1, on.bodies[s1].center.x);
    CHECK(fabsf(wx0) > 2.0f);
    CHECK(fabsf(wx1) < 0.5f * fabsf(wx0));
    CHECK(fabsf(on.bodies[s1].center.x) < 0.15f);
}


#ifdef __CUDACC__

static void test_gpu_batch(void) {
    int devices = 0;
    cudaGetDeviceCount(&devices);
    if (devices <= 0) {
        printf("  gpu_batch skipped (no CUDA device)\n");
        return;
    }
    const int n = 256;
    B3World* host = (B3World*)calloc(n, sizeof(B3World));
    for (int i = 0; i < n; i++) {
        make_hello(&host[i], 3.0f + 0.01f * (float)i);
    }
    B3World* dev = NULL;
    CHECK(cudaMalloc((void**)&dev, n * sizeof(B3World)) == cudaSuccess);
    CHECK(cudaMemcpy(dev, host, n * sizeof(B3World),
        cudaMemcpyHostToDevice) == cudaSuccess);
    int threads = 64;
    int blocks = (n + threads - 1) / threads;
    for (int s = 0; s < 120; s++) {
        b3_step_kernel<<<blocks, threads>>>(dev, n, 1.0f / 60.0f, 4);
    }
    CHECK(cudaDeviceSynchronize() == cudaSuccess);
    CHECK(cudaMemcpy(host, dev, n * sizeof(B3World),
        cudaMemcpyDeviceToHost) == cudaSuccess);
    for (int i = 0; i < n; i++) {
        float y = host[i].bodies[1].position.y;
        if (y < 0.85f || y > 1.35f) {
            fprintf(stderr, "gpu env %d y=%g\n", i, y);
            exit(1);
        }
    }
    printf("  gpu_batch %d worlds rest y0=%.4f y255=%.4f\n",
        n, host[0].bodies[1].position.y, host[n - 1].bodies[1].position.y);
    cudaFree(dev);
    free(host);
}
#endif

int main(void) {
    printf("puffysics cuda tests\n");
    test_free_fall();
    test_rest_on_ground();
    test_sphere_bounce();
    test_kinematic_lift();
    test_sphere_sphere();
    test_weld_rigid();
    test_revolute_pendulum();
    test_revolute_motor();
    test_revolute_limit();
    test_two_link_leg();
    test_rolling_spin();
#ifdef __CUDACC__
    test_gpu_batch();
#endif
    printf("all passed\n");
    return 0;
}
