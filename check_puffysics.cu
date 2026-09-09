#define B3_MAX_BODIES 48
#define B3_MAX_SHAPES 48
#define B3_MAX_CONTACTS 256
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

static double now_s(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_nsec * 1e-9 + (double)t.tv_sec;
}

static void dump_body(const char* scene, int step, int i, const B3Body* b) {
    printf("T %s %d %d %.6f %.6f %.6f %.6f %.6f %.6f\n",
        scene, step, i, b->position.x, b->position.y, b->position.z,
        b->lin_vel.x, b->lin_vel.y, b->lin_vel.z);
}

static void make_ground(B3World* w, B3Vec3 pos, B3Vec3 half,
        float friction, float rest) {
    B3BodyDef gd = b3_default_body();
    gd.position = pos;
    int ground = b3_create_body(w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = friction;
    sd.restitution = rest;
    b3_create_box(w, ground, half, &sd);
    b3_finalize_mass(w, ground);
}

static void test_invariants(void) {
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
    float mass = 1.0f / w.bodies[id].inv_mass;
    float expect_m = (4.0f / 3.0f) * B3_PI * 0.125f;
    CHECK_NEAR(mass, expect_m, 1.0e-4f);

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
    CHECK(fabsf(w.bodies[id].position.x) < 1.0e-5f);
    printf("  mass_sphere=%.6f free_fall_y=%.4f\n", mass, expected);

    b3_world_init(&w);
    make_ground(&w, b3_v(0.0f, -1.0f, 0.0f), b3_v(50.0f, 1.0f, 50.0f),
        0.3f, 0.0f);
    bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 4.0f, 0.0f);
    int box = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    sd.friction = 0.3f;
    sd.restitution = 0.0f;
    b3_create_box(&w, box, b3_v(1.0f, 1.0f, 1.0f), &sd);
    b3_finalize_mass(&w, box);
    CHECK_NEAR(1.0f / w.bodies[box].inv_mass, 8.0f, 1.0e-4f);
    for (int i = 0; i < 180; i++) {
        b3_step(&w, dt, 4);
    }
    float y = w.bodies[box].position.y;
    CHECK(y > 0.97f && y < 1.05f);
    CHECK(fabsf(w.bodies[box].lin_vel.y) < 0.05f);
    CHECK(w.contact_count >= 1);
    printf("  rest y=%.4f vy=%.4f contacts=%d\n",
        y, w.bodies[box].lin_vel.y, w.contact_count);

    b3_world_init(&w);
    w.gravity = b3_v(0.0f, 0.0f, 0.0f);
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
    float p0 = w.bodies[ia].lin_vel.x / w.bodies[ia].inv_mass
        + w.bodies[ib].lin_vel.x / w.bodies[ib].inv_mass;
    for (int i = 0; i < 90; i++) {
        b3_step(&w, dt, 4);
    }
    float p1 = w.bodies[ia].lin_vel.x / w.bodies[ia].inv_mass
        + w.bodies[ib].lin_vel.x / w.bodies[ib].inv_mass;
    CHECK_NEAR(p1, p0, 0.05f);
    CHECK_NEAR(w.bodies[ia].lin_vel.x, 0.0f, 0.05f);
    CHECK_NEAR(w.bodies[ib].lin_vel.x, 2.0f, 0.05f);
    printf("  momentum p0=%.4f p1=%.4f va=%.4f vb=%.4f\n",
        p0, p1, w.bodies[ia].lin_vel.x, w.bodies[ib].lin_vel.x);

    b3_world_init(&w);
    make_ground(&w, b3_v(0.0f, -0.5f, 0.0f), b3_v(20.0f, 0.5f, 20.0f),
        0.5f, 0.0f);
    int boxes[4];
    sd.density = 1.0f;
    sd.friction = 0.5f;
    sd.restitution = 0.0f;
    for (int i = 0; i < 4; i++) {
        B3BodyDef d = b3_default_body();
        d.type = B3_DYNAMIC;
        d.position = b3_v(0.0f, 0.5f + 1.05f * (float)i, 0.0f);
        boxes[i] = b3_create_body(&w, &d);
        b3_create_box(&w, boxes[i], b3_v(0.5f, 0.5f, 0.5f), &sd);
        b3_finalize_mass(&w, boxes[i]);
    }
    for (int s = 0; s < 240; s++) {
        b3_step(&w, dt, 4);
    }
    for (int i = 0; i < 4; i++) {
        float yi = w.bodies[boxes[i]].position.y;
        CHECK(yi > 0.4f + 0.9f * (float)i);
        CHECK(yi < 0.7f + 1.2f * (float)i);
        CHECK(fabsf(w.bodies[boxes[i]].lin_vel.y) < 0.2f);
        printf("  stack[%d] y=%.4f\n", i, yi);
    }

    b3_world_init(&w);
    make_ground(&w, b3_v(0.0f, -1.0f, 0.0f), b3_v(20.0f, 1.0f, 20.0f),
        0.0f, 0.8f);
    bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 3.0f, 0.0f);
    int sph = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    sd.friction = 0.0f;
    sd.restitution = 0.8f;
    b3_create_sphere(&w, sph, b3_v(0.0f, 0.0f, 0.0f), 0.5f, &sd);
    b3_finalize_mass(&w, sph);
    float min_y = 3.0f;
    float peak = -1.0e9f;
    int bounced = 0;
    for (int i = 0; i < 180; i++) {
        b3_step(&w, dt, 4);
        y = w.bodies[sph].position.y;
        if (y < min_y) {
            min_y = y;
        }
        if (min_y < 1.0f && y > peak) {
            peak = y;
            bounced = 1;
        }
    }
    CHECK(bounced);
    CHECK(peak > 1.4f);
    CHECK(min_y > 0.35f);
    printf("  bounce min_y=%.4f peak=%.4f\n", min_y, peak);
}

static void dump_scenes(void) {
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
    dump_body("free_fall", 0, 0, &w.bodies[id]);
    for (int i = 1; i <= 30; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
        dump_body("free_fall", i, 0, &w.bodies[id]);
    }

    b3_world_init(&w);
    make_ground(&w, b3_v(0.0f, -1.0f, 0.0f), b3_v(50.0f, 1.0f, 50.0f),
        0.3f, 0.0f);
    bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 4.0f, 0.0f);
    int box = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    sd.friction = 0.3f;
    sd.restitution = 0.0f;
    b3_create_box(&w, box, b3_v(1.0f, 1.0f, 1.0f), &sd);
    b3_finalize_mass(&w, box);
    dump_body("rest", 0, 1, &w.bodies[box]);
    for (int i = 1; i <= 180; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
        if (i == 60 || i == 120 || i == 180) {
            dump_body("rest", i, 1, &w.bodies[box]);
        }
    }

    b3_world_init(&w);
    make_ground(&w, b3_v(0.0f, -1.0f, 0.0f), b3_v(20.0f, 1.0f, 20.0f),
        0.0f, 0.8f);
    bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 3.0f, 0.0f);
    int sph = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    sd.friction = 0.0f;
    sd.restitution = 0.8f;
    b3_create_sphere(&w, sph, b3_v(0.0f, 0.0f, 0.0f), 0.5f, &sd);
    b3_finalize_mass(&w, sph);
    float min_y = 3.0f;
    float peak = -1.0e9f;
    int seen = 0;
    for (int i = 1; i <= 180; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
        float y = w.bodies[sph].position.y;
        if (y < min_y) {
            min_y = y;
        }
        if (min_y < 1.0f && y > peak) {
            peak = y;
            seen = 1;
        }
        if (i == 30 || i == 60 || i == 90 || i == 180) {
            dump_body("bounce", i, 1, &w.bodies[sph]);
        }
    }
    printf("S bounce min_y %.6f peak %.6f seen %d\n",
        min_y, seen ? peak : 0.0f, seen);

    b3_world_init(&w);
    w.gravity = b3_v(0.0f, 0.0f, 0.0f);
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
    for (int i = 1; i <= 90; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
        if (i == 30 || i == 60 || i == 90) {
            dump_body("sphere_hit", i, 0, &w.bodies[ia]);
            dump_body("sphere_hit", i, 1, &w.bodies[ib]);
        }
    }

    b3_world_init(&w);
    B3BodyDef kd = b3_default_body();
    kd.type = B3_KINEMATIC;
    kd.position = b3_v(0.0f, 0.0f, 0.0f);
    kd.lin_vel = b3_v(0.0f, 1.0f, 0.0f);
    int plat = b3_create_body(&w, &kd);
    sd.density = 0.0f;
    sd.friction = 0.8f;
    sd.restitution = 0.0f;
    b3_create_box(&w, plat, b3_v(2.0f, 0.1f, 2.0f), &sd);
    b3_finalize_mass(&w, plat);
    bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 0.7f, 0.0f);
    box = b3_create_body(&w, &bd);
    sd.density = 1.0f;
    b3_create_box(&w, box, b3_v(0.4f, 0.4f, 0.4f), &sd);
    b3_finalize_mass(&w, box);
    for (int i = 1; i <= 60; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    dump_body("kinematic", 60, 0, &w.bodies[plat]);
    dump_body("kinematic", 60, 1, &w.bodies[box]);

    b3_world_init(&w);
    make_ground(&w, b3_v(0.0f, -0.5f, 0.0f), b3_v(20.0f, 0.5f, 20.0f),
        0.5f, 0.0f);
    int boxes[4];
    sd.density = 1.0f;
    sd.friction = 0.5f;
    sd.restitution = 0.0f;
    for (int i = 0; i < 4; i++) {
        B3BodyDef d = b3_default_body();
        d.type = B3_DYNAMIC;
        d.position = b3_v(0.0f, 0.5f + 1.05f * (float)i, 0.0f);
        boxes[i] = b3_create_body(&w, &d);
        b3_create_box(&w, boxes[i], b3_v(0.5f, 0.5f, 0.5f), &sd);
        b3_finalize_mass(&w, boxes[i]);
    }
    for (int s = 0; s < 240; s++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    for (int i = 0; i < 4; i++) {
        dump_body("stack", 240, i, &w.bodies[boxes[i]]);
    }
}

static void bench_hello(int worlds, int steps) {
    B3World* ws = (B3World*)calloc((size_t)worlds, sizeof(B3World));
    for (int i = 0; i < worlds; i++) {
        b3_world_init(&ws[i]);
        make_ground(&ws[i], b3_v(0.0f, -1.0f, 0.0f),
            b3_v(50.0f, 1.0f, 50.0f), 0.3f, 0.0f);
        B3BodyDef bd = b3_default_body();
        bd.type = B3_DYNAMIC;
        bd.position = b3_v(0.0f, 4.0f + 0.01f * (float)i, 0.0f);
        int box = b3_create_body(&ws[i], &bd);
        B3ShapeDef sd = b3_default_shape();
        sd.density = 1.0f;
        sd.friction = 0.3f;
        b3_create_box(&ws[i], box, b3_v(1.0f, 1.0f, 1.0f), &sd);
        b3_finalize_mass(&ws[i], box);
    }
    double t0 = now_s();
    for (int s = 0; s < steps; s++) {
        for (int i = 0; i < worlds; i++) {
            b3_step(&ws[i], 1.0f / 60.0f, 4);
        }
    }
    double dt = now_s() - t0;
    printf("P hello worlds=%d steps=%d time=%.4fs sps=%.0f\n",
        worlds, steps, dt, (double)worlds * (double)steps / dt);
    free(ws);
}

static void bench_pile(int n, int steps) {
    B3World w;
    b3_world_init(&w);
    make_ground(&w, b3_v(0.0f, -0.5f, 0.0f), b3_v(20.0f, 0.5f, 20.0f),
        0.4f, 0.0f);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1.0f;
    sd.friction = 0.3f;
    for (int i = 0; i < n; i++) {
        int xi = i % 4;
        int zi = (i / 4) % 4;
        int yi = i / 16;
        B3BodyDef bd = b3_default_body();
        bd.type = B3_DYNAMIC;
        bd.position = b3_v(
            -1.5f + 1.0f * (float)xi,
            1.0f + 1.1f * (float)yi,
            -1.5f + 1.0f * (float)zi);
        int id = b3_create_body(&w, &bd);
        b3_create_sphere(&w, id, b3_v(0.0f, 0.0f, 0.0f), 0.4f, &sd);
        b3_finalize_mass(&w, id);
    }
    double t0 = now_s();
    for (int s = 0; s < steps; s++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    double dt = now_s() - t0;
    printf("P pile bodies=%d steps=%d time=%.4fs sps=%.0f\n",
        n, steps, dt, (double)steps / dt);
}

#ifdef __CUDACC__
static void bench_gpu(int n, int steps) {
    int devices = 0;
    cudaGetDeviceCount(&devices);
    if (devices <= 0) {
        printf("P gpu skipped (no CUDA device)\n");
        return;
    }
    B3World* host = (B3World*)calloc((size_t)n, sizeof(B3World));
    for (int i = 0; i < n; i++) {
        b3_world_init(&host[i]);
        make_ground(&host[i], b3_v(0.0f, -1.0f, 0.0f),
            b3_v(50.0f, 1.0f, 50.0f), 0.3f, 0.0f);
        B3BodyDef bd = b3_default_body();
        bd.type = B3_DYNAMIC;
        bd.position = b3_v(0.0f, 4.0f + 0.01f * (float)i, 0.0f);
        int box = b3_create_body(&host[i], &bd);
        B3ShapeDef sd = b3_default_shape();
        sd.density = 1.0f;
        sd.friction = 0.3f;
        b3_create_box(&host[i], box, b3_v(1.0f, 1.0f, 1.0f), &sd);
        b3_finalize_mass(&host[i], box);
    }
    B3World* dev = NULL;
    CHECK(cudaMalloc((void**)&dev, (size_t)n * sizeof(B3World))
        == cudaSuccess);
    CHECK(cudaMemcpy(dev, host, (size_t)n * sizeof(B3World),
        cudaMemcpyHostToDevice) == cudaSuccess);
    int threads = 64;
    int blocks = (n + threads - 1) / threads;
    cudaDeviceSynchronize();
    double t0 = now_s();
    for (int s = 0; s < steps; s++) {
        b3_step_kernel<<<blocks, threads>>>(dev, n, 1.0f / 60.0f, 4);
    }
    CHECK(cudaDeviceSynchronize() == cudaSuccess);
    double dt = now_s() - t0;
    printf("P gpu worlds=%d steps=%d time=%.4fs sps=%.0f\n",
        n, steps, dt, (double)n * (double)steps / dt);
    cudaFree(dev);
    free(host);
}
#endif

int main(int argc, char** argv) {
    int dump = 0;
    int bench = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dump") == 0) {
            dump = 1;
        }
        if (strcmp(argv[i], "--bench") == 0) {
            bench = 1;
        }
    }
    if (dump) {
        printf("engine cuda_port\n");
        dump_scenes();
        return 0;
    }
    printf("puffysics cuda correctness\n");
    test_invariants();
    printf("invariants passed\n");
    if (bench) {
        bench_hello(1, 2000);
        bench_hello(128, 120);
        bench_pile(32, 300);
#ifdef __CUDACC__
        bench_gpu(128, 120);
#endif
    }
    return 0;
}
