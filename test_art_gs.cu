#define B3_MAX_BODIES 16
#define B3_MAX_SHAPES 16
#define B3_MAX_CONTACTS 8
#define B3_MAX_JOINTS 16
#define B3_REVOLUTE_ONLY 1
#define B3_INTERLEAVE_CONTACTS 1
#define B3_SKIP_RESTITUTION 1
#define B3_PACKED_GS 1
#define B3_PERSISTENT_GS 1
#define B3_PACKED_INTEGRATE 1
#ifndef B3_JOINT_ITERS
#define B3_JOINT_ITERS 4
#endif

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
    if (!(isfinite(_a) && isfinite(_e)) || fabsf(_a - _e) > (tol)) { \
        fprintf(stderr, "FAIL %s:%d: %s=%g expected %g tol=%g\n", \
            __FILE__, __LINE__, #a, _a, _e, (float)(tol)); \
        exit(1); \
    } \
} while (0)

#ifdef __CUDACC__
#define CUDA_OK(call) do { \
    cudaError_t _e = (call); \
    if (_e != cudaSuccess) { \
        fprintf(stderr, "CUDA %s:%d: %s\n", __FILE__, __LINE__, \
            cudaGetErrorString(_e)); \
        exit(1); \
    } \
} while (0)
#endif

static float hinge_vgap(const B3World* w, int ji) {
    const B3Joint* j = &w->joints[ji];
    const B3Body* a = &w->bodies[j->body_a];
    const B3Body* b = &w->bodies[j->body_b];
    B3Vec3 pa = b3_xf_point(a->position, a->rotation, j->local_anchor_a);
    B3Vec3 pb = b3_xf_point(b->position, b->rotation, j->local_anchor_b);
    B3Vec3 ra = b3_sub(pa, a->center);
    B3Vec3 rb = b3_sub(pb, b->center);
    B3Vec3 va = b3_add(a->lin_vel, b3_cross(a->ang_vel, ra));
    B3Vec3 vb = b3_add(b->lin_vel, b3_cross(b->ang_vel, rb));
    return b3_len(b3_sub(vb, va));
}

static float hinge_pgap(const B3World* w, int ji) {
    const B3Joint* j = &w->joints[ji];
    const B3Body* a = &w->bodies[j->body_a];
    const B3Body* b = &w->bodies[j->body_b];
    B3Vec3 pa = b3_xf_point(a->position, a->rotation, j->local_anchor_a);
    B3Vec3 pb = b3_xf_point(b->position, b->rotation, j->local_anchor_b);
    return b3_len(b3_sub(pb, pa));
}

static void make_pendulum(B3World* w, float L, float r) {
    b3_world_init(w);
    B3BodyDef gd = b3_default_body();
    int ground = b3_create_body(w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_box(w, ground, b3_v(0.05f, 0.05f, 0.05f), &sd);
    b3_finalize_mass(w, ground);

    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(L, 0.0f, 0.0f);
    int bob = b3_create_body(w, &bd);
    sd.density = 1000.0f;
    b3_create_sphere(w, bob, b3_v(0.0f, 0.0f, 0.0f), r, &sd);
    b3_finalize_mass(w, bob);
    b3_create_revolute(w, ground, bob,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-L, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
}

static void make_double_pendulum(B3World* w, float L) {
    b3_world_init(w);
    B3BodyDef gd = b3_default_body();
    int ground = b3_create_body(w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_box(w, ground, b3_v(0.05f, 0.05f, 0.05f), &sd);
    b3_finalize_mass(w, ground);

    sd.density = 1000.0f;
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(L, 0.0f, 0.0f);
    int a = b3_create_body(w, &bd);
    b3_create_sphere(w, a, b3_v(0.0f, 0.0f, 0.0f), 0.08f, &sd);
    b3_finalize_mass(w, a);
    b3_create_revolute(w, ground, a,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-L, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 1.0f));

    bd.position = b3_v(2.0f * L, 0.0f, 0.0f);
    int b = b3_create_body(w, &bd);
    b3_create_sphere(w, b, b3_v(0.0f, 0.0f, 0.0f), 0.08f, &sd);
    b3_finalize_mass(w, b);
    b3_create_revolute(w, a, b,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-L, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
}

static void seed_tip_contact(B3GsContact* c, int ground, int tip) {
    memset(c, 0, sizeof(*c));
    c->body_a = ground;
    c->body_b = tip;
    c->point_count = 1;
    c->normal = b3_v(0.0f, 1.0f, 0.0f);
    c->tangent1 = b3_v(1.0f, 0.0f, 0.0f);
    c->tangent2 = b3_v(0.0f, 0.0f, 1.0f);
    c->center_a = b3_v(0.0f, 0.0f, 0.0f);
    c->center_b = b3_v(0.0f, -0.08f, 0.0f);
    c->friction = 0.0f;
    c->points[0].r_a = b3_v(0.0f, 0.0f, 0.0f);
    c->points[0].r_b = b3_v(0.0f, -0.08f, 0.0f);
    c->points[0].base_sep = -0.02f;
    c->softness.mass_scale = 1.0f;
    c->softness.bias_rate = 12.0f;
    c->softness.impulse_scale = 0.0f;
}

static void test_gs_delassus_vgap(void) {
    const float L = 1.0f;
    B3World artw;
    B3World indw;
    make_double_pendulum(&artw, L);
    make_double_pendulum(&indw, L);
    int tip = 2;
    B3Vec3 rb = b3_v(0.0f, -0.08f, 0.0f);
    B3Vec3 n = b3_v(0.0f, 1.0f, 0.0f);
    B3Vec3 rn = b3_cross(rb, n);
    float kn = indw.bodies[tip].inv_mass
        + b3_dot(rn, b3_mv(indw.bodies[tip].inv_i_world, rn));

    B3GsBody abl[16], ibl[16];
    B3GsJoint ajl[16], ijl[16];
    B3GsContact acl[8], icl[8];
    b3_gs_load(&artw, abl, ajl, acl);
    b3_gs_load(&indw, ibl, ijl, icl);
    seed_tip_contact(&acl[0], 0, tip);
    seed_tip_contact(&icl[0], 0, tip);
    icl[0].points[0].normal_mass = kn > 0.0f ? 1.0f / kn : 0.0f;

    b3_solve_contacts_gs_w(&artw, 0, acl, 1, abl, 60.0f, 10.0f, 1);
    b3_solve_contacts_gs(icl, 1, ibl, 60.0f, 10.0f, 1);
    b3_gs_store_velocities(&artw, abl);
    b3_gs_store_velocities(&indw, ibl);

    float va0 = hinge_vgap(&artw, 0);
    float va1 = hinge_vgap(&artw, 1);
    float vi0 = hinge_vgap(&indw, 0);
    float vi1 = hinge_vgap(&indw, 1);
    printf("  gs art mid_wz=%g tip_vy=%g vgap0=%g vgap1=%g\n",
        artw.bodies[1].ang_vel.z, artw.bodies[tip].lin_vel.y, va0, va1);
    printf("  gs ind mid_wz=%g tip_vy=%g vgap0=%g vgap1=%g nimp=%g/%g\n",
        indw.bodies[1].ang_vel.z, indw.bodies[tip].lin_vel.y, vi0, vi1,
        acl[0].points[0].normal_impulse, icl[0].points[0].normal_impulse);
    CHECK(isfinite(artw.bodies[tip].lin_vel.y));
    CHECK(fabsf(artw.bodies[1].ang_vel.z) > 1.0e-4f);
    CHECK(va0 < 2.0e-4f);
    CHECK(va1 < 2.0e-4f);
    CHECK(fabsf(indw.bodies[1].ang_vel.z) < 1.0e-6f);
    CHECK(vi1 > 10.0f * va1 + 1.0e-4f);
}

static void test_gs_step_drop(void) {
    B3World w;
    make_pendulum(&w, 1.0f, 0.1f);
    B3BodyDef fd = b3_default_body();
    fd.position = b3_v(0.0f, -0.25f, 0.0f);
    int floor = b3_create_body(&w, &fd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.4f;
    b3_create_box(&w, floor, b3_v(4.0f, 0.1f, 4.0f), &sd);
    b3_finalize_mass(&w, floor);

    for (int s = 0; s < 80; s++) {
        b3_step(&w, 1.0f / 60.0f, 4);
        CHECK(isfinite(w.bodies[1].center.y));
        CHECK(isfinite(w.bodies[1].lin_vel.y));
    }
    float gap = hinge_pgap(&w, 0);
    printf("  gs-step drop com.y=%g hinge=%g contacts=%d\n",
        w.bodies[1].center.y, gap, w.contact_count);
    CHECK(gap < 5.0e-2f);
    CHECK(w.bodies[1].center.y > -0.25f);
}

static void test_gs_grain_rest(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, -0.05f, 0.0f);
    int ground = b3_create_body(&w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.6f;
    b3_create_box(&w, ground, b3_v(4.0f, 0.05f, 4.0f), &sd);
    b3_finalize_mass(&w, ground);

    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 1.2f, 0.0f);
    int id = b3_create_body(&w, &bd);
    sd.density = 1000.0f;
    b3_create_sphere(&w, id, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(&w, id);

    CHECK(w.joint_count == 0);
    for (int s = 0; s < 90; s++) {
        b3_step(&w, 1.0f / 60.0f, 4);
        CHECK(isfinite(w.bodies[id].center.y));
    }
    printf("  gs grain y=%g vy=%g contacts=%d\n",
        w.bodies[id].center.y, w.bodies[id].lin_vel.y, w.contact_count);
    CHECK(w.bodies[id].center.y > 0.18f);
    CHECK(w.bodies[id].center.y < 0.30f);
    CHECK(fabsf(w.bodies[id].lin_vel.y) < 0.2f);
}

#ifdef __CUDACC__
__global__ void gs_drop_kernel(B3World* worlds, int n) {
    int i = (int)blockIdx.x;
    if (i >= n) {
        return;
    }
    for (int s = 0; s < 40; s++) {
        b3_step(&worlds[i], 1.0f / 60.0f, 4);
    }
}

static void test_gpu_gs_drop(void) {
    const int n = 8;
    B3World host;
    make_pendulum(&host, 1.0f, 0.1f);
    B3BodyDef fd = b3_default_body();
    fd.position = b3_v(0.0f, -0.25f, 0.0f);
    int floor = b3_create_body(&host, &fd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.4f;
    b3_create_box(&host, floor, b3_v(4.0f, 0.1f, 4.0f), &sd);
    b3_finalize_mass(&host, floor);

    B3World* worlds = (B3World*)malloc((size_t)n * sizeof(B3World));
    CHECK(worlds != 0);
    for (int i = 0; i < n; i++) {
        worlds[i] = host;
    }
    B3World* d = 0;
    CUDA_OK(cudaMalloc((void**)&d, (size_t)n * sizeof(B3World)));
    CUDA_OK(cudaMemcpy(d, worlds, (size_t)n * sizeof(B3World),
        cudaMemcpyHostToDevice));
    gs_drop_kernel<<<n, 1>>>(d, n);
    CUDA_OK(cudaGetLastError());
    CUDA_OK(cudaDeviceSynchronize());
    CUDA_OK(cudaMemcpy(worlds, d, (size_t)n * sizeof(B3World),
        cudaMemcpyDeviceToHost));
    for (int i = 0; i < n; i++) {
        CHECK(isfinite(worlds[i].bodies[1].center.y));
        CHECK(hinge_pgap(&worlds[i], 0) < 5.0e-2f);
    }
    printf("  gpu-gs n=%d com.y=%g hinge=%g\n",
        n, worlds[0].bodies[1].center.y, hinge_pgap(&worlds[0], 0));
    cudaFree(d);
    free(worlds);
}
#endif


static void test_gs_rolling(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, -0.05f, 0.0f);
    int ground = b3_create_body(&w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.0f;
    sd.rolling = 0.5f;
    b3_create_box(&w, ground, b3_v(4.0f, 0.05f, 4.0f), &sd);
    b3_finalize_mass(&w, ground);

    B3BodyDef hd = b3_default_body();
    hd.position = b3_v(6.0f, 1.0f, 0.0f);
    int hub = b3_create_body(&w, &hd);
    sd.rolling = 0.0f;
    b3_create_box(&w, hub, b3_v(0.05f, 0.05f, 0.05f), &sd);
    b3_finalize_mass(&w, hub);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(6.5f, 1.0f, 0.0f);
    int bob = b3_create_body(&w, &bd);
    sd.density = 1000.0f;
    b3_create_sphere(&w, bob, b3_v(0.0f, 0.0f, 0.0f), 0.08f, &sd);
    b3_finalize_mass(&w, bob);
    b3_create_revolute(&w, hub, bob,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-0.5f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 1.0f));

    bd.position = b3_v(0.0f, 0.2f, 0.0f);
    int ball = b3_create_body(&w, &bd);
    sd.friction = 0.0f;
    sd.rolling = 0.5f;
    b3_create_sphere(&w, ball, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(&w, ball);

    for (int i = 0; i < 40; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    w.bodies[ball].ang_vel = b3_v(4.0f, 0.0f, 0.0f);
    w.bodies[ball].lin_vel = b3_v(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 60; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
        CHECK(isfinite(w.bodies[ball].ang_vel.x));
    }
    printf("  gs rolling wx=%g x=%g\n",
        w.bodies[ball].ang_vel.x, w.bodies[ball].center.x);
    CHECK(fabsf(w.bodies[ball].ang_vel.x) < 1.0f);
    CHECK(fabsf(w.bodies[ball].center.x) < 0.2f);
}

int main(void) {
    test_gs_delassus_vgap();
    printf("gs-delassus: OK\n");
    test_gs_step_drop();
    printf("gs-step: OK\n");
    test_gs_grain_rest();
    printf("gs-grain: OK\n");
    test_gs_rolling();
    printf("gs-rolling: OK\n");
#ifdef __CUDACC__
    test_gpu_gs_drop();
    printf("gpu-gs: OK\n");
#endif
    return 0;
}
