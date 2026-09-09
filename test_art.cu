#define B3_MAX_BODIES 16
#define B3_MAX_SHAPES 16
#define B3_MAX_CONTACTS 8
#define B3_MAX_JOINTS 16

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "puffysics.cuh"
#include "b3_art.cuh"

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

static void make_pendulum(B3World* w, float L, float r) {
    b3_world_init(w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, 0.0f, 0.0f);
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

static void pendulum_truth(const B3World* w, int bob, float L,
        float* qdd, float* I_z, float* mass) {
    const B3Body* b = &w->bodies[bob];
    float m = 1.0f / b->inv_mass;
    float Icom = 1.0f / b->inv_inertia.z;
    *mass = m;
    *I_z = Icom + m * L * L;
    *qdd = -(m * (-w->gravity.y) * L) / *I_z;
}

static void test_pendulum_qdd(void) {
    const float L = 1.0f;
    const float r = 0.1f;
    B3World w;
    make_pendulum(&w, L, r);
    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    CHECK(art.n_links == 2);
    CHECK(art.n_q == 1);
    CHECK(art.fixed[0]);
    CHECK(!art.fixed[1]);
    CHECK(art.parent[1] == 0);

    b3_art_aba(&art, &w);
    float qdd_ref, Iz, m;
    pendulum_truth(&w, 1, L, &qdd_ref, &Iz, &m);
    printf("  pendulum qdd=%g ref=%g  Dinv=%g Iz=%g m=%g\n",
        art.qdd[1], qdd_ref, art.Dinv[1], Iz, m);
    CHECK_NEAR(1.0f / art.Dinv[1], Iz, 1.0e-3f * Iz);
    CHECK_NEAR(art.qdd[1], qdd_ref, 2.0e-3f * fabsf(qdd_ref) + 1.0e-4f);
    CHECK(isfinite(art.a[1].v.x) && isfinite(art.a[1].v.y));
}

static void test_pendulum_delassus(void) {
    const float L = 1.0f;
    const float r = 0.1f;
    B3World w;
    make_pendulum(&w, L, r);
    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    int bob = b3_art_link_of(&art, 1);
    CHECK(bob == 1);

    B3ArtRow row;
    row.link_a = bob;
    row.link_b = -1;
    row.torque = 0;
    row.ra = b3_v(0.0f, 0.0f, 0.0f);
    row.rb = b3_v(0.0f, 0.0f, 0.0f);
    row.n = b3_v(0.0f, 1.0f, 0.0f);
    float x = 1.0f, y = 0.0f;
    b3_art_delassus_apply(&art, &w, &row, &x, &y, 1);

    float qdd_ref, Iz, m;
    pendulum_truth(&w, 1, L, &qdd_ref, &Iz, &m);
    float y_ref = (L * L) / Iz;
    printf("  delassus y=%g ref=%g\n", y, y_ref);
    CHECK_NEAR(y, y_ref, 2.0e-3f * y_ref + 1.0e-5f);

    row.n = b3_v(1.0f, 0.0f, 0.0f);
    b3_art_delassus_apply(&art, &w, &row, &x, &y, 1);
    CHECK_NEAR(y, 0.0f, 1.0e-4f);
}

static void test_free_body(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 2.0f, 0.0f);
    int id = b3_create_body(&w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1000.0f;
    b3_create_sphere(&w, id, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(&w, id);

    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    CHECK(art.n_links == 1);
    CHECK(art.floating[0]);
    b3_art_aba(&art, &w);
    float g = w.gravity.y;
    printf("  free a.y=%g g=%g\n", art.a[0].v.y, g);
    CHECK_NEAR(art.a[0].v.y, g, 1.0e-4f);
    CHECK_NEAR(art.a[0].v.x, 0.0f, 1.0e-4f);
    CHECK_NEAR(art.a[0].w.z, 0.0f, 1.0e-4f);

    B3ArtRow row;
    row.link_a = 0;
    row.link_b = -1;
    row.torque = 0;
    row.ra = b3_v(0.0f, 0.0f, 0.0f);
    row.rb = b3_v(0.0f, 0.0f, 0.0f);
    row.n = b3_v(0.0f, 1.0f, 0.0f);
    float x = 1.0f, y = 0.0f;
    b3_art_delassus_apply(&art, &w, &row, &x, &y, 1);
    float inv_m = w.bodies[id].inv_mass;
    printf("  free delassus=%g inv_m=%g\n", y, inv_m);
    CHECK_NEAR(y, inv_m, 1.0e-5f);
}

static void test_chain_build(void) {
    const int n = 8;
    B3World w;
    b3_world_init(&w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, (float)n + 2.0f, 0.0f);
    int prev = b3_create_body(&w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_box(&w, prev, b3_v(0.1f, 0.1f, 0.1f), &sd);
    b3_finalize_mass(&w, prev);
    sd.density = 1000.0f;
    for (int i = 0; i < n; i++) {
        B3BodyDef bd = b3_default_body();
        bd.type = B3_DYNAMIC;
        bd.position = b3_v(0.0f, (float)n + 1.5f - (float)i, 0.0f);
        int id = b3_create_body(&w, &bd);
        b3_create_box(&w, id, b3_v(0.1f, 0.5f, 0.1f), &sd);
        b3_finalize_mass(&w, id);
        b3_create_revolute(&w, prev, id,
            b3_v(0.0f, i ? -0.5f : 0.0f, 0.0f),
            b3_v(0.0f, 0.5f, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
        prev = id;
    }
    int tip = w.body_count - 1;
    w.bodies[tip].position.x += 0.25f;
    w.bodies[tip].center.x += 0.25f;

    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    CHECK(art.n_links == n + 1);
    CHECK(art.n_q == n);
    CHECK(art.fixed[0]);
    b3_art_aba(&art, &w);
    int finite = 1;
    float qdd_abs = 0.0f;
    for (int i = 1; i < art.n_links; i++) {
        if (!isfinite(art.qdd[i]) || !isfinite(art.a[i].v.x)) {
            finite = 0;
        }
        qdd_abs += fabsf(art.qdd[i]);
    }
    printf("  chain n_q=%d tip_qdd=%g sum|qdd|=%g\n",
        art.n_q, art.qdd[art.n_links - 1], qdd_abs);
    CHECK(finite);
    CHECK(qdd_abs > 1.0e-3f);
}

#ifdef __CUDACC__
__global__ void art_pendulum_kernel(B3World* worlds, float* qdd, int n) {
    int i = (int)blockIdx.x;
    if (i >= n) {
        return;
    }
    B3Art art;
    if (!b3_art_from_world(&art, &worlds[i])) {
        qdd[i] = 0.0f / 0.0f;
        return;
    }
    b3_art_aba(&art, &worlds[i]);
    qdd[i] = art.qdd[1];
}

static void test_gpu_pendulum(void) {
    const int n = 32;
    const float L = 1.0f;
    B3World host;
    make_pendulum(&host, L, 0.1f);
    B3Art art;
    CHECK(b3_art_from_world(&art, &host));
    b3_art_aba(&art, &host);
    float href = art.qdd[1];

    B3World* worlds = (B3World*)malloc((size_t)n * sizeof(B3World));
    CHECK(worlds);
    for (int i = 0; i < n; i++) {
        worlds[i] = host;
    }
    B3World* d = NULL;
    float* dq = NULL;
    CUDA_OK(cudaMalloc((void**)&d, (size_t)n * sizeof(B3World)));
    CUDA_OK(cudaMalloc((void**)&dq, (size_t)n * sizeof(float)));
    CUDA_OK(cudaMemcpy(d, worlds, (size_t)n * sizeof(B3World),
        cudaMemcpyHostToDevice));
    art_pendulum_kernel<<<n, 1>>>(d, dq, n);
    CUDA_OK(cudaGetLastError());
    CUDA_OK(cudaDeviceSynchronize());
    float hq[32];
    CUDA_OK(cudaMemcpy(hq, dq, (size_t)n * sizeof(float),
        cudaMemcpyDeviceToHost));
    for (int i = 0; i < n; i++) {
        CHECK(isfinite(hq[i]));
        CHECK_NEAR(hq[i], href, 1.0e-5f);
    }
    printf("  gpu n=%d qdd=%g\n", n, hq[0]);
    CUDA_OK(cudaFree(d));
    CUDA_OK(cudaFree(dq));
    free(worlds);
}
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

static void make_double_pendulum(B3World* w, float L) {
    b3_world_init(w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, 0.0f, 0.0f);
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

static void test_response_indep(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 1.0f, 0.0f);
    int id = b3_create_body(&w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1000.0f;
    b3_create_sphere(&w, id, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(&w, id);

    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    B3ArtRow row;
    CHECK(b3_art_make_row(&art, id, -1, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 1.0f, 0.0f), &row));
    float w_art = b3_art_response_w(&art, &w, &row);
    float w_ind = b3_art_indep_w(&w.bodies[id], b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 1.0f, 0.0f));
    printf("  indep w_art=%g w_ind=%g\n", w_art, w_ind);
    CHECK_NEAR(w_art, w_ind, 1.0e-5f);

    B3Vec3 r = b3_v(0.1f, 0.0f, 0.0f);
    CHECK(b3_art_make_row(&art, id, -1, r, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 1.0f, 0.0f), &row));
    w_art = b3_art_response_w(&art, &w, &row);
    w_ind = b3_art_indep_w(&w.bodies[id], r, b3_v(0.0f, 1.0f, 0.0f));
    CHECK_NEAR(w_art, w_ind, 1.0e-4f * w_ind + 1.0e-6f);
}

static void test_response_pendulum(void) {
    const float L = 1.0f;
    B3World w;
    make_pendulum(&w, L, 0.1f);
    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    B3ArtRow row;
    CHECK(b3_art_make_row(&art, 1, -1, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 1.0f, 0.0f), &row));
    float w_art = b3_art_response_w(&art, &w, &row);
    float qdd, Iz, m;
    pendulum_truth(&w, 1, L, &qdd, &Iz, &m);
    float w_ref = (L * L) / Iz;
    printf("  pend COM w_art=%g w_ref=%g\n", w_art, w_ref);
    CHECK_NEAR(w_art, w_ref, 2.0e-3f * w_ref + 1.0e-5f);

    /* Pivot is fixed: coupled Δ=0. Independent body still has 1/m + L²/I. */
    B3Vec3 rp = b3_v(-L, 0.0f, 0.0f);
    CHECK(b3_art_make_row(&art, 1, -1, rp, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 1.0f, 0.0f), &row));
    float w_piv = b3_art_response_w(&art, &w, &row);
    float w_ind = b3_art_indep_w(&w.bodies[1], rp, b3_v(0.0f, 1.0f, 0.0f));
    printf("  pend pivot w_art=%g w_indep=%g\n", w_piv, w_ind);
    CHECK(w_piv < 1.0e-3f);
    CHECK(w_ind > 0.5f);
}

static void test_foot_impulse(void) {
    const float L = 1.0f;
    B3World w;
    make_double_pendulum(&w, L);
    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    CHECK(art.n_q == 2);

    int tip = 2;
    B3ArtRow row;
    row.link_a = -1;
    row.link_b = b3_art_link_of(&art, tip);
    row.torque = 0;
    row.ra = b3_v(0.0f, 0.0f, 0.0f);
    row.rb = b3_v(0.0f, 0.0f, 0.0f);
    row.n = b3_v(0.0f, 1.0f, 0.0f);
    CHECK(row.link_b >= 0);
    b3_art_apply_impulse(&art, &w, &row, 1.0f);

    float v0 = hinge_vgap(&w, 0);
    float v1 = hinge_vgap(&w, 1);
    printf("  foot mid_wz=%g tip_vy=%g vgap0=%g vgap1=%g\n",
        w.bodies[1].ang_vel.z, w.bodies[tip].lin_vel.y, v0, v1);
    CHECK(fabsf(w.bodies[1].ang_vel.z) > 1.0e-4f);
    CHECK(fabsf(w.bodies[tip].lin_vel.y) > 1.0e-4f);
    CHECK(v0 < 2.0e-4f);
    CHECK(v1 < 2.0e-4f);

    B3World indep;
    make_double_pendulum(&indep, L);
    B3Body* tb = &indep.bodies[tip];
    tb->lin_vel = b3_madd(tb->lin_vel, tb->inv_mass, b3_v(0.0f, 1.0f, 0.0f));
    float iv0 = hinge_vgap(&indep, 0);
    float iv1 = hinge_vgap(&indep, 1);
    printf("  indep mid_wz=%g vgap1=%g\n", indep.bodies[1].ang_vel.z, iv1);
    CHECK_NEAR(indep.bodies[1].ang_vel.z, 0.0f, 1.0e-6f);
    CHECK(iv1 > 10.0f * v1 + 1.0e-4f);
    (void)iv0;
}

static void test_mixed_grain(void) {
    const float L = 1.0f;
    B3World w;
    make_pendulum(&w, L, 0.1f);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(L, -0.5f, 0.0f);
    int grain = b3_create_body(&w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1000.0f;
    b3_create_sphere(&w, grain, b3_v(0.0f, 0.0f, 0.0f), 0.15f, &sd);
    b3_finalize_mass(&w, grain);

    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    CHECK(art.n_q == 1);
    int n_float = 0;
    for (int i = 0; i < art.n_links; i++) {
        n_float += art.floating[i];
    }
    CHECK(n_float == 1);

    B3ArtRow row;
    CHECK(b3_art_make_row(&art, 1, grain, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 1.0f, 0.0f), &row));
    float w_art = b3_art_response_w(&art, &w, &row);
    float qdd, Iz, m;
    pendulum_truth(&w, 1, L, &qdd, &Iz, &m);
    float w_ref = (L * L) / Iz + w.bodies[grain].inv_mass;
    printf("  mixed w=%g ref=%g\n", w_art, w_ref);
    CHECK_NEAR(w_art, w_ref, 2.0e-3f * w_ref + 1.0e-5f);

    b3_art_apply_impulse(&art, &w, &row, 1.0f);
    CHECK(w.bodies[grain].lin_vel.y > 0.0f);
    CHECK(w.bodies[1].lin_vel.y < 0.0f);
    CHECK(fabsf(w.bodies[1].ang_vel.z) > 1.0e-4f);
    CHECK(hinge_vgap(&w, 0) < 2.0e-4f);
}

static void test_damped_inverse(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 1.0f, 0.0f);
    int id = b3_create_body(&w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1000.0f;
    b3_create_sphere(&w, id, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(&w, id);

    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    B3ArtRow row;
    CHECK(b3_art_make_row(&art, id, -1, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 1.0f, 0.0f), &row));
    float mu = 2.0f;
    float y = 1.0f;
    float lambda = 0.0f;
    b3_art_damped_solve(&art, &w, &row, &mu, &y, &lambda, 1);
    float inv_m = w.bodies[id].inv_mass;
    float lam_ref = y / (1.0f / mu + inv_m);
    printf("  free damped λ=%g ref=%g\n", lambda, lam_ref);
    CHECK_NEAR(lambda, lam_ref, 2.0e-4f * fabsf(lam_ref) + 1.0e-6f);

    make_pendulum(&w, 1.0f, 0.1f);
    CHECK(b3_art_from_world(&art, &w));
    CHECK(b3_art_make_row(&art, 1, -1, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 1.0f, 0.0f), &row));
    b3_art_damped_solve(&art, &w, &row, &mu, &y, &lambda, 1);
    float d = 0.0f;
    b3_art_delassus_apply(&art, &w, &row, &lambda, &d, 1);
    float residual = d + lambda / mu;
    printf("  pend damped λ=%g (Δ+μ^{-1})λ=%g\n", lambda, residual);
    CHECK_NEAR(residual, y, 2.0e-3f);
}


static float art_hinge_pgap(const B3Art* art, const B3World* w, int ji) {
    const B3Joint* j = &w->joints[ji];
    int la = b3_art_link_of(art, j->body_a);
    int lb = b3_art_link_of(art, j->body_b);
    CHECK(la >= 0 && lb >= 0);
    B3Vec3 pa = b3_xf_point(art->pos[la], art->rot[la], j->local_anchor_a);
    B3Vec3 pb = b3_xf_point(art->pos[lb], art->rot[lb], j->local_anchor_b);
    return b3_len(b3_sub(pb, pa));
}

static void make_two_bar(B3World* w) {
    const float s = 0.8660254f;
    const float h = 0.4330127f;
    b3_world_init(w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, 0.0f, 0.0f);
    int ground = b3_create_body(w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_box(w, ground, b3_v(0.05f, 0.05f, 0.05f), &sd);
    b3_finalize_mass(w, ground);

    sd.density = 1000.0f;
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.25f, h, 0.0f);
    int a = b3_create_body(w, &bd);
    b3_create_sphere(w, a, b3_v(0.0f, 0.0f, 0.0f), 0.08f, &sd);
    b3_finalize_mass(w, a);

    bd.position = b3_v(0.75f, h, 0.0f);
    int b = b3_create_body(w, &bd);
    b3_create_sphere(w, b, b3_v(0.0f, 0.0f, 0.0f), 0.08f, &sd);
    b3_finalize_mass(w, b);

    b3_create_revolute(w, ground, a,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-0.25f, -h, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
    b3_create_revolute(w, a, b,
        b3_v(0.25f, h, 0.0f), b3_v(-0.25f, h, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
    b3_create_revolute(w, b, ground,
        b3_v(0.25f, -h, 0.0f), b3_v(1.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
    (void)s;
}

static void test_fk_roundtrip(void) {
    const float L = 1.0f;
    B3World w;
    make_pendulum(&w, L, 0.1f);
    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    b3_art_refresh(&art, &w);
    int bob = 1;
    B3Vec3 com0 = art.com[bob];
    B3Quat rot0 = art.rot[bob];
    float q0 = art.q[bob];
    CHECK_NEAR(q0, 0.0f, 1.0e-5f);
    CHECK_NEAR(art_hinge_pgap(&art, &w, 0), 0.0f, 1.0e-6f);
    b3_art_fk_link(&art, bob);
    CHECK_NEAR(art.com[bob].x, com0.x, 1.0e-5f);
    CHECK_NEAR(art.com[bob].y, com0.y, 1.0e-5f);
    CHECK_NEAR(art.com[bob].z, com0.z, 1.0e-5f);
    CHECK_NEAR(b3_qdot(art.rot[bob], rot0), 1.0f, 1.0e-5f);
    CHECK_NEAR(art_hinge_pgap(&art, &w, 0), 0.0f, 1.0e-6f);

    art.q[bob] += 0.3f;
    b3_art_fk_link(&art, bob);
    float gap = art_hinge_pgap(&art, &w, 0);
    float ang = 0.3f;
    printf("  fk q+=0.3 com=(%g,%g) gap=%g\n",
        art.com[bob].x, art.com[bob].y, gap);
    CHECK_NEAR(art.com[bob].x, L * cosf(ang), 2.0e-5f);
    CHECK_NEAR(art.com[bob].y, L * sinf(ang), 2.0e-5f);
    CHECK(gap < 1.0e-6f);
}

static void test_art_step_swing(void) {
    B3World w;
    make_pendulum(&w, 1.0f, 0.1f);
    float y0 = w.bodies[1].center.y;
    float max_gap = 0.0f;
    for (int s = 0; s < 150; s++) {
        b3_art_step(&w, 1.0f / 60.0f, 4);
        CHECK(isfinite(w.bodies[1].center.y));
        CHECK(isfinite(w.bodies[1].lin_vel.y));
        float gap = hinge_pgap(&w, 0);
        if (gap > max_gap) {
            max_gap = gap;
        }
        CHECK(gap < 1.0e-5f);
    }
    printf("  swing com.y=%g max_gap=%g\n", w.bodies[1].center.y, max_gap);
    CHECK(w.bodies[1].center.y < y0 - 0.1f);
}

static void test_friction_indep(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 1.0f, 0.0f);
    int id = b3_create_body(&w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1000.0f;
    b3_create_sphere(&w, id, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(&w, id);

    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    B3ArtRow row;
    CHECK(b3_art_make_row(&art, -1, id, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(1.0f, 0.0f, 0.0f), &row));
    float w_art = b3_art_response_w(&art, &w, &row);
    float inv_m = w.bodies[id].inv_mass;
    printf("  fric indep Δ=%g inv_m=%g\n", w_art, inv_m);
    CHECK_NEAR(w_art, inv_m, 1.0e-5f);
    b3_art_apply_impulse(&art, &w, &row, 1.0f);
    CHECK_NEAR(w.bodies[id].lin_vel.x, inv_m, 1.0e-5f);
    CHECK_NEAR(w.bodies[id].lin_vel.y, 0.0f, 1.0e-6f);
}

static void test_friction_coupled(void) {
    const float L = 1.0f;
    B3World w;
    make_double_pendulum(&w, L);
    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    int tip = 2;
    B3Vec3 rb = b3_v(0.0f, -0.08f, 0.0f);
    B3ArtRow row;
    row.link_a = -1;
    row.link_b = b3_art_link_of(&art, tip);
    row.torque = 0;
    row.ra = b3_v(0.0f, 0.0f, 0.0f);
    row.rb = rb;
    row.n = b3_v(1.0f, 0.0f, 0.0f);
    CHECK(row.link_b >= 0);
    CHECK(b3_art_response_w(&art, &w, &row) > 1.0e-4f);
    b3_art_apply_impulse(&art, &w, &row, 1.0f);
    float v0 = hinge_vgap(&w, 0);
    float v1 = hinge_vgap(&w, 1);
    printf("  fric mid_wz=%g tip_wz=%g tip_vx=%g vgap0=%g vgap1=%g\n",
        w.bodies[1].ang_vel.z, w.bodies[tip].ang_vel.z,
        w.bodies[tip].lin_vel.x, v0, v1);
    CHECK(fabsf(w.bodies[1].ang_vel.z) > 1.0e-4f);
    CHECK(fabsf(w.bodies[tip].ang_vel.z) > 1.0e-4f);
    CHECK(v0 < 2.0e-4f);
    CHECK(v1 < 2.0e-4f);

    B3World indep;
    make_double_pendulum(&indep, L);
    B3Body* tb = &indep.bodies[tip];
    B3Vec3 P = b3_v(1.0f, 0.0f, 0.0f);
    tb->lin_vel = b3_madd(tb->lin_vel, tb->inv_mass, P);
    tb->ang_vel = b3_add(tb->ang_vel, b3_mv(tb->inv_i_world, b3_cross(rb, P)));
    CHECK_NEAR(indep.bodies[1].ang_vel.z, 0.0f, 1.0e-6f);
    CHECK(hinge_vgap(&indep, 1) > 10.0f * v1 + 1.0e-4f);
}


static void make_two_free(B3World* w, float gap) {
    b3_world_init(w);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1000.0f;
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 0.0f, 0.0f);
    int a = b3_create_body(w, &bd);
    b3_create_sphere(w, a, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(w, a);
    bd.position = b3_v(gap, 0.0f, 0.0f);
    int b = b3_create_body(w, &bd);
    b3_create_sphere(w, b, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(w, b);
}

static void make_four_bar(B3World* w) {
    const float L = 1.0f;
    b3_world_init(w);
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, 0.0f, 0.0f);
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
    bd.position = b3_v(L, L, 0.0f);
    int b = b3_create_body(w, &bd);
    b3_create_sphere(w, b, b3_v(0.0f, 0.0f, 0.0f), 0.08f, &sd);
    b3_finalize_mass(w, b);
    bd.position = b3_v(0.0f, L, 0.0f);
    int c = b3_create_body(w, &bd);
    b3_create_sphere(w, c, b3_v(0.0f, 0.0f, 0.0f), 0.08f, &sd);
    b3_finalize_mass(w, c);

    b3_create_revolute(w, ground, a,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-L, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
    b3_create_revolute(w, a, b,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, -L, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
    b3_create_revolute(w, ground, c,
        b3_v(0.0f, L, 0.0f), b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
    b3_create_revolute(w, c, b,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(-L, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 1.0f));
}

static void test_damped_two_body(void) {
    B3World w;
    make_two_free(&w, 1.0f);
    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    int ia = b3_art_link_of(&art, 0);
    int ib = b3_art_link_of(&art, 1);
    CHECK(ia >= 0 && ib >= 0);
    B3ArtRow row;
    row.link_a = ia;
    row.link_b = ib;
    row.torque = 0;
    row.ra = b3_v(0.0f, 0.0f, 0.0f);
    row.rb = b3_v(0.0f, 0.0f, 0.0f);
    row.n = b3_v(1.0f, 0.0f, 0.0f);
    float mu = 2.0f;
    float y = 1.0f;
    float lambda = 0.0f;
    b3_art_damped_solve(&art, &w, &row, &mu, &y, &lambda, 1);
    float ima = w.bodies[0].inv_mass;
    float imb = w.bodies[1].inv_mass;
    float lam_ref = y / (1.0f / mu + ima + imb);
    float d = 0.0f;
    b3_art_delassus_apply(&art, &w, &row, &lambda, &d, 1);
    float residual = d + lambda / mu;
    printf("  2body λ=%g ref=%g (Δ+μ^{-1})λ=%g Δ=%g\n",
        lambda, lam_ref, residual, ima + imb);
    CHECK_NEAR(lambda, lam_ref, 2.0e-4f * fabsf(lam_ref) + 1.0e-6f);
    CHECK_NEAR(residual, y, 2.0e-3f);
    CHECK(ima + imb > ima * 1.5f);
}

static void test_four_bar(void) {
    B3World w;
    make_four_bar(&w);
    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    printf("  4bar n_links=%d n_q=%d n_cuts=%d\n",
        art.n_links, art.n_q, art.n_cuts);
    CHECK(art.ok);
    CHECK(art.n_q == 3);
    CHECK(art.n_cuts == 1);
    CHECK(art.n_links == 4);
    int la = b3_art_link_of(&art, art.cut_body_a[0]);
    int lb = b3_art_link_of(&art, art.cut_body_b[0]);
    CHECK(la >= 0 && lb >= 0);
    CHECK(art.fixed[la] == 0 && art.fixed[lb] == 0);

    b3_art_refresh(&art, &w);
    B3ArtRow rows[5];
    float err[5];
    CHECK(b3_art_cut_rows(&art, &w, 0, rows, err) == 5);
    float D[25];
    b3_art_delassus_matrix(&art, &w, rows, D, 5);
    float off = 0.0f;
    for (int r = 0; r < 5; r++) {
        for (int c = 0; c < 5; c++) {
            if (r != c) {
                off += fabsf(D[r * 5 + c]);
            }
        }
    }
    printf("  4bar Δ off-diag L1=%g D00=%g\n", off, D[0]);
    CHECK(off > 1.0e-6f);

    float mu[5];
    float y[5];
    float lam[5];
    for (int k = 0; k < 5; k++) {
        mu[k] = 2.0f;
        y[k] = (k == 0) ? 1.0f : 0.0f;
        lam[k] = 0.0f;
    }
    b3_art_damped_solve(&art, &w, rows, mu, y, lam, 5);
    float yhat[5];
    b3_art_delassus_apply(&art, &w, rows, lam, yhat, 5);
    float rmax = 0.0f;
    for (int k = 0; k < 5; k++) {
        float rk = yhat[k] + lam[k] / mu[k] - y[k];
        if (fabsf(rk) > rmax) {
            rmax = fabsf(rk);
        }
    }
    printf("  4bar damped residual max=%g lam0=%g\n", rmax, lam[0]);
    CHECK(rmax < 2.0e-3f);

    float max_gap = 0.0f;
    for (int s = 0; s < 60; s++) {
        b3_art_step(&w, 1.0f / 60.0f, 4);
        for (int b = 1; b < w.body_count; b++) {
            CHECK(isfinite(w.bodies[b].center.y));
            CHECK(isfinite(w.bodies[b].lin_vel.y));
        }
        for (int j = 0; j < w.joint_count; j++) {
            float gap = hinge_pgap(&w, j);
            if (gap > max_gap) {
                max_gap = gap;
            }
        }
    }
    printf("  4bar step max_gap=%g By=%g Cy=%g\n",
        max_gap, w.bodies[2].center.y, w.bodies[3].center.y);
    CHECK(max_gap < 5.0e-3f);
}

static void test_twist_indep(void) {
    B3World w;
    b3_world_init(&w);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0.0f, 1.0f, 0.0f);
    int id = b3_create_body(&w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 1000.0f;
    b3_create_sphere(&w, id, b3_v(0.0f, 0.0f, 0.0f), 0.2f, &sd);
    b3_finalize_mass(&w, id);

    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    B3ArtRow row;
    CHECK(b3_art_make_row(&art, -1, id, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 1.0f, 0.0f), &row));
    row.torque = 1;
    float w_art = b3_art_response_w(&art, &w, &row);
    B3Vec3 n = b3_v(0.0f, 1.0f, 0.0f);
    float w_ind = b3_dot(n, b3_mv(w.bodies[id].inv_i_world, n));
    printf("  twist Δ=%g nI^{-1}n=%g\n", w_art, w_ind);
    CHECK_NEAR(w_art, w_ind, 1.0e-5f);
    b3_art_apply_impulse(&art, &w, &row, 1.0f);
    CHECK_NEAR(w.bodies[id].ang_vel.y, w_ind, 1.0e-5f);
    CHECK_NEAR(w.bodies[id].lin_vel.x, 0.0f, 1.0e-6f);
    CHECK_NEAR(w.bodies[id].lin_vel.y, 0.0f, 1.0e-6f);
}

static void test_loop_build(void) {
    B3World w;
    make_two_bar(&w);
    B3Art art;
    CHECK(b3_art_from_world(&art, &w));
    printf("  loop n_links=%d n_q=%d n_cuts=%d\n",
        art.n_links, art.n_q, art.n_cuts);
    CHECK(art.ok);
    CHECK(art.n_q == 2);
    CHECK(art.n_cuts == 1);
    CHECK(art.n_links == 3);
    b3_art_refresh(&art, &w);
    B3ArtRow rows[5];
    float err[5];
    CHECK(b3_art_cut_rows(&art, &w, 0, rows, err) == 5);
    float emax = 0.0f;
    for (int k = 0; k < 5; k++) {
        if (fabsf(err[k]) > emax) {
            emax = fabsf(err[k]);
        }
    }
    printf("  loop cut |err|_max=%g\n", emax);
    CHECK(emax < 2.0e-4f);
}

static void test_loop_step(void) {
    B3World w;
    make_two_bar(&w);
    float max_gap = 0.0f;
    for (int s = 0; s < 60; s++) {
        b3_art_step(&w, 1.0f / 60.0f, 4);
        for (int b = 1; b < w.body_count; b++) {
            CHECK(isfinite(w.bodies[b].center.y));
            CHECK(isfinite(w.bodies[b].lin_vel.y));
        }
        for (int j = 0; j < w.joint_count; j++) {
            float gap = hinge_pgap(&w, j);
            if (gap > max_gap) {
                max_gap = gap;
            }
        }
    }
    printf("  loop step max_gap=%g comA.y=%g comB.y=%g\n",
        max_gap, w.bodies[1].center.y, w.bodies[2].center.y);
    CHECK(max_gap < 5.0e-3f);
}



static void test_rolling_tree(void) {
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

    /* Dummy hinge so joint_count > 0 takes the Delassus contact path. */
    B3BodyDef hd = b3_default_body();
    hd.position = b3_v(6.0f, 1.0f, 0.0f);
    int hub = b3_create_body(&w, &hd);
    sd.density = 0.0f;
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

    CHECK(w.joint_count == 1);
    for (int i = 0; i < 40; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
    }
    w.bodies[ball].ang_vel = b3_v(4.0f, 0.0f, 0.0f);
    w.bodies[ball].lin_vel = b3_v(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 60; i++) {
        b3_step(&w, 1.0f / 60.0f, 4);
        CHECK(isfinite(w.bodies[ball].ang_vel.x));
    }
    printf("  rolling tree wx=%g x=%g joints=%d\n",
        w.bodies[ball].ang_vel.x, w.bodies[ball].center.x, w.joint_count);
    CHECK(fabsf(w.bodies[ball].ang_vel.x) < 1.0f);
    CHECK(fabsf(w.bodies[ball].center.x) < 0.2f);
}

static void test_soft_step_drop(void) {
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
    printf("  soft-step drop com.y=%g hinge=%g contacts=%d\n",
        w.bodies[1].center.y, gap, w.contact_count);
    CHECK(isfinite(w.bodies[1].center.y));
    CHECK(gap < 5.0e-2f);
    CHECK(w.bodies[1].center.y > -0.25f);
}

static void test_art_step_drop(void) {
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
        b3_art_step(&w, 1.0f / 60.0f, 4);
        CHECK(isfinite(w.bodies[1].center.y));
        CHECK(isfinite(w.bodies[1].lin_vel.y));
    }
    float gap = hinge_pgap(&w, 0);
    printf("  drop com.y=%g min_gap=%g contacts=%d\n",
        w.bodies[1].center.y, gap, w.contact_count);
    CHECK(gap < 1.0e-4f);
    CHECK(w.bodies[1].center.y > -0.2f);
}

#ifdef __CUDACC__
__global__ void art_impulse_kernel(B3World* worlds, float* vy, float* wz, int n) {
    int i = (int)blockIdx.x;
    if (i >= n) {
        return;
    }
    B3Art art;
    if (!b3_art_from_world(&art, &worlds[i])) {
        vy[i] = 0.0f / 0.0f;
        wz[i] = 0.0f / 0.0f;
        return;
    }
    B3ArtRow row;
    row.link_a = -1;
    row.link_b = 1;
    row.torque = 0;
    row.ra = b3_v(0.0f, 0.0f, 0.0f);
    row.rb = b3_v(0.0f, 0.0f, 0.0f);
    row.n = b3_v(0.0f, 1.0f, 0.0f);
    b3_art_apply_impulse(&art, &worlds[i], &row, 1.0f);
    vy[i] = worlds[i].bodies[1].lin_vel.y;
    wz[i] = worlds[i].bodies[1].ang_vel.z;
}

static void test_gpu_impulse(void) {
    const int n = 32;
    B3World host;
    make_pendulum(&host, 1.0f, 0.1f);
    B3Art art;
    CHECK(b3_art_from_world(&art, &host));
    B3ArtRow row;
    row.link_a = -1;
    row.link_b = 1;
    row.torque = 0;
    row.ra = b3_v(0.0f, 0.0f, 0.0f);
    row.rb = b3_v(0.0f, 0.0f, 0.0f);
    row.n = b3_v(0.0f, 1.0f, 0.0f);
    B3World href = host;
    b3_art_apply_impulse(&art, &href, &row, 1.0f);

    B3World* worlds = (B3World*)malloc((size_t)n * sizeof(B3World));
    CHECK(worlds);
    for (int i = 0; i < n; i++) {
        worlds[i] = host;
    }
    B3World* d = NULL;
    float* dvy = NULL;
    float* dwz = NULL;
    CUDA_OK(cudaMalloc((void**)&d, (size_t)n * sizeof(B3World)));
    CUDA_OK(cudaMalloc((void**)&dvy, (size_t)n * sizeof(float)));
    CUDA_OK(cudaMalloc((void**)&dwz, (size_t)n * sizeof(float)));
    CUDA_OK(cudaMemcpy(d, worlds, (size_t)n * sizeof(B3World),
        cudaMemcpyHostToDevice));
    art_impulse_kernel<<<n, 1>>>(d, dvy, dwz, n);
    CUDA_OK(cudaGetLastError());
    CUDA_OK(cudaDeviceSynchronize());
    float hvy[32], hwz[32];
    CUDA_OK(cudaMemcpy(hvy, dvy, (size_t)n * sizeof(float),
        cudaMemcpyDeviceToHost));
    CUDA_OK(cudaMemcpy(hwz, dwz, (size_t)n * sizeof(float),
        cudaMemcpyDeviceToHost));
    for (int i = 0; i < n; i++) {
        CHECK_NEAR(hvy[i], href.bodies[1].lin_vel.y, 1.0e-5f);
        CHECK_NEAR(hwz[i], href.bodies[1].ang_vel.z, 1.0e-5f);
    }
    printf("  gpu impulse vy=%g wz=%g\n", hvy[0], hwz[0]);
    CUDA_OK(cudaFree(d));
    CUDA_OK(cudaFree(dvy));
    CUDA_OK(cudaFree(dwz));
    free(worlds);
}
#endif

int main(void) {
    test_pendulum_qdd();
    printf("pendulum: OK\n");
    test_pendulum_delassus();
    printf("delassus: OK\n");
    test_free_body();
    printf("free-body: OK\n");
    test_chain_build();
    printf("chain: OK\n");
    test_response_indep();
    printf("response-indep: OK\n");
    test_response_pendulum();
    printf("response-tree: OK\n");
    test_foot_impulse();
    printf("foot-impulse: OK\n");
    test_mixed_grain();
    printf("mixed-grain: OK\n");
    test_damped_inverse();
    printf("damped: OK\n");
    test_fk_roundtrip();
    printf("fk: OK\n");
    test_art_step_swing();
    printf("swing: OK\n");
    test_art_step_drop();
    printf("art-step: OK\n");
    test_friction_indep();
    printf("friction-indep: OK\n");
    test_friction_coupled();
    printf("friction-tree: OK\n");
    test_loop_build();
    printf("loop-build: OK\n");
    test_loop_step();
    printf("loop-step: OK\n");
    test_damped_two_body();
    printf("lcaba-2body: OK\n");
    test_four_bar();
    printf("lcaba-4bar: OK\n");
    test_twist_indep();
    printf("twist-indep: OK\n");
    test_soft_step_drop();
    printf("soft-step: OK\n");
    test_rolling_tree();
    printf("rolling-tree: OK\n");
#ifdef __CUDACC__
    test_gpu_pendulum();
    printf("gpu: OK\n");
    test_gpu_impulse();
    printf("gpu-impulse: OK\n");
#endif
    return 0;
}
