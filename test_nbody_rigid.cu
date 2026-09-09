/* Rigid-world coupling for direct gravity. World gravity must be zeroed;
 * otherwise uniform g is added on top of mutual attraction.
 * g++ -O2 -x c++ ocean/puffysics/test_nbody_rigid.cu -o /tmp/test-nbody-rigid-cpu
 * /tmp/test-nbody-rigid-cpu --cpu
 * nvcc -O2 -arch=native ocean/puffysics/test_nbody_rigid.cu -o /tmp/test-nbody-rigid
 * LD_LIBRARY_PATH=/run/opengl-driver/lib /tmp/test-nbody-rigid
 */
#include "nbody_rigid.cuh"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        exit(1); \
    } \
} while (0)

static int spawn_mass(B3World *w, B3Vec3 pos, B3Vec3 vel, float mass, int type) {
    B3BodyDef bd = b3_default_body();
    B3ShapeDef sd;
    int id;
    bd.type = type;
    bd.position = pos;
    bd.lin_vel = vel;
    id = b3_create_body(w, &bd);
    sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_sphere(w, id, b3_v(0, 0, 0), 0.08f, &sd);
    b3_set_inertial(w, id, type == B3_DYNAMIC ? mass : 0.0f,
        b3_v(0, 0, 0), b3_v(0.4f * mass * 0.0064f, 0.4f * mass * 0.0064f,
            0.4f * mass * 0.0064f));
    return id;
}

static void test_pair_force(void) {
    B3World w;
    int a, b;
    NbodyConfig cfg = {0.7f, 0.25f};
    double r2, fx;
    b3_world_init(&w);
    w.gravity = b3_v(0, 0, 0);
    a = spawn_mass(&w, b3_v(-1, 0, 0), b3_v(0, 0, 0), 2.0f, B3_DYNAMIC);
    b = spawn_mass(&w, b3_v(1, 0, 0), b3_v(0, 0, 0), 5.0f, B3_DYNAMIC);
    w.bodies[a].force = b3_v(0.1f, 0, 0);
    b3_nbody_apply_forces(&w, cfg, 0);
    r2 = 4.0 + (double)cfg.softening * cfg.softening;
    fx = (double)cfg.gravity * 2.0 * 5.0 * 2.0 / (r2 * sqrt(r2));
    CHECK(fabs((double)w.bodies[a].force.x - (0.1 + fx)) < 2e-6);
    CHECK(fabs((double)w.bodies[b].force.x + fx) < 2e-6);
    CHECK(w.bodies[a].force.y == 0 && w.bodies[b].force.y == 0);
    printf("rigid pair: Fa.x=%.6f Fb.x=%.6f (existing 0.1 preserved)\n",
        w.bodies[a].force.x, w.bodies[b].force.x);
}

static void test_static_source(void) {
    B3World w;
    int s, d, i;
    float mass[B3_MAX_BODIES];
    NbodyConfig cfg = {1.0f, 0.2f};
    b3_world_init(&w);
    w.gravity = b3_v(0, 0, 0);
    s = spawn_mass(&w, b3_v(0, 0, 0), b3_v(0, 0, 0), 0.0f, B3_STATIC);
    d = spawn_mass(&w, b3_v(2, 0, 0), b3_v(0, 0, 0), 1.0f, B3_DYNAMIC);
    for (i = 0; i < B3_MAX_BODIES; i++) {
        mass[i] = 0;
    }
    mass[s] = 8.0f;
    mass[d] = 1.0f;
    b3_nbody_apply_forces(&w, cfg, mass);
    CHECK(w.bodies[s].force.x == 0 && w.bodies[s].force.y == 0);
    CHECK(w.bodies[d].force.x < 0);
    printf("static source: dynamic Fx=%.6f (static remains %g)\n",
        w.bodies[d].force.x, w.bodies[s].force.x);
}

static void test_orbit(void) {
    B3World w;
    NbodyConfig cfg = {1.0f, 0.15f};
    double sep = 2.0;
    double r2 = sep * sep + (double)cfg.softening * cfg.softening;
    double omega = sqrt(2.0 * cfg.gravity / (r2 * sqrt(r2)));
    int a, b, s;
    float dt = 1.0f / 240.0f;
    double max_r = 0, max_p = 0, dx, dz, px, pz;
    b3_world_init(&w);
    w.gravity = b3_v(0, 0, 0);
    a = spawn_mass(&w, b3_v(-1, 0, 0), b3_v(0, 0, (float)(-omega)), 1.0f,
        B3_DYNAMIC);
    b = spawn_mass(&w, b3_v(1, 0, 0), b3_v(0, 0, (float)omega), 1.0f,
        B3_DYNAMIC);
    for (s = 0; s < 240 * 4; s++) {
        b3_nbody_step(&w, dt, 4, cfg, 0);
        CHECK(isfinite(w.bodies[a].center.x) && isfinite(w.bodies[b].center.x));
        dx = (double)w.bodies[b].center.x - w.bodies[a].center.x;
        dz = (double)w.bodies[b].center.z - w.bodies[a].center.z;
        if (fabs(sqrt(dx * dx + dz * dz) - sep) > max_r) {
            max_r = fabs(sqrt(dx * dx + dz * dz) - sep);
        }
        px = w.bodies[a].lin_vel.x / w.bodies[a].inv_mass
            + w.bodies[b].lin_vel.x / w.bodies[b].inv_mass;
        pz = w.bodies[a].lin_vel.z / w.bodies[a].inv_mass
            + w.bodies[b].lin_vel.z / w.bodies[b].inv_mass;
        if (fabs(px) + fabs(pz) > max_p) {
            max_p = fabs(px) + fabs(pz);
        }
    }
    printf("rigid 4s orbit: max |r-2|=%.6g  |P|=%.6g  az=%.4f bz=%.4f\n",
        max_r, max_p, w.bodies[a].center.z, w.bodies[b].center.z);
    CHECK(max_r < 0.08);
    CHECK(max_p < 2e-4);
    CHECK(fabs((double)w.bodies[a].center.y) < 1e-4);
}

#ifdef __CUDACC__
static void test_gpu_worlds(void) {
    int devices = 0;
    B3World host[2], out[2];
    NbodyConfig cfg = {1.0f, 0.2f};
    B3World *dev = 0;
    int i;
    CHECK(cudaGetDeviceCount(&devices) == cudaSuccess && devices > 0);
    for (i = 0; i < 2; i++) {
        b3_world_init(&host[i]);
        host[i].gravity = b3_v(0, 0, 0);
        spawn_mass(&host[i], b3_v(-1, 0, 0), b3_v(0, 0, 0), 1.0f, B3_DYNAMIC);
        spawn_mass(&host[i], b3_v(1, (float)i, 0), b3_v(0, 0, 0), 1.0f,
            B3_DYNAMIC);
    }
    CHECK(cudaMalloc((void **)&dev, sizeof(host)) == cudaSuccess);
    CHECK(cudaMemcpy(dev, host, sizeof(host), cudaMemcpyHostToDevice)
        == cudaSuccess);
    b3_nbody_step_kernel<<<1, 32>>>(dev, 2, 1.0f / 120.0f, 2, cfg, 0);
    CHECK(cudaDeviceSynchronize() == cudaSuccess);
    CHECK(cudaMemcpy(out, dev, sizeof(out), cudaMemcpyDeviceToHost)
        == cudaSuccess);
    CHECK(out[0].bodies[0].center.x > out[1].bodies[0].center.x);
    printf("gpu worlds: dx0=%.5f dx1=%.5f (isolated)\n",
        out[0].bodies[1].center.x - out[0].bodies[0].center.x,
        out[1].bodies[1].center.x - out[1].bodies[0].center.x);
    cudaFree(dev);
}
#endif

int main(int argc, char **argv) {
    int cpu = 0;
    if (argc == 2 && strcmp(argv[1], "--cpu") == 0) {
        cpu = 1;
    } else if (argc != 1) {
        fprintf(stderr, "usage: %s [--cpu]\n", argv[0]);
        return 1;
    }
#ifndef __CUDACC__
    if (!cpu) {
        fprintf(stderr, "host-only binary; pass --cpu\n");
        return 1;
    }
#endif
    test_pair_force();
    test_static_source();
    test_orbit();
#ifdef __CUDACC__
    if (!cpu) {
        test_gpu_worlds();
    }
#endif
    printf("PASS: Puffysics rigid N-body (%s)\n", cpu ? "CPU" : "CPU + CUDA");
    return 0;
}
