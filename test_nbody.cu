/* Standalone numerical regressions; no engine dependency.
 * g++ -O2 -x c++ ocean/puffysics/test_nbody.cu -o /tmp/test-nbody-cpu
 * /tmp/test-nbody-cpu --cpu
 * nvcc -O2 -arch=native ocean/puffysics/test_nbody.cu -o /tmp/test-nbody
 * /tmp/test-nbody                 (requires a CUDA device; never silently skips)
 */
#include "nbody.cuh"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void require(int condition, const char *message) {
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static double vector_error(NbodyVec a, NbodyVec b) {
    double dx = fabs((double)a.x - b.x);
    double dy = fabs((double)a.y - b.y);
    double dz = fabs((double)a.z - b.z);
    return dx > dy ? (dx > dz ? dx : dz) : (dy > dz ? dy : dz);
}

static void test_force_and_potential(void) {
    NbodyConfig cfg = {0.7f, 0.25f};
    NbodyPoint p[3] = {{0, 0, 0, 2}, {2, 0, 0, 5}, {0, 0, 0, 0}};
    NbodyVec a[3], v[3], overlap[3], shifted_a[3];
    NbodyPoint shifted[3];
    double r2, factor, force_error, energy, expected, overlap_error;
    double translation_error;
    int i;
    memset(v, 0, sizeof(v));
    nbody_acceleration(p, a, 3, cfg);
    r2 = 4 + (double)cfg.softening * cfg.softening;
    factor = cfg.gravity * 2 / (r2 * sqrt(r2));
    force_error = fabs(a[0].x - 5 * factor);
    if (fabs(a[1].x + 2 * factor) > force_error) {
        force_error = fabs(a[1].x + 2 * factor);
    }
    require(force_error < 2e-7,
        "acceleration must depend on source, not target mass");
    require(vector_error(a[0], a[2]) == 0,
        "massless tracer must feel full gravity");
    energy = nbody_energy(p, v, 3, cfg);
    expected = -(double)cfg.gravity * 10 / sqrt(r2);
    require(fabs(energy - expected) < 1e-12,
        "softened pair potential or tracer energy wrong");

    p[2].mass = 3;
    nbody_acceleration(p, overlap, 3, cfg);
    overlap_error = vector_error(a[0], overlap[0]);
    if (fabs(overlap[1].x + 5 * factor) > overlap_error) {
        overlap_error = fabs(overlap[1].x + 5 * factor);
    }
    require(overlap_error < 2e-7 && vector_error(overlap[0], overlap[2]) == 0,
        "coincident softened sources must remain finite");
    expected = -(double)cfg.gravity * (25 / sqrt(r2) + 6 / cfg.softening);
    require(fabs(nbody_energy(p, v, 3, cfg) - expected) < 1e-12,
        "overlap potential must use epsilon and exclude self energy");

    for (i = 0; i < 3; i++) {
        shifted[i] = p[i];
        shifted[i].x += 3;
        shifted[i].y -= 1.5f;
        shifted[i].z += 0.25f;
    }
    nbody_acceleration(shifted, shifted_a, 3, cfg);
    translation_error = 0;
    for (i = 0; i < 3; i++) {
        if (vector_error(overlap[i], shifted_a[i]) > translation_error) {
            translation_error = vector_error(overlap[i], shifted_a[i]);
        }
    }
    require(translation_error == 0
        && nbody_energy(shifted, v, 3, cfg) == nbody_energy(p, v, 3, cfg),
        "uniform translation changed gravity or potential");
    nbody_acceleration(0, 0, 0, cfg);
    nbody_step(0, 0, 0, 0, 0.01f, 3, cfg);
    require(nbody_energy(0, 0, 0, cfg) == 0, "empty-world energy");
    printf("force: asymmetry error %.3g; overlap error %.3g; "
        "translation error %.3g\n",
        force_error, overlap_error, translation_error);
}

static void test_circular_orbit(void) {
    double pi = 3.14159265358979323846;
    NbodyConfig cfg = {0.7f, 0.15f};
    double r2 = 4 + (double)cfg.softening * cfg.softening;
    double omega = sqrt(4 * cfg.gravity / (r2 * sqrt(r2)));
    float dt = (float)(2 * pi / (omega * 4096));
    NbodyPoint p[2] = {{-1.5f, 0, 0, 1}, {0.5f, 0, 0, 3}};
    NbodyVec v[2] = {{0, (float)(-1.5 * omega), 0},
        {0, (float)(0.5 * omega), 0}};
    NbodyVec a[2];
    double initial_energy = nbody_energy(p, v, 2, cfg);
    double max_energy_error = 0, max_phase_error = 0, max_radius_error = 0;
    double phase = 0, previous = 0, x, y, angle, energy_error, phase_error;
    double radius_error;
    int samples = 100 * 4096 / 64;
    int sample;
    for (sample = 1; sample <= samples; sample++) {
        nbody_step(p, v, a, 2, dt, 64, cfg);
        x = (double)p[1].x - p[0].x;
        y = (double)p[1].y - p[0].y;
        angle = atan2(y, x);
        phase += remainder(angle - previous, 2 * pi);
        previous = angle;
        energy_error = fabs((nbody_energy(p, v, 2, cfg) - initial_energy)
            / initial_energy);
        phase_error = fabs(phase - omega * (double)dt * (sample * 64));
        radius_error = fabs(sqrt(x * x + y * y) - 2);
        require(isfinite(energy_error) && isfinite(phase_error)
            && isfinite(radius_error), "circular orbit became nonfinite");
        if (energy_error > max_energy_error) {
            max_energy_error = energy_error;
        }
        if (phase_error > max_phase_error) {
            max_phase_error = phase_error;
        }
        if (radius_error > max_radius_error) {
            max_radius_error = radius_error;
        }
    }
    printf("100 softened orbits: max relative energy %.6g; phase %.6g rad; "
        "radius %.6g\n",
        max_energy_error, max_phase_error, max_radius_error);
    require(max_energy_error < 2e-4, "KDK orbit energy must remain bounded");
    require(max_phase_error < 0.03, "KDK orbit phase drift excessive");
    require(max_radius_error < 5e-4, "softened circular orbit radius drift");
}

#ifdef __CUDACC__
static void make_cloud(NbodyPoint *p, NbodyVec *v, int n) {
    int i;
    for (i = 0; i < n; i++) {
        p[i].x = (float)(i % 7) * 0.375f - 1;
        p[i].y = (float)((i / 7) % 7) * 0.3125f - 0.875f;
        p[i].z = (float)(i / 49) * 0.4375f - 1;
        p[i].mass = i % 11 == 0 ? 0.0f : 0.01f * (float)(1 + i % 5);
        v[i].x = 0.025f * p[i].y;
        v[i].y = -0.025f * p[i].x;
        v[i].z = 0.01f * (float)((i % 3) - 1);
    }
    p[128].x = p[0].x;
    p[128].y = p[0].y;
    p[128].z = p[0].z;
    p[n - 1].mass = 0;
}

static void compare_gpu(NbodyGpu *g, NbodyPoint *p, NbodyVec *v, int n,
        const char *label) {
    NbodyPoint *result = (NbodyPoint *)malloc((size_t)n * sizeof(*result));
    NbodyVec *velocity = (NbodyVec *)malloc((size_t)n * sizeof(*velocity));
    NbodyVec pos, got;
    double max_position = 0, max_velocity = 0, dp, dv;
    int i;
    require(result && velocity, "compare_gpu alloc");
    nbody_gpu_download(g, result, velocity);
    for (i = 0; i < n; i++) {
        pos.x = p[i].x;
        pos.y = p[i].y;
        pos.z = p[i].z;
        got.x = result[i].x;
        got.y = result[i].y;
        got.z = result[i].z;
        dp = vector_error(pos, got);
        dv = vector_error(v[i], velocity[i]);
        require(isfinite(result[i].x) && isfinite(result[i].y)
            && isfinite(result[i].z) && isfinite(velocity[i].x)
            && isfinite(velocity[i].y) && isfinite(velocity[i].z),
            "GPU trajectory became nonfinite");
        require(result[i].mass == p[i].mass,
            "integration changed a particle mass");
        if (dp > max_position) {
            max_position = dp;
        }
        if (dv > max_velocity) {
            max_velocity = dv;
        }
    }
    printf("GPU %-28s n=%d: max position %.6g; velocity %.6g\n",
        label, n, max_position, max_velocity);
    require(max_position < 3e-5 && max_velocity < 3e-5,
        "CPU/GPU trajectory mismatch");
    free(result);
    free(velocity);
}

static void test_gpu(void) {
    int devices = 0;
    NbodyGpu empty, g;
    NbodyConfig cfg = {0.7f, 0.075f};
    const int n = 259;
    NbodyPoint *p = (NbodyPoint *)malloc((size_t)n * sizeof(*p));
    NbodyVec *v = (NbodyVec *)malloc((size_t)n * sizeof(*v));
    NbodyVec *a = (NbodyVec *)malloc((size_t)n * sizeof(*a));
    int i;
    require(cudaGetDeviceCount(&devices) == cudaSuccess && devices > 0,
        "CUDA device required unless --cpu is specified");
    require(p && v && a, "cloud alloc");
    memset(&empty, 0, sizeof(empty));
    nbody_gpu_init(&empty, 0);
    nbody_gpu_upload(&empty, 0, 0);
    nbody_gpu_step(&empty, 0.1f, 2, cfg);
    nbody_gpu_download(&empty, 0, 0);
    nbody_gpu_free(&empty);

    make_cloud(p, v, n);
    memset(&g, 0, sizeof(g));
    nbody_gpu_init(&g, n);
    nbody_gpu_upload(&g, p, v);
    nbody_step(p, v, a, n, 0.002f, 64, cfg);
    nbody_gpu_step(&g, 0.002f, 64, cfg);
    compare_gpu(&g, p, v, n, "partial tiles + overlap");

    nbody_step(p, v, a, n, 0.002f, 16, cfg);
    nbody_gpu_step(&g, 0.002f, 16, cfg);
    compare_gpu(&g, p, v, n, "cached continuation");

    cfg.softening = 0.9f;
    nbody_step(p, v, a, n, 0.08f, 1, cfg);
    nbody_gpu_step(&g, 0.08f, 1, cfg);
    compare_gpu(&g, p, v, n, "softening invalidation");
    cfg.gravity = 0;
    nbody_step(p, v, a, n, 0.08f, 1, cfg);
    nbody_gpu_step(&g, 0.08f, 1, cfg);
    compare_gpu(&g, p, v, n, "gravity invalidation");
    cfg.gravity = 0.7f;
    nbody_step(p, v, a, n, 0.02f, 2, cfg);
    nbody_gpu_step(&g, 0.02f, 2, cfg);
    compare_gpu(&g, p, v, n, "restored gravity");

    make_cloud(p, v, n);
    for (i = 0; i < n; i++) {
        p[i].x *= 0.5f;
        p[i].y *= 0.75f;
        p[i].mass *= 2;
    }
    nbody_gpu_upload(&g, p, v);
    nbody_step(p, v, a, n, 0.08f, 1, cfg);
    nbody_gpu_step(&g, 0.08f, 1, cfg);
    compare_gpu(&g, p, v, n, "upload invalidation");
    nbody_gpu_free(&g);
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
        fprintf(stderr, "This binary is host-only; pass --cpu or build with nvcc.\n");
        return 1;
    }
#endif
    test_force_and_potential();
    test_circular_orbit();
#ifdef __CUDACC__
    if (!cpu) {
        test_gpu();
    }
#endif
    printf("PASS: Puffysics direct N-body (%s)\n", cpu ? "CPU" : "CPU + CUDA");
    return 0;
}
