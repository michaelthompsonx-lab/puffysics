/* Gravity on a B3World. Include this file (pulls puffysics.cuh and
 * nbody.cuh). Env API:
 *   NbodyConfig cfg
 *   b3_nbody_apply_forces(w, cfg, gravitational_mass or NULL)
 *   b3_nbody_step(w, dt, substeps, cfg, gravitational_mass or NULL)
 * CUDA: b3_nbody_step_kernel(worlds, n, dt, substeps, cfg, masses)
 * Point masses; no tidal torque. Contacts stay non-symplectic.
 * NULL masses use inverse inertial mass and skip static sources.
 */
#pragma once
#include "puffysics.cuh"
#include "nbody.cuh"

/* Optional gravitational_mass is body_count charges. */
static B3_HD B3_INL void b3_nbody_masses(B3World *w, NbodyConfig cfg,
        float *gravitational_mass, float *masses) {
    int i;
    B3Body *b;
    float mass;
    assert(w && masses);
    assert(w->body_count >= 0 && w->body_count <= B3_MAX_BODIES);
    assert(nbody_cfg_ok(cfg));
    for (i = 0; i < w->body_count; i++) {
        b = &w->bodies[i];
        mass = gravitational_mass ? gravitational_mass[i]
            : (b->type == B3_DYNAMIC && b->inv_mass > 0
                ? 1.0f / b->inv_mass : 0);
        assert(mass >= 0);
        masses[i] = mass;
    }
}

static B3_HD B3_INL void b3_nbody_accumulate(B3World *w, NbodyConfig cfg,
        float *masses) {
    int i, j;
    float eps2, q, inv;
    B3Body *a, *b;
    B3Vec3 r, force;
    eps2 = cfg.softening * cfg.softening;
    for (i = 0; i < w->body_count; i++) {
        if (masses[i] == 0) {
            continue;
        }
        a = &w->bodies[i];
        for (j = i + 1; j < w->body_count; j++) {
            b = &w->bodies[j];
            if (masses[j] == 0
                    || (a->type != B3_DYNAMIC && b->type != B3_DYNAMIC)) {
                continue;
            }
            r = b3_sub(b->center, a->center);
            q = b3_dot(r, r) + eps2;
            inv = 1.0f / sqrtf(q);
            force = b3_mul(r, cfg.gravity * masses[i] * masses[j]
                * inv * inv * inv);
            if (a->type == B3_DYNAMIC) {
                a->force = b3_add(a->force, force);
            }
            if (b->type == B3_DYNAMIC) {
                b->force = b3_sub(b->force, force);
            }
        }
    }
}

static B3_HD B3_INL void b3_nbody_apply_forces(B3World *w, NbodyConfig cfg,
        float *gravitational_mass) {
    float masses[B3_MAX_BODIES];
    b3_nbody_masses(w, cfg, gravitational_mass, masses);
    b3_nbody_accumulate(w, cfg, masses);
}

/* Rebuild gravity each substep. Incoming force/torque held over dt.
 * Do not also put this law in B3_USER_FORCES.
 */
static B3_HD B3_INL void b3_nbody_step(B3World *w, float dt, int substeps,
        NbodyConfig cfg, float *gravitational_mass) {
    float masses[B3_MAX_BODIES];
    B3Vec3 force[B3_MAX_BODIES], torque[B3_MAX_BODIES];
    int count, s, i;
    float h;
    assert(dt >= 0);
    b3_nbody_masses(w, cfg, gravitational_mass, masses);
    if (dt == 0) {
        return;
    }
    count = substeps < 1 ? 1 : substeps;
    h = dt / count;
    assert(h != 0);
    for (i = 0; i < w->body_count; i++) {
        force[i] = w->bodies[i].force;
        torque[i] = w->bodies[i].torque;
    }
    for (s = 0; s < count; s++) {
        for (i = 0; i < w->body_count; i++) {
            w->bodies[i].force = force[i];
            w->bodies[i].torque = torque[i];
        }
        b3_nbody_accumulate(w, cfg, masses);
        b3_step(w, h, 1);
    }
}

#ifdef __CUDACC__
/* One small rigid world per thread. Mass array is world-major.
 */
static __global__ void b3_nbody_step_kernel(B3World *worlds, int n,
        float dt, int substeps, NbodyConfig cfg, float *masses) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        b3_nbody_step(&worlds[i], dt, substeps, cfg,
            masses ? masses + (size_t)i * B3_MAX_BODIES : 0);
    }
}
#endif
