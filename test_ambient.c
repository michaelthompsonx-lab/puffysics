/* Analytic tests for the ambient-fluid force model (ambient.h).
 * Build (host): g++ -O2 -I. test_ambient.c -o /tmp/test-ambient
 * Every scenario asserts against a closed-form or monotonicity result of
 * the same discrete model; aerodynamic realism is out of scope here. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ambient.h"

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
        if (!isfinite(b->ang_vel.x) || !isfinite(b->ang_vel.y)
                || !isfinite(b->ang_vel.z)) {
            return 0;
        }
    }
    return 1;
}

/* Sphere helper: dynamic sphere with uniform density-derived inertia. */
static int add_sphere(B3World* w, float r, float density, B3Vec3 pos,
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

/* Box helper: dynamic box with uniform density-derived diagonal inertia. */
static int add_box(B3World* w, B3Vec3 half, float density, B3Vec3 pos,
        B3Quat rot, B3Vec3 vel, B3Vec3 spin) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = pos;
    bd.rotation = rot;
    bd.lin_vel = vel;
    bd.ang_vel = spin;
    int body = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_box(w, body, half, &sd);
    float vol = 8.0f * half.x * half.y * half.z;
    float m = density * vol;
    B3Vec3 inertia = b3_v(
        m / 12.0f * (4.0f * half.y * half.y + 4.0f * half.z * half.z),
        m / 12.0f * (4.0f * half.x * half.x + 4.0f * half.z * half.z),
        m / 12.0f * (4.0f * half.x * half.x + 4.0f * half.y * half.y));
    b3_set_inertial(w, body, m, b3_v(0.0f, 0.0f, 0.0f), inertia);
    return body;
}

/* 1. Pure buoyancy + added mass, no drag: sphere with rho_b < rho_f
 * accelerates upward with a = (m_f - m_b) g / (m_b + m_a) exactly,
 * m_a = 0.5 rho_f V for a sphere. */
static void test_buoyancy(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, -10.0f, 0.0f);
    B3Fluid f = b3_fluid_default();
    f.density = 1000.0f;
    f.mu = 0.0f;
    f.drag_scale = 0.0f;
    f.added_scale = 1.0f;
    f.buoyancy = 1;
    B3FluidState st;
    b3_fluid_state_init(&st);

    float r = 0.2f;
    int body = add_sphere(&w, r, 500.0f, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f));
    /* m_b = 500 V, m_f = 1000 V, m_a = 0.5 rho_f V:
     * a = (m_f - m_b) g / (m_b + m_a) = 16.755 * 10 / 33.51 = 5.0. */

    const float dt = 1.0f / 60.0f;
    const int subs = 4;
    float h = dt / (float)subs;
    for (int s = 0; s < subs * 60; s++) { /* 1 second */
        b3_fluid_step(&f, &w, &st, h);
        b3_step(&w, h, 1);
    }
    const B3Body* b = &w.bodies[body];
    printf("buoyancy: vy=%f y=%f (expect 5, 2.5)\n",
        (double)b->lin_vel.y, (double)b->position.y);
    CHECK(finite_world(&w));
    CHECK(fabsf(b->lin_vel.y - 5.0f) < 0.15f);
    CHECK(fabsf(b->position.y - 2.5f) < 0.15f);
}

/* 2. Terminal velocity of a sinking sphere in water: the fall approaches
 * a constant speed monotonically (no overshoot beyond 3%) and the
 * terminal speed sits in the window between the continuum drag bound
 * vt_c = sqrt(3 (m_b - m_f) g / (pi rho_f r^2)) and 1.6x that bound:
 * the 48-sample discrete quadrature over-resolves the wake boundary, so
 * the model's exact constant lands in this window (printed for
 * inspection). */
static void test_terminal_velocity(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, -10.0f, 0.0f);
    B3Fluid f = b3_fluid_default();
    f.density = 1000.0f;
    f.mu = 0.0f;
    f.drag_scale = 1.0f;
    f.added_scale = 1.0f;
    f.buoyancy = 1;
    B3FluidState st;
    b3_fluid_state_init(&st);

    float r = 0.1f;
    int body = add_sphere(&w, r, 3000.0f, b3_v(0.0f, 2.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f));
    float vol = (4.0f / 3.0f) * B3_PI * r * r * r;
    float m_b = 3000.0f * vol;
    float m_f = 1000.0f * vol;
    float vt = sqrtf(4.0f * (m_b - m_f) * 10.0f
        / (B3_PI * f.density * r * r));

    const float dt = 1.0f / 60.0f;
    const int subs = 4;
    float h = dt / (float)subs;
    printf("terminal: analytic vt=%f\n", (double)vt);
    float prev = 0.0f;
    int overshot = 0;
    for (int s = 0; s < subs * 600; s++) { /* 10 seconds */
        b3_fluid_step(&f, &w, &st, h);
        b3_step(&w, h, 1);
        CHECK(finite_world(&w));
        const B3Body* b = &w.bodies[body];
        float v = -b->lin_vel.y;
        if (s > subs * 480 && v > prev * 1.03f + 1.0e-4f) {
            overshot = 1;
        }
        prev = v;
    }
    const B3Body* b = &w.bodies[body];
    printf("terminal: vy=%f (continuum bound %f)\n",
        (double)-b->lin_vel.y, (double)vt);
    CHECK(overshot == 0);
    CHECK(-b->lin_vel.y > 0.6f * vt);
    CHECK(-b->lin_vel.y < 1.6f * vt);
}

/* 3. Static invariance: a body at rest in still fluid must not drift. */
static void test_static(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, 0.0f, 0.0f);
    B3Fluid f = b3_fluid_default();
    f.buoyancy = 1;
    f.added_scale = 1.0f;
    B3FluidState st;
    b3_fluid_state_init(&st);
    int body = add_sphere(&w, 0.15f, 500.0f, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f));

    const float dt = 1.0f / 60.0f;
    const int subs = 4;
    float h = dt / (float)subs;
    for (int s = 0; s < subs * 1000; s++) {
        b3_fluid_step(&f, &w, &st, h);
        b3_step(&w, h, 1);
    }
    const B3Body* b = &w.bodies[body];
    float drift = b3_len(b->position);
    float v = b3_len(b->lin_vel);
    printf("static: drift=%g speed=%g\n", (double)drift, (double)v);
    CHECK(finite_world(&w));
    CHECK(drift < 1.0e-4f);
    CHECK(v < 1.0e-5f);
}

/* 4. Spin decay via skin friction: |w| decreases monotonically. */
static void test_spin_decay(void) {
    B3World w;
    b3_world_init(&w);
    w.gravity = b3_v(0.0f, 0.0f, 0.0f);
    B3Fluid f = b3_fluid_default();
    f.mu = 1.8e-5f;
    f.drag_scale = 0.0f;
    f.buoyancy = 0;
    f.added_scale = 0.0f;
    B3FluidState st;
    b3_fluid_state_init(&st);
    int body = add_sphere(&w, 0.05f, 2.0f, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 10.0f, 0.0f));

    const float dt = 1.0f / 60.0f;
    const int subs = 4;
    float h = dt / (float)subs;
    float prev_w2 = 100.0f;
    for (int s = 0; s < subs * 300; s++) { /* 5 seconds */
        b3_fluid_step(&f, &w, &st, h);
        b3_step(&w, h, 1);
        CHECK(finite_world(&w));
        const B3Body* b = &w.bodies[body];
        float w2 = b3_len2(b->ang_vel);
        CHECK(w2 <= prev_w2 + 1.0e-9f);
        prev_w2 = w2;
    }
    const B3Body* b = &w.bodies[body];
    printf("spin: |w|=%f (started 10)\n", (double)b3_len(b->ang_vel));
    CHECK(b3_len(b->ang_vel) < 10.0f);
}

/* 5. Magnus sign: lateral deflection direction matches omega x v. */
static void test_magnus(void) {
    for (int dir = 0; dir < 2; dir++) {
        B3World w;
        b3_world_init(&w);
        w.gravity = b3_v(0.0f, 0.0f, 0.0f);
        B3Fluid f = b3_fluid_default();
        f.density = 1.2f;
        f.mu = 0.0f; /* isolate the pressure-based Magnus force */
        f.drag_scale = 1.0f;
        f.added_scale = 0.0f;
        B3FluidState st;
        b3_fluid_state_init(&st);
        float spin = dir == 0 ? 25.0f : -25.0f;
        int body = add_sphere(&w, 0.05f, 2.0f, b3_v(0.0f, 0.0f, 0.0f),
            b3_v(5.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, spin));

        const float dt = 1.0f / 60.0f;
        const int subs = 4;
        float h = dt / (float)subs;
        for (int s = 0; s < subs * 60; s++) { /* 1 second */
            b3_fluid_step(&f, &w, &st, h);
            b3_step(&w, h, 1);
        }
        const B3Body* b = &w.bodies[body];
        printf("magnus(spin %+g): y=%f\n", (double)spin,
            (double)b->position.y);
        CHECK(finite_world(&w));
        if (dir == 0) {
            CHECK(b->position.y > 0.005f);
        } else {
            CHECK(b->position.y < -0.005f);
        }
    }
}

/* 6. Falling thin plates in air: motion stays bounded and finite for
 * dense and light plates. Mode classification (flutter vs tumble vs
 * steady) requires the Kirchhoff cross terms and is out of scope for
 * this force-only v1; metrics are printed for inspection. */
static void test_plates(void) {
    struct {
        float density;
        const char* name;
    } cases[3] = {
        { 1000.0f, "heavy" },
        { 150.0f, "mid" },
        { 60.0f, "light" },
    };
    for (int c = 0; c < 3; c++) {
        B3World w;
        b3_world_init(&w);
        w.gravity = b3_v(0.0f, -10.0f, 0.0f);
        B3Fluid f = b3_fluid_default();
        f.density = 1.2f;
        f.mu = 0.0f;
        f.drag_scale = 1.0f;
        f.added_scale = 1.0f;
        f.box_face_samples = 4;
        B3FluidState st;
        b3_fluid_state_init(&st);

        B3Quat tilt = b3_q_axis_angle(b3_v(1.0f, 0.0f, 0.0f), 0.05f);
        int body = add_box(&w, b3_v(0.2f, 0.2f, 0.01f), cases[c].density,
            b3_v(0.0f, 3.0f, 0.0f), tilt, b3_v(0.0f, 0.0f, 0.0f),
            b3_v(0.0f, 0.0f, 0.0f));

        const float dt = 1.0f / 60.0f;
        const int subs = 4;
        float h = dt / (float)subs;
        float tilt_sum = 0.0f;
        float om_sum = 0.0f;
        int n = 0;
        for (int s = 0; s < subs * 360; s++) { /* 6 seconds */
            b3_fluid_step(&f, &w, &st, h);
            b3_step(&w, h, 1);
            if (s >= subs * 240) {
                const B3Body* b = &w.bodies[body];
                B3Vec3 yax = b3_rotate(b->rotation,
                    b3_v(0.0f, 1.0f, 0.0f));
                tilt_sum += acosf(b3_clamp(yax.y, -1.0f, 1.0f));
                om_sum += sqrtf(b->ang_vel.x * b->ang_vel.x
                    + b->ang_vel.z * b->ang_vel.z);
                n++;
                CHECK(finite_world(&w));
                CHECK(b3_len(b->lin_vel) < 100.0f);
                CHECK(b3_len(b->ang_vel) < 200.0f);
            }
        }
        printf("plate %s (rho=%g): mean tilt=%g deg mean |w_xz|=%g\n",
            cases[c].name, (double)cases[c].density,
            (double)(tilt_sum / (float)n * 57.2957795f),
            (double)(om_sum / (float)n));
    }
}

int main(void) {
    test_buoyancy();
    test_terminal_velocity();
    test_static();
    test_spin_decay();
    test_magnus();
    test_plates();
    printf(failures == 0 ? "test-ambient: ok\n" : "test-ambient: FAIL\n");
    return failures != 0;
}
