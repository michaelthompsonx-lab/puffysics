// Reference traces from Erin Catto's Box3D, same scenes as the CUDA port.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static double now_s(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_nsec * 1e-9 + (double)t.tv_sec;
}

static b3WorldId make_world(void) {
    b3WorldDef d = b3DefaultWorldDef();
    d.gravity = (b3Vec3){0.0f, -10.0f, 0.0f};
    d.enableSleep = false;
    d.enableContinuous = false;
    d.workerCount = 1;
    return b3CreateWorld(&d);
}

static b3ShapeDef shape_def(float density, float friction, float rest) {
    b3ShapeDef d = b3DefaultShapeDef();
    d.density = density;
    d.baseMaterial.friction = friction;
    d.baseMaterial.restitution = rest;
    d.baseMaterial.rollingResistance = 0.0f;
    return d;
}

static void add_box(b3BodyId body, float hx, float hy, float hz,
        float density, float friction, float rest) {
    b3ShapeDef d = shape_def(density, friction, rest);
    b3BoxHull hull = b3MakeBoxHull(hx, hy, hz);
    b3CreateHullShape(body, &d, &hull.base);
}

static void add_sphere(b3BodyId body, float r, float density,
        float friction, float rest) {
    b3ShapeDef d = shape_def(density, friction, rest);
    b3Sphere s = {{0.0f, 0.0f, 0.0f}, r};
    b3CreateSphereShape(body, &d, &s);
}

static void dump_body(const char* scene, int step, int i, b3BodyId id) {
    b3Pos p = b3Body_GetPosition(id);
    b3Vec3 v = b3Body_GetLinearVelocity(id);
    printf("T %s %d %d %.6f %.6f %.6f %.6f %.6f %.6f\n",
        scene, step, i, p.x, p.y, p.z, v.x, v.y, v.z);
}

static void scene_free_fall(void) {
    b3WorldId w = make_world();
    b3BodyDef bd = b3DefaultBodyDef();
    bd.type = b3_dynamicBody;
    bd.position = (b3Vec3){0.0f, 10.0f, 0.0f};
    bd.enableSleep = false;
    b3BodyId id = b3CreateBody(w, &bd);
    add_sphere(id, 0.5f, 1.0f, 0.0f, 0.0f);
    dump_body("free_fall", 0, 0, id);
    for (int i = 1; i <= 30; i++) {
        b3World_Step(w, 1.0f / 60.0f, 4);
        dump_body("free_fall", i, 0, id);
    }
    b3DestroyWorld(w);
}

static void scene_rest(void) {
    b3WorldId w = make_world();
    b3BodyDef gd = b3DefaultBodyDef();
    gd.position = (b3Vec3){0.0f, -1.0f, 0.0f};
    gd.enableSleep = false;
    b3BodyId ground = b3CreateBody(w, &gd);
    add_box(ground, 50.0f, 1.0f, 50.0f, 0.0f, 0.3f, 0.0f);

    b3BodyDef bd = b3DefaultBodyDef();
    bd.type = b3_dynamicBody;
    bd.position = (b3Vec3){0.0f, 4.0f, 0.0f};
    bd.enableSleep = false;
    b3BodyId box = b3CreateBody(w, &bd);
    add_box(box, 1.0f, 1.0f, 1.0f, 1.0f, 0.3f, 0.0f);

    dump_body("rest", 0, 1, box);
    for (int i = 1; i <= 180; i++) {
        b3World_Step(w, 1.0f / 60.0f, 4);
        if (i == 60 || i == 120 || i == 180) {
            dump_body("rest", i, 1, box);
        }
    }
    b3DestroyWorld(w);
}

static void scene_bounce(void) {
    b3WorldId w = make_world();
    b3BodyDef gd = b3DefaultBodyDef();
    gd.position = (b3Vec3){0.0f, -1.0f, 0.0f};
    gd.enableSleep = false;
    b3BodyId ground = b3CreateBody(w, &gd);
    add_box(ground, 20.0f, 1.0f, 20.0f, 0.0f, 0.0f, 0.8f);

    b3BodyDef bd = b3DefaultBodyDef();
    bd.type = b3_dynamicBody;
    bd.position = (b3Vec3){0.0f, 3.0f, 0.0f};
    bd.enableSleep = false;
    b3BodyId sph = b3CreateBody(w, &bd);
    add_sphere(sph, 0.5f, 1.0f, 0.0f, 0.8f);

    float min_y = 3.0f;
    float peak = -1.0e9f;
    int seen = 0;
    for (int i = 1; i <= 180; i++) {
        b3World_Step(w, 1.0f / 60.0f, 4);
        b3Pos p = b3Body_GetPosition(sph);
        if (p.y < min_y) {
            min_y = p.y;
        }
        if (min_y < 1.0f && p.y > peak) {
            peak = p.y;
            seen = 1;
        }
        if (i == 30 || i == 60 || i == 90 || i == 180) {
            dump_body("bounce", i, 1, sph);
        }
    }
    printf("S bounce min_y %.6f peak %.6f seen %d\n",
        min_y, seen ? peak : 0.0f, seen);
    b3DestroyWorld(w);
}

static void scene_sphere_hit(void) {
    b3WorldId w = make_world();
    b3World_SetGravity(w, (b3Vec3){0.0f, 0.0f, 0.0f});

    b3BodyDef a = b3DefaultBodyDef();
    a.type = b3_dynamicBody;
    a.position = (b3Vec3){-1.5f, 0.0f, 0.0f};
    a.linearVelocity = (b3Vec3){2.0f, 0.0f, 0.0f};
    a.enableSleep = false;
    b3BodyId ia = b3CreateBody(w, &a);
    add_sphere(ia, 0.5f, 1.0f, 0.0f, 1.0f);

    b3BodyDef b = b3DefaultBodyDef();
    b.type = b3_dynamicBody;
    b.position = (b3Vec3){1.5f, 0.0f, 0.0f};
    b.enableSleep = false;
    b3BodyId ib = b3CreateBody(w, &b);
    add_sphere(ib, 0.5f, 1.0f, 0.0f, 1.0f);

    for (int i = 1; i <= 90; i++) {
        b3World_Step(w, 1.0f / 60.0f, 4);
        if (i == 30 || i == 60 || i == 90) {
            dump_body("sphere_hit", i, 0, ia);
            dump_body("sphere_hit", i, 1, ib);
        }
    }
    b3DestroyWorld(w);
}

static void scene_kinematic(void) {
    b3WorldId w = make_world();
    b3BodyDef kd = b3DefaultBodyDef();
    kd.type = b3_kinematicBody;
    kd.position = (b3Vec3){0.0f, 0.0f, 0.0f};
    kd.linearVelocity = (b3Vec3){0.0f, 1.0f, 0.0f};
    kd.enableSleep = false;
    b3BodyId plat = b3CreateBody(w, &kd);
    add_box(plat, 2.0f, 0.1f, 2.0f, 0.0f, 0.8f, 0.0f);

    b3BodyDef bd = b3DefaultBodyDef();
    bd.type = b3_dynamicBody;
    bd.position = (b3Vec3){0.0f, 0.7f, 0.0f};
    bd.enableSleep = false;
    b3BodyId box = b3CreateBody(w, &bd);
    add_box(box, 0.4f, 0.4f, 0.4f, 1.0f, 0.8f, 0.0f);

    for (int i = 1; i <= 60; i++) {
        b3World_Step(w, 1.0f / 60.0f, 4);
    }
    dump_body("kinematic", 60, 0, plat);
    dump_body("kinematic", 60, 1, box);
    b3DestroyWorld(w);
}

static void scene_stack(void) {
    b3WorldId w = make_world();
    b3BodyDef gd = b3DefaultBodyDef();
    gd.position = (b3Vec3){0.0f, -0.5f, 0.0f};
    gd.enableSleep = false;
    b3BodyId ground = b3CreateBody(w, &gd);
    add_box(ground, 20.0f, 0.5f, 20.0f, 0.0f, 0.5f, 0.0f);

    b3BodyId boxes[4];
    for (int i = 0; i < 4; i++) {
        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = b3_dynamicBody;
        bd.position = (b3Vec3){0.0f, 0.5f + 1.05f * (float)i, 0.0f};
        bd.enableSleep = false;
        boxes[i] = b3CreateBody(w, &bd);
        add_box(boxes[i], 0.5f, 0.5f, 0.5f, 1.0f, 0.5f, 0.0f);
    }
    for (int s = 0; s < 240; s++) {
        b3World_Step(w, 1.0f / 60.0f, 4);
    }
    for (int i = 0; i < 4; i++) {
        dump_body("stack", 240, i, boxes[i]);
    }
    b3DestroyWorld(w);
}

static void bench_hello(int worlds, int steps) {
    b3WorldId* ids = (b3WorldId*)malloc((size_t)worlds * sizeof(b3WorldId));
    for (int i = 0; i < worlds; i++) {
        b3WorldId w = make_world();
        b3BodyDef gd = b3DefaultBodyDef();
        gd.position = (b3Vec3){0.0f, -1.0f, 0.0f};
        gd.enableSleep = false;
        b3BodyId ground = b3CreateBody(w, &gd);
        add_box(ground, 50.0f, 1.0f, 50.0f, 0.0f, 0.3f, 0.0f);
        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = b3_dynamicBody;
        bd.position = (b3Vec3){0.0f, 4.0f + 0.01f * (float)i, 0.0f};
        bd.enableSleep = false;
        b3BodyId box = b3CreateBody(w, &bd);
        add_box(box, 1.0f, 1.0f, 1.0f, 1.0f, 0.3f, 0.0f);
        ids[i] = w;
    }
    double t0 = now_s();
    for (int s = 0; s < steps; s++) {
        for (int i = 0; i < worlds; i++) {
            b3World_Step(ids[i], 1.0f / 60.0f, 4);
        }
    }
    double dt = now_s() - t0;
    double sps = (double)worlds * (double)steps / dt;
    printf("P hello worlds=%d steps=%d time=%.4fs sps=%.0f\n",
        worlds, steps, dt, sps);
    for (int i = 0; i < worlds; i++) {
        b3DestroyWorld(ids[i]);
    }
    free(ids);
}

static void bench_pile(int n, int steps) {
    b3WorldId w = make_world();
    b3BodyDef gd = b3DefaultBodyDef();
    gd.position = (b3Vec3){0.0f, -0.5f, 0.0f};
    gd.enableSleep = false;
    b3BodyId ground = b3CreateBody(w, &gd);
    add_box(ground, 20.0f, 0.5f, 20.0f, 0.0f, 0.4f, 0.0f);
    for (int i = 0; i < n; i++) {
        int xi = i % 4;
        int zi = (i / 4) % 4;
        int yi = i / 16;
        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = b3_dynamicBody;
        bd.position = (b3Vec3){
            -1.5f + 1.0f * (float)xi,
            1.0f + 1.1f * (float)yi,
            -1.5f + 1.0f * (float)zi};
        bd.enableSleep = false;
        b3BodyId id = b3CreateBody(w, &bd);
        add_sphere(id, 0.4f, 1.0f, 0.3f, 0.0f);
    }
    double t0 = now_s();
    for (int s = 0; s < steps; s++) {
        b3World_Step(w, 1.0f / 60.0f, 4);
    }
    double dt = now_s() - t0;
    printf("P pile bodies=%d steps=%d time=%.4fs sps=%.0f\n",
        n, steps, dt, (double)steps / dt);
    b3DestroyWorld(w);
}

int main(int argc, char** argv) {
    int do_bench = argc > 1;
    printf("engine box3d_ref\n");
    scene_free_fall();
    scene_rest();
    scene_bounce();
    scene_sphere_hit();
    scene_kinematic();
    scene_stack();
    if (do_bench) {
        bench_hello(1, 2000);
        bench_hello(128, 120);
        bench_pile(32, 300);
    }
    return 0;
}
