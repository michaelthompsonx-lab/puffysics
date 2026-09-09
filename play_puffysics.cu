/* Rigid capability viewer: rolling, grains, stack, hinges, springs, motors, loops.
 *   g++ -O2 -I. -Iocean/puffysics -Iraylib-5.5_linux_amd64/include -x c++ \
 *       ocean/puffysics/play_puffysics.cu -o /tmp/play-puffysics \
 *       -Lraylib-5.5_linux_amd64/lib -lraylib -lm -lpthread -ldl
 * Keys: 1-8 scene, SPACE relaunch, P pause, N step, Q quit.
 */
#define B3_MAX_BODIES 64
#define B3_MAX_SHAPES 64
#define B3_MAX_CONTACTS 256
#define B3_MAX_JOINTS 16
#define B3_JOINT_ITERS 8
#include "puffysics.cuh"
#include "play_draw.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

enum {
    SCENE_ROLLING = 1,
    SCENE_GRAINS,
    SCENE_STACK,
    SCENE_PENDULUM,
    SCENE_DOUBLE,
    SCENE_LEG,
    SCENE_CRANK,
    SCENE_LOOP,
    SCENE_LAST = SCENE_LOOP
};

typedef struct Demo {
    B3World w;
    int scene;
    int focus[8];
    int nfocus;
} Demo;

static int add_ground(B3World* w, float half, float friction, float rolling) {
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, -0.05f, 0.0f);
    int id = b3_create_body(w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = friction;
    sd.rolling = rolling;
    sd.restitution = 0.0f;
    b3_create_box(w, id, b3_v(half, 0.05f, half), &sd);
    b3_set_inertial(w, id, 0.0f, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f));
    return id;
}

static int add_sphere(B3World* w, B3Vec3 pos, float r, float density,
        float friction, float rolling, float rest) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = pos;
    int id = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = density;
    sd.friction = friction;
    sd.rolling = rolling;
    sd.restitution = rest;
    b3_create_sphere(w, id, b3_v(0.0f, 0.0f, 0.0f), r, &sd);
    b3_finalize_mass(w, id);
    return id;
}

static int add_box(B3World* w, B3Vec3 pos, B3Vec3 half, float density,
        float friction, float rest) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = pos;
    int id = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = density;
    sd.friction = friction;
    sd.restitution = rest;
    b3_create_box(w, id, half, &sd);
    b3_finalize_mass(w, id);
    return id;
}

static int add_capsule(B3World* w, B3Vec3 pos, B3Quat rot, float half,
        float r, float density, float friction) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = pos;
    bd.rotation = rot;
    int id = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = density;
    sd.friction = friction;
    sd.restitution = 0.05f;
    b3_create_capsule(w, id, half, r, &sd);
    b3_finalize_mass(w, id);
    return id;
}

static int add_static_box(B3World* w, B3Vec3 pos, B3Vec3 half) {
    B3BodyDef bd = b3_default_body();
    bd.position = pos;
    int id = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.6f;
    b3_create_box(w, id, half, &sd);
    b3_set_inertial(w, id, 0.0f, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f));
    return id;
}

static float hinge_gap(const B3World* w, int ji) {
    const B3Joint* j = &w->joints[ji];
    const B3Body* a = &w->bodies[j->body_a];
    const B3Body* b = &w->bodies[j->body_b];
    B3Vec3 pa = b3_xf_point(a->position, a->rotation, j->local_anchor_a);
    B3Vec3 pb = b3_xf_point(b->position, b->rotation, j->local_anchor_b);
    return b3_len(b3_sub(pb, pa));
}

static void setup_scene(Demo* d, int scene) {
    memset(d, 0, sizeof(*d));
    b3_world_init(&d->w);
    d->w.gravity = b3_v(0.0f, -10.0f, 0.0f);
    d->scene = scene;

    if (scene == SCENE_ROLLING) {
        add_ground(&d->w, 6.0f, 0.0f, 0.0f);
        int left = add_sphere(&d->w, b3_v(-0.8f, 0.2f, 0.0f), 0.2f,
            1000.0f, 0.0f, 0.0f, 0.0f);
        int right = add_sphere(&d->w, b3_v(0.8f, 0.2f, 0.0f), 0.2f,
            1000.0f, 0.0f, 0.4f, 0.0f);
        d->w.bodies[left].ang_vel = b3_v(4.0f, 0.0f, 0.0f);
        d->w.bodies[right].ang_vel = b3_v(4.0f, 0.0f, 0.0f);
        d->focus[d->nfocus++] = left;
        d->focus[d->nfocus++] = right;
    } else if (scene == SCENE_GRAINS) {
        add_ground(&d->w, 4.0f, 0.5f, 0.12f);
        float r = 0.12f;
        int n = 0;
        for (int j = 0; j < 3; j++) {
            for (int i = 0; i < 5; i++) {
                for (int k = 0; k < 2; k++) {
                    float x = -0.70f + 0.32f * (float)i + 0.04f * (float)j;
                    float y = 0.35f + 0.28f * (float)j;
                    float z = -0.22f + 0.28f * (float)k;
                    int id = add_sphere(&d->w, b3_v(x, y, z), r,
                        800.0f, 0.45f, 0.15f, 0.04f);
                    if (n < 8) {
                        d->focus[d->nfocus++] = id;
                    }
                    n++;
                }
            }
        }
    } else if (scene == SCENE_STACK) {
        add_ground(&d->w, 4.0f, 0.7f, 0.0f);
        B3Vec3 half = b3_v(0.18f, 0.10f, 0.18f);
        for (int i = 0; i < 8; i++) {
            float y = 0.11f + 0.22f * (float)i;
            float x = (i & 1) ? 0.015f : -0.015f;
            int id = add_box(&d->w, b3_v(x, y, 0.0f), half, 400.0f, 0.7f, 0.0f);
            d->focus[d->nfocus++] = id;
        }
    } else if (scene == SCENE_PENDULUM) {
        add_ground(&d->w, 4.0f, 0.4f, 0.0f);
        int pivot = add_static_box(&d->w, b3_v(0.0f, 0.85f, 0.0f),
            b3_v(0.04f, 0.04f, 0.04f));
        int bob = add_sphere(&d->w, b3_v(0.95f, 0.85f, 0.0f), 0.10f,
            1000.0f, 0.4f, 0.0f, 0.0f);
        b3_create_revolute(&d->w, pivot, bob,
            b3_v(0.0f, 0.0f, 0.0f), b3_v(-0.95f, 0.0f, 0.0f),
            b3_v(0.0f, 0.0f, 1.0f));
        d->focus[d->nfocus++] = bob;
    } else if (scene == SCENE_DOUBLE) {
        add_ground(&d->w, 4.0f, 0.4f, 0.0f);
        int pivot = add_static_box(&d->w, b3_v(0.0f, 0.90f, 0.0f),
            b3_v(0.04f, 0.04f, 0.04f));
        float L = 0.70f;
        int mid = add_sphere(&d->w, b3_v(L, 0.90f, 0.0f), 0.08f,
            1000.0f, 0.3f, 0.0f, 0.0f);
        int tip = add_sphere(&d->w, b3_v(2.0f * L, 0.90f, 0.0f), 0.08f,
            1000.0f, 0.3f, 0.0f, 0.0f);
        b3_create_revolute(&d->w, pivot, mid,
            b3_v(0.0f, 0.0f, 0.0f), b3_v(-L, 0.0f, 0.0f),
            b3_v(0.0f, 0.0f, 1.0f));
        b3_create_revolute(&d->w, mid, tip,
            b3_v(0.0f, 0.0f, 0.0f), b3_v(-L, 0.0f, 0.0f),
            b3_v(0.0f, 0.0f, 1.0f));
        d->focus[d->nfocus++] = mid;
        d->focus[d->nfocus++] = tip;
    } else if (scene == SCENE_LEG) {
        add_ground(&d->w, 4.0f, 0.7f, 0.15f);
        int hip = add_static_box(&d->w, b3_v(0.0f, 1.35f, 0.0f),
            b3_v(0.05f, 0.05f, 0.05f));
        int thigh = add_capsule(&d->w, b3_v(0.0f, 1.10f, 0.0f),
            b3_q_id(), 0.18f, 0.06f, 1000.0f, 0.6f);
        int shin = add_capsule(&d->w, b3_v(0.0f, 0.68f, 0.0f),
            b3_q_id(), 0.16f, 0.05f, 1000.0f, 0.6f);
        int jh = b3_create_revolute(&d->w, hip, thigh,
            b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.22f, 0.0f),
            b3_v(1.0f, 0.0f, 0.0f));
        int jk = b3_create_revolute(&d->w, thigh, shin,
            b3_v(0.0f, -0.22f, 0.0f), b3_v(0.0f, 0.20f, 0.0f),
            b3_v(1.0f, 0.0f, 0.0f));
        b3_joint_enable_spring(&d->w, jh, 1);
        b3_joint_set_spring(&d->w, jh, 0.35f, 8.0f, 0.7f);
        b3_joint_enable_limit(&d->w, jk, 1);
        b3_joint_set_limits(&d->w, jk, 0.0f, 2.0f);
        d->focus[d->nfocus++] = thigh;
        d->focus[d->nfocus++] = shin;
    } else if (scene == SCENE_CRANK) {
        add_ground(&d->w, 4.0f, 0.45f, 0.0f);
        int hub = add_static_box(&d->w, b3_v(0.0f, 0.62f, 0.0f),
            b3_v(0.04f, 0.04f, 0.04f));
        float half = 0.42f;
        B3Quat rot = b3_q_axis_angle(b3_v(0.0f, 0.0f, 1.0f), -0.5f * B3_PI);
        int arm = add_capsule(&d->w, b3_v(half, 0.62f, 0.0f), rot,
            half, 0.045f, 600.0f, 0.2f);
        d->w.bodies[arm].linear_damping = 0.15f;
        d->w.bodies[arm].angular_damping = 0.25f;
        int j = b3_create_revolute(&d->w, hub, arm,
            b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, -half, 0.0f),
            b3_v(0.0f, 1.0f, 0.0f));
        b3_joint_enable_motor(&d->w, j, 1);
        b3_joint_set_motor(&d->w, j, 1.2f, 1.8f);
        B3Vec3 bh = b3_v(0.10f, 0.10f, 0.10f);
        int box0 = add_box(&d->w, b3_v(0.95f, 0.12f, 0.55f), bh,
            180.0f, 0.45f, 0.02f);
        int box1 = add_box(&d->w, b3_v(0.95f, 0.12f, -0.55f), bh,
            180.0f, 0.45f, 0.02f);
        d->w.bodies[box0].linear_damping = 0.6f;
        d->w.bodies[box1].linear_damping = 0.6f;
        d->focus[d->nfocus++] = arm;
        d->focus[d->nfocus++] = box0;
        d->focus[d->nfocus++] = box1;
    } else if (scene == SCENE_LOOP) {
        int ground = add_static_box(&d->w, b3_v(0.0f, 0.20f, 0.0f),
            b3_v(0.05f, 0.05f, 0.05f));
        float L = 0.55f;
        int a = add_sphere(&d->w, b3_v(L, 0.20f, 0.0f), 0.07f,
            1000.0f, 0.2f, 0.0f, 0.0f);
        int b = add_sphere(&d->w, b3_v(L, 0.20f + L, 0.0f), 0.07f,
            1000.0f, 0.2f, 0.0f, 0.0f);
        int c = add_sphere(&d->w, b3_v(0.0f, 0.20f + L, 0.0f), 0.07f,
            1000.0f, 0.2f, 0.0f, 0.0f);
        b3_create_revolute(&d->w, ground, a,
            b3_v(0.0f, 0.0f, 0.0f), b3_v(-L, 0.0f, 0.0f),
            b3_v(0.0f, 0.0f, 1.0f));
        b3_create_revolute(&d->w, a, b,
            b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, -L, 0.0f),
            b3_v(0.0f, 0.0f, 1.0f));
        b3_create_revolute(&d->w, ground, c,
            b3_v(0.0f, L, 0.0f), b3_v(0.0f, 0.0f, 0.0f),
            b3_v(0.0f, 0.0f, 1.0f));
        b3_create_revolute(&d->w, c, b,
            b3_v(0.0f, 0.0f, 0.0f), b3_v(-L, 0.0f, 0.0f),
            b3_v(0.0f, 0.0f, 1.0f));
        d->w.bodies[a].ang_vel = b3_v(0.0f, 0.0f, 1.6f);
        d->focus[d->nfocus++] = a;
        d->focus[d->nfocus++] = b;
        d->focus[d->nfocus++] = c;
    }
}

static void draw_world(const Demo* d, PlayLook* look) {
    BeginShaderMode(look->lit);
    play_draw_floor(10.0f);
    for (int i = 0; i < d->w.shape_count; i++) {
        const B3Shape* s = &d->w.shapes[i];
        const B3Body* b = &d->w.bodies[s->body];
        int dyn = b->type == B3_DYNAMIC;
        if (!dyn && s->type == B3_BOX && s->half.x > 1.5f) {
            continue;
        }
        Color col = play_tint_body(look, s->body,
            dyn ? play_palette(s->body) : (Color){48, 54, 64, 255});
        if (s->type == B3_SPHERE) {
            if (dyn) {
                play_draw_shadow(b->center, s->radius);
                play_draw_sphere(b->center, s->radius, col);
            } else {
                play_draw_sphere(b->center, s->radius, col);
            }
        } else if (s->type == B3_CAPSULE) {
            if (dyn) {
                play_draw_shadow(b->center, s->radius);
            }
            play_draw_capsule(b, s->half.y, s->radius, col);
        } else if (s->type == B3_BOX) {
            Color fill = dyn ? col : (Color){40, 46, 56, 255};
            play_draw_box(b->center, b->rotation, s->half, fill,
                (Color){255, 255, 255, 0});
        }
    }
    EndShaderMode();

    for (int i = 0; i < d->w.shape_count; i++) {
        const B3Shape* s = &d->w.shapes[i];
        const B3Body* b = &d->w.bodies[s->body];
        if (b->type == B3_DYNAMIC && s->type == B3_SPHERE) {
            play_draw_spin_tick(b, s->radius);
        }
    }
    for (int i = 0; i < d->w.joint_count; i++) {
        const B3Joint* j = &d->w.joints[i];
        const B3Body* a = &d->w.bodies[j->body_a];
        const B3Body* b = &d->w.bodies[j->body_b];
        B3Vec3 pa = b3_xf_point(a->position, a->rotation, j->local_anchor_a);
        B3Vec3 pb = b3_xf_point(b->position, b->rotation, j->local_anchor_b);
        Color rod = (Color){250, 210, 90, 255};
        play_draw_joint_rod(a->center, b->center, rod);
        DrawSphereEx(play_v3(pa), 0.022f, 8, 8, rod);
        DrawSphereEx(play_v3(pb), 0.018f, 8, 8, (Color){250, 210, 90, 180});
    }
}

static const char* scene_name(int scene) {
    switch (scene) {
    case SCENE_ROLLING: return "rolling resistance";
    case SCENE_GRAINS: return "grain pile";
    case SCENE_STACK: return "box stack";
    case SCENE_PENDULUM: return "hinge + floor";
    case SCENE_DOUBLE: return "double pendulum (Delassus)";
    case SCENE_LEG: return "spring / limit leg";
    case SCENE_CRANK: return "motor crank";
    case SCENE_LOOP: return "4-bar loop";
    default: return "?";
    }
}

static const char* scene_blurb(int scene) {
    switch (scene) {
    case SCENE_ROLLING:
        return "friction=0, both spin wx=4. Left rolling=0 keeps spinning; right rolling=0.4 damps in place.";
    case SCENE_GRAINS:
        return "Soft Step sphere contacts. Independent 1/m — no joints.";
    case SCENE_STACK:
        return "Frictional box manifolds. Warm-started contacts keep the tower from exploding.";
    case SCENE_PENDULUM:
        return "Revolute + floor contact. Delassus on the jointed world keeps the hinge tight.";
    case SCENE_DOUBLE:
        return "Tip hits the plane. Mid-link should start rotating; hinge gap stays small.";
    case SCENE_LEG:
        return "Capsules, hip spring, knee limit. Same joint GS as motors/welds.";
    case SCENE_CRANK:
        return "Raised yaw motor. Right-click a box to shove it into the spinner.";
    case SCENE_LOOP:
        return "Closed 4-bar. Tree-dual declines the cycle; joint GS + cut rows keep it closed.";
    default:
        return "";
    }
}

static void format_hud(const Demo* d, char* line, size_t n) {
    if (d->scene == SCENE_ROLLING && d->nfocus >= 2) {
        snprintf(line, n,
            "wx  left(rolling=0)=%.2f   right(rolling=0.4)=%.2f   x_right=%.3f",
            d->w.bodies[d->focus[0]].ang_vel.x,
            d->w.bodies[d->focus[1]].ang_vel.x,
            d->w.bodies[d->focus[1]].center.x);
    } else if (d->scene == SCENE_PENDULUM && d->w.joint_count >= 1) {
        snprintf(line, n, "hinge gap=%.2e   com.y=%.3f   contacts=%d",
            hinge_gap(&d->w, 0),
            d->w.bodies[d->focus[0]].center.y,
            d->w.contact_count);
    } else if (d->scene == SCENE_DOUBLE && d->w.joint_count >= 2
            && d->nfocus >= 2) {
        snprintf(line, n,
            "gap0=%.2e  gap1=%.2e   mid wz=%.3f   tip y=%.3f   contacts=%d",
            hinge_gap(&d->w, 0), hinge_gap(&d->w, 1),
            d->w.bodies[d->focus[0]].ang_vel.z,
            d->w.bodies[d->focus[1]].center.y,
            d->w.contact_count);
    } else if (d->scene == SCENE_LEG && d->w.joint_count >= 2) {
        snprintf(line, n,
            "hip gap=%.2e  knee gap=%.2e  knee angle=%.2f  shin.y=%.3f",
            hinge_gap(&d->w, 0), hinge_gap(&d->w, 1),
            b3_joint_angle(&d->w, 1),
            d->w.bodies[d->focus[1]].center.y);
    } else if (d->scene == SCENE_CRANK && d->nfocus >= 1) {
        snprintf(line, n, "arm wy=%.2f   contacts=%d",
            d->w.bodies[d->focus[0]].ang_vel.y, d->w.contact_count);
    } else if (d->scene == SCENE_LOOP && d->w.joint_count >= 4) {
        float gmax = 0.0f;
        for (int i = 0; i < d->w.joint_count; i++) {
            float g = hinge_gap(&d->w, i);
            if (g > gmax) gmax = g;
        }
        snprintf(line, n, "max hinge gap=%.2e   joints=%d",
            gmax, d->w.joint_count);
    } else {
        snprintf(line, n, "bodies=%d  contacts=%d  joints=%d",
            d->w.body_count, d->w.contact_count, d->w.joint_count);
    }
}

#ifdef PLAY_HEADLESS
int main(void) {
    int fail = 0;
    for (int scene = 1; scene <= SCENE_LAST; scene++) {
        Demo d;
        setup_scene(&d, scene);
        for (int i = 0; i < 120; i++) {
            b3_step(&d.w, 1.0f / 60.0f, 4);
        }
        for (int b = 0; b < d.w.body_count; b++) {
            B3Vec3 c = d.w.bodies[b].center;
            if (!isfinite(c.x) || !isfinite(c.y) || !isfinite(c.z)) {
                printf("FAIL scene %d body %d nan\n", scene, b);
                fail = 1;
            }
        }
        char line[256];
        format_hud(&d, line, sizeof(line));
        printf("  %d %s | %s\n", scene, scene_name(scene), line);
        if (scene == SCENE_GRAINS && d.nfocus >= 1) {
            Ray ray;
            ray.position = (Vector3){-2.0f, 1.0f, 0.0f};
            ray.direction = (Vector3){1.0f, -0.2f, 0.0f};
            int id = d.focus[0];
            float vx0 = d.w.bodies[id].lin_vel.x;
            play_poke_body(&d.w, id, ray, 3.0f);
            if (!(d.w.bodies[id].lin_vel.x > vx0 + 1.0f)) {
                printf("FAIL poke vx %g -> %g\n",
                    vx0, d.w.bodies[id].lin_vel.x);
                fail = 1;
            }
        }
        if (scene == SCENE_ROLLING && d.nfocus >= 2) {
            float wl = fabsf(d.w.bodies[d.focus[0]].ang_vel.x);
            float wr = fabsf(d.w.bodies[d.focus[1]].ang_vel.x);
            float xr = fabsf(d.w.bodies[d.focus[1]].center.x - 0.8f);
            if (!(wl > 2.0f && wr < 1.0f && xr < 0.35f)) {
                printf("FAIL rolling wx L=%g R=%g dx=%g\n", wl, wr, xr);
                fail = 1;
            }
        }
        if (scene == SCENE_CRANK && d.nfocus >= 1 && d.w.joint_count >= 1) {
            float wy = fabsf(d.w.bodies[d.focus[0]].ang_vel.y);
            float g = hinge_gap(&d.w, 0);
            if (!(wy < 3.0f && g < 0.04f)) {
                printf("FAIL crank wy=%g gap=%g\n", wy, g);
                fail = 1;
            }
        }
        if (scene == SCENE_STACK && d.nfocus >= 8) {
            float y = d.w.bodies[d.focus[7]].center.y;
            if (!(y > 1.0f && y < 2.2f)) {
                printf("FAIL stack top y=%g\n", y);
                fail = 1;
            }
        }
        if ((scene == SCENE_PENDULUM || scene == SCENE_DOUBLE
                || scene == SCENE_LOOP) && d.w.joint_count > 0) {
            float gmax = 0.0f;
            for (int i = 0; i < d.w.joint_count; i++) {
                float g = hinge_gap(&d.w, i);
                if (g > gmax) gmax = g;
            }
            if (gmax > 0.08f) {
                printf("FAIL scene %d hinge gap %g\n", scene, gmax);
                fail = 1;
            }
        }
    }
    printf(fail ? "play-puffysics headless: FAIL\n"
        : "play-puffysics headless: OK\n");
    return fail;
}
#else
int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "Puffysics | rigid capability viewer");
    if (!IsWindowReady()) {
        fprintf(stderr, "puffysics: GLFW/X11/GLX window initialization failed\n");
        return 1;
    }
    SetTargetFPS(60);
    SetExitKey(KEY_Q);

    PlayLook look = play_look_init();
    Demo d;
    setup_scene(&d, SCENE_ROLLING);
    float cam_yaw = 1.05f;
    float cam_pitch = 0.42f;
    float cam_dist = 5.5f;
    Vector2 prev = GetMousePosition();
    int paused = 0;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_ONE)) setup_scene(&d, SCENE_ROLLING);
        if (IsKeyPressed(KEY_TWO)) setup_scene(&d, SCENE_GRAINS);
        if (IsKeyPressed(KEY_THREE)) setup_scene(&d, SCENE_STACK);
        if (IsKeyPressed(KEY_FOUR)) setup_scene(&d, SCENE_PENDULUM);
        if (IsKeyPressed(KEY_FIVE)) setup_scene(&d, SCENE_DOUBLE);
        if (IsKeyPressed(KEY_SIX)) setup_scene(&d, SCENE_LEG);
        if (IsKeyPressed(KEY_SEVEN)) setup_scene(&d, SCENE_CRANK);
        if (IsKeyPressed(KEY_EIGHT)) setup_scene(&d, SCENE_LOOP);
        if (IsKeyPressed(KEY_SPACE)) setup_scene(&d, d.scene);
        if (IsKeyPressed(KEY_P)) paused = !paused;

        Camera3D camera = {0};
        camera.up = (Vector3){0.0f, 1.0f, 0.0f};
        camera.fovy = 42.0f;
        camera.projection = CAMERA_PERSPECTIVE;
        camera.target = (Vector3){0.0f, 0.55f, 0.0f};
        camera.position = (Vector3){
            camera.target.x + cam_dist * cosf(cam_pitch) * cosf(cam_yaw),
            camera.target.y + cam_dist * sinf(cam_pitch),
            camera.target.z + cam_dist * cosf(cam_pitch) * sinf(cam_yaw)
        };

        Vector2 mouse = GetMousePosition();
        Ray ray = GetScreenToWorldRay(mouse, camera);
        look.hover = play_pick_body(&d.w, ray);
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && look.hover >= 0) {
            play_poke_body(&d.w, look.hover, ray, 3.2f);
            look.poke = look.hover;
            look.poke_age = 0.28f;
        } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && look.hover >= 0) {
            play_poke_body(&d.w, look.hover, ray, 0.42f);
            look.poke = look.hover;
            look.poke_age = 0.16f;
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            cam_yaw -= (mouse.x - prev.x) * 0.006f;
            cam_pitch += (mouse.y - prev.y) * 0.005f;
            if (cam_pitch > 1.45f) cam_pitch = 1.45f;
            if (cam_pitch < 0.08f) cam_pitch = 0.08f;
        }
        prev = mouse;
        cam_dist *= expf(-GetMouseWheelMove() * 0.12f);
        if (cam_dist < 1.5f) cam_dist = 1.5f;
        if (cam_dist > 16.0f) cam_dist = 16.0f;

        int ticks = paused ? (IsKeyPressed(KEY_N) ? 1 : 0) : 1;
        for (int t = 0; t < ticks; t++) {
            b3_step(&d.w, 1.0f / 60.0f, 4);
        }
        play_look_tick(&look, 1.0f / 60.0f);
        play_look_set_view(&look, camera.position);

        BeginDrawing();
        ClearBackground((Color){10, 13, 18, 255});
        BeginMode3D(camera);
        draw_world(&d, &look);
        EndMode3D();
        DrawText(TextFormat("scene %d: %s", d.scene, scene_name(d.scene)),
            12, 12, 20, (Color){220, 224, 230, 255});
        DrawText(scene_blurb(d.scene), 12, 36, 16, (Color){150, 156, 168, 255});
        DrawText("1-8 scenes   SPACE relaunch   P pause   N step   RMB poke   Q quit",
            12, 56, 16, (Color){150, 156, 168, 255});
        char line[256];
        format_hud(&d, line, sizeof(line));
        DrawText(line, 12, 80, 16, (Color){250, 210, 90, 255});
        EndDrawing();
    }
    play_look_free(&look);
    CloseWindow();
    return 0;
}
#endif
