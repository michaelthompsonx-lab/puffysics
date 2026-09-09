/* Ambient-fluid visualizer: buoyancy, pressure, skin friction, added mass.
 * Full-submersion ambient.h is scaled by a waterline fraction here.
 *   g++ -O2 -I. -Iocean/puffysics -Iraylib-5.5_linux_amd64/include -x c++ \
 *       ocean/puffysics/play_fluid.cu -o /tmp/play-fluid \
 *       -Lraylib-5.5_linux_amd64/lib -lraylib -lm -lpthread -ldl
 * Keys: 1/2/3 scene, SPACE relaunch, P pause, N step, Q quit.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ambient.h"
#include "play_draw.h"

enum { SCENE_PLATES = 1, SCENE_MAGNUS = 2, SCENE_WATER = 3 };
enum { TRAIL = 240, MAX_FOCUS = 8 };

typedef struct Demo {
    B3World w;
    B3Fluid fluid;
    B3FluidState st;
    int body[MAX_FOCUS];
    int n;
    int scene;
    B3Vec3 trail[MAX_FOCUS][TRAIL];
    int trail_n[MAX_FOCUS];
    float half_x[MAX_FOCUS];
    float half_y[MAX_FOCUS];
    float half_z[MAX_FOCUS];
    float radius[MAX_FOCUS];
    int is_box[MAX_FOCUS];
    float water_y;
} Demo;

static int spawn_sphere(B3World* w, float r, float density, B3Vec3 pos,
        B3Vec3 vel, B3Vec3 spin, float lin_damp, float ang_damp) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = pos;
    bd.lin_vel = vel;
    bd.ang_vel = spin;
    bd.linear_damping = lin_damp;
    bd.angular_damping = ang_damp;
    int body = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.35f;
    sd.restitution = 0.05f;
    b3_create_sphere(w, body, b3_v(0.0f, 0.0f, 0.0f), r, &sd);
    float vol = (4.0f / 3.0f) * B3_PI * r * r * r;
    float m = density * vol;
    B3Vec3 inertia = b3_v(0.4f * m * r * r, 0.4f * m * r * r,
        0.4f * m * r * r);
    b3_set_inertial(w, body, m, b3_v(0.0f, 0.0f, 0.0f), inertia);
    return body;
}

static int spawn_box(B3World* w, B3Vec3 half, float density, B3Vec3 pos,
        B3Quat rot, B3Vec3 vel, B3Vec3 spin, float friction,
        float lin_damp, float ang_damp) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = pos;
    bd.rotation = rot;
    bd.lin_vel = vel;
    bd.ang_vel = spin;
    bd.linear_damping = lin_damp;
    bd.angular_damping = ang_damp;
    int body = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = friction;
    sd.restitution = 0.0f;
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

static void add_floor(B3World* w, float half, float y_top) {
    B3BodyDef gd = b3_default_body();
    gd.position = b3_v(0.0f, y_top - 0.06f, 0.0f);
    int id = b3_create_body(w, &gd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 0.45f;
    sd.restitution = 0.0f;
    b3_create_box(w, id, b3_v(half, 0.06f, half), &sd);
    b3_set_inertial(w, id, 0.0f, b3_v(0.0f, 0.0f, 0.0f),
        b3_v(0.0f, 0.0f, 0.0f));
}

static void remember(Demo* d, int id, int box, B3Vec3 half, float r) {
    int i = d->n++;
    d->body[i] = id;
    d->is_box[i] = box;
    d->half_x[i] = half.x;
    d->half_y[i] = half.y;
    d->half_z[i] = half.z;
    d->radius[i] = r;
    d->trail_n[i] = 0;
}

static void setup_scene(Demo* d, int scene) {
    memset(d, 0, sizeof(*d));
    b3_world_init(&d->w);
    d->w.gravity = b3_v(0.0f, -10.0f, 0.0f);
    d->fluid = b3_fluid_default();
    b3_fluid_state_init(&d->st);
    d->scene = scene;

    if (scene == SCENE_PLATES) {
        d->fluid.density = 1.2f;
        d->fluid.mu = 1.8e-5f;
        d->fluid.box_face_samples = 4;
        add_floor(&d->w, 6.0f, 0.0f);
        float density[3] = {40.0f, 150.0f, 1000.0f};
        B3Vec3 half = b3_v(0.22f, 0.012f, 0.22f);
        for (int i = 0; i < 3; i++) {
            int id = spawn_box(&d->w, half, density[i],
                b3_v(-1.1f + 1.1f * (float)i, 2.8f, 0.0f),
                b3_q_axis_angle(b3_v(1.0f, 0.0f, 0.0f), 0.18f),
                b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f), 0.25f,
                0.4f, 0.8f);
            remember(d, id, 1, half, 0.0f);
        }
    } else if (scene == SCENE_MAGNUS) {
        d->fluid.density = 1.2f;
        d->fluid.mu = 0.0f;
        add_floor(&d->w, 14.0f, 0.0f);
        float spin[3] = {28.0f, 0.0f, -28.0f};
        for (int i = 0; i < 3; i++) {
            int id = spawn_sphere(&d->w, 0.13f, 80.0f,
                b3_v(-4.2f, 1.15f, 0.55f * (float)(i - 1)),
                b3_v(7.2f, 1.6f, 0.0f),
                b3_v(0.0f, spin[i], 0.0f), 0.15f, 0.05f);
            remember(d, id, 0, b3_v(0, 0, 0), 0.13f);
        }
    } else if (scene == SCENE_WATER) {
        d->fluid.density = 998.0f;
        d->fluid.mu = 1.0e-3f;
        d->fluid.added_scale = 0.0f; /* lagged added mass + floor contacts explode */
        d->water_y = 2.2f;
        add_floor(&d->w, 3.0f, 0.0f);
        float density[4] = {500.0f, 800.0f, 1500.0f, 2500.0f};
        for (int i = 0; i < 4; i++) {
            int id = spawn_sphere(&d->w, 0.14f, density[i],
                b3_v(-1.2f + 0.80f * (float)i, 1.15f, 0.0f),
                b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f),
                3.5f, 2.0f);
            remember(d, id, 0, b3_v(0, 0, 0), 0.14f);
        }
    }
}

static float focus_half_y(const Demo* d, int i, const B3Body* b) {
    if (!d->is_box[i]) {
        return d->radius[i];
    }
    B3Vec3 hx = b3_rotate(b->rotation, b3_v(d->half_x[i], 0.0f, 0.0f));
    B3Vec3 hy = b3_rotate(b->rotation, b3_v(0.0f, d->half_y[i], 0.0f));
    B3Vec3 hz = b3_rotate(b->rotation, b3_v(0.0f, 0.0f, d->half_z[i]));
    return fabsf(hx.y) + fabsf(hy.y) + fabsf(hz.y);
}

static float submerged_frac(const Demo* d, int bi) {
    if (!(d->water_y > 0.0f)) {
        return 1.0f;
    }
    const B3Body* b = &d->w.bodies[bi];
    int fi = -1;
    for (int i = 0; i < d->n; i++) {
        if (d->body[i] == bi) {
            fi = i;
            break;
        }
    }
    if (fi < 0) {
        return 0.0f;
    }
    float r = focus_half_y(d, fi, b);
    float y = b->center.y;
    float H = d->water_y;
    if (y + r <= H) {
        return 1.0f;
    }
    if (y - r >= H) {
        return 0.0f;
    }
    if (!d->is_box[fi]) {
        float h = H - (y - r);
        return (h * h * (3.0f * r - h)) / (4.0f * r * r * r);
    }
    return (H - (y - r)) / (2.0f * r);
}

static void step_demo(Demo* d, float dt) {
    const int subs = 4;
    float h = dt / (float)subs;
    for (int s = 0; s < subs; s++) {
        for (int i = 0; i < d->w.body_count; i++) {
            d->w.bodies[i].force = b3_v(0.0f, 0.0f, 0.0f);
            d->w.bodies[i].torque = b3_v(0.0f, 0.0f, 0.0f);
        }
        if (d->water_y > 0.0f) {
            B3Fluid f = d->fluid;
            for (int bi = 0; bi < d->w.body_count; bi++) {
                f.density = d->fluid.density * submerged_frac(d, bi);
                b3_fluid_body(&f, &d->w, &d->st, bi, h);
            }
            d->st.have_prev = 1;
        } else {
            b3_fluid_step(&d->fluid, &d->w, &d->st, h);
        }
        b3_step(&d->w, h, 1);
    }
    for (int i = 0; i < d->n; i++) {
        d->trail[i][d->trail_n[i] % TRAIL] = d->w.bodies[d->body[i]].center;
        d->trail_n[i]++;
    }
}

static const char* scene_name(int scene) {
    if (scene == SCENE_PLATES) return "falling plates (air)";
    if (scene == SCENE_MAGNUS) return "Magnus curve (air)";
    return "underwater buoyancy";
}

static const char* scene_blurb(int scene) {
    if (scene == SCENE_PLATES) {
        return "Gold 40 / blue 150 / red 1000 kg/m3. Light plates couple to the air; heavy ones drop.";
    }
    if (scene == SCENE_MAGNUS) {
        return "Same throw, wy = +28 / 0 / -28. Pressure Magnus bends gold and red apart in Z.";
    }
    return "Waterline y=2.2. Gold 500 / teal 800 float; red/orange sink. Right-click hold to shove.";
}

#ifdef PLAY_HEADLESS
int main(void) {
    int fail = 0;
    for (int scene = 1; scene <= 3; scene++) {
        Demo d;
        setup_scene(&d, scene);
        for (int i = 0; i < 180; i++) {
            step_demo(&d, 1.0f / 60.0f);
        }
        for (int i = 0; i < d.n; i++) {
            B3Vec3 c = d.w.bodies[d.body[i]].center;
            B3Vec3 v = d.w.bodies[d.body[i]].lin_vel;
            int ok = isfinite(c.x) && isfinite(c.y) && isfinite(c.z)
                && isfinite(v.x) && isfinite(v.y);
            printf("  scene %d body %d y=%.3f vy=%.3f x=%.3f z=%.3f\n",
                scene, i, c.y, v.y, c.x, c.z);
            if (!ok) fail = 1;
        }
        if (scene == SCENE_WATER) {
            float y0 = d.w.bodies[d.body[0]].center.y;
            float y3 = d.w.bodies[d.body[3]].center.y;
            int through = 0;
            for (int i = 0; i < d.n; i++) {
                if (d.w.bodies[d.body[i]].center.y < -0.05f) through = 1;
            }
            int escaped = 0;
            for (int i = 0; i < d.n; i++) {
                B3Vec3 c = d.w.bodies[d.body[i]].center;
                if (c.y > 3.2f || !isfinite(c.y)) {
                    escaped = 1;
                }
            }
            if (through || escaped || !(y0 > 1.6f && y0 < 2.6f)
                    || !(y3 > 0.08f && y3 < 0.35f)) {
                printf("FAIL water: floater y=%g sinker y=%g "
                    "through=%d escaped=%d\n",
                    y0, y3, through, escaped);
                fail = 1;
            }
        }
        if (scene == SCENE_MAGNUS) {
            float z0 = d.w.bodies[d.body[0]].center.z;
            float z2 = d.w.bodies[d.body[2]].center.z;
            printf("  magnus z gold=%g red=%g\n", z0, z2);
        }
    }
    printf(fail ? "play-fluid headless: FAIL\n" : "play-fluid headless: OK\n");
    return fail;
}
#else
int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "Puffysics | ambient fluid");
    if (!IsWindowReady()) {
        fprintf(stderr, "fluid: GLFW/X11/GLX window initialization failed\n");
        return 1;
    }
    SetTargetFPS(60);
    SetExitKey(KEY_Q);

    PlayLook look = play_look_init();
    Demo d;
    setup_scene(&d, SCENE_PLATES);
    float cam_yaw = 0.95f;
    float cam_pitch = 0.38f;
    float cam_dist = 7.5f;
    Vector2 prev = GetMousePosition();
    int paused = 0;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_ONE)) setup_scene(&d, SCENE_PLATES);
        if (IsKeyPressed(KEY_TWO)) setup_scene(&d, SCENE_MAGNUS);
        if (IsKeyPressed(KEY_THREE)) setup_scene(&d, SCENE_WATER);
        if (IsKeyPressed(KEY_SPACE)) setup_scene(&d, d.scene);
        if (IsKeyPressed(KEY_P)) paused = !paused;

        Vector3 target = {0.0f, 1.1f, 0.0f};
        if (d.scene == SCENE_MAGNUS && d.n > 0) {
            B3Vec3 mid = b3_v(0, 0, 0);
            for (int i = 0; i < d.n; i++) {
                mid = b3_add(mid, d.w.bodies[d.body[i]].center);
            }
            mid = b3_mul(mid, 1.0f / (float)d.n);
            target = (Vector3){mid.x, 0.9f, 0.0f};
        } else if (d.scene == SCENE_WATER) {
            target = (Vector3){0.0f, 1.1f, 0.0f};
        }

        Camera3D camera = {0};
        camera.up = (Vector3){0.0f, 1.0f, 0.0f};
        camera.fovy = 42.0f;
        camera.projection = CAMERA_PERSPECTIVE;
        camera.target = target;
        camera.position = (Vector3){
            target.x + cam_dist * cosf(cam_pitch) * cosf(cam_yaw),
            target.y + cam_dist * sinf(cam_pitch),
            target.z + cam_dist * cosf(cam_pitch) * sinf(cam_yaw)
        };

        Vector2 mouse = GetMousePosition();
        Ray ray = GetScreenToWorldRay(mouse, camera);
        look.hover = play_pick_body(&d.w, ray);
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && look.hover >= 0) {
            play_poke_body(&d.w, look.hover, ray, 2.8f);
            look.poke = look.hover;
            look.poke_age = 0.28f;
        } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && look.hover >= 0) {
            play_poke_body(&d.w, look.hover, ray, 0.38f);
            look.poke = look.hover;
            look.poke_age = 0.16f;
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            cam_yaw -= (mouse.x - prev.x) * 0.006f;
            cam_pitch += (mouse.y - prev.y) * 0.005f;
            if (cam_pitch > 1.35f) cam_pitch = 1.35f;
            if (cam_pitch < 0.06f) cam_pitch = 0.06f;
        }
        prev = mouse;
        cam_dist *= expf(-GetMouseWheelMove() * 0.12f);
        if (cam_dist < 2.0f) cam_dist = 2.0f;
        if (cam_dist > 18.0f) cam_dist = 18.0f;

        int ticks = paused ? (IsKeyPressed(KEY_N) ? 1 : 0) : 1;
        for (int t = 0; t < ticks; t++) {
            step_demo(&d, 1.0f / 60.0f);
        }
        play_look_tick(&look, 1.0f / 60.0f);
        play_look_set_view(&look, camera.position);

        Color bg = d.scene == SCENE_WATER
            ? (Color){6, 14, 22, 255} : (Color){10, 13, 18, 255};
        BeginDrawing();
        ClearBackground(bg);
        BeginMode3D(camera);
        BeginShaderMode(look.lit);
        play_draw_floor(d.scene == SCENE_MAGNUS ? 14.0f : 8.0f);
        for (int i = 0; i < d.n; i++) {
            const B3Body* b = &d.w.bodies[d.body[i]];
            Color col = play_tint_body(&look, d.body[i], play_palette(i));
            if (d.scene == SCENE_WATER && d.water_y > b->center.y) {
                col = play_underwater_tint(col, d.water_y - b->center.y);
            }
            play_draw_shadow(b->center,
                d.is_box[i] ? d.half_x[i] : d.radius[i]);
            if (d.is_box[i]) {
                play_draw_box(b->center, b->rotation,
                    b3_v(d.half_x[i], d.half_y[i], d.half_z[i]),
                    col, (Color){255, 255, 255, 0});
            } else {
                play_draw_sphere(b->center, d.radius[i], col);
            }
        }
        EndShaderMode();
        for (int i = 0; i < d.n; i++) {
            if (!d.is_box[i]) {
                play_draw_spin_tick(&d.w.bodies[d.body[i]], d.radius[i]);
            }
            Color col = play_tint_body(&look, d.body[i], play_palette(i));
            play_draw_trail(d.trail[i], d.trail_n[i], TRAIL, col);
        }
        if (d.scene == SCENE_WATER) {
            play_draw_water(&look, 2.4f, d.water_y);
        }
        EndMode3D();
        DrawText(TextFormat("scene %d: %s", d.scene, scene_name(d.scene)),
            12, 12, 20, (Color){220, 224, 230, 255});
        DrawText(scene_blurb(d.scene), 12, 36, 16, (Color){150, 156, 168, 255});
        DrawText("1/2/3 scenes   SPACE relaunch   P pause   N step   RMB poke   Q quit",
            12, 56, 16, (Color){150, 156, 168, 255});
        char line[320];
        int off = 0;
        off += snprintf(line, sizeof(line), d.water_y > 0.0f
            ? "rho_f=%.0f  water_y=%.1f" : "rho_f=%.0f",
            d.fluid.density, d.water_y);
        for (int i = 0; i < d.n && off < (int)sizeof(line) - 40; i++) {
            const B3Body* b = &d.w.bodies[d.body[i]];
            off += snprintf(line + off, sizeof(line) - (size_t)off,
                "   %c y=%.2f vy=%+.2f", "GBRTOA"[i], b->center.y, b->lin_vel.y);
        }
        DrawText(line, 12, 80, 16, (Color){250, 210, 90, 255});
        EndDrawing();
    }
    play_look_free(&look);
    CloseWindow();
    return 0;
}
#endif
