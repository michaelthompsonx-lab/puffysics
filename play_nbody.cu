/* Isolated Plummer-softened N-body. No expansion, collisions, or fixed sun.
 * nvcc -O3 -arch=native ocean/puffysics/play_nbody.cu -o /tmp/play-nbody \
 *   -Iraylib-5.5_linux_amd64/include -Lraylib-5.5_linux_amd64/lib \
 *   -lraylib -lGL -lm -lpthread -ldl
 * Keys: 1/2/3 scenes, SPACE reset, P pause, N step, +/- rate, T trails, Q quit
 */
#include "nbody.cuh"
#ifndef PLAY_HEADLESS
#include "raylib.h"
#include "rlgl.h"
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef PLAY_HEADLESS
typedef struct Color {
    unsigned char r, g, b, a;
} Color;
typedef struct Vector3 {
    float x, y, z;
} Vector3;
#endif

#define PI 3.14159265358979323846f
#define TRAIL_COUNT 24
#define TRAIL_LENGTH 180

static Color blue = {100, 190, 255, 255};
static Color amber = {255, 171, 94, 255};
#ifndef PLAY_HEADLESS
static Color muted = {147, 164, 189, 255};
#endif
static const char *names[] = {
    "DISK ENCOUNTER",
    "BINARY + CIRCUMBINARY DISK",
    "COLD COLLAPSE"
};
#ifndef PLAY_HEADLESS
static const char *descriptions[] = {
    "Two inclined, self-gravitating disks. Tidal tails emerge from mutual gravity.",
    "A live equal-mass binary and a light, fully interacting outer disk.",
    "A cold sphere falls inward. Softening resolves the force, not physical collisions."
};
#endif

typedef struct Demo {
    NbodyPoint *p;
    NbodyVec *v, *scratch;
    Color *color;
    Vector3 trails[TRAIL_COUNT][TRAIL_LENGTH];
    int n, trail_size, trail_head, scene, cpu;
    NbodyConfig cfg;
    float dt;
    double time, initial_energy, energy, elapsed_ms;
#ifdef __CUDACC__
    NbodyGpu gpu;
#endif
} Demo;

static unsigned random_state = 0x71bc029u;

static float uniform(void) {
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return (float)(random_state >> 8) * (1.0f / 16777216.0f);
}

static double now_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}

static NbodyVec incline(NbodyVec p, float angle) {
    NbodyVec q;
    q.x = p.x;
    q.y = p.y * cosf(angle) - p.z * sinf(angle);
    q.z = p.y * sinf(angle) + p.z * cosf(angle);
    return q;
}

static void diagnostics(Demo *d) {
    d->energy = nbody_energy(d->p, d->v, d->n, d->cfg);
}

static void reset(Demo *d, int scene) {
    int n, i, group;
    int populations[2];
    float bulk, r, a, tilt, enclosed, speed, separation, speed_disk;
    float z, radius;
    double mass, px, py, pz, vx, vy, vz, m;
    NbodyVec p, v;
    d->scene = scene;
    d->time = 0;
    d->trail_size = 0;
    d->trail_head = 0;
    random_state = 0x71bc029u;
    n = d->n;
    memset(d->p, 0, (size_t)n * sizeof(*d->p));
    memset(d->v, 0, (size_t)n * sizeof(*d->v));
    for (i = 0; i < n; i++) {
        d->color[i] = blue;
    }
    if (scene == 0) {
        bulk = 1.25f;
        d->p[0].x = -6;
        d->p[0].mass = 50;
        d->v[0].z = bulk;
        d->color[0] = amber;
        d->p[1].x = 6;
        d->p[1].mass = 50;
        d->v[1].z = -bulk;
        d->color[1] = blue;
        populations[0] = (n - 1) / 2;
        populations[1] = (n - 2) / 2;
        for (i = 2; i < n; i++) {
            group = i % 2;
            r = 0.85f + 3.5f * sqrtf(uniform());
            a = 2 * PI * uniform();
            tilt = group ? -0.60f : 0.23f;
            enclosed = 50 + 8 * (r - 0.85f) * (r - 0.85f) / (3.5f * 3.5f);
            speed = sqrtf(d->cfg.gravity * enclosed * r * r
                / powf(r * r + d->cfg.softening * d->cfg.softening, 1.5f));
            p.x = r * cosf(a);
            p.y = 0.07f * (uniform() - 0.5f);
            p.z = r * sinf(a);
            p = incline(p, tilt);
            v.x = -speed * sinf(a);
            v.y = 0;
            v.z = speed * cosf(a);
            v = incline(v, tilt);
            d->p[i].x = p.x + d->p[group].x;
            d->p[i].y = p.y;
            d->p[i].z = p.z;
            d->p[i].mass = 8.0f / populations[group];
            d->v[i].x = v.x;
            d->v[i].y = v.y;
            d->v[i].z = v.z + d->v[group].z;
            d->color[i] = group ? blue : amber;
        }
    } else if (scene == 1) {
        separation = 2.0f;
        speed = sqrtf(d->cfg.gravity * 25 * separation * separation
            / (2 * powf(separation * separation
                + d->cfg.softening * d->cfg.softening, 1.5f)));
        d->p[0].x = -1;
        d->p[0].mass = 25;
        d->p[1].x = 1;
        d->p[1].mass = 25;
        d->v[0].z = -speed;
        d->v[1].z = speed;
        d->color[0] = amber;
        for (i = 2; i < n; i++) {
            r = 3.8f + 3.0f * uniform();
            a = 2 * PI * uniform();
            speed_disk = sqrtf(d->cfg.gravity * 50 / r);
            d->p[i].x = r * cosf(a);
            d->p[i].y = 0.04f * (uniform() - 0.5f);
            d->p[i].z = r * sinf(a);
            d->p[i].mass = 1.0f / (n - 2);
            d->v[i].x = -speed_disk * sinf(a);
            d->v[i].z = speed_disk * cosf(a);
            d->color[i] = i % 7 == 0 ? amber : blue;
        }
    } else {
        for (i = 0; i < n; i++) {
            z = 2 * uniform() - 1;
            a = 2 * PI * uniform();
            radius = 6.0f * cbrtf(uniform());
            r = radius * sqrtf(1 - z * z);
            d->p[i].x = r * cosf(a);
            d->p[i].y = radius * z;
            d->p[i].z = r * sinf(a);
            d->p[i].mass = 100.0f / n;
            d->v[i].x = -0.045f * d->p[i].z;
            d->v[i].z = 0.045f * d->p[i].x;
            d->color[i] = radius > 4.2f ? blue : amber;
        }
    }
    mass = px = py = pz = vx = vy = vz = 0;
    for (i = 0; i < n; i++) {
        m = d->p[i].mass;
        mass += m;
        px += m * d->p[i].x;
        py += m * d->p[i].y;
        pz += m * d->p[i].z;
        vx += m * d->v[i].x;
        vy += m * d->v[i].y;
        vz += m * d->v[i].z;
    }
    for (i = 0; i < n; i++) {
        d->p[i].x -= (float)(px / mass);
        d->p[i].y -= (float)(py / mass);
        d->p[i].z -= (float)(pz / mass);
        d->v[i].x -= (float)(vx / mass);
        d->v[i].y -= (float)(vy / mass);
        d->v[i].z -= (float)(vz / mass);
    }
#ifdef __CUDACC__
    if (!d->cpu) {
        nbody_gpu_upload(&d->gpu, d->p, d->v);
    }
#endif
    diagnostics(d);
    d->initial_energy = d->energy;
    printf("scene=%s backend=%s bodies=%d dt=%.8g softening=%g E0=%.12g\n",
        names[scene], d->cpu ? "CPU" : "CUDA", n, d->dt, d->cfg.softening,
        d->energy);
    fflush(stdout);
}

static void advance(Demo *d, int steps) {
    int t, body;
    double start = now_ms();
#ifdef __CUDACC__
    if (!d->cpu) {
        nbody_gpu_step(&d->gpu, d->dt, steps, d->cfg);
        nbody_gpu_download(&d->gpu, d->p, d->v);
    } else
#endif
    nbody_step(d->p, d->v, d->scratch, d->n, d->dt, steps, d->cfg);
    d->elapsed_ms = now_ms() - start;
    d->time += (double)d->dt * steps;
    for (t = 0; t < TRAIL_COUNT; t++) {
        body = t < 2 ? t : 2 + (t - 2) * (d->n - 2) / (TRAIL_COUNT - 2);
        d->trails[t][d->trail_head] = (Vector3){
            d->p[body].x, d->p[body].y, d->p[body].z};
    }
    d->trail_head = (d->trail_head + 1) % TRAIL_LENGTH;
    if (d->trail_size < TRAIL_LENGTH) {
        d->trail_size++;
    }
}

#ifndef PLAY_HEADLESS
static Texture2D make_glow(void) {
    Image image = GenImageColor(64, 64, BLANK);
    Color *pixels = (Color *)image.data;
    int x, y;
    float dx, dy, r2, alpha;
    for (y = 0; y < 64; y++) {
        for (x = 0; x < 64; x++) {
            dx = (x - 31.5f) / 31.5f;
            dy = (y - 31.5f) / 31.5f;
            r2 = dx * dx + dy * dy;
            alpha = expf(-5 * r2) * fmaxf(0, 1 - r2);
            pixels[y * 64 + x].r = 255;
            pixels[y * 64 + x].g = 255;
            pixels[y * 64 + x].b = 255;
            pixels[y * 64 + x].a = (unsigned char)(255 * alpha);
        }
    }
    Texture2D texture = LoadTextureFromImage(image);
    UnloadImage(image);
    return texture;
}
#endif

static int integer(char *value, int lo, int hi) {
    char *end;
    long n = strtol(value, &end, 10);
    if (*value == 0 || *end || n < lo || n > hi) {
        fprintf(stderr, "Invalid integer: %s (range %d..%d)\n",
            value, lo, hi);
        exit(2);
    }
    return (int)n;
}

static float positive(char *value) {
    char *end;
    float x = strtof(value, &end);
    if (*value == 0 || *end || !isfinite(x) || x <= 0) {
        fprintf(stderr, "Expected positive finite value: %s\n", value);
        exit(2);
    }
    return x;
}

int main(int argc, char **argv) {
    Demo d;
    int n = 2048, scene = 0, headless_steps = 480, frames = 0;
    int headless = 0, paused = 0, i;
    char *screenshot = 0;
    char *arg, *value;
    double mass, momentum[3], center[3], m;
#ifndef PLAY_HEADLESS
    int rate, frame, trails, width, height, s, g, t, k, a, b, body, core;
    float yaw, pitch, distance;
    Texture2D glow;
    Camera3D camera;
    Vector2 delta;
    Color c;
    Vector3 pos;
#endif
    memset(&d, 0, sizeof(d));
    d.cfg.gravity = 1.0f;
    d.cfg.softening = 0.12f;
    d.dt = 1.0f / 480.0f;
#ifndef __CUDACC__
    d.cpu = 1;
#endif
    for (i = 1; i < argc; i++) {
        arg = argv[i];
        if (strcmp(arg, "--cpu") == 0) {
            d.cpu = 1;
        } else if (strcmp(arg, "--headless") == 0) {
            headless = 1;
        } else if (strcmp(arg, "--paused") == 0) {
            paused = 1;
        } else if (strcmp(arg, "--help") == 0) {
            puts("Puffysics N-body: [--cpu] [--bodies 24..32768] "
                "[--scene encounter|binary|collapse]\n"
                "  [--dt positive] [--softening positive] "
                "[--headless --steps N]\n"
                "  [--frames N --screenshot path.png] [--paused]\n"
                "Controls: 1/2/3 scene, SPACE reset, P pause, N single "
                "step, +/- rate, T trails, drag orbit, wheel zoom, Q quit");
            return 0;
        } else {
            if (i + 1 == argc) {
                fprintf(stderr, "Missing value: %s\n", arg);
                return 2;
            }
            value = argv[++i];
            if (strcmp(arg, "--bodies") == 0) {
                n = integer(value, 24, 32768);
            } else if (strcmp(arg, "--steps") == 0) {
                headless_steps = integer(value, 0, 10000000);
            } else if (strcmp(arg, "--frames") == 0) {
                frames = integer(value, 1, 10000000);
            } else if (strcmp(arg, "--screenshot") == 0) {
                screenshot = value;
            } else if (strcmp(arg, "--dt") == 0) {
                d.dt = positive(value);
            } else if (strcmp(arg, "--softening") == 0) {
                d.cfg.softening = positive(value);
            } else if (strcmp(arg, "--scene") == 0) {
                if (strcmp(value, "encounter") == 0) {
                    scene = 0;
                } else if (strcmp(value, "binary") == 0) {
                    scene = 1;
                } else if (strcmp(value, "collapse") == 0) {
                    scene = 2;
                } else {
                    fprintf(stderr, "Unknown scene: %s\n", value);
                    return 2;
                }
            } else {
                fprintf(stderr, "Unknown option: %s\n", arg);
                return 2;
            }
        }
    }
#ifdef PLAY_HEADLESS
    headless = 1;
#endif
    if (screenshot && (!frames || headless)) {
        fprintf(stderr, "--screenshot requires --frames and a graphical run\n");
        return 2;
    }
    d.n = n;
    d.p = (NbodyPoint *)malloc((size_t)n * sizeof(*d.p));
    d.v = (NbodyVec *)malloc((size_t)n * sizeof(*d.v));
    d.scratch = (NbodyVec *)malloc((size_t)n * sizeof(*d.scratch));
    d.color = (Color *)malloc((size_t)n * sizeof(*d.color));
    assert(d.p && d.v && d.scratch && d.color);
#ifdef __CUDACC__
    if (!d.cpu) {
        nbody_gpu_init(&d.gpu, n);
    }
#endif
    reset(&d, scene);
    if (headless) {
        if (!paused) {
            advance(&d, headless_steps);
        }
        diagnostics(&d);
        mass = momentum[0] = momentum[1] = momentum[2] = 0;
        center[0] = center[1] = center[2] = 0;
        for (i = 0; i < n; i++) {
            m = d.p[i].mass;
            mass += m;
            momentum[0] += m * d.v[i].x;
            momentum[1] += m * d.v[i].y;
            momentum[2] += m * d.v[i].z;
            center[0] += m * d.p[i].x;
            center[1] += m * d.p[i].y;
            center[2] += m * d.p[i].z;
        }
        printf("steps=%d t=%.8g step_and_readback_ms=%.6f "
            "energy_relative_change=%+.9g momentum=(%.7g,%.7g,%.7g) "
            "com=(%.7g,%.7g,%.7g)\n",
            headless_steps, d.time, d.elapsed_ms,
            (d.energy - d.initial_energy) / fabs(d.initial_energy),
            momentum[0], momentum[1], momentum[2],
            center[0] / mass, center[1] / mass, center[2] / mass);
    } else {
#ifndef PLAY_HEADLESS
        SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
        InitWindow(1440, 900, "Puffysics | N-body laboratory");
        if (!IsWindowReady()) {
            fprintf(stderr, "InitWindow failed (display/GL unavailable)\n");
            return 1;
        }
        SetExitKey(KEY_Q);
        SetTargetFPS(60);
        glow = make_glow();
        yaw = 0.72f;
        pitch = 0.65f;
        distance = scene == 0 ? 27.0f : 21.0f;
        rate = 8;
        frame = 0;
        trails = 1;
        while (!WindowShouldClose() && (!frames || frame < frames)) {
            for (s = 0; s < 3; s++) {
                if (IsKeyPressed(KEY_ONE + s)) {
                    reset(&d, s);
                    distance = s == 0 ? 27.0f : 21.0f;
                }
            }
            if (IsKeyPressed(KEY_SPACE)) {
                reset(&d, d.scene);
            }
            if (IsKeyPressed(KEY_P)) {
                paused = !paused;
            }
            if (IsKeyPressed(KEY_T)) {
                trails = !trails;
            }
            if (IsKeyPressed(KEY_EQUAL)) {
                rate = rate * 2 < 64 ? rate * 2 : 64;
            }
            if (IsKeyPressed(KEY_MINUS)) {
                rate = rate / 2 > 1 ? rate / 2 : 1;
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                delta = GetMouseDelta();
                yaw -= delta.x * 0.006f;
                pitch += delta.y * 0.006f;
                if (pitch < -1.45f) {
                    pitch = -1.45f;
                }
                if (pitch > 1.45f) {
                    pitch = 1.45f;
                }
            }
            distance *= expf(-GetMouseWheelMove() * 0.12f);
            if (distance < 3.0f) {
                distance = 3.0f;
            }
            if (distance > 150.0f) {
                distance = 150.0f;
            }
            if (!paused) {
                advance(&d, rate);
            } else if (IsKeyPressed(KEY_N)) {
                advance(&d, 1);
            }
            if (frame % 60 == 0) {
                diagnostics(&d);
            }
            memset(&camera, 0, sizeof(camera));
            camera.position.x = distance * cosf(pitch) * cosf(yaw);
            camera.position.y = distance * sinf(pitch);
            camera.position.z = distance * cosf(pitch) * sinf(yaw);
            camera.up.y = 1;
            camera.fovy = 45;
            camera.projection = CAMERA_PERSPECTIVE;
            width = GetScreenWidth();
            height = GetScreenHeight();
            BeginDrawing();
            ClearBackground((Color){6, 10, 19, 255});
            BeginMode3D(camera);
            for (g = -12; g <= 12; g += 2) {
                DrawLine3D((Vector3){(float)g, -5, -12},
                    (Vector3){(float)g, -5, 12},
                    (Color){22, 32, 48, 255});
                DrawLine3D((Vector3){-12, -5, (float)g},
                    (Vector3){12, -5, (float)g},
                    (Color){22, 32, 48, 255});
            }
            if (trails) {
                for (t = 0; t < TRAIL_COUNT; t++) {
                    body = t < 2 ? t : 2 + (t - 2) * (n - 2)
                        / (TRAIL_COUNT - 2);
                    for (k = 1; k < d.trail_size; k++) {
                        a = (d.trail_head - d.trail_size + k - 1
                            + TRAIL_LENGTH) % TRAIL_LENGTH;
                        b = (a + 1) % TRAIL_LENGTH;
                        c = d.color[body];
                        c.a = (unsigned char)(100 * k / d.trail_size);
                        DrawLine3D(d.trails[t][a], d.trails[t][b], c);
                    }
                }
            }
            rlDisableDepthMask();
            BeginBlendMode(BLEND_ADDITIVE);
            for (i = 0; i < n; i++) {
                core = d.scene != 2 && i < 2;
                pos = (Vector3){d.p[i].x, d.p[i].y, d.p[i].z};
                DrawBillboard(camera, glow, pos, core ? 0.95f : 0.17f,
                    d.color[i]);
                if (core) {
                    DrawBillboard(camera, glow, pos, 0.30f, WHITE);
                }
            }
            EndBlendMode();
            rlEnableDepthMask();
            EndMode3D();
            DrawRectangle(0, 0, width, 150, (Color){6, 10, 19, 235});
            DrawRectangle(26, 26, 4, 86, amber);
            DrawText("PUFFYSICS", 44, 24, 18, muted);
            DrawText(names[d.scene], 43, 51, 30, RAYWHITE);
            DrawText(descriptions[d.scene], 44, 92, 16, muted);
            DrawText(TextFormat("%s  |  %d mutually interacting bodies  |  "
                "direct gravity + kick-drift-kick",
                d.cpu ? "CPU" : "CUDA / shared-memory tiles", n),
                44, 119, 16, blue);
            DrawRectangle(0, height - 113, width, 113,
                (Color){6, 10, 19, 240});
            DrawText(TextFormat("t %.3f     h %.6f     softening %.3f     "
                "%d substeps/frame     %s",
                d.time, d.dt, d.cfg.softening, rate,
                paused ? "PAUSED" : "RUNNING"),
                28, height - 96, 18, RAYWHITE);
            DrawText(TextFormat("Energy change %+.3e (sampled each 60 frames)"
                "    Step + readback %.2f ms    %d FPS",
                (d.energy - d.initial_energy) / fabs(d.initial_energy),
                d.elapsed_ms, GetFPS()),
                28, height - 69, 16, blue);
            DrawText("1/2/3 scenes   SPACE reset   P pause   N step   "
                "+/- rate   T trails   Drag orbit   Wheel zoom   Q quit",
                28, height - 39, 16, muted);
            EndDrawing();
            frame++;
            if (screenshot && frame == frames) {
                TakeScreenshot(screenshot);
            }
        }
        diagnostics(&d);
        printf("rendered_frames=%d t=%.9g energy_relative_change=%+.9g\n",
            frame, d.time,
            (d.energy - d.initial_energy) / fabs(d.initial_energy));
        UnloadTexture(glow);
        CloseWindow();
#endif
    }
#ifdef __CUDACC__
    if (!d.cpu) {
        nbody_gpu_free(&d.gpu);
    }
#endif
    return 0;
}
