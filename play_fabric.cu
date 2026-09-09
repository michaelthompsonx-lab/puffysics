/* GPU fabric visualizer. Mesh stays on device; sphere centers come back per frame.
 *   nvcc -O2 -arch=native -I. -Iocean/puffysics \
 *       -Iraylib-5.5_linux_amd64/include \
 *       ocean/puffysics/play_fabric.cu -o /tmp/play-fabric \
 *       -Lraylib-5.5_linux_amd64/lib -lraylib -lGL -lm -lpthread -ldl
 */
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "dat_cloth_gpu.cuh"
#include "fabric_render.h"

static void fabric_cuda_check(cudaError_t err, const char* operation) {
    if (err != cudaSuccess) {
        fprintf(stderr, "fabric CUDA: %s: %s\n", operation, cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
}
#define FAB_CUDA(call) fabric_cuda_check((call), #call)

static __global__ void fabric_mesh_kernel(DatCloth c, float* vertices,
        float* normals) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= c.n) return;
    int i = k % c.W;
    int j = k / c.W;
    B3Vec3 p = c.pos[k];
    B3Vec3 dx = b3_sub(c.pos[i + 1 < c.W ? k + 1 : k],
        c.pos[i > 0 ? k - 1 : k]);
    B3Vec3 dz = b3_sub(c.pos[j + 1 < c.H ? k + c.W : k],
        c.pos[j > 0 ? k - c.W : k]);
    B3Vec3 n = b3_cross(dz, dx);
    n = b3_len2(n) > 1.0e-12f ? b3_norm(n) : b3_v(0, 1, 0);
    vertices[3 * k] = p.x;
    vertices[3 * k + 1] = p.y;
    vertices[3 * k + 2] = p.z;
    normals[3 * k] = n.x;
    normals[3 * k + 1] = n.y;
    normals[3 * k + 2] = n.z;
}

static void fabric_update_mesh(DatGpu* gpu,
        cudaGraphicsResource_t* resources) {
    FAB_CUDA(cudaGraphicsMapResources(2, resources, gpu->stream));
    float* vertices = NULL;
    float* normals = NULL;
    size_t vertex_bytes = 0;
    size_t normal_bytes = 0;
    FAB_CUDA(cudaGraphicsResourceGetMappedPointer((void**)&vertices,
        &vertex_bytes, resources[0]));
    FAB_CUDA(cudaGraphicsResourceGetMappedPointer((void**)&normals,
        &normal_bytes, resources[1]));
    size_t required = (size_t)gpu->cloth.n * 3 * sizeof(float);
    if (vertex_bytes < required || normal_bytes < required) {
        fprintf(stderr, "fabric: interop mesh buffers are smaller than the cloth\n");
        exit(EXIT_FAILURE);
    }
    fabric_mesh_kernel<<<(gpu->cloth.n + 127) / 128, 128, 0, gpu->stream>>>(
        gpu->cloth, vertices, normals);
    FAB_CUDA(cudaGetLastError());
    FAB_CUDA(cudaGraphicsUnmapResources(2, resources, gpu->stream));
}

static int fabric_pick_ball(const B3Vec3* pos, const float* radii, int nb, Ray ray) {
    int hit = -1;
    float best = 1.0e9f;
    for (int q = 0; q < nb; q++) {
        Vector3 center = {pos[q].x, pos[q].y, pos[q].z};
        RayCollision c = GetRayCollisionSphere(ray, center, radii[q]);
        if (c.hit && c.distance < best) {
            best = c.distance;
            hit = q;
        }
    }
    return hit;
}

static __global__ void fabric_poke_kernel(B3World* world, const int* bodies,
        int q, B3Vec3 dir, float dv) {
    if (threadIdx.x || blockIdx.x) {
        return;
    }
    B3Body* b = &world->bodies[bodies[q]];
    if (b->type != B3_DYNAMIC || !(b->inv_mass > 0.0f)) {
        return;
    }
    float ln = b3_len(dir);
    if (ln < 1.0e-6f) {
        return;
    }
    dir = b3_mul(dir, 1.0f / ln);
    dir = b3_norm(b3_add(dir, b3_v(0.0f, 0.18f, 0.0f)));
    b->lin_vel = b3_add(b->lin_vel, b3_mul(dir, dv));
    B3Vec3 side = b3_cross(dir, b3_v(0.0f, 1.0f, 0.0f));
    if (b3_len2(side) < 1.0e-8f) {
        side = b3_cross(dir, b3_v(1.0f, 0.0f, 0.0f));
    }
    B3Vec3 r = b3_mul(b3_norm(side), 0.08f);
    b->ang_vel = b3_add(b->ang_vel, b3_mul(b3_cross(r, dir), 6.0f * dv));
}

static void fabric_poke(DatGpu* gpu, int q, Ray ray, float dv) {
    if (q < 0 || q >= gpu->nb) {
        return;
    }
    B3Vec3 dir = b3_v(ray.direction.x, ray.direction.y, ray.direction.z);
    fabric_poke_kernel<<<1, 1, 0, gpu->stream>>>(gpu->world, gpu->body_ids,
        q, dir, dv);
    FAB_CUDA(cudaGetLastError());
}

static void fabric_initial_state(DatCloth* cloth, B3World* world,
        int* body_ids, float* radii) {
    const int width = 25;
    const float spacing = 0.1f;
    const float half = 0.5f * spacing * (width - 1);
    dat_cloth_init(cloth, width, width, spacing, b3_v(-half, 0, -half));
    cloth->gravity = 0.0f;
    cloth->node_mass = 0.04f;
    cloth->damping = 0.015f;
    cloth->relax = 0.04f;
    cloth->iters = 4;
    b3_world_init(world);
    world->gravity = b3_v(0, -10, 0);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(0, 0.35f, 0);
    body_ids[0] = b3_create_body(world, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    radii[0] = 0.16f;
    b3_create_sphere(world, body_ids[0], b3_v(0, 0, 0), radii[0], &sd);
    float mass = 23.0f * (4.0f / 3.0f) * B3_PI * powf(radii[0], 3.0f);
    float inertia = 0.4f * mass * radii[0] * radii[0];
    b3_set_inertial(world, body_ids[0], mass, b3_v(0, 0, 0),
        b3_v(inertia, inertia, inertia));
    for (int q = 1; q < 8; q++) radii[q] = 0.075f;
}

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1440, 900, "Puffysics | CUDA fabric");
    if (!IsWindowReady()) {
        fprintf(stderr, "fabric: GLFW/X11/GLX window initialization failed\n");
        return EXIT_FAILURE;
    }
    SetWindowMinSize(960, 640);
    SetTargetFPS(60);
    SetExitKey(KEY_Q);

    unsigned int device_count = 0;
    int device = 0;
    FAB_CUDA(cudaGLGetDevices(&device_count, &device, 1, cudaGLDeviceListAll));
    if (device_count == 0) {
        fprintf(stderr, "fabric: the OpenGL context has no CUDA-compatible GPU\n");
        CloseWindow();
        return EXIT_FAILURE;
    }
    FAB_CUDA(cudaSetDevice(device));
    cudaDeviceProp props;
    FAB_CUDA(cudaGetDeviceProperties(&props, device));
    printf("fabric: CUDA physics + CUDA/OpenGL mesh interop on %s\n", props.name);
    cudaStream_t stream;
    FAB_CUDA(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));
    cudaEvent_t step_start, step_stop;
    FAB_CUDA(cudaEventCreate(&step_start));
    FAB_CUDA(cudaEventCreate(&step_stop));

    DatCloth initial_cloth;
    B3World initial_world;
    int body_ids[8] = {};
    float radii[8];
    fabric_initial_state(&initial_cloth, &initial_world, body_ids, radii);
    DatGpu gpu = {};
    dat_gpu_init(&gpu, &initial_cloth, &initial_world, body_ids, radii, 1, stream);
    FabricRenderer renderer = {};
    fabric_render_init(&renderer, initial_cloth.W, initial_cloth.H,
        initial_cloth.spacing);
    cudaGraphicsResource_t resources[2] = {};
    FAB_CUDA(cudaGraphicsGLRegisterBuffer(&resources[0], renderer.mesh.vboId[0],
        cudaGraphicsRegisterFlagsWriteDiscard));
    FAB_CUDA(cudaGraphicsGLRegisterBuffer(&resources[1], renderer.mesh.vboId[2],
        cudaGraphicsRegisterFlagsWriteDiscard));
    B3Vec3* ball_positions = NULL;
    FAB_CUDA(cudaMallocHost((void**)&ball_positions, 8 * sizeof(B3Vec3)));
    dat_gpu_read_balls(&gpu, ball_positions);
    fabric_update_mesh(&gpu, resources);

    enum { TRAIL = 240 };
    B3Vec3 trails[7][TRAIL] = {};
    int trail_counts[7] = {};
    float cam_yaw = 1.05f;
    float cam_pitch = 0.52f;
    float cam_dist = 4.3f;
    Vector2 previous_mouse = GetMousePosition();
    Camera3D camera = {};
    camera.up = Vector3{0, 1, 0};
    camera.fovy = 42.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    const float dt = 1.0f / 60.0f;
    const int substeps = 4;
    double accumulator = 0.0;
    float sim_time = 0.0f;
    float gpu_ms = 0.0f;
    int paused = 0;
    int hover = -1;
    int poke = -1;
    float poke_age = 0.0f;
    int frame = 0;
    const char* shots = getenv("FAB_SHOTS");
    const char* auto_spawn_env = getenv("FAB_AUTOSPAWN");
    int auto_spawn = auto_spawn_env ? atoi(auto_spawn_env) : 0;
    while (!WindowShouldClose()) {
        Vector2 mouse = GetMousePosition();
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            cam_yaw -= (mouse.x - previous_mouse.x) * 0.006f;
            cam_pitch += (mouse.y - previous_mouse.y) * 0.005f;
            cam_pitch = fminf(1.45f, fmaxf(0.08f, cam_pitch));
        }
        previous_mouse = mouse;
        cam_dist *= expf(-GetMouseWheelMove() * 0.12f);
        cam_dist = fminf(8.0f, fmaxf(1.2f, cam_dist));
        if (IsKeyPressed(KEY_P)) {
            paused = !paused;
            accumulator = 0.0;
        }
        int changed = 0;
        int reset = IsKeyPressed(KEY_R);
        if (reset) {
            dat_gpu_reset(&gpu, &initial_cloth, &initial_world, body_ids, radii, 1);
            for (int q = 0; q < 7; q++) trail_counts[q] = 0;
            sim_time = 0.0f;
            gpu_ms = 0.0f;
            accumulator = 0.0;
            hover = -1;
            poke = -1;
            poke_age = 0.0f;
            changed = 1;
        }
        if (IsKeyPressed(KEY_SPACE) && gpu.nb < 8) {
            dat_gpu_spawn_planet(&gpu, gpu.nb - 1);
            changed = 1;
        }
        while (auto_spawn > 0 && gpu.nb < 8 && sim_time >= 1.5f) {
            dat_gpu_spawn_planet(&gpu, gpu.nb - 1);
            auto_spawn--;
            changed = 1;
        }
        B3Vec3 star = ball_positions[0];
        camera.target = Vector3{0, star.y * 0.25f, 0};
        camera.position = Vector3{
            camera.target.x + cam_dist * cosf(cam_pitch) * cosf(cam_yaw),
            camera.target.y + cam_dist * sinf(cam_pitch),
            camera.target.z + cam_dist * cosf(cam_pitch) * sinf(cam_yaw)};
        Ray ray = GetScreenToWorldRay(mouse, camera);
        hover = fabric_pick_ball(ball_positions, radii, gpu.nb, ray);
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && hover >= 0) {
            fabric_poke(&gpu, hover, ray, 3.0f);
            poke = hover;
            poke_age = 0.28f;
        } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && hover >= 0) {
            fabric_poke(&gpu, hover, ray, 0.42f);
            poke = hover;
            poke_age = 0.16f;
        }
        if (poke_age > 0.0f) {
            poke_age -= dt;
            if (poke_age <= 0.0f) {
                poke = -1;
                poke_age = 0.0f;
            }
        }

        int ticks = 0;
        if (paused) {
            if (IsKeyPressed(KEY_N)) ticks = 1;
        } else if (!reset) {
            accumulator += fminf(GetFrameTime(), 0.1f);
            ticks = (int)(accumulator / dt);
            accumulator -= ticks * dt;
        }
        if (ticks > 0) {
            FAB_CUDA(cudaEventRecord(step_start, stream));
            for (int tick = 0; tick < ticks; tick++) {
                dat_gpu_step(&gpu, dt, substeps);
                sim_time += dt;
            }
            FAB_CUDA(cudaEventRecord(step_stop, stream));
            changed = 1;
        }
        if (changed) {
            dat_gpu_read_balls(&gpu, ball_positions);
            if (ticks > 0) {
                FAB_CUDA(cudaEventElapsedTime(&gpu_ms, step_start, step_stop));
                gpu_ms /= ticks;
                for (int q = 1; q < gpu.nb; q++) {
                    int t = q - 1;
                    trails[t][trail_counts[t] % TRAIL] = ball_positions[q];
                    trail_counts[t]++;
                }
            }
            fabric_update_mesh(&gpu, resources);
        }
        BeginDrawing();
        ClearBackground(Color{5, 9, 17, 255});
        DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(),
            Color{12, 20, 34, 255}, Color{3, 6, 12, 255});
        for (int i = 0; i < 120; i++) {
            int x = (i * 137 + 19) % GetScreenWidth();
            int y = (i * 293 + 41) % GetScreenHeight();
            unsigned char a = (unsigned char)(22 + (i * 17) % 80);
            DrawPixel(x, y, Color{190, 205, 230, a});
        }
        BeginMode3D(camera);
        fabric_render_draw(&renderer, camera, ball_positions, radii, gpu.nb,
            &trails[0][0], trail_counts, TRAIL, hover, poke, poke_age);
        EndMode3D();
        fabric_render_hud(gpu.nb, paused, sim_time, gpu_ms);
        EndDrawing();
        if (shots && frame % 20 == 0 && frame <= 400) {
            char src[64];
            char dest[1024];
            snprintf(src, sizeof(src), "fab_%04d.png", frame);
            TakeScreenshot(src);
            snprintf(dest, sizeof(dest), "%s/shot_%04d.png", shots, frame);
            rename(src, dest);
        }
        frame++;
    }
    FAB_CUDA(cudaStreamSynchronize(stream));
    FAB_CUDA(cudaGraphicsUnregisterResource(resources[0]));
    FAB_CUDA(cudaGraphicsUnregisterResource(resources[1]));
    dat_gpu_free(&gpu);
    FAB_CUDA(cudaFreeHost(ball_positions));
    FAB_CUDA(cudaEventDestroy(step_start));
    FAB_CUDA(cudaEventDestroy(step_stop));
    FAB_CUDA(cudaStreamDestroy(stream));
    fabric_render_free(&renderer);
    dat_cloth_free(&initial_cloth);
    CloseWindow();
    return EXIT_SUCCESS;
}
