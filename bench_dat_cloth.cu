/* Headless CPU vs CUDA fabric timing.
 *
 * Measures the actual implementations, not a same-algorithm pair:
 * CPU uses vertex-only coupling (dat_cloth.h); GPU uses triangle-surface
 * coupling (dat_cloth_gpu.cuh). Same sheet, masses, dt, and substep count.
 *
 *   nvcc -O2 -arch=native -I. ocean/puffysics/bench_dat_cloth.cu -o /tmp/bench-fabric
 *   LD_LIBRARY_PATH=/run/opengl-driver/lib /tmp/bench-fabric
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "dat_cloth_gpu.cuh"

static void die(const char* message) {
    fprintf(stderr, "bench-fabric: %s\n", message);
    exit(EXIT_FAILURE);
}

static double now_s(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + 1e-9 * (double)t.tv_nsec;
}

static void fabric_tune(DatCloth* c) {
    c->gravity = 0.0f;
    c->node_mass = 0.04f;
    c->damping = 0.015f;
    c->relax = 0.04f;
    c->iters = 4;
}

static int add_sphere(B3World* w, float r, float density, B3Vec3 pos, B3Vec3 vel) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = pos;
    bd.lin_vel = vel;
    int body = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    b3_create_sphere(w, body, b3_v(0, 0, 0), r, &sd);
    float mass = density * (4.0f / 3.0f) * B3_PI * r * r * r;
    float inertia = 0.4f * mass * r * r;
    b3_set_inertial(w, body, mass, b3_v(0, 0, 0),
        b3_v(inertia, inertia, inertia));
    return body;
}

static void make_scene(DatCloth* c, B3World* w, int* bodies, float* radii,
        int planets) {
    const int W = 25;
    const float spacing = 0.1f;
    const float half = 0.5f * spacing * (float)(W - 1);
    dat_cloth_init(c, W, W, spacing, b3_v(-half, 0, -half));
    fabric_tune(c);
    b3_world_init(w);
    w->gravity = b3_v(0, -10, 0);
    bodies[0] = add_sphere(w, 0.16f, 23.0f, b3_v(0, 0.35f, 0), b3_v(0, 0, 0));
    radii[0] = 0.16f;
    for (int p = 0; p < planets; p++) {
        float distance = 0.55f + 0.12f * (float)(p % 3);
        float angle = 0.9f + 0.7f * (float)p;
        B3Vec3 pos = b3_v(distance * cosf(angle), 0.12f, distance * sinf(angle));
        B3Vec3 vel = b3_v(-1.4f * sinf(angle), 0, 1.4f * cosf(angle));
        bodies[p + 1] = add_sphere(w, 0.075f, 25.0f, pos, vel);
        radii[p + 1] = 0.075f;
    }
}

static void cpu_frame(B3World* w, DatCloth* c, int* bodies, float* radii,
        int nb, float dt, int subs) {
    float h = dt / (float)subs;
    B3Vec3 snap[8];
    for (int s = 0; s < subs; s++) {
        b3_step(w, h, 1);
        dat_cloth_step(c, h);
        for (int q = 0; q < nb; q++) snap[q] = w->bodies[bodies[q]].center;
        dat_couple(c, w, snap, bodies, radii, nb, h);
        dat_couple_velocities(w, snap, bodies, nb, h);
    }
}

static int cmp_double(const void* a, const void* b) {
    double da = *(const double*)a, db = *(const double*)b;
    return (da > db) - (da < db);
}

static void cloth_extent(const DatCloth* c, float* well) {
    float miny = 1e9f, maxy = -1e9f;
    for (int k = 0; k < c->n; k++) {
        if (c->pos[k].y < miny) miny = c->pos[k].y;
        if (c->pos[k].y > maxy) maxy = c->pos[k].y;
    }
    *well = maxy - miny;
}

static void report(const char* path, int frames, int planets,
        double med_ms, double min_ms, double max_ms, float star_y, float well) {
    printf("%-14s  planets=%d  frames=%-4d  median=%7.3f ms  min=%7.3f  max=%7.3f  "
           "frame/s=%8.1f  star_y=%7.4f  well=%7.4f\n",
        path, planets, frames, med_ms, min_ms, max_ms,
        1000.0 / med_ms, star_y, well);
}

int main(int argc, char** argv) {
    int frames = 600;
    int repeats = 5;
    int warmup = 30;
    int scale_only = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--frames") && i + 1 < argc) frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--repeats") && i + 1 < argc) repeats = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--warmup") && i + 1 < argc) warmup = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--scale-only")) scale_only = 1;
        else die("usage: bench-fabric [--frames N] [--repeats R] [--warmup W] [--scale-only]");
    }
    if (frames < 1 || repeats < 1 || repeats > 16 || warmup < 0) die("invalid settings");

    int devices = 0;
    cudaGetDeviceCount(&devices);
    if (devices <= 0) die("no CUDA device");
    cudaDeviceProp props;
    cudaGetDeviceProperties(&props, 0);
    int occ256 = 0, occ32 = 0, sms = 0;
    dat_gpu_occupancy(DAT_GPU_VIEWER_THREADS, &occ256, &sms);
    dat_gpu_occupancy(DAT_GPU_BATCH_THREADS, &occ32, &sms);
    printf("device=%s  sm=%d.%d  SMs=%d  cloth=25x25  dt=1/60  substeps=4  "
           "warmup=%d  repeats=%d\n",
        props.name, props.major, props.minor, sms, warmup, repeats);
    printf("occupancy  256-thread viewer: %d blocks/SM -> %d concurrent worlds\n",
        occ256, occ256 * sms);
    printf("occupancy  32-thread batch:   %d blocks/SM -> %d concurrent worlds\n",
        occ32, occ32 * sms);
    printf("CPU = dat_cloth.h vertex coupling.\n"
           "GPU viewer = 256-thread triangle coupling.\n"
           "GPU batch = one 32-thread warp per world.\n"
           "gpu-device excludes host readback; gpu-readback matches the visualizer "
           "(sphere centers only).\n\n");

    const float dt = 1.0f / 60.0f;
    const int subs = 4;
    const int planet_counts[] = {0, 3, 7};
    double cpu_med[3] = {0}, gpu_med[3] = {0}, read_med[3] = {0};

    for (int si = 0; si < 3 && !scale_only; si++) {
        int planets = planet_counts[si];
        int nb = 1 + planets;
        double cpu_ms[16], gpu_ms[16], read_ms[16];
        float cpu_y = 0, cpu_well = 0, gpu_y = 0, gpu_well = 0;

        for (int trial = 0; trial < repeats; trial++) {
            DatCloth c;
            B3World w;
            int bodies[8];
            float radii[8];
            make_scene(&c, &w, bodies, radii, planets);
            for (int i = 0; i < warmup; i++)
                cpu_frame(&w, &c, bodies, radii, nb, dt, subs);
            double t0 = now_s();
            for (int i = 0; i < frames; i++)
                cpu_frame(&w, &c, bodies, radii, nb, dt, subs);
            cpu_ms[trial] = 1000.0 * (now_s() - t0) / (double)frames;
            cpu_y = w.bodies[bodies[0]].center.y;
            cloth_extent(&c, &cpu_well);
            dat_cloth_free(&c);
        }
        qsort(cpu_ms, (size_t)repeats, sizeof(double), cmp_double);
        cpu_med[si] = cpu_ms[repeats / 2];
        report("cpu", frames, planets, cpu_ms[repeats / 2], cpu_ms[0],
            cpu_ms[repeats - 1], cpu_y, cpu_well);

        for (int trial = 0; trial < repeats; trial++) {
            DatCloth c;
            B3World w;
            int bodies[8];
            float radii[8];
            make_scene(&c, &w, bodies, radii, planets);
            DatGpu gpu = {};
            dat_gpu_init(&gpu, &c, &w, bodies, radii, nb, 0);
            for (int i = 0; i < warmup; i++) dat_gpu_step(&gpu, dt, subs);
            cudaDeviceSynchronize();
            cudaEvent_t start, stop;
            cudaEventCreate(&start);
            cudaEventCreate(&stop);
            cudaEventRecord(start);
            for (int i = 0; i < frames; i++) dat_gpu_step(&gpu, dt, subs);
            cudaEventRecord(stop);
            cudaEventSynchronize(stop);
            float ms = 0;
            cudaEventElapsedTime(&ms, start, stop);
            gpu_ms[trial] = (double)ms / (double)frames;
            DatCloth out;
            B3World ow;
            dat_cloth_init(&out, 25, 25, 0.1f, c.origin);
            dat_gpu_download(&gpu, &out, &ow);
            gpu_y = ow.bodies[0].center.y;
            cloth_extent(&out, &gpu_well);
            dat_cloth_free(&out);
            dat_gpu_free(&gpu);
            dat_cloth_free(&c);
            cudaEventDestroy(start);
            cudaEventDestroy(stop);
        }
        qsort(gpu_ms, (size_t)repeats, sizeof(double), cmp_double);
        gpu_med[si] = gpu_ms[repeats / 2];
        report("gpu-device", frames, planets, gpu_ms[repeats / 2], gpu_ms[0],
            gpu_ms[repeats - 1], gpu_y, gpu_well);

        for (int trial = 0; trial < repeats; trial++) {
            DatCloth c;
            B3World w;
            int bodies[8];
            float radii[8];
            make_scene(&c, &w, bodies, radii, planets);
            DatGpu gpu = {};
            dat_gpu_init(&gpu, &c, &w, bodies, radii, nb, 0);
            B3Vec3 centers[8];
            for (int i = 0; i < warmup; i++) {
                dat_gpu_step(&gpu, dt, subs);
                dat_gpu_read_balls(&gpu, centers);
            }
            double t0 = now_s();
            for (int i = 0; i < frames; i++) {
                dat_gpu_step(&gpu, dt, subs);
                dat_gpu_read_balls(&gpu, centers);
            }
            read_ms[trial] = 1000.0 * (now_s() - t0) / (double)frames;
            dat_gpu_free(&gpu);
            dat_cloth_free(&c);
        }
        qsort(read_ms, (size_t)repeats, sizeof(double), cmp_double);
        read_med[si] = read_ms[repeats / 2];
        report("gpu-readback", frames, planets, read_ms[repeats / 2], read_ms[0],
            read_ms[repeats - 1], gpu_y, gpu_well);
        printf("\n");
    }

    if (!scale_only) {
        printf("speedup (cpu median / gpu median)\n");
        for (int si = 0; si < 3; si++) {
            printf("  planets=%d  device %.2fx  with-readback %.2fx\n",
                planet_counts[si], cpu_med[si] / gpu_med[si],
                cpu_med[si] / read_med[si]);
        }
        printf("\n");
    }

    printf("batch scaling (32-thread warp/world, star only)\n");
    const int world_counts[] = {1, 80, 256, 1024};
    const int scale_frames = scale_only ? frames : 120;
    const int scale_repeats = scale_only ? repeats : 3;
    const int scale_warmup = scale_only ? warmup : 5;
    for (int wi = 0; wi < 4; wi++) {
        int n = world_counts[wi];
        DatCloth* cloths = (DatCloth*)malloc((size_t)n * sizeof(DatCloth));
        B3World* worlds = (B3World*)malloc((size_t)n * sizeof(B3World));
        DatGpu* gpus = (DatGpu*)malloc((size_t)n * sizeof(DatGpu));
        int (*bodies)[8] = (int(*)[8])malloc((size_t)n * sizeof(*bodies));
        float (*radii)[8] = (float(*)[8])malloc((size_t)n * sizeof(*radii));
        if (!cloths || !worlds || !gpus || !bodies || !radii) die("batch alloc");
        for (int i = 0; i < n; i++) {
            make_scene(&cloths[i], &worlds[i], bodies[i], radii[i], 0);
            gpus[i] = DatGpu{};
            dat_gpu_init(&gpus[i], &cloths[i], &worlds[i], bodies[i], radii[i], 1, 0);
        }
        DatGpuBatch batch = {};
        dat_gpu_batch_bind(&batch, gpus, n, 0);
        for (int i = 0; i < scale_warmup; i++) dat_gpu_batch_step(&batch, dt, subs);
        cudaDeviceSynchronize();
        double times[16];
        for (int trial = 0; trial < scale_repeats; trial++) {
            cudaEvent_t start, stop;
            cudaEventCreate(&start);
            cudaEventCreate(&stop);
            cudaEventRecord(start);
            for (int i = 0; i < scale_frames; i++) dat_gpu_batch_step(&batch, dt, subs);
            cudaEventRecord(stop);
            cudaEventSynchronize(stop);
            float ms = 0;
            cudaEventElapsedTime(&ms, start, stop);
            times[trial] = (double)ms / (double)scale_frames;
            cudaEventDestroy(start);
            cudaEventDestroy(stop);
        }
        qsort(times, (size_t)scale_repeats, sizeof(double), cmp_double);
        DatCloth out;
        B3World ow;
        dat_cloth_init(&out, 25, 25, 0.1f, cloths[0].origin);
        dat_gpu_download(&gpus[0], &out, &ow);
        float well = 0;
        cloth_extent(&out, &well);
        printf("  worlds=%-5d  median=%7.3f ms/frame  %7.1f us/world  "
               "world-steps/s=%9.0f  star_y=%7.4f  well=%7.4f\n",
            n, times[scale_repeats / 2],
            1000.0 * times[scale_repeats / 2] / (double)n,
            1000.0 * (double)n / times[scale_repeats / 2],
            ow.bodies[0].center.y, well);
        dat_cloth_free(&out);
        dat_gpu_batch_free(&batch);
        for (int i = 0; i < n; i++) {
            dat_gpu_free(&gpus[i]);
            dat_cloth_free(&cloths[i]);
        }
        free(cloths); free(worlds); free(gpus); free(bodies); free(radii);
    }
    return 0;
}
