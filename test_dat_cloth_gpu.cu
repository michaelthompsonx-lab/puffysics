/* CUDA fabric regression: continuous sheet support, pinned boundaries, and
 * reset/spawn state transitions. Requires a CUDA device; no CPU fallback.
 * nvcc -O2 -arch=native -I. ocean/puffysics/test_dat_cloth_gpu.cu \
 *     -o /tmp/test-dat-gpu
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "dat_cloth_gpu.cuh"

static void require(bool condition, const char* message) {
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(EXIT_FAILURE);
    }
}

static void setup(DatCloth* cloth, B3World* world, float radius,
        B3Vec3 position, B3Vec3 velocity) {
    dat_cloth_init(cloth, 25, 25, 0.1f, b3_v(-1.2f, 0, -1.2f));
    cloth->gravity = 0.0f;
    cloth->node_mass = 0.04f;
    cloth->damping = 0.015f;
    cloth->relax = 0.04f;
    cloth->iters = 4;
    b3_world_init(world);
    world->gravity = b3_v(0, -10, 0);
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = position;
    bd.lin_vel = velocity;
    int body = b3_create_body(world, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0;
    b3_create_sphere(world, body, b3_v(0, 0, 0), radius, &sd);
    float mass = 23.0f * (4.0f / 3.0f) * B3_PI * radius * radius * radius;
    float inertia = 0.4f * mass * radius * radius;
    b3_set_inertial(world, body, mass, b3_v(0, 0, 0),
        b3_v(inertia, inertia, inertia));
}

static void check_cloth(const DatCloth* initial, const DatCloth* result) {
    for (int k = 0; k < result->n; k++) {
        B3Vec3 p = result->pos[k];
        B3Vec3 v = result->vel[k];
        require(isfinite(p.x) && isfinite(p.y) && isfinite(p.z)
            && isfinite(v.x) && isfinite(v.y) && isfinite(v.z),
            "cloth position/velocity became nonfinite");
        if (initial->pinned[k]) {
            require(b3_len(b3_sub(p, initial->pos[k])) < 1.0e-6f,
                "pinned boundary moved");
            require(b3_len2(v) < 1.0e-12f, "pinned boundary gained velocity");
        }
    }
}

/* Independent geometric check: locate the vertical projection in the actual
 * deformed triangles. Unlike vertex separation, this detects a sphere below
 * the sheet even when it has completely escaped all vertex contacts. */
static float support_clearance(const DatCloth* cloth, B3Vec3 center, float r) {
    float clearance = 1.0e30f;
    bool found = false;
    for (int j = 0; j < cloth->H - 1; j++) {
        for (int i = 0; i < cloth->W - 1; i++) {
            int k = j * cloth->W + i;
            int tri[2][3] = {{k, k + cloth->W, k + 1},
                {k + 1, k + cloth->W, k + cloth->W + 1}};
            for (int t = 0; t < 2; t++) {
                B3Vec3 a = cloth->pos[tri[t][0]];
                B3Vec3 ab = b3_sub(cloth->pos[tri[t][1]], a);
                B3Vec3 ac = b3_sub(cloth->pos[tri[t][2]], a);
                B3Vec3 ap = b3_sub(center, a);
                float det = ab.x * ac.z - ab.z * ac.x;
                if (fabsf(det) < 1.0e-10f) continue;
                float u = (ap.x * ac.z - ap.z * ac.x) / det;
                float v = (ab.x * ap.z - ab.z * ap.x) / det;
                if (u < -1.0e-5f || v < -1.0e-5f || u + v > 1.00001f)
                    continue;
                B3Vec3 n = b3_norm(b3_cross(ab, ac));
                if (n.y < 0) n = b3_mul(n, -1);
                float gap = b3_dot(ap, n) - r;
                clearance = fminf(clearance, gap);
                found = true;
            }
        }
    }
    require(found, "test sphere left the supported membrane footprint");
    return clearance;
}

static void drop_case(float radius, float speed, B3Vec3 start, int frames) {
    DatCloth initial, result;
    B3World world, result_world;
    setup(&initial, &world, radius, start, b3_v(0, -speed, 0));
    dat_cloth_init(&result, 25, 25, 0.1f, initial.origin);
    int body = 0;
    DatGpu gpu = {};
    dat_gpu_init(&gpu, &initial, &world, &body, &radius, 1, 0);
    float worst = 1.0e30f;
    for (int frame = 0; frame < frames; frame++) {
        dat_gpu_step(&gpu, 1.0f / 60.0f, 4);
        dat_gpu_download(&gpu, &result, &result_world);
        check_cloth(&initial, &result);
        B3Body b = result_world.bodies[0];
        require(isfinite(b.center.x) && isfinite(b.center.y)
            && isfinite(b.center.z) && isfinite(b.lin_vel.x)
            && isfinite(b.lin_vel.y) && isfinite(b.lin_vel.z),
            "sphere position/velocity became nonfinite");
        float gap = support_clearance(&result, b.center, radius);
        worst = fminf(worst, gap);
        require(gap > -0.005f, "sphere penetrated or escaped through a cloth triangle");
    }
    float lowest = 0;
    for (int k = 0; k < result.n; k++) lowest = fminf(lowest, result.pos[k].y);
    if (radius == 0.16f && speed == 0) {
        require(lowest < -0.02f, "loaded membrane did not deform into a well");
        require(result_world.bodies[0].center.y > -1.2f,
            "central mass did not remain supported");
    }
    printf("GPU drop r=%.3f speed=%.1f t=%.1fs: min_clearance=%.6f star_y=%.4f well=%.4f\n",
        radius, speed, frames / 60.0f, worst,
        result_world.bodies[0].center.y, -lowest);
    dat_gpu_free(&gpu);
    dat_cloth_free(&result);
    dat_cloth_free(&initial);
}

static void reset_and_spawn(void) {
    DatCloth initial, result;
    B3World world, result_world;
    setup(&initial, &world, 0.16f, b3_v(0, 0.35f, 0), b3_v(0, 0, 0));
    dat_cloth_init(&result, 25, 25, 0.1f, initial.origin);
    int body = 0;
    float radius = 0.16f;
    DatGpu gpu = {};
    dat_gpu_init(&gpu, &initial, &world, &body, &radius, 1, 0);
    for (int frame = 0; frame < 120; frame++)
        dat_gpu_step(&gpu, 1.0f / 60.0f, 4);
    for (int p = 0; p < 7; p++) dat_gpu_spawn_planet(&gpu, p);
    dat_gpu_download(&gpu, &result, &result_world);
    require(gpu.nb == 8 && result_world.body_count == 8
        && result_world.shape_count == 8, "planet spawning lost physics bodies");
    for (int q = 1; q < 8; q++) {
        require(support_clearance(&result, result_world.bodies[q].center, 0.075f)
            > -0.005f, "planet spawned inside the cloth");
    }
    for (int frame = 0; frame < 120; frame++)
        dat_gpu_step(&gpu, 1.0f / 60.0f, 4);
    dat_gpu_reset(&gpu, &initial, &world, &body, &radius, 1);
    dat_gpu_download(&gpu, &result, &result_world);
    require(gpu.nb == 1 && result_world.body_count == 1
        && result_world.shape_count == 1, "reset retained spawned bodies");
    require(b3_len(b3_sub(result_world.bodies[0].center, world.bodies[0].center))
        < 1.0e-6f, "reset did not restore initial sphere transform");
    for (int k = 0; k < result.n; k++) {
        require(b3_len(b3_sub(result.pos[k], initial.pos[k])) < 1.0e-6f
            && b3_len2(result.vel[k]) < 1.0e-12f,
            "reset retained membrane deformation or velocity");
    }
    dat_gpu_spawn_planet(&gpu, 0);
    dat_gpu_step(&gpu, 1.0f / 60.0f, 4);
    dat_gpu_download(&gpu, &result, &result_world);
    require(result_world.body_count == 2, "spawn after reset failed");
    check_cloth(&initial, &result);
    printf("GPU lifecycle: seven planets, reset, respawn OK\n");
    dat_gpu_free(&gpu);
    dat_cloth_free(&result);
    dat_cloth_free(&initial);
}

static void batch_worlds(void) {
    const int n = 8;
    DatCloth initial[8], result[8];
    B3World world[8], result_world[8];
    DatGpu gpu[8];
    int body[8];
    float radius[8];
    DatGpuBatch batch = {};
    for (int i = 0; i < n; i++) {
        float x = 0.04f * (float)(i - n / 2);
        setup(&initial[i], &world[i], 0.16f, b3_v(x, 0.35f, 0), b3_v(0, 0, 0));
        dat_cloth_init(&result[i], 25, 25, 0.1f, initial[i].origin);
        body[i] = 0;
        radius[i] = 0.16f;
        gpu[i] = DatGpu{};
        dat_gpu_init(&gpu[i], &initial[i], &world[i], &body[i], &radius[i], 1, 0);
    }
    dat_gpu_batch_bind(&batch, gpu, n, 0);
    for (int frame = 0; frame < 180; frame++)
        dat_gpu_batch_step(&batch, 1.0f / 60.0f, 4);
    for (int i = 0; i < n; i++) {
        dat_gpu_download(&gpu[i], &result[i], &result_world[i]);
        check_cloth(&initial[i], &result[i]);
        float gap = support_clearance(&result[i], result_world[i].bodies[0].center, 0.16f);
        require(gap > -0.005f, "batched world lost triangle support");
        require(result_world[i].bodies[0].center.y > -1.2f,
            "batched star fell through the sheet");
        float lowest = 0;
        for (int k = 0; k < result[i].n; k++)
            lowest = fminf(lowest, result[i].pos[k].y);
        require(lowest < -0.02f, "batched membrane did not form a well");
        require(fabsf(result_world[i].bodies[0].center.x -
                world[i].bodies[0].center.x) < 0.08f,
            "batched worlds mixed star spawn positions");
    }
    int blocks = 0, sms = 0;
    dat_gpu_occupancy(DAT_GPU_BATCH_THREADS, &blocks, &sms);
    require(blocks * sms > 100, "warp-per-world occupancy still around one SM per world");
    printf("GPU batch: %d worlds held the sheet  occupancy %d blocks/SM * %d SMs = %d worlds\n",
        n, blocks, sms, blocks * sms);
    dat_gpu_batch_free(&batch);
    for (int i = 0; i < n; i++) {
        dat_gpu_free(&gpu[i]);
        dat_cloth_free(&result[i]);
        dat_cloth_free(&initial[i]);
    }
}

int main(void) {
    drop_case(0.16f, 0, b3_v(0, 0.35f, 0), 1200);
    /* Smaller than a grid gap, off the nodes: vertex-only contact misses it. */
    drop_case(0.03f, 10, b3_v(0.05f, 0.4f, 0.05f), 180);
    /* Moves farther than its diameter in one substep. */
    drop_case(0.03f, 80, b3_v(0.05f, 0.4f, 0.05f), 180);
    reset_and_spawn();
    batch_worlds();
    puts("GPU fabric regression: OK");
    return EXIT_SUCCESS;
}
