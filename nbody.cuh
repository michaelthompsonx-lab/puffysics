#ifndef NBODY_CUH
#define NBODY_CUH

/* Isolated Plummer-softened gravity. Include this file only; it does
 * not pull puffysics.cuh. Env API:
 *   NbodyConfig {gravity >= 0, softening > 0}
 *   NbodyPoint {x, y, z, mass}   NbodyVec {x, y, z}
 *   nbody_step(p, v, scratch, n, dt, steps, cfg)
 *   nbody_energy(p, v, n, cfg)
 * CUDA: NbodyGpu; nbody_gpu_init, nbody_gpu_free,
 *   nbody_gpu_upload, nbody_gpu_download, nbody_gpu_step
 * Hierarchical: nbody_tree.cuh Treecode2 (CPU oct-tree, CUDA Morton BRT).
 * Zero-mass tracers feel gravity but do not source it. Mercury-Opal
 * (arXiv:2601.19654) motivates modest-n GPU gravity; this is KDK,
 * not that paper's hybrid integrator. One float pair-force; device uses
 * rsqrtf, host IEEE 1/(r2*sqrt). Energy stays double.
 */
#include <assert.h>
#include <math.h>
#include <string.h>

#ifdef __CUDACC__
#define NBODY_HD __host__ __device__
#else
#define NBODY_HD
#endif

#ifndef NBODY_TILE
#define NBODY_TILE 128
#endif

typedef struct NbodyVec {
    float x, y, z;
} NbodyVec;

typedef struct NbodyPoint {
    float x, y, z, mass;
} NbodyPoint;

typedef struct NbodyConfig {
    float gravity, softening;
} NbodyConfig;

static NBODY_HD int nbody_cfg_ok(NbodyConfig cfg) {
    return cfg.gravity >= 0 && cfg.softening > 0;
}

/* Device: rsqrt^3. Host: IEEE 1/(r2*sqrt). */
static NBODY_HD float nbody_inv_r3(float r2) {
#ifdef __CUDA_ARCH__
    float inv = rsqrtf(r2);
    return inv * inv * inv;
#else
    return 1.0f / (r2 * sqrtf(r2));
#endif
}

static NBODY_HD void nbody_add_source(NbodyPoint t, NbodyPoint s,
        float gravity, float eps2, float *ax, float *ay, float *az) {
    float dx = s.x - t.x;
    float dy = s.y - t.y;
    float dz = s.z - t.z;
    float f = gravity * s.mass * nbody_inv_r3(dx * dx + dy * dy + dz * dz
        + eps2);
    *ax += f * dx;
    *ay += f * dy;
    *az += f * dz;
}

static void nbody_acceleration(NbodyPoint *p, NbodyVec *a,
        int n, NbodyConfig cfg) {
    int i, j;
    float eps2, ax, ay, az;
    if (n < 0 || !nbody_cfg_ok(cfg) || (n > 0 && !(p && a))) {
        return;
    }
    assert(n >= 0 && nbody_cfg_ok(cfg));
    assert(n == 0 || (p && a));
    eps2 = cfg.softening * cfg.softening;
    for (i = 0; i < n; i++) {
        ax = ay = az = 0;
        for (j = 0; j < n; j++) {
            if (i != j && p[j].mass != 0) {
                nbody_add_source(p[i], p[j], cfg.gravity, eps2,
                    &ax, &ay, &az);
            }
        }
        a[i].x = ax;
        a[i].y = ay;
        a[i].z = az;
    }
}

static void nbody_step(NbodyPoint *p, NbodyVec *v, NbodyVec *scratch,
        int n, float dt, int steps, NbodyConfig cfg) {
    int step, i;
    float half;
    if (n < 0 || steps < 0 || !nbody_cfg_ok(cfg)
            || (n > 0 && !(p && v && scratch))) {
        return;
    }
    assert(n >= 0 && steps >= 0 && nbody_cfg_ok(cfg));
    assert(n == 0 || (p && v && scratch));
    if (n == 0 || steps == 0 || dt == 0) {
        return;
    }
    nbody_acceleration(p, scratch, n, cfg);
    half = 0.5f * dt;
    for (step = 0; step < steps; step++) {
        for (i = 0; i < n; i++) {
            v[i].x += half * scratch[i].x;
            v[i].y += half * scratch[i].y;
            v[i].z += half * scratch[i].z;
            p[i].x += dt * v[i].x;
            p[i].y += dt * v[i].y;
            p[i].z += dt * v[i].z;
        }
        nbody_acceleration(p, scratch, n, cfg);
        for (i = 0; i < n; i++) {
            v[i].x += half * scratch[i].x;
            v[i].y += half * scratch[i].y;
            v[i].z += half * scratch[i].z;
        }
    }
}

static double nbody_energy(NbodyPoint *p, NbodyVec *v, int n,
        NbodyConfig cfg) {
    int i, j;
    double eps2, energy, dx, dy, dz;
    if (n < 0 || !nbody_cfg_ok(cfg) || (n > 0 && !(p && v))) {
        return 0;
    }
    assert(n >= 0 && nbody_cfg_ok(cfg));
    assert(n == 0 || (p && v));
    eps2 = (double)cfg.softening * cfg.softening;
    energy = 0;
    for (i = 0; i < n; i++) {
        if (p[i].mass == 0) {
            continue;
        }
        energy += 0.5 * p[i].mass * ((double)v[i].x * v[i].x
            + (double)v[i].y * v[i].y + (double)v[i].z * v[i].z);
        for (j = 0; j < i; j++) {
            if (p[j].mass == 0) {
                continue;
            }
            dx = (double)p[j].x - p[i].x;
            dy = (double)p[j].y - p[i].y;
            dz = (double)p[j].z - p[i].z;
            energy -= (double)cfg.gravity * p[i].mass * p[j].mass
                / sqrt(dx * dx + dy * dy + dz * dz + eps2);
        }
    }
    return energy;
}

#ifdef __CUDACC__
#include <cuda_runtime.h>

typedef struct NbodyGpu {
    NbodyPoint *p;
    NbodyVec *v, *a;
    cudaStream_t stream;
    int n, acceleration_valid;
    NbodyConfig cached_config;
} NbodyGpu;

/* Every lane loads tiles and hits both barriers. Self-force is 0. */
static __global__ void nbody_acceleration_kernel(NbodyPoint *p,
        NbodyVec *a, int n, NbodyConfig cfg) {
    __shared__ NbodyPoint tile[NBODY_TILE];
    int lane = threadIdx.x;
    int i = blockIdx.x * NBODY_TILE + lane;
    int base, count, j;
    NbodyPoint t = {0}, s;
    float ax = 0, ay = 0, az = 0, eps2;
    if (i < n) {
        t = p[i];
    }
    eps2 = cfg.softening * cfg.softening;
    for (base = 0; base < n; base += count) {
        count = n - base < NBODY_TILE ? n - base : NBODY_TILE;
        if (lane < count) {
            tile[lane] = p[base + lane];
        }
        __syncthreads();
        if (i < n) {
            for (j = 0; j < count; j++) {
                s = tile[j];
                nbody_add_source(t, s, cfg.gravity, eps2, &ax, &ay, &az);
            }
        }
        __syncthreads();
    }
    if (i < n) {
        a[i].x = ax;
        a[i].y = ay;
        a[i].z = az;
    }
}

static __global__ void nbody_kick_kernel(NbodyPoint *p, NbodyVec *v,
        NbodyVec *a, int n, float dt, int drift) {
    int i = blockIdx.x * NBODY_TILE + threadIdx.x;
    float half;
    NbodyVec u;
    if (i >= n) {
        return;
    }
    half = 0.5f * dt;
    u = v[i];
    u.x += half * a[i].x;
    u.y += half * a[i].y;
    u.z += half * a[i].z;
    v[i] = u;
    if (drift) {
        p[i].x += dt * u.x;
        p[i].y += dt * u.y;
        p[i].z += dt * u.z;
    }
}

// All GPU entry points return cudaSuccess (0) or the first CUDA failure.
// Callers must check: errors are never read-and-discarded here, and the
// library never aborts on them. Ignoring the return is allowed in C but
// discouraged; the play_* viewers fall back to CPU on failure.

static cudaError_t nbody_gpu_free(NbodyGpu *g) {
    cudaError_t err, first = cudaSuccess;
    if (!g) {
        return cudaErrorInvalidValue;
    }
    if (g->stream) {
        err = cudaStreamSynchronize(g->stream);
        if (err != cudaSuccess && first == cudaSuccess) {
            first = err;
        }
        err = cudaStreamDestroy(g->stream);
        if (err != cudaSuccess && first == cudaSuccess) {
            first = err;
        }
    }
    // cudaFree(NULL) still initializes the driver; skip empty slots so
    // freeing a zero/partial handle never touches the device.
    if (g->p) {
        err = cudaFree(g->p);
        if (err != cudaSuccess && first == cudaSuccess) {
            first = err;
        }
    }
    if (g->v) {
        err = cudaFree(g->v);
        if (err != cudaSuccess && first == cudaSuccess) {
            first = err;
        }
    }
    if (g->a) {
        err = cudaFree(g->a);
        if (err != cudaSuccess && first == cudaSuccess) {
            first = err;
        }
    }
    memset(g, 0, sizeof(*g));
    return first;
}

static cudaError_t nbody_gpu_init(NbodyGpu *g, int n) {
    cudaError_t err;
    if (!g || n < 0) {
        return cudaErrorInvalidValue;
    }
    /* Re-init of a live handle must release previous buffers. A zeroed
     * handle is a no-op (no driver). First-use callers zero the struct. */
    nbody_gpu_free(g);
    g->n = n;
    if (n == 0) {
        return cudaSuccess;
    }
    err = cudaStreamCreateWithFlags(&g->stream, cudaStreamNonBlocking);
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->p, (size_t)n * sizeof(*g->p));
    }
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->v, (size_t)n * sizeof(*g->v));
    }
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->a, (size_t)n * sizeof(*g->a));
    }
    if (err != cudaSuccess) {
        nbody_gpu_free(g);
        return err;
    }
    return cudaSuccess;
}

static cudaError_t nbody_gpu_upload(NbodyGpu *g, NbodyPoint *p, NbodyVec *v) {
    cudaError_t err;
    if (!g) {
        return cudaErrorInvalidValue;
    }
    g->acceleration_valid = 0;
    if (g->n == 0) {
        return cudaSuccess;
    }
    if (!p || !v) {
        return cudaErrorInvalidValue;
    }
    err = cudaMemcpyAsync(g->p, p, (size_t)g->n * sizeof(*p),
        cudaMemcpyHostToDevice, g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    return cudaMemcpyAsync(g->v, v, (size_t)g->n * sizeof(*v),
        cudaMemcpyHostToDevice, g->stream);
}

static cudaError_t nbody_gpu_download(NbodyGpu *g, NbodyPoint *p, NbodyVec *v) {
    cudaError_t err;
    if (!g) {
        return cudaErrorInvalidValue;
    }
    if (g->n == 0) {
        return cudaSuccess;
    }
    if (!p || !v) {
        return cudaErrorInvalidValue;
    }
    err = cudaMemcpyAsync(p, g->p, (size_t)g->n * sizeof(*p),
        cudaMemcpyDeviceToHost, g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    err = cudaMemcpyAsync(v, g->v, (size_t)g->n * sizeof(*v),
        cudaMemcpyDeviceToHost, g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    return cudaStreamSynchronize(g->stream);
}

static cudaError_t nbody_gpu_step(NbodyGpu *g, float dt, int steps,
        NbodyConfig cfg) {
    unsigned blocks;
    int step;
    cudaError_t err;
    if (!g || steps < 0 || !nbody_cfg_ok(cfg)) {
        return cudaErrorInvalidValue;
    }
    if (g->n == 0 || steps == 0 || dt == 0) {
        return cudaSuccess;
    }
    blocks = ((unsigned)g->n + NBODY_TILE - 1) / NBODY_TILE;
    if (!g->acceleration_valid
            || cfg.gravity != g->cached_config.gravity
            || cfg.softening != g->cached_config.softening) {
        nbody_acceleration_kernel<<<blocks, NBODY_TILE, 0, g->stream>>>(
            g->p, g->a, g->n, cfg);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
    }
    for (step = 0; step < steps; step++) {
        nbody_kick_kernel<<<blocks, NBODY_TILE, 0, g->stream>>>(
            g->p, g->v, g->a, g->n, dt, 1);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
        nbody_acceleration_kernel<<<blocks, NBODY_TILE, 0, g->stream>>>(
            g->p, g->a, g->n, cfg);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
        nbody_kick_kernel<<<blocks, NBODY_TILE, 0, g->stream>>>(
            g->p, g->v, g->a, g->n, dt, 0);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
    }
    err = cudaStreamSynchronize(g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    g->acceleration_valid = 1;
    g->cached_config = cfg;
    return cudaSuccess;
}
#endif
#endif
