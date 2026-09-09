#ifndef DAT_CLOTH_GPU_CUH
#define DAT_CLOTH_GPU_CUH

/* Device DAT membrane: warp-per-world XPBD stars + triangle CCD on a
 * height field. Macklin α̃ = α/Δt². Init/reset consume host state; later
 * state is device-only. Rebind DatGpuBatch after spawn/reset.
 */
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include "dat_cloth.h"

#define DAT_GPU_MAX_NODES 1024
#define DAT_GPU_MAX_BALLS 8
#define DAT_GPU_VIEWER_THREADS 256
#define DAT_GPU_BATCH_THREADS 32
#define DAT_GPU_STAR 32
#define DAT_GPU_THREADS DAT_GPU_VIEWER_THREADS

struct DatGpuLaunch {
    DatCloth cloth;
    B3World* world;
    int* body_ids;
    float* radii;
    int nb;
};

struct DatGpu {
    DatCloth cloth;
    B3World* world;
    int nb;
    cudaStream_t stream;
    int* body_ids;
    float* radii;
    B3Vec3* centers;
    DatGpuLaunch* launch;
    int body_count;
    int shape_count;
};

struct DatGpuBatch {
    DatGpuLaunch* device;
    int n;
    cudaStream_t stream;
};

static inline void dat_gpu_check(cudaError_t err, const char* operation) {
    if (err != cudaSuccess) {
        fprintf(stderr, "fabric CUDA %s: %s\n", operation,
            cudaGetErrorString(err));
        abort();
    }
}

static inline void dat_gpu_require(int ok, const char* message) {
    if (!ok) {
        fprintf(stderr, "fabric CUDA: %s\n", message);
        abort();
    }
}

struct DatGpuContact {
    int a, b, c;
    B3Vec3 weights;
    B3Vec3 normal;
    float depth;
};

static __device__ __forceinline__ void dat_gpu_triangle(int t, int W,
        int* a, int* b, int* c) {
    int q = t / 2;
    int k = (q / (W - 1)) * W + q % (W - 1);
    if (!(t & 1)) {
        *a = k; *b = k + W; *c = k + 1;
    } else {
        *a = k + 1; *b = k + W; *c = k + W + 1;
    }
}

static __device__ __forceinline__ B3Vec3 dat_gpu_bary_point(B3Vec3 a,
        B3Vec3 b, B3Vec3 c, B3Vec3 w) {
    return b3_add(b3_mul(a, w.x), b3_add(b3_mul(b, w.y), b3_mul(c, w.z)));
}

static __device__ __forceinline__ int dat_gpu_plane_bary(B3Vec3 p,
        B3Vec3 a, B3Vec3 b, B3Vec3 c, B3Vec3* w) {
    B3Vec3 u = b3_sub(b, a), v = b3_sub(c, a), d = b3_sub(p, a);
    float uu = b3_dot(u, u), uv = b3_dot(u, v), vv = b3_dot(v, v);
    float du = b3_dot(d, u), dv = b3_dot(d, v);
    float det = uu * vv - uv * uv;
    if (det <= 1.0e-16f) return 0;
    float y = (vv * du - uv * dv) / det;
    float z = (uu * dv - uv * du) / det;
    *w = b3_v(1.0f - y - z, y, z);
    return 1;
}

static __device__ __forceinline__ int dat_gpu_inside(B3Vec3 w) {
    return w.x >= 0.0f && w.y >= 0.0f && w.z >= 0.0f;
}

static __device__ __forceinline__ B3Vec3 dat_gpu_closest_bary(B3Vec3 p,
        B3Vec3 a, B3Vec3 b, B3Vec3 c) {
    B3Vec3 ab = b3_sub(b, a), ac = b3_sub(c, a), ap = b3_sub(p, a);
    float d1 = b3_dot(ab, ap), d2 = b3_dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return b3_v(1, 0, 0);
    B3Vec3 bp = b3_sub(p, b);
    float d3 = b3_dot(ab, bp), d4 = b3_dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b3_v(0, 1, 0);
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        float v = d1 / (d1 - d3);
        return b3_v(1 - v, v, 0);
    }
    B3Vec3 cp = b3_sub(p, c);
    float d5 = b3_dot(ab, cp), d6 = b3_dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return b3_v(0, 0, 1);
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        float v = d2 / (d2 - d6);
        return b3_v(1 - v, 0, v);
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
        float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b3_v(0, 1 - v, v);
    }
    float inv = 1.0f / (va + vb + vc);
    float v = vb * inv, w = vc * inv;
    return b3_v(1 - v - w, v, w);
}

static __device__ __forceinline__ B3Vec3 dat_gpu_up_normal(B3Vec3 a,
        B3Vec3 b, B3Vec3 c) {
    B3Vec3 n = b3_cross(b3_sub(b, a), b3_sub(c, a));
    float l2 = b3_len2(n);
    if (l2 <= 1.0e-16f) return b3_v(0, 0, 0);
    return b3_mul(n, (n.y < 0 ? -1.0f : 1.0f) * rsqrtf(l2));
}

static __device__ __forceinline__ DatGpuContact dat_gpu_contact(int t,
        int W, const B3Vec3* p, const B3Vec3* prev, B3Vec3 ball,
        B3Vec3 ball_prev, float target, int swept) {
    DatGpuContact ct;
    dat_gpu_triangle(t, W, &ct.a, &ct.b, &ct.c);
    B3Vec3 a = p[ct.a], b = p[ct.b], c = p[ct.c];
    ct.normal = dat_gpu_up_normal(a, b, c);
    ct.weights = b3_v(1, 0, 0);
    ct.depth = -FLT_MAX;
    if (b3_len2(ct.normal) < 0.5f) return ct;
    B3Vec3 w;
    if (!dat_gpu_plane_bary(ball, a, b, c, &w)) return ct;
    float signed_dist = b3_dot(b3_sub(ball, a), ct.normal);
    if (dat_gpu_inside(w)) {
        ct.weights = w;
        ct.depth = target - signed_dist;
        return ct;
    }
    w = dat_gpu_closest_bary(ball, a, b, c);
    B3Vec3 d = b3_sub(ball, dat_gpu_bary_point(a, b, c, w));
    float distance = b3_len(d);
    if (distance <= target + 1.0e-4f) {
        ct.weights = w;
        if (signed_dist >= 0 && distance > 1.0e-8f)
            ct.normal = b3_mul(d, 1.0f / distance);
        ct.depth = target - b3_dot(d, ct.normal);
        return ct;
    }
    if (!swept || signed_dist >= 0) return ct;
    B3Vec3 pa = prev[ct.a], pb = prev[ct.b], pc = prev[ct.c];
    B3Vec3 pn = dat_gpu_up_normal(pa, pb, pc);
    if (b3_dot(b3_sub(ball_prev, pa), pn) < 0) return ct;
    float lo = 0, hi = 1;
    for (int it = 0; it < 12; it++) {
        float f = 0.5f * (lo + hi);
        B3Vec3 aa = b3_madd(pa, f, b3_sub(a, pa));
        B3Vec3 bb = b3_madd(pb, f, b3_sub(b, pb));
        B3Vec3 cc = b3_madd(pc, f, b3_sub(c, pc));
        B3Vec3 pp = b3_madd(ball_prev, f, b3_sub(ball, ball_prev));
        B3Vec3 nn = dat_gpu_up_normal(aa, bb, cc);
        if (b3_dot(b3_sub(pp, aa), nn) >= 0) lo = f;
        else hi = f;
    }
    float f = 0.5f * (lo + hi);
    B3Vec3 aa = b3_madd(pa, f, b3_sub(a, pa));
    B3Vec3 bb = b3_madd(pb, f, b3_sub(b, pb));
    B3Vec3 cc = b3_madd(pc, f, b3_sub(c, pc));
    B3Vec3 pp = b3_madd(ball_prev, f, b3_sub(ball, ball_prev));
    if (dat_gpu_plane_bary(pp, aa, bb, cc, &w) && dat_gpu_inside(w)) {
        ct.weights = w;
        ct.depth = target - b3_dot(b3_sub(ball,
            dat_gpu_bary_point(a, b, c, w)), ct.normal);
    }
    return ct;
}

static __device__ __forceinline__ int dat_gpu_select(float score, int id,
        float* scores, int* ids) {
    int lane = threadIdx.x & 31, warp = threadIdx.x >> 5;
    for (int d = 16; d; d >>= 1) {
        float other = __shfl_down_sync(0xffffffffu, score, d);
        int oi = __shfl_down_sync(0xffffffffu, id, d);
        if (lane + d < 32 && (other > score || (other == score && oi < id))) {
            score = other; id = oi;
        }
    }
    if (!lane) { scores[warp] = score; ids[warp] = id; }
    __syncthreads();
    if (!threadIdx.x) {
        int nwarps = (int)((blockDim.x + 31) >> 5);
        for (int w = 1; w < nwarps; w++) {
            if (scores[w] > scores[0] ||
                    (scores[w] == scores[0] && ids[w] < ids[0])) {
                scores[0] = scores[w]; ids[0] = ids[w];
            }
        }
    }
    __syncthreads();
    return scores[0] > 0 ? ids[0] : -1;
}

static __device__ __noinline__ void dat_gpu_rigid_step(B3World* w, float h) {
    b3_step(w, h, 1);
}

static __device__ __forceinline__ float dat_gpu_inv(const DatCloth cloth, int k) {
    return cloth.pinned[k] ? 0.0f : 1.0f / cloth.node_mass;
}

struct DatGpuStar {
    float hub_x, hub_y, hub_z, hub_w;
    int sat_id[DAT_GPU_STAR];
    float sat_x[DAT_GPU_STAR];
    float sat_y[DAT_GPU_STAR];
    float sat_z[DAT_GPU_STAR];
    float sat_w[DAT_GPU_STAR];
    float sat_l[DAT_GPU_STAR];
    int nsat;
};

static __device__ __forceinline__ float dat_gpu_warp_sum(float v) {
    for (int d = 16; d; d >>= 1)
        v += __shfl_down_sync(0xffffffffu, v, d);
    return v;
}

static __device__ void dat_gpu_star_gather(DatGpuStar* st, DatCloth cloth,
        const B3Vec3* p, B3Vec3 hub, B3Vec3 hub_prev, float target) {
    if (threadIdx.x == 0) {
        float ox = cloth.origin.x, oz = cloth.origin.z, s = cloth.spacing;
        int cx = (int)lrintf(((hub.x + hub_prev.x) * 0.5f - ox) / s);
        int cz = (int)lrintf(((hub.z + hub_prev.z) * 0.5f - oz) / s);
        float reach = target + s;
        int rmax = (int)ceilf(reach / s) + 1;
        if (rmax > 5) rmax = 5;
        int n = 0;
        for (int ring = 0; ring <= rmax && n < DAT_GPU_STAR; ring++) {
            for (int dj = -ring; dj <= ring && n < DAT_GPU_STAR; dj++) {
                for (int di = -ring; di <= ring && n < DAT_GPU_STAR; di++) {
                    if (ring && abs(di) != ring && abs(dj) != ring) continue;
                    int i = cx + di, j = cz + dj;
                    if (i < 0 || i >= cloth.W || j < 0 || j >= cloth.H) continue;
                    st->sat_id[n++] = j * cloth.W + i;
                }
            }
        }
        st->nsat = n;
        st->hub_x = hub.x;
        st->hub_y = hub.y;
        st->hub_z = hub.z;
    }
    __syncthreads();
    if (threadIdx.x < DAT_GPU_STAR) {
        int i = (int)threadIdx.x;
        if (i < st->nsat) {
            int k = st->sat_id[i];
            B3Vec3 q = p[k];
            st->sat_x[i] = q.x;
            st->sat_y[i] = q.y;
            st->sat_z[i] = q.z;
            st->sat_w[i] = dat_gpu_inv(cloth, k);
            st->sat_l[i] = 0.0f;
        } else {
            st->sat_id[i] = -1;
            st->sat_x[i] = st->sat_y[i] = st->sat_z[i] = 0.0f;
            st->sat_w[i] = 0.0f;
            st->sat_l[i] = 0.0f;
        }
    }
    __syncthreads();
}

static __device__ void dat_gpu_star_xpbd(DatGpuStar* st, float target,
        float atilde, int inner) {
    for (int xp = 0; xp < inner; xp++) {
        float dhx = 0.0f, dhy = 0.0f, dhz = 0.0f;
        if (threadIdx.x < DAT_GPU_STAR) {
            int i = (int)threadIdx.x;
            if (i < st->nsat) {
                float rx = st->sat_x[i] - st->hub_x;
                float ry = st->sat_y[i] - st->hub_y;
                float rz = st->sat_z[i] - st->hub_z;
                float d2 = rx * rx + ry * ry + rz * rz;
                if (d2 > 1.0e-16f) {
                    float dist = sqrtf(d2);
                    float C = dist - target;
                    float lambda = st->sat_l[i];
                    if (C < 0.0f || lambda > 0.0f) {
                        float invd = 1.0f / dist;
                        float nx = rx * invd, ny = ry * invd, nz = rz * invd;
                        float ws = st->sat_w[i] + st->hub_w;
                        float denom = ws + atilde;
                        float dlambda = denom > 0.0f ? (-C - atilde * lambda) / denom : 0.0f;
                        float newl = lambda + dlambda;
                        if (newl < 0.0f) {
                            dlambda = -lambda;
                            newl = 0.0f;
                        }
                        st->sat_l[i] = newl;
                        st->sat_x[i] += st->sat_w[i] * dlambda * nx;
                        st->sat_y[i] += st->sat_w[i] * dlambda * ny;
                        st->sat_z[i] += st->sat_w[i] * dlambda * nz;
                        dhx = -st->hub_w * dlambda * nx;
                        dhy = -st->hub_w * dlambda * ny;
                        dhz = -st->hub_w * dlambda * nz;
                    }
                }
            }
            dhx = dat_gpu_warp_sum(dhx);
            dhy = dat_gpu_warp_sum(dhy);
            dhz = dat_gpu_warp_sum(dhz);
            if (i == 0) {
                st->hub_x += dhx;
                st->hub_y += dhy;
                st->hub_z += dhz;
            }
        }
        __syncthreads();
    }
}

static __device__ void dat_gpu_star_scatter(DatGpuStar* st, B3Vec3* p,
        B3Body* body) {
    if (threadIdx.x < DAT_GPU_STAR && threadIdx.x < (unsigned)st->nsat) {
        int k = st->sat_id[threadIdx.x];
        p[k] = b3_v(st->sat_x[threadIdx.x], st->sat_y[threadIdx.x],
            st->sat_z[threadIdx.x]);
    }
    if (threadIdx.x == 0) {
        body->center = b3_v(st->hub_x, st->hub_y, st->hub_z);
    }
    __syncthreads();
}

static __device__ void dat_gpu_star_impulse(DatGpuStar* st, const B3Vec3* pos,
        B3Vec3* vel, B3Body* body, float target, float restitution) {
    if (threadIdx.x == 0) {
        st->hub_x = body->lin_vel.x;
        st->hub_y = body->lin_vel.y;
        st->hub_z = body->lin_vel.z;
    }
    __syncthreads();
    if (threadIdx.x < DAT_GPU_STAR && threadIdx.x < (unsigned)st->nsat) {
        int k = st->sat_id[threadIdx.x];
        B3Vec3 q = pos[k];
        st->sat_x[threadIdx.x] = q.x;
        st->sat_y[threadIdx.x] = q.y;
        st->sat_z[threadIdx.x] = q.z;
        B3Vec3 v = vel[k];
        st->sat_l[threadIdx.x] = v.x;
    }
    __syncthreads();
    float dhx = 0.0f, dhy = 0.0f, dhz = 0.0f;
    if (threadIdx.x < DAT_GPU_STAR) {
        int i = (int)threadIdx.x;
        if (i < st->nsat) {
            int k = st->sat_id[i];
            B3Vec3 q = pos[k];
            B3Vec3 v = vel[k];
            float rx = q.x - body->center.x;
            float ry = q.y - body->center.y;
            float rz = q.z - body->center.z;
            float d2 = rx * rx + ry * ry + rz * rz;
            if (d2 > 1.0e-16f) {
                float dist = sqrtf(d2);
                if (dist <= target + 1.0e-4f) {
                    float invd = 1.0f / dist;
                    float nx = rx * invd, ny = ry * invd, nz = rz * invd;
                    float closing = -((v.x - st->hub_x) * nx
                        + (v.y - st->hub_y) * ny + (v.z - st->hub_z) * nz);
                    if (closing > 0.0f) {
                        float ws = st->sat_w[i] + st->hub_w;
                        float dlambda = ws > 0.0f
                            ? (1.0f + restitution) * closing / ws : 0.0f;
                        vel[k] = b3_v(v.x + st->sat_w[i] * dlambda * nx,
                            v.y + st->sat_w[i] * dlambda * ny,
                            v.z + st->sat_w[i] * dlambda * nz);
                        dhx = st->hub_w * dlambda * nx;
                        dhy = st->hub_w * dlambda * ny;
                        dhz = st->hub_w * dlambda * nz;
                    }
                }
            }
        }
        dhx = dat_gpu_warp_sum(dhx);
        dhy = dat_gpu_warp_sum(dhy);
        dhz = dat_gpu_warp_sum(dhz);
        if (i == 0) {
            body->lin_vel = b3_v(st->hub_x + dhx, st->hub_y + dhy, st->hub_z + dhz);
        }
    }
    __syncthreads();
}

static __global__ __launch_bounds__(DAT_GPU_VIEWER_THREADS)
void dat_gpu_step_kernel(const DatGpuLaunch* worlds, int nworlds,
        float h, int substeps) {
    int wi = (int)blockIdx.x;
    if (wi >= nworlds) return;
    DatGpuLaunch g = worlds[wi];
    DatCloth cloth = g.cloth;
    B3World* world = g.world;
    const int* bodies = g.body_ids;
    const float* radii = g.radii;
    int nb = g.nb;
    int stride = (int)blockDim.x;
    __shared__ B3Vec3 ball_prev[DAT_GPU_MAX_BALLS];
    __shared__ B3Vec3 ball_pred[DAT_GPU_MAX_BALLS];
    __shared__ float scores[DAT_GPU_VIEWER_THREADS / 32];
    __shared__ int ids[DAT_GPU_VIEWER_THREADS / 32];
    __shared__ DatGpuContact selected;
    __shared__ float multiplier;
    __shared__ DatGpuStar star;
    int lane = (int)threadIdx.x;
    int nt = 2 * (cloth.W - 1) * (cloth.H - 1);
    B3Vec3* p = cloth.pos;
    B3Vec3* prev = cloth.snap;
    for (int s = 0; s < substeps; s++) {
        for (int k = lane; k < cloth.n; k += stride) {
            prev[k] = p[k];
            float wi_mass = dat_gpu_inv(cloth, k);
            if (wi_mass) {
                B3Vec3 v = b3_mul(cloth.vel[k], 1.0f - cloth.damping);
                v.y += h * cloth.gravity;
                p[k] = b3_madd(p[k], h, v);
            }
        }
        if (!lane) {
            for (int q = 0; q < nb; q++) ball_prev[q] = world->bodies[bodies[q]].center;
            dat_gpu_rigid_step(world, h);
            for (int q = 0; q < nb; q++) ball_pred[q] = world->bodies[bodies[q]].center;
        }
        __syncthreads();
        int iterations = cloth.iters > 0 ? cloth.iters : 1;
        float atilde = (cloth.compliance > 0.0f && h > 0.0f)
            ? cloth.compliance / (h * h) : 0.0f;
        for (int it = 0; it < iterations; it++) {
            for (int e = 0; e < 4; e++) {
                int dx = e != 1;
                int dy = e == 1 || e == 2 ? 1 : (e == 3 ? -1 : 0);
                float rest = cloth.spacing * (e >= 2 ? 1.41421356237f : 1.0f);
                for (int color = 0; color < 2; color++) {
                    for (int k = lane; k < cloth.n; k += stride) {
                        int x = k % cloth.W, y = k / cloth.W;
                        if (((e == 1 ? y : x) & 1) != color ||
                                x + dx >= cloth.W || y + dy < 0 || y + dy >= cloth.H) continue;
                        int k2 = k + dx + dy * cloth.W;
                        float w0 = dat_gpu_inv(cloth, k);
                        float w1 = dat_gpu_inv(cloth, k2);
                        float ws = w0 + w1;
                        B3Vec3 d = b3_sub(p[k2], p[k]);
                        float length = b3_len(d);
                        if (ws <= 0 || length <= 1.0e-9f) continue;
                        float C = length - rest;
                        float dlambda = (-C) / (ws + atilde);
                        if (cloth.compliance <= 0.0f) dlambda *= cloth.relax;
                        float invl = 1.0f / length;
                        p[k] = b3_madd(p[k], w0 * dlambda * (-invl), d);
                        p[k2] = b3_madd(p[k2], w1 * dlambda * invl, d);
                    }
                    __syncthreads();
                }
            }
            for (int q = 0; q < nb; q++) {
                B3Body* body = &world->bodies[bodies[q]];
                float target = radii[q] + cloth.gamma * 0.1f * cloth.spacing;
                dat_gpu_star_gather(&star, cloth, p, body->center, ball_prev[q], target);
                if (!lane) star.hub_w = body->inv_mass;
                __syncthreads();
                dat_gpu_star_xpbd(&star, target, atilde, 4);
                dat_gpu_star_scatter(&star, p, body);
            }
            for (int sweep = 0; sweep < 2; sweep++) {
                for (int q = 0; q < nb; q++) {
                    B3Body* body = &world->bodies[bodies[q]];
                    float target = radii[q] + cloth.gamma * 0.1f * cloth.spacing;
                    float best = 0; int best_id = 0x7fffffff;
                    for (int t = lane; t < nt; t += stride) {
                        DatGpuContact ct = dat_gpu_contact(t, cloth.W, p, prev,
                            body->center, ball_prev[q], target, 1);
                        if (ct.depth > best || (ct.depth == best && t < best_id)) {
                            best = ct.depth; best_id = t;
                        }
                    }
                    int t = dat_gpu_select(best, best_id, scores, ids);
                    if (t >= 0) {
                        if (!lane) {
                            selected = dat_gpu_contact(t, cloth.W, p, prev,
                                body->center, ball_prev[q], target, 1);
                            B3Vec3 w = selected.weights;
                            float ws = body->inv_mass
                                + dat_gpu_inv(cloth, selected.a) * w.x * w.x
                                + dat_gpu_inv(cloth, selected.b) * w.y * w.y
                                + dat_gpu_inv(cloth, selected.c) * w.z * w.z;
                            multiplier = ws > 0 ? selected.depth / ws : 0;
                            body->center = b3_madd(body->center,
                                multiplier * body->inv_mass, selected.normal);
                        }
                        __syncthreads();
                        for (int k = lane; k < cloth.n; k += stride) {
                            float w = k == selected.a ? selected.weights.x :
                                (k == selected.b ? selected.weights.y :
                                (k == selected.c ? selected.weights.z : 0));
                            float wm = dat_gpu_inv(cloth, k);
                            if (w && wm) p[k] = b3_msub(p[k],
                                multiplier * wm * w, selected.normal);
                        }
                    }
                    __syncthreads();
                }
            }
        }
        for (int k = lane; k < cloth.n; k += stride) {
            float wm = dat_gpu_inv(cloth, k);
            cloth.vel[k] = wm ? b3_mul(b3_sub(p[k], prev[k]), 1.0f / h)
                : b3_v(0, 0, 0);
        }
        if (!lane) {
            for (int q = 0; q < nb; q++) {
                B3Body* body = &world->bodies[bodies[q]];
                body->lin_vel = b3_add(body->lin_vel,
                    b3_mul(b3_sub(body->center, ball_pred[q]), 1.0f / h));
                body->position = b3_sub(body->center,
                    b3_rotate(body->rotation, body->local_center));
            }
        }
        __syncthreads();
        for (int q = 0; q < nb; q++) {
            B3Body* body = &world->bodies[bodies[q]];
            float target = radii[q] + cloth.gamma * 0.1f * cloth.spacing;
            dat_gpu_star_gather(&star, cloth, p, body->center, ball_prev[q], target);
            if (!lane) star.hub_w = body->inv_mass;
            __syncthreads();
            dat_gpu_star_impulse(&star, p, cloth.vel, body, target, cloth.restitution);
        }
        for (int sweep = 0; sweep < 2; sweep++) {
            for (int q = 0; q < nb; q++) {
                B3Body* body = &world->bodies[bodies[q]];
                float target = radii[q] + cloth.gamma * 0.1f * cloth.spacing;
                float best = 0; int best_id = 0x7fffffff;
                for (int t = lane; t < nt; t += stride) {
                    DatGpuContact ct = dat_gpu_contact(t, cloth.W, p, prev,
                        body->center, ball_prev[q], target, 0);
                    if (ct.depth < -1.0e-4f) continue;
                    B3Vec3 v = dat_gpu_bary_point(cloth.vel[ct.a], cloth.vel[ct.b],
                        cloth.vel[ct.c], ct.weights);
                    float closing = -b3_dot(b3_sub(body->lin_vel, v), ct.normal);
                    if (closing > best || (closing == best && t < best_id)) {
                        best = closing; best_id = t;
                    }
                }
                int t = dat_gpu_select(best, best_id, scores, ids);
                if (t >= 0) {
                    if (!lane) {
                        selected = dat_gpu_contact(t, cloth.W, p, prev,
                            body->center, ball_prev[q], target, 0);
                        B3Vec3 w = selected.weights;
                        float ws = body->inv_mass
                            + dat_gpu_inv(cloth, selected.a) * w.x * w.x
                            + dat_gpu_inv(cloth, selected.b) * w.y * w.y
                            + dat_gpu_inv(cloth, selected.c) * w.z * w.z;
                        multiplier = ws > 0 ? (1 + cloth.restitution) * scores[0] / ws : 0;
                        body->lin_vel = b3_madd(body->lin_vel,
                            multiplier * body->inv_mass, selected.normal);
                    }
                    __syncthreads();
                    for (int k = lane; k < cloth.n; k += stride) {
                        float w = k == selected.a ? selected.weights.x :
                            (k == selected.b ? selected.weights.y :
                            (k == selected.c ? selected.weights.z : 0));
                        float wm = dat_gpu_inv(cloth, k);
                        if (w && wm) cloth.vel[k] = b3_msub(cloth.vel[k],
                            multiplier * wm * w, selected.normal);
                    }
                }
                __syncthreads();
            }
        }
    }
}

static __global__ void dat_gpu_spawn_kernel(DatCloth cloth, B3World* world,
        int* bodies, float* radii, int slot, int planet_index) {
    if (threadIdx.x || blockIdx.x) return;
    float radius = 0.075f;
    float distance = 0.55f + 0.12f * (planet_index % 3);
    float angle = 0.9f + 0.7f * planet_index;
    float x = distance * cosf(angle), z = distance * sinf(angle);
    float height = -FLT_MAX;
    B3Vec3 probe = b3_v(x, 0, z);
    int nt = 2 * (cloth.W - 1) * (cloth.H - 1);
    for (int t = 0; t < nt; t++) {
        int ia, ib, ic;
        dat_gpu_triangle(t, cloth.W, &ia, &ib, &ic);
        B3Vec3 a = cloth.pos[ia], b = cloth.pos[ib], c = cloth.pos[ic];
        B3Vec3 w;
        B3Vec3 aa = b3_v(a.x, 0, a.z), bb = b3_v(b.x, 0, b.z), cc = b3_v(c.x, 0, c.z);
        if (!dat_gpu_plane_bary(probe, aa, bb, cc, &w)) continue;
        w = dat_gpu_closest_bary(probe, aa, bb, cc);
        B3Vec3 closest = dat_gpu_bary_point(aa, bb, cc, w);
        if (b3_len2(b3_sub(probe, closest)) <= radius * radius)
            height = fmaxf(height, fmaxf(a.y, fmaxf(b.y, c.y)));
    }
    if (height == -FLT_MAX) height = cloth.origin.y;
    B3BodyDef bd = b3_default_body();
    bd.type = B3_DYNAMIC;
    bd.position = b3_v(x, height + radius + 0.035f, z);
    bd.lin_vel = b3_v(-1.4f * sinf(angle), 0, 1.4f * cosf(angle));
    int id = b3_create_body(world, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0;
    b3_create_sphere(world, id, b3_v(0, 0, 0), radius, &sd);
    float mass = 25.0f * (4.0f / 3.0f) * B3_PI * radius * radius * radius;
    float inertia = 0.4f * mass * radius * radius;
    b3_set_inertial(world, id, mass, b3_v(0, 0, 0), b3_v(inertia, inertia, inertia));
    bodies[slot] = id;
    radii[slot] = radius;
}

static __global__ void dat_gpu_centers_kernel(const B3World* world,
        const int* bodies, B3Vec3* centers, int nb) {
    int q = threadIdx.x;
    if (q < nb) centers[q] = world->bodies[bodies[q]].center;
}

static inline void dat_gpu_validate(const DatCloth* c, const B3World* w,
        const int* bodies, const float* radii, int nb) {
    dat_gpu_require(c && w && c->W >= 2 && c->H >= 2 &&
        c->W <= DAT_GPU_MAX_NODES && c->H <= DAT_GPU_MAX_NODES &&
        c->n == c->W * c->H && c->n <= DAT_GPU_MAX_NODES,
        "sheet must have 2..1024 nodes per dimension and at most 1024 total nodes");
    dat_gpu_require(c->pos && c->vel && c->snap && c->pinned,
        "host cloth arrays must be allocated");
    dat_gpu_require(isfinite(c->spacing) && c->spacing > 0 &&
        isfinite(c->node_mass) && c->node_mass > 0 && isfinite(c->gravity) &&
        isfinite(c->damping) && c->damping >= 0 && c->damping < 1 &&
        isfinite(c->relax) && c->relax > 0 && c->relax <= 1 &&
        isfinite(c->gamma) && c->gamma >= 0 && c->gamma <= 1 &&
        isfinite(c->restitution) && c->restitution >= 0 && c->restitution <= 1 &&
        isfinite(c->compliance) && c->compliance >= 0,
        "invalid membrane mass, spacing, relaxation, damping, or restitution");
    dat_gpu_require(nb >= 0 && nb <= DAT_GPU_MAX_BALLS &&
        (!nb || (bodies && radii)), "expected zero to eight sphere bindings");
    dat_gpu_require(w->body_count >= 0 && w->body_count <= B3_MAX_BODIES &&
        w->shape_count >= 0 && w->shape_count <= B3_MAX_SHAPES,
        "invalid rigid world capacity");
    for (int q = 0; q < nb; q++) {
        dat_gpu_require(bodies[q] >= 0 && bodies[q] < w->body_count &&
            isfinite(radii[q]) && radii[q] > 0, "invalid sphere binding");
        dat_gpu_require(w->bodies[bodies[q]].type == B3_DYNAMIC &&
            w->bodies[bodies[q]].inv_mass > 0, "coupled spheres must be dynamic");
        for (int j = 0; j < q; j++)
            dat_gpu_require(bodies[j] != bodies[q], "duplicate sphere binding");
    }
}

static inline DatGpuLaunch dat_gpu_pack(const DatGpu* g) {
    DatGpuLaunch launch;
    launch.cloth = g->cloth;
    launch.world = g->world;
    launch.body_ids = g->body_ids;
    launch.radii = g->radii;
    launch.nb = g->nb;
    return launch;
}

static inline void dat_gpu_refresh_launch(DatGpu* g) {
    DatGpuLaunch launch = dat_gpu_pack(g);
    dat_gpu_check(cudaMemcpyAsync(g->launch, &launch, sizeof(launch),
        cudaMemcpyHostToDevice, g->stream), "refreshing launch record");
}

static inline int dat_gpu_substeps(float dt, int substeps) {
    dat_gpu_require(isfinite(dt), "nonfinite timestep");
    if (dt <= 0) return 0;
    float required = ceilf(dt / (1.0f / 240.0f));
    dat_gpu_require(required < 2147483648.0f, "timestep exceeds substep capacity");
    int count = (int)required;
    if (substeps > count) count = substeps;
    return count;
}

static inline void dat_gpu_occupancy(int threads, int* blocks_per_sm, int* sm_count) {
    dat_gpu_require(threads > 0 && (threads % 32) == 0, "occupancy thread count");
    int sms = 0;
    dat_gpu_check(cudaDeviceGetAttribute(&sms, cudaDevAttrMultiProcessorCount, 0),
        "querying SM count");
    int blocks = 0;
    dat_gpu_check(cudaOccupancyMaxActiveBlocksPerMultiprocessor(
        &blocks, dat_gpu_step_kernel, threads, 0), "querying kernel occupancy");
    if (blocks_per_sm) *blocks_per_sm = blocks;
    if (sm_count) *sm_count = sms;
}

static inline void dat_gpu_free(DatGpu* g) {
    if (!g) return;
    dat_gpu_check(cudaStreamSynchronize(g->stream), "synchronizing before free");
    dat_gpu_check(cudaFree(g->cloth.pos), "freeing positions");
    dat_gpu_check(cudaFree(g->cloth.vel), "freeing velocities");
    dat_gpu_check(cudaFree(g->cloth.snap), "freeing snapshots");
    dat_gpu_check(cudaFree(g->cloth.pinned), "freeing pins");
    dat_gpu_check(cudaFree(g->world), "freeing rigid world");
    dat_gpu_check(cudaFree(g->body_ids), "freeing sphere bindings");
    dat_gpu_check(cudaFree(g->radii), "freeing radii");
    dat_gpu_check(cudaFree(g->centers), "freeing center staging");
    dat_gpu_check(cudaFree(g->launch), "freeing launch record");
    memset(g, 0, sizeof(*g));
}

static inline void dat_gpu_init(DatGpu* g, const DatCloth* c,
        const B3World* w, const int* bodies, const float* radii, int nb,
        cudaStream_t stream);

static inline void dat_gpu_reset(DatGpu* g, const DatCloth* c,
        const B3World* w, const int* bodies, const float* radii, int nb) {
    dat_gpu_validate(c, w, bodies, radii, nb);
    if (g->cloth.n != c->n) {
        cudaStream_t stream = g->stream;
        dat_gpu_free(g);
        dat_gpu_init(g, c, w, bodies, radii, nb, stream);
        return;
    }
    DatCloth device = *c;
    device.pos = g->cloth.pos;
    device.vel = g->cloth.vel;
    device.snap = g->cloth.snap;
    device.pinned = g->cloth.pinned;
    g->cloth = device;
    g->nb = nb;
    g->body_count = w->body_count;
    g->shape_count = w->shape_count;
    size_t bytes = (size_t)c->n * sizeof(B3Vec3);
    dat_gpu_check(cudaMemcpyAsync(device.pos, c->pos, bytes,
        cudaMemcpyHostToDevice, g->stream), "uploading positions");
    dat_gpu_check(cudaMemcpyAsync(device.vel, c->vel, bytes,
        cudaMemcpyHostToDevice, g->stream), "uploading velocities");
    dat_gpu_check(cudaMemcpyAsync(device.snap, c->snap, bytes,
        cudaMemcpyHostToDevice, g->stream), "uploading snapshots");
    dat_gpu_check(cudaMemcpyAsync(device.pinned, c->pinned, (size_t)c->n,
        cudaMemcpyHostToDevice, g->stream), "uploading pins");
    dat_gpu_check(cudaMemcpyAsync(g->world, w, sizeof(*w),
        cudaMemcpyHostToDevice, g->stream), "uploading rigid world");
    if (nb) {
        dat_gpu_check(cudaMemcpyAsync(g->body_ids, bodies, (size_t)nb * sizeof(int),
            cudaMemcpyHostToDevice, g->stream), "uploading sphere bindings");
        dat_gpu_check(cudaMemcpyAsync(g->radii, radii, (size_t)nb * sizeof(float),
            cudaMemcpyHostToDevice, g->stream), "uploading radii");
    }
    dat_gpu_refresh_launch(g);
    dat_gpu_check(cudaStreamSynchronize(g->stream), "finishing reset");
}

static inline void dat_gpu_init(DatGpu* g, const DatCloth* c,
        const B3World* w, const int* bodies, const float* radii, int nb,
        cudaStream_t stream) {
    dat_gpu_validate(c, w, bodies, radii, nb);
    memset(g, 0, sizeof(*g));
    g->stream = stream;
    g->cloth.n = c->n;
    size_t bytes = (size_t)c->n * sizeof(B3Vec3);
    dat_gpu_check(cudaMalloc((void**)&g->cloth.pos, bytes), "allocating positions");
    dat_gpu_check(cudaMalloc((void**)&g->cloth.vel, bytes), "allocating velocities");
    dat_gpu_check(cudaMalloc((void**)&g->cloth.snap, bytes), "allocating snapshots");
    dat_gpu_check(cudaMalloc((void**)&g->cloth.pinned, (size_t)c->n), "allocating pins");
    dat_gpu_check(cudaMalloc((void**)&g->world, sizeof(B3World)), "allocating rigid world");
    dat_gpu_check(cudaMalloc((void**)&g->body_ids, DAT_GPU_MAX_BALLS * sizeof(int)), "allocating bindings");
    dat_gpu_check(cudaMalloc((void**)&g->radii, DAT_GPU_MAX_BALLS * sizeof(float)), "allocating radii");
    dat_gpu_check(cudaMalloc((void**)&g->centers, DAT_GPU_MAX_BALLS * sizeof(B3Vec3)), "allocating staging");
    dat_gpu_check(cudaMalloc((void**)&g->launch, sizeof(DatGpuLaunch)), "allocating launch record");
    dat_gpu_reset(g, c, w, bodies, radii, nb);
}

static inline void dat_gpu_step_launch(DatGpu* g, float dt, int substeps, int threads) {
    int count = dat_gpu_substeps(dt, substeps);
    if (!count) return;
    dat_gpu_require(threads > 0 && (threads % 32) == 0 &&
        threads <= DAT_GPU_VIEWER_THREADS, "invalid step thread count");
    dat_gpu_step_kernel<<<1, threads, 0, g->stream>>>(g->launch, 1, dt / count, count);
    dat_gpu_check(cudaGetLastError(), "launching coupled step");
}

static inline void dat_gpu_step(DatGpu* g, float dt, int substeps) {
    dat_gpu_step_launch(g, dt, substeps, DAT_GPU_VIEWER_THREADS);
}

static inline void dat_gpu_batch_free(DatGpuBatch* b) {
    if (!b) return;
    dat_gpu_check(cudaStreamSynchronize(b->stream), "synchronizing batch before free");
    dat_gpu_check(cudaFree(b->device), "freeing batch launches");
    memset(b, 0, sizeof(*b));
}

static inline void dat_gpu_batch_bind(DatGpuBatch* b, DatGpu* gs, int n,
        cudaStream_t stream) {
    dat_gpu_require(b && gs && n > 0, "batch bind requires worlds");
    if (b->device && b->n != n) dat_gpu_batch_free(b);
    if (!b->device) {
        dat_gpu_check(cudaMalloc((void**)&b->device, (size_t)n * sizeof(DatGpuLaunch)),
            "allocating batch launches");
    }
    b->n = n;
    b->stream = stream;
    DatGpuLaunch* host = (DatGpuLaunch*)malloc((size_t)n * sizeof(DatGpuLaunch));
    dat_gpu_require(host != NULL, "host batch launch alloc");
    for (int i = 0; i < n; i++) host[i] = dat_gpu_pack(&gs[i]);
    dat_gpu_check(cudaMemcpyAsync(b->device, host, (size_t)n * sizeof(DatGpuLaunch),
        cudaMemcpyHostToDevice, stream), "uploading batch launches");
    dat_gpu_check(cudaStreamSynchronize(stream), "binding batch");
    free(host);
}

static inline void dat_gpu_batch_step(DatGpuBatch* b, float dt, int substeps) {
    dat_gpu_require(b && b->device && b->n > 0, "batch step requires a bound batch");
    int count = dat_gpu_substeps(dt, substeps);
    if (!count) return;
    dat_gpu_step_kernel<<<b->n, DAT_GPU_BATCH_THREADS, 0, b->stream>>>(
        b->device, b->n, dt / count, count);
    dat_gpu_check(cudaGetLastError(), "launching coupled batch step");
}

static inline void dat_gpu_spawn_planet(DatGpu* g, int planet_index) {
    if (g->nb >= DAT_GPU_MAX_BALLS) return;
    dat_gpu_require(planet_index >= 0, "negative planet index");
    dat_gpu_require(g->body_count < B3_MAX_BODIES && g->shape_count < B3_MAX_SHAPES,
        "rigid world has no capacity for another sphere");
    dat_gpu_spawn_kernel<<<1, 1, 0, g->stream>>>(g->cloth, g->world,
        g->body_ids, g->radii, g->nb, planet_index);
    dat_gpu_check(cudaGetLastError(), "launching planet spawn");
    g->nb++;
    g->body_count++;
    g->shape_count++;
    dat_gpu_refresh_launch(g);
}

static inline void dat_gpu_read_balls(DatGpu* g, B3Vec3* host_positions) {
    if (g->nb) {
        dat_gpu_require(host_positions != NULL, "missing host center array");
        dat_gpu_centers_kernel<<<1, 32, 0, g->stream>>>(g->world, g->body_ids,
            g->centers, g->nb);
        dat_gpu_check(cudaGetLastError(), "launching center gather");
        dat_gpu_check(cudaMemcpyAsync(host_positions, g->centers,
            (size_t)g->nb * sizeof(B3Vec3), cudaMemcpyDeviceToHost, g->stream),
            "reading sphere centers");
    }
    dat_gpu_check(cudaStreamSynchronize(g->stream), "finishing center readback");
}

static inline void dat_gpu_download(DatGpu* g, DatCloth* c, B3World* w) {
    dat_gpu_require(c && w && c->n == g->cloth.n && c->pos && c->vel &&
        c->snap && c->pinned, "download requires matching allocated host state");
    DatCloth host = g->cloth;
    host.pos = c->pos; host.vel = c->vel; host.snap = c->snap; host.pinned = c->pinned;
    *c = host;
    size_t bytes = (size_t)c->n * sizeof(B3Vec3);
    dat_gpu_check(cudaMemcpyAsync(c->pos, g->cloth.pos, bytes,
        cudaMemcpyDeviceToHost, g->stream), "downloading positions");
    dat_gpu_check(cudaMemcpyAsync(c->vel, g->cloth.vel, bytes,
        cudaMemcpyDeviceToHost, g->stream), "downloading velocities");
    dat_gpu_check(cudaMemcpyAsync(c->snap, g->cloth.snap, bytes,
        cudaMemcpyDeviceToHost, g->stream), "downloading snapshots");
    dat_gpu_check(cudaMemcpyAsync(c->pinned, g->cloth.pinned, (size_t)c->n,
        cudaMemcpyDeviceToHost, g->stream), "downloading pins");
    dat_gpu_check(cudaMemcpyAsync(w, g->world, sizeof(*w),
        cudaMemcpyDeviceToHost, g->stream), "downloading rigid world");
    dat_gpu_check(cudaStreamSynchronize(g->stream), "finishing verification download");
}

#endif
