// Standalone workload benchmark. Compile with nvcc, or g++ -x c++ for CPU.
// Override B3_BENCH_HEADER to compare an unchanged engine snapshot.
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <vector>
#include <algorithm>
#ifndef B3_BENCH_HEADER
#define B3_BENCH_HEADER "puffysics.cuh"
#endif
#include B3_BENCH_HEADER

#ifdef __CUDACC__
#define CUDA(call) do { cudaError_t e = (call); if (e != cudaSuccess) { \
    fprintf(stderr, "%s: %s\n", #call, cudaGetErrorString(e)); exit(2); } } while (0)
__global__ void bench_find(B3World* worlds, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) b3_find_contacts(worlds + i);
}

/* Prototype: one block per world. The O(n^2) pair sweep, AABB build and
 * narrowphase run cooperatively across lanes; contacts are appended in the
 * exact serial (shape_a, shape_b) order via a deterministic per-round
 * prefix scan, so the downstream serial solver sees a bitwise-identical
 * contact list. Lane 0 then runs the unchanged serial step tail. */
#define B3_BLOCK_T 128

typedef struct {
    B3Mani mani;
    int i;
    int j;
} B3BlockHit;

__device__ int block_pair_row_start(int t, int i) {
    return i * t - i - i * (i - 1) / 2;
}

__device__ void block_find_contacts(B3World* w) {
    __shared__ B3Warm sh_old[B3_MAX_CONTACTS];
    __shared__ B3AABB sh_aabb[B3_MAX_SHAPES];
    __shared__ B3BlockHit sh_hit[B3_BLOCK_T];
    __shared__ int sh_scan[B3_BLOCK_T];
    __shared__ uint64_t sh_conn[B3_MAX_BODIES][B3_CONNECT_WORDS];
    __shared__ int sh_base;
    __shared__ int sh_total;
    __shared__ int sh_old_n;
    const int tid = threadIdx.x;
    const int nt = B3_BLOCK_T;
    const int T = w->shape_count;
    const int P = T * (T - 1) / 2;
    if (tid == 0) {
        sh_old_n = w->contact_count;
    }
    __syncthreads();
    for (int k = tid; k < sh_old_n; k += nt) {
        const B3Contact* src = &w->contacts[k];
        B3Warm* dst = &sh_old[k];
        dst->shape_a = src->shape_a;
        dst->shape_b = src->shape_b;
        dst->point_count = src->point_count;
        dst->friction_impulse = src->friction_impulse;
        dst->twist_impulse = src->twist_impulse;
        dst->rolling_impulse = src->rolling_impulse;
        for (int p = 0; p < src->point_count; p++) {
            dst->feature[p] = src->points[p].feature;
            dst->normal_impulse[p] = src->points[p].normal_impulse;
        }
    }
    for (int k = tid; k < w->body_count; k += nt) {
        for (int wd = 0; wd < B3_CONNECT_WORDS; wd++) {
            sh_conn[k][wd] = 0;
        }
    }
    __syncthreads();
    if (tid == 0) {
        for (int k = 0; k < w->joint_count; k++) {
            const B3Joint* jn = &w->joints[k];
            if (jn->collide_connected) {
                continue;
            }
            sh_conn[jn->body_a][jn->body_b >> 6]
                |= (1ull << (jn->body_b & 63));
            sh_conn[jn->body_b][jn->body_a >> 6]
                |= (1ull << (jn->body_a & 63));
        }
    }
    B3Vec3 pad = b3_v(B3_SPECULATIVE, B3_SPECULATIVE, B3_SPECULATIVE);
    for (int k = tid; k < T; k += nt) {
        sh_aabb[k] = b3_shape_aabb(&w->bodies[w->shapes[k].body],
            &w->shapes[k]);
        sh_aabb[k].lo = b3_sub(sh_aabb[k].lo, pad);
        sh_aabb[k].hi = b3_add(sh_aabb[k].hi, pad);
    }
    __syncthreads();
    if (tid == 0) {
        w->contact_count = 0;
        sh_base = 0;
    }
    __syncthreads();
    for (int p0 = 0; p0 < P; p0 += nt) {
        int p = p0 + tid;
        int hit = 0;
        int i = 0;
        int j = 0;
        if (p < P) {
            int lo = 0;
            int hi = T - 1;
            while (lo < hi) {
                int mid = (lo + hi + 1) >> 1;
                if (block_pair_row_start(T, mid) <= p) {
                    lo = mid;
                } else {
                    hi = mid - 1;
                }
            }
            i = lo;
            j = i + 1 + (p - block_pair_row_start(T, i));
            B3Shape* sa = &w->shapes[i];
            B3Shape* sb = &w->shapes[j];
            B3Body* ba = &w->bodies[sa->body];
            B3Body* bb = &w->bodies[sb->body];
            if (sa->body != sb->body
                && (sh_conn[sa->body][sb->body >> 6]
                    & (1ull << (sb->body & 63))) == 0
                && !(ba->type == B3_STATIC && bb->type == B3_STATIC)
                && (sa->category & sb->mask) != 0
                && (sb->category & sa->mask) != 0
                && b3_aabb_overlap(sh_aabb[i], sh_aabb[j])) {
                b3_collide_shapes(&sh_hit[tid].mani, w, sa, sb);
                sh_hit[tid].i = i;
                sh_hit[tid].j = j;
                hit = 1;
            }
        }
        __syncthreads();
        int real = hit && sh_hit[tid].mani.count > 0;
        sh_scan[tid] = real ? 1 : 0;
        __syncthreads();
        if (tid == 0) {
            int total = 0;
            for (int k = 0; k < nt; k++) {
                int f = sh_scan[k];
                sh_scan[k] = total;
                total += f;
            }
            sh_total = total;
        }
        __syncthreads();
        if (real) {
            int slot = sh_base + sh_scan[tid];
            if (slot < B3_MAX_CONTACTS) {
                B3Shape* sa = &w->shapes[i];
                B3Shape* sb = &w->shapes[j];
                B3Body* ba = &w->bodies[sa->body];
                B3Body* bb = &w->bodies[sb->body];
                B3Contact* c = &w->contacts[slot];
                b3_contact_from_mani(c, i, j, sa, sb, ba, bb,
                    &sh_hit[tid].mani);
                for (int k = 0; k < sh_old_n; k++) {
                    if (sh_old[k].shape_a != i || sh_old[k].shape_b != j) {
                        continue;
                    }
                    for (int p = 0; p < c->point_count; p++) {
                        for (int q = 0; q < sh_old[k].point_count; q++) {
                            if (c->points[p].feature == sh_old[k].feature[q]) {
                                c->points[p].normal_impulse =
                                    sh_old[k].normal_impulse[q];
                            }
                        }
                    }
                    c->friction_impulse = sh_old[k].friction_impulse;
                    c->twist_impulse = sh_old[k].twist_impulse;
                    c->rolling_impulse = sh_old[k].rolling_impulse;
                }
            }
        }
        __syncthreads();
        if (tid == 0 && sh_total) {
            sh_base += sh_total;
            if (sh_base > B3_MAX_CONTACTS) {
                sh_base = B3_MAX_CONTACTS;
            }
        }
        __syncthreads();
    }
    if (tid == 0) {
        w->contact_count = sh_base;
    }
}

__global__ void bench_find_block(B3World* worlds, int n) {
    if (blockIdx.x < n) {
        block_find_contacts(worlds + blockIdx.x);
    }
}

__global__ void bench_step_block(B3World* worlds, int n, float dt,
        int substeps) {
    if (blockIdx.x >= n || dt <= 0.0f) {
        return;
    }
    B3World* w = worlds + blockIdx.x;
    block_find_contacts(w);
    __syncthreads();
    if (threadIdx.x != 0) {
        return;
    }
    int subs = substeps < 1 ? 1 : substeps;
    float h, inv_h, inv_dt;
    B3Soft cs, ss;
    b3_soft_step_params(w, dt, substeps, &h, &inv_h, &inv_dt, &cs, &ss);
    b3_prepare_contacts(w, cs, ss);
    b3_prepare_joints(w, h);
    b3_warm_start(w);
    b3_warm_start_joints(w);
    for (int s = 0; s < subs; s++) {
        b3_step_sub(w, h, inv_h, inv_dt);
    }
    b3_finalize_transforms(w);
}

/* Hybrid: cooperative block-per-world find in its own launch, then the
* unchanged thread-per-world tail. State-identical to the serial kernel. */
__global__ void bench_step_tail(B3World* worlds, int n, float dt,
        int substeps) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n || dt <= 0.0f) {
        return;
    }
    B3World* w = worlds + i;
    int subs = substeps < 1 ? 1 : substeps;
    float h, inv_h, inv_dt;
    B3Soft cs, ss;
    b3_soft_step_params(w, dt, substeps, &h, &inv_h, &inv_dt, &cs, &ss);
    b3_prepare_contacts(w, cs, ss);
    b3_prepare_joints(w, h);
    b3_warm_start(w);
    b3_warm_start_joints(w);
    for (int s = 0; s < subs; s++) {
        b3_step_sub(w, h, inv_h, inv_dt);
    }
    b3_finalize_transforms(w);
}
#endif

#ifdef __CUDACC__
/* Prototype: block-per-world colored Gauss-Seidel for contacts. Contacts
* are greedily colored by shared-body conflict; within a color, contacts
* touch disjoint bodies and solve concurrently per lane using the verbatim
* serial per-contact math on shared-resident bodies. Between colors and
* iterations there is a full barrier, so the schedule is deterministic
* (colors ascending, contacts within a color write disjoint state).
* Ordering differs from the serial sweep, so trajectories are not bitwise
* identical - this is a Jacobi-within-color, GS-across-colors variant. */
__device__ void block_solve_contacts(B3World* w, float inv_h,
        int use_bias) {
    __shared__ B3Body sh_b[B3_MAX_BODIES];
    __shared__ int sh_color[B3_MAX_CONTACTS];
    __shared__ int sh_maxc;
    const int tid = threadIdx.x;
    const int nt = B3_BLOCK_T;
    for (int k = tid; k < w->body_count; k += nt) {
        sh_b[k] = w->bodies[k];
    }
    if (tid == 0) {
        int maxc = 0;
        for (int c = 0; c < w->contact_count; c++) {
            const B3Contact* cc = &w->contacts[c];
            uint64_t used0 = 0;
            uint64_t used1 = 0;
            for (int q = 0; q < c; q++) {
                const B3Contact* cq = &w->contacts[q];
                if (cq->body_a != cc->body_a && cq->body_a != cc->body_b
                    && cq->body_b != cc->body_a
                    && cq->body_b != cc->body_b) {
                    continue;
                }
                if (sh_color[q] < 64) {
                    used0 |= 1ull << sh_color[q];
                } else {
                    used1 |= 1ull << (sh_color[q] - 64);
                }
            }
            int col = 0;
            for (;;) {
                uint64_t word = col < 64 ? used0 : used1;
                int bit = col & 63;
                if (!((word >> bit) & 1ull)) {
                    break;
                }
                col++;
            }
            sh_color[c] = col;
            if (col + 1 > maxc) {
                maxc = col + 1;
            }
        }
        sh_maxc = maxc;
    }
    __syncthreads();
    for (int color = 0; color < sh_maxc; color++) {
        for (int c = tid; c < w->contact_count; c += nt) {
            if (sh_color[c] != color) {
                continue;
            }
            B3Contact* cc = &w->contacts[c];
            b3_solve_one_contact(cc, &sh_b[cc->body_a],
                &sh_b[cc->body_b], inv_h, w->contact_speed, use_bias);
        }
        __syncthreads();
    }
    for (int k = tid; k < w->body_count; k += nt) {
        w->bodies[k].lin_vel = sh_b[k].lin_vel;
        w->bodies[k].ang_vel = sh_b[k].ang_vel;
        w->bodies[k].delta_pos = sh_b[k].delta_pos;
        w->bodies[k].delta_rot = sh_b[k].delta_rot;
    }
}

__global__ void bench_contacts_block(B3World* worlds, int n, float inv_h,
        int use_bias) {
    if (blockIdx.x < n) {
        block_solve_contacts(worlds + blockIdx.x, inv_h, use_bias);
    }
}

__global__ void bench_t_begin(B3World* worlds, int n, float dt,
        int substeps) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) {
        return;
    }
    B3World* w = worlds + i;
    float h, inv_h, inv_dt;
    B3Soft cs, ss;
    b3_soft_step_params(w, dt, substeps, &h, &inv_h, &inv_dt, &cs, &ss);
    b3_prepare_contacts(w, cs, ss);
    b3_prepare_joints(w, h);
    b3_warm_start(w);
    b3_warm_start_joints(w);
}

__global__ void bench_t_vel(B3World* worlds, int n, float h) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        b3_integrate_velocities(worlds + i, h);
    }
}

__global__ void bench_t_joint_bias_pos(B3World* worlds, int n, float dt,
        int substeps) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) {
        return;
    }
    B3World* w = worlds + i;
    float h, inv_h, inv_dt;
    B3Soft cs, ss;
    b3_soft_step_params(w, dt, substeps, &h, &inv_h, &inv_dt, &cs, &ss);
    b3_solve_joints(w, h, inv_h, 1);
    b3_integrate_positions(w, h, inv_dt, w->max_linear_speed);
}

__global__ void bench_t_joint_relax_rest(B3World* worlds, int n, float dt,
        int substeps) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) {
        return;
    }
    B3World* w = worlds + i;
    float h, inv_h, inv_dt;
    B3Soft cs, ss;
    b3_soft_step_params(w, dt, substeps, &h, &inv_h, &inv_dt, &cs, &ss);
    b3_solve_joints(w, h, inv_h, 0);
#ifndef B3_SKIP_RESTITUTION
    b3_apply_restitution(w, w->restitution_threshold);
#endif
}
#endif

__global__ void bench_t_finalize(B3World* worlds, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        b3_finalize_transforms(worlds + i);
    }
}

static int add_body(B3World* w, int type, B3Vec3 pos, bool sphere,
        B3Vec3 half, float density = 1.0f) {
    B3BodyDef bd = b3_default_body();
    bd.type = type;
    bd.position = pos;
    int id = b3_create_body(w, &bd);
    B3ShapeDef sd = b3_default_shape();
    sd.density = density;
    sd.friction = 0.5f;
    sd.restitution = 0;
    if (sphere) b3_create_sphere(w, id, b3_v(0,0,0), half.x, &sd);
    else b3_create_box(w, id, half, &sd);
    b3_finalize_mass(w, id);
    return id;
}

static void scene(B3World* w, const char* name, int n) {
    b3_world_init(w);
    bool flight = !strcmp(name, "flight"), chain = !strcmp(name, "chain");
    if (!flight && !chain)
        add_body(w, B3_STATIC, b3_v(0,-0.5f,0), false, b3_v(100,0.5f,100), 0);
    if (chain)
        add_body(w, B3_STATIC, b3_v(0, n + 2.0f, 0), false, b3_v(0.1f,0.1f,0.1f), 0);
    int width = (int)ceilf(cbrtf((float)n));
    for (int i = 0; i < n; i++) {
        B3Vec3 pos, half = b3_v(0.5f,0.5f,0.5f);
        if (flight) pos = b3_v(2.0f*i, 100, 0);
        else if (chain) {
            pos = b3_v(0, n + 1.5f - i, 0);
            half = b3_v(0.1f,0.5f,0.1f);
        } else if (!strcmp(name, "stack")) pos = b3_v(0, 0.5f + i*1.01f, 0);
        else pos = b3_v((i%width)*1.0f, 0.5f + (i/(width*width))*1.0f,
            ((i/width)%width)*1.0f);
        int id = add_body(w, B3_DYNAMIC, pos, !strcmp(name,"grains"), half);
        if (chain) {
            int j = b3_create_revolute(w, id-1, id,
                b3_v(0,i ? -0.5f : 0,0), b3_v(0,0.5f,0), b3_v(0,0,1));
            (void)j;
#if B3_BENCH_CHAIN_SPRING
            // Opt-in math workload: the free chain does not need twist angles.
            b3_joint_set_spring(w, j, 0.0f, 2.0f, 0.7f);
            b3_joint_enable_spring(w, j, 1);
#endif
            // Exercise the chain away from its vertical equilibrium.
            if (i == n-1) w->bodies[id].lin_vel.x = 1.0f;
        }
    }
}

static double seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static bool finite_body(const B3Body& b) {
    return isfinite(b.position.x) && isfinite(b.position.y) && isfinite(b.position.z)
        && isfinite(b.lin_vel.x) && isfinite(b.lin_vel.y) && isfinite(b.lin_vel.z)
        && isfinite(b.ang_vel.x) && isfinite(b.ang_vel.y) && isfinite(b.ang_vel.z)
        && isfinite(b.rotation.s) && isfinite(b.rotation.v.x)
        && isfinite(b.rotation.v.y) && isfinite(b.rotation.v.z);
}

int main(int argc, char** argv) {
    const char* name = "stack";
    int worlds = 128, bodies = 16, steps = 120, subs = 4, repeats = 3, threads = 64;
    float dt = 1.0f/60;
    bool find = false, cpu = false, block = false, hybrid = false;
    bool color = false;
    for (int i=1; i<argc; i++) {
        if (!strcmp(argv[i],"--cpu")) { cpu=true; continue; }
        if (i+1 == argc) { fprintf(stderr,"Missing option value\n"); return 2; }
        const char* key=argv[i++]; const char* val=argv[i];
        if (!strcmp(key,"--scene")) name=val;
        else if (!strcmp(key,"--worlds")) worlds=atoi(val);
        else if (!strcmp(key,"--bodies")) bodies=atoi(val);
        else if (!strcmp(key,"--steps")) steps=atoi(val);
        else if (!strcmp(key,"--substeps")) subs=atoi(val);
        else if (!strcmp(key,"--repeats")) repeats=atoi(val);
        else if (!strcmp(key,"--threads")) threads=atoi(val);
        else if (!strcmp(key,"--dt")) dt=strtof(val, NULL);
        else if (!strcmp(key,"--phase")) {
            if (strcmp(val,"find") && strcmp(val,"step")) return 2;
            find=!strcmp(val,"find");
        } else if (!strcmp(key,"--mode")) {
            if (strcmp(val,"thread") && strcmp(val,"block")
                && strcmp(val,"hybrid") && strcmp(val,"color")) return 2;
            block=!strcmp(val,"block");
            hybrid=!strcmp(val,"hybrid");
            color=!strcmp(val,"color");
        } else { fprintf(stderr,"Unknown option: %s\n", key); return 2; }
    }
    bool flight=!strcmp(name,"flight"), chain=!strcmp(name,"chain");
    if (!flight && !chain && strcmp(name,"stack") && strcmp(name,"grains")) return 2;
    int total=bodies + !flight;
    if (worlds<1 || bodies<1 || steps<1 || subs<1 || repeats<1 || threads<1 || threads>1024
        || !isfinite(dt) || dt<=0 || total>B3_MAX_BODIES || total>B3_MAX_SHAPES
        || (chain && bodies>B3_MAX_JOINTS)) {
        fprintf(stderr,"Invalid settings or body/shape/joint capacity exceeded\n"); return 2;
    }
#ifndef __CUDACC__
    if (!cpu) { fprintf(stderr,"CPU build requires --cpu\n"); return 2; }
#endif
    B3World initial;
    scene(&initial,name,bodies);
    // Warm the contact cache for the isolated collision benchmark.
    if (find) b3_find_contacts(&initial);
    std::vector<B3World> host(worlds, initial);
    std::vector<double> times;
#ifdef __CUDACC__
    B3World* dev=NULL;
    cudaEvent_t start, stop;
    if (!cpu) {
        int count=0; CUDA(cudaGetDeviceCount(&count));
        if (!count) { fprintf(stderr,"No CUDA devices; benchmark failed\n"); return 2; }
        cudaDeviceProp prop; CUDA(cudaGetDeviceProperties(&prop,0));
        fprintf(stderr,"device=%s world_bytes=%zu capacities=%d/%d/%d/%d\n",prop.name,
            sizeof(B3World), B3_MAX_BODIES,B3_MAX_SHAPES,B3_MAX_CONTACTS,B3_MAX_JOINTS);
        CUDA(cudaMalloc((void**)&dev,host.size()*sizeof(B3World)));
        CUDA(cudaEventCreate(&start)); CUDA(cudaEventCreate(&stop));
        cudaFuncSetAttribute(bench_find_block,
            cudaFuncAttributePreferredSharedMemoryCarveout, 100);
    }
#endif
    // One untimed trial, then independent trials from identical initial state.
    for (int trial=-1; trial<repeats; trial++) {
        std::fill(host.begin(),host.end(),initial);
        double elapsed=0;
        if (cpu) {
            double t=seconds();
            for (int s=0; s<steps; s++) for (int i=0; i<worlds; i++) {
                if (find) b3_find_contacts(&host[i]); else b3_step(&host[i],dt,subs);
            }
            elapsed=seconds()-t;
        }
#ifdef __CUDACC__
        else {
            CUDA(cudaMemcpy(dev,host.data(),host.size()*sizeof(B3World),cudaMemcpyHostToDevice));
            CUDA(cudaEventRecord(start));
            for (int s=0; s<steps; s++) {
                if (color) {
                    bench_find_block<<<worlds,B3_BLOCK_T>>>(dev,worlds);
                    bench_t_begin<<<(worlds+threads-1)/threads,threads>>>(
                        dev,worlds,dt,subs);
                    float hh = dt / (float)subs;
                    float idt = 1.0f / dt;
                    float ih = (float)subs * idt;
                    for (int q = 0; q < subs; q++) {
                        bench_t_vel<<<(worlds+threads-1)/threads,threads>>>(
                            dev,worlds,hh);
                        bench_contacts_block<<<worlds,B3_BLOCK_T>>>(
                            dev,worlds,ih,1);
                        bench_t_joint_bias_pos<<<(worlds+threads-1)/threads,
                            threads>>>(dev,worlds,dt,subs);
                        bench_contacts_block<<<worlds,B3_BLOCK_T>>>(
                            dev,worlds,ih,0);
                        bench_t_joint_relax_rest<<<(worlds+threads-1)/threads,
                            threads>>>(dev,worlds,dt,subs);
                    }
                    bench_t_finalize<<<(worlds+threads-1)/threads,threads>>>(
                        dev,worlds);
                } else if (hybrid) {
                    bench_find_block<<<worlds,B3_BLOCK_T>>>(dev,worlds);
                    bench_step_tail<<<(worlds+threads-1)/threads,threads>>>(
                        dev,worlds,dt,subs);
                } else if (block) {
                    if (find) bench_find_block<<<worlds,B3_BLOCK_T>>>(dev,worlds);
                    else bench_step_block<<<worlds,B3_BLOCK_T>>>(dev,worlds,dt,subs);
                } else {
                    if (find) bench_find<<<(worlds+threads-1)/threads,threads>>>(dev,worlds);
                    else b3_step_kernel<<<(worlds+threads-1)/threads,threads>>>(dev,worlds,dt,subs);
                }
            }
            CUDA(cudaGetLastError()); CUDA(cudaEventRecord(stop)); CUDA(cudaEventSynchronize(stop));
            float ms=0; CUDA(cudaEventElapsedTime(&ms,start,stop)); elapsed=ms/1000.0;
            CUDA(cudaMemcpy(host.data(),dev,host.size()*sizeof(B3World),cudaMemcpyDeviceToHost));
        }
#endif
        if (trial>=0) times.push_back(elapsed);
    }
    // Independent CPU execution checks GPU consistency, not physical truth.
    B3World ref=initial;
    for (int s=0;s<steps;s++) { if (find) b3_find_contacts(&ref); else b3_step(&ref,dt,subs); }
    float pos_delta=0, vel_delta=0, anchor_gap=0, penetration=0, flight_error=0, stack_error=0;
    int nonfinite=0, max_contacts=0, saturated=0;
    for (int e=0;e<worlds;e++) {
        B3World& w=host[e];
        max_contacts=std::max(max_contacts,w.contact_count);
        saturated += w.contact_count==B3_MAX_CONTACTS;
        for (int b=0;b<w.body_count;b++) {
            const B3Body& v=w.bodies[b];
            if (!finite_body(v)) { nonfinite++; continue; }
            pos_delta=std::max(pos_delta,b3_len(b3_sub(v.position,ref.bodies[b].position)));
            vel_delta=std::max(vel_delta,b3_len(b3_sub(v.lin_vel,ref.bodies[b].lin_vel)));
            if (!strcmp(name,"stack") && b>0)
                stack_error=std::max(stack_error,fabsf(v.position.y-(b-0.5f)));
            if (flight && !find) {
                double t=(double)steps*dt;
                flight_error=std::max(flight_error,(float)fabs(v.position.y-(100-5*t*t)));
            }
        }
        for (int j=0;j<w.joint_count;j++) {
            B3Joint& q=w.joints[j];
            B3Body& a=w.bodies[q.body_a]; B3Body& b=w.bodies[q.body_b];
            B3Vec3 pa=b3_add(a.position,b3_rotate(a.rotation,q.local_anchor_a));
            B3Vec3 pb=b3_add(b.position,b3_rotate(b.rotation,q.local_anchor_b));
            anchor_gap=std::max(anchor_gap,b3_len(b3_sub(pa,pb)));
        }
        // Rebuild manifolds at final transforms, outside the timed region.
        b3_find_contacts(&w);
        saturated += w.contact_count==B3_MAX_CONTACTS;
        for (int c=0;c<w.contact_count;c++) for (int p=0;p<w.contacts[c].point_count;p++) {
            const B3Contact& q=w.contacts[c];
            const B3Point& point=q.points[p];
            float sep=point.base_sep+b3_dot(b3_sub(point.r_b,point.r_a),q.normal);
            penetration=std::max(penetration,-sep);
        }
    }
    std::sort(times.begin(),times.end());
    double t=times[times.size()/2];
    printf("scene,backend,phase,worlds,dynamic_bodies,steps,substeps,dt,threads,world_bytes,median_ms,min_ms,max_ms,world_steps_s,body_steps_s,final_contacts,cpu_pos_delta,cpu_vel_delta,final_anchor_gap,final_penetration,flight_analytic_error,final_stack_height_error,nonfinite,saturated_samples\n");
    printf("%s,%s,%s,%d,%d,%d,%d,%.9g,%d,%zu,%.6f,%.6f,%.6f,%.3f,%.3f,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d\n",
        name,cpu?"cpu":"cuda",find?"find":"step",worlds,bodies,steps,subs,dt,threads,sizeof(B3World),
        t*1000,times.front()*1000,times.back()*1000,(double)worlds*steps/t,(double)worlds*steps*bodies/t,
        max_contacts,pos_delta,vel_delta,anchor_gap,penetration,flight_error,stack_error,nonfinite,saturated);
#ifdef __CUDACC__
    if (!cpu) { CUDA(cudaFree(dev)); CUDA(cudaEventDestroy(start)); CUDA(cudaEventDestroy(stop)); }
#endif
    return nonfinite || saturated ? 1 : 0;
}
