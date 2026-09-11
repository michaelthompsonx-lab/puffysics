#ifndef NBODY_TREE_GPU_CUH
#define NBODY_TREE_GPU_CUH

/* Device Treecode2: random CS + quadrupole + Barnes opening, built as a
 * Karras binary radix tree on 30-bit Morton codes. 3-bit prefixes are
 * the same octree cells as the CPU walker; extra binary nodes only
 * refine the walk. Lock-free — no per-cell atomics in a warp.
 *
 *   NbodyTreeGpu
 *   nbody_tree_gpu_init / nbody_tree_gpu_free
 *   nbody_tree_gpu_upload / nbody_tree_gpu_download
 *   nbody_tree_gpu_acceleration / nbody_tree_gpu_step
 */
#ifndef NBODY_TREE_CUH
#error "include nbody_tree.cuh, not nbody_tree_gpu.cuh"
#endif

#define NBODY_TREE_GPU_END 0x7FFFFFFF
#define NBODY_TREE_GPU_BLOCK 256
#define NBODY_TREE_GPU_WALK 128

typedef struct NbodyTreeCS {
    float R[9], S, invS, Tx, Ty, Tz, theta;
    int quadrupole, soft_quad, cm_open;
} NbodyTreeCS;

typedef struct NbodyTreeGpuI {
    int left, right, parent, more, next, first, last;
    float mass, cx, cy, cz, gx, gy, gz, hsim;
    float qxx, qyy, qzz, qxy, qxz, qyz, qtilde;
} NbodyTreeGpuI;

typedef struct NbodyTreeGpu {
    NbodyTreeConfig cfg;
    unsigned rng;
    float R[9], S, Tx, Ty, Tz;
    NbodyPoint *p;
    NbodyVec *v, *a;
    float *tx, *ty, *tz;
    unsigned long long *keys[2];
    int *ids[2];
    int *order, *rank, *leaf_next, *leaf_parent, *ready;
    int *hist, *cursor;
    int *stk_node, *stk_succ;
    unsigned char *stk_st;
    NbodyTreeGpuI *nodes;
    float *rmax, *extent, *root_half;
    int *root;
    cudaStream_t stream;
    int n, n_int;
} NbodyTreeGpu;

static int nbody_tree_gpu_blocks(int n, int b) {
    return n <= 0 ? 1 : (n + b - 1) / b;
}

static __device__ __host__ void nbody_tree_gpu_mul(const float R[9],
        float x, float y, float z, float *ox, float *oy, float *oz) {
    *ox = R[0] * x + R[1] * y + R[2] * z;
    *oy = R[3] * x + R[4] * y + R[5] * z;
    *oz = R[6] * x + R[7] * y + R[8] * z;
}

static __device__ __host__ void nbody_tree_gpu_mulT(const float R[9],
        float x, float y, float z, float *ox, float *oy, float *oz) {
    *ox = R[0] * x + R[3] * y + R[6] * z;
    *oy = R[1] * x + R[4] * y + R[7] * z;
    *oz = R[2] * x + R[5] * y + R[8] * z;
}

static __device__ void nbody_tree_gpu_from_tree(const NbodyTreeCS *c,
        float x, float y, float z, float *ox, float *oy, float *oz) {
    float ux, uy, uz;
    nbody_tree_gpu_mulT(c->R, x, y, z, &ux, &uy, &uz);
    *ox = c->Tx + c->invS * ux;
    *oy = c->Ty + c->invS * uy;
    *oz = c->Tz + c->invS * uz;
}

static __device__ void nbody_tree_gpu_atomic_maxf(float *addr, float val) {
    int *iaddr = (int *)addr;
    int old = *iaddr, assumed;
    do {
        assumed = old;
        old = atomicCAS(iaddr, assumed,
            __float_as_int(fmaxf(val, __int_as_float(assumed))));
    } while (assumed != old);
}

static __device__ unsigned nbody_tree_gpu_expand(unsigned v) {
    v &= 1023u;
    v = (v | (v << 16)) & 0x030000FFu;
    v = (v | (v << 8)) & 0x0300F00Fu;
    v = (v | (v << 4)) & 0x030C30C3u;
    v = (v | (v << 2)) & 0x09249249u;
    return v;
}

static __device__ int nbody_tree_gpu_delta(const unsigned long long *k,
        int n, int i, int j) {
    unsigned long long x;
    if (j < 0 || j >= n || i < 0 || i >= n) {
        return -1;
    }
    x = k[i] ^ k[j];
    if (x == 0) {
        return 64 + __clz(i ^ j);
    }
    return __clzll(x);
}

static __device__ void nbody_tree_gpu_add_q(NbodyTreeGpuI *c, float m,
        float dx, float dy, float dz) {
    float r2 = dx * dx + dy * dy + dz * dz;
    c->qxx += m * (3.0f * dx * dx - r2);
    c->qyy += m * (3.0f * dy * dy - r2);
    c->qzz += m * (3.0f * dz * dz - r2);
    c->qxy += m * (3.0f * dx * dy);
    c->qxz += m * (3.0f * dx * dz);
    c->qyz += m * (3.0f * dy * dz);
    c->qtilde += m * r2;
}

static __device__ void nbody_tree_gpu_add_cell(const NbodyTreeGpuI *c,
        NbodyPoint dest, float G, float eps2, int use_q, int use_soft,
        float *ax, float *ay, float *az) {
    float sx, sy, sz, r2, re2, inv, inv3, inv5, inv7;
    float Qsx, Qsy, Qsz, qform, corr;
    if (c->mass == 0.0f) {
        return;
    }
    sx = dest.x - c->cx;
    sy = dest.y - c->cy;
    sz = dest.z - c->cz;
    r2 = sx * sx + sy * sy + sz * sz;
    re2 = r2 + eps2;
    inv = rsqrtf(re2);
    inv3 = inv * inv * inv;
    *ax -= G * c->mass * sx * inv3;
    *ay -= G * c->mass * sy * inv3;
    *az -= G * c->mass * sz * inv3;
    if (!use_q) {
        return;
    }
    Qsx = c->qxx * sx + c->qxy * sy + c->qxz * sz;
    Qsy = c->qxy * sx + c->qyy * sy + c->qyz * sz;
    Qsz = c->qxz * sx + c->qyz * sy + c->qzz * sz;
    qform = sx * Qsx + sy * Qsy + sz * Qsz;
    corr = use_soft ? qform - eps2 * c->qtilde : qform;
    inv5 = inv3 * inv * inv;
    inv7 = inv5 * inv * inv;
    *ax += G * Qsx * inv5 - 2.5f * G * corr * sx * inv7;
    *ay += G * Qsy * inv5 - 2.5f * G * corr * sy * inv7;
    *az += G * Qsz * inv5 - 2.5f * G * corr * sz * inv7;
}

static __global__ void nbody_tree_gpu_rmax_k(const NbodyPoint *p, int n,
        float *rmax) {
    float local = 0.0f, r;
    int i, stride = blockDim.x * gridDim.x;
    for (i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += stride) {
        r = p[i].x * p[i].x + p[i].y * p[i].y + p[i].z * p[i].z;
        local = fmaxf(local, r);
    }
    __shared__ float sh[NBODY_TREE_GPU_BLOCK];
    sh[threadIdx.x] = local;
    __syncthreads();
    for (i = blockDim.x / 2; i > 0; i >>= 1) {
        if (threadIdx.x < i) {
            sh[threadIdx.x] = fmaxf(sh[threadIdx.x], sh[threadIdx.x + i]);
        }
        __syncthreads();
    }
    if (threadIdx.x == 0) {
        nbody_tree_gpu_atomic_maxf(rmax, sh[0]);
    }
}

static __global__ void nbody_tree_gpu_xform_k(const NbodyPoint *p,
        float *tx, float *ty, float *tz, int n, NbodyTreeCS cs,
        float *extent) {
    float x, y, z, sx, sy, sz, ax, local = 0.0f;
    int i, stride = blockDim.x * gridDim.x;
    for (i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += stride) {
        sx = cs.S * (p[i].x - cs.Tx);
        sy = cs.S * (p[i].y - cs.Ty);
        sz = cs.S * (p[i].z - cs.Tz);
        nbody_tree_gpu_mul(cs.R, sx, sy, sz, &x, &y, &z);
        tx[i] = x;
        ty[i] = y;
        tz[i] = z;
        ax = fabsf(x);
        if (fabsf(y) > ax) {
            ax = fabsf(y);
        }
        if (fabsf(z) > ax) {
            ax = fabsf(z);
        }
        local = fmaxf(local, ax);
    }
    __shared__ float sh[NBODY_TREE_GPU_BLOCK];
    sh[threadIdx.x] = local;
    __syncthreads();
    for (i = blockDim.x / 2; i > 0; i >>= 1) {
        if (threadIdx.x < i) {
            sh[threadIdx.x] = fmaxf(sh[threadIdx.x], sh[threadIdx.x + i]);
        }
        __syncthreads();
    }
    if (threadIdx.x == 0) {
        nbody_tree_gpu_atomic_maxf(extent, sh[0]);
    }
}

static __global__ void nbody_tree_gpu_half_k(const float *extent,
        float *root_half) {
    float m, h;
    if (threadIdx.x || blockIdx.x) {
        return;
    }
    m = extent[0] * 1.000001f;
    h = 1.0f;
    while (h < m && h < 1.0e20f) {
        h *= 2.0f;
    }
    *root_half = h;
}

static __global__ void nbody_tree_gpu_morton_k(const float *tx,
        const float *ty, const float *tz, unsigned long long *keys,
        int *ids, int n, const float *root_half) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    float h, s, u, v, w;
    int xi, yi, zi;
    unsigned morton;
    if (i >= n) {
        return;
    }
    h = *root_half;
    s = (h > 0.0f) ? (512.0f / h) : 0.0f;
    u = (tx[i] + h) * s;
    v = (ty[i] + h) * s;
    w = (tz[i] + h) * s;
    xi = (int)u;
    yi = (int)v;
    zi = (int)w;
    if (xi < 0) {
        xi = 0;
    }
    if (yi < 0) {
        yi = 0;
    }
    if (zi < 0) {
        zi = 0;
    }
    if (xi > 1023) {
        xi = 1023;
    }
    if (yi > 1023) {
        yi = 1023;
    }
    if (zi > 1023) {
        zi = 1023;
    }
    morton = (nbody_tree_gpu_expand((unsigned)zi) << 2)
        | (nbody_tree_gpu_expand((unsigned)yi) << 1)
        | nbody_tree_gpu_expand((unsigned)xi);
    keys[i] = ((unsigned long long)morton << 32) | (unsigned)i;
    ids[i] = i;
}

static __global__ void nbody_tree_gpu_hist_k(const unsigned long long *keys,
        int n, int shift, int *hist) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        atomicAdd(&hist[(int)((keys[i] >> shift) & 255ull)], 1);
    }
}

static __global__ void nbody_tree_gpu_scan_k(const int *hist, int *cursor) {
    int i, sum;
    if (threadIdx.x || blockIdx.x) {
        return;
    }
    sum = 0;
    for (i = 0; i < 256; i++) {
        cursor[i] = sum;
        sum += hist[i];
    }
}

static __global__ void nbody_tree_gpu_scatter_k(
        const unsigned long long *ki, const int *vi,
        unsigned long long *ko, int *vo, int n, int shift,
        const int *cursor) {
    int local[256];
    int i, d, pos;
    if (threadIdx.x || blockIdx.x) {
        return;
    }
    for (i = 0; i < 256; i++) {
        local[i] = cursor[i];
    }
    for (i = 0; i < n; i++) {
        d = (int)((ki[i] >> shift) & 255ull);
        pos = local[d]++;
        if (pos >= 0 && pos < n) {
            ko[pos] = ki[i];
            vo[pos] = vi[i];
        }
    }
}

static __global__ void nbody_tree_gpu_rank_k(const int *order, int *rank,
        int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        rank[order[i]] = i;
    }
}

static __global__ void nbody_tree_gpu_clear_nodes_k(NbodyTreeGpuI *nodes,
        int n_int, int *root) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i == 0 && root) {
        *root = -1;
    }
    if (i >= n_int) {
        return;
    }
    nodes[i].parent = -1;
    nodes[i].left = nodes[i].right = NBODY_TREE_GPU_END;
    nodes[i].more = nodes[i].next = NBODY_TREE_GPU_END;
}

static __global__ void nbody_tree_gpu_parents_k(NbodyTreeGpuI *nodes,
        int *leaf_parent, int *root, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int left, right;
    if (i >= n - 1) {
        return;
    }
    left = nodes[i].left;
    right = nodes[i].right;
    if (left >= 0) {
        nodes[left].parent = i;
    } else {
        leaf_parent[~left] = i;
    }
    if (right >= 0) {
        nodes[right].parent = i;
    } else {
        leaf_parent[~right] = i;
    }
    if (nodes[i].first == 0 && nodes[i].last == n - 1) {
        *root = i;
    }
}

static __global__ void nbody_tree_gpu_karras_k(const unsigned long long *keys,
        NbodyTreeGpuI *nodes, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int d, delta_min, lmax, l, t, j, first, last, split, left, right;
    if (i >= n - 1) {
        return;
    }
    d = (nbody_tree_gpu_delta(keys, n, i, i + 1)
        >= nbody_tree_gpu_delta(keys, n, i, i - 1)) ? 1 : -1;
    delta_min = nbody_tree_gpu_delta(keys, n, i, i - d);
    lmax = 2;
    while (nbody_tree_gpu_delta(keys, n, i, i + lmax * d) > delta_min) {
        lmax <<= 1;
    }
    l = 0;
    for (t = lmax >> 1; t >= 1; t >>= 1) {
        if (nbody_tree_gpu_delta(keys, n, i, i + (l + t) * d) > delta_min) {
            l += t;
        }
    }
    j = i + l * d;
    first = i < j ? i : j;
    last = i > j ? i : j;
    {
        unsigned long long fc = keys[first], lc = keys[last];
        int common, stride, middle, prefix;
        if (fc == lc) {
            split = (first + last) >> 1;
        } else {
            common = __clzll(fc ^ lc);
            split = first;
            stride = last - first;
            do {
                stride = (stride + 1) >> 1;
                middle = split + stride;
                if (middle < last) {
                    prefix = __clzll(fc ^ keys[middle]);
                    if (prefix > common) {
                        split = middle;
                    }
                }
            } while (stride > 1);
        }
    }
    left = (split == first) ? ~split : split;
    right = (split + 1 == last) ? ~(split + 1) : (split + 1);
    nodes[i].left = left;
    nodes[i].right = right;
    nodes[i].first = first;
    nodes[i].last = last;
    nodes[i].mass = 0.0f;
}

static __global__ void nbody_tree_gpu_clear_ready_k(int *ready, int n_int) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n_int) {
        ready[i] = 0;
    }
}

static __device__ void nbody_tree_gpu_load_child(int h,
        const NbodyTreeGpuI *nodes, const NbodyPoint *p, const int *order,
        float *m, float *cx, float *cy, float *cz,
        float *qxx, float *qyy, float *qzz, float *qxy, float *qxz,
        float *qyz, float *qtilde) {
    const NbodyTreeGpuI *c;
    int b;
    if (h < 0) {
        b = order[~h];
        *m = p[b].mass;
        *cx = p[b].x;
        *cy = p[b].y;
        *cz = p[b].z;
        *qxx = *qyy = *qzz = *qxy = *qxz = *qyz = *qtilde = 0.0f;
        return;
    }
    c = &nodes[h];
    *m = c->mass;
    *cx = c->cx;
    *cy = c->cy;
    *cz = c->cz;
    *qxx = c->qxx;
    *qyy = c->qyy;
    *qzz = c->qzz;
    *qxy = c->qxy;
    *qxz = c->qxz;
    *qyz = c->qyz;
    *qtilde = c->qtilde;
}

static __device__ void nbody_tree_gpu_geom(NbodyTreeGpuI *nd,
        const unsigned long long *keys, float root_half,
        const NbodyTreeCS *cs) {
    unsigned mi, mj, xorv, morton, k, oct, ix, iy, iz;
    int common, level;
    float width, gtx, gty, gtz;
    mi = (unsigned)(keys[nd->first] >> 32);
    mj = (unsigned)(keys[nd->last] >> 32);
    xorv = mi ^ mj;
    common = xorv ? (__clz(xorv) - 2) : 30;
    if (common < 0) {
        common = 0;
    }
    if (common > 30) {
        common = 30;
    }
    level = common / 3;
    morton = mi;
    ix = iy = iz = 0;
    for (k = 0; k < (unsigned)level; k++) {
        oct = (morton >> (30u - 3u * (k + 1u))) & 7u;
        ix = (ix << 1) | (oct & 1u);
        iy = (iy << 1) | ((oct >> 1) & 1u);
        iz = (iz << 1) | ((oct >> 2) & 1u);
    }
    width = (2.0f * root_half) / (float)(1 << level);
    gtx = -root_half + ((float)ix + 0.5f) * width;
    gty = -root_half + ((float)iy + 0.5f) * width;
    gtz = -root_half + ((float)iz + 0.5f) * width;
    nbody_tree_gpu_from_tree(cs, gtx, gty, gtz, &nd->gx, &nd->gy, &nd->gz);
    nd->hsim = (0.5f * width) * cs->invS;
}

static __global__ void nbody_tree_gpu_moments_k(NbodyTreeGpuI *nodes,
        const NbodyPoint *p, const int *order, const int *leaf_parent,
        int *ready, const unsigned long long *keys, int n,
        float root_half, NbodyTreeCS cs) {
    int s = blockIdx.x * blockDim.x + threadIdx.x;
    int node, h;
    float mL, mR, cxL, cyL, czL, cxR, cyR, czR, inv;
    float qxxL, qyyL, qzzL, qxyL, qxzL, qyzL, qtL;
    float qxxR, qyyR, qzzR, qxyR, qxzR, qyzR, qtR;
    NbodyTreeGpuI *nd;
    if (s >= n) {
        return;
    }
    node = leaf_parent[s];
    while (node >= 0) {
        if (atomicAdd(&ready[node], 1) != 1) {
            return;
        }
        __threadfence();
        nd = &nodes[node];
        nbody_tree_gpu_load_child(nd->left, nodes, p, order, &mL, &cxL,
            &cyL, &czL, &qxxL, &qyyL, &qzzL, &qxyL, &qxzL, &qyzL, &qtL);
        nbody_tree_gpu_load_child(nd->right, nodes, p, order, &mR, &cxR,
            &cyR, &czR, &qxxR, &qyyR, &qzzR, &qxyR, &qxzR, &qyzR, &qtR);
        nd->mass = mL + mR;
        if (nd->mass > 0.0f) {
            inv = 1.0f / nd->mass;
            nd->cx = (mL * cxL + mR * cxR) * inv;
            nd->cy = (mL * cyL + mR * cyR) * inv;
            nd->cz = (mL * czL + mR * czR) * inv;
        } else {
            nd->cx = nd->cy = nd->cz = 0.0f;
        }
        nd->qxx = nd->qyy = nd->qzz = 0.0f;
        nd->qxy = nd->qxz = nd->qyz = nd->qtilde = 0.0f;
        if (cs.quadrupole) {
            nd->qxx = qxxL + qxxR;
            nd->qyy = qyyL + qyyR;
            nd->qzz = qzzL + qzzR;
            nd->qxy = qxyL + qxyR;
            nd->qxz = qxzL + qxzR;
            nd->qyz = qyzL + qyzR;
            nd->qtilde = qtL + qtR;
            if (mL != 0.0f) {
                nbody_tree_gpu_add_q(nd, mL, cxL - nd->cx, cyL - nd->cy,
                    czL - nd->cz);
            }
            if (mR != 0.0f) {
                nbody_tree_gpu_add_q(nd, mR, cxR - nd->cx, cyR - nd->cy,
                    czR - nd->cz);
            }
        }
        nbody_tree_gpu_geom(nd, keys, root_half, &cs);
        __threadfence();
        h = nd->parent;
        node = h;
    }
}

static __global__ void nbody_tree_gpu_links_k(NbodyTreeGpuI *nodes,
        int *leaf_next, int root, int n, int *stk_node, int *stk_succ,
        unsigned char *stk_st) {
    int sp, node, succ, left, right;
    unsigned char st;
    if (threadIdx.x || blockIdx.x || n < 2) {
        return;
    }
    sp = 0;
    stk_node[0] = root;
    stk_succ[0] = NBODY_TREE_GPU_END;
    stk_st[0] = 0;
    sp = 1;
    while (sp > 0) {
        node = stk_node[sp - 1];
        succ = stk_succ[sp - 1];
        st = stk_st[sp - 1];
        if (node < 0) {
            leaf_next[~node] = succ;
            sp--;
            continue;
        }
        left = nodes[node].left;
        right = nodes[node].right;
        if (sp >= 2 * n - 1) {
            return;
        }
        if (st == 0) {
            nodes[node].next = succ;
            nodes[node].more = left;
            stk_st[sp - 1] = 1;
            stk_node[sp] = right;
            stk_succ[sp] = succ;
            stk_st[sp] = 0;
            sp++;
        } else if (st == 1) {
            stk_st[sp - 1] = 2;
            stk_node[sp] = left;
            stk_succ[sp] = right;
            stk_st[sp] = 0;
            sp++;
        } else {
            sp--;
        }
    }
}

static __device__ int nbody_tree_gpu_must_open(const NbodyTreeGpuI *c,
        int dest_rank, float px, float py, float pz, const NbodyTreeCS *cs) {
    float sx, sy, sz, s2, dx, dy, dz, d, ell, lim;
    if (dest_rank >= c->first && dest_rank <= c->last) {
        return 1;
    }
    sx = px - c->cx;
    sy = py - c->cy;
    sz = pz - c->cz;
    s2 = sx * sx + sy * sy + sz * sz;
    dx = c->cx - c->gx;
    dy = c->cy - c->gy;
    dz = c->cz - c->gz;
    d = cs->cm_open ? sqrtf(dx * dx + dy * dy + dz * dz) : 0.0f;
    ell = 2.0f * c->hsim;
    lim = ell / cs->theta + d;
    return s2 < lim * lim;
}

static __global__ void nbody_tree_gpu_walk_k(const NbodyTreeGpuI *nodes,
        const NbodyPoint *p, NbodyVec *a, const int *order,
        const int *rank, const int *leaf_next, int n, int root,
        NbodyConfig g, NbodyTreeCS cs) {
    int dest = blockIdx.x * blockDim.x + threadIdx.x;
    int h, b, dest_rank, guard;
    float ax, ay, az, eps2;
    NbodyPoint destp, src;
    if (dest >= n) {
        return;
    }
    ax = ay = az = 0.0f;
    destp = p[dest];
    dest_rank = rank[dest];
    eps2 = g.softening * g.softening;
    if (n < 2) {
        a[dest].x = a[dest].y = a[dest].z = 0.0f;
        return;
    }
    h = root;
    guard = 0;
    while (h != NBODY_TREE_GPU_END && guard++ < 2 * n + 2) {
        if (h < 0) {
            b = order[~h];
            if (b != dest && p[b].mass != 0.0f) {
                src = p[b];
                nbody_add_source(destp, src, g.gravity, eps2, &ax, &ay, &az);
            }
            h = leaf_next[~h];
        } else if (nbody_tree_gpu_must_open(&nodes[h], dest_rank, destp.x,
                destp.y, destp.z, &cs)) {
            h = nodes[h].more;
        } else {
            nbody_tree_gpu_add_cell(&nodes[h], destp, g.gravity, eps2,
                cs.quadrupole, cs.soft_quad, &ax, &ay, &az);
            h = nodes[h].next;
        }
    }
    a[dest].x = ax;
    a[dest].y = ay;
    a[dest].z = az;
}

static cudaError_t nbody_tree_gpu_sort(NbodyTreeGpu *g) {
    cudaError_t err;
    int shift, src, n, b;
    n = g->n;
    b = nbody_tree_gpu_blocks(n, NBODY_TREE_GPU_BLOCK);
    src = 0;
    for (shift = 0; shift < 64; shift += 8) {
        err = cudaMemsetAsync(g->hist, 0, 256 * sizeof(int), g->stream);
        if (err != cudaSuccess) {
            return err;
        }
        nbody_tree_gpu_hist_k<<<b, NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
            g->keys[src], n, shift, g->hist);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
        nbody_tree_gpu_scan_k<<<1, 1, 0, g->stream>>>(g->hist, g->cursor);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
        nbody_tree_gpu_scatter_k<<<1, 1, 0, g->stream>>>(
            g->keys[src], g->ids[src], g->keys[src ^ 1], g->ids[src ^ 1],
            n, shift, g->cursor);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
        src ^= 1;
    }
    /* 8 passes: result in keys[0], ids[0] */
    return cudaSuccess;
}

static NbodyTreeCS nbody_tree_gpu_cs(const NbodyTreeGpu *g) {
    NbodyTreeCS c;
    int i;
    for (i = 0; i < 9; i++) {
        c.R[i] = g->R[i];
    }
    c.S = g->S;
    c.invS = (g->S != 0.0f) ? 1.0f / g->S : 1.0f;
    c.Tx = g->Tx;
    c.Ty = g->Ty;
    c.Tz = g->Tz;
    c.theta = g->cfg.theta;
    c.quadrupole = g->cfg.quadrupole;
    c.soft_quad = g->cfg.soft_quad;
    c.cm_open = g->cfg.cm_open;
    return c;
}

static cudaError_t nbody_tree_gpu_free(NbodyTreeGpu *g) {
    cudaError_t err, first = cudaSuccess;
    int i;
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
#define NBODY_TREE_GPU_FREE(ptr) do { \
    if (g->ptr) { \
        err = cudaFree(g->ptr); \
        if (err != cudaSuccess && first == cudaSuccess) { \
            first = err; \
        } \
    } \
} while (0)
    NBODY_TREE_GPU_FREE(p);
    NBODY_TREE_GPU_FREE(v);
    NBODY_TREE_GPU_FREE(a);
    NBODY_TREE_GPU_FREE(tx);
    NBODY_TREE_GPU_FREE(ty);
    NBODY_TREE_GPU_FREE(tz);
    for (i = 0; i < 2; i++) {
        if (g->keys[i]) {
            err = cudaFree(g->keys[i]);
            if (err != cudaSuccess && first == cudaSuccess) {
                first = err;
            }
        }
        if (g->ids[i]) {
            err = cudaFree(g->ids[i]);
            if (err != cudaSuccess && first == cudaSuccess) {
                first = err;
            }
        }
    }
    NBODY_TREE_GPU_FREE(order);
    NBODY_TREE_GPU_FREE(rank);
    NBODY_TREE_GPU_FREE(leaf_next);
    NBODY_TREE_GPU_FREE(leaf_parent);
    NBODY_TREE_GPU_FREE(ready);
    NBODY_TREE_GPU_FREE(hist);
    NBODY_TREE_GPU_FREE(cursor);
    NBODY_TREE_GPU_FREE(stk_node);
    NBODY_TREE_GPU_FREE(stk_succ);
    NBODY_TREE_GPU_FREE(stk_st);
    NBODY_TREE_GPU_FREE(nodes);
    NBODY_TREE_GPU_FREE(rmax);
    NBODY_TREE_GPU_FREE(extent);
    NBODY_TREE_GPU_FREE(root_half);
    NBODY_TREE_GPU_FREE(root);
#undef NBODY_TREE_GPU_FREE
    memset(g, 0, sizeof(*g));
    return first;
}

static cudaError_t nbody_tree_gpu_init(NbodyTreeGpu *g, int n) {
    cudaError_t err;
    size_t ni, ns;
    if (!g || n < 0) {
        return cudaErrorInvalidValue;
    }
    nbody_tree_gpu_free(g);
    g->cfg = nbody_tree_default();
    g->rng = g->cfg.seed;
    g->S = 1.0f;
    g->R[0] = g->R[4] = g->R[8] = 1.0f;
    g->n = n;
    g->n_int = n > 1 ? n - 1 : 0;
    if (n == 0) {
        return cudaSuccess;
    }
    err = cudaStreamCreateWithFlags(&g->stream, cudaStreamNonBlocking);
    ni = (size_t)g->n_int;
    ns = (size_t)n;
#define NBODY_TREE_GPU_ALLOC(ptr, bytes) do { \
    if (err == cudaSuccess && (bytes) > 0) { \
        err = cudaMalloc((void **)&(g->ptr), (bytes)); \
    } \
} while (0)
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->p, ns * sizeof(*g->p));
    }
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->v, ns * sizeof(*g->v));
    }
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->a, ns * sizeof(*g->a));
    }
    NBODY_TREE_GPU_ALLOC(tx, ns * sizeof(float));
    NBODY_TREE_GPU_ALLOC(ty, ns * sizeof(float));
    NBODY_TREE_GPU_ALLOC(tz, ns * sizeof(float));
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->keys[0], ns * sizeof(unsigned long long));
    }
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->keys[1], ns * sizeof(unsigned long long));
    }
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->ids[0], ns * sizeof(int));
    }
    if (err == cudaSuccess) {
        err = cudaMalloc((void **)&g->ids[1], ns * sizeof(int));
    }
    NBODY_TREE_GPU_ALLOC(order, ns * sizeof(int));
    NBODY_TREE_GPU_ALLOC(rank, ns * sizeof(int));
    NBODY_TREE_GPU_ALLOC(leaf_next, ns * sizeof(int));
    NBODY_TREE_GPU_ALLOC(leaf_parent, ns * sizeof(int));
    NBODY_TREE_GPU_ALLOC(ready, ni * sizeof(int));
    NBODY_TREE_GPU_ALLOC(hist, 256 * sizeof(int));
    NBODY_TREE_GPU_ALLOC(cursor, 256 * sizeof(int));
    NBODY_TREE_GPU_ALLOC(stk_node, ns * 2 * sizeof(int));
    NBODY_TREE_GPU_ALLOC(stk_succ, ns * 2 * sizeof(int));
    NBODY_TREE_GPU_ALLOC(stk_st, ns * 2 * sizeof(unsigned char));
    NBODY_TREE_GPU_ALLOC(nodes, ni * sizeof(*g->nodes));
    NBODY_TREE_GPU_ALLOC(rmax, sizeof(float));
    NBODY_TREE_GPU_ALLOC(extent, sizeof(float));
    NBODY_TREE_GPU_ALLOC(root_half, sizeof(float));
    NBODY_TREE_GPU_ALLOC(root, sizeof(int));
#undef NBODY_TREE_GPU_ALLOC
    if (err != cudaSuccess) {
        nbody_tree_gpu_free(g);
        return err;
    }
    return cudaSuccess;
}

static cudaError_t nbody_tree_gpu_upload(NbodyTreeGpu *g, NbodyPoint *p,
        NbodyVec *v) {
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
    err = cudaMemcpyAsync(g->p, p, (size_t)g->n * sizeof(*p),
        cudaMemcpyHostToDevice, g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    return cudaMemcpyAsync(g->v, v, (size_t)g->n * sizeof(*v),
        cudaMemcpyHostToDevice, g->stream);
}

static cudaError_t nbody_tree_gpu_download(NbodyTreeGpu *g, NbodyPoint *p,
        NbodyVec *v) {
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

static cudaError_t nbody_tree_gpu_acceleration(NbodyTreeGpu *g,
        NbodyConfig cfg) {
    cudaError_t err;
    NbodyTreeCS cs;
    int b, wb, n, n_int, hroot;
    float hrmax = 0.0f, hhalf;
    if (!g || !nbody_cfg_ok(cfg) || !nbody_tree_cfg_ok(g->cfg)) {
        return cudaErrorInvalidValue;
    }
    n = g->n;
    n_int = g->n_int;
    if (n == 0) {
        return cudaSuccess;
    }
    if (g->rng == 0) {
        g->rng = g->cfg.seed ? g->cfg.seed : 1u;
    }
    b = nbody_tree_gpu_blocks(n, NBODY_TREE_GPU_BLOCK);
    wb = nbody_tree_gpu_blocks(n, NBODY_TREE_GPU_WALK);
    if (g->cfg.randomize && g->cfg.t_max <= 0.0f) {
        err = cudaMemsetAsync(g->rmax, 0, sizeof(float), g->stream);
        if (err != cudaSuccess) {
            return err;
        }
        nbody_tree_gpu_rmax_k<<<b, NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
            g->p, n, g->rmax);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
        err = cudaMemcpyAsync(&hrmax, g->rmax, sizeof(float),
            cudaMemcpyDeviceToHost, g->stream);
        if (err != cudaSuccess) {
            return err;
        }
        err = cudaStreamSynchronize(g->stream);
        if (err != cudaSuccess) {
            return err;
        }
        hrmax = sqrtf(hrmax);
    }
    nbody_tree_fill_cs(g->cfg, &g->rng, hrmax, g->R, &g->S, &g->Tx, &g->Ty,
        &g->Tz);
    cs = nbody_tree_gpu_cs(g);
    err = cudaMemsetAsync(g->extent, 0, sizeof(float), g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    nbody_tree_gpu_xform_k<<<b, NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
        g->p, g->tx, g->ty, g->tz, n, cs, g->extent);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    nbody_tree_gpu_half_k<<<1, 1, 0, g->stream>>>(g->extent, g->root_half);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    if (n < 2) {
        nbody_tree_gpu_walk_k<<<wb, NBODY_TREE_GPU_WALK, 0, g->stream>>>(
            g->nodes, g->p, g->a, g->ids[0], g->rank, g->leaf_next, n, 0,
            cfg, cs);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
        return cudaStreamSynchronize(g->stream);
    }
    nbody_tree_gpu_morton_k<<<b, NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
        g->tx, g->ty, g->tz, g->keys[0], g->ids[0], n, g->root_half);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    err = nbody_tree_gpu_sort(g);
    if (err != cudaSuccess) {
        return err;
    }
    err = cudaMemcpyAsync(g->order, g->ids[0], (size_t)n * sizeof(int),
        cudaMemcpyDeviceToDevice, g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    nbody_tree_gpu_rank_k<<<b, NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
        g->order, g->rank, n);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    nbody_tree_gpu_clear_nodes_k<<<nbody_tree_gpu_blocks(n_int,
            NBODY_TREE_GPU_BLOCK), NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
        g->nodes, n_int, g->root);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    nbody_tree_gpu_karras_k<<<nbody_tree_gpu_blocks(n_int,
            NBODY_TREE_GPU_BLOCK), NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
        g->keys[0], g->nodes, n);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    nbody_tree_gpu_parents_k<<<nbody_tree_gpu_blocks(n_int,
            NBODY_TREE_GPU_BLOCK), NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
        g->nodes, g->leaf_parent, g->root, n);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    nbody_tree_gpu_clear_ready_k<<<nbody_tree_gpu_blocks(n_int,
            NBODY_TREE_GPU_BLOCK), NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
        g->ready, n_int);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    err = cudaMemcpyAsync(&hhalf, g->root_half, sizeof(float),
        cudaMemcpyDeviceToHost, g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    err = cudaMemcpyAsync(&hroot, g->root, sizeof(int),
        cudaMemcpyDeviceToHost, g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    err = cudaStreamSynchronize(g->stream);
    if (err != cudaSuccess) {
        return err;
    }
    if (hroot < 0 || hroot >= n_int) {
        return cudaErrorUnknown;
    }
    nbody_tree_gpu_moments_k<<<b, NBODY_TREE_GPU_BLOCK, 0, g->stream>>>(
        g->nodes, g->p, g->order, g->leaf_parent, g->ready, g->keys[0],
        n, hhalf, cs);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    nbody_tree_gpu_links_k<<<1, 1, 0, g->stream>>>(g->nodes, g->leaf_next,
        hroot, n, g->stk_node, g->stk_succ, g->stk_st);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    nbody_tree_gpu_walk_k<<<wb, NBODY_TREE_GPU_WALK, 0, g->stream>>>(
        g->nodes, g->p, g->a, g->order, g->rank, g->leaf_next, n, hroot,
        cfg, cs);
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        return err;
    }
    return cudaStreamSynchronize(g->stream);
}

static cudaError_t nbody_tree_gpu_step(NbodyTreeGpu *g, float dt, int steps,
        NbodyConfig cfg) {
    unsigned blocks;
    int step;
    cudaError_t err;
    if (!g || steps < 0 || !nbody_cfg_ok(cfg)
            || !nbody_tree_cfg_ok(g->cfg)) {
        return cudaErrorInvalidValue;
    }
    if (g->n == 0 || steps == 0 || dt == 0) {
        return cudaSuccess;
    }
    err = nbody_tree_gpu_acceleration(g, cfg);
    if (err != cudaSuccess) {
        return err;
    }
    blocks = ((unsigned)g->n + NBODY_TILE - 1) / NBODY_TILE;
    for (step = 0; step < steps; step++) {
        nbody_kick_kernel<<<blocks, NBODY_TILE, 0, g->stream>>>(
            g->p, g->v, g->a, g->n, dt, 1);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            return err;
        }
        err = nbody_tree_gpu_acceleration(g, cfg);
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
    return cudaStreamSynchronize(g->stream);
}

#endif
