#ifndef NBODY_TREE_CUH
#define NBODY_TREE_CUH

/* Barnes-Hut oct-tree with Treecode2 pluralism (Barnes, MNRAS 2026,
 * arXiv:2602.06295). Include this file; it pulls nbody.cuh.
 *
 * Each force evaluation may rebuild the tree in a random coordinate
 * system r' = R S (r - T). Averaging over those frames is the paper's
 * "pluralism": one new CS per acceleration, not N_avg trees per step.
 * Cell moments and forces stay in simulation coordinates.
 *
 * Paper I force algorithm: quadrupole, Nitadori softening, Barnes 1994
 * opening, Makino Next/More walk. Not the static-test harness, not
 * Appendix A interaction lists.
 * CPU: pointer oct-tree. CUDA: Morton/Karras BRT of the same cells.
 *
 *   NbodyTreeConfig, nbody_tree_default
 *   nbody_tree_init / nbody_tree_free
 *   nbody_tree_acceleration, nbody_tree_step
 * CUDA: NbodyTreeGpu; nbody_tree_gpu_init/free/upload/download/step
 */
#include "nbody.cuh"
#include <stdint.h>
#include <stdlib.h>
#include <limits.h>

#ifndef NBODY_TREE_MAX_DEPTH
#define NBODY_TREE_MAX_DEPTH 24
#endif

#define NBODY_TREE_CELL 0
#define NBODY_TREE_BODY 1

typedef struct NbodyTreeConfig {
    float theta;
    float s_max;
    float t_max;
    unsigned seed;
    int quadrupole;
    int soft_quad;
    int cm_open;
    int randomize;
} NbodyTreeConfig;

typedef struct NbodyTreeNode {
    int kind;
    int more, next, body;
    int child[8];
    float tx, ty, tz, half;
    float mass, cx, cy, cz;
    float gx, gy, gz, hsim;
    float qxx, qyy, qzz, qxy, qxz, qyz, qtilde;
} NbodyTreeNode;

typedef struct NbodyTree {
    NbodyTreeNode *nodes;
    int node_cap, n_nodes, root, n, missed;
    unsigned rng;
    NbodyTreeConfig cfg;
    float R[9], S, Tx, Ty, Tz;
} NbodyTree;

static NbodyTreeConfig nbody_tree_default(void) {
    NbodyTreeConfig c;
    c.theta = 0.8f;
    c.s_max = 1.41421356f;
    c.t_max = 0.0f;
    c.seed = 1u;
    c.quadrupole = 1;
    c.soft_quad = 1;
    c.cm_open = 1;
    c.randomize = 1;
    return c;
}

static int nbody_tree_cfg_ok(NbodyTreeConfig c) {
    return c.theta > 0.0f && c.theta <= 2.0f && c.s_max >= 1.0f
        && c.t_max >= 0.0f && (c.quadrupole == 0 || c.quadrupole == 1)
        && (c.soft_quad == 0 || c.soft_quad == 1)
        && (c.cm_open == 0 || c.cm_open == 1)
        && (c.randomize == 0 || c.randomize == 1);
}

static void nbody_tree_free(NbodyTree *t) {
    if (!t) {
        return;
    }
    free(t->nodes);
    memset(t, 0, sizeof(*t));
}

static int nbody_tree_init(NbodyTree *t, int n_hint) {
    size_t cap;
    if (!t || n_hint < 0) {
        return 0;
    }
    memset(t, 0, sizeof(*t));
    t->cfg = nbody_tree_default();
    t->rng = t->cfg.seed;
    t->S = 1.0f;
    t->R[0] = t->R[4] = t->R[8] = 1.0f;
    t->root = -1;
    cap = 16;
    if (n_hint > 0) {
        if ((size_t)n_hint > (SIZE_MAX / sizeof(NbodyTreeNode) - 16) / 2) {
            return 0;
        }
        cap = (size_t)n_hint * 2 + 16;
    }
    if (cap > (size_t)INT_MAX) {
        return 0;
    }
    t->nodes = (NbodyTreeNode *)calloc(cap, sizeof(NbodyTreeNode));
    if (!t->nodes) {
        return 0;
    }
    t->node_cap = (int)cap;
    return 1;
}

static unsigned nbody_tree_u32(unsigned *s) {
    unsigned x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

static float nbody_tree_u01(unsigned *s) {
    return (nbody_tree_u32(s) >> 8) * (1.0f / 16777216.0f);
}

static NBODY_HD void nbody_tree_mul(const float R[9], float x, float y, float z,
        float *ox, float *oy, float *oz) {
    *ox = R[0] * x + R[1] * y + R[2] * z;
    *oy = R[3] * x + R[4] * y + R[5] * z;
    *oz = R[6] * x + R[7] * y + R[8] * z;
}

static NBODY_HD void nbody_tree_mulT(const float R[9], float x, float y, float z,
        float *ox, float *oy, float *oz) {
    *ox = R[0] * x + R[3] * y + R[6] * z;
    *oy = R[1] * x + R[4] * y + R[7] * z;
    *oz = R[2] * x + R[5] * y + R[8] * z;
}

static void nbody_tree_to_tree(const NbodyTree *t, float x, float y, float z,
        float *ox, float *oy, float *oz) {
    float sx = t->S * (x - t->Tx);
    float sy = t->S * (y - t->Ty);
    float sz = t->S * (z - t->Tz);
    nbody_tree_mul(t->R, sx, sy, sz, ox, oy, oz);
}

static void nbody_tree_from_tree(const NbodyTree *t, float x, float y, float z,
        float *ox, float *oy, float *oz) {
    float ux, uy, uz, inv = 1.0f / t->S;
    nbody_tree_mulT(t->R, x, y, z, &ux, &uy, &uz);
    *ox = t->Tx + inv * ux;
    *oy = t->Ty + inv * uy;
    *oz = t->Tz + inv * uz;
}

static void nbody_tree_rand_R(unsigned *rng, float R[9]) {
    float u1 = nbody_tree_u01(rng);
    float u2 = nbody_tree_u01(rng);
    float u3 = nbody_tree_u01(rng);
    float a1 = 6.28318530718f * u2;
    float a2 = 6.28318530718f * u3;
    float s1 = sqrtf(1.0f - u1);
    float s2 = sqrtf(u1);
    float x = s1 * sinf(a1);
    float y = s1 * cosf(a1);
    float z = s2 * sinf(a2);
    float w = s2 * cosf(a2);
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;
    R[0] = 1.0f - 2.0f * (yy + zz);
    R[1] = 2.0f * (xy - wz);
    R[2] = 2.0f * (xz + wy);
    R[3] = 2.0f * (xy + wz);
    R[4] = 1.0f - 2.0f * (xx + zz);
    R[5] = 2.0f * (yz - wx);
    R[6] = 2.0f * (xz - wy);
    R[7] = 2.0f * (yz + wx);
    R[8] = 1.0f - 2.0f * (xx + yy);
}

static void nbody_tree_identity_cs(NbodyTree *t) {
    memset(t->R, 0, sizeof(t->R));
    t->R[0] = t->R[4] = t->R[8] = 1.0f;
    t->S = 1.0f;
    t->Tx = t->Ty = t->Tz = 0.0f;
}

static void nbody_tree_fill_cs(NbodyTreeConfig cfg, unsigned *rng,
        float rmax, float R[9], float *S, float *Tx, float *Ty, float *Tz) {
    float tmax, u, a, b, c, nrm;
    if (!cfg.randomize) {
        memset(R, 0, 9 * sizeof(float));
        R[0] = R[4] = R[8] = 1.0f;
        *S = 1.0f;
        *Tx = *Ty = *Tz = 0.0f;
        return;
    }
    nbody_tree_rand_R(rng, R);
    if (cfg.s_max <= 1.0f) {
        *S = 1.0f;
    } else {
        u = nbody_tree_u01(rng) * 2.0f - 1.0f;
        *S = powf(cfg.s_max, u);
        if (!(*S > 0.0f) || *S < 1.0e-6f || *S > 1.0e6f) {
            *S = 1.0f;
        }
    }
    tmax = cfg.t_max;
    if (tmax <= 0.0f && rmax > 0.0f) {
        tmax = 0.5f * rmax;
        if (tmax < 1.0f) {
            tmax = 1.0f;
        }
    }
    if (tmax <= 0.0f) {
        *Tx = *Ty = *Tz = 0.0f;
        return;
    }
    do {
        a = nbody_tree_u01(rng) * 2.0f - 1.0f;
        b = nbody_tree_u01(rng) * 2.0f - 1.0f;
        c = nbody_tree_u01(rng) * 2.0f - 1.0f;
        nrm = a * a + b * b + c * c;
    } while (nrm > 1.0f || nrm < 1.0e-8f);
    nrm = 1.0f / sqrtf(nrm);
    u = cbrtf(nbody_tree_u01(rng)) * tmax;
    *Tx = a * nrm * u;
    *Ty = b * nrm * u;
    *Tz = c * nrm * u;
}

static void nbody_tree_sample_cs(NbodyTree *t, const NbodyPoint *p, int n) {
    float rmax = 0.0f, r;
    int i;
    if (!t->cfg.randomize) {
        nbody_tree_identity_cs(t);
        return;
    }
    if (t->cfg.t_max <= 0.0f && p && n > 0) {
        for (i = 0; i < n; i++) {
            r = p[i].x * p[i].x + p[i].y * p[i].y + p[i].z * p[i].z;
            if (r > rmax) {
                rmax = r;
            }
        }
        rmax = sqrtf(rmax);
    }
    nbody_tree_fill_cs(t->cfg, &t->rng, rmax, t->R, &t->S, &t->Tx, &t->Ty,
        &t->Tz);
}

static int nbody_tree_new_node(NbodyTree *t, int kind) {
    NbodyTreeNode *nd;
    int id, k;
    size_t cap;
    if (t->n_nodes >= t->node_cap) {
        cap = t->node_cap > 0 ? (size_t)t->node_cap * 2 : 64;
        if (cap > (size_t)INT_MAX
                || cap > SIZE_MAX / sizeof(NbodyTreeNode)) {
            return -1;
        }
        nd = (NbodyTreeNode *)realloc(t->nodes, cap * sizeof(*nd));
        if (!nd) {
            return -1;
        }
        t->nodes = nd;
        t->node_cap = (int)cap;
    }
    id = t->n_nodes++;
    nd = &t->nodes[id];
    memset(nd, 0, sizeof(*nd));
    nd->kind = kind;
    nd->more = nd->next = nd->body = -1;
    for (k = 0; k < 8; k++) {
        nd->child[k] = -1;
    }
    return id;
}

static int nbody_tree_octant(const NbodyTreeNode *c, float x, float y,
        float z) {
    int o = 0;
    if (x >= c->tx) {
        o |= 1;
    }
    if (y >= c->ty) {
        o |= 2;
    }
    if (z >= c->tz) {
        o |= 4;
    }
    return o;
}

static int nbody_tree_ensure_child(NbodyTree *t, int cell, int oct) {
    int id;
    float h;
    if (t->nodes[cell].child[oct] >= 0) {
        return t->nodes[cell].child[oct];
    }
    h = 0.5f * t->nodes[cell].half;
    id = nbody_tree_new_node(t, NBODY_TREE_CELL);
    if (id < 0) {
        return -1;
    }
    t->nodes[cell].child[oct] = id;
    t->nodes[id].half = h;
    t->nodes[id].tx = t->nodes[cell].tx + ((oct & 1) ? h : -h);
    t->nodes[id].ty = t->nodes[cell].ty + ((oct & 2) ? h : -h);
    t->nodes[id].tz = t->nodes[cell].tz + ((oct & 4) ? h : -h);
    return id;
}

static int nbody_tree_same_pos(const NbodyTreeNode *a,
        const NbodyTreeNode *b) {
    return a->tx == b->tx && a->ty == b->ty && a->tz == b->tz;
}

static int nbody_tree_insert(NbodyTree *t, int cell, int bnode, int depth);

static int nbody_tree_insert_child(NbodyTree *t, int cell, int bnode,
        int depth) {
    int oct, ch;
    oct = nbody_tree_octant(&t->nodes[cell], t->nodes[bnode].tx,
        t->nodes[bnode].ty, t->nodes[bnode].tz);
    ch = nbody_tree_ensure_child(t, cell, oct);
    if (ch < 0) {
        return 0;
    }
    return nbody_tree_insert(t, ch, bnode, depth + 1);
}

static int nbody_tree_insert(NbodyTree *t, int cell, int bnode, int depth) {
    int old, nxt, k, internal;
    internal = 0;
    for (k = 0; k < 8; k++) {
        if (t->nodes[cell].child[k] >= 0) {
            internal = 1;
            break;
        }
    }
    if (!internal) {
        if (t->nodes[cell].body < 0) {
            t->nodes[cell].body = bnode;
            return 1;
        }
        if (depth >= NBODY_TREE_MAX_DEPTH
                || nbody_tree_same_pos(&t->nodes[t->nodes[cell].body],
                    &t->nodes[bnode])) {
            t->nodes[bnode].next = t->nodes[cell].body;
            t->nodes[cell].body = bnode;
            return 1;
        }
        old = t->nodes[cell].body;
        t->nodes[cell].body = -1;
        while (old >= 0) {
            nxt = t->nodes[old].next;
            t->nodes[old].next = -1;
            if (!nbody_tree_insert_child(t, cell, old, depth)) {
                return 0;
            }
            old = nxt;
        }
    }
    return nbody_tree_insert_child(t, cell, bnode, depth);
}

static void nbody_tree_add_q(NbodyTreeNode *c, float m, float dx, float dy,
        float dz) {
    float r2 = dx * dx + dy * dy + dz * dz;
    c->qxx += m * (3.0f * dx * dx - r2);
    c->qyy += m * (3.0f * dy * dy - r2);
    c->qzz += m * (3.0f * dz * dz - r2);
    c->qxy += m * (3.0f * dx * dy);
    c->qxz += m * (3.0f * dx * dz);
    c->qyz += m * (3.0f * dy * dz);
    c->qtilde += m * r2;
}

static void nbody_tree_moments(NbodyTree *t, int cell, const NbodyPoint *p) {
    int i, b, kid;
    float m, inv;
    t->nodes[cell].mass = 0;
    t->nodes[cell].cx = t->nodes[cell].cy = t->nodes[cell].cz = 0;
    t->nodes[cell].qxx = t->nodes[cell].qyy = t->nodes[cell].qzz = 0;
    t->nodes[cell].qxy = t->nodes[cell].qxz = t->nodes[cell].qyz = 0;
    t->nodes[cell].qtilde = 0;
    nbody_tree_from_tree(t, t->nodes[cell].tx, t->nodes[cell].ty,
        t->nodes[cell].tz, &t->nodes[cell].gx, &t->nodes[cell].gy,
        &t->nodes[cell].gz);
    t->nodes[cell].hsim = t->nodes[cell].half / t->S;
    for (i = 0; i < 8; i++) {
        kid = t->nodes[cell].child[i];
        if (kid < 0) {
            continue;
        }
        nbody_tree_moments(t, kid, p);
        m = t->nodes[kid].mass;
        t->nodes[cell].mass += m;
        t->nodes[cell].cx += m * t->nodes[kid].cx;
        t->nodes[cell].cy += m * t->nodes[kid].cy;
        t->nodes[cell].cz += m * t->nodes[kid].cz;
    }
    b = t->nodes[cell].body;
    while (b >= 0) {
        i = t->nodes[b].body;
        m = p[i].mass;
        if (m != 0.0f) {
            t->nodes[cell].mass += m;
            t->nodes[cell].cx += m * p[i].x;
            t->nodes[cell].cy += m * p[i].y;
            t->nodes[cell].cz += m * p[i].z;
        }
        b = t->nodes[b].next;
    }
    if (t->nodes[cell].mass > 0.0f) {
        inv = 1.0f / t->nodes[cell].mass;
        t->nodes[cell].cx *= inv;
        t->nodes[cell].cy *= inv;
        t->nodes[cell].cz *= inv;
    } else {
        t->nodes[cell].cx = t->nodes[cell].gx;
        t->nodes[cell].cy = t->nodes[cell].gy;
        t->nodes[cell].cz = t->nodes[cell].gz;
    }
    if (!t->cfg.quadrupole) {
        return;
    }
    for (i = 0; i < 8; i++) {
        kid = t->nodes[cell].child[i];
        if (kid < 0) {
            continue;
        }
        m = t->nodes[kid].mass;
        t->nodes[cell].qxx += t->nodes[kid].qxx;
        t->nodes[cell].qyy += t->nodes[kid].qyy;
        t->nodes[cell].qzz += t->nodes[kid].qzz;
        t->nodes[cell].qxy += t->nodes[kid].qxy;
        t->nodes[cell].qxz += t->nodes[kid].qxz;
        t->nodes[cell].qyz += t->nodes[kid].qyz;
        t->nodes[cell].qtilde += t->nodes[kid].qtilde;
        if (m != 0.0f) {
            nbody_tree_add_q(&t->nodes[cell], m,
                t->nodes[kid].cx - t->nodes[cell].cx,
                t->nodes[kid].cy - t->nodes[cell].cy,
                t->nodes[kid].cz - t->nodes[cell].cz);
        }
    }
    b = t->nodes[cell].body;
    while (b >= 0) {
        i = t->nodes[b].body;
        m = p[i].mass;
        if (m != 0.0f) {
            nbody_tree_add_q(&t->nodes[cell], m, p[i].x - t->nodes[cell].cx,
                p[i].y - t->nodes[cell].cy, p[i].z - t->nodes[cell].cz);
        }
        b = t->nodes[b].next;
    }
}

static void nbody_tree_thread(NbodyTree *t, int cell) {
    int oct, ch, succ, b, nxt;
    succ = t->nodes[cell].next;
    t->nodes[cell].more = -1;
    for (oct = 7; oct >= 0; oct--) {
        ch = t->nodes[cell].child[oct];
        if (ch < 0) {
            continue;
        }
        t->nodes[ch].next = succ;
        nbody_tree_thread(t, ch);
        succ = ch;
        t->nodes[cell].more = ch;
    }
    if (t->nodes[cell].body >= 0) {
        if (t->nodes[cell].more < 0) {
            t->nodes[cell].more = t->nodes[cell].body;
        }
        b = t->nodes[cell].body;
        while (b >= 0) {
            nxt = t->nodes[b].next;
            t->nodes[b].next = nxt >= 0 ? nxt : succ;
            b = nxt;
        }
    }
}

static int nbody_tree_build(NbodyTree *t, const NbodyPoint *p, int n) {
    int i, b, root;
    float x, y, z, ax, m;
    if (!t || n < 0 || (n > 0 && !p)) {
        return 0;
    }
    t->n_nodes = 0;
    t->n = n;
    t->missed = 0;
    t->root = -1;
    if (n == 0) {
        return 1;
    }
    root = nbody_tree_new_node(t, NBODY_TREE_CELL);
    if (root < 0) {
        return 0;
    }
    t->root = root;
    m = 0.0f;
    for (i = 0; i < n; i++) {
        nbody_tree_to_tree(t, p[i].x, p[i].y, p[i].z, &x, &y, &z);
        ax = fabsf(x);
        if (fabsf(y) > ax) {
            ax = fabsf(y);
        }
        if (fabsf(z) > ax) {
            ax = fabsf(z);
        }
        if (ax > m) {
            m = ax;
        }
    }
    m *= 1.000001f;
    t->nodes[root].half = 1.0f;
    while (t->nodes[root].half < m && t->nodes[root].half < 1.0e20f) {
        t->nodes[root].half *= 2.0f;
    }
    if (t->nodes[root].half < m) {
        return 0;
    }
    for (i = 0; i < n; i++) {
        b = nbody_tree_new_node(t, NBODY_TREE_BODY);
        if (b < 0) {
            return 0;
        }
        t->nodes[b].body = i;
        t->nodes[b].mass = p[i].mass;
        nbody_tree_to_tree(t, p[i].x, p[i].y, p[i].z,
            &t->nodes[b].tx, &t->nodes[b].ty, &t->nodes[b].tz);
        if (!nbody_tree_insert(t, root, b, 0)) {
            return 0;
        }
    }
    nbody_tree_moments(t, root, p);
    t->nodes[root].next = -1;
    nbody_tree_thread(t, root);
    return 1;
}

static int nbody_tree_in_cell(const NbodyTreeNode *c, float x, float y,
        float z) {
    return fabsf(x - c->tx) <= c->half && fabsf(y - c->ty) <= c->half
        && fabsf(z - c->tz) <= c->half;
}

static int nbody_tree_must_open(const NbodyTree *t, const NbodyTreeNode *c,
        float px, float py, float pz, float tx, float ty, float tz) {
    float sx, sy, sz, s2, dx, dy, dz, d, ell, lim;
    if (nbody_tree_in_cell(c, tx, ty, tz)) {
        return 1;
    }
    sx = px - c->cx;
    sy = py - c->cy;
    sz = pz - c->cz;
    s2 = sx * sx + sy * sy + sz * sz;
    dx = c->cx - c->gx;
    dy = c->cy - c->gy;
    dz = c->cz - c->gz;
    d = t->cfg.cm_open ? sqrtf(dx * dx + dy * dy + dz * dz) : 0.0f;
    ell = 2.0f * c->hsim;
    lim = ell / t->cfg.theta + d;
    return s2 < lim * lim;
}

static void nbody_tree_add_cell(const NbodyTreeNode *c, NbodyPoint dest,
        float G, float eps2, int use_q, int use_soft,
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
    inv = 1.0f / sqrtf(re2);
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

static void nbody_tree_walk_one(NbodyTree *t, const NbodyPoint *p,
        int dest, NbodyConfig g, float *ax, float *ay, float *az) {
    const NbodyTreeNode *nd;
    NbodyPoint src;
    int n, seen, guard;
    float eps2, tx, ty, tz;
    *ax = *ay = *az = 0.0f;
    if (t->root < 0) {
        return;
    }
    eps2 = g.softening * g.softening;
    nbody_tree_to_tree(t, p[dest].x, p[dest].y, p[dest].z, &tx, &ty, &tz);
    n = t->root;
    seen = 0;
    guard = 0;
    while (n >= 0 && guard < t->n_nodes + 2) {
        guard++;
        nd = &t->nodes[n];
        if (nd->kind == NBODY_TREE_CELL) {
            if (nbody_tree_must_open(t, nd, p[dest].x, p[dest].y, p[dest].z,
                    tx, ty, tz)) {
                n = nd->more;
            } else {
                nbody_tree_add_cell(nd, p[dest], g.gravity, eps2,
                    t->cfg.quadrupole, t->cfg.soft_quad, ax, ay, az);
                n = nd->next;
            }
        } else {
            if (nd->body == dest) {
                seen = 1;
            } else if (p[nd->body].mass != 0.0f) {
                src = p[nd->body];
                nbody_add_source(p[dest], src, g.gravity, eps2, ax, ay, az);
            }
            n = nd->next;
        }
    }
    if (!seen) {
        t->missed++;
    }
}

static int nbody_tree_acceleration(NbodyTree *t, const NbodyPoint *p,
        NbodyVec *a, int n, NbodyConfig g) {
    int i;
    float ax, ay, az;
    if (!t || n < 0 || !nbody_cfg_ok(g) || !nbody_tree_cfg_ok(t->cfg)
            || (n > 0 && !(p && a))) {
        return 0;
    }
    if (t->rng == 0) {
        t->rng = t->cfg.seed ? t->cfg.seed : 1u;
    }
    if (n == 0) {
        t->n = 0;
        t->root = -1;
        return 1;
    }
    nbody_tree_sample_cs(t, p, n);
    if (!nbody_tree_build(t, p, n)) {
        return 0;
    }
    for (i = 0; i < n; i++) {
        nbody_tree_walk_one(t, p, i, g, &ax, &ay, &az);
        a[i].x = ax;
        a[i].y = ay;
        a[i].z = az;
    }
    return 1;
}

static int nbody_tree_step(NbodyTree *t, NbodyPoint *p, NbodyVec *v,
        NbodyVec *scratch, int n, float dt, int steps, NbodyConfig g) {
    int step, i;
    float half;
    if (!t || n < 0 || steps < 0 || !nbody_cfg_ok(g)
            || !nbody_tree_cfg_ok(t->cfg)
            || (n > 0 && !(p && v && scratch))) {
        return 0;
    }
    if (n == 0 || steps == 0 || dt == 0) {
        return 1;
    }
    if (!nbody_tree_acceleration(t, p, scratch, n, g)) {
        return 0;
    }
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
        if (!nbody_tree_acceleration(t, p, scratch, n, g)) {
            return 0;
        }
        for (i = 0; i < n; i++) {
            v[i].x += half * scratch[i].x;
            v[i].y += half * scratch[i].y;
            v[i].z += half * scratch[i].z;
        }
    }
    return 1;
}

#ifdef __CUDACC__
#include "nbody_tree_gpu.cuh"
#endif
#endif
