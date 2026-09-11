#ifndef DAT_CLOTH_H
#define DAT_CLOTH_H
/* Planar DAT cloth-sphere coupling after Chen, Hsu, Ayman and Macklin,
 * "Divide and Truncate: A Penetration and Inversion Free Framework for
 * Coupled Multi-physics Systems" (arXiv:2604.15513). Exclusive (ball,
 * node) pairs split positions by inverse mass; a later impulse removes
 * closing normal speed at restitution e. Tangential velocity is free.
 */
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "puffysics.cuh"

#ifndef DAT_GAMMA
#define DAT_GAMMA 0.8f
#endif

typedef struct DatCloth {
    int W;
    int H;
    int n;
    float spacing;
    float node_mass;
    float gravity;
    float damping;
    float relax;
    int iters;
    float restitution;
    float gamma;
    float compliance;
    B3Vec3 origin;
    B3Vec3* pos;
    B3Vec3* vel;
    B3Vec3* snap;
    uint8_t* pinned;
} DatCloth;

static void dat_cloth_free(DatCloth* c);

static int dat_cloth_init(DatCloth* c, int W, int H, float spacing,
        B3Vec3 origin) {
    memset(c, 0, sizeof(*c));
    if (W <= 0 || H <= 0 || spacing <= 0.0f || !isfinite(spacing)) {
        return 0;
    }
    if ((size_t)W > (size_t)INT_MAX / (size_t)H) {
        return 0;
    }
    c->W = W;
    c->H = H;
    c->n = W * H;
    c->spacing = spacing;
    c->node_mass = 0.2f;
    c->gravity = -9.81f;
    c->damping = 0.03f;
    c->relax = 0.6f;
    c->iters = 10;
    c->restitution = 0.0f;
    c->gamma = DAT_GAMMA;
    c->origin = origin;
    c->pos = (B3Vec3*)malloc((size_t)c->n * sizeof(B3Vec3));
    c->vel = (B3Vec3*)malloc((size_t)c->n * sizeof(B3Vec3));
    c->snap = (B3Vec3*)malloc((size_t)c->n * sizeof(B3Vec3));
    c->pinned = (uint8_t*)malloc((size_t)c->n);
    if (!c->pos || !c->vel || !c->snap || !c->pinned) {
        dat_cloth_free(c);
        memset(c, 0, sizeof(*c));
        return 0;
    }
    for (int j = 0; j < H; j++) {
        for (int i = 0; i < W; i++) {
            int k = j * W + i;
            c->pos[k] = b3_v(c->origin.x + spacing * (float)i,
                c->origin.y, c->origin.z + spacing * (float)j);
            c->vel[k] = b3_v(0.0f, 0.0f, 0.0f);
            c->snap[k] = c->pos[k];
            c->pinned[k] = (uint8_t)(i == 0 || i == W - 1 || j == 0
                || j == H - 1);
        }
    }
    return 1;
}

static void dat_cloth_free(DatCloth* c) {
    free(c->pos);
    free(c->vel);
    free(c->snap);
    free(c->pinned);
    memset(c, 0, sizeof(*c));
}

static void dat_cloth_step(DatCloth* c, float h) {
    if (h <= 0.0f) {
        return;
    }
    float damp = 1.0f - c->damping;
    for (int k = 0; k < c->n; k++) {
        c->snap[k] = c->pos[k];
        if (c->pinned[k]) {
            c->vel[k] = b3_v(0.0f, 0.0f, 0.0f);
            continue;
        }
        c->vel[k] = b3_mul(c->vel[k], damp);
        c->vel[k] = b3_madd(c->vel[k], h, b3_v(0.0f, c->gravity, 0.0f));
        c->pos[k] = b3_madd(c->pos[k], h, c->vel[k]);
    }
    for (int it = 0; it < c->iters; it++) {
        for (int j = 0; j < c->H; j++) {
            for (int i = 0; i < c->W; i++) {
                int k = j * c->W + i;
                for (int e = 0; e < 4; e++) {
                    int dx = (e == 0 || e == 2 || e == 3) ? 1 : 0;
                    int dy = (e == 1 || e == 2) ? 1 : (e == 3 ? -1 : 0);
                    int i2 = i + dx;
                    int j2 = j + dy;
                    if (i2 >= c->W || j2 >= c->H || j2 < 0) {
                        continue;
                    }
                    int k2 = j2 * c->W + i2;
                    float rest = c->spacing
                        * ((e == 2 || e == 3) ? 1.41421356f : 1.0f);
                    B3Vec3 d = b3_sub(c->pos[k2], c->pos[k]);
                    float raw = b3_len(d);
                    if (raw <= 1.0e-9f) {
                        continue;
                    }
                    float diff = raw - rest;
                    if (diff > 0.2f * rest) {
                        diff = 0.2f * rest;
                    }
                    if (diff < -0.5f * rest) {
                        diff = -0.5f * rest;
                    }
                    float wi = c->pinned[k] ? 0.0f : 1.0f / c->node_mass;
                    float wj = c->pinned[k2] ? 0.0f : 1.0f / c->node_mass;
                    float ws = wi + wj;
                    if (ws <= 0.0f) {
                        continue;
                    }
                    float s = diff / raw * c->relax / ws;
                    c->pos[k] = b3_madd(c->pos[k], s * wi, d);
                    c->pos[k2] = b3_msub(c->pos[k2], s * wj, d);
                }
            }
        }
    }
    for (int k = 0; k < c->n; k++) {
        if (c->pinned[k]) {
            c->vel[k] = b3_v(0.0f, 0.0f, 0.0f);
            continue;
        }
        c->vel[k] = b3_mul(b3_sub(c->pos[k], c->snap[k]), 1.0f / h);
    }
}

static int dat_world_balls(const B3World* w, int* bodies, float* radii,
        int max) {
    int count = 0;
    for (int bi = 0; bi < w->body_count && count < max; bi++) {
        const B3Body* b = &w->bodies[bi];
        if (b->type != B3_DYNAMIC) {
            continue;
        }
        int mine = 0;
        int spheres = 0;
        int only = -1;
        for (int si = 0; si < w->shape_count; si++) {
            if (w->shapes[si].body != bi) {
                continue;
            }
            mine++;
            if (w->shapes[si].type == B3_SPHERE) {
                spheres++;
                only = si;
            }
        }
        if (mine == 1 && spheres == 1) {
            bodies[count] = bi;
            radii[count] = w->shapes[only].radius;
            count++;
        }
    }
    return count;
}

static void dat_couple(DatCloth* c, B3World* w, const B3Vec3* ball_prev,
        const int* ball_body, const float* ball_r, int nb, float h) {
    for (int sweep = 0; sweep < 2; sweep++) {
        for (int k = 0; k < c->n; k++) {
            for (int q = 0; q < nb; q++) {
                B3Body* b = &w->bodies[ball_body[q]];
                float r = ball_r[q];
                B3Vec3 d = b3_sub(c->pos[k], b->center);
                float dist = b3_len(d);
                float slack = 0.1f * c->spacing;
                if (dist >= r + slack || dist <= 1.0e-9f) {
                    continue;
                }
                B3Vec3 u = b3_mul(d, 1.0f / dist);
                float wi = c->pinned[k] ? 0.0f : 1.0f / c->node_mass;
                float wb = b->inv_mass;
                float ws = wi + wb;
                if (ws <= 0.0f) {
                    continue;
                }
                float push = r + c->gamma * slack - dist;
                if (push > 0.0f) {
                    c->pos[k] = b3_madd(c->pos[k], push * wi / ws, u);
                    b->center = b3_msub(b->center, push * wb / ws, u);
                    b->position = b3_sub(b->center,
                        b3_rotate(b->rotation, b->local_center));
                }
            }
        }
    }
}

static void dat_couple_velocities(B3World* w, const B3Vec3* ball_before,
        const int* ball_body, int nb, float h) {
    if (h <= 0.0f) {
        return;
    }
    for (int q = 0; q < nb; q++) {
        B3Body* b = &w->bodies[ball_body[q]];
        b->lin_vel = b3_add(b->lin_vel,
            b3_mul(b3_sub(b->center, ball_before[q]), 1.0f / h));
    }
}
#endif
