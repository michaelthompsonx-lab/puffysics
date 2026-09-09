// Runtime-sized free-body island: integrate, collide, colored contact GS.
// Train worlds stay on B3_MAX_* in B3World; hinges stay there too.
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "puffysics.cuh"

#ifndef MJCF_COL_GROUND
#define MJCF_COL_GROUND 1ull
#endif
#ifndef MJCF_COL_FEET
#define MJCF_COL_FEET 4ull
#endif
#ifndef MJCF_COL_CUBE
#define MJCF_COL_CUBE 8ull
#endif
#ifndef MJCF_COL_BODY
#define MJCF_COL_BODY 16ull
#endif

#ifndef B3_LOOSE_ITERS
#define B3_LOOSE_ITERS 8
#endif
#ifndef B3_LOOSE_COLORS
#define B3_LOOSE_COLORS 16
#endif
#ifndef B3_LOOSE_XCAP
#define B3_LOOSE_XCAP 48
#endif
/* Loose pairs skip B3_SPECULATIVE (20 mm): it ghosts contacts on 3 cm cubes. */
#ifndef B3_LOOSE_SPECULATIVE
#define B3_LOOSE_SPECULATIVE 0.002f
#endif
#ifndef B3_LOOSE_NB
#define B3_LOOSE_NB 6
#endif

typedef struct B3Loose {
    B3Vec3 gravity;
    float contact_hertz;
    float contact_damping;
    float contact_speed;
    float max_linear_speed;
    int n_bodies;
    int n_shapes;
    int n_contacts;
    int cap_bodies;
    int cap_shapes;
    int cap_contacts;
    int overflow;
    B3Body* bodies;
    B3Shape* shapes;
    B3Contact* contacts;
    int* nb;
    int nb_hops;
} B3Loose;

static B3_HD B3_INL void b3_loose_defaults(B3Loose* s) {
    s->gravity = b3_v(0.0f, -10.0f, 0.0f);
    s->contact_hertz = 60.0f;
    s->contact_damping = 10.0f;
    s->contact_speed = 3.0f;
    s->max_linear_speed = 400.0f;
    s->n_bodies = 0;
    s->n_shapes = 0;
    s->n_contacts = 0;
    s->overflow = 0;
    s->nb = NULL;
    s->nb_hops = 0;
}

static B3_HD B3_INL float b3_mani_min_sep(const B3Mani* m) {
    float s = FLT_MAX;
    for (int i = 0; i < m->count; i++) {
        if (m->sep[i] < s) {
            s = m->sep[i];
        }
    }
    return s;
}

static inline int b3_loose_init(B3Loose* s, int cap_b, int cap_s, int cap_c) {
    memset(s, 0, sizeof(*s));
    b3_loose_defaults(s);
    s->cap_bodies = cap_b;
    s->cap_shapes = cap_s;
    s->cap_contacts = cap_c;
    s->bodies = (B3Body*)calloc((size_t)cap_b, sizeof(B3Body));
    s->shapes = (B3Shape*)calloc((size_t)cap_s, sizeof(B3Shape));
    s->contacts = (B3Contact*)calloc((size_t)cap_c, sizeof(B3Contact));
    s->nb = (int*)malloc((size_t)cap_b * (size_t)B3_LOOSE_NB * sizeof(int));
    if (s->nb) {
        for (int i = 0; i < cap_b * B3_LOOSE_NB; i++) {
            s->nb[i] = -1;
        }
    }
    return s->bodies && s->shapes && s->contacts && s->nb;
}

static inline void b3_loose_free(B3Loose* s) {
    free(s->bodies);
    free(s->shapes);
    free(s->contacts);
    free(s->nb);
    memset(s, 0, sizeof(*s));
}

static inline int b3_loose_add_body(B3Loose* s, const B3BodyDef* def) {
    if (s->n_bodies >= s->cap_bodies) {
        s->overflow = 1;
        return -1;
    }
    int id = s->n_bodies++;
    b3_body_from_def(&s->bodies[id], def);
    return id;
}

static inline int b3_loose_add_shape(B3Loose* s, int body, int type,
        B3Vec3 local_pos, B3Quat local_rot, float radius, B3Vec3 half,
        const B3ShapeDef* def) {
    if (s->n_shapes >= s->cap_shapes) {
        s->overflow = 1;
        return -1;
    }
    int id = s->n_shapes++;
    b3_shape_fill(&s->shapes[id], body, type, local_pos, local_rot,
        radius, half, def);
    return id;
}

static inline void b3_loose_finalize_mass(B3Loose* s, int body) {
    b3_finalize_mass_of(&s->bodies[body], s->shapes, s->n_shapes, body);
}

static inline int b3_loose_add_ground(B3Loose* s) {
    B3BodyDef bd = b3_default_body();
    bd.type = B3_STATIC;
    bd.position = b3_v(0.0f, -0.05f, 0.0f);
    int id = b3_loose_add_body(s, &bd);
    if (id < 0) {
        return -1;
    }
    B3ShapeDef sd = b3_default_shape();
    sd.density = 0.0f;
    sd.friction = 1.0f;
    sd.restitution = 0.0f;
    sd.category = MJCF_COL_GROUND;
    sd.mask = MJCF_COL_CUBE;
    b3_loose_add_shape(s, id, B3_BOX, b3_v(0, 0, 0), b3_q_id(), 0.0f,
        b3_v(50.0f, 0.05f, 50.0f), &sd);
    b3_loose_finalize_mass(s, id);
    return id;
}

static inline int b3_loose_spawn_grid(B3Loose* s, int nx, int ny, int nz,
        float size, float gap, float density, B3Vec3 origin) {
    nx = nx < 1 ? 1 : nx;
    ny = ny < 1 ? 1 : ny;
    nz = nz < 1 ? 1 : nz;
    float pitch = size + gap;
    float half = 0.5f * size;
    B3ShapeDef sd = b3_default_shape();
    sd.density = density;
    sd.friction = 0.8f;
    sd.restitution = 0.0f;
    sd.category = MJCF_COL_CUBE;
    sd.mask = MJCF_COL_GROUND | MJCF_COL_CUBE | MJCF_COL_FEET | MJCF_COL_BODY;
    int added = 0;
    for (int iy = 0; iy < ny; iy++) {
        for (int iz = 0; iz < nz; iz++) {
            for (int ix = 0; ix < nx; ix++) {
                B3BodyDef bd = b3_default_body();
                bd.type = B3_DYNAMIC;
                bd.linear_damping = 0.25f;
                bd.angular_damping = 0.40f;
                bd.gravity_scale = 1.0f;
                bd.position = b3_v(
                    origin.x + ((float)ix - 0.5f * (float)(nx - 1)) * pitch,
                    origin.y + half + (float)iy * pitch,
                    origin.z + ((float)iz - 0.5f * (float)(nz - 1)) * pitch);
                int id = b3_loose_add_body(s, &bd);
                if (id < 0) {
                    return added;
                }
                b3_loose_add_shape(s, id, B3_BOX, b3_v(0, 0, 0), b3_q_id(),
                    0.0f, b3_v(half, half, half), &sd);
                b3_loose_finalize_mass(s, id);
                added += 1;
            }
        }
    }
    return added;
}

static inline void b3_loose_bind_grid(B3Loose* s, int nx, int ny, int nz) {
    int n = nx * ny * nz;
    if (n < 1 || !s->nb) {
        return;
    }
    int first = s->n_bodies - n;
    if (first < 0) {
        first = 0;
    }
    s->nb_hops = nx + ny + nz;
    if (s->nb_hops < 1) {
        s->nb_hops = 1;
    }
    for (int iy = 0; iy < ny; iy++) {
        for (int iz = 0; iz < nz; iz++) {
            for (int ix = 0; ix < nx; ix++) {
                int id = first + iy * (nx * nz) + iz * nx + ix;
                int* nb = &s->nb[id * B3_LOOSE_NB];
                for (int k = 0; k < B3_LOOSE_NB; k++) {
                    nb[k] = -1;
                }
                if (ix > 0) {
                    nb[0] = id - 1;
                }
                if (ix + 1 < nx) {
                    nb[1] = id + 1;
                }
                if (iy > 0) {
                    nb[2] = id - nx * nz;
                }
                if (iy + 1 < ny) {
                    nb[3] = id + nx * nz;
                }
                if (iz > 0) {
                    nb[4] = id - nx;
                }
                if (iz + 1 < nz) {
                    nb[5] = id + nx;
                }
            }
        }
    }
}

static inline void b3_loose_wake_flood(B3Loose* s) {
    if (!s->nb) {
        return;
    }
    int hops = s->nb_hops < 1 ? 1 : s->nb_hops;
    for (int h = 0; h < hops; h++) {
        for (int i = 0; i < s->n_bodies; i++) {
            if (s->bodies[i].type != B3_DYNAMIC) {
                continue;
            }
            for (int k = 0; k < B3_LOOSE_NB; k++) {
                int j = s->nb[i * B3_LOOSE_NB + k];
                if (j >= 0 && j < s->n_bodies
                        && s->bodies[j].inv_mass > 0.0f) {
                    s->bodies[j].type = B3_DYNAMIC;
                    s->bodies[j].flags |= B3_FLAG_DYNAMIC;
                }
            }
        }
    }
}

static inline void b3_loose_freeze(B3Loose* s) {
    for (int i = 0; i < s->n_bodies; i++) {
        B3Body* b = &s->bodies[i];
        if (b->inv_mass <= 0.0f) {
            continue;
        }
        b->type = B3_STATIC;
        b->flags &= ~B3_FLAG_DYNAMIC;
        b->lin_vel = b3_v(0.0f, 0.0f, 0.0f);
        b->ang_vel = b3_v(0.0f, 0.0f, 0.0f);
        b->force = b3_v(0.0f, 0.0f, 0.0f);
        b->torque = b3_v(0.0f, 0.0f, 0.0f);
        b->delta_pos = b3_v(0.0f, 0.0f, 0.0f);
        b->delta_rot = b3_q_id();
    }
}

static inline void b3_x_wake(B3Loose* cubes, const B3Contact* xc, int n) {
    for (int i = 0; i < n; i++) {
        B3Body* b = &cubes->bodies[xc[i].body_b];
        if (b->inv_mass <= 0.0f) {
            continue;
        }
        b->type = B3_DYNAMIC;
        b->flags |= B3_FLAG_DYNAMIC;
    }
}

static inline int b3_loose_any_dynamic(const B3Loose* s) {
    for (int i = 0; i < s->n_bodies; i++) {
        if (s->bodies[i].type == B3_DYNAMIC) {
            return 1;
        }
    }
    return 0;
}

static B3_HD B3_INL void b3_gyro_step(B3Body* b, float h) {
    if (b->type != B3_DYNAMIC) {
        return;
    }
    B3Vec3 inv = b->inv_inertia;
    B3Vec3 w_local = b3_inv_rotate(b->rotation, b->ang_vel);
    B3Vec3 Iwl = b3_v(
        inv.x > 0.0f ? w_local.x / inv.x : 0.0f,
        inv.y > 0.0f ? w_local.y / inv.y : 0.0f,
        inv.z > 0.0f ? w_local.z / inv.z : 0.0f);
    B3Vec3 Iw = b3_rotate(b->rotation, Iwl);
    B3Vec3 gyro = b3_cross(b->ang_vel, Iw);
    b->ang_vel = b3_sub(b->ang_vel, b3_mul(b3_mv(b->inv_i_world, gyro), h));
}

static B3_HD B3_INL void b3_loose_body_int_v(B3Body* b, B3Vec3 gravity,
        float h) {
    b3_integrate_velocity_state(b, gravity, h, &b->lin_vel, &b->ang_vel);
    b3_gyro_step(b, h);
}

static B3_HD B3_INL void b3_loose_int_v(B3Loose* s, float h) {
    for (int i = 0; i < s->n_bodies; i++) {
        b3_loose_body_int_v(&s->bodies[i], s->gravity, h);
    }
}

static B3_HD B3_INL void b3_loose_body_int_p(B3Body* b, float h, float inv_dt,
        float max_lin) {
    if (b->type == B3_STATIC) {
        return;
    }
    float max_ang = B3_MAX_ROTATION * inv_dt;
    B3Vec3 v = b->lin_vel;
    B3Vec3 av = b->ang_vel;
    float v2 = b3_len2(v);
    float max_lin2 = max_lin * max_lin;
    if (v2 > max_lin2 && v2 > 0.0f) {
        v = b3_mul(v, max_lin / sqrtf(v2));
    }
    float w2 = b3_len2(av);
    float max_ang2 = max_ang * max_ang;
    if (w2 > max_ang2 && w2 > 0.0f) {
        av = b3_mul(av, max_ang / sqrtf(w2));
    }
    b->lin_vel = v;
    b->ang_vel = av;
    b->delta_pos = b3_madd(b->delta_pos, h, v);
    b->delta_rot = b3_q_integrate(b->delta_rot, b3_mul(av, h));
}

static B3_HD B3_INL void b3_loose_int_p(B3Loose* s, float h, float inv_dt) {
    for (int i = 0; i < s->n_bodies; i++) {
        b3_loose_body_int_p(&s->bodies[i], h, inv_dt, s->max_linear_speed);
    }
}

static B3_HD B3_INL void b3_loose_fin(B3Loose* s) {
    for (int i = 0; i < s->n_bodies; i++) {
        b3_body_fin(&s->bodies[i]);
    }
}

static inline void b3_loose_find(B3Loose* s) {
    B3AABB* aabb = (B3AABB*)malloc((size_t)s->n_shapes * sizeof(B3AABB));
    if (!aabb) {
        s->overflow = 1;
        s->n_contacts = 0;
        return;
    }
    B3Vec3 pad = b3_v(B3_LOOSE_SPECULATIVE, B3_LOOSE_SPECULATIVE,
        B3_LOOSE_SPECULATIVE);
    for (int i = 0; i < s->n_shapes; i++) {
        aabb[i] = b3_shape_aabb(&s->bodies[s->shapes[i].body], &s->shapes[i]);
        aabb[i].lo = b3_sub(aabb[i].lo, pad);
        aabb[i].hi = b3_add(aabb[i].hi, pad);
    }
    int old_n = s->n_contacts;
    B3Warm* old = NULL;
    if (old_n > 0) {
        old = (B3Warm*)malloc((size_t)old_n * sizeof(B3Warm));
        if (old) {
            for (int i = 0; i < old_n; i++) {
                const B3Contact* src = &s->contacts[i];
                old[i].shape_a = src->shape_a;
                old[i].shape_b = src->shape_b;
                old[i].point_count = src->point_count;
                old[i].friction_impulse = src->friction_impulse;
                old[i].twist_impulse = src->twist_impulse;
                old[i].rolling_impulse = src->rolling_impulse;
                for (int p = 0; p < src->point_count; p++) {
                    old[i].feature[p] = src->points[p].feature;
                    old[i].normal_impulse[p] = src->points[p].normal_impulse;
                }
            }
        }
    }
    s->n_contacts = 0;
    for (int i = 0; i < s->n_shapes; i++) {
        B3Shape* sa = &s->shapes[i];
        B3Body* ba = &s->bodies[sa->body];
        for (int j = i + 1; j < s->n_shapes; j++) {
            B3Shape* sb = &s->shapes[j];
            B3Body* bb = &s->bodies[sb->body];
            if (sa->body == sb->body
                    || (ba->type == B3_STATIC && bb->type == B3_STATIC)
                    || (sa->category & sb->mask) == 0
                    || (sb->category & sa->mask) == 0
                    || !b3_aabb_overlap(aabb[i], aabb[j])) {
                continue;
            }
            B3Mani mani;
            b3_collide_pair(&mani, ba, sa, bb, sb);
            if (mani.count == 0
                    || b3_mani_min_sep(&mani) > B3_LOOSE_SPECULATIVE) {
                continue;
            }
            if (s->n_contacts >= s->cap_contacts) {
                s->overflow = 1;
                continue;
            }
            B3Contact* c = &s->contacts[s->n_contacts++];
            b3_contact_from_mani(c, i, j, sa, sb, ba, bb, &mani);
            if (old) {
                for (int k = 0; k < old_n; k++) {
                    if (old[k].shape_a != i || old[k].shape_b != j) {
                        continue;
                    }
                    for (int p = 0; p < c->point_count; p++) {
                        for (int q = 0; q < old[k].point_count; q++) {
                            if (c->points[p].feature == old[k].feature[q]) {
                                c->points[p].normal_impulse =
                                    old[k].normal_impulse[q];
                            }
                        }
                    }
                    c->friction_impulse = old[k].friction_impulse;
                    c->twist_impulse = old[k].twist_impulse;
                    c->rolling_impulse = old[k].rolling_impulse;
                }
            }
        }
    }
    free(old);
    free(aabb);
}

static inline void b3_loose_prepare(B3Loose* s, B3Soft cs, B3Soft ss) {
    for (int i = 0; i < s->n_contacts; i++) {
        B3Contact* c = &s->contacts[i];
        b3_prepare_one_contact(c, &s->bodies[c->body_a],
            &s->bodies[c->body_b], cs, ss);
    }
}

static inline void b3_loose_warm(B3Loose* s) {
    for (int i = 0; i < s->n_contacts; i++) {
        B3Contact* c = &s->contacts[i];
        b3_warm_one_contact(c, &s->bodies[c->body_a], &s->bodies[c->body_b]);
    }
}

static inline void b3_loose_gs(B3Loose* s, float inv_h, int use_bias,
        int iters) {
    int n = iters < 1 ? B3_LOOSE_ITERS : iters;
    for (int it = 0; it < n; it++) {
        b3_solve_contacts_n(s->contacts, s->n_contacts, s->bodies,
            inv_h, s->contact_speed, use_bias);
    }
}

static inline void b3_loose_begin(B3Loose* s, B3Soft cs, B3Soft ss) {
    b3_loose_find(s);
    b3_loose_prepare(s, cs, ss);
    b3_loose_warm(s);
}

static inline void b3_loose_step(B3Loose* s, float dt, int substeps) {
    if (dt <= 0.0f || s->n_bodies < 1) {
        return;
    }
    int subs = substeps < 1 ? 1 : substeps;
    float h = dt / (float)subs;
    float inv_dt = 1.0f / dt;
    float inv_h = (float)subs * inv_dt;
    float hertz = b3_minf(s->contact_hertz, 0.25f * inv_h);
    B3Soft cs = b3_make_soft(hertz, s->contact_damping, h);
    B3Soft ss = b3_make_soft(2.0f * hertz, 0.5f * s->contact_damping, h);
    b3_loose_begin(s, cs, ss);
    for (int k = 0; k < subs; k++) {
        b3_loose_int_v(s, h);
        b3_loose_gs(s, inv_h, 1, B3_LOOSE_ITERS);
        b3_loose_int_p(s, h, inv_dt);
        b3_loose_gs(s, inv_h, 0, B3_LOOSE_ITERS);
    }
    b3_loose_fin(s);
}

static inline int b3_x_find(B3World* robot, B3Loose* cubes, B3Contact* xc,
        int cap) {
    int n = 0;
    B3Vec3 pad = b3_v(B3_SPECULATIVE, B3_SPECULATIVE, B3_SPECULATIVE);
    for (int i = 0; i < robot->shape_count; i++) {
        B3Shape* sa = &robot->shapes[i];
        if ((sa->mask & MJCF_COL_CUBE) == 0
                && (sa->category & MJCF_COL_CUBE) == 0) {
            continue;
        }
        B3Body* ba = &robot->bodies[sa->body];
        B3AABB aa = b3_shape_aabb(ba, sa);
        aa.lo = b3_sub(aa.lo, pad);
        aa.hi = b3_add(aa.hi, pad);
        for (int j = 0; j < cubes->n_shapes; j++) {
            B3Shape* sb = &cubes->shapes[j];
            B3Body* bb = &cubes->bodies[sb->body];
            if ((bb->type == B3_STATIC && ba->type == B3_STATIC)
                    || (sa->category & sb->mask) == 0
                    || (sb->category & sa->mask) == 0) {
                continue;
            }
            B3AABB ab = b3_shape_aabb(bb, sb);
            ab.lo = b3_sub(ab.lo, pad);
            ab.hi = b3_add(ab.hi, pad);
            if (!b3_aabb_overlap(aa, ab)) {
                continue;
            }
            B3Mani mani;
            b3_collide_pair(&mani, ba, sa, bb, sb);
            if (mani.count == 0) {
                continue;
            }
            if (n >= cap) {
                cubes->overflow = 1;
                return n;
            }
            b3_contact_from_mani(&xc[n], i, j, sa, sb, ba, bb, &mani);
            n += 1;
        }
    }
    return n;
}

static inline void b3_x_prepare(B3World* robot, B3Loose* cubes, B3Contact* xc,
        int n, B3Soft cs, B3Soft ss) {
    for (int i = 0; i < n; i++) {
        B3Contact* c = &xc[i];
        b3_prepare_one_contact(c, &robot->bodies[c->body_a],
            &cubes->bodies[c->body_b], cs, ss);
    }
}

static inline void b3_x_warm(B3World* robot, B3Loose* cubes, B3Contact* xc,
        int n) {
    for (int i = 0; i < n; i++) {
        B3Contact* c = &xc[i];
        b3_warm_one_contact(c, &robot->bodies[c->body_a],
            &cubes->bodies[c->body_b]);
    }
}

static inline void b3_x_gs(B3World* robot, B3Loose* cubes, B3Contact* xc,
        int n, float inv_h, float speed, int use_bias, int iters) {
    int k = iters < 1 ? B3_LOOSE_ITERS : iters;
    for (int it = 0; it < k; it++) {
        for (int i = 0; i < n; i++) {
            B3Contact* c = &xc[i];
            b3_solve_one_contact(c, &robot->bodies[c->body_a],
                &cubes->bodies[c->body_b], inv_h, speed, use_bias);
        }
    }
}

static inline void b3_loose_coupled_step(B3World* robot, B3Loose* cubes,
        B3Contact* xc, int xcap, int* n_x, float dt, int substeps) {
    if (dt <= 0.0f) {
        return;
    }
    int subs = substeps < 1 ? 1 : substeps;
    float h, inv_h, inv_dt;
    B3Soft rcs, rss;
    b3_soft_step_params(robot, dt, substeps, &h, &inv_h, &inv_dt, &rcs, &rss);
    float hertz = b3_minf(cubes->contact_hertz, 0.25f * inv_h);
    B3Soft ccs = b3_make_soft(hertz, cubes->contact_damping, h);
    B3Soft css = b3_make_soft(2.0f * hertz, 0.5f * cubes->contact_damping, h);
    b3_step_begin(robot, h, rcs, rss);
    int nx = b3_x_find(robot, cubes, xc, xcap);
    if (n_x) {
        *n_x = nx;
    }
    b3_x_wake(cubes, xc, nx);
    b3_loose_wake_flood(cubes);
    if (nx < 1 && !b3_loose_any_dynamic(cubes)) {
        for (int k = 0; k < subs; k++) {
            b3_step_sub(robot, h, inv_h, inv_dt);
        }
        b3_finalize_transforms(robot);
        return;
    }
    b3_loose_begin(cubes, ccs, css);
    b3_x_prepare(robot, cubes, xc, nx, rcs, rss);
    b3_x_warm(robot, cubes, xc, nx);
    for (int k = 0; k < subs; k++) {
        b3_integrate_velocities(robot, h);
        b3_loose_int_v(cubes, h);
        b3_solve_joints(robot, h, inv_h, 1);
        b3_loose_gs(cubes, inv_h, 1, B3_LOOSE_ITERS);
        b3_x_gs(robot, cubes, xc, nx, inv_h, cubes->contact_speed, 1, 4);
        b3_integrate_positions(robot, h, inv_dt, robot->max_linear_speed);
        b3_loose_int_p(cubes, h, inv_dt);
        b3_solve_joints(robot, h, inv_h, 0);
        b3_loose_gs(cubes, inv_h, 0, B3_LOOSE_ITERS);
        b3_x_gs(robot, cubes, xc, nx, inv_h, cubes->contact_speed, 0, 4);
    }
    b3_finalize_transforms(robot);
    b3_loose_fin(cubes);
}

#ifdef __CUDACC__
static inline int b3_loose_grid(int n, int block) {
    return n > 0 ? (n + block - 1) / block : 1;
}

__global__ void b3_loose_k_int_v(B3Body* bodies, int n, B3Vec3 gravity,
        float h, const int* gate) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        b3_loose_body_int_v(&bodies[i], gravity, h);
    }
}

__global__ void b3_loose_k_int_p(B3Body* bodies, int n, float h, float inv_dt,
        float max_lin, const int* gate) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        b3_loose_body_int_p(&bodies[i], h, inv_dt, max_lin);
    }
}

__global__ void b3_loose_k_fin(B3Body* bodies, int n, const int* gate) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        b3_body_fin(&bodies[i]);
    }
}

__global__ void b3_loose_k_pairs(B3Body* bodies, B3Shape* shapes, int n_shapes,
        B3Contact* contacts, int cap, int* n_contacts, int* overflow) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= n_shapes || j >= n_shapes || j <= i) {
        return;
    }
    B3Shape* sa = &shapes[i];
    B3Shape* sb = &shapes[j];
    B3Body* ba = &bodies[sa->body];
    B3Body* bb = &bodies[sb->body];
    if (sa->body == sb->body
            || (ba->type == B3_STATIC && bb->type == B3_STATIC)
            || (sa->category & sb->mask) == 0
            || (sb->category & sa->mask) == 0) {
        return;
    }
    B3Vec3 pad = b3_v(B3_LOOSE_SPECULATIVE, B3_LOOSE_SPECULATIVE,
        B3_LOOSE_SPECULATIVE);
    B3AABB aa = b3_shape_aabb(ba, sa);
    B3AABB ab = b3_shape_aabb(bb, sb);
    aa.lo = b3_sub(aa.lo, pad);
    aa.hi = b3_add(aa.hi, pad);
    ab.lo = b3_sub(ab.lo, pad);
    ab.hi = b3_add(ab.hi, pad);
    if (!b3_aabb_overlap(aa, ab)) {
        return;
    }
    B3Mani mani;
    b3_collide_pair(&mani, ba, sa, bb, sb);
    if (mani.count == 0 || b3_mani_min_sep(&mani) > B3_LOOSE_SPECULATIVE) {
        return;
    }
    int slot = atomicAdd(n_contacts, 1);
    if (slot >= cap) {
        atomicAdd(overflow, 1);
        return;
    }
    b3_contact_from_mani(&contacts[slot], i, j, sa, sb, ba, bb, &mani);
}

__global__ void b3_loose_k_find_seq(B3Body* bodies, B3Shape* shapes,
        int n_shapes, B3Contact* contacts, int cap, int* n_contacts,
        int* overflow) {
    if (threadIdx.x != 0 || blockIdx.x != 0) {
        return;
    }
    *n_contacts = 0;
    *overflow = 0;
    B3Vec3 pad = b3_v(B3_LOOSE_SPECULATIVE, B3_LOOSE_SPECULATIVE,
        B3_LOOSE_SPECULATIVE);
    for (int i = 0; i < n_shapes; i++) {
        B3Shape* sa = &shapes[i];
        B3Body* ba = &bodies[sa->body];
        B3AABB aa = b3_shape_aabb(ba, sa);
        aa.lo = b3_sub(aa.lo, pad);
        aa.hi = b3_add(aa.hi, pad);
        for (int j = i + 1; j < n_shapes; j++) {
            B3Shape* sb = &shapes[j];
            B3Body* bb = &bodies[sb->body];
            if (sa->body == sb->body
                    || (ba->type == B3_STATIC && bb->type == B3_STATIC)
                    || (sa->category & sb->mask) == 0
                    || (sb->category & sa->mask) == 0) {
                continue;
            }
            B3AABB ab = b3_shape_aabb(bb, sb);
            ab.lo = b3_sub(ab.lo, pad);
            ab.hi = b3_add(ab.hi, pad);
            if (!b3_aabb_overlap(aa, ab)) {
                continue;
            }
            B3Mani mani;
            b3_collide_pair(&mani, ba, sa, bb, sb);
            if (mani.count == 0
                    || b3_mani_min_sep(&mani) > B3_LOOSE_SPECULATIVE) {
                continue;
            }
            int slot = *n_contacts;
            if (slot >= cap) {
                *overflow += 1;
                continue;
            }
            *n_contacts = slot + 1;
            b3_contact_from_mani(&contacts[slot], i, j, sa, sb, ba, bb, &mani);
        }
    }
}

__global__ void b3_loose_k_prepare(B3Contact* contacts, B3Body* bodies,
        const int* n_contacts, int cap, B3Soft cs, B3Soft ss) {
    int n = *n_contacts;
    if (n > cap) {
        n = cap;
    }
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) {
        return;
    }
    B3Contact* c = &contacts[i];
    b3_prepare_one_contact(c, &bodies[c->body_a], &bodies[c->body_b], cs, ss);
}

__global__ void b3_loose_k_wake_hit(B3Contact* xc, const int* n_x, int cap,
        B3Body* bodies) {
    if (threadIdx.x != 0 || blockIdx.x != 0) {
        return;
    }
    int n = *n_x;
    if (n > cap) {
        n = cap;
    }
    for (int i = 0; i < n; i++) {
        B3Body* b = &bodies[xc[i].body_b];
        if (b->inv_mass > 0.0f) {
            b->type = B3_DYNAMIC;
            b->flags |= B3_FLAG_DYNAMIC;
        }
    }
}

__global__ void b3_loose_k_wake_nb(B3Body* bodies, const int* nb, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n || !nb || bodies[i].type != B3_DYNAMIC) {
        return;
    }
    for (int k = 0; k < B3_LOOSE_NB; k++) {
        int j = nb[i * B3_LOOSE_NB + k];
        if (j >= 0 && j < n && bodies[j].inv_mass > 0.0f) {
            bodies[j].type = B3_DYNAMIC;
            bodies[j].flags |= B3_FLAG_DYNAMIC;
        }
    }
}

__global__ void b3_loose_k_color_clear(int* color, int* lock, int n_c, int n_b) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n_c) {
        color[i] = -1;
    }
    if (i < n_b) {
        lock[i] = -1;
    }
}

__global__ void b3_loose_k_lock_reset(int* lock, int n_b) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n_b) {
        lock[i] = -1;
    }
}

__global__ void b3_loose_k_color_pass(B3Contact* contacts, B3Body* bodies,
        int n, int* color, int* lock, int pass, int* assigned) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n || color[i] >= 0) {
        return;
    }
    B3Contact* c = &contacts[i];
    int a = c->body_a;
    int b = c->body_b;
    int da = bodies[a].type == B3_DYNAMIC;
    int db = bodies[b].type == B3_DYNAMIC;
    int got_a = 1;
    int got_b = 1;
    if (da) {
        got_a = atomicCAS(&lock[a], -1, i) == -1;
    }
    if (db) {
        got_b = atomicCAS(&lock[b], -1, i) == -1;
    }
    if (!got_a || !got_b) {
        if (da && got_a) {
            lock[a] = -1;
        }
        if (db && got_b) {
            lock[b] = -1;
        }
        return;
    }
    color[i] = pass;
    atomicAdd(assigned, 1);
}

__global__ void b3_loose_k_gs_color(B3Contact* contacts, B3Body* bodies,
        int n, const int* color, int col, float inv_h, float speed,
        int use_bias) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n || color[i] != col) {
        return;
    }
    B3Contact* c = &contacts[i];
    b3_solve_one_contact(c, &bodies[c->body_a], &bodies[c->body_b],
        inv_h, speed, use_bias);
}

__global__ void b3_loose_k_gs_seq(B3Contact* contacts, B3Body* bodies,
        const int* n_contacts, int cap, float inv_h, float speed,
        int use_bias, int iters, const int* gate) {
    if (threadIdx.x != 0 || blockIdx.x != 0) {
        return;
    }
    int n = *n_contacts;
    if (n > cap) {
        n = cap;
    }
    for (int it = 0; it < iters; it++) {
        b3_solve_contacts_n(contacts, n, bodies, inv_h, speed, use_bias);
    }
}

__global__ void b3_loose_k_gs_block(B3Contact* contacts, B3Body* bodies,
        int n, const int* color, int n_colors, float inv_h, float speed,
        int use_bias, int iters) {
    for (int it = 0; it < iters; it++) {
        for (int col = 0; col < n_colors; col++) {
            for (int i = threadIdx.x; i < n; i += blockDim.x) {
                if (color[i] == col) {
                    b3_solve_one_contact(&contacts[i],
                        &bodies[contacts[i].body_a],
                        &bodies[contacts[i].body_b], inv_h, speed, use_bias);
                }
            }
            __syncthreads();
        }
    }
}

static inline int b3_loose_gpu_color(B3Contact* contacts, B3Body* bodies,
        int n_c, int n_b, int* d_color, int* d_lock, int* d_assigned,
        cudaStream_t stream) {
    int block = 256;
    b3_loose_k_color_clear<<<b3_loose_grid(n_c > n_b ? n_c : n_b, block),
        block, 0, stream>>>(d_color, d_lock, n_c, n_b);
    int colored = 0;
    int n_colors = 0;
    for (int pass = 0; pass < B3_LOOSE_COLORS && colored < n_c; pass++) {
        b3_loose_k_lock_reset<<<b3_loose_grid(n_b, block), block, 0, stream>>>(
            d_lock, n_b);
        int zero = 0;
        cudaMemcpyAsync(d_assigned, &zero, sizeof(int),
            cudaMemcpyHostToDevice, stream);
        b3_loose_k_color_pass<<<b3_loose_grid(n_c, block), block, 0, stream>>>(
            contacts, bodies, n_c, d_color, d_lock, pass, d_assigned);
        int got = 0;
        cudaMemcpyAsync(&got, d_assigned, sizeof(int),
            cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);
        if (got < 1) {
            break;
        }
        colored += got;
        n_colors = pass + 1;
    }
    return n_colors < 1 ? 1 : n_colors;
}
#endif
