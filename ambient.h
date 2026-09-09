#ifndef B3_AMBIENT_H
#define B3_AMBIENT_H
/* Ambient-fluid forces after Padilla, Segall and Sorkine-Hornung,
 * "Rigid Body Dynamics in Ambient Fluids" (arXiv:2601.13971).
 * Buoyancy, dynamic pressure, Blasius skin friction, explicit added-mass
 * reaction f = -A a. Force-only: mass/inertia/impulse solver unchanged.
 * Full-submersion buoyancy; falling-plate spectrum needs Kirchhoff K.
 */
#include <math.h>
#include <string.h>
#include "puffysics.cuh"

#ifndef B3_FLUID_SPHERE_SAMPLES
#define B3_FLUID_SPHERE_SAMPLES 48
#endif
#ifndef B3_FLUID_BOX_FACE_SAMPLES
#define B3_FLUID_BOX_FACE_SAMPLES 3
#endif
#ifndef B3_FLUID_CAPSULE_RINGS
#define B3_FLUID_CAPSULE_RINGS 8
#endif
#ifndef B3_FLUID_ALPHA_N
#define B3_FLUID_ALPHA_N 32
#endif
#define B3_FLUID_GOLDEN_ANGLE 2.39996322972865332f

typedef struct B3Fluid {
    float density;
    float separation_cos;
    float mu;
    float drag_scale;
    float fric_scale;
    float added_scale;
    int sphere_samples;
    int box_face_samples;
    int capsule_rings;
    int buoyancy;
} B3Fluid;

typedef struct B3FluidState {
    B3Vec3 prev_lin[B3_MAX_BODIES];
    B3Vec3 prev_ang[B3_MAX_BODIES];
    int have_prev;
} B3FluidState;

static B3_HD B3_INL B3Fluid b3_fluid_default(void) {
    B3Fluid f;
    memset(&f, 0, sizeof(f));
    f.density = 1.2f;
    f.separation_cos = 0.0f;
    f.mu = 1.8e-5f;
    f.drag_scale = 1.0f;
    f.fric_scale = 1.0f;
    f.added_scale = 1.0f;
    f.sphere_samples = B3_FLUID_SPHERE_SAMPLES;
    f.box_face_samples = B3_FLUID_BOX_FACE_SAMPLES;
    f.capsule_rings = B3_FLUID_CAPSULE_RINGS;
    f.buoyancy = 1;
    return f;
}

static B3_HD B3_INL void b3_fluid_state_init(B3FluidState* st) {
    memset(st, 0, sizeof(*st));
}

static B3_HD B3_INL void b3_fluid_state_reset(B3FluidState* st) {
    st->have_prev = 0;
}

static B3_HD B3_INL float b3_fcomp(B3Vec3 v, int i) {
    return i == 0 ? v.x : (i == 1 ? v.y : v.z);
}

static B3_HD B3_INL B3Vec3 b3_fset(B3Vec3 v, int i, float x) {
    if (i == 0) {
        v.x = x;
    } else if (i == 1) {
        v.y = x;
    } else {
        v.z = x;
    }
    return v;
}

/* Lamb Art. 114 ellipsoid added-mass: Simpson on t in (0,1) after
 * u = L^2 t^2 / (1 - t^2), L = max semi-axis.
 */
static B3_HD B3_INL float b3_fluid_alpha(float s2, float a, float b, float c) {
    float l2 = a * a;
    if (b * b > l2) {
        l2 = b * b;
    }
    if (c * c > l2) {
        l2 = c * c;
    }
    if (l2 <= 1.0e-12f) {
        return 0.0f;
    }
    float abc = a * b * c;
    int n = B3_FLUID_ALPHA_N;
    if (n & 1) {
        n++;
    }
    float hstep = 1.0f / (float)n;
    float sum = 0.0f;
    for (int i = 0; i <= n; i++) {
        float t = (float)i * hstep;
        float inv = 1.0f - t * t;
        float g = 0.0f;
        if (inv > 1.0e-12f) {
            float u = l2 * t * t / inv;
            float du = 2.0f * l2 * t / (inv * inv);
            float d = sqrtf((a * a + u) * (b * b + u) * (c * c + u));
            g = abc * du / ((s2 + u) * d);
        }
        float w = (i == 0 || i == n) ? 1.0f : ((i & 1) ? 4.0f : 2.0f);
        sum += w * g;
    }
    return sum * hstep / 3.0f;
}

static B3_HD B3_INL void b3_fluid_added_diag(const B3Fluid* f,
        float a, float b, float c, float vol, B3Vec3* out) {
    float ax = b3_fluid_alpha(a * a, a, b, c);
    float ay = b3_fluid_alpha(b * b, a, b, c);
    float az = b3_fluid_alpha(c * c, a, b, c);
    out->x = f->density * vol * ax / (2.0f - ax);
    out->y = f->density * vol * ay / (2.0f - ay);
    out->z = f->density * vol * az / (2.0f - az);
}

static B3_HD B3_INL void b3_fluid_volume(const B3Shape* s,
        float* vol, B3Vec3* centroid) {
    if (s->type == B3_SPHERE) {
        *vol = (4.0f / 3.0f) * B3_PI * s->radius * s->radius * s->radius;
        *centroid = s->local_pos;
        return;
    }
    if (s->type == B3_BOX) {
        *vol = 8.0f * s->half.x * s->half.y * s->half.z;
        *centroid = s->local_pos;
        return;
    }
    float r = s->radius;
    *vol = B3_PI * r * r * (2.0f * s->half.y)
        + (4.0f / 3.0f) * B3_PI * r * r * r;
    *centroid = s->local_pos;
}

static B3_HD B3_INL void b3_fluid_equiv_axes(const B3Shape* s,
        float* a, float* b, float* c) {
    if (s->type == B3_SPHERE) {
        *a = *b = *c = s->radius;
        return;
    }
    if (s->type == B3_BOX) {
        *a = s->half.x;
        *b = s->half.y;
        *c = s->half.z;
        return;
    }
    float r = s->radius;
    *a = r;
    *c = r;
    *b = 1.5f * s->half.y + r;
}

static B3_HD B3_INL int b3_fluid_sample(const B3Fluid* f, const B3Shape* s,
        int k, B3Vec3* pos, B3Vec3* normal, float* area) {
    if (s->type == B3_SPHERE) {
        int n = f->sphere_samples > 0 ? f->sphere_samples : 16;
        if (k >= n) {
            return 0;
        }
        float y = 1.0f - 2.0f * ((float)k + 0.5f) / (float)n;
        float r = sqrtf(b3_maxf(0.0f, 1.0f - y * y));
        float th = B3_FLUID_GOLDEN_ANGLE * (float)k;
        *pos = b3_v(s->radius * r * cosf(th),
            s->radius * y, s->radius * r * sinf(th));
        *normal = b3_norm(*pos);
        *area = 4.0f * B3_PI * s->radius * s->radius / (float)n;
        return 1;
    }
    if (s->type == B3_BOX) {
        int kx = f->box_face_samples > 0 ? f->box_face_samples : 1;
        int per_face = kx * kx;
        int face = k / per_face;
        int cell = k - face * per_face;
        if (face >= 6) {
            return 0;
        }
        int axis = face >> 1;
        float sign = (face & 1) ? -1.0f : 1.0f;
        int ua = (axis + 1) % 3;
        int va = (axis + 2) % 3;
        int iu = cell / kx;
        int iv = cell - iu * kx;
        float hu = b3_fcomp(s->half, ua);
        float hv = b3_fcomp(s->half, va);
        float u = -hu + 2.0f * hu * ((float)iu + 0.5f) / (float)kx;
        float v = -hv + 2.0f * hv * ((float)iv + 0.5f) / (float)kx;
        B3Vec3 p = b3_v(0.0f, 0.0f, 0.0f);
        p = b3_fset(p, axis, sign * b3_fcomp(s->half, axis));
        p = b3_fset(p, ua, u);
        p = b3_fset(p, va, v);
        B3Vec3 nn = b3_v(0.0f, 0.0f, 0.0f);
        nn = b3_fset(nn, axis, sign);
        *pos = p;
        *normal = nn;
        *area = (2.0f * hu) * (2.0f * hv) / (float)per_face;
        return 1;
    }
    if (s->type == B3_CAPSULE) {
        int m = f->capsule_rings > 0 ? f->capsule_rings : 2;
        int ka = 8;
        int cyl = m * ka;
        if (k < cyl) {
            int ring = k / ka;
            int around = k - ring * ka;
            float y = -s->half.y + 2.0f * s->half.y
                * ((float)ring + 0.5f) / (float)m;
            float th = B3_FLUID_GOLDEN_ANGLE * (float)around;
            *pos = b3_v(s->radius * cosf(th), y, s->radius * sinf(th));
            *normal = b3_v(cosf(th), 0.0f, sinf(th));
            *area = (2.0f * B3_PI * s->radius) * (2.0f * s->half.y)
                / (float)(m * ka);
            return 1;
        }
        int cn = f->sphere_samples > 0 ? f->sphere_samples : 16;
        int kc = k - cyl;
        int per_cap = cn >> 1;
        if (kc >= 2 * per_cap) {
            return 0;
        }
        int sign = kc >= per_cap ? -1 : 1;
        int j = kc - (sign < 0 ? per_cap : 0);
        float y = 1.0f - 2.0f * ((float)j + 0.5f) / (float)cn;
        float r = sqrtf(b3_maxf(0.0f, 1.0f - y * y));
        float th = B3_FLUID_GOLDEN_ANGLE * (float)j;
        *pos = b3_v(s->radius * r * cosf(th),
            sign * s->half.y + s->radius * y, s->radius * r * sinf(th));
        *normal = b3_norm(*pos);
        *area = 2.0f * B3_PI * s->radius * s->radius / (float)per_cap;
        return 1;
    }
    return 0;
}

static B3_HD B3_INL void b3_fluid_body(const B3Fluid* f, B3World* w,
        B3FluidState* st, int bi, float h) {
    B3Body* b = &w->bodies[bi];
    if (b->type != B3_DYNAMIC) {
        return;
    }
    B3Vec3 f_acc = b3_v(0.0f, 0.0f, 0.0f);
    B3Vec3 t_acc = b3_v(0.0f, 0.0f, 0.0f);
    B3Vec3 a_est = b3_v(0.0f, 0.0f, 0.0f);
    int use_added = st->have_prev && f->added_scale > 0.0f && h > 0.0f;
    if (use_added) {
        a_est = b3_mul(b3_sub(b->lin_vel, st->prev_lin[bi]), 1.0f / h);
    }
    for (int si = 0; si < w->shape_count; si++) {
        B3Shape* s = &w->shapes[si];
        if (s->body != bi) {
            continue;
        }
        B3Vec3 sp = b3_xf_point(b->position, b->rotation, s->local_pos);
        B3Quat sq = b3_qmul(b->rotation, s->local_rot);
        float vol;
        B3Vec3 c_local;
        b3_fluid_volume(s, &vol, &c_local);
        if (f->buoyancy) {
            B3Vec3 cw = b3_xf_point(sp, sq, c_local);
            B3Vec3 fb = b3_mul(w->gravity, -f->density * vol);
            f_acc = b3_add(f_acc, fb);
            t_acc = b3_add(t_acc,
                b3_cross(b3_sub(cw, b->center), fb));
        }
        if (use_added) {
            B3Vec3 ma;
            float ax, ay, az;
            b3_fluid_equiv_axes(s, &ax, &ay, &az);
            b3_fluid_added_diag(f, ax, ay, az, vol, &ma);
            ma = b3_mul(ma, f->added_scale);
            for (int i = 0; i < 3; i++) {
                B3Vec3 axis = b3_v(0.0f, 0.0f, 0.0f);
                axis = b3_fset(axis, i, 1.0f);
                axis = b3_rotate(sq, axis);
                float proj = b3_dot(axis, a_est);
                float m_ai = b3_fcomp(ma, i);
                f_acc = b3_msub(f_acc, m_ai * proj, axis);
            }
        }
        float L = 0.0f;
        if (f->mu > 0.0f && f->fric_scale > 0.0f) {
            if (s->type == B3_SPHERE) {
                L = 2.0f * s->radius * sqrtf(B3_PI);
            } else if (s->type == B3_BOX) {
                L = sqrtf(8.0f * (s->half.x * s->half.y
                    + s->half.y * s->half.z + s->half.x * s->half.z));
            } else {
                float rr = s->radius;
                L = sqrtf(2.0f * B3_PI * rr * (2.0f * s->half.y)
                    + 4.0f * B3_PI * rr * rr);
            }
        }
        B3Vec3 lp, ln;
        float area;
        for (int k = 0; b3_fluid_sample(f, s, k, &lp, &ln, &area); k++) {
            B3Vec3 p = b3_xf_point(sp, sq, lp);
            B3Vec3 n = b3_rotate(sq, ln);
            B3Vec3 r = b3_sub(p, b->center);
            B3Vec3 rdot = b3_add(b->lin_vel, b3_cross(b->ang_vel, r));
            float spd = b3_len(rdot);
            if (spd <= 1.0e-9f) {
                continue;
            }
            float nd = b3_dot(n, rdot);
            if (nd / spd < f->separation_cos) {
                continue;
            }
            float u2 = b3_len2(rdot) - nd * nd;
            float pdyn = -0.5f * f->density * u2 * f->drag_scale;
            B3Vec3 df = b3_mul(n, pdyn * area);
            f_acc = b3_add(f_acc, df);
            t_acc = b3_add(t_acc, b3_cross(r, df));
            if (f->mu > 0.0f && f->fric_scale > 0.0f) {
                B3Vec3 ut = b3_msub(rdot, nd, n);
                float us = b3_len(ut);
                if (us > 1.0e-9f) {
                    float re = f->density * us * L / f->mu;
                    float cf = 0.0576f * powf(re, -0.2f);
                    B3Vec3 ff = b3_mul(ut,
                        (-0.5f * cf * f->density * us * area)
                            * f->fric_scale);
                    f_acc = b3_add(f_acc, ff);
                    t_acc = b3_add(t_acc, b3_cross(r, ff));
                }
            }
        }
    }
    b->force = b3_add(b->force, f_acc);
    b->torque = b3_add(b->torque, t_acc);
    st->prev_lin[bi] = b->lin_vel;
    st->prev_ang[bi] = b->ang_vel;
}

static B3_HD B3_INL void b3_fluid_step(const B3Fluid* f, B3World* w,
        B3FluidState* st, float h) {
    for (int bi = 0; bi < w->body_count; bi++) {
        b3_fluid_body(f, w, st, bi, h);
    }
    st->have_prev = 1;
}
#endif
