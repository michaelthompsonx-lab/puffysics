// ==== puffysics 05_collision.inl: INTERNAL: manifolds, primitive tests, SAT box collision ====
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
B3_HD B3_INL B3Vec3 b3_closest_seg(B3Vec3 a, B3Vec3 b, B3Vec3 p) {
    B3Vec3 ab = b3_sub(b, a);
    float d = b3_len2(ab);
    if (d <= 1.0e-12f) {
        return a;
    }
    float t = b3_clamp(b3_dot(b3_sub(p, a), ab) / d, 0.0f, 1.0f);
    return b3_madd(a, t, ab);
}

B3_HD B3_INL void b3_closest_segs(B3Vec3 a1, B3Vec3 a2, B3Vec3 b1,
        B3Vec3 b2, B3Vec3* pa, B3Vec3* pb) {
    B3Vec3 d1 = b3_sub(a2, a1);
    B3Vec3 d2 = b3_sub(b2, b1);
    B3Vec3 r = b3_sub(a1, b1);
    float a = b3_len2(d1);
    float e = b3_len2(d2);
    float f = b3_dot(d2, r);
    float s;
    float t;
    if (a <= 1.0e-12f && e <= 1.0e-12f) {
        *pa = a1;
        *pb = b1;
        return;
    }
    if (a <= 1.0e-12f) {
        s = 0.0f;
        t = b3_clamp(f / e, 0.0f, 1.0f);
    } else {
        float c = b3_dot(d1, r);
        if (e <= 1.0e-12f) {
            t = 0.0f;
            s = b3_clamp(-c / a, 0.0f, 1.0f);
        } else {
            float b = b3_dot(d1, d2);
            float den = a * e - b * b;
            s = den != 0.0f ? b3_clamp((b * f - c * e) / den, 0.0f, 1.0f)
                : 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = b3_clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = b3_clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }
    *pa = b3_madd(a1, s, d1);
    *pb = b3_madd(b1, t, d2);
}

typedef struct B3Mani {
    int count;
    B3Vec3 normal;
    B3Vec3 p_a[B3_MAX_MANIFOLD];
    B3Vec3 p_b[B3_MAX_MANIFOLD];
    float sep[B3_MAX_MANIFOLD];
    uint32_t feature[B3_MAX_MANIFOLD];
} B3Mani;

B3_HD B3_INL void b3_mani_clear(B3Mani* m) {
    m->count = 0;
    m->normal = b3_v(0.0f, 1.0f, 0.0f);
}

B3_HD B3_INL void b3_mani_push(B3Mani* m, B3Vec3 pa, B3Vec3 pb,
        float sep, uint32_t feat) {
    if (m->count >= B3_MAX_MANIFOLD) {
        return;
    }
    int i = m->count++;
    m->p_a[i] = pa;
    m->p_b[i] = pb;
    m->sep[i] = sep;
    m->feature[i] = feat;
}

B3_HD B3_INL void b3_collide_balls(B3Mani* m, B3Vec3 ca, float ra,
        B3Vec3 cb, float rb, uint32_t feat) {
    b3_mani_clear(m);
    B3Vec3 d = b3_sub(cb, ca);
    float dist2 = b3_len2(d);
    float rad = ra + rb;
    if (dist2 > rad * rad) {
        return;
    }
    float dist = sqrtf(dist2);
    B3Vec3 n = dist > 1.0e-8f ? b3_mul(d, b3_rsqrt(dist2))
        : b3_v(0.0f, 1.0f, 0.0f);
    m->normal = n;
    b3_mani_push(m, b3_madd(ca, ra, n), b3_msub(cb, rb, n),
        dist - rad, feat);
}

B3_HD B3_INL B3Vec3 b3_closest_obb(B3Vec3 c, B3Quat q, B3Vec3 half,
        B3Vec3 p, int* inside, int* face) {
    B3Vec3 local = b3_inv_rotate(q, b3_sub(p, c));
    B3Vec3 cl = b3_v(
        b3_clamp(local.x, -half.x, half.x),
        b3_clamp(local.y, -half.y, half.y),
        b3_clamp(local.z, -half.z, half.z));
    int in = fabsf(local.x) <= half.x
        && fabsf(local.y) <= half.y
        && fabsf(local.z) <= half.z;
    *inside = in;
    if (in) {
        float dx = half.x - fabsf(local.x);
        float dy = half.y - fabsf(local.y);
        float dz = half.z - fabsf(local.z);
        if (dx <= dy && dx <= dz) {
            cl.x = local.x >= 0.0f ? half.x : -half.x;
            *face = 0;
        } else if (dy <= dz) {
            cl.y = local.y >= 0.0f ? half.y : -half.y;
            *face = 1;
        } else {
            cl.z = local.z >= 0.0f ? half.z : -half.z;
            *face = 2;
        }
    } else {
        *face = -1;
    }
    return b3_xf_point(c, q, cl);
}

B3_HD B3_INL void b3_collide_ball_point(B3Mani* m, B3Vec3 p, float r,
        B3Vec3 q, int inside, B3Vec3 n_fb, uint32_t feat) {
    B3Vec3 d = b3_sub(p, q);
    float dist2 = b3_len2(d);
    if (!inside && dist2 > r * r) {
        return;
    }
    B3Vec3 n;
    float sep;
    if (inside) {
        n = dist2 > 1.0e-12f ? b3_norm(d) : n_fb;
        sep = -b3_len(d) - r;
    } else {
        float dist = sqrtf(dist2);
        n = b3_mul(d, b3_rsqrt(dist2));
        sep = dist - r;
    }
    m->normal = n;
    b3_mani_push(m, b3_msub(p, r, n), q, sep, feat);
}

B3_HD B3_INL void b3_collide_sphere_box(B3Mani* m, B3Vec3 sc, float r,
        B3Vec3 bc, B3Quat bq, B3Vec3 half) {
    b3_mani_clear(m);
    int inside;
    int face;
    B3Vec3 q = b3_closest_obb(bc, bq, half, sc, &inside, &face);
    B3Vec3 ax, ay, az;
    b3_axes(bq, &ax, &ay, &az);
    B3Vec3 n_fb = face == 0 ? ax : (face == 1 ? ay : az);
    b3_collide_ball_point(m, sc, r, q, inside, n_fb, 3u);
}

B3_HD B3_INL void b3_collide_capsule_box(B3Mani* m,
        B3Vec3 c1, B3Vec3 c2, float r, B3Vec3 bc, B3Quat bq, B3Vec3 half) {
    b3_mani_clear(m);
    B3Vec3 mid = b3_mul(b3_add(c1, c2), 0.5f);
    int inside;
    int face;
    B3Vec3 q = b3_closest_obb(bc, bq, half, mid, &inside, &face);
    B3Vec3 p = b3_closest_seg(c1, c2, q);
    q = b3_closest_obb(bc, bq, half, p, &inside, &face);
    p = b3_closest_seg(c1, c2, q);
    b3_collide_ball_point(m, p, r, q, inside, b3_v(0.0f, 1.0f, 0.0f), 5u);
}

typedef struct B3Obb {
    B3Vec3 c;
    B3Vec3 ax[3];
    B3Vec3 h;
} B3Obb;

B3_HD B3_INL B3Obb b3_obb(B3Vec3 c, B3Quat q, B3Vec3 h) {
    B3Obb o;
    o.c = c;
    o.h = h;
    b3_axes(q, &o.ax[0], &o.ax[1], &o.ax[2]);
    return o;
}

B3_HD B3_INL float b3_half_i(B3Vec3 h, int i) {
    return i == 0 ? h.x : (i == 1 ? h.y : h.z);
}

B3_HD B3_INL B3Vec3 b3_support(const B3Obb* o, B3Vec3 dir) {
    float sx = b3_dot(o->ax[0], dir) < 0.0f ? -o->h.x : o->h.x;
    float sy = b3_dot(o->ax[1], dir) < 0.0f ? -o->h.y : o->h.y;
    float sz = b3_dot(o->ax[2], dir) < 0.0f ? -o->h.z : o->h.z;
    return b3_add(o->c, b3_add(b3_mul(o->ax[0], sx),
        b3_add(b3_mul(o->ax[1], sy), b3_mul(o->ax[2], sz))));
}

B3_HD B3_INL int b3_clip_plane(const B3Vec3* in, int n, B3Vec3* out,
        B3Vec3 plane_n, float plane_o) {
    if (n <= 0) {
        return 0;
    }
    int c = 0;
    B3Vec3 prev = in[n - 1];
    float pd = b3_dot(prev, plane_n) - plane_o;
    int pin = pd <= 0.0f;
    for (int i = 0; i < n; i++) {
        B3Vec3 cur = in[i];
        float cd = b3_dot(cur, plane_n) - plane_o;
        int cin = cd <= 0.0f;
        if (cin != pin && c < 8) {
            float t = pd / (pd - cd);
            out[c++] = b3_madd(prev, t, b3_sub(cur, prev));
        }
        if (cin && c < 8) {
            out[c++] = cur;
        }
        prev = cur;
        pd = cd;
        pin = cin;
    }
    return c;
}

B3_HD B3_INL void b3_face_pts(const B3Obb* o, int axis, float sign,
        B3Vec3 out[4]) {
    int ta = (axis + 1) % 3;
    int tb = (axis + 2) % 3;
    B3Vec3 c = b3_madd(o->c, sign * b3_half_i(o->h, axis), o->ax[axis]);
    B3Vec3 a = b3_mul(o->ax[ta], b3_half_i(o->h, ta));
    B3Vec3 b = b3_mul(o->ax[tb], b3_half_i(o->h, tb));
    out[0] = b3_sub(b3_sub(c, a), b);
    out[1] = b3_add(b3_sub(c, a), b);
    out[2] = b3_add(b3_add(c, a), b);
    out[3] = b3_sub(b3_add(c, a), b);
}

B3_HD B3_INL int b3_best_axis(const B3Obb* o, B3Vec3 n) {
    float b0 = fabsf(b3_dot(o->ax[0], n));
    float b1 = fabsf(b3_dot(o->ax[1], n));
    float b2 = fabsf(b3_dot(o->ax[2], n));
    if (b0 >= b1 && b0 >= b2) {
        return 0;
    }
    return b1 >= b2 ? 1 : 2;
}

B3_HD B3_INL void b3_mani_support(B3Mani* m, const B3Obb* a, const B3Obb* b,
        B3Vec3 n, int feat) {
    B3Vec3 pa = b3_support(a, n);
    B3Vec3 pb = b3_support(b, b3_neg(n));
    b3_mani_push(m, pa, pb, b3_dot(b3_sub(pb, pa), n), (uint32_t)feat);
}

B3_HD B3_INL int b3_try_face(float s, B3Vec3 axis, float side,
        float* best, B3Vec3* n, int feat, int* out_feat) {
    if (s > B3_SPECULATIVE) {
        return -1;
    }
    if (s > *best) {
        *best = s;
        *n = side < 0.0f ? b3_neg(axis) : axis;
        *out_feat = feat;
    }
    return 0;
}

B3_HD B3_INL int b3_try_edge(float e, float ra, float rb, B3Vec3 cr,
        float* best, B3Vec3* n, int feat, int* out_feat) {
    float cl2 = b3_len2(cr);
    if (cl2 <= 1.0e-3f) {
        return 0;
    }
    float inv = b3_rsqrt(cl2);
    float s = (fabsf(e) - ra - rb) * inv;
    if (s > B3_SPECULATIVE && cl2 > 0.05f) {
        return -1;
    }
    if (s > B3_SPECULATIVE) {
        return 0;
    }
    if (s > *best + 0.002f) {
        *best = s;
        *n = e < 0.0f ? b3_mul(cr, -inv) : b3_mul(cr, inv);
        *out_feat = feat;
    }
    return 0;
}

B3_HD B3_INL void b3_collide_boxes(B3Mani* m, const B3Obb* a,
        const B3Obb* b) {
    b3_mani_clear(m);
    B3Vec3 d = b3_sub(b->c, a->c);
    float t0 = b3_dot(d, a->ax[0]);
    float t1 = b3_dot(d, a->ax[1]);
    float t2 = b3_dot(d, a->ax[2]);
    float r00 = b3_dot(a->ax[0], b->ax[0]);
    float r01 = b3_dot(a->ax[0], b->ax[1]);
    float r02 = b3_dot(a->ax[0], b->ax[2]);
    float r10 = b3_dot(a->ax[1], b->ax[0]);
    float r11 = b3_dot(a->ax[1], b->ax[1]);
    float r12 = b3_dot(a->ax[1], b->ax[2]);
    float r20 = b3_dot(a->ax[2], b->ax[0]);
    float r21 = b3_dot(a->ax[2], b->ax[1]);
    float r22 = b3_dot(a->ax[2], b->ax[2]);
    float ar00 = fabsf(r00);
    float ar01 = fabsf(r01);
    float ar02 = fabsf(r02);
    float ar10 = fabsf(r10);
    float ar11 = fabsf(r11);
    float ar12 = fabsf(r12);
    float ar20 = fabsf(r20);
    float ar21 = fabsf(r21);
    float ar22 = fabsf(r22);
    float ax = a->h.x;
    float ay = a->h.y;
    float az = a->h.z;
    float bx = b->h.x;
    float by = b->h.y;
    float bz = b->h.z;
    float best = -FLT_MAX;
    B3Vec3 n = a->ax[0];
    int feat = 0;

    if (b3_try_face(fabsf(t0) - ax - (bx * ar00 + by * ar01 + bz * ar02),
            a->ax[0], t0, &best, &n, 0, &feat) < 0) {
        return;
    }
    if (b3_try_face(fabsf(t1) - ay - (bx * ar10 + by * ar11 + bz * ar12),
            a->ax[1], t1, &best, &n, 1, &feat) < 0) {
        return;
    }
    if (b3_try_face(fabsf(t2) - az - (bx * ar20 + by * ar21 + bz * ar22),
            a->ax[2], t2, &best, &n, 2, &feat) < 0) {
        return;
    }
    float u0 = t0 * r00 + t1 * r10 + t2 * r20;
    float u1 = t0 * r01 + t1 * r11 + t2 * r21;
    float u2 = t0 * r02 + t1 * r12 + t2 * r22;
    if (b3_try_face(fabsf(u0) - bx - (ax * ar00 + ay * ar10 + az * ar20),
            b->ax[0], u0, &best, &n, 3, &feat) < 0) {
        return;
    }
    if (b3_try_face(fabsf(u1) - by - (ax * ar01 + ay * ar11 + az * ar21),
            b->ax[1], u1, &best, &n, 4, &feat) < 0) {
        return;
    }
    if (b3_try_face(fabsf(u2) - bz - (ax * ar02 + ay * ar12 + az * ar22),
            b->ax[2], u2, &best, &n, 5, &feat) < 0) {
        return;
    }

    if (b3_try_edge(t2 * r10 - t1 * r20,
            ay * ar20 + az * ar10, by * ar02 + bz * ar01,
            b3_cross(a->ax[0], b->ax[0]), &best, &n, 6, &feat) < 0) {
        return;
    }
    if (b3_try_edge(t2 * r11 - t1 * r21,
            ay * ar21 + az * ar11, bz * ar00 + bx * ar02,
            b3_cross(a->ax[0], b->ax[1]), &best, &n, 7, &feat) < 0) {
        return;
    }
    if (b3_try_edge(t2 * r12 - t1 * r22,
            ay * ar22 + az * ar12, bx * ar01 + by * ar00,
            b3_cross(a->ax[0], b->ax[2]), &best, &n, 8, &feat) < 0) {
        return;
    }
    if (b3_try_edge(t0 * r20 - t2 * r00,
            az * ar00 + ax * ar20, by * ar12 + bz * ar11,
            b3_cross(a->ax[1], b->ax[0]), &best, &n, 9, &feat) < 0) {
        return;
    }
    if (b3_try_edge(t0 * r21 - t2 * r01,
            az * ar01 + ax * ar21, bz * ar10 + bx * ar12,
            b3_cross(a->ax[1], b->ax[1]), &best, &n, 10, &feat) < 0) {
        return;
    }
    if (b3_try_edge(t0 * r22 - t2 * r02,
            az * ar02 + ax * ar22, bx * ar11 + by * ar10,
            b3_cross(a->ax[1], b->ax[2]), &best, &n, 11, &feat) < 0) {
        return;
    }
    if (b3_try_edge(t1 * r00 - t0 * r10,
            ax * ar10 + ay * ar00, by * ar22 + bz * ar21,
            b3_cross(a->ax[2], b->ax[0]), &best, &n, 12, &feat) < 0) {
        return;
    }
    if (b3_try_edge(t1 * r01 - t0 * r11,
            ax * ar11 + ay * ar01, bz * ar20 + bx * ar22,
            b3_cross(a->ax[2], b->ax[1]), &best, &n, 13, &feat) < 0) {
        return;
    }
    if (b3_try_edge(t1 * r02 - t0 * r12,
            ax * ar12 + ay * ar02, bx * ar21 + by * ar20,
            b3_cross(a->ax[2], b->ax[2]), &best, &n, 14, &feat) < 0) {
        return;
    }

    if (b3_dot(n, d) < 0.0f) {
        n = b3_neg(n);
    }
    m->normal = n;

    if (feat >= 6) {
        b3_mani_support(m, a, b, n, feat);
        return;
    }

    const B3Obb* ref = feat < 3 ? a : b;
    const B3Obb* inc = feat < 3 ? b : a;
    int ref_axis = feat < 3 ? feat : feat - 3;
    float rd = b3_dot(ref->ax[ref_axis], n);
    float ref_sign = (feat < 3 ? rd < 0.0f : rd > 0.0f) ? -1.0f : 1.0f;
    // The incident face opposes the reference face's outward normal.
    // When B is the reference, that normal is opposite the A-to-B normal n.
    B3Vec3 rn = b3_mul(ref->ax[ref_axis], ref_sign);
    int inc_axis = b3_best_axis(inc, rn);
    float inc_sign = b3_dot(inc->ax[inc_axis], rn) > 0.0f ? -1.0f : 1.0f;
    B3Vec3 poly[8];
    B3Vec3 tmp[8];
    b3_face_pts(inc, inc_axis, inc_sign, poly);
    int count = 4;
    int ta = (ref_axis + 1) % 3;
    int tb = (ref_axis + 2) % 3;
    B3Vec3 face_c = b3_madd(ref->c,
        ref_sign * b3_half_i(ref->h, ref_axis), ref->ax[ref_axis]);
    for (int i = 0; i < 4; i++) {
        int t = i < 2 ? ta : tb;
        float s = (i & 1) ? 1.0f : -1.0f;
        B3Vec3 cn = b3_mul(ref->ax[t], s);
        float co = b3_dot(cn, ref->c) + b3_half_i(ref->h, t);
        count = b3_clip_plane(poly, count, tmp, cn, co);
        for (int k = 0; k < count; k++) {
            poly[k] = tmp[k];
        }
        if (count == 0) {
            b3_mani_support(m, a, b, n, feat);
            return;
        }
    }
    float ro = b3_dot(rn, face_c);
    for (int i = 0; i < count && m->count < B3_MAX_MANIFOLD; i++) {
        float plane_s = b3_dot(poly[i], rn) - ro;
        if (plane_s <= B3_SPECULATIVE) {
            B3Vec3 on_ref = b3_msub(poly[i], plane_s, rn);
            B3Vec3 pa = feat < 3 ? on_ref : poly[i];
            B3Vec3 pb = feat < 3 ? poly[i] : on_ref;
            float sep = b3_dot(b3_sub(pb, pa), n);
            b3_mani_push(m, pa, pb, sep,
                (feat < 3 ? 16u : 20u) + (uint32_t)i);
        }
    }
    if (m->count == 0) {
        b3_mani_support(m, a, b, n, feat);
    }
}

B3_HD B3_INL void b3_mani_flip(B3Mani* m) {
    m->normal = b3_neg(m->normal);
    for (int i = 0; i < m->count; i++) {
        B3Vec3 t = m->p_a[i];
        m->p_a[i] = m->p_b[i];
        m->p_b[i] = t;
    }
}

B3_HD B3_INL void b3_collide_pair(B3Mani* m, const B3Body* ba, const B3Shape* sa,
        const B3Body* bb, const B3Shape* sb) {
    B3Vec3 pa = b3_shape_pos(ba, sa);
    B3Vec3 pb = b3_shape_pos(bb, sb);
    B3Vec3 center_a = pa;
    B3Vec3 center_b = pb;
    B3Quat qa = b3_shape_rot(ba, sa);
    B3Quat qb = b3_shape_rot(bb, sb);
    int ta = sa->type;
    int tb = sb->type;
    int flip = 0;
    if (ta > tb) {
        const B3Shape* t = sa;
        sa = sb;
        sb = t;
        B3Vec3 tp = pa;
        pa = pb;
        pb = tp;
        B3Quat tq = qa;
        qa = qb;
        qb = tq;
        int tt = ta;
        ta = tb;
        tb = tt;
        flip = 1;
    }
    if (ta == B3_SPHERE && tb == B3_SPHERE) {
        b3_collide_balls(m, pa, sa->radius, pb, sb->radius, 1u);
    } else if (ta == B3_SPHERE && tb == B3_CAPSULE) {
        B3Vec3 y = b3_rotate(qb, b3_v(0.0f, sb->half.y, 0.0f));
        b3_collide_balls(m, b3_closest_seg(b3_sub(pb, y), b3_add(pb, y),
            pa), sb->radius, pa, sa->radius, 4u);
        b3_mani_flip(m);
    } else if (ta == B3_SPHERE && tb == B3_BOX) {
        b3_collide_sphere_box(m, pa, sa->radius, pb, qb, sb->half);
    } else if (ta == B3_CAPSULE && tb == B3_CAPSULE) {
        B3Vec3 ya = b3_rotate(qa, b3_v(0.0f, sa->half.y, 0.0f));
        B3Vec3 yb = b3_rotate(qb, b3_v(0.0f, sb->half.y, 0.0f));
        B3Vec3 ca, cb;
        b3_closest_segs(b3_sub(pa, ya), b3_add(pa, ya),
            b3_sub(pb, yb), b3_add(pb, yb), &ca, &cb);
        b3_collide_balls(m, ca, sa->radius, cb, sb->radius, 2u);
    } else if (ta == B3_CAPSULE && tb == B3_BOX) {
        B3Vec3 y = b3_rotate(qa, b3_v(0.0f, sa->half.y, 0.0f));
        b3_collide_capsule_box(m, b3_sub(pa, y), b3_add(pa, y),
            sa->radius, pb, qb, sb->half);
    } else {
        B3Obb oa = b3_obb(pa, qa, sa->half);
        B3Obb ob = b3_obb(pb, qb, sb->half);
        b3_collide_boxes(m, &oa, &ob);
    }
    if (flip && m->count > 0) {
        b3_mani_flip(m);
    }
    if (m->count > 0) {
        if (b3_dot(m->normal, b3_sub(center_b, center_a)) < 0.0f) {
            m->normal = b3_neg(m->normal);
        }
        for (int i = 0; i < m->count; i++) {
            m->sep[i] = b3_dot(b3_sub(m->p_b[i], m->p_a[i]), m->normal);
        }
    }
}

B3_HD B3_INL void b3_collide_shapes(B3Mani* m, const B3World* w,
        const B3Shape* sa, const B3Shape* sb) {
    b3_collide_pair(m, &w->bodies[sa->body], sa, &w->bodies[sb->body], sb);
}
