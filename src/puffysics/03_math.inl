// ==== puffysics 03_math.inl: INTERNAL: vector/quaternion/matrix helpers, twist, hinge frame math ====
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
B3_HD B3_INL B3Vec3 b3_v(float x, float y, float z) {
    B3Vec3 r;
    r.x = x;
    r.y = y;
    r.z = z;
    return r;
}

B3_HD B3_INL B3Vec3 b3_add(B3Vec3 a, B3Vec3 b) {
    return b3_v(a.x + b.x, a.y + b.y, a.z + b.z);
}

B3_HD B3_INL B3Vec3 b3_sub(B3Vec3 a, B3Vec3 b) {
    return b3_v(a.x - b.x, a.y - b.y, a.z - b.z);
}

B3_HD B3_INL B3Vec3 b3_neg(B3Vec3 a) {
    return b3_v(-a.x, -a.y, -a.z);
}

B3_HD B3_INL B3Vec3 b3_mul(B3Vec3 a, float s) {
    return b3_v(a.x * s, a.y * s, a.z * s);
}

B3_HD B3_INL B3Vec3 b3_madd(B3Vec3 a, float s, B3Vec3 b) {
    return b3_v(a.x + s * b.x, a.y + s * b.y, a.z + s * b.z);
}

B3_HD B3_INL B3Vec3 b3_msub(B3Vec3 a, float s, B3Vec3 b) {
    return b3_v(a.x - s * b.x, a.y - s * b.y, a.z - s * b.z);
}

B3_HD B3_INL float b3_dot(B3Vec3 a, B3Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

B3_HD B3_INL B3Vec3 b3_cross(B3Vec3 a, B3Vec3 b) {
    return b3_v(a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x);
}

B3_HD B3_INL float b3_len2(B3Vec3 a) {
    return b3_dot(a, a);
}

B3_HD B3_INL float b3_len(B3Vec3 a) {
    return sqrtf(b3_len2(a));
}

/* a / sqrt(x): single-rounded when off, fused intrinsic when on. The clamp
 * sites need the off path to stay bit-identical to the historical code. */
B3_HD B3_INL float b3_rsqrt_scale(float a, float x) {
#if defined(__CUDA_ARCH__) && B3_RSQRT_MATH
    return a * rsqrtf(x);
#else
    return a / sqrtf(x);
#endif
}

B3_HD B3_INL float b3_rsqrt(float x) {
#if defined(__CUDA_ARCH__) && B3_RSQRT_MATH
    return rsqrtf(x);
#else
    return 1.0f / sqrtf(x);
#endif
}

B3_HD B3_INL B3Vec3 b3_norm(B3Vec3 a) {
    float l2 = b3_len2(a);
    return l2 > 0.0f ? b3_mul(a, b3_rsqrt(l2)) : b3_v(0.0f, 1.0f, 0.0f);
}

B3_HD B3_INL float b3_clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

B3_HD B3_INL float b3_maxf(float a, float b) {
    return a > b ? a : b;
}

B3_HD B3_INL float b3_minf(float a, float b) {
    return a < b ? a : b;
}

B3_HD B3_INL B3Quat b3_q(float x, float y, float z, float s) {
    B3Quat q;
    q.v = b3_v(x, y, z);
    q.s = s;
    return q;
}

B3_HD B3_INL B3Quat b3_q_id(void) {
    return b3_q(0.0f, 0.0f, 0.0f, 1.0f);
}

B3_HD B3_INL float b3_qdot(B3Quat a, B3Quat b) {
    return b3_dot(a.v, b.v) + a.s * b.s;
}

B3_HD B3_INL B3Quat b3_qnorm(B3Quat q) {
    float d2 = b3_qdot(q, q);
    if (d2 <= 0.0f) {
        return b3_q_id();
    }
    float inv = b3_rsqrt(d2);
    return b3_q(q.v.x * inv, q.v.y * inv, q.v.z * inv, q.s * inv);
}

B3_HD B3_INL B3Quat b3_qmul(B3Quat a, B3Quat b) {
    B3Quat r;
    r.v = b3_add(b3_add(b3_mul(b.v, a.s), b3_mul(a.v, b.s)),
        b3_cross(a.v, b.v));
    r.s = a.s * b.s - b3_dot(a.v, b.v);
    return r;
}

B3_HD B3_INL B3Vec3 b3_rotate(B3Quat q, B3Vec3 v) {
    B3Vec3 t = b3_mul(b3_cross(q.v, v), 2.0f);
    return b3_add(v, b3_add(b3_mul(t, q.s), b3_cross(q.v, t)));
}

B3_HD B3_INL B3Vec3 b3_inv_rotate(B3Quat q, B3Vec3 v) {
    B3Quat c = b3_q(-q.v.x, -q.v.y, -q.v.z, q.s);
    return b3_rotate(c, v);
}

B3_HD B3_INL B3Quat b3_q_integrate(B3Quat q, B3Vec3 dw) {
#if B3_POLY_ROTATION
    float a2 = b3_len2(dw);
    float s, c;
    if (a2 <= B3_MAX_ROTATION * B3_MAX_ROTATION) {
        // Taylor polynomials in |dw|^2 for sin(|dw|/2)/|dw| and
        // cos(|dw|/2). At pi/4, omitted terms are < 7.9e-10 and
        // 1.6e-8 respectively, before float rounding. No sqrt/trig.
        s = 0.5f + a2 * (-1.0f / 48.0f + a2 * (1.0f / 3840.0f
            + a2 * (-1.0f / 645120.0f)));
        c = 1.0f + a2 * (-1.0f / 8.0f + a2 * (1.0f / 384.0f
            + a2 * (-1.0f / 46080.0f)));
    } else {
        // Public helper callers need not respect the stepping speed cap.
        float a = sqrtf(a2);
        s = sinf(0.5f * a) / a;
        c = cosf(0.5f * a);
    }
    // Retain normalization to prevent accumulated float norm drift.
    return b3_qnorm(b3_qmul(b3_q(s * dw.x, s * dw.y, s * dw.z, c), q));
#else
    B3Quat qd = b3_q(0.5f * dw.x, 0.5f * dw.y, 0.5f * dw.z, 0.0f);
    qd = b3_qmul(qd, q);
    return b3_qnorm(b3_q(q.v.x + qd.v.x, q.v.y + qd.v.y,
        q.v.z + qd.v.z, q.s + qd.s));
#endif
}

B3_HD B3_INL void b3_axes(B3Quat q, B3Vec3* x, B3Vec3* y, B3Vec3* z) {
    *x = b3_rotate(q, b3_v(1.0f, 0.0f, 0.0f));
    *y = b3_rotate(q, b3_v(0.0f, 1.0f, 0.0f));
    *z = b3_rotate(q, b3_v(0.0f, 0.0f, 1.0f));
}

B3_HD B3_INL B3Vec3 b3_mv(B3Mat3 m, B3Vec3 v) {
    return b3_add(b3_add(b3_mul(m.cx, v.x), b3_mul(m.cy, v.y)),
        b3_mul(m.cz, v.z));
}

B3_HD B3_INL B3Mat3 b3_maddm(B3Mat3 a, B3Mat3 b) {
    B3Mat3 r;
    r.cx = b3_add(a.cx, b.cx);
    r.cy = b3_add(a.cy, b.cy);
    r.cz = b3_add(a.cz, b.cz);
    return r;
}

B3_HD B3_INL B3Mat3 b3_mat0(void) {
    B3Mat3 m;
    m.cx = b3_v(0.0f, 0.0f, 0.0f);
    m.cy = b3_v(0.0f, 0.0f, 0.0f);
    m.cz = b3_v(0.0f, 0.0f, 0.0f);
    return m;
}

B3_HD B3_INL B3Mat3 b3_world_inv_i(B3Quat q, B3Vec3 inv) {
    B3Vec3 ax, ay, az;
    b3_axes(q, &ax, &ay, &az);
    B3Mat3 r;
    r.cx = b3_add(b3_add(b3_mul(ax, inv.x * ax.x),
        b3_mul(ay, inv.y * ay.x)), b3_mul(az, inv.z * az.x));
    r.cy = b3_add(b3_add(b3_mul(ax, inv.x * ax.y),
        b3_mul(ay, inv.y * ay.y)), b3_mul(az, inv.z * az.y));
    r.cz = b3_add(b3_add(b3_mul(ax, inv.x * ax.z),
        b3_mul(ay, inv.y * ay.z)), b3_mul(az, inv.z * az.z));
    return r;
}

B3_HD B3_INL B3Mat3 b3_invert3(B3Mat3 m) {
    B3Vec3 c0 = b3_cross(m.cy, m.cz);
    float det = b3_dot(m.cx, c0);
    B3Mat3 r = b3_mat0();
    if (fabsf(det) < 1.0e-12f) {
        return r;
    }
    float inv = 1.0f / det;
    B3Vec3 r0 = b3_mul(c0, inv);
    B3Vec3 r1 = b3_mul(b3_cross(m.cz, m.cx), inv);
    B3Vec3 r2 = b3_mul(b3_cross(m.cx, m.cy), inv);
    r.cx = b3_v(r0.x, r1.x, r2.x);
    r.cy = b3_v(r0.y, r1.y, r2.y);
    r.cz = b3_v(r0.z, r1.z, r2.z);
    return r;
}

B3_HD B3_INL B3Mat2 b3_invert2(B3Mat2 k) {
    float det = k.cx.x * k.cy.y - k.cx.y * k.cy.x;
    B3Mat2 r;
    r.cx = (B3Vec2){0.0f, 0.0f};
    r.cy = (B3Vec2){0.0f, 0.0f};
    if (fabsf(det) < 1.0e-12f) {
        return r;
    }
    float inv = 1.0f / det;
    r.cx.x = k.cy.y * inv;
    r.cx.y = -k.cx.y * inv;
    r.cy.x = -k.cy.x * inv;
    r.cy.y = k.cx.x * inv;
    return r;
}

B3_HD B3_INL B3Vec2 b3_mv2(B3Mat2 m, B3Vec2 v) {
    B3Vec2 r;
    r.x = m.cx.x * v.x + m.cy.x * v.y;
    r.y = m.cx.y * v.x + m.cy.y * v.y;
    return r;
}

B3_HD B3_INL B3Vec3 b3_perp(B3Vec3 n) {
    if (fabsf(n.x) >= 0.57735027f) {
        return b3_norm(b3_v(-n.y, n.x, 0.0f));
    }
    return b3_norm(b3_v(0.0f, -n.z, n.y));
}

B3_HD B3_INL B3Soft b3_make_soft(float hertz, float zeta, float h) {
    B3Soft s;
    s.bias_rate = 0.0f;
    s.mass_scale = 0.0f;
    s.impulse_scale = 0.0f;
    if (hertz == 0.0f) {
        return s;
    }
    float omega = 2.0f * B3_PI * hertz;
    float a1 = 2.0f * zeta + h * omega;
    float a2 = h * omega * a1;
    float a3 = 1.0f / (1.0f + a2);
    s.bias_rate = omega / a1;
    s.mass_scale = a2 * a3;
    s.impulse_scale = a3;
    return s;
}

B3_HD B3_INL B3Quat b3_qneg(B3Quat q) {
    return b3_q(-q.v.x, -q.v.y, -q.v.z, -q.s);
}

B3_HD B3_INL B3Quat b3_qconj(B3Quat q) {
    return b3_q(-q.v.x, -q.v.y, -q.v.z, q.s);
}

B3_HD B3_INL B3Quat b3_qinv_mul(B3Quat a, B3Quat b) {
    B3Vec3 t1 = b3_cross(b.v, a.v);
    B3Vec3 t2 = b3_madd(t1, a.s, b.v);
    B3Vec3 t3 = b3_msub(t2, b.s, a.v);
    return b3_q(t3.x, t3.y, t3.z, a.s * b.s + b3_dot(a.v, b.v));
}

// 0: libm; 1: degree-19 polynomial; 2: interpolated read-only LUT;
// 3: same LUT in CUDA constant memory; 4: cubic/quadratic rational.
// All experiments default off.
#ifndef B3_TWIST_APPROX
#define B3_TWIST_APPROX 0
#endif
#if B3_TWIST_APPROX < 0 || B3_TWIST_APPROX > 4
#error "Unknown twist approximation"
#endif
#if B3_TWIST_APPROX
#include "atan_approx.cuh"
#endif

B3_HD B3_INL float b3_twist(B3Quat q) {
#if B3_TWIST_APPROX
    float y = q.s < 0.0f ? -q.v.z : q.v.z;
    float x = q.s < 0.0f ? -q.s : q.s;
    float ay = fabsf(y);
    float hi = b3_maxf(x, ay), lo = b3_minf(x, ay);
    // Preserve atan2's signed-zero, nonfinite and degenerate behavior.
    if (!(hi > 0.0f) || !isfinite(x) || !isfinite(y) || x == 0.0f)
        return 2.0f * atan2f(y, x);
    float a = b3_atan_unit(lo / hi);
    if (ay > x) a = 0.5f * B3_PI - a;
    return 2.0f * copysignf(a, y);
#else
    float t = q.s < 0.0f ? atan2f(-q.v.z, -q.s) : atan2f(q.v.z, q.s);
    return 2.0f * t;
#endif
}

B3_HD B3_INL void b3_hinge_perps(B3Quat qa, B3Quat rel,
        B3Vec3* px, B3Vec3* py) {
    B3Vec3 ex = b3_v(1.0f, 0.0f, 0.0f);
    B3Vec3 ey = b3_v(0.0f, 1.0f, 0.0f);
    *px = b3_mul(b3_rotate(qa,
        b3_add(b3_mul(ex, rel.s), b3_cross(rel.v, ex))), 0.5f);
    *py = b3_mul(b3_rotate(qa,
        b3_add(b3_mul(ey, rel.s), b3_cross(rel.v, ey))), 0.5f);
}

B3_HD B3_INL B3Quat b3_q_axis_angle(B3Vec3 axis, float radians) {
    axis = b3_norm(axis);
    float h = 0.5f * radians;
    float s = sinf(h);
    return b3_q(s * axis.x, s * axis.y, s * axis.z, cosf(h));
}

B3_HD B3_INL B3Quat b3_q_from_z(B3Vec3 z) {
    z = b3_norm(z);
    B3Vec3 from = b3_v(0.0f, 0.0f, 1.0f);
    float d = b3_dot(from, z);
    if (d > 0.999999f) {
        return b3_q_id();
    }
    if (d < -0.999999f) {
        B3Vec3 axis = b3_perp(from);
        return b3_q(axis.x, axis.y, axis.z, 0.0f);
    }
    B3Vec3 axis = b3_cross(from, z);
    return b3_qnorm(b3_q(axis.x, axis.y, axis.z, 1.0f + d));
}

B3_HD B3_INL B3Vec3 b3_solve3(B3Mat3 a, B3Vec3 b) {
    return b3_mv(b3_invert3(a), b);
}

B3_HD B3_INL B3Vec2 b3_solve2(B3Mat2 m, B3Vec2 b) {
    float det = m.cx.x * m.cy.y - m.cx.y * m.cy.x;
    B3Vec2 r;
    r.x = 0.0f;
    r.y = 0.0f;
    if (det <= 1.0e-12f) {
        return r;
    }
    float inv = 1.0f / det;
    r.x = inv * (m.cy.y * b.x - m.cy.x * b.y);
    r.y = inv * (-m.cx.y * b.x + m.cx.x * b.y);
    return r;
}

B3_HD B3_INL B3Vec3 b3_xf_point(B3Vec3 p, B3Quat q, B3Vec3 local) {
    return b3_add(p, b3_rotate(q, local));
}

B3_HD B3_INL int b3_aabb_overlap(B3AABB a, B3AABB b) {
    return a.lo.x <= b.hi.x && a.hi.x >= b.lo.x
        && a.lo.y <= b.hi.y && a.hi.y >= b.lo.y
        && a.lo.z <= b.hi.z && a.hi.z >= b.lo.z;
}
