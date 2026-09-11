// ==== puffysics 06_joints.inl: INTERNAL: weld/revolute creation, setters, joint solvers, hinge cache ====
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
B3_HD B3_INL B3Joint* b3_add_joint(B3World* w, int type, int body_a,
        int body_b, B3Vec3 anchor_a, B3Vec3 anchor_b, B3Quat rot_a,
        B3Quat rot_b) {
    assert(w != NULL);
    assert(w->joint_count >= 0 && w->joint_count <= B3_MAX_JOINTS);
    if (w->joint_count >= B3_MAX_JOINTS) {
        w->joint_overflow = 1;
        return NULL;
    }
    if (body_a < 0 || body_a >= w->body_count
            || body_b < 0 || body_b >= w->body_count) {
        w->joint_overflow = 1;
        return NULL;
    }
    int id = w->joint_count++;
    B3Joint* j = &w->joints[id];
    memset(j, 0, sizeof(*j));
#ifndef B3_REVOLUTE_ONLY
    j->type = type;
#else
    (void)type;
#endif
    j->body_a = body_a;
    j->body_b = body_b;
    j->local_anchor_a = anchor_a;
    j->local_anchor_b = anchor_b;
    j->local_rot_a = b3_qnorm(rot_a);
    j->local_rot_b = b3_qnorm(rot_b);
    j->constraint_hertz = 90.0f;
    j->constraint_damping = 2.0f;
    return j;
}

#ifdef B3_REVOLUTE_ONLY
B3_HD B3_INL int b3_create_weld(B3World* w, int body_a, int body_b,
        B3Vec3 local_anchor_a, B3Vec3 local_anchor_b) {
    return -1;
}
#else
B3_HD B3_INL int b3_create_weld(B3World* w, int body_a, int body_b,
        B3Vec3 local_anchor_a, B3Vec3 local_anchor_b) {
    assert(w != NULL);
    if (body_a < 0 || body_a >= w->body_count
            || body_b < 0 || body_b >= w->body_count) {
        w->joint_overflow = 1;
        return -1;
    }
    const B3Body* ba = &w->bodies[body_a];
    const B3Body* bb = &w->bodies[body_b];
    B3Quat rot_a = b3_q_id();
    B3Quat rot_b = b3_qinv_mul(bb->rotation, ba->rotation);
    B3Joint* j = b3_add_joint(w, B3_JOINT_WELD, body_a, body_b,
        local_anchor_a, local_anchor_b, rot_a, rot_b);
    if (j == NULL) {
        return -1;
    }
    return (int)(j - w->joints);
}

#endif

B3_HD B3_INL int b3_create_revolute(B3World* w, int body_a, int body_b,
        B3Vec3 local_anchor_a, B3Vec3 local_anchor_b, B3Vec3 local_axis_a) {
    assert(w != NULL);
    if (body_a < 0 || body_a >= w->body_count
            || body_b < 0 || body_b >= w->body_count) {
        w->joint_overflow = 1;
        return -1;
    }
    const B3Body* ba = &w->bodies[body_a];
    const B3Body* bb = &w->bodies[body_b];
    B3Quat rot_a = b3_q_from_z(local_axis_a);
    B3Quat world = b3_qmul(ba->rotation, rot_a);
    B3Quat rot_b = b3_qinv_mul(bb->rotation, world);
    B3Joint* j = b3_add_joint(w, B3_JOINT_REVOLUTE, body_a, body_b,
        local_anchor_a, local_anchor_b, rot_a, rot_b);
    if (j == NULL) {
        return -1;
    }
    return (int)(j - w->joints);
}

B3_HD B3_INL void b3_joint_enable_motor(B3World* w, int joint, int enable) {
#ifndef B3_REVOLUTE_ONLY
    B3Joint* j = &w->joints[joint];
    if (j->enable_motor != enable) {
        j->motor_impulse = 0.0f;
    }
    j->enable_motor = enable;
#endif
}

B3_HD B3_INL void b3_joint_set_motor(B3World* w, int joint, float speed,
        float max_torque) {
    B3Joint* j = &w->joints[joint];
#ifndef B3_REVOLUTE_ONLY
    j->motor_speed = speed;
#endif
    j->max_motor_torque = b3_maxf(max_torque, 0.0f);
}

B3_HD B3_INL void b3_joint_enable_limit(B3World* w, int joint, int enable) {
    B3Joint* j = &w->joints[joint];
    if (j->enable_limit != enable) {
        j->lower_impulse = 0.0f;
        j->upper_impulse = 0.0f;
    }
    j->enable_limit = enable;
}

B3_HD B3_INL void b3_joint_set_limits(B3World* w, int joint, float lower,
        float upper) {
    B3Joint* j = &w->joints[joint];
    float lo = b3_minf(lower, upper);
    float hi = b3_maxf(lower, upper);
    j->lower_angle = b3_clamp(lo, -0.99f * B3_PI, 0.99f * B3_PI);
    j->upper_angle = b3_clamp(hi, -0.99f * B3_PI, 0.99f * B3_PI);
}

B3_HD B3_INL void b3_joint_enable_spring(B3World* w, int joint, int enable) {
    B3Joint* j = &w->joints[joint];
    if (j->enable_spring != enable) {
        j->spring_impulse = 0.0f;
    }
    j->enable_spring = enable;
}

B3_HD B3_INL void b3_joint_set_spring(B3World* w, int joint, float target,
        float hertz, float damping) {
    B3Joint* j = &w->joints[joint];
    j->target_angle = b3_clamp(target, -B3_PI, B3_PI);
    j->hertz = b3_maxf(hertz, 0.0f);
    j->damping_ratio = b3_maxf(damping, 0.0f);
}

B3_HD B3_INL float b3_joint_angle(const B3World* w, int joint) {
    const B3Joint* j = &w->joints[joint];
    const B3Body* ba = &w->bodies[j->body_a];
    const B3Body* bb = &w->bodies[j->body_b];
    B3Quat qa = b3_qmul(ba->rotation, j->local_rot_a);
    B3Quat qb = b3_qmul(bb->rotation, j->local_rot_b);
    if (b3_qdot(qa, qb) < 0.0f) {
        qb = b3_qneg(qb);
    }
    return b3_twist(b3_qinv_mul(qa, qb));
}

B3_HD B3_INL float b3_joint_speed(const B3World* w, int joint) {
    const B3Joint* j = &w->joints[joint];
    const B3Body* ba = &w->bodies[j->body_a];
    const B3Body* bb = &w->bodies[j->body_b];
    B3Vec3 axis = b3_rotate(b3_qmul(ba->rotation, j->local_rot_a),
        b3_v(0.0f, 0.0f, 1.0f));
    return b3_dot(b3_sub(bb->ang_vel, ba->ang_vel), axis);
}

B3_HD B3_INL void b3_prepare_joints(B3World* w, float h) {
    for (int i = 0; i < w->joint_count; i++) {
        B3Joint* j = &w->joints[i];
        const B3Body* ba = &w->bodies[j->body_a];
        const B3Body* bb = &w->bodies[j->body_b];
        j->inv_mass_a = ba->type == B3_DYNAMIC ? ba->inv_mass : 0.0f;
        j->inv_mass_b = bb->type == B3_DYNAMIC ? bb->inv_mass : 0.0f;
        j->inv_i_a = ba->type == B3_DYNAMIC ? ba->inv_i_world : b3_mat0();
        j->inv_i_b = bb->type == B3_DYNAMIC ? bb->inv_i_world : b3_mat0();
        B3Mat3 isum = b3_maddm(j->inv_i_a, j->inv_i_b);
        j->fixed_rotation = b3_dot(isum.cx, b3_cross(isum.cy, isum.cz))
            < 1.0e-20f;
        j->softness = b3_make_soft(j->constraint_hertz,
            j->constraint_damping, h);
        j->frame_q_a = b3_qmul(ba->rotation, j->local_rot_a);
        j->frame_q_b = b3_qmul(bb->rotation, j->local_rot_b);
        j->frame_p_a = b3_rotate(ba->rotation,
            b3_sub(j->local_anchor_a, ba->local_center));
        j->frame_p_b = b3_rotate(bb->rotation,
            b3_sub(j->local_anchor_b, bb->local_center));
        j->delta_center = b3_sub(bb->center, ba->center);
#ifdef B3_REVOLUTE_ONLY
        {
#else
        if (j->type == B3_JOINT_WELD) {
            j->angular_mass = b3_invert3(isum);
            j->linear_spring = j->linear_hertz == 0.0f ? j->softness
                : b3_make_soft(j->linear_hertz, j->linear_damping, h);
            j->angular_spring = j->angular_hertz == 0.0f ? j->softness
                : b3_make_soft(j->angular_hertz, j->angular_damping, h);
        } else {
#endif
            B3Vec3 axis = b3_rotate(j->frame_q_a, b3_v(0.0f, 0.0f, 1.0f));
            float k = b3_dot(axis, b3_mv(isum, axis));
            j->axial_mass = k > 0.0f ? 1.0f / k : 0.0f;
            j->rotation_axis = axis;
            B3Quat rel = b3_qinv_mul(j->frame_q_a, j->frame_q_b);
            b3_hinge_perps(j->frame_q_a, rel, &j->perp_x, &j->perp_y);
            j->spring_softness = b3_make_soft(j->hertz, j->damping_ratio, h);
        }
    }
}

B3_HD B3_INL void b3_warm_start_joints(B3World* w) {
    for (int i = 0; i < w->joint_count; i++) {
        B3Joint* j = &w->joints[i];
        B3Body* ba = &w->bodies[j->body_a];
        B3Body* bb = &w->bodies[j->body_b];
        B3Vec3 ra = b3_rotate(ba->delta_rot, j->frame_p_a);
        B3Vec3 rb = b3_rotate(bb->delta_rot, j->frame_p_b);
#ifdef B3_REVOLUTE_ONLY
        B3Vec3 ang = b3_v(0.0f, 0.0f, 0.0f);
        {
            float axial = j->spring_impulse
                + j->lower_impulse - j->upper_impulse;
#else
        B3Vec3 ang = j->angular_impulse;
        if (j->type == B3_JOINT_REVOLUTE) {
            float axial = j->spring_impulse + j->motor_impulse
                + j->lower_impulse - j->upper_impulse;
#endif
            ang = b3_add(b3_mul(j->perp_x, j->perp_impulse.x),
                b3_mul(j->perp_y, j->perp_impulse.y));
            ang = b3_madd(ang, axial, j->rotation_axis);
        }
        if (ba->flags & B3_FLAG_DYNAMIC) {
            ba->lin_vel = b3_msub(ba->lin_vel, j->inv_mass_a,
                j->linear_impulse);
            ba->ang_vel = b3_sub(ba->ang_vel, b3_mv(j->inv_i_a,
                b3_add(b3_cross(ra, j->linear_impulse), ang)));
        }
        if (bb->flags & B3_FLAG_DYNAMIC) {
            bb->lin_vel = b3_madd(bb->lin_vel, j->inv_mass_b,
                j->linear_impulse);
            bb->ang_vel = b3_add(bb->ang_vel, b3_mv(j->inv_i_b,
                b3_add(b3_cross(rb, j->linear_impulse), ang)));
        }
    }
}

B3_HD B3_INL B3Mat3 b3_point_k_gs(float ma, float mb, B3Mat3 ia, B3Mat3 ib,
        B3Vec3 ra, B3Vec3 rb) {
    float msum = ma + mb;
    B3Mat3 k;
    k.cx = b3_v(msum, 0.0f, 0.0f);
    k.cy = b3_v(0.0f, msum, 0.0f);
    k.cz = b3_v(0.0f, 0.0f, msum);
    k.cx = b3_sub(k.cx, b3_cross(ra, b3_mv(ia, b3_v(0.0f, ra.z, -ra.y))));
    k.cy = b3_sub(k.cy, b3_cross(ra, b3_mv(ia, b3_v(-ra.z, 0.0f, ra.x))));
    k.cz = b3_sub(k.cz, b3_cross(ra, b3_mv(ia, b3_v(ra.y, -ra.x, 0.0f))));
    k.cx = b3_sub(k.cx, b3_cross(rb, b3_mv(ib, b3_v(0.0f, rb.z, -rb.y))));
    k.cy = b3_sub(k.cy, b3_cross(rb, b3_mv(ib, b3_v(-rb.z, 0.0f, rb.x))));
    k.cz = b3_sub(k.cz, b3_cross(rb, b3_mv(ib, b3_v(rb.y, -rb.x, 0.0f))));
    return k;
}

B3_HD B3_INL void b3_solve_point(B3Joint* j, B3Body* ba, B3Body* bb,
        B3Vec3* va, B3Vec3* wa, B3Vec3* vb, B3Vec3* wb, B3Soft soft,
        int use_bias) {
    B3Vec3 ra = b3_rotate(ba->delta_rot, j->frame_p_a);
    B3Vec3 rb = b3_rotate(bb->delta_rot, j->frame_p_b);
    B3Vec3 cdot = b3_sub(b3_add(*vb, b3_cross(*wb, rb)),
        b3_add(*va, b3_cross(*wa, ra)));
    B3Vec3 bias = b3_v(0.0f, 0.0f, 0.0f);
    float mscale = 1.0f;
    float iscale = 0.0f;
    if (use_bias) {
        B3Vec3 sep = b3_add(b3_add(b3_sub(bb->delta_pos, ba->delta_pos),
            b3_sub(rb, ra)), j->delta_center);
        bias = b3_mul(sep, soft.bias_rate);
        mscale = soft.mass_scale;
        iscale = soft.impulse_scale;
    }
    B3Vec3 b = b3_solve3(b3_point_k_gs(j->inv_mass_a, j->inv_mass_b,
        j->inv_i_a, j->inv_i_b, ra, rb), b3_add(cdot, bias));
    B3Vec3 impulse = b3_msub(b3_mul(b, -mscale), iscale, j->linear_impulse);
    j->linear_impulse = b3_add(j->linear_impulse, impulse);
    *va = b3_msub(*va, j->inv_mass_a, impulse);
    *wa = b3_sub(*wa, b3_mv(j->inv_i_a, b3_cross(ra, impulse)));
    *vb = b3_madd(*vb, j->inv_mass_b, impulse);
    *wb = b3_add(*wb, b3_mv(j->inv_i_b, b3_cross(rb, impulse)));
}

#ifndef B3_REVOLUTE_ONLY
B3_HD B3_INL void b3_solve_weld(B3Joint* j, B3Body* ba, B3Body* bb,
        int use_bias) {
    B3Vec3 va = ba->lin_vel;
    B3Vec3 wa = ba->ang_vel;
    B3Vec3 vb = bb->lin_vel;
    B3Vec3 wb = bb->ang_vel;
    B3Quat qa = b3_qmul(ba->delta_rot, j->frame_q_a);
    B3Quat qb = b3_qmul(bb->delta_rot, j->frame_q_b);
    if (b3_qdot(qa, qb) < 0.0f) {
        qb = b3_qneg(qb);
    }
    B3Quat rel = b3_qinv_mul(qa, qb);
    if (!j->fixed_rotation) {
        B3Vec3 bias = b3_v(0.0f, 0.0f, 0.0f);
        float mscale = 1.0f;
        float iscale = 0.0f;
        if (use_bias || j->angular_hertz > 0.0f) {
            B3Quat s = rel;
            if (rel.s < 0.0f) {
                s = b3_qneg(rel);
            }
            B3Quat diff = b3_q(-s.v.x, -s.v.y, -s.v.z, 1.0f - s.s);
            B3Vec3 c = b3_neg(b3_rotate(qa,
                b3_mul(b3_qmul(diff, b3_qconj(s)).v, 2.0f)));
            bias = b3_mul(c, j->angular_spring.bias_rate);
            mscale = j->angular_spring.mass_scale;
            iscale = j->angular_spring.impulse_scale;
        }
        B3Vec3 cdot = b3_sub(wb, wa);
        B3Vec3 impulse = b3_msub(
            b3_mul(b3_mv(j->angular_mass, b3_add(cdot, bias)), -mscale),
            iscale, j->angular_impulse);
        j->angular_impulse = b3_add(j->angular_impulse, impulse);
        wa = b3_sub(wa, b3_mv(j->inv_i_a, impulse));
        wb = b3_add(wb, b3_mv(j->inv_i_b, impulse));
    }
    int lin_bias = use_bias || j->linear_hertz > 0.0f;
    b3_solve_point(j, ba, bb, &va, &wa, &vb, &wb,
        j->linear_spring, lin_bias);
    if (ba->flags & B3_FLAG_DYNAMIC) {
        ba->lin_vel = va;
        ba->ang_vel = wa;
    }
    if (bb->flags & B3_FLAG_DYNAMIC) {
        bb->lin_vel = vb;
        bb->ang_vel = wb;
    }
}

#endif

#if defined(B3_CACHE_JOINTS) || defined(B3_PACKED_GS)
// Shared hinge-cache fill for b3_cache_revolute/_gs (ra/rb, invK, twist,
// perps, angK, axial invI*axis). Struct writes stay in the callers.
typedef struct B3HingeCache {
    B3Vec3 ra, rb;
    B3Mat3 point_invk;
    float rel_x, rel_y;
    float twist;
    B3Vec3 px, py;
    B3Mat2 ang_invk;
    B3Vec3 ia_ax, ib_ax;
} B3HingeCache;

B3_HD B3_INL B3HingeCache b3_hinge_cache_fill(B3Vec3 frame_p_a,
        B3Vec3 frame_p_b, B3Quat frame_q_a, B3Quat frame_q_b,
        B3Quat delta_rot_a, B3Quat delta_rot_b, float inv_mass_a,
        float inv_mass_b, B3Mat3 inv_i_a, B3Mat3 inv_i_b,
        B3Vec3 rotation_axis, int fixed) {
    B3HingeCache hc;
    hc.ra = b3_rotate(delta_rot_a, frame_p_a);
    hc.rb = b3_rotate(delta_rot_b, frame_p_b);
    hc.point_invk = b3_invert3(b3_point_k_gs(inv_mass_a, inv_mass_b,
        inv_i_a, inv_i_b, hc.ra, hc.rb));
    B3Quat qa = b3_qmul(delta_rot_a, frame_q_a);
    B3Quat qb = b3_qmul(delta_rot_b, frame_q_b);
    if (b3_qdot(qa, qb) < 0.0f) {
        qb = b3_qneg(qb);
    }
    B3Quat rel = b3_qinv_mul(qa, qb);
    hc.rel_x = rel.v.x;
    hc.rel_y = rel.v.y;
    hc.twist = b3_twist(rel);
    if (fixed) {
        hc.px = b3_v(0.0f, 0.0f, 0.0f);
        hc.py = b3_v(0.0f, 0.0f, 0.0f);
        hc.ang_invk.cx.x = 0.0f;
        hc.ang_invk.cx.y = 0.0f;
        hc.ang_invk.cy.x = 0.0f;
        hc.ang_invk.cy.y = 0.0f;
        hc.ia_ax = b3_v(0.0f, 0.0f, 0.0f);
        hc.ib_ax = b3_v(0.0f, 0.0f, 0.0f);
        return hc;
    }
    b3_hinge_perps(qa, rel, &hc.px, &hc.py);
    B3Mat3 isum = b3_maddm(inv_i_a, inv_i_b);
    B3Mat2 k;
    k.cx.x = b3_dot(hc.px, b3_mv(isum, hc.px));
    k.cy.y = b3_dot(hc.py, b3_mv(isum, hc.py));
    k.cx.y = k.cy.x = b3_dot(hc.px, b3_mv(isum, hc.py));
    hc.ang_invk = b3_invert2(k);
    hc.ia_ax = b3_mv(inv_i_a, rotation_axis);
    hc.ib_ax = b3_mv(inv_i_b, rotation_axis);
    return hc;
}
#endif

#ifdef B3_CACHE_JOINTS
B3_HD B3_INL void b3_cache_revolute(B3Joint* j, const B3Body* ba,
        const B3Body* bb) {
    B3HingeCache hc = b3_hinge_cache_fill(j->frame_p_a, j->frame_p_b,
        j->frame_q_a, j->frame_q_b, ba->delta_rot, bb->delta_rot,
        j->inv_mass_a, j->inv_mass_b, j->inv_i_a, j->inv_i_b,
        j->rotation_axis, j->fixed_rotation);
    j->cache_ra = hc.ra;
    j->cache_rb = hc.rb;
    j->cache_point_invk = hc.point_invk;
    j->cache_rel_x = hc.rel_x;
    j->cache_rel_y = hc.rel_y;
    j->cache_twist = hc.twist;
    if (!j->fixed_rotation) {
        j->perp_x = hc.px;
        j->perp_y = hc.py;
    }
    j->cache_ang_invk = hc.ang_invk;
    j->cache_ia_ax = hc.ia_ax;
    j->cache_ib_ax = hc.ib_ax;
}
#endif

#ifndef B3_ABLATE_NO_LIMIT
// One clamped hinge-limit row shared by b3_solve_revolute and
// b3_solve_axial_gs (lower/upper differ only in c, cdot and impulse slot).
// Returns the clamped accumulated impulse; the caller applies new minus old.
B3_HD B3_INL float b3_limit_impulse(float c, float cdot, float inv_h,
        int use_bias, B3Soft softness, float axial_mass, float old) {
    float bias = 0.0f;
    float mscale = 1.0f;
    float iscale = 0.0f;
    if (c > 0.0f) {
        bias = c * inv_h;
    } else if (use_bias) {
        bias = softness.bias_rate * c;
        mscale = softness.mass_scale;
        iscale = softness.impulse_scale;
    }
    float dimp = -mscale * axial_mass * (cdot + bias) - iscale * old;
    return b3_maxf(old + dimp, 0.0f);
}
#endif

B3_HD B3_INL void b3_solve_revolute(B3Joint* j, B3Body* ba, B3Body* bb,
        float h, float inv_h, int use_bias) {
    B3Vec3 va = ba->lin_vel;
    B3Vec3 wa = ba->ang_vel;
    B3Vec3 vb = bb->lin_vel;
    B3Vec3 wb = bb->ang_vel;
#ifdef B3_CACHE_JOINTS
    B3Vec3 axis = j->rotation_axis;
#ifdef B3_ABLATE_NO_IA_CACHE
    B3Vec3 ia_ax = b3_v(0.0f, 0.0f, 0.0f);
    B3Vec3 ib_ax = b3_v(0.0f, 0.0f, 0.0f);
    if (!j->fixed_rotation) {
        ia_ax = b3_mv(j->inv_i_a, axis);
        ib_ax = b3_mv(j->inv_i_b, axis);
    }
#else
    B3Vec3 ia_ax = j->cache_ia_ax;
    B3Vec3 ib_ax = j->cache_ib_ax;
#endif
    float twist = j->cache_twist;
#else
    B3Quat qa = b3_qmul(ba->delta_rot, j->frame_q_a);
    B3Quat qb = b3_qmul(bb->delta_rot, j->frame_q_b);
    if (b3_qdot(qa, qb) < 0.0f) {
        qb = b3_qneg(qb);
    }
    B3Quat rel = b3_qinv_mul(qa, qb);
    B3Vec3 axis = j->rotation_axis;
    B3Vec3 ia_ax = b3_v(0.0f, 0.0f, 0.0f);
    B3Vec3 ib_ax = b3_v(0.0f, 0.0f, 0.0f);
    float twist = 0.0f;
    int need_twist = (j->enable_spring || j->enable_limit
#ifndef B3_REVOLUTE_ONLY
            || j->enable_motor
#endif
            ) && !j->fixed_rotation;
    if (!j->fixed_rotation) {
        ia_ax = b3_mv(j->inv_i_a, axis);
        ib_ax = b3_mv(j->inv_i_b, axis);
    }
    if (need_twist) {
        twist = b3_twist(rel);
    }
#endif
#ifndef B3_ABLATE_NO_SPRING
    if (j->enable_spring && !j->fixed_rotation) {
        float c = twist - j->target_angle;
        float bias = j->spring_softness.bias_rate * c;
        float mscale = j->spring_softness.mass_scale;
        float iscale = j->spring_softness.impulse_scale;
        float cdot = b3_dot(b3_sub(wb, wa), axis);
        float dimp = -mscale * j->axial_mass * (cdot + bias)
            - iscale * j->spring_impulse;
        float old = j->spring_impulse;
        float nimp = old + dimp;
        if (j->max_motor_torque > 0.0f) {
            float maxp = j->max_motor_torque * h;
            nimp = b3_clamp(nimp, -maxp, maxp);
        }
        dimp = nimp - old;
        j->spring_impulse = nimp;
        wa = b3_msub(wa, dimp, ia_ax);
        wb = b3_madd(wb, dimp, ib_ax);
    }
#endif
#ifndef B3_REVOLUTE_ONLY
    if (j->enable_motor && !j->fixed_rotation) {
        int blocked = 0;
        if (j->enable_limit) {
            float ang = twist;
            if (ang >= j->upper_angle - 0.01f && j->motor_speed > 0.0f) {
                blocked = 1;
            }
            if (ang <= j->lower_angle + 0.01f && j->motor_speed < 0.0f) {
                blocked = 1;
            }
        }
        if (blocked) {
            j->motor_impulse = 0.0f;
        } else {
        float cdot = b3_dot(b3_sub(wb, wa), axis) - j->motor_speed;
        float dimp = -j->axial_mass * cdot;
        float nimp = j->motor_impulse + dimp;
        float maxp = j->max_motor_torque * h;
        nimp = b3_clamp(nimp, -maxp, maxp);
        dimp = nimp - j->motor_impulse;
        j->motor_impulse = nimp;
        wa = b3_msub(wa, dimp, ia_ax);
        wb = b3_madd(wb, dimp, ib_ax);
        }
    }
#endif
#ifndef B3_ABLATE_NO_LIMIT
    if (j->enable_limit && !j->fixed_rotation) {
        float angle = twist;
        {
            float old = j->lower_impulse;
            j->lower_impulse = b3_limit_impulse(angle - j->lower_angle,
                b3_dot(b3_sub(wb, wa), axis), inv_h, use_bias, j->softness,
                j->axial_mass, old);
            float dimp = j->lower_impulse - old;
            wa = b3_msub(wa, dimp, ia_ax);
            wb = b3_madd(wb, dimp, ib_ax);
        }
        {
            float old = j->upper_impulse;
            j->upper_impulse = b3_limit_impulse(j->upper_angle - angle,
                b3_dot(b3_sub(wa, wb), axis), inv_h, use_bias, j->softness,
                j->axial_mass, old);
            float dimp = j->upper_impulse - old;
            wa = b3_madd(wa, dimp, ia_ax);
            wb = b3_msub(wb, dimp, ib_ax);
        }
    }
#endif
#ifndef B3_ABLATE_NO_PERP
    if (!j->fixed_rotation) {
        B3Vec2 bias;
        bias.x = 0.0f;
        bias.y = 0.0f;
        float mscale = 1.0f;
        float iscale = 0.0f;
        if (use_bias) {
#ifdef B3_CACHE_JOINTS
            bias.x = j->softness.bias_rate * j->cache_rel_x;
            bias.y = j->softness.bias_rate * j->cache_rel_y;
#else
            bias.x = j->softness.bias_rate * rel.v.x;
            bias.y = j->softness.bias_rate * rel.v.y;
#endif
            mscale = j->softness.mass_scale;
            iscale = j->softness.impulse_scale;
        }
#ifdef B3_CACHE_JOINTS
        B3Vec3 px = j->perp_x;
        B3Vec3 py = j->perp_y;
        B3Mat2 kang = j->cache_ang_invk;
#else
        B3Vec3 px, py;
        b3_hinge_perps(qa, rel, &px, &py);
        j->perp_x = px;
        j->perp_y = py;
        B3Mat3 isum = b3_maddm(j->inv_i_a, j->inv_i_b);
        B3Mat2 kang;
        kang.cx.x = b3_dot(px, b3_mv(isum, px));
        kang.cy.y = b3_dot(py, b3_mv(isum, py));
        kang.cx.y = kang.cy.x = b3_dot(px, b3_mv(isum, py));
#endif
        B3Vec3 wrel = b3_sub(wb, wa);
        B3Vec2 rhs;
        rhs.x = b3_dot(wrel, px) + bias.x;
        rhs.y = b3_dot(wrel, py) + bias.y;
#ifdef B3_CACHE_JOINTS
        B3Vec2 sol = b3_mv2(kang, rhs);
#else
        B3Vec2 sol = b3_solve2(kang, rhs);
#endif
        B3Vec2 old = j->perp_impulse;
        B3Vec2 dimp;
        dimp.x = -mscale * sol.x - iscale * old.x;
        dimp.y = -mscale * sol.y - iscale * old.y;
        j->perp_impulse.x += dimp.x;
        j->perp_impulse.y += dimp.y;
        B3Vec3 ang = b3_add(b3_mul(px, dimp.x), b3_mul(py, dimp.y));
        wa = b3_sub(wa, b3_mv(j->inv_i_a, ang));
        wb = b3_add(wb, b3_mv(j->inv_i_b, ang));
    }
#endif
#ifndef B3_ABLATE_NO_POINT
#ifdef B3_CACHE_JOINTS
    {
        B3Vec3 ra = j->cache_ra;
        B3Vec3 rb = j->cache_rb;
        B3Vec3 cdot = b3_sub(b3_add(vb, b3_cross(wb, rb)),
            b3_add(va, b3_cross(wa, ra)));
        B3Vec3 bias = b3_v(0.0f, 0.0f, 0.0f);
        float mscale = 1.0f;
        float iscale = 0.0f;
        if (use_bias) {
            B3Vec3 sep = b3_add(b3_add(b3_sub(bb->delta_pos, ba->delta_pos),
                b3_sub(rb, ra)), j->delta_center);
            bias = b3_mul(sep, j->softness.bias_rate);
            mscale = j->softness.mass_scale;
            iscale = j->softness.impulse_scale;
        }
        B3Vec3 rhs = b3_mv(j->cache_point_invk, b3_add(cdot, bias));
        B3Vec3 impulse = b3_msub(b3_mul(rhs, -mscale), iscale,
            j->linear_impulse);
        j->linear_impulse = b3_add(j->linear_impulse, impulse);
        va = b3_msub(va, j->inv_mass_a, impulse);
        wa = b3_sub(wa, b3_mv(j->inv_i_a, b3_cross(ra, impulse)));
        vb = b3_madd(vb, j->inv_mass_b, impulse);
        wb = b3_add(wb, b3_mv(j->inv_i_b, b3_cross(rb, impulse)));
    }
#else
    b3_solve_point(j, ba, bb, &va, &wa, &vb, &wb, j->softness, use_bias);
#endif
#endif
    if (ba->flags & B3_FLAG_DYNAMIC) {
        ba->lin_vel = va;
        ba->ang_vel = wa;
    }
    if (bb->flags & B3_FLAG_DYNAMIC) {
        bb->lin_vel = vb;
        bb->ang_vel = wb;
    }
}
