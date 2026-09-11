// ==== puffysics 07_packed.inl: INTERNAL: packed-GS snapshots, interleaved solves, articulation hooks ====
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
B3_HD B3_INL void b3_prepare_one_contact(B3Contact* c, const B3Body* ba,
        const B3Body* bb, B3Soft contact_s, B3Soft static_s);
B3_HD B3_INL void b3_warm_one_contact(B3Contact* c, B3Body* ba, B3Body* bb);
B3_HD B3_INL void b3_solve_one_contact(B3Contact* c, B3Body* ba, B3Body* bb,
        float inv_h, float contact_speed, int use_bias);
B3_HD B3_INL void b3_solve_contacts_n(B3Contact* contacts, int n,
        B3Body* bodies, float inv_h, float contact_speed, int use_bias);

B3_HD B3_INL void b3_solve_contacts(B3World* w, float inv_h,
        float contact_speed, int use_bias);
B3_HD B3_INL void b3_integrate_position_state(const B3Body* b, float h,
        float max_lin, float max_ang, float max_lin2, float max_ang2,
        B3Vec3* lin_vel, B3Vec3* ang_vel, B3Vec3* delta_pos, B3Quat* delta_rot);
#if B3_ART_CONTACTS
#include "b3_art.cuh"
#endif

/* Rolling is a torque-only disk in the contact tangent plane. Solve the
 * projected 2x2 inertia so locked normal-axis rotation is harmless. */
B3_HD B3_INL void b3_solve_rolling(B3Vec3 t1, B3Vec3 t2, float rolling,
        float total_n, B3Mat3 ia, B3Mat3 ib, B3Vec3* impulse,
        B3Vec3* wa, B3Vec3* wb) {
    if (rolling <= 0.0f) return;
    B3Mat3 isum = b3_maddm(ia, ib);
    B3Mat2 k;
    k.cx.x = b3_dot(t1, b3_mv(isum, t1));
    k.cy.y = b3_dot(t2, b3_mv(isum, t2));
    k.cx.y = k.cy.x = b3_dot(t1, b3_mv(isum, t2));
    B3Vec3 wr = b3_sub(*wb, *wa);
    B3Vec2 rhs = {b3_dot(wr, t1), b3_dot(wr, t2)};
    B3Vec2 d = b3_mv2(b3_invert2(k), rhs);
    B3Vec3 next = b3_sub(*impulse, b3_add(b3_mul(t1, d.x), b3_mul(t2, d.y)));
    float limit = rolling * total_n;
    float l2 = b3_len2(next);
    if (l2 > limit * limit && l2 > 0.0f)
        next = b3_mul(next, b3_rsqrt_scale(limit, l2));
    B3Vec3 delta = b3_sub(next, *impulse);
    *impulse = next;
    *wa = b3_sub(*wa, b3_mv(ia, delta));
    *wb = b3_add(*wb, b3_mv(ib, delta));
}


B3_HD B3_INL void b3_solve_joints_global(B3World* w, float h, float inv_h,
        int use_bias) {
#ifdef B3_CACHE_JOINTS
    for (int i = 0; i < w->joint_count; i++) {
        B3Joint* j = &w->joints[i];
#ifndef B3_REVOLUTE_ONLY
        if (j->type == B3_JOINT_WELD) {
            continue;
        }
#endif
        b3_cache_revolute(j, &w->bodies[j->body_a], &w->bodies[j->body_b]);
    }
#endif
    int iters = use_bias ? B3_JOINT_ITERS : B3_RELAX_ITERS;
    for (int iter = 0; iter < iters; iter++) {
#if defined(B3_INTERLEAVE_CONTACTS) && !defined(B3_ABLATE_NO_CONTACT)
        b3_solve_contacts(w, inv_h, w->contact_speed, use_bias);
#endif
        for (int i = 0; i < w->joint_count; i++) {
            B3Joint* j = &w->joints[i];
            B3Body* ba = &w->bodies[j->body_a];
            B3Body* bb = &w->bodies[j->body_b];
#ifdef B3_REVOLUTE_ONLY
            b3_solve_revolute(j, ba, bb, h, inv_h, use_bias);
#else
            if (j->type == B3_JOINT_WELD) {
                b3_solve_weld(j, ba, bb, use_bias);
            } else {
                b3_solve_revolute(j, ba, bb, h, inv_h, use_bias);
            }
#endif
        }
    }
}

/* Snapshot joints/bodies/contacts into per-thread local memory for the
 * 8-iter GS sweep. Measured 1.7x vs global; does not prove DRAM vs ALU. */
B3_HD B3_INL void b3_solve_joints_local(B3World* w, float h, float inv_h,
        int use_bias) {
#if B3_ART_CONTACTS && defined(B3_INTERLEAVE_CONTACTS)
    // Delassus impulses can change every link; use the world-backed sweep.
    if (w->joint_count > 0 && w->contact_count > 0) {
        b3_solve_joints_global(w, h, inv_h, use_bias);
        return;
    }
#endif
    B3Body bl[B3_MAX_BODIES];
    B3Joint jl[B3_MAX_JOINTS];
    B3Contact cl[B3_MAX_CONTACTS];
    int nb = w->body_count;
    int nj = w->joint_count;
    int nc = w->contact_count;
    for (int i = 0; i < nb; i++) {
        bl[i] = w->bodies[i];
    }
    for (int i = 0; i < nj; i++) {
        jl[i] = w->joints[i];
    }
    for (int i = 0; i < nc; i++) {
        cl[i] = w->contacts[i];
    }
#ifdef B3_CACHE_JOINTS
    for (int i = 0; i < nj; i++) {
#ifndef B3_REVOLUTE_ONLY
        if (jl[i].type == B3_JOINT_WELD) {
            continue;
        }
#endif
        b3_cache_revolute(&jl[i], &bl[jl[i].body_a], &bl[jl[i].body_b]);
    }
#endif
    int iters = use_bias ? B3_JOINT_ITERS : B3_RELAX_ITERS;
    for (int iter = 0; iter < iters; iter++) {
#if defined(B3_INTERLEAVE_CONTACTS) && !defined(B3_ABLATE_NO_CONTACT)
        b3_solve_contacts_n(cl, nc, bl, inv_h, w->contact_speed, use_bias);
#endif
        for (int i = 0; i < nj; i++) {
#ifdef B3_REVOLUTE_ONLY
            b3_solve_revolute(&jl[i], &bl[jl[i].body_a], &bl[jl[i].body_b],
                h, inv_h, use_bias);
#else
            if (jl[i].type == B3_JOINT_WELD) {
                b3_solve_weld(&jl[i], &bl[jl[i].body_a],
                    &bl[jl[i].body_b], use_bias);
            } else {
                b3_solve_revolute(&jl[i], &bl[jl[i].body_a],
                    &bl[jl[i].body_b], h, inv_h, use_bias);
            }
#endif
        }
    }
    for (int i = 0; i < nb; i++) {
        w->bodies[i].lin_vel = bl[i].lin_vel;
        w->bodies[i].ang_vel = bl[i].ang_vel;
    }
    for (int i = 0; i < nj; i++) {
        w->joints[i] = jl[i];
    }
    for (int i = 0; i < nc; i++) {
        w->contacts[i] = cl[i];
    }
}

#ifdef B3_PACKED_GS
B3_HD B3_INL void b3_cache_revolute_gs(B3GsJoint* j, const B3Joint* src,
        const B3GsBody* ba, const B3GsBody* bb) {
    int fixed = (j->bits & B3_GS_FIXED) != 0;
    B3HingeCache hc = b3_hinge_cache_fill(src->frame_p_a, src->frame_p_b,
        src->frame_q_a, src->frame_q_b, ba->delta_rot, bb->delta_rot,
        ba->inv_mass, bb->inv_mass, ba->inv_i, bb->inv_i,
        j->rotation_axis, fixed);
    j->cache_ra = hc.ra;
    j->cache_rb = hc.rb;
    j->cache_point_invk = hc.point_invk;
    j->cache_rel_x = hc.rel_x;
    j->cache_rel_y = hc.rel_y;
    j->cache_twist = hc.twist;
    j->cache_ia_ax = hc.ia_ax;
    j->cache_ib_ax = hc.ib_ax;
    j->cache_ang_invk = hc.ang_invk;
    if (!fixed) {
        j->perp_x = hc.px;
        j->perp_y = hc.py;
#if B3_COUPLED_HINGE
        // Block effective mass [A B; B^T D] for anchor xyz + hinge alignment xy.
        // A^-1 already exists. Cache A^-1 B and (D - B^T A^-1 B)^-1.
        B3Vec3 ra = hc.ra;
        B3Vec3 rb = hc.rb;
        B3Vec3 px = hc.px;
        B3Vec3 py = hc.py;
        B3Mat3 isum = b3_maddm(ba->inv_i, bb->inv_i);
        B3Mat2 k;
        k.cx.x = b3_dot(px, b3_mv(isum, px));
        k.cy.y = b3_dot(py, b3_mv(isum, py));
        k.cx.y = k.cy.x = b3_dot(px, b3_mv(isum, py));
        B3Vec3 bx = b3_neg(b3_add(b3_cross(ra, b3_mv(ba->inv_i, px)),
            b3_cross(rb, b3_mv(bb->inv_i, px))));
        B3Vec3 by = b3_neg(b3_add(b3_cross(ra, b3_mv(ba->inv_i, py)),
            b3_cross(rb, b3_mv(bb->inv_i, py))));
        j->cache_point_perp_x = b3_mv(j->cache_point_invk, bx);
        j->cache_point_perp_y = b3_mv(j->cache_point_invk, by);
        k.cx.x -= b3_dot(bx, j->cache_point_perp_x);
        k.cy.y -= b3_dot(by, j->cache_point_perp_y);
        k.cx.y = k.cy.x = k.cx.y - b3_dot(bx, j->cache_point_perp_y);
        j->cache_ang_invk = b3_invert2(k);
#endif
    }
}

B3_HD B3_INL void b3_solve_axial_gs(B3GsJoint* j, B3GsBody* ba,
        B3GsBody* bb, float h, float inv_h, int use_bias) {
    B3Vec3 wa = ba->ang_vel;
    B3Vec3 wb = bb->ang_vel;
    B3Vec3 axis = j->rotation_axis;
    B3Vec3 ia_ax = j->cache_ia_ax;
    B3Vec3 ib_ax = j->cache_ib_ax;
    float twist = j->cache_twist;
    int fixed = j->bits & B3_GS_FIXED;
#ifndef B3_ABLATE_NO_SPRING
    if ((j->bits & B3_GS_SPRING) && !fixed) {
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
#ifndef B3_ABLATE_NO_LIMIT
    if ((j->bits & B3_GS_LIMIT) && !fixed) {
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
    if (ba->flags & B3_FLAG_DYNAMIC) ba->ang_vel = wa;
    if (bb->flags & B3_FLAG_DYNAMIC) bb->ang_vel = wb;
}

B3_HD B3_INL void b3_solve_revolute_gs(B3GsJoint* j, B3GsBody* ba,
        B3GsBody* bb, float h, float inv_h, int use_bias) {
    b3_solve_axial_gs(j, ba, bb, h, inv_h, use_bias);
    B3Vec3 va = ba->lin_vel, wa = ba->ang_vel;
    B3Vec3 vb = bb->lin_vel, wb = bb->ang_vel;
    int fixed = j->bits & B3_GS_FIXED;
#if B3_COUPLED_HINGE
    {
        B3Vec3 ra = j->cache_ra;
        B3Vec3 rb = j->cache_rb;
        B3Vec3 rhs_p = b3_sub(b3_add(vb, b3_cross(wb, rb)),
            b3_add(va, b3_cross(wa, ra)));
        float mscale = use_bias ? j->softness.mass_scale : 1.0f;
        float iscale = use_bias ? j->softness.impulse_scale : 0.0f;
        if (use_bias) {
            B3Vec3 sep = b3_add(b3_add(b3_sub(bb->delta_pos, ba->delta_pos),
                b3_sub(rb, ra)), j->delta_center);
            rhs_p = b3_madd(rhs_p, j->softness.bias_rate, sep);
        }
        B3Vec3 sol_p = b3_mv(j->cache_point_invk, rhs_p);
        B3Vec3 ang = b3_v(0, 0, 0);
        if (!fixed) {
            B3Vec3 wrel = b3_sub(wb, wa);
            B3Vec2 rhs_a;
            rhs_a.x = b3_dot(wrel, j->perp_x);
            rhs_a.y = b3_dot(wrel, j->perp_y);
            if (use_bias) {
                rhs_a.x += j->softness.bias_rate * j->cache_rel_x;
                rhs_a.y += j->softness.bias_rate * j->cache_rel_y;
            }
            rhs_a.x -= b3_dot(j->cache_point_perp_x, rhs_p);
            rhs_a.y -= b3_dot(j->cache_point_perp_y, rhs_p);
            B3Vec2 sol_a = b3_mv2(j->cache_ang_invk, rhs_a);
            sol_p = b3_sub(sol_p, b3_add(b3_mul(j->cache_point_perp_x, sol_a.x),
                b3_mul(j->cache_point_perp_y, sol_a.y)));
            B3Vec2 da;
            da.x = -mscale * sol_a.x - iscale * j->perp_impulse.x;
            da.y = -mscale * sol_a.y - iscale * j->perp_impulse.y;
            j->perp_impulse.x += da.x;
            j->perp_impulse.y += da.y;
            ang = b3_add(b3_mul(j->perp_x, da.x), b3_mul(j->perp_y, da.y));
        }
        B3Vec3 impulse = b3_msub(b3_mul(sol_p, -mscale), iscale, j->linear_impulse);
        j->linear_impulse = b3_add(j->linear_impulse, impulse);
        va = b3_msub(va, ba->inv_mass, impulse);
        wa = b3_sub(wa, b3_mv(ba->inv_i, b3_add(b3_cross(ra, impulse), ang)));
        vb = b3_madd(vb, bb->inv_mass, impulse);
        wb = b3_add(wb, b3_mv(bb->inv_i, b3_add(b3_cross(rb, impulse), ang)));
    }
#else
#ifndef B3_ABLATE_NO_PERP
    if (!fixed) {
        B3Vec2 bias;
        bias.x = 0.0f;
        bias.y = 0.0f;
        float mscale = 1.0f;
        float iscale = 0.0f;
        if (use_bias) {
            bias.x = j->softness.bias_rate * j->cache_rel_x;
            bias.y = j->softness.bias_rate * j->cache_rel_y;
            mscale = j->softness.mass_scale;
            iscale = j->softness.impulse_scale;
        }
        B3Vec3 px = j->perp_x;
        B3Vec3 py = j->perp_y;
        B3Vec3 wrel = b3_sub(wb, wa);
        B3Vec2 rhs;
        rhs.x = b3_dot(wrel, px) + bias.x;
        rhs.y = b3_dot(wrel, py) + bias.y;
        B3Vec2 sol = b3_mv2(j->cache_ang_invk, rhs);
        B3Vec2 old = j->perp_impulse;
        B3Vec2 dimp;
        dimp.x = -mscale * sol.x - iscale * old.x;
        dimp.y = -mscale * sol.y - iscale * old.y;
        j->perp_impulse.x += dimp.x;
        j->perp_impulse.y += dimp.y;
        B3Vec3 ang = b3_add(b3_mul(px, dimp.x), b3_mul(py, dimp.y));
        wa = b3_sub(wa, b3_mv(ba->inv_i, ang));
        wb = b3_add(wb, b3_mv(bb->inv_i, ang));
    }
#endif
#ifndef B3_ABLATE_NO_POINT
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
        va = b3_msub(va, ba->inv_mass, impulse);
        wa = b3_sub(wa, b3_mv(ba->inv_i, b3_cross(ra, impulse)));
        vb = b3_madd(vb, bb->inv_mass, impulse);
        wb = b3_add(wb, b3_mv(bb->inv_i, b3_cross(rb, impulse)));
    }
#endif
#endif // B3_COUPLED_HINGE
    if (ba->flags & B3_FLAG_DYNAMIC) {
        ba->lin_vel = va;
        ba->ang_vel = wa;
    }
    if (bb->flags & B3_FLAG_DYNAMIC) {
        bb->lin_vel = vb;
        bb->ang_vel = wb;
    }
}

B3_HD B3_INL void b3_solve_contacts_gs(B3GsContact* contacts, int n,
        B3GsBody* bodies, float inv_h, float contact_speed, int use_bias) {
    for (int i = 0; i < n; i++) {
        B3GsContact* c = &contacts[i];
        B3GsBody* ba = &bodies[c->body_a];
        B3GsBody* bb = &bodies[c->body_b];
        B3Vec3 va = ba->lin_vel;
        B3Vec3 wa = ba->ang_vel;
        B3Vec3 vb = bb->lin_vel;
        B3Vec3 wb = bb->ang_vel;
        B3Quat dqa = ba->delta_rot;
        B3Quat dqb = bb->delta_rot;
        B3Vec3 dp = b3_sub(bb->delta_pos, ba->delta_pos);
        B3Vec3 nrm = c->normal;
        float total_n = 0.0f;
        float twist_lim = 0.0f;
        for (int p = 0; p < c->point_count; p++) {
            B3GsPoint* cp = &c->points[p];
            B3Vec3 ra = cp->r_a;
            B3Vec3 rb = cp->r_b;
            B3Vec3 ds = b3_add(dp, b3_sub(b3_rotate(dqb, rb),
                b3_rotate(dqa, ra)));
            float sep = b3_dot(ds, nrm) + cp->base_sep;
            float vbias = 0.0f;
            float mscale = 1.0f;
            float iscale = 0.0f;
            if (sep > 0.0f) {
                vbias = sep * inv_h;
            } else if (use_bias) {
                vbias = b3_maxf(c->softness.mass_scale
                    * c->softness.bias_rate * sep, -contact_speed);
                mscale = c->softness.mass_scale;
                iscale = c->softness.impulse_scale;
            }
            B3Vec3 vra = b3_add(va, b3_cross(wa, ra));
            B3Vec3 vrb = b3_add(vb, b3_cross(wb, rb));
            float vn = b3_dot(b3_sub(vrb, vra), nrm);
            float dimp = -cp->normal_mass * (mscale * vn + vbias)
                - iscale * cp->normal_impulse;
            float nimp = b3_maxf(cp->normal_impulse + dimp, 0.0f);
            dimp = nimp - cp->normal_impulse;
            cp->normal_impulse = nimp;
            cp->total_normal += nimp;
            total_n += nimp;
            twist_lim += cp->lever * cp->normal_impulse;
            B3Vec3 P = b3_mul(nrm, dimp);
            va = b3_msub(va, ba->inv_mass, P);
            wa = b3_sub(wa, b3_mv(ba->inv_i, b3_cross(ra, P)));
            vb = b3_madd(vb, bb->inv_mass, P);
            wb = b3_add(wb, b3_mv(bb->inv_i, b3_cross(rb, P)));
        }
        if (!use_bias) {
            float twist_s = b3_dot(nrm, b3_sub(wb, wa));
            float max_t = c->friction * twist_lim;
            float dtw = -c->twist_mass * twist_s;
            float old_t = c->twist_impulse;
            c->twist_impulse = b3_clamp(old_t + dtw, -max_t, max_t);
            dtw = c->twist_impulse - old_t;
            wa = b3_sub(wa, b3_mv(ba->inv_i, b3_mul(nrm, dtw)));
            wb = b3_add(wb, b3_mv(bb->inv_i, b3_mul(nrm, dtw)));

            B3Vec3 t1 = c->tangent1;
            B3Vec3 t2 = c->tangent2;
            B3Vec3 ra = c->center_a;
            B3Vec3 rb = c->center_b;
            B3Vec3 vra = b3_add(va, b3_cross(wa, ra));
            B3Vec3 vrb = b3_add(vb, b3_cross(wb, rb));
            B3Vec3 vr = b3_sub(vrb, vra);
            B3Vec2 vt;
            vt.x = b3_dot(vr, t1);
            vt.y = b3_dot(vr, t2);
            B3Vec2 tm = b3_mv2(c->tangent_mass, vt);
            B3Vec2 ni;
            ni.x = c->friction_impulse.x - tm.x;
            ni.y = c->friction_impulse.y - tm.y;
            float max_f = c->friction * total_n;
            float fl2 = ni.x * ni.x + ni.y * ni.y;
            if (fl2 > max_f * max_f && fl2 > 0.0f) {
                float sc = b3_rsqrt_scale(max_f, fl2);
                ni.x *= sc;
                ni.y *= sc;
            }
            B3Vec2 df;
            df.x = ni.x - c->friction_impulse.x;
            df.y = ni.y - c->friction_impulse.y;
            c->friction_impulse = ni;
            B3Vec3 P = b3_add(b3_mul(t1, df.x), b3_mul(t2, df.y));
            va = b3_msub(va, ba->inv_mass, P);
            wa = b3_sub(wa, b3_mv(ba->inv_i, b3_cross(ra, P)));
            vb = b3_madd(vb, bb->inv_mass, P);
            wb = b3_add(wb, b3_mv(bb->inv_i, b3_cross(rb, P)));
            b3_solve_rolling(c->tangent1, c->tangent2, c->rolling, total_n,
                ba->inv_i, bb->inv_i, &c->rolling_impulse, &wa, &wb);
        }
        if (ba->flags & B3_FLAG_DYNAMIC) {
            ba->lin_vel = va;
            ba->ang_vel = wa;
        }
        if (bb->flags & B3_FLAG_DYNAMIC) {
            bb->lin_vel = vb;
            bb->ang_vel = wb;
        }
    }
}

#if B3_ART_CONTACTS
B3_HD B3_INL void b3_solve_contacts_gs_w(B3World* w, B3Art* art,
        B3GsContact* contacts, int n, B3GsBody* bodies, float inv_h,
        float contact_speed, int use_bias) {
    if (n == 0) return;
    if (art) {
        b3_art_solve_contacts_gs(art, w, contacts, n, bodies,
            inv_h, contact_speed, use_bias, 1);
        return;
    }
    if (w->joint_count > 0) {
        B3Art local;
        if (b3_art_bind(&local, w)) {
            b3_art_solve_contacts_gs(&local, w, contacts, n, bodies,
                inv_h, contact_speed, use_bias, 1);
            return;
        }
    }
    b3_solve_contacts_gs(contacts, n, bodies, inv_h, contact_speed, use_bias);
}
#endif

B3_HD B3_INL void b3_gs_load(const B3World* w, B3GsBody* bl,
        B3GsJoint* jl, B3GsContact* cl) {
    int nb = w->body_count;
    int nj = w->joint_count;
    int nc = w->contact_count;
    for (int i = 0; i < nb; i++) {
        const B3Body* b = &w->bodies[i];
        bl[i].lin_vel = b->lin_vel;
        bl[i].ang_vel = b->ang_vel;
        bl[i].delta_pos = b->delta_pos;
        bl[i].delta_rot = b->delta_rot;
        bl[i].inv_mass = b->type == B3_DYNAMIC ? b->inv_mass : 0.0f;
        bl[i].inv_i = b->type == B3_DYNAMIC ? b->inv_i_world : b3_mat0();
        bl[i].flags = b->flags;
    }
    for (int i = 0; i < nj; i++) {
        const B3Joint* src = &w->joints[i];
        B3GsJoint* j = &jl[i];
        j->body_a = src->body_a;
        j->body_b = src->body_b;
        j->bits = 0;
        if (src->fixed_rotation) {
            j->bits |= B3_GS_FIXED;
        }
        if (src->enable_spring) {
            j->bits |= B3_GS_SPRING;
        }
        if (src->enable_limit) {
            j->bits |= B3_GS_LIMIT;
        }
        j->target_angle = src->target_angle;
        j->lower_angle = src->lower_angle;
        j->upper_angle = src->upper_angle;
        j->axial_mass = src->axial_mass;
        j->max_motor_torque = src->max_motor_torque;
        j->spring_impulse = src->spring_impulse;
        j->lower_impulse = src->lower_impulse;
        j->upper_impulse = src->upper_impulse;
        j->softness = src->softness;
        j->spring_softness = src->spring_softness;
        j->rotation_axis = src->rotation_axis;
        j->delta_center = src->delta_center;
        j->linear_impulse = src->linear_impulse;
        j->perp_impulse = src->perp_impulse;
    }
    for (int i = 0; i < nc; i++) {
        const B3Contact* src = &w->contacts[i];
        B3GsContact* c = &cl[i];
        c->body_a = src->body_a;
        c->body_b = src->body_b;
        c->point_count = src->point_count;
        c->normal = src->normal;
        c->tangent1 = src->tangent1;
        c->tangent2 = src->tangent2;
        c->center_a = src->center_a;
        c->center_b = src->center_b;
        c->friction = src->friction;
        c->rolling = src->rolling;
        c->rolling_impulse = src->rolling_impulse;
        c->twist_mass = src->twist_mass;
        c->twist_impulse = src->twist_impulse;
        c->friction_impulse = src->friction_impulse;
        c->tangent_mass = src->tangent_mass;
        c->softness = src->softness;
        for (int p = 0; p < src->point_count; p++) {
            c->points[p].r_a = src->points[p].r_a;
            c->points[p].r_b = src->points[p].r_b;
            c->points[p].base_sep = src->points[p].base_sep;
            c->points[p].normal_impulse = src->points[p].normal_impulse;
            c->points[p].total_normal = src->points[p].total_normal;
            c->points[p].normal_mass = src->points[p].normal_mass;
            c->points[p].lever = src->points[p].lever;
        }
    }
}

// Only velocities and delta transforms change during the substeps. Mass,
// inertia, flags, prepared joint data and contact geometry stay fixed.
B3_HD B3_INL void b3_gs_refresh_bodies(const B3World* w, B3GsBody* bl) {
    for (int i = 0; i < w->body_count; i++) {
        bl[i].lin_vel = w->bodies[i].lin_vel;
        bl[i].ang_vel = w->bodies[i].ang_vel;
        bl[i].delta_pos = w->bodies[i].delta_pos;
        bl[i].delta_rot = w->bodies[i].delta_rot;
    }
}

B3_HD B3_INL void b3_gs_solve_cached(const B3World* w, B3GsBody* bl,
        B3GsJoint* jl, B3GsContact* cl, float h, float inv_h, int use_bias, int refresh) {
    int nj = w->joint_count;
    int nc = w->contact_count;
    // Only position integration invalidates these pose-dependent caches.
    if (refresh) for (int i = 0; i < nj; i++) {
        b3_cache_revolute_gs(&jl[i], &w->joints[i],
            &bl[jl[i].body_a], &bl[jl[i].body_b]);
    }
#if B3_ART_CONTACTS && defined(B3_INTERLEAVE_CONTACTS) && !defined(B3_ABLATE_NO_CONTACT)
    B3Art art;
    B3Art* artp = nc > 0 && b3_art_bind(&art, w) ? &art : 0;
#endif
    int iters = use_bias ? B3_JOINT_ITERS : B3_RELAX_ITERS;
    for (int iter = 0; iter < iters; iter++) {
#if defined(B3_INTERLEAVE_CONTACTS) && !defined(B3_ABLATE_NO_CONTACT)
#if B3_ART_CONTACTS
        b3_solve_contacts_gs_w((B3World*)w, artp, cl, nc, bl,
            inv_h, w->contact_speed, use_bias);
#else
        b3_solve_contacts_gs(cl, nc, bl, inv_h, w->contact_speed, use_bias);
#endif
#endif
        for (int k = 0; k < nj; k++) {
#if B3_ALTERNATE_JOINT_ORDER
            int i = (iter & 1) ? nj - 1 - k : k;
#else
            int i = k;
#endif
            b3_solve_revolute_gs(&jl[i], &bl[jl[i].body_a],
                &bl[jl[i].body_b], h, inv_h, use_bias);
        }
    }
}

B3_HD B3_INL void b3_gs_solve(const B3World* w, B3GsBody* bl,
        B3GsJoint* jl, B3GsContact* cl, float h, float inv_h, int use_bias) {
    b3_gs_solve_cached(w, bl, jl, cl, h, inv_h, use_bias, 1);
}

B3_HD B3_INL void b3_gs_store_velocities(B3World* w, const B3GsBody* bl) {
    for (int i = 0; i < w->body_count; i++) {
        w->bodies[i].lin_vel = bl[i].lin_vel;
        w->bodies[i].ang_vel = bl[i].ang_vel;
    }
}

B3_HD B3_INL void b3_gs_store_constraints(B3World* w, const B3GsJoint* jl,
        const B3GsContact* cl) {
    for (int i = 0; i < w->joint_count; i++) {
        B3Joint* dst = &w->joints[i];
        const B3GsJoint* j = &jl[i];
        dst->spring_impulse = j->spring_impulse;
        dst->lower_impulse = j->lower_impulse;
        dst->upper_impulse = j->upper_impulse;
        dst->linear_impulse = j->linear_impulse;
        dst->perp_impulse = j->perp_impulse;
    }
    for (int i = 0; i < w->contact_count; i++) {
        B3Contact* dst = &w->contacts[i];
        const B3GsContact* c = &cl[i];
        dst->twist_impulse = c->twist_impulse;
        dst->friction_impulse = c->friction_impulse;
        dst->rolling_impulse = c->rolling_impulse;
        for (int p = 0; p < dst->point_count; p++) {
            dst->points[p].normal_impulse = c->points[p].normal_impulse;
            dst->points[p].total_normal = c->points[p].total_normal;
        }
    }
}

B3_HD B3_INL void b3_solve_joints_packed(B3World* w, float h, float inv_h,
        int use_bias) {
    B3GsBody bl[B3_MAX_BODIES];
    B3GsJoint jl[B3_MAX_JOINTS];
    B3GsContact cl[B3_MAX_CONTACTS];
    b3_gs_load(w, bl, jl, cl);
    b3_gs_solve(w, bl, jl, cl, h, inv_h, use_bias);
    b3_gs_store_velocities(w, bl);
    b3_gs_store_constraints(w, jl, cl);
}
#endif


B3_HD B3_INL void b3_solve_joints(B3World* w, float h, float inv_h,
        int use_bias) {
#ifdef B3_PACKED_GS
    b3_solve_joints_packed(w, h, inv_h, use_bias);
#elif defined(B3_COMPACT_GS)
    b3_solve_joints_local(w, h, inv_h, use_bias);
#else
    b3_solve_joints_global(w, h, inv_h, use_bias);
#endif
}
