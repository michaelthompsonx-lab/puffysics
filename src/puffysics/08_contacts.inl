// ==== puffysics 08_contacts.inl: INTERNAL: contact generation, warm start, contact sweeps, restitution ====
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
B3_HD B3_INL void b3_contact_from_mani(B3Contact* c, int si, int sj,
        const B3Shape* sa, const B3Shape* sb, const B3Body* ba,
        const B3Body* bb, const B3Mani* mani) {
    c->shape_a = si;
    c->shape_b = sj;
    c->body_a = sa->body;
    c->body_b = sb->body;
    c->point_count = mani->count;
    c->normal = mani->normal;
    c->tangent1 = b3_perp(mani->normal);
    c->tangent2 = b3_cross(c->tangent1, mani->normal);
    c->friction = sqrtf(sa->friction * sb->friction);
    c->restitution = sa->restitution > sb->restitution ? sa->restitution : sb->restitution;
    c->rolling = b3_maxf(sa->rolling, sb->rolling);
    c->static_contact = ba->type != B3_DYNAMIC || bb->type != B3_DYNAMIC;
    B3Vec3 ca = ba->center;
    B3Vec3 cb = bb->center;
    c->friction_impulse.x = 0.0f;
    c->friction_impulse.y = 0.0f;
    c->twist_impulse = 0.0f;
    c->rolling_impulse = b3_v(0.0f, 0.0f, 0.0f);
    for (int p = 0; p < mani->count; p++) {
        c->points[p].r_a = b3_sub(mani->p_a[p], ca);
        c->points[p].r_b = b3_sub(mani->p_b[p], cb);
        c->points[p].base_sep = mani->sep[p]
            - b3_dot(b3_sub(c->points[p].r_b, c->points[p].r_a),
                mani->normal);
        c->points[p].feature = mani->feature[p];
        c->points[p].normal_impulse = 0.0f;
        c->points[p].total_normal = 0.0f;
        c->points[p].rel_vel = 0.0f;
        c->points[p].normal_mass = 0.0f;
        c->points[p].lever = 0.0f;
    }
}

/* All-pairs AABB + filter predicate. Isolated so a later sweep/hash can
 * replace this without touching narrowphase. Returns 1 if the pair should
 * run contact generation. Default: O(n^2) is cheaper than a tree on the
 * intended robot/RL world sizes. */
B3_HD B3_INL int b3_broadphase_pair(const B3World* w, int i, int j,
        const B3AABB* aabb,
        const uint64_t connected[][B3_CONNECT_WORDS]) {
    const B3Shape* sa = &w->shapes[i];
    const B3Shape* sb = &w->shapes[j];
    const B3Body* ba = &w->bodies[sa->body];
    const B3Body* bb = &w->bodies[sb->body];
    if (sa->body == sb->body
            || (connected[sa->body][sb->body >> 6]
                & (1ull << (sb->body & 63))) != 0
            || (ba->type == B3_STATIC && bb->type == B3_STATIC)
            || (sa->category & sb->mask) == 0
            || (sb->category & sa->mask) == 0
            || !b3_aabb_overlap(aabb[i], aabb[j])) {
        return 0;
    }
    return 1;
}

B3_HD B3_INL void b3_find_contacts(B3World* w) {
    B3Warm old[B3_MAX_CONTACTS];
    B3AABB aabb[B3_MAX_SHAPES];
    int old_n = w->contact_count;
#if B3_MERGE_WARM_CACHE
    int old_sorted = 1;
#endif
    for (int i = 0; i < old_n; i++) {
        const B3Contact* src = &w->contacts[i];
#if B3_MERGE_WARM_CACHE
        if (i > 0 && (old[i - 1].shape_a > src->shape_a
                || (old[i - 1].shape_a == src->shape_a
                    && old[i - 1].shape_b >= src->shape_b))) {
            old_sorted = 0;
        }
#endif
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
    w->contact_count = 0;
#if B3_MERGE_WARM_CACHE
    int old_cursor = 0;
#endif
    uint64_t connected[B3_MAX_BODIES][B3_CONNECT_WORDS];
    for (int i = 0; i < w->body_count; i++) {
        for (int wdi = 0; wdi < B3_CONNECT_WORDS; wdi++) {
            connected[i][wdi] = 0;
        }
    }
    for (int i = 0; i < w->joint_count; i++) {
        const B3Joint* jn = &w->joints[i];
        if (jn->collide_connected) {
            continue;
        }
        connected[jn->body_a][jn->body_b >> 6] |= (1ull << (jn->body_b & 63));
        connected[jn->body_b][jn->body_a >> 6] |= (1ull << (jn->body_a & 63));
    }
    B3Vec3 pad = b3_v(B3_SPECULATIVE, B3_SPECULATIVE, B3_SPECULATIVE);
    for (int i = 0; i < w->shape_count; i++) {
        aabb[i] = b3_shape_aabb(&w->bodies[w->shapes[i].body], &w->shapes[i]);
        aabb[i].lo = b3_sub(aabb[i].lo, pad);
        aabb[i].hi = b3_add(aabb[i].hi, pad);
    }
    for (int i = 0; i < w->shape_count; i++) {
        B3Shape* sa = &w->shapes[i];
        B3Body* ba = &w->bodies[sa->body];
        for (int j = i + 1; j < w->shape_count; j++) {
            B3Shape* sb = &w->shapes[j];
            B3Body* bb = &w->bodies[sb->body];
            if (!b3_broadphase_pair(w, i, j, aabb, connected)) {
                continue;
            }
            B3Mani mani;
            b3_collide_shapes(&mani, w, sa, sb);
            if (mani.count == 0) {
                continue;
            }
            if (w->contact_count >= B3_MAX_CONTACTS) {
                w->contact_overflow = 1;
                w->contacts_dropped++;
                continue;
            }
            B3Contact* c = &w->contacts[w->contact_count++];
            b3_contact_from_mani(c, i, j, sa, sb, ba, bb, &mani);
#if B3_MERGE_WARM_CACHE
            // Both generated streams are ordered by (shape_a, shape_b).
            // Merge them in O(old contacts + new contacts), rather than
            // scanning the entire old cache for every new contact. Retain
            // the original scan if a caller reordered/duplicated contacts.
            int first = 0;
            int end = old_n;
            if (old_sorted) {
                while (old_cursor < old_n && (old[old_cursor].shape_a < i
                        || (old[old_cursor].shape_a == i
                            && old[old_cursor].shape_b < j))) {
                    old_cursor++;
                }
                first = old_cursor;
                end = old_cursor < old_n ? old_cursor + 1 : old_n;
            }
            for (int k = first; k < end; k++) {
#else
            for (int k = 0; k < old_n; k++) {
#endif
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

B3_HD B3_INL void b3_prepare_contacts(B3World* w, B3Soft contact_s,
        B3Soft static_s) {
    for (int i = 0; i < w->contact_count; i++) {
        B3Contact* c = &w->contacts[i];
        b3_prepare_one_contact(c, &w->bodies[c->body_a],
            &w->bodies[c->body_b], contact_s, static_s);
    }
}

B3_HD B3_INL void b3_apply_impulse(B3Body* b, float m, B3Mat3 i,
        B3Vec3 r, B3Vec3 p, int sign) {
    if ((b->flags & B3_FLAG_DYNAMIC) == 0) {
        return;
    }
    float s = sign > 0 ? 1.0f : -1.0f;
    b->lin_vel = b3_madd(b->lin_vel, s * m, p);
    b->ang_vel = b3_add(b->ang_vel, b3_mv(i, b3_mul(b3_cross(r, p), s)));
}

B3_HD B3_INL void b3_warm_start(B3World* w) {
#if B3_ART_CONTACTS
    if (w->joint_count > 0 && w->contact_count > 0) {
        B3Art art;
        if (b3_art_bind(&art, w)) {
            for (int i = 0; i < w->contact_count; i++) {
                B3Contact* c = &w->contacts[i];
                B3ArtRow row;
                for (int p = 0; p < c->point_count; p++) {
                    B3Point* cp = &c->points[p];
                    if (b3_art_make_row(&art, c->body_a, c->body_b,
                            cp->r_a, cp->r_b, c->normal, &row))
                        b3_art_apply_impulse(&art, w, &row, cp->normal_impulse);
                }
                B3Vec3 t[2] = {c->tangent1, c->tangent2};
                float f[2] = {c->friction_impulse.x, c->friction_impulse.y};
                for (int k = 0; k < 2; k++) {
                    if (b3_art_make_row(&art, c->body_a, c->body_b,
                            c->center_a, c->center_b, t[k], &row))
                        b3_art_apply_impulse(&art, w, &row, f[k]);
                    row.torque = 1;
                    b3_art_apply_impulse(&art, w, &row,
                        b3_dot(c->rolling_impulse, t[k]));
                }
                if (b3_art_make_row(&art, c->body_a, c->body_b,
                        b3_v(0, 0, 0), b3_v(0, 0, 0), c->normal, &row)) {
                    row.torque = 1;
                    b3_art_apply_impulse(&art, w, &row, c->twist_impulse);
                }
            }
            return;
        }
    }
#endif
    for (int i = 0; i < w->contact_count; i++) {
        B3Contact* c = &w->contacts[i];
        b3_warm_one_contact(c, &w->bodies[c->body_a], &w->bodies[c->body_b]);
    }
}

B3_HD B3_INL void b3_prepare_one_contact(B3Contact* c, const B3Body* ba,
        const B3Body* bb, B3Soft contact_s, B3Soft static_s) {
    float ma = ba->type == B3_DYNAMIC ? ba->inv_mass : 0.0f;
    float mb = bb->type == B3_DYNAMIC ? bb->inv_mass : 0.0f;
    B3Mat3 ia = ba->type == B3_DYNAMIC ? ba->inv_i_world : b3_mat0();
    B3Mat3 ib = bb->type == B3_DYNAMIC ? bb->inv_i_world : b3_mat0();
    c->inv_mass_a = ma;
    c->inv_mass_b = mb;
    c->inv_i_a = ia;
    c->inv_i_b = ib;
    c->rolling_mass = b3_invert3(b3_maddm(ia, ib));
    c->softness = c->static_contact ? static_s : contact_s;
    B3Vec3 n = c->normal;
    B3Vec3 center_a = b3_v(0.0f, 0.0f, 0.0f);
    B3Vec3 center_b = b3_v(0.0f, 0.0f, 0.0f);
    float wsum = 0.0f;
    float inv_tau = 1.0f / B3_SPECULATIVE;
    for (int p = 0; p < c->point_count; p++) {
        B3Point* cp = &c->points[p];
        B3Vec3 rn_a = b3_cross(cp->r_a, n);
        B3Vec3 rn_b = b3_cross(cp->r_b, n);
        float kn = ma + mb + b3_dot(rn_a, b3_mv(ia, rn_a))
            + b3_dot(rn_b, b3_mv(ib, rn_b));
        cp->normal_mass = kn > 0.0f ? 1.0f / kn : 0.0f;
        B3Vec3 vra = b3_add(ba->lin_vel, b3_cross(ba->ang_vel, cp->r_a));
        B3Vec3 vrb = b3_add(bb->lin_vel, b3_cross(bb->ang_vel, cp->r_b));
        cp->rel_vel = b3_dot(n, b3_sub(vrb, vra));
        cp->total_normal = 0.0f;
        float sep = cp->base_sep
            + b3_dot(b3_sub(cp->r_b, cp->r_a), n);
        float weight = b3_clamp(2.0f - sep * inv_tau,
            B3_MIN_FRICTION_W, 1.0f);
        center_a = b3_madd(center_a, weight, cp->r_a);
        center_b = b3_madd(center_b, weight, cp->r_b);
        wsum += weight;
    }
    float invw = wsum > 0.0f ? 1.0f / wsum : 0.0f;
    c->center_a = b3_mul(center_a, invw);
    c->center_b = b3_mul(center_b, invw);
    for (int p = 0; p < c->point_count; p++) {
        c->points[p].lever = b3_len(
            b3_sub(c->points[p].r_a, c->center_a));
    }
    B3Vec3 rt_a1 = b3_cross(c->center_a, c->tangent1);
    B3Vec3 rt_a2 = b3_cross(c->center_a, c->tangent2);
    B3Vec3 rt_b1 = b3_cross(c->center_b, c->tangent1);
    B3Vec3 rt_b2 = b3_cross(c->center_b, c->tangent2);
    B3Mat2 k;
    k.cx.x = ma + mb + b3_dot(rt_a1, b3_mv(ia, rt_a1))
        + b3_dot(rt_b1, b3_mv(ib, rt_b1));
    k.cy.y = ma + mb + b3_dot(rt_a2, b3_mv(ia, rt_a2))
        + b3_dot(rt_b2, b3_mv(ib, rt_b2));
    k.cx.y = k.cy.x = b3_dot(rt_a1, b3_mv(ia, rt_a2))
        + b3_dot(rt_b1, b3_mv(ib, rt_b2));
    c->tangent_mass = b3_invert2(k);
    float kt = b3_dot(n, b3_mv(b3_maddm(ia, ib), n));
    c->twist_mass = kt > 0.0f ? 1.0f / kt : 0.0f;
}

B3_HD B3_INL void b3_warm_one_contact(B3Contact* c, B3Body* ba, B3Body* bb) {
    B3Vec3 n = c->normal;
    for (int p = 0; p < c->point_count; p++) {
        B3Point* cp = &c->points[p];
        B3Vec3 P = b3_mul(n, cp->normal_impulse);
        b3_apply_impulse(ba, c->inv_mass_a, c->inv_i_a, cp->r_a, P, -1);
        b3_apply_impulse(bb, c->inv_mass_b, c->inv_i_b, cp->r_b, P, 1);
    }
    B3Vec3 f = b3_add(b3_mul(c->tangent1, c->friction_impulse.x),
        b3_mul(c->tangent2, c->friction_impulse.y));
    b3_apply_impulse(ba, c->inv_mass_a, c->inv_i_a, c->center_a, f, -1);
    b3_apply_impulse(bb, c->inv_mass_b, c->inv_i_b, c->center_b, f, 1);
    if ((ba->flags & B3_FLAG_DYNAMIC) != 0) {
        ba->ang_vel = b3_sub(ba->ang_vel,
            b3_mv(c->inv_i_a, b3_add(b3_mul(n, c->twist_impulse), c->rolling_impulse)));
    }
    if ((bb->flags & B3_FLAG_DYNAMIC) != 0) {
        bb->ang_vel = b3_add(bb->ang_vel,
            b3_mv(c->inv_i_b, b3_add(b3_mul(n, c->twist_impulse), c->rolling_impulse)));
    }
}

B3_HD B3_INL void b3_solve_one_contact(B3Contact* c, B3Body* ba, B3Body* bb,
        float inv_h, float contact_speed, int use_bias) {
    B3Vec3 va = ba->lin_vel;
    B3Vec3 wa = ba->ang_vel;
    B3Vec3 vb = bb->lin_vel;
    B3Vec3 wb = bb->ang_vel;
    B3Quat dqa = ba->delta_rot;
    B3Quat dqb = bb->delta_rot;
    B3Vec3 dp = b3_sub(bb->delta_pos, ba->delta_pos);
    B3Vec3 n = c->normal;
    float total_n = 0.0f;
    float twist_lim = 0.0f;
    for (int p = 0; p < c->point_count; p++) {
        B3Point* cp = &c->points[p];
        B3Vec3 ra = cp->r_a;
        B3Vec3 rb = cp->r_b;
        B3Vec3 ds = b3_add(dp, b3_sub(b3_rotate(dqb, rb),
            b3_rotate(dqa, ra)));
        float sep = b3_dot(ds, n) + cp->base_sep;
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
        float vn = b3_dot(b3_sub(vrb, vra), n);
        float dimp = -cp->normal_mass * (mscale * vn + vbias)
            - iscale * cp->normal_impulse;
        float nimp = b3_maxf(cp->normal_impulse + dimp, 0.0f);
        dimp = nimp - cp->normal_impulse;
        cp->normal_impulse = nimp;
        cp->total_normal += nimp;
        total_n += nimp;
        twist_lim += cp->lever * cp->normal_impulse;
        B3Vec3 P = b3_mul(n, dimp);
        va = b3_msub(va, c->inv_mass_a, P);
        wa = b3_sub(wa, b3_mv(c->inv_i_a, b3_cross(ra, P)));
        vb = b3_madd(vb, c->inv_mass_b, P);
        wb = b3_add(wb, b3_mv(c->inv_i_b, b3_cross(rb, P)));
    }
    if (!use_bias) {
        float twist_s = b3_dot(n, b3_sub(wb, wa));
        float max_t = c->friction * twist_lim;
        float dtw = -c->twist_mass * twist_s;
        float old_t = c->twist_impulse;
        c->twist_impulse = b3_clamp(old_t + dtw, -max_t, max_t);
        dtw = c->twist_impulse - old_t;
        wa = b3_sub(wa, b3_mv(c->inv_i_a, b3_mul(n, dtw)));
        wb = b3_add(wb, b3_mv(c->inv_i_b, b3_mul(n, dtw)));

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
        va = b3_msub(va, c->inv_mass_a, P);
        wa = b3_sub(wa, b3_mv(c->inv_i_a, b3_cross(ra, P)));
        vb = b3_madd(vb, c->inv_mass_b, P);
        wb = b3_add(wb, b3_mv(c->inv_i_b, b3_cross(rb, P)));
        b3_solve_rolling(c->tangent1, c->tangent2, c->rolling, total_n,
            c->inv_i_a, c->inv_i_b, &c->rolling_impulse, &wa, &wb);
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

B3_HD B3_INL void b3_solve_contacts_n(B3Contact* contacts, int n,
        B3Body* bodies, float inv_h, float contact_speed, int use_bias) {
    for (int i = 0; i < n; i++) {
        B3Contact* c = &contacts[i];
        b3_solve_one_contact(c, &bodies[c->body_a], &bodies[c->body_b],
            inv_h, contact_speed, use_bias);
    }
}

B3_HD B3_INL void b3_solve_contacts(B3World* w, float inv_h,
        float contact_speed, int use_bias) {
#if B3_ART_CONTACTS
    if (w->joint_count > 0 && w->contact_count > 0) {
        B3Art art;
        if (b3_art_bind(&art, w)) {
            b3_art_solve_contacts(&art, w, inv_h, contact_speed, use_bias, 1);
            return;
        }
    }
#endif
    b3_solve_contacts_n(w->contacts, w->contact_count, w->bodies,
        inv_h, contact_speed, use_bias);
}

B3_HD B3_INL void b3_apply_restitution(B3World* w, float threshold) {
    for (int i = 0; i < w->contact_count; i++) {
        B3Contact* c = &w->contacts[i];
        if (c->restitution == 0.0f) {
            continue;
        }
        B3Body* ba = &w->bodies[c->body_a];
        B3Body* bb = &w->bodies[c->body_b];
        B3Vec3 va = ba->lin_vel;
        B3Vec3 wa = ba->ang_vel;
        B3Vec3 vb = bb->lin_vel;
        B3Vec3 wb = bb->ang_vel;
        B3Vec3 n = c->normal;
        for (int p = 0; p < c->point_count; p++) {
            B3Point* cp = &c->points[p];
            if (cp->rel_vel > -threshold || cp->total_normal == 0.0f) {
                continue;
            }
            B3Vec3 vra = b3_add(va, b3_cross(wa, cp->r_a));
            B3Vec3 vrb = b3_add(vb, b3_cross(wb, cp->r_b));
            float vn = b3_dot(b3_sub(vrb, vra), n);
            float imp = -cp->normal_mass
                * (vn + c->restitution * cp->rel_vel);
            float nimp = b3_maxf(cp->normal_impulse + imp, 0.0f);
            imp = nimp - cp->normal_impulse;
            cp->normal_impulse = nimp;
            B3Vec3 P = b3_mul(n, imp);
            va = b3_msub(va, c->inv_mass_a, P);
            wa = b3_sub(wa, b3_mv(c->inv_i_a, b3_cross(cp->r_a, P)));
            vb = b3_madd(vb, c->inv_mass_b, P);
            wb = b3_add(wb, b3_mv(c->inv_i_b, b3_cross(cp->r_b, P)));
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
