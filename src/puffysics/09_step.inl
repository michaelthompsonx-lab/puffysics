// ==== puffysics 09_step.inl: INTERNAL: integrators, substep driver, b3_step, CUDA kernel ====
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
B3_HD B3_INL void b3_integrate_velocity_state(const B3Body* b, B3Vec3 gravity,
        float h, B3Vec3* lin_vel, B3Vec3* ang_vel) {
    if (b->type != B3_DYNAMIC) {
        return;
    }
    float ld = 1.0f / (1.0f + h * b->linear_damping);
    float ad = 1.0f / (1.0f + h * b->angular_damping);
    float gs = b->inv_mass > 0.0f ? b->gravity_scale : 0.0f;
    B3Vec3 dv = b3_add(b3_mul(b->force, h * b->inv_mass),
        b3_mul(gravity, h * gs));
    *lin_vel = b3_madd(dv, ld, *lin_vel);
    B3Vec3 dw = b3_mul(b3_mv(b->inv_i_world, b->torque), h);
    *ang_vel = b3_madd(dw, ad, *ang_vel);
}

#if B3_HAS_USER_FORCES
typedef struct B3UserForceState {
    B3Vec3 force[B3_MAX_BODIES], torque[B3_MAX_BODIES];
} B3UserForceState;

B3_HD B3_INL void b3_user_forces_begin(B3World* w, float h, B3UserForceState* saved) {
    for (int i = 0; i < w->body_count; i++) {
        saved->force[i] = w->bodies[i].force;
        saved->torque[i] = w->bodies[i].torque;
    }
    B3_USER_FORCES(w, h);
}

B3_HD B3_INL void b3_user_forces_end(B3World* w, const B3UserForceState* saved) {
    for (int i = 0; i < w->body_count; i++) {
        w->bodies[i].force = saved->force[i];
        w->bodies[i].torque = saved->torque[i];
    }
}
#endif

B3_HD B3_INL void b3_integrate_velocities(B3World* w, float h) {
#if B3_HAS_USER_FORCES
    B3UserForceState saved;
    b3_user_forces_begin(w, h, &saved);
#endif
    for (int i = 0; i < w->body_count; i++) {
        B3Body* b = &w->bodies[i];
        b3_integrate_velocity_state(b, w->gravity, h, &b->lin_vel, &b->ang_vel);
    }
#if B3_HAS_USER_FORCES
    b3_user_forces_end(w, &saved);
#endif
}

B3_HD B3_INL void b3_integrate_position_state(const B3Body* b, float h,
        float max_lin, float max_ang, float max_lin2, float max_ang2,
        B3Vec3* lin_vel, B3Vec3* ang_vel, B3Vec3* delta_pos, B3Quat* delta_rot) {
    if (b->type == B3_STATIC) {
        return;
    }
    B3Vec3 v = *lin_vel;
    B3Vec3 av = *ang_vel;
    if (b->flags & B3_LOCK_LIN_X) {
        v.x = 0.0f;
    }
    if (b->flags & B3_LOCK_LIN_Y) {
        v.y = 0.0f;
    }
    if (b->flags & B3_LOCK_LIN_Z) {
        v.z = 0.0f;
    }
    if (b->flags & B3_LOCK_ANG_X) {
        av.x = 0.0f;
    }
    if (b->flags & B3_LOCK_ANG_Y) {
        av.y = 0.0f;
    }
    if (b->flags & B3_LOCK_ANG_Z) {
        av.z = 0.0f;
    }
    float v2 = b3_len2(v);
    if (v2 > max_lin2 && v2 > 0.0f) {
        v = b3_mul(v, b3_rsqrt_scale(max_lin, v2));
    }
#if !B3_UNCLAMPED_ROTATION
    float w2 = b3_len2(av);
    if (w2 > max_ang2 && w2 > 0.0f) {
        av = b3_mul(av, b3_rsqrt_scale(max_ang, w2));
    }
#endif
    *lin_vel = v;
    *ang_vel = av;
    *delta_pos = b3_madd(*delta_pos, h, v);
    *delta_rot = b3_q_integrate(*delta_rot, b3_mul(av, h));
}

B3_HD B3_INL void b3_integrate_positions(B3World* w, float h,
        float inv_dt, float max_lin) {
    float max_ang = B3_MAX_ROTATION * inv_dt;
    float max_lin2 = max_lin * max_lin;
    float max_ang2 = max_ang * max_ang;
    for (int i = 0; i < w->body_count; i++) {
        B3Body* b = &w->bodies[i];
        b3_integrate_position_state(b, h, max_lin, max_ang, max_lin2, max_ang2,
            &b->lin_vel, &b->ang_vel, &b->delta_pos, &b->delta_rot);
    }
}

B3_HD B3_INL void b3_body_fin(B3Body* b) {
    if (b->type == B3_STATIC) {
        return;
    }
    b->center = b3_add(b->center, b->delta_pos);
    b->rotation = b3_qnorm(b3_qmul(b->delta_rot, b->rotation));
    b->position = b3_sub(b->center,
        b3_rotate(b->rotation, b->local_center));
    b->delta_pos = b3_v(0.0f, 0.0f, 0.0f);
    b->delta_rot = b3_q_id();
    b->force = b3_v(0.0f, 0.0f, 0.0f);
    b->torque = b3_v(0.0f, 0.0f, 0.0f);
    if (b->type == B3_DYNAMIC) {
        b->inv_i_world = b3_world_inv_i(b->rotation, b->inv_inertia);
    }
}

B3_HD B3_INL void b3_finalize_transforms(B3World* w) {
    for (int i = 0; i < w->body_count; i++) {
        b3_body_fin(&w->bodies[i]);
    }
}

B3_HD B3_INL void b3_apply_force(B3World* w, int body, B3Vec3 force) {
    w->bodies[body].force = b3_add(w->bodies[body].force, force);
}

B3_HD B3_INL void b3_apply_torque(B3World* w, int body, B3Vec3 torque) {
    w->bodies[body].torque = b3_add(w->bodies[body].torque, torque);
}

B3_HD B3_INL void b3_apply_linear_impulse(B3World* w, int body,
        B3Vec3 impulse) {
    B3Body* b = &w->bodies[body];
    if (b->type == B3_DYNAMIC) {
        b->lin_vel = b3_madd(b->lin_vel, b->inv_mass, impulse);
    }
}

B3_HD B3_INL void b3_soft_step_params(const B3World* w, float dt, int substeps,
        float* h, float* inv_h, float* inv_dt, B3Soft* cs, B3Soft* ss) {
    int subs = substeps < 1 ? 1 : substeps;
    *h = dt / (float)subs;
    *inv_dt = 1.0f / dt;
    *inv_h = (float)subs * (*inv_dt);
    float hertz = b3_minf(w->contact_hertz, 0.25f * (*inv_h));
    *cs = b3_make_soft(hertz, w->contact_damping, *h);
    *ss = b3_make_soft(2.0f * hertz, 0.5f * w->contact_damping, *h);
}

B3_HD B3_INL void b3_step_begin(B3World* w, float h, B3Soft cs, B3Soft ss) {
    b3_find_contacts(w);
    b3_prepare_contacts(w, cs, ss);
    b3_prepare_joints(w, h);
    b3_warm_start(w);
    b3_warm_start_joints(w);
}

B3_HD B3_INL void b3_step_sub(B3World* w, float h, float inv_h, float inv_dt) {
    b3_integrate_velocities(w, h);
#ifndef B3_INTERLEAVE_CONTACTS
    b3_solve_contacts(w, inv_h, w->contact_speed, 1);
#endif
    b3_solve_joints(w, h, inv_h, 1);
    b3_integrate_positions(w, h, inv_dt, w->max_linear_speed);
#ifndef B3_INTERLEAVE_CONTACTS
    b3_solve_contacts(w, inv_h, w->contact_speed, 0);
#endif
    b3_solve_joints(w, h, inv_h, 0);
#ifndef B3_SKIP_RESTITUTION
    b3_apply_restitution(w, w->restitution_threshold);
#endif
}

B3_HD B3_INL void b3_step_indep(B3World* w, float dt, int substeps) {
    if (dt <= 0.0f) {
        return;
    }
    int subs = substeps < 1 ? 1 : substeps;
    float h, inv_h, inv_dt;
    B3Soft cs, ss;
    b3_soft_step_params(w, dt, substeps, &h, &inv_h, &inv_dt, &cs, &ss);
    b3_step_begin(w, h, cs, ss);
#if B3_PERSISTENT_GS
    B3GsBody bl[B3_MAX_BODIES];
    B3GsJoint jl[B3_MAX_JOINTS];
    B3GsContact cl[B3_MAX_CONTACTS];
    b3_gs_load(w, bl, jl, cl);
#if B3_PACKED_INTEGRATE
    float max_ang = B3_MAX_ROTATION * inv_dt;
    float max_lin = w->max_linear_speed;
    float max_lin2 = max_lin * max_lin;
    float max_ang2 = max_ang * max_ang;
    for (int s = 0; s < subs; s++) {
#if B3_HAS_USER_FORCES
        // Hooks see current packed velocities and delta poses, not last step.
        for (int i = 0; i < w->body_count; i++) {
            w->bodies[i].lin_vel = bl[i].lin_vel;
            w->bodies[i].ang_vel = bl[i].ang_vel;
            w->bodies[i].delta_pos = bl[i].delta_pos;
            w->bodies[i].delta_rot = bl[i].delta_rot;
        }
        b3_integrate_velocities(w, h);
        for (int i = 0; i < w->body_count; i++) {
            bl[i].lin_vel = w->bodies[i].lin_vel;
            bl[i].ang_vel = w->bodies[i].ang_vel;
        }
#else
        for (int i = 0; i < w->body_count; i++) {
            b3_integrate_velocity_state(&w->bodies[i], w->gravity, h,
                &bl[i].lin_vel, &bl[i].ang_vel);
        }
#endif
        // Velocity integration and relaxation leave delta poses unchanged.
        b3_gs_solve_cached(w, bl, jl, cl, h, inv_h, 1,
            !B3_REUSE_GS_CACHE || s == 0);
        for (int i = 0; i < w->body_count; i++) {
            b3_integrate_position_state(&w->bodies[i], h, max_lin, max_ang,
                max_lin2, max_ang2, &bl[i].lin_vel, &bl[i].ang_vel,
                &bl[i].delta_pos, &bl[i].delta_rot);
        }
        b3_gs_solve(w, bl, jl, cl, h, inv_h, 0);
    }
    b3_gs_store_velocities(w, bl);
    for (int i = 0; i < w->body_count; i++) {
        w->bodies[i].delta_pos = bl[i].delta_pos;
        w->bodies[i].delta_rot = bl[i].delta_rot;
    }
#else
    for (int s = 0; s < subs; s++) {
        b3_integrate_velocities(w, h);
        b3_gs_refresh_bodies(w, bl);
        // Velocity integration and relaxation leave delta poses unchanged.
        b3_gs_solve_cached(w, bl, jl, cl, h, inv_h, 1,
            !B3_REUSE_GS_CACHE || s == 0);
        b3_gs_store_velocities(w, bl);
        b3_integrate_positions(w, h, inv_dt, w->max_linear_speed);
        b3_gs_refresh_bodies(w, bl);
        b3_gs_solve(w, bl, jl, cl, h, inv_h, 0);
        b3_gs_store_velocities(w, bl);
    }
#endif
    b3_gs_store_constraints(w, jl, cl);
#else
    for (int s = 0; s < subs; s++) {
        b3_step_sub(w, h, inv_h, inv_dt);
    }
#endif
    b3_finalize_transforms(w);
}

/* The ordinary integrator retains Soft Step joint motors/springs/limits;
 * articulated contact response is selected inside the contact sweeps. */
B3_HD B3_INL void b3_step(B3World* w, float dt, int substeps) {
    b3_step_indep(w, dt, substeps);
}

#if B3_ART_CONTACTS
/* Reduced-coordinate ABA/FK for passive revolute trees and loop cuts.
 * Actuated, welded, locked or kinematic worlds use the ordinary joint solver,
 * rather than silently ignoring a contract the reduced integrator lacks. */
B3_HD B3_INL void b3_art_step(B3World* w, float dt, int substeps) {
    if (dt <= 0.0f) return;
    for (int j = 0; j < w->joint_count; j++) {
        const B3Joint* joint = &w->joints[j];
        int unsupported = joint->fixed_rotation || joint->enable_spring || joint->enable_limit;
#ifndef B3_REVOLUTE_ONLY
        unsupported |= joint->type != B3_JOINT_REVOLUTE || joint->enable_motor;
#endif
        if (unsupported) {
            b3_step_indep(w, dt, substeps);
            return;
        }
    }
    for (int i = 0; i < w->body_count; i++) {
        if (w->bodies[i].type == B3_KINEMATIC ||
                (w->bodies[i].flags & (B3_LOCK_LIN_X | B3_LOCK_LIN_Y | B3_LOCK_LIN_Z |
                    B3_LOCK_ANG_X | B3_LOCK_ANG_Y | B3_LOCK_ANG_Z))) {
            b3_step_indep(w, dt, substeps);
            return;
        }
    }
    B3Art art;
    if (!b3_art_bind(&art, w)) {
        b3_step_indep(w, dt, substeps);
        return;
    }
    int subs = substeps < 1 ? 1 : substeps;
    float h, inv_h, inv_dt;
    B3Soft cs, ss;
    b3_soft_step_params(w, dt, subs, &h, &inv_h, &inv_dt, &cs, &ss);
    b3_find_contacts(w);
    b3_prepare_contacts(w, cs, ss);
    b3_warm_start(w);
    for (int s = 0; s < subs; s++) {
#if B3_HAS_USER_FORCES
        B3UserForceState saved;
        b3_user_forces_begin(w, h, &saved);
#endif
        b3_art_integrate_vel(&art, w, h);
#if B3_HAS_USER_FORCES
        b3_user_forces_end(w, &saved);
#endif
        b3_art_solve_contacts(&art, w, inv_h, w->contact_speed, 1, B3_ART_CONTACT_ITERS);
        b3_art_solve_cuts(&art, w, inv_h, 1, B3_ART_CUT_ITERS);
        b3_art_integrate_pos(&art, w, h, inv_dt);
        b3_art_solve_contacts(&art, w, inv_h, w->contact_speed, 0, B3_ART_CONTACT_ITERS);
        b3_art_solve_cuts(&art, w, inv_h, 0, B3_ART_CUT_ITERS);
#ifndef B3_SKIP_RESTITUTION
        b3_apply_restitution(w, w->restitution_threshold);
#endif
    }
    b3_finalize_transforms(w);
}
#endif

#ifdef __CUDACC__
static __global__ void b3_step_kernel(B3World* worlds, int n, float dt,
        int substeps) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        b3_step(&worlds[i], dt, substeps);
    }
}
#endif
