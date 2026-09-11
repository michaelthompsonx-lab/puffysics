// ==== puffysics 04_world.inl: INTERNAL: defaults, world init, capacity API, body/shape creation, mass ====
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
B3_HD B3_INL B3BodyDef b3_default_body(void) {
    B3BodyDef d;
    memset(&d, 0, sizeof(d));
    d.type = B3_STATIC;
    d.rotation = b3_q_id();
    d.gravity_scale = 1.0f;
    return d;
}

B3_HD B3_INL B3ShapeDef b3_default_shape(void) {
    B3ShapeDef d;
    d.density = 1000.0f;
    d.friction = 0.6f;
    d.restitution = 0.0f;
    d.rolling = 0.0f;
    d.category = ~0ull;
    d.mask = ~0ull;
    return d;
}

B3_HD B3_INL void b3_world_init(B3World* w) {
    memset(w, 0, sizeof(*w));
    w->gravity = b3_v(0.0f, -10.0f, 0.0f);
    w->contact_hertz = 30.0f;
    w->contact_damping = 10.0f;
    w->contact_speed = 3.0f;
    w->restitution_threshold = 1.0f;
    w->max_linear_speed = 400.0f;
    // memset clears the sticky overflow/dropped counters above.
}

// Capacity queries (compile-time B3_MAX_*); callers need not read macros.
B3_HD B3_INL int b3_body_capacity(void) { return B3_MAX_BODIES; }
B3_HD B3_INL int b3_shape_capacity(void) { return B3_MAX_SHAPES; }
B3_HD B3_INL int b3_contact_capacity(void) { return B3_MAX_CONTACTS; }
B3_HD B3_INL int b3_joint_capacity(void) { return B3_MAX_JOINTS; }
B3_HD B3_INL unsigned long long b3_config_signature(void) {
    return B3_CONFIG_SIGNATURE;
}
B3_HD B3_INL int b3_config_matches(unsigned long long sig) {
    return sig == B3_CONFIG_SIGNATURE;
}

// Clear sticky overflow/dropped-contact flags without touching content.
B3_HD B3_INL void b3_clear_errors(B3World* w) {
    w->body_overflow = 0;
    w->shape_overflow = 0;
    w->joint_overflow = 0;
    w->contact_overflow = 0;
    w->contacts_dropped = 0;
}

// Snapshot counts plus sticky flags. Returns nonzero if any overflow or
// contact loss is recorded since init / last clear.
B3_HD B3_INL int b3_step_stats(const B3World* w, B3StepStats* out) {
    out->body_count = w->body_count;
    out->shape_count = w->shape_count;
    out->contact_count = w->contact_count;
    out->joint_count = w->joint_count;
    out->body_overflow = w->body_overflow;
    out->shape_overflow = w->shape_overflow;
    out->joint_overflow = w->joint_overflow;
    out->contact_overflow = w->contact_overflow;
    out->contacts_dropped = w->contacts_dropped;
    return (w->body_overflow || w->shape_overflow || w->joint_overflow
        || w->contact_overflow);
}

// Nonzero if any capacity error or contact loss is currently recorded.
B3_HD B3_INL int b3_world_error(const B3World* w) {
    return (w->body_overflow || w->shape_overflow || w->joint_overflow
        || w->contact_overflow);
}

B3_HD B3_INL void b3_body_from_def(B3Body* b, const B3BodyDef* def) {
    memset(b, 0, sizeof(*b));
    b->position = def->position;
    b->rotation = b3_qnorm(def->rotation);
    b->center = def->position;
    b->lin_vel = def->lin_vel;
    b->ang_vel = def->ang_vel;
    b->delta_rot = b3_q_id();
    b->linear_damping = def->linear_damping;
    b->angular_damping = def->angular_damping;
    b->gravity_scale = def->gravity_scale;
    b->type = def->type;
    b->flags = def->flags;
    if (def->type == B3_DYNAMIC) {
        b->flags |= B3_FLAG_DYNAMIC;
    }
}

B3_HD B3_INL void b3_shape_fill(B3Shape* s, int body, int type,
        B3Vec3 local_pos, B3Quat local_rot, float radius, B3Vec3 half,
        const B3ShapeDef* def) {
    s->body = body;
    s->type = type;
    s->local_pos = local_pos;
    s->local_rot = b3_qnorm(local_rot);
    s->radius = radius;
    s->half = half;
    s->friction = def->friction;
    s->restitution = def->restitution;
    s->rolling = def->rolling;
    s->density = def->density;
    s->category = def->category;
    s->mask = def->mask;
}

B3_HD B3_INL int b3_create_body(B3World* w, const B3BodyDef* def) {
    assert(w != NULL && def != NULL);
    assert(w->body_count >= 0 && w->body_count <= B3_MAX_BODIES);
    if (w->body_count >= B3_MAX_BODIES) {
        w->body_overflow = 1;
        return -1;
    }
    int id = w->body_count++;
    b3_body_from_def(&w->bodies[id], def);
    return id;
}

B3_HD B3_INL int b3_add_shape(B3World* w, int body, int type,
        B3Vec3 local_pos, B3Quat local_rot, float radius, B3Vec3 half,
        const B3ShapeDef* def) {
    assert(w != NULL && def != NULL);
    assert(w->shape_count >= 0 && w->shape_count <= B3_MAX_SHAPES);
    if (w->shape_count >= B3_MAX_SHAPES) {
        w->shape_overflow = 1;
        return -1;
    }
    if (body < 0 || body >= w->body_count
            || type < B3_SPHERE || type > B3_BOX) {
        w->shape_overflow = 1;
        return -1;
    }
    int id = w->shape_count++;
    b3_shape_fill(&w->shapes[id], body, type, local_pos, local_rot,
        radius, half, def);
    return id;
}

B3_HD B3_INL int b3_create_sphere(B3World* w, int body, B3Vec3 c,
        float r, const B3ShapeDef* def) {
    return b3_add_shape(w, body, B3_SPHERE, c, b3_q_id(), r,
        b3_v(0.0f, 0.0f, 0.0f), def);
}

B3_HD B3_INL int b3_create_capsule(B3World* w, int body, float half_len,
        float r, const B3ShapeDef* def) {
    return b3_add_shape(w, body, B3_CAPSULE, b3_v(0.0f, 0.0f, 0.0f),
        b3_q_id(), r, b3_v(0.0f, half_len, 0.0f), def);
}

B3_HD B3_INL int b3_create_box(B3World* w, int body, B3Vec3 half,
        const B3ShapeDef* def) {
    return b3_add_shape(w, body, B3_BOX, b3_v(0.0f, 0.0f, 0.0f),
        b3_q_id(), 0.0f, half, def);
}

B3_HD B3_INL int b3_create_box_local(B3World* w, int body, B3Vec3 half,
        B3Vec3 local_pos, B3Quat local_rot, const B3ShapeDef* def) {
    return b3_add_shape(w, body, B3_BOX, local_pos, local_rot, 0.0f,
        half, def);
}

// Shared inverse-mass write for b3_set_inertial/b3_finalize_mass_of.
B3_HD B3_INL void b3_write_inv_mass(B3Body* b, float inv_mass,
        B3Vec3 inertia) {
    b->inv_mass = inv_mass;
    b->inv_inertia = b3_v(
        inertia.x > 0.0f ? 1.0f / inertia.x : 0.0f,
        inertia.y > 0.0f ? 1.0f / inertia.y : 0.0f,
        inertia.z > 0.0f ? 1.0f / inertia.z : 0.0f);
    b->inv_i_world = b3_world_inv_i(b->rotation, b->inv_inertia);
}

B3_HD B3_INL void b3_set_inertial(B3World* w, int body, float mass,
        B3Vec3 local_com, B3Vec3 inertia) {
    B3Body* b = &w->bodies[body];
    b->local_center = local_com;
    b->center = b3_xf_point(b->position, b->rotation, local_com);
    if (b->type != B3_DYNAMIC || mass <= 0.0f) {
        b3_write_inv_mass(b, 0.0f, b3_v(0.0f, 0.0f, 0.0f));
        return;
    }
    b3_write_inv_mass(b, 1.0f / mass, inertia);
}

B3_HD B3_INL void b3_shape_mass(const B3Shape* s, float* mass,
        B3Vec3* com, B3Vec3* inertia) {
    if (s->type == B3_SPHERE) {
        float r = s->radius;
        float m = (4.0f / 3.0f) * B3_PI * r * r * r * s->density;
        *mass = m;
        *com = s->local_pos;
        float i = 0.4f * m * r * r;
        *inertia = b3_v(i, i, i);
        return;
    }
    if (s->type == B3_BOX) {
        float hx = s->half.x;
        float hy = s->half.y;
        float hz = s->half.z;
        float m = 8.0f * hx * hy * hz * s->density;
        *mass = m;
        *com = s->local_pos;
        *inertia = b3_v(
            (m / 3.0f) * (hy * hy + hz * hz),
            (m / 3.0f) * (hx * hx + hz * hz),
            (m / 3.0f) * (hx * hx + hy * hy));
        return;
    }
    float r = s->radius;
    float h = 2.0f * s->half.y;
    float cyl_v = B3_PI * r * r * h;
    float sph_v = (4.0f / 3.0f) * B3_PI * r * r * r;
    float cyl_m = cyl_v * s->density;
    float sph_m = sph_v * s->density;
    float m = cyl_m + sph_m;
    *mass = m;
    *com = s->local_pos;
    float ix = 0.5f * cyl_m * r * r + 0.4f * sph_m * r * r;
    float iy = (1.0f / 12.0f) * cyl_m * (3.0f * r * r + h * h)
        + 0.4f * sph_m * r * r
        + 0.125f * sph_m * (3.0f * r + 2.0f * h) * h;
    *inertia = b3_v(iy, ix, iy);
}

B3_HD B3_INL void b3_finalize_mass_of(B3Body* b, B3Shape* shapes, int n,
        int body) {
    if (b->type != B3_DYNAMIC) {
        b3_write_inv_mass(b, 0.0f, b3_v(0.0f, 0.0f, 0.0f));
        b->center = b->position;
        return;
    }
    float mass = 0.0f;
    B3Vec3 com = b3_v(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < n; i++) {
        B3Shape* s = &shapes[i];
        if (s->body != body || s->density <= 0.0f) {
            continue;
        }
        float sm;
        B3Vec3 sc, si;
        b3_shape_mass(s, &sm, &sc, &si);
        mass += sm;
        com = b3_madd(com, sm, sc);
    }
    if (mass <= 0.0f) {
        b->inv_mass = 0.0f;
        b->inv_inertia = b3_v(0.0f, 0.0f, 0.0f);
        return;
    }
    com = b3_mul(com, 1.0f / mass);
    b->local_center = com;
    b->center = b3_xf_point(b->position, b->rotation, com);
    // Diagonal of the true body-frame inertia about the combined COM: each
    // shape diagonal rotated by its local orientation, plus the
    // parallel-axis shift from the combined COM. Off-diagonals are dropped:
    // the engine stores diagonal body inertia. Exact for axis-aligned
    // compounds; diagonal-exact for rotated shapes.
    B3Vec3 diag = b3_v(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < n; i++) {
        B3Shape* s = &shapes[i];
        if (s->body != body || s->density <= 0.0f) {
            continue;
        }
        float sm;
        B3Vec3 sc, si;
        b3_shape_mass(s, &sm, &sc, &si);
        B3Vec3 ax, ay, az;
        b3_axes(s->local_rot, &ax, &ay, &az);
        B3Vec3 d = b3_sub(sc, com);
        float d2 = b3_dot(d, d);
        diag.x += si.x * ax.x * ax.x + si.y * ay.x * ay.x
            + si.z * az.x * az.x + sm * (d2 - d.x * d.x);
        diag.y += si.x * ax.y * ax.y + si.y * ay.y * ay.y
            + si.z * az.y * az.y + sm * (d2 - d.y * d.y);
        diag.z += si.x * ax.z * ax.z + si.y * ay.z * ay.z
            + si.z * az.z * az.z + sm * (d2 - d.z * d.z);
    }
    b3_write_inv_mass(b, 1.0f / mass, diag);
}

B3_HD B3_INL void b3_finalize_mass(B3World* w, int body) {
    b3_finalize_mass_of(&w->bodies[body], w->shapes, w->shape_count, body);
}

B3_HD B3_INL B3Quat b3_shape_rot(const B3Body* b, const B3Shape* s) {
    return b3_qmul(b->rotation, s->local_rot);
}

B3_HD B3_INL B3Vec3 b3_shape_pos(const B3Body* b, const B3Shape* s) {
    return b3_xf_point(b->position, b->rotation, s->local_pos);
}

B3_HD B3_INL B3AABB b3_shape_aabb(const B3Body* b, const B3Shape* s) {
    B3Vec3 p = b3_shape_pos(b, s);
    B3Quat q = b3_shape_rot(b, s);
    B3AABB a;
    if (s->type == B3_SPHERE) {
        B3Vec3 e = b3_v(s->radius, s->radius, s->radius);
        a.lo = b3_sub(p, e);
        a.hi = b3_add(p, e);
        return a;
    }
    if (s->type == B3_CAPSULE) {
        B3Vec3 y = b3_rotate(q, b3_v(0.0f, s->half.y, 0.0f));
        B3Vec3 c1 = b3_sub(p, y);
        B3Vec3 c2 = b3_add(p, y);
        B3Vec3 e = b3_v(s->radius, s->radius, s->radius);
        a.lo = b3_v(b3_minf(c1.x, c2.x), b3_minf(c1.y, c2.y),
            b3_minf(c1.z, c2.z));
        a.hi = b3_v(b3_maxf(c1.x, c2.x), b3_maxf(c1.y, c2.y),
            b3_maxf(c1.z, c2.z));
        a.lo = b3_sub(a.lo, e);
        a.hi = b3_add(a.hi, e);
        return a;
    }
    B3Vec3 ax, ay, az;
    b3_axes(q, &ax, &ay, &az);
    B3Vec3 e = b3_v(
        fabsf(ax.x) * s->half.x + fabsf(ay.x) * s->half.y
            + fabsf(az.x) * s->half.z,
        fabsf(ax.y) * s->half.x + fabsf(ay.y) * s->half.y
            + fabsf(az.y) * s->half.z,
        fabsf(ax.z) * s->half.x + fabsf(ay.z) * s->half.y
            + fabsf(az.z) * s->half.z);
    a.lo = b3_sub(p, e);
    a.hi = b3_add(p, e);
    return a;
}
