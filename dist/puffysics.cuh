// GENERATED — do not edit. Source of truth: src/puffysics/*.inl
// (see include/puffysics/puffysics.cuh). Regenerate with tools/amalgamate.py.

#pragma once
#ifndef PUFFYSICS_CORE_INCLUDED
#define PUFFYSICS_CORE_INCLUDED
//
// PUBLIC API (stable surface; bodies below are INTERNAL implementation):
//   world: b3_world_init, b3_create_body, b3_add_shape, b3_create_sphere,
//     b3_create_capsule, b3_create_box, b3_create_box_local,
//     b3_finalize_mass, b3_set_inertial, b3_body/shape_capacity,
//     b3_clear_errors, b3_step_stats, b3_world_error,
//     b3_config_signature, b3_config_matches
//   joints: b3_create_weld, b3_create_revolute, b3_joint_enable/set_motor,
//     b3_joint_enable/set_limits, b3_joint_enable/set_spring,
//     b3_joint_angle, b3_joint_speed, b3_joint_capacity
//   step: b3_step, b3_step_kernel, b3_find_contacts, b3_contact_capacity
//   mass: b3_shape_mass
// Topics below map to src/puffysics/*.inl; do not include .inl directly.
// ==== puffysics 01_config.inl: INTERNAL: includes, B3_HD/B3_INL, capacities, feature flags ====
// Puffysics — PufferLib physics engine (puffer + physics).
// SPDX-License-Identifier: MIT
// Grew out of a Box3D Soft Step port; the solver is no longer a port.
// Scope: kinematics + ABA/Delassus articulations + primitive contacts
// + weld/revolute joints (motor, spring, limits).
// Out of scope: CCD, sleep, meshes, sensors, events, other joints.
//
// One B3World per environment. Host: build bodies/shapes, copy worlds to
// device, launch b3_step_kernel. Device: b3_step(&worlds[env], dt, 4).
// Override B3_MAX_BODIES / B3_MAX_SHAPES / B3_MAX_CONTACTS before include.
// ABI is B3_* / b3_*. Optional modules keep their file-stem prefix:
// nbody.cuh (nbody_), nbody_rigid.cuh (b3_nbody_), ambient.h (b3_fluid_).


#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef __CUDACC__
#define B3_HD __host__ __device__
#define B3_INL __forceinline__
#elif defined(__cplusplus)
#define B3_HD
#define B3_INL inline
#else
// Plain C has no extern-inline model here: every TU gets its own copies.
// This is what makes the header consumable from C without a shim .c file.
#define B3_HD
#define B3_INL static inline
#endif

/* PUBLIC SUPPORTED CONFIGURATION (override before include; ABI-stable meaning):
 *   B3_MAX_BODIES / B3_MAX_SHAPES / B3_MAX_CONTACTS / B3_MAX_JOINTS
 *   B3_CONFIG_SIGNATURE (derived; query via b3_config_signature)
 *   B3_JOINT_ITERS / B3_RELAX_ITERS (positive solver iteration counts)
 *   B3_ART_CONTACTS (0 default; 1 opts into articulation contact response)
 *   B3_USER_FORCES (optional per-substep force hook)
 * Everything else in this file is INTERNAL / EXPERIMENTAL / ABLATION:
 *   B3_MERGE_WARM_CACHE, B3_COMPACT_PAIRS, B3_UNCLAMPED_ROTATION,
 *   B3_STATIC_RESTITUTION, B3_PERSISTENT_GS, B3_REUSE_GS_CACHE,
 *   B3_PACKED_INTEGRATE, B3_COUPLED_HINGE, B3_ALTERNATE_JOINT_ORDER,
 *   B3_POLY_ROTATION, B3_RSQRT_MATH, B3_PACKED_GS, B3_REVOLUTE_ONLY,
 *   B3_INTERLEAVE_CONTACTS, B3_SKIP_RESTITUTION, B3_TWIST_APPROX,
 *   B3_CACHE_JOINTS, B3_ABLATE_*. Not a supported public contract.
 * Defaults fit an articulated agent plus extra rigid cubes / stair boxes. */
#ifndef B3_MAX_BODIES
#define B3_MAX_BODIES 80
#endif
#ifndef B3_MAX_SHAPES
#define B3_MAX_SHAPES 128
#endif
#ifndef B3_MAX_CONTACTS
#define B3_MAX_CONTACTS 128
#endif
#ifndef B3_MAX_JOINTS
#define B3_MAX_JOINTS 32
#endif

/* Compile-time ABI fingerprint. B3_MAX_* change sizeof(B3World); every
 * translation unit in one binary must share this value. Query with
 * b3_config_signature() and reject mixed-config worlds. 16-bit fields
 * cover the supported capacity range. */
#define B3_CONFIG_SIGNATURE ( \
    ((unsigned long long)(B3_MAX_BODIES) & 0xffffull) \
  | (((unsigned long long)(B3_MAX_SHAPES) & 0xffffull) << 16) \
  | (((unsigned long long)(B3_MAX_CONTACTS) & 0xffffull) << 32) \
  | (((unsigned long long)(B3_MAX_JOINTS) & 0xffffull) << 48))

// Experimental: faster on some dense-contact batches, but increases register
// pressure and can slow sparse worlds. Benchmark before enabling.
#ifndef B3_MERGE_WARM_CACHE
#define B3_MERGE_WARM_CACHE 0
#endif
/* 1: compact metadata. 2: also iterate eligible shape bitsets and reject
 * separated spheres from cached centers. Both preserve contact order and use
 * the original narrow phase, including its final separation calculation. */
#ifndef B3_COMPACT_PAIRS
#define B3_COMPACT_PAIRS 0
#endif
#if B3_COMPACT_PAIRS >= 2 && B3_MAX_SHAPES > 32
#error "Bitset broad phase requires at most 32 shapes"
#endif
#ifndef B3_UNCLAMPED_ROTATION
#define B3_UNCLAMPED_ROTATION 0
#endif
#ifndef B3_STATIC_RESTITUTION
#define B3_STATIC_RESTITUTION 0
#endif
#ifndef B3_JOINT_ITERS
#define B3_JOINT_ITERS 2
#endif
#ifndef B3_RELAX_ITERS
#define B3_RELAX_ITERS B3_JOINT_ITERS
#endif
#if B3_JOINT_ITERS < 1 || B3_RELAX_ITERS < 1
#error "Solver iteration counts must be positive"
#endif
#ifndef B3_PERSISTENT_GS
#define B3_PERSISTENT_GS 0
#endif
#ifndef B3_REUSE_GS_CACHE
#define B3_REUSE_GS_CACHE 0
#endif
#ifndef B3_PACKED_INTEGRATE
#define B3_PACKED_INTEGRATE 0
#endif
#if B3_PACKED_INTEGRATE && !B3_PERSISTENT_GS
#error "Packed integration requires persistent packed state"
#endif
#ifndef B3_COUPLED_HINGE
#define B3_COUPLED_HINGE 0
#endif
#ifndef B3_ALTERNATE_JOINT_ORDER
#define B3_ALTERNATE_JOINT_ORDER 0
#endif
#if B3_ALTERNATE_JOINT_ORDER && !defined(B3_PACKED_GS)
#error "Alternate joint ordering experiment requires packed solver"
#endif
#if B3_COUPLED_HINGE && (!defined(B3_PACKED_GS) || !defined(B3_REVOLUTE_ONLY))
#error "Coupled hinge experiment requires packed revolute-only solver"
#endif
#if B3_COUPLED_HINGE && (defined(B3_ABLATE_NO_PERP) || defined(B3_ABLATE_NO_POINT))
#error "Coupled hinge experiment cannot ablate one of its coupled blocks"
#endif
#if B3_PERSISTENT_GS && (!defined(B3_PACKED_GS) || !defined(B3_REVOLUTE_ONLY) \
    || !defined(B3_INTERLEAVE_CONTACTS) || !defined(B3_SKIP_RESTITUTION))
#error "Persistent GS requires packed revolute-only, interleaved contacts, and no restitution"
#endif
#ifndef B3_CONNECT_WORDS
#define B3_CONNECT_WORDS ((B3_MAX_BODIES + 63) / 64)
#endif

#define B3_PI 3.14159265359f
#define B3_LINEAR_SLOP 0.005f
#define B3_SPECULATIVE 0.020f
#define B3_MAX_MANIFOLD 4
#define B3_MAX_ROTATION (0.25f * B3_PI)
#define B3_GYRO_ITERS 1
#define B3_MIN_FRICTION_W 1.0e-10f

// Experimental exponential-map integration; benchmark before enabling.
// Changes integration semantics from normalized Euler, not solver iterations.
#ifndef B3_POLY_ROTATION
#define B3_POLY_ROTATION 0
#endif

// B3_TREE_DUAL (experimental packed tree projection) was removed, with
// B3_TREE_SPRINGS and B3_TREE_OUTER_ITERS. The projected GS sweeps below
// are the only joint path. Defining removed knobs to non-default errors.
#if (defined(B3_TREE_DUAL) && B3_TREE_DUAL) \
    || (defined(B3_TREE_SPRINGS) && B3_TREE_SPRINGS) \
    || (defined(B3_TREE_OUTER_ITERS) && B3_TREE_OUTER_ITERS != 2)
#error "B3_TREE_DUAL was removed; use the default joint solver"
#endif

// B3_JOINT_DUAL (experimental block-tridiagonal dual KKT joint solve)
// was removed; the iterated Gauss-Seidel sweeps below are the only
// joint path. Defining B3_JOINT_DUAL to nonzero is now an error.
#if defined(B3_JOINT_DUAL) && B3_JOINT_DUAL
#error "B3_JOINT_DUAL was removed; use the default joint solver"
#endif

// Experimental: fuse sqrt+divide pairs into the reciprocal-square-root
// intrinsic on device (max 2 ulp vs ~1 ulp for the IEEE pair). Touches
// normalization and cone/velocity clamps only; solver reciprocals of
// effective masses stay exact. Benchmark before enabling.
#ifndef B3_RSQRT_MATH
#define B3_RSQRT_MATH 0
#endif

/* Default 0: independent-body contact effective masses with the joint GS
 * solver, for batched RL throughput. Motors, springs, limits and welds stay
 * available. Set 1 before including the core to opt into Delassus contact
 * mass/apply for jointed worlds and the b3_art_* API. That mode changes the
 * contact response and can be substantially more expensive; it is not a
 * drop-in accuracy/performance equivalent. Worlds without joints use the
 * independent-body path in either mode. */
#ifndef B3_ART_CONTACTS
#define B3_ART_CONTACTS 0
#endif

/* Optional per-substep force law. Define before include. The hook may add
 * to externally applied forces: its contributions are consumed once, then
 * the original force/torque are restored for the next substep. Read current
 * poses as center + delta_pos and delta_rot * rotation. No default overhead. */
#ifndef B3_USER_FORCES
#define B3_USER_FORCES(w, h) ((void)0)
#define B3_HAS_USER_FORCES 0
#else
#define B3_HAS_USER_FORCES 1
#endif

// ==== puffysics 02_types.inl: INTERNAL: enum constants, math/body/shape/contact/joint/world structs ====
#define B3_STATIC 0
#define B3_KINEMATIC 1
#define B3_DYNAMIC 2

#define B3_SPHERE 0
#define B3_CAPSULE 1
#define B3_BOX 2

#define B3_JOINT_WELD 0
#define B3_JOINT_REVOLUTE 1

#define B3_FLAG_DYNAMIC 0x00001000u
#define B3_LOCK_LIN_X 0x00000001u
#define B3_LOCK_LIN_Y 0x00000002u
#define B3_LOCK_LIN_Z 0x00000004u
#define B3_LOCK_ANG_X 0x00000008u
#define B3_LOCK_ANG_Y 0x00000010u
#define B3_LOCK_ANG_Z 0x00000020u

typedef struct B3Vec3 {
    float x, y, z;
} B3Vec3;

typedef struct B3Vec2 {
    float x, y;
} B3Vec2;

typedef struct B3Quat {
    B3Vec3 v;
    float s;
} B3Quat;

typedef struct B3Mat3 {
    B3Vec3 cx, cy, cz;
} B3Mat3;

typedef struct B3Mat2 {
    B3Vec2 cx, cy;
} B3Mat2;

typedef struct B3AABB {
    B3Vec3 lo, hi;
} B3AABB;

typedef struct B3Soft {
    float bias_rate;
    float mass_scale;
    float impulse_scale;
} B3Soft;

typedef struct B3Body {
    B3Vec3 position;
    B3Quat rotation;
    B3Vec3 center;
    B3Vec3 local_center;
    B3Vec3 lin_vel;
    B3Vec3 ang_vel;
    B3Vec3 force;
    B3Vec3 torque;
    B3Vec3 delta_pos;
    B3Quat delta_rot;
    float inv_mass;
    B3Vec3 inv_inertia;
    B3Mat3 inv_i_world;
    float linear_damping;
    float angular_damping;
    float gravity_scale;
    int type;
    uint32_t flags;
} B3Body;

typedef struct B3Shape {
    int body;
    int type;
    B3Vec3 local_pos;
    B3Quat local_rot;
    float radius;
    B3Vec3 half;
    float friction;
    float restitution;
    float rolling;
    float density;
    uint64_t category;
    uint64_t mask;
} B3Shape;

typedef struct B3Point {
    B3Vec3 r_a;
    B3Vec3 r_b;
    float base_sep;
    float rel_vel;
    float normal_impulse;
    float total_normal;
    float normal_mass;
    float lever;
    uint32_t feature;
} B3Point;

typedef struct B3Contact {
    int shape_a;
    int shape_b;
    int body_a;
    int body_b;
    int point_count;
    int static_contact;
    B3Vec3 normal;
    B3Vec3 tangent1;
    B3Vec3 tangent2;
    B3Point points[B3_MAX_MANIFOLD];
    B3Vec3 center_a;
    B3Vec3 center_b;
    float friction;
    float restitution;
    float rolling;
    float twist_mass;
    float twist_impulse;
    B3Vec2 friction_impulse;
    B3Vec3 rolling_impulse;

    B3Mat2 tangent_mass;
    B3Mat3 rolling_mass;
    float inv_mass_a;
    float inv_mass_b;
    B3Mat3 inv_i_a;
    B3Mat3 inv_i_b;
    B3Soft softness;
} B3Contact;

typedef struct B3Warm {
    int shape_a;
    int shape_b;
    int point_count;
    uint32_t feature[B3_MAX_MANIFOLD];
    float normal_impulse[B3_MAX_MANIFOLD];
    B3Vec2 friction_impulse;
    float twist_impulse;
    B3Vec3 rolling_impulse;
} B3Warm;

typedef struct B3Joint {
#ifndef B3_REVOLUTE_ONLY
    int type;
#endif
    int body_a;
    int body_b;
    int collide_connected;
    int fixed_rotation;
    B3Vec3 local_anchor_a;
    B3Vec3 local_anchor_b;
    B3Quat local_rot_a;
    B3Quat local_rot_b;
    float constraint_hertz;
    float constraint_damping;
    B3Soft softness;
    float inv_mass_a;
    float inv_mass_b;
    B3Mat3 inv_i_a;
    B3Mat3 inv_i_b;
    B3Vec3 frame_p_a;
    B3Vec3 frame_p_b;
    B3Quat frame_q_a;
    B3Quat frame_q_b;
    B3Vec3 delta_center;
    B3Vec3 linear_impulse;
#ifndef B3_REVOLUTE_ONLY
    B3Vec3 angular_impulse;
    B3Mat3 angular_mass;
    float linear_hertz;
    float linear_damping;
    float angular_hertz;
    float angular_damping;
    B3Soft linear_spring;
    B3Soft angular_spring;
#endif
    B3Vec2 perp_impulse;
    float spring_impulse;
#ifndef B3_REVOLUTE_ONLY
    float motor_impulse;
#endif
    float lower_impulse;
    float upper_impulse;
    float hertz;
    float damping_ratio;
    float max_motor_torque;
#ifndef B3_REVOLUTE_ONLY
    float motor_speed;
#endif
    float target_angle;
    float lower_angle;
    float upper_angle;
    int enable_spring;
#ifndef B3_REVOLUTE_ONLY
    int enable_motor;
#endif
    int enable_limit;
    B3Vec3 rotation_axis;
    B3Vec3 perp_x;
    B3Vec3 perp_y;
    float axial_mass;
    B3Soft spring_softness;
#ifdef B3_CACHE_JOINTS
    /* Geometry + inv(K) for one GS pass. K is constant while delta_rot
     * is frozen, so invert once and matvec in the inner loop. */
    B3Vec3 cache_ra;
    B3Vec3 cache_rb;
    B3Mat3 cache_point_invk;
    B3Mat2 cache_ang_invk;
    B3Vec3 cache_ia_ax;
    B3Vec3 cache_ib_ax;
    float cache_twist;
    float cache_rel_x;
    float cache_rel_y;
#endif
} B3Joint;

#ifdef B3_PACKED_GS
/* Inner-loop snapshot. Bodies keep v/w/Δp/Δq + dyn inv(M). Joints keep
 * impulses, cached inv(K), and the hinge rows. Contacts drop unused
 * prepare fields. inv_i lives on the body so 14 hinges do not each
 * carry two 3x3 copies. */
#define B3_GS_FIXED 1
#define B3_GS_SPRING 2
#define B3_GS_LIMIT 4

typedef struct B3GsBody {
    B3Vec3 lin_vel;
    B3Vec3 ang_vel;
    B3Vec3 delta_pos;
    B3Quat delta_rot;
    float inv_mass;
    B3Mat3 inv_i;
    uint32_t flags;
} B3GsBody;

typedef struct B3GsJoint {
    int body_a;
    int body_b;
    int bits;
    float target_angle;
    float lower_angle;
    float upper_angle;
    float axial_mass;
    float max_motor_torque;
    float spring_impulse;
    float lower_impulse;
    float upper_impulse;
    float cache_twist;
    float cache_rel_x;
    float cache_rel_y;
    B3Soft softness;
    B3Soft spring_softness;
    B3Vec3 rotation_axis;
    B3Vec3 perp_x;
    B3Vec3 perp_y;
    B3Vec3 cache_ra;
    B3Vec3 cache_rb;
    B3Vec3 cache_ia_ax;
    B3Vec3 cache_ib_ax;
    B3Vec3 delta_center;
    B3Vec3 linear_impulse;
    B3Vec2 perp_impulse;
    B3Mat3 cache_point_invk;
    B3Mat2 cache_ang_invk;
#if B3_COUPLED_HINGE
    B3Vec3 cache_point_perp_x;
    B3Vec3 cache_point_perp_y;
#endif
} B3GsJoint;

typedef struct B3GsPoint {
    B3Vec3 r_a;
    B3Vec3 r_b;
    float base_sep;
    float normal_impulse;
    float total_normal;
    float normal_mass;
    float lever;
} B3GsPoint;

typedef struct B3GsContact {
    int body_a;
    int body_b;
    int point_count;
    B3Vec3 normal;
    B3Vec3 tangent1;
    B3Vec3 tangent2;
    B3Vec3 center_a;
    B3Vec3 center_b;
    float friction;
    float rolling;
    B3Vec3 rolling_impulse;
    float twist_mass;
    float twist_impulse;
    B3Vec2 friction_impulse;
    B3Mat2 tangent_mass;
    B3Soft softness;
    B3GsPoint points[B3_MAX_MANIFOLD];
} B3GsContact;
#endif


typedef struct B3World {
    B3Vec3 gravity;
    float contact_hertz;
    float contact_damping;
    float contact_speed;
    float restitution_threshold;
    float max_linear_speed;
    int body_count;
    int shape_count;
    int contact_count;
    int joint_count;
    // Sticky capacity diagnostics. Set on overflow, cleared by
    // b3_world_init() / b3_clear_errors(). Creation returns -1 (NULL for
    // b3_add_joint) without writing out of bounds. Contact generation
    // counts dropped pairs in contacts_dropped and sets contact_overflow
    // instead of silently losing them. Part of the struct layout; capacities
    // stay compile-time (B3_MAX_*), so all TUs must use one build config.
    int body_overflow;
    int shape_overflow;
    int joint_overflow;
    int contact_overflow;
    int contacts_dropped;
    B3Body bodies[B3_MAX_BODIES];
    B3Shape shapes[B3_MAX_SHAPES];
    B3Contact contacts[B3_MAX_CONTACTS];
    B3Joint joints[B3_MAX_JOINTS];
} B3World;

// Step/capacity diagnostics snapshot. contacts_dropped counts pairs with a
// non-empty manifold rejected because contact storage was full.
typedef struct B3StepStats {
    int body_count;
    int shape_count;
    int contact_count;
    int joint_count;
    int body_overflow;
    int shape_overflow;
    int joint_overflow;
    int contact_overflow;
    int contacts_dropped;
} B3StepStats;

typedef struct B3BodyDef {
    int type;
    B3Vec3 position;
    B3Quat rotation;
    B3Vec3 lin_vel;
    B3Vec3 ang_vel;
    float linear_damping;
    float angular_damping;
    float gravity_scale;
    uint32_t flags;
} B3BodyDef;

typedef struct B3ShapeDef {
    float density;
    float friction;
    float restitution;
    float rolling;
    uint64_t category;
    uint64_t mask;
} B3ShapeDef;

// ==== puffysics 03_math.inl: INTERNAL: vector/quaternion/matrix helpers, twist, hinge frame math ====
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
#ifndef ATAN_APPROX_CUH
#define ATAN_APPROX_CUH
// Generated by gen_atan.py; do not edit coefficients by hand.
#if B3_TWIST_APPROX == 2 || B3_TWIST_APPROX == 3
static const float b3_atan_host[257] = {
    0.0000000000e+00f, 3.9062301320e-03f, 7.8123410601e-03f, 1.1718213602e-02f,
    1.5623728620e-02f, 1.9528767041e-02f, 2.3433209879e-02f, 2.7336938258e-02f,
    3.1239833430e-02f, 3.5141776803e-02f, 3.9042649955e-02f, 4.2942334662e-02f,
    4.6840712916e-02f, 5.0737666945e-02f, 5.4633079239e-02f, 5.8526832566e-02f,
    6.2418809996e-02f, 6.6308894920e-02f, 7.0196971072e-02f, 7.4082922549e-02f,
    7.7966633832e-02f, 8.1847989803e-02f, 8.5726875771e-02f, 8.9603177485e-02f,
    9.3476781159e-02f, 9.7347573487e-02f, 1.0121544167e-01f, 1.0508027342e-01f,
    1.0894195699e-01f, 1.1280038120e-01f, 1.1665543544e-01f, 1.2050700969e-01f,
    1.2435499455e-01f, 1.2819928123e-01f, 1.3203976161e-01f, 1.3587632823e-01f,
    1.3970887429e-01f, 1.4353729370e-01f, 1.4736148109e-01f, 1.5118133180e-01f,
    1.5499674192e-01f, 1.5880760832e-01f, 1.6261382860e-01f, 1.6641530118e-01f,
    1.7021192529e-01f, 1.7400360094e-01f, 1.7779022899e-01f, 1.8157171116e-01f,
    1.8534795000e-01f, 1.8911884893e-01f, 1.9288431226e-01f, 1.9664424519e-01f,
    2.0039855383e-01f, 2.0414714518e-01f, 2.0788992720e-01f, 2.1162680877e-01f,
    2.1535769970e-01f, 2.1908251078e-01f, 2.2280115376e-01f, 2.2651354136e-01f,
    2.3021958728e-01f, 2.3391920621e-01f, 2.3761231387e-01f, 2.4129882693e-01f,
    2.4497866313e-01f, 2.4865174119e-01f, 2.5231798089e-01f, 2.5597730301e-01f,
    2.5962962941e-01f, 2.6327488296e-01f, 2.6691298759e-01f, 2.7054386829e-01f,
    2.7416745112e-01f, 2.7778366318e-01f, 2.8139243265e-01f, 2.8499368878e-01f,
    2.8858736189e-01f, 2.9217338339e-01f, 2.9575168575e-01f, 2.9932220253e-01f,
    3.0288486837e-01f, 3.0643961901e-01f, 3.0998639125e-01f, 3.1352512299e-01f,
    3.1705575321e-01f, 3.2057822199e-01f, 3.2409247049e-01f, 3.2759844095e-01f,
    3.3109607670e-01f, 3.3458532217e-01f, 3.3806612284e-01f, 3.4153842530e-01f,
    3.4500217721e-01f, 3.4845732731e-01f, 3.5190382541e-01f, 3.5534162242e-01f,
    3.5877067027e-01f, 3.6219092200e-01f, 3.6560233171e-01f, 3.6900485453e-01f,
    3.7239844668e-01f, 3.7578306541e-01f, 3.7915866903e-01f, 3.8252521690e-01f,
    3.8588266940e-01f, 3.8923098795e-01f, 3.9257013501e-01f, 3.9590007406e-01f,
    3.9922076958e-01f, 4.0253218708e-01f, 4.0583429307e-01f, 4.0912705508e-01f,
    4.1241044160e-01f, 4.1568442212e-01f, 4.1894896713e-01f, 4.2220404808e-01f,
    4.2544963737e-01f, 4.2868570839e-01f, 4.3191223547e-01f, 4.3512919389e-01f,
    4.3833655986e-01f, 4.4153431053e-01f, 4.4472242396e-01f, 4.4790087915e-01f,
    4.5106965599e-01f, 4.5422873527e-01f, 4.5737809867e-01f, 4.6051772877e-01f,
    4.6364760900e-01f, 4.6676772368e-01f, 4.6987805798e-01f, 4.7297859790e-01f,
    4.7606933032e-01f, 4.7915024293e-01f, 4.8222132423e-01f, 4.8528256356e-01f,
    4.8833395106e-01f, 4.9137547765e-01f, 4.9440713507e-01f, 4.9742891581e-01f,
    5.0044081315e-01f, 5.0344282111e-01f, 5.0643493448e-01f, 5.0941714880e-01f,
    5.1238946031e-01f, 5.1535186601e-01f, 5.1830436360e-01f, 5.2124695149e-01f,
    5.2417962878e-01f, 5.2710239527e-01f, 5.3001525142e-01f, 5.3291819839e-01f,
    5.3581123796e-01f, 5.3869437260e-01f, 5.4156760539e-01f, 5.4443094007e-01f,
    5.4728438099e-01f, 5.5012793310e-01f, 5.5296160199e-01f, 5.5578539382e-01f,
    5.5859931534e-01f, 5.6140337389e-01f, 5.6419757736e-01f, 5.6698193422e-01f,
    5.6975645348e-01f, 5.7252114470e-01f, 5.7527601796e-01f, 5.7802108387e-01f,
    5.8075635357e-01f, 5.8348183869e-01f, 5.8619755136e-01f, 5.8890350420e-01f,
    5.9159971034e-01f, 5.9428618332e-01f, 5.9696293722e-01f, 5.9962998650e-01f,
    6.0228734613e-01f, 6.0493503149e-01f, 6.0757305839e-01f, 6.1020144306e-01f,
    6.1282020217e-01f, 6.1542935275e-01f, 6.1802891228e-01f, 6.2061889860e-01f,
    6.2319932993e-01f, 6.2577022489e-01f, 6.2833160243e-01f, 6.3088348190e-01f,
    6.3342588297e-01f, 6.3595882567e-01f, 6.3848233035e-01f, 6.4099641773e-01f,
    6.4350110879e-01f, 6.4599642489e-01f, 6.4848238764e-01f, 6.5095901900e-01f,
    6.5342634118e-01f, 6.5588437671e-01f, 6.5833314838e-01f, 6.6077267927e-01f,
    6.6320299271e-01f, 6.6562411228e-01f, 6.6803606186e-01f, 6.7043886551e-01f,
    6.7283254759e-01f, 6.7521713266e-01f, 6.7759264552e-01f, 6.7995911118e-01f,
    6.8231655487e-01f, 6.8466500205e-01f, 6.8700447834e-01f, 6.8933500960e-01f,
    6.9165662185e-01f, 6.9396934132e-01f, 6.9627319441e-01f, 6.9856820768e-01f,
    7.0085440788e-01f, 7.0313182192e-01f, 7.0540047687e-01f, 7.0766039992e-01f,
    7.0991161846e-01f, 7.1215415999e-01f, 7.1438805216e-01f, 7.1661332273e-01f,
    7.1882999962e-01f, 7.2103811085e-01f, 7.2323768458e-01f, 7.2542874904e-01f,
    7.2761133263e-01f, 7.2978546379e-01f, 7.3195117112e-01f, 7.3410848326e-01f,
    7.3625742898e-01f, 7.3839803712e-01f, 7.4053033661e-01f, 7.4265435645e-01f,
    7.4477012572e-01f, 7.4687767356e-01f, 7.4897702918e-01f, 7.5106822187e-01f,
    7.5315128096e-01f, 7.5522623584e-01f, 7.5729311594e-01f, 7.5935195075e-01f,
    7.6140276981e-01f, 7.6344560268e-01f, 7.6548047897e-01f, 7.6750742832e-01f,
    7.6952648041e-01f, 7.7153766492e-01f, 7.7354101159e-01f, 7.7553655016e-01f,
    7.7752431037e-01f, 7.7950432202e-01f, 7.8147661487e-01f, 7.8344121873e-01f,
    7.8539816340e-01f
};
#ifdef __CUDACC__
#if B3_TWIST_APPROX == 3
static __device__ __constant__ float b3_atan_device[257] = {
#else
static __device__ const float b3_atan_device[257] = {
#endif
    0.0000000000e+00f, 3.9062301320e-03f, 7.8123410601e-03f, 1.1718213602e-02f,
    1.5623728620e-02f, 1.9528767041e-02f, 2.3433209879e-02f, 2.7336938258e-02f,
    3.1239833430e-02f, 3.5141776803e-02f, 3.9042649955e-02f, 4.2942334662e-02f,
    4.6840712916e-02f, 5.0737666945e-02f, 5.4633079239e-02f, 5.8526832566e-02f,
    6.2418809996e-02f, 6.6308894920e-02f, 7.0196971072e-02f, 7.4082922549e-02f,
    7.7966633832e-02f, 8.1847989803e-02f, 8.5726875771e-02f, 8.9603177485e-02f,
    9.3476781159e-02f, 9.7347573487e-02f, 1.0121544167e-01f, 1.0508027342e-01f,
    1.0894195699e-01f, 1.1280038120e-01f, 1.1665543544e-01f, 1.2050700969e-01f,
    1.2435499455e-01f, 1.2819928123e-01f, 1.3203976161e-01f, 1.3587632823e-01f,
    1.3970887429e-01f, 1.4353729370e-01f, 1.4736148109e-01f, 1.5118133180e-01f,
    1.5499674192e-01f, 1.5880760832e-01f, 1.6261382860e-01f, 1.6641530118e-01f,
    1.7021192529e-01f, 1.7400360094e-01f, 1.7779022899e-01f, 1.8157171116e-01f,
    1.8534795000e-01f, 1.8911884893e-01f, 1.9288431226e-01f, 1.9664424519e-01f,
    2.0039855383e-01f, 2.0414714518e-01f, 2.0788992720e-01f, 2.1162680877e-01f,
    2.1535769970e-01f, 2.1908251078e-01f, 2.2280115376e-01f, 2.2651354136e-01f,
    2.3021958728e-01f, 2.3391920621e-01f, 2.3761231387e-01f, 2.4129882693e-01f,
    2.4497866313e-01f, 2.4865174119e-01f, 2.5231798089e-01f, 2.5597730301e-01f,
    2.5962962941e-01f, 2.6327488296e-01f, 2.6691298759e-01f, 2.7054386829e-01f,
    2.7416745112e-01f, 2.7778366318e-01f, 2.8139243265e-01f, 2.8499368878e-01f,
    2.8858736189e-01f, 2.9217338339e-01f, 2.9575168575e-01f, 2.9932220253e-01f,
    3.0288486837e-01f, 3.0643961901e-01f, 3.0998639125e-01f, 3.1352512299e-01f,
    3.1705575321e-01f, 3.2057822199e-01f, 3.2409247049e-01f, 3.2759844095e-01f,
    3.3109607670e-01f, 3.3458532217e-01f, 3.3806612284e-01f, 3.4153842530e-01f,
    3.4500217721e-01f, 3.4845732731e-01f, 3.5190382541e-01f, 3.5534162242e-01f,
    3.5877067027e-01f, 3.6219092200e-01f, 3.6560233171e-01f, 3.6900485453e-01f,
    3.7239844668e-01f, 3.7578306541e-01f, 3.7915866903e-01f, 3.8252521690e-01f,
    3.8588266940e-01f, 3.8923098795e-01f, 3.9257013501e-01f, 3.9590007406e-01f,
    3.9922076958e-01f, 4.0253218708e-01f, 4.0583429307e-01f, 4.0912705508e-01f,
    4.1241044160e-01f, 4.1568442212e-01f, 4.1894896713e-01f, 4.2220404808e-01f,
    4.2544963737e-01f, 4.2868570839e-01f, 4.3191223547e-01f, 4.3512919389e-01f,
    4.3833655986e-01f, 4.4153431053e-01f, 4.4472242396e-01f, 4.4790087915e-01f,
    4.5106965599e-01f, 4.5422873527e-01f, 4.5737809867e-01f, 4.6051772877e-01f,
    4.6364760900e-01f, 4.6676772368e-01f, 4.6987805798e-01f, 4.7297859790e-01f,
    4.7606933032e-01f, 4.7915024293e-01f, 4.8222132423e-01f, 4.8528256356e-01f,
    4.8833395106e-01f, 4.9137547765e-01f, 4.9440713507e-01f, 4.9742891581e-01f,
    5.0044081315e-01f, 5.0344282111e-01f, 5.0643493448e-01f, 5.0941714880e-01f,
    5.1238946031e-01f, 5.1535186601e-01f, 5.1830436360e-01f, 5.2124695149e-01f,
    5.2417962878e-01f, 5.2710239527e-01f, 5.3001525142e-01f, 5.3291819839e-01f,
    5.3581123796e-01f, 5.3869437260e-01f, 5.4156760539e-01f, 5.4443094007e-01f,
    5.4728438099e-01f, 5.5012793310e-01f, 5.5296160199e-01f, 5.5578539382e-01f,
    5.5859931534e-01f, 5.6140337389e-01f, 5.6419757736e-01f, 5.6698193422e-01f,
    5.6975645348e-01f, 5.7252114470e-01f, 5.7527601796e-01f, 5.7802108387e-01f,
    5.8075635357e-01f, 5.8348183869e-01f, 5.8619755136e-01f, 5.8890350420e-01f,
    5.9159971034e-01f, 5.9428618332e-01f, 5.9696293722e-01f, 5.9962998650e-01f,
    6.0228734613e-01f, 6.0493503149e-01f, 6.0757305839e-01f, 6.1020144306e-01f,
    6.1282020217e-01f, 6.1542935275e-01f, 6.1802891228e-01f, 6.2061889860e-01f,
    6.2319932993e-01f, 6.2577022489e-01f, 6.2833160243e-01f, 6.3088348190e-01f,
    6.3342588297e-01f, 6.3595882567e-01f, 6.3848233035e-01f, 6.4099641773e-01f,
    6.4350110879e-01f, 6.4599642489e-01f, 6.4848238764e-01f, 6.5095901900e-01f,
    6.5342634118e-01f, 6.5588437671e-01f, 6.5833314838e-01f, 6.6077267927e-01f,
    6.6320299271e-01f, 6.6562411228e-01f, 6.6803606186e-01f, 6.7043886551e-01f,
    6.7283254759e-01f, 6.7521713266e-01f, 6.7759264552e-01f, 6.7995911118e-01f,
    6.8231655487e-01f, 6.8466500205e-01f, 6.8700447834e-01f, 6.8933500960e-01f,
    6.9165662185e-01f, 6.9396934132e-01f, 6.9627319441e-01f, 6.9856820768e-01f,
    7.0085440788e-01f, 7.0313182192e-01f, 7.0540047687e-01f, 7.0766039992e-01f,
    7.0991161846e-01f, 7.1215415999e-01f, 7.1438805216e-01f, 7.1661332273e-01f,
    7.1882999962e-01f, 7.2103811085e-01f, 7.2323768458e-01f, 7.2542874904e-01f,
    7.2761133263e-01f, 7.2978546379e-01f, 7.3195117112e-01f, 7.3410848326e-01f,
    7.3625742898e-01f, 7.3839803712e-01f, 7.4053033661e-01f, 7.4265435645e-01f,
    7.4477012572e-01f, 7.4687767356e-01f, 7.4897702918e-01f, 7.5106822187e-01f,
    7.5315128096e-01f, 7.5522623584e-01f, 7.5729311594e-01f, 7.5935195075e-01f,
    7.6140276981e-01f, 7.6344560268e-01f, 7.6548047897e-01f, 7.6750742832e-01f,
    7.6952648041e-01f, 7.7153766492e-01f, 7.7354101159e-01f, 7.7553655016e-01f,
    7.7752431037e-01f, 7.7950432202e-01f, 7.8147661487e-01f, 7.8344121873e-01f,
    7.8539816340e-01f
};
#endif
#endif
B3_HD B3_INL float b3_atan_unit(float r) {
#if B3_TWIST_APPROX == 1
    float u = r*r;
    float p = -1.7011699740e-03f;
    p = fmaf(p, u, 1.0487648856e-02f);
    p = fmaf(p, u, -3.0351864049e-02f);
    p = fmaf(p, u, 5.7089555192e-02f);
    p = fmaf(p, u, -8.3497249248e-02f);
    p = fmaf(p, u, 1.0932341486e-01f);
    p = fmaf(p, u, -1.4260016080e-01f);
    p = fmaf(p, u, 1.9998075281e-01f);
    p = fmaf(p, u, -3.3333276292e-01f);
    p = fmaf(p, u, 9.9999999716e-01f);
    return r*p;
#elif B3_TWIST_APPROX == 4
    float u = r*r;
    return r * fmaf(1.9114707101e-01f, u, 9.9918033571e-01f) / fmaf(5.1538252916e-01f, u, 1.0f);
#else
    float x = r*256.0f;
    int i = (int)x;
    i = i < 256 ? i : 255;
#ifdef __CUDA_ARCH__
    const float* table = b3_atan_device;
#else
    const float* table = b3_atan_host;
#endif
    return fmaf(x-i, table[i+1]-table[i], table[i]);
#endif
}

#endif // ATAN_APPROX_CUH
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

// ==== puffysics 04_world.inl: INTERNAL: defaults, world init, capacity API, body/shape creation, mass ====
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

// ==== puffysics 05_collision.inl: INTERNAL: manifolds, primitive tests, SAT box collision ====
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

// ==== puffysics 06_joints.inl: INTERNAL: weld/revolute creation, setters, joint solvers, hinge cache ====
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

// ==== puffysics 07_packed.inl: INTERNAL: packed-GS snapshots, interleaved solves, articulation hooks ====
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
#ifndef B3_ART_CUH
#define B3_ART_CUH
// Reduced-coordinate articulation: Featherstone ABA + matrix-free Delassus.
// Device/host, one world per caller (do not split a tree across a warp).
//
// Operators (Sathya, Montaut, de Mont-Marin, Carpentier, RAL 2026):
//   J^T λ   Alg. 1  RNEA backward
//   M^{-1} τ  Alg. 2  ABA
//   J qdd   Alg. 3  FK forward
//   Δx = J M^{-1} J^T x   merged two-sweep (zero bias)
// Damped inverse: dense (μ^{-1}+Δ) from matrix-free Delassus. Default
// b3_step is unchanged.
// Position: reduced-q + FK (tree hinges exact). Loops: spanning tree + cuts.
//
// Spatial convention: angular-first at each link COM, world-aligned axes.
//   motion ν = (ω, v_com), force f = (n, f)
// Revolute S at the child COM: (u, u × (com - anchor)).
// Included from puffysics.cuh after the rigid types.
// b3_art_step lives in puffysics.cuh (needs b3_step_indep).

#ifndef B3_ART_MAX_LINKS
#define B3_ART_MAX_LINKS B3_MAX_BODIES
#endif
#ifndef B3_ART_MAX_Q
#define B3_ART_MAX_Q B3_MAX_JOINTS
#endif
#ifndef B3_ART_MAX_ROWS
#define B3_ART_MAX_ROWS 32
#endif
#ifndef B3_ART_MAX_DEG
#define B3_ART_MAX_DEG 8
#endif
#ifndef B3_ART_MAX_CUTS
#define B3_ART_MAX_CUTS 8
#endif
#ifndef B3_ART_CUT_ITERS
#define B3_ART_CUT_ITERS 4
#endif
#ifndef B3_ART_DMIN
#define B3_ART_DMIN 1.0e-8f
#endif

typedef struct B3Motion {
    B3Vec3 w;
    B3Vec3 v;
} B3Motion;

typedef struct B3Force {
    B3Vec3 n;
    B3Vec3 f;
} B3Force;

typedef struct B3Inertia {
    B3Mat3 ww;
    B3Mat3 wv;
    B3Mat3 vw;
    B3Mat3 vv;
} B3Inertia;

typedef struct B3ArtRow {
    int link_a;
    int link_b;
    int torque;
    B3Vec3 ra;
    B3Vec3 rb;
    B3Vec3 n;
} B3ArtRow;

typedef struct B3Art {
    int n_links;
    int n_q;
    int ok;
    int parent[B3_ART_MAX_LINKS];
    int body[B3_ART_MAX_LINKS];
    int joint[B3_ART_MAX_LINKS];
    int fixed[B3_ART_MAX_LINKS];
    int floating[B3_ART_MAX_LINKS];
    B3Vec3 local_anchor_a[B3_ART_MAX_LINKS];
    B3Vec3 local_anchor_b[B3_ART_MAX_LINKS];
    B3Quat local_rot_a[B3_ART_MAX_LINKS];
    B3Quat local_rot_b[B3_ART_MAX_LINKS];
    B3Vec3 local_center[B3_ART_MAX_LINKS];
    B3Vec3 I_local[B3_ART_MAX_LINKS];
    float mass[B3_ART_MAX_LINKS];
    float gravity_scale[B3_ART_MAX_LINKS];
    B3Vec3 com[B3_ART_MAX_LINKS];
    B3Vec3 pos[B3_ART_MAX_LINKS];
    B3Quat rot[B3_ART_MAX_LINKS];
    B3Vec3 r[B3_ART_MAX_LINKS];
    B3Motion S[B3_ART_MAX_LINKS];
    B3Motion v[B3_ART_MAX_LINKS];
    B3Motion a[B3_ART_MAX_LINKS];
    B3Motion c[B3_ART_MAX_LINKS];
    B3Inertia I[B3_ART_MAX_LINKS];
    B3Inertia IA[B3_ART_MAX_LINKS];
    B3Force U[B3_ART_MAX_LINKS];
    B3Force pA[B3_ART_MAX_LINKS];
    float q[B3_ART_MAX_LINKS];
    float qd[B3_ART_MAX_LINKS];
    float qdd[B3_ART_MAX_LINKS];
    float tau[B3_ART_MAX_LINKS];
    float Dinv[B3_ART_MAX_LINKS];
    int n_cuts;
    int cut_joint[B3_ART_MAX_CUTS];
    int cut_body_a[B3_ART_MAX_CUTS];
    int cut_body_b[B3_ART_MAX_CUTS];
} B3Art;

B3_HD B3_INL B3Motion b3_motion0(void) {
    B3Motion m;
    m.w = b3_v(0.0f, 0.0f, 0.0f);
    m.v = b3_v(0.0f, 0.0f, 0.0f);
    return m;
}

B3_HD B3_INL B3Force b3_force0(void) {
    B3Force f;
    f.n = b3_v(0.0f, 0.0f, 0.0f);
    f.f = b3_v(0.0f, 0.0f, 0.0f);
    return f;
}

B3_HD B3_INL B3Inertia b3_I0(void) {
    B3Inertia I;
    I.ww = b3_mat0();
    I.wv = b3_mat0();
    I.vw = b3_mat0();
    I.vv = b3_mat0();
    return I;
}

B3_HD B3_INL B3Mat3 b3_mat3_sub(B3Mat3 a, B3Mat3 b) {
    B3Mat3 r;
    r.cx = b3_sub(a.cx, b.cx);
    r.cy = b3_sub(a.cy, b.cy);
    r.cz = b3_sub(a.cz, b.cz);
    return r;
}

B3_HD B3_INL B3Mat3 b3_mat3_scale(B3Mat3 a, float s) {
    B3Mat3 r;
    r.cx = b3_mul(a.cx, s);
    r.cy = b3_mul(a.cy, s);
    r.cz = b3_mul(a.cz, s);
    return r;
}

B3_HD B3_INL B3Mat3 b3_outer(B3Vec3 a, B3Vec3 b) {
    B3Mat3 r;
    r.cx = b3_mul(a, b.x);
    r.cy = b3_mul(a, b.y);
    r.cz = b3_mul(a, b.z);
    return r;
}

B3_HD B3_INL void b3_I_set_col(B3Inertia* I, int col, B3Force f) {
    B3Mat3* ang = col < 3 ? &I->ww : &I->wv;
    B3Mat3* lin = col < 3 ? &I->vw : &I->vv;
    int c = col < 3 ? col : col - 3;
    if (c == 0) {
        ang->cx = f.n;
        lin->cx = f.f;
    } else if (c == 1) {
        ang->cy = f.n;
        lin->cy = f.f;
    } else {
        ang->cz = f.n;
        lin->cz = f.f;
    }
}

B3_HD B3_INL B3Force b3_I_mul(B3Inertia I, B3Motion m) {
    B3Force f;
    f.n = b3_add(b3_mv(I.ww, m.w), b3_mv(I.wv, m.v));
    f.f = b3_add(b3_mv(I.vw, m.w), b3_mv(I.vv, m.v));
    return f;
}

B3_HD B3_INL B3Inertia b3_I_add(B3Inertia a, B3Inertia b) {
    B3Inertia r;
    r.ww = b3_maddm(a.ww, b.ww);
    r.wv = b3_maddm(a.wv, b.wv);
    r.vw = b3_maddm(a.vw, b.vw);
    r.vv = b3_maddm(a.vv, b.vv);
    return r;
}

B3_HD B3_INL B3Inertia b3_I_shift(B3Inertia I, B3Vec3 r) {
    B3Inertia O = b3_I0();
    for (int k = 0; k < 6; k++) {
        B3Motion m = b3_motion0();
        if (k < 3) {
            m.w = b3_v(k == 0 ? 1.0f : 0.0f, k == 1 ? 1.0f : 0.0f,
                k == 2 ? 1.0f : 0.0f);
        } else {
            int t = k - 3;
            m.v = b3_v(t == 0 ? 1.0f : 0.0f, t == 1 ? 1.0f : 0.0f,
                t == 2 ? 1.0f : 0.0f);
        }
        m.v = b3_add(m.v, b3_cross(m.w, r));
        B3Force f = b3_I_mul(I, m);
        B3Force fp;
        fp.n = b3_add(f.n, b3_cross(r, f.f));
        fp.f = f.f;
        b3_I_set_col(&O, k, fp);
    }
    return O;
}

B3_HD B3_INL B3Inertia b3_I_rank1(B3Inertia I, B3Force U, float dinv) {
    I.ww = b3_mat3_sub(I.ww, b3_mat3_scale(b3_outer(U.n, U.n), dinv));
    I.wv = b3_mat3_sub(I.wv, b3_mat3_scale(b3_outer(U.n, U.f), dinv));
    I.vw = b3_mat3_sub(I.vw, b3_mat3_scale(b3_outer(U.f, U.n), dinv));
    I.vv = b3_mat3_sub(I.vv, b3_mat3_scale(b3_outer(U.f, U.f), dinv));
    return I;
}

B3_HD B3_INL float b3_S_dot(B3Motion S, B3Force f) {
    return b3_dot(S.w, f.n) + b3_dot(S.v, f.f);
}

B3_HD B3_INL B3Motion b3_S_mul(B3Motion S, float s) {
    B3Motion m;
    m.w = b3_mul(S.w, s);
    m.v = b3_mul(S.v, s);
    return m;
}

B3_HD B3_INL int b3_solve6(const float A[36], const float b[6],
        float x[6]) {
    float M[6][7];
    for (int r = 0; r < 6; r++) {
        for (int c = 0; c < 6; c++) {
            M[r][c] = A[r * 6 + c];
        }
        M[r][6] = b[r];
    }
    for (int k = 0; k < 6; k++) {
        int piv = k;
        float best = fabsf(M[k][k]);
        for (int r = k + 1; r < 6; r++) {
            float v = fabsf(M[r][k]);
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        if (best < 1.0e-12f) {
            return 0;
        }
        if (piv != k) {
            for (int c = k; c < 7; c++) {
                float t = M[k][c];
                M[k][c] = M[piv][c];
                M[piv][c] = t;
            }
        }
        float inv = 1.0f / M[k][k];
        for (int c = k; c < 7; c++) {
            M[k][c] *= inv;
        }
        for (int r = 0; r < 6; r++) {
            if (r == k) {
                continue;
            }
            float s = M[r][k];
            for (int c = k; c < 7; c++) {
                M[r][c] -= s * M[k][c];
            }
        }
    }
    for (int r = 0; r < 6; r++) {
        x[r] = M[r][6];
    }
    return 1;
}

B3_HD B3_INL void b3_I_pack(B3Inertia I, float A[36]) {
    for (int c = 0; c < 3; c++) {
        B3Vec3 n = c == 0 ? I.ww.cx : (c == 1 ? I.ww.cy : I.ww.cz);
        B3Vec3 f = c == 0 ? I.vw.cx : (c == 1 ? I.vw.cy : I.vw.cz);
        A[0 * 6 + c] = n.x;
        A[1 * 6 + c] = n.y;
        A[2 * 6 + c] = n.z;
        A[3 * 6 + c] = f.x;
        A[4 * 6 + c] = f.y;
        A[5 * 6 + c] = f.z;
    }
    for (int c = 0; c < 3; c++) {
        B3Vec3 n = c == 0 ? I.wv.cx : (c == 1 ? I.wv.cy : I.wv.cz);
        B3Vec3 f = c == 0 ? I.vv.cx : (c == 1 ? I.vv.cy : I.vv.cz);
        A[0 * 6 + 3 + c] = n.x;
        A[1 * 6 + 3 + c] = n.y;
        A[2 * 6 + 3 + c] = n.z;
        A[3 * 6 + 3 + c] = f.x;
        A[4 * 6 + 3 + c] = f.y;
        A[5 * 6 + 3 + c] = f.z;
    }
}

B3_HD B3_INL int b3_I_solve(B3Inertia I, B3Force rhs, B3Motion* a) {
    float A[36], b[6], x[6];
    b3_I_pack(I, A);
    b[0] = rhs.n.x;
    b[1] = rhs.n.y;
    b[2] = rhs.n.z;
    b[3] = rhs.f.x;
    b[4] = rhs.f.y;
    b[5] = rhs.f.z;
    if (!b3_solve6(A, b, x)) {
        *a = b3_motion0();
        return 0;
    }
    a->w = b3_v(x[0], x[1], x[2]);
    a->v = b3_v(x[3], x[4], x[5]);
    return 1;
}

B3_HD B3_INL int b3_art_clear(B3Art* art) {
    memset(art, 0, sizeof(*art));
    for (int i = 0; i < B3_ART_MAX_LINKS; i++) {
        art->parent[i] = -1;
        art->joint[i] = -1;
    }
    return 1;
}

B3_HD B3_INL void b3_art_link_from_body(B3Art* art, int li,
        const B3Body* bd) {
    art->mass[li] = bd->inv_mass > 0.0f ? 1.0f / bd->inv_mass : 0.0f;
    art->I_local[li] = b3_v(
        bd->inv_inertia.x > 0.0f ? 1.0f / bd->inv_inertia.x : 0.0f,
        bd->inv_inertia.y > 0.0f ? 1.0f / bd->inv_inertia.y : 0.0f,
        bd->inv_inertia.z > 0.0f ? 1.0f / bd->inv_inertia.z : 0.0f);
    art->local_center[li] = bd->local_center;
    art->gravity_scale[li] = bd->gravity_scale;
}

B3_HD B3_INL int b3_art_from_world(B3Art* art, const B3World* w) {
    b3_art_clear(art);
    int deg[B3_MAX_BODIES];
    int adj_b[B3_MAX_BODIES][B3_ART_MAX_DEG];
    int adj_j[B3_MAX_BODIES][B3_ART_MAX_DEG];
    int body_link[B3_MAX_BODIES];
    unsigned char seen[B3_MAX_BODIES];
    for (int i = 0; i < B3_MAX_BODIES; i++) {
        deg[i] = 0;
        body_link[i] = -1;
        seen[i] = 0;
    }
    for (int k = 0; k < w->joint_count; k++) {
        const B3Joint* j = &w->joints[k];
#ifndef B3_REVOLUTE_ONLY
        if (j->type != B3_JOINT_REVOLUTE) {
            continue;
        }
#endif
        if (j->body_a == j->body_b) {
            art->ok = 0;
            return 0;
        }
        int a = j->body_a;
        int b = j->body_b;
        if (deg[a] >= B3_ART_MAX_DEG || deg[b] >= B3_ART_MAX_DEG) {
            art->ok = 0;
            return 0;
        }
        adj_b[a][deg[a]] = b;
        adj_j[a][deg[a]] = k;
        deg[a]++;
        adj_b[b][deg[b]] = a;
        adj_j[b][deg[b]] = k;
        deg[b]++;
    }

    int q[B3_MAX_BODIES];
    int parent_body[B3_MAX_BODIES];
    int parent_joint[B3_MAX_BODIES];
    for (int i = 0; i < B3_MAX_BODIES; i++) {
        parent_body[i] = -1;
        parent_joint[i] = -1;
    }

    for (int pass = 0; pass < 2; pass++) {
        for (int s = 0; s < w->body_count; s++) {
            int is_static = (w->bodies[s].type != B3_DYNAMIC);
            if (deg[s] == 0 || seen[s] || is_static != (pass == 0)) {
                continue;
            }
            int qh = 0, qt = 0;
            q[qt++] = s;
            seen[s] = 1;
            parent_body[s] = -1;
            while (qh < qt) {
                int u = q[qh++];
                for (int e = 0; e < deg[u]; e++) {
                    int v = adj_b[u][e];
                    if (seen[v]) {
                        if (parent_body[u] != v && parent_body[v] != u) {
                            int jk = adj_j[u][e];
                            int dup = 0;
                            for (int c = 0; c < art->n_cuts; c++) {
                                if (art->cut_joint[c] == jk) {
                                    dup = 1;
                                    break;
                                }
                            }
                            if (!dup) {
                                if (art->n_cuts >= B3_ART_MAX_CUTS) {
                                    art->ok = 0;
                                    return 0;
                                }
                                int ci = art->n_cuts++;
                                art->cut_joint[ci] = jk;
                                art->cut_body_a[ci] = w->joints[jk].body_a;
                                art->cut_body_b[ci] = w->joints[jk].body_b;
                            }
                        }
                        continue;
                    }
                    seen[v] = 1;
                    parent_body[v] = u;
                    parent_joint[v] = adj_j[u][e];
                    q[qt++] = v;
                }
            }
            for (int t = 0; t < qt; t++) {
                int b = q[t];
                if (art->n_links >= B3_ART_MAX_LINKS) {
                    art->ok = 0;
                    return 0;
                }
                int li = art->n_links++;
                body_link[b] = li;
                art->body[li] = b;
                const B3Body* bd = &w->bodies[b];
                art->fixed[li] = (bd->type != B3_DYNAMIC);
                art->floating[li] = (parent_body[b] < 0 && !art->fixed[li]);
                b3_art_link_from_body(art, li, bd);
                if (parent_body[b] < 0) {
                    art->parent[li] = -1;
                    art->joint[li] = -1;
                } else {
                    art->parent[li] = body_link[parent_body[b]];
                    int jk = parent_joint[b];
                    art->joint[li] = jk;
                    const B3Joint* j = &w->joints[jk];
                    if (j->body_b == b) {
                        art->local_anchor_a[li] = j->local_anchor_a;
                        art->local_anchor_b[li] = j->local_anchor_b;
                        art->local_rot_a[li] = j->local_rot_a;
                        art->local_rot_b[li] = j->local_rot_b;
                    } else {
                        art->local_anchor_a[li] = j->local_anchor_b;
                        art->local_anchor_b[li] = j->local_anchor_a;
                        art->local_rot_a[li] = j->local_rot_b;
                        art->local_rot_b[li] = j->local_rot_a;
                    }
                    art->n_q++;
                }
            }
        }
    }
    for (int s = 0; s < w->body_count; s++) {
        if (seen[s] || w->bodies[s].type != B3_DYNAMIC) {
            continue;
        }
        if (art->n_links >= B3_ART_MAX_LINKS) {
            art->ok = 0;
            return 0;
        }
        int li = art->n_links++;
        body_link[s] = li;
        art->body[li] = s;
        const B3Body* bd = &w->bodies[s];
        art->fixed[li] = 0;
        art->floating[li] = 1;
        art->parent[li] = -1;
        art->joint[li] = -1;
        b3_art_link_from_body(art, li, bd);
        seen[s] = 1;
    }
    art->ok = art->n_links > 0;
    return art->ok;
}

B3_HD B3_INL int b3_art_bind(B3Art* art, const B3World* w) {
    return w->joint_count > 0 && b3_art_from_world(art, w);
}

B3_HD B3_INL void b3_art_lived_pose(const B3Body* b, B3Vec3 lc,
        B3Vec3* pos, B3Quat* rot, B3Vec3* com) {
    *rot = b3_qnorm(b3_qmul(b->delta_rot, b->rotation));
    *com = b3_add(b->center, b->delta_pos);
    *pos = b3_sub(*com, b3_rotate(*rot, lc));
}

B3_HD B3_INL float b3_art_joint_q(B3Quat rp, B3Quat rc,
        B3Quat la, B3Quat lb) {
    B3Quat qa = b3_qmul(rp, la);
    B3Quat qb = b3_qmul(rc, lb);
    if (b3_qdot(qa, qb) < 0.0f) {
        qb = b3_qneg(qb);
    }
    return b3_twist(b3_qinv_mul(qa, qb));
}

B3_HD B3_INL void b3_art_fk_link(B3Art* art, int i) {
    int p = art->parent[i];
    if (p < 0) {
        return;
    }
    B3Quat rz = b3_q_axis_angle(b3_v(0.0f, 0.0f, 1.0f), art->q[i]);
    B3Quat rot = b3_qnorm(b3_qmul(art->rot[p],
        b3_qmul(art->local_rot_a[i],
        b3_qmul(rz, b3_qconj(art->local_rot_b[i])))));
    B3Vec3 anchor = b3_xf_point(art->pos[p], art->rot[p],
        art->local_anchor_a[i]);
    B3Vec3 com = b3_add(anchor, b3_rotate(rot,
        b3_sub(art->local_center[i], art->local_anchor_b[i])));
    art->rot[i] = rot;
    art->com[i] = com;
    art->pos[i] = b3_sub(com, b3_rotate(rot, art->local_center[i]));
}

B3_HD B3_INL void b3_art_refresh(B3Art* art, const B3World* w) {
    for (int i = 0; i < art->n_links; i++) {
        const B3Body* b = &w->bodies[art->body[i]];
        b3_art_lived_pose(b, art->local_center[i],
            &art->pos[i], &art->rot[i], &art->com[i]);
        art->v[i].w = b->ang_vel;
        art->v[i].v = b->lin_vel;
        if (art->fixed[i]) {
            art->I[i] = b3_I0();
            art->mass[i] = 0.0f;
        } else {
            B3Mat3 Iw = b3_world_inv_i(art->rot[i], art->I_local[i]);
            art->I[i] = b3_I0();
            art->I[i].ww = Iw;
            float m = art->mass[i];
            art->I[i].vv.cx = b3_v(m, 0.0f, 0.0f);
            art->I[i].vv.cy = b3_v(0.0f, m, 0.0f);
            art->I[i].vv.cz = b3_v(0.0f, 0.0f, m);
        }
        art->tau[i] = 0.0f;
        art->qdd[i] = 0.0f;
        art->S[i] = b3_motion0();
        art->r[i] = b3_v(0.0f, 0.0f, 0.0f);
        art->qd[i] = 0.0f;
        art->q[i] = 0.0f;
        art->c[i] = b3_motion0();
        int p = art->parent[i];
        if (p < 0) {
            continue;
        }
        B3Vec3 anchor = b3_xf_point(art->pos[p], art->rot[p],
            art->local_anchor_a[i]);
        B3Vec3 axis_local = b3_rotate(art->local_rot_a[i],
            b3_v(0.0f, 0.0f, 1.0f));
        B3Vec3 u = b3_norm(b3_rotate(art->rot[p], axis_local));
        art->r[i] = b3_sub(art->com[i], art->com[p]);
        art->S[i].w = u;
        art->S[i].v = b3_cross(u, b3_sub(art->com[i], anchor));
        B3Vec3 wrel = b3_sub(art->v[i].w, art->v[p].w);
        art->qd[i] = b3_dot(wrel, u);
        art->q[i] = b3_art_joint_q(art->rot[p], art->rot[i],
            art->local_rot_a[i], art->local_rot_b[i]);
    }
}

B3_HD B3_INL void b3_art_seed_bias(B3Art* art, const B3World* w,
        int linear) {
    B3Vec3 g = w->gravity;
    for (int i = 0; i < art->n_links; i++) {
        art->IA[i] = art->I[i];
        art->pA[i] = b3_force0();
        if (art->fixed[i]) {
            art->c[i] = b3_motion0();
            continue;
        }
        const B3Body* b = &w->bodies[art->body[i]];
        B3Force ext = b3_force0();
        if (!linear) {
            ext.f = b3_add(b->force, b3_mul(g, art->mass[i] * art->gravity_scale[i]));
            ext.n = b->torque;
            B3Force Iw = b3_I_mul(art->I[i], art->v[i]);
            art->pA[i].n = b3_cross(art->v[i].w, Iw.n);
            art->pA[i].f = b3_cross(art->v[i].w, Iw.f);
        }
        art->pA[i].n = b3_sub(art->pA[i].n, ext.n);
        art->pA[i].f = b3_sub(art->pA[i].f, ext.f);

        int p = art->parent[i];
        if (p < 0 || linear) {
            art->c[i] = b3_motion0();
            continue;
        }
        B3Motion Sqd = b3_S_mul(art->S[i], art->qd[i]);
        B3Vec3 wp = art->v[p].w;
        art->c[i].w = b3_cross(art->v[i].w, Sqd.w);
        art->c[i].v = b3_add(b3_cross(art->v[i].w, Sqd.v),
            b3_cross(wp, b3_cross(wp, art->r[i])));
    }
}

B3_HD B3_INL void b3_art_backward(B3Art* art) {
    for (int i = art->n_links - 1; i >= 0; i--) {
        int p = art->parent[i];
        if (p < 0) {
            continue;
        }
        B3Motion S = art->S[i];
        art->U[i] = b3_I_mul(art->IA[i], S);
        float D = b3_S_dot(S, art->U[i]);
        if (D < B3_ART_DMIN) {
            D = B3_ART_DMIN;
        }
        art->Dinv[i] = 1.0f / D;
        float u = art->tau[i] - b3_S_dot(S, art->pA[i]);
        B3Inertia IAp = b3_I_rank1(art->IA[i], art->U[i], art->Dinv[i]);
        B3Force pA = art->pA[i];
        B3Force Ic = b3_I_mul(art->IA[i], art->c[i]);
        pA.n = b3_add(b3_add(pA.n, Ic.n), b3_mul(art->U[i].n, u * art->Dinv[i]));
        pA.f = b3_add(b3_add(pA.f, Ic.f), b3_mul(art->U[i].f, u * art->Dinv[i]));
        if (!art->fixed[p]) {
            art->IA[p] = b3_I_add(art->IA[p], b3_I_shift(IAp, art->r[i]));
            art->pA[p].n = b3_add(art->pA[p].n,
                b3_add(pA.n, b3_cross(art->r[i], pA.f)));
            art->pA[p].f = b3_add(art->pA[p].f, pA.f);
        }
    }
}

B3_HD B3_INL void b3_art_forward(B3Art* art) {
    for (int i = 0; i < art->n_links; i++) {
        int p = art->parent[i];
        if (p < 0) {
            if (art->fixed[i]) {
                art->a[i] = b3_motion0();
            } else if (art->floating[i]) {
                B3Force rhs;
                rhs.n = b3_neg(art->pA[i].n);
                rhs.f = b3_neg(art->pA[i].f);
                if (!b3_I_solve(art->IA[i], rhs, &art->a[i])) {
                    art->a[i] = b3_motion0();
                }
            } else {
                art->a[i] = b3_motion0();
            }
            continue;
        }
        B3Motion ap;
        ap.w = art->a[p].w;
        ap.v = b3_add(art->a[p].v, b3_cross(art->a[p].w, art->r[i]));
        B3Motion rhs = ap;
        rhs.w = b3_add(rhs.w, art->c[i].w);
        rhs.v = b3_add(rhs.v, art->c[i].v);
        float u = art->tau[i] - b3_S_dot(art->S[i], art->pA[i]);
        u -= b3_S_dot(art->S[i], b3_I_mul(art->IA[i], rhs));
        art->qdd[i] = art->Dinv[i] * u;
        B3Motion Sq = b3_S_mul(art->S[i], art->qdd[i]);
        art->a[i].w = b3_add(rhs.w, Sq.w);
        art->a[i].v = b3_add(rhs.v, Sq.v);
    }
}

B3_HD B3_INL void b3_art_aba(B3Art* art, const B3World* w) {
    b3_art_refresh(art, w);
    b3_art_seed_bias(art, w, 0);
    b3_art_backward(art);
    b3_art_forward(art);
}

B3_HD B3_INL B3Force b3_art_row_wrench_ex(B3Vec3 r, B3Vec3 n, float x,
        int torque) {
    B3Force f;
    if (torque == 1) {
        f.n = b3_mul(n, x);
        f.f = b3_v(0.0f, 0.0f, 0.0f);
    } else {
        f.f = b3_mul(n, x);
        f.n = b3_cross(r, f.f);
    }
    return f;
}

B3_HD B3_INL float b3_art_row_eval(const B3Art* art, const B3ArtRow* row,
        int accel) {
    float y = 0.0f;
    const B3Motion* m = accel ? art->a : art->v;
    if (row->torque == 1) {
        if (row->link_a >= 0) {
            y += b3_dot(m[row->link_a].w, row->n);
        }
        if (row->link_b >= 0) {
            y -= b3_dot(m[row->link_b].w, row->n);
        }
        return y;
    }
    if (row->link_a >= 0) {
        B3Vec3 ac = b3_add(m[row->link_a].v,
            b3_cross(m[row->link_a].w, row->ra));
        if (accel) {
            const B3Motion* v = &art->v[row->link_a];
            ac = b3_add(ac, b3_cross(v->w, b3_cross(v->w, row->ra)));
        }
        y += b3_dot(ac, row->n);
    }
    if (row->link_b >= 0) {
        B3Vec3 ac = b3_add(m[row->link_b].v,
            b3_cross(m[row->link_b].w, row->rb));
        if (accel) {
            const B3Motion* v = &art->v[row->link_b];
            ac = b3_add(ac, b3_cross(v->w, b3_cross(v->w, row->rb)));
        }
        y -= b3_dot(ac, row->n);
    }
    return y;
}

/* Δx = J M^{-1} J^T x. Bias / gravity / velocity products off. */
B3_HD B3_INL void b3_art_delassus_apply(B3Art* art, const B3World* w,
        const B3ArtRow* rows, const float* x, float* y, int n_rows) {
    b3_art_refresh(art, w);
    for (int i = 0; i < art->n_links; i++) {
        art->v[i] = b3_motion0();
        art->qd[i] = 0.0f;
    }
    b3_art_seed_bias(art, w, 1);
    for (int e = 0; e < n_rows; e++) {
        const B3ArtRow* row = &rows[e];
        if (row->link_a >= 0 && !art->fixed[row->link_a]) {
            B3Force f = b3_art_row_wrench_ex(row->ra, row->n, x[e], row->torque);
            art->pA[row->link_a].n = b3_sub(art->pA[row->link_a].n, f.n);
            art->pA[row->link_a].f = b3_sub(art->pA[row->link_a].f, f.f);
        }
        if (row->link_b >= 0 && !art->fixed[row->link_b]) {
            B3Force f = b3_art_row_wrench_ex(row->rb, row->n, x[e], row->torque);
            art->pA[row->link_b].n = b3_add(art->pA[row->link_b].n, f.n);
            art->pA[row->link_b].f = b3_add(art->pA[row->link_b].f, f.f);
        }
    }
    b3_art_backward(art);
    b3_art_forward(art);
    for (int e = 0; e < n_rows; e++) {
        y[e] = b3_art_row_eval(art, &rows[e], 1);
    }
}

B3_HD B3_INL int b3_art_link_of(const B3Art* art, int body) {
    for (int i = 0; i < art->n_links; i++) {
        if (art->body[i] == body) {
            return i;
        }
    }
    return -1;
}

/* Independent-body contact mass (Soft Step). Wrong for a jointed tree. */
B3_HD B3_INL float b3_art_indep_w(const B3Body* b, B3Vec3 r, B3Vec3 n) {
    if (b->type != B3_DYNAMIC || b->inv_mass <= 0.0f) {
        return 0.0f;
    }
    B3Vec3 rn = b3_cross(r, n);
    return b->inv_mass + b3_dot(rn, b3_mv(b->inv_i_world, rn));
}

B3_HD B3_INL int b3_art_make_row(const B3Art* art, int body_a, int body_b,
        B3Vec3 ra, B3Vec3 rb, B3Vec3 n, B3ArtRow* row) {
    row->link_a = body_a >= 0 ? b3_art_link_of(art, body_a) : -1;
    row->link_b = body_b >= 0 ? b3_art_link_of(art, body_b) : -1;
    row->torque = 0;
    row->ra = ra;
    row->rb = rb;
    row->n = n;
    return row->link_a >= 0 || row->link_b >= 0;
}

/* Δ = n · J M^{-1} J^T n. Isolated free bodies match b3_art_indep_w. */
B3_HD B3_INL float b3_art_response_w(B3Art* art, const B3World* w,
        const B3ArtRow* row) {
    float x = 1.0f;
    float y = 0.0f;
    b3_art_delassus_apply(art, w, row, &x, &y, 1);
    return y;
}

B3_HD B3_INL void b3_art_add_delta_vel(const B3Art* art, B3World* w) {
    for (int i = 0; i < art->n_links; i++) {
        if (art->fixed[i]) {
            continue;
        }
        B3Body* b = &w->bodies[art->body[i]];
        b->lin_vel = b3_add(b->lin_vel, art->a[i].v);
        b->ang_vel = b3_add(b->ang_vel, art->a[i].w);
    }
}

/* Soft Step contact convention: λ > 0 pushes B along n and A against n. */
B3_HD B3_INL void b3_art_apply_impulse(B3Art* art, B3World* w,
        const B3ArtRow* row, float lambda) {
    float x = -lambda;
    float y = 0.0f;
    b3_art_delassus_apply(art, w, row, &x, &y, 1);
    b3_art_add_delta_vel(art, w);
}

/* Torque-only rolling in the tangent plane. Twist stays on the normal row. */
B3_HD B3_INL void b3_art_solve_rolling_fields(B3Art* art, B3World* w,
        int body_a, int body_b, B3Vec3 t1, B3Vec3 t2, float rolling,
        float total_n, B3Vec3* rolling_impulse) {
    float max_r = rolling * total_n;
    if (!(max_r > 0.0f)) {
        return;
    }
    B3ArtRow rows[2];
    if (!b3_art_make_row(art, body_a, body_b,
            b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f), t1, &rows[0])) {
        return;
    }
    rows[0].torque = 1;
    b3_art_make_row(art, body_a, body_b,
        b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f), t2, &rows[1]);
    rows[1].torque = 1;
    float e0[2] = {1.0f, 0.0f};
    float e1[2] = {0.0f, 1.0f};
    float col0[2], col1[2];
    b3_art_delassus_apply(art, w, rows, e0, col0, 2);
    b3_art_delassus_apply(art, w, rows, e1, col1, 2);
    B3Mat2 K;
    K.cx.x = col0[0];
    K.cx.y = col0[1];
    K.cy.x = col1[0];
    K.cy.y = col1[1];
    B3Mat2 Minv = b3_invert2(K);
    B3Body* ba = &w->bodies[body_a];
    B3Body* bb = &w->bodies[body_b];
    B3Vec3 dw = b3_sub(bb->ang_vel, ba->ang_vel);
    B3Vec2 vt;
    vt.x = b3_dot(dw, t1);
    vt.y = b3_dot(dw, t2);
    B3Vec2 tm = b3_mv2(Minv, vt);
    B3Vec3 old = *rolling_impulse;
    B3Vec2 oi;
    oi.x = b3_dot(old, t1);
    oi.y = b3_dot(old, t2);
    B3Vec2 ni;
    ni.x = oi.x - tm.x;
    ni.y = oi.y - tm.y;
    float fl2 = ni.x * ni.x + ni.y * ni.y;
    if (fl2 > max_r * max_r && fl2 > 0.0f) {
        float sc = b3_rsqrt_scale(max_r, fl2);
        ni.x *= sc;
        ni.y *= sc;
    }
    B3Vec2 df;
    df.x = ni.x - oi.x;
    df.y = ni.y - oi.y;
    *rolling_impulse = b3_add(b3_mul(t1, ni.x), b3_mul(t2, ni.y));
    if (df.x != 0.0f) {
        b3_art_apply_impulse(art, w, &rows[0], df.x);
    }
    if (df.y != 0.0f) {
        b3_art_apply_impulse(art, w, &rows[1], df.y);
    }
}

B3_HD B3_INL int b3_solve_n(int n, const float* A, const float* b,
        float* x) {
    if (n < 1) {
        return 1;
    }
    if (n > B3_ART_MAX_ROWS) {
        n = B3_ART_MAX_ROWS;
    }
    float M[B3_ART_MAX_ROWS][B3_ART_MAX_ROWS + 1];
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++) {
            M[r][c] = A[r * n + c];
        }
        M[r][n] = b[r];
    }
    for (int k = 0; k < n; k++) {
        int piv = k;
        float best = fabsf(M[k][k]);
        for (int r = k + 1; r < n; r++) {
            float v = fabsf(M[r][k]);
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        if (best < 1.0e-12f) {
            return 0;
        }
        if (piv != k) {
            for (int c = k; c <= n; c++) {
                float tmp = M[k][c];
                M[k][c] = M[piv][c];
                M[piv][c] = tmp;
            }
        }
        float inv = 1.0f / M[k][k];
        for (int c = k; c <= n; c++) {
            M[k][c] *= inv;
        }
        for (int r = 0; r < n; r++) {
            if (r == k) {
                continue;
            }
            float s = M[r][k];
            for (int c = k; c <= n; c++) {
                M[r][c] -= s * M[k][c];
            }
        }
    }
    for (int r = 0; r < n; r++) {
        x[r] = M[r][n];
    }
    return 1;
}

/* Δ = J M^{-1} J^T, including two-body and row-row fill-in. */
B3_HD B3_INL void b3_art_delassus_matrix(B3Art* art, const B3World* w,
        const B3ArtRow* rows, float* D, int n) {
    float x[B3_ART_MAX_ROWS];
    float y[B3_ART_MAX_ROWS];
    for (int i = 0; i < n; i++) {
        x[i] = 0.0f;
    }
    for (int c = 0; c < n; c++) {
        x[c] = 1.0f;
        b3_art_delassus_apply(art, w, rows, x, y, n);
        for (int r = 0; r < n; r++) {
            D[r * n + c] = y[r];
        }
        x[c] = 0.0f;
    }
}

B3_HD B3_INL void b3_art_apply_impulses(B3Art* art, B3World* w,
        const B3ArtRow* rows, const float* lambda, int n) {
    float x[B3_ART_MAX_ROWS];
    float y[B3_ART_MAX_ROWS];
    int any = 0;
    for (int e = 0; e < n; e++) {
        x[e] = -lambda[e];
        if (lambda[e] != 0.0f) {
            any = 1;
        }
    }
    if (!any) {
        return;
    }
    b3_art_delassus_apply(art, w, rows, x, y, n);
    b3_art_add_delta_vel(art, w);
}

/* λ = (μ^{-1} + Δ)^{-1} y. Δ is J M^{-1} J^T from the two-sweep. */
B3_HD B3_INL void b3_art_damped_solve(B3Art* art, const B3World* w,
        const B3ArtRow* rows, const float* mu, const float* y, float* lambda,
        int n_rows) {
    int n = n_rows < B3_ART_MAX_ROWS ? n_rows : B3_ART_MAX_ROWS;
    if (n < 1) {
        return;
    }
    float D[B3_ART_MAX_ROWS * B3_ART_MAX_ROWS];
    b3_art_delassus_matrix(art, w, rows, D, n);
    for (int e = 0; e < n; e++) {
        float r = mu[e] > 1.0e-12f ? (1.0f / mu[e]) : 1.0e12f;
        D[e * n + e] += r;
    }
    if (!b3_solve_n(n, D, y, lambda)) {
        for (int e = 0; e < n; e++) {
            lambda[e] = 0.0f;
        }
    }
}

B3_HD B3_INL void b3_art_write_delta(B3Art* art, B3World* w, int i) {
    if (art->fixed[i]) {
        return;
    }
    B3Body* b = &w->bodies[art->body[i]];
    b->delta_pos = b3_sub(art->com[i], b->center);
    b->delta_rot = b3_qnorm(b3_qmul(art->rot[i], b3_qconj(b->rotation)));
}

B3_HD B3_INL void b3_art_integrate_vel(B3Art* art, B3World* w, float h) {
    b3_art_aba(art, w);
    for (int i = 0; i < art->n_links; i++) {
        if (art->fixed[i]) {
            continue;
        }
        B3Body* b = &w->bodies[art->body[i]];
        b->lin_vel = b3_madd(b->lin_vel, h, art->a[i].v);
        b->ang_vel = b3_madd(b->ang_vel, h, art->a[i].w);
    }
}

B3_HD B3_INL void b3_art_integrate_pos(B3Art* art, B3World* w,
        float h, float inv_dt) {
    float max_lin = w->max_linear_speed;
    float max_ang = B3_MAX_ROTATION * inv_dt;
    float max_lin2 = max_lin * max_lin;
    float max_ang2 = max_ang * max_ang;
    b3_art_refresh(art, w);
    for (int i = 0; i < art->n_links; i++) {
        int p = art->parent[i];
        if (p < 0) {
            if (art->fixed[i]) {
                continue;
            }
            B3Body* b = &w->bodies[art->body[i]];
            b3_integrate_position_state(b, h, max_lin, max_ang, max_lin2,
                max_ang2, &b->lin_vel, &b->ang_vel, &b->delta_pos,
                &b->delta_rot);
            b3_art_lived_pose(b, art->local_center[i],
                &art->pos[i], &art->rot[i], &art->com[i]);
            continue;
        }
        float qd = art->qd[i];
        if (qd * qd > max_ang2 && max_ang2 > 0.0f) {
            qd = copysignf(max_ang, qd);
            art->qd[i] = qd;
        }
        art->q[i] += h * qd;
        b3_art_fk_link(art, i);
        b3_art_write_delta(art, w, i);
    }
}

B3_HD B3_INL void b3_art_solve_friction(B3Art* art, B3World* w,
        B3Contact* c) {
    float total_n = 0.0f;
    float twist_lim = 0.0f;
    for (int p = 0; p < c->point_count; p++) {
        total_n += c->points[p].normal_impulse;
        twist_lim += c->points[p].lever * c->points[p].normal_impulse;
    }
    float max_t = c->friction * twist_lim;
    if (max_t > 0.0f) {
        B3ArtRow tw;
        if (b3_art_make_row(art, c->body_a, c->body_b,
                b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f),
                c->normal, &tw)) {
            tw.torque = 1;
            float ww = b3_art_response_w(art, w, &tw);
            c->twist_mass = ww > 0.0f ? 1.0f / ww : 0.0f;
            B3Body* ba = &w->bodies[c->body_a];
            B3Body* bb = &w->bodies[c->body_b];
            float twist_s = b3_dot(c->normal,
                b3_sub(bb->ang_vel, ba->ang_vel));
            float dtw = -c->twist_mass * twist_s;
            float old_t = c->twist_impulse;
            float nt = old_t + dtw;
            if (nt > max_t) {
                nt = max_t;
            } else if (nt < -max_t) {
                nt = -max_t;
            }
            dtw = nt - old_t;
            c->twist_impulse = nt;
            if (dtw != 0.0f) {
                b3_art_apply_impulse(art, w, &tw, dtw);
            }
        }
    }
    float max_f = c->friction * total_n;
    if (max_f > 0.0f) {
    B3ArtRow rows[2];
    if (b3_art_make_row(art, c->body_a, c->body_b, c->center_a, c->center_b,
            c->tangent1, &rows[0])) {
    b3_art_make_row(art, c->body_a, c->body_b, c->center_a, c->center_b,
        c->tangent2, &rows[1]);
    float e0[2] = {1.0f, 0.0f};
    float e1[2] = {0.0f, 1.0f};
    float col0[2], col1[2];
    b3_art_delassus_apply(art, w, rows, e0, col0, 2);
    b3_art_delassus_apply(art, w, rows, e1, col1, 2);
    B3Mat2 K;
    K.cx.x = col0[0];
    K.cx.y = col0[1];
    K.cy.x = col1[0];
    K.cy.y = col1[1];
    B3Mat2 Minv = b3_invert2(K);
    c->tangent_mass = Minv;
    B3Body* ba = &w->bodies[c->body_a];
    B3Body* bb = &w->bodies[c->body_b];
    B3Vec3 vra = b3_add(ba->lin_vel, b3_cross(ba->ang_vel, c->center_a));
    B3Vec3 vrb = b3_add(bb->lin_vel, b3_cross(bb->ang_vel, c->center_b));
    B3Vec3 vr = b3_sub(vrb, vra);
    B3Vec2 vt;
    vt.x = b3_dot(vr, c->tangent1);
    vt.y = b3_dot(vr, c->tangent2);
    B3Vec2 tm = b3_mv2(Minv, vt);
    B3Vec2 ni;
    ni.x = c->friction_impulse.x - tm.x;
    ni.y = c->friction_impulse.y - tm.y;
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
    if (df.x != 0.0f) {
        b3_art_apply_impulse(art, w, &rows[0], df.x);
    }
    if (df.y != 0.0f) {
        b3_art_apply_impulse(art, w, &rows[1], df.y);
    }
        }
    }
    b3_art_solve_rolling_fields(art, w, c->body_a, c->body_b,
        c->tangent1, c->tangent2, c->rolling, total_n, &c->rolling_impulse);
}

B3_HD B3_INL int b3_art_cut_rows(B3Art* art, const B3World* w, int ci,
        B3ArtRow rows[5], float err[5]) {
    const B3Joint* j = &w->joints[art->cut_joint[ci]];
    int la = b3_art_link_of(art, j->body_a);
    int lb = b3_art_link_of(art, j->body_b);
    if (la < 0 || lb < 0) {
        return 0;
    }
    B3Vec3 pa = b3_xf_point(art->pos[la], art->rot[la], j->local_anchor_a);
    B3Vec3 pb = b3_xf_point(art->pos[lb], art->rot[lb], j->local_anchor_b);
    B3Vec3 ra = b3_sub(pa, art->com[la]);
    B3Vec3 rb = b3_sub(pb, art->com[lb]);
    B3Vec3 d = b3_sub(pb, pa);
    B3Vec3 ax[3] = {
        b3_v(1.0f, 0.0f, 0.0f),
        b3_v(0.0f, 1.0f, 0.0f),
        b3_v(0.0f, 0.0f, 1.0f)
    };
    for (int k = 0; k < 3; k++) {
        rows[k].link_a = la;
        rows[k].link_b = lb;
        rows[k].torque = 0;
        rows[k].ra = ra;
        rows[k].rb = rb;
        rows[k].n = ax[k];
        err[k] = b3_dot(d, ax[k]);
    }
    B3Quat qa = b3_qmul(art->rot[la], j->local_rot_a);
    B3Quat qb = b3_qmul(art->rot[lb], j->local_rot_b);
    if (b3_qdot(qa, qb) < 0.0f) {
        qb = b3_qneg(qb);
    }
    B3Quat rel = b3_qinv_mul(qa, qb);
    B3Vec3 px = b3_rotate(qa, b3_v(1.0f, 0.0f, 0.0f));
    B3Vec3 py = b3_rotate(qa, b3_v(0.0f, 1.0f, 0.0f));
    rows[3].link_a = la;
    rows[3].link_b = lb;
    rows[3].torque = 1;
    rows[3].ra = b3_v(0.0f, 0.0f, 0.0f);
    rows[3].rb = b3_v(0.0f, 0.0f, 0.0f);
    rows[3].n = px;
    err[3] = 2.0f * rel.v.x;
    rows[4].link_a = la;
    rows[4].link_b = lb;
    rows[4].torque = 1;
    rows[4].ra = b3_v(0.0f, 0.0f, 0.0f);
    rows[4].rb = b3_v(0.0f, 0.0f, 0.0f);
    rows[4].n = py;
    err[4] = 2.0f * rel.v.y;
    return 5;
}

/* Cut revolute = 3 linear + 2 angular rows. One 5×5 Delassus block per cut.
 * PGS is only the fallback if Δ is singular. */
B3_HD B3_INL void b3_art_solve_cuts(B3Art* art, B3World* w,
        float inv_h, int use_bias, int iters) {
    if (art->n_cuts < 1) {
        return;
    }
    if (iters < 1) {
        iters = 1;
    }
    b3_art_refresh(art, w);
    for (int it = 0; it < iters; it++) {
        for (int c = 0; c < art->n_cuts; c++) {
            B3ArtRow rows[5];
            float err[5];
            if (b3_art_cut_rows(art, w, c, rows, err) != 5) {
                continue;
            }
            b3_art_refresh(art, w);
            float rhs[5];
            for (int k = 0; k < 5; k++) {
                float vn = b3_art_row_eval(art, &rows[k], 0);
                float vbias = 0.0f;
                if (use_bias) {
                    vbias = 0.2f * inv_h * err[k];
                }
                rhs[k] = vn - vbias;
            }
            float D[25];
            b3_art_delassus_matrix(art, w, rows, D, 5);
            for (int k = 0; k < 5; k++) {
                D[k * 5 + k] += 1.0e-8f;
            }
            float lam[5];
            if (!b3_solve_n(5, D, rhs, lam)) {
                for (int k = 0; k < 5; k++) {
                    float ww = D[k * 5 + k];
                    lam[k] = ww > 1.0e-10f ? rhs[k] / ww : 0.0f;
                }
            }
            b3_art_apply_impulses(art, w, rows, lam, 5);
        }
    }
}

B3_HD B3_INL void b3_art_solve_contacts(B3Art* art, B3World* w,
        float inv_h, float contact_speed, int use_bias, int iters) {
    if (iters < 1) {
        iters = 1;
    }
    for (int it = 0; it < iters; it++) {
        for (int i = 0; i < w->contact_count; i++) {
            B3Contact* c = &w->contacts[i];
            B3Body* ba = &w->bodies[c->body_a];
            B3Body* bb = &w->bodies[c->body_b];
            B3Vec3 n = c->normal;
            B3Vec3 dp = b3_sub(bb->delta_pos, ba->delta_pos);
            B3Quat dqa = ba->delta_rot;
            B3Quat dqb = bb->delta_rot;
            float total_n = 0.0f;
            for (int p = 0; p < c->point_count; p++) {
                B3Point* cp = &c->points[p];
                B3ArtRow row;
                if (!b3_art_make_row(art, c->body_a, c->body_b,
                        cp->r_a, cp->r_b, n, &row)) {
                    continue;
                }
                if (it == 0) {
                    float ww = b3_art_response_w(art, w, &row);
                    cp->normal_mass = ww > 0.0f ? 1.0f / ww : 0.0f;
                }
                B3Vec3 ds = b3_add(dp, b3_sub(b3_rotate(dqb, cp->r_b),
                    b3_rotate(dqa, cp->r_a)));
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
                B3Vec3 vra = b3_add(ba->lin_vel, b3_cross(ba->ang_vel, cp->r_a));
                B3Vec3 vrb = b3_add(bb->lin_vel, b3_cross(bb->ang_vel, cp->r_b));
                float vn = b3_dot(b3_sub(vrb, vra), n);
                float dimp = -cp->normal_mass * (mscale * vn + vbias)
                    - iscale * cp->normal_impulse;
                float nimp = b3_maxf(cp->normal_impulse + dimp, 0.0f);
                dimp = nimp - cp->normal_impulse;
                cp->normal_impulse = nimp;
                cp->total_normal += dimp;
                total_n += nimp;
                if (dimp != 0.0f) {
                    b3_art_apply_impulse(art, w, &row, dimp);
                }
            }
            if (!use_bias) {
                b3_art_solve_friction(art, w, c);
            }
        }
    }
}

#ifndef B3_ART_CONTACT_ITERS
#define B3_ART_CONTACT_ITERS 4
#endif

#ifdef B3_PACKED_GS
B3_HD B3_INL void b3_art_pull_gs(B3World* w, const B3GsBody* bl) {
    for (int i = 0; i < w->body_count; i++) {
        w->bodies[i].lin_vel = bl[i].lin_vel;
        w->bodies[i].ang_vel = bl[i].ang_vel;
        w->bodies[i].delta_pos = bl[i].delta_pos;
        w->bodies[i].delta_rot = bl[i].delta_rot;
    }
}

B3_HD B3_INL void b3_art_push_gs(const B3World* w, B3GsBody* bl) {
    for (int i = 0; i < w->body_count; i++) {
        if ((bl[i].flags & B3_FLAG_DYNAMIC) == 0) {
            continue;
        }
        bl[i].lin_vel = w->bodies[i].lin_vel;
        bl[i].ang_vel = w->bodies[i].ang_vel;
    }
}

B3_HD B3_INL void b3_art_solve_friction_gs(B3Art* art, B3World* w,
        B3GsContact* c) {
    float total_n = 0.0f;
    float twist_lim = 0.0f;
    for (int p = 0; p < c->point_count; p++) {
        total_n += c->points[p].normal_impulse;
        twist_lim += c->points[p].lever * c->points[p].normal_impulse;
    }
    float max_t = c->friction * twist_lim;
    if (max_t > 0.0f) {
        B3ArtRow tw;
        if (b3_art_make_row(art, c->body_a, c->body_b,
                b3_v(0.0f, 0.0f, 0.0f), b3_v(0.0f, 0.0f, 0.0f),
                c->normal, &tw)) {
            tw.torque = 1;
            float ww = b3_art_response_w(art, w, &tw);
            c->twist_mass = ww > 0.0f ? 1.0f / ww : 0.0f;
            B3Body* ba = &w->bodies[c->body_a];
            B3Body* bb = &w->bodies[c->body_b];
            float twist_s = b3_dot(c->normal,
                b3_sub(bb->ang_vel, ba->ang_vel));
            float dtw = -c->twist_mass * twist_s;
            float old_t = c->twist_impulse;
            float nt = old_t + dtw;
            if (nt > max_t) {
                nt = max_t;
            } else if (nt < -max_t) {
                nt = -max_t;
            }
            dtw = nt - old_t;
            c->twist_impulse = nt;
            if (dtw != 0.0f) {
                b3_art_apply_impulse(art, w, &tw, dtw);
            }
        }
    }
    float max_f = c->friction * total_n;
    if (max_f > 0.0f) {
    B3ArtRow rows[2];
    if (b3_art_make_row(art, c->body_a, c->body_b, c->center_a, c->center_b,
            c->tangent1, &rows[0])) {
    b3_art_make_row(art, c->body_a, c->body_b, c->center_a, c->center_b,
        c->tangent2, &rows[1]);
    float e0[2] = {1.0f, 0.0f};
    float e1[2] = {0.0f, 1.0f};
    float col0[2], col1[2];
    b3_art_delassus_apply(art, w, rows, e0, col0, 2);
    b3_art_delassus_apply(art, w, rows, e1, col1, 2);
    B3Mat2 K;
    K.cx.x = col0[0];
    K.cx.y = col0[1];
    K.cy.x = col1[0];
    K.cy.y = col1[1];
    B3Mat2 Minv = b3_invert2(K);
    c->tangent_mass = Minv;
    B3Body* ba = &w->bodies[c->body_a];
    B3Body* bb = &w->bodies[c->body_b];
    B3Vec3 vra = b3_add(ba->lin_vel, b3_cross(ba->ang_vel, c->center_a));
    B3Vec3 vrb = b3_add(bb->lin_vel, b3_cross(bb->ang_vel, c->center_b));
    B3Vec3 vr = b3_sub(vrb, vra);
    B3Vec2 vt;
    vt.x = b3_dot(vr, c->tangent1);
    vt.y = b3_dot(vr, c->tangent2);
    B3Vec2 tm = b3_mv2(Minv, vt);
    B3Vec2 ni;
    ni.x = c->friction_impulse.x - tm.x;
    ni.y = c->friction_impulse.y - tm.y;
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
    if (df.x != 0.0f) {
        b3_art_apply_impulse(art, w, &rows[0], df.x);
    }
    if (df.y != 0.0f) {
        b3_art_apply_impulse(art, w, &rows[1], df.y);
    }
    }
    }
    b3_art_solve_rolling_fields(art, w, c->body_a, c->body_b,
        c->tangent1, c->tangent2, c->rolling, total_n, &c->rolling_impulse);
}

B3_HD B3_INL void b3_art_solve_contacts_gs(B3Art* art, B3World* w,
        B3GsContact* contacts, int n, B3GsBody* bodies, float inv_h,
        float contact_speed, int use_bias, int iters) {
    if (iters < 1) {
        iters = 1;
    }
    b3_art_pull_gs(w, bodies);
    for (int it = 0; it < iters; it++) {
        for (int i = 0; i < n; i++) {
            B3GsContact* c = &contacts[i];
            B3Body* ba = &w->bodies[c->body_a];
            B3Body* bb = &w->bodies[c->body_b];
            B3Vec3 nrm = c->normal;
            B3Vec3 dp = b3_sub(bb->delta_pos, ba->delta_pos);
            B3Quat dqa = ba->delta_rot;
            B3Quat dqb = bb->delta_rot;
            for (int p = 0; p < c->point_count; p++) {
                B3GsPoint* cp = &c->points[p];
                B3ArtRow row;
                if (!b3_art_make_row(art, c->body_a, c->body_b,
                        cp->r_a, cp->r_b, nrm, &row)) {
                    continue;
                }
                if (it == 0) {
                    float ww = b3_art_response_w(art, w, &row);
                    cp->normal_mass = ww > 0.0f ? 1.0f / ww : 0.0f;
                }
                B3Vec3 ds = b3_add(dp, b3_sub(b3_rotate(dqb, cp->r_b),
                    b3_rotate(dqa, cp->r_a)));
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
                B3Vec3 vra = b3_add(ba->lin_vel, b3_cross(ba->ang_vel, cp->r_a));
                B3Vec3 vrb = b3_add(bb->lin_vel, b3_cross(bb->ang_vel, cp->r_b));
                float vn = b3_dot(b3_sub(vrb, vra), nrm);
                float dimp = -cp->normal_mass * (mscale * vn + vbias)
                    - iscale * cp->normal_impulse;
                float nimp = b3_maxf(cp->normal_impulse + dimp, 0.0f);
                dimp = nimp - cp->normal_impulse;
                cp->normal_impulse = nimp;
                cp->total_normal += dimp;
                if (dimp != 0.0f) {
                    b3_art_apply_impulse(art, w, &row, dimp);
                }
            }
            if (!use_bias) {
                b3_art_solve_friction_gs(art, w, c);
            }
        }
    }
    b3_art_push_gs(w, bodies);
}
#endif

#endif // B3_ART_CUH
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

// ==== puffysics 08_contacts.inl: INTERNAL: contact generation, warm start, contact sweeps, restitution ====
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

// ==== puffysics 09_step.inl: INTERNAL: integrators, substep driver, b3_step, CUDA kernel ====
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

#endif // PUFFYSICS_CORE_INCLUDED
